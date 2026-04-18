#include "pch.h"
#include "Drive.h"

#include "DriveBase.h"
#include "MazeMapSharedRuntime.h"
#include "MissionStartPolicy.h"
#include "MotionTargetProjection.h"
#include "TurnWallEdgeTracker.h"
#include "WheelControlProfile.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace MazeMap::App::Internal
{
    namespace
    {
        using ControlVector = LoopController::ControlVector;

        constexpr std::size_t kPrimitiveStorageBytes = 64U * sizeof(std::uint32_t);
        constexpr float kMinimumTrackedSpeedMps = 1.0e-4f;

        struct HoldState final
        {
            std::uint16_t requestedTicks{};
            std::uint16_t remainingTicks{};
            bool resetOnNonStationary{};
        };

        struct LinearMotionState final
        {
            float distanceM{};
            float cruiseSpeedMps{};
            float exitSpeedMps{};
            float targetYawRad{};
            float projectedTargetDistanceM{};
            float startDistanceM{};
            float commandedSpeedMps{};
            bool hasProjectedTargetDistance{};
        };

        struct TurnState final
        {
            float targetYawRad{};
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker{};
        };

        struct TurnTransitionState final
        {
            float distanceM{};
            float dCurvatureDs{};
            float initialSpeedMps{};
            float initialYawRateRadps{};
            float startDistanceM{};
        };

        struct ArcState final
        {
            float distanceM{};
            float curvature{};
            float initialSpeedMps{};
            float startDistanceM{};
        };

        struct ManeuverState final
        {
            MazeMap::ManeuverInstance maneuver{};
            float startDistanceM{};
            float startYawRad{};
        };

        union PrimitiveState final
        {
            HoldState hold;
            LinearMotionState linearMotion;
            TurnState turn;
            TurnTransitionState turnTransition;
            ArcState arc;
            ManeuverState maneuver;
        };

        static_assert(sizeof(PrimitiveState) <= kPrimitiveStorageBytes);

        template <typename T>
        T* StorageAs(void* storage) noexcept
        {
            return std::launder(reinterpret_cast<T*>(storage));
        }

        template <typename T>
        const T* StorageAs(const void* storage) noexcept
        {
            return std::launder(reinterpret_cast<const T*>(storage));
        }

        float AverageDistanceMeters(const DriveTelemetry& telemetry) noexcept
        {
            return 0.5f * (telemetry.leftDistanceM + telemetry.rightDistanceM);
        }

        float HeadingYawRad(const Eigen::Vector2f& headingUnit) noexcept
        {
            return std::atan2(headingUnit.x(), headingUnit.y());
        }

        ControlVector AddControlVectors(
            const ControlVector& lhs,
            const ControlVector& rhs) noexcept
        {
            return ControlVector::RawMotorPwm(
                MazeMap::ClampWheelDriveCommand(lhs.leftMotorPwm + rhs.leftMotorPwm),
                MazeMap::ClampWheelDriveCommand(lhs.rightMotorPwm + rhs.rightMotorPwm));
        }

        float ResolveInitialLinearSpeedMps(
            const LoopController::ModeState* state,
            const DriveBase& drive) noexcept
        {
            if ((state != nullptr) && std::isfinite(state->estimate.linearSpeedMps))
            {
                return state->estimate.linearSpeedMps;
            }

            const float lastCommandMps = drive.GetLastLinearCommandMps();
            return std::isfinite(lastCommandMps) ? lastCommandMps : 0.0f;
        }

        float ResolveInitialYawRateRadps(
            const LoopController::ModeState* state,
            const DriveBase& drive) noexcept
        {
            if ((state != nullptr) && std::isfinite(state->estimate.angularSpeedRadps))
            {
                return state->estimate.angularSpeedRadps;
            }

            const float lastCommandRadps = drive.GetLastAngularCommandRadps();
            return std::isfinite(lastCommandRadps) ? lastCommandRadps : 0.0f;
        }

        float ResolveMotionDirection(float cruiseSpeedMps, float exitSpeedMps) noexcept
        {
            const float cruiseSign = SignF(cruiseSpeedMps);
            if (cruiseSign != 0.0f)
            {
                return cruiseSign;
            }

            const float exitSign = SignF(exitSpeedMps);
            return (exitSign != 0.0f) ? exitSign : 1.0f;
        }

        float ResolveManeuverSpeedMps(
            const MazeMap::ManeuverInstance& maneuver,
            const MotionLimits& limits,
            const MazeMap::Vehicle& vehicle) noexcept
        {
            const float speedLimit =
                (maneuver.getCode() == MazeMap::MC_NONE) ?
                0.0f :
                (maneuver.IsStraight() ?
                    limits.maxSpeedMps :
                    (std::min)(limits.maxSpeedMps, maneuver.GetSpeedLimit(vehicle)));

            float requestedSpeedMps = maneuver.getEntrySpeed();
            if (!(std::isfinite(requestedSpeedMps) && (std::fabs(requestedSpeedMps) > kMinimumTrackedSpeedMps)))
            {
                requestedSpeedMps = maneuver.getExitSpeed();
            }
            if (!(std::isfinite(requestedSpeedMps) && (std::fabs(requestedSpeedMps) > kMinimumTrackedSpeedMps)))
            {
                requestedSpeedMps = speedLimit;
            }

            if (!(std::isfinite(requestedSpeedMps) && (std::fabs(requestedSpeedMps) > 0.0f)))
            {
                return 0.0f;
            }

            const float magnitudeLimit =
                (std::isfinite(speedLimit) && (speedLimit > 0.0f)) ?
                speedLimit :
                std::fabs(requestedSpeedMps);
            return SignF(requestedSpeedMps) * (std::min)(std::fabs(requestedSpeedMps), magnitudeLimit);
        }
    }

    Drive::Drive()
    {
        _commandPdSettings.heading = MazeMap::CommandPD::StateHeadingPD;
        _commandPdSettings.yawRate = MazeMap::CommandPD::StateWheelOmegaPD | MazeMap::CommandPD::IMUYaw;
        _commandPdSettings.velocity = MazeMap::CommandPD::StateWheelOmegaPD;
        _commandPdSettings.distance = MazeMap::CommandPD::RawCommand;
    }

    void Drive::SetOperationMode(const OperationMode mode) noexcept
    {
        _operationMode = mode;
    }

    Drive::OperationMode Drive::GetOperationMode() const noexcept
    {
        return _operationMode;
    }

    void Drive::SetLimits(const MotionLimits& limits) noexcept
    {
        _limits = limits;
    }

    const MotionLimits& Drive::GetLimits() const noexcept
    {
        return _limits;
    }

    void Drive::SetCommandPDSettings(const CommandPDSettings& settings) noexcept
    {
        _commandPdSettings = settings;
    }

    const Drive::CommandPDSettings& Drive::GetCommandPDSettings() const noexcept
    {
        return _commandPdSettings;
    }

    bool Drive::Active() const noexcept
    {
        return _activePrimitive != ActivePrimitive::None;
    }

    void Drive::Cancel() noexcept
    {
        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
    }

    void Drive::StartHold(const std::uint16_t durationMs, const bool requireContinuous)
    {
        if (!CanStart())
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
        if (durationMs == 0U)
        {
            return;
        }

        (void)::new (_primitiveStorageWords) HoldState{ durationMs, durationMs, requireContinuous };
        _activePrimitive = ActivePrimitive::Hold;
    }

    void Drive::StartStraight(
        float distanceM,
        float cruiseSpeed,
        float exitSpeed,
        const Eigen::Vector2f* targetHeadingOverride,
        const Eigen::Vector2f* targetPositionOverride)
    {
        if (!CanStart() ||
            !std::isfinite(distanceM) ||
            !(distanceM > 0.0f) ||
            !std::isfinite(cruiseSpeed) ||
            !std::isfinite(exitSpeed))
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;

        float targetYawRad = _drive->GetPose().yawRad;
        if ((targetHeadingOverride != nullptr) &&
            std::isfinite(targetHeadingOverride->x()) &&
            std::isfinite(targetHeadingOverride->y()))
        {
            targetYawRad = HeadingYawRad(*targetHeadingOverride);
        }

        float projectedTargetDistanceM = 0.0f;
        bool hasProjectedTargetDistance = false;
        const PoseEstimate& pose = _drive->GetPose();
        if ((targetPositionOverride != nullptr) &&
            MazeMap::TryComputeProjectedDistanceToTargetM(
                pose.xMeters,
                pose.yMeters,
                targetPositionOverride->x(),
                targetPositionOverride->y(),
                HeadingUnitFromYawRad(targetYawRad).x(),
                HeadingUnitFromYawRad(targetYawRad).y(),
                projectedTargetDistanceM))
        {
            projectedTargetDistanceM = std::fabs(projectedTargetDistanceM);
            hasProjectedTargetDistance = projectedTargetDistanceM > 0.0f;
        }

        float commandedSpeedMps = ResolveInitialLinearSpeedMps(TryGetLoopState(), *_drive);
        const float direction = ResolveMotionDirection(cruiseSpeed, exitSpeed);
        if (SignF(commandedSpeedMps) != direction)
        {
            commandedSpeedMps = 0.0f;
        }

        (void)::new (_primitiveStorageWords) LinearMotionState{
            distanceM,
            cruiseSpeed,
            exitSpeed,
            targetYawRad,
            projectedTargetDistanceM,
            _drive->GetAverageDistanceMeters(),
            commandedSpeedMps,
            hasProjectedTargetDistance };
        _activePrimitive = ActivePrimitive::LinearMotion;
    }

    void Drive::StartTurn(float angleRad, MazeMap::TurnWallEdgeTracker* wallEdgeTracker)
    {
        if (!CanStart() || !std::isfinite(angleRad))
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
        (void)::new (_primitiveStorageWords) TurnState{
            WrapAngleRad(_drive->GetPose().yawRad + angleRad),
            wallEdgeTracker };
        _activePrimitive = ActivePrimitive::Turn;
    }

    void Drive::StartTurnTransition(float distanceM, float dCurvatureDs)
    {
        if (!CanStart() ||
            !std::isfinite(distanceM) ||
            !(distanceM > 0.0f) ||
            !std::isfinite(dCurvatureDs))
        {
            return;
        }

        const LoopController::ModeState* const state = TryGetLoopState();
        const float initialSpeedMps = ResolveInitialLinearSpeedMps(state, *_drive);
        if (!(std::isfinite(initialSpeedMps) && (std::fabs(initialSpeedMps) > kMinimumTrackedSpeedMps)))
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
        (void)::new (_primitiveStorageWords) TurnTransitionState{
            distanceM,
            dCurvatureDs,
            initialSpeedMps,
            ResolveInitialYawRateRadps(state, *_drive),
            _drive->GetAverageDistanceMeters() };
        _activePrimitive = ActivePrimitive::TurnTransition;
    }

    void Drive::StartArc(float distanceM, float curvature)
    {
        if (!CanStart() ||
            !std::isfinite(distanceM) ||
            !(distanceM > 0.0f) ||
            !std::isfinite(curvature))
        {
            return;
        }

        const float initialSpeedMps = ResolveInitialLinearSpeedMps(TryGetLoopState(), *_drive);
        if (!(std::isfinite(initialSpeedMps) && (std::fabs(initialSpeedMps) > kMinimumTrackedSpeedMps)))
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
        (void)::new (_primitiveStorageWords) ArcState{
            distanceM,
            curvature,
            initialSpeedMps,
            _drive->GetAverageDistanceMeters() };
        _activePrimitive = ActivePrimitive::Arc;
    }

    void Drive::StartManeuver(const MazeMap::ManeuverInstance& maneuver)
    {
        if (!CanStart() || (maneuver.getCode() == MazeMap::MC_NONE))
        {
            return;
        }

        ResetActivePrimitive();
        _faultReason = nullptr;
        _faulted = false;
        (void)::new (_primitiveStorageWords) ManeuverState{
            maneuver,
            _drive->GetAverageDistanceMeters(),
            _drive->GetPose().yawRad };
        _activePrimitive = ActivePrimitive::Maneuver;
    }

    LoopController::ControlVector Drive::GetNextControls(bool& done)
    {
        done = false;
        if (_faulted)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (_activePrimitive == ActivePrimitive::None)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        const LoopController::ModeState* const state = TryGetLoopState();
        if (state == nullptr)
        {
            SetFault("Drive requires an active LoopController mode state");
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (!state->estimatorHealthy)
        {
            SetFault((state->faultReason != nullptr) ? state->faultReason : "Drive estimator unhealthy");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        ControlVector control = LoopController::ControlVector::Brake;
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            control = HoldControls(done);
            break;
        case ActivePrimitive::LinearMotion:
            control = LinearMotionControls(*state, done);
            break;
        case ActivePrimitive::Turn:
            control = TurnControls(*state, done);
            break;
        case ActivePrimitive::TurnTransition:
            control = TurnTransitionControls(*state, done);
            break;
        case ActivePrimitive::Arc:
            control = ArcControls(*state, done);
            break;
        case ActivePrimitive::Maneuver:
            control = ManeuverControls(*state, done);
            break;
        default:
            done = true;
            break;
        }

        if (done)
        {
            ResetActivePrimitive();
        }
        return control;
    }

    void Drive::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _loopController = &runtime.ControlLoop();
        _drive = &runtime.Drive();
        _speedVehicle = &runtime.SpeedVehicle();
        _maze = &runtime.Maze();
        _limits.maxSpeedMps = _speedVehicle->GetMaxSpeed();
        _limits.accelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        _limits.decelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        _limits.maxAngularSpeedRadps = _speedVehicle->GetMaxRotationalVelocity();
        _limits.angularAccelRadps2 = _speedVehicle->GetMaxAngularAcceleration();
    }

    bool Drive::CanStart() const noexcept
    {
        return (_runtime != nullptr) &&
            (_loopController != nullptr) &&
            (_drive != nullptr) &&
            (_speedVehicle != nullptr) &&
            (_maze != nullptr);
    }

    void Drive::ResetActivePrimitive() noexcept
    {
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            StorageAs<HoldState>(_primitiveStorageWords)->~HoldState();
            break;
        case ActivePrimitive::LinearMotion:
            StorageAs<LinearMotionState>(_primitiveStorageWords)->~LinearMotionState();
            break;
        case ActivePrimitive::Turn:
            StorageAs<TurnState>(_primitiveStorageWords)->~TurnState();
            break;
        case ActivePrimitive::TurnTransition:
            StorageAs<TurnTransitionState>(_primitiveStorageWords)->~TurnTransitionState();
            break;
        case ActivePrimitive::Arc:
            StorageAs<ArcState>(_primitiveStorageWords)->~ArcState();
            break;
        case ActivePrimitive::Maneuver:
            StorageAs<ManeuverState>(_primitiveStorageWords)->~ManeuverState();
            break;
        default:
            break;
        }

        std::memset(_primitiveStorageWords, 0, sizeof(_primitiveStorageWords));
        _activePrimitive = ActivePrimitive::None;
    }

    void Drive::SetFault(const char* reason) noexcept
    {
        _faultReason = reason;
        _faulted = true;
        ResetActivePrimitive();
        if (_runtime != nullptr)
        {
            (void)_runtime->FailActiveMode((_faultReason != nullptr) ? _faultReason : "Drive fault");
        }
    }

    bool Drive::IsDriveMotionSettled(
        const DriveTelemetry& stationaryReferenceTelemetry,
        unsigned long stationaryReferenceMs,
        const DriveTelemetry& telemetry,
        const SensorSnapshot& snapshot,
        unsigned long nowMs) const
    {
        const unsigned long elapsedMs = nowMs - stationaryReferenceMs;
        return MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            telemetry.leftDistanceM - stationaryReferenceTelemetry.leftDistanceM,
            telemetry.rightDistanceM - stationaryReferenceTelemetry.rightDistanceM,
            static_cast<float>(elapsedMs) * 1.0e-3f,
            snapshot.gyroRadps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    const LoopController::ModeState* Drive::TryGetLoopState() const noexcept
    {
        return (_loopController != nullptr) ? _loopController->CurrentModeState() : nullptr;
    }

    float Drive::ManeuverSpeedLimit(
        MazeMap::ManeuverCode code,
        const MotionLimits& limits,
        const MazeMap::Vehicle& vehicle)
    {
        const MazeMap::ManeuverInstance maneuver(code, MazeMap::DirectionalLocation());
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (maneuver.IsStraight())
        {
            return limits.maxSpeedMps;
        }
        return (std::min)(limits.maxSpeedMps, maneuver.GetSpeedLimit(vehicle));
    }

    LoopController::ControlVector Drive::HoldControls(bool& done)
    {
        const LoopController::ModeState* const state = TryGetLoopState();
        if (state == nullptr)
        {
            SetFault("Drive hold requires current loop state");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        auto& hold = *StorageAs<HoldState>(_primitiveStorageWords);
        const bool stationary = MazeMap::IsMissionStartupStationaryFromSensors(
            state->driveTelemetry.leftVelocityMps,
            state->driveTelemetry.rightVelocityMps,
            state->sensors.gyroRadps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
        if (stationary)
        {
            if (hold.remainingTicks > 0U)
            {
                --hold.remainingTicks;
            }
        }
        else if (hold.resetOnNonStationary)
        {
            hold.remainingTicks = hold.requestedTicks;
        }

        done = hold.remainingTicks == 0U;
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector Drive::LinearMotionControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& linear = *StorageAs<LinearMotionState>(_primitiveStorageWords);
        const float traveledM = std::fabs(AverageDistanceMeters(state.driveTelemetry) - linear.startDistanceM);
        const float targetDistanceM = linear.hasProjectedTargetDistance ? linear.projectedTargetDistanceM : linear.distanceM;
        const float remainingM = (std::max)(0.0f, targetDistanceM - traveledM);
        const float direction = ResolveMotionDirection(linear.cruiseSpeedMps, linear.exitSpeedMps);
        const float cruiseMagnitudeMps = (std::max)(std::fabs(linear.cruiseSpeedMps), std::fabs(linear.exitSpeedMps));
        const float exitMagnitudeMps = std::fabs(linear.exitSpeedMps);

        float desiredSpeedMps = direction * linear.commandedSpeedMps;
        if (remainingM > Config::kDistanceToleranceM)
        {
            const float accelLimitedSpeedMps = (std::min)(
                cruiseMagnitudeMps,
                std::fabs(linear.commandedSpeedMps) + (_limits.accelMps2 * state.dtSeconds));
            const float decelLimitedSpeedMps =
                ReachableSpeedWithBoundary(exitMagnitudeMps, remainingM, _limits.decelMps2);
            linear.commandedSpeedMps = direction * (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);
            desiredSpeedMps = linear.commandedSpeedMps;
        }
        else
        {
            desiredSpeedMps = direction * exitMagnitudeMps;
            done =
                (exitMagnitudeMps <= Config::kSpeedToleranceMps) ?
                MazeMap::IsMissionStartupStationaryFromSensors(
                    state.driveTelemetry.leftVelocityMps,
                    state.driveTelemetry.rightVelocityMps,
                    state.sensors.gyroRadps,
                    Config::kMotionSettleSpeedThresholdMps,
                    Config::kMotionSettleAngularSpeedThresholdRadps) :
                (std::fabs(state.estimate.linearSpeedMps - desiredSpeedMps) <= Config::kSpeedToleranceMps);
        }

        float desiredYawRateRadps =
            (Config::kStraightHeadingKp *
                HeadingErrorRad(
                    HeadingUnitFromYawRad(linear.targetYawRad),
                    state.estimate.headingUnit)) -
            (Config::kStraightYawD * state.estimate.angularSpeedRadps);
        if ((_operationMode == OperationMode::Maze) &&
            IsApproximatelyDiagonalHeadingUnit(HeadingUnitFromYawRad(linear.targetYawRad)))
        {
            desiredYawRateRadps += ComputeDiagonalWallCenterOmegaRadps(
                gWallDistanceCalibration,
                state.sensors.sideLeftDifferentialLight,
                state.sensors.sideRightDifferentialLight);
        }
        desiredYawRateRadps = (std::clamp)(
            desiredYawRateRadps,
            -_limits.maxAngularSpeedRadps,
            _limits.maxAngularSpeedRadps);

        ControlVector control = _drive->PointControlVector(
            desiredSpeedMps,
            desiredYawRateRadps,
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
        if (_commandPdSettings.heading != MazeMap::CommandPD::RawCommand)
        {
            control = AddControlVectors(control, _drive->FeedbackCommand(linear.targetYawRad, _commandPdSettings.heading));
        }
        return control;
    }

    LoopController::ControlVector Drive::TurnControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& turn = *StorageAs<TurnState>(_primitiveStorageWords);
        if (turn.wallEdgeTracker != nullptr)
        {
            MazeMap::ObserveTurnWallStates(*turn.wallEdgeTracker, state.sensors.leftWall, state.sensors.rightWall);
        }

        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_limits);
        const float remainingRad = AngleErrorRad(turn.targetYawRad, state.estimate.yawRad);
        if (MazeMap::IsInPlaceTurnComplete(remainingRad, state.estimate.angularSpeedRadps, turnProfile))
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                remainingRad,
                state.estimate.angularSpeedRadps,
                turnProfile,
                angularCommandRadps))
        {
            SetFault("Drive turn profile became invalid");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        return _drive->PointControlVector(0.0f, angularCommandRadps, _commandPdSettings.yawRate);
    }

    LoopController::ControlVector Drive::TurnTransitionControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& transition = *StorageAs<TurnTransitionState>(_primitiveStorageWords);
        const float traveledM = std::fabs(AverageDistanceMeters(state.driveTelemetry) - transition.startDistanceM);
        const float progressM = (std::min)(traveledM, transition.distanceM);
        const float finalYawRateRadps =
            transition.initialYawRateRadps +
            (transition.initialSpeedMps * transition.dCurvatureDs * transition.distanceM);
        const float desiredYawRateRadps =
            transition.initialYawRateRadps +
            (transition.initialSpeedMps * transition.dCurvatureDs * progressM);
        done =
            (traveledM >= (transition.distanceM - Config::kDistanceToleranceM)) &&
            (std::fabs(state.estimate.angularSpeedRadps - finalYawRateRadps) <= _limits.angularSpeedToleranceRadps);
        return _drive->PointControlVector(
            transition.initialSpeedMps,
            done ? finalYawRateRadps : desiredYawRateRadps,
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
    }

    LoopController::ControlVector Drive::ArcControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& arc = *StorageAs<ArcState>(_primitiveStorageWords);
        const float traveledM = std::fabs(AverageDistanceMeters(state.driveTelemetry) - arc.startDistanceM);
        done = traveledM >= (arc.distanceM - Config::kDistanceToleranceM);
        return _drive->PointControlVector(
            arc.initialSpeedMps,
            arc.initialSpeedMps * arc.curvature,
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
    }

    LoopController::ControlVector Drive::ManeuverControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& maneuverState = *StorageAs<ManeuverState>(_primitiveStorageWords);
        const MazeMap::ManeuverInstance& maneuver = maneuverState.maneuver;
        const MazeMap::ManeuverCode code = maneuver.getCode();
        const float traveledM = std::fabs(AverageDistanceMeters(state.driveTelemetry) - maneuverState.startDistanceM);
        const float totalDistanceM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM);
        const float desiredSpeedMps = ResolveManeuverSpeedMps(maneuver, _limits, *_speedVehicle);

        if (maneuver.SupportsPointTracking())
        {
            MazeMap::ManeuverPoint point{};
            if (!maneuver.TryGetManeuverPoint(
                    (std::min)(traveledM, totalDistanceM),
                    desiredSpeedMps,
                    point,
                    Config::kCellSizeM))
            {
                SetFault("Drive maneuver point became invalid");
                done = true;
                return LoopController::ControlVector::Brake;
            }

            done = traveledM >= (totalDistanceM - Config::kDistanceToleranceM);
            return _drive->PointControlVector(
                point.Velocity,
                (std::clamp)(point.Omega, -_limits.maxAngularSpeedRadps, _limits.maxAngularSpeedRadps),
                _commandPdSettings.velocity | _commandPdSettings.yawRate);
        }

        const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        if (totalDistanceM <= Config::kDistanceToleranceM)
        {
            const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_limits);
            const float targetYawRad = WrapAngleRad(maneuverState.startYawRad + angleRad);
            const float remainingRad = AngleErrorRad(targetYawRad, state.estimate.yawRad);
            if (MazeMap::IsInPlaceTurnComplete(remainingRad, state.estimate.angularSpeedRadps, turnProfile))
            {
                done = true;
                return LoopController::ControlVector::Brake;
            }

            float angularCommandRadps = 0.0f;
            if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                    remainingRad,
                    state.estimate.angularSpeedRadps,
                    turnProfile,
                    angularCommandRadps))
            {
                SetFault("Drive maneuver turn profile became invalid");
                done = true;
                return LoopController::ControlVector::Brake;
            }

            return _drive->PointControlVector(0.0f, angularCommandRadps, _commandPdSettings.yawRate);
        }

        if (std::fabs(angleRad) > 0.0f)
        {
            done = traveledM >= (totalDistanceM - Config::kDistanceToleranceM);
            return _drive->PointControlVector(
                desiredSpeedMps,
                desiredSpeedMps * (angleRad / totalDistanceM),
                _commandPdSettings.velocity | _commandPdSettings.yawRate);
        }

        const float remainingM = (std::max)(0.0f, totalDistanceM - traveledM);
        const float exitMagnitudeMps = std::fabs(maneuver.getExitSpeed());
        const float sign = (SignF(desiredSpeedMps) != 0.0f) ? SignF(desiredSpeedMps) : 1.0f;
        const float desiredStraightSpeedMps =
            (remainingM <= Config::kDistanceToleranceM) ?
            (sign * exitMagnitudeMps) :
            (sign * (std::min)(std::fabs(desiredSpeedMps), ReachableSpeedWithBoundary(exitMagnitudeMps, remainingM, _limits.decelMps2)));
        done =
            (remainingM <= Config::kDistanceToleranceM) &&
            ((exitMagnitudeMps <= Config::kSpeedToleranceMps) ?
                MazeMap::IsMissionStartupStationaryFromSensors(
                    state.driveTelemetry.leftVelocityMps,
                    state.driveTelemetry.rightVelocityMps,
                    state.sensors.gyroRadps,
                    Config::kMotionSettleSpeedThresholdMps,
                    Config::kMotionSettleAngularSpeedThresholdRadps) :
                (std::fabs(state.estimate.linearSpeedMps - desiredStraightSpeedMps) <= Config::kSpeedToleranceMps));

        ControlVector control = _drive->PointControlVector(
            desiredStraightSpeedMps,
            0.0f,
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
        if (_commandPdSettings.heading != MazeMap::CommandPD::RawCommand)
        {
            control = AddControlVectors(
                control,
                _drive->FeedbackCommand(maneuverState.startYawRad, _commandPdSettings.heading));
        }
        return control;
    }
}
