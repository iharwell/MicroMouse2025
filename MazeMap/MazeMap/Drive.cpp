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

        constexpr std::size_t kPrimitiveStorageBytes = 16U * sizeof(std::uint32_t);
        constexpr std::size_t kPrimitiveStorageAlignment = 16U;
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
            float maneuverSpeedMps{};
            MazeMap::ManeuverPoint lastPoint{};
            bool lastPointValid{};
        };

        static_assert(sizeof(HoldState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(LinearMotionState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(TurnState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(TurnTransitionState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(ArcState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(ManeuverState) <= kPrimitiveStorageBytes);
        static_assert(alignof(HoldState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(LinearMotionState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(TurnState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(TurnTransitionState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(ArcState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(ManeuverState) <= kPrimitiveStorageAlignment);

        float FallbackFinite(const float preferred, const float replacement) noexcept
        {
            return std::isfinite(preferred) ? preferred : replacement;
        }

        bool HasPositiveLimit(const float value) noexcept
        {
            return std::isfinite(value) && (value > 0.0f);
        }

        float FallbackMagnitude(const float preferred, const float replacement) noexcept
        {
            return (std::isfinite(preferred) && (std::fabs(preferred) > 0.0f)) ? preferred : replacement;
        }

        float ResolveMotionSettleSpeedThresholdMps() noexcept
        {
            const float baseThresholdMps = Config::kMotionSettleSpeedThresholdMps;
            return (GetMissionFanDutyCycle() > 0.0f) ?
                (baseThresholdMps * 5.0f) :
                baseThresholdMps;
        }

        float LimitByConfiguredMagnitude(const float command, const float configuredLimit) noexcept
        {
            const float maxMagnitude =
                HasPositiveLimit(configuredLimit) ?
                configuredLimit :
                std::fabs(command);
            return SignF(command) * (std::min)(std::fabs(command), maxMagnitude);
        }

        bool WithinConfiguredTolerance(const float error, const float configuredTolerance) noexcept
        {
            return !HasPositiveLimit(configuredTolerance) || (std::fabs(error) <= configuredTolerance);
        }

        float ReachableSpeedWithConfiguredLimit(
            const float boundarySpeedMps,
            const float remainingDistanceM,
            const float accelLimitMps2,
            const float fallbackSpeedMps) noexcept
        {
            return HasPositiveLimit(accelLimitMps2) ?
                ReachableSpeedWithBoundary(boundarySpeedMps, remainingDistanceM, accelLimitMps2) :
                fallbackSpeedMps;
        }

        bool IsTurnComplete(
            const float remainingRad,
            const float measuredYawRateRadps,
            const MotionLimits& limits) noexcept
        {
            return
                WithinConfiguredTolerance(remainingRad, limits.angleToleranceRad) &&
                WithinConfiguredTolerance(measuredYawRateRadps, limits.angularSpeedToleranceRadps);
        }

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

        float ResolveInitialLinearSpeedMps(
            const LoopController::ModeState* state,
            const DriveBase& drive) noexcept
        {
            if (state != nullptr)
            {
                return FallbackFinite(state->estimate.linearSpeedMps, drive.GetLastLinearCommandMps());
            }

            return drive.GetLastLinearCommandMps();
        }

        float ResolveInitialYawRateRadps(
            const LoopController::ModeState* state,
            const DriveBase& drive) noexcept
        {
            if (state != nullptr)
            {
                return FallbackFinite(state->estimate.angularSpeedRadps, drive.GetLastAngularCommandRadps());
            }

            return drive.GetLastAngularCommandRadps();
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
            float speedLimitMps = 0.0f;
            bool hasSpeedLimit = false;
            if (maneuver.getCode() != MazeMap::MC_NONE)
            {
                if (maneuver.IsStraight())
                {
                    if (HasPositiveLimit(limits.maxSpeedMps))
                    {
                        speedLimitMps = limits.maxSpeedMps;
                        hasSpeedLimit = true;
                    }
                }
                else
                {
                    const float maneuverLimitMps = maneuver.GetSpeedLimit(vehicle);
                    const bool hasConfiguredLimit = HasPositiveLimit(limits.maxSpeedMps);
                    const bool hasManeuverLimit = HasPositiveLimit(maneuverLimitMps);
                    if (hasConfiguredLimit && hasManeuverLimit)
                    {
                        speedLimitMps = (std::min)(limits.maxSpeedMps, maneuverLimitMps);
                        hasSpeedLimit = true;
                    }
                    else if (hasConfiguredLimit)
                    {
                        speedLimitMps = limits.maxSpeedMps;
                        hasSpeedLimit = true;
                    }
                    else if (hasManeuverLimit)
                    {
                        speedLimitMps = maneuverLimitMps;
                        hasSpeedLimit = true;
                    }
                }
            }

            float requestedSpeedMps = maneuver.getEntrySpeed();
            if (!(std::fabs(requestedSpeedMps) > kMinimumTrackedSpeedMps))
            {
                requestedSpeedMps = maneuver.getExitSpeed();
            }
            if (!(std::fabs(requestedSpeedMps) > kMinimumTrackedSpeedMps) && hasSpeedLimit)
            {
                requestedSpeedMps = speedLimitMps;
            }

            const float usableRequestedSpeedMps =
                FallbackMagnitude(requestedSpeedMps, hasSpeedLimit ? speedLimitMps : 0.0f);
            if (!hasSpeedLimit)
            {
                return usableRequestedSpeedMps;
            }

            return
                SignF(usableRequestedSpeedMps) *
                (std::min)(std::fabs(usableRequestedSpeedMps), speedLimitMps);
        }

        LoopController::ModeState BuildFallbackModeState(const DriveBase& drive) noexcept
        {
            LoopController::ModeState state{};
            state.estimate = drive.GetPose();
            state.driveTelemetry = drive.GetTelemetry();
            state.measured.linearSpeedMps = drive.GetLastLinearCommandMps();
            state.measured.angularSpeedRadps = drive.GetLastAngularCommandRadps();
            return state;
        }
    }

    Drive::Drive()
    {
        _commandPdSettings.heading = Config::kDriveHeadingCommandPd;
        _commandPdSettings.yawRate = Config::kDriveYawRateCommandPd;
        _commandPdSettings.velocity = Config::kDriveVelocityCommandPd;
        _commandPdSettings.distance = Config::kDriveDistanceCommandPd;
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
    }

    void Drive::StartHold(const std::uint16_t durationMs, const bool requireContinuous)
    {
        if (!CanStart())
        {
            return;
        }

        ResetActivePrimitive();
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
        if (!CanStart())
        {
            return;
        }

        ResetActivePrimitive();

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
        if (!CanStart())
        {
            return;
        }

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) TurnState{
            WrapAngleRad(_drive->GetPose().yawRad + angleRad),
            wallEdgeTracker };
        _activePrimitive = ActivePrimitive::Turn;
    }

    void Drive::StartTurnTransition(float distanceM, float dCurvatureDs)
    {
        if (!CanStart())
        {
            return;
        }

        const LoopController::ModeState* const state = TryGetLoopState();
        const float initialSpeedMps = ResolveInitialLinearSpeedMps(state, *_drive);

        ResetActivePrimitive();
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
        if (!CanStart())
        {
            return;
        }

        const float initialSpeedMps = ResolveInitialLinearSpeedMps(TryGetLoopState(), *_drive);

        ResetActivePrimitive();
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

        const float maneuverSpeedMps = ResolveManeuverSpeedMps(maneuver, _limits, *_speedVehicle);
        const float totalDistanceM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM);
        MazeMap::ManeuverPoint initialPoint{};
        bool initialPointValid = false;
        if (maneuver.SupportsPointTracking())
        {
            if (maneuver.TryGetManeuverPoint(
                    0.0f,
                    maneuverSpeedMps,
                    initialPoint,
                    Config::kCellSizeM))
            {
                initialPointValid = totalDistanceM > Config::kDistanceToleranceM;
            }
        }

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) ManeuverState{
            maneuver,
            _drive->GetAverageDistanceMeters(),
            _drive->GetPose().yawRad,
            maneuverSpeedMps,
            initialPoint,
            initialPointValid };
        _activePrimitive = ActivePrimitive::Maneuver;
    }

    LoopController::ControlVector Drive::GetNextControls(bool& done)
    {
        done = false;
        if (_activePrimitive == ActivePrimitive::None)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        const LoopController::ModeState* const state = TryGetLoopState();
        const LoopController::ModeState fallbackState =
            (_drive != nullptr) ? BuildFallbackModeState(*_drive) : LoopController::ModeState{};
        const LoopController::ModeState& commandState = (state != nullptr) ? *state : fallbackState;

        ControlVector control = LoopController::ControlVector::Brake;
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            control = HoldControls(commandState, done);
            break;
        case ActivePrimitive::LinearMotion:
            control = LinearMotionControls(commandState, done);
            break;
        case ActivePrimitive::Turn:
            control = TurnControls(commandState, done);
            break;
        case ActivePrimitive::TurnTransition:
            control = TurnTransitionControls(commandState, done);
            break;
        case ActivePrimitive::Arc:
            control = ArcControls(commandState, done);
            break;
        case ActivePrimitive::Maneuver:
            control = ManeuverControls(commandState, done);
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
        _loopController = &runtime.ControlLoop();
        _drive = &runtime.Drive();
        _speedVehicle = &runtime.SpeedVehicle();
        _maze = &runtime.Maze();
        MotionLimits runtimeLimits{};
        runtimeLimits.maxSpeedMps = _speedVehicle->GetMaxSpeed();
        runtimeLimits.accelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        runtimeLimits.decelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        runtimeLimits.maxAngularSpeedRadps = _speedVehicle->GetMaxRotationalVelocity();
        runtimeLimits.angularAccelRadps2 = _speedVehicle->GetMaxAngularAcceleration();
        SetLimits(runtimeLimits);
    }

    bool Drive::CanStart() const noexcept
    {
        return (_loopController != nullptr) &&
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

    const LoopController::ModeState* Drive::TryGetLoopState() const noexcept
    {
        return (_loopController != nullptr) ? _loopController->CurrentModeState() : nullptr;
    }

    LoopController::ControlVector Drive::HoldControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        auto& hold = *StorageAs<HoldState>(_primitiveStorageWords);
        const bool stationary = MazeMap::IsMissionStartupStationaryFromSensors(
            state.driveTelemetry.leftVelocityMps,
            state.driveTelemetry.rightVelocityMps,
            state.sensors.gyroRadps,
            ResolveMotionSettleSpeedThresholdMps(),
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
                (HasPositiveLimit(_limits.accelMps2) && std::isfinite(state.dtSeconds) && (state.dtSeconds >= 0.0f)) ?
                    (std::fabs(linear.commandedSpeedMps) + (_limits.accelMps2 * state.dtSeconds)) :
                    cruiseMagnitudeMps);
            const float decelLimitedSpeedMps = ReachableSpeedWithConfiguredLimit(
                exitMagnitudeMps,
                remainingM,
                _limits.decelMps2,
                cruiseMagnitudeMps);
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
                    ResolveMotionSettleSpeedThresholdMps(),
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
        desiredYawRateRadps = LimitByConfiguredMagnitude(desiredYawRateRadps, _limits.maxAngularSpeedRadps);

        return _drive->PointControlVector(
            desiredSpeedMps,
            desiredYawRateRadps,
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
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

        const float remainingRad = AngleErrorRad(turn.targetYawRad, state.estimate.yawRad);
        if (IsTurnComplete(remainingRad, state.estimate.angularSpeedRadps, _limits))
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        const float resolvedRemainingRad = FallbackFinite(remainingRad, 0.0f);
        const float feedforwardYawRateRadps =
            HasPositiveLimit(_limits.angularAccelRadps2) ?
            (SignF(resolvedRemainingRad) *
                ReachableSpeedWithBoundary(0.0f, std::fabs(resolvedRemainingRad), _limits.angularAccelRadps2)) :
            0.0f;
        return _drive->PointControlVectorWithHeadingTarget(
            0.0f,
            LimitByConfiguredMagnitude(feedforwardYawRateRadps, _limits.maxAngularSpeedRadps),
            turn.targetYawRad,
            _commandPdSettings.yawRate,
            _commandPdSettings.heading);
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
            WithinConfiguredTolerance(state.estimate.angularSpeedRadps - finalYawRateRadps, _limits.angularSpeedToleranceRadps);
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
        const float desiredSpeedMps = maneuverState.maneuverSpeedMps;

        if (maneuver.SupportsPointTracking())
        {
            MazeMap::ManeuverPoint point = maneuverState.lastPoint;
            if (maneuver.TryGetManeuverPoint(
                    (std::min)(traveledM, totalDistanceM),
                    desiredSpeedMps,
                    point,
                    Config::kCellSizeM))
            {
                maneuverState.lastPoint = point;
                maneuverState.lastPointValid = true;
            }

            if (maneuverState.lastPointValid)
            {
                done = traveledM >= (totalDistanceM - Config::kDistanceToleranceM);
                return _drive->PointControlVector(
                    maneuverState.lastPoint.Velocity,
                    LimitByConfiguredMagnitude(maneuverState.lastPoint.Omega, _limits.maxAngularSpeedRadps),
                    _commandPdSettings.velocity | _commandPdSettings.yawRate);
            }
        }

        const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        if (totalDistanceM <= Config::kDistanceToleranceM)
        {
            const float targetYawRad = WrapAngleRad(maneuverState.startYawRad + angleRad);
            const float remainingRad = AngleErrorRad(targetYawRad, state.estimate.yawRad);
            if (IsTurnComplete(remainingRad, state.estimate.angularSpeedRadps, _limits))
            {
                done = true;
                return LoopController::ControlVector::Brake;
            }

            const float resolvedRemainingRad = FallbackFinite(remainingRad, 0.0f);
            const float feedforwardYawRateRadps =
                HasPositiveLimit(_limits.angularAccelRadps2) ?
                (SignF(resolvedRemainingRad) *
                    ReachableSpeedWithBoundary(0.0f, std::fabs(resolvedRemainingRad), _limits.angularAccelRadps2)) :
                0.0f;
            return _drive->PointControlVectorWithHeadingTarget(
                0.0f,
                LimitByConfiguredMagnitude(feedforwardYawRateRadps, _limits.maxAngularSpeedRadps),
                targetYawRad,
                _commandPdSettings.yawRate,
                _commandPdSettings.heading);
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
            (sign * (std::min)(
                std::fabs(desiredSpeedMps),
                ReachableSpeedWithConfiguredLimit(
                    exitMagnitudeMps,
                    remainingM,
                    _limits.decelMps2,
                    std::fabs(desiredSpeedMps))));
        done =
            (remainingM <= Config::kDistanceToleranceM) &&
            ((exitMagnitudeMps <= Config::kSpeedToleranceMps) ?
                MazeMap::IsMissionStartupStationaryFromSensors(
                    state.driveTelemetry.leftVelocityMps,
                    state.driveTelemetry.rightVelocityMps,
                    state.sensors.gyroRadps,
                    ResolveMotionSettleSpeedThresholdMps(),
                    Config::kMotionSettleAngularSpeedThresholdRadps) :
                (std::fabs(state.estimate.linearSpeedMps - desiredStraightSpeedMps) <= Config::kSpeedToleranceMps));

        return _drive->PointControlVectorWithHeadingTarget(
            desiredStraightSpeedMps,
            0.0f,
            maneuverState.startYawRad,
            _commandPdSettings.velocity | _commandPdSettings.yawRate,
            _commandPdSettings.heading);
    }
}
