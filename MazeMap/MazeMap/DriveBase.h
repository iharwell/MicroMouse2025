#pragma once
// Defines the runtime drive controller that owns motor actuation, odometry, and closed-loop wheel control.
#include "BootUtilityModeFramework.h"
#include "InPlaceTurnProfile.h"
#include "LaunchAssistProfile.h"
#include "MazeMapRuntimeCore.h"
#include "MotorEncoderDrive.h"
#include "MouseUkfFacade.h"
#include "OpenLoopDriveCommand.h"
#include "WheelControlProfile.h"

// Private drive runtime implementation for the MazeMap application runtime.
inline MazeMap::WheelControlProfile BuildNominalWheelControlProfile()
{
    MazeMap::WheelControlProfile profile{};
    profile.velocityKpScale = Config::kNominalWheelVelocityKpScale;
    profile.velocityKiScale = Config::kNominalWheelVelocityKiScale;
    profile.integralLimitScale = Config::kNominalWheelIntegralLimitScale;
    return profile;
}

inline MazeMap::WheelControlProfile BuildMappingWheelControlProfile()
{
    MazeMap::WheelControlProfile profile = BuildNominalWheelControlProfile();
    profile.accelerationResponseScale = Config::kMappingWheelAccelerationResponseScale;
    return profile;
}

namespace MazeMap::Internal
{
    inline void ResolveVelocityTargetAsapAccelerations(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        float longitudinalAccelLimitMps2,
        float yawAccelLimitRadps2,
        float responseTimeS,
        float& desiredLongitudinalAccelMps2,
        float& desiredYawAccelRadps2) noexcept
    {
        desiredLongitudinalAccelMps2 = 0.0f;
        desiredYawAccelRadps2 = 0.0f;
        const float resolvedLongitudinalAccelLimitMps2 =
            (std::isfinite(longitudinalAccelLimitMps2) && (longitudinalAccelLimitMps2 > 0.0f)) ?
            longitudinalAccelLimitMps2 :
            0.0f;
        const float resolvedYawAccelLimitRadps2 =
            (std::isfinite(yawAccelLimitRadps2) && (yawAccelLimitRadps2 > 0.0f)) ?
            yawAccelLimitRadps2 :
            0.0f;

        const float resolvedResponseTimeS =
            (std::isfinite(responseTimeS) && (responseTimeS > 0.0f)) ?
            responseTimeS :
            MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS;
        if (!(resolvedResponseTimeS > 0.0f) || !std::isfinite(resolvedResponseTimeS))
        {
            return;
        }

        if (std::isfinite(currentForwardVelocityMps) && std::isfinite(targetForwardVelocityMps))
        {
            desiredLongitudinalAccelMps2 =
                (targetForwardVelocityMps - currentForwardVelocityMps) / resolvedResponseTimeS;
        }
        if (std::isfinite(currentYawRateRadps) && std::isfinite(targetYawRateRadps))
        {
            desiredYawAccelRadps2 =
                (targetYawRateRadps - currentYawRateRadps) / resolvedResponseTimeS;
        }

        if (!(resolvedLongitudinalAccelLimitMps2 > 0.0f))
        {
            desiredLongitudinalAccelMps2 = 0.0f;
        }
        if (!(resolvedYawAccelLimitRadps2 > 0.0f))
        {
            desiredYawAccelRadps2 = 0.0f;
        }

        const float normalizedLongitudinalDemand =
            (resolvedLongitudinalAccelLimitMps2 > 0.0f) ?
            (std::fabs(desiredLongitudinalAccelMps2) / resolvedLongitudinalAccelLimitMps2) :
            0.0f;
        const float normalizedYawDemand =
            (resolvedYawAccelLimitRadps2 > 0.0f) ?
            (std::fabs(desiredYawAccelRadps2) / resolvedYawAccelLimitRadps2) :
            0.0f;
        const float balanceScale =
            (std::max)(1.0f, (std::max)(normalizedLongitudinalDemand, normalizedYawDemand));

        desiredLongitudinalAccelMps2 /= balanceScale;
        desiredYawAccelRadps2 /= balanceScale;

        if (resolvedLongitudinalAccelLimitMps2 > 0.0f)
        {
            desiredLongitudinalAccelMps2 =
                (std::clamp)(
                    desiredLongitudinalAccelMps2,
                    -resolvedLongitudinalAccelLimitMps2,
                    resolvedLongitudinalAccelLimitMps2);
        }
        if (resolvedYawAccelLimitRadps2 > 0.0f)
        {
            desiredYawAccelRadps2 =
                (std::clamp)(
                    desiredYawAccelRadps2,
                    -resolvedYawAccelLimitRadps2,
                    resolvedYawAccelLimitRadps2);
        }
    }
}

inline MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(float maxAngularSpeedRadps, float angularAccelRadps2)
{
    MazeMap::InPlaceTurnProfile profile{};
    profile.maxAngularSpeedRadps = maxAngularSpeedRadps;
    profile.angularAccelRadps2 = angularAccelRadps2;
    profile.headingKp = Config::kTurnHeadingKp;
    profile.yawD = Config::kTurnYawD;
    profile.angleToleranceRad = Config::kAngleToleranceRad;
    profile.angularSpeedToleranceRadps = Config::kAngularSpeedToleranceRadps;
    return profile;
}

inline MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(const MazeMap::Vehicle& vehicle)
{
    return BuildSharedInPlaceTurnProfile(
        vehicle.GetMaxRotationalVelocity(),
        vehicle.GetMaxAngularAcceleration());
}

inline MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(const MotionLimits& limits)
{
    MazeMap::InPlaceTurnProfile profile = BuildSharedInPlaceTurnProfile(
        limits.maxAngularSpeedRadps,
        limits.angularAccelRadps2);
    profile.angleToleranceRad =
        (std::isfinite(limits.angleToleranceRad) && (limits.angleToleranceRad > 0.0f)) ?
        limits.angleToleranceRad :
        Config::kAngleToleranceRad;
    profile.angularSpeedToleranceRadps =
        (std::isfinite(limits.angularSpeedToleranceRadps) && (limits.angularSpeedToleranceRadps > 0.0f)) ?
        limits.angularSpeedToleranceRadps :
        Config::kAngularSpeedToleranceRadps;
    return profile;
}

// Serves as the runtime drive subsystem for the MazeMap application by coordinating motors, odometry, and motion control.
class DriveBase
{
private:
    struct ClosedLoopVelocityCommand;
    static constexpr float kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2 = 9.0f;
    static constexpr float kDefaultCommandVelocityAsapYawAccelLimitRadps2 = 400.0f;
    bool startSet = false;
public:
    static constexpr uint16_t kModeClosedLoop = 1u << 0;
    static constexpr uint16_t kModeOpenLoop = 1u << 1;
    static constexpr uint16_t kModeRawOpenLoop = 1u << 2;
    static constexpr uint16_t kModeBraking = 1u << 3;
    static constexpr uint16_t kModeLaunchAssistLeft = 1u << 4;
    static constexpr uint16_t kModeLaunchAssistRight = 1u << 5;

    DriveBase()
        : _leftMotor(MazeMap::MotorEncoderDrive::CreateDefaultLeftDrive())
        , _rightMotor(MazeMap::MotorEncoderDrive::CreateDefaultRightDrive())
        , _ukf(MazeMap::PlantParams::Default())
        , _poseCache{}
        , _leftIntegral(0.0f)
        , _rightIntegral(0.0f)
        , _lastLinearCommandMps(0.0f)
        , _lastAngularCommandRadps(0.0f)
        , _wheelControlProfile(BuildNominalWheelControlProfile())
    {
    }

    bool Begin()
    {
        const bool leftOk = _leftMotor.begin();
        const bool rightOk = _rightMotor.begin();
        _leftMotor.resetEncoderDistanceMeters();
        _rightMotor.resetEncoderDistanceMeters();
        ResetEncoderTracking();
        ResetPoseEstimate(0.0f, 0.0f, DirectionToYawRad(MazeMap::Up));
        ResetControllers();
        Brake();
        return leftOk && rightOk;
    }

    void ResetControllers()
    {
        _leftIntegral = 0.0f;
        _rightIntegral = 0.0f;
        ResetLaunchAssist();
    }

    void SetWheelControlProfile(const MazeMap::WheelControlProfile& profile)
    {
        _wheelControlProfile = MazeMap::NormalizeWheelControlProfile(profile);
        const float integralLimit = GetWheelIntegralLimit();
        _leftIntegral = (std::clamp)(_leftIntegral, -integralLimit, integralLimit);
        _rightIntegral = (std::clamp)(_rightIntegral, -integralLimit, integralLimit);
    }

    void UseNominalWheelControlProfile()
    {
        SetWheelControlProfile(BuildNominalWheelControlProfile());
    }

    void SetStartPoint(MazeMap::DirectionalLocation logical)
    {
		assert(!startSet);
        float xMeters = 0.0f;
        float yMeters = 0.0f;
        logical.GetLocation().GetPhysicalLocation(Config::kCellSizeM, xMeters, yMeters);
        ResetPoseEstimate(xMeters, yMeters, DirectionToYawRad(logical.GetDirection()));
        ResetControllers();
		startSet = true;
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPose(float xMeters, float yMeters, float yawRad)
    {
        ResetPoseEstimate(xMeters, yMeters, yawRad);
        ResetControllers();
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPoseXMeters(float xMeters)
    {
        if (std::isfinite(xMeters))
        {
            SetEstimatorCoordinate(MazeMap::VehicleState::kPx, xMeters);
        }
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPoseYMeters(float yMeters)
    {
        if (std::isfinite(yMeters))
        {
            SetEstimatorCoordinate(MazeMap::VehicleState::kPy, yMeters);
        }
    }

    void UpdateOdometry(
        float dtSeconds,
        const SensorSnapshot& snapshot,
        const MazeMap::Maze* map = nullptr,
        ControlCycleTiming* timing = nullptr)
    {
        UpdatePoseEstimate(
            dtSeconds,
            snapshot,
            map,
            timing,
            MazeMap::NoopUkfLoopHook{},
            []() noexcept {});
    }

    template <typename LoopHook, typename BeforeYawUpdate>
    void UpdateOdometry(
        float dtSeconds,
        const SensorSnapshot& snapshot,
        const MazeMap::Maze* map,
        ControlCycleTiming* timing,
        LoopHook&& loopHook,
        BeforeYawUpdate&& beforeYawUpdate)
    {
        UpdatePoseEstimate(dtSeconds, snapshot, map, timing, loopHook, beforeYawUpdate);
    }

    void UpdateOdometry(
        float dtSeconds,
        const DiagnosticSensorSnapshot& snapshot,
        const MazeMap::Maze* map = nullptr,
        ControlCycleTiming* timing = nullptr)
    {
        UpdatePoseEstimate(
            dtSeconds,
            snapshot,
            map,
            timing,
            MazeMap::NoopUkfLoopHook{},
            []() noexcept {});
    }

    template <typename LoopHook, typename BeforeYawUpdate>
    void UpdateOdometry(
        float dtSeconds,
        const DiagnosticSensorSnapshot& snapshot,
        const MazeMap::Maze* map,
        ControlCycleTiming* timing,
        LoopHook&& loopHook,
        BeforeYawUpdate&& beforeYawUpdate)
    {
        UpdatePoseEstimate(dtSeconds, snapshot, map, timing, loopHook, beforeYawUpdate);
    }

    struct MeasuredKinematics
    {
        float leftVelocityMps = 0.0f;
        float rightVelocityMps = 0.0f;
        float linearSpeedMps = 0.0f;
        float angularSpeedRadps = 0.0f;
    };

    MeasuredKinematics GetMeasuredKinematics(
        float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN()) const
    {
        MeasuredKinematics kinematics{};
        const MazeMap::PlantParams& params = _ukf.ukf().params();
        kinematics.leftVelocityMps = _leftEncoderVelocityMps;
        kinematics.rightVelocityMps = _rightEncoderVelocityMps;
        kinematics.linearSpeedMps = 0.5f * (kinematics.leftVelocityMps + kinematics.rightVelocityMps);
        float trackWidthM = params.trackWidthM;
        if (!(trackWidthM > 0.0f) || !std::isfinite(trackWidthM))
        {
            trackWidthM = MazeMap::Vehicle::GetPhysicalModel().trackWidthM;
        }

        const float fallbackYawRateRadps =
            ((trackWidthM > 0.0f) && std::isfinite(trackWidthM)) ?
            ((kinematics.leftVelocityMps - kinematics.rightVelocityMps) / trackWidthM) :
            0.0f;
        kinematics.angularSpeedRadps =
            std::isfinite(measuredYawRateRadps) ?
            measuredYawRateRadps :
            fallbackYawRateRadps;
        return kinematics;
    }

    void ProjectMeasuredKinematics(float dtSeconds, float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN())
    {
        if (_estimatorFaulted)
        {
            return;
        }

        MazeMap::VehicleState::StateVector state = _ukf.ukf().state();
        const MazeMap::VehicleState::StateMatrix covariance = _ukf.ukf().covariance();
        const MazeMap::PlantParams& params = _ukf.ukf().params();
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            SyncPoseEstimate();
            return;
        }

        const MeasuredKinematics measured = GetMeasuredKinematics(measuredYawRateRadps);

        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float midYawRad =
                WrapAngleRad(state(MazeMap::VehicleState::kPsi) + (0.5f * measured.angularSpeedRadps * dtSeconds));
            const Eigen::Vector2f midHeading = HeadingUnitFromYawRad(midYawRad);
            state(MazeMap::VehicleState::kPx) += measured.linearSpeedMps * midHeading.x() * dtSeconds;
            state(MazeMap::VehicleState::kPy) += measured.linearSpeedMps * midHeading.y() * dtSeconds;
            state(MazeMap::VehicleState::kPsi) =
                WrapAngleRad(state(MazeMap::VehicleState::kPsi) + (measured.angularSpeedRadps * dtSeconds));
        }

        state(MazeMap::VehicleState::kU) = measured.linearSpeedMps;
        state(MazeMap::VehicleState::kV) = 0.0f;
        state(MazeMap::VehicleState::kR) = measured.angularSpeedRadps;
        state(MazeMap::VehicleState::kOmegaL) = measured.leftVelocityMps / params.wheelRadiusM;
        state(MazeMap::VehicleState::kOmegaR) = measured.rightVelocityMps / params.wheelRadiusM;
        MazeMap::VehicleState::NormalizeStateVector(state);
        (void)_ukf.ukf().setState(state, covariance);
        SyncPoseEstimate();
    }

    // CommandVelocity assumes callers want the requested linear speed and yaw rate as soon as practical.
    // The default path resolves the current operating point, applies the canonical default operating envelope
    // subject to the smaller plant-reported technical limits, then forwards through the explicit envelope
    // overload below so the plant feedforward carries nearly all of the transition effort.
    void CommandVelocity(float linearSpeedMps, float angularSpeedRadps, float dtSeconds)
    {
        if (_estimatorFaulted)
        {
            Brake();
            return;
        }

        MazeMap::VehicleState::StateVector presentState = MazeMap::VehicleState::StateVector::Zero();
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingState(presentState, batteryVoltageV);
        (void)batteryVoltageV;

        float maxLongitudinalAccelMps2 = kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2;
        float maxYawAccelRadps2 = kDefaultCommandVelocityAsapYawAccelLimitRadps2;
        ResolveDefaultVelocityTargetOperatingEnvelope(
            presentState,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
        const ClosedLoopVelocityCommand command =
            BuildClosedLoopVelocityCommandForVelocityTarget(
                linearSpeedMps,
                angularSpeedRadps,
                presentState,
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);
        CommandVelocityInternal(
            linearSpeedMps,
            angularSpeedRadps,
            command,
            dtSeconds);
    }

    // Explicit operating-envelope overload for callers that already know the present body rates and the
    // acceleration envelope they want the plant feedforward to honor over the canonical response horizon.
    void CommandVelocity(
        float linearSpeedMps,
        float angularSpeedRadps,
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float maxLongitudinalAccelMps2,
        float maxYawAccelRadps2,
        float dtSeconds)
    {
        if (_estimatorFaulted)
        {
            Brake();
            return;
        }

        const ClosedLoopVelocityCommand command =
            BuildClosedLoopVelocityCommandForVelocityTarget(
                linearSpeedMps,
                angularSpeedRadps,
                presentLinearSpeedMps,
                presentYawRateRadps,
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);
        CommandVelocityInternal(linearSpeedMps, angularSpeedRadps, command, dtSeconds);
    }

    void CommandVelocity(
        float linearSpeedMps,
        float angularSpeedRadps,
        float desiredAccelerationMps2,
        float dtSeconds)
    {
        if (_estimatorFaulted)
        {
            Brake();
            return;
        }

        const ClosedLoopVelocityCommand command =
            BuildClosedLoopVelocityCommand(
                linearSpeedMps,
                angularSpeedRadps,
                desiredAccelerationMps2);
        CommandVelocityInternal(linearSpeedMps, angularSpeedRadps, command, dtSeconds);
    }

    MazeMap::OpenLoopDriveCommand ResolveAccelerationDriveCommandRaw(
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float dtSeconds)
    {
        if (_estimatorFaulted)
        {
            return {};
        }

        float batteryVoltageV = 0.0f;
        MazeMap::VehicleState::StateVector unusedPresentState = MazeMap::VehicleState::StateVector::Zero();
        GetVelocityCommandOperatingState(unusedPresentState, batteryVoltageV);

        const float resolvedPresentLinearSpeedMps =
            (std::isfinite(presentLinearSpeedMps)) ?
            presentLinearSpeedMps :
            0.0f;
        const float resolvedPresentYawRateRadps =
            (std::isfinite(presentYawRateRadps)) ?
            presentYawRateRadps :
            0.0f;

        const float resolvedLongitudinalAccelMps2 =
            (std::isfinite(desiredLongitudinalAccelMps2)) ?
            desiredLongitudinalAccelMps2 :
            0.0f;
        const float resolvedYawAccelRadps2 =
            (std::isfinite(desiredYawAccelRadps2)) ?
            desiredYawAccelRadps2 :
            0.0f;

        // Raw command resolution must stay feedforward-only; wheel-speed PI belongs to CommandVelocity().
        MazeMap::PlantModel plantModel;
        const MazeMap::DriveCommandSolution solution =
            plantModel.solveClosedLoopDriveCommands(
                resolvedPresentLinearSpeedMps,
                resolvedLongitudinalAccelMps2,
                resolvedPresentYawRateRadps,
                resolvedYawAccelRadps2,
                _ukf.ukf().preparedParams(),
                GetMissionFanDutyCycle(),
                batteryVoltageV);
        (void)dtSeconds;
        return MazeMap::ClampOpenLoopDriveCommand(
            MazeMap::MakeOpenLoopDriveCommand(
                solution.control.leftMotorCommand,
                solution.control.rightMotorCommand));
    }

    MazeMap::OpenLoopDriveCommand ResolveVelocityDriveCommandRaw(
        float linearSpeedMps,
        float angularSpeedRadps,
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float maxLongitudinalAccelMps2,
        float maxYawAccelRadps2,
        float dtSeconds)
    {
        if (_estimatorFaulted)
        {
            return {};
        }

        const ClosedLoopVelocityCommand command =
            BuildClosedLoopVelocityCommandForVelocityTarget(
                linearSpeedMps,
                angularSpeedRadps,
                presentLinearSpeedMps,
                presentYawRateRadps,
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);
        return ResolveClosedLoopVelocityDriveSignal(command, dtSeconds).driveCommand;
    }

    MazeMap::OpenLoopDriveCommand ResolveStraightHeadingHoldAccelerationDriveCommandRaw(
        float targetYawRad,
        float measuredYawRad,
        float presentLinearSpeedMps,
        float estimatedYawRateRadps,
        float desiredLongitudinalAccelMps2,
        float dtSeconds,
        float* resolvedAngularCommandRadps = nullptr)
    {
        const float resolvedLongitudinalAccelMps2 =
            std::isfinite(desiredLongitudinalAccelMps2) ?
            desiredLongitudinalAccelMps2 :
            0.0f;
        const float resolvedPresentLinearSpeedMps =
            std::isfinite(presentLinearSpeedMps) ?
            presentLinearSpeedMps :
            0.0f;
        const float resolvedEstimatedYawRateRadps =
            std::isfinite(estimatedYawRateRadps) ?
            estimatedYawRateRadps :
            0.0f;
        const float angularCommandRadps =
            ResolveStraightHeadingYawRateCommand(
                targetYawRad,
                measuredYawRad,
                resolvedEstimatedYawRateRadps);
        if (resolvedAngularCommandRadps != nullptr)
        {
            *resolvedAngularCommandRadps = angularCommandRadps;
        }

        const float responseTimeS =
            (std::isfinite(MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS) &&
             (MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS > 0.0f)) ?
            MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS :
            0.0f;
        const float desiredYawAccelRadps2 =
            (responseTimeS > 0.0f) ?
            ((angularCommandRadps - resolvedEstimatedYawRateRadps) / responseTimeS) :
            0.0f;

        return ResolveAccelerationDriveCommandRaw(
            resolvedPresentLinearSpeedMps,
            resolvedEstimatedYawRateRadps,
            resolvedLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            dtSeconds);
    }

    void CommandVelocityInternal(
        float linearSpeedMps,
        float angularSpeedRadps,
        const ClosedLoopVelocityCommand& command,
        float dtSeconds)
    {
        _lastLinearCommandMps = linearSpeedMps;
        _lastAngularCommandRadps = angularSpeedRadps;
        const ResolvedVelocityDriveSignal resolved = ResolveClosedLoopVelocityDriveSignal(command, dtSeconds);
        _lastLeftFeedforwardCommand = resolved.leftFeedforwardCommand;
        _lastRightFeedforwardCommand = resolved.rightFeedforwardCommand;
        _lastLeftFeedbackCommand = resolved.driveCommand.leftDriveCommand - resolved.leftFeedforwardCommand;
        _lastRightFeedbackCommand = resolved.driveCommand.rightDriveCommand - resolved.rightFeedforwardCommand;
        _lastLeftTargetVelocityMps = resolved.leftTargetMps;
        _lastRightTargetVelocityMps = resolved.rightTargetMps;
        _lastLeftLaunchAssistFloor = resolved.leftLaunchAssistFloor;
        _lastRightLaunchAssistFloor = resolved.rightLaunchAssistFloor;
        _lastModeFlags = kModeClosedLoop |
            ((_lastLeftLaunchAssistFloor > 0.0f) ? kModeLaunchAssistLeft : 0u) |
            ((_lastRightLaunchAssistFloor > 0.0f) ? kModeLaunchAssistRight : 0u);
        _lastSaturationFlags =
            ((std::fabs(resolved.driveCommand.leftDriveCommand) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(resolved.driveCommand.rightDriveCommand) >= 0.999f) ? 0x2u : 0u);
        _leftMotor.setDriveCommand(resolved.driveCommand.leftDriveCommand);
        _rightMotor.setDriveCommand(resolved.driveCommand.rightDriveCommand);
    }

    void CommandOpenLoop(float leftDriveCommand, float rightDriveCommand)
    {
        if (_estimatorFaulted)
        {
            Brake();
            return;
        }

        const float leftMeasuredMps = _leftEncoderVelocityMps;
        const float rightMeasuredMps = _rightEncoderVelocityMps;
        const unsigned long nowMs = millis();
        if (UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftDriveCommand, _leftMotor.getDriveCommand(), nowMs))
        {
            _lastLeftLaunchAssistFloor = GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs);
            leftDriveCommand = ApplyLaunchAssistFloor(
                leftDriveCommand,
                leftDriveCommand,
                _lastLeftLaunchAssistFloor);
        }
        else
        {
            _lastLeftLaunchAssistFloor = 0.0f;
        }
        if (UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightDriveCommand, _rightMotor.getDriveCommand(), nowMs))
        {
            _lastRightLaunchAssistFloor = GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs);
            rightDriveCommand = ApplyLaunchAssistFloor(
                rightDriveCommand,
                rightDriveCommand,
                _lastRightLaunchAssistFloor);
        }
        else
        {
            _lastRightLaunchAssistFloor = 0.0f;
        }

        _lastLeftFeedforwardCommand = 0.0f;
        _lastRightFeedforwardCommand = 0.0f;
        _lastLeftFeedbackCommand = leftDriveCommand;
        _lastRightFeedbackCommand = rightDriveCommand;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastModeFlags = kModeOpenLoop |
            ((_lastLeftLaunchAssistFloor > 0.0f) ? kModeLaunchAssistLeft : 0u) |
            ((_lastRightLaunchAssistFloor > 0.0f) ? kModeLaunchAssistRight : 0u);
        _lastSaturationFlags =
            ((std::fabs(leftDriveCommand) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(rightDriveCommand) >= 0.999f) ? 0x2u : 0u);
        SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
    }

    void CommandOpenLoop(const MazeMap::OpenLoopDriveCommand& command)
    {
        CommandOpenLoop(command.leftDriveCommand, command.rightDriveCommand);
    }

    void CommandOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        if (_estimatorFaulted)
        {
            Brake();
            return;
        }

        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _lastLeftFeedforwardCommand = 0.0f;
        _lastRightFeedforwardCommand = 0.0f;
        _lastLeftFeedbackCommand = leftDriveCommand;
        _lastRightFeedbackCommand = rightDriveCommand;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastLeftLaunchAssistFloor = 0.0f;
        _lastRightLaunchAssistFloor = 0.0f;
        _lastModeFlags = kModeRawOpenLoop;
        _lastSaturationFlags =
            ((std::fabs(leftDriveCommand) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(rightDriveCommand) >= 0.999f) ? 0x2u : 0u);
        SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
    }

    void CommandOpenLoopRaw(const MazeMap::OpenLoopDriveCommand& command)
    {
        CommandOpenLoopRaw(command.leftDriveCommand, command.rightDriveCommand);
    }

    void Brake()
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _lastLeftFeedforwardCommand = 0.0f;
        _lastRightFeedforwardCommand = 0.0f;
        _lastLeftFeedbackCommand = 0.0f;
        _lastRightFeedbackCommand = 0.0f;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastLeftLaunchAssistFloor = 0.0f;
        _lastRightLaunchAssistFloor = 0.0f;
        _lastModeFlags = kModeBraking;
        _lastSaturationFlags = 0u;
        _leftMotor.brake();
        _rightMotor.brake();
    }

    float GetAverageDistanceMeters() const
    {
        return 0.5f * (
            (_leftEncoderDistanceMeters + _leftMotor.pulsesToDistance(_leftMotor.getEncoderCount())) +
            (_rightEncoderDistanceMeters + _rightMotor.pulsesToDistance(_rightMotor.getEncoderCount())));
    }

    const PoseEstimate& GetPose() const
    {
        return _poseCache;
    }

    const MazeMap::VehicleState::StateVector& GetEstimatorStateVector() const noexcept
    {
        return _ukf.ukf().state();
    }

    bool HasEstimatorFault() const noexcept
    {
        return _estimatorFaulted;
    }

    const char* GetEstimatorFaultReason() const noexcept
    {
        return (_estimatorFaultReason[0] != '\0') ? _estimatorFaultReason : "ukf_failure";
    }

    template <typename Sink>
    bool WriteUkfDebugTextDump(Sink&& sink) const noexcept
    {
        return _ukf.ukf().WriteDebugTextDump(static_cast<Sink&&>(sink));
    }

    float GetLastLinearCommandMps() const
    {
        return _lastLinearCommandMps;
    }

    float GetLastAngularCommandRadps() const
    {
        return _lastAngularCommandRadps;
    }

    DriveTelemetry GetTelemetry() const
    {
        DriveTelemetry telemetry{};
        const int32_t pendingLeftCounts = _leftMotor.getEncoderCount();
        const int32_t pendingRightCounts = _rightMotor.getEncoderCount();
        telemetry.leftDriveCommand = _leftMotor.getDriveCommand();
        telemetry.rightDriveCommand = _rightMotor.getDriveCommand();
        telemetry.leftFeedforwardCommand = _lastLeftFeedforwardCommand;
        telemetry.rightFeedforwardCommand = _lastRightFeedforwardCommand;
        telemetry.leftFeedbackCommand = _lastLeftFeedbackCommand;
        telemetry.rightFeedbackCommand = _lastRightFeedbackCommand;
        telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
        telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
        telemetry.leftLaunchAssistFloor = _lastLeftLaunchAssistFloor;
        telemetry.rightLaunchAssistFloor = _lastRightLaunchAssistFloor;
        telemetry.leftEncoderCount = _leftEncoderCountTotal + pendingLeftCounts;
        telemetry.rightEncoderCount = _rightEncoderCountTotal + pendingRightCounts;
        telemetry.leftDistanceM = _leftEncoderDistanceMeters + _leftMotor.pulsesToDistance(pendingLeftCounts);
        telemetry.rightDistanceM = _rightEncoderDistanceMeters + _rightMotor.pulsesToDistance(pendingRightCounts);
        telemetry.leftVelocityMps = _leftEncoderVelocityMps;
        telemetry.rightVelocityMps = _rightEncoderVelocityMps;
        telemetry.leftEncoderOmegaRadps = _lastEncoderObservation.omegaLeftRadps;
        telemetry.rightEncoderOmegaRadps = _lastEncoderObservation.omegaRightRadps;
        telemetry.modeFlags = _lastModeFlags;
        telemetry.saturationFlags = _lastSaturationFlags;
        telemetry.ukfModeId = _ukf.ukf().operatingModeId();
        telemetry.ukfYawValidForFeedforward = _ukf.ukf().yawValidForFeedforward() ? 1U : 0U;
        telemetry.ukfBiasUpdateEnabled = _ukf.ukf().biasUpdateEnabled() ? 1U : 0U;
        telemetry.ukfNhcEnabled = _ukf.ukf().nonholonomicConstraintEnabled() ? 1U : 0U;
        telemetry.ukfGyroBiasAnchorRadps = _ukf.ukf().gyroBiasAnchorRadps();
        telemetry.ukfYawConsistencyLowPassRadps = _ukf.ukf().yawConsistencyLowPassRadps();
        telemetry.ukfYawWindowMismatchRad = _ukf.ukf().yawWindowMismatchRad();
        telemetry.ukfNhcSigmaMps = _ukf.ukf().nhcSigmaMps();
        telemetry.ukfNhcResidualMps = _ukf.ukf().nhcResidualMps();
        telemetry.ukfNhcResidualSigma = _ukf.ukf().nhcResidualSigma();
        telemetry.ukfFeedforwardYawRateRadps = _ukf.ukf().resolveYawRateForFeedforward(_lastGyroRawRadps);
        telemetry.encoderObservationValid = _encoderObservationValid;
        return telemetry;
    }

    static void BuildLoggedFrontPairObservations(
        const DiagnosticSensorSnapshot& snapshot,
        float maxRangeM,
        MazeMap::WallObs& left,
        MazeMap::WallObs& right) noexcept
    {
        BuildUkfFrontPairObservations(snapshot, maxRangeM, left, right);
    }

    static MazeMap::WallObs BuildLoggedLeftSideObservation(
        const DiagnosticSensorSnapshot& snapshot,
        float maxRangeM) noexcept
    {
        return BuildUkfLeftSideObservation(snapshot, maxRangeM);
    }

    static MazeMap::WallObs BuildLoggedRightSideObservation(
        const DiagnosticSensorSnapshot& snapshot,
        float maxRangeM) noexcept
    {
        return BuildUkfRightSideObservation(snapshot, maxRangeM);
    }

private:
    struct EncoderCycleSample
    {
        int32_t leftCounts = 0;
        int32_t rightCounts = 0;
        float leftDistanceM = 0.0f;
        float rightDistanceM = 0.0f;
        float leftVelocityMps = 0.0f;
        float rightVelocityMps = 0.0f;
        float leftOmegaRadps = 0.0f;
        float rightOmegaRadps = 0.0f;
    };

    static MazeMap::VehicleState::StateMatrix BuildEstimatorCovariance()
    {
        return MazeMap::SrUkfCore::BuildDefaultInitialCovariance();
    }

    void ResetEncoderTracking() noexcept
    {
        _leftEncoderCountTotal = 0;
        _rightEncoderCountTotal = 0;
        _leftEncoderDistanceMeters = 0.0f;
        _rightEncoderDistanceMeters = 0.0f;
        _leftEncoderVelocityMps = 0.0f;
        _rightEncoderVelocityMps = 0.0f;
        _lastEncoderObservation = MazeMap::EncoderObs{};
        _encoderObservationValid = false;
    }

    EncoderCycleSample ConsumeEncoderCycleSample(float dtSeconds, const MazeMap::PlantParams& params)
    {
        EncoderCycleSample sample{};
        sample.leftCounts = _leftMotor.consumeEncoderCount();
        sample.rightCounts = _rightMotor.consumeEncoderCount();
        sample.leftDistanceM = _leftMotor.pulsesToDistance(sample.leftCounts);
        sample.rightDistanceM = _rightMotor.pulsesToDistance(sample.rightCounts);

        _leftEncoderCountTotal += sample.leftCounts;
        _rightEncoderCountTotal += sample.rightCounts;
        _leftEncoderDistanceMeters += sample.leftDistanceM;
        _rightEncoderDistanceMeters += sample.rightDistanceM;

        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float invDt = 1.0f / dtSeconds;
            sample.leftVelocityMps = sample.leftDistanceM * invDt;
            sample.rightVelocityMps = sample.rightDistanceM * invDt;
        }

        _leftEncoderVelocityMps = sample.leftVelocityMps;
        _rightEncoderVelocityMps = sample.rightVelocityMps;

        if ((params.wheelRadiusM > 0.0f) && std::isfinite(params.wheelRadiusM))
        {
            const float invWheelRadiusM = 1.0f / params.wheelRadiusM;
            sample.leftOmegaRadps = sample.leftVelocityMps * invWheelRadiusM;
            sample.rightOmegaRadps = sample.rightVelocityMps * invWheelRadiusM;
        }

        return sample;
    }

    void ResetPoseEstimate(float xMeters, float yMeters, float yawRad)
    {
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(MazeMap::VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(MazeMap::VehicleState::kPsi) = WrapAngleRad(yawRad);
        ClearEstimatorFault();
        (void)_ukf.reset(state, BuildEstimatorCovariance());
        SyncPoseEstimate();
    }

    void SetEstimatorCoordinate(int stateIndex, float coordinateM)
    {
        MazeMap::VehicleState::StateVector state = _ukf.ukf().state();
        const MazeMap::VehicleState::StateMatrix covariance = _ukf.ukf().covariance();
        state(stateIndex) = coordinateM;
        MazeMap::VehicleState::NormalizeStateVector(state);
        (void)_ukf.ukf().setState(state, covariance);
        SyncPoseEstimate();
    }

    void SyncPoseEstimate()
    {
        const MazeMap::VehicleState::StateVector& state = _ukf.ukf().state();
        _poseCache.xMeters = state(MazeMap::VehicleState::kPx);
        _poseCache.yMeters = state(MazeMap::VehicleState::kPy);
        _poseCache.yawRad = WrapAngleRad(state(MazeMap::VehicleState::kPsi));
        _poseCache.headingUnit = HeadingUnitFromYawRad(_poseCache.yawRad);
        _poseCache.linearSpeedMps = state(MazeMap::VehicleState::kU);
        _poseCache.angularSpeedRadps = state(MazeMap::VehicleState::kR);
    }

    MazeMap::LocalMapView BuildUkfMapView(const MazeMap::Maze* map) const
    {
        MazeMap::LocalMapView view{};
        if (map == nullptr)
        {
            return view;
        }

        view.maze = map;
        view.cellSizeM = Config::kCellSizeM;
        view.wallThicknessM = Config::kMazeWallThicknessM;
        view.postHalfWidthM = 0.5f * Config::kMazeWallThicknessM;
        view.noHitRangeM = _ukf.ukf().params().noHitRangeM;
        view.freezeMapMutation = true;
        return view;
    }

    static bool IsFinitePositive(float value) noexcept
    {
        return std::isfinite(value) && (value > 0.0f);
    }

    static float ClampMeasuredRange(float value, float maxRangeM) noexcept
    {
        return (std::clamp)(value, 0.01f, maxRangeM);
    }

    void ClearEstimatorFault() noexcept
    {
        _estimatorFaulted = false;
        _estimatorFaultReason[0] = '\0';
    }

    void TriggerEstimatorFault(const char* reason) noexcept
    {
        Brake();
        if (_estimatorFaulted)
        {
            return;
        }

        _estimatorFaulted = true;
        std::snprintf(
            _estimatorFaultReason,
            sizeof(_estimatorFaultReason),
            "%s",
            (reason != nullptr && reason[0] != '\0') ? reason : "ukf_failure");

        char traceLine[96] = {};
        std::snprintf(traceLine, sizeof(traceLine), "ukf_fault:%s", _estimatorFaultReason);
        MazeMap::App::Internal::BootUtilityModeFramework::AppendStartupTrace(traceLine);
    }

    static MazeMap::ImuMergedObs BuildUkfImuObservation(const SensorSnapshot& snapshot) noexcept
    {
        MazeMap::ImuMergedObs observation{};
        if (!snapshot.accelBiasValid ||
            !std::isfinite(snapshot.gyroRadps) ||
            !std::isfinite(snapshot.accelBodyXMps2) ||
            !std::isfinite(snapshot.accelBodyYMps2))
        {
            return observation;
        }

        observation.valid = true;
        observation.gyroZRadps = snapshot.gyroRawRadps;
        observation.accelBodyXMps2 = snapshot.accelBodyXMps2;
        observation.accelBodyYMps2 = snapshot.accelBodyYMps2;
        return observation;
    }

    static MazeMap::ImuMergedObs BuildUkfImuObservation(const DiagnosticSensorSnapshot& snapshot) noexcept
    {
        MazeMap::ImuMergedObs observation{};
        if (!snapshot.accelBiasValid ||
            !std::isfinite(snapshot.gyroRadps) ||
            !std::isfinite(snapshot.accelBodyXMps2) ||
            !std::isfinite(snapshot.accelBodyYMps2))
        {
            return observation;
        }

        observation.valid = true;
        observation.gyroZRadps = snapshot.gyroRawRadps;
        observation.accelBodyXMps2 = snapshot.accelBodyXMps2;
        observation.accelBodyYMps2 = snapshot.accelBodyYMps2;
        return observation;
    }

    static void BuildUkfFrontPairObservations(
        const SensorSnapshot& snapshot,
        float maxRangeM,
        MazeMap::WallObs& left,
        MazeMap::WallObs& right) noexcept
    {
        left = MazeMap::WallObs{};
        right = MazeMap::WallObs{};
        if (!snapshot.frontWallObservationValid ||
            !snapshot.frontWall ||
            !IsFinitePositive(snapshot.frontLeftDistanceM) ||
            !IsFinitePositive(snapshot.frontRightDistanceM))
        {
            return;
        }

        const float confidence =
            snapshot.frontWallUsesCharacterizationDetection ? 0.90f :
            (snapshot.frontWallUsesFallbackDetection ? 0.60f : 0.80f);
        left.valid = true;
        left.rho = ClampMeasuredRange(snapshot.frontLeftDistanceM, maxRangeM);
        left.confidence = confidence;
        left.cls = MazeMap::ObsClass::WallLike;
        right.valid = true;
        right.rho = ClampMeasuredRange(snapshot.frontRightDistanceM, maxRangeM);
        right.confidence = confidence;
        right.cls = MazeMap::ObsClass::WallLike;
    }

    static void BuildUkfFrontPairObservations(
        const DiagnosticSensorSnapshot& snapshot,
        float maxRangeM,
        MazeMap::WallObs& left,
        MazeMap::WallObs& right) noexcept
    {
        left = MazeMap::WallObs{};
        right = MazeMap::WallObs{};
        if (!snapshot.frontWall ||
            !IsFinitePositive(snapshot.frontLeft.distanceM) ||
            !IsFinitePositive(snapshot.frontRight.distanceM))
        {
            return;
        }

        left.valid = true;
        left.rho = ClampMeasuredRange(snapshot.frontLeft.distanceM, maxRangeM);
        left.confidence = 0.80f;
        left.cls = MazeMap::ObsClass::WallLike;
        right.valid = true;
        right.rho = ClampMeasuredRange(snapshot.frontRight.distanceM, maxRangeM);
        right.confidence = 0.80f;
        right.cls = MazeMap::ObsClass::WallLike;
    }

    static MazeMap::WallObs BuildUkfLeftSideObservation(const SensorSnapshot& snapshot, float maxRangeM) noexcept
    {
        MazeMap::WallObs observation{};
        if (!snapshot.leftDistanceValidForControl ||
            snapshot.leftTransitionDetected ||
            !snapshot.leftWallObservation ||
            !IsFinitePositive(snapshot.sideLeftDistanceM))
        {
            return observation;
        }

        observation.valid = true;
        observation.rho = ClampMeasuredRange(snapshot.sideLeftDistanceM, maxRangeM);
        observation.confidence = 0.80f;
        observation.cls = MazeMap::ObsClass::WallLike;
        return observation;
    }

    static MazeMap::WallObs BuildUkfRightSideObservation(const SensorSnapshot& snapshot, float maxRangeM) noexcept
    {
        MazeMap::WallObs observation{};
        if (!snapshot.rightDistanceValidForControl ||
            snapshot.rightTransitionDetected ||
            !snapshot.rightWallObservation ||
            !IsFinitePositive(snapshot.sideRightDistanceM))
        {
            return observation;
        }

        observation.valid = true;
        observation.rho = ClampMeasuredRange(snapshot.sideRightDistanceM, maxRangeM);
        observation.confidence = 0.80f;
        observation.cls = MazeMap::ObsClass::WallLike;
        return observation;
    }

    static MazeMap::WallObs BuildUkfLeftSideObservation(const DiagnosticSensorSnapshot& snapshot, float maxRangeM) noexcept
    {
        MazeMap::WallObs observation{};
        if (!snapshot.leftDistanceValidForControl ||
            !snapshot.leftWall ||
            !IsFinitePositive(snapshot.sideLeft.distanceM))
        {
            return observation;
        }

        observation.valid = true;
        observation.rho = ClampMeasuredRange(snapshot.sideLeft.distanceM, maxRangeM);
        observation.confidence = 0.80f;
        observation.cls = MazeMap::ObsClass::WallLike;
        return observation;
    }

    static MazeMap::WallObs BuildUkfRightSideObservation(const DiagnosticSensorSnapshot& snapshot, float maxRangeM) noexcept
    {
        MazeMap::WallObs observation{};
        if (!snapshot.rightDistanceValidForControl ||
            !snapshot.rightWall ||
            !IsFinitePositive(snapshot.sideRight.distanceM))
        {
            return observation;
        }

        observation.valid = true;
        observation.rho = ClampMeasuredRange(snapshot.sideRight.distanceM, maxRangeM);
        observation.confidence = 0.80f;
        observation.cls = MazeMap::ObsClass::WallLike;
        return observation;
    }

    template <typename TSnapshot>
    void UpdatePoseEstimate(
        float dtSeconds,
        const TSnapshot& snapshot,
        const MazeMap::Maze* map,
        ControlCycleTiming* timing)
    {
        UpdatePoseEstimate(
            dtSeconds,
            snapshot,
            map,
            timing,
            MazeMap::NoopUkfLoopHook{},
            []() noexcept {});
    }

    template <typename TSnapshot, typename LoopHook, typename BeforeYawUpdate>
    void UpdatePoseEstimate(
        float dtSeconds,
        const TSnapshot& snapshot,
        const MazeMap::Maze* map,
        ControlCycleTiming* timing,
        LoopHook&& loopHook,
        BeforeYawUpdate&& beforeYawUpdate)
    {
        if (_estimatorFaulted)
        {
            SyncPoseEstimate();
            return;
        }

        const MazeMap::PlantParams& params = _ukf.ukf().params();
        (void)map;
        MazeMap::ControlInput control{};
        control.leftMotorCommand = _leftMotor.getDriveCommand();
        control.rightMotorCommand = _rightMotor.getDriveCommand();
        control.fanDutyCycle = GetMissionFanDutyCycle();
        control.batteryVoltageV = 0.5f * (_leftMotor.getVoltage() + _rightMotor.getVoltage());
        _lastGyroRawRadps = snapshot.gyroRawRadps;
        _ukf.ukf().setRuntimeContext(
            _lastLinearCommandMps,
            _lastAngularCommandRadps,
            _lastSaturationFlags,
            _lastLeftLaunchAssistFloor,
            _lastRightLaunchAssistFloor,
            snapshot.accelBiasValid,
            snapshot.accelBodyXMps2,
            snapshot.accelBodyYMps2);
        const EncoderCycleSample encoderSample = ConsumeEncoderCycleSample(dtSeconds, params);

        if (timing != nullptr)
        {
            timing->ukfPredictStartUs = micros();
        }
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            if (!_ukf.predict(dtSeconds, control, loopHook))
            {
                TriggerEstimatorFault("predict_failed");
                if (timing != nullptr)
                {
                    timing->ukfPredictEndUs = micros();
                    timing->ukfPredictDurationUs = timing->ukfPredictEndUs - timing->ukfPredictStartUs;
                    timing->ukfUpdateStartUs = timing->ukfPredictEndUs;
                    timing->ukfUpdateEndUs = timing->ukfPredictEndUs;
                    timing->ukfUpdateDurationUs = 0U;
                }
                return;
            }
        }
        if (timing != nullptr)
        {
            timing->ukfPredictEndUs = micros();
            timing->ukfPredictDurationUs = timing->ukfPredictEndUs - timing->ukfPredictStartUs;
            timing->ukfUpdateStartUs = micros();
        }

        MazeMap::EncoderObs encoderObservation{};
        encoderObservation.totalLeftCounts = encoderSample.leftCounts;
        encoderObservation.totalRightCounts = encoderSample.rightCounts;
        encoderObservation.omegaLeftRadps = encoderSample.leftOmegaRadps;
        encoderObservation.omegaRightRadps = encoderSample.rightOmegaRadps;
        _lastEncoderObservation = encoderObservation;
        _encoderObservationValid = true;
        (void)_ukf.updateEncoderPair(encoderObservation, dtSeconds, loopHook);

        beforeYawUpdate();

        if (std::isfinite(snapshot.gyroRawRadps))
        {
            const MazeMap::MeasurementUpdateResult yawUpdate = _ukf.updateYawRate(snapshot.gyroRawRadps, loopHook);
            if (!yawUpdate.accepted)
            {
                TriggerEstimatorFault("yaw_update_failed");
                if (timing != nullptr)
                {
                    timing->ukfUpdateEndUs = micros();
                    timing->ukfUpdateDurationUs = timing->ukfUpdateEndUs - timing->ukfUpdateStartUs;
                }
                return;
            }
        }

        if (snapshot.accelBiasValid)
        {
            MazeMap::ImuAccelObs accelObservation{};
            accelObservation.valid =
                std::isfinite(snapshot.accelBodyXMps2) &&
                std::isfinite(snapshot.accelBodyYMps2);
            accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
            accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
            (void)_ukf.updatePlanarAccel(accelObservation, loopHook);
        }

        if (map != nullptr)
        {
            const MazeMap::LocalMapView mapView = BuildUkfMapView(map);
            MazeMap::WallObs frontLeftObs{};
            MazeMap::WallObs frontRightObs{};
            BuildUkfFrontPairObservations(snapshot, params.noHitRangeM, frontLeftObs, frontRightObs);
            if (frontLeftObs.valid && frontRightObs.valid)
            {
                (void)_ukf.updateFrontPair(frontLeftObs, frontRightObs, mapView);
            }

            const MazeMap::WallObs leftSideObs = BuildUkfLeftSideObservation(snapshot, params.noHitRangeM);
            if (leftSideObs.valid)
            {
                (void)_ukf.updateSideSensor(MazeMap::Side::Left, leftSideObs, mapView);
            }

            const MazeMap::WallObs rightSideObs = BuildUkfRightSideObservation(snapshot, params.noHitRangeM);
            if (rightSideObs.valid)
            {
                (void)_ukf.updateSideSensor(MazeMap::Side::Right, rightSideObs, mapView);
            }
        }
        if (timing != nullptr)
        {
            timing->ukfUpdateEndUs = micros();
            timing->ukfUpdateDurationUs = timing->ukfUpdateEndUs - timing->ukfUpdateStartUs;
        }

        SyncPoseEstimate();
    }

    void SetOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        _leftMotor.setDriveCommand((std::clamp)(leftDriveCommand, -1.0f, 1.0f));
        _rightMotor.setDriveCommand((std::clamp)(rightDriveCommand, -1.0f, 1.0f));
    }
    MazeMap::MotorEncoderDrive _leftMotor;
    MazeMap::MotorEncoderDrive _rightMotor;
    MazeMap::MouseUkfFacade _ukf;
    PoseEstimate _poseCache;
    float _leftIntegral;
    float _rightIntegral;
    float _lastLinearCommandMps;
    float _lastAngularCommandRadps;
    float _lastLeftFeedforwardCommand = 0.0f;
    float _lastRightFeedforwardCommand = 0.0f;
    float _lastLeftFeedbackCommand = 0.0f;
    float _lastRightFeedbackCommand = 0.0f;
    float _lastLeftTargetVelocityMps = 0.0f;
    float _lastRightTargetVelocityMps = 0.0f;
    float _lastLeftLaunchAssistFloor = 0.0f;
    float _lastRightLaunchAssistFloor = 0.0f;
    int32_t _leftEncoderCountTotal = 0;
    int32_t _rightEncoderCountTotal = 0;
    float _leftEncoderDistanceMeters = 0.0f;
    float _rightEncoderDistanceMeters = 0.0f;
    float _leftEncoderVelocityMps = 0.0f;
    float _rightEncoderVelocityMps = 0.0f;
    float _lastGyroRawRadps = 0.0f;
    MazeMap::EncoderObs _lastEncoderObservation{};
    uint16_t _lastModeFlags = kModeBraking;
    uint16_t _lastSaturationFlags = 0u;
    bool _encoderObservationValid = false;
    MazeMap::WheelControlProfile _wheelControlProfile;
    struct ClosedLoopVelocityCommand
    {
        float leftTargetMps = 0.0f;
        float rightTargetMps = 0.0f;
        float leftTargetAccelMps2 = 0.0f;
        float rightTargetAccelMps2 = 0.0f;
        float leftFeedforwardCommand = 0.0f;
        float rightFeedforwardCommand = 0.0f;
    };
    struct ResolvedVelocityDriveSignal
    {
        MazeMap::OpenLoopDriveCommand driveCommand{};
        float leftTargetMps = 0.0f;
        float rightTargetMps = 0.0f;
        float leftFeedforwardCommand = 0.0f;
        float rightFeedforwardCommand = 0.0f;
        float leftLaunchAssistFloor = 0.0f;
        float rightLaunchAssistFloor = 0.0f;
    };
    struct WheelLaunchAssistState
    {
        bool active = false;
        unsigned long startMs = 0UL;
        float requestedDirection = 0.0f;
    };
    WheelLaunchAssistState _leftLaunchAssist;
    WheelLaunchAssistState _rightLaunchAssist;
    bool _estimatorFaulted = false;
    char _estimatorFaultReason[64] = {};

    void GetVelocityCommandOperatingState(
        MazeMap::VehicleState::StateVector& presentState,
        float& batteryVoltageV) const
    {
        presentState = _ukf.ukf().state();
        const MeasuredKinematics measured = GetMeasuredKinematics();
        const MazeMap::PlantParams& params = _ukf.ukf().params();
        const float wheelRadiusM =
            (std::isfinite(params.wheelRadiusM) && (params.wheelRadiusM > 0.0f)) ?
            params.wheelRadiusM :
            0.0f;
        if (!std::isfinite(presentState(MazeMap::VehicleState::kPx)))
        {
            presentState(MazeMap::VehicleState::kPx) = 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kPy)))
        {
            presentState(MazeMap::VehicleState::kPy) = 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kPsi)))
        {
            presentState(MazeMap::VehicleState::kPsi) = 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kBgz)))
        {
            presentState(MazeMap::VehicleState::kBgz) = 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kU)))
        {
            presentState(MazeMap::VehicleState::kU) = measured.linearSpeedMps;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kV)))
        {
            presentState(MazeMap::VehicleState::kV) = 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kR)))
        {
            presentState(MazeMap::VehicleState::kR) = measured.angularSpeedRadps;
        }
        const float feedforwardYawRateRadps = _ukf.ukf().resolveYawRateForFeedforward(_lastGyroRawRadps);
        if (std::isfinite(feedforwardYawRateRadps))
        {
            presentState(MazeMap::VehicleState::kR) = feedforwardYawRateRadps;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kOmegaL)))
        {
            presentState(MazeMap::VehicleState::kOmegaL) =
                (wheelRadiusM > 0.0f) ? (measured.leftVelocityMps / wheelRadiusM) : 0.0f;
        }
        if (!std::isfinite(presentState(MazeMap::VehicleState::kOmegaR)))
        {
            presentState(MazeMap::VehicleState::kOmegaR) =
                (wheelRadiusM > 0.0f) ? (measured.rightVelocityMps / wheelRadiusM) : 0.0f;
        }
        MazeMap::VehicleState::NormalizeStateVector(presentState);
        batteryVoltageV = 0.5f * (_leftMotor.getVoltage() + _rightMotor.getVoltage());
    }

    void GetVelocityCommandOperatingPoint(
        float& presentLinearSpeedMps,
        float& presentYawRateRadps,
        float& batteryVoltageV) const
    {
        MazeMap::VehicleState::StateVector presentState = MazeMap::VehicleState::StateVector::Zero();
        GetVelocityCommandOperatingState(presentState, batteryVoltageV);
        presentLinearSpeedMps =
            presentState(MazeMap::VehicleState::kU);
        presentYawRateRadps =
            presentState(MazeMap::VehicleState::kR);
    }

    ClosedLoopVelocityCommand BuildClosedLoopVelocityCommandFromSolution(
        float linearSpeedMps,
        float angularSpeedRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        const MazeMap::DriveCommandSolution& solution) const
    {
        ClosedLoopVelocityCommand command{};
        const float effectiveTrackWidthM = MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(linearSpeedMps, angularSpeedRadps);
        command.leftTargetMps = linearSpeedMps + (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        command.rightTargetMps = linearSpeedMps - (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        command.leftTargetAccelMps2 =
            desiredLongitudinalAccelMps2 + (0.5f * effectiveTrackWidthM * desiredYawAccelRadps2);
        command.rightTargetAccelMps2 =
            desiredLongitudinalAccelMps2 - (0.5f * effectiveTrackWidthM * desiredYawAccelRadps2);
        command.leftFeedforwardCommand = solution.control.leftMotorCommand;
        command.rightFeedforwardCommand = solution.control.rightMotorCommand;
        return command;
    }

    float ResolveStraightHeadingYawRateCommand(
        float targetYawRad,
        float measuredYawRad,
        float estimatedYawRateRadps) const noexcept
    {
        if (!(std::isfinite(targetYawRad) && std::isfinite(measuredYawRad)))
        {
            return 0.0f;
        }

        const float resolvedEstimatedYawRateRadps =
            std::isfinite(estimatedYawRateRadps) ?
            estimatedYawRateRadps :
            0.0f;
        const float headingErrorRad =
            HeadingErrorRad(
                HeadingUnitFromYawRad(targetYawRad),
                HeadingUnitFromYawRad(measuredYawRad));
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * resolvedEstimatedYawRateRadps);
        if (!std::isfinite(angularCommandRadps))
        {
            return 0.0f;
        }
        return angularCommandRadps;
    }

    void ResolveDefaultVelocityTargetOperatingEnvelope(
        const MazeMap::VehicleState::StateVector& presentState,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const
    {
        maxLongitudinalAccelMps2 = kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2;
        maxYawAccelRadps2 = kDefaultCommandVelocityAsapYawAccelLimitRadps2;

        MazeMap::PlantModel plantModel;
        float technicalLongitudinalAccelMps2 = 0.0f;
        float technicalYawAccelRadps2 = 0.0f;
        plantModel.velocityTargetTechnicalLimits(
            presentState,
            _ukf.ukf().preparedParams(),
            technicalLongitudinalAccelMps2,
            technicalYawAccelRadps2,
            GetMissionFanDutyCycle());

        if (std::isfinite(technicalLongitudinalAccelMps2) && (technicalLongitudinalAccelMps2 > 0.0f))
        {
            maxLongitudinalAccelMps2 =
                (std::min)(maxLongitudinalAccelMps2, technicalLongitudinalAccelMps2);
        }
        if (std::isfinite(technicalYawAccelRadps2) && (technicalYawAccelRadps2 > 0.0f))
        {
            maxYawAccelRadps2 =
                (std::min)(maxYawAccelRadps2, technicalYawAccelRadps2);
        }
    }

    ClosedLoopVelocityCommand BuildClosedLoopVelocityCommandForVelocityTarget(
        float linearSpeedMps,
        float angularSpeedRadps,
        const MazeMap::VehicleState::StateVector& presentState,
        float maxLongitudinalAccelMps2,
        float maxYawAccelRadps2) const
    {
        float batteryVoltageV = 0.0f;
        batteryVoltageV = 0.5f * (_leftMotor.getVoltage() + _rightMotor.getVoltage());

        const float presentLinearSpeedMps = presentState(MazeMap::VehicleState::kU);
        const float presentYawRateRadps = presentState(MazeMap::VehicleState::kR);
        float desiredLongitudinalAccelMps2 = 0.0f;
        float desiredYawAccelRadps2 = 0.0f;
        MazeMap::Internal::ResolveVelocityTargetAsapAccelerations(
            presentLinearSpeedMps,
            linearSpeedMps,
            presentYawRateRadps,
            angularSpeedRadps,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2,
            MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);

        MazeMap::PlantModel plantModel;
        const MazeMap::DriveCommandSolution solution =
            plantModel.solveClosedLoopDriveCommands(
                presentState,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                _ukf.ukf().preparedParams(),
                GetMissionFanDutyCycle(),
                batteryVoltageV);
        return BuildClosedLoopVelocityCommandFromSolution(
            linearSpeedMps,
            angularSpeedRadps,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            solution);
    }

    ClosedLoopVelocityCommand BuildClosedLoopVelocityCommandForVelocityTarget(
        float linearSpeedMps,
        float angularSpeedRadps,
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float maxLongitudinalAccelMps2,
        float maxYawAccelRadps2) const
    {
        MazeMap::VehicleState::StateVector presentState = MazeMap::VehicleState::StateVector::Zero();
        presentState(MazeMap::VehicleState::kU) = presentLinearSpeedMps;
        presentState(MazeMap::VehicleState::kR) = presentYawRateRadps;
        MazeMap::VehicleState::NormalizeStateVector(presentState);
        return BuildClosedLoopVelocityCommandForVelocityTarget(
            linearSpeedMps,
            angularSpeedRadps,
            presentState,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
    }

    ClosedLoopVelocityCommand BuildClosedLoopVelocityCommand(
        float linearSpeedMps,
        float angularSpeedRadps,
        float desiredAccelerationMps2) const
    {
        MazeMap::VehicleState::StateVector presentState = MazeMap::VehicleState::StateVector::Zero();
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingState(presentState, batteryVoltageV);

        const MeasuredKinematics measured = GetMeasuredKinematics();
        const float presentLinearSpeedMps = presentState(MazeMap::VehicleState::kU);
        const float effectiveTrackWidthM = MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(linearSpeedMps, angularSpeedRadps);
        const float targetLeftMps = linearSpeedMps + (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        const float targetRightMps = linearSpeedMps - (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        const float accelerationLimitMps2 =
            (std::isfinite(desiredAccelerationMps2) && (desiredAccelerationMps2 > 0.0f)) ?
            desiredAccelerationMps2 :
            0.0f;
        float leftTargetAccelMps2 = 0.0f;
        float rightTargetAccelMps2 = 0.0f;
        if ((effectiveTrackWidthM > 0.0f) && (accelerationLimitMps2 > 0.0f))
        {
            leftTargetAccelMps2 =
                (std::clamp)(
                    (targetLeftMps - measured.leftVelocityMps) / MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS,
                    -accelerationLimitMps2,
                    accelerationLimitMps2);
            rightTargetAccelMps2 =
                (std::clamp)(
                    (targetRightMps - measured.rightVelocityMps) / MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS,
                    -accelerationLimitMps2,
                    accelerationLimitMps2);
        }

        const float desiredLongitudinalAccelMps2 = 0.5f * (leftTargetAccelMps2 + rightTargetAccelMps2);
        const float desiredYawAccelRadps2 =
            (effectiveTrackWidthM > 0.0f) ?
            ((leftTargetAccelMps2 - rightTargetAccelMps2) / effectiveTrackWidthM) :
            0.0f;

        MazeMap::PlantModel plantModel;
        const MazeMap::DriveCommandSolution solution =
            plantModel.solveClosedLoopDriveCommands(
                presentState,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                _ukf.ukf().preparedParams(),
                GetMissionFanDutyCycle(),
                batteryVoltageV);
        return BuildClosedLoopVelocityCommandFromSolution(
            linearSpeedMps,
            angularSpeedRadps,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            solution);
    }

    ResolvedVelocityDriveSignal ResolveClosedLoopVelocityDriveSignal(
        const ClosedLoopVelocityCommand& command,
        float dtSeconds)
    {
        ResolvedVelocityDriveSignal resolved{};
        resolved.leftTargetMps = command.leftTargetMps;
        resolved.rightTargetMps = command.rightTargetMps;
        resolved.leftFeedforwardCommand = command.leftFeedforwardCommand;
        resolved.rightFeedforwardCommand = command.rightFeedforwardCommand;

        const float leftMeasuredMps = _leftEncoderVelocityMps;
        const float rightMeasuredMps = _rightEncoderVelocityMps;
        const unsigned long nowMs = millis();
        const bool leftLaunchAssistActive = UpdateWheelLaunchAssistState(
            _leftLaunchAssist,
            leftMeasuredMps,
            resolved.leftTargetMps,
            _leftMotor.getDriveCommand(),
            nowMs);
        const bool rightLaunchAssistActive = UpdateWheelLaunchAssistState(
            _rightLaunchAssist,
            rightMeasuredMps,
            resolved.rightTargetMps,
            _rightMotor.getDriveCommand(),
            nowMs);

        const float leftErrorMps = resolved.leftTargetMps - leftMeasuredMps;
        const float rightErrorMps = resolved.rightTargetMps - rightMeasuredMps;
        const float integralLimit = GetWheelIntegralLimit();
        float leftCommandUnclamped = ComputeVelocityCommandFromErrorUnclamped(
            resolved.leftFeedforwardCommand,
            command.leftTargetMps,
            command.leftTargetAccelMps2,
            leftErrorMps,
            _leftIntegral);
        float rightCommandUnclamped = ComputeVelocityCommandFromErrorUnclamped(
            resolved.rightFeedforwardCommand,
            command.rightTargetMps,
            command.rightTargetAccelMps2,
            rightErrorMps,
            _rightIntegral);
        float leftCommand = MazeMap::ClampWheelDriveCommand(leftCommandUnclamped);
        float rightCommand = MazeMap::ClampWheelDriveCommand(rightCommandUnclamped);

        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            if (MazeMap::ShouldAccumulateWheelVelocityIntegral(leftCommandUnclamped, leftCommand, leftErrorMps))
            {
                _leftIntegral = (std::clamp)(_leftIntegral + (leftErrorMps * dtSeconds), -integralLimit, integralLimit);
                leftCommandUnclamped = ComputeVelocityCommandFromErrorUnclamped(
                    resolved.leftFeedforwardCommand,
                    command.leftTargetMps,
                    command.leftTargetAccelMps2,
                    leftErrorMps,
                    _leftIntegral);
                leftCommand = MazeMap::ClampWheelDriveCommand(leftCommandUnclamped);
            }

            if (MazeMap::ShouldAccumulateWheelVelocityIntegral(rightCommandUnclamped, rightCommand, rightErrorMps))
            {
                _rightIntegral = (std::clamp)(_rightIntegral + (rightErrorMps * dtSeconds), -integralLimit, integralLimit);
                rightCommandUnclamped = ComputeVelocityCommandFromErrorUnclamped(
                    resolved.rightFeedforwardCommand,
                    command.rightTargetMps,
                    command.rightTargetAccelMps2,
                    rightErrorMps,
                    _rightIntegral);
                rightCommand = MazeMap::ClampWheelDriveCommand(rightCommandUnclamped);
            }
        }
        else
        {
            _leftIntegral = (std::clamp)(_leftIntegral, -integralLimit, integralLimit);
            _rightIntegral = (std::clamp)(_rightIntegral, -integralLimit, integralLimit);
        }

        if (leftLaunchAssistActive)
        {
            resolved.leftLaunchAssistFloor = GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs);
            leftCommand = ApplyLaunchAssistFloor(
                leftCommand,
                resolved.leftTargetMps,
                resolved.leftLaunchAssistFloor);
        }
        if (rightLaunchAssistActive)
        {
            resolved.rightLaunchAssistFloor = GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs);
            rightCommand = ApplyLaunchAssistFloor(
                rightCommand,
                resolved.rightTargetMps,
                resolved.rightLaunchAssistFloor);
        }

        resolved.driveCommand = MazeMap::MakeOpenLoopDriveCommand(leftCommand, rightCommand);
        return resolved;
    }

    float ComputeVelocityCommandFromErrorUnclamped(
        float feedforwardCommand,
        float targetSpeedMps,
        float targetAccelMps2,
        float errorMps,
        float integral) const
    {
        float command = feedforwardCommand;
        if (std::fabs(targetSpeedMps) > 0.01f)
        {
            command += SignF(targetSpeedMps) * Config::kWheelStaticFeedforward;
        }
        command += Config::kWheelVelocityFeedforward * targetSpeedMps;
        command += ComputeWheelAccelerationResponseCommand(targetAccelMps2, errorMps);
        command += GetWheelVelocityKp() * errorMps;
        command += GetWheelVelocityKi() * integral;
        return command;
    }

    float ComputeWheelAccelerationResponseCommand(float targetAccelMps2, float errorMps) const
    {
        const float accelerationResponseScale = GetWheelAccelerationResponseScale();
        if (!(accelerationResponseScale > 0.0f) ||
            !std::isfinite(targetAccelMps2) ||
            !std::isfinite(errorMps) ||
            ((targetAccelMps2 * errorMps) <= 0.0f))
        {
            return 0.0f;
        }

        const float deltaWindowMps = Config::kWheelAccelerationResponseDeltaWindowMps;
        if (!(deltaWindowMps > 0.0f) || !std::isfinite(deltaWindowMps))
        {
            return 0.0f;
        }

        const float closenessScale = (std::clamp)(std::fabs(errorMps) / deltaWindowMps, 0.0f, 1.0f);
        return Config::kWheelAccelerationResponseGainPerMps2 * accelerationResponseScale * targetAccelMps2 * closenessScale;
    }

    float GetWheelVelocityKp() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, _wheelControlProfile.velocityKpScale);
    }

    float GetWheelVelocityKi() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKi, _wheelControlProfile.velocityKiScale);
    }

    float GetWheelIntegralLimit() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, _wheelControlProfile.integralLimitScale);
    }

    float GetWheelAccelerationResponseScale() const
    {
        return _wheelControlProfile.accelerationResponseScale;
    }

    void ResetLaunchAssist()
    {
        _leftLaunchAssist = WheelLaunchAssistState{};
        _rightLaunchAssist = WheelLaunchAssistState{};
    }

    static void ResetWheelLaunchAssistState(WheelLaunchAssistState& state, unsigned long nowMs)
    {
        state.active = false;
        state.startMs = nowMs;
        state.requestedDirection = 0.0f;
    }

    static bool UpdateWheelLaunchAssistState(
        WheelLaunchAssistState& state,
        float measuredMps,
        float requestedDirection,
        float priorDriveCommand,
        unsigned long nowMs)
    {
        if ((std::fabs(requestedDirection) <= 0.01f) ||
            (std::fabs(measuredMps) > Config::kWheelRestLaunchSpeedThresholdMps))
        {
            ResetWheelLaunchAssistState(state, nowMs);
            return false;
        }

        if (!state.active)
        {
            if (std::fabs(priorDriveCommand) > Config::kWheelRestLaunchDriveThreshold)
            {
                return false;
            }

            state.active = true;
            state.startMs = nowMs;
            state.requestedDirection = SignF(requestedDirection);
            return true;
        }

        const float requestedSign = SignF(requestedDirection);
        if ((requestedSign != 0.0f) &&
            (state.requestedDirection != 0.0f) &&
            (requestedSign != state.requestedDirection))
        {
            state.startMs = nowMs;
            state.requestedDirection = requestedSign;
        }
        else if ((state.requestedDirection == 0.0f) && (requestedSign != 0.0f))
        {
            state.startMs = nowMs;
            state.requestedDirection = requestedSign;
        }

        return true;
    }

    static float GetWheelLaunchAssistFloor(const WheelLaunchAssistState& state, unsigned long nowMs)
    {
        if (!state.active)
        {
            return 0.0f;
        }

        return MazeMap::ComputeLaunchAssistDriveFloor(
            Config::kWheelRestLaunchDriveCommand,
            Config::kWheelRestLaunchMaxDriveCommand,
            nowMs - state.startMs,
            Config::kWheelRestLaunchRampMs);
    }

    static float ApplyLaunchAssistFloor(float command, float requestedDirection, float launchFloor)
    {
        if ((std::fabs(requestedDirection) <= 0.01f) || !(launchFloor > 0.0f))
        {
            return command;
        }

        const float magnitude = std::fabs(command);
        if (magnitude >= launchFloor)
        {
            return command;
        }

        return SignF(requestedDirection) * launchFloor;
    }

};



