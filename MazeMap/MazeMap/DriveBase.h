#pragma once
// Defines the drive-local command solver that proposes wheel commands and maintains controller
// state for closed-loop command generation. Hardware application and sensor capture belong to the
// runtime vehicle/sensor owners.
#include "CommandVector.h"
#include "CommandPD.h"
#include "DriveTelemetry.h"
#include "EncoderObs.h"
#include "LaunchAssistProfile.h"
#include "Maneuver.h"
#include "MazeMapRuntimeCore.h"
#include "PlantModel.h"
#include "ProportionalDerivativeCluster.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

// Serves as the concrete drive command-proposal subsystem for the MazeMap application. It should not become
// the owner of hardware application, sensor capture, higher-level maneuver scheduling, or shared
// multi-tick motion-routine orchestration.
class DriveBase
{
private:
    struct CommandContext;
    struct CommandTargets;
    static constexpr float kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2 = 9.0f;
    static constexpr float kDefaultCommandVelocityAsapYawAccelLimitRadps2 = 400.0f;
public:
    static constexpr uint16_t kModeClosedLoop = 1u << 0;
    static constexpr uint16_t kModeBraking = 1u << 3;
    static constexpr uint16_t kModeLaunchAssistLeft = 1u << 4;
    static constexpr uint16_t kModeLaunchAssistRight = 1u << 5;

    explicit DriveBase(
        const MazeMap::PlantModel& plantModel,
        const MazeMap::VehicleState& runtimeState,
        const MazeMap::ProportionalDerivativeCluster& proportionalDerivativeCluster)
        : _runtimeState(runtimeState)
        , _plantModel(plantModel)
        , _proportionalDerivativeCluster(&proportionalDerivativeCluster)
        , _lastLinearCommandMps(0.0f)
        , _lastAngularCommandRadps(0.0f)
    {
    }

    void SetProportionalDerivativeCluster(const MazeMap::ProportionalDerivativeCluster& proportionalDerivativeCluster) noexcept
    {
        _proportionalDerivativeCluster = &proportionalDerivativeCluster;
    }

    const MazeMap::ProportionalDerivativeCluster& GetProportionalDerivativeCluster() noexcept { return *_proportionalDerivativeCluster; }
    const MazeMap::ProportionalDerivativeCluster& GetProportionalDerivativeCluster() const noexcept { return *_proportionalDerivativeCluster; }

    bool Begin()
    {
        ResetEncoderTracking();
        ResetControllers();
        Brake();
        return true;
    }

    void ResetControllers()
    {
        ResetLaunchAssist();
    }

    MazeMap::App::Internal::CommandVector CurrentControlVector() const noexcept
    {
        return _lastProposedCommand;
    }

    float CurrentBatteryVoltageV() const noexcept
    {
        return 0.0f;
    }

    // DeltaCommand resolves a feedforward command from an explicit operating point and an explicit
    // longitudinal acceleration request. The result is symmetric by construction.
    // Supported `pd` flags here: `StateAccelerationPD`, `IMUForwardAccel`.
    // Any heading, yaw-rate, wheel-speed, encoder, or lateral-accel selection has no explicit target
    // on this overload and therefore defaults to hold/no-op behavior.
    EXPORT MazeMap::App::Internal::CommandVector DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // DeltaCommand resolves the fully coupled plant feedforward from an explicit body-speed operating
    // point and explicit longitudinal/yaw acceleration requests. This is the canonical "delta from the
    // current state" entry point when both translation and rotation matter at once.
    // Supported `pd` flags here: `StateAccelerationPD`, `IMUForwardAccel`.
    // This overload does not expose a separate yaw-acceleration feedback-source selector, so heading,
    // yaw-rate, wheel-speed, encoder, and lateral-accel flags remain hold/no-op selections.
    EXPORT MazeMap::App::Internal::CommandVector DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // DeltaYawRateCommand resolves a zero-mean command from the present yaw rate and a desired yaw
    // acceleration. This is the single-axis rotational variant of `DeltaCommand`.
    // No additional `pd` feedback target is currently exposed on this overload; every flag presently
    // behaves the same as `RawCommand`.
    EXPORT MazeMap::App::Internal::CommandVector DeltaYawRateCommand(
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommand resolves a symmetric command that drives the present forward speed toward the requested
    // forward speed over the canonical roll-off horizon while respecting the plant-reported acceleration
    // envelope. Wheel-speed or other optional loops use `desiredLinearSpeedMps` as their setpoint when
    // that association is meaningful; all unrelated loops hold their present values.
    // Supported `pd` flags here: `StateVelocityPD`, `EncoderVelocity`.
    // This overload does not set a new heading, yaw-rate, longitudinal-acceleration, or
    // lateral-acceleration target.
    EXPORT MazeMap::App::Internal::CommandVector PointCommand(
        float desiredLinearSpeedMps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommand resolves the fully coupled command that drives the present forward speed and yaw rate
    // toward the requested targets over the canonical roll-off horizon while respecting the plant envelope.
    // This replaces the old ambiguous "velocity command" entry point.
    // Supported `pd` flags here: `StateVelocityPD`, `StateYawPD`, `EncoderVelocity`, `IMUYaw`.
    // This overload does not set a heading, longitudinal-acceleration, or lateral-acceleration target.
    EXPORT MazeMap::App::Internal::CommandVector PointCommand(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointControlVector resolves the same coupled target as `PointCommand`. It remains as the
    // stable entry point for loop-controller call sites that already operate in control-vector space.
    EXPORT MazeMap::App::Internal::CommandVector PointControlVector(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommandWithHeadingTarget resolves the same coupled velocity/yaw-rate target as
    // `PointCommand`, but it also lets the caller add DriveBase-owned heading correction in the
    // same composed command so feedforward/feedback decomposition remains authoritative here.
    EXPORT MazeMap::App::Internal::CommandVector PointCommandWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::CommandPD pointPd,
        MazeMap::CommandPD headingPd) const;

    // PointControlVectorWithHeadingTarget is the stable control-vector-space wrapper for
    // `PointCommandWithHeadingTarget`.
    EXPORT MazeMap::App::Internal::CommandVector PointControlVectorWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::CommandPD pointPd,
        MazeMap::CommandPD headingPd) const;

    // PointCommand consumes the drive-relevant target fields from a maneuver point. Higher-level
    // maneuver execution should target this overload instead of rebuilding scalar command bridges.
    // It exposes the same `pd` selections as the scalar `(desiredLinearSpeedMps, desiredYawRateRadps)`
    // overload because it forwards directly to that entry point.
    EXPORT MazeMap::App::Internal::CommandVector PointCommand(
        const MazeMap::ManeuverPoint& point,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointControlVector resolves the same maneuver-point target as `PointCommand`. It remains as the
    // stable entry point for loop-controller call sites that already operate in control-vector space.
    EXPORT MazeMap::App::Internal::CommandVector PointControlVector(
        const MazeMap::ManeuverPoint& point,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointYawRateCommand resolves a zero-mean command that drives the present yaw rate toward the
    // requested yaw-rate target over the canonical roll-off horizon while respecting the yaw-acceleration
    // limit reported by the plant.
    // Supported `pd` flags here: `StateYawPD`, `EncoderVelocity`, `IMUYaw`.
    // This overload does not set a new linear-speed, heading, longitudinal-acceleration, or
    // lateral-acceleration target.
    EXPORT MazeMap::App::Internal::CommandVector PointYawRateCommand(
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // FeedbackCommand produces a pure feedback command cluster. Each selected loop uses `setpoint` as
    // its target. Unselected loops contribute nothing. The returned command starts from the plant command
    // that preserves the present motion state, then layers the requested feedback objectives on top.
    // Supported `pd` flags here: `StateHeadingPD`, `StateYawPD`, `StateVelocityPD`,
    // `StateAccelerationPD`, `EncoderVelocity`, `IMUYaw`,
    // `IMUForwardAccel`, `IMULateralAccel`.
    // `IMULateralAccel` still requires a nonzero present or target speed so the requested lateral
    // acceleration can be converted into a yaw-rate correction.
    EXPORT MazeMap::App::Internal::CommandVector FeedbackCommand(
        float setpoint,
        MazeMap::CommandPD pd) const;

    void Brake()
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _lastFeedforwardCommand = {};
        _lastFeedbackCommand = {};
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastLeftLaunchAssistFloor = 0.0f;
        _lastRightLaunchAssistFloor = 0.0f;
        _lastModeFlags = kModeBraking;
        _lastSaturationFlags = 0u;
        _lastProposedCommand = {};
    }

    float GetAverageDistanceMeters() const
    {
        RefreshSensorSnapshotDerivedState();
        return 0.5f * (_leftEncoderDistanceMeters + _rightEncoderDistanceMeters);
    }

    float GetLastLinearCommandMps() const
    {
        return _lastLinearCommandMps;
    }

    float GetLastAngularCommandRadps() const
    {
        return _lastAngularCommandRadps;
    }

    uint16_t GetLastSaturationFlags() const noexcept
    {
        return _lastSaturationFlags;
    }

    float GetLastLeftLaunchAssistFloor() const noexcept
    {
        return _lastLeftLaunchAssistFloor;
    }

    float GetLastRightLaunchAssistFloor() const noexcept
    {
        return _lastRightLaunchAssistFloor;
    }

    // The generated-command decomposition is cached at the point where DriveBase still owns both
    // the plant-model feedforward command and the PD-only correction.
    MazeMap::App::Internal::CommandVector GetLastFeedforward() const noexcept
    {
        return _lastFeedforwardCommand;
    }

    MazeMap::App::Internal::CommandVector GetLastFeedback() const noexcept
    {
        return _lastFeedbackCommand;
    }

    DriveTelemetry GetGeneratedTelemetry(
        const MazeMap::App::Internal::CommandVector& command) const noexcept
    {
        DriveTelemetry telemetry{};
        telemetry.leftDriveCommand = command.LeftMotorPwm();
        telemetry.rightDriveCommand = command.RightMotorPwm();
        telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
        telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
        telemetry.leftFeedforwardCommand = _lastFeedforwardCommand.LeftMotorPwm();
        telemetry.rightFeedforwardCommand = _lastFeedforwardCommand.RightMotorPwm();
        telemetry.leftFeedbackCommand = _lastFeedbackCommand.LeftMotorPwm();
        telemetry.rightFeedbackCommand = _lastFeedbackCommand.RightMotorPwm();
        telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
        telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
        telemetry.leftLaunchAssistFloor = _lastLeftLaunchAssistFloor;
        telemetry.rightLaunchAssistFloor = _lastRightLaunchAssistFloor;
        telemetry.modeFlags = _lastModeFlags;
        telemetry.saturationFlags = _lastSaturationFlags;
        return telemetry;
    }

    DriveTelemetry GetTelemetry() const
    {
        RefreshSensorSnapshotDerivedState();
        DriveTelemetry telemetry{};
        const MazeMap::App::Internal::CommandVector appliedControl = CurrentControlVector();
        telemetry.leftDriveCommand = appliedControl.LeftMotorPwm();
        telemetry.rightDriveCommand = appliedControl.RightMotorPwm();
        telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
        telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
        telemetry.leftFeedforwardCommand = _lastFeedforwardCommand.LeftMotorPwm();
        telemetry.rightFeedforwardCommand = _lastFeedforwardCommand.RightMotorPwm();
        telemetry.leftFeedbackCommand = _lastFeedbackCommand.LeftMotorPwm();
        telemetry.rightFeedbackCommand = _lastFeedbackCommand.RightMotorPwm();
        telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
        telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
        telemetry.leftLaunchAssistFloor = _lastLeftLaunchAssistFloor;
        telemetry.rightLaunchAssistFloor = _lastRightLaunchAssistFloor;
        telemetry.leftEncoderCount = _leftEncoderCountTotal;
        telemetry.rightEncoderCount = _rightEncoderCountTotal;
        telemetry.leftDistanceM = _leftEncoderDistanceMeters;
        telemetry.rightDistanceM = _rightEncoderDistanceMeters;
        telemetry.leftVelocityMps = _leftEncoderVelocityMps;
        telemetry.rightVelocityMps = _rightEncoderVelocityMps;
        telemetry.leftEncoderOmegaRadps = _lastEncoderObservation.omegaLeftRadps;
        telemetry.rightEncoderOmegaRadps = _lastEncoderObservation.omegaRightRadps;
        telemetry.modeFlags = _lastModeFlags;
        telemetry.saturationFlags = _lastSaturationFlags;
        telemetry.encoderObservationValid = _encoderObservationValid;
        return telemetry;
    }

private:
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
        _hasProcessedEncoderSnapshot = false;
        _lastEncoderSnapshotTimeS = std::numeric_limits<float>::quiet_NaN();
    }

    EXPORT void RefreshSensorSnapshotDerivedState() const noexcept;

    const MazeMap::VehicleState& _runtimeState;
    const MazeMap::PlantModel& _plantModel;
    const MazeMap::ProportionalDerivativeCluster* _proportionalDerivativeCluster;
    mutable float _lastLinearCommandMps;
    mutable float _lastAngularCommandRadps;
    mutable MazeMap::App::Internal::CommandVector _lastFeedforwardCommand{};
    mutable MazeMap::App::Internal::CommandVector _lastFeedbackCommand{};
    mutable MazeMap::App::Internal::CommandVector _lastProposedCommand{};
    mutable float _lastLeftTargetVelocityMps = 0.0f;
    mutable float _lastRightTargetVelocityMps = 0.0f;
    mutable float _lastLeftLaunchAssistFloor = 0.0f;
    mutable float _lastRightLaunchAssistFloor = 0.0f;
    mutable int32_t _leftEncoderCountTotal = 0;
    mutable int32_t _rightEncoderCountTotal = 0;
    mutable float _leftEncoderDistanceMeters = 0.0f;
    mutable float _rightEncoderDistanceMeters = 0.0f;
    mutable float _leftEncoderVelocityMps = 0.0f;
    mutable float _rightEncoderVelocityMps = 0.0f;
    mutable float _lastGyroRawRadps = 0.0f;
    mutable float _lastImuYawRateRadps = 0.0f;
    mutable float _lastImuAccelBodyXMps2 = 0.0f;
    mutable float _lastImuAccelBodyYMps2 = 0.0f;
    mutable MazeMap::EncoderObs _lastEncoderObservation{};
    mutable uint16_t _lastModeFlags = kModeBraking;
    mutable uint16_t _lastSaturationFlags = 0u;
    mutable float _lastEncoderSnapshotTimeS = std::numeric_limits<float>::quiet_NaN();
    mutable bool _encoderObservationValid = false;
    mutable bool _hasProcessedEncoderSnapshot = false;
    mutable bool _lastImuYawRateValid = false;
    mutable bool _lastImuAccelValid = false;
    struct CommandContext
    {
        MazeMap::PlantDerivatives presentDerivatives{};
        float batteryVoltageV = 0.0f;
        float presentYawRad = 0.0f;
        float presentLinearSpeedMps = 0.0f;
        float presentYawRateRadps = 0.0f;
        float stateLongitudinalAccelMps2 = 0.0f;
        float stateImuForwardAccelMps2 = 0.0f;
        float stateImuLateralAccelMps2 = 0.0f;
        float imuYawRateRadps = 0.0f;
        float imuForwardAccelMps2 = 0.0f;
        float imuLateralAccelMps2 = 0.0f;
        float encoderLeftVelocityMps = 0.0f;
        float encoderRightVelocityMps = 0.0f;
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
    };

    struct CommandTargets
    {
        bool hasHeadingTarget = false;
        float headingTargetYawRad = 0.0f;
        bool hasVelocityTarget = false;
        float velocityTargetMps = 0.0f;
        bool hasYawRateTarget = false;
        float yawRateTargetRadps = 0.0f;
        bool hasLongitudinalAccelTarget = false;
        float longitudinalAccelTargetMps2 = 0.0f;
        bool hasYawAccelTarget = false;
        float yawAccelTargetRadps2 = 0.0f;
        bool hasLateralAccelTarget = false;
        float lateralAccelTargetMps2 = 0.0f;
        bool hasWheelLinearTargets = false;
        float leftWheelLinearTargetMps = 0.0f;
        float rightWheelLinearTargetMps = 0.0f;
    };

    struct WheelLaunchAssistState
    {
        bool active = false;
        unsigned long startMs = 0UL;
        float requestedDirection = 0.0f;
    };
    WheelLaunchAssistState _leftLaunchAssist;
    WheelLaunchAssistState _rightLaunchAssist;

    void GetVelocityCommandOperatingPoint(
        float& presentLinearSpeedMps,
        float& presentYawRateRadps,
        float& batteryVoltageV) const;

    static float ResolveCommandResponseTimeS() noexcept;

    static float ResolvePositiveOrZero(float value) noexcept;

    static float ClampMagnitude(float value, float limit) noexcept;

    CommandTargets BuildHoldTargets(const CommandContext& context) const noexcept;

    CommandContext CaptureCommandContext() const;

    void ResolveWheelTargets(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        CommandTargets& targets) const;

    void ResolveVelocityPointAcceleration(
        const CommandContext& context,
        float desiredLinearSpeedMps,
        float& desiredLongitudinalAccelMps2) const noexcept;

    void ResolveYawPointAcceleration(
        const CommandContext& context,
        float desiredYawRateRadps,
        float& desiredYawAccelRadps2) const noexcept;

    void ResolveVelocityPointAccelerations(
        const CommandContext& context,
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float& desiredLongitudinalAccelMps2,
        float& desiredYawAccelRadps2) const noexcept;

    MazeMap::App::Internal::CommandVector ResolveRawAccelerationCommand(
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2) const;

    MazeMap::App::Internal::CommandVector ResolveRawVelocityTargetCommand(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps) const;

    MazeMap::App::Internal::CommandVector ResolveLongitudinalCorrectionCommand(
        const CommandContext& context,
        float desiredLongitudinalAccelCorrectionMps2) const;

    MazeMap::App::Internal::CommandVector ResolveYawCorrectionCommand(
        const CommandContext& context,
        float desiredYawAccelCorrectionRadps2) const;

    MazeMap::App::Internal::CommandVector ComposeGeneratedCommand(
        const MazeMap::App::Internal::CommandVector& baseCommand,
        const CommandContext& context,
        const CommandTargets& targets,
        MazeMap::CommandPD pd) const;

    void CacheGeneratedCommandTelemetry(
        const MazeMap::App::Internal::CommandVector& feedforwardCommand,
        const MazeMap::App::Internal::CommandVector& feedbackCommand) const noexcept;

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
        const MazeMap::ProportionalDerivative& headingPD =
            GetProportionalDerivativeCluster().GetHeadingPD(MazeMap::CommandPD::StateHeadingPD);
        const float angularCommandRadps =
            headingPD.Compute(headingErrorRad, -resolvedEstimatedYawRateRadps);
        if (!std::isfinite(angularCommandRadps))
        {
            return 0.0f;
        }
        return angularCommandRadps;
    }

    void ResolveDefaultVelocityTargetCommandEnvelope(
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const
    {
        maxLongitudinalAccelMps2 = kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2;
        maxYawAccelRadps2 = kDefaultCommandVelocityAsapYawAccelLimitRadps2;

        float technicalLongitudinalAccelMps2 = 0.0f;
        float technicalYawAccelRadps2 = 0.0f;
        _plantModel.velocityTargetTechnicalLimits(
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




