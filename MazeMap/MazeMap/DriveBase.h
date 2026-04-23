#pragma once
// Defines the concrete runtime motion owner that produces wheel/motor commands, maintains odometry,
// and runs the closed-loop wheel/pose control machinery. This is not the planned higher-level
// "Drive" translation layer; it is the low-level destination for concrete motion commands.
#include "BootUtilityModeFramework.h"
#include "CommandPD.h"
#include "DriveTelemetry.h"
#include "LaunchAssistProfile.h"
#include "LoopController.h"
#include "Maneuver.h"
#include "MazeMapRuntimeCore.h"
#include "MotorEncoderDrive.h"
#include "MouseUkfFacade.h"
#include "PlantModel.h"
#include "ProportionalDerivativeCluster.h"
#include "SensorSnapshot.h"
#include "WheelControlProfile.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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

// Serves as the concrete runtime drive subsystem for the MazeMap application by coordinating motors,
// odometry, estimator-facing pose state, and motion-command production. It should not become the
// owner of higher-level maneuver scheduling or shared multi-tick motion-routine orchestration.
class DriveBase
{
private:
    struct CommandContext;
    struct CommandTargets;
    static constexpr float kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2 = 9.0f;
    static constexpr float kDefaultCommandVelocityAsapYawAccelLimitRadps2 = 400.0f;
    bool startSet = false;
public:
    static constexpr uint16_t kModeClosedLoop = 1u << 0;
    static constexpr uint16_t kModeRawOpenLoop = 1u << 2;
    static constexpr uint16_t kModeBraking = 1u << 3;
    static constexpr uint16_t kModeLaunchAssistLeft = 1u << 4;
    static constexpr uint16_t kModeLaunchAssistRight = 1u << 5;

    explicit DriveBase(
        const MazeMap::PlantModel& plantModel,
        const MazeMap::ProportionalDerivativeCluster& proportionalDerivativeCluster)
        : _leftMotor(MazeMap::MotorEncoderDrive::CreateDefaultLeftDrive())
        , _rightMotor(MazeMap::MotorEncoderDrive::CreateDefaultRightDrive())
        , _ukf(MazeMap::PlantParams::Default())
        , _plantModel(plantModel)
        , _proportionalDerivativeCluster(&proportionalDerivativeCluster)
        , _poseCache{}
        , _leftIntegral(0.0f)
        , _rightIntegral(0.0f)
        , _lastLinearCommandMps(0.0f)
        , _lastAngularCommandRadps(0.0f)
        , _wheelControlProfile(BuildNominalWheelControlProfile())
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
        kinematics.leftVelocityMps = _leftEncoderVelocityMps;
        kinematics.rightVelocityMps = _rightEncoderVelocityMps;
        const float fallbackYawRateRadps =
            [&]() noexcept
            {
                float resolvedLinearSpeedMps = 0.0f;
                float resolvedYawRateRadps = 0.0f;
                _plantModel.resolveBodyVelocityFromWheelSpeeds(
                    kinematics.leftVelocityMps,
                    kinematics.rightVelocityMps,
                    _ukf.ukf().preparedParams(),
                    resolvedLinearSpeedMps,
                    resolvedYawRateRadps);
                kinematics.linearSpeedMps = resolvedLinearSpeedMps;
                return resolvedYawRateRadps;
            }();
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

    // DeltaCommand resolves a feedforward command from an explicit operating point and an explicit
    // longitudinal acceleration request. The result is symmetric by construction.
    // Supported `pd` flags here: `StateAccelerationPD`, `IMUForwardAccel`.
    // Any heading, yaw-rate, wheel-speed, encoder, or lateral-accel selection has no explicit target
    // on this overload and therefore defaults to hold/no-op behavior.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // DeltaCommand resolves the fully coupled plant feedforward from an explicit body-speed operating
    // point and explicit longitudinal/yaw acceleration requests. This is the canonical "delta from the
    // current state" entry point when both translation and rotation matter at once.
    // Supported `pd` flags here: `StateAccelerationPD`, `IMUForwardAccel`.
    // This overload does not expose a separate yaw-acceleration feedback-source selector, so heading,
    // yaw-rate, wheel-speed, encoder, and lateral-accel flags remain hold/no-op selections.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // DeltaYawRateCommand resolves a zero-mean command from the present yaw rate and a desired yaw
    // acceleration. This is the single-axis rotational variant of `DeltaCommand`.
    // No additional `pd` feedback target is currently exposed on this overload; every flag presently
    // behaves the same as `RawCommand`.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector DeltaYawRateCommand(
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommand resolves a symmetric command that drives the present forward speed toward the requested
    // forward speed over the canonical roll-off horizon while respecting the plant-reported acceleration
    // envelope. Wheel-speed or other optional loops use `desiredLinearSpeedMps` as their setpoint when
    // that association is meaningful; all unrelated loops hold their present values.
    // Supported `pd` flags here: `StateVelocityPD`, `StateWheelOmegaPD`, `EncoderVelocity`.
    // This overload does not set a new heading, yaw-rate, longitudinal-acceleration, or
    // lateral-acceleration target.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointCommand(
        float desiredLinearSpeedMps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommand resolves the fully coupled command that drives the present forward speed and yaw rate
    // toward the requested targets over the canonical roll-off horizon while respecting the plant envelope.
    // This replaces the old ambiguous "velocity command" entry point.
    // Supported `pd` flags here: `StateVelocityPD`, `StateYawPD`, `StateWheelOmegaPD`,
    // `EncoderVelocity`, `IMUYaw`.
    // This overload does not set a heading, longitudinal-acceleration, or lateral-acceleration target.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointCommand(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointControlVector resolves the same coupled target as `PointCommand`. It remains as the
    // stable entry point for loop-controller call sites that already operate in control-vector space.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointControlVector(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointCommandWithHeadingTarget resolves the same coupled velocity/yaw-rate target as
    // `PointCommand`, but it also lets the caller add DriveBase-owned heading correction in the
    // same composed command so feedforward/feedback decomposition remains authoritative here.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointCommandWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::CommandPD pointPd,
        MazeMap::CommandPD headingPd) const;

    // PointControlVectorWithHeadingTarget is the stable control-vector-space wrapper for
    // `PointCommandWithHeadingTarget`.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointControlVectorWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::CommandPD pointPd,
        MazeMap::CommandPD headingPd) const;

    // PointCommand consumes the drive-relevant target fields from a maneuver point. Higher-level
    // maneuver execution should target this overload instead of rebuilding scalar command bridges.
    // It exposes the same `pd` selections as the scalar `(desiredLinearSpeedMps, desiredYawRateRadps)`
    // overload because it forwards directly to that entry point.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointCommand(
        const MazeMap::ManeuverPoint& point,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointControlVector resolves the same maneuver-point target as `PointCommand`. It remains as the
    // stable entry point for loop-controller call sites that already operate in control-vector space.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointControlVector(
        const MazeMap::ManeuverPoint& point,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // PointYawRateCommand resolves a zero-mean command that drives the present yaw rate toward the
    // requested yaw-rate target over the canonical roll-off horizon while respecting the yaw-acceleration
    // limit reported by the plant.
    // Supported `pd` flags here: `StateYawPD`, `StateWheelOmegaPD`, `EncoderVelocity`, `IMUYaw`.
    // This overload does not set a new linear-speed, heading, longitudinal-acceleration, or
    // lateral-acceleration target.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector PointYawRateCommand(
        float desiredYawRateRadps,
        MazeMap::CommandPD pd = MazeMap::CommandPD::RawCommand) const;

    // FeedbackCommand produces a pure feedback command cluster. Each selected loop uses `setpoint` as
    // its target. Unselected loops contribute nothing. The returned command starts from the plant command
    // that preserves the present motion state, then layers the requested feedback objectives on top.
    // Supported `pd` flags here: `StateHeadingPD`, `StateYawPD`, `StateWheelOmegaPD`,
    // `StateVelocityPD`, `StateAccelerationPD`, `EncoderVelocity`, `IMUYaw`,
    // `IMUForwardAccel`, `IMULateralAccel`.
    // `IMULateralAccel` still requires a nonzero present or target speed so the requested lateral
    // acceleration can be converted into a yaw-rate correction.
    EXPORT MazeMap::App::Internal::LoopController::ControlVector FeedbackCommand(
        float setpoint,
        MazeMap::CommandPD pd) const;

    EXPORT void CommandGenerated(
        const MazeMap::App::Internal::LoopController::ControlVector& command,
        float linearSpeedMps,
        float angularSpeedRadps,
        bool applyLaunchAssist = true);

    EXPORT void CommandOpenLoopRaw(const MazeMap::App::Internal::LoopController::ControlVector& command);

    void Brake()
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _lastFeedforwardCommandAverage = 0.0f;
        _lastFeedforwardCommandDelta = 0.0f;
        _lastFeedbackCommandAverage = 0.0f;
        _lastFeedbackCommandDelta = 0.0f;
        _lastLeftFeedforwardCommand = 0.0f;
        _lastRightFeedforwardCommand = 0.0f;
        _lastLeftFeedbackCommand = 0.0f;
        _lastRightFeedbackCommand = 0.0f;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastLeftLaunchAssistFloor = 0.0f;
        _lastRightLaunchAssistFloor = 0.0f;
        _lastFeedforwardUsedAlignedCycleContext = false;
        _lastFeedforwardUsedGripOnlyFallback = false;
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

    MazeMap::VehicleState GetVehicleState() const noexcept
    {
        MazeMap::VehicleState vehicleState{};
        vehicleState.SetStateVector(_ukf.ukf().state());
        vehicleState.SetCovariance(_ukf.ukf().covariance());
        return vehicleState;
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

    // The generated-command decomposition is cached at the point where DriveBase still owns both
    // the plant-model feedforward command and the PD-only correction. The avg/delta convention is
    // `left = average + delta`, `right = average - delta`.
    float GetLastFeedforwardCommandAverage() const noexcept
    {
        return _lastFeedforwardCommandAverage;
    }

    float GetLastFeedforwardCommandDelta() const noexcept
    {
        return _lastFeedforwardCommandDelta;
    }

    float GetLastFeedbackCommandAverage() const noexcept
    {
        return _lastFeedbackCommandAverage;
    }

    float GetLastFeedbackCommandDelta() const noexcept
    {
        return _lastFeedbackCommandDelta;
    }

    DriveTelemetry GetGeneratedTelemetry(
        const MazeMap::App::Internal::LoopController::ControlVector& command) const noexcept
    {
        DriveTelemetry telemetry{};
        const MazeMap::ModelCycleContext& cycleContext = _ukf.ukf().modelCycleContext();
        const MazeMap::VehicleState::StateMatrix covariance = _ukf.ukf().covariance();
        telemetry.leftDriveCommand = command.leftMotorPwm;
        telemetry.rightDriveCommand = command.rightMotorPwm;
        telemetry.leftFeedforwardCommand = _lastLeftFeedforwardCommand;
        telemetry.rightFeedforwardCommand = _lastRightFeedforwardCommand;
        telemetry.leftFeedbackCommand = _lastLeftFeedbackCommand;
        telemetry.rightFeedbackCommand = _lastRightFeedbackCommand;
        telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
        telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
        telemetry.leftLaunchAssistFloor = _lastLeftLaunchAssistFloor;
        telemetry.rightLaunchAssistFloor = _lastRightLaunchAssistFloor;
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
        telemetry.ukfClosureResidualLeftMps = _ukf.ukf().closureResidualLeftMps();
        telemetry.ukfClosureResidualRightMps = _ukf.ukf().closureResidualRightMps();
        telemetry.ukfLongitudinalClosureSeverity = cycleContext.utilization.longitudinalClosureSeverity;
        telemetry.ukfDifferentialClosureSeverity = cycleContext.utilization.differentialClosureSeverity;
        telemetry.ukfLateralAccelerationSeverity = cycleContext.utilization.lateralAccelerationSeverity;
        telemetry.ukfYawConsistencySeverity = cycleContext.utilization.yawConsistencySeverity;
        telemetry.ukfLeftBankAnomalySeverity = cycleContext.utilization.leftBankAnomalySeverity;
        telemetry.ukfRightBankAnomalySeverity = cycleContext.utilization.rightBankAnomalySeverity;
        telemetry.ukfLeftPreProjectionUtilization = cycleContext.utilization.leftBankPreProjectionUtilization;
        telemetry.ukfRightPreProjectionUtilization = cycleContext.utilization.rightBankPreProjectionUtilization;
        telemetry.ukfLeftBankMemory = cycleContext.memory.leftBankMemory;
        telemetry.ukfRightBankMemory = cycleContext.memory.rightBankMemory;
        telemetry.ukfLeftBankRecoveryScore = cycleContext.regrip.leftBankRecoveryScore;
        telemetry.ukfRightBankRecoveryScore = cycleContext.regrip.rightBankRecoveryScore;
        telemetry.ukfLeftBankRecoveryTimeRemainingS = cycleContext.regrip.leftBankRecoveryTimeRemainingS;
        telemetry.ukfRightBankRecoveryTimeRemainingS = cycleContext.regrip.rightBankRecoveryTimeRemainingS;
        telemetry.ukfStationaryCandidateDwellS = _ukf.ukf().stationaryCandidateDwellS();
        telemetry.ukfLaunchHoldRemainingS = _ukf.ukf().launchHoldRemainingS();
        telemetry.ukfInconsistentHoldRemainingS = _ukf.ukf().inconsistentHoldRemainingS();
        telemetry.ukfNhcReenableDelayRemainingS = _ukf.ukf().nhcReenableDelayRemainingS();
        telemetry.ukfForwardProcessNoiseScale = cycleContext.schedule.forwardSpeedProcessNoiseScale;
        telemetry.ukfLateralProcessNoiseScale = cycleContext.schedule.lateralSpeedProcessNoiseScale;
        telemetry.ukfYawRateProcessNoiseScale = cycleContext.schedule.yawRateProcessNoiseScale;
        telemetry.ukfLeftWheelProcessNoiseScale = cycleContext.schedule.leftWheelSpeedProcessNoiseScale;
        telemetry.ukfRightWheelProcessNoiseScale = cycleContext.schedule.rightWheelSpeedProcessNoiseScale;
        telemetry.ukfClosureCovarianceScaleLeft = cycleContext.schedule.closureCovarianceScaleLeft;
        telemetry.ukfClosureCovarianceScaleRight = cycleContext.schedule.closureCovarianceScaleRight;
        telemetry.ukfLateralPseudoCovarianceScale = cycleContext.schedule.lateralPseudoMeasurementCovarianceScale;
        telemetry.ukfAppliedLeftBankTorqueNm = cycleContext.appliedTorque.leftAppliedBankTorqueNm;
        telemetry.ukfAppliedRightBankTorqueNm = cycleContext.appliedTorque.rightAppliedBankTorqueNm;
        telemetry.ukfGyroInnovationRadps = _ukf.ukf().gyroInnovationRadps();
        telemetry.ukfForwardAccelInnovationMps2 = _ukf.ukf().forwardAccelInnovationMps2();
        telemetry.ukfLateralAccelInnovationMps2 = _ukf.ukf().lateralAccelInnovationMps2();
        telemetry.ukfGyroInnovationNis = _ukf.ukf().gyroInnovationNis();
        telemetry.ukfForwardAccelInnovationNis = _ukf.ukf().forwardAccelInnovationNis();
        telemetry.ukfLateralAccelInnovationNis = _ukf.ukf().lateralAccelInnovationNis();
        telemetry.ukfClosureLeftNis = _ukf.ukf().closureLeftNis();
        telemetry.ukfClosureRightNis = _ukf.ukf().closureRightNis();
        telemetry.ukfLateralPseudoNis = _ukf.ukf().lateralPseudoNis();
        telemetry.ukfForwardSpeedVariance = covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU);
        telemetry.ukfLateralSpeedVariance = covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV);
        telemetry.ukfYawRateVariance = covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR);
        telemetry.ukfLeftWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL);
        telemetry.ukfRightWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR);
        telemetry.ukfGyroBiasVariance = covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz);
        telemetry.ukfExactStationaryLock = cycleContext.schedule.exactStationaryLock ? 1U : 0U;
        telemetry.ukfLeftBankHoldoffActive = cycleContext.schedule.leftBankHoldoffActive ? 1U : 0U;
        telemetry.ukfRightBankHoldoffActive = cycleContext.schedule.rightBankHoldoffActive ? 1U : 0U;
        telemetry.ukfLeftBankInRecovery = cycleContext.regrip.leftBankInRecovery ? 1U : 0U;
        telemetry.ukfRightBankInRecovery = cycleContext.regrip.rightBankInRecovery ? 1U : 0U;
        telemetry.ukfDirectWheelUpdateBodyInvariant = _ukf.ukf().directWheelUpdateBodyStateInvariant() ? 1U : 0U;
        telemetry.ukfReleaseInflationApplied = _ukf.ukf().releaseInflationApplied() ? 1U : 0U;
        telemetry.feedforwardUsedAlignedCycleContext = _lastFeedforwardUsedAlignedCycleContext ? 1U : 0U;
        telemetry.feedforwardUsedGripOnlyFallback = _lastFeedforwardUsedGripOnlyFallback ? 1U : 0U;
        return telemetry;
    }

    DriveTelemetry GetTelemetry() const
    {
        DriveTelemetry telemetry{};
        const int32_t pendingLeftCounts = _leftMotor.getEncoderCount();
        const int32_t pendingRightCounts = _rightMotor.getEncoderCount();
        const MazeMap::VehicleState::StateMatrix covariance = _ukf.ukf().covariance();
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
        const MazeMap::ModelCycleContext& cycleContext = _ukf.ukf().modelCycleContext();
        telemetry.ukfClosureResidualLeftMps = _ukf.ukf().closureResidualLeftMps();
        telemetry.ukfClosureResidualRightMps = _ukf.ukf().closureResidualRightMps();
        telemetry.ukfLongitudinalClosureSeverity = cycleContext.utilization.longitudinalClosureSeverity;
        telemetry.ukfDifferentialClosureSeverity = cycleContext.utilization.differentialClosureSeverity;
        telemetry.ukfLateralAccelerationSeverity = cycleContext.utilization.lateralAccelerationSeverity;
        telemetry.ukfYawConsistencySeverity = cycleContext.utilization.yawConsistencySeverity;
        telemetry.ukfLeftBankAnomalySeverity = cycleContext.utilization.leftBankAnomalySeverity;
        telemetry.ukfRightBankAnomalySeverity = cycleContext.utilization.rightBankAnomalySeverity;
        telemetry.ukfLeftPreProjectionUtilization = cycleContext.utilization.leftBankPreProjectionUtilization;
        telemetry.ukfRightPreProjectionUtilization = cycleContext.utilization.rightBankPreProjectionUtilization;
        telemetry.ukfLeftBankMemory = cycleContext.memory.leftBankMemory;
        telemetry.ukfRightBankMemory = cycleContext.memory.rightBankMemory;
        telemetry.ukfLeftBankRecoveryScore = cycleContext.regrip.leftBankRecoveryScore;
        telemetry.ukfRightBankRecoveryScore = cycleContext.regrip.rightBankRecoveryScore;
        telemetry.ukfLeftBankRecoveryTimeRemainingS = cycleContext.regrip.leftBankRecoveryTimeRemainingS;
        telemetry.ukfRightBankRecoveryTimeRemainingS = cycleContext.regrip.rightBankRecoveryTimeRemainingS;
        telemetry.ukfStationaryCandidateDwellS = _ukf.ukf().stationaryCandidateDwellS();
        telemetry.ukfLaunchHoldRemainingS = _ukf.ukf().launchHoldRemainingS();
        telemetry.ukfInconsistentHoldRemainingS = _ukf.ukf().inconsistentHoldRemainingS();
        telemetry.ukfNhcReenableDelayRemainingS = _ukf.ukf().nhcReenableDelayRemainingS();
        telemetry.ukfForwardProcessNoiseScale = cycleContext.schedule.forwardSpeedProcessNoiseScale;
        telemetry.ukfLateralProcessNoiseScale = cycleContext.schedule.lateralSpeedProcessNoiseScale;
        telemetry.ukfYawRateProcessNoiseScale = cycleContext.schedule.yawRateProcessNoiseScale;
        telemetry.ukfLeftWheelProcessNoiseScale = cycleContext.schedule.leftWheelSpeedProcessNoiseScale;
        telemetry.ukfRightWheelProcessNoiseScale = cycleContext.schedule.rightWheelSpeedProcessNoiseScale;
        telemetry.ukfClosureCovarianceScaleLeft = cycleContext.schedule.closureCovarianceScaleLeft;
        telemetry.ukfClosureCovarianceScaleRight = cycleContext.schedule.closureCovarianceScaleRight;
        telemetry.ukfLateralPseudoCovarianceScale = cycleContext.schedule.lateralPseudoMeasurementCovarianceScale;
        telemetry.ukfAppliedLeftBankTorqueNm = cycleContext.appliedTorque.leftAppliedBankTorqueNm;
        telemetry.ukfAppliedRightBankTorqueNm = cycleContext.appliedTorque.rightAppliedBankTorqueNm;
        telemetry.ukfGyroInnovationRadps = _ukf.ukf().gyroInnovationRadps();
        telemetry.ukfForwardAccelInnovationMps2 = _ukf.ukf().forwardAccelInnovationMps2();
        telemetry.ukfLateralAccelInnovationMps2 = _ukf.ukf().lateralAccelInnovationMps2();
        telemetry.ukfGyroInnovationNis = _ukf.ukf().gyroInnovationNis();
        telemetry.ukfForwardAccelInnovationNis = _ukf.ukf().forwardAccelInnovationNis();
        telemetry.ukfLateralAccelInnovationNis = _ukf.ukf().lateralAccelInnovationNis();
        telemetry.ukfClosureLeftNis = _ukf.ukf().closureLeftNis();
        telemetry.ukfClosureRightNis = _ukf.ukf().closureRightNis();
        telemetry.ukfLateralPseudoNis = _ukf.ukf().lateralPseudoNis();
        telemetry.ukfForwardSpeedVariance = covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU);
        telemetry.ukfLateralSpeedVariance = covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV);
        telemetry.ukfYawRateVariance = covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR);
        telemetry.ukfLeftWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL);
        telemetry.ukfRightWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR);
        telemetry.ukfGyroBiasVariance = covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz);
        telemetry.ukfExactStationaryLock = cycleContext.schedule.exactStationaryLock ? 1U : 0U;
        telemetry.ukfLeftBankHoldoffActive = cycleContext.schedule.leftBankHoldoffActive ? 1U : 0U;
        telemetry.ukfRightBankHoldoffActive = cycleContext.schedule.rightBankHoldoffActive ? 1U : 0U;
        telemetry.ukfLeftBankInRecovery = cycleContext.regrip.leftBankInRecovery ? 1U : 0U;
        telemetry.ukfRightBankInRecovery = cycleContext.regrip.rightBankInRecovery ? 1U : 0U;
        telemetry.ukfDirectWheelUpdateBodyInvariant = _ukf.ukf().directWheelUpdateBodyStateInvariant() ? 1U : 0U;
        telemetry.ukfReleaseInflationApplied = _ukf.ukf().releaseInflationApplied() ? 1U : 0U;
        telemetry.feedforwardUsedAlignedCycleContext = _lastFeedforwardUsedAlignedCycleContext ? 1U : 0U;
        telemetry.feedforwardUsedGripOnlyFallback = _lastFeedforwardUsedGripOnlyFallback ? 1U : 0U;
        telemetry.encoderObservationValid = _encoderObservationValid;
        return telemetry;
    }

    static void BuildLoggedFrontPairObservations(
        const SensorSnapshot& snapshot,
        float maxRangeM,
        MazeMap::WallObs& left,
        MazeMap::WallObs& right) noexcept
    {
        BuildUkfFrontPairObservations(snapshot, maxRangeM, left, right);
    }

    static MazeMap::WallObs BuildLoggedLeftSideObservation(
        const SensorSnapshot& snapshot,
        float maxRangeM) noexcept
    {
        return BuildUkfLeftSideObservation(snapshot, maxRangeM);
    }

    static MazeMap::WallObs BuildLoggedRightSideObservation(
        const SensorSnapshot& snapshot,
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
        _lastImuYawRateRadps = snapshot.gyroRadps;
        _lastImuYawRateValid = std::isfinite(snapshot.gyroRadps);
        _lastImuAccelBodyXMps2 = snapshot.accelBodyXMps2;
        _lastImuAccelBodyYMps2 = snapshot.accelBodyYMps2;
        _lastImuAccelValid =
            snapshot.accelBiasValid &&
            std::isfinite(snapshot.accelBodyXMps2) &&
            std::isfinite(snapshot.accelBodyYMps2);
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
                    timing->ukfTotalDurationUs = timing->ukfPredictDurationUs;
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
        const bool updateYawFromEncoder = !std::isfinite(snapshot.gyroRawRadps);
        (void)_ukf.updateEncoderPair(encoderObservation, dtSeconds, updateYawFromEncoder, loopHook);

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
                    timing->ukfTotalDurationUs = timing->ukfPredictDurationUs + timing->ukfUpdateDurationUs;
                }
                return;
            }
        }

        MazeMap::ImuAccelObs accelObservation{};
        accelObservation.valid =
            snapshot.accelBiasValid &&
            std::isfinite(snapshot.accelBodyXMps2) &&
            std::isfinite(snapshot.accelBodyYMps2);
        accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
        accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
        (void)_ukf.updatePlanarAccel(accelObservation, loopHook);

        if (map != nullptr)
        {
            MazeMap::WallObs frontLeftObs{};
            MazeMap::WallObs frontRightObs{};
            BuildUkfFrontPairObservations(snapshot, params.noHitRangeM, frontLeftObs, frontRightObs);
            if (frontLeftObs.valid && frontRightObs.valid)
            {
                (void)_ukf.updateFrontPair(frontLeftObs, frontRightObs, *map, true);
            }

            const MazeMap::WallObs leftSideObs = BuildUkfLeftSideObservation(snapshot, params.noHitRangeM);
            if (leftSideObs.valid)
            {
                (void)_ukf.updateSideSensor(MazeMap::Side::Left, leftSideObs, *map, true);
            }

            const MazeMap::WallObs rightSideObs = BuildUkfRightSideObservation(snapshot, params.noHitRangeM);
            if (rightSideObs.valid)
            {
                (void)_ukf.updateSideSensor(MazeMap::Side::Right, rightSideObs, *map, true);
            }
        }
        if (timing != nullptr)
        {
            timing->ukfUpdateEndUs = micros();
            timing->ukfUpdateDurationUs = timing->ukfUpdateEndUs - timing->ukfUpdateStartUs;
            timing->ukfTotalDurationUs = timing->ukfPredictDurationUs + timing->ukfUpdateDurationUs;
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
    const MazeMap::PlantModel& _plantModel;
    const MazeMap::ProportionalDerivativeCluster* _proportionalDerivativeCluster;
    PoseEstimate _poseCache;
    float _leftIntegral;
    float _rightIntegral;
    mutable float _lastLinearCommandMps;
    mutable float _lastAngularCommandRadps;
    mutable float _lastFeedforwardCommandAverage = 0.0f;
    mutable float _lastFeedforwardCommandDelta = 0.0f;
    mutable float _lastFeedbackCommandAverage = 0.0f;
    mutable float _lastFeedbackCommandDelta = 0.0f;
    mutable float _lastLeftFeedforwardCommand = 0.0f;
    mutable float _lastRightFeedforwardCommand = 0.0f;
    mutable float _lastLeftFeedbackCommand = 0.0f;
    mutable float _lastRightFeedbackCommand = 0.0f;
    mutable float _lastLeftTargetVelocityMps = 0.0f;
    mutable float _lastRightTargetVelocityMps = 0.0f;
    mutable float _lastLeftLaunchAssistFloor = 0.0f;
    mutable float _lastRightLaunchAssistFloor = 0.0f;
    mutable bool _lastFeedforwardUsedAlignedCycleContext = false;
    mutable bool _lastFeedforwardUsedGripOnlyFallback = false;
    int32_t _leftEncoderCountTotal = 0;
    int32_t _rightEncoderCountTotal = 0;
    float _leftEncoderDistanceMeters = 0.0f;
    float _rightEncoderDistanceMeters = 0.0f;
    float _leftEncoderVelocityMps = 0.0f;
    float _rightEncoderVelocityMps = 0.0f;
    float _lastGyroRawRadps = 0.0f;
    float _lastImuYawRateRadps = 0.0f;
    float _lastImuAccelBodyXMps2 = 0.0f;
    float _lastImuAccelBodyYMps2 = 0.0f;
    MazeMap::EncoderObs _lastEncoderObservation{};
    mutable uint16_t _lastModeFlags = kModeBraking;
    mutable uint16_t _lastSaturationFlags = 0u;
    bool _encoderObservationValid = false;
    bool _lastImuYawRateValid = false;
    bool _lastImuAccelValid = false;
    MazeMap::WheelControlProfile _wheelControlProfile;
    struct CommandContext
    {
        MazeMap::VehicleState::StateVector presentState = MazeMap::VehicleState::StateVector::Zero();
        MazeMap::PlantDerivatives presentDerivatives{};
        float batteryVoltageV = 0.0f;
        float wheelRadiusM = 0.0f;
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
        bool hasWheelOmegaTargets = false;
        float leftWheelOmegaTargetRadps = 0.0f;
        float rightWheelOmegaTargetRadps = 0.0f;
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
        float& batteryVoltageV) const;

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

    static float ResolveCommandResponseTimeS() noexcept;

    static MazeMap::App::Internal::LoopController::ControlVector AddDriveCommands(
        const MazeMap::App::Internal::LoopController::ControlVector& lhs,
        const MazeMap::App::Internal::LoopController::ControlVector& rhs) noexcept;

    static MazeMap::App::Internal::LoopController::ControlVector SubtractDriveCommands(
        const MazeMap::App::Internal::LoopController::ControlVector& lhs,
        const MazeMap::App::Internal::LoopController::ControlVector& rhs) noexcept;

    static float AverageDriveCommand(
        const MazeMap::App::Internal::LoopController::ControlVector& command) noexcept;

    static float DeltaDriveCommand(
        const MazeMap::App::Internal::LoopController::ControlVector& command) noexcept;

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

    MazeMap::App::Internal::LoopController::ControlVector ResolveRawAccelerationCommand(
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2) const;

    MazeMap::App::Internal::LoopController::ControlVector ResolveRawVelocityTargetCommand(
        const CommandContext& context,
        float desiredLinearSpeedMps,
        float desiredYawRateRadps) const;

    MazeMap::App::Internal::LoopController::ControlVector ResolveLongitudinalCorrectionCommand(
        const CommandContext& context,
        float desiredLongitudinalAccelCorrectionMps2) const;

    MazeMap::App::Internal::LoopController::ControlVector ResolveYawCorrectionCommand(
        const CommandContext& context,
        float desiredYawAccelCorrectionRadps2) const;

    MazeMap::App::Internal::LoopController::ControlVector ComposeGeneratedCommand(
        const MazeMap::App::Internal::LoopController::ControlVector& baseCommand,
        const CommandContext& context,
        const CommandTargets& targets,
        MazeMap::CommandPD pd) const;

    void CacheGeneratedCommandTelemetry(
        const MazeMap::App::Internal::LoopController::ControlVector& feedforwardCommand,
        const MazeMap::App::Internal::LoopController::ControlVector& feedbackCommand) const noexcept;

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

    void ResolveDefaultVelocityTargetOperatingEnvelope(
        const MazeMap::VehicleState::StateVector& presentState,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const
    {
        maxLongitudinalAccelMps2 = kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2;
        maxYawAccelRadps2 = kDefaultCommandVelocityAsapYawAccelLimitRadps2;

        float technicalLongitudinalAccelMps2 = 0.0f;
        float technicalYawAccelRadps2 = 0.0f;
        _plantModel.velocityTargetTechnicalLimits(
            presentState,
            _ukf.ukf().preparedParams(),
            _ukf.ukf().modelCycleContext(),
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

    float GetWheelVelocityKp() const;

    float GetWheelVelocityKi() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKi, _wheelControlProfile.velocityKiScale);
    }

    float GetWheelIntegralLimit() const;

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



