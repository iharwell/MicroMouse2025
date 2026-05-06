#pragma once
// Defines the concrete runtime motion owner that produces wheel/motor commands, maintains odometry,
// and runs the closed-loop wheel/pose control machinery. This is not the planned higher-level
// "Drive" translation layer; it is the low-level destination for concrete motion commands.
#include "CommandPD.h"
#include "DriveTelemetry.h"
#include "EncoderObs.h"
#include "LaunchAssistProfile.h"
#include "LoopController.h"
#include "Maneuver.h"
#include "MazeMapRuntimeCore.h"
#include "MotorEncoderDrive.h"
#include "Estimator.h"
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
// encoder measurement capture, borrowed estimator state, and motion-command production. It should not become the
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
        MazeMap::Estimator& estimator,
        const MazeMap::ProportionalDerivativeCluster& proportionalDerivativeCluster)
        : _leftMotor(MazeMap::MotorEncoderDrive::CreateDefaultLeftDrive())
        , _rightMotor(MazeMap::MotorEncoderDrive::CreateDefaultRightDrive())
        , _estimator(estimator)
        , _plantModel(plantModel)
        , _proportionalDerivativeCluster(&proportionalDerivativeCluster)
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
        (void)_estimator.ResetPose(0.0f, 0.0f, DirectionToYawRad(MazeMap::Up));
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
        (void)_estimator.ResetPose(xMeters, yMeters, DirectionToYawRad(logical.GetDirection()));
        ResetControllers();
		startSet = true;
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPose(float xMeters, float yMeters, float yawRad)
    {
        (void)_estimator.ResetPose(xMeters, yMeters, yawRad);
        ResetControllers();
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPoseXMeters(float xMeters)
    {
        if (std::isfinite(xMeters))
        {
            (void)_estimator.SetStateCoordinate(MazeMap::VehicleState::kPx, xMeters);
        }
    }

    // DANGEROUS!!!! Do not use unless you've literally made contact with a physical reference like a wall.
    void SetPoseYMeters(float yMeters)
    {
        if (std::isfinite(yMeters))
        {
            (void)_estimator.SetStateCoordinate(MazeMap::VehicleState::kPy, yMeters);
        }
    }

    void SetGyroBiasZ(float gyroBiasRadps)
    {
        (void)_estimator.SetGyroBiasZ(gyroBiasRadps);
    }

    bool RestoreSessionStartPhysicalState(float xMeters, float yMeters, float yawRad) noexcept
    {
        (void)ConsumeEncoderObservation(0.0f);
        (void)ConsumeEncoderObservation(0.0f);
        ResetEncoderTracking();
        _lastGyroRawRadps = 0.0f;
        _lastImuYawRateRadps = 0.0f;
        _lastImuAccelBodyXMps2 = 0.0f;
        _lastImuAccelBodyYMps2 = 0.0f;
        _lastImuYawRateValid = false;
        _lastImuAccelValid = false;
        const bool restored = _estimator.ResetForSessionTransition(xMeters, yMeters, yawRad);
        ResetControllers();
        Brake();
        return restored;
    }

    void RecordMeasurementInputs(const SensorSnapshot& snapshot) noexcept
    {
        _lastGyroRawRadps = snapshot.gyroRawRadps;
        _lastImuYawRateRadps = snapshot.gyroRadps;
        _lastImuYawRateValid = std::isfinite(snapshot.gyroRadps);
        _lastImuAccelBodyXMps2 = snapshot.accelBodyXMps2;
        _lastImuAccelBodyYMps2 = snapshot.accelBodyYMps2;
        _lastImuAccelValid =
            snapshot.accelBiasValid &&
            std::isfinite(snapshot.accelBodyXMps2) &&
            std::isfinite(snapshot.accelBodyYMps2);
    }

    MazeMap::App::Internal::LoopController::ControlVector CurrentControlVector() const noexcept
    {
        return MazeMap::App::Internal::LoopController::ControlVector::RawMotorPwm(
            _leftMotor.getDriveCommand(),
            _rightMotor.getDriveCommand());
    }

    float CurrentBatteryVoltageV() const noexcept
    {
        return 0.5f * (_leftMotor.getVoltage() + _rightMotor.getVoltage());
    }

    MazeMap::EncoderObs ConsumeEncoderObservation(float dtSeconds)
    {
        const EncoderCycleSample sample = ConsumeEncoderCycleSample(dtSeconds, _estimator.ukf().params());
        MazeMap::EncoderObs encoderObservation{};
        encoderObservation.totalLeftCounts = sample.leftCounts;
        encoderObservation.totalRightCounts = sample.rightCounts;
        encoderObservation.omegaLeftRadps = sample.leftOmegaRadps;
        encoderObservation.omegaRightRadps = sample.rightOmegaRadps;
        _lastEncoderObservation = encoderObservation;
        _encoderObservationValid = true;
        return encoderObservation;
    }

    void ProjectMeasuredKinematics(float dtSeconds, float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN())
    {
        _estimator.ProjectMeasuredKinematics(dtSeconds, _lastEncoderObservation, measuredYawRateRadps);
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

    template <typename Sink>
    bool WriteUkfDebugTextDump(Sink&& sink) const noexcept
    {
        return _estimator.ukf().WriteDebugTextDump(static_cast<Sink&&>(sink));
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
        const MazeMap::SrUkfCore& ukf = _estimator.ukf();
        const MazeMap::VehicleState::StateMatrix covariance = _estimator.ukf().covariance();
        telemetry.leftDriveCommand = command.leftMotorPwm;
        telemetry.rightDriveCommand = command.rightMotorPwm;
        telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
        telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
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
        telemetry.ukfModeId = ukf.operatingModeId();
        telemetry.ukfYawValidForFeedforward = ukf.yawValidForFeedforward() ? 1U : 0U;
        telemetry.ukfBiasUpdateEnabled = ukf.biasUpdateEnabled() ? 1U : 0U;
        telemetry.ukfNhcEnabled = ukf.nonholonomicConstraintEnabled() ? 1U : 0U;
        telemetry.ukfGyroBiasAnchorRadps = ukf.gyroBiasAnchorRadps();
        telemetry.ukfYawConsistencyLowPassRadps = ukf.yawConsistencyLowPassRadps();
        telemetry.ukfYawWindowMismatchRad = ukf.yawWindowMismatchRad();
        telemetry.ukfNhcSigmaMps = ukf.nhcSigmaMps();
        telemetry.ukfNhcResidualMps = ukf.nhcResidualMps();
        telemetry.ukfNhcResidualSigma = ukf.nhcResidualSigma();
        telemetry.ukfFeedforwardYawRateRadps = ukf.resolveYawRateForFeedforward(_lastGyroRawRadps);
        telemetry.ukfClosureResidualLeftMps = ukf.closureResidualLeftMps();
        telemetry.ukfClosureResidualRightMps = ukf.closureResidualRightMps();
        telemetry.ukfLongitudinalClosureSeverity = ukf.longitudinalClosureSeverity();
        telemetry.ukfDifferentialClosureSeverity = ukf.differentialClosureSeverity();
        telemetry.ukfLateralAccelerationSeverity = ukf.lateralAccelerationSeverity();
        telemetry.ukfYawConsistencySeverity = ukf.yawConsistencySeverity();
        telemetry.ukfLeftBankAnomalySeverity = ukf.leftBankAnomalySeverity();
        telemetry.ukfRightBankAnomalySeverity = ukf.rightBankAnomalySeverity();
        telemetry.ukfLeftPreProjectionUtilization = ukf.leftBankPreProjectionUtilization();
        telemetry.ukfRightPreProjectionUtilization = ukf.rightBankPreProjectionUtilization();
        telemetry.ukfLeftBankMemory = ukf.leftBankMemory();
        telemetry.ukfRightBankMemory = ukf.rightBankMemory();
        telemetry.ukfLeftBankRecoveryScore = ukf.leftBankRecoveryScore();
        telemetry.ukfRightBankRecoveryScore = ukf.rightBankRecoveryScore();
        telemetry.ukfLeftBankRecoveryTimeRemainingS = ukf.leftBankRecoveryTimeRemainingS();
        telemetry.ukfRightBankRecoveryTimeRemainingS = ukf.rightBankRecoveryTimeRemainingS();
        telemetry.ukfStationaryCandidateDwellS = ukf.stationaryCandidateDwellS();
        telemetry.ukfLaunchHoldRemainingS = ukf.launchHoldRemainingS();
        telemetry.ukfInconsistentHoldRemainingS = ukf.inconsistentHoldRemainingS();
        telemetry.ukfNhcReenableDelayRemainingS = ukf.nhcReenableDelayRemainingS();
        telemetry.ukfForwardProcessNoiseScale = ukf.forwardSpeedProcessNoiseScale();
        telemetry.ukfLateralProcessNoiseScale = ukf.lateralSpeedProcessNoiseScale();
        telemetry.ukfYawRateProcessNoiseScale = ukf.yawRateProcessNoiseScale();
        telemetry.ukfLeftWheelProcessNoiseScale = ukf.leftWheelSpeedProcessNoiseScale();
        telemetry.ukfRightWheelProcessNoiseScale = ukf.rightWheelSpeedProcessNoiseScale();
        telemetry.ukfClosureCovarianceScaleLeft = ukf.closureCovarianceScaleLeft();
        telemetry.ukfClosureCovarianceScaleRight = ukf.closureCovarianceScaleRight();
        telemetry.ukfLateralPseudoCovarianceScale = ukf.lateralPseudoMeasurementCovarianceScale();
        telemetry.ukfAppliedLeftBankTorqueNm = ukf.appliedLeftBankTorqueNm();
        telemetry.ukfAppliedRightBankTorqueNm = ukf.appliedRightBankTorqueNm();
        telemetry.ukfGyroInnovationRadps = ukf.gyroInnovationRadps();
        telemetry.ukfForwardAccelInnovationMps2 = ukf.forwardAccelInnovationMps2();
        telemetry.ukfLateralAccelInnovationMps2 = ukf.lateralAccelInnovationMps2();
        telemetry.ukfGyroInnovationNis = ukf.gyroInnovationNis();
        telemetry.ukfForwardAccelInnovationNis = ukf.forwardAccelInnovationNis();
        telemetry.ukfLateralAccelInnovationNis = ukf.lateralAccelInnovationNis();
        telemetry.ukfClosureLeftNis = ukf.closureLeftNis();
        telemetry.ukfClosureRightNis = ukf.closureRightNis();
        telemetry.ukfLateralPseudoNis = ukf.lateralPseudoNis();
        telemetry.ukfForwardSpeedVariance = covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU);
        telemetry.ukfLateralSpeedVariance = covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV);
        telemetry.ukfYawRateVariance = covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR);
        telemetry.ukfLeftWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL);
        telemetry.ukfRightWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR);
        telemetry.ukfGyroBiasVariance = covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz);
        telemetry.ukfExactStationaryLock = ukf.exactStationaryLock() ? 1U : 0U;
        telemetry.ukfLeftBankHoldoffActive = ukf.leftBankHoldoffActive() ? 1U : 0U;
        telemetry.ukfRightBankHoldoffActive = ukf.rightBankHoldoffActive() ? 1U : 0U;
        telemetry.ukfLeftBankInRecovery = ukf.leftBankInRecovery() ? 1U : 0U;
        telemetry.ukfRightBankInRecovery = ukf.rightBankInRecovery() ? 1U : 0U;
        telemetry.ukfDirectWheelUpdateBodyInvariant = ukf.directWheelUpdateBodyStateInvariant() ? 1U : 0U;
        telemetry.ukfReleaseInflationApplied = ukf.releaseInflationApplied() ? 1U : 0U;
        return telemetry;
    }

    DriveTelemetry GetTelemetry() const
    {
        DriveTelemetry telemetry{};
        const MazeMap::App::Internal::LoopController::ControlVector appliedControl = CurrentControlVector();
        const int32_t pendingLeftCounts = _leftMotor.getEncoderCount();
        const int32_t pendingRightCounts = _rightMotor.getEncoderCount();
        const MazeMap::VehicleState::StateMatrix covariance = _estimator.ukf().covariance();
        telemetry.leftDriveCommand = appliedControl.leftMotorPwm;
        telemetry.rightDriveCommand = appliedControl.rightMotorPwm;
        telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
        telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
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
        telemetry.ukfModeId = _estimator.ukf().operatingModeId();
        telemetry.ukfYawValidForFeedforward = _estimator.ukf().yawValidForFeedforward() ? 1U : 0U;
        telemetry.ukfBiasUpdateEnabled = _estimator.ukf().biasUpdateEnabled() ? 1U : 0U;
        telemetry.ukfNhcEnabled = _estimator.ukf().nonholonomicConstraintEnabled() ? 1U : 0U;
        telemetry.ukfGyroBiasAnchorRadps = _estimator.ukf().gyroBiasAnchorRadps();
        telemetry.ukfYawConsistencyLowPassRadps = _estimator.ukf().yawConsistencyLowPassRadps();
        telemetry.ukfYawWindowMismatchRad = _estimator.ukf().yawWindowMismatchRad();
        telemetry.ukfNhcSigmaMps = _estimator.ukf().nhcSigmaMps();
        telemetry.ukfNhcResidualMps = _estimator.ukf().nhcResidualMps();
        telemetry.ukfNhcResidualSigma = _estimator.ukf().nhcResidualSigma();
        telemetry.ukfFeedforwardYawRateRadps = _estimator.ukf().resolveYawRateForFeedforward(_lastGyroRawRadps);
        const MazeMap::SrUkfCore& ukf = _estimator.ukf();
        telemetry.ukfClosureResidualLeftMps = _estimator.ukf().closureResidualLeftMps();
        telemetry.ukfClosureResidualRightMps = _estimator.ukf().closureResidualRightMps();
        telemetry.ukfLongitudinalClosureSeverity = ukf.longitudinalClosureSeverity();
        telemetry.ukfDifferentialClosureSeverity = ukf.differentialClosureSeverity();
        telemetry.ukfLateralAccelerationSeverity = ukf.lateralAccelerationSeverity();
        telemetry.ukfYawConsistencySeverity = ukf.yawConsistencySeverity();
        telemetry.ukfLeftBankAnomalySeverity = ukf.leftBankAnomalySeverity();
        telemetry.ukfRightBankAnomalySeverity = ukf.rightBankAnomalySeverity();
        telemetry.ukfLeftPreProjectionUtilization = ukf.leftBankPreProjectionUtilization();
        telemetry.ukfRightPreProjectionUtilization = ukf.rightBankPreProjectionUtilization();
        telemetry.ukfLeftBankMemory = ukf.leftBankMemory();
        telemetry.ukfRightBankMemory = ukf.rightBankMemory();
        telemetry.ukfLeftBankRecoveryScore = ukf.leftBankRecoveryScore();
        telemetry.ukfRightBankRecoveryScore = ukf.rightBankRecoveryScore();
        telemetry.ukfLeftBankRecoveryTimeRemainingS = ukf.leftBankRecoveryTimeRemainingS();
        telemetry.ukfRightBankRecoveryTimeRemainingS = ukf.rightBankRecoveryTimeRemainingS();
        telemetry.ukfStationaryCandidateDwellS = ukf.stationaryCandidateDwellS();
        telemetry.ukfLaunchHoldRemainingS = ukf.launchHoldRemainingS();
        telemetry.ukfInconsistentHoldRemainingS = ukf.inconsistentHoldRemainingS();
        telemetry.ukfNhcReenableDelayRemainingS = ukf.nhcReenableDelayRemainingS();
        telemetry.ukfForwardProcessNoiseScale = ukf.forwardSpeedProcessNoiseScale();
        telemetry.ukfLateralProcessNoiseScale = ukf.lateralSpeedProcessNoiseScale();
        telemetry.ukfYawRateProcessNoiseScale = ukf.yawRateProcessNoiseScale();
        telemetry.ukfLeftWheelProcessNoiseScale = ukf.leftWheelSpeedProcessNoiseScale();
        telemetry.ukfRightWheelProcessNoiseScale = ukf.rightWheelSpeedProcessNoiseScale();
        telemetry.ukfClosureCovarianceScaleLeft = ukf.closureCovarianceScaleLeft();
        telemetry.ukfClosureCovarianceScaleRight = ukf.closureCovarianceScaleRight();
        telemetry.ukfLateralPseudoCovarianceScale = ukf.lateralPseudoMeasurementCovarianceScale();
        telemetry.ukfAppliedLeftBankTorqueNm = ukf.appliedLeftBankTorqueNm();
        telemetry.ukfAppliedRightBankTorqueNm = ukf.appliedRightBankTorqueNm();
        telemetry.ukfGyroInnovationRadps = _estimator.ukf().gyroInnovationRadps();
        telemetry.ukfForwardAccelInnovationMps2 = _estimator.ukf().forwardAccelInnovationMps2();
        telemetry.ukfLateralAccelInnovationMps2 = _estimator.ukf().lateralAccelInnovationMps2();
        telemetry.ukfGyroInnovationNis = _estimator.ukf().gyroInnovationNis();
        telemetry.ukfForwardAccelInnovationNis = _estimator.ukf().forwardAccelInnovationNis();
        telemetry.ukfLateralAccelInnovationNis = _estimator.ukf().lateralAccelInnovationNis();
        telemetry.ukfClosureLeftNis = _estimator.ukf().closureLeftNis();
        telemetry.ukfClosureRightNis = _estimator.ukf().closureRightNis();
        telemetry.ukfLateralPseudoNis = _estimator.ukf().lateralPseudoNis();
        telemetry.ukfForwardSpeedVariance = covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU);
        telemetry.ukfLateralSpeedVariance = covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV);
        telemetry.ukfYawRateVariance = covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR);
        telemetry.ukfLeftWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL);
        telemetry.ukfRightWheelSpeedVariance = covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR);
        telemetry.ukfGyroBiasVariance = covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz);
        telemetry.ukfExactStationaryLock = ukf.exactStationaryLock() ? 1U : 0U;
        telemetry.ukfLeftBankHoldoffActive = ukf.leftBankHoldoffActive() ? 1U : 0U;
        telemetry.ukfRightBankHoldoffActive = ukf.rightBankHoldoffActive() ? 1U : 0U;
        telemetry.ukfLeftBankInRecovery = ukf.leftBankInRecovery() ? 1U : 0U;
        telemetry.ukfRightBankInRecovery = ukf.rightBankInRecovery() ? 1U : 0U;
        telemetry.ukfDirectWheelUpdateBodyInvariant = ukf.directWheelUpdateBodyStateInvariant() ? 1U : 0U;
        telemetry.ukfReleaseInflationApplied = ukf.releaseInflationApplied() ? 1U : 0U;
        telemetry.encoderObservationValid = _encoderObservationValid;
        return telemetry;
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

    void SetOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        _leftMotor.setDriveCommand((std::clamp)(leftDriveCommand, -1.0f, 1.0f));
        _rightMotor.setDriveCommand((std::clamp)(rightDriveCommand, -1.0f, 1.0f));
    }
    MazeMap::MotorEncoderDrive _leftMotor;
    MazeMap::MotorEncoderDrive _rightMotor;
    MazeMap::Estimator& _estimator;
    const MazeMap::PlantModel& _plantModel;
    const MazeMap::ProportionalDerivativeCluster* _proportionalDerivativeCluster;
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
        float trackWidthM = 0.0f;
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
        _estimator.ukf().alignedVelocityTargetTechnicalLimits(
            presentState,
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




