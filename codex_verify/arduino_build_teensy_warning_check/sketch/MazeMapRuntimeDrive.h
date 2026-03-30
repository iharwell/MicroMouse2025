#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapRuntimeDrive.h"
#pragma once
#include "MazeMapRuntimeCore.h"
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

class DriveBase
{
public:
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

    void SnapTo(MazeMap::DirectionalLocation logical)
    {
        float xMeters = 0.0f;
        float yMeters = 0.0f;
        logical.GetLocation().GetPhysicalLocation(Config::kCellSizeM, xMeters, yMeters);
        ResetPoseEstimate(xMeters, yMeters, DirectionToYawRad(logical.GetDirection()));
        ResetControllers();
    }

    void SetPose(float xMeters, float yMeters, float yawRad)
    {
        ResetPoseEstimate(xMeters, yMeters, yawRad);
        ResetControllers();
    }

    void SetPoseXMeters(float xMeters)
    {
        if (std::isfinite(xMeters))
        {
            SetEstimatorCoordinate(MazeMap::VehicleState::kPx, xMeters);
        }
    }

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
        const MazeMap::Maze* map = nullptr)
    {
        UpdatePoseEstimate(dtSeconds, snapshot, map);
    }

    void UpdateOdometry(
        float dtSeconds,
        const DiagnosticSensorSnapshot& snapshot,
        const MazeMap::Maze* map = nullptr)
    {
        UpdatePoseEstimate(dtSeconds, snapshot, map);
    }

    void CommandVelocity(float linearSpeedMps, float angularSpeedRadps, float dtSeconds)
    {
        const float previousLinearCommandMps = _lastLinearCommandMps;
        const float previousAngularCommandRadps = _lastAngularCommandRadps;
        _lastLinearCommandMps = linearSpeedMps;
        _lastAngularCommandRadps = angularSpeedRadps;
        const float effectiveTrackWidthM = MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(linearSpeedMps, angularSpeedRadps);
        const float leftTargetMps = linearSpeedMps - (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        const float rightTargetMps = linearSpeedMps + (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        float linearTargetAccelMps2 = 0.0f;
        float angularTargetAccelRadps2 = 0.0f;
        float leftTargetAccelMps2 = 0.0f;
        float rightTargetAccelMps2 = 0.0f;
        if (_targetVelocityInitialized && (dtSeconds > 0.0f))
        {
            linearTargetAccelMps2 = (linearSpeedMps - previousLinearCommandMps) / dtSeconds;
            angularTargetAccelRadps2 = (angularSpeedRadps - previousAngularCommandRadps) / dtSeconds;
            const float previousEffectiveTrackWidthM =
                MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(previousLinearCommandMps, previousAngularCommandRadps);
            const float previousLeftTargetMps =
                previousLinearCommandMps - (0.5f * previousEffectiveTrackWidthM * previousAngularCommandRadps);
            const float previousRightTargetMps =
                previousLinearCommandMps + (0.5f * previousEffectiveTrackWidthM * previousAngularCommandRadps);
            leftTargetAccelMps2 = (leftTargetMps - previousLeftTargetMps) / dtSeconds;
            rightTargetAccelMps2 = (rightTargetMps - previousRightTargetMps) / dtSeconds;
        }
        _targetVelocityInitialized = true;
        const float leftMeasuredMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        const float rightMeasuredMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        const unsigned long nowMs = millis();
        const bool leftLaunchAssistActive = UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftTargetMps, _leftMotor.getDriveCommand(), nowMs);
        const bool rightLaunchAssistActive = UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightTargetMps, _rightMotor.getDriveCommand(), nowMs);

        const float leftErrorMps = leftTargetMps - leftMeasuredMps;
        const float rightErrorMps = rightTargetMps - rightMeasuredMps;
        const float integralLimit = GetWheelIntegralLimit();

        _leftIntegral = (std::clamp)(_leftIntegral + (leftErrorMps * dtSeconds), -integralLimit, integralLimit);
        _rightIntegral = (std::clamp)(_rightIntegral + (rightErrorMps * dtSeconds), -integralLimit, integralLimit);

        const float leftFeedforwardCommand = ModelDriveFeedforwardForTargetMotion(
            _leftMotor,
            leftTargetMps,
            linearTargetAccelMps2,
            angularTargetAccelRadps2,
            effectiveTrackWidthM,
            true);
        const float rightFeedforwardCommand = ModelDriveFeedforwardForTargetMotion(
            _rightMotor,
            rightTargetMps,
            linearTargetAccelMps2,
            angularTargetAccelRadps2,
            effectiveTrackWidthM,
            false);
        float leftCommand = VelocityCommandFromError(
            leftFeedforwardCommand,
            leftTargetMps,
            leftTargetAccelMps2,
            leftErrorMps,
            _leftIntegral);
        float rightCommand = VelocityCommandFromError(
            rightFeedforwardCommand,
            rightTargetMps,
            rightTargetAccelMps2,
            rightErrorMps,
            _rightIntegral);
        if (leftLaunchAssistActive)
        {
            leftCommand = ApplyLaunchAssistFloor(
                leftCommand,
                leftTargetMps,
                GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs));
        }
        if (rightLaunchAssistActive)
        {
            rightCommand = ApplyLaunchAssistFloor(
                rightCommand,
                rightTargetMps,
                GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs));
        }

        _leftMotor.setDriveCommand(leftCommand);
        _rightMotor.setDriveCommand(rightCommand);
    }

    void CommandOpenLoop(float leftDriveCommand, float rightDriveCommand)
    {
        const float leftMeasuredMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        const float rightMeasuredMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        const unsigned long nowMs = millis();
        if (UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftDriveCommand, _leftMotor.getDriveCommand(), nowMs))
        {
            leftDriveCommand = ApplyLaunchAssistFloor(
                leftDriveCommand,
                leftDriveCommand,
                GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs));
        }
        if (UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightDriveCommand, _rightMotor.getDriveCommand(), nowMs))
        {
            rightDriveCommand = ApplyLaunchAssistFloor(
                rightDriveCommand,
                rightDriveCommand,
                GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs));
        }

        SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
    }

    void CommandOpenLoop(const MazeMap::OpenLoopDriveCommand& command)
    {
        CommandOpenLoop(command.leftDriveCommand, command.rightDriveCommand);
    }

    void CommandOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _targetVelocityInitialized = false;
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
        _targetVelocityInitialized = false;
        _leftMotor.brake();
        _rightMotor.brake();
    }

    float GetAverageDistanceMeters() const
    {
        return 0.5f * (_leftMotor.getEncoderDistanceMeters() + _rightMotor.getEncoderDistanceMeters());
    }

    const PoseEstimate& GetPose() const
    {
        return _poseCache;
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
        telemetry.leftDriveCommand = _leftMotor.getDriveCommand();
        telemetry.rightDriveCommand = _rightMotor.getDriveCommand();
        telemetry.leftEncoderCount = _leftMotor.getEncoderCount();
        telemetry.rightEncoderCount = _rightMotor.getEncoderCount();
        telemetry.leftDistanceM = _leftMotor.getEncoderDistanceMeters();
        telemetry.rightDistanceM = _rightMotor.getEncoderDistanceMeters();
        telemetry.leftVelocityMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        telemetry.rightVelocityMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        return telemetry;
    }

private:
    static MazeMap::VehicleState::StateMatrix BuildEstimatorCovariance()
    {
        MazeMap::VehicleState::StateMatrix covariance =
            MazeMap::VehicleState::StateMatrix::Identity() * 1.0e-3f;
        covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL) = 0.25f;
        covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR) = 0.25f;
        return covariance;
    }

    void ResetPoseEstimate(float xMeters, float yMeters, float yawRad)
    {
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(MazeMap::VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(MazeMap::VehicleState::kPsi) = WrapAngleRad(yawRad);
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

    static MazeMap::ImuMergedObs BuildUkfImuObservation(const SensorSnapshot& snapshot) noexcept
    {
        MazeMap::ImuMergedObs observation{};
        if (!std::isfinite(snapshot.gyroRadps) ||
            !std::isfinite(snapshot.accelBodyXMps2) ||
            !std::isfinite(snapshot.accelBodyYMps2))
        {
            return observation;
        }

        observation.valid = true;
        observation.yawRateRadps = snapshot.gyroRadps;
        observation.accelXComMps2 = snapshot.accelBodyXMps2;
        observation.accelYComMps2 = snapshot.accelBodyYMps2;
        return observation;
    }

    static MazeMap::ImuMergedObs BuildUkfImuObservation(const DiagnosticSensorSnapshot& snapshot) noexcept
    {
        MazeMap::ImuMergedObs observation{};
        if (!std::isfinite(snapshot.gyroRadps) ||
            !std::isfinite(snapshot.accelBodyXMps2) ||
            !std::isfinite(snapshot.accelBodyYMps2))
        {
            return observation;
        }

        observation.valid = true;
        observation.yawRateRadps = snapshot.gyroRadps;
        observation.accelXComMps2 = snapshot.accelBodyXMps2;
        observation.accelYComMps2 = snapshot.accelBodyYMps2;
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
    void UpdatePoseEstimate(float dtSeconds, const TSnapshot& snapshot, const MazeMap::Maze* map)
    {
        const MazeMap::PlantParams& params = _ukf.ukf().params();
        (void)map;
        MazeMap::ControlInput control{};
        control.leftMotorCommand = _leftMotor.getDriveCommand();
        control.rightMotorCommand = _rightMotor.getDriveCommand();
        control.fanDutyCycle = GetMissionFanDutyCycle();

        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            if (!_ukf.predict(dtSeconds, control))
            {
                AppendStartupTrace("pose_ukf_predict_rejected");
            }
        }

        MazeMap::EncoderObs encoderObservation{};
        encoderObservation.totalLeftCounts = _leftMotor.getEncoderCount();
        encoderObservation.totalRightCounts = _rightMotor.getEncoderCount();
        if ((params.wheelRadiusM > 0.0f) && std::isfinite(params.wheelRadiusM))
        {
            encoderObservation.omegaLeftRadps = _leftMotor.getEncoderVelocityMetersPerSecond() / params.wheelRadiusM;
            encoderObservation.omegaRightRadps = _rightMotor.getEncoderVelocityMetersPerSecond() / params.wheelRadiusM;
        }
        (void)_ukf.updateEncoderPair(encoderObservation, dtSeconds);

        if (std::isfinite(snapshot.gyroRadps))
        {
            (void)_ukf.ukf().updateYawRate(snapshot.gyroRadps);
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
    bool _targetVelocityInitialized = false;
    MazeMap::WheelControlProfile _wheelControlProfile;
    struct WheelLaunchAssistState
    {
        bool active = false;
        unsigned long startMs = 0UL;
        float requestedDirection = 0.0f;
    };
    WheelLaunchAssistState _leftLaunchAssist;
    WheelLaunchAssistState _rightLaunchAssist;

    float ModelDriveFeedforwardForTargetMotion(
        const MazeMap::MotorEncoderDrive& motor,
        float targetSpeedMps,
        float linearTargetAccelMps2,
        float angularTargetAccelRadps2,
        float effectiveTrackWidthM,
        bool isLeftWheel) const
    {
        // Use the shared vehicle mass/inertia plus the motor model so wheel feedforward covers both the commanded
        // wheel-speed back-EMF and the longitudinal/yaw force required to hit the requested v/omega acceleration.
        const MazeMap::VehiclePhysicalModel& physicalModel = MazeMap::Vehicle::GetPhysicalModel();
        float trackWidthM = effectiveTrackWidthM;
        if (!(trackWidthM > 0.0f) || !std::isfinite(trackWidthM))
        {
            trackWidthM = physicalModel.trackWidthM;
        }

        const float yawInertiaKgM2 = physicalModel.yawInertiaKgM2;
        const float sharedForceN = 0.5f * physicalModel.massKg * linearTargetAccelMps2;
        const float yawForceN = (trackWidthM > 0.0f) ? ((yawInertiaKgM2 * angularTargetAccelRadps2) / trackWidthM) : 0.0f;
        const float wheelForceN = isLeftWheel ? (sharedForceN - yawForceN) : (sharedForceN + yawForceN);
        return motor.getDriveCommandForGroundForce(wheelForceN, targetSpeedMps);
    }

    float VelocityCommandFromError(
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
        return (std::clamp)(command, -1.0f, 1.0f);
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



