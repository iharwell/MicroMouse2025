#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapRuntimeInfrastructure.h"
#pragma once
#include "MazeMapRuntimeDrive.h"
#include "MazeMapRuntimeSensors.h"
#include "MazeMapRuntimeCsvLog.h"

#include <stdio.h>

// Private application infrastructure helpers for the MazeMap runtime.

class DiagnosticLogger
{
public:
    DiagnosticLogger()
        : _log()
        , _phaseId(0UL)
        , _sampleCount(0UL)
    {
        _fileName[0] = '\0';
    }

    bool Begin(
        const DiagnosticSensorSuite& sensors,
        const char* fileName = nullptr,
        unsigned long controlPeriodUs = DiagnosticConfig::kControlPeriodUs,
        const char* modeName = nullptr)
    {
        if (!SelectFileName(fileName))
        {
            return false;
        }

        if (!WriteMetadata("file", _fileName))
        {
            return false;
        }
        if (modeName != nullptr && modeName[0] != '\0' && !WriteMetadata("mode", modeName))
        {
            return false;
        }
        if (!WriteMetadataUL("control_period_us", controlPeriodUs))
        {
            return false;
        }
        const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(controlPeriodUs);
        if (imuSampleRateHz > 0UL && !WriteMetadataUL("imu_sample_rate_hz", imuSampleRateHz))
        {
            return false;
        }
        const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(controlPeriodUs);
        if (imuAccelLpf2CutoffHz > 0.0f && !WriteMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3))
        {
            return false;
        }
        const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(controlPeriodUs);
        if (imuGyroLpf1ReferenceHz > 0.0f && !WriteMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6))
        {
            return false;
        }
        if (!WriteDiagnosticTuningMetadata())
        {
            return false;
        }
        if (!WriteSummaryInstructions())
        {
            return false;
        }

        return _log.Write(
            "sample,phase_id,t_us,dt_us,stationary,"
            "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,"
            "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
            "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
            "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
            "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
            "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
            "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
            "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps\n");
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;

        const bool ok = _log.WritePhase(_phaseId, micros(), name);
        FlushIfNeeded(true);
        return ok;
    }

    bool WriteEvent(const char* type, const char* message)
    {
        const bool ok = _log.WriteEvent(micros(), type, message);
        FlushIfNeeded(true);
        return ok;
    }

    bool LogSample(
        bool stationary,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot)
    {
        char line[2048] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%lu,%lu,%lu,%lu,%u,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.4f,%.4f,"
            "%ld,%ld,%.6f,%.6f,%.6f,%.6f,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            static_cast<unsigned long>(_sampleCount),
            static_cast<unsigned long>(_phaseId),
            static_cast<unsigned long>(timestampUs),
            static_cast<unsigned long>(dtUs),
            stationary ? 1U : 0U,
            pose.xMeters,
            pose.yMeters,
            pose.yawRad,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            drive.GetLastLinearCommandMps(),
            drive.GetLastAngularCommandRadps(),
            driveTelemetry.leftDriveCommand,
            driveTelemetry.rightDriveCommand,
            static_cast<long>(driveTelemetry.leftEncoderCount),
            static_cast<long>(driveTelemetry.rightEncoderCount),
            driveTelemetry.leftDistanceM,
            driveTelemetry.rightDistanceM,
            driveTelemetry.leftVelocityMps,
            driveTelemetry.rightVelocityMps,
            sensorSnapshot.imuFrontRight.status,
            sensorSnapshot.imuFrontRight.gyroX,
            sensorSnapshot.imuFrontRight.gyroY,
            sensorSnapshot.imuFrontRight.gyroZ,
            sensorSnapshot.imuFrontRight.accelX,
            sensorSnapshot.imuFrontRight.accelY,
            sensorSnapshot.imuFrontRight.accelZ,
            sensorSnapshot.imuFrontRight.temp,
            sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U,
            sensorSnapshot.imuBackLeft.status,
            sensorSnapshot.imuBackLeft.gyroX,
            sensorSnapshot.imuBackLeft.gyroY,
            sensorSnapshot.imuBackLeft.gyroZ,
            sensorSnapshot.imuBackLeft.accelX,
            sensorSnapshot.imuBackLeft.accelY,
            sensorSnapshot.imuBackLeft.accelZ,
            sensorSnapshot.imuBackLeft.temp,
            sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U,
            sensorSnapshot.frontLeft.ambientLight,
            sensorSnapshot.frontLeft.litLight,
            sensorSnapshot.frontLeft.differentialLight,
            sensorSnapshot.frontLeft.rawDistanceM,
            sensorSnapshot.frontLeft.distanceM,
            sensorSnapshot.frontRight.ambientLight,
            sensorSnapshot.frontRight.litLight,
            sensorSnapshot.frontRight.differentialLight,
            sensorSnapshot.frontRight.rawDistanceM,
            sensorSnapshot.frontRight.distanceM,
            sensorSnapshot.sideLeft.ambientLight,
            sensorSnapshot.sideLeft.litLight,
            sensorSnapshot.sideLeft.differentialLight,
            sensorSnapshot.sideLeft.rawDistanceM,
            sensorSnapshot.sideLeft.distanceM,
            sensorSnapshot.sideRight.ambientLight,
            sensorSnapshot.sideRight.litLight,
            sensorSnapshot.sideRight.differentialLight,
            sensorSnapshot.sideRight.rawDistanceM,
            sensorSnapshot.sideRight.distanceM,
            sensorSnapshot.frontWall ? 1U : 0U,
            sensorSnapshot.leftWall ? 1U : 0U,
            sensorSnapshot.rightWall ? 1U : 0U,
            sensorSnapshot.corridorErrorM,
            sensorSnapshot.frontSkewM,
            sensorSnapshot.gyroBiasRadps,
            sensorSnapshot.gyroRawRadps,
            sensorSnapshot.gyroRadps);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        if (!_log.Write(line))
        {
            return false;
        }

        ++_sampleCount;
        FlushIfNeeded(false);
        return true;
    }

    void Flush()
    {
        _log.Flush();
    }

    void Close()
    {
        _log.Close();
    }

    const char* GetFileName() const
    {
        return _fileName;
    }

    bool WriteMetadataUnsigned(const char* key, unsigned long value)
    {
        return WriteMetadataUL(key, value);
    }

    bool WriteMetadataValueFloat(const char* key, float value, uint8_t precision)
    {
        return WriteMetadataFloat(key, value, precision);
    }

private:
    MazeMapApp::Internal::Runtime::RuntimeCsvLogFile _log;
    char _fileName[24];
    unsigned long _phaseId;
    unsigned long _sampleCount;

    bool SelectFileName(const char* explicitFileName)
    {
        if (!_log.Begin(explicitFileName, "diag%03u.csv", "diagnostic_log.csv"))
        {
            return false;
        }

        snprintf(_fileName, sizeof(_fileName), "%s", _log.GetFileName());
        return true;
    }

    bool WriteMetadata(const char* key, const char* value)
    {
        return _log.WriteMetadata(key, value);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        return _log.WriteMetadataUnsigned(key, value);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        return _log.WriteMetadataFloat(key, value, precision);
    }

    bool WriteDiagnosticTuningMetadata()
    {
        const auto& driveModel = MazeMap::MotorEncoderDrive::GetSharedPhysicalModel();
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        auto writeUL = [this](const char* key, unsigned long value) -> bool
        {
            return WriteMetadataUL(key, value);
        };
        auto writeFloat = [this](const char* key, float value) -> bool
        {
            return WriteMetadataFloat(key, value, 6);
        };

        if (!writeUL("kGyroBiasSamples", static_cast<unsigned long>(Config::kGyroBiasSamples))) return false;
        if (!writeUL("kGyroBiasMinimumAveragingWindowMs", static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs))) return false;
        if (!writeFloat("kGyroBiasUpdateMaxAbsRateRadps", Config::kGyroBiasUpdateMaxAbsRateRadps)) return false;
        if (!writeFloat("kTrackWidthM", Config::kTrackWidthM)) return false;
        if (!writeFloat("kArcTrackWidthTightRadiusM", vehicleModel.arcTrackWidthInterpolation.tightRadiusM)) return false;
        if (!writeFloat("kArcTrackWidthTightM", vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM)) return false;
        if (!writeFloat("kArcTrackWidthWideRadiusM", vehicleModel.arcTrackWidthInterpolation.wideRadiusM)) return false;
        if (!writeFloat("kArcTrackWidthWideM", vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM)) return false;
        if (!writeFloat("kWheelDiameterM", driveModel.wheelDiameterM)) return false;
        if (!writeUL("kEncoderCountsPerRev", static_cast<unsigned long>(driveModel.pulsesPerRev))) return false;
        if (!writeFloat("kMotorToWheelGearRatio", driveModel.gearRatio)) return false;
        if (!writeFloat("kMotorNominalVoltageV", driveModel.nominalVoltageV)) return false;
        if (!writeFloat("kMotorNominalNoLoadSpeedRpm", driveModel.nominalNoLoadSpeedRpm)) return false;
        if (!writeFloat("kMotorSupplyVoltageV", driveModel.supplyVoltageV)) return false;
        if (!writeFloat("kMotorTerminalResistanceOhms", driveModel.resistanceOhms)) return false;
        if (!writeFloat("kMotorTorqueConstantNmPerA", driveModel.torqueConstantNmPerA)) return false;
        if (!writeFloat("kMotorSpeedConstantRadpsPerVolt", driveModel.speedConstantRadpsPerVolt)) return false;
        if (!writeFloat("kMotorNoLoadCurrentA", driveModel.noLoadCurrentA)) return false;
        if (!writeFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle)) return false;
        if (!writeUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        if (!writeFloat("kWheelStaticFeedforward", Config::kWheelStaticFeedforward)) return false;
        if (!writeFloat("kWheelRestLaunchDriveCommand", Config::kWheelRestLaunchDriveCommand)) return false;
        if (!writeFloat("kWheelRestLaunchMaxDriveCommand", Config::kWheelRestLaunchMaxDriveCommand)) return false;
        if (!writeUL("kWheelRestLaunchRampMs", Config::kWheelRestLaunchRampMs)) return false;
        if (!writeFloat("kWheelRestLaunchSpeedThresholdMps", Config::kWheelRestLaunchSpeedThresholdMps)) return false;
        if (!writeFloat("kWheelRestLaunchDriveThreshold", Config::kWheelRestLaunchDriveThreshold)) return false;
        if (!writeFloat("kWheelVelocityFeedforward", Config::kWheelVelocityFeedforward)) return false;
        if (!writeFloat("kWheelAccelerationResponseGainPerMps2", Config::kWheelAccelerationResponseGainPerMps2)) return false;
        if (!writeFloat("kWheelAccelerationResponseDeltaWindowMps", Config::kWheelAccelerationResponseDeltaWindowMps)) return false;
        if (!writeFloat("kMappingWheelAccelerationResponseScale", Config::kMappingWheelAccelerationResponseScale)) return false;
        if (!writeFloat("kWheelVelocityKp", Config::kWheelVelocityKp)) return false;
        if (!writeFloat("kWheelVelocityKi", Config::kWheelVelocityKi)) return false;
        if (!writeFloat("kWheelIntegralLimit", Config::kWheelIntegralLimit)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKpScale", DiagnosticConfig::kDiagnosticWheelVelocityKpScale)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKiScale", DiagnosticConfig::kDiagnosticWheelVelocityKiScale)) return false;
        if (!writeFloat("kDiagnosticWheelIntegralLimitScale", DiagnosticConfig::kDiagnosticWheelIntegralLimitScale)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKpEffective", MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, DiagnosticConfig::kDiagnosticWheelVelocityKpScale))) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKiEffective", MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKi, DiagnosticConfig::kDiagnosticWheelVelocityKiScale))) return false;
        if (!writeFloat("kDiagnosticWheelIntegralLimitEffective", MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, DiagnosticConfig::kDiagnosticWheelIntegralLimitScale))) return false;
        if (!writeFloat("kStraightHeadingKp", Config::kStraightHeadingKp)) return false;
        if (!writeFloat("kStraightYawD", Config::kStraightYawD)) return false;
        if (!writeFloat("kWallCenterGain", Config::kWallCenterGain)) return false;
        if (!writeFloat("kWallCenterD", Config::kWallCenterD)) return false;
        if (!writeFloat("kWallCenterDerivativeFilterTauSeconds", Config::kWallCenterDerivativeFilterTauSeconds)) return false;
        if (!writeFloat("kWallCenterMaxClosurePerCellM", Config::kWallCenterMaxClosurePerCellM)) return false;
        if (!writeFloat("kArcHeadingKp", Config::kArcHeadingKp)) return false;
        if (!writeFloat("kArcYawD", Config::kArcYawD)) return false;
        if (!writeFloat("kSmoothTurnYawRateKp", Config::kSmoothTurnYawRateKp)) return false;
        if (!writeFloat("kSmoothTurnYawRateKd", Config::kSmoothTurnYawRateKd)) return false;
        if (!writeFloat("kTurnHeadingKp", Config::kTurnHeadingKp)) return false;
        if (!writeFloat("kTurnYawD", Config::kTurnYawD)) return false;
        if (!writeFloat("kDistanceToleranceM", Config::kDistanceToleranceM)) return false;
        if (!writeFloat("kAngleToleranceRad", Config::kAngleToleranceRad)) return false;
        if (!writeFloat("kSpeedToleranceMps", Config::kSpeedToleranceMps)) return false;
        if (!writeFloat("kAngularSpeedToleranceRadps", Config::kAngularSpeedToleranceRadps)) return false;
        if (!writeFloat("kMappingAngleToleranceRad", Config::kMappingAngleToleranceRad)) return false;
        if (!writeFloat("kMappingAngularSpeedToleranceRadps", Config::kMappingAngularSpeedToleranceRadps)) return false;
        if (!writeFloat("kObservedDiagnosticMinimumSustainableSpeedMps", Config::kObservedDiagnosticMinimumSustainableSpeedMps)) return false;
        if (!writeFloat("kMinimumAllowedCruiseSpeedMps", Config::kMinimumAllowedCruiseSpeedMps)) return false;
        if (!writeFloat("kEncoderProgressEpsilonM", Config::kEncoderProgressEpsilonM)) return false;
        if (!writeFloat("kEncoderStallCommandThresholdMps", Config::kEncoderStallCommandThresholdMps)) return false;
        if (!writeUL("kEncoderStallTimeoutMs", Config::kEncoderStallTimeoutMs)) return false;
        if (!writeUL("kEncoderStallStartupGraceMs", Config::kEncoderStallStartupGraceMs)) return false;

        if (!writeUL("kModeSelectPinA", static_cast<unsigned long>(DiagnosticConfig::kModeSelectPinA))) return false;
        if (!writeUL("kModeSelectPinB", static_cast<unsigned long>(DiagnosticConfig::kModeSelectPinB))) return false;
        if (!writeUL("kControlPeriodUs", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!writeUL("kStartupSettleMs", static_cast<unsigned long>(DiagnosticConfig::kStartupSettleMs))) return false;
        if (!writeUL("kBaselineHoldMs", static_cast<unsigned long>(DiagnosticConfig::kBaselineHoldMs))) return false;
        if (!writeUL("kInterTestHoldMs", static_cast<unsigned long>(DiagnosticConfig::kInterTestHoldMs))) return false;
        if (!writeUL("kLogFlushPeriodMs", static_cast<unsigned long>(DiagnosticConfig::kLogFlushPeriodMs))) return false;
        if (!writeFloat("kBoundaryHalfSpanM", DiagnosticConfig::kBoundaryHalfSpanM)) return false;
        if (!writeFloat("kShortStraightDistanceM", DiagnosticConfig::kShortStraightDistanceM)) return false;
        if (!writeFloat("kLongStraightDistanceM", DiagnosticConfig::kLongStraightDistanceM)) return false;
        if (!writeFloat("kSquareLegDistanceM", DiagnosticConfig::kSquareLegDistanceM)) return false;
        if (!writeFloat("kArcHalfCircleDistanceM", DiagnosticConfig::kArcHalfCircleDistanceM)) return false;
        if (!writeFloat("kSlowStraightSpeedMps", DiagnosticConfig::kSlowStraightSpeedMps)) return false;
        if (!writeFloat("kCircleMediumSpeedMps", DiagnosticConfig::kCircleMediumSpeedMps)) return false;
        if (!writeFloat("kFastStraightSpeedMps", DiagnosticConfig::kFastStraightSpeedMps)) return false;
        if (!writeFloat("kStraightAccelMps2", DiagnosticConfig::kStraightAccelMps2)) return false;
        if (!writeFloat("kStraightDecelMps2", DiagnosticConfig::kStraightDecelMps2)) return false;
        static const MazeMap::Vehicle sharedVehicle{};
        const MazeMap::InPlaceTurnProfile inPlaceTurnProfile = BuildSharedInPlaceTurnProfile(sharedVehicle);
        if (!writeFloat("kInPlaceTurnMaxOmegaRadps", inPlaceTurnProfile.maxAngularSpeedRadps)) return false;
        if (!writeFloat("kInPlaceTurnAccelRadps2", inPlaceTurnProfile.angularAccelRadps2)) return false;
        if (!writeFloat("kKickoffSweepMinDriveCommand", DiagnosticConfig::kKickoffSweepMinDriveCommand)) return false;
        if (!writeFloat("kKickoffSweepMaxDriveCommand", DiagnosticConfig::kKickoffSweepMaxDriveCommand)) return false;
        if (!writeFloat("kKickoffSweepStepDriveCommand", DiagnosticConfig::kKickoffSweepStepDriveCommand)) return false;
        if (!writeUL("kKickoffSweepPulseMs", static_cast<unsigned long>(DiagnosticConfig::kKickoffSweepPulseMs))) return false;
        if (!writeFloat("kKickoffSweepMoveThresholdM", DiagnosticConfig::kKickoffSweepMoveThresholdM)) return false;
        if (!writeFloat("kKickoffSweepMoveThresholdMps", DiagnosticConfig::kKickoffSweepMoveThresholdMps)) return false;
        if (!writeFloat("kForwardSweepKickoffDriveCommand", DiagnosticConfig::kForwardSweepKickoffDriveCommand)) return false;
        if (!writeUL("kForwardSweepKickoffMs", static_cast<unsigned long>(DiagnosticConfig::kForwardSweepKickoffMs))) return false;
        if (!writeFloat("kForwardSweepMinDriveCommand", DiagnosticConfig::kForwardSweepMinDriveCommand)) return false;
        if (!writeFloat("kForwardSweepMaxDriveCommand", DiagnosticConfig::kForwardSweepMaxDriveCommand)) return false;
        if (!writeFloat("kForwardSweepStepDriveCommand", DiagnosticConfig::kForwardSweepStepDriveCommand)) return false;
        if (!writeUL("kForwardSweepHoldMs", static_cast<unsigned long>(DiagnosticConfig::kForwardSweepHoldMs))) return false;
        if (!writeFloat("kForwardSweepCarryThresholdMps", DiagnosticConfig::kForwardSweepCarryThresholdMps)) return false;
        if (!writeFloat("kForwardSweepCarryThresholdM", DiagnosticConfig::kForwardSweepCarryThresholdM)) return false;
        if (!writeFloat("kCharacterizationBoundaryReserveM", DiagnosticConfig::kCharacterizationBoundaryReserveM)) return false;
        if (!writeUL("kCharacterizationSettleMs", static_cast<unsigned long>(DiagnosticConfig::kCharacterizationSettleMs))) return false;
        if (!writeFloat("kCharacterizationRecoverySpeedMps", DiagnosticConfig::kCharacterizationRecoverySpeedMps)) return false;

        return true;
    }

    bool WriteSummaryInstructions()
    {
        for (size_t index = 0U; index < MazeMap::GetDiagnosticSummaryInstructionCount(); ++index)
        {
            if (!WriteEvent("summary", MazeMap::GetDiagnosticSummaryInstruction(index).message))
            {
                return false;
            }
        }
        return true;
    }

    void FlushIfNeeded(bool force)
    {
        _log.FlushIfNeeded(force, DiagnosticConfig::kLogFlushPeriodMs);
    }
};

inline bool IsPinPairStrapped(uint8_t pinA, uint8_t pinB)
{
#if defined(ARDUINO_TEENSY41)
    pinMode(pinA, OUTPUT);
    digitalWrite(pinA, LOW);
    pinMode(pinB, INPUT_PULLUP);
    delay(2);
    const bool forwardSense = (digitalRead(pinB) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, OUTPUT);
    digitalWrite(pinB, LOW);
    delay(2);
    const bool reverseSense = (digitalRead(pinA) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    return forwardSense && reverseSense;
#else
    (void)pinA;
    (void)pinB;
    return false;
#endif
}

inline bool IsPrimaryDiagnosticModeRequested()
{
    return IsPinPairStrapped(DiagnosticConfig::kModeSelectPinA, DiagnosticConfig::kModeSelectPinB);
}

inline bool IsManeuverTestModeRequested()
{
    return IsPinPairStrapped(29U, DiagnosticConfig::kModeSelectPinA);
}

inline bool IsAuxiliaryMeasurementModeRequested()
{
    return IsPinPairStrapped(AuxMeasurementConfig::kModeSelectPinA, AuxMeasurementConfig::kModeSelectPinB);
}

inline bool IsFrontWallCharacterizationModeRequested()
{
    return IsPinPairStrapped(
        FrontWallCharacterizationConfig::kModeSelectPinA,
        FrontWallCharacterizationConfig::kModeSelectPinB);
}

inline bool IsInterRunServiceJumperInstalled()
{
    return IsPinPairStrapped(Config::kInterRunServicePinA, Config::kInterRunServicePinB);
}

inline bool IsWallSensorLedCalibrationModeRequested()
{
    return IsPinPairStrapped(LedCalibrationConfig::kModeSelectPinA, LedCalibrationConfig::kModeSelectPinB);
}

inline bool ResetStartupTrace(const char* firstLine)
{
#if defined(ARDUINO_TEENSY41)
    if (firstLine == nullptr || firstLine[0] == '\0')
    {
        return false;
    }

    MazeMap::CoreFileExport file;
    if (!file.Open("startup_trace.txt"))
    {
        return false;
    }

    if (!file.Write(firstLine) || !file.WriteChar('\n'))
    {
        return false;
    }

    file.Flush();
    return true;
#else
    (void)firstLine;
    return false;
#endif
}

inline bool AppendStartupTrace(const char* line)
{
#if defined(ARDUINO_TEENSY41)
    if (line == nullptr || line[0] == '\0')
    {
        return false;
    }

    File file = SD.open("startup_trace.txt", FILE_WRITE);
    if (!file)
    {
        return false;
    }

    const bool ok = (file.print(line) > 0U) && (file.write('\n') == 1U);
    file.flush();
    file.close();
    return ok;
#else
    (void)line;
    return false;
#endif
}

inline bool WritePersistedFrontWallCharacterization(
    const MazeMap::FrontWallCharacterizationStorage& storage)
{
#if defined(ARDUINO_TEENSY41)
    if (!MazeMap::IsValidFrontWallCharacterizationStorage(storage))
    {
        return false;
    }

    if (FrontWallCharacterizationConfig::kStorageAddress < 0 ||
        (FrontWallCharacterizationConfig::kStorageAddress + static_cast<int>(sizeof(storage))) > EEPROM.length())
    {
        return false;
    }

    MazeMap::FrontWallCharacterizationStorage persisted = storage;
    EEPROM.put(FrontWallCharacterizationConfig::kStorageAddress, persisted);

    MazeMap::FrontWallCharacterizationStorage verify{};
    EEPROM.get(FrontWallCharacterizationConfig::kStorageAddress, verify);
    return std::memcmp(&persisted, &verify, sizeof(persisted)) == 0;
#else
    (void)storage;
    return false;
#endif
}

inline bool TryReadPersistedFrontWallCharacterization(
    MazeMap::FrontWallCharacterizationStorage& storage)
{
#if defined(ARDUINO_TEENSY41)
    storage = {};
    if (FrontWallCharacterizationConfig::kStorageAddress < 0 ||
        (FrontWallCharacterizationConfig::kStorageAddress + static_cast<int>(sizeof(storage))) > EEPROM.length())
    {
        return false;
    }

    EEPROM.get(FrontWallCharacterizationConfig::kStorageAddress, storage);
    return MazeMap::IsValidFrontWallCharacterizationStorage(storage);
#else
    storage = {};
    return false;
#endif
}

inline const char* AuxMeasurementRoutineName(AuxMeasurementConfig::Routine routine)
{
    switch (routine)
    {
    case AuxMeasurementConfig::Routine::FanStaticSurvey:
        return "fan_static_survey";
    case AuxMeasurementConfig::Routine::TurningTractionSweep:
        return "turning_traction_sweep";
    case AuxMeasurementConfig::Routine::CorridorRepeatabilitySweep:
        return "corridor_repeatability_sweep";
    case AuxMeasurementConfig::Routine::PositionAccuracyAudit:
        return "position_accuracy_audit";
    default:
        return "unknown";
    }
}

class AuxMeasurementLogger
{
public:
    AuxMeasurementLogger()
        : _log()
        , _phaseId(0UL)
        , _sampleCount(0UL)
    {
        _fileName[0] = '\0';
    }

    bool Begin(
        const DiagnosticSensorSuite& sensors,
        AuxMeasurementConfig::Routine routine,
        const char* fileName = nullptr)
    {
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        if (!_log.Begin(fileName, "aux%03u.csv", "aux_measurement_log.csv"))
        {
            return false;
        }

        snprintf(_fileName, sizeof(_fileName), "%s", _log.GetFileName());
        if (!WriteMetadata("file", _fileName)) return false;
        if (!WriteMetadata("mode", "aux_measurement")) return false;
        if (!WriteMetadata("routine", AuxMeasurementRoutineName(routine))) return false;
        if (!WriteMetadataUL("control_period_us", AuxMeasurementConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !WriteMetadataUL("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuAccelLpf2CutoffHz > 0.0f && !WriteMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !WriteMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!WriteMetadataUL("startup_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs))) return false;
        if (!WriteMetadataUL("log_flush_period_ms", static_cast<unsigned long>(AuxMeasurementConfig::kLogFlushPeriodMs))) return false;
        if (!WriteMetadataUL("mode_select_pin_a", static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinA))) return false;
        if (!WriteMetadataUL("mode_select_pin_b", static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinB))) return false;
        if (!WriteMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!WriteMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!WriteMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6)) return false;
        if (!WriteMetadataFloat("kTrackWidthM", Config::kTrackWidthM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthTightRadiusM", vehicleModel.arcTrackWidthInterpolation.tightRadiusM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthTightM", vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthWideRadiusM", vehicleModel.arcTrackWidthInterpolation.wideRadiusM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthWideM", vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM, 6)) return false;
        if (routine == AuxMeasurementConfig::Routine::FanStaticSurvey)
        {
            if (!WriteMetadataUL("baseline_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kBaselineHoldMs))) return false;
            if (!WriteMetadataUL("fan_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kFanHoldMs))) return false;
            if (!WriteMetadataUL("recovery_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kRecoveryHoldMs))) return false;
            if (!WriteMetadataFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle, 6)) return false;
            if (!WriteMetadataUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        }
        if (routine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteMetadata("turn_direction", AuxMeasurementConfig::kTurningTractionSweepClockwise ? "cw" : "ccw")) return false;
            if (!WriteMetadataFloat("turning_traction_radius_m", AuxMeasurementConfig::kTurningTractionSweepRadiusM, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_start_speed_mps", AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_accel_mps2", AuxMeasurementConfig::kTurningTractionSweepAccelMps2, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_max_speed_mps", AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps, 6)) return false;
            if (!WriteMetadataUL("turning_traction_fan_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepFanSettleMs))) return false;
            if (!WriteMetadataUL("turning_traction_launch_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionLaunchMs))) return false;
            if (!WriteMetadataFloat("turning_traction_max_omega_radps", AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_kp_scale", AuxMeasurementConfig::kTurningTractionWheelVelocityKpScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_ki_scale", AuxMeasurementConfig::kTurningTractionWheelVelocityKiScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_integral_limit_scale", AuxMeasurementConfig::kTurningTractionWheelIntegralLimitScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_plateau_min_speed_mps", AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_plateau_delta_mps", AuxMeasurementConfig::kTurningTractionPlateauDeltaMps, 6)) return false;
            if (!WriteMetadataUL("turning_traction_plateau_window_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionPlateauWindowMs))) return false;
            if (!WriteMetadataFloat("turning_traction_actuator_ceiling_cmd", AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_curvature_ramp_m_inv_per_s", AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_min_speed_mps", AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_min_lat_accel_mps2", AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_yaw_coherence_floor", AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_planar_coherence_floor", AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor, 6)) return false;
            if (!WriteMetadataUL("turning_traction_slip_confirm_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSlipConfirmMs))) return false;
            if (!WriteMetadataUL("turning_traction_timeout_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepTimeoutMs))) return false;
            if (!WriteMetadataFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle, 6)) return false;
            if (!WriteMetadataUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        }
        if (!WriteEvent("summary", "Enter by shorting pins 28 and 29 at boot. Those pins only select this mode; they are not measurement inputs.")) return false;
        if (!WriteEvent("summary", "Edit AuxMeasurementConfig::kRoutine and RunSelectedRoutine() to repurpose this mode for one-off internal measurements.")) return false;
        if (routine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteEvent("summary", "The default routine enables the mission fan, ramps circle speed without a software ceiling, and if speed plateaus before slip it tightens curvature until sustained encoder-vs-gyro/IMU mismatch indicates traction loss.")) return false;
            if (!WriteEvent("summary", "Use traction_limit_result and the last steady samples before it to estimate the maximum sustainable circle speed, yaw rate, and lateral acceleration.")) return false;
        }
        else
        {
            if (!WriteEvent("summary", "The default routine logs stationary fan-off, fan-on, and recovery phases so you can quantify fan-induced sensor and vibration shifts.")) return false;
        }

        return _log.Write(
            "sample,phase_id,t_us,dt_us,stationary,fan_enabled,"
            "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,planar_accel_mps2,"
            "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
            "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
            "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
            "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
            "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
            "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
            "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps\n");
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;

        const bool ok = _log.WritePhase(_phaseId, micros(), name);
        FlushIfNeeded(true);
        return ok;
    }

    bool WriteEvent(const char* type, const char* message)
    {
        const bool ok = _log.WriteEvent(micros(), type, message);
        FlushIfNeeded(true);
        return ok;
    }

    bool LogSample(
        bool stationary,
        bool fanEnabled,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot,
        float planarAccelMps2)
    {
        char line[2048] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%lu,%lu,%lu,%lu,%u,%u,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.4f,%.4f,"
            "%ld,%ld,%.6f,%.6f,%.6f,%.6f,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            _sampleCount,
            _phaseId,
            static_cast<unsigned long>(timestampUs),
            static_cast<unsigned long>(dtUs),
            stationary ? 1U : 0U,
            fanEnabled ? 1U : 0U,
            pose.xMeters,
            pose.yMeters,
            pose.yawRad,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            planarAccelMps2,
            drive.GetLastLinearCommandMps(),
            drive.GetLastAngularCommandRadps(),
            driveTelemetry.leftDriveCommand,
            driveTelemetry.rightDriveCommand,
            static_cast<long>(driveTelemetry.leftEncoderCount),
            static_cast<long>(driveTelemetry.rightEncoderCount),
            driveTelemetry.leftDistanceM,
            driveTelemetry.rightDistanceM,
            driveTelemetry.leftVelocityMps,
            driveTelemetry.rightVelocityMps,
            sensorSnapshot.imuFrontRight.status,
            sensorSnapshot.imuFrontRight.gyroX,
            sensorSnapshot.imuFrontRight.gyroY,
            sensorSnapshot.imuFrontRight.gyroZ,
            sensorSnapshot.imuFrontRight.accelX,
            sensorSnapshot.imuFrontRight.accelY,
            sensorSnapshot.imuFrontRight.accelZ,
            sensorSnapshot.imuFrontRight.temp,
            sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U,
            sensorSnapshot.imuBackLeft.status,
            sensorSnapshot.imuBackLeft.gyroX,
            sensorSnapshot.imuBackLeft.gyroY,
            sensorSnapshot.imuBackLeft.gyroZ,
            sensorSnapshot.imuBackLeft.accelX,
            sensorSnapshot.imuBackLeft.accelY,
            sensorSnapshot.imuBackLeft.accelZ,
            sensorSnapshot.imuBackLeft.temp,
            sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U,
            sensorSnapshot.frontLeft.ambientLight,
            sensorSnapshot.frontLeft.litLight,
            sensorSnapshot.frontLeft.differentialLight,
            sensorSnapshot.frontLeft.rawDistanceM,
            sensorSnapshot.frontLeft.distanceM,
            sensorSnapshot.frontRight.ambientLight,
            sensorSnapshot.frontRight.litLight,
            sensorSnapshot.frontRight.differentialLight,
            sensorSnapshot.frontRight.rawDistanceM,
            sensorSnapshot.frontRight.distanceM,
            sensorSnapshot.sideLeft.ambientLight,
            sensorSnapshot.sideLeft.litLight,
            sensorSnapshot.sideLeft.differentialLight,
            sensorSnapshot.sideLeft.rawDistanceM,
            sensorSnapshot.sideLeft.distanceM,
            sensorSnapshot.sideRight.ambientLight,
            sensorSnapshot.sideRight.litLight,
            sensorSnapshot.sideRight.differentialLight,
            sensorSnapshot.sideRight.rawDistanceM,
            sensorSnapshot.sideRight.distanceM,
            sensorSnapshot.frontWall ? 1U : 0U,
            sensorSnapshot.leftWall ? 1U : 0U,
            sensorSnapshot.rightWall ? 1U : 0U,
            sensorSnapshot.corridorErrorM,
            sensorSnapshot.frontSkewM,
            sensorSnapshot.gyroBiasRadps,
            sensorSnapshot.gyroRawRadps,
            sensorSnapshot.gyroRadps);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        if (!_log.Write(line))
        {
            return false;
        }

        ++_sampleCount;
        FlushIfNeeded(false);
        return true;
    }

    void Flush()
    {
        _log.Flush();
    }

    void Close()
    {
        _log.Close();
    }

    const char* GetFileName() const
    {
        return _fileName;
    }

private:
    MazeMapApp::Internal::Runtime::RuntimeCsvLogFile _log;
    char _fileName[24];
    unsigned long _phaseId;
    unsigned long _sampleCount;

    bool WriteMetadata(const char* key, const char* value)
    {
        return _log.WriteMetadata(key, value);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        return _log.WriteMetadataUnsigned(key, value);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        return _log.WriteMetadataFloat(key, value, precision);
    }

    void FlushIfNeeded(bool force)
    {
        _log.FlushIfNeeded(force, AuxMeasurementConfig::kLogFlushPeriodMs);
    }
};



