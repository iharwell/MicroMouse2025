#pragma once
#include "OpenFloorMeasurementSpec.h"
#include "MazeMapRuntimeDrive.h"
#include "MazeMapRuntimeSensors.h"
#include "MazeMapRuntimeCsvLog.h"
#include "MazeMapRuntimeMmLog.h"

#include <initializer_list>
#include <array>
#include <cstring>
#include <limits>
#include <stdio.h>

// Private application infrastructure helpers for the MazeMap runtime.

namespace MazeMapApp::Internal::Runtime
{
    static constexpr uint32_t kRuntimeBinaryLogFlags = mmlog::FLAG_HAS_METADATA | mmlog::FLAG_HAS_NOTES;

    inline bool AppendRuntimeBinaryNotes(RuntimeTextBlockBuilder<512U>& notes, const char* eventFileName)
    {
        if (!notes.AppendLine("record_words=32bit_little_endian"))
        {
            return false;
        }
        if (!notes.AppendLine("text_fields=TAG4_for_<=4_chars_else_fnv1a32"))
        {
            return false;
        }
        if (eventFileName != nullptr && eventFileName[0] != '\0')
        {
            return notes.AppendKeyValue("event_sidecar", eventFileName);
        }
        return true;
    }

    class OptionalRuntimeEventLog
    {
    public:
        OptionalRuntimeEventLog() noexcept
            : _log()
            , _enabled(false)
        {
            _fileName[0] = '\0';
        }

        bool BeginSibling(const char* dataFileName)
        {
            Close();
            if (!BuildSiblingRuntimeFileName(_fileName, sizeof(_fileName), dataFileName, ".events.txt"))
            {
                return false;
            }

            _enabled = _log.Begin(_fileName, _fileName, _fileName);
            if (!_enabled)
            {
                _fileName[0] = '\0';
            }
            return true;
        }

        bool WriteMetadata(const char* key, const char* value)
        {
            return !_enabled || _log.WriteMetadata(key, value);
        }

        bool WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name)
        {
            return !_enabled || _log.WritePhase(phaseId, timestampUs, name);
        }

        bool WriteEvent(unsigned long timestampUs, const char* type, const char* message)
        {
            return !_enabled || _log.WriteEvent(timestampUs, type, message);
        }

        void Flush()
        {
            if (_enabled)
            {
                _log.Flush();
            }
        }

        void Close()
        {
            if (_enabled)
            {
                _log.Flush();
                _log.Close();
            }
            _enabled = false;
            _fileName[0] = '\0';
        }

        bool IsEnabled() const noexcept
        {
            return _enabled;
        }

        const char* GetFileName() const noexcept
        {
            return _enabled ? _fileName : "";
        }

    private:
        RuntimeCsvLogFile _log;
        bool _enabled;
        char _fileName[64];
    };

    template <std::size_t N>
    inline bool AppendBinaryRecord(RuntimeBinaryLogFile& log, const RuntimeRecordBuilder<N>& record)
    {
        return record.IsFull() && log.AppendRecord(record.Data(), record.Count());
    }

    template <std::size_t N>
    inline void AppendDriveTelemetryFields(RuntimeRecordBuilder<N>& record, const DriveTelemetry& telemetry)
    {
        record.I32(telemetry.leftEncoderCount);
        record.I32(telemetry.rightEncoderCount);
        record.F32(telemetry.leftDistanceM);
        record.F32(telemetry.rightDistanceM);
        record.F32(telemetry.leftVelocityMps);
        record.F32(telemetry.rightVelocityMps);
    }

    template <std::size_t N>
    inline void AppendImuTelemetryFields(RuntimeRecordBuilder<N>& record, const ImuTelemetry& imu)
    {
        record.U32(imu.status);
        record.I32(imu.gyroX);
        record.I32(imu.gyroY);
        record.I32(imu.gyroZ);
        record.I32(imu.accelX);
        record.I32(imu.accelY);
        record.I32(imu.accelZ);
        record.I32(imu.temp);
        record.U32(imu.interruptHigh ? 1U : 0U);
    }

    template <std::size_t N>
    inline void AppendWallSensorFields(RuntimeRecordBuilder<N>& record, const WallSensorTelemetry& sensor)
    {
        record.F32(sensor.ambientLight);
        record.F32(sensor.litLight);
        record.F32(sensor.differentialLight);
        record.F32(sensor.rawDistanceM);
        record.F32(sensor.distanceM);
    }

    template <std::size_t N>
    inline void AppendControlTimingFields(RuntimeRecordBuilder<N>& record, const ControlCycleTiming& timing)
    {
        record.U32(timing.controlStartUs);
        record.U32(timing.controlEndUs);
        record.U32(timing.pwmLatchUs);
        record.U32(timing.encoderLatchUs);
        record.U32(timing.encoderReadDoneUs);
        record.U32(timing.ukfPredictStartUs);
        record.U32(timing.ukfPredictEndUs);
        record.U32(timing.ukfPredictDurationUs);
        record.U32(timing.ukfUpdateStartUs);
        record.U32(timing.ukfUpdateEndUs);
        record.U32(timing.ukfUpdateDurationUs);
        record.U32(timing.cycleCounterStart);
        record.U32(timing.cycleCounterEnd);
    }

    template <std::size_t N>
    inline void AppendImuTimingFields(RuntimeRecordBuilder<N>& record, const ImuObservationTiming& timing)
    {
        record.U32(timing.drdyUs);
        record.U32(timing.readStartUs);
        record.U32(timing.readDoneUs);
    }

    template <std::size_t N>
    inline void AppendOpticalTimingFields(RuntimeRecordBuilder<N>& record, const OpticalObservationTiming& timing)
    {
        record.U32(timing.ledOnCommandUs);
        record.U32(timing.adcOnSampleUs);
        record.U32(timing.ledOffCommandUs);
        record.U32(timing.adcOffSampleUs);
        record.U32(timing.observationReadyUs);
    }
}

class DiagnosticLogger
{
public:
    DiagnosticLogger()
        : _sampleLog()
        , _eventLog()
        , _metadata()
        , _notes()
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
        _phaseId = 0UL;
        _sampleCount = 0UL;
        _metadata.Clear();
        _notes.Clear();

        if (!SelectFileName(fileName))
        {
            return false;
        }
        if (!_eventLog.BeginSibling(_fileName))
        {
            return false;
        }

        if (!WriteMetadata("file", _fileName))
        {
            return false;
        }
        if (_eventLog.IsEnabled() && !WriteMetadata("events_file", _eventLog.GetFileName()))
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
        const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
            controlPeriodUs,
            Config::kMissionRuntimeAccelFilterFreq);
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
        if (!WriteMetadataUL("mission_accel_bias_valid", sensors.HasAccelBias() ? 1UL : 0UL))
        {
            return false;
        }
        if (sensors.HasAccelBias())
        {
            if (!WriteMetadataFloat("mission_accel_bias_x_mg", sensors.GetAccelBiasXG() * 1000.0f, 3) ||
                !WriteMetadataFloat("mission_accel_bias_y_mg", sensors.GetAccelBiasYG() * 1000.0f, 3))
            {
                return false;
            }
        }
        if (!WriteDiagnosticTuningMetadata())
        {
            return false;
        }
        if (!MazeMapApp::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName()))
        {
            return false;
        }

        if (!_sampleLog.BeginSelected(
                _fileName,
                kDiagnosticSchema,
                kDiagnosticFieldCount,
                _metadata.Data(),
                _notes.Data(),
                MazeMapApp::Internal::Runtime::kRuntimeBinaryLogFlags,
                0U,
                micros()))
        {
            _eventLog.Close();
            return false;
        }

        if (_eventLog.IsEnabled())
        {
            (void)_eventLog.WriteMetadata("file", _eventLog.GetFileName());
            (void)_eventLog.WriteMetadata("data_file", _fileName);
            (void)_eventLog.WriteMetadata("mode", (modeName != nullptr && modeName[0] != '\0') ? modeName : "diagnostic");
        }
        return WriteSummaryInstructions();
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;
        return _eventLog.WritePhase(_phaseId, micros(), name);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        return _eventLog.WriteEvent(micros(), type, message);
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
        MazeMapApp::Internal::Runtime::RuntimeRecordBuilder<kDiagnosticFieldCount> record;
        record.U32(static_cast<uint32_t>(_sampleCount));
        record.U32(static_cast<uint32_t>(_phaseId));
        record.U32(timestampUs);
        record.U32(dtUs);
        record.U32(stationary ? 1U : 0U);
        record.F32(pose.xMeters);
        record.F32(pose.yMeters);
        record.F32(pose.yawRad);
        record.F32(pose.linearSpeedMps);
        record.F32(pose.angularSpeedRadps);
        record.F32(drive.GetLastLinearCommandMps());
        record.F32(drive.GetLastAngularCommandRadps());
        record.F32(driveTelemetry.leftDriveCommand);
        record.F32(driveTelemetry.rightDriveCommand);
        MazeMapApp::Internal::Runtime::AppendDriveTelemetryFields(record, driveTelemetry);
        MazeMapApp::Internal::Runtime::AppendImuTelemetryFields(record, sensorSnapshot.imuFrontRight);
        MazeMapApp::Internal::Runtime::AppendImuTelemetryFields(record, sensorSnapshot.imuBackLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.frontLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.frontRight);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.sideLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.sideRight);
        record.U32(sensorSnapshot.frontWall ? 1U : 0U);
        record.U32(sensorSnapshot.leftWall ? 1U : 0U);
        record.U32(sensorSnapshot.rightWall ? 1U : 0U);
        record.F32(sensorSnapshot.corridorErrorM);
        record.F32(sensorSnapshot.frontSkewM);
        record.F32(sensorSnapshot.gyroBiasRadps);
        record.F32(sensorSnapshot.gyroRawRadps);
        record.F32(sensorSnapshot.gyroRadps);

        if (!MazeMapApp::Internal::Runtime::AppendBinaryRecord(_sampleLog, record))
        {
            return false;
        }

        ++_sampleCount;
        return true;
    }

    void Service()
    {
        (void)_sampleLog.Service(1U);
    }

    void Flush()
    {
        _sampleLog.Flush();
        _eventLog.Flush();
    }

    void Close()
    {
        _sampleLog.Close();
        _eventLog.Close();
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
    static constexpr uint32_t kDiagnosticFieldCount = 66U;
    static constexpr const char* kDiagnosticSchema =
        "sample,phase_id,t_us,dt_us,stationary,"
        "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,"
        "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
        "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
        "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
        "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
        "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
        "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
        "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps";

    MazeMapApp::Internal::Runtime::RuntimeBinaryLogFile _sampleLog;
    MazeMapApp::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<12288U> _metadata;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<512U> _notes;
    char _fileName[64];
    unsigned long _phaseId;
    unsigned long _sampleCount;

    bool SelectFileName(const char* explicitFileName)
    {
        return MazeMapApp::Internal::Runtime::SelectSequentialRuntimeFileName(
            _fileName,
            sizeof(_fileName),
            explicitFileName,
            "diag%03u.mmlog",
            "diagnostic_log.mmlog");
    }

    bool WriteMetadata(const char* key, const char* value)
    {
        return _metadata.AppendKeyValue(key, value);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        return _metadata.AppendUnsigned(key, value);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        return _metadata.AppendFloat(key, value, precision);
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
};

struct OpenFloorMeasurementLabels
{
    MazeMap::OpenFloorSectionId sectionId = MazeMap::OpenFloorSectionId::Sec00Timing;
    MazeMap::OpenFloorMarkerId startMarkerId = MazeMap::OpenFloorMarkerId::C;
    const char* primitiveId = "NONE";
    const char* direction = "NONE";
    MazeMap::OpenFloorPhaseId phaseId = MazeMap::OpenFloorPhaseId::Idle;
    MazeMap::OpenFloorSpeedBin speedBin = MazeMap::OpenFloorSpeedBin::None;
    uint16_t repeatIndex = 0U;
    float progressNorm = 0.0f;
    bool abortMarker = false;
};

struct OpenFloorMeasurementCycle
{
    uint32_t masterTimeUs = 0UL;
    uint32_t dtUs = 0UL;
    ControlCycleTiming controlTiming{};
    DriveTelemetry driveTelemetry{};
    DiagnosticSensorSnapshot sensorSnapshot{};
    float measuredLinearSpeedMps = 0.0f;
    float measuredAngularSpeedRadps = 0.0f;
    float batteryVoltage = std::numeric_limits<float>::quiet_NaN();
    float boardTemperatureC = std::numeric_limits<float>::quiet_NaN();
    float fanDutyCycle = 0.0f;
    uint32_t errorFlags = 0UL;
};

class OpenFloorTimingLogger
{
public:
    bool Begin(const char* runId)
    {
        _metadata.Clear();
        _notes.Clear();
        if (!_eventLog.BeginSibling("timing_boot.mmlog"))
        {
            return false;
        }
        if (!_metadata.AppendKeyValue("file", "timing_boot.mmlog")) return false;
        if (_eventLog.IsEnabled() && !_metadata.AppendKeyValue("events_file", _eventLog.GetFileName())) return false;
        if (!_metadata.AppendKeyValue("mode", "open_floor_measurement")) return false;
        if (runId != nullptr && runId[0] != '\0' && !_metadata.AppendKeyValue("run_id", runId)) return false;
        if (!MazeMapApp::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName())) return false;
        if (!_sampleLog.BeginSelected(
                "timing_boot.mmlog",
                kTimingSchema,
                kTimingFieldCount,
                _metadata.Data(),
                _notes.Data(),
                MazeMapApp::Internal::Runtime::kRuntimeBinaryLogFlags,
                0U,
                micros()))
        {
            return false;
        }
        return true;
    }

    bool LogSample(
        uint16_t controlTickIndex,
        const ControlCycleTiming& controlTiming,
        const DiagnosticSensorSnapshot& snapshot)
    {
        return LogRow(
                   "sample",
                   "control",
                   controlTickIndex,
                   controlTiming.controlStartUs,
                   controlTiming,
                   ImuObservationTiming{},
                   OpticalObservationTiming{},
                   "control") &&
            LogRow(
                   "sample",
                   "encoder",
                   controlTickIndex,
                   controlTiming.encoderReadDoneUs,
                   controlTiming,
                   ImuObservationTiming{},
                   OpticalObservationTiming{},
                   "encoder") &&
            LogRow(
                   "sample",
                   "imu",
                   controlTickIndex,
                   snapshot.imuTiming.readDoneUs,
                   controlTiming,
                   snapshot.imuTiming,
                   OpticalObservationTiming{},
                   "imu") &&
            LogRow(
                   "sample",
                   "front_pair_stream",
                   controlTickIndex,
                   snapshot.frontTiming.observationReadyUs,
                   controlTiming,
                   ImuObservationTiming{},
                   snapshot.frontTiming,
                   "front") &&
            LogRow(
                   "sample",
                   "left_side_stream",
                   controlTickIndex,
                   snapshot.leftTiming.observationReadyUs,
                   controlTiming,
                   ImuObservationTiming{},
                   snapshot.leftTiming,
                   "left") &&
            LogRow(
                   "sample",
                   "right_side_stream",
                   controlTickIndex,
                   snapshot.rightTiming.observationReadyUs,
                   controlTiming,
                   ImuObservationTiming{},
                   snapshot.rightTiming,
                   "right");
    }

    bool LogSummary(const char* streamId, unsigned long sampleCount, float meanDelayUs, float jitterUs)
    {
        char message[160] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;n=%lu;mean_delay_us=%.3f;jitter_us=%.3f",
            (streamId != nullptr) ? streamId : "stream",
            sampleCount,
            meanDelayUs,
            jitterUs);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }
        return _eventLog.WriteEvent(micros(), "summary", message);
    }

    bool LogFailure(const char* reason)
    {
        return _eventLog.WriteEvent(micros(), "fault", (reason != nullptr) ? reason : "timing_failure");
    }

    void Service()
    {
        (void)_sampleLog.Service(1U);
    }

    void Flush()
    {
        _sampleLog.Flush();
        _eventLog.Flush();
    }

    void Close()
    {
        _sampleLog.Close();
        _eventLog.Close();
    }

private:
    static constexpr uint32_t kTimingFieldCount = 27U;
    static constexpr const char* kTimingSchema =
        "mono_time,timing_section_id,event_type,stream_id,control_tick_index,"
        "t_control_start,t_control_end,t_pwm_latch,t_enc_latch,t_enc_read_done,"
        "t_ukf_predict_start,t_ukf_predict_end,dt_ukf_predict,t_ukf_update_start,t_ukf_update_end,dt_ukf_update,"
        "t_imu_drdy,t_imu_read_start,t_imu_read_done,led_mode,"
        "led_on_cmd_time,adc_on_sample_time,led_off_cmd_time,adc_off_sample_time,obs_ready_time,"
        "cycle_counter_start,cycle_counter_end";
    MazeMapApp::Internal::Runtime::RuntimeBinaryLogFile _sampleLog;
    MazeMapApp::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<1024U> _metadata;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<512U> _notes;

    bool LogRow(
        const char* eventType,
        const char* streamId,
        uint16_t controlTickIndex,
        uint32_t monoTimeUs,
        const ControlCycleTiming& controlTiming,
        const ImuObservationTiming& imuTiming,
        const OpticalObservationTiming& opticalTiming,
        const char* ledMode)
    {
        MazeMapApp::Internal::Runtime::RuntimeRecordBuilder<kTimingFieldCount> record;
        record.U32(monoTimeUs);
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorSectionName(MazeMap::OpenFloorSectionId::Sec00Timing)));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(eventType));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(streamId));
        record.U32(controlTickIndex);
        record.U32(controlTiming.controlStartUs);
        record.U32(controlTiming.controlEndUs);
        record.U32(controlTiming.pwmLatchUs);
        record.U32(controlTiming.encoderLatchUs);
        record.U32(controlTiming.encoderReadDoneUs);
        record.U32(controlTiming.ukfPredictStartUs);
        record.U32(controlTiming.ukfPredictEndUs);
        record.U32(controlTiming.ukfPredictDurationUs);
        record.U32(controlTiming.ukfUpdateStartUs);
        record.U32(controlTiming.ukfUpdateEndUs);
        record.U32(controlTiming.ukfUpdateDurationUs);
        MazeMapApp::Internal::Runtime::AppendImuTimingFields(record, imuTiming);
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(ledMode));
        MazeMapApp::Internal::Runtime::AppendOpticalTimingFields(record, opticalTiming);
        record.U32(controlTiming.cycleCounterStart);
        record.U32(controlTiming.cycleCounterEnd);
        if (!MazeMapApp::Internal::Runtime::AppendBinaryRecord(_sampleLog, record))
        {
            return false;
        }
        return true;
    }
};

class OpenFloorMainLogger
{
public:
    OpenFloorMainLogger()
        : _sampleCount(0UL)
    {
    }

    bool Begin(const DiagnosticSensorSuite& sensors, const char* runId)
    {
        _sampleCount = 0UL;
        _metadata.Clear();
        _notes.Clear();
        if (!_eventLog.BeginSibling("open_floor_main.mmlog")) return false;
        if (!_metadata.AppendKeyValue("file", "open_floor_main.mmlog")) return false;
        if (_eventLog.IsEnabled() && !_metadata.AppendKeyValue("events_file", _eventLog.GetFileName())) return false;
        if (!_metadata.AppendKeyValue("mode", "open_floor_measurement")) return false;
        if (runId != nullptr && runId[0] != '\0' && !_metadata.AppendKeyValue("run_id", runId)) return false;
        if (!_metadata.AppendUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!_metadata.AppendFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_metadata.AppendFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!MazeMapApp::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName())) return false;
        return _sampleLog.BeginSelected(
            "open_floor_main.mmlog",
            kOpenFloorMainSchema,
            kOpenFloorMainFieldCount,
            _metadata.Data(),
            _notes.Data(),
            MazeMapApp::Internal::Runtime::kRuntimeBinaryLogFlags,
            0U,
            micros());
    }

    bool BeginSection(const OpenFloorMeasurementLabels& labels)
    {
        return WriteSectionMarker("section_start", labels, nullptr);
    }

    bool EndSection(const OpenFloorMeasurementLabels& labels)
    {
        return WriteSectionMarker("section_end", labels, nullptr);
    }

    bool AbortSection(const OpenFloorMeasurementLabels& labels, const char* reason)
    {
        return WriteSectionMarker("abort", labels, reason);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        return _eventLog.WriteEvent(micros(), type, message);
    }

    bool LogSample(
        const char* runId,
        const OpenFloorMeasurementLabels& labels,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const OpenFloorMeasurementCycle& cycle)
    {
        MazeMapApp::Internal::Runtime::RuntimeRecordBuilder<kOpenFloorMainFieldCount> record;
        record.U32(cycle.masterTimeUs);
        record.U32(cycle.dtUs);
        record.U32(cycle.controlTiming.controlStartUs);
        record.U32(cycle.controlTiming.controlEndUs);
        record.U32(cycle.controlTiming.pwmLatchUs);
        record.U32(cycle.controlTiming.encoderLatchUs);
        record.U32(cycle.controlTiming.encoderReadDoneUs);
        record.U32(cycle.controlTiming.ukfPredictStartUs);
        record.U32(cycle.controlTiming.ukfPredictEndUs);
        record.U32(cycle.controlTiming.ukfPredictDurationUs);
        record.U32(cycle.controlTiming.ukfUpdateStartUs);
        record.U32(cycle.controlTiming.ukfUpdateEndUs);
        record.U32(cycle.controlTiming.ukfUpdateDurationUs);
        MazeMapApp::Internal::Runtime::AppendImuTimingFields(record, cycle.sensorSnapshot.imuTiming);
        MazeMapApp::Internal::Runtime::AppendOpticalTimingFields(record, cycle.sensorSnapshot.frontTiming);
        MazeMapApp::Internal::Runtime::AppendOpticalTimingFields(record, cycle.sensorSnapshot.leftTiming);
        MazeMapApp::Internal::Runtime::AppendOpticalTimingFields(record, cycle.sensorSnapshot.rightTiming);
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(runId));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorSectionName(labels.sectionId)));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash((labels.primitiveId != nullptr) ? labels.primitiveId : "NONE"));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorDirectionLabel(labels.direction)));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorPhaseName(labels.phaseId)));
        record.U32(labels.repeatIndex);
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorMarkerName(labels.startMarkerId)));
        record.U32(MazeMapApp::Internal::Runtime::PackTextTagOrHash(MazeMap::OpenFloorSpeedBinName(labels.speedBin)));
        record.F32(labels.progressNorm);
        record.F32(pose.xMeters);
        record.F32(pose.yMeters);
        record.F32(MazeMap::OpenFloorMetersToHalfSteps(pose.xMeters));
        record.F32(MazeMap::OpenFloorMetersToHalfSteps(pose.yMeters));
        record.F32(pose.yawRad);
        record.F32(cycle.measuredLinearSpeedMps);
        record.F32(cycle.measuredAngularSpeedRadps);
        record.F32(drive.GetLastLinearCommandMps());
        record.F32(drive.GetLastAngularCommandRadps());
        record.F32(cycle.driveTelemetry.leftDriveCommand);
        record.F32(cycle.driveTelemetry.rightDriveCommand);
        MazeMapApp::Internal::Runtime::AppendDriveTelemetryFields(record, cycle.driveTelemetry);
        record.U32(cycle.sensorSnapshot.imuBackLeft.status);
        record.I32(cycle.sensorSnapshot.imuBackLeft.gyroX);
        record.I32(cycle.sensorSnapshot.imuBackLeft.gyroY);
        record.I32(cycle.sensorSnapshot.imuBackLeft.gyroZ);
        record.I32(cycle.sensorSnapshot.imuBackLeft.accelX);
        record.I32(cycle.sensorSnapshot.imuBackLeft.accelY);
        record.I32(cycle.sensorSnapshot.imuBackLeft.accelZ);
        record.I32(cycle.sensorSnapshot.imuBackLeft.temp);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, cycle.sensorSnapshot.frontLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, cycle.sensorSnapshot.frontRight);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, cycle.sensorSnapshot.sideLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, cycle.sensorSnapshot.sideRight);
        record.U32(cycle.sensorSnapshot.frontWall ? 1U : 0U);
        record.U32(cycle.sensorSnapshot.leftWall ? 1U : 0U);
        record.U32(cycle.sensorSnapshot.rightWall ? 1U : 0U);
        record.F32(cycle.sensorSnapshot.corridorErrorM);
        record.F32(cycle.sensorSnapshot.frontSkewM);
        record.F32(cycle.batteryVoltage);
        record.F32(cycle.boardTemperatureC);
        record.F32(cycle.fanDutyCycle);
        record.U32(cycle.errorFlags);
        if (!MazeMapApp::Internal::Runtime::AppendBinaryRecord(_sampleLog, record))
        {
            return false;
        }
        ++_sampleCount;
        return true;
    }

    void Service()
    {
        (void)_sampleLog.Service(1U);
    }

    void Flush()
    {
        _sampleLog.Flush();
        _eventLog.Flush();
    }

    void Close()
    {
        _sampleLog.Close();
        _eventLog.Close();
    }

private:
    static constexpr uint32_t kOpenFloorMainFieldCount = 94U;
    static constexpr const char* kOpenFloorMainSchema =
        "t_master,dt_control,t_control_start,t_control_end,t_pwm_latch,t_enc_latch,t_enc_read_done,"
        "t_ukf_predict_start,t_ukf_predict_end,dt_ukf_predict,t_ukf_update_start,t_ukf_update_end,dt_ukf_update,"
        "t_imu_drdy,t_imu_read_start,t_imu_read_done,"
        "t_front_led_on,t_front_adc_on,t_front_led_off,t_front_adc_off,t_front_obs_ready,"
        "t_left_led_on,t_left_adc_on,t_left_led_off,t_left_adc_off,t_left_obs_ready,"
        "t_right_led_on,t_right_adc_on,t_right_led_off,t_right_adc_off,t_right_obs_ready,"
        "run_id,section_id,primitive_id,direction,phase,repeat_index,start_marker_id,speed_bin,progress_norm,"
        "origin_x_m,origin_y_m,origin_x_half_steps,origin_y_half_steps,yaw_rad,linear_speed_mps,angular_speed_radps,"
        "cmd_linear_mps,cmd_angular_radps,u_left_cmd,u_right_cmd,"
        "encoder_count_left,encoder_count_right,encoder_dist_left_m,encoder_dist_right_m,encoder_vel_left_mps,encoder_vel_right_mps,"
        "imu_status,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_accel_x,imu_accel_y,imu_accel_z,imu_temp,"
        "front_left_ambient,front_left_lit,front_left_delta,front_left_raw_distance_m,front_left_distance_m,"
        "front_right_ambient,front_right_lit,front_right_delta,front_right_raw_distance_m,front_right_distance_m,"
        "side_left_ambient,side_left_lit,side_left_delta,side_left_raw_distance_m,side_left_distance_m,"
        "side_right_ambient,side_right_lit,side_right_delta,side_right_raw_distance_m,side_right_distance_m,"
        "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,"
        "battery_voltage_v,board_temperature_c,fan_duty_cycle,error_flags";
    MazeMapApp::Internal::Runtime::RuntimeBinaryLogFile _sampleLog;
    MazeMapApp::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<2048U> _metadata;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<512U> _notes;
    unsigned long _sampleCount;

    bool WriteSectionMarker(const char* type, const OpenFloorMeasurementLabels& labels, const char* reason)
    {
        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "section_id=%s;primitive_id=%s;direction=%s;start_marker=%s;repeat_index=%u;speed_bin=%s%s%s",
            MazeMap::OpenFloorSectionName(labels.sectionId),
            (labels.primitiveId != nullptr) ? labels.primitiveId : "NONE",
            MazeMap::OpenFloorDirectionLabel(labels.direction),
            MazeMap::OpenFloorMarkerName(labels.startMarkerId),
            static_cast<unsigned>(labels.repeatIndex),
            MazeMap::OpenFloorSpeedBinName(labels.speedBin),
            (reason != nullptr && reason[0] != '\0') ? ";reason=" : "",
            (reason != nullptr && reason[0] != '\0') ? reason : "");
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }
        return _eventLog.WriteEvent(micros(), type, message);
    }
};

class OpenFloorRunManifestWriter
{
public:
    bool WriteManifest(const char* runId, bool pinsLatchedAtBoot)
    {
        MazeMap::CoreFileExport file;
        if (!file.Open("run_manifest.json"))
        {
            return false;
        }

        const char* safeRunId = (runId != nullptr) ? runId : "";
        if (!file.Write("{\n")) return false;
        if (!WriteJsonString(file, "build_id", __DATE__ " " __TIME__, true)) return false;
        if (!WriteJsonString(file, "run_id", safeRunId, true)) return false;
        if (!WriteJsonString(file, "boot_reason", "pins_27_28_shorted_at_boot", true)) return false;
        if (!WriteJsonString(file, "selected_routine", "open_floor_measurement", true)) return false;
        if (!WriteJsonBool(file, "pins_27_28_shorted_at_boot", pinsLatchedAtBoot, true)) return false;
        if (!WriteJsonStringArray(
                file,
                "files_written",
                { "run_manifest.json", "timing_boot.mmlog", "timing_boot.events.txt", "open_floor_main.mmlog", "open_floor_main.events.txt" },
                true)) return false;
        if (!WriteJsonActiveConstants(file, true)) return false;
        if (!WriteJsonWorkspaceDefinition(file, true)) return false;
        if (!WriteJsonMarkers(file, true)) return false;
        if (!WriteJsonSectionDefinitions(file, true)) return false;
        if (!WriteJsonPrimitiveSchedule(file, true)) return false;
        if (!WriteJsonDeferredPrimitives(file, false)) return false;
        if (!file.Write("}\n")) return false;

        file.Flush();
        file.Close();
        return true;
    }

private:
    static bool WriteJsonString(MazeMap::CoreFileExport& file, const char* key, const char* value, bool trailingComma)
    {
        char line[256] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "  \"%s\": \"%s\"%s\n",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "",
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteJsonBool(MazeMap::CoreFileExport& file, const char* key, bool value, bool trailingComma)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "  \"%s\": %s%s\n",
            (key != nullptr) ? key : "",
            value ? "true" : "false",
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteJsonUnsigned(MazeMap::CoreFileExport& file, const char* key, unsigned long value, bool trailingComma)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "  \"%s\": %lu%s\n",
            (key != nullptr) ? key : "",
            value,
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteJsonFloat(MazeMap::CoreFileExport& file, const char* key, float value, bool trailingComma)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "  \"%s\": %.3f%s\n",
            (key != nullptr) ? key : "",
            value,
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteJsonStringArray(
        MazeMap::CoreFileExport& file,
        const char* key,
        std::initializer_list<const char*> values,
        bool trailingComma)
    {
        char line[512] = {};
        int length = snprintf(line, sizeof(line), "  \"%s\": [", (key != nullptr) ? key : "");
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        bool first = true;
        for (const char* value : values)
        {
            const int written = snprintf(
                line + length,
                sizeof(line) - static_cast<size_t>(length),
                "%s\"%s\"",
                first ? "" : ", ",
                (value != nullptr) ? value : "");
            if (written <= 0 || (length + written) >= static_cast<int>(sizeof(line)))
            {
                return false;
            }
            length += written;
            first = false;
        }

        const int tail = snprintf(
            line + length,
            sizeof(line) - static_cast<size_t>(length),
            "]%s\n",
            trailingComma ? "," : "");
        return tail > 0 && (length + tail) < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteJsonActiveConstants(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        if (!file.Write("  \"active_constants\": {\n"))
        {
            return false;
        }

        if (!WriteIndentedUnsigned(file, 4U, "control_period_us", DiagnosticConfig::kControlPeriodUs, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "timing_capture_cycles", DiagnosticConfig::kTimingCaptureCycles, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "workspace_half_steps", DiagnosticConfig::kWorkspaceSizeHalfSteps, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "half_step_mm", DiagnosticConfig::kHalfStepMm, 1U, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "static_hold_ms", DiagnosticConfig::kStaticHoldMs, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "launch_repeats_per_magnitude", DiagnosticConfig::kLaunchRepeatsPerMagnitude, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "straight_repeats_per_speed_direction", DiagnosticConfig::kStraightRepeatsPerSpeed, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "yaw_repeats_per_primitive_speed", DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "smooth_repeats_per_primitive_speed", DiagnosticConfig::kSmoothRepeatsPerPrimitiveSpeed, true)) return false;
        if (!WriteIndentedUnsigned(file, 4U, "loop_repeats", DiagnosticConfig::kLoopRepeats, true)) return false;
        if (!WriteIndentedFloatArray(file, 4U, "straight_speed_bins_mps", MazeMap::kOpenFloorStraightSpeedBinsMps, 2U, true)) return false;
        if (!WriteIndentedFloatArray(file, 4U, "yaw_omega_bins_radps", MazeMap::kOpenFloorYawOmegaBinsRadps, 2U, true)) return false;
        if (!WriteIndentedFloatArray(file, 4U, "smooth_speed_bins_mps", MazeMap::kOpenFloorSmoothSpeedBinsMps, 2U, true)) return false;
        if (!WriteIndentedFloatArray(file, 4U, "launch_drive_magnitudes", MazeMap::kOpenFloorLaunchDriveMagnitudes, 2U, false)) return false;

        return file.Write(trailingComma ? "  },\n" : "  }\n");
    }

    static bool WriteJsonWorkspaceDefinition(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        if (!file.Write("  \"workspace_definition\": {\n"))
        {
            return false;
        }

        if (!WriteIndentedString(file, 4U, "rule", "origin_only", true)) return false;
        if (!WriteIndentedBool(file, 4U, "origin_bounded", true, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "x_half_steps_min", 0.0f, 1U, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "x_half_steps_max", static_cast<float>(DiagnosticConfig::kWorkspaceSizeHalfSteps), 1U, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "y_half_steps_min", 0.0f, 1U, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "y_half_steps_max", static_cast<float>(DiagnosticConfig::kWorkspaceSizeHalfSteps), 1U, true)) return false;
        if (!WriteIndentedFloat(file, 4U, "half_step_mm", DiagnosticConfig::kHalfStepMm, 1U, false)) return false;

        return file.Write(trailingComma ? "  },\n" : "  }\n");
    }

    static bool WriteJsonMarkers(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        if (!file.Write("  \"start_markers\": [\n"))
        {
            return false;
        }

        for (size_t index = 0U; index < MazeMap::kOpenFloorMarkers.size(); ++index)
        {
            const MazeMap::OpenFloorMarkerPose& marker = MazeMap::kOpenFloorMarkers[index];
            char line[256] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "    {\"id\": \"%s\", \"x_half_steps\": %.1f, \"y_half_steps\": %.1f, \"heading\": \"%s\"}%s\n",
                marker.name,
                marker.xHalfSteps,
                marker.yHalfSteps,
                DirectionName(marker.heading),
                (index + 1U < MazeMap::kOpenFloorMarkers.size()) ? "," : "");
            if (length <= 0 || length >= static_cast<int>(sizeof(line)) || !file.Write(line))
            {
                return false;
            }
        }

        return file.Write(trailingComma ? "  ],\n" : "  ]\n");
    }

    static bool WriteJsonSectionDefinitions(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        if (!file.Write("  \"section_definitions\": [\n"))
        {
            return false;
        }

        for (size_t index = 0U; index < MazeMap::kOpenFloorSections.size(); ++index)
        {
            const MazeMap::OpenFloorSectionDefinition& section = MazeMap::kOpenFloorSections[index];
            char line[256] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "    {\"section_id\": \"%s\", \"start_marker\": \"%s\"}%s\n",
                section.name,
                MazeMap::OpenFloorMarkerName(section.startMarker),
                (index + 1U < MazeMap::kOpenFloorSections.size()) ? "," : "");
            if (length <= 0 || length >= static_cast<int>(sizeof(line)) || !file.Write(line))
            {
                return false;
            }
        }

        return file.Write(trailingComma ? "  ],\n" : "  ]\n");
    }

    static bool WriteJsonPrimitiveSchedule(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        return file.Write(
                   "  \"primitive_schedule\": [\n"
                   "    {\"section_id\": \"SEC_00_TIMING\", \"start_marker\": \"C\", \"sequence\": [], \"repeats\": 1, \"notes\": \"timing characterization only\"},\n"
                   "    {\"section_id\": \"SEC_10_STATIC\", \"start_marker\": \"C\", \"sequence\": [], \"repeats\": 1, \"notes\": \"stationary hold\"},\n"
                   "    {\"section_id\": \"SEC_20_LAUNCH\", \"start_marker\": \"C\", \"sequence\": [\"OPEN_LOOP_LAUNCH\"], \"signs\": [\"positive\", \"negative\"], \"repeats_per_magnitude\": 5, \"magnitudes\": [0.18, 0.24, 0.30, 0.36], \"recovery\": \"return_to_C_at_low_speed\"},\n"
                   "    {\"section_id\": \"SEC_30_STRAIGHT\", \"start_marker\": \"N\", \"sequence\": [\"STR4\"], \"paired_start_marker\": \"S\", \"directions\": [\"northbound\", \"southbound\"], \"repeats_per_speed_direction\": 3, \"speed_bins_mps\": [0.25, 0.40, 0.55]},\n"
                   "    {\"section_id\": \"SEC_40_YAW\", \"start_marker\": \"C\", \"sequence\": [\"IP90\", \"IP90_M\", \"IP180\"], \"repeats_per_primitive_speed\": 3, \"omega_bins_radps\": [3.00, 6.00, 9.00]},\n"
                   "    {\"section_id\": \"SEC_50_SMOOTH\", \"start_marker\": \"C\", \"sequence\": [\"S45SS\", \"S45SS_M\", \"S90SS\", \"S90SS_M\", \"S135SS\", \"S135SS_M\"], \"repeats_per_primitive_speed\": 5, \"speed_bins_mps\": [0.25, 0.35, 0.45]},\n"
                   "    {\"section_id\": \"SEC_60_LOOP_CW\", \"start_marker\": \"CW\", \"sequence\": [\"STR2\", \"IP90\", \"STR2\", \"IP90\", \"STR2\", \"IP90\", \"STR2\", \"IP90\"], \"repeats\": 5},\n"
                   "    {\"section_id\": \"SEC_70_LOOP_CCW\", \"start_marker\": \"CCW\", \"sequence\": [\"STR2\", \"IP90_M\", \"STR2\", \"IP90_M\", \"STR2\", \"IP90_M\", \"STR2\", \"IP90_M\"], \"repeats\": 5}\n"
                   "  ]")
            && file.Write(trailingComma ? ",\n" : "\n");
    }

    static bool WriteJsonDeferredPrimitives(MazeMap::CoreFileExport& file, bool trailingComma)
    {
        if (!file.Write("  \"deferred_primitive_list\": [\n"))
        {
            return false;
        }

        for (size_t index = 0U; index < MazeMap::kOpenFloorDeferredPrimitiveIds.size(); ++index)
        {
            char line[96] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "    \"%s\"%s\n",
                MazeMap::kOpenFloorDeferredPrimitiveIds[index],
                (index + 1U < MazeMap::kOpenFloorDeferredPrimitiveIds.size()) ? "," : "");
            if (length <= 0 || length >= static_cast<int>(sizeof(line)) || !file.Write(line))
            {
                return false;
            }
        }

        return file.Write(trailingComma ? "  ],\n" : "  ]\n");
    }

    static bool WriteIndentedString(
        MazeMap::CoreFileExport& file,
        uint8_t indent,
        const char* key,
        const char* value,
        bool trailingComma)
    {
        char line[256] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%*s\"%s\": \"%s\"%s\n",
            static_cast<int>(indent),
            "",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "",
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteIndentedBool(
        MazeMap::CoreFileExport& file,
        uint8_t indent,
        const char* key,
        bool value,
        bool trailingComma)
    {
        char line[256] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%*s\"%s\": %s%s\n",
            static_cast<int>(indent),
            "",
            (key != nullptr) ? key : "",
            value ? "true" : "false",
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteIndentedUnsigned(
        MazeMap::CoreFileExport& file,
        uint8_t indent,
        const char* key,
        unsigned long value,
        bool trailingComma)
    {
        char line[256] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%*s\"%s\": %lu%s\n",
            static_cast<int>(indent),
            "",
            (key != nullptr) ? key : "",
            value,
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    static bool WriteIndentedFloat(
        MazeMap::CoreFileExport& file,
        uint8_t indent,
        const char* key,
        float value,
        uint8_t precision,
        bool trailingComma)
    {
        char format[48] = {};
        if (snprintf(format, sizeof(format), "%%*s\"%%s\": %%.%uf%%s\n", static_cast<unsigned>(precision)) <= 0)
        {
            return false;
        }

        char line[256] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            format,
            static_cast<int>(indent),
            "",
            (key != nullptr) ? key : "",
            value,
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

    template <size_t N>
    static bool WriteIndentedFloatArray(
        MazeMap::CoreFileExport& file,
        uint8_t indent,
        const char* key,
        const std::array<float, N>& values,
        uint8_t precision,
        bool trailingComma)
    {
        char format[48] = {};
        if (snprintf(format, sizeof(format), "%%s%%.%uf", static_cast<unsigned>(precision)) <= 0)
        {
            return false;
        }

        char line[512] = {};
        int length = snprintf(
            line,
            sizeof(line),
            "%*s\"%s\": [",
            static_cast<int>(indent),
            "",
            (key != nullptr) ? key : "");
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        for (size_t index = 0U; index < values.size(); ++index)
        {
            char valueBuffer[32] = {};
            const int valueLength = snprintf(
                valueBuffer,
                sizeof(valueBuffer),
                format,
                (index == 0U) ? "" : ", ",
                values[index]);
            if (valueLength <= 0 || (length + valueLength) >= static_cast<int>(sizeof(line)))
            {
                return false;
            }
            memcpy(line + length, valueBuffer, static_cast<size_t>(valueLength));
            length += valueLength;
            line[length] = '\0';
        }

        const int tailLength = snprintf(
            line + length,
            sizeof(line) - static_cast<size_t>(length),
            "]%s\n",
            trailingComma ? "," : "");
        return tailLength > 0 && (length + tailLength) < static_cast<int>(sizeof(line)) && file.Write(line);
    }
};

inline bool IsPinPairStrapped(uint8_t pinA, uint8_t pinB)
{
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
}

inline bool IsPrimaryDiagnosticModeRequested()
{
    return IsPinPairStrapped(DiagnosticConfig::kModeSelectPinA, DiagnosticConfig::kModeSelectPinB);
}

inline bool IsManeuverTestModeRequested()
{
    return IsPinPairStrapped(29U, 30U);
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

namespace MazeMapApp::Internal::Runtime
{
    struct WallTouchObservation
    {
        float frontSkewM = 0.0f;
        bool frontWall = false;
        bool frontLeftWall = false;
        bool frontRightWall = false;
        bool leftWall = false;
        bool rightWall = false;
        bool leftDistanceValidForControl = false;
        bool rightDistanceValidForControl = false;
    };

    inline WallTouchObservation MakeWallTouchObservation(const SensorSnapshot& snapshot)
    {
        WallTouchObservation observation{};
        observation.frontSkewM = snapshot.frontSkewM;
        observation.frontWall = snapshot.frontWall;
        observation.frontLeftWall = snapshot.frontLeftWall;
        observation.frontRightWall = snapshot.frontRightWall;
        observation.leftWall = snapshot.leftWall;
        observation.rightWall = snapshot.rightWall;
        observation.leftDistanceValidForControl = snapshot.leftDistanceValidForControl;
        observation.rightDistanceValidForControl = snapshot.rightDistanceValidForControl;
        return observation;
    }

    inline WallTouchObservation MakeWallTouchObservation(const DiagnosticSensorSnapshot& snapshot)
    {
        WallTouchObservation observation{};
        observation.frontSkewM = snapshot.frontSkewM;
        observation.frontWall = snapshot.frontWall;
        observation.frontLeftWall = snapshot.frontLeft.wall;
        observation.frontRightWall = snapshot.frontRight.wall;
        observation.leftWall = snapshot.leftWall;
        observation.rightWall = snapshot.rightWall;
        observation.leftDistanceValidForControl = snapshot.leftDistanceValidForControl;
        observation.rightDistanceValidForControl = snapshot.rightDistanceValidForControl;
        return observation;
    }

    enum class WallTouchState : uint8_t
    {
        EntryConditioning = 0U,
        ContactSeek,
        SeatingPreloadRamp,
        InitialSeatingDwell,
        SquareUpDither,
        PostSquareSeatedHold,
        ControlledRelease,
        ReverseToClearance,
        Handoff
    };

    inline const char* WallTouchStateName(WallTouchState state)
    {
        switch (state)
        {
        case WallTouchState::EntryConditioning:
            return "entry";
        case WallTouchState::ContactSeek:
            return "seek";
        case WallTouchState::SeatingPreloadRamp:
            return "preload_ramp";
        case WallTouchState::InitialSeatingDwell:
            return "seat_dwell";
        case WallTouchState::SquareUpDither:
            return "square_dither";
        case WallTouchState::PostSquareSeatedHold:
            return "post_square_hold";
        case WallTouchState::ControlledRelease:
            return "release";
        case WallTouchState::ReverseToClearance:
            return "reverse_clear";
        case WallTouchState::Handoff:
            return "handoff";
        default:
            return "unknown";
        }
    }

    struct WallTouchExecutionResult
    {
        WallTouchOutcome outcome = WallTouchOutcome::SeatedContact;
        float seatedTravelM = 0.0f;
        float finalTravelM = 0.0f;
        float reverseDistanceM = 0.0f;
        float seatedYawErrorRad = 0.0f;
        unsigned long confirmedContactMs = 0UL;
        uint8_t completedFullCycles = 0U;
    };

    inline bool HasWallTouchEncoderMotion(
        const DriveTelemetry& reference,
        const DriveTelemetry& current,
        float minimumPerWheelDistanceDeltaM)
    {
        if (!std::isfinite(reference.leftDistanceM) ||
            !std::isfinite(reference.rightDistanceM) ||
            !std::isfinite(current.leftDistanceM) ||
            !std::isfinite(current.rightDistanceM) ||
            !std::isfinite(minimumPerWheelDistanceDeltaM) ||
            minimumPerWheelDistanceDeltaM < 0.0f)
        {
            return false;
        }

        return (std::fabs(current.leftDistanceM - reference.leftDistanceM) >= minimumPerWheelDistanceDeltaM) ||
            (std::fabs(current.rightDistanceM - reference.rightDistanceM) >= minimumPerWheelDistanceDeltaM);
    }

    inline float ComputeWallTouchApproachDriveCommand(
        float traveledDistanceM,
        float minLatchTravelM)
    {
        if (!std::isfinite(traveledDistanceM) || !std::isfinite(minLatchTravelM))
        {
            return Config::kWallTouchFinalApproachDriveCommand;
        }

        const float effectiveFinalApproachWindowM =
            (std::clamp)(Config::kWallTouchFinalApproachWindowM, 0.0f, 0.5f * minLatchTravelM);
        const float remainingToLatchM = minLatchTravelM - traveledDistanceM;
        if ((effectiveFinalApproachWindowM > 0.0f) &&
            (remainingToLatchM <= effectiveFinalApproachWindowM))
        {
            return Config::kWallTouchFinalApproachDriveCommand;
        }

        return Config::kWallTouchDriveCommand;
    }

    inline bool ShouldBrakeWallTouchApproachForEncoderSpeed(const DriveTelemetry& telemetry)
    {
        const float encoderSpeedMps = MazeMap::ComputeAverageEncoderAbsSpeedMps(
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        return std::isfinite(encoderSpeedMps) &&
            (encoderSpeedMps >= Config::kWallTouchMaxApproachEncoderSpeedMps);
    }

    inline float LimitWallTouchApproachDriveCommandByEncoderSpeed(
        float requestedDriveCommand,
        const DriveTelemetry& telemetry)
    {
        if (!std::isfinite(requestedDriveCommand))
        {
            return Config::kWallTouchFinalApproachDriveCommand;
        }

        const float encoderSpeedMps = MazeMap::ComputeAverageEncoderAbsSpeedMps(
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        if (!(std::isfinite(encoderSpeedMps) && (encoderSpeedMps > 0.0f)))
        {
            return requestedDriveCommand;
        }

        const float scale = Config::kWallTouchMaxApproachEncoderSpeedMps / encoderSpeedMps;
        return requestedDriveCommand * (std::clamp)(scale, 0.0f, 1.0f);
    }

    template <typename TickFn, typename TraceFn, typename FinishFn, typename FailFn, typename SeatedResetFn>
    inline bool ExecuteSharedWallTouchOff(
        DriveBase& drive,
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        TickFn&& tickControl,
        TraceFn&& appendTraceLine,
        FinishFn&& finishWallTouch,
        FailFn&& fail,
        SeatedResetFn&& onSeatedHold,
        WallTouchExecutionResult& result)
    {
        result = {};
        result.outcome = WallTouchOutcome::SeatedContact;

        const float clampedMinLatchTravelM = (std::max)(0.0f, minLatchTravelM);
        const float clampedMaxApproachTravelM = (std::max)(clampedMinLatchTravelM, maxApproachTravelM);
        if (!(std::isfinite(clampedMaxApproachTravelM) && (clampedMaxApproachTravelM > 0.0f)))
        {
            return fail("Wall touch-off max travel is invalid");
        }

        const float startDistanceM = drive.GetAverageDistanceMeters();
        const unsigned long touchStartMs = millis();
        const float motionEpsilonM = Config::kWallTouchProgressStallDistanceM;
        DriveTelemetry lastMotionTelemetry = drive.GetTelemetry();
        unsigned long lastMotionMs = touchStartMs;
        WallTouchState state = WallTouchState::EntryConditioning;
        unsigned long stateStartMs = touchStartMs;
        unsigned long contactCandidateStartMs = 0UL;
        bool contactCandidateActive = false;
        unsigned long contactConfirmedStartMs = 0UL;
        bool seatedResetApplied = false;
        unsigned long frontSignalMissingStartMs = 0UL;
        float approachDriveCommand = Config::kWallTouchDriveCommand;
        float ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
        float previousCycleFrontSkewMagnitudeM = std::numeric_limits<float>::infinity();
        float currentCycleStartYawRad = drive.GetPose().yawRad;
        float currentCycleMaxFrontSkewMagnitudeM = 0.0f;
        float currentCycleMaxResidualYawRateRadps = 0.0f;
        bool currentCycleFrontSignalValid = true;
        bool haveSquareSample = false;
        unsigned long lastHalfCycleIndex = 0UL;
        float lastSquareYawRad = drive.GetPose().yawRad;
        float lastSquareFrontSkewM = 0.0f;
        float lastSquareYawRateRadps = 0.0f;
        bool lastSquareFrontSignalValid = false;
        uint8_t completedHalfCycles = 0U;
        uint8_t consecutiveGoodFullCycles = 0U;

        auto traceStateTransition = [&](WallTouchState fromState, WallTouchState toState, float traveledDistanceM)
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                WallTouchStateName(fromState),
                WallTouchStateName(toState),
                static_cast<unsigned long>(millis() - touchStartMs),
                traveledDistanceM);
            appendTraceLine(line);
        };

        state = WallTouchState::ContactSeek;
        traceStateTransition(WallTouchState::EntryConditioning, state, 0.0f);

        while (true)
        {
            float dtSeconds = 0.0f;
            WallTouchObservation observation{};
            if (!tickControl(false, dtSeconds, observation))
            {
                return false;
            }

            const unsigned long nowMs = millis();
            const unsigned long elapsedMs = nowMs - touchStartMs;
            const unsigned long stateElapsedMs = nowMs - stateStartMs;
            const PoseEstimate& pose = drive.GetPose();
            const DriveTelemetry telemetry = drive.GetTelemetry();
            const float traveledDistanceM = std::fabs(drive.GetAverageDistanceMeters() - startDistanceM);
            const float encoderSpeedMps = MazeMap::ComputeAverageEncoderAbsSpeedMps(
                telemetry.leftVelocityMps,
                telemetry.rightVelocityMps);
            const bool frontSignalActive =
                observation.frontWall ||
                observation.frontLeftWall ||
                observation.frontRightWall;

            result.finalTravelM = traveledDistanceM;
            if ((state != WallTouchState::ControlledRelease) &&
                (state != WallTouchState::ReverseToClearance) &&
                (traveledDistanceM >= clampedMaxApproachTravelM))
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:max_travel,state=%s,travel=%.4f,expected=%.4f,max=%.4f",
                    WallTouchStateName(state),
                    traveledDistanceM,
                    clampedMinLatchTravelM,
                    clampedMaxApproachTravelM);
                appendTraceLine(line);
                if (allowPassThroughNoWall && (contactConfirmedStartMs == 0UL))
                {
                    result.outcome = WallTouchOutcome::PassedThroughNoWall;
                    return finishWallTouch("Wall touch-off failed to settle after pass-through");
                }
                return fail("Wall touch-off exceeded max travel");
            }

            if (HasWallTouchEncoderMotion(lastMotionTelemetry, telemetry, motionEpsilonM))
            {
                lastMotionMs = nowMs;
                lastMotionTelemetry = telemetry;
            }

            if (state == WallTouchState::ContactSeek)
            {
                approachDriveCommand = ComputeWallTouchApproachDriveCommand(
                    traveledDistanceM,
                    clampedMinLatchTravelM);
                if (ShouldBrakeWallTouchApproachForEncoderSpeed(telemetry))
                {
                    drive.Brake();
                }
                else
                {
                    approachDriveCommand = LimitWallTouchApproachDriveCommandByEncoderSpeed(
                        approachDriveCommand,
                        telemetry);
                    drive.CommandOpenLoopRaw(MazeMap::MakeSymmetricOpenLoopDriveCommand(approachDriveCommand));
                }

                const bool motionCollapseIndicator = MazeMap::IsWallTouchContactSample(
                    traveledDistanceM,
                    pose.linearSpeedMps,
                    Config::kWallTouchMinApproachDistanceM,
                    clampedMinLatchTravelM,
                    Config::kMotionSettleSpeedThresholdMps,
                    elapsedMs,
                    Config::kWallTouchMinCommandTimeMs);
                const bool progressStallIndicator =
                    (elapsedMs >= Config::kWallTouchMinCommandTimeMs) &&
                    ((nowMs - lastMotionMs) >= Config::kWallTouchProgressStallWindowMs);
                const uint8_t indicatorCount = MazeMap::CountWallTouchContactIndicators(
                    frontSignalActive,
                    motionCollapseIndicator,
                    progressStallIndicator);
                const bool contactIndicatorsSatisfied = indicatorCount >= 2U;
                if (contactIndicatorsSatisfied)
                {
                    if (!contactCandidateActive)
                    {
                        contactCandidateStartMs = nowMs;
                        contactCandidateActive = true;
                    }
                    else if (MazeMap::HasWallTouchConfirmedContact(
                        nowMs - contactCandidateStartMs,
                        Config::kWallTouchContactConfirmationMs,
                        indicatorCount))
                    {
                        contactConfirmedStartMs = contactCandidateStartMs;
                        state = WallTouchState::SeatingPreloadRamp;
                        stateStartMs = nowMs;
                        char line[224] = {};
                        snprintf(
                            line,
                            sizeof(line),
                            "startup_cal_touch:contact_confirmed,travel=%.4f,elapsed_ms=%lu,front=%u,collapse=%u,stall=%u",
                            traveledDistanceM,
                            elapsedMs,
                            frontSignalActive ? 1U : 0U,
                            motionCollapseIndicator ? 1U : 0U,
                            progressStallIndicator ? 1U : 0U);
                        appendTraceLine(line);
                        traceStateTransition(WallTouchState::ContactSeek, state, traveledDistanceM);
                    }
                }
                else
                {
                    contactCandidateActive = false;
                }

                continue;
            }

            if (state == WallTouchState::SeatingPreloadRamp)
            {
                const float rampAlpha =
                    static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchSeatRampMs))) /
                    static_cast<float>((std::max)(Config::kWallTouchSeatRampMs, static_cast<uint16_t>(1U)));
                const float seatDriveCommand =
                    approachDriveCommand +
                    ((Config::kWallTouchSeatRampMaxDriveCommand - approachDriveCommand) * rampAlpha);
                drive.CommandOpenLoopRaw(MazeMap::MakeSymmetricOpenLoopDriveCommand(seatDriveCommand));
                if (stateElapsedMs >= Config::kWallTouchSeatRampMs)
                {
                    state = WallTouchState::InitialSeatingDwell;
                    stateStartMs = nowMs;
                    traceStateTransition(WallTouchState::SeatingPreloadRamp, state, traveledDistanceM);
                }
                continue;
            }

            if (state == WallTouchState::InitialSeatingDwell)
            {
                drive.CommandOpenLoopRaw(MazeMap::MakeSymmetricOpenLoopDriveCommand(Config::kWallTouchSeatRampMaxDriveCommand));
                if (stateElapsedMs >= Config::kWallTouchInitialSeatDwellMs)
                {
                    state = WallTouchState::SquareUpDither;
                    stateStartMs = nowMs;
                    currentCycleStartYawRad = pose.yawRad;
                    currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                    currentCycleMaxResidualYawRateRadps = 0.0f;
                    currentCycleFrontSignalValid = true;
                    haveSquareSample = false;
                    completedHalfCycles = 0U;
                    result.completedFullCycles = 0U;
                    consecutiveGoodFullCycles = 0U;
                    ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
                    frontSignalMissingStartMs = 0UL;
                    traceStateTransition(WallTouchState::InitialSeatingDwell, state, traveledDistanceM);
                }
                continue;
            }

            if (state == WallTouchState::SquareUpDither)
            {
                const unsigned long contactDurationMs =
                    (contactConfirmedStartMs > 0UL) ?
                    (nowMs - contactConfirmedStartMs) :
                    0UL;
                result.confirmedContactMs = contactDurationMs;
                if (!frontSignalActive)
                {
                    if (frontSignalMissingStartMs == 0UL)
                    {
                        frontSignalMissingStartMs = nowMs;
                    }
                    else if ((nowMs - frontSignalMissingStartMs) >= Config::kWallTouchContactConfirmationMs)
                    {
                        char line[192] = {};
                        snprintf(
                            line,
                            sizeof(line),
                            "startup_cal_touch:front_signal_invalid,elapsed_ms=%lu,travel=%.4f",
                            contactDurationMs,
                            traveledDistanceM);
                        appendTraceLine(line);
                        return fail("Wall touch-off front sensors invalid during square-up");
                    }
                }
                else
                {
                    frontSignalMissingStartMs = 0UL;
                }

                const MazeMap::OpenLoopDriveCommand ditherCommand = MazeMap::ComputeOpenLoopYawDitherCommand(
                    Config::kWallTouchSeatRampMaxDriveCommand,
                    stateElapsedMs,
                    Config::kWallTouchSeatWiggleHalfPeriodMs,
                    Config::kWallTouchSeatWiggleBlendMs,
                    ditherTurnFraction,
                    Config::kWallTouchSeatWiggleRetainedForwardFraction);
                drive.CommandOpenLoopRaw(ditherCommand);

                const unsigned long halfCycleIndex =
                    stateElapsedMs /
                    (std::max)(Config::kWallTouchSeatWiggleHalfPeriodMs, static_cast<uint16_t>(1U));
                if (haveSquareSample && (halfCycleIndex != lastHalfCycleIndex))
                {
                    ++completedHalfCycles;
                    currentCycleMaxFrontSkewMagnitudeM = (std::max)(
                        currentCycleMaxFrontSkewMagnitudeM,
                        std::fabs(lastSquareFrontSkewM));
                    currentCycleMaxResidualYawRateRadps = (std::max)(
                        currentCycleMaxResidualYawRateRadps,
                        std::fabs(lastSquareYawRateRadps));
                    currentCycleFrontSignalValid = currentCycleFrontSignalValid && lastSquareFrontSignalValid;

                    char halfCycleLine[256] = {};
                    snprintf(
                        halfCycleLine,
                        sizeof(halfCycleLine),
                        "startup_cal_touch:half_cycle,index=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,turn_fraction=%.3f",
                        static_cast<unsigned>(completedHalfCycles),
                        std::fabs(lastSquareFrontSkewM),
                        std::fabs(lastSquareYawRateRadps),
                        ditherTurnFraction);
                    appendTraceLine(halfCycleLine);

                    if ((completedHalfCycles & 1U) == 0U)
                    {
                        ++result.completedFullCycles;
                        const float netYawChangeMagnitudeRad = std::fabs(AngleErrorRad(currentCycleStartYawRad, lastSquareYawRad));
                        const bool cycleGood = MazeMap::IsWallTouchSquareCycleGood(
                            currentCycleMaxFrontSkewMagnitudeM,
                            Config::kWallTouchSquareFrontSkewThresholdM,
                            currentCycleMaxResidualYawRateRadps,
                            Config::kWallTouchSquareResidualYawRateThresholdRadps,
                            netYawChangeMagnitudeRad,
                            Config::kWallTouchSquareNetYawChangeThresholdRad,
                            currentCycleFrontSignalValid);
                        consecutiveGoodFullCycles = cycleGood ? (consecutiveGoodFullCycles + 1U) : 0U;

                        char cycleLine[320] = {};
                        snprintf(
                            cycleLine,
                            sizeof(cycleLine),
                            "startup_cal_touch:full_cycle,index=%u,good=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,net_yaw_deg=%.2f,contact_ms=%lu,turn_fraction=%.3f",
                            static_cast<unsigned>(result.completedFullCycles),
                            cycleGood ? 1U : 0U,
                            currentCycleMaxFrontSkewMagnitudeM,
                            currentCycleMaxResidualYawRateRadps,
                            RAD_TO_DEG_F * netYawChangeMagnitudeRad,
                            contactDurationMs,
                            ditherTurnFraction);
                        appendTraceLine(cycleLine);

                        if (!cycleGood &&
                            (ditherTurnFraction < Config::kWallTouchSeatWiggleMaxTurnFraction) &&
                            (MazeMap::HasWallTouchSquareUpSaturated(
                                previousCycleFrontSkewMagnitudeM,
                                currentCycleMaxFrontSkewMagnitudeM,
                                Config::kWallTouchSquareImprovementSaturationThresholdM) ||
                                (result.completedFullCycles >= Config::kWallTouchSeatMinimumFullCycles)))
                        {
                            ditherTurnFraction = MazeMap::ComputeWallTouchSeatWiggleTurnFraction(
                                result.completedFullCycles,
                                Config::kWallTouchSeatWiggleTurnFraction,
                                Config::kWallTouchSeatWiggleTurnFractionStep,
                                Config::kWallTouchSeatWiggleMaxTurnFraction);
                        }

                        previousCycleFrontSkewMagnitudeM = currentCycleMaxFrontSkewMagnitudeM;
                        currentCycleStartYawRad = lastSquareYawRad;
                        currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                        currentCycleMaxResidualYawRateRadps = 0.0f;
                        currentCycleFrontSignalValid = true;

                        if (MazeMap::IsWallTouchSquareSuccessEligible(
                            contactDurationMs,
                            Config::kWallTouchMinimumConfirmedContactMs,
                            result.completedFullCycles,
                            Config::kWallTouchSeatMinimumFullCycles,
                            consecutiveGoodFullCycles,
                            Config::kWallTouchSeatRequiredGoodFullCycles))
                        {
                            state = WallTouchState::PostSquareSeatedHold;
                            stateStartMs = nowMs;
                            result.seatedTravelM = traveledDistanceM;
                            result.seatedYawErrorRad = AngleErrorRad(targetYawRad, pose.yawRad);
                            traceStateTransition(WallTouchState::SquareUpDither, state, traveledDistanceM);
                            continue;
                        }
                    }
                }

                if (contactDurationMs >= Config::kWallTouchSquareUpTimeoutMs)
                {
                    char line[192] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:square_timeout,contact_ms=%lu,turn_fraction=%.3f,cycles=%u",
                        contactDurationMs,
                        ditherTurnFraction,
                        static_cast<unsigned>(result.completedFullCycles));
                    appendTraceLine(line);
                    return fail("Wall touch-off square-up timed out");
                }

                haveSquareSample = true;
                lastHalfCycleIndex = halfCycleIndex;
                lastSquareYawRad = pose.yawRad;
                lastSquareFrontSkewM = observation.frontSkewM;
                lastSquareYawRateRadps = pose.angularSpeedRadps;
                lastSquareFrontSignalValid = frontSignalActive;
                continue;
            }

            if (state == WallTouchState::PostSquareSeatedHold)
            {
                drive.CommandOpenLoopRaw(MazeMap::MakeSymmetricOpenLoopDriveCommand(Config::kWallTouchSeatRampMaxDriveCommand));
                if (!seatedResetApplied &&
                    (stateElapsedMs >= (Config::kWallTouchPostSquareHoldMs / 2U)))
                {
                    if (!onSeatedHold(result))
                    {
                        return false;
                    }
                    seatedResetApplied = true;
                }
                if (stateElapsedMs >= Config::kWallTouchPostSquareHoldMs)
                {
                    if (!seatedResetApplied)
                    {
                        if (!onSeatedHold(result))
                        {
                            return false;
                        }
                        seatedResetApplied = true;
                    }
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:reset_pose,x=%.4f,y=%.4f,yaw_deg=%.2f,travel=%.4f",
                        drive.GetPose().xMeters,
                        drive.GetPose().yMeters,
                        RAD_TO_DEG_F * drive.GetPose().yawRad,
                        result.seatedTravelM);
                    appendTraceLine(line);
                    state = WallTouchState::ControlledRelease;
                    stateStartMs = nowMs;
                    traceStateTransition(WallTouchState::PostSquareSeatedHold, state, traveledDistanceM);
                }
                continue;
            }

            if (state == WallTouchState::ControlledRelease)
            {
                const float releaseAlpha =
                    static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseRampMs))) /
                    static_cast<float>((std::max)(Config::kWallTouchReleaseRampMs, static_cast<uint16_t>(1U)));
                const float forwardPreloadCommand =
                    Config::kWallTouchSeatRampMaxDriveCommand * (1.0f - releaseAlpha);
                float reverseCommand = 0.0f;
                if (Config::kWallTouchReleaseReverseOverlapMs >= Config::kWallTouchReleaseRampMs)
                {
                    reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * releaseAlpha;
                }
                else if (stateElapsedMs >= (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs))
                {
                    const unsigned long reverseElapsedMs =
                        stateElapsedMs - (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs);
                    const float reverseAlpha =
                        static_cast<float>((std::min)(reverseElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseReverseOverlapMs))) /
                        static_cast<float>((std::max)(Config::kWallTouchReleaseReverseOverlapMs, static_cast<uint16_t>(1U)));
                    reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * reverseAlpha;
                }

                drive.CommandOpenLoopRaw(MazeMap::MakeSymmetricOpenLoopDriveCommand(forwardPreloadCommand - reverseCommand));
                result.reverseDistanceM = (std::max)(0.0f, result.seatedTravelM - traveledDistanceM);
                if ((result.reverseDistanceM >= Config::kDistanceToleranceM) && !frontSignalActive)
                {
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:release_clear,reverse_m=%.4f,elapsed_ms=%lu",
                        result.reverseDistanceM,
                        stateElapsedMs);
                    appendTraceLine(line);
                    state = WallTouchState::ReverseToClearance;
                    stateStartMs = nowMs;
                    traceStateTransition(WallTouchState::ControlledRelease, state, traveledDistanceM);
                    continue;
                }

                if (stateElapsedMs >= Config::kWallTouchReleaseRampMs)
                {
                    state = WallTouchState::ReverseToClearance;
                    stateStartMs = nowMs;
                    traceStateTransition(WallTouchState::ControlledRelease, state, traveledDistanceM);
                }
                continue;
            }

            if (state == WallTouchState::ReverseToClearance)
            {
                result.reverseDistanceM = (std::max)(0.0f, result.seatedTravelM - traveledDistanceM);
                const float headingErrorRad = AngleErrorRad(targetYawRad, pose.yawRad);
                float angularCommandRadps =
                    (Config::kStraightHeadingKp * headingErrorRad) -
                    (Config::kStraightYawD * pose.angularSpeedRadps);
                angularCommandRadps = (std::clamp)(
                    angularCommandRadps,
                    -Config::kWallTouchReverseMaxAngularCommandRadps,
                    Config::kWallTouchReverseMaxAngularCommandRadps);
                drive.CommandVelocity(-Config::kWallTouchReverseSpeedMps, angularCommandRadps, dtSeconds);

                if (result.reverseDistanceM >= Config::kWallTouchFrontClearanceDistanceM)
                {
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:clearance_reached,reverse_m=%.4f,target_m=%.4f",
                        result.reverseDistanceM,
                        Config::kWallTouchFrontClearanceDistanceM);
                    appendTraceLine(line);
                    drive.Brake();
                    state = WallTouchState::Handoff;
                    traceStateTransition(WallTouchState::ReverseToClearance, state, traveledDistanceM);
                    return true;
                }

                if ((stateElapsedMs >= Config::kMotionSettleTimeoutMs) &&
                    frontSignalActive)
                {
                    char line[192] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:clearance_failed,reverse_m=%.4f,elapsed_ms=%lu",
                        result.reverseDistanceM,
                        stateElapsedMs);
                    appendTraceLine(line);
                    return fail("Wall touch-off failed to establish front-wall clearance");
                }
            }
        }
    }
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
        : _sampleLog()
        , _eventLog()
        , _metadata()
        , _notes()
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
        _phaseId = 0UL;
        _sampleCount = 0UL;
        _metadata.Clear();
        _notes.Clear();
        if (!MazeMapApp::Internal::Runtime::SelectSequentialRuntimeFileName(
                _fileName,
                sizeof(_fileName),
                fileName,
                "aux%03u.mmlog",
                "aux_measurement_log.mmlog"))
        {
            return false;
        }
        if (!_eventLog.BeginSibling(_fileName))
        {
            return false;
        }
        if (!WriteMetadata("file", _fileName)) return false;
        if (_eventLog.IsEnabled() && !WriteMetadata("events_file", _eventLog.GetFileName())) return false;
        if (!WriteMetadata("mode", "aux_measurement")) return false;
        if (!WriteMetadata("routine", AuxMeasurementRoutineName(routine))) return false;
        if (!WriteMetadataUL("control_period_us", AuxMeasurementConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !WriteMetadataUL("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                AuxMeasurementConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
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
        if (!WriteMetadataUL("mission_accel_bias_valid", sensors.HasAccelBias() ? 1UL : 0UL)) return false;
        if (sensors.HasAccelBias() &&
            (!WriteMetadataFloat("mission_accel_bias_x_mg", sensors.GetAccelBiasXG() * 1000.0f, 3) ||
             !WriteMetadataFloat("mission_accel_bias_y_mg", sensors.GetAccelBiasYG() * 1000.0f, 3))) return false;
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
        if (!MazeMapApp::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName())) return false;
        return _sampleLog.BeginSelected(
            _fileName,
            kAuxSchema,
            kAuxFieldCount,
            _metadata.Data(),
            _notes.Data(),
            MazeMapApp::Internal::Runtime::kRuntimeBinaryLogFlags,
            0U,
            micros());
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;
        return _eventLog.WritePhase(_phaseId, micros(), name);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        return _eventLog.WriteEvent(micros(), type, message);
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
        MazeMapApp::Internal::Runtime::RuntimeRecordBuilder<kAuxFieldCount> record;
        record.U32(static_cast<uint32_t>(_sampleCount));
        record.U32(static_cast<uint32_t>(_phaseId));
        record.U32(timestampUs);
        record.U32(dtUs);
        record.U32(stationary ? 1U : 0U);
        record.U32(fanEnabled ? 1U : 0U);
        record.F32(pose.xMeters);
        record.F32(pose.yMeters);
        record.F32(pose.yawRad);
        record.F32(pose.linearSpeedMps);
        record.F32(pose.angularSpeedRadps);
        record.F32(planarAccelMps2);
        record.F32(drive.GetLastLinearCommandMps());
        record.F32(drive.GetLastAngularCommandRadps());
        record.F32(driveTelemetry.leftDriveCommand);
        record.F32(driveTelemetry.rightDriveCommand);
        MazeMapApp::Internal::Runtime::AppendDriveTelemetryFields(record, driveTelemetry);
        MazeMapApp::Internal::Runtime::AppendImuTelemetryFields(record, sensorSnapshot.imuFrontRight);
        MazeMapApp::Internal::Runtime::AppendImuTelemetryFields(record, sensorSnapshot.imuBackLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.frontLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.frontRight);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.sideLeft);
        MazeMapApp::Internal::Runtime::AppendWallSensorFields(record, sensorSnapshot.sideRight);
        record.U32(sensorSnapshot.frontWall ? 1U : 0U);
        record.U32(sensorSnapshot.leftWall ? 1U : 0U);
        record.U32(sensorSnapshot.rightWall ? 1U : 0U);
        record.F32(sensorSnapshot.corridorErrorM);
        record.F32(sensorSnapshot.frontSkewM);
        record.F32(sensorSnapshot.gyroBiasRadps);
        record.F32(sensorSnapshot.gyroRawRadps);
        record.F32(sensorSnapshot.gyroRadps);

        if (!MazeMapApp::Internal::Runtime::AppendBinaryRecord(_sampleLog, record))
        {
            return false;
        }

        ++_sampleCount;
        return true;
    }

    void Service()
    {
        (void)_sampleLog.Service(1U);
    }

    void Flush()
    {
        _sampleLog.Flush();
        _eventLog.Flush();
    }

    void Close()
    {
        _sampleLog.Close();
        _eventLog.Close();
    }

    const char* GetFileName() const
    {
        return _fileName;
    }

private:
    static constexpr uint32_t kAuxFieldCount = 68U;
    static constexpr const char* kAuxSchema =
        "sample,phase_id,t_us,dt_us,stationary,fan_enabled,"
        "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,planar_accel_mps2,"
        "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
        "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
        "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
        "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
        "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
        "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
        "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps";
    MazeMapApp::Internal::Runtime::RuntimeBinaryLogFile _sampleLog;
    MazeMapApp::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<8192U> _metadata;
    MazeMapApp::Internal::Runtime::RuntimeTextBlockBuilder<512U> _notes;
    char _fileName[64];
    unsigned long _phaseId;
    unsigned long _sampleCount;

    bool WriteMetadata(const char* key, const char* value)
    {
        return _metadata.AppendKeyValue(key, value);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        return _metadata.AppendUnsigned(key, value);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        return _metadata.AppendFloat(key, value, precision);
    }
};



