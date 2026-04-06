#pragma once
// Declares runtime logging, telemetry, and measurement-capture infrastructure for the MazeMap application runtime.
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OptionalRuntimeEventLog.h"
#include "RuntimeBinaryLogSupport.h"
#include "RuntimeControlLogSupport.h"

// Private application infrastructure helpers for the MazeMap runtime.

#define DIAGNOSTIC_LOG_FIELDS(X)                  \
    X(std::uint32_t, sample)                     \
    X(std::uint32_t, phase_id)                   \
    X(std::uint32_t, t_us)                       \
    X(std::uint32_t, dt_us)                      \
    X(std::uint32_t, stationary)                 \
    X(float,         pose_x_m)                   \
    X(float,         pose_y_m)                   \
    X(float,         yaw_rad)                    \
    X(float,         linear_speed_mps)           \
    X(float,         angular_speed_radps)        \
    X(float,         cmd_linear_mps)             \
    X(float,         cmd_angular_radps)          \
    X(float,         left_drive_cmd)             \
    X(float,         right_drive_cmd)            \
    X(std::int32_t,  left_encoder_count)         \
    X(std::int32_t,  right_encoder_count)        \
    X(float,         left_distance_m)            \
    X(float,         right_distance_m)           \
    X(float,         left_velocity_mps)          \
    X(float,         right_velocity_mps)         \
    X(std::uint32_t, imu_fr_status)              \
    X(std::int32_t,  imu_fr_gyro_x)              \
    X(std::int32_t,  imu_fr_gyro_y)              \
    X(std::int32_t,  imu_fr_gyro_z)              \
    X(std::int32_t,  imu_fr_accel_x)             \
    X(std::int32_t,  imu_fr_accel_y)             \
    X(std::int32_t,  imu_fr_accel_z)             \
    X(std::int32_t,  imu_fr_temp)                \
    X(std::uint32_t, imu_fr_int)                 \
    X(std::uint32_t, imu_bl_status)              \
    X(std::int32_t,  imu_bl_gyro_x)              \
    X(std::int32_t,  imu_bl_gyro_y)              \
    X(std::int32_t,  imu_bl_gyro_z)              \
    X(std::int32_t,  imu_bl_accel_x)             \
    X(std::int32_t,  imu_bl_accel_y)             \
    X(std::int32_t,  imu_bl_accel_z)             \
    X(std::int32_t,  imu_bl_temp)                \
    X(std::uint32_t, imu_bl_int)                 \
    X(float,         ws_fl_ambient)              \
    X(float,         ws_fl_lit)                  \
    X(float,         ws_fl_delta)                \
    X(float,         ws_fl_raw_distance_m)       \
    X(float,         ws_fl_distance_m)           \
    X(float,         ws_fr_ambient)              \
    X(float,         ws_fr_lit)                  \
    X(float,         ws_fr_delta)                \
    X(float,         ws_fr_raw_distance_m)       \
    X(float,         ws_fr_distance_m)           \
    X(float,         ws_sl_ambient)              \
    X(float,         ws_sl_lit)                  \
    X(float,         ws_sl_delta)                \
    X(float,         ws_sl_raw_distance_m)       \
    X(float,         ws_sl_distance_m)           \
    X(float,         ws_sr_ambient)              \
    X(float,         ws_sr_lit)                  \
    X(float,         ws_sr_delta)                \
    X(float,         ws_sr_raw_distance_m)       \
    X(float,         ws_sr_distance_m)           \
    X(std::uint32_t, front_wall)                 \
    X(std::uint32_t, left_wall)                  \
    X(std::uint32_t, right_wall)                 \
    X(float,         corridor_error_m)           \
    X(float,         front_skew_m)               \
    X(float,         gyro_bias_radps)            \
    X(float,         gyro_raw_radps)             \
    X(float,         gyro_radps)

MMLOG_DEFINE_ROW(DiagnosticLogRow, DIAGNOSTIC_LOG_FIELDS);


namespace MazeMap::App::Internal::Runtime
{
    inline bool WriteMmLogAccelBiasMetadata(
        MazeMap::mmlog::MmLogLogger& log,
        const DiagnosticSensorSuite& sensors)
    {
        char value[48] = {};
        if (!sensors.HasAccelBias())
        {
            return log.writeMetadata("mission_accel_bias_mg", "na");
        }

        const int length = snprintf(
            value,
            sizeof(value),
            "x=%.3f;y=%.3f",
            sensors.GetAccelBiasXG() * 1000.0f,
            sensors.GetAccelBiasYG() * 1000.0f);
        return
            length > 0 &&
            length < static_cast<int>(sizeof(value)) &&
            log.writeMetadata("mission_accel_bias_mg", value);
    }

    inline bool WriteDiagnosticTuningEvents(OptionalRuntimeEventLog& eventLog)
    {
        const auto& driveModel = MazeMap::MotorEncoderDrive::GetSharedPhysicalModel();
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        static const MazeMap::Vehicle sharedVehicle{};
        const MazeMap::InPlaceTurnProfile inPlaceTurnProfile = BuildSharedInPlaceTurnProfile(sharedVehicle);
        char message[256] = {};
        auto writeConfig = [&eventLog, &message](const char* format, auto... args) -> bool
        {
            const int length = snprintf(message, sizeof(message), format, args...);
            if (length <= 0 || length >= static_cast<int>(sizeof(message)))
            {
                return false;
            }
            return eventLog.WriteEvent(micros(), "config", message);
        };

        return
            writeConfig(
                "drive_geometry:track_width_m=%.6f;tight_r_m=%.6f;tight_w_m=%.6f;wide_r_m=%.6f;wide_w_m=%.6f",
                Config::kTrackWidthM,
                vehicleModel.arcTrackWidthInterpolation.tightRadiusM,
                vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM,
                vehicleModel.arcTrackWidthInterpolation.wideRadiusM,
                vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM) &&
            writeConfig(
                "motor_model:wheel_diam_m=%.6f;encoder_cpr=%lu;gear=%.6f;nom_v=%.3f;no_load_rpm=%.1f;supply_v=%.3f",
                driveModel.wheelDiameterM,
                static_cast<unsigned long>(driveModel.pulsesPerRev),
                driveModel.gearRatio,
                driveModel.nominalVoltageV,
                driveModel.nominalNoLoadSpeedRpm,
                driveModel.supplyVoltageV) &&
            writeConfig(
                "wheel_control:static_ff=%.6f;vel_ff=%.6f;accel_gain=%.6f;vel_kp=%.6f;vel_ki=%.6f;i_lim=%.6f",
                Config::kWheelStaticFeedforward,
                Config::kWheelVelocityFeedforward,
                Config::kWheelAccelerationResponseGainPerMps2,
                Config::kWheelVelocityKp,
                Config::kWheelVelocityKi,
                Config::kWheelIntegralLimit) &&
            writeConfig(
                "launch_fan:racing_duty=%.6f;fan_ramp_ms=%lu;launch_cmd=%.6f;launch_max=%.6f;launch_ramp_ms=%lu",
                Config::kRacingFanDutyCycle,
                static_cast<unsigned long>(Config::kRacingFanRampMs),
                Config::kWheelRestLaunchDriveCommand,
                Config::kWheelRestLaunchMaxDriveCommand,
                static_cast<unsigned long>(Config::kWheelRestLaunchRampMs)) &&
            writeConfig(
                "heading_wall:straight_kp=%.6f;straight_d=%.6f;wall_kp=%.6f;wall_d=%.6f;arc_kp=%.6f;arc_d=%.6f",
                Config::kStraightHeadingKp,
                Config::kStraightYawD,
                Config::kWallCenterGain,
                Config::kWallCenterD,
                Config::kArcHeadingKp,
                Config::kArcYawD) &&
            writeConfig(
                "turn_tol:turn_kp=%.6f;turn_d=%.6f;smooth_kp=%.6f;smooth_kd=%.6f;dist_tol_m=%.6f;ang_tol_rad=%.6f",
                Config::kTurnHeadingKp,
                Config::kTurnYawD,
                Config::kSmoothTurnYawRateKp,
                Config::kSmoothTurnYawRateKd,
                Config::kDistanceToleranceM,
                Config::kAngleToleranceRad) &&
            writeConfig(
                "stall_diag:stall_ms=%lu;stall_grace_ms=%lu;stall_eps_m=%.6f;stall_cmd_mps=%.6f;diag_kp_scale=%.6f;diag_ki_scale=%.6f",
                static_cast<unsigned long>(Config::kEncoderStallTimeoutMs),
                static_cast<unsigned long>(Config::kEncoderStallStartupGraceMs),
                Config::kEncoderProgressEpsilonM,
                Config::kEncoderStallCommandThresholdMps,
                DiagnosticConfig::kDiagnosticWheelVelocityKpScale,
                DiagnosticConfig::kDiagnosticWheelVelocityKiScale) &&
            writeConfig(
                "routine_cfg:startup_ms=%lu;baseline_ms=%lu;inter_ms=%lu;flush_ms=%lu;short_m=%.3f;long_m=%.3f",
                static_cast<unsigned long>(DiagnosticConfig::kStartupSettleMs),
                static_cast<unsigned long>(DiagnosticConfig::kBaselineHoldMs),
                static_cast<unsigned long>(DiagnosticConfig::kInterTestHoldMs),
                static_cast<unsigned long>(DiagnosticConfig::kLogFlushPeriodMs),
                DiagnosticConfig::kShortStraightDistanceM,
                DiagnosticConfig::kLongStraightDistanceM) &&
            writeConfig(
                "characterization:square_leg_m=%.3f;arc_half_m=%.3f;slow_v=%.3f;circle_v=%.3f;fast_v=%.3f;straight_a=%.3f",
                DiagnosticConfig::kSquareLegDistanceM,
                DiagnosticConfig::kArcHalfCircleDistanceM,
                DiagnosticConfig::kSlowStraightSpeedMps,
                DiagnosticConfig::kCircleMediumSpeedMps,
                DiagnosticConfig::kFastStraightSpeedMps,
                DiagnosticConfig::kStraightAccelMps2) &&
            writeConfig(
                "characterization2:straight_d=%.3f;turn_max_w=%.3f;turn_a=%.3f;kickoff_min=%.3f;kickoff_max=%.3f;kickoff_step=%.3f",
                DiagnosticConfig::kStraightDecelMps2,
                inPlaceTurnProfile.maxAngularSpeedRadps,
                inPlaceTurnProfile.angularAccelRadps2,
                DiagnosticConfig::kKickoffSweepMinDriveCommand,
                DiagnosticConfig::kKickoffSweepMaxDriveCommand,
                DiagnosticConfig::kKickoffSweepStepDriveCommand);
    }

    inline bool WriteDiagnosticSummaryInstructions(OptionalRuntimeEventLog& eventLog)
    {
        for (size_t index = 0U; index < MazeMap::GetDiagnosticSummaryInstructionCount(); ++index)
        {
            if (!eventLog.WriteEvent(micros(), "summary", MazeMap::GetDiagnosticSummaryInstruction(index).message))
            {
                return false;
            }
        }
        return true;
    }

    inline void PopulateDiagnosticLogRow(
        DiagnosticLogRow& row,
        unsigned long sampleCount,
        unsigned long phaseId,
        bool stationary,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot)
    {
        row.sample = static_cast<std::uint32_t>(sampleCount);
        row.phase_id = static_cast<std::uint32_t>(phaseId);
        row.t_us = timestampUs;
        row.dt_us = dtUs;
        row.stationary = stationary ? 1U : 0U;
        row.pose_x_m = pose.xMeters;
        row.pose_y_m = pose.yMeters;
        row.yaw_rad = pose.yawRad;
        row.linear_speed_mps = pose.linearSpeedMps;
        row.angular_speed_radps = pose.angularSpeedRadps;
        row.cmd_linear_mps = drive.GetLastLinearCommandMps();
        row.cmd_angular_radps = drive.GetLastAngularCommandRadps();
        row.left_drive_cmd = driveTelemetry.leftDriveCommand;
        row.right_drive_cmd = driveTelemetry.rightDriveCommand;
        row.left_encoder_count = driveTelemetry.leftEncoderCount;
        row.right_encoder_count = driveTelemetry.rightEncoderCount;
        row.left_distance_m = driveTelemetry.leftDistanceM;
        row.right_distance_m = driveTelemetry.rightDistanceM;
        row.left_velocity_mps = driveTelemetry.leftVelocityMps;
        row.right_velocity_mps = driveTelemetry.rightVelocityMps;
        row.imu_fr_status = sensorSnapshot.imuFrontRight.status;
        row.imu_fr_gyro_x = sensorSnapshot.imuFrontRight.gyroX;
        row.imu_fr_gyro_y = sensorSnapshot.imuFrontRight.gyroY;
        row.imu_fr_gyro_z = sensorSnapshot.imuFrontRight.gyroZ;
        row.imu_fr_accel_x = sensorSnapshot.imuFrontRight.accelX;
        row.imu_fr_accel_y = sensorSnapshot.imuFrontRight.accelY;
        row.imu_fr_accel_z = sensorSnapshot.imuFrontRight.accelZ;
        row.imu_fr_temp = sensorSnapshot.imuFrontRight.temp;
        row.imu_fr_int = sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U;
        row.imu_bl_status = sensorSnapshot.imuBackLeft.status;
        row.imu_bl_gyro_x = sensorSnapshot.imuBackLeft.gyroX;
        row.imu_bl_gyro_y = sensorSnapshot.imuBackLeft.gyroY;
        row.imu_bl_gyro_z = sensorSnapshot.imuBackLeft.gyroZ;
        row.imu_bl_accel_x = sensorSnapshot.imuBackLeft.accelX;
        row.imu_bl_accel_y = sensorSnapshot.imuBackLeft.accelY;
        row.imu_bl_accel_z = sensorSnapshot.imuBackLeft.accelZ;
        row.imu_bl_temp = sensorSnapshot.imuBackLeft.temp;
        row.imu_bl_int = sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        row.ws_fl_ambient = sensorSnapshot.frontLeft.ambientLight;
        row.ws_fl_lit = sensorSnapshot.frontLeft.litLight;
        row.ws_fl_delta = sensorSnapshot.frontLeft.differentialLight;
        row.ws_fl_raw_distance_m = sensorSnapshot.frontLeft.rawDistanceM;
        row.ws_fl_distance_m = sensorSnapshot.frontLeft.distanceM;
        row.ws_fr_ambient = sensorSnapshot.frontRight.ambientLight;
        row.ws_fr_lit = sensorSnapshot.frontRight.litLight;
        row.ws_fr_delta = sensorSnapshot.frontRight.differentialLight;
        row.ws_fr_raw_distance_m = sensorSnapshot.frontRight.rawDistanceM;
        row.ws_fr_distance_m = sensorSnapshot.frontRight.distanceM;
        row.ws_sl_ambient = sensorSnapshot.sideLeft.ambientLight;
        row.ws_sl_lit = sensorSnapshot.sideLeft.litLight;
        row.ws_sl_delta = sensorSnapshot.sideLeft.differentialLight;
        row.ws_sl_raw_distance_m = sensorSnapshot.sideLeft.rawDistanceM;
        row.ws_sl_distance_m = sensorSnapshot.sideLeft.distanceM;
        row.ws_sr_ambient = sensorSnapshot.sideRight.ambientLight;
        row.ws_sr_lit = sensorSnapshot.sideRight.litLight;
        row.ws_sr_delta = sensorSnapshot.sideRight.differentialLight;
        row.ws_sr_raw_distance_m = sensorSnapshot.sideRight.rawDistanceM;
        row.ws_sr_distance_m = sensorSnapshot.sideRight.distanceM;
        row.front_wall = sensorSnapshot.frontWall ? 1U : 0U;
        row.left_wall = sensorSnapshot.leftWall ? 1U : 0U;
        row.right_wall = sensorSnapshot.rightWall ? 1U : 0U;
        row.corridor_error_m = sensorSnapshot.corridorErrorM;
        row.front_skew_m = sensorSnapshot.frontSkewM;
        row.gyro_bias_radps = sensorSnapshot.gyroBiasRadps;
        row.gyro_raw_radps = sensorSnapshot.gyroRawRadps;
        row.gyro_radps = sensorSnapshot.gyroRadps;
    }
}

class OpenFloorRunManifestWriter
{
public:
    bool WriteManifest(const char* runId, bool pinsLatchedAtBoot, float batteryVoltageStart, float fanDutyCycleStart)
    {
        MazeMap::CoreFileExport file;
        if (!file.Open(MazeMap::kOpenFloorManifestFileName))
        {
            return false;
        }

        const char* safeRunId = (runId != nullptr) ? runId : "";
        if (!file.Write("{\n")) return false;
        if (!WriteJsonString(file, "format_version", MazeMap::kOpenFloorFormatVersion, true)) return false;
        if (!WriteJsonString(file, "run_id", safeRunId, true)) return false;
        if (!WriteJsonString(file, "firmware_revision", __DATE__ " " __TIME__, true)) return false;
        if (!WriteJsonString(file, "boot_reason", "pins_27_28_shorted_at_boot", true)) return false;
        if (!WriteJsonString(file, "selected_routine", MazeMap::kOpenFloorSelectedRoutineName, true)) return false;
        if (!WriteJsonString(file, "primitive_schedule_revision", MazeMap::kOpenFloorPrimitiveScheduleRevision, true)) return false;
        if (!WriteJsonString(file, "phase_binning_revision", MazeMap::kOpenFloorPhaseBinningRevision, true)) return false;
        if (!WriteJsonString(file, "active_imu_id", MazeMap::kOpenFloorActiveImuId, true)) return false;
        if (!WriteJsonString(file, "imu_extrinsics_revision", MazeMap::kOpenFloorImuExtrinsicsRevision, true)) return false;
        if (!WriteJsonString(file, "start_marker_definitions_revision", MazeMap::kOpenFloorStartMarkerDefinitionsRevision, true)) return false;
        if (!WriteJsonString(file, "logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision, true)) return false;
        if (!WriteJsonBool(file, "pins_27_28_shorted_at_boot", pinsLatchedAtBoot, true)) return false;
        if (!WriteJsonFloat(file, "battery_voltage_start", batteryVoltageStart, true)) return false;
        if (!WriteJsonFanState(file, fanDutyCycleStart, true)) return false;
        if (!WriteJsonStringArray(
                file,
                "files_written",
                {
                    MazeMap::kOpenFloorManifestFileName,
                    MazeMap::kOpenFloorTimingFileName,
                    MazeMap::kOpenFloorMainFileName,
                    MazeMap::App::Internal::Runtime::kRuntimeControlLogFileName
                },
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
    static bool WriteJsonFanState(MazeMap::CoreFileExport& file, float fanDutyCycle, bool trailingComma)
    {
        char line[160] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "  \"fan_state_start\": {\"enabled\": %s, \"duty_cycle\": %.3f}%s\n",
            fanDutyCycle > 0.0f ? "true" : "false",
            fanDutyCycle,
            trailingComma ? "," : "");
        return length > 0 && length < static_cast<int>(sizeof(line)) && file.Write(line);
    }

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

namespace MazeMap::App::Internal::Runtime
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

#define AUX_MEASUREMENT_LOG_FIELDS(X)             \
    X(std::uint32_t, sample)                     \
    X(std::uint32_t, phase_id)                   \
    X(std::uint32_t, t_us)                       \
    X(std::uint32_t, dt_us)                      \
    X(std::uint32_t, stationary)                 \
    X(std::uint32_t, fan_enabled)                \
    X(float,         pose_x_m)                   \
    X(float,         pose_y_m)                   \
    X(float,         yaw_rad)                    \
    X(float,         linear_speed_mps)           \
    X(float,         angular_speed_radps)        \
    X(float,         planar_accel_mps2)          \
    X(float,         cmd_linear_mps)             \
    X(float,         cmd_angular_radps)          \
    X(float,         left_drive_cmd)             \
    X(float,         right_drive_cmd)            \
    X(std::int32_t,  left_encoder_count)         \
    X(std::int32_t,  right_encoder_count)        \
    X(float,         left_distance_m)            \
    X(float,         right_distance_m)           \
    X(float,         left_velocity_mps)          \
    X(float,         right_velocity_mps)         \
    X(std::uint32_t, imu_fr_status)              \
    X(std::int32_t,  imu_fr_gyro_x)              \
    X(std::int32_t,  imu_fr_gyro_y)              \
    X(std::int32_t,  imu_fr_gyro_z)              \
    X(std::int32_t,  imu_fr_accel_x)             \
    X(std::int32_t,  imu_fr_accel_y)             \
    X(std::int32_t,  imu_fr_accel_z)             \
    X(std::int32_t,  imu_fr_temp)                \
    X(std::uint32_t, imu_fr_int)                 \
    X(std::uint32_t, imu_bl_status)              \
    X(std::int32_t,  imu_bl_gyro_x)              \
    X(std::int32_t,  imu_bl_gyro_y)              \
    X(std::int32_t,  imu_bl_gyro_z)              \
    X(std::int32_t,  imu_bl_accel_x)             \
    X(std::int32_t,  imu_bl_accel_y)             \
    X(std::int32_t,  imu_bl_accel_z)             \
    X(std::int32_t,  imu_bl_temp)                \
    X(std::uint32_t, imu_bl_int)                 \
    X(float,         ws_fl_ambient)              \
    X(float,         ws_fl_lit)                  \
    X(float,         ws_fl_delta)                \
    X(float,         ws_fl_raw_distance_m)       \
    X(float,         ws_fl_distance_m)           \
    X(float,         ws_fr_ambient)              \
    X(float,         ws_fr_lit)                  \
    X(float,         ws_fr_delta)                \
    X(float,         ws_fr_raw_distance_m)       \
    X(float,         ws_fr_distance_m)           \
    X(float,         ws_sl_ambient)              \
    X(float,         ws_sl_lit)                  \
    X(float,         ws_sl_delta)                \
    X(float,         ws_sl_raw_distance_m)       \
    X(float,         ws_sl_distance_m)           \
    X(float,         ws_sr_ambient)              \
    X(float,         ws_sr_lit)                  \
    X(float,         ws_sr_delta)                \
    X(float,         ws_sr_raw_distance_m)       \
    X(float,         ws_sr_distance_m)           \
    X(std::uint32_t, front_wall)                 \
    X(std::uint32_t, left_wall)                  \
    X(std::uint32_t, right_wall)                 \
    X(float,         corridor_error_m)           \
    X(float,         front_skew_m)               \
    X(float,         gyro_bias_radps)            \
    X(float,         gyro_raw_radps)             \
    X(float,         gyro_radps)

MMLOG_DEFINE_ROW(AuxMeasurementLogRow, AUX_MEASUREMENT_LOG_FIELDS);

namespace MazeMap::App::Internal::Runtime
{
    inline void PopulateAuxMeasurementLogRow(
        AuxMeasurementLogRow& row,
        unsigned long sampleCount,
        unsigned long phaseId,
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
        row.sample = static_cast<std::uint32_t>(sampleCount);
        row.phase_id = static_cast<std::uint32_t>(phaseId);
        row.t_us = timestampUs;
        row.dt_us = dtUs;
        row.stationary = stationary ? 1U : 0U;
        row.fan_enabled = fanEnabled ? 1U : 0U;
        row.pose_x_m = pose.xMeters;
        row.pose_y_m = pose.yMeters;
        row.yaw_rad = pose.yawRad;
        row.linear_speed_mps = pose.linearSpeedMps;
        row.angular_speed_radps = pose.angularSpeedRadps;
        row.planar_accel_mps2 = planarAccelMps2;
        row.cmd_linear_mps = drive.GetLastLinearCommandMps();
        row.cmd_angular_radps = drive.GetLastAngularCommandRadps();
        row.left_drive_cmd = driveTelemetry.leftDriveCommand;
        row.right_drive_cmd = driveTelemetry.rightDriveCommand;
        row.left_encoder_count = driveTelemetry.leftEncoderCount;
        row.right_encoder_count = driveTelemetry.rightEncoderCount;
        row.left_distance_m = driveTelemetry.leftDistanceM;
        row.right_distance_m = driveTelemetry.rightDistanceM;
        row.left_velocity_mps = driveTelemetry.leftVelocityMps;
        row.right_velocity_mps = driveTelemetry.rightVelocityMps;
        row.imu_fr_status = sensorSnapshot.imuFrontRight.status;
        row.imu_fr_gyro_x = sensorSnapshot.imuFrontRight.gyroX;
        row.imu_fr_gyro_y = sensorSnapshot.imuFrontRight.gyroY;
        row.imu_fr_gyro_z = sensorSnapshot.imuFrontRight.gyroZ;
        row.imu_fr_accel_x = sensorSnapshot.imuFrontRight.accelX;
        row.imu_fr_accel_y = sensorSnapshot.imuFrontRight.accelY;
        row.imu_fr_accel_z = sensorSnapshot.imuFrontRight.accelZ;
        row.imu_fr_temp = sensorSnapshot.imuFrontRight.temp;
        row.imu_fr_int = sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U;
        row.imu_bl_status = sensorSnapshot.imuBackLeft.status;
        row.imu_bl_gyro_x = sensorSnapshot.imuBackLeft.gyroX;
        row.imu_bl_gyro_y = sensorSnapshot.imuBackLeft.gyroY;
        row.imu_bl_gyro_z = sensorSnapshot.imuBackLeft.gyroZ;
        row.imu_bl_accel_x = sensorSnapshot.imuBackLeft.accelX;
        row.imu_bl_accel_y = sensorSnapshot.imuBackLeft.accelY;
        row.imu_bl_accel_z = sensorSnapshot.imuBackLeft.accelZ;
        row.imu_bl_temp = sensorSnapshot.imuBackLeft.temp;
        row.imu_bl_int = sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        row.ws_fl_ambient = sensorSnapshot.frontLeft.ambientLight;
        row.ws_fl_lit = sensorSnapshot.frontLeft.litLight;
        row.ws_fl_delta = sensorSnapshot.frontLeft.differentialLight;
        row.ws_fl_raw_distance_m = sensorSnapshot.frontLeft.rawDistanceM;
        row.ws_fl_distance_m = sensorSnapshot.frontLeft.distanceM;
        row.ws_fr_ambient = sensorSnapshot.frontRight.ambientLight;
        row.ws_fr_lit = sensorSnapshot.frontRight.litLight;
        row.ws_fr_delta = sensorSnapshot.frontRight.differentialLight;
        row.ws_fr_raw_distance_m = sensorSnapshot.frontRight.rawDistanceM;
        row.ws_fr_distance_m = sensorSnapshot.frontRight.distanceM;
        row.ws_sl_ambient = sensorSnapshot.sideLeft.ambientLight;
        row.ws_sl_lit = sensorSnapshot.sideLeft.litLight;
        row.ws_sl_delta = sensorSnapshot.sideLeft.differentialLight;
        row.ws_sl_raw_distance_m = sensorSnapshot.sideLeft.rawDistanceM;
        row.ws_sl_distance_m = sensorSnapshot.sideLeft.distanceM;
        row.ws_sr_ambient = sensorSnapshot.sideRight.ambientLight;
        row.ws_sr_lit = sensorSnapshot.sideRight.litLight;
        row.ws_sr_delta = sensorSnapshot.sideRight.differentialLight;
        row.ws_sr_raw_distance_m = sensorSnapshot.sideRight.rawDistanceM;
        row.ws_sr_distance_m = sensorSnapshot.sideRight.distanceM;
        row.front_wall = sensorSnapshot.frontWall ? 1U : 0U;
        row.left_wall = sensorSnapshot.leftWall ? 1U : 0U;
        row.right_wall = sensorSnapshot.rightWall ? 1U : 0U;
        row.corridor_error_m = sensorSnapshot.corridorErrorM;
        row.front_skew_m = sensorSnapshot.frontSkewM;
        row.gyro_bias_radps = sensorSnapshot.gyroBiasRadps;
        row.gyro_raw_radps = sensorSnapshot.gyroRawRadps;
        row.gyro_radps = sensorSnapshot.gyroRadps;
    }

    inline bool WriteAuxRoutineConfigEvents(
        OptionalRuntimeEventLog& eventLog,
        AuxMeasurementConfig::Routine routine,
        const MazeMap::VehiclePhysicalModel& vehicleModel)
    {
        char message[256] = {};
        auto writeConfig = [&eventLog, &message](const char* format, auto... args) -> bool
        {
            const int length = snprintf(message, sizeof(message), format, args...);
            if (length <= 0 || length >= static_cast<int>(sizeof(message)))
            {
                return false;
            }
            return eventLog.WriteEvent(micros(), "config", message);
        };

        if (!writeConfig(
                "aux_common:startup_ms=%lu;flush_ms=%lu;pin_a=%lu;pin_b=%lu;track_width_m=%.6f",
                static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kLogFlushPeriodMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinA),
                static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinB),
                Config::kTrackWidthM))
        {
            return false;
        }

        if (!writeConfig(
                "aux_track:tight_r_m=%.6f;tight_w_m=%.6f;wide_r_m=%.6f;wide_w_m=%.6f",
                vehicleModel.arcTrackWidthInterpolation.tightRadiusM,
                vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM,
                vehicleModel.arcTrackWidthInterpolation.wideRadiusM,
                vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM))
        {
            return false;
        }

        if (routine == AuxMeasurementConfig::Routine::FanStaticSurvey)
        {
            return writeConfig(
                "fan_static:baseline_ms=%lu;fan_ms=%lu;recovery_ms=%lu;fan_duty=%.6f;fan_ramp_ms=%lu",
                static_cast<unsigned long>(AuxMeasurementConfig::kBaselineHoldMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kFanHoldMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kRecoveryHoldMs),
                Config::kRacingFanDutyCycle,
                static_cast<unsigned long>(Config::kRacingFanRampMs));
        }

        return
            writeConfig(
                "turning_sweep:dir=%s;radius_m=%.6f;start_v_mps=%.6f;accel_mps2=%.6f;max_v_mps=%.6f",
                AuxMeasurementConfig::kTurningTractionSweepClockwise ? "cw" : "ccw",
                AuxMeasurementConfig::kTurningTractionSweepRadiusM,
                AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps,
                AuxMeasurementConfig::kTurningTractionSweepAccelMps2,
                AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps) &&
            writeConfig(
                "turning_limits:fan_settle_ms=%lu;launch_ms=%lu;max_w_radps=%.6f;plateau_v_mps=%.6f;plateau_dv_mps=%.6f",
                static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepFanSettleMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionLaunchMs),
                AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps,
                AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps,
                AuxMeasurementConfig::kTurningTractionPlateauDeltaMps) &&
            writeConfig(
                "turning_slip:plateau_ms=%lu;ceiling_cmd=%.6f;curv_ramp_m_invps=%.6f;slip_v_mps=%.6f;slip_lat_mps2=%.6f",
                static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionPlateauWindowMs),
                AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand,
                AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec,
                AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps,
                AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2) &&
            writeConfig(
                "turning_slip2:yaw_floor=%.6f;planar_floor=%.6f;confirm_ms=%lu;timeout_ms=%lu;fan_duty=%.6f",
                AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor,
                AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor,
                static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSlipConfirmMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepTimeoutMs),
                Config::kRacingFanDutyCycle);
    }
}




