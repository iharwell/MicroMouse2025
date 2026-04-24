#pragma once
// Declares runtime logging, telemetry, and measurement-capture infrastructure for the MazeMap application runtime.
#include "CellCoordinates.h"
#include "Direction.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"

// Private application infrastructure helpers for the MazeMap runtime.

class RuntimeSensorSuite;
namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;
}

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
    EXPORT bool BeginDiagnosticUtilityTelemetryLog(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        RuntimeSensorSuite& sensors,
        DiagnosticLogRow& row,
        const char* fileName,
        const char* modeName,
        unsigned long& phaseId,
        unsigned long& sampleCount);

    inline void PopulateDiagnosticLogRow(
        DiagnosticLogRow& row,
        const std::uint32_t sample,
        const std::uint32_t phaseId,
        const bool stationary,
        const LoopController& loopController,
        const MazeMap::VehicleState& state,
        const DriveBase& drive)
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = drive.GetTelemetry();
        row = {};
        row.sample = sample;
        row.phase_id = phaseId;
        row.t_us = loopController.CurrentTickStartUs();
        row.dt_us = loopController.CurrentTickDtUs();
        row.stationary = stationary ? 1U : 0U;
        row.pose_x_m = state.GetPositionX();
        row.pose_y_m = state.GetPositionY();
        row.yaw_rad = state.GetOrientation();
        row.linear_speed_mps = state.GetVelocity();
        row.angular_speed_radps = state.GetRotationalVelocity();
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
        row.imu_fr_status = sensors.imuFrontRight.status;
        row.imu_fr_gyro_x = sensors.imuFrontRight.gyroX;
        row.imu_fr_gyro_y = sensors.imuFrontRight.gyroY;
        row.imu_fr_gyro_z = sensors.imuFrontRight.gyroZ;
        row.imu_fr_accel_x = sensors.imuFrontRight.accelX;
        row.imu_fr_accel_y = sensors.imuFrontRight.accelY;
        row.imu_fr_accel_z = sensors.imuFrontRight.accelZ;
        row.imu_fr_temp = sensors.imuFrontRight.temp;
        row.imu_fr_int = sensors.imuFrontRight.interruptHigh ? 1U : 0U;
        row.imu_bl_status = sensors.imuBackLeft.status;
        row.imu_bl_gyro_x = sensors.imuBackLeft.gyroX;
        row.imu_bl_gyro_y = sensors.imuBackLeft.gyroY;
        row.imu_bl_gyro_z = sensors.imuBackLeft.gyroZ;
        row.imu_bl_accel_x = sensors.imuBackLeft.accelX;
        row.imu_bl_accel_y = sensors.imuBackLeft.accelY;
        row.imu_bl_accel_z = sensors.imuBackLeft.accelZ;
        row.imu_bl_temp = sensors.imuBackLeft.temp;
        row.imu_bl_int = sensors.imuBackLeft.interruptHigh ? 1U : 0U;
        row.ws_fl_ambient = sensors.frontLeft.ambientLight;
        row.ws_fl_lit = sensors.frontLeft.litLight;
        row.ws_fl_delta = sensors.frontLeft.differentialLight;
        row.ws_fl_raw_distance_m = sensors.frontLeft.rawDistanceM;
        row.ws_fl_distance_m = sensors.frontLeft.distanceM;
        row.ws_fr_ambient = sensors.frontRight.ambientLight;
        row.ws_fr_lit = sensors.frontRight.litLight;
        row.ws_fr_delta = sensors.frontRight.differentialLight;
        row.ws_fr_raw_distance_m = sensors.frontRight.rawDistanceM;
        row.ws_fr_distance_m = sensors.frontRight.distanceM;
        row.ws_sl_ambient = sensors.sideLeft.ambientLight;
        row.ws_sl_lit = sensors.sideLeft.litLight;
        row.ws_sl_delta = sensors.sideLeft.differentialLight;
        row.ws_sl_raw_distance_m = sensors.sideLeft.rawDistanceM;
        row.ws_sl_distance_m = sensors.sideLeft.distanceM;
        row.ws_sr_ambient = sensors.sideRight.ambientLight;
        row.ws_sr_lit = sensors.sideRight.litLight;
        row.ws_sr_delta = sensors.sideRight.differentialLight;
        row.ws_sr_raw_distance_m = sensors.sideRight.rawDistanceM;
        row.ws_sr_distance_m = sensors.sideRight.distanceM;
        row.front_wall = sensors.frontWall ? 1U : 0U;
        row.left_wall = sensors.leftWall ? 1U : 0U;
        row.right_wall = sensors.rightWall ? 1U : 0U;
        row.corridor_error_m = sensors.corridorErrorM;
        row.front_skew_m = sensors.frontSkewM;
        row.gyro_bias_radps = sensors.gyroBiasRadps;
        row.gyro_raw_radps = sensors.gyroRawRadps;
        row.gyro_radps = sensors.gyroRadps;
    }

    inline bool WriteMmLogAccelBiasMetadata(
        MazeMap::mmlog::MmLogLogger& log,
        const RuntimeSensorSuite& sensors)
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

    template <typename WriteEventFn>
    inline bool WriteDiagnosticTuningEvents(WriteEventFn&& writeEvent)
    {
        const auto& driveModel = MazeMap::MotorEncoderDrive::GetSharedPhysicalModel();
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        char message[256] = {};
        auto writeConfig = [&writeEvent, &message](const char* format, auto... args) -> bool
        {
            const int length = snprintf(message, sizeof(message), format, args...);
            if (length <= 0 || length >= static_cast<int>(sizeof(message)))
            {
                return false;
            }
            return writeEvent("config", message);
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
                "routine_cfg:startup_ms=%lu;baseline_ms=%lu;inter_ms=%lu;flush_ms=%lu;static_ms=%lu;recovery_v=%.3f",
                static_cast<unsigned long>(DiagnosticConfig::kStartupSettleMs),
                static_cast<unsigned long>(DiagnosticConfig::kBaselineHoldMs),
                static_cast<unsigned long>(DiagnosticConfig::kInterTestHoldMs),
                static_cast<unsigned long>(DiagnosticConfig::kLogFlushPeriodMs),
                static_cast<unsigned long>(DiagnosticConfig::kStaticHoldMs),
                DiagnosticConfig::kCharacterizationRecoverySpeedMps) &&
            writeConfig(
                "open_floor_repeats:launch=%u;straight=%u;yaw=%u;smooth=%u;loop=%u",
                static_cast<unsigned>(DiagnosticConfig::kLaunchRepeatsPerMagnitude),
                static_cast<unsigned>(DiagnosticConfig::kStraightRepeatsPerSpeed),
                static_cast<unsigned>(DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed),
                static_cast<unsigned>(DiagnosticConfig::kSmoothRepeatsPerPrimitiveSpeed),
                static_cast<unsigned>(DiagnosticConfig::kLoopRepeats)) &&
            writeConfig(
                "open_floor_bins:launch_cmd_start=%.3f;launch_cmd_end=%.3f;launch_cmd_step=%.3f;launch_cmd_count=%u;straight_v=%.3f,%.3f,%.3f;yaw_w=%.3f,%.3f,%.3f",
                MazeMap::kOpenFloorLaunchDriveMagnitudeStart,
                MazeMap::kOpenFloorLaunchDriveMagnitudeEnd,
                MazeMap::kOpenFloorLaunchDriveMagnitudeStep,
                static_cast<unsigned>(MazeMap::kOpenFloorLaunchDriveMagnitudeCount),
                MazeMap::kOpenFloorStraightSpeedBinsMps[0],
                MazeMap::kOpenFloorStraightSpeedBinsMps[1],
                MazeMap::kOpenFloorStraightSpeedBinsMps[2],
                MazeMap::kOpenFloorYawOmegaBinsRadps[0],
                MazeMap::kOpenFloorYawOmegaBinsRadps[1],
                MazeMap::kOpenFloorYawOmegaBinsRadps[2]) &&
            writeConfig(
                "open_floor_motion:smooth_v=%.3f,%.3f,%.3f;straight_a=%.3f;straight_d=%.3f;turn_max_w=%.3f;turn_a=%.3f;launch_ms=%lu;post_segment_hold_ms=%lu",
                MazeMap::kOpenFloorSmoothSpeedBinsMps[0],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[1],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[2],
                DiagnosticConfig::kStraightAccelMps2,
                DiagnosticConfig::kStraightDecelMps2,
                DiagnosticConfig::kTurnMaxOmegaRadps,
                DiagnosticConfig::kTurnAccelRadps2,
                static_cast<unsigned long>(MazeMap::kOpenFloorLaunchPulseMs),
                static_cast<unsigned long>(MazeMap::kOpenFloorPostSegmentHoldMs));
    }

    template <typename WriteEventFn>
    inline bool WriteDiagnosticSummaryInstructions(WriteEventFn&& writeEvent)
    {
        for (size_t index = 0U; index < MazeMap::GetDiagnosticSummaryInstructionCount(); ++index)
        {
            if (!writeEvent("summary", MazeMap::GetDiagnosticSummaryInstruction(index).message))
            {
                return false;
            }
        }
        return true;
    }

}

inline bool IsInterRunServiceJumperInstalled()
{
    return IsPinPairStrapped(Config::kInterRunServicePinA, Config::kInterRunServicePinB);
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
}

inline const char* AuxMeasurementRoutineName(AuxMeasurementConfig::Routine routine)
{
    switch (routine)
    {
    case AuxMeasurementConfig::Routine::FanStaticSurvey:
        return "fan_static_survey";
    case AuxMeasurementConfig::Routine::TurningTractionSweep:
        return "turning_traction_sweep";
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
}




