#pragma once
// Declares runtime logging, telemetry, and measurement-capture infrastructure for the MazeMap application runtime.
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "RuntimeBinaryLogSupport.h"

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
                "open_floor_motion:smooth_v=%.3f,%.3f,%.3f;straight_a=%.3f;straight_d=%.3f;turn_max_w=%.3f;turn_a=%.3f;launch_ms=%lu",
                MazeMap::kOpenFloorSmoothSpeedBinsMps[0],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[1],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[2],
                DiagnosticConfig::kStraightAccelMps2,
                DiagnosticConfig::kStraightDecelMps2,
                DiagnosticConfig::kTurnMaxOmegaRadps,
                DiagnosticConfig::kTurnAccelRadps2,
                static_cast<unsigned long>(MazeMap::kOpenFloorLaunchPulseMs));
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

}




