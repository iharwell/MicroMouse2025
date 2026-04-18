#pragma once
// Declares runtime logging, telemetry, and measurement-capture infrastructure for the MazeMap application runtime.
#include "CellCoordinates.h"
#include "Direction.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
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
        const LoopController::ModeState& state,
        const DriveBase& drive)
    {
        row = {};
        row.sample = sample;
        row.phase_id = phaseId;
        row.t_us = state.tickStartUs;
        row.dt_us = state.dtUs;
        row.stationary = stationary ? 1U : 0U;
        row.pose_x_m = state.estimate.xMeters;
        row.pose_y_m = state.estimate.yMeters;
        row.yaw_rad = state.estimate.yawRad;
        row.linear_speed_mps = state.estimate.linearSpeedMps;
        row.angular_speed_radps = state.estimate.angularSpeedRadps;
        row.cmd_linear_mps = drive.GetLastLinearCommandMps();
        row.cmd_angular_radps = drive.GetLastAngularCommandRadps();
        row.left_drive_cmd = state.driveTelemetry.leftDriveCommand;
        row.right_drive_cmd = state.driveTelemetry.rightDriveCommand;
        row.left_encoder_count = state.driveTelemetry.leftEncoderCount;
        row.right_encoder_count = state.driveTelemetry.rightEncoderCount;
        row.left_distance_m = state.driveTelemetry.leftDistanceM;
        row.right_distance_m = state.driveTelemetry.rightDistanceM;
        row.left_velocity_mps = state.driveTelemetry.leftVelocityMps;
        row.right_velocity_mps = state.driveTelemetry.rightVelocityMps;
        row.imu_fr_status = state.sensors.imuFrontRight.status;
        row.imu_fr_gyro_x = state.sensors.imuFrontRight.gyroX;
        row.imu_fr_gyro_y = state.sensors.imuFrontRight.gyroY;
        row.imu_fr_gyro_z = state.sensors.imuFrontRight.gyroZ;
        row.imu_fr_accel_x = state.sensors.imuFrontRight.accelX;
        row.imu_fr_accel_y = state.sensors.imuFrontRight.accelY;
        row.imu_fr_accel_z = state.sensors.imuFrontRight.accelZ;
        row.imu_fr_temp = state.sensors.imuFrontRight.temp;
        row.imu_fr_int = state.sensors.imuFrontRight.interruptHigh ? 1U : 0U;
        row.imu_bl_status = state.sensors.imuBackLeft.status;
        row.imu_bl_gyro_x = state.sensors.imuBackLeft.gyroX;
        row.imu_bl_gyro_y = state.sensors.imuBackLeft.gyroY;
        row.imu_bl_gyro_z = state.sensors.imuBackLeft.gyroZ;
        row.imu_bl_accel_x = state.sensors.imuBackLeft.accelX;
        row.imu_bl_accel_y = state.sensors.imuBackLeft.accelY;
        row.imu_bl_accel_z = state.sensors.imuBackLeft.accelZ;
        row.imu_bl_temp = state.sensors.imuBackLeft.temp;
        row.imu_bl_int = state.sensors.imuBackLeft.interruptHigh ? 1U : 0U;
        row.ws_fl_ambient = state.sensors.frontLeft.ambientLight;
        row.ws_fl_lit = state.sensors.frontLeft.litLight;
        row.ws_fl_delta = state.sensors.frontLeft.differentialLight;
        row.ws_fl_raw_distance_m = state.sensors.frontLeft.rawDistanceM;
        row.ws_fl_distance_m = state.sensors.frontLeft.distanceM;
        row.ws_fr_ambient = state.sensors.frontRight.ambientLight;
        row.ws_fr_lit = state.sensors.frontRight.litLight;
        row.ws_fr_delta = state.sensors.frontRight.differentialLight;
        row.ws_fr_raw_distance_m = state.sensors.frontRight.rawDistanceM;
        row.ws_fr_distance_m = state.sensors.frontRight.distanceM;
        row.ws_sl_ambient = state.sensors.sideLeft.ambientLight;
        row.ws_sl_lit = state.sensors.sideLeft.litLight;
        row.ws_sl_delta = state.sensors.sideLeft.differentialLight;
        row.ws_sl_raw_distance_m = state.sensors.sideLeft.rawDistanceM;
        row.ws_sl_distance_m = state.sensors.sideLeft.distanceM;
        row.ws_sr_ambient = state.sensors.sideRight.ambientLight;
        row.ws_sr_lit = state.sensors.sideRight.litLight;
        row.ws_sr_delta = state.sensors.sideRight.differentialLight;
        row.ws_sr_raw_distance_m = state.sensors.sideRight.rawDistanceM;
        row.ws_sr_distance_m = state.sensors.sideRight.distanceM;
        row.front_wall = state.sensors.frontWall ? 1U : 0U;
        row.left_wall = state.sensors.leftWall ? 1U : 0U;
        row.right_wall = state.sensors.rightWall ? 1U : 0U;
        row.corridor_error_m = state.sensors.corridorErrorM;
        row.front_skew_m = state.sensors.frontSkewM;
        row.gyro_bias_radps = state.sensors.gyroBiasRadps;
        row.gyro_raw_radps = state.sensors.gyroRawRadps;
        row.gyro_radps = state.sensors.gyroRadps;
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
                "open_floor_motion:smooth_v=%.3f,%.3f,%.3f;straight_a=%.3f;straight_d=%.3f;turn_max_w=%.3f;turn_a=%.3f;launch_ms=%lu;launch_settle_ms=%lu;straight_yaw_settle_ms=%lu",
                MazeMap::kOpenFloorSmoothSpeedBinsMps[0],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[1],
                MazeMap::kOpenFloorSmoothSpeedBinsMps[2],
                DiagnosticConfig::kStraightAccelMps2,
                DiagnosticConfig::kStraightDecelMps2,
                DiagnosticConfig::kTurnMaxOmegaRadps,
                DiagnosticConfig::kTurnAccelRadps2,
                static_cast<unsigned long>(MazeMap::kOpenFloorLaunchPulseMs),
                static_cast<unsigned long>(MazeMap::kOpenFloorLaunchSettleMs),
                static_cast<unsigned long>(MazeMap::kOpenFloorLaunchSettleMs));
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

    struct WallTouchLoopState final
    {
        MazeMap::CellCoordinates wallCell{};
        MazeMap::Direction wallDirection{ MazeMap::None };
        CalibrationWall calibrationWall{ CalibrationWall::West };
        float targetCoordinateM{};
        float expectedTravelM{};
        float targetYawRad{};
        float minLatchTravelM{};
        float maxApproachTravelM{};
        bool allowPassThroughNoWall{};
        WallTouchExecutionResult result{};
        DriveTelemetry lastMotionTelemetry{};
        unsigned long touchStartMs{};
        unsigned long stateStartMs{};
        unsigned long contactCandidateStartMs{};
        unsigned long contactConfirmedStartMs{};
        unsigned long frontSignalMissingStartMs{};
        unsigned long lastMotionMs{};
        float poseResetXMeters{};
        float poseResetYMeters{};
        float poseResetYawRad{};
        float startDistanceM{};
        float approachDriveCommand{};
        float ditherTurnFraction{};
        float previousCycleFrontSkewMagnitudeM{};
        float currentCycleStartYawRad{};
        float currentCycleMaxFrontSkewMagnitudeM{};
        float currentCycleMaxResidualYawRateRadps{};
        bool currentCycleFrontSignalValid{};
        bool haveSquareSample{};
        unsigned long lastHalfCycleIndex{};
        float lastSquareYawRad{};
        float lastSquareFrontSkewM{};
        float lastSquareYawRateRadps{};
        bool lastSquareFrontSignalValid{};
        uint8_t completedHalfCycles{};
        uint8_t consecutiveGoodFullCycles{};
        bool poseResetEnabled{};
        bool launchBaselineCaptured{};
        bool contactCandidateActive{};
        bool seatedResetApplied{};
        WallTouchState runtimeState{ WallTouchState::EntryConditioning };
    };

    struct WallTouchLoopHooks final
    {
        using AppendTraceLineFn = void (*)(void* context, const char* line) noexcept;
        using BeginPassThroughSettleFn = LoopController::ControlVector (*)(
            void* context,
            void* rawState,
            LoopController::TickServices& services);
        using OnPoseResetFn = void (*)(void* context) noexcept;
        using FaultFn = LoopController::ControlVector (*)(
            void* context,
            LoopController::TickServices& services,
            const char* reason) noexcept;
        using CompleteFn = LoopController::ControlVector (*)(void* context, LoopController::TickServices& services);

        void* context{};
        AppendTraceLineFn appendTraceLine{};
        BeginPassThroughSettleFn beginPassThroughSettle{};
        OnPoseResetFn onPoseReset{};
        FaultFn fault{};
        CompleteFn complete{};
    };

    EXPORT LoopController::ControlVector DriveSharedWallTouchLoopTick(
        DriveBase& drive,
        void* rawState,
        WallTouchLoopState& wallTouch,
        const LoopController::ModeState& state,
        LoopController::TickServices& services,
        const WallTouchLoopHooks& hooks,
        MazeMap::CommandPD trackingCommandPd);

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
}




