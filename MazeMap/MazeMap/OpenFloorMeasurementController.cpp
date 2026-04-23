#include "pch.h"
#include "OpenFloorMeasurementController.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeMmLog.h"
#include "ManeuverQueue.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SigmaPointSetSimplex.h"
#include "StartupCalibration.h"
#include "VehicleState.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace MazeMap::App::Internal::Runtime
{
#define OPEN_FLOOR_TIMING_FIELDS(X)              \
    X(std::uint32_t, mono_time_us)              \
    X(std::uint32_t, control_tick_sequence)     \
    X(std::uint32_t, dt_us)                     \
    X(std::uint32_t, section_id)                \
    X(std::uint32_t, control_start_us)          \
    X(std::uint32_t, control_end_us)            \
    X(std::uint32_t, pwm_latch_us)              \
    X(std::uint32_t, encoder_latch_us)          \
    X(std::uint32_t, encoder_read_done_us)      \
    X(std::uint32_t, ukf_predict_start_us)      \
    X(std::uint32_t, ukf_predict_end_us)        \
    X(std::uint32_t, ukf_predict_duration_us)   \
    X(std::uint32_t, ukf_update_start_us)       \
    X(std::uint32_t, ukf_update_end_us)         \
    X(std::uint32_t, ukf_update_duration_us)    \
    X(std::uint32_t, imu_drdy_us)               \
    X(std::uint32_t, imu_read_start_us)         \
    X(std::uint32_t, imu_read_done_us)          \
    X(std::uint32_t, front_led_on_us)           \
    X(std::uint32_t, front_adc_on_us)           \
    X(std::uint32_t, front_led_off_us)          \
    X(std::uint32_t, front_adc_off_us)          \
    X(std::uint32_t, front_ready_us)            \
    X(std::uint32_t, left_led_on_us)            \
    X(std::uint32_t, left_adc_on_us)            \
    X(std::uint32_t, left_led_off_us)           \
    X(std::uint32_t, left_adc_off_us)           \
    X(std::uint32_t, left_ready_us)             \
    X(std::uint32_t, right_led_on_us)           \
    X(std::uint32_t, right_adc_on_us)           \
    X(std::uint32_t, right_led_off_us)          \
    X(std::uint32_t, right_adc_off_us)          \
    X(std::uint32_t, right_ready_us)            \
    X(std::uint32_t, wall_adc_cfg_before_start) \
    X(std::uint32_t, wall_adc_gc_before_start)  \
    X(std::uint32_t, wall_adc_cfg_after_start)  \
    X(std::uint32_t, wall_adc_gc_after_start)   \
    X(std::uint32_t, wall_adc_target_cfg)       \
    X(std::uint32_t, wall_adc_ipg_clock_hz)     \
    X(std::uint32_t, cycle_counter_start)       \
    X(std::uint32_t, cycle_counter_end)

    MMLOG_DEFINE_ROW(OpenFloorTimingRow, OPEN_FLOOR_TIMING_FIELDS);

#define OPEN_FLOOR_MAIN_FIELDS(X)                   \
    X(std::uint32_t, master_time_us)               \
    X(std::uint32_t, control_tick_sequence)        \
    X(std::uint32_t, dt_us)                        \
    X(std::uint8_t,  section_id)                   \
    X(std::uint8_t,  primitive_id)                 \
    X(std::uint8_t,  primitive_family)             \
    X(std::uint8_t,  direction_id)                 \
    X(std::uint8_t,  phase_id)                     \
    X(std::uint8_t,  speed_bin)                    \
    X(std::uint8_t,  start_marker_id)              \
    X(std::uint16_t, repeat_index)                 \
    X(float,         progress_norm)                \
    X(std::uint16_t, mode_flags)                   \
    X(std::uint32_t, clipping_flags)               \
    X(std::uint16_t, saturation_flags)             \
    X(std::uint8_t,  ukf_mode_id)                  \
    X(std::uint8_t,  ukf_yaw_valid_for_feedforward)\
    X(std::uint8_t,  bias_update_enabled)          \
    X(float,         ukf_state_px_m)               \
    X(float,         ukf_state_py_m)               \
    X(float,         ukf_state_psi_rad)            \
    X(float,         ukf_state_u_mps)              \
    X(float,         ukf_state_v_mps)              \
    X(float,         ukf_state_r_radps)            \
    X(float,         ukf_state_omega_l_radps)      \
    X(float,         ukf_state_omega_r_radps)      \
    X(float,         ukf_state_bgz_radps)          \
    X(float,         gyro_bias_anchor_radps)       \
    X(float,         yaw_consistency_lp_radps)     \
    X(float,         yaw_window_mismatch_rad)      \
    X(float,         nhc_sigma_mps)                \
    X(float,         nhc_residual_mps)             \
    X(float,         nhc_residual_sigma)           \
    X(float,         measured_linear_speed_mps)    \
    X(float,         measured_angular_speed_radps) \
    X(float,         cmd_linear_mps)               \
    X(float,         cmd_angular_radps)            \
    X(float,         left_drive_command)           \
    X(float,         right_drive_command)          \
    X(float,         left_feedforward_command)     \
    X(float,         right_feedforward_command)    \
    X(float,         left_feedback_command)        \
    X(float,         right_feedback_command)       \
    X(float,         left_target_velocity_mps)     \
    X(float,         right_target_velocity_mps)    \
    X(float,         left_launch_assist_floor)     \
    X(float,         right_launch_assist_floor)    \
    X(std::uint32_t, encoder_timestamp_us)         \
    X(std::int32_t,  left_encoder_count)           \
    X(std::int32_t,  right_encoder_count)          \
    X(float,         left_encoder_omega_radps)     \
    X(float,         right_encoder_omega_radps)    \
    X(float,         left_encoder_distance_m)      \
    X(float,         right_encoder_distance_m)     \
    X(float,         left_encoder_velocity_mps)    \
    X(float,         right_encoder_velocity_mps)   \
    X(std::uint32_t, imu_timestamp_us)             \
    X(std::uint8_t,  imu_status)                   \
    X(std::uint8_t,  accel_bias_valid)             \
    X(std::int16_t,  imu_gyro_x)                   \
    X(std::int16_t,  imu_gyro_y)                   \
    X(std::int16_t,  imu_gyro_z)                   \
    X(std::int16_t,  imu_accel_x)                  \
    X(std::int16_t,  imu_accel_y)                  \
    X(std::int16_t,  imu_accel_z)                  \
    X(std::int16_t,  imu_temp)                     \
    X(float,         gyro_raw_radps)               \
    X(float,         gyro_bias_radps)              \
    X(float,         gyro_radps)                   \
    X(float,         accel_body_x_mps2)            \
    X(float,         accel_body_y_mps2)            \
    X(float,         planar_accel_mps2)            \
    X(std::uint32_t, front_timestamp_us)           \
    X(std::uint32_t, left_timestamp_us)            \
    X(std::uint32_t, right_timestamp_us)           \
    X(float,         fan_duty_cycle)

    MMLOG_DEFINE_ROW(OpenFloorMainRow, OPEN_FLOOR_MAIN_FIELDS);
}

namespace MazeMap::App::Internal
{
    struct OpenFloorMeasurementLabels final
    {
        MazeMap::OpenFloorSectionId sectionId = MazeMap::OpenFloorSectionId::Sec00Timing;
        MazeMap::OpenFloorMarkerId startMarkerId = MazeMap::OpenFloorMarkerId::C;
        MazeMap::OpenFloorPrimitiveId primitiveId = MazeMap::OpenFloorPrimitiveId::None;
        MazeMap::OpenFloorDirectionId directionId = MazeMap::OpenFloorDirectionId::None;
        MazeMap::OpenFloorPhaseId phaseId = MazeMap::OpenFloorPhaseId::Idle;
        MazeMap::OpenFloorSpeedBin speedBin = MazeMap::OpenFloorSpeedBin::None;
        std::uint16_t repeatIndex = 0U;
        float progressNorm = 0.0f;
        bool abortMarker = false;
    };

    struct CommandTelemetrySnapshot final
    {
        DriveTelemetry driveTelemetry{};
        float linearCommandMps{};
        float angularCommandRadps{};
    };
}

namespace
{
    using MazeMap::App::Internal::OpenFloorMeasurementLabels;
    using MazeMap::App::Internal::CommandTelemetrySnapshot;
    using MazeMap::App::Internal::Drive;
    using MazeMap::App::Internal::LoopController;
    using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
    using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

    constexpr const char* kOpenFloorMeasurementSelectorRemovedReason =
        "Open-floor measurement selector jumper removed";
    constexpr float kOpenFloorMeasurementMaxSmoothSpeedMps = MazeMap::kOpenFloorSmoothSpeedBinsMps[2];
    constexpr MazeMap::ManeuverCode kOpenFloorMeasurementSpeedChangeStraightCode = MazeMap::S1;

    constexpr std::array<MazeMap::ManeuverCode, 26U> kOpenFloorMeasurementSmoothCycle = {
        MazeMap::S135LS,
        MazeMap::S90SD,
        MazeMap::S90SD_M,
        MazeMap::S135LD_M,
        MazeMap::S135LS_M,
        MazeMap::S135LD,
        MazeMap::S135SS,
        MazeMap::S45LD,
        MazeMap::S135SS_M,
        MazeMap::S45LD_M,
        MazeMap::S180LS_M,
        MazeMap::S45LS,
        MazeMap::S135SD,
        MazeMap::S45LS_M,
        MazeMap::S135SD_M,
        MazeMap::S45SS_M,
        MazeMap::S45SD_M,
        MazeMap::S90LS_M,
        MazeMap::S180LS,
        MazeMap::S45SS,
        MazeMap::S45SD,
        MazeMap::S90SS,
        MazeMap::S90LS,
        MazeMap::S180SS_M,
        MazeMap::S90SS_M,
        MazeMap::S180SS,
    };

    MotionLimits BuildOpenFloorMeasurementLimits(
        const MazeMap::Vehicle& vehicle,
        const float maxSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }

    constexpr float OpenFloorMeasurementSmoothSpeedMps(const std::uint8_t speedIndex) noexcept
    {
        return (speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size()) ?
            MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex] :
            kOpenFloorMeasurementMaxSmoothSpeedMps;
    }

    MazeMap::DirectionalLocation OpenFloorMeasurementSmoothQueueStartLocation(
        const std::uint8_t speedIndex) noexcept
    {
        return MazeMap::DirectionalLocation(
            2U,
            static_cast<std::uint8_t>(3U + speedIndex),
            MazeMap::Up);
    }

    bool BuildOpenFloorMeasurementSmoothQueue(
        MazeMap::Vehicle& vehicle,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float initialEntrySpeedMps,
        MazeMap::ManeuverQueue& queue,
        float& exitBoundarySpeedMps)
    {
        queue.clear();
        exitBoundarySpeedMps = 0.0f;

        MazeMap::DirectionalLocation current = OpenFloorMeasurementSmoothQueueStartLocation(speedIndex);
        if (!queue.push_back(kOpenFloorMeasurementSpeedChangeStraightCode, current))
        {
            return false;
        }
        current = queue.back().getEnd();

        for (const MazeMap::ManeuverCode code : kOpenFloorMeasurementSmoothCycle)
        {
            if (!queue.push_back(code, current))
            {
                queue.clear();
                exitBoundarySpeedMps = 0.0f;
                return false;
            }
            current = queue.back().getEnd();
        }

        const bool isLastSpeedBin =
            (speedIndex + 1U) >= MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
        if (isLastSpeedBin && !queue.push_back(kOpenFloorMeasurementSpeedChangeStraightCode, current))
        {
            queue.clear();
            exitBoundarySpeedMps = 0.0f;
            return false;
        }

        const float finalExitSpeedMps = isLastSpeedBin ? 0.0f : cruiseSpeedMps;
        queue.ComputeSpeeds(vehicle, initialEntrySpeedMps, finalExitSpeedMps);
        if (queue.empty())
        {
            return false;
        }

        exitBoundarySpeedMps = queue.back().getExitSpeed();
        return true;
    }

    MazeMap::DirectionalLocation OpenFloorMeasurementLoopQueueStartLocation(const bool clockwise) noexcept
    {
        return clockwise ?
            MazeMap::DirectionalLocation(3U, 3U, MazeMap::Up) :
            MazeMap::DirectionalLocation(7U, 3U, MazeMap::Up);
    }

    bool BuildOpenFloorMeasurementLoopQueue(
        MazeMap::Vehicle& vehicle,
        const bool clockwise,
        MazeMap::ManeuverQueue& queue)
    {
        queue.clear();

        MazeMap::DirectionalLocation current = OpenFloorMeasurementLoopQueueStartLocation(clockwise);
        const MazeMap::ManeuverCode turnCode = clockwise ? MazeMap::IP90 : MazeMap::IP90_M;
        for (std::uint8_t side = 0U; side < 4U; ++side)
        {
            if (!queue.push_back(MazeMap::S2, current))
            {
                queue.clear();
                return false;
            }
            current = queue.back().getEnd();

            if (!queue.push_back(turnCode, current))
            {
                queue.clear();
                return false;
            }
            current = queue.back().getEnd();
        }

        queue.ComputeSpeeds(vehicle, 0.0f, 0.0f);
        return !queue.empty();
    }

    bool WriteOpenFloorV62Metadata(MazeMap::App::Internal::SharedRobotRuntime& runtime)
    {
        const MazeMap::PlantModel::PreparedParams prepared =
            MazeMap::PlantModel::Prepare(MazeMap::PlantParams::Default());
        return
            runtime.WriteUtilityDataLogMetadata("ukfver", "v6.2") &&
            runtime.WriteUtilityDataLogMetadata("ukfset", "splx") &&
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nx",
                static_cast<unsigned long>(MazeMap::VehicleState::kDimension)) &&
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nsig",
                static_cast<unsigned long>(
                    MazeMap::SigmaPointSetSimplex::ActiveSigmaCountForDimension(
                        MazeMap::VehicleState::kDimension))) &&
            runtime.WriteUtilityDataLogMetadataFloat("re_m", prepared.wheelRadiusM, 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("we_m", prepared.trackWidthM, 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("imu_x", prepared.imuPositionBodyM.x(), 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("imu_y", prepared.imuPositionBodyM.y(), 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("jw_kgm2", prepared.wheelInertiaKgM2, 9);
    }

    CommandTelemetrySnapshot BuildRawCommandTelemetrySnapshot(
        const LoopController::ControlVector& control) noexcept
    {
        CommandTelemetrySnapshot snapshot{};
        if (std::isfinite(control.leftMotorPwm) && std::isfinite(control.rightMotorPwm))
        {
            snapshot.driveTelemetry.leftDriveCommand = control.leftMotorPwm;
            snapshot.driveTelemetry.rightDriveCommand = control.rightMotorPwm;
            snapshot.driveTelemetry.leftFeedforwardCommand = control.leftMotorPwm;
            snapshot.driveTelemetry.rightFeedforwardCommand = control.rightMotorPwm;
            snapshot.driveTelemetry.modeFlags = ::DriveBase::kModeRawOpenLoop;
            snapshot.driveTelemetry.saturationFlags =
                ((std::fabs(control.leftMotorPwm) >= 0.999f) ? 0x1u : 0u) |
                ((std::fabs(control.rightMotorPwm) >= 0.999f) ? 0x2u : 0u);
        }
        else
        {
            snapshot.driveTelemetry.modeFlags = ::DriveBase::kModeBraking;
        }

        return snapshot;
    }

    void ApplyControlTimingToTimingRow(
        const ControlCycleTiming& timing,
        OpenFloorTimingRow& row) noexcept
    {
        row.control_start_us = timing.controlStartUs;
        row.control_end_us = timing.controlEndUs;
        row.pwm_latch_us = timing.pwmLatchUs;
        row.encoder_latch_us = timing.encoderLatchUs;
        row.encoder_read_done_us = timing.encoderReadDoneUs;
        row.ukf_predict_start_us = timing.ukfPredictStartUs;
        row.ukf_predict_end_us = timing.ukfPredictEndUs;
        row.ukf_predict_duration_us = timing.ukfPredictDurationUs;
        row.ukf_update_start_us = timing.ukfUpdateStartUs;
        row.ukf_update_end_us = timing.ukfUpdateEndUs;
        row.ukf_update_duration_us = timing.ukfUpdateDurationUs;
        row.cycle_counter_start = timing.cycleCounterStart;
        row.cycle_counter_end = timing.cycleCounterEnd;
    }

    void ApplyControlTimingToMainRow(
        const ControlCycleTiming& timing,
        OpenFloorMainRow& row) noexcept
    {
        row.encoder_timestamp_us = timing.encoderReadDoneUs;
    }
}

namespace MazeMap::App::Internal
{
    class OpenFloorMeasurementController::State final
    {
    public:
        explicit State(SharedRobotRuntime& runtime);

        bool Begin();
        void Run();

    private:
        using ModeWorkCallback = LoopController::ModeWorkCallback;

        enum class PauseAction : std::uint8_t
        {
            None,
            TimingToMain,
        };

        enum class HoldContinuation : std::uint8_t
        {
            None,
            LaunchRepeat,
            Launch,
            StraightRepeat,
            Straight,
            YawRepeat,
            Yaw,
            Smooth,
            LoopCw,
            LoopCcw,
            Complete,
        };

        struct TimingState final
        {
            std::uint16_t tickIndex{};
        };

        struct StaticState final
        {
            bool initialized{};
            std::uint32_t deadlineMs{};
            OpenFloorMeasurementLabels labels{};
        };

        struct HoldState final
        {
            HoldContinuation continuation{ HoldContinuation::None };
        };

        struct LaunchState final
        {
            std::size_t magnitudeIndex{};
            std::uint8_t repeatIteration{};
            bool negativeNext{};
            std::uint16_t nextRepeatIndex{};
            bool active{};
            OpenFloorMeasurementLabels labels{};
            float signedDriveCommand{};
            std::uint32_t pulseDeadlineMs{};
        };

        struct StraightState final
        {
            std::size_t speedIndex{};
            std::uint8_t repeatIteration{};
            bool negativeNext{};
            std::uint16_t nextRepeatIndex{};
            bool active{};
            OpenFloorMeasurementLabels labels{};
            float distanceM{};
            float signedCruiseSpeedMps{};
            float startDistanceM{};
        };

        struct YawState final
        {
            std::size_t speedIndex{};
            std::uint8_t repeatIteration{};
            std::uint8_t primitiveIndex{};
            std::uint16_t nextRepeatIndex{};
            bool active{};
            OpenFloorMeasurementLabels labels{};
            float targetYawRad{};
            float targetMagnitudeRad{};
        };

        struct SmoothState final
        {
            std::uint8_t speedIndex{};
            std::uint16_t entryIndex{};
            bool queueLoaded{};
            bool active{};
            float entryBoundarySpeedMps{};
            float exitBoundarySpeedMps{};
            float startDistanceM{};
            float totalDistanceM{};
            float targetYawRad{};
            float targetMagnitudeRad{};
            OpenFloorMeasurementLabels labels{};
            MazeMap::ManeuverQueue queue{};
        };

        struct LoopState final
        {
            bool clockwise{};
            std::uint8_t repeatIndex{};
            std::uint16_t entryIndex{};
            bool queueLoaded{};
            bool active{};
            float startDistanceM{};
            float totalDistanceM{};
            float targetYawRad{};
            float targetMagnitudeRad{};
            OpenFloorMeasurementLabels labels{};
            MazeMap::ManeuverQueue queue{};
        };

        static LoopController::PauseDisposition PauseThunk(
            void* context,
            const LoopController::PauseContext& pause);
        static LoopController::ControlVector TimingTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector InterPhaseHoldTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector StaticTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector LaunchTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector StraightTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector YawTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector SmoothTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector LoopCwTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector LoopCcwTickThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        static OpenFloorPrimitiveId PrimitiveIdForCode(MazeMap::ManeuverCode code) noexcept;
        static OpenFloorDirectionId DirectionIdForCode(MazeMap::ManeuverCode code) noexcept;
        static OpenFloorSpeedBin SpeedBinForIndex(std::size_t speedIndex) noexcept;
        static OpenFloorPhaseId StraightPhaseForProgress(float progress) noexcept;
        static OpenFloorPhaseId TurnPhaseForProgress(float progress) noexcept;
        static OpenFloorPhaseId SmoothPhaseForProgress(float progress) noexcept;
        static OpenFloorPhaseId ManeuverPhaseForProgress(
            MazeMap::ManeuverCode code,
            float progress) noexcept;

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        void ResetState() noexcept;
        bool BeginTimingLog();
        bool FlushPendingTimingSample(
            LoopController::TickServices& services,
            const char* failureReason);
        void StagePendingTimingSample(const OpenFloorTimingRow& row) noexcept;
        bool BeginMainLog();
        bool FlushPendingMainSample(
            LoopController::TickServices& services,
            const char* failureReason);
        void StagePendingMainSample(const OpenFloorMainRow& row) noexcept;
        void PopulateTimingRowFromState(
            const LoopController::ModeState& state,
            OpenFloorTimingRow& row) const noexcept;
        void PopulateMainRowFromState(
            const OpenFloorMeasurementLabels& labels,
            const LoopController::ModeState& state,
            const CommandTelemetrySnapshot& commandTelemetry,
            OpenFloorMainRow& row) const;
        CommandTelemetrySnapshot CaptureDriveCommandTelemetry(
            const LoopController::ControlVector& control) const noexcept;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        void SwitchPhase(
            LoopController::TickServices& services,
            ModeWorkCallback callback) noexcept;
        void StartInterPhaseHold(HoldContinuation continuation, std::uint16_t durationMs) noexcept;
        bool SwitchToHoldContinuation(LoopController::TickServices& services);
        bool CheckTimingFault(
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        bool CheckMainFault(
            OpenFloorMeasurementLabels& labels,
            LoopController::TickServices& services,
            const LoopController::ModeState& state,
            const char* estimatorReason);
        bool HasRemainingLaunchSamples() const noexcept;
        bool StartNextLaunchSample();
        bool HasRemainingStraightSamples() const noexcept;
        bool StartNextStraightSample();
        bool HasRemainingYawSamples() const noexcept;
        bool StartNextYawSample();
        bool StartNextSmoothEntry();
        void ResetLoopPhase(bool clockwise) noexcept;
        bool StartNextLoopEntry();
        float CurrentManeuverProgress(
            MazeMap::ManeuverCode code,
            float startDistanceM,
            float totalDistanceM,
            float targetYawRad,
            float targetMagnitudeRad) const noexcept;
        Drive::OperationMode OpenFloorOperationMode() const noexcept;
        MotionLimits StraightLimits(float speedMps) const noexcept;
        MotionLimits TurnLimits(float maxOmegaRadps) const noexcept;
        LoopController::PauseDisposition OnPauseGranted(const LoopController::PauseContext& pause);

        LoopController::ControlVector TimingTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector InterPhaseHoldTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector StaticTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector LaunchTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector StraightTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector YawTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector SmoothTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector LoopCwTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector LoopCcwTick(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        RuntimeSensorSuite& _sensors;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        bool _timingLogOpen{};
        bool _mainLogOpen{};
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        PauseAction _pauseAction{ PauseAction::None };
        TimingState _timingState{};
        bool _pendingTimingSampleValid{};
        OpenFloorTimingRow _pendingTimingRow{};
        StaticState _staticState{};
        HoldState _holdState{};
        LaunchState _launchState{};
        StraightState _straightState{};
        YawState _yawState{};
        SmoothState _smoothState{};
        LoopState _loopState{};
        CommandTelemetrySnapshot _savedCommandTelemetry{};
        bool _pendingMainSampleValid{};
        OpenFloorMainRow _pendingMainRow{};
    };

    OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
        : _state(std::make_unique<State>(runtime))
    {
    }

    OpenFloorMeasurementController::~OpenFloorMeasurementController() = default;

    bool OpenFloorMeasurementController::Begin()
    {
        return _state->Begin();
    }

    void OpenFloorMeasurementController::Run()
    {
        _state->Run();
    }

    OpenFloorMeasurementController::State::State(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    bool OpenFloorMeasurementController::State::Begin()
    {
        ResetState();
        if (!SetupHardware())
        {
            return _runtime.FailActiveMode("Open-floor measurement hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:open_floor_measurement");
        (void)_runtime.AppendTextLogLine("Open-floor measurement mode");
        (void)_runtime.AppendTextLogLine(
            "Open-floor battery: timing -> static -> launch -> straight -> yaw -> smooth -> loop cw -> loop ccw");

        if (!_drive.Begin())
        {
            return _runtime.FailActiveMode("Open-floor measurement drive base init failed");
        }
        _drive.UseNominalWheelControlProfile();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        if (!_startupCalibration.BringUp())
        {
            return _runtime.FailActiveMode("Open-floor measurement startup bring-up failed");
        }
        SetMissionLevelFanEnabled(true);

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            return _runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
        }

        if (!BeginTimingLog())
        {
            return _runtime.FailActiveMode("Open-floor measurement timing log setup failed");
        }

        return true;
    }

    void OpenFloorMeasurementController::State::Run()
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &OpenFloorMeasurementController::State::TimingTickThunk;
        callbacks.context = this;

        bool completed = false;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            (void)_runtime.FailActiveMode("Open-floor measurement loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            completed = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }

        if (completed && _mainLogOpen && _pendingMainSampleValid)
        {
            ApplyControlTimingToMainRow(_loopController.LastDiagnostics().controlTiming, _pendingMainRow);
            if (_runtime.LogUtilityDataRow(_pendingMainRow))
            {
                _pendingMainSampleValid = false;
            }
        }

        if (completed && _timingLogOpen && _pendingTimingSampleValid)
        {
            ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _pendingTimingRow);
            if (_runtime.LogUtilityDataRow(_pendingTimingRow))
            {
                _pendingTimingSampleValid = false;
            }
        }

        ReleaseSelectorMonitor();
        _startupCalibration.Cancel();
        _driveService.Cancel();
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        SetMissionLevelFanEnabled(false);

        if (completed)
        {
            (void)_runtime.AppendTextLogLine("Open-floor measurement complete");
        }
    }

    LoopController::SessionOptions OpenFloorMeasurementController::State::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    void OpenFloorMeasurementController::State::ResetState() noexcept
    {
        _driveService.Cancel();
        _startupCalibration.Cancel();
        _timingLogOpen = false;
        _mainLogOpen = false;
        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
        _selectorMonitorArmed = false;
        _pauseAction = PauseAction::None;
        _timingState = {};
        _pendingTimingSampleValid = false;
        _pendingTimingRow = {};
        _staticState = {};
        _holdState = {};
        _launchState = {};
        _straightState = {};
        _yawState = {};
        _smoothState = {};
        _loopState = {};
        _savedCommandTelemetry =
            BuildRawCommandTelemetrySnapshot(LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f));
        _pendingMainSampleValid = false;
        _pendingMainRow = {};
    }

    bool OpenFloorMeasurementController::State::BeginTimingLog()
    {
        if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorTimingFileName))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorTimingStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(_runtime)) return false;

        OpenFloorTimingRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _timingLogOpen = true;
        return true;
    }

    bool OpenFloorMeasurementController::State::FlushPendingTimingSample(
        LoopController::TickServices& services,
        const char* failureReason)
    {
        if (!_pendingTimingSampleValid)
        {
            return true;
        }

        ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _pendingTimingRow);
        if (!_runtime.LogUtilityDataRow(_pendingTimingRow))
        {
            services.Fault(failureReason);
            return false;
        }

        _pendingTimingSampleValid = false;
        _pendingTimingRow = {};
        return true;
    }

    void OpenFloorMeasurementController::State::StagePendingTimingSample(
        const OpenFloorTimingRow& row) noexcept
    {
        _pendingTimingRow = row;
        _pendingTimingSampleValid = true;
    }

    bool OpenFloorMeasurementController::State::BeginMainLog()
    {
        if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorMainFileName))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorMainStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(_runtime)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;

        OpenFloorMainRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _mainLogOpen = true;
        return true;
    }

    bool OpenFloorMeasurementController::State::FlushPendingMainSample(
        LoopController::TickServices& services,
        const char* failureReason)
    {
        if (!_pendingMainSampleValid)
        {
            return true;
        }

        ApplyControlTimingToMainRow(_loopController.LastDiagnostics().controlTiming, _pendingMainRow);
        if (!_runtime.LogUtilityDataRow(_pendingMainRow))
        {
            services.Fault(failureReason);
            return false;
        }

        _pendingMainSampleValid = false;
        _pendingMainRow = {};
        return true;
    }

    void OpenFloorMeasurementController::State::StagePendingMainSample(
        const OpenFloorMainRow& row) noexcept
    {
        _pendingMainRow = row;
        _pendingMainSampleValid = true;
    }

    void OpenFloorMeasurementController::State::PopulateTimingRowFromState(
        const LoopController::ModeState& state,
        OpenFloorTimingRow& row) const noexcept
    {
        row.mono_time_us = state.tickStartUs;
        row.control_tick_sequence = state.sequence;
        row.dt_us = state.dtUs;
        row.section_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
        row.imu_drdy_us = state.sensors.imuTiming.drdyUs;
        row.imu_read_start_us = state.sensors.imuTiming.readStartUs;
        row.imu_read_done_us = state.sensors.imuTiming.readDoneUs;
        row.front_led_on_us = state.sensors.frontTiming.ledOnCommandUs;
        row.front_adc_on_us = state.sensors.frontTiming.adcOnSampleUs;
        row.front_led_off_us = state.sensors.frontTiming.ledOffCommandUs;
        row.front_adc_off_us = state.sensors.frontTiming.adcOffSampleUs;
        row.front_ready_us = state.sensors.frontTiming.observationReadyUs;
        row.left_led_on_us = state.sensors.leftTiming.ledOnCommandUs;
        row.left_adc_on_us = state.sensors.leftTiming.adcOnSampleUs;
        row.left_led_off_us = state.sensors.leftTiming.ledOffCommandUs;
        row.left_adc_off_us = state.sensors.leftTiming.adcOffSampleUs;
        row.left_ready_us = state.sensors.leftTiming.observationReadyUs;
        row.right_led_on_us = state.sensors.rightTiming.ledOnCommandUs;
        row.right_adc_on_us = state.sensors.rightTiming.adcOnSampleUs;
        row.right_led_off_us = state.sensors.rightTiming.ledOffCommandUs;
        row.right_adc_off_us = state.sensors.rightTiming.adcOffSampleUs;
        row.right_ready_us = state.sensors.rightTiming.observationReadyUs;
        row.wall_adc_cfg_before_start = state.sensors.wallSensorAdcCfgBeforeStart;
        row.wall_adc_gc_before_start = state.sensors.wallSensorAdcGcBeforeStart;
        row.wall_adc_cfg_after_start = state.sensors.wallSensorAdcCfgAfterStart;
        row.wall_adc_gc_after_start = state.sensors.wallSensorAdcGcAfterStart;
        row.wall_adc_target_cfg = state.sensors.wallSensorAdcTargetCfg;
        row.wall_adc_ipg_clock_hz = state.sensors.wallSensorAdcIpgClockHz;
    }

    void OpenFloorMeasurementController::State::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        const BootModeRegistryEntry* const entry =
            FindBootModeRegistryEntry(BootModeId::OpenFloorMeasurement);
        if ((entry == nullptr) || (entry->selector.kind != BootModeSelectorKind::PinPair))
        {
            return;
        }

        _selectorDrivePin = entry->selector.pinA;
        _selectorSensePin = entry->selector.pinB;
        BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        _selectorMonitorArmed = true;
    }

    void OpenFloorMeasurementController::State::ReleaseSelectorMonitor() noexcept
    {
        if (_selectorMonitorArmed)
        {
            EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        }
        _selectorMonitorArmed = false;
        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
    }

    bool OpenFloorMeasurementController::State::SelectorRemoved() const noexcept
    {
        return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
    }

    void OpenFloorMeasurementController::State::PopulateMainRowFromState(
        const OpenFloorMeasurementLabels& labels,
        const LoopController::ModeState& state,
        const CommandTelemetrySnapshot& commandTelemetry,
        OpenFloorMainRow& row) const
    {
        const MazeMap::VehicleState::StateVector estimatorState = _drive.GetEstimatorStateVector();

        row.master_time_us = state.tickStartUs;
        row.control_tick_sequence = state.sequence;
        row.dt_us = state.dtUs;
        row.section_id = static_cast<std::uint8_t>(labels.sectionId);
        row.primitive_id = static_cast<std::uint8_t>(labels.primitiveId);
        row.primitive_family = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveFamilyForId(labels.primitiveId));
        row.direction_id = static_cast<std::uint8_t>(labels.directionId);
        row.phase_id = static_cast<std::uint8_t>(labels.phaseId);
        row.speed_bin = static_cast<std::uint8_t>(labels.speedBin);
        row.start_marker_id = static_cast<std::uint8_t>(labels.startMarkerId);
        row.repeat_index = labels.repeatIndex;
        row.progress_norm = labels.progressNorm;
        row.mode_flags = commandTelemetry.driveTelemetry.modeFlags;
        row.clipping_flags = 0U;
        row.saturation_flags = commandTelemetry.driveTelemetry.saturationFlags;
        row.ukf_mode_id = state.driveTelemetry.ukfModeId;
        row.ukf_yaw_valid_for_feedforward = state.driveTelemetry.ukfYawValidForFeedforward;
        row.bias_update_enabled = state.driveTelemetry.ukfBiasUpdateEnabled;
        row.gyro_bias_anchor_radps = state.driveTelemetry.ukfGyroBiasAnchorRadps;
        row.yaw_consistency_lp_radps = state.driveTelemetry.ukfYawConsistencyLowPassRadps;
        row.yaw_window_mismatch_rad = state.driveTelemetry.ukfYawWindowMismatchRad;
        row.nhc_sigma_mps = state.driveTelemetry.ukfNhcSigmaMps;
        row.nhc_residual_mps = state.driveTelemetry.ukfNhcResidualMps;
        row.nhc_residual_sigma = state.driveTelemetry.ukfNhcResidualSigma;
        row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        row.measured_linear_speed_mps = state.measured.linearSpeedMps;
        row.measured_angular_speed_radps = state.measured.angularSpeedRadps;
        row.cmd_linear_mps = commandTelemetry.linearCommandMps;
        row.cmd_angular_radps = commandTelemetry.angularCommandRadps;
        row.left_drive_command = commandTelemetry.driveTelemetry.leftDriveCommand;
        row.right_drive_command = commandTelemetry.driveTelemetry.rightDriveCommand;
        row.left_feedforward_command = commandTelemetry.driveTelemetry.leftFeedforwardCommand;
        row.right_feedforward_command = commandTelemetry.driveTelemetry.rightFeedforwardCommand;
        row.left_feedback_command = commandTelemetry.driveTelemetry.leftFeedbackCommand;
        row.right_feedback_command = commandTelemetry.driveTelemetry.rightFeedbackCommand;
        row.left_target_velocity_mps = commandTelemetry.driveTelemetry.leftTargetVelocityMps;
        row.right_target_velocity_mps = commandTelemetry.driveTelemetry.rightTargetVelocityMps;
        row.left_launch_assist_floor = commandTelemetry.driveTelemetry.leftLaunchAssistFloor;
        row.right_launch_assist_floor = commandTelemetry.driveTelemetry.rightLaunchAssistFloor;
        row.encoder_timestamp_us = 0U;
        row.left_encoder_count = state.driveTelemetry.leftEncoderCount;
        row.right_encoder_count = state.driveTelemetry.rightEncoderCount;
        row.left_encoder_omega_radps = state.driveTelemetry.leftEncoderOmegaRadps;
        row.right_encoder_omega_radps = state.driveTelemetry.rightEncoderOmegaRadps;
        row.left_encoder_distance_m = state.driveTelemetry.leftDistanceM;
        row.right_encoder_distance_m = state.driveTelemetry.rightDistanceM;
        row.left_encoder_velocity_mps = state.driveTelemetry.leftVelocityMps;
        row.right_encoder_velocity_mps = state.driveTelemetry.rightVelocityMps;
        row.imu_timestamp_us = state.sensors.imuTiming.readDoneUs;
        row.imu_status = state.sensors.imuBackLeft.status;
        row.accel_bias_valid = state.sensors.accelBiasValid ? 1U : 0U;
        row.imu_gyro_x = state.sensors.imuBackLeft.gyroX;
        row.imu_gyro_y = state.sensors.imuBackLeft.gyroY;
        row.imu_gyro_z = state.sensors.imuBackLeft.gyroZ;
        row.imu_accel_x = state.sensors.imuBackLeft.accelX;
        row.imu_accel_y = state.sensors.imuBackLeft.accelY;
        row.imu_accel_z = state.sensors.imuBackLeft.accelZ;
        row.imu_temp = state.sensors.imuBackLeft.temp;
        row.gyro_raw_radps = state.sensors.gyroRawRadps;
        row.gyro_bias_radps = state.sensors.gyroBiasRadps;
        row.gyro_radps = state.sensors.gyroRadps;
        row.accel_body_x_mps2 = state.sensors.accelBodyXMps2;
        row.accel_body_y_mps2 = state.sensors.accelBodyYMps2;
        row.planar_accel_mps2 = state.sensors.planarAccelMps2;
        row.front_timestamp_us = state.sensors.frontTiming.observationReadyUs;
        row.left_timestamp_us = state.sensors.leftTiming.observationReadyUs;
        row.right_timestamp_us = state.sensors.rightTiming.observationReadyUs;
        row.fan_duty_cycle = GetMissionFanDutyCycle();
    }

    CommandTelemetrySnapshot OpenFloorMeasurementController::State::CaptureDriveCommandTelemetry(
        const LoopController::ControlVector& control) const noexcept
    {
        CommandTelemetrySnapshot snapshot{};
        snapshot.linearCommandMps = _drive.GetLastLinearCommandMps();
        snapshot.angularCommandRadps = _drive.GetLastAngularCommandRadps();
        snapshot.driveTelemetry = _drive.GetGeneratedTelemetry(control);
        return snapshot;
    }

    OpenFloorPrimitiveId OpenFloorMeasurementController::State::PrimitiveIdForCode(
        const MazeMap::ManeuverCode code) noexcept
    {
        switch (code)
        {
        case MazeMap::S1:
            return OpenFloorPrimitiveId::Str1;
        case MazeMap::S2:
            return OpenFloorPrimitiveId::Str2;
        case MazeMap::IP90:
            return OpenFloorPrimitiveId::Ip90;
        case MazeMap::IP90_M:
            return OpenFloorPrimitiveId::Ip90M;
        case MazeMap::IP180:
            return OpenFloorPrimitiveId::Ip180;
        case MazeMap::IP180_M:
            return OpenFloorPrimitiveId::Ip180M;
        case MazeMap::S45SD:
            return OpenFloorPrimitiveId::S45sd;
        case MazeMap::S45SD_M:
            return OpenFloorPrimitiveId::S45sdM;
        case MazeMap::S45SS:
            return OpenFloorPrimitiveId::S45ss;
        case MazeMap::S45SS_M:
            return OpenFloorPrimitiveId::S45ssM;
        case MazeMap::S45LS:
            return OpenFloorPrimitiveId::S45ls;
        case MazeMap::S45LS_M:
            return OpenFloorPrimitiveId::S45lsM;
        case MazeMap::S45LD:
            return OpenFloorPrimitiveId::S45ld;
        case MazeMap::S45LD_M:
            return OpenFloorPrimitiveId::S45ldM;
        case MazeMap::S90SD:
            return OpenFloorPrimitiveId::S90sd;
        case MazeMap::S90SD_M:
            return OpenFloorPrimitiveId::S90sdM;
        case MazeMap::S90SS:
            return OpenFloorPrimitiveId::S90ss;
        case MazeMap::S90SS_M:
            return OpenFloorPrimitiveId::S90ssM;
        case MazeMap::S90LS:
            return OpenFloorPrimitiveId::S90ls;
        case MazeMap::S90LS_M:
            return OpenFloorPrimitiveId::S90lsM;
        case MazeMap::S135SD:
            return OpenFloorPrimitiveId::S135sd;
        case MazeMap::S135SD_M:
            return OpenFloorPrimitiveId::S135sdM;
        case MazeMap::S135SS:
            return OpenFloorPrimitiveId::S135ss;
        case MazeMap::S135SS_M:
            return OpenFloorPrimitiveId::S135ssM;
        case MazeMap::S135LS:
            return OpenFloorPrimitiveId::S135ls;
        case MazeMap::S135LS_M:
            return OpenFloorPrimitiveId::S135lsM;
        case MazeMap::S135LD:
            return OpenFloorPrimitiveId::S135ld;
        case MazeMap::S135LD_M:
            return OpenFloorPrimitiveId::S135ldM;
        case MazeMap::S180SS:
            return OpenFloorPrimitiveId::S180ss;
        case MazeMap::S180SS_M:
            return OpenFloorPrimitiveId::S180ssM;
        case MazeMap::S180LS:
            return OpenFloorPrimitiveId::S180ls;
        case MazeMap::S180LS_M:
            return OpenFloorPrimitiveId::S180lsM;
        default:
            return OpenFloorPrimitiveId::None;
        }
    }

    OpenFloorDirectionId OpenFloorMeasurementController::State::DirectionIdForCode(
        const MazeMap::ManeuverCode code) noexcept
    {
        switch (code)
        {
        case MazeMap::S1:
        case MazeMap::S2:
            return OpenFloorDirectionId::Positive;
        case MazeMap::IP90:
            return OpenFloorDirectionId::Clockwise;
        case MazeMap::IP90_M:
            return OpenFloorDirectionId::CounterClockwise;
        case MazeMap::IP180:
            return OpenFloorDirectionId::Clockwise;
        case MazeMap::IP180_M:
            return OpenFloorDirectionId::CounterClockwise;
        default:
            break;
        }

        const OpenFloorPrimitiveId primitiveId = PrimitiveIdForCode(code);
        if (primitiveId == OpenFloorPrimitiveId::None)
        {
            return OpenFloorDirectionId::None;
        }

        return OpenFloorPrimitiveIsMirrored(primitiveId) ?
            OpenFloorDirectionId::Left :
            OpenFloorDirectionId::Right;
    }

    OpenFloorSpeedBin OpenFloorMeasurementController::State::SpeedBinForIndex(const std::size_t speedIndex) noexcept
    {
        return (speedIndex == 0U) ? OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? OpenFloorSpeedBin::Medium :
            OpenFloorSpeedBin::High;
    }

    OpenFloorPhaseId OpenFloorMeasurementController::State::StraightPhaseForProgress(const float progress) noexcept
    {
        if (progress < 0.2f)
        {
            return OpenFloorPhaseId::Accel;
        }
        if (progress > 0.85f)
        {
            return OpenFloorPhaseId::Brake;
        }
        return OpenFloorPhaseId::Cruise;
    }

    OpenFloorPhaseId OpenFloorMeasurementController::State::TurnPhaseForProgress(const float progress) noexcept
    {
        if (progress < 0.2f)
        {
            return OpenFloorPhaseId::Startup;
        }
        if (progress > 0.85f)
        {
            return OpenFloorPhaseId::Stop;
        }
        return OpenFloorPhaseId::SteadyRotation;
    }

    OpenFloorPhaseId OpenFloorMeasurementController::State::SmoothPhaseForProgress(const float progress) noexcept
    {
        if (progress < 0.2f)
        {
            return OpenFloorPhaseId::Entry;
        }
        if (progress > 0.85f)
        {
            return OpenFloorPhaseId::Exit;
        }
        return OpenFloorPhaseId::Middle;
    }

    OpenFloorPhaseId OpenFloorMeasurementController::State::ManeuverPhaseForProgress(
        const MazeMap::ManeuverCode code,
        const float progress) noexcept
    {
        return (code == MazeMap::IP90 || code == MazeMap::IP90_M) ?
            TurnPhaseForProgress(progress) :
            (code == MazeMap::S1 || code == MazeMap::S2) ?
                StraightPhaseForProgress(progress) :
                SmoothPhaseForProgress(progress);
    }

    void OpenFloorMeasurementController::State::SwitchPhase(
        LoopController::TickServices& services,
        const ModeWorkCallback callback) noexcept
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = callback;
        callbacks.context = this;
        services.SetNextModeWorkCallbacks(callbacks);
    }

    void OpenFloorMeasurementController::State::StartInterPhaseHold(
        const HoldContinuation continuation,
        const std::uint16_t durationMs) noexcept
    {
        _driveService.Cancel();
        _holdState = {};
        _holdState.continuation = continuation;
        _driveService.SetLimits(StraightLimits(0.0f));
        _driveService.SetOperationMode(OpenFloorOperationMode());
        _driveService.StartHold(durationMs, false);
    }

    Drive::OperationMode OpenFloorMeasurementController::State::OpenFloorOperationMode() const noexcept
    {
        return Drive::OperationMode::OpenFloor;
    }

    MotionLimits OpenFloorMeasurementController::State::StraightLimits(const float speedMps) const noexcept
    {
        return BuildOpenFloorMeasurementLimits(_vehicle, speedMps);
    }

    MotionLimits OpenFloorMeasurementController::State::TurnLimits(const float maxOmegaRadps) const noexcept
    {
        MotionLimits limits = BuildOpenFloorMeasurementLimits(_vehicle, 0.0f);
        limits.maxAngularSpeedRadps = maxOmegaRadps;
        return limits;
    }

    float OpenFloorMeasurementController::State::CurrentManeuverProgress(
        const MazeMap::ManeuverCode code,
        const float startDistanceM,
        const float totalDistanceM,
        const float targetYawRad,
        const float targetMagnitudeRad) const noexcept
    {
        if (totalDistanceM > Config::kDistanceToleranceM)
        {
            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            return (std::clamp)(traveledM / totalDistanceM, 0.0f, 1.0f);
        }

        if ((code == MazeMap::IP90 || code == MazeMap::IP90_M) && (targetMagnitudeRad > 0.0f))
        {
            const float remainingRad = std::fabs(AngleErrorRad(targetYawRad, _drive.GetPose().yawRad));
            return (std::clamp)(1.0f - (remainingRad / targetMagnitudeRad), 0.0f, 1.0f);
        }

        return 1.0f;
    }

    bool OpenFloorMeasurementController::State::CheckTimingFault(
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        const bool selectorRemoved = SelectorRemoved();
        if (state.estimatorHealthy && !selectorRemoved)
        {
            return false;
        }

        services.Fault(!state.estimatorHealthy ?
            "Estimator fault during timing capture" :
            kOpenFloorMeasurementSelectorRemovedReason);
        return true;
    }

    bool OpenFloorMeasurementController::State::CheckMainFault(
        OpenFloorMeasurementLabels& labels,
        LoopController::TickServices& services,
        const LoopController::ModeState& state,
        const char* estimatorReason)
    {
        if (SelectorRemoved())
        {
            labels.abortMarker = true;
        }
        if (state.estimatorHealthy && !labels.abortMarker)
        {
            return false;
        }

        services.Fault(!state.estimatorHealthy ? estimatorReason : kOpenFloorMeasurementSelectorRemovedReason);
        return true;
    }

    bool OpenFloorMeasurementController::State::HasRemainingLaunchSamples() const noexcept
    {
        std::size_t magnitudeIndex = _launchState.magnitudeIndex;
        std::uint8_t repeatIteration = _launchState.repeatIteration;
        while (magnitudeIndex < MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
        {
            if (repeatIteration >= MazeMap::kOpenFloorLaunchRepeatsPerMagnitude)
            {
                repeatIteration = 0U;
                ++magnitudeIndex;
                continue;
            }

            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::StartNextLaunchSample()
    {
        while (_launchState.magnitudeIndex < MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
        {
            if (_launchState.repeatIteration >= MazeMap::kOpenFloorLaunchRepeatsPerMagnitude)
            {
                _launchState.repeatIteration = 0U;
                _launchState.negativeNext = false;
                ++_launchState.magnitudeIndex;
                continue;
            }

            const float magnitude = MazeMap::kOpenFloorLaunchDriveMagnitudes[_launchState.magnitudeIndex];
            const bool negative = _launchState.negativeNext;
            const std::uint16_t repeatIndex =
                static_cast<std::uint16_t>(_launchState.nextRepeatIndex + 1U);
            _launchState.nextRepeatIndex = repeatIndex;
            if (_launchState.negativeNext)
            {
                _launchState.negativeNext = false;
                ++_launchState.repeatIteration;
            }
            else
            {
                _launchState.negativeNext = true;
            }

            _launchState.active = true;
            _launchState.labels = {};
            _launchState.labels.sectionId = OpenFloorSectionId::Sec20Launch;
            _launchState.labels.startMarkerId = OpenFloorMarkerId::C;
            _launchState.labels.primitiveId = OpenFloorPrimitiveId::OpenLoopLaunch;
            _launchState.labels.directionId = negative ?
                OpenFloorDirectionId::Negative :
                OpenFloorDirectionId::Positive;
            _launchState.labels.phaseId = OpenFloorPhaseId::LaunchPulse;
            _launchState.labels.repeatIndex = repeatIndex;
            _launchState.signedDriveCommand = negative ? -magnitude : magnitude;
            _launchState.pulseDeadlineMs = millis() + MazeMap::kOpenFloorLaunchPulseMs;
            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::HasRemainingStraightSamples() const noexcept
    {
        std::size_t speedIndex = _straightState.speedIndex;
        std::uint8_t repeatIteration = _straightState.repeatIteration;
        while (speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size())
        {
            if (repeatIteration >= MazeMap::kOpenFloorStraightRepeatsPerSpeed)
            {
                repeatIteration = 0U;
                ++speedIndex;
                continue;
            }

            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::StartNextStraightSample()
    {
        const float straightDistanceM = MazeMap::OpenFloorStrEquivalentDistanceMeters(4U);
        while (_straightState.speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size())
        {
            if (_straightState.repeatIteration >= MazeMap::kOpenFloorStraightRepeatsPerSpeed)
            {
                _straightState.repeatIteration = 0U;
                _straightState.negativeNext = false;
                ++_straightState.speedIndex;
                continue;
            }

            const bool negative = _straightState.negativeNext;
            const std::uint16_t repeatIndex =
                static_cast<std::uint16_t>(_straightState.nextRepeatIndex + 1U);
            _straightState.nextRepeatIndex = repeatIndex;
            if (_straightState.negativeNext)
            {
                _straightState.negativeNext = false;
                ++_straightState.repeatIteration;
            }
            else
            {
                _straightState.negativeNext = true;
            }

            const float speedMps = MazeMap::kOpenFloorStraightSpeedBinsMps[_straightState.speedIndex];
            _driveService.Cancel();
            _driveService.SetLimits(StraightLimits(speedMps));
            _driveService.SetOperationMode(OpenFloorOperationMode());
            _driveService.StartStraight(straightDistanceM, negative ? -speedMps : speedMps, 0.0f);
            if (!_driveService.Active())
            {
                return false;
            }

            _straightState.active = true;
            _straightState.labels = {};
            _straightState.labels.sectionId = OpenFloorSectionId::Sec30Straight;
            _straightState.labels.startMarkerId = OpenFloorMarkerId::C;
            _straightState.labels.primitiveId = OpenFloorPrimitiveId::Str4;
            _straightState.labels.directionId = negative ?
                OpenFloorDirectionId::Negative :
                OpenFloorDirectionId::Positive;
            _straightState.labels.repeatIndex = repeatIndex;
            _straightState.labels.speedBin = SpeedBinForIndex(_straightState.speedIndex);
            _straightState.distanceM = straightDistanceM;
            _straightState.signedCruiseSpeedMps = negative ? -speedMps : speedMps;
            _straightState.startDistanceM = _drive.GetAverageDistanceMeters();
            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::StartNextYawSample()
    {
        while (_yawState.speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size())
        {
            if (_yawState.repeatIteration >= DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed)
            {
                _yawState.repeatIteration = 0U;
                _yawState.primitiveIndex = 0U;
                ++_yawState.speedIndex;
                continue;
            }

            const std::uint8_t primitiveIndex = _yawState.primitiveIndex;
            const std::uint16_t repeatIndex =
                static_cast<std::uint16_t>(_yawState.nextRepeatIndex + 1U);
            _yawState.nextRepeatIndex = repeatIndex;
            ++_yawState.primitiveIndex;
            if (_yawState.primitiveIndex >= MazeMap::kOpenFloorYawPrimitiveIds.size())
            {
                _yawState.primitiveIndex = 0U;
                ++_yawState.repeatIteration;
            }

            const OpenFloorPrimitiveId primitiveId = MazeMap::kOpenFloorYawPrimitiveIds[primitiveIndex];
            const OpenFloorDirectionId directionId = MazeMap::kOpenFloorYawDirectionIds[primitiveIndex];
            const float angleRad = MazeMap::kOpenFloorYawNominalAnglesRad[primitiveIndex];

            const float maxOmegaRadps = MazeMap::kOpenFloorYawOmegaBinsRadps[_yawState.speedIndex];
            _driveService.Cancel();
            _driveService.SetLimits(TurnLimits(maxOmegaRadps));
            _driveService.SetOperationMode(OpenFloorOperationMode());
            _driveService.StartTurn(angleRad);
            if (!_driveService.Active())
            {
                return false;
            }

            _yawState.active = true;
            _yawState.labels = {};
            _yawState.labels.sectionId = OpenFloorSectionId::Sec40Yaw;
            _yawState.labels.startMarkerId = OpenFloorMarkerId::C;
            _yawState.labels.primitiveId = primitiveId;
            _yawState.labels.directionId = directionId;
            _yawState.labels.repeatIndex = repeatIndex;
            _yawState.labels.speedBin = SpeedBinForIndex(_yawState.speedIndex);
            _yawState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
            _yawState.targetMagnitudeRad = std::fabs(angleRad);
            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::HasRemainingYawSamples() const noexcept
    {
        std::size_t speedIndex = _yawState.speedIndex;
        std::uint8_t repeatIteration = _yawState.repeatIteration;
        std::uint8_t primitiveIndex = _yawState.primitiveIndex;
        while (speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size())
        {
            if (repeatIteration >= DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed)
            {
                repeatIteration = 0U;
                primitiveIndex = 0U;
                ++speedIndex;
                continue;
            }

            if (primitiveIndex < MazeMap::kOpenFloorYawPrimitiveIds.size())
            {
                return true;
            }

            primitiveIndex = 0U;
            ++repeatIteration;
        }

        return false;
    }

    bool OpenFloorMeasurementController::State::StartNextSmoothEntry()
    {
        while (_smoothState.speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size())
        {
            if (!_smoothState.queueLoaded)
            {
                if (!BuildOpenFloorMeasurementSmoothQueue(
                        _vehicle,
                        _smoothState.speedIndex,
                        OpenFloorMeasurementSmoothSpeedMps(_smoothState.speedIndex),
                        _smoothState.entryBoundarySpeedMps,
                        _smoothState.queue,
                        _smoothState.exitBoundarySpeedMps))
                {
                    return false;
                }
                _smoothState.entryIndex = 0U;
                _smoothState.queueLoaded = true;
            }

            if (_smoothState.entryIndex >= _smoothState.queue.size())
            {
                _smoothState.entryBoundarySpeedMps = _smoothState.exitBoundarySpeedMps;
                _smoothState.queueLoaded = false;
                ++_smoothState.speedIndex;
                continue;
            }

            const MazeMap::ManeuverInstance& entry = _smoothState.queue[_smoothState.entryIndex];
            _driveService.Cancel();
            _driveService.SetLimits(StraightLimits(OpenFloorMeasurementSmoothSpeedMps(_smoothState.speedIndex)));
            _driveService.SetOperationMode(OpenFloorOperationMode());
            _driveService.StartManeuver(entry);
            if (!_driveService.Active())
            {
                return false;
            }

            const float angleRad = static_cast<float>(MazeMap::CodeDegrees(entry.getCode())) * DEG_TO_RAD_F;
            _smoothState.active = true;
            _smoothState.labels = {};
            _smoothState.labels.sectionId = OpenFloorSectionId::Sec50Smooth;
            _smoothState.labels.startMarkerId = OpenFloorMarkerId::C;
            _smoothState.labels.primitiveId = PrimitiveIdForCode(entry.getCode());
            _smoothState.labels.directionId = DirectionIdForCode(entry.getCode());
            _smoothState.labels.repeatIndex = 1U;
            _smoothState.labels.speedBin = SpeedBinForIndex(_smoothState.speedIndex);
            _smoothState.startDistanceM = _drive.GetAverageDistanceMeters();
            _smoothState.totalDistanceM = entry.GetTravelDistanceMeters();
            _smoothState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
            _smoothState.targetMagnitudeRad = std::fabs(angleRad);
            return _smoothState.labels.primitiveId != OpenFloorPrimitiveId::None;
        }

        return false;
    }

    void OpenFloorMeasurementController::State::ResetLoopPhase(const bool clockwise) noexcept
    {
        _loopState = {};
        _loopState.clockwise = clockwise;
        _loopState.repeatIndex = 1U;
    }

    bool OpenFloorMeasurementController::State::StartNextLoopEntry()
    {
        while (_loopState.repeatIndex <= DiagnosticConfig::kLoopRepeats)
        {
            if (!_loopState.queueLoaded)
            {
                if (!BuildOpenFloorMeasurementLoopQueue(_vehicle, _loopState.clockwise, _loopState.queue))
                {
                    return false;
                }
                _loopState.entryIndex = 0U;
                _loopState.queueLoaded = true;
            }

            if (_loopState.entryIndex >= _loopState.queue.size())
            {
                _loopState.queueLoaded = false;
                ++_loopState.repeatIndex;
                continue;
            }

            const MazeMap::ManeuverInstance& entry = _loopState.queue[_loopState.entryIndex];
            _driveService.Cancel();
            _driveService.SetLimits(StraightLimits(MazeMap::kOpenFloorStraightSpeedBinsMps[0]));
            _driveService.SetOperationMode(OpenFloorOperationMode());
            _driveService.StartManeuver(entry);
            if (!_driveService.Active())
            {
                return false;
            }

            const float angleRad = static_cast<float>(MazeMap::CodeDegrees(entry.getCode())) * DEG_TO_RAD_F;
            _loopState.active = true;
            _loopState.labels = {};
            _loopState.labels.sectionId = _loopState.clockwise ?
                OpenFloorSectionId::Sec60LoopCw :
                OpenFloorSectionId::Sec70LoopCcw;
            _loopState.labels.startMarkerId = _loopState.clockwise ?
                OpenFloorMarkerId::CW :
                OpenFloorMarkerId::CCW;
            _loopState.labels.primitiveId = PrimitiveIdForCode(entry.getCode());
            _loopState.labels.directionId = _loopState.clockwise ?
                OpenFloorDirectionId::Clockwise :
                OpenFloorDirectionId::CounterClockwise;
            _loopState.labels.repeatIndex = _loopState.repeatIndex;
            _loopState.labels.speedBin = OpenFloorSpeedBin::Low;
            _loopState.startDistanceM = _drive.GetAverageDistanceMeters();
            _loopState.totalDistanceM = entry.GetTravelDistanceMeters();
            _loopState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
            _loopState.targetMagnitudeRad = std::fabs(angleRad);
            return _loopState.labels.primitiveId != OpenFloorPrimitiveId::None;
        }

        return false;
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::State::OnPauseGranted(
        const LoopController::PauseContext& pause)
    {
        (void)pause;
        if (_pauseAction != PauseAction::TimingToMain)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement pause granted without a pending timing transition");
        }

        if (_pendingTimingSampleValid)
        {
            ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _pendingTimingRow);
            if (!_runtime.LogUtilityDataRow(_pendingTimingRow))
            {
                return LoopController::PauseDisposition::StopByRuntime(
                    "Open-floor measurement timing log write failed during timing transition");
            }
            _pendingTimingSampleValid = false;
            _pendingTimingRow = {};
        }

        if (_timingLogOpen)
        {
            _timingLogOpen = false;
        }
        if (!_mainLogOpen && !BeginMainLog())
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement main log setup failed after timing capture");
        }

        _pauseAction = PauseAction::None;
        return LoopController::PauseDisposition::Resume();
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::State::PauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->OnPauseGranted(pause) :
            LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement pause callback context was null");
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::TimingTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->TimingTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::InterPhaseHoldTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->InterPhaseHoldTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::StaticTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->StaticTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LaunchTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->LaunchTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::StraightTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->StraightTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::YawTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->YawTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::SmoothTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->SmoothTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LoopCwTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->LoopCwTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LoopCcwTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController::State*>(context);
        return (self != nullptr) ?
            self->LoopCcwTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    bool OpenFloorMeasurementController::State::SwitchToHoldContinuation(
        LoopController::TickServices& services)
    {
        switch (_holdState.continuation)
        {
        case HoldContinuation::LaunchRepeat:
            if (!StartNextLaunchSample())
            {
                services.Fault("Open-floor launch sample could not start after the inter-motion hold");
                return false;
            }
            SwitchPhase(services, &OpenFloorMeasurementController::State::LaunchTickThunk);
            return true;
        case HoldContinuation::Launch:
            _launchState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::State::LaunchTickThunk);
            return true;
        case HoldContinuation::StraightRepeat:
            if (!StartNextStraightSample())
            {
                services.Fault("Open-floor straight sample could not start after the inter-motion hold");
                return false;
            }
            SwitchPhase(services, &OpenFloorMeasurementController::State::StraightTickThunk);
            return true;
        case HoldContinuation::Straight:
            _straightState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::State::StraightTickThunk);
            return true;
        case HoldContinuation::YawRepeat:
            if (!StartNextYawSample())
            {
                services.Fault("Open-floor yaw sample could not start after the inter-motion hold");
                return false;
            }
            SwitchPhase(services, &OpenFloorMeasurementController::State::YawTickThunk);
            return true;
        case HoldContinuation::Yaw:
            _yawState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::State::YawTickThunk);
            return true;
        case HoldContinuation::Smooth:
            _smoothState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::State::SmoothTickThunk);
            return true;
        case HoldContinuation::LoopCw:
            ResetLoopPhase(true);
            SwitchPhase(services, &OpenFloorMeasurementController::State::LoopCwTickThunk);
            return true;
        case HoldContinuation::LoopCcw:
            ResetLoopPhase(false);
            SwitchPhase(services, &OpenFloorMeasurementController::State::LoopCcwTickThunk);
            return true;
        case HoldContinuation::Complete:
            services.RequestEndLoop();
            return true;
        case HoldContinuation::None:
        default:
            return false;
        }
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::TimingTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!FlushPendingTimingSample(services, "Open-floor measurement timing log write failed"))
        {
            return stopControl;
        }
        const bool timingFault = CheckTimingFault(state, services);
        OpenFloorTimingRow row{};
        PopulateTimingRowFromState(state, row);
        StagePendingTimingSample(row);
        if (timingFault)
        {
            return stopControl;
        }

        ++_timingState.tickIndex;
        if (_timingState.tickIndex >= DiagnosticConfig::kTimingCaptureCycles)
        {
            _pauseAction = PauseAction::TimingToMain;
            _staticState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::State::StaticTickThunk);

            LoopController::PauseRequest request{};
            request.onPauseGranted = &OpenFloorMeasurementController::State::PauseThunk;
            request.reason = "open_floor_timing_to_main";
            request.flushLogsBeforeGrant = true;
            request.resetClockOnResume = true;
            services.RequestPause(request);
        }

        return stopControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::InterPhaseHoldTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (_mainLogOpen &&
            !FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (SelectorRemoved())
        {
            services.Fault(kOpenFloorMeasurementSelectorRemovedReason);
            return stopControl;
        }
        if (!state.estimatorHealthy)
        {
            services.Fault("Estimator fault during inter-phase hold");
            return stopControl;
        }

        if (!_driveService.Active())
        {
            services.Fault("Open-floor cumulative hold primitive was not active");
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector control = _driveService.GetNextControls(done);
        if (done)
        {
            if (!SwitchToHoldContinuation(services))
            {
                services.Fault("Open-floor measurement hold continuation was not configured");
            }
            return stopControl;
        }

        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::StaticTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector control = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        const CommandTelemetrySnapshot nextCommandTelemetry = BuildRawCommandTelemetrySnapshot(control);
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return control;
        }
        if (!_staticState.initialized)
        {
            _staticState.initialized = true;
            _staticState.deadlineMs = millis() + DiagnosticConfig::kStaticHoldMs;
            _staticState.labels = {};
            _staticState.labels.sectionId = OpenFloorSectionId::Sec10Static;
            _staticState.labels.startMarkerId = OpenFloorMarkerId::C;
            _staticState.labels.primitiveId = OpenFloorPrimitiveId::StaticHold;
            _staticState.labels.phaseId = OpenFloorPhaseId::Hold;
            _staticState.labels.repeatIndex = 1U;
        }

        const float staticRemainingMs = static_cast<float>((std::max)(
            0L,
            static_cast<long>(_staticState.deadlineMs - millis())));
        _staticState.labels.progressNorm = (DiagnosticConfig::kStaticHoldMs > 0U) ?
            (std::clamp)(
                (static_cast<float>(DiagnosticConfig::kStaticHoldMs) - staticRemainingMs) /
                    static_cast<float>(DiagnosticConfig::kStaticHoldMs),
                0.0f,
                1.0f) :
            1.0f;
        const bool mainFault = CheckMainFault(
            _staticState.labels,
            services,
            state,
            "Estimator fault during static hold");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_staticState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        _savedCommandTelemetry = nextCommandTelemetry;
        if (mainFault)
        {
            return control;
        }

        if (static_cast<long>(_staticState.deadlineMs - millis()) <= 0)
        {
            StartInterPhaseHold(HoldContinuation::Launch, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
        }

        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LaunchTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_launchState.active && !StartNextLaunchSample())
        {
            StartInterPhaseHold(HoldContinuation::Straight, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }

        const unsigned long nowMs = millis();
        _launchState.labels.phaseId = OpenFloorPhaseId::LaunchPulse;
        _launchState.labels.progressNorm = (MazeMap::kOpenFloorLaunchPulseMs > 0UL) ?
            (std::clamp)(
                static_cast<float>(MazeMap::kOpenFloorLaunchPulseMs - (std::max)(0L, static_cast<long>(_launchState.pulseDeadlineMs - nowMs))) /
                    static_cast<float>(MazeMap::kOpenFloorLaunchPulseMs),
                0.0f,
                1.0f) :
            1.0f;
        const LoopController::ControlVector control = LoopController::ControlVector::RawMotorPwm(
            _launchState.signedDriveCommand,
            _launchState.signedDriveCommand);
        const CommandTelemetrySnapshot nextCommandTelemetry = BuildRawCommandTelemetrySnapshot(control);

        const bool mainFault = CheckMainFault(
            _launchState.labels,
            services,
            state,
            "Estimator fault during launch section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_launchState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        _savedCommandTelemetry = mainFault ? stopTelemetry : nextCommandTelemetry;
        if (mainFault)
        {
            return stopControl;
        }

        _savedCommandTelemetry = nextCommandTelemetry;
        if (static_cast<long>(_launchState.pulseDeadlineMs - nowMs) <= 0)
        {
            _launchState.active = false;
            StartInterPhaseHold(
                HasRemainingLaunchSamples() ?
                    HoldContinuation::LaunchRepeat :
                    HoldContinuation::Straight,
                MazeMap::kOpenFloorPostSegmentHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::StraightTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_straightState.active && !StartNextStraightSample())
        {
            StartInterPhaseHold(HoldContinuation::Yaw, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }

        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _straightState.startDistanceM);
        _straightState.labels.progressNorm = (std::clamp)(
            traveledM / _straightState.distanceM,
            0.0f,
            1.0f);
        _straightState.labels.phaseId = StraightPhaseForProgress(_straightState.labels.progressNorm);
        const bool mainFault = CheckMainFault(
            _straightState.labels,
            services,
            state,
            "Estimator fault during straight section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_straightState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        if (mainFault)
        {
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry(control);
        _savedCommandTelemetry = nextCommandTelemetry;

        if (done)
        {
            _straightState.active = false;
            StartInterPhaseHold(
                HasRemainingStraightSamples() ?
                    HoldContinuation::StraightRepeat :
                    HoldContinuation::Yaw,
                MazeMap::kOpenFloorPostSegmentHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::YawTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_yawState.active && !StartNextYawSample())
        {
            StartInterPhaseHold(HoldContinuation::Smooth, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }

        const float remainingRad = std::fabs(AngleErrorRad(_yawState.targetYawRad, _drive.GetPose().yawRad));
        _yawState.labels.progressNorm = (_yawState.targetMagnitudeRad > 0.0f) ?
            (std::clamp)(1.0f - (remainingRad / _yawState.targetMagnitudeRad), 0.0f, 1.0f) :
            1.0f;
        _yawState.labels.phaseId = TurnPhaseForProgress(_yawState.labels.progressNorm);
        const bool mainFault = CheckMainFault(
            _yawState.labels,
            services,
            state,
            "Estimator fault during yaw section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_yawState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        if (mainFault)
        {
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry(control);
        _savedCommandTelemetry = nextCommandTelemetry;

        if (done)
        {
            _yawState.active = false;
            StartInterPhaseHold(
                HasRemainingYawSamples() ?
                    HoldContinuation::YawRepeat :
                    HoldContinuation::Smooth,
                MazeMap::kOpenFloorPostSegmentHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::SmoothTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_smoothState.active && !StartNextSmoothEntry())
        {
            StartInterPhaseHold(HoldContinuation::LoopCw, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }

        const MazeMap::ManeuverCode code = _smoothState.queue[_smoothState.entryIndex].getCode();
        _smoothState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _smoothState.startDistanceM,
            _smoothState.totalDistanceM,
            _smoothState.targetYawRad,
            _smoothState.targetMagnitudeRad);
        _smoothState.labels.phaseId = ManeuverPhaseForProgress(code, _smoothState.labels.progressNorm);
        const bool mainFault = CheckMainFault(
            _smoothState.labels,
            services,
            state,
            "Estimator fault during smooth section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_smoothState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        if (mainFault)
        {
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry(control);
        _savedCommandTelemetry = nextCommandTelemetry;

        if (done)
        {
            _smoothState.active = false;
            ++_smoothState.entryIndex;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LoopCwTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_loopState.active && !StartNextLoopEntry())
        {
            StartInterPhaseHold(HoldContinuation::LoopCcw, MazeMap::kOpenFloorInterPhaseHoldMs);
            SwitchPhase(services, &OpenFloorMeasurementController::State::InterPhaseHoldTickThunk);
            return stopControl;
        }

        const MazeMap::ManeuverCode code = _loopState.queue[_loopState.entryIndex].getCode();
        _loopState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _loopState.startDistanceM,
            _loopState.totalDistanceM,
            _loopState.targetYawRad,
            _loopState.targetMagnitudeRad);
        _loopState.labels.phaseId = ManeuverPhaseForProgress(code, _loopState.labels.progressNorm);
        const bool mainFault = CheckMainFault(
            _loopState.labels,
            services,
            state,
            "Estimator fault during clockwise loop section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_loopState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        if (mainFault)
        {
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry(control);
        _savedCommandTelemetry = nextCommandTelemetry;

        if (done)
        {
            _loopState.active = false;
            ++_loopState.entryIndex;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::State::LoopCcwTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        const CommandTelemetrySnapshot appliedCommandTelemetry = _savedCommandTelemetry;
        if (!FlushPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }
        if (!_loopState.active && !StartNextLoopEntry())
        {
            services.RequestEndLoop();
            return stopControl;
        }

        const MazeMap::ManeuverCode code = _loopState.queue[_loopState.entryIndex].getCode();
        _loopState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _loopState.startDistanceM,
            _loopState.totalDistanceM,
            _loopState.targetYawRad,
            _loopState.targetMagnitudeRad);
        _loopState.labels.phaseId = ManeuverPhaseForProgress(code, _loopState.labels.progressNorm);
        const bool mainFault = CheckMainFault(
            _loopState.labels,
            services,
            state,
            "Estimator fault during counter-clockwise loop section");
        OpenFloorMainRow row{};
        PopulateMainRowFromState(_loopState.labels, state, appliedCommandTelemetry, row);
        StagePendingMainSample(row);
        if (mainFault)
        {
            _savedCommandTelemetry = stopTelemetry;
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry(control);
        _savedCommandTelemetry = nextCommandTelemetry;

        if (done)
        {
            _loopState.active = false;
            ++_loopState.entryIndex;
            return stopControl;
        }
        return control;
    }

    IApplicationMode& GetOpenFloorMeasurementMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::OpenFloorMeasurement,
            BootModeCategory::Utility,
            "open_floor_measurement",
            "Run the ordered open-floor measurement battery with timing, static, launch, straight, yaw, smooth, and closed maneuver loops.",
            "logging.txt, open_floor_timing.mmlog, open_floor_main.mmlog",
            &GetOpenFloorMeasurementMode,
            "GetOpenFloorMeasurementMode",
            "OpenFloorMeasurementController.cpp",
            "timing capture; static hold; launch PWM pulses; straight drive tests; yaw drive tests; smooth maneuver sweep; clockwise closed maneuver loop; counter-clockwise closed maneuver loop",
            "DiagnosticConfig linear limits; OpenFloorMeasurementSpec speed bins; shared startup calibration; shared drive service",
            "Inter-phase 500 ms brake holds; launch and straight samples insert 100 ms brake holds between motions; smooth phase uses the current hand-picked closed maneuver sequence; loop sections are maneuver-driven",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetOpenFloorMeasurementMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}

