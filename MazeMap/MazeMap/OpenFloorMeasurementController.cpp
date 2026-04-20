#include "pch.h"
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
#include "MazeMapSharedRuntime.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "RuntimeSensorSuite.h"
#include "StartupCalibration.h"

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
    X(std::uint16_t, watchdog_flags)               \
    X(std::uint16_t, measurement_flags)            \
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
    X(std::uint8_t,  imu_interrupt_high)           \
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
    X(std::uint8_t,  front_left_obs_class)         \
    X(std::uint8_t,  front_right_obs_class)        \
    X(std::uint8_t,  left_obs_class)               \
    X(std::uint8_t,  right_obs_class)              \
    X(float,         front_left_obs_rho_m)         \
    X(float,         front_right_obs_rho_m)        \
    X(float,         left_obs_rho_m)               \
    X(float,         right_obs_rho_m)              \
    X(float,         front_left_obs_confidence)    \
    X(float,         front_right_obs_confidence)   \
    X(float,         left_obs_confidence)          \
    X(float,         right_obs_confidence)         \
    X(float,         fan_duty_cycle)

    MMLOG_DEFINE_ROW(OpenFloorMainRow, OPEN_FLOOR_MAIN_FIELDS);

    inline constexpr std::uint16_t kOpenFloorMeasurementFlagAbortMarker = 1u << 0;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagEstimatorFault = 1u << 2;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagFanEnabled = 1u << 3;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagEncoderValid = 1u << 4;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagImuValid = 1u << 5;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagAccelBiasValid = 1u << 6;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagFrontLeftObsValid = 1u << 7;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagFrontRightObsValid = 1u << 8;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagLeftObsValid = 1u << 9;
    inline constexpr std::uint16_t kOpenFloorMeasurementFlagRightObsValid = 1u << 10;

    inline std::uint16_t BuildOpenFloorMeasurementFlags(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        const bool encoderValid,
        const bool imuValid,
        const MazeMap::WallObs& frontLeftObs,
        const MazeMap::WallObs& frontRightObs,
        const MazeMap::WallObs& leftObs,
        const MazeMap::WallObs& rightObs)
    {
        std::uint16_t flags = 0U;
        if (labels.abortMarker)
        {
            flags |= kOpenFloorMeasurementFlagAbortMarker;
        }
        if (cycle.estimatorFault)
        {
            flags |= kOpenFloorMeasurementFlagEstimatorFault;
        }
        if (cycle.fanDutyCycle > 0.0f)
        {
            flags |= kOpenFloorMeasurementFlagFanEnabled;
        }
        if (encoderValid)
        {
            flags |= kOpenFloorMeasurementFlagEncoderValid;
        }
        if (imuValid)
        {
            flags |= kOpenFloorMeasurementFlagImuValid;
        }
        if (cycle.sensorSnapshot.accelBiasValid)
        {
            flags |= kOpenFloorMeasurementFlagAccelBiasValid;
        }
        if (frontLeftObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagFrontLeftObsValid;
        }
        if (frontRightObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagFrontRightObsValid;
        }
        if (leftObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagLeftObsValid;
        }
        if (rightObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagRightObsValid;
        }

        return flags;
    }
}

namespace MazeMap::App::Internal
{
    struct CommandTelemetrySnapshot final
    {
        DriveTelemetry driveTelemetry{};
        float linearCommandMps{};
        float angularCommandRadps{};
    };
}

namespace
{
    using MazeMap::App::Internal::CommandTelemetrySnapshot;
    using MazeMap::App::Internal::Drive;
    using MazeMap::App::Internal::LoopController;
    using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
    using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

    constexpr const char* kPrimaryDiagnosticStableId = "primary_diagnostic";
    constexpr const char* kPrimaryDiagnosticSelectorRemovedReason =
        "Primary diagnostic selector jumper removed";
    constexpr std::uint16_t kPrimaryDiagnosticInterPhaseHoldMs = 500U;
    constexpr float kPrimaryDiagnosticMaxSmoothSpeedMps = MazeMap::kOpenFloorSmoothSpeedBinsMps[2];
    constexpr MazeMap::ManeuverCode kPrimaryDiagnosticSpeedChangeStraightCode = MazeMap::S1;

    constexpr std::array<MazeMap::ManeuverCode, 26U> kPrimaryDiagnosticSmoothCycle = {
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

    MotionLimits BuildPrimaryDiagnosticLimits(
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

    constexpr float PrimaryDiagnosticSmoothSpeedMps(const std::uint8_t speedIndex) noexcept
    {
        return (speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size()) ?
            MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex] :
            kPrimaryDiagnosticMaxSmoothSpeedMps;
    }

    MazeMap::DirectionalLocation PrimaryDiagnosticSmoothQueueStartLocation(
        const std::uint8_t speedIndex) noexcept
    {
        return MazeMap::DirectionalLocation(
            2U,
            static_cast<std::uint8_t>(3U + speedIndex),
            MazeMap::Up);
    }

    bool BuildPrimaryDiagnosticSmoothQueue(
        MazeMap::Vehicle& vehicle,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float initialEntrySpeedMps,
        MazeMap::ManeuverQueue& queue,
        float& exitBoundarySpeedMps)
    {
        queue.clear();
        exitBoundarySpeedMps = 0.0f;

        MazeMap::DirectionalLocation current = PrimaryDiagnosticSmoothQueueStartLocation(speedIndex);
        if (!queue.push_back(kPrimaryDiagnosticSpeedChangeStraightCode, current))
        {
            return false;
        }
        current = queue.back().getEnd();

        for (const MazeMap::ManeuverCode code : kPrimaryDiagnosticSmoothCycle)
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
        if (isLastSpeedBin && !queue.push_back(kPrimaryDiagnosticSpeedChangeStraightCode, current))
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

    MazeMap::DirectionalLocation PrimaryDiagnosticLoopQueueStartLocation(const bool clockwise) noexcept
    {
        return clockwise ?
            MazeMap::DirectionalLocation(3U, 3U, MazeMap::Up) :
            MazeMap::DirectionalLocation(7U, 3U, MazeMap::Up);
    }

    bool BuildPrimaryDiagnosticLoopQueue(
        MazeMap::Vehicle& vehicle,
        const bool clockwise,
        MazeMap::ManeuverQueue& queue)
    {
        queue.clear();

        MazeMap::DirectionalLocation current = PrimaryDiagnosticLoopQueueStartLocation(clockwise);
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

    CommandTelemetrySnapshot BuildRawCommandTelemetrySnapshot(
        const LoopController::ControlVector& control) noexcept
    {
        CommandTelemetrySnapshot snapshot{};
        if (std::isfinite(control.leftMotorPwm) && std::isfinite(control.rightMotorPwm))
        {
            snapshot.driveTelemetry.leftDriveCommand = control.leftMotorPwm;
            snapshot.driveTelemetry.rightDriveCommand = control.rightMotorPwm;
            snapshot.driveTelemetry.leftFeedbackCommand = control.leftMotorPwm;
            snapshot.driveTelemetry.rightFeedbackCommand = control.rightMotorPwm;
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
}

namespace MazeMap::App::Internal
{
    class OpenFloorMeasurementController final : public IApplicationMode
    {
    public:
        explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

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
            Launch,
            Straight,
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
            bool initialized{};
            std::uint32_t deadlineMs{};
            HoldContinuation continuation{ HoldContinuation::None };
        };

        struct LaunchState final
        {
            enum class Stage : std::uint8_t
            {
                Pulse,
                Settle,
            };

            std::size_t magnitudeIndex{};
            std::uint8_t repeatIteration{};
            bool negativeNext{};
            std::uint16_t nextRepeatIndex{};
            bool active{};
            Stage stage{ Stage::Pulse };
            OpenFloorMeasurementLabels labels{};
            float signedDriveCommand{};
            std::uint32_t pulseDeadlineMs{};
            MazeMap::VehicleState stationaryCheckState{};
            bool previousStationary{};
            bool launchFlippedStationary{};
            std::uint32_t settleStartMs{};
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

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
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
        void CloseTimingLog();
        bool FlushPendingTimingSample(
            LoopController::TickServices& services,
            const char* failureReason);
        void StagePendingTimingSample(const OpenFloorMeasurementCycle& cycle) noexcept;
        bool BeginMainLog();
        void CloseMainLog();
        bool FlushPendingMainSample(
            LoopController::TickServices& services,
            const char* failureReason);
        void StagePendingMainSample(
            const OpenFloorMeasurementLabels& labels,
            const OpenFloorMeasurementCycle& cycle,
            const CommandTelemetrySnapshot& commandTelemetry) noexcept;
        void PopulateCycleFromState(
            const LoopController::ModeState& state,
            OpenFloorMeasurementCycle& cycle) const;
        bool LogTimingSample(const OpenFloorMeasurementCycle& cycle);
        bool LogMainSample(
            const OpenFloorMeasurementLabels& labels,
            const OpenFloorMeasurementCycle& cycle,
            const CommandTelemetrySnapshot& commandTelemetry);
        CommandTelemetrySnapshot CaptureDriveCommandTelemetry() const noexcept;
        float ReadBatteryVoltage() const noexcept;
        float ReadBoardTemperatureC(const SensorSnapshot& snapshot) const noexcept;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        void SwitchPhase(
            LoopController::TickServices& services,
            ModeWorkCallback callback) noexcept;
        void StartInterPhaseHold(HoldContinuation continuation) noexcept;
        bool SwitchToHoldContinuation(LoopController::TickServices& services);
        bool CheckTimingFault(
            const OpenFloorMeasurementCycle& cycle,
            LoopController::TickServices& services);
        bool CheckMainFault(
            OpenFloorMeasurementLabels& labels,
            const OpenFloorMeasurementCycle& cycle,
            LoopController::TickServices& services,
            const char* estimatorReason,
            const CommandTelemetrySnapshot& commandTelemetry);
        bool StartNextLaunchSample();
        bool StartNextStraightSample();
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
        OpenFloorMeasurementCycle _pendingTimingCycle{};
        StaticState _staticState{};
        HoldState _holdState{};
        LaunchState _launchState{};
        StraightState _straightState{};
        YawState _yawState{};
        SmoothState _smoothState{};
        LoopState _loopState{};
        CommandTelemetrySnapshot _pendingCommandTelemetry{};
        bool _pendingMainSampleValid{};
        OpenFloorMeasurementLabels _pendingMainLabels{};
        OpenFloorMeasurementCycle _pendingMainCycle{};
    };

    OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    bool OpenFloorMeasurementController::Begin()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(
                &OpenFloorMeasurementController::TeardownOnRuntimeFault,
                this,
                kPrimaryDiagnosticStableId))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return _runtime.FailActiveMode("Primary diagnostic hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:primary_diagnostic");
        (void)_runtime.AppendTextLogLine("Primary diagnostic mode");
        (void)_runtime.AppendTextLogLine(
            "Open-floor battery: timing -> static -> launch -> straight -> yaw -> smooth -> loop cw -> loop ccw");

        if (!_drive.Begin())
        {
            return _runtime.FailActiveMode("Primary diagnostic drive base init failed");
        }
        _drive.UseNominalWheelControlProfile();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        if (!_startupCalibration.BringUp())
        {
            return _runtime.FailActiveMode("Primary diagnostic startup bring-up failed");
        }

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            return _runtime.FailActiveMode(kPrimaryDiagnosticSelectorRemovedReason);
        }

        if (!BeginTimingLog())
        {
            return _runtime.FailActiveMode("Primary diagnostic timing log setup failed");
        }

        return true;
    }

    void OpenFloorMeasurementController::Run()
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &OpenFloorMeasurementController::TimingTickThunk;
        callbacks.context = this;

        bool completed = false;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            (void)_runtime.FailActiveMode("Primary diagnostic loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            completed = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }

        if (_mainLogOpen && _pendingMainSampleValid)
        {
            _pendingMainCycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
            if (LogMainSample(_pendingMainLabels, _pendingMainCycle, _pendingCommandTelemetry))
            {
                _pendingMainSampleValid = false;
            }
        }

        if (_timingLogOpen)
        {
            if (_pendingTimingSampleValid)
            {
                _pendingTimingCycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
                if (LogTimingSample(_pendingTimingCycle))
                {
                    _pendingTimingSampleValid = false;
                }
            }
            CloseTimingLog();
        }
        if (_mainLogOpen)
        {
            CloseMainLog();
        }

        ReleaseSelectorMonitor();
        _startupCalibration.Cancel();
        _driveService.Cancel();
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();

        if (completed)
        {
            (void)_runtime.AppendTextLogLine("Primary diagnostic complete");
        }
    }

    LoopController::SessionOptions OpenFloorMeasurementController::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    void OpenFloorMeasurementController::ResetState() noexcept
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
        _pendingTimingCycle = {};
        _staticState = {};
        _holdState = {};
        _launchState = {};
        _straightState = {};
        _yawState = {};
        _smoothState = {};
        _loopState = {};
        _pendingCommandTelemetry =
            BuildRawCommandTelemetrySnapshot(LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f));
        _pendingMainSampleValid = false;
        _pendingMainLabels = {};
        _pendingMainCycle = {};
    }

    bool OpenFloorMeasurementController::BeginTimingLog()
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

        OpenFloorTimingRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _timingLogOpen = true;
        return true;
    }

    void OpenFloorMeasurementController::CloseTimingLog()
    {
        (void)_runtime.CloseUtilityDataLog();
        _timingLogOpen = false;
    }

    bool OpenFloorMeasurementController::FlushPendingTimingSample(
        LoopController::TickServices& services,
        const char* failureReason)
    {
        if (!_pendingTimingSampleValid)
        {
            return true;
        }

        _pendingTimingCycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
        if (!LogTimingSample(_pendingTimingCycle))
        {
            services.Fault(failureReason);
            return false;
        }

        _pendingTimingSampleValid = false;
        _pendingTimingCycle = {};
        return true;
    }

    void OpenFloorMeasurementController::StagePendingTimingSample(
        const OpenFloorMeasurementCycle& cycle) noexcept
    {
        _pendingTimingCycle = cycle;
        _pendingTimingSampleValid = true;
    }

    bool OpenFloorMeasurementController::BeginMainLog()
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

    void OpenFloorMeasurementController::CloseMainLog()
    {
        (void)_runtime.CloseUtilityDataLog();
        _mainLogOpen = false;
    }

    bool OpenFloorMeasurementController::FlushPendingMainSample(
        LoopController::TickServices& services,
        const char* failureReason)
    {
        if (!_pendingMainSampleValid)
        {
            return true;
        }

        _pendingMainCycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
        if (!LogMainSample(_pendingMainLabels, _pendingMainCycle, _pendingCommandTelemetry))
        {
            services.Fault(failureReason);
            return false;
        }

        _pendingMainSampleValid = false;
        _pendingMainLabels = {};
        _pendingMainCycle = {};
        return true;
    }

    void OpenFloorMeasurementController::StagePendingMainSample(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        const CommandTelemetrySnapshot& commandTelemetry) noexcept
    {
        _pendingMainLabels = labels;
        _pendingMainCycle = cycle;
        _pendingCommandTelemetry = commandTelemetry;
        _pendingMainSampleValid = true;
    }

    void OpenFloorMeasurementController::PopulateCycleFromState(
        const LoopController::ModeState& state,
        OpenFloorMeasurementCycle& cycle) const
    {
        cycle.masterTimeUs = state.tickStartUs;
        cycle.controlTickSequence = state.sequence;
        cycle.dtUs = state.dtUs;
        cycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
        cycle.driveTelemetry = state.driveTelemetry;
        cycle.sensorSnapshot = state.sensors;
        cycle.estimatorState = _drive.GetEstimatorStateVector();
        cycle.measuredLinearSpeedMps = state.measured.linearSpeedMps;
        cycle.measuredAngularSpeedRadps = state.measured.angularSpeedRadps;
        cycle.planarAccelMps2 = cycle.sensorSnapshot.planarAccelMps2;
        cycle.batteryVoltage = ReadBatteryVoltage();
        cycle.boardTemperatureC = ReadBoardTemperatureC(cycle.sensorSnapshot);
        cycle.fanDutyCycle = GetMissionFanDutyCycle();
        cycle.selectorJumperRemoved = SelectorRemoved();
        cycle.estimatorFault = !state.estimatorHealthy;
    }

    float OpenFloorMeasurementController::ReadBatteryVoltage() const noexcept
    {
        return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
    }

    float OpenFloorMeasurementController::ReadBoardTemperatureC(const SensorSnapshot& snapshot) const noexcept
    {
        return 25.0f + (static_cast<float>(snapshot.imuBackLeft.temp) / 256.0f);
    }

    void OpenFloorMeasurementController::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        const BootModeRegistryEntry* const entry =
            FindBootModeRegistryEntry(BootModeId::PrimaryDiagnostic);
        if ((entry == nullptr) || (entry->selector.kind != BootModeSelectorKind::PinPair))
        {
            return;
        }

        _selectorDrivePin = entry->selector.pinA;
        _selectorSensePin = entry->selector.pinB;
        BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        _selectorMonitorArmed = true;
    }

    void OpenFloorMeasurementController::ReleaseSelectorMonitor() noexcept
    {
        if (_selectorMonitorArmed)
        {
            EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        }
        _selectorMonitorArmed = false;
        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
    }

    bool OpenFloorMeasurementController::SelectorRemoved() const noexcept
    {
        return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
    }

    bool OpenFloorMeasurementController::LogTimingSample(const OpenFloorMeasurementCycle& cycle)
    {
        OpenFloorTimingRow row{};
        row.mono_time_us = cycle.masterTimeUs;
        row.control_tick_sequence = cycle.controlTickSequence;
        row.dt_us = cycle.dtUs;
        row.section_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
        row.control_start_us = cycle.controlTiming.controlStartUs;
        row.control_end_us = cycle.controlTiming.controlEndUs;
        row.pwm_latch_us = cycle.controlTiming.pwmLatchUs;
        row.encoder_latch_us = cycle.controlTiming.encoderLatchUs;
        row.encoder_read_done_us = cycle.controlTiming.encoderReadDoneUs;
        row.ukf_predict_start_us = cycle.controlTiming.ukfPredictStartUs;
        row.ukf_predict_end_us = cycle.controlTiming.ukfPredictEndUs;
        row.ukf_predict_duration_us = cycle.controlTiming.ukfPredictDurationUs;
        row.ukf_update_start_us = cycle.controlTiming.ukfUpdateStartUs;
        row.ukf_update_end_us = cycle.controlTiming.ukfUpdateEndUs;
        row.ukf_update_duration_us = cycle.controlTiming.ukfUpdateDurationUs;
        row.imu_drdy_us = cycle.sensorSnapshot.imuTiming.drdyUs;
        row.imu_read_start_us = cycle.sensorSnapshot.imuTiming.readStartUs;
        row.imu_read_done_us = cycle.sensorSnapshot.imuTiming.readDoneUs;
        row.front_led_on_us = cycle.sensorSnapshot.frontTiming.ledOnCommandUs;
        row.front_adc_on_us = cycle.sensorSnapshot.frontTiming.adcOnSampleUs;
        row.front_led_off_us = cycle.sensorSnapshot.frontTiming.ledOffCommandUs;
        row.front_adc_off_us = cycle.sensorSnapshot.frontTiming.adcOffSampleUs;
        row.front_ready_us = cycle.sensorSnapshot.frontTiming.observationReadyUs;
        row.left_led_on_us = cycle.sensorSnapshot.leftTiming.ledOnCommandUs;
        row.left_adc_on_us = cycle.sensorSnapshot.leftTiming.adcOnSampleUs;
        row.left_led_off_us = cycle.sensorSnapshot.leftTiming.ledOffCommandUs;
        row.left_adc_off_us = cycle.sensorSnapshot.leftTiming.adcOffSampleUs;
        row.left_ready_us = cycle.sensorSnapshot.leftTiming.observationReadyUs;
        row.right_led_on_us = cycle.sensorSnapshot.rightTiming.ledOnCommandUs;
        row.right_adc_on_us = cycle.sensorSnapshot.rightTiming.adcOnSampleUs;
        row.right_led_off_us = cycle.sensorSnapshot.rightTiming.ledOffCommandUs;
        row.right_adc_off_us = cycle.sensorSnapshot.rightTiming.adcOffSampleUs;
        row.right_ready_us = cycle.sensorSnapshot.rightTiming.observationReadyUs;
        row.wall_adc_cfg_before_start = cycle.sensorSnapshot.wallSensorAdcCfgBeforeStart;
        row.wall_adc_gc_before_start = cycle.sensorSnapshot.wallSensorAdcGcBeforeStart;
        row.wall_adc_cfg_after_start = cycle.sensorSnapshot.wallSensorAdcCfgAfterStart;
        row.wall_adc_gc_after_start = cycle.sensorSnapshot.wallSensorAdcGcAfterStart;
        row.wall_adc_target_cfg = cycle.sensorSnapshot.wallSensorAdcTargetCfg;
        row.wall_adc_ipg_clock_hz = cycle.sensorSnapshot.wallSensorAdcIpgClockHz;
        row.cycle_counter_start = cycle.controlTiming.cycleCounterStart;
        row.cycle_counter_end = cycle.controlTiming.cycleCounterEnd;
        return _runtime.LogUtilityDataRow(row);
    }

    bool OpenFloorMeasurementController::LogMainSample(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        const CommandTelemetrySnapshot& commandTelemetry)
    {
        const bool encoderValid = cycle.driveTelemetry.encoderObservationValid;
        const bool imuValid = std::isfinite(cycle.sensorSnapshot.gyroRawRadps);
        const float maxRangeM = MazeMap::PlantParams::Default().noHitRangeM;
        MazeMap::WallObs frontLeftObs{};
        MazeMap::WallObs frontRightObs{};
        DriveBase::BuildLoggedFrontPairObservations(
            cycle.sensorSnapshot,
            maxRangeM,
            frontLeftObs,
            frontRightObs);
        const MazeMap::WallObs leftObs =
            DriveBase::BuildLoggedLeftSideObservation(cycle.sensorSnapshot, maxRangeM);
        const MazeMap::WallObs rightObs =
            DriveBase::BuildLoggedRightSideObservation(cycle.sensorSnapshot, maxRangeM);
        const MazeMap::VehicleState::StateVector& estimatorState = cycle.estimatorState;

        OpenFloorMainRow row{};
        row.master_time_us = cycle.masterTimeUs;
        row.control_tick_sequence = cycle.controlTickSequence;
        row.dt_us = cycle.dtUs;
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
        row.clipping_flags = cycle.clippingFlags;
        row.saturation_flags = commandTelemetry.driveTelemetry.saturationFlags;
        row.watchdog_flags = cycle.watchdogFlags;
        row.measurement_flags = Runtime::BuildOpenFloorMeasurementFlags(
            labels,
            cycle,
            encoderValid,
            imuValid,
            frontLeftObs,
            frontRightObs,
            leftObs,
            rightObs);
        row.ukf_mode_id = cycle.driveTelemetry.ukfModeId;
        row.ukf_yaw_valid_for_feedforward = cycle.driveTelemetry.ukfYawValidForFeedforward;
        row.bias_update_enabled = cycle.driveTelemetry.ukfBiasUpdateEnabled;
        row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        row.gyro_bias_anchor_radps = cycle.driveTelemetry.ukfGyroBiasAnchorRadps;
        row.yaw_consistency_lp_radps = cycle.driveTelemetry.ukfYawConsistencyLowPassRadps;
        row.yaw_window_mismatch_rad = cycle.driveTelemetry.ukfYawWindowMismatchRad;
        row.nhc_sigma_mps = cycle.driveTelemetry.ukfNhcSigmaMps;
        row.nhc_residual_mps = cycle.driveTelemetry.ukfNhcResidualMps;
        row.nhc_residual_sigma = cycle.driveTelemetry.ukfNhcResidualSigma;
        row.measured_linear_speed_mps = cycle.measuredLinearSpeedMps;
        row.measured_angular_speed_radps = cycle.measuredAngularSpeedRadps;
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
        row.encoder_timestamp_us = cycle.controlTiming.encoderReadDoneUs;
        row.left_encoder_count = cycle.driveTelemetry.leftEncoderCount;
        row.right_encoder_count = cycle.driveTelemetry.rightEncoderCount;
        row.left_encoder_omega_radps = cycle.driveTelemetry.leftEncoderOmegaRadps;
        row.right_encoder_omega_radps = cycle.driveTelemetry.rightEncoderOmegaRadps;
        row.left_encoder_distance_m = cycle.driveTelemetry.leftDistanceM;
        row.right_encoder_distance_m = cycle.driveTelemetry.rightDistanceM;
        row.left_encoder_velocity_mps = cycle.driveTelemetry.leftVelocityMps;
        row.right_encoder_velocity_mps = cycle.driveTelemetry.rightVelocityMps;
        row.imu_timestamp_us = cycle.sensorSnapshot.imuTiming.readDoneUs;
        row.imu_status = cycle.sensorSnapshot.imuBackLeft.status;
        row.imu_interrupt_high = cycle.sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        row.accel_bias_valid = cycle.sensorSnapshot.accelBiasValid ? 1U : 0U;
        row.imu_gyro_x = cycle.sensorSnapshot.imuBackLeft.gyroX;
        row.imu_gyro_y = cycle.sensorSnapshot.imuBackLeft.gyroY;
        row.imu_gyro_z = cycle.sensorSnapshot.imuBackLeft.gyroZ;
        row.imu_accel_x = cycle.sensorSnapshot.imuBackLeft.accelX;
        row.imu_accel_y = cycle.sensorSnapshot.imuBackLeft.accelY;
        row.imu_accel_z = cycle.sensorSnapshot.imuBackLeft.accelZ;
        row.imu_temp = cycle.sensorSnapshot.imuBackLeft.temp;
        row.gyro_raw_radps = cycle.sensorSnapshot.gyroRawRadps;
        row.gyro_bias_radps = cycle.sensorSnapshot.gyroBiasRadps;
        row.gyro_radps = cycle.sensorSnapshot.gyroRadps;
        row.accel_body_x_mps2 = cycle.sensorSnapshot.accelBodyXMps2;
        row.accel_body_y_mps2 = cycle.sensorSnapshot.accelBodyYMps2;
        row.planar_accel_mps2 = cycle.planarAccelMps2;
        row.front_timestamp_us = cycle.sensorSnapshot.frontTiming.observationReadyUs;
        row.left_timestamp_us = cycle.sensorSnapshot.leftTiming.observationReadyUs;
        row.right_timestamp_us = cycle.sensorSnapshot.rightTiming.observationReadyUs;
        row.front_left_obs_class = static_cast<std::uint8_t>(frontLeftObs.cls);
        row.front_right_obs_class = static_cast<std::uint8_t>(frontRightObs.cls);
        row.left_obs_class = static_cast<std::uint8_t>(leftObs.cls);
        row.right_obs_class = static_cast<std::uint8_t>(rightObs.cls);
        row.front_left_obs_rho_m = frontLeftObs.rho;
        row.front_right_obs_rho_m = frontRightObs.rho;
        row.left_obs_rho_m = leftObs.rho;
        row.right_obs_rho_m = rightObs.rho;
        row.front_left_obs_confidence = frontLeftObs.confidence;
        row.front_right_obs_confidence = frontRightObs.confidence;
        row.left_obs_confidence = leftObs.confidence;
        row.right_obs_confidence = rightObs.confidence;
        row.fan_duty_cycle = cycle.fanDutyCycle;
        return _runtime.LogUtilityDataRow(row);
    }

    CommandTelemetrySnapshot OpenFloorMeasurementController::CaptureDriveCommandTelemetry() const noexcept
    {
        CommandTelemetrySnapshot snapshot{};
        snapshot.linearCommandMps = _drive.GetLastLinearCommandMps();
        snapshot.angularCommandRadps = _drive.GetLastAngularCommandRadps();
        snapshot.driveTelemetry = _drive.GetTelemetry();
        return snapshot;
    }

    OpenFloorPrimitiveId OpenFloorMeasurementController::PrimitiveIdForCode(
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

    OpenFloorDirectionId OpenFloorMeasurementController::DirectionIdForCode(
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

    OpenFloorSpeedBin OpenFloorMeasurementController::SpeedBinForIndex(const std::size_t speedIndex) noexcept
    {
        return (speedIndex == 0U) ? OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? OpenFloorSpeedBin::Medium :
            OpenFloorSpeedBin::High;
    }

    OpenFloorPhaseId OpenFloorMeasurementController::StraightPhaseForProgress(const float progress) noexcept
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

    OpenFloorPhaseId OpenFloorMeasurementController::TurnPhaseForProgress(const float progress) noexcept
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

    OpenFloorPhaseId OpenFloorMeasurementController::SmoothPhaseForProgress(const float progress) noexcept
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

    OpenFloorPhaseId OpenFloorMeasurementController::ManeuverPhaseForProgress(
        const MazeMap::ManeuverCode code,
        const float progress) noexcept
    {
        return (code == MazeMap::IP90 || code == MazeMap::IP90_M) ?
            TurnPhaseForProgress(progress) :
            (code == MazeMap::S1 || code == MazeMap::S2) ?
                StraightPhaseForProgress(progress) :
                SmoothPhaseForProgress(progress);
    }

    void OpenFloorMeasurementController::SwitchPhase(
        LoopController::TickServices& services,
        const ModeWorkCallback callback) noexcept
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = callback;
        callbacks.context = this;
        services.SetNextModeWorkCallbacks(callbacks);
    }

    void OpenFloorMeasurementController::StartInterPhaseHold(
        const HoldContinuation continuation) noexcept
    {
        _driveService.Cancel();
        _holdState = {};
        _holdState.continuation = continuation;
    }

    Drive::OperationMode OpenFloorMeasurementController::OpenFloorOperationMode() const noexcept
    {
        return Drive::OperationMode::OpenFloor;
    }

    MotionLimits OpenFloorMeasurementController::StraightLimits(const float speedMps) const noexcept
    {
        return BuildPrimaryDiagnosticLimits(_vehicle, speedMps);
    }

    MotionLimits OpenFloorMeasurementController::TurnLimits(const float maxOmegaRadps) const noexcept
    {
        MotionLimits limits = BuildPrimaryDiagnosticLimits(_vehicle, 0.0f);
        limits.maxAngularSpeedRadps = maxOmegaRadps;
        return limits;
    }

    float OpenFloorMeasurementController::CurrentManeuverProgress(
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

    bool OpenFloorMeasurementController::CheckTimingFault(
        const OpenFloorMeasurementCycle& cycle,
        LoopController::TickServices& services)
    {
        if (!cycle.estimatorFault && !cycle.selectorJumperRemoved)
        {
            return false;
        }

        services.Fault(cycle.estimatorFault ?
            "Estimator fault during timing capture" :
            kPrimaryDiagnosticSelectorRemovedReason);
        return true;
    }

    bool OpenFloorMeasurementController::CheckMainFault(
        OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        LoopController::TickServices& services,
        const char* estimatorReason,
        const CommandTelemetrySnapshot& commandTelemetry)
    {
        if (cycle.selectorJumperRemoved)
        {
            labels.abortMarker = true;
        }
        if (!cycle.estimatorFault && !cycle.selectorJumperRemoved)
        {
            return false;
        }

        StagePendingMainSample(labels, cycle, commandTelemetry);
        services.Fault(cycle.estimatorFault ? estimatorReason : kPrimaryDiagnosticSelectorRemovedReason);
        return true;
    }

    bool OpenFloorMeasurementController::StartNextLaunchSample()
    {
        while (_launchState.magnitudeIndex < MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
        {
            if (_launchState.repeatIteration >= DiagnosticConfig::kLaunchRepeatsPerMagnitude)
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
            _launchState.stage = LaunchState::Stage::Pulse;
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
            _launchState.stationaryCheckState.SetStateVector(_drive.GetEstimatorStateVector());
            _launchState.previousStationary = _launchState.stationaryCheckState.IsStationary();
            _launchState.launchFlippedStationary = false;
            _launchState.settleStartMs = 0U;
            return true;
        }

        return false;
    }

    bool OpenFloorMeasurementController::StartNextStraightSample()
    {
        const float straightDistanceM = MazeMap::OpenFloorStrEquivalentDistanceMeters(4U);
        while (_straightState.speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size())
        {
            if (_straightState.repeatIteration >= DiagnosticConfig::kStraightRepeatsPerSpeed)
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

    bool OpenFloorMeasurementController::StartNextYawSample()
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
            if (_yawState.primitiveIndex >= 3U)
            {
                _yawState.primitiveIndex = 0U;
                ++_yawState.repeatIteration;
            }

            OpenFloorPrimitiveId primitiveId = OpenFloorPrimitiveId::None;
            OpenFloorDirectionId directionId = OpenFloorDirectionId::None;
            float angleRad = 0.0f;
            switch (primitiveIndex)
            {
            case 0U:
                primitiveId = OpenFloorPrimitiveId::Ip90;
                directionId = OpenFloorDirectionId::Clockwise;
                angleRad = HALF_PI_F;
                break;
            case 1U:
                primitiveId = OpenFloorPrimitiveId::Ip90M;
                directionId = OpenFloorDirectionId::CounterClockwise;
                angleRad = -HALF_PI_F;
                break;
            case 2U:
            default:
                primitiveId = OpenFloorPrimitiveId::Ip180;
                directionId = OpenFloorDirectionId::Flip;
                angleRad = PI_F;
                break;
            }

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

    bool OpenFloorMeasurementController::StartNextSmoothEntry()
    {
        while (_smoothState.speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size())
        {
            if (!_smoothState.queueLoaded)
            {
                if (!BuildPrimaryDiagnosticSmoothQueue(
                        _vehicle,
                        _smoothState.speedIndex,
                        PrimaryDiagnosticSmoothSpeedMps(_smoothState.speedIndex),
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
            _driveService.SetLimits(StraightLimits(PrimaryDiagnosticSmoothSpeedMps(_smoothState.speedIndex)));
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

    void OpenFloorMeasurementController::ResetLoopPhase(const bool clockwise) noexcept
    {
        _loopState = {};
        _loopState.clockwise = clockwise;
        _loopState.repeatIndex = 1U;
    }

    bool OpenFloorMeasurementController::StartNextLoopEntry()
    {
        while (_loopState.repeatIndex <= DiagnosticConfig::kLoopRepeats)
        {
            if (!_loopState.queueLoaded)
            {
                if (!BuildPrimaryDiagnosticLoopQueue(_vehicle, _loopState.clockwise, _loopState.queue))
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

    LoopController::PauseDisposition OpenFloorMeasurementController::OnPauseGranted(
        const LoopController::PauseContext& pause)
    {
        (void)pause;
        if (_pauseAction != PauseAction::TimingToMain)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Primary diagnostic pause granted without a pending timing transition");
        }

        if (_pendingTimingSampleValid)
        {
            _pendingTimingCycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
            if (!LogTimingSample(_pendingTimingCycle))
            {
                return LoopController::PauseDisposition::StopByRuntime(
                    "Primary diagnostic timing log write failed during timing transition");
            }
            _pendingTimingSampleValid = false;
            _pendingTimingCycle = {};
        }

        if (_timingLogOpen)
        {
            CloseTimingLog();
        }
        if (!_mainLogOpen && !BeginMainLog())
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Primary diagnostic main log setup failed after timing capture");
        }

        _pauseAction = PauseAction::None;
        return LoopController::PauseDisposition::Resume();
    }

    void OpenFloorMeasurementController::TeardownOnRuntimeFault(
        void* context,
        const char* reason) noexcept
    {
        (void)reason;
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        if (self == nullptr)
        {
            return;
        }

        self->ReleaseSelectorMonitor();
        self->_startupCalibration.Cancel();
        self->_driveService.Cancel();
        self->_drive.Brake();
        self->_drive.UseNominalWheelControlProfile();
        self->_timingLogOpen = false;
        self->_mainLogOpen = false;
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::PauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->OnPauseGranted(pause) :
            LoopController::PauseDisposition::StopByRuntime(
                "Primary diagnostic pause callback context was null");
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->TimingTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::InterPhaseHoldTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->InterPhaseHoldTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StaticTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->StaticTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LaunchTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->LaunchTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StraightTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->StraightTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::YawTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->YawTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::SmoothTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->SmoothTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopCwTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->LoopCwTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopCcwTickThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->LoopCcwTick(loopEndTimeUs, state, services) :
            LoopController::ControlVector::Brake;
    }

    bool OpenFloorMeasurementController::SwitchToHoldContinuation(
        LoopController::TickServices& services)
    {
        switch (_holdState.continuation)
        {
        case HoldContinuation::Launch:
            _launchState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::LaunchTickThunk);
            return true;
        case HoldContinuation::Straight:
            _straightState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::StraightTickThunk);
            return true;
        case HoldContinuation::Yaw:
            _yawState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::YawTickThunk);
            return true;
        case HoldContinuation::Smooth:
            _smoothState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::SmoothTickThunk);
            return true;
        case HoldContinuation::LoopCw:
            ResetLoopPhase(true);
            SwitchPhase(services, &OpenFloorMeasurementController::LoopCwTickThunk);
            return true;
        case HoldContinuation::LoopCcw:
            ResetLoopPhase(false);
            SwitchPhase(services, &OpenFloorMeasurementController::LoopCcwTickThunk);
            return true;
        case HoldContinuation::Complete:
            services.RequestEndLoop();
            return true;
        case HoldContinuation::None:
        default:
            return false;
        }
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!FlushPendingTimingSample(services, "Primary diagnostic timing log write failed"))
        {
            return stopControl;
        }
        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        if (CheckTimingFault(cycle, services))
        {
            StagePendingTimingSample(cycle);
            return stopControl;
        }
        StagePendingTimingSample(cycle);

        ++_timingState.tickIndex;
        if (_timingState.tickIndex >= DiagnosticConfig::kTimingCaptureCycles)
        {
            _pauseAction = PauseAction::TimingToMain;
            _staticState = {};
            SwitchPhase(services, &OpenFloorMeasurementController::StaticTickThunk);

            LoopController::PauseRequest request{};
            request.onPauseGranted = &OpenFloorMeasurementController::PauseThunk;
            request.reason = "open_floor_timing_to_main";
            request.flushLogsBeforeGrant = true;
            request.resetClockOnResume = true;
            services.RequestPause(request);
        }

        return stopControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::InterPhaseHoldTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (_mainLogOpen &&
            !FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (SelectorRemoved())
        {
            services.Fault(kPrimaryDiagnosticSelectorRemovedReason);
            return stopControl;
        }
        if (!state.estimatorHealthy)
        {
            services.Fault("Estimator fault during inter-phase hold");
            return stopControl;
        }

        if (!_holdState.initialized)
        {
            _holdState.initialized = true;
            _holdState.deadlineMs = millis() + kPrimaryDiagnosticInterPhaseHoldMs;
        }
        if (static_cast<long>(_holdState.deadlineMs - millis()) <= 0)
        {
            if (!SwitchToHoldContinuation(services))
            {
                services.Fault("Primary diagnostic hold continuation was not configured");
            }
        }

        return stopControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StaticTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector control = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot nextCommandTelemetry = BuildRawCommandTelemetrySnapshot(control);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
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

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
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
        if (CheckMainFault(
                _staticState.labels,
                cycle,
                services,
                "Estimator fault during static hold",
                nextCommandTelemetry))
        {
            return control;
        }
        StagePendingMainSample(_staticState.labels, cycle, nextCommandTelemetry);

        if (static_cast<long>(_staticState.deadlineMs - millis()) <= 0)
        {
            StartInterPhaseHold(HoldContinuation::Launch);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
        }

        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LaunchTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_launchState.active && !StartNextLaunchSample())
        {
            StartInterPhaseHold(HoldContinuation::Straight);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        _launchState.stationaryCheckState.SetStateVector(_drive.GetEstimatorStateVector());
        const bool estimatorStationary = _launchState.stationaryCheckState.IsStationary();
        const unsigned long nowMs = millis();

        if (_launchState.stage == LaunchState::Stage::Pulse)
        {
            if (_launchState.previousStationary && !estimatorStationary)
            {
                _launchState.launchFlippedStationary = true;
            }
            _launchState.previousStationary = estimatorStationary;
            _launchState.labels.phaseId = OpenFloorPhaseId::LaunchPulse;
            _launchState.labels.progressNorm = (MazeMap::kOpenFloorLaunchPulseMs > 0UL) ?
                (std::clamp)(
                    static_cast<float>(MazeMap::kOpenFloorLaunchPulseMs - (std::max)(0L, static_cast<long>(_launchState.pulseDeadlineMs - nowMs))) /
                        static_cast<float>(MazeMap::kOpenFloorLaunchPulseMs),
                    0.0f,
                    1.0f) :
                1.0f;
        }
        else
        {
            if (!estimatorStationary)
            {
                _launchState.settleStartMs = 0U;
                _launchState.labels.phaseId = OpenFloorPhaseId::Brake;
                _launchState.labels.progressNorm = 0.0f;
            }
            else
            {
                if (_launchState.settleStartMs == 0U)
                {
                    _launchState.settleStartMs = nowMs;
                }
                _launchState.labels.phaseId = OpenFloorPhaseId::Hold;
                _launchState.labels.progressNorm = (MazeMap::kOpenFloorLaunchSettleMs > 0UL) ?
                    (std::clamp)(
                        static_cast<float>(nowMs - _launchState.settleStartMs) /
                            static_cast<float>(MazeMap::kOpenFloorLaunchSettleMs),
                        0.0f,
                        1.0f) :
                1.0f;
            }
        }

        const LoopController::ControlVector control =
            (_launchState.stage == LaunchState::Stage::Pulse) ?
                LoopController::ControlVector::RawMotorPwm(
                    _launchState.signedDriveCommand,
                    _launchState.signedDriveCommand) :
                stopControl;
        const CommandTelemetrySnapshot nextCommandTelemetry = BuildRawCommandTelemetrySnapshot(control);

        if (CheckMainFault(
                _launchState.labels,
                cycle,
                services,
                "Estimator fault during launch section",
                stopTelemetry))
        {
            return stopControl;
        }
        StagePendingMainSample(_launchState.labels, cycle, nextCommandTelemetry);

        if (_launchState.stage == LaunchState::Stage::Pulse)
        {
            if (static_cast<long>(_launchState.pulseDeadlineMs - nowMs) <= 0)
            {
                if (_launchState.launchFlippedStationary)
                {
                    _launchState.stage = LaunchState::Stage::Settle;
                    _launchState.settleStartMs = 0U;
                }
                else
                {
                    _launchState.active = false;
                }
            }

            return control;
        }

        if (estimatorStationary &&
            (_launchState.settleStartMs != 0U) &&
            ((nowMs - _launchState.settleStartMs) >= MazeMap::kOpenFloorLaunchSettleMs))
        {
            _launchState.active = false;
        }

        return stopControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StraightTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_straightState.active && !StartNextStraightSample())
        {
            StartInterPhaseHold(HoldContinuation::Yaw);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _straightState.startDistanceM);
        _straightState.labels.progressNorm = (std::clamp)(
            traveledM / _straightState.distanceM,
            0.0f,
            1.0f);
        _straightState.labels.phaseId = StraightPhaseForProgress(_straightState.labels.progressNorm);
        if (CheckMainFault(
                _straightState.labels,
                cycle,
                services,
                "Estimator fault during straight section",
                stopTelemetry))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry();

        StagePendingMainSample(_straightState.labels, cycle, nextCommandTelemetry);
        if (done)
        {
            _straightState.active = false;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::YawTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_yawState.active && !StartNextYawSample())
        {
            StartInterPhaseHold(HoldContinuation::Smooth);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        const float remainingRad = std::fabs(AngleErrorRad(_yawState.targetYawRad, _drive.GetPose().yawRad));
        _yawState.labels.progressNorm = (_yawState.targetMagnitudeRad > 0.0f) ?
            (std::clamp)(1.0f - (remainingRad / _yawState.targetMagnitudeRad), 0.0f, 1.0f) :
            1.0f;
        _yawState.labels.phaseId = TurnPhaseForProgress(_yawState.labels.progressNorm);
        if (CheckMainFault(
                _yawState.labels,
                cycle,
                services,
                "Estimator fault during yaw section",
                stopTelemetry))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry();

        StagePendingMainSample(_yawState.labels, cycle, nextCommandTelemetry);
        if (done)
        {
            _yawState.active = false;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::SmoothTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_smoothState.active && !StartNextSmoothEntry())
        {
            StartInterPhaseHold(HoldContinuation::LoopCw);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        const MazeMap::ManeuverCode code = _smoothState.queue[_smoothState.entryIndex].getCode();
        _smoothState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _smoothState.startDistanceM,
            _smoothState.totalDistanceM,
            _smoothState.targetYawRad,
            _smoothState.targetMagnitudeRad);
        _smoothState.labels.phaseId = ManeuverPhaseForProgress(code, _smoothState.labels.progressNorm);
        if (CheckMainFault(
                _smoothState.labels,
                cycle,
                services,
                "Estimator fault during smooth section",
                stopTelemetry))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry();

        StagePendingMainSample(_smoothState.labels, cycle, nextCommandTelemetry);
        if (done)
        {
            _smoothState.active = false;
            ++_smoothState.entryIndex;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopCwTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_loopState.active && !StartNextLoopEntry())
        {
            StartInterPhaseHold(HoldContinuation::LoopCcw);
            SwitchPhase(services, &OpenFloorMeasurementController::InterPhaseHoldTickThunk);
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        const MazeMap::ManeuverCode code = _loopState.queue[_loopState.entryIndex].getCode();
        _loopState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _loopState.startDistanceM,
            _loopState.totalDistanceM,
            _loopState.targetYawRad,
            _loopState.targetMagnitudeRad);
        _loopState.labels.phaseId = ManeuverPhaseForProgress(code, _loopState.labels.progressNorm);
        if (CheckMainFault(
                _loopState.labels,
                cycle,
                services,
                "Estimator fault during clockwise loop section",
                stopTelemetry))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry();

        StagePendingMainSample(_loopState.labels, cycle, nextCommandTelemetry);
        if (done)
        {
            _loopState.active = false;
            ++_loopState.entryIndex;
            return stopControl;
        }
        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopCcwTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        const CommandTelemetrySnapshot stopTelemetry = BuildRawCommandTelemetrySnapshot(stopControl);
        if (!FlushPendingMainSample(services, "Primary diagnostic main log write failed"))
        {
            return stopControl;
        }
        if (!_loopState.active && !StartNextLoopEntry())
        {
            services.RequestEndLoop();
            return stopControl;
        }

        OpenFloorMeasurementCycle cycle{};
        PopulateCycleFromState(state, cycle);
        const MazeMap::ManeuverCode code = _loopState.queue[_loopState.entryIndex].getCode();
        _loopState.labels.progressNorm = CurrentManeuverProgress(
            code,
            _loopState.startDistanceM,
            _loopState.totalDistanceM,
            _loopState.targetYawRad,
            _loopState.targetMagnitudeRad);
        _loopState.labels.phaseId = ManeuverPhaseForProgress(code, _loopState.labels.progressNorm);
        if (CheckMainFault(
                _loopState.labels,
                cycle,
                services,
                "Estimator fault during counter-clockwise loop section",
                stopTelemetry))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = _driveService.GetNextControls(done);
        const LoopController::ControlVector control = done ? stopControl : candidateControl;
        const CommandTelemetrySnapshot nextCommandTelemetry =
            done ? stopTelemetry : CaptureDriveCommandTelemetry();

        StagePendingMainSample(_loopState.labels, cycle, nextCommandTelemetry);
        if (done)
        {
            _loopState.active = false;
            ++_loopState.entryIndex;
            return stopControl;
        }
        return control;
    }

    IApplicationMode& GetDiagnosticMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PrimaryDiagnostic,
            BootModeCategory::Utility,
            "primary_diagnostic",
            "Run the ordered open-floor measurement battery with timing, static, launch, straight, yaw, smooth, and closed maneuver loops.",
            "logging.txt, open_floor_timing.mmlog, open_floor_main.mmlog",
            &GetDiagnosticMode,
            "GetDiagnosticMode",
            "OpenFloorMeasurementController.cpp",
            "timing capture; static hold; launch PWM pulses; straight drive tests; yaw drive tests; smooth maneuver sweep; clockwise closed maneuver loop; counter-clockwise closed maneuver loop",
            "DiagnosticConfig linear limits; OpenFloorMeasurementSpec speed bins; shared startup calibration; shared drive service",
            "Inter-phase 500 ms brake holds; smooth phase uses the current hand-picked closed maneuver sequence; loop sections are maneuver-driven",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
