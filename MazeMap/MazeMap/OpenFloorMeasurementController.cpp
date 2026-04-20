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

namespace
{
    constexpr const char* kPrimaryDiagnosticStableId = "primary_diagnostic";
    constexpr const char* kPrimaryDiagnosticSelectorRemovedReason =
        "Primary diagnostic selector jumper removed";
    constexpr std::uint16_t kPrimaryDiagnosticCompletionHoldMs = 250U;
    constexpr float kPrimaryDiagnosticMaxSmoothSpeedMps = MazeMap::kOpenFloorSmoothSpeedBinsMps[2];
    constexpr MazeMap::ManeuverCode kPrimaryDiagnosticSpeedChangeStraightCode = MazeMap::S1;

    // This straight/diagonal-valid closed cycle stays inside a 4x4-cell open-floor box while
    // leaving room for one half-cell S1 speed-change straight before each retained smooth-speed
    // bin. Starting from half-step (2,3) facing +Y/Up, the bins run:
    //   bin 1: S1 lead-in to (2,4), loop
    //   bin 2: S1 lead-in to (2,5), loop
    //   bin 3: S1 lead-in to (2,6), loop, S1 stop straight to (2,7)
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
}

using MazeMap::App::Internal::Runtime::OpenFloorMainRow;

namespace MazeMap::App::Internal
{
    class OpenFloorMeasurementController final : public IApplicationMode
    {
    public:
        explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _vehicle(runtime.SpeedVehicle())
            , _sensors(runtime.Sensors())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        bool Begin() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&OpenFloorMeasurementController::TeardownOnRuntimeFault, this, kPrimaryDiagnosticStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return _runtime.FailActiveMode("Primary diagnostic hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:primary_diagnostic");
            (void)_runtime.AppendTextLogLine("Primary diagnostic mode");
            (void)_runtime.AppendTextLogLine("Shared-service open-floor smooth-cycle diagnostic sweep");
            (void)_runtime.AppendTextLogLine(
                "Hard-coded 26-maneuver non-in-place cycle, replayed once per retained smooth-speed bin");

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

            if (!BeginMainLog())
            {
                return _runtime.FailActiveMode("Primary diagnostic main log setup failed");
            }

            return true;
        }

        void Run() override
        {
            _phase = Phase::LaunchStartupHold;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &OpenFloorMeasurementController::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)_runtime.FailActiveMode("Primary diagnostic loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                const bool completed =
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();

                if (completed)
                {
                    (void)_runtime.AppendTextLogLine("Primary diagnostic complete");
                }
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
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            LaunchStartupHold,
            RunStartupHold,
            LaunchSmoothCycleSpeed,
            LaunchSmoothCycleEntry,
            RunSmoothCycleEntry,
            LaunchCompletionHold,
            RunCompletionHold,
            Complete
        };

        enum class ProgressMode : std::uint8_t
        {
            None,
            Time,
            Distance,
        };

        struct ActiveLogState final
        {
            OpenFloorMeasurementLabels labels{};
            ProgressMode progressMode{ ProgressMode::None };
            float startDistanceM{};
            float totalDistanceM{};
            unsigned long startMs{};
            unsigned long durationMs{};
        };

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
        {
            (void)reason;
            auto* const self = static_cast<OpenFloorMeasurementController*>(context);
            if (self == nullptr)
            {
                return;
            }

            self->ReleaseSelectorMonitor();
            self->_phase = Phase::Idle;
            self->_startupCalibration.Cancel();
            self->_driveService.Cancel();
            self->_drive.Brake();
            self->_drive.UseNominalWheelControlProfile();
            self->_mainLogOpen = false;
            self->ResetActiveLogState();
        }

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<OpenFloorMeasurementController*>(context);
            if (self == nullptr)
            {
                services.Fault("Primary diagnostic callback context was not installed");
                return LoopController::ControlVector::Brake;
            }

            return self->RunTick(loopEndTimeUs, state, services);
        }

        static MazeMap::OpenFloorPrimitiveId PrimitiveIdForCode(const MazeMap::ManeuverCode code) noexcept
        {
            switch (code)
            {
            case MazeMap::S1:
                return MazeMap::OpenFloorPrimitiveId::Str1;
            case MazeMap::S45SD:
                return MazeMap::OpenFloorPrimitiveId::S45sd;
            case MazeMap::S45SD_M:
                return MazeMap::OpenFloorPrimitiveId::S45sdM;
            case MazeMap::S45SS:
                return MazeMap::OpenFloorPrimitiveId::S45ss;
            case MazeMap::S45SS_M:
                return MazeMap::OpenFloorPrimitiveId::S45ssM;
            case MazeMap::S45LS:
                return MazeMap::OpenFloorPrimitiveId::S45ls;
            case MazeMap::S45LS_M:
                return MazeMap::OpenFloorPrimitiveId::S45lsM;
            case MazeMap::S45LD:
                return MazeMap::OpenFloorPrimitiveId::S45ld;
            case MazeMap::S45LD_M:
                return MazeMap::OpenFloorPrimitiveId::S45ldM;
            case MazeMap::S90SD:
                return MazeMap::OpenFloorPrimitiveId::S90sd;
            case MazeMap::S90SD_M:
                return MazeMap::OpenFloorPrimitiveId::S90sdM;
            case MazeMap::S90SS:
                return MazeMap::OpenFloorPrimitiveId::S90ss;
            case MazeMap::S90SS_M:
                return MazeMap::OpenFloorPrimitiveId::S90ssM;
            case MazeMap::S90LS:
                return MazeMap::OpenFloorPrimitiveId::S90ls;
            case MazeMap::S90LS_M:
                return MazeMap::OpenFloorPrimitiveId::S90lsM;
            case MazeMap::S135SD:
                return MazeMap::OpenFloorPrimitiveId::S135sd;
            case MazeMap::S135SD_M:
                return MazeMap::OpenFloorPrimitiveId::S135sdM;
            case MazeMap::S135SS:
                return MazeMap::OpenFloorPrimitiveId::S135ss;
            case MazeMap::S135SS_M:
                return MazeMap::OpenFloorPrimitiveId::S135ssM;
            case MazeMap::S135LS:
                return MazeMap::OpenFloorPrimitiveId::S135ls;
            case MazeMap::S135LS_M:
                return MazeMap::OpenFloorPrimitiveId::S135lsM;
            case MazeMap::S135LD:
                return MazeMap::OpenFloorPrimitiveId::S135ld;
            case MazeMap::S135LD_M:
                return MazeMap::OpenFloorPrimitiveId::S135ldM;
            case MazeMap::S180SS:
                return MazeMap::OpenFloorPrimitiveId::S180ss;
            case MazeMap::S180SS_M:
                return MazeMap::OpenFloorPrimitiveId::S180ssM;
            case MazeMap::S180LS:
                return MazeMap::OpenFloorPrimitiveId::S180ls;
            case MazeMap::S180LS_M:
                return MazeMap::OpenFloorPrimitiveId::S180lsM;
            default:
                return MazeMap::OpenFloorPrimitiveId::None;
            }
        }

        static MazeMap::OpenFloorDirectionId DirectionIdForCode(const MazeMap::ManeuverCode code) noexcept
        {
            if (code == MazeMap::S1)
            {
                return MazeMap::OpenFloorDirectionId::Positive;
            }

            const MazeMap::OpenFloorPrimitiveId primitiveId = PrimitiveIdForCode(code);
            if (primitiveId == MazeMap::OpenFloorPrimitiveId::None)
            {
                return MazeMap::OpenFloorDirectionId::None;
            }

            return MazeMap::OpenFloorPrimitiveIsMirrored(primitiveId) ?
                MazeMap::OpenFloorDirectionId::Left :
                MazeMap::OpenFloorDirectionId::Right;
        }

        static MazeMap::OpenFloorSpeedBin SpeedBinForIndex(const std::uint8_t speedIndex) noexcept
        {
            switch (speedIndex)
            {
            case 0U:
                return MazeMap::OpenFloorSpeedBin::Low;
            case 1U:
                return MazeMap::OpenFloorSpeedBin::Medium;
            case 2U:
            default:
                return MazeMap::OpenFloorSpeedBin::High;
            }
        }

        static MazeMap::OpenFloorPhaseId SmoothPhaseForProgress(const float progress) noexcept
        {
            if (progress < 0.20f)
            {
                return MazeMap::OpenFloorPhaseId::Entry;
            }
            if (progress > 0.85f)
            {
                return MazeMap::OpenFloorPhaseId::Exit;
            }
            return MazeMap::OpenFloorPhaseId::Middle;
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
            options.workPlan.useWallUpdates = false;
            return options;
        }

        void ResetState() noexcept
        {
            _phase = Phase::Idle;
            _smoothQueue.clear();
            _smoothQueueActiveIndex = 0U;
            _smoothSpeedIndex = 0U;
            _smoothQueueEntryBoundarySpeedMps = 0.0f;
            _smoothQueueExitBoundarySpeedMps = 0.0f;
            _mainLogOpen = false;
            _selectorDrivePin = 0U;
            _selectorSensePin = 0U;
            _selectorMonitorArmed = false;
            _startupCalibration.Cancel();
            _driveService.Cancel();
            _mainLogRow = {};
            ResetActiveLogState();
        }

        bool BeginMainLog()
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
            if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
            if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
            if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
            if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
            if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
            if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
            if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;

            _mainLogRow = {};
            if (!_runtime.BeginUtilityDataLogSchema(_mainLogRow))
            {
                return false;
            }

            _mainLogOpen = true;
            return true;
        }

        void CloseMainLog()
        {
            if (!_mainLogOpen)
            {
                return;
            }

            (void)_runtime.CloseUtilityDataLog();
            _mainLogOpen = false;
        }

        float ReadBatteryVoltage() const noexcept
        {
            return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
        }

        float ReadBoardTemperatureC(const SensorSnapshot& snapshot) const noexcept
        {
            return 25.0f + (static_cast<float>(snapshot.imuBackLeft.temp) / 256.0f);
        }

        void ResetActiveLogState() noexcept
        {
            _activeLogState = {};
        }

        void SetActiveHoldLogState(
            const MazeMap::OpenFloorPhaseId phaseId,
            const std::uint16_t durationMs) noexcept
        {
            _activeLogState = {};
            _activeLogState.labels.sectionId = MazeMap::OpenFloorSectionId::Sec50Smooth;
            _activeLogState.labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
            _activeLogState.labels.primitiveId = MazeMap::OpenFloorPrimitiveId::StaticHold;
            _activeLogState.labels.phaseId = phaseId;
            _activeLogState.progressMode = ProgressMode::Time;
            _activeLogState.startMs = millis();
            _activeLogState.durationMs = durationMs;
        }

        void SetActiveManeuverLogState(const MazeMap::ManeuverInstance& entry) noexcept
        {
            _activeLogState = {};
            _activeLogState.labels.sectionId = MazeMap::OpenFloorSectionId::Sec50Smooth;
            _activeLogState.labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
            _activeLogState.labels.primitiveId = PrimitiveIdForCode(entry.getCode());
            _activeLogState.labels.directionId = DirectionIdForCode(entry.getCode());
            _activeLogState.labels.phaseId = MazeMap::OpenFloorPhaseId::Entry;
            _activeLogState.labels.speedBin = SpeedBinForIndex(_smoothSpeedIndex);
            _activeLogState.labels.repeatIndex = 1U;
            _activeLogState.progressMode = ProgressMode::Distance;
            _activeLogState.startDistanceM = _drive.GetAverageDistanceMeters();
            _activeLogState.totalDistanceM = entry.GetTravelDistanceMeters();
        }

        void PopulateCycleFromState(
            const LoopController::ModeState& state,
            OpenFloorMeasurementCycle& cycle)
        {
            cycle.masterTimeUs = state.tickStartUs;
            cycle.controlTickSequence = state.sequence;
            cycle.dtUs = state.dtUs;
            cycle.controlTiming = _loopController.LastDiagnostics().controlTiming;
            cycle.driveTelemetry = state.driveTelemetry;
            cycle.sensorSnapshot = state.sensors;
            cycle.measuredLinearSpeedMps = state.measured.linearSpeedMps;
            cycle.measuredAngularSpeedRadps = state.measured.angularSpeedRadps;
            cycle.planarAccelMps2 = cycle.sensorSnapshot.planarAccelMps2;
            cycle.batteryVoltage = ReadBatteryVoltage();
            cycle.boardTemperatureC = ReadBoardTemperatureC(cycle.sensorSnapshot);
            cycle.fanDutyCycle = GetMissionFanDutyCycle();
            cycle.selectorJumperRemoved = SelectorRemoved();
            cycle.estimatorFault = !state.estimatorHealthy;
        }

        bool LogActiveMainSample(const OpenFloorMeasurementCycle& cycle)
        {
            if (!_mainLogOpen)
            {
                return true;
            }

            OpenFloorMeasurementLabels labels = _activeLogState.labels;
            switch (_activeLogState.progressMode)
            {
            case ProgressMode::Time:
                labels.progressNorm = (_activeLogState.durationMs > 0UL) ?
                    (std::clamp)(
                        static_cast<float>(millis() - _activeLogState.startMs) /
                            static_cast<float>(_activeLogState.durationMs),
                        0.0f,
                        1.0f) :
                    1.0f;
                break;

            case ProgressMode::Distance:
            {
                const float traveledM =
                    std::fabs(_drive.GetAverageDistanceMeters() - _activeLogState.startDistanceM);
                labels.progressNorm = (_activeLogState.totalDistanceM > 0.0f) ?
                    (std::clamp)(traveledM / _activeLogState.totalDistanceM, 0.0f, 1.0f) :
                    1.0f;
                labels.phaseId = SmoothPhaseForProgress(labels.progressNorm);
                break;
            }

            case ProgressMode::None:
            default:
                break;
            }

            _activeLogState.labels.progressNorm = labels.progressNorm;
            _activeLogState.labels.phaseId = labels.phaseId;

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
            const MazeMap::VehicleState::StateVector& estimatorState = _drive.GetEstimatorStateVector();

            _mainLogRow = {};
            _mainLogRow.master_time_us = cycle.masterTimeUs;
            _mainLogRow.control_tick_sequence = cycle.controlTickSequence;
            _mainLogRow.dt_us = cycle.dtUs;
            _mainLogRow.section_id = static_cast<std::uint8_t>(labels.sectionId);
            _mainLogRow.primitive_id = static_cast<std::uint8_t>(labels.primitiveId);
            _mainLogRow.primitive_family =
                static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveFamilyForId(labels.primitiveId));
            _mainLogRow.direction_id = static_cast<std::uint8_t>(labels.directionId);
            _mainLogRow.phase_id = static_cast<std::uint8_t>(labels.phaseId);
            _mainLogRow.speed_bin = static_cast<std::uint8_t>(labels.speedBin);
            _mainLogRow.start_marker_id = static_cast<std::uint8_t>(labels.startMarkerId);
            _mainLogRow.repeat_index = labels.repeatIndex;
            _mainLogRow.progress_norm = labels.progressNorm;
            _mainLogRow.mode_flags = cycle.driveTelemetry.modeFlags;
            _mainLogRow.clipping_flags = cycle.clippingFlags;
            _mainLogRow.saturation_flags = cycle.driveTelemetry.saturationFlags;
            _mainLogRow.watchdog_flags = cycle.watchdogFlags;
            _mainLogRow.measurement_flags = Runtime::BuildOpenFloorMeasurementFlags(
                labels,
                cycle,
                encoderValid,
                imuValid,
                frontLeftObs,
                frontRightObs,
                leftObs,
                rightObs);
            _mainLogRow.ukf_mode_id = cycle.driveTelemetry.ukfModeId;
            _mainLogRow.ukf_yaw_valid_for_feedforward = cycle.driveTelemetry.ukfYawValidForFeedforward;
            _mainLogRow.bias_update_enabled = cycle.driveTelemetry.ukfBiasUpdateEnabled;
            _mainLogRow.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
            _mainLogRow.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
            _mainLogRow.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
            _mainLogRow.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
            _mainLogRow.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
            _mainLogRow.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
            _mainLogRow.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
            _mainLogRow.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
            _mainLogRow.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
            _mainLogRow.gyro_bias_anchor_radps = cycle.driveTelemetry.ukfGyroBiasAnchorRadps;
            _mainLogRow.yaw_consistency_lp_radps = cycle.driveTelemetry.ukfYawConsistencyLowPassRadps;
            _mainLogRow.yaw_window_mismatch_rad = cycle.driveTelemetry.ukfYawWindowMismatchRad;
            _mainLogRow.nhc_sigma_mps = cycle.driveTelemetry.ukfNhcSigmaMps;
            _mainLogRow.nhc_residual_mps = cycle.driveTelemetry.ukfNhcResidualMps;
            _mainLogRow.nhc_residual_sigma = cycle.driveTelemetry.ukfNhcResidualSigma;
            _mainLogRow.measured_linear_speed_mps = cycle.measuredLinearSpeedMps;
            _mainLogRow.measured_angular_speed_radps = cycle.measuredAngularSpeedRadps;
            _mainLogRow.cmd_linear_mps = _drive.GetLastLinearCommandMps();
            _mainLogRow.cmd_angular_radps = _drive.GetLastAngularCommandRadps();
            _mainLogRow.left_drive_command = cycle.driveTelemetry.leftDriveCommand;
            _mainLogRow.right_drive_command = cycle.driveTelemetry.rightDriveCommand;
            _mainLogRow.left_feedforward_command = cycle.driveTelemetry.leftFeedforwardCommand;
            _mainLogRow.right_feedforward_command = cycle.driveTelemetry.rightFeedforwardCommand;
            _mainLogRow.left_feedback_command = cycle.driveTelemetry.leftFeedbackCommand;
            _mainLogRow.right_feedback_command = cycle.driveTelemetry.rightFeedbackCommand;
            _mainLogRow.left_target_velocity_mps = cycle.driveTelemetry.leftTargetVelocityMps;
            _mainLogRow.right_target_velocity_mps = cycle.driveTelemetry.rightTargetVelocityMps;
            _mainLogRow.left_launch_assist_floor = cycle.driveTelemetry.leftLaunchAssistFloor;
            _mainLogRow.right_launch_assist_floor = cycle.driveTelemetry.rightLaunchAssistFloor;
            _mainLogRow.encoder_timestamp_us = cycle.controlTiming.encoderReadDoneUs;
            _mainLogRow.left_encoder_count = cycle.driveTelemetry.leftEncoderCount;
            _mainLogRow.right_encoder_count = cycle.driveTelemetry.rightEncoderCount;
            _mainLogRow.left_encoder_omega_radps = cycle.driveTelemetry.leftEncoderOmegaRadps;
            _mainLogRow.right_encoder_omega_radps = cycle.driveTelemetry.rightEncoderOmegaRadps;
            _mainLogRow.left_encoder_distance_m = cycle.driveTelemetry.leftDistanceM;
            _mainLogRow.right_encoder_distance_m = cycle.driveTelemetry.rightDistanceM;
            _mainLogRow.left_encoder_velocity_mps = cycle.driveTelemetry.leftVelocityMps;
            _mainLogRow.right_encoder_velocity_mps = cycle.driveTelemetry.rightVelocityMps;
            _mainLogRow.imu_timestamp_us = cycle.sensorSnapshot.imuTiming.readDoneUs;
            _mainLogRow.imu_status = cycle.sensorSnapshot.imuBackLeft.status;
            _mainLogRow.imu_interrupt_high = cycle.sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
            _mainLogRow.accel_bias_valid = cycle.sensorSnapshot.accelBiasValid ? 1U : 0U;
            _mainLogRow.imu_gyro_x = cycle.sensorSnapshot.imuBackLeft.gyroX;
            _mainLogRow.imu_gyro_y = cycle.sensorSnapshot.imuBackLeft.gyroY;
            _mainLogRow.imu_gyro_z = cycle.sensorSnapshot.imuBackLeft.gyroZ;
            _mainLogRow.imu_accel_x = cycle.sensorSnapshot.imuBackLeft.accelX;
            _mainLogRow.imu_accel_y = cycle.sensorSnapshot.imuBackLeft.accelY;
            _mainLogRow.imu_accel_z = cycle.sensorSnapshot.imuBackLeft.accelZ;
            _mainLogRow.imu_temp = cycle.sensorSnapshot.imuBackLeft.temp;
            _mainLogRow.gyro_raw_radps = cycle.sensorSnapshot.gyroRawRadps;
            _mainLogRow.gyro_bias_radps = cycle.sensorSnapshot.gyroBiasRadps;
            _mainLogRow.gyro_radps = cycle.sensorSnapshot.gyroRadps;
            _mainLogRow.accel_body_x_mps2 = cycle.sensorSnapshot.accelBodyXMps2;
            _mainLogRow.accel_body_y_mps2 = cycle.sensorSnapshot.accelBodyYMps2;
            _mainLogRow.planar_accel_mps2 = cycle.planarAccelMps2;
            _mainLogRow.front_timestamp_us = cycle.sensorSnapshot.frontTiming.observationReadyUs;
            _mainLogRow.left_timestamp_us = cycle.sensorSnapshot.leftTiming.observationReadyUs;
            _mainLogRow.right_timestamp_us = cycle.sensorSnapshot.rightTiming.observationReadyUs;
            _mainLogRow.front_left_obs_class = static_cast<std::uint8_t>(frontLeftObs.cls);
            _mainLogRow.front_right_obs_class = static_cast<std::uint8_t>(frontRightObs.cls);
            _mainLogRow.left_obs_class = static_cast<std::uint8_t>(leftObs.cls);
            _mainLogRow.right_obs_class = static_cast<std::uint8_t>(rightObs.cls);
            _mainLogRow.front_left_obs_rho_m = frontLeftObs.rho;
            _mainLogRow.front_right_obs_rho_m = frontRightObs.rho;
            _mainLogRow.left_obs_rho_m = leftObs.rho;
            _mainLogRow.right_obs_rho_m = rightObs.rho;
            _mainLogRow.front_left_obs_confidence = frontLeftObs.confidence;
            _mainLogRow.front_right_obs_confidence = frontRightObs.confidence;
            _mainLogRow.left_obs_confidence = leftObs.confidence;
            _mainLogRow.right_obs_confidence = rightObs.confidence;
            _mainLogRow.fan_duty_cycle = cycle.fanDutyCycle;
            return _runtime.LogUtilityDataRow(_mainLogRow);
        }

        void ConfigureSelectorMonitor() noexcept
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

        void ReleaseSelectorMonitor() noexcept
        {
            if (_selectorMonitorArmed)
            {
                EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
            }
            _selectorMonitorArmed = false;
            _selectorDrivePin = 0U;
            _selectorSensePin = 0U;
        }

        bool SelectorRemoved() const noexcept
        {
            return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
        }

        bool StartHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildPrimaryDiagnosticLimits(_vehicle, kPrimaryDiagnosticMaxSmoothSpeedMps));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
        }

        float CurrentSmoothSpeedMps() const noexcept
        {
            return PrimaryDiagnosticSmoothSpeedMps(_smoothSpeedIndex);
        }

        bool LoadSmoothCycleForCurrentSpeed()
        {
            _smoothQueue.clear();
            _smoothQueueActiveIndex = 0U;
            return BuildPrimaryDiagnosticSmoothQueue(
                _vehicle,
                _smoothSpeedIndex,
                CurrentSmoothSpeedMps(),
                _smoothQueueEntryBoundarySpeedMps,
                _smoothQueue,
                _smoothQueueExitBoundarySpeedMps);
        }

        bool StartSmoothCycleEntry(const MazeMap::ManeuverInstance& entry) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildPrimaryDiagnosticLimits(_vehicle, CurrentSmoothSpeedMps()));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartManeuver(entry);
            return _driveService.Active();
        }

        LoopController::ControlVector PollDrive(const Phase nextPhase)
        {
            bool done = false;
            const LoopController::ControlVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = nextPhase;
            return LoopController::ControlVector::Brake;
        }

        LoopController::ControlVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            (void)loopEndTimeUs;
            OpenFloorMeasurementCycle cycle{};
            PopulateCycleFromState(state, cycle);

            auto returnLogged =
                [this, &cycle, &services](const LoopController::ControlVector& control) -> LoopController::ControlVector
                {
                    if (!LogActiveMainSample(cycle))
                    {
                        services.Fault("Primary diagnostic main log write failed");
                        return LoopController::ControlVector::Brake;
                    }

                    return control;
                };

            if (cycle.selectorJumperRemoved)
            {
                _activeLogState.labels.abortMarker = true;
                if (!LogActiveMainSample(cycle))
                {
                    services.Fault("Primary diagnostic main log write failed");
                    return LoopController::ControlVector::Brake;
                }
                services.Fault(kPrimaryDiagnosticSelectorRemovedReason);
                return LoopController::ControlVector::Brake;
            }

            switch (_phase)
            {
            case Phase::LaunchStartupHold:
                if (!StartHold(DiagnosticConfig::kStartupSettleMs))
                {
                    services.Fault("Primary diagnostic startup hold could not start");
                }
                else
                {
                    SetActiveHoldLogState(MazeMap::OpenFloorPhaseId::Startup, DiagnosticConfig::kStartupSettleMs);
                    _phase = Phase::RunStartupHold;
                }
                return returnLogged(LoopController::ControlVector::Brake);

            case Phase::RunStartupHold:
                return returnLogged(PollDrive(Phase::LaunchSmoothCycleSpeed));

            case Phase::LaunchSmoothCycleSpeed:
                if (_smoothSpeedIndex >= MazeMap::kOpenFloorSmoothSpeedBinsMps.size())
                {
                    _phase = Phase::LaunchCompletionHold;
                }
                else if (!LoadSmoothCycleForCurrentSpeed())
                {
                    services.Fault("Primary diagnostic smooth-cycle queue could not be built");
                }
                else
                {
                    (void)_runtime.AppendTextLogFormatted(
                        "Primary diagnostic smooth bin %u start: v=%.3f mps; queue_entries=%u; start_half=(2,%u) Up",
                        static_cast<unsigned>(_smoothSpeedIndex) + 1U,
                        static_cast<double>(CurrentSmoothSpeedMps()),
                        static_cast<unsigned>(_smoothQueue.size()),
                        static_cast<unsigned>(3U + _smoothSpeedIndex));
                    _phase = Phase::LaunchSmoothCycleEntry;
                }
                return returnLogged(LoopController::ControlVector::Brake);

            case Phase::LaunchSmoothCycleEntry:
                if (_smoothQueueActiveIndex >= _smoothQueue.size())
                {
                    (void)_runtime.AppendTextLogFormatted(
                        "Primary diagnostic smooth bin %u complete: v=%.3f mps",
                        static_cast<unsigned>(_smoothSpeedIndex) + 1U,
                        static_cast<double>(CurrentSmoothSpeedMps()));
                    _smoothQueueEntryBoundarySpeedMps = _smoothQueueExitBoundarySpeedMps;
                    ++_smoothSpeedIndex;
                    _phase = Phase::LaunchSmoothCycleSpeed;
                }
                else if (!StartSmoothCycleEntry(_smoothQueue[_smoothQueueActiveIndex]))
                {
                    services.Fault("Primary diagnostic smooth-cycle maneuver could not start");
                }
                else
                {
                    SetActiveManeuverLogState(_smoothQueue[_smoothQueueActiveIndex]);
                    _phase = Phase::RunSmoothCycleEntry;
                }
                return returnLogged(LoopController::ControlVector::Brake);

            case Phase::RunSmoothCycleEntry:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return returnLogged(control);
                }

                ++_smoothQueueActiveIndex;
                _phase = Phase::LaunchSmoothCycleEntry;
                return returnLogged(LoopController::ControlVector::Brake);
            }

            case Phase::LaunchCompletionHold:
                if (!StartHold(kPrimaryDiagnosticCompletionHoldMs))
                {
                    services.Fault("Primary diagnostic completion hold could not start");
                }
                else
                {
                    SetActiveHoldLogState(MazeMap::OpenFloorPhaseId::Stop, kPrimaryDiagnosticCompletionHoldMs);
                    _phase = Phase::RunCompletionHold;
                }
                return returnLogged(LoopController::ControlVector::Brake);

            case Phase::RunCompletionHold:
                return returnLogged(PollDrive(Phase::Complete));

            case Phase::Complete:
                services.RequestEndLoop();
                return returnLogged(LoopController::ControlVector::Brake);

            case Phase::Idle:
            default:
                services.Fault("Primary diagnostic phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        RuntimeSensorSuite& _sensors;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        MazeMap::ManeuverQueue _smoothQueue{};
        std::uint16_t _smoothQueueActiveIndex{};
        std::uint8_t _smoothSpeedIndex{};
        float _smoothQueueEntryBoundarySpeedMps{};
        float _smoothQueueExitBoundarySpeedMps{};
        bool _mainLogOpen{};
        OpenFloorMainRow _mainLogRow{};
        ActiveLogState _activeLogState{};
        Phase _phase{ Phase::Idle };
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
    };

    IApplicationMode& GetDiagnosticMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PrimaryDiagnostic,
            BootModeCategory::Utility,
            "primary_diagnostic",
            "Run the hard-coded open-floor smooth-cycle diagnostic sweep for primary diagnostic mode.",
            "logging.txt, open_floor_main.mmlog",
            &GetDiagnosticMode,
            "GetDiagnosticMode",
            "OpenFloorMeasurementController.cpp",
            "shared bring-up; startup hold; smooth-cycle bin 1 with S1 lead-in; smooth-cycle bin 2 with S1 lead-in; smooth-cycle bin 3 with S1 lead-in and S1 stop straight; completion hold",
            "DiagnosticConfig linear limits; OpenFloorMeasurementSpec smooth-speed bins; shared startup-calibration bring-up; shared drive service",
            "Behavior is intentionally reduced to the hard-coded 4x4 non-in-place smooth-cycle replay with S1 speed-change straights",
            "open_floor_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
