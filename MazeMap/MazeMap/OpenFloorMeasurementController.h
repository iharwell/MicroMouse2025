#pragma once

#include "Defines.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "IApplicationMode.h"
#include "LoopController.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementSpec.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

class DriveBase;
class RuntimeSensorSuite;

namespace MazeMap
{
    class ManeuverQueue;
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;
    class StartupCalibration;
}

namespace MazeMap::App::Internal::Runtime
{
#define OPEN_FLOOR_TIMING_FIELDS(X)              \
    X(std::uint32_t, mono_time_us)              \
    X(std::uint32_t, control_tick_sequence)     \
    X(std::uint32_t, dt_us)                     \
    X(std::uint32_t, phase_id)                  \
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
    X(std::uint8_t,  phase_id)                     \
    X(std::uint8_t,  primitive_id)                 \
    X(std::uint8_t,  speed_bin)                    \
    X(std::uint16_t, repeat_index)                 \
    X(std::uint16_t, mode_flags)                   \
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
    X(std::uint32_t, right_timestamp_us)

    MMLOG_DEFINE_ROW(OpenFloorMainRow, OPEN_FLOOR_MAIN_FIELDS);

#undef OPEN_FLOOR_MAIN_FIELDS
#undef OPEN_FLOOR_TIMING_FIELDS
}

namespace MazeMap::App::Internal
{
    class EXPORT OpenFloorMeasurementController final : public IApplicationMode
    {
    public:
        explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime);
        ~OpenFloorMeasurementController() override;

        OpenFloorMeasurementController(const OpenFloorMeasurementController&) = delete;
        OpenFloorMeasurementController& operator=(const OpenFloorMeasurementController&) = delete;
        OpenFloorMeasurementController(OpenFloorMeasurementController&&) = delete;
        OpenFloorMeasurementController& operator=(OpenFloorMeasurementController&&) = delete;

        bool Begin() override;
        void Run() override;

    private:
        static constexpr std::size_t kMainScheduledWorkCapacity = 320U;
        static constexpr std::size_t kRepeatedSequenceScratchCapacity = 96U;
        static constexpr std::size_t kLaunchPhaseCaseCapacity =
            kOpenFloorLaunchDriveMagnitudeCount * kOpenFloorLaunchRepeatsPerMagnitude * 2U;
        static constexpr std::size_t kStraightPhaseCaseCapacity =
            kOpenFloorStraightSpeedBinsMps.size() * kOpenFloorStraightRepeatsPerSpeed * 2U;
        static constexpr std::size_t kYawPhaseCaseCapacity =
            kOpenFloorYawOmegaBinsRadps.size() *
            DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed *
            kOpenFloorYawPrimitiveIds.size();
        static constexpr std::size_t kSmoothPhaseCaseCapacity =
            (kOpenFloorSmoothSpeedBinsMps.size() * 27U) + 1U;
        static constexpr std::size_t kLoopPhaseCaseCapacity = 8U;

        using StageTick = LoopController::ControlVector (OpenFloorMeasurementController::*)(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        enum class PauseAction : std::uint8_t
        {
            None,
            TimingToMain,
        };

        enum class MeasurementPhaseId : std::uint8_t
        {
            Timing = 0U,
            Static = 1U,
            Launch = 2U,
            Straight = 3U,
            Yaw = 4U,
            Smooth = 5U,
            LoopClockwise = 6U,
            LoopCounterClockwise = 7U,
        };

        class MainRowLabel final
        {
        public:
            constexpr MainRowLabel() = default;
            constexpr MainRowLabel(
                MeasurementPhaseId phaseId,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin,
                std::uint16_t repeatIndex) noexcept
                : _phaseId(phaseId)
                , _primitiveId(primitiveId)
                , _speedBin(speedBin)
                , _repeatIndex(repeatIndex)
            {
            }

            constexpr MeasurementPhaseId PhaseId() const noexcept
            {
                return _phaseId;
            }

            constexpr OpenFloorPrimitiveId PrimitiveId() const noexcept
            {
                return _primitiveId;
            }

            constexpr OpenFloorSpeedBin SpeedBin() const noexcept
            {
                return _speedBin;
            }

            constexpr std::uint16_t RepeatIndex() const noexcept
            {
                return _repeatIndex;
            }

            constexpr MainRowLabel WithRepeatIndex(const std::uint16_t repeatIndex) const noexcept
            {
                return MainRowLabel(_phaseId, _primitiveId, _speedBin, repeatIndex);
            }

        private:
            MeasurementPhaseId _phaseId{ MeasurementPhaseId::Timing };
            OpenFloorPrimitiveId _primitiveId{ OpenFloorPrimitiveId::None };
            OpenFloorSpeedBin _speedBin{ OpenFloorSpeedBin::None };
            std::uint16_t _repeatIndex{};
        };

        class MainStage;

        class MainMeasurementPhase
        {
        public:
            virtual ~MainMeasurementPhase() = default;

            virtual MeasurementPhaseId PhaseId() const noexcept = 0;
            virtual void Reset() noexcept = 0;
            virtual bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) = 0;
            virtual void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) = 0;
            virtual LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) = 0;
        };

        class StaticMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;
        };

        class LaunchMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;

        private:
            std::array<float, kLaunchPhaseCaseCapacity> _leftCommands{};
            std::array<float, kLaunchPhaseCaseCapacity> _rightCommands{};
            std::uint16_t _caseCount{};
            float _activeLeftCommand{};
            float _activeRightCommand{};
            std::uint32_t _deadlineMs{};
        };

        class StraightMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;

        private:
            std::array<float, kStraightPhaseCaseCapacity> _speedsMps{};
            std::uint16_t _caseCount{};
        };

        class YawMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;

        private:
            std::array<float, kYawPhaseCaseCapacity> _yawRad{};
            std::array<float, kYawPhaseCaseCapacity> _maxOmegaRadps{};
            std::uint16_t _caseCount{};
        };

        class SmoothMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;

        private:
            bool BuildQueue(
                MazeMap::Vehicle& vehicle,
                std::uint8_t speedIndex,
                float cruiseSpeedMps,
                float initialEntrySpeedMps,
                MazeMap::ManeuverQueue& queue,
                float& exitBoundarySpeedMps) const;

            std::array<MazeMap::ManeuverInstance, kSmoothPhaseCaseCapacity> _maneuvers{};
            std::array<float, kSmoothPhaseCaseCapacity> _speedLimitsMps{};
            std::uint16_t _caseCount{};
        };

        class LoopMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            LoopMeasurementPhase(MeasurementPhaseId phaseId, bool clockwise) noexcept;

            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Compile(OpenFloorMeasurementController& controller, MainStage& stage) override;
            void BeginCase(OpenFloorMeasurementController& controller, std::uint16_t caseIndex) override;
            LoopController::ControlVector TickCase(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) override;

        private:
            bool BuildQueue(MazeMap::Vehicle& vehicle, MazeMap::ManeuverQueue& queue) const;

            MeasurementPhaseId _phaseId;
            bool _clockwise{};
            std::array<MazeMap::ManeuverInstance, kLoopPhaseCaseCapacity> _maneuvers{};
            std::uint16_t _caseCount{};
        };

        class TimingStage final
        {
        public:
            void Reset() noexcept;
            bool Begin(OpenFloorMeasurementController& controller);
            LoopController::ControlVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services);
            LoopController::PauseDisposition CompleteTimingToMainHandoff(
                OpenFloorMeasurementController& controller,
                MainStage& mainStage,
                const LoopController::PauseContext& pause);
            void FinalizeCompletedRun(OpenFloorMeasurementController& controller) noexcept;

        private:
            bool FlushPending(
                OpenFloorMeasurementController& controller,
                const char* failureReason,
                LoopController::TickServices* services);
            void StageRow(const Runtime::OpenFloorTimingRow& row);
            bool CaptureComplete() const noexcept;

            std::uint16_t _tickIndex{};
            bool _logOpen{};
            std::optional<Runtime::OpenFloorTimingRow> _pendingRow{};
        };

        class MainStage final
        {
        public:
            void Reset() noexcept;
            bool CompilePlan(OpenFloorMeasurementController& controller);
            bool Begin(OpenFloorMeasurementController& controller);
            LoopController::ControlVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services);
            void FinalizeCompletedRun(OpenFloorMeasurementController& controller) noexcept;

            bool RegisterUnrepeatedCase(
                MainMeasurementPhase& phase,
                std::uint16_t caseIndex,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin);
            bool RegisterSequentialCase(
                MainMeasurementPhase& phase,
                std::uint16_t caseIndex,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin);
            bool BeginRepeatedSequence(std::uint16_t repeatCount) noexcept;
            bool RegisterRepeatedSequenceCase(
                MainMeasurementPhase& phase,
                std::uint16_t caseIndex,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin);
            bool EndRepeatedSequence();

        private:
            class ScheduledWork final
            {
            public:
                static ScheduledWork PhaseCase(
                    MainMeasurementPhase& phase,
                    std::uint16_t caseIndex,
                    const MainRowLabel& labels) noexcept;
                static ScheduledWork Hold(
                    MainMeasurementPhase& phase,
                    std::uint16_t caseIndex,
                    const MainRowLabel& labels,
                    std::uint16_t durationMs) noexcept;

                bool IsHold() const noexcept;
                MainMeasurementPhase& Phase() const noexcept;
                std::uint16_t CaseIndex() const noexcept;
                const MainRowLabel& Labels() const noexcept;
                std::uint16_t HoldDurationMs() const noexcept;

            private:
                enum class Kind : std::uint8_t
                {
                    PhaseCase,
                    Hold,
                };

                Kind _kind{ Kind::PhaseCase };
                MainMeasurementPhase* _phase{};
                MainRowLabel _labels{};
                std::uint16_t _caseIndex{};
                std::uint16_t _holdDurationMs{};
            };

            bool CompilePhase(
                OpenFloorMeasurementController& controller,
                MainMeasurementPhase& phase,
                std::uint16_t interCaseHoldMs,
                std::uint16_t interPhaseHoldMs);
            void ClearCompileState() noexcept;
            bool AppendScheduledCase(
                MainMeasurementPhase& phase,
                std::uint16_t caseIndex,
                const MainRowLabel& labels);
            bool AppendHold(
                MainMeasurementPhase& phase,
                std::uint16_t caseIndex,
                const MainRowLabel& labels,
                std::uint16_t durationMs);
            bool FlushPending(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices* services,
                const char* failureReason);
            void StageRow(const Runtime::OpenFloorMainRow& row);
            bool CheckFault(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices& services);
            void Advance() noexcept;
            const ScheduledWork* ActiveWork() const noexcept;

            std::array<ScheduledWork, kMainScheduledWorkCapacity> _scheduledWork{};
            std::uint16_t _scheduledWorkCount{};
            std::uint16_t _nextWorkIndex{};
            bool _logOpen{};
            bool _completionPending{};
            bool _activeWorkStarted{};
            MainMeasurementPhase* _activePhase{};
            std::uint16_t _activeCaseIndex{};
            MainRowLabel _activeLabels{};
            std::optional<Runtime::OpenFloorMainRow> _pendingRow{};

            MainMeasurementPhase* _compilingPhase{};
            std::uint16_t _compilingInterCaseHoldMs{};
            std::uint16_t _compilingInterPhaseHoldMs{};
            std::uint16_t _compilingSequentialRepeatIndex{};
            bool _compilingPhaseHasCases{};
            MainMeasurementPhase* _lastCompiledPhase{};
            std::uint16_t _lastCompiledCaseIndex{};
            MainRowLabel _lastCompiledLabels{};
            bool _sequenceActive{};
            std::uint16_t _sequenceRepeatCount{};
            std::uint16_t _sequenceSize{};
            std::array<ScheduledWork, kRepeatedSequenceScratchCapacity> _sequenceScratch{};

            StaticMeasurementPhase _staticPhase{};
            LaunchMeasurementPhase _launchPhase{};
            StraightMeasurementPhase _straightPhase{};
            YawMeasurementPhase _yawPhase{};
            SmoothMeasurementPhase _smoothPhase{};
            LoopMeasurementPhase _clockwiseLoopPhase{ MeasurementPhaseId::LoopClockwise, true };
            LoopMeasurementPhase _counterClockwiseLoopPhase{
                MeasurementPhaseId::LoopCounterClockwise,
                false,
            };
        };

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        static LoopController::PauseDisposition PauseThunk(
            void* context,
            const LoopController::PauseContext& pause);
        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        void ResetState() noexcept;
        void PopulateTimingRowFromState(
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorTimingRow& row) const noexcept;
        void PopulateMainRowFromState(
            const MainRowLabel& labels,
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorMainRow& row) const;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        LoopController::PauseDisposition OnPauseGranted(const LoopController::PauseContext& pause);
        LoopController::ControlVector TimingStageTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector MainStageTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        static OpenFloorSpeedBin SpeedBinForIndex(std::size_t speedIndex) noexcept;

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        RuntimeSensorSuite& _sensors;
        ::DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        StageTick _activeStageTick{ &OpenFloorMeasurementController::TimingStageTick };
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        PauseAction _pauseAction{ PauseAction::None };
        TimingStage _timingStage{};
        MainStage _mainStage{};
    };

    IApplicationMode& GetOpenFloorMeasurementMode();
    const MazeMap::App::BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
}
