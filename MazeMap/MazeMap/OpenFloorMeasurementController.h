#pragma once

#include "Defines.h"
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
#include <variant>

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

class DriveBase;
class RuntimeSensorSuite;

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
        static constexpr std::size_t kMainSegmentCapacity = 320U;
        static constexpr std::size_t kCompiledManeuverCapacity = 170U;

        using StageTick = LoopController::ControlVector (OpenFloorMeasurementController::*)(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        enum class PauseAction : std::uint8_t
        {
            None,
            TimingToMain,
        };

        enum class BatteryPhaseId : std::uint8_t
        {
            Timing = static_cast<std::uint8_t>(OpenFloorSectionId::Sec00Timing),
            Static = static_cast<std::uint8_t>(OpenFloorSectionId::Sec10Static),
            Launch = static_cast<std::uint8_t>(OpenFloorSectionId::Sec20Launch),
            Straight = static_cast<std::uint8_t>(OpenFloorSectionId::Sec30Straight),
            Yaw = static_cast<std::uint8_t>(OpenFloorSectionId::Sec40Yaw),
            Smooth = static_cast<std::uint8_t>(OpenFloorSectionId::Sec50Smooth),
            LoopClockwise = static_cast<std::uint8_t>(OpenFloorSectionId::Sec60LoopCw),
            LoopCounterClockwise = static_cast<std::uint8_t>(OpenFloorSectionId::Sec70LoopCcw),
        };

        struct LoggedRowIdentity final
        {
            constexpr LoggedRowIdentity() = default;
            constexpr LoggedRowIdentity(
                BatteryPhaseId phaseId,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin,
                std::uint16_t repeatIndex) noexcept
                : phaseId(phaseId)
                , primitiveId(primitiveId)
                , speedBin(speedBin)
                , repeatIndex(repeatIndex)
            {
            }

            BatteryPhaseId phaseId = BatteryPhaseId::Timing;
            OpenFloorPrimitiveId primitiveId = OpenFloorPrimitiveId::None;
            OpenFloorSpeedBin speedBin = OpenFloorSpeedBin::None;
            std::uint16_t repeatIndex = 0U;
        };

        class CompiledSegment;
        class MainStage;

        class ActiveSegmentExecution final
        {
        public:
            void Reset() noexcept;
            void Rebind(const CompiledSegment& segment) noexcept;

        private:
            friend class CompiledSegment;

            struct HoldExecution final
            {
                bool started{};
            };

            struct WheelCommandProfileExecution final
            {
                bool started{};
                bool settling{};
                std::uint32_t deadlineMs{};
                HoldExecution settlingHold{};
            };

            struct StraightExecution final
            {
                bool started{};
                bool settling{};
                float startDistanceM{};
                float totalDistanceM{};
                HoldExecution settlingHold{};
            };

            struct TurnExecution final
            {
                bool started{};
                bool settling{};
                float targetYawRad{};
                float targetMagnitudeRad{};
                HoldExecution settlingHold{};
            };

            struct ManeuverExecution final
            {
                bool started{};
                bool settling{};
                float startDistanceM{};
                float totalDistanceM{};
                float targetYawRad{};
                float targetMagnitudeRad{};
                HoldExecution settlingHold{};
            };

            using ExecutionState = std::variant<
                std::monostate,
                HoldExecution,
                WheelCommandProfileExecution,
                StraightExecution,
                TurnExecution,
                ManeuverExecution>;

            ExecutionState _state{};
        };

        class CompiledSegment final
        {
        public:
            CompiledSegment() = default;

            static CompiledSegment Hold(
                LoggedRowIdentity identity,
                std::uint16_t durationMs) noexcept;
            static CompiledSegment WheelCommandProfile(
                LoggedRowIdentity identity,
                std::uint16_t durationMs,
                float leftCommand,
                float rightCommand,
                std::uint16_t settlingHoldMs) noexcept;
            static CompiledSegment Straight(
                LoggedRowIdentity identity,
                float distanceM,
                float speedMps,
                std::uint16_t settlingHoldMs) noexcept;
            static CompiledSegment Turn(
                LoggedRowIdentity identity,
                float yawRad,
                float maxOmegaRadps,
                std::uint16_t settlingHoldMs) noexcept;
            static CompiledSegment Maneuver(
                LoggedRowIdentity identity,
                std::uint16_t maneuverIndex,
                float speedMps,
                std::uint16_t settlingHoldMs) noexcept;

            const LoggedRowIdentity& RowIdentity() const noexcept;
            BatteryPhaseId PhaseId() const noexcept;
            const char* FaultReasonText() const noexcept;
            LoopController::ControlVector TickExecution(
                OpenFloorMeasurementController& controller,
                ActiveSegmentExecution& execution,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                bool& done) const;

        private:
            friend class ActiveSegmentExecution;

            struct HoldPlan final
            {
                std::uint16_t durationMs{};
            };

            struct WheelCommandProfilePlan final
            {
                std::uint16_t durationMs{};
                float leftCommand{};
                float rightCommand{};
            };

            struct StraightPlan final
            {
                float distanceM{};
                float speedMps{};
            };

            struct TurnPlan final
            {
                float yawRad{};
                float maxOmegaRadps{};
            };

            struct ManeuverPlan final
            {
                std::uint16_t maneuverIndex{};
                float speedMps{};
            };

            using Plan = std::variant<
                HoldPlan,
                WheelCommandProfilePlan,
                StraightPlan,
                TurnPlan,
                ManeuverPlan>;

            CompiledSegment(
                LoggedRowIdentity identity,
                std::uint16_t settlingHoldMs,
                const Plan& plan) noexcept;
            LoopController::ControlVector TickHoldExecution(
                OpenFloorMeasurementController& controller,
                std::uint16_t durationMs,
                ActiveSegmentExecution::HoldExecution& execution,
                bool& done) const;
            LoopController::ControlVector TickWheelCommandProfileExecution(
                OpenFloorMeasurementController& controller,
                const WheelCommandProfilePlan& plan,
                ActiveSegmentExecution::WheelCommandProfileExecution& execution,
                bool& done) const;
            LoopController::ControlVector TickStraightExecution(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices& services,
                const StraightPlan& plan,
                ActiveSegmentExecution::StraightExecution& execution,
                bool& done) const;
            LoopController::ControlVector TickTurnExecution(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices& services,
                const TurnPlan& plan,
                ActiveSegmentExecution::TurnExecution& execution,
                bool& done) const;
            LoopController::ControlVector TickManeuverExecution(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices& services,
                const ManeuverPlan& plan,
                ActiveSegmentExecution::ManeuverExecution& execution,
                bool& done) const;

            LoggedRowIdentity _rowIdentity{};
            std::uint16_t _settlingHoldMs{};
            Plan _plan{ HoldPlan{} };
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
            bool AppendSegment(const CompiledSegment& segment);
            bool StoreCompiledManeuver(
                const MazeMap::ManeuverInstance& maneuver,
                std::uint16_t& maneuverIndex);
            const MazeMap::ManeuverInstance* CompiledManeuverAt(std::uint16_t maneuverIndex) const noexcept;

        private:
            bool FlushPending(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices* services,
                const char* failureReason);
            void StageRow(const Runtime::OpenFloorMainRow& row);
            bool CheckFault(
                OpenFloorMeasurementController& controller,
                LoopController::TickServices& services);
            void Advance() noexcept;
            const CompiledSegment* ActiveSegment() const noexcept;

            std::array<CompiledSegment, kMainSegmentCapacity> _plan{};
            std::array<MazeMap::ManeuverInstance, kCompiledManeuverCapacity> _maneuvers{};
            std::uint16_t _planSize{};
            std::uint16_t _maneuverCount{};
            std::uint16_t _nextSegmentIndex{};
            bool _logOpen{};
            bool _completionPending{};
            ActiveSegmentExecution _activeExecution{};
            char _estimatorFaultReason[64]{};
            std::optional<Runtime::OpenFloorMainRow> _pendingRow{};
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
            const LoggedRowIdentity& identity,
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
