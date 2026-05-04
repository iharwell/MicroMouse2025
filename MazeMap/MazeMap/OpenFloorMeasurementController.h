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

        void SetupMode() override;
        LoopController::ControlVector RunTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override;

    private:
        static constexpr std::size_t kLaunchPhaseSlotCount =
            kOpenFloorLaunchDriveMagnitudeCount * kOpenFloorLaunchRepeatsPerMagnitude * 2U;
        static constexpr std::size_t kStraightPhaseRepeatSlotCount =
            kOpenFloorStraightRepeatsPerSpeed * 2U;
        static constexpr std::size_t kSmoothPhaseCyclePrimitiveCount = 26U;
        static constexpr std::size_t kSmoothPhaseMaxPrimitiveCount =
            kSmoothPhaseCyclePrimitiveCount + 2U;
        static constexpr std::size_t kSmoothPhaseSlotCapacity =
            kOpenFloorSmoothSpeedBinsMps.size() * kSmoothPhaseMaxPrimitiveCount;
        static constexpr std::size_t kLoopPhasePrimitiveCount = 8U;
        static constexpr std::size_t kMainPhaseCount = 7U;

        using StageTick = LoopController::ControlVector (OpenFloorMeasurementController::*)(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        enum class SessionBoundaryAction : std::uint8_t
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
                MazeMap::ManeuverCode primitiveCode,
                std::uint8_t speedBinLogId,
                std::uint16_t repeatIndex) noexcept
                : _phaseId(phaseId)
                , _primitiveCode(primitiveCode)
                , _speedBinLogId(speedBinLogId)
                , _repeatIndex(repeatIndex)
            {
            }

            constexpr MeasurementPhaseId PhaseId() const noexcept
            {
                return _phaseId;
            }

            constexpr MazeMap::ManeuverCode PrimitiveCode() const noexcept
            {
                return _primitiveCode;
            }

            constexpr std::uint8_t SpeedBinLogId() const noexcept
            {
                return _speedBinLogId;
            }

            constexpr std::uint16_t RepeatIndex() const noexcept
            {
                return _repeatIndex;
            }

        private:
            MeasurementPhaseId _phaseId{ MeasurementPhaseId::Timing };
            MazeMap::ManeuverCode _primitiveCode{ MazeMap::MC_NONE };
            std::uint8_t _speedBinLogId{ kOpenFloorSpeedBinLogIdNone };
            std::uint16_t _repeatIndex{};
        };

        class MainStage;

        class MainMeasurementPhase
        {
        public:
            class SlotIdentity final
            {
            public:
                constexpr SlotIdentity() = default;
                constexpr SlotIdentity(
                    const MazeMap::ManeuverCode primitiveCode,
                    const std::uint8_t speedBinLogId) noexcept
                    : _primitiveCode(primitiveCode)
                    , _speedBinLogId(speedBinLogId)
                {
                }

                constexpr MazeMap::ManeuverCode PrimitiveCode() const noexcept
                {
                    return _primitiveCode;
                }

                constexpr std::uint8_t SpeedBinLogId() const noexcept
                {
                    return _speedBinLogId;
                }

            private:
                MazeMap::ManeuverCode _primitiveCode{ MazeMap::MC_NONE };
                std::uint8_t _speedBinLogId{ kOpenFloorSpeedBinLogIdNone };
            };

            virtual ~MainMeasurementPhase() = default;

            virtual const char* Name() const noexcept = 0;
            virtual MeasurementPhaseId PhaseId() const noexcept = 0;
            virtual void Reset() noexcept = 0;
            virtual bool Prepare(OpenFloorMeasurementController& controller) = 0;
            virtual std::uint16_t MaxPrimitiveCount() const noexcept = 0;
            virtual std::uint8_t MaxSpeedBinCount() const noexcept = 0;
            virtual std::uint16_t MaxRepeatCount() const noexcept = 0;
            virtual bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept = 0;
            virtual SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept = 0;
            virtual void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) = 0;
            virtual LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) = 0;
        };

        class StaticMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;
        };

        class LaunchMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;

        private:
            float _activeLeftCommand{};
            float _activeRightCommand{};
            std::uint32_t _deadlineMs{};
            bool _holdActive{};
        };

        class StraightMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;

        private:
            bool _holdActive{};
        };

        class YawMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;

        private:
            bool _holdActive{};
        };

        class SmoothMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;

        private:
            static constexpr std::size_t SlotOffset(
                std::uint8_t speedBinIndex,
                std::uint16_t primitiveIndex) noexcept
            {
                return
                    (static_cast<std::size_t>(speedBinIndex) * kSmoothPhaseMaxPrimitiveCount) +
                    static_cast<std::size_t>(primitiveIndex);
            }

            bool IsLastActiveSlot(std::uint16_t primitiveIndex, std::uint8_t speedBinIndex) const noexcept;
            bool BuildQueue(
                MazeMap::Vehicle& vehicle,
                std::uint8_t speedIndex,
                float cruiseSpeedMps,
                float initialEntrySpeedMps,
                MazeMap::ManeuverQueue& queue,
                float& exitBoundarySpeedMps) const;

            std::array<MazeMap::ManeuverInstance, kSmoothPhaseSlotCapacity> _maneuvers{};
            std::array<float, kSmoothPhaseSlotCapacity> _speedLimitsMps{};
            std::array<std::uint8_t, kOpenFloorSmoothSpeedBinsMps.size()> _primitiveCounts{};
            std::uint16_t _activePostSlotHoldMs{};
            bool _holdActive{};
        };

        class LoopMeasurementPhase final : public MainMeasurementPhase
        {
        public:
            LoopMeasurementPhase(MeasurementPhaseId phaseId, bool clockwise) noexcept;

            const char* Name() const noexcept override;
            MeasurementPhaseId PhaseId() const noexcept override;
            void Reset() noexcept override;
            bool Prepare(OpenFloorMeasurementController& controller) override;
            std::uint16_t MaxPrimitiveCount() const noexcept override;
            std::uint8_t MaxSpeedBinCount() const noexcept override;
            std::uint16_t MaxRepeatCount() const noexcept override;
            bool IsSlotActive(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            SlotIdentity DescribeSlot(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) const noexcept override;
            void BeginSlot(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex) override;
            LoopController::ControlVector TickActiveSlot(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController,
                bool& done) override;

        private:
            bool BuildQueue(MazeMap::Vehicle& vehicle, MazeMap::ManeuverQueue& queue) const;

            MeasurementPhaseId _phaseId;
            bool _clockwise{};
            std::array<MazeMap::ManeuverInstance, kLoopPhasePrimitiveCount> _maneuvers{};
            std::uint16_t _activePostSlotHoldMs{};
            bool _holdActive{};
        };

        class TimingStage final
        {
        public:
            void Reset() noexcept;
            bool Begin(OpenFloorMeasurementController& controller);
            LoopController::ControlVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController);
            void CompleteTimingToMainSessionTransition(
                OpenFloorMeasurementController& controller,
                MainStage& mainStage,
                LoopController& loopController);
            void FinalizeCompletedRun(OpenFloorMeasurementController& controller) noexcept;

        private:
            bool WriteBufferedRow(
                OpenFloorMeasurementController& controller,
                const char* failureReason);
            void BufferRow(const Runtime::OpenFloorTimingRow& row);
            bool CaptureComplete() const noexcept;

            std::uint16_t _tickIndex{};
            bool _logOpen{};
            std::optional<Runtime::OpenFloorTimingRow> _bufferedRow{};
        };

        class MainStage final
        {
        public:
            void Reset() noexcept;
            bool PreparePhases(OpenFloorMeasurementController& controller);
            bool Begin(OpenFloorMeasurementController& controller);
            LoopController::ControlVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController);
            void FinalizeCompletedRun(OpenFloorMeasurementController& controller) noexcept;

        private:
            MainMeasurementPhase& ActivePhase() const noexcept;
            void ResetCursor() noexcept;
            bool MoveToFirstActiveSlot() noexcept;
            bool MoveToNextActiveSlot() noexcept;
            bool SeekActiveSlotFromCurrent() noexcept;
            bool AdvanceCursorOneStep() noexcept;
            MainRowLabel BuildActiveLabels() const noexcept;
            bool WriteBufferedRow(
                OpenFloorMeasurementController& controller,
                const char* failureReason);
            void BufferRow(const Runtime::OpenFloorMainRow& row);
            bool CheckFault(OpenFloorMeasurementController& controller);

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
            std::array<MainMeasurementPhase*, kMainPhaseCount> _phases{
                &_staticPhase,
                &_launchPhase,
                &_straightPhase,
                &_yawPhase,
                &_smoothPhase,
                &_clockwiseLoopPhase,
                &_counterClockwiseLoopPhase,
            };
            std::size_t _activePhaseIndex{};
            std::uint16_t _activePrimitiveIndex{};
            std::uint8_t _activeSpeedBinIndex{};
            std::uint16_t _activeRepeatIndex{};
            bool _logOpen{};
            bool _completionPending{};
            bool _slotStarted{};
            MainRowLabel _activeLabels{};
            std::optional<Runtime::OpenFloorMainRow> _bufferedRow{};
        };

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        static void TimingToMainEndSessionThunk(void* context, LoopController& loopController);

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
        void CompleteTimingToMainEndSession(LoopController& loopController);
        void FinalizeSuccessfulRun() noexcept;
        LoopController::ControlVector TimingStageTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        LoopController::ControlVector MainStageTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        static std::uint8_t SpeedBinForIndex(std::size_t speedIndex) noexcept;

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
        SessionBoundaryAction _sessionBoundaryAction{ SessionBoundaryAction::None };
        TimingStage _timingStage{};
        MainStage _mainStage{};
    };

    IApplicationMode& GetOpenFloorMeasurementMode();
    const MazeMap::App::BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
}
