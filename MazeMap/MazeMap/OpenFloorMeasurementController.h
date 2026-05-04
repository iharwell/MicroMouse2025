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
    X(float,         speed_bin)                    \
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
        static constexpr std::size_t kSmoothPhasePrimitiveCountPerSpeed =
            kSmoothPhaseCyclePrimitiveCount + 1U;
        static constexpr std::size_t kSmoothPhasePrimitiveCount =
            (kOpenFloorSmoothSpeedBinsMps.size() * kSmoothPhasePrimitiveCountPerSpeed) + 1U;
        static constexpr std::size_t kLoopPhasePrimitiveCount = 8U;
        static constexpr std::size_t kMainRegimeCount = 7U;

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

        class MainStage;

        class MainMeasurementRegime
        {
        public:
            virtual ~MainMeasurementRegime() = default;

            virtual const char* Title() const noexcept = 0;
            virtual MeasurementPhaseId LogId() const noexcept = 0;
            virtual std::uint16_t PrimitiveCount() const noexcept = 0;
            virtual std::uint8_t SpeedCount() const noexcept = 0;
            virtual std::uint16_t RepeatCount() const noexcept = 0;
            virtual MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex) const noexcept = 0;
            virtual float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept = 0;
            virtual LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) = 0;
        };

        class StaticMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            explicit StaticMeasurementRegime(SharedRobotRuntime& runtime) noexcept;

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            SharedRobotRuntime& _runtime;
            bool _needsStart{ true };
        };

        class LaunchMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            explicit LaunchMeasurementRegime(SharedRobotRuntime& runtime) noexcept;

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            SharedRobotRuntime& _runtime;
            std::uint16_t _repeatIndex{};
            bool _needsStart{ true };
            float _activeLeftCommand{};
            float _activeRightCommand{};
            std::uint32_t _deadlineMs{};
            bool _holdActive{};
        };

        class StraightMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            explicit StraightMeasurementRegime(SharedRobotRuntime& runtime) noexcept;

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            SharedRobotRuntime& _runtime;
            bool _selectionValid{};
            std::uint8_t _activeSpeedIndex{};
            std::uint16_t _repeatIndex{};
            bool _needsStart{};
            bool _holdActive{};
        };

        class YawMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            explicit YawMeasurementRegime(SharedRobotRuntime& runtime) noexcept;

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            SharedRobotRuntime& _runtime;
            bool _selectionValid{};
            std::uint16_t _activePrimitiveIndex{};
            std::uint8_t _activeSpeedIndex{};
            bool _needsStart{};
            bool _holdActive{};
        };

        class SmoothMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            explicit SmoothMeasurementRegime(SharedRobotRuntime& runtime);

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            bool IsLastPrimitive(std::uint16_t primitiveIndex) const noexcept;
            bool BuildQueue(
                MazeMap::Vehicle& vehicle,
                std::uint8_t speedIndex,
                float cruiseSpeedMps,
                float initialEntrySpeedMps,
                MazeMap::ManeuverQueue& queue,
                float& exitBoundarySpeedMps) const;

            SharedRobotRuntime& _runtime;
            bool _valid{ true };
            std::uint16_t _primitiveCount{};
            bool _selectionValid{};
            std::uint16_t _activePrimitiveIndex{};
            bool _needsStart{};
            std::array<MazeMap::ManeuverCode, kSmoothPhasePrimitiveCount> _primitiveCodes{};
            std::array<MazeMap::ManeuverInstance, kSmoothPhasePrimitiveCount> _maneuvers{};
            std::array<float, kSmoothPhasePrimitiveCount> _speedLimitsMps{};
            std::array<float, kSmoothPhasePrimitiveCount> _speedBinValues{};
            std::uint16_t _activePostSlotHoldMs{};
            bool _holdActive{};
        };

        class LoopMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            LoopMeasurementRegime(
                SharedRobotRuntime& runtime,
                MeasurementPhaseId phaseId,
                bool clockwise);

            const char* Title() const noexcept override;
            MeasurementPhaseId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(std::uint16_t primitiveIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            LoopController::ControlVector Tick(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            bool BuildQueue(MazeMap::Vehicle& vehicle, MazeMap::ManeuverQueue& queue) const;

            SharedRobotRuntime& _runtime;
            bool _valid{ true };
            MeasurementPhaseId _phaseId;
            bool _clockwise{};
            bool _selectionValid{};
            std::uint16_t _activePrimitiveIndex{};
            std::uint16_t _repeatIndex{};
            bool _needsStart{};
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
            MainStage(MainMeasurementRegime* const* regimes, std::size_t regimeCount) noexcept;

            void Reset() noexcept;
            bool Begin(OpenFloorMeasurementController& controller);
            LoopController::ControlVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController);
            void FinalizeCompletedRun(OpenFloorMeasurementController& controller) noexcept;

        private:
            MainMeasurementRegime& ActiveRegime() const noexcept;
            void ResetIndices() noexcept;
            bool AdvanceIndices() noexcept;
            bool WriteBufferedRow(
                OpenFloorMeasurementController& controller,
                const char* failureReason);
            void BufferRow(const Runtime::OpenFloorMainRow& row);
            bool CheckFault(OpenFloorMeasurementController& controller);

            MainMeasurementRegime* const* _regimes{};
            std::size_t _regimeCount{};
            std::size_t _activeRegimeIndex{};
            std::uint16_t _activePrimitiveIndex{};
            std::uint8_t _activeSpeedIndex{};
            std::uint16_t _activeRepeatIndex{};
            bool _logOpen{};
            bool _completionPending{};
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
            MeasurementPhaseId phaseId,
            MazeMap::ManeuverCode primitiveCode,
            float speedBinValue,
            std::uint16_t repeatIndex,
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

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        RuntimeSensorSuite& _sensors;
        ::DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        StaticMeasurementRegime _staticRegime;
        LaunchMeasurementRegime _launchRegime;
        StraightMeasurementRegime _straightRegime;
        YawMeasurementRegime _yawRegime;
        SmoothMeasurementRegime _smoothRegime;
        LoopMeasurementRegime _clockwiseLoopRegime;
        LoopMeasurementRegime _counterClockwiseLoopRegime;
        std::array<MainMeasurementRegime*, kMainRegimeCount> _mainRegimes{};
        StageTick _activeStageTick{ &OpenFloorMeasurementController::TimingStageTick };
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        SessionBoundaryAction _sessionBoundaryAction{ SessionBoundaryAction::None };
        TimingStage _timingStage{};
        MainStage _mainStage;
    };

    IApplicationMode& GetOpenFloorMeasurementMode();
    const MazeMap::App::BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
}
