#pragma once

#include "CommandVector.h"
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
#include <limits>
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
        CommandVector RunTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override;

    private:
        static constexpr std::size_t kSmoothPhaseCyclePrimitiveCount = 26U;
        static constexpr std::size_t kSmoothPhasePrimitiveCountPerSpeed =
            kSmoothPhaseCyclePrimitiveCount + 1U;
        static constexpr std::size_t kSmoothPhasePrimitiveCount =
            kOpenFloorSmoothSpeedBinsMps.size() * kSmoothPhasePrimitiveCountPerSpeed;
        static constexpr std::size_t kLoopPhasePrimitiveCount = 8U;

        using StageTick = CommandVector (OpenFloorMeasurementController::*)(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        using MeasurementLogId = std::uint8_t;
        static constexpr MeasurementLogId kTimingLogId = 0U;

        class MainStage;

        class MainMeasurementRegime
        {
        public:
            virtual ~MainMeasurementRegime() = default;

            virtual const char* Title() const noexcept = 0;
            virtual MeasurementLogId LogId() const noexcept = 0;
            virtual std::uint16_t PrimitiveCount() const noexcept = 0;
            virtual std::uint8_t SpeedCount() const noexcept = 0;
            virtual std::uint16_t RepeatCount() const noexcept = 0;
            virtual MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept = 0;
            virtual float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept = 0;
            virtual bool EnsureReady(OpenFloorMeasurementController& controller);
            virtual CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) = 0;
        };

        class StaticMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static StaticMeasurementRegime& SharedInstance() noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr MeasurementLogId kLogId = 1U;

            bool _needsStart{ true };
        };

        class LaunchMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static LaunchMeasurementRegime& SharedInstance() noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr MeasurementLogId kLogId = 2U;
            static constexpr std::uint32_t kPulseTickCount =
                ((static_cast<std::uint32_t>(MazeMap::kOpenFloorLaunchPulseMs) * 1000U) +
                    DiagnosticConfig::kControlPeriodUs - 1U) /
                DiagnosticConfig::kControlPeriodUs;

            std::uint32_t _tickCounter{};
        };

        class YawLaunchMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static YawLaunchMeasurementRegime& SharedInstance() noexcept
            {
                static YawLaunchMeasurementRegime regime{};
                return regime;
            }

            const char* Title() const noexcept override { return "Yaw Launch"; }
            MeasurementLogId LogId() const noexcept override { return kLogId; }
            std::uint16_t PrimitiveCount() const noexcept override { return 2U; }
            std::uint8_t SpeedCount() const noexcept override { return 5U; }
            std::uint16_t RepeatCount() const noexcept override { return 10U; }
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override
            {
                (void)primitiveIndex;
                (void)speedIndex;
                return ManeuverCode::MC_NONE;
            }
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override
            {
                return (0.5f + speedIndex * 0.05f) * ((primitiveIndex == 0U) ? 1.0f : -1.0f);
            }
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override
            {
				done = false;
                // This is guaranteed to catch on the first tick because all
                // regimes end on a hold completion, and it's also true at boot.
                if(controller._driveService.IsEffectivelyComplete())
                {
                    _counter = 0U;
                    // In this regime, no need to wait to prime drive since we won't use it for anything but the closing hold.
                    controller._driveService.StartHold(350U, false);
				}
                if (_counter < 350U)
                {
                    _counter++;
					// Since the actual drive values are already calculated for logging purposes,
                    // we can just pull them directly with the sign naturally providing the direction.
                    const float commandVal = SpeedBinValue(primitiveIndex, speedIndex);
                    return CommandVector(commandVal, -commandVal);
                }
                else
                {
                    // No need to keep incrementing a counter we don't use.
                    return controller._driveService.GetNextControls(done);
                }
            }

        private:
            static constexpr MeasurementLogId kLogId = 20U;

            std::uint32_t _counter{0};
        };

        class StraightMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static StraightMeasurementRegime& SharedInstance() noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr MeasurementLogId kLogId = 3U;

            uint8_t _donesDetected{};
        };

        class YawMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static YawMeasurementRegime& SharedInstance() noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr MeasurementLogId kLogId = 4U;
            static constexpr std::uint16_t kRepeatCount = 3U;

            std::uint8_t _donesDetected{};
        };

        class SmoothMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static SmoothMeasurementRegime& SharedInstance() noexcept;
            SmoothMeasurementRegime() noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            bool EnsureReady(OpenFloorMeasurementController& controller) override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr MeasurementLogId kLogId = 5U;

            bool BuildQueue(
                MazeMap::Vehicle& vehicle,
                std::uint8_t speedIndex,
                const MazeMap::DirectionalLocation& startLocation,
                float cruiseSpeedMps,
                float initialEntrySpeedMps,
                MazeMap::ManeuverQueue& queue,
                float& exitBoundarySpeedMps,
                MazeMap::DirectionalLocation& endLocation) const;
            bool EnsureInitialized(MazeMap::Vehicle& vehicle);
            std::size_t ManeuverOffset(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept;

            std::uint16_t _primitiveCount{};
            const MazeMap::Vehicle* _configuredVehicle{};
            bool _maneuversInitialized{};
            std::array<MazeMap::ManeuverCode, kSmoothPhasePrimitiveCountPerSpeed> _primitiveCodes{};
            std::array<MazeMap::ManeuverInstance, kSmoothPhasePrimitiveCount> _maneuvers{};
            std::uint8_t _stage{};
        };

        class LoopMeasurementRegime final : public MainMeasurementRegime
        {
        public:
            static LoopMeasurementRegime& ClockwiseSharedInstance() noexcept;
            static LoopMeasurementRegime& CounterClockwiseSharedInstance() noexcept;

            LoopMeasurementRegime(MeasurementLogId logId, bool clockwise) noexcept;

            const char* Title() const noexcept override;
            MeasurementLogId LogId() const noexcept override;
            std::uint16_t PrimitiveCount() const noexcept override;
            std::uint8_t SpeedCount() const noexcept override;
            std::uint16_t RepeatCount() const noexcept override;
            MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex) const noexcept override;
            bool EnsureReady(OpenFloorMeasurementController& controller) override;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                std::uint16_t primitiveIndex,
                std::uint8_t speedIndex,
                bool& done) override;

        private:
            static constexpr std::uint16_t kRepeatCount = 5U;

            bool BuildQueue(MazeMap::Vehicle& vehicle, MazeMap::ManeuverQueue& queue) const;
            bool EnsureInitialized(MazeMap::Vehicle& vehicle);

            MeasurementLogId _logId;
            bool _clockwise{};
            const MazeMap::Vehicle* _configuredVehicle{};
            bool _maneuversInitialized{};
            std::array<MazeMap::ManeuverInstance, kLoopPhasePrimitiveCount> _maneuvers{};
            std::uint8_t _stage{};
        };

        class TimingStage final
        {
        public:
            bool OpenTimingLog(OpenFloorMeasurementController& controller);
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController);

        private:
            bool WriteBufferedRow(
                OpenFloorMeasurementController& controller,
                const char* failureReason);
            void BufferRow(const Runtime::OpenFloorTimingRow& row);
            bool CaptureComplete() const noexcept;

            std::uint16_t _tickIndex{};
            std::optional<Runtime::OpenFloorTimingRow> _bufferedRow{};
        };

        class MainStage final
        {
        public:
            MainStage(MainMeasurementRegime* const* regimes, std::size_t regimeCount) noexcept;

            bool OpenMainLog(OpenFloorMeasurementController& controller);
            bool IsCurrentSlotLastInRegime() const noexcept;
            CommandVector Tick(
                OpenFloorMeasurementController& controller,
                const MazeMap::VehicleState& state,
                LoopController& loopController);

        private:
            MainMeasurementRegime& ActiveRegime() const noexcept;
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
            bool _completionPending{};
            std::optional<Runtime::OpenFloorMainRow> _bufferedRow{};
        };

        static MainStage BuildRegisteredMainStage() noexcept;
        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        void PopulateTimingRowFromState(
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorTimingRow& row) const noexcept;
        void PopulateMainRowFromState(
            MeasurementLogId phaseId,
            MazeMap::ManeuverCode primitiveCode,
            float speedBinValue,
            std::uint16_t repeatIndex,
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorMainRow& row) const;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        void FinalizeSuccessfulRun() noexcept;
        bool ActiveMainStageSlotEndsRegime() const noexcept;
        CommandVector TimingStageTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        CommandVector MainStageTick(
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
        StageTick _activeStageTick{ &OpenFloorMeasurementController::TimingStageTick };
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        TimingStage _timingStage{};
        MainStage _mainStage;
        float _sessionStartPointX{ std::numeric_limits<float>::quiet_NaN() };
        float _sessionStartPointY{ std::numeric_limits<float>::quiet_NaN() };
    };

    IApplicationMode& GetOpenFloorMeasurementMode();
    const MazeMap::App::BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
}
