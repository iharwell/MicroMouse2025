#pragma once

#include "BootModeDescriptor.h"
#include "LoopController.h"
#include "MazeMapApplicationMode.h"
#include "MazeMapRuntimeMmLog.h"
#include "SmoothTurnYawRateController.h"

#include <cstdint>

class RuntimeSensorSuite;
class DriveBase;

namespace MazeMap::App::Internal
{
#define TOP_SPEED_MEASUREMENT_FIELDS(X)              \
    X(std::uint32_t, master_time_us)                \
    X(std::uint32_t, control_tick_sequence)         \
    X(std::uint32_t, dt_us)                         \
    X(std::uint32_t, elapsed_test_us)               \
    X(std::uint8_t,  braking)                       \
    X(std::uint8_t,  impact_detected)               \
    X(std::uint8_t,  selector_removed)              \
    X(float,         ukf_state_px_m)                \
    X(float,         ukf_state_py_m)                \
    X(float,         ukf_state_psi_rad)             \
    X(float,         ukf_state_u_mps)               \
    X(float,         ukf_state_v_mps)               \
    X(float,         ukf_state_r_radps)             \
    X(float,         ukf_state_omega_l_radps)       \
    X(float,         ukf_state_omega_r_radps)       \
    X(float,         ukf_state_bgz_radps)           \
    X(float,         measured_linear_speed_mps)     \
    X(float,         measured_angular_speed_radps)  \
    X(float,         cmd_linear_mps)                \
    X(float,         cmd_angular_radps)             \
    X(float,         heading_error_rad)             \
    X(float,         cmd_linear_accel_mps2)         \
    X(float,         left_drive_command)            \
    X(float,         right_drive_command)           \
    X(float,         left_feedforward_command)      \
    X(float,         right_feedforward_command)     \
    X(float,         left_feedback_command)         \
    X(float,         right_feedback_command)        \
    X(float,         left_target_velocity_mps)      \
    X(float,         right_target_velocity_mps)     \
    X(float,         left_launch_assist_floor)      \
    X(float,         right_launch_assist_floor)     \
    X(std::int32_t,  left_encoder_count)            \
    X(std::int32_t,  right_encoder_count)           \
    X(float,         left_encoder_omega_radps)      \
    X(float,         right_encoder_omega_radps)     \
    X(float,         left_encoder_distance_m)       \
    X(float,         right_encoder_distance_m)      \
    X(float,         left_encoder_velocity_mps)     \
    X(float,         right_encoder_velocity_mps)    \
    X(std::uint32_t, imu_timestamp_us)              \
    X(std::uint8_t,  imu_status)                    \
    X(std::uint8_t,  imu_interrupt_high)            \
    X(std::uint8_t,  accel_bias_valid)              \
    X(std::int16_t,  imu_gyro_x)                    \
    X(std::int16_t,  imu_gyro_y)                    \
    X(std::int16_t,  imu_gyro_z)                    \
    X(std::int16_t,  imu_accel_x)                   \
    X(std::int16_t,  imu_accel_y)                   \
    X(std::int16_t,  imu_accel_z)                   \
    X(std::int16_t,  imu_temp)                      \
    X(float,         gyro_raw_radps)                \
    X(float,         gyro_bias_radps)               \
    X(float,         gyro_radps)                    \
    X(float,         accel_body_x_mps2)             \
    X(float,         accel_body_y_mps2)             \
    X(float,         planar_accel_mps2)             \
    X(std::uint32_t, front_timestamp_us)            \
    X(std::uint32_t, left_timestamp_us)             \
    X(std::uint32_t, right_timestamp_us)            \
    X(float,         front_left_wall_distance_m)    \
    X(float,         front_right_wall_distance_m)   \
    X(float,         side_left_wall_distance_m)     \
    X(float,         side_right_wall_distance_m)    \
    X(float,         front_left_differential_light) \
    X(float,         front_right_differential_light)\
    X(float,         side_left_differential_light)  \
    X(float,         side_right_differential_light) \
    X(float,         fan_duty_cycle)

    MMLOG_DEFINE_ROW(TopSpeedMeasurementRow, TOP_SPEED_MEASUREMENT_FIELDS);
#undef TOP_SPEED_MEASUREMENT_FIELDS

    class SharedRobotRuntime;

    class TopSpeedMeasurementMode final : public IApplicationMode
    {
    public:
        explicit TopSpeedMeasurementMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        enum class RunPhase : std::uint8_t
        {
            PrelaunchWait,
            Running,
            Braking
        };

        enum class BrakeTrigger : std::uint8_t
        {
            None,
            TimedWindowElapsed,
            ImpactDetected,
            GyroSpikeDetected,
            SelectorRemoved
        };

        enum class CompletionReason : std::uint8_t
        {
            None,
            SettledStop,
            SelectorRemoved
        };

        static void HandleRuntimeFault(void* context, const char* reason) noexcept;
        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        bool BeginLog();
        bool FailLogSetupStep(const char* step);
        bool FlushTextLogStep(const char* step);
        void CloseLog() noexcept;
        bool WriteEvent(const char* type, const char* message);
        bool WriteRunStartEvent();
        bool WriteResultSummary(std::uint32_t completedTicks);
        bool EnterBrakingPhase(
            BrakeTrigger trigger,
            const char* eventMessage,
            LoopController::TickServices& services);
        bool EnsureSelectorStillPresent();
        bool SelectorRemoved() const noexcept;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool ImpactDetected(const LoopController::ModeState& state) const noexcept;
        bool GyroSpikeDetected(const LoopController::ModeState& state) const noexcept;
        bool EncoderMotionSettled(const LoopController::ModeState& state) noexcept;
        void UpdatePeaks(const LoopController::ModeState& state) noexcept;
        LoopController::ControlVector RunTick(
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        bool Fail(const char* reason);
        void OnRuntimeFault(const char* reason) noexcept;
        void ResetRunState() noexcept;
        void SetLastCommandInputs(float linearSpeedMps, float angularRateRadps) noexcept;
        float ReadBatteryVoltage() const noexcept;

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        RuntimeSensorSuite& _sensors;
        DriveBase& _drive;
        bool _faulted{};
        bool _logOpen{};
        bool _pinsLatchedAtBoot{};
        RunPhase _phase{ RunPhase::PrelaunchWait };
        BrakeTrigger _brakeTrigger{ BrakeTrigger::None };
        CompletionReason _completionReason{ CompletionReason::None };
        std::uint32_t _phaseStartUs{};
        std::uint32_t _measurementStartUs{};
        std::uint32_t _controlTickSequence{};
        float _batteryVoltageStart{};
        float _fanDutyCycleStart{};
        float _measurementStartYawRad{};
        float _peakMeasuredSpeedMps{};
        float _peakPlanarAccelMps2{};
        float _peakHeadingDeviationRad{};
        float _mostNegativeForwardAccelMps2{};
        bool _impactDetected{};
        float _impactSpeedMps{};
        float _impactForwardAccelMps2{};
        std::int16_t _impactGyroXRawLsb{};
        std::int16_t _impactGyroYRawLsb{};
        float _lastCommandInputLinearSpeedMps{};
        float _lastCommandInputAngularRateRadps{};
        MazeMap::SmoothTurnYawRateControllerState _gyroZHoldControllerState{};
        std::uint8_t _settledEncoderTicks{};
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        char _runId[32]{};
        TopSpeedMeasurementRow _logRow{};
    };

    IApplicationMode& GetTopSpeedMeasurementMode();
    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor();
}
