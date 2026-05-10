#pragma once

#include "CommandVector.h"
#include "Defines.h"
#include "DriveTelemetry.h"
#include "LoopController.h"
#include "IApplicationMode.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"

#include <cstddef>
#include <cstdint>

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

namespace MazeMap
{
    class Vehicle;
}

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;
}

namespace MazeMap::App::Internal::Runtime
{
#define SHOWCASING_DONUT_MAIN_ROW_FIELDS(FIELD) \
    FIELD(std::uint32_t, master_time_us) \
    FIELD(std::uint32_t, control_tick_sequence) \
    FIELD(std::uint32_t, dt_us) \
    FIELD(std::uint8_t, section_id) \
    FIELD(std::uint8_t, primitive_id) \
    FIELD(std::uint8_t, primitive_family) \
    FIELD(std::uint8_t, direction_id) \
    FIELD(std::uint8_t, phase_id) \
    FIELD(std::uint8_t, speed_bin) \
    FIELD(std::uint8_t, start_marker_id) \
    FIELD(std::uint16_t, repeat_index) \
    FIELD(float, progress_norm) \
    FIELD(std::uint16_t, mode_flags) \
    FIELD(std::uint32_t, clipping_flags) \
    FIELD(std::uint16_t, saturation_flags) \
    FIELD(std::uint16_t, watchdog_flags) \
    FIELD(std::uint16_t, measurement_flags) \
    FIELD(std::uint8_t, ukf_mode_id) \
    FIELD(std::uint8_t, ukf_yaw_valid_for_feedforward) \
    FIELD(std::uint8_t, bias_update_enabled) \
    FIELD(float, ukf_state_px_m) \
    FIELD(float, ukf_state_py_m) \
    FIELD(float, ukf_state_psi_rad) \
    FIELD(float, ukf_state_u_mps) \
    FIELD(float, ukf_state_v_mps) \
    FIELD(float, ukf_state_r_radps) \
    FIELD(float, ukf_state_omega_l_radps) \
    FIELD(float, ukf_state_omega_r_radps) \
    FIELD(float, ukf_state_bgz_radps) \
    FIELD(float, gyro_bias_anchor_radps) \
    FIELD(float, yaw_consistency_lp_radps) \
    FIELD(float, yaw_window_mismatch_rad) \
    FIELD(float, nhc_sigma_mps) \
    FIELD(float, nhc_residual_mps) \
    FIELD(float, nhc_residual_sigma) \
    FIELD(float, measured_linear_speed_mps) \
    FIELD(float, measured_angular_speed_radps) \
    FIELD(float, cmd_linear_mps) \
    FIELD(float, cmd_angular_radps) \
    FIELD(float, left_drive_command) \
    FIELD(float, right_drive_command) \
    FIELD(float, left_feedforward_command) \
    FIELD(float, right_feedforward_command) \
    FIELD(float, left_feedback_command) \
    FIELD(float, right_feedback_command) \
    FIELD(float, left_target_velocity_mps) \
    FIELD(float, right_target_velocity_mps) \
    FIELD(float, left_launch_assist_floor) \
    FIELD(float, right_launch_assist_floor) \
    FIELD(std::uint32_t, encoder_timestamp_us) \
    FIELD(std::int32_t, left_encoder_count) \
    FIELD(std::int32_t, right_encoder_count) \
    FIELD(float, left_encoder_omega_radps) \
    FIELD(float, right_encoder_omega_radps) \
    FIELD(float, left_encoder_distance_m) \
    FIELD(float, right_encoder_distance_m) \
    FIELD(float, left_encoder_velocity_mps) \
    FIELD(float, right_encoder_velocity_mps) \
    FIELD(std::uint32_t, imu_timestamp_us) \
    FIELD(std::uint8_t, imu_status) \
    FIELD(std::uint8_t, imu_interrupt_high) \
    FIELD(std::uint8_t, accel_bias_valid) \
    FIELD(std::int16_t, imu_gyro_x) \
    FIELD(std::int16_t, imu_gyro_y) \
    FIELD(std::int16_t, imu_gyro_z) \
    FIELD(std::int16_t, imu_accel_x) \
    FIELD(std::int16_t, imu_accel_y) \
    FIELD(std::int16_t, imu_accel_z) \
    FIELD(std::int16_t, imu_temp) \
    FIELD(float, gyro_raw_radps) \
    FIELD(float, gyro_bias_radps) \
    FIELD(float, gyro_radps) \
    FIELD(float, accel_body_x_mps2) \
    FIELD(float, accel_body_y_mps2) \
    FIELD(float, planar_accel_mps2) \
    FIELD(std::uint32_t, front_timestamp_us) \
    FIELD(std::uint32_t, left_timestamp_us) \
    FIELD(std::uint32_t, right_timestamp_us) \
    FIELD(std::uint8_t, front_left_obs_class) \
    FIELD(std::uint8_t, front_right_obs_class) \
    FIELD(std::uint8_t, left_obs_class) \
    FIELD(std::uint8_t, right_obs_class) \
    FIELD(float, front_left_obs_rho_m) \
    FIELD(float, front_right_obs_rho_m) \
    FIELD(float, left_obs_rho_m) \
    FIELD(float, right_obs_rho_m) \
    FIELD(float, front_left_obs_confidence) \
    FIELD(float, front_right_obs_confidence) \
    FIELD(float, left_obs_confidence) \
    FIELD(float, right_obs_confidence) \
    FIELD(float, fan_duty_cycle)

    MMLOG_DEFINE_ROW(ShowcasingDonutMainRow, SHOWCASING_DONUT_MAIN_ROW_FIELDS);

#undef SHOWCASING_DONUT_MAIN_ROW_FIELDS
}

namespace MazeMap::App::Internal
{
    class EXPORT ShowcasingDonutController final : public IApplicationMode
    {
    public:
        explicit ShowcasingDonutController(SharedRobotRuntime& runtime);
        ~ShowcasingDonutController() override = default;

        ShowcasingDonutController(const ShowcasingDonutController&) = delete;
        ShowcasingDonutController& operator=(const ShowcasingDonutController&) = delete;
        ShowcasingDonutController(ShowcasingDonutController&&) = delete;
        ShowcasingDonutController& operator=(ShowcasingDonutController&&) = delete;

        void SetupMode() override;
        CommandVector RunTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override;

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            DonutSweep,
            FlashyMoves,
            Complete,
        };

        enum class EndReason : std::uint8_t
        {
            None,
            TractionLoss,
            SpeedCap,
        };

        struct LogLabels final
        {
            std::uint8_t sectionId{};
            std::uint8_t primitiveId{};
            std::uint8_t directionId{};
            std::uint8_t phaseId{};
            std::uint8_t speedBin{};
            std::uint8_t startMarkerId{};
            std::uint16_t repeatIndex{};
            float progressNorm{};
        };

        struct TractionLossState final
        {
            float mismatchDurationS{};
            float lastYawCoherence{};
            float lastPlanarCoherence{};
        };

        static constexpr std::size_t kLogFileNameCapacity = 96U;

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        void ResetState() noexcept;
        bool BeginMainLog();
        bool WriteBufferedMainRow();
        bool LogCurrentSample(
            const LogLabels& labels,
            const MazeMap::VehicleState& state,
            bool abortMarker);
        void PopulateMainRow(
            const LogLabels& labels,
            const MazeMap::VehicleState& state,
            bool abortMarker,
            Runtime::ShowcasingDonutMainRow& row) const;
        void UpdatePeaks(const MazeMap::VehicleState& state) noexcept;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        bool BeginDonutSweep() noexcept;
        bool BeginFlashTurn(float angleRad) noexcept;
        bool TractionLossDetected(const MazeMap::VehicleState& state) noexcept;
        bool EndConditionReached(const MazeMap::VehicleState& state);
        void RecordRequestedEnd(EndReason reason);
        const char* EndReasonText(EndReason reason) const noexcept;
        LogLabels CurrentLabels() const noexcept;
        std::uint8_t SpeedBinForSpeed(float speedMps) const noexcept;
        float EncoderAverageSpeedMps(const MazeMap::VehicleState& state) const noexcept;
        float CommandedYawRateRadps() const noexcept;

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        ::DriveBase& _drive;
        Drive& _driveService;
        Phase _phase{ Phase::Idle };
        EndReason _endReason{ EndReason::None };
        char _logFileName[kLogFileNameCapacity]{};
        bool _mainLogOpen{};
        Runtime::ShowcasingDonutMainRow _bufferedMainRow{};
        bool _bufferedMainRowValid{};
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        float _commandedSpeedMps{};
        float _peakCommandedSpeedMps{};
        float _peakEncoderSpeedMps{};
        float _peakYawRateRadps{};
        float _peakPlanarAccelMps2{};
        std::uint8_t _flashTurnsStarted{};
        TractionLossState _tractionLoss{};
    };

    IApplicationMode& GetShowcasingDonutMode();
    const MazeMap::App::BootModeDescriptor& GetShowcasingDonutBootModeDescriptor();
}
