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

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

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

        enum class DrivePrimitiveKind : std::uint8_t
        {
            Straight,
            Turn,
            Maneuver,
        };

        enum class ManeuverQueueKind : std::uint8_t
        {
            SmoothSweep,
            Loop,
        };

        struct SegmentIdentity final
        {
            constexpr SegmentIdentity() = default;
            constexpr SegmentIdentity(
                OpenFloorSectionId phaseId,
                OpenFloorPrimitiveId primitiveId,
                OpenFloorSpeedBin speedBin,
                std::uint16_t repeatIndex) noexcept
                : phaseId(phaseId)
                , primitiveId(primitiveId)
                , speedBin(speedBin)
                , repeatIndex(repeatIndex)
            {
            }

            OpenFloorSectionId phaseId = OpenFloorSectionId::Sec00Timing;
            OpenFloorPrimitiveId primitiveId = OpenFloorPrimitiveId::None;
            OpenFloorSpeedBin speedBin = OpenFloorSpeedBin::None;
            std::uint16_t repeatIndex = 0U;
        };

        struct SegmentTickResult final
        {
            bool done{};
        };

        struct MainSegment;

        struct HoldSegmentPayload final
        {
            std::uint16_t durationMs{};
        };

        struct WheelCommandProfilePayload final
        {
            std::uint16_t durationMs{};
            float leftCommand{};
            float rightCommand{};
        };

        struct StraightDrivePrimitivePayload final
        {
            float distanceM{};
            float speedMps{};
        };

        struct TurnDrivePrimitivePayload final
        {
            float yawRad{};
            float maxOmegaRadps{};
        };

        struct ManeuverDrivePrimitivePayload final
        {
            std::uint16_t maneuverIndex{};
            float speedMps{};
        };

        struct DrivePrimitivePayload final
        {
            DrivePrimitiveKind kind{ DrivePrimitiveKind::Straight };
            StraightDrivePrimitivePayload straight{};
            TurnDrivePrimitivePayload turn{};
            ManeuverDrivePrimitivePayload maneuver{};
        };

        struct WheelCommandSweepDefinition final
        {
            OpenFloorSectionId phaseId{ OpenFloorSectionId::Sec00Timing };
            OpenFloorPrimitiveId primitiveId{ OpenFloorPrimitiveId::None };
            const float* magnitudes{};
            std::size_t magnitudeCount{};
            std::uint16_t durationMs{};
            std::uint8_t repeatsPerMagnitude{};
            float leftScale{};
            float rightScale{};
            bool alternateSign{};
            std::uint16_t settlingHoldMs{};
        };

        struct StraightSweepDefinition final
        {
            OpenFloorSectionId phaseId{ OpenFloorSectionId::Sec00Timing };
            OpenFloorPrimitiveId primitiveId{ OpenFloorPrimitiveId::None };
            const float* speedsMps{};
            std::size_t speedCount{};
            float distanceM{};
            std::uint8_t repeatsPerSpeed{};
            bool alternateDirection{};
            std::uint16_t settlingHoldMs{};
        };

        struct TurnSweepDefinition final
        {
            OpenFloorSectionId phaseId{ OpenFloorSectionId::Sec00Timing };
            const OpenFloorPrimitiveId* primitiveIds{};
            const float* nominalAnglesRad{};
            std::size_t primitiveCount{};
            const float* omegaBinsRadps{};
            std::size_t omegaBinCount{};
            std::uint8_t repeatsPerOmegaBin{};
            std::uint16_t settlingHoldMs{};
        };

        struct ManeuverQueueDefinition final
        {
            ManeuverQueueKind kind{ ManeuverQueueKind::SmoothSweep };
            OpenFloorSectionId phaseId{ OpenFloorSectionId::Sec00Timing };
            const float* speedBinsMps{};
            std::size_t speedCount{};
            bool clockwise{};
            std::uint16_t repeatCount{};
            std::uint16_t settlingHoldMs{};
        };

        struct HoldSegmentRuntime final
        {
            bool started{};
        };

        struct WheelCommandProfileRuntime final
        {
            bool started{};
            bool settling{};
            std::uint32_t deadlineMs{};
        };

        struct StraightDrivePrimitiveRuntime final
        {
            float startDistanceM{};
            float totalDistanceM{};
        };

        struct TurnDrivePrimitiveRuntime final
        {
            float targetYawRad{};
            float targetMagnitudeRad{};
        };

        struct ManeuverDrivePrimitiveRuntime final
        {
            float startDistanceM{};
            float totalDistanceM{};
            float targetYawRad{};
            float targetMagnitudeRad{};
        };

        struct DrivePrimitiveRuntime final
        {
            bool started{};
            bool settling{};
            StraightDrivePrimitiveRuntime straight{};
            TurnDrivePrimitiveRuntime turn{};
            ManeuverDrivePrimitiveRuntime maneuver{};
        };

        struct SegmentRuntime final
        {
            HoldSegmentRuntime hold{};
            WheelCommandProfileRuntime wheelCommandProfile{};
            DrivePrimitiveRuntime drivePrimitive{};
        };

        struct SegmentExecutor final
        {
            using TickFn = LoopController::ControlVector (*)(
                OpenFloorMeasurementController& controller,
                SegmentRuntime& runtime,
                const MainSegment& segment,
                const MazeMap::VehicleState& state,
                LoopController::TickServices& services,
                SegmentTickResult& result);

            TickFn tick{};
        };

        struct MainSegment final
        {
            const SegmentExecutor* executor{};
            SegmentIdentity identity{};
            std::uint16_t settlingHoldMs{};
            HoldSegmentPayload hold{};
            WheelCommandProfilePayload wheelCommandProfile{};
            DrivePrimitivePayload drivePrimitive{};
        };

        struct TimingStage final
        {
            std::uint16_t tickIndex{};
            bool logOpen{};
            bool pendingSampleValid{};
            Runtime::OpenFloorTimingRow pendingRow{};
        };

        struct MainStage final
        {
            std::array<MainSegment, kMainSegmentCapacity> plan{};
            std::array<MazeMap::ManeuverInstance, kCompiledManeuverCapacity> maneuvers{};
            std::uint16_t planSize{};
            std::uint16_t maneuverCount{};
            std::uint16_t nextSegmentIndex{};
            bool logOpen{};
            bool completionPending{};
            SegmentRuntime activeRuntime{};
            char estimatorFaultReason[64]{};
            bool pendingSampleValid{};
            Runtime::OpenFloorMainRow pendingRow{};
        };

        static const SegmentExecutor kHoldSegmentExecutor;
        static const SegmentExecutor kWheelCommandProfileExecutor;
        static const SegmentExecutor kDrivePrimitiveExecutor;

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        static LoopController::PauseDisposition PauseThunk(
            void* context,
            const LoopController::PauseContext& pause);
        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);
        static LoopController::ControlVector ExecuteHoldSegment(
            OpenFloorMeasurementController& controller,
            SegmentRuntime& runtime,
            const MainSegment& segment,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services,
            SegmentTickResult& result);
        static LoopController::ControlVector ExecuteWheelCommandProfileSegment(
            OpenFloorMeasurementController& controller,
            SegmentRuntime& runtime,
            const MainSegment& segment,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services,
            SegmentTickResult& result);
        static LoopController::ControlVector ExecuteDrivePrimitiveSegment(
            OpenFloorMeasurementController& controller,
            SegmentRuntime& runtime,
            const MainSegment& segment,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services,
            SegmentTickResult& result);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        void ResetState() noexcept;
        bool BeginTimingLog();
        void StagePendingTimingSample(const Runtime::OpenFloorTimingRow& row) noexcept;
        bool BeginMainLog();
        bool CommitPendingMainSample(
            LoopController::TickServices& services,
            const char* failureReason);
        void StagePendingMainSample(const Runtime::OpenFloorMainRow& row) noexcept;
        void PopulateTimingRowFromState(
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorTimingRow& row) const noexcept;
        void PopulateMainRowFromState(
            const SegmentIdentity& identity,
            const MazeMap::VehicleState& state,
            Runtime::OpenFloorMainRow& row) const;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool SelectorRemoved() const noexcept;
        bool CompileMainPlan();
        bool ResetMainPlan() noexcept;
        bool AppendSegment(const MainSegment& segment);
        bool AppendCompiledManeuverSegment(
            MainSegment segment,
            const MazeMap::ManeuverInstance& maneuver);
        bool StoreCompiledManeuver(
            const MazeMap::ManeuverInstance& maneuver,
            std::uint16_t& maneuverIndex);
        bool CompileWheelCommandSweep(const WheelCommandSweepDefinition& definition);
        bool CompileStraightSweep(const StraightSweepDefinition& definition);
        bool CompileTurnSweep(const TurnSweepDefinition& definition);
        bool CompileManeuverQueue(const ManeuverQueueDefinition& definition);
        bool CheckFault(LoopController::TickServices& services, bool mainStage);
        void AdvanceMainSegment() noexcept;
        const MainSegment* ActiveMainSegment() const noexcept;
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
