#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

namespace MazeMap::App::Internal::Runtime
{
#define OPEN_FLOOR_TIMING_FIELDS(X)                \
    X(std::uint32_t, mono_time_us)                \
    X(std::uint32_t, control_tick_sequence)       \
    X(std::uint32_t, dt_us)                       \
    X(std::uint32_t, section_id)                  \
    X(std::uint32_t, control_start_us)            \
    X(std::uint32_t, control_end_us)              \
    X(std::uint32_t, pwm_latch_us)                \
    X(std::uint32_t, encoder_latch_us)            \
    X(std::uint32_t, encoder_read_done_us)        \
    X(std::uint32_t, ukf_predict_start_us)        \
    X(std::uint32_t, ukf_predict_end_us)          \
    X(std::uint32_t, ukf_predict_duration_us)     \
    X(std::uint32_t, ukf_update_start_us)         \
    X(std::uint32_t, ukf_update_end_us)           \
    X(std::uint32_t, ukf_update_duration_us)      \
    X(std::uint32_t, imu_drdy_us)                 \
    X(std::uint32_t, imu_read_start_us)           \
    X(std::uint32_t, imu_read_done_us)            \
    X(std::uint32_t, front_led_on_us)             \
    X(std::uint32_t, front_adc_on_us)             \
    X(std::uint32_t, front_led_off_us)            \
    X(std::uint32_t, front_adc_off_us)            \
    X(std::uint32_t, front_ready_us)              \
    X(std::uint32_t, left_led_on_us)              \
    X(std::uint32_t, left_adc_on_us)              \
    X(std::uint32_t, left_led_off_us)             \
    X(std::uint32_t, left_adc_off_us)             \
    X(std::uint32_t, left_ready_us)               \
    X(std::uint32_t, right_led_on_us)             \
    X(std::uint32_t, right_adc_on_us)             \
    X(std::uint32_t, right_led_off_us)            \
    X(std::uint32_t, right_adc_off_us)            \
    X(std::uint32_t, right_ready_us)              \
    X(std::uint32_t, wall_adc_cfg_before_start)   \
    X(std::uint32_t, wall_adc_gc_before_start)    \
    X(std::uint32_t, wall_adc_cfg_after_start)    \
    X(std::uint32_t, wall_adc_gc_after_start)     \
    X(std::uint32_t, wall_adc_target_cfg)         \
    X(std::uint32_t, wall_adc_ipg_clock_hz)       \
    X(std::uint32_t, cycle_counter_start)         \
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

inline constexpr std::uint16_t kOpenFloorLoggerFlagOverflow = 1u << 0;
inline constexpr std::uint16_t kOpenFloorLoggerFlagWriteFailure = 1u << 1;

inline constexpr std::uint16_t kOpenFloorMeasurementFlagAbortMarker = 1u << 0;
// Keep bit 1 unused so historic open-floor logs remain layout-compatible.
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
    bool encoderValid,
    bool imuValid,
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

using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

class OpenFloorMeasurementController : public IApplicationMode
{
public:
    explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime);

    bool Begin() override;
    void Run() override;
    void OnRuntimeFault(const char* message) noexcept;

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    using PhaseFn = LoopController::ControlVector (OpenFloorMeasurementController::*)(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    static constexpr uint16_t kWatchdogFlagTranslationStall = 1u << 0;
    static constexpr uint16_t kWatchdogFlagSectionTimeout = 1u << 1;
    static constexpr uint16_t kWatchdogFlagRecoveryTimeout = 1u << 2;
    // Policy: no watchdog timer in this codebase may trigger in under 90 seconds.
    static constexpr unsigned long kMinimumFailureTimeoutMs = 90000UL;

    enum class PendingLogKind : std::uint8_t
    {
        None,
        TimingSample,
        TimingFault,
        MainSample,
        MainFault
    };

    struct PendingLog final
    {
        PendingLogKind kind{ PendingLogKind::None };
        OpenFloorMeasurementCycle cycle{};
        OpenFloorMeasurementLabels labels{};
        MazeMap::OpenFloorFaultCode faultCode{};
        bool controlHalted{};
        std::uint32_t extra0{};
        std::uint32_t extra1{};
    };

    struct TimingBlockState final
    {
        std::uint16_t tickIndex{};
    };

    struct StaticHoldState final
    {
        OpenFloorMeasurementLabels labels{};
        unsigned long deadlineMs{};
    };

    struct RecoveryState final
    {
        OpenFloorMeasurementLabels labels{};
        float targetX{};
        float targetY{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        MotionLimits limits{};
        unsigned long deadlineMs{};
        float initialDistanceOutsideZoneM{};
    };

    struct LaunchPulseState final
    {
        enum class Phase : std::uint8_t
        {
            LaunchPulse,
            BrakeToStop,
            HoldSettled
        } phase{ Phase::LaunchPulse };

        OpenFloorMeasurementLabels labels{};
        float signedDriveCommand{};
        unsigned long pulseDeadlineMs{};
        MazeMap::VehicleState stationaryCheckState{};
        bool previousStationary{};
        bool launchFlippedStationary{};
        unsigned long settleStartMs{};
    };

    struct StraightSectionState final
    {
        OpenFloorMeasurementLabels labels{};
        float distanceM{};
        float cruiseSpeedMps{};
        MotionLimits limits{};
        float straightDirectionSign{};
        float startDistanceM{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        float commandedSpeedMagnitudeMps{};
        EncoderProgressWatchdog translationWatchdog{};
        unsigned long timeoutMs{};
    };

    struct SectionSettleState final
    {
        enum class Destination : std::uint8_t
        {
            None,
            Yaw
        } destination{ Destination::None };

        OpenFloorMeasurementLabels labels{};
        MazeMap::VehicleState stationaryCheckState{};
        bool holdingSettled{};
        unsigned long settleStartMs{};
    };

    struct TurnSectionState final
    {
        OpenFloorMeasurementLabels labels{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        float targetYawRad{};
        float targetMagnitude{};
        unsigned long timeoutMs{};
    };

    struct SmoothTurnState final
    {
        OpenFloorMeasurementLabels labels{};
        MazeMap::ManeuverInstance maneuver{};
        float cruiseSpeed{};
        float totalDistanceM{};
        float startDistanceM{};
        EncoderProgressWatchdog translationWatchdog{};
        unsigned long timeoutMs{};
        MotionLimits limits{};
        MazeMap::SmoothTurnYawRateControllerState yawRateController{};
    };

    enum class PauseAction : std::uint8_t
    {
        None,
        TimingToMain
    };

    struct LaunchSequenceState final
    {
        std::size_t magnitudeIndex{};
        std::uint8_t repeatIteration{};
        bool negativeNext{};
        std::uint16_t nextRepeatIndex{};
    };

    struct StraightSequenceState final
    {
        std::size_t speedIndex{};
        std::uint8_t repeatIteration{};
        bool southboundNext{};
        std::uint16_t nextRepeatIndex{};
    };

    struct YawSequenceState final
    {
        std::size_t speedIndex{};
        std::uint8_t repeatIteration{};
        std::uint8_t primitiveIndex{};
        std::uint16_t nextRepeatIndex{};
    };

    struct SmoothSequenceState final
    {
        std::size_t speedIndex{};
        std::size_t codeIndex{};
        std::uint8_t repeatIteration{};
        std::uint16_t nextRepeatIndex{};
    };

    struct LoopSequenceState final
    {
        bool clockwise{};
        std::uint16_t repeatIndex{};
        std::uint8_t legIndex{};
        OpenFloorMeasurementLabels loopLabels{};
        OpenFloorMeasurementLabels recoveryLabels{};
    };

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _vehicle;
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    bool _pinsLatchedAtBoot;
    float _batteryVoltageStart;
    float _fanDutyCycleStart;
    bool _timingOverflowed;
    bool _timingWriteFailed;
    bool _mainOverflowed;
    bool _mainWriteFailed;
    bool _faulted;
    bool _timingLogOpen;
    bool _mainLogOpen;
    unsigned long _primaryDiagnosticSelectorInactiveStartMs;
    uint32_t _controlTickSequence;
    char _runId[24];
    uint8_t _primaryDiagnosticSelectorDrivePin;
    uint8_t _primaryDiagnosticSelectorSensePin;
    bool _primaryDiagnosticSelectorMonitorArmed;
    bool _primaryDiagnosticSelectorInactive;
    OpenFloorTimingRow _timingLogRow{};
    OpenFloorMainRow _mainLogRow{};
    PhaseFn _phaseFn{};
    PauseAction _pauseAction{};
    PendingLog _pendingLog{};
    TimingBlockState _timingBlockState{};
    StaticHoldState _staticHoldState{};
    RecoveryState _recoveryState{};
    LaunchPulseState _launchPulseState{};
    StraightSectionState _straightSectionState{};
    SectionSettleState _sectionSettleState{};
    TurnSectionState _turnSectionState{};
    SmoothTurnState _smoothTurnState{};
    LaunchSequenceState _launchSequenceState{};
    StraightSequenceState _straightSequenceState{};
    YawSequenceState _yawSequenceState{};
    SmoothSequenceState _smoothSequenceState{};
    LoopSequenceState _loopSequenceState{};

    static MotionLimits MeasurementLimits(float maxSpeedMps);
    static void HandleRuntimeFault(void* context, const char* reason) noexcept;
    static LoopController::PauseDisposition PauseThunk(
        void* context,
        const LoopController::PauseContext& pause);
    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    static uint32_t ReadCycleCounter();
    static MazeMap::OpenFloorPrimitiveId PrimitiveIdForSmoothCode(MazeMap::ManeuverCode code);
    static unsigned long FailureTimeoutMs(unsigned long requestedTimeoutMs);
    static MazeMap::OpenFloorPhaseId StraightPhaseForProgress(float progress);
    static MazeMap::OpenFloorPhaseId TurnPhaseForProgress(float progress);
    static MazeMap::OpenFloorSpeedBin SpeedBinForIndex(std::size_t speedIndex) noexcept;

    LoopController::SessionOptions BuildLoopOptions() const;
    void ResetPendingLog() noexcept;
    bool EmitPendingLog();
    bool EmitPendingLog(LoopController::TickServices& services);
    void ApplyPublishedTiming(OpenFloorMeasurementCycle& cycle) const noexcept;
    void StageTimingSample(const OpenFloorMeasurementCycle& cycle) noexcept;
    void StageTimingFault(
        const OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        bool controlHalted,
        std::uint32_t extra0 = 0UL,
        std::uint32_t extra1 = 0UL) noexcept;
    void StageMainSample(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle) noexcept;
    void StageMainFault(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        bool controlHalted,
        std::uint32_t extra0 = 0UL,
        std::uint32_t extra1 = 0UL) noexcept;

    bool BeginTimingLog();
    bool LogTimingFault(
        const OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        bool controlHalted,
        uint32_t extra0 = 0UL,
        uint32_t extra1 = 0UL);
    void ServiceLogs();
    void CloseTimingLog();
    void RecordTimingLogFailure() noexcept;

    bool BeginMainLog();
    bool LogMainFault(
        const OpenFloorMeasurementLabels& labels,
        const OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        bool controlHalted,
        uint32_t extra0 = 0UL,
        uint32_t extra1 = 0UL);
    bool WriteMainSectionMarker(const char* type, const OpenFloorMeasurementLabels& labels, const char* reason = nullptr);
    bool BeginMainSection(const OpenFloorMeasurementLabels& labels);
    bool EndMainSection(const OpenFloorMeasurementLabels& labels);
    bool AbortMainSection(const OpenFloorMeasurementLabels& labels, const char* reason);
    bool WriteMainEvent(const char* type, const char* message);
    bool WriteFaultDumpTextEntry(const char* type, const char* message);
    bool DumpUkfFaultToTextLog(const char* reason);
    void CloseMainLog();
    void RecordMainLogFailure() noexcept;
    bool FailLogSetupStep(const char* logName, const char* step);

    bool Fail(const char* message);
    void SeedPoseAtMarker(MazeMap::OpenFloorMarkerId markerId);
    void ConfigurePrimaryDiagnosticSelectorMonitor() noexcept;
    void ReleasePrimaryDiagnosticSelectorMonitor() noexcept;
    bool HasPrimaryDiagnosticSelectorRemovalFault() noexcept;
    LoopController::PauseDisposition OnPauseGranted(const LoopController::PauseContext& pause);
    bool TransitionFromTimingToMain();
    float ReadBatteryVoltage() const;
    float ReadBoardTemperatureC(const SensorSnapshot& snapshot) const;
    void PopulateCycleFromState(
        const LoopController::ModeState& state,
        OpenFloorMeasurementCycle& cycle);
    bool QueueMeasurementCaptureFault(
        OpenFloorMeasurementLabels& labels,
        OpenFloorMeasurementCycle& cycle);
    bool StartTimingBlockPhase();
    bool StartTimingToMainPausePhase();
    bool StartStaticSectionPhase();
    bool StartLaunchPulsePhase(float signedDriveCommand, uint16_t repeatIndex);
    bool StartStraightDistancePhase(
        MazeMap::OpenFloorSectionId sectionId,
        MazeMap::OpenFloorMarkerId markerId,
        MazeMap::OpenFloorDirectionId directionId,
        float distanceM,
        float cruiseSpeedMps,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin,
        MazeMap::OpenFloorPrimitiveId primitiveId,
        bool emitSectionMarkers = true);
    bool StartInPlaceTurnPhase(
        MazeMap::OpenFloorPrimitiveId primitiveId,
        MazeMap::OpenFloorDirectionId directionId,
        float angleRad,
        float maxOmegaRadps,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin,
        bool emitSectionMarkers = true);
    bool StartSectionSettlePhase(const OpenFloorMeasurementLabels& labels, SectionSettleState::Destination destination);
    bool StartSmoothTurnPhase(
        MazeMap::ManeuverCode code,
        float cruiseSpeed,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin);
    bool StartRecoveryPhase(
        const OpenFloorMeasurementLabels& labels,
        MazeMap::OpenFloorMarkerId markerId,
        float maxSpeedMps,
        unsigned long timeoutMs,
        bool emitSectionMarkers);
    bool AdvanceLaunchSequence();
    bool AdvanceStraightSequence();
    bool AdvanceYawSequence();
    bool AdvanceSmoothSequence();
    bool StartLoopSequence(bool clockwise);
    bool AdvanceLoopSequenceAfterRepeat();
    bool AdvanceLoopAfterRecovery();
    bool AdvanceLoopAfterStraight();
    bool AdvanceLoopAfterTurn();

    LoopController::ControlVector RunTimingBlockTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector TimingToMainPauseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector RunStaticSectionTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector RecoverToMarkerTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector ExecuteLaunchPulseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector ExecuteStraightDistanceTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector ExecuteSectionSettleTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector ExecuteInPlaceTurnTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
    LoopController::ControlVector ExecuteSmoothTurnTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);
};

OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
    : _runtime(runtime)
    , _loopController(runtime.ControlLoop())
    , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
    , _drive(runtime.Drive())
    , _pinsLatchedAtBoot(false)
    , _batteryVoltageStart(0.0f)
    , _fanDutyCycleStart(0.0f)
    , _timingOverflowed(false)
    , _timingWriteFailed(false)
    , _mainOverflowed(false)
    , _mainWriteFailed(false)
    , _faulted(false)
    , _timingLogOpen(false)
    , _mainLogOpen(false)
    , _primaryDiagnosticSelectorInactiveStartMs(0UL)
    , _controlTickSequence(0UL)
    , _primaryDiagnosticSelectorDrivePin(0U)
    , _primaryDiagnosticSelectorSensePin(0U)
    , _primaryDiagnosticSelectorMonitorArmed(false)
    , _primaryDiagnosticSelectorInactive(false)
{
    _runId[0] = '\0';
}

MazeMap::App::Internal::LoopController::SessionOptions OpenFloorMeasurementController::BuildLoopOptions() const
{
    LoopController::SessionOptions options{};
    options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
    return options;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ModeWorkThunk(
    void* context,
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    auto* const self = static_cast<OpenFloorMeasurementController*>(context);
    if ((self == nullptr) || (self->_phaseFn == nullptr))
    {
        services.Fault("Open-floor measurement phase callback was not installed");
        return LoopController::ControlVector::Brake;
    }

    if (!self->EmitPendingLog(services))
    {
        return LoopController::ControlVector::Brake;
    }

    return (self->*self->_phaseFn)(loopEndTimeUs, state, services);
}

MazeMap::App::Internal::LoopController::PauseDisposition OpenFloorMeasurementController::PauseThunk(
    void* context,
    const MazeMap::App::Internal::LoopController::PauseContext& pause)
{
    auto* const self = static_cast<OpenFloorMeasurementController*>(context);
    if (self == nullptr)
    {
        return LoopController::PauseDisposition::StopByRuntime(
            "Open-floor measurement pause callback context was null");
    }

    return self->OnPauseGranted(pause);
}

MazeMap::OpenFloorSpeedBin OpenFloorMeasurementController::SpeedBinForIndex(const std::size_t speedIndex) noexcept
{
    return (speedIndex == 0U) ? MazeMap::OpenFloorSpeedBin::Low :
        (speedIndex == 1U) ? MazeMap::OpenFloorSpeedBin::Medium :
        MazeMap::OpenFloorSpeedBin::High;
}

void OpenFloorMeasurementController::ResetPendingLog() noexcept
{
    _pendingLog = PendingLog{};
}

bool OpenFloorMeasurementController::EmitPendingLog()
{
    if (_pendingLog.kind == PendingLogKind::None)
    {
        return true;
    }

    PendingLog pending = _pendingLog;
    ResetPendingLog();
    ApplyPublishedTiming(pending.cycle);
    auto emitTimingSample = [this, &pending]() -> bool
    {
        _timingLogRow = {};
        _timingLogRow.mono_time_us = pending.cycle.masterTimeUs;
        _timingLogRow.control_tick_sequence = pending.cycle.controlTickSequence;
        _timingLogRow.dt_us = pending.cycle.dtUs;
        _timingLogRow.section_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
        _timingLogRow.control_start_us = pending.cycle.controlTiming.controlStartUs;
        _timingLogRow.control_end_us = pending.cycle.controlTiming.controlEndUs;
        _timingLogRow.pwm_latch_us = pending.cycle.controlTiming.pwmLatchUs;
        _timingLogRow.encoder_latch_us = pending.cycle.controlTiming.encoderLatchUs;
        _timingLogRow.encoder_read_done_us = pending.cycle.controlTiming.encoderReadDoneUs;
        _timingLogRow.ukf_predict_start_us = pending.cycle.controlTiming.ukfPredictStartUs;
        _timingLogRow.ukf_predict_end_us = pending.cycle.controlTiming.ukfPredictEndUs;
        _timingLogRow.ukf_predict_duration_us = pending.cycle.controlTiming.ukfPredictDurationUs;
        _timingLogRow.ukf_update_start_us = pending.cycle.controlTiming.ukfUpdateStartUs;
        _timingLogRow.ukf_update_end_us = pending.cycle.controlTiming.ukfUpdateEndUs;
        _timingLogRow.ukf_update_duration_us = pending.cycle.controlTiming.ukfUpdateDurationUs;
        _timingLogRow.imu_drdy_us = pending.cycle.sensorSnapshot.imuTiming.drdyUs;
        _timingLogRow.imu_read_start_us = pending.cycle.sensorSnapshot.imuTiming.readStartUs;
        _timingLogRow.imu_read_done_us = pending.cycle.sensorSnapshot.imuTiming.readDoneUs;
        _timingLogRow.front_led_on_us = pending.cycle.sensorSnapshot.frontTiming.ledOnCommandUs;
        _timingLogRow.front_adc_on_us = pending.cycle.sensorSnapshot.frontTiming.adcOnSampleUs;
        _timingLogRow.front_led_off_us = pending.cycle.sensorSnapshot.frontTiming.ledOffCommandUs;
        _timingLogRow.front_adc_off_us = pending.cycle.sensorSnapshot.frontTiming.adcOffSampleUs;
        _timingLogRow.front_ready_us = pending.cycle.sensorSnapshot.frontTiming.observationReadyUs;
        _timingLogRow.left_led_on_us = pending.cycle.sensorSnapshot.leftTiming.ledOnCommandUs;
        _timingLogRow.left_adc_on_us = pending.cycle.sensorSnapshot.leftTiming.adcOnSampleUs;
        _timingLogRow.left_led_off_us = pending.cycle.sensorSnapshot.leftTiming.ledOffCommandUs;
        _timingLogRow.left_adc_off_us = pending.cycle.sensorSnapshot.leftTiming.adcOffSampleUs;
        _timingLogRow.left_ready_us = pending.cycle.sensorSnapshot.leftTiming.observationReadyUs;
        _timingLogRow.right_led_on_us = pending.cycle.sensorSnapshot.rightTiming.ledOnCommandUs;
        _timingLogRow.right_adc_on_us = pending.cycle.sensorSnapshot.rightTiming.adcOnSampleUs;
        _timingLogRow.right_led_off_us = pending.cycle.sensorSnapshot.rightTiming.ledOffCommandUs;
        _timingLogRow.right_adc_off_us = pending.cycle.sensorSnapshot.rightTiming.adcOffSampleUs;
        _timingLogRow.right_ready_us = pending.cycle.sensorSnapshot.rightTiming.observationReadyUs;
        _timingLogRow.wall_adc_cfg_before_start = pending.cycle.sensorSnapshot.wallSensorAdcCfgBeforeStart;
        _timingLogRow.wall_adc_gc_before_start = pending.cycle.sensorSnapshot.wallSensorAdcGcBeforeStart;
        _timingLogRow.wall_adc_cfg_after_start = pending.cycle.sensorSnapshot.wallSensorAdcCfgAfterStart;
        _timingLogRow.wall_adc_gc_after_start = pending.cycle.sensorSnapshot.wallSensorAdcGcAfterStart;
        _timingLogRow.wall_adc_target_cfg = pending.cycle.sensorSnapshot.wallSensorAdcTargetCfg;
        _timingLogRow.wall_adc_ipg_clock_hz = pending.cycle.sensorSnapshot.wallSensorAdcIpgClockHz;
        _timingLogRow.cycle_counter_start = pending.cycle.controlTiming.cycleCounterStart;
        _timingLogRow.cycle_counter_end = pending.cycle.controlTiming.cycleCounterEnd;

        if (_runtime.LogUtilityDataRow(_timingLogRow))
        {
            return true;
        }

        RecordTimingLogFailure();
        return false;
    };
    auto emitMainSample = [this, &pending]() -> bool
    {
        const bool encoderValid = pending.cycle.driveTelemetry.encoderObservationValid;
        const bool imuValid = std::isfinite(pending.cycle.sensorSnapshot.gyroRawRadps);
        const float maxRangeM = MazeMap::PlantParams::Default().noHitRangeM;
        MazeMap::WallObs frontLeftObs{};
        MazeMap::WallObs frontRightObs{};
        DriveBase::BuildLoggedFrontPairObservations(pending.cycle.sensorSnapshot, maxRangeM, frontLeftObs, frontRightObs);
        const MazeMap::WallObs leftObs = DriveBase::BuildLoggedLeftSideObservation(pending.cycle.sensorSnapshot, maxRangeM);
        const MazeMap::WallObs rightObs = DriveBase::BuildLoggedRightSideObservation(pending.cycle.sensorSnapshot, maxRangeM);
        const MazeMap::VehicleState::StateVector& estimatorState = _drive.GetEstimatorStateVector();

        _mainLogRow = {};
        _mainLogRow.master_time_us = pending.cycle.masterTimeUs;
        _mainLogRow.control_tick_sequence = pending.cycle.controlTickSequence;
        _mainLogRow.dt_us = pending.cycle.dtUs;
        _mainLogRow.section_id = static_cast<std::uint8_t>(pending.labels.sectionId);
        _mainLogRow.primitive_id = static_cast<std::uint8_t>(pending.labels.primitiveId);
        _mainLogRow.primitive_family = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveFamilyForId(pending.labels.primitiveId));
        _mainLogRow.direction_id = static_cast<std::uint8_t>(pending.labels.directionId);
        _mainLogRow.phase_id = static_cast<std::uint8_t>(pending.labels.phaseId);
        _mainLogRow.speed_bin = static_cast<std::uint8_t>(pending.labels.speedBin);
        _mainLogRow.start_marker_id = static_cast<std::uint8_t>(pending.labels.startMarkerId);
        _mainLogRow.repeat_index = pending.labels.repeatIndex;
        _mainLogRow.progress_norm = pending.labels.progressNorm;
        _mainLogRow.mode_flags = pending.cycle.driveTelemetry.modeFlags;
        _mainLogRow.clipping_flags = pending.cycle.clippingFlags;
        _mainLogRow.saturation_flags = pending.cycle.driveTelemetry.saturationFlags;
        _mainLogRow.watchdog_flags = pending.cycle.watchdogFlags;
        _mainLogRow.measurement_flags = MazeMap::App::Internal::Runtime::BuildOpenFloorMeasurementFlags(
            pending.labels,
            pending.cycle,
            encoderValid,
            imuValid,
            frontLeftObs,
            frontRightObs,
            leftObs,
            rightObs);
        _mainLogRow.ukf_mode_id = pending.cycle.driveTelemetry.ukfModeId;
        _mainLogRow.ukf_yaw_valid_for_feedforward = pending.cycle.driveTelemetry.ukfYawValidForFeedforward;
        _mainLogRow.bias_update_enabled = pending.cycle.driveTelemetry.ukfBiasUpdateEnabled;
        _mainLogRow.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        _mainLogRow.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        _mainLogRow.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        _mainLogRow.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        _mainLogRow.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        _mainLogRow.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        _mainLogRow.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        _mainLogRow.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        _mainLogRow.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        _mainLogRow.gyro_bias_anchor_radps = pending.cycle.driveTelemetry.ukfGyroBiasAnchorRadps;
        _mainLogRow.yaw_consistency_lp_radps = pending.cycle.driveTelemetry.ukfYawConsistencyLowPassRadps;
        _mainLogRow.yaw_window_mismatch_rad = pending.cycle.driveTelemetry.ukfYawWindowMismatchRad;
        _mainLogRow.nhc_sigma_mps = pending.cycle.driveTelemetry.ukfNhcSigmaMps;
        _mainLogRow.nhc_residual_mps = pending.cycle.driveTelemetry.ukfNhcResidualMps;
        _mainLogRow.nhc_residual_sigma = pending.cycle.driveTelemetry.ukfNhcResidualSigma;
        _mainLogRow.measured_linear_speed_mps = pending.cycle.measuredLinearSpeedMps;
        _mainLogRow.measured_angular_speed_radps = pending.cycle.measuredAngularSpeedRadps;
        _mainLogRow.cmd_linear_mps = _drive.GetLastLinearCommandMps();
        _mainLogRow.cmd_angular_radps = _drive.GetLastAngularCommandRadps();
        _mainLogRow.left_drive_command = pending.cycle.driveTelemetry.leftDriveCommand;
        _mainLogRow.right_drive_command = pending.cycle.driveTelemetry.rightDriveCommand;
        _mainLogRow.left_feedforward_command = pending.cycle.driveTelemetry.leftFeedforwardCommand;
        _mainLogRow.right_feedforward_command = pending.cycle.driveTelemetry.rightFeedforwardCommand;
        _mainLogRow.left_feedback_command = pending.cycle.driveTelemetry.leftFeedbackCommand;
        _mainLogRow.right_feedback_command = pending.cycle.driveTelemetry.rightFeedbackCommand;
        _mainLogRow.left_target_velocity_mps = pending.cycle.driveTelemetry.leftTargetVelocityMps;
        _mainLogRow.right_target_velocity_mps = pending.cycle.driveTelemetry.rightTargetVelocityMps;
        _mainLogRow.left_launch_assist_floor = pending.cycle.driveTelemetry.leftLaunchAssistFloor;
        _mainLogRow.right_launch_assist_floor = pending.cycle.driveTelemetry.rightLaunchAssistFloor;
        _mainLogRow.encoder_timestamp_us = pending.cycle.controlTiming.encoderReadDoneUs;
        _mainLogRow.left_encoder_count = pending.cycle.driveTelemetry.leftEncoderCount;
        _mainLogRow.right_encoder_count = pending.cycle.driveTelemetry.rightEncoderCount;
        _mainLogRow.left_encoder_omega_radps = pending.cycle.driveTelemetry.leftEncoderOmegaRadps;
        _mainLogRow.right_encoder_omega_radps = pending.cycle.driveTelemetry.rightEncoderOmegaRadps;
        _mainLogRow.left_encoder_distance_m = pending.cycle.driveTelemetry.leftDistanceM;
        _mainLogRow.right_encoder_distance_m = pending.cycle.driveTelemetry.rightDistanceM;
        _mainLogRow.left_encoder_velocity_mps = pending.cycle.driveTelemetry.leftVelocityMps;
        _mainLogRow.right_encoder_velocity_mps = pending.cycle.driveTelemetry.rightVelocityMps;
        _mainLogRow.imu_timestamp_us = pending.cycle.sensorSnapshot.imuTiming.readDoneUs;
        _mainLogRow.imu_status = pending.cycle.sensorSnapshot.imuBackLeft.status;
        _mainLogRow.imu_interrupt_high = pending.cycle.sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        _mainLogRow.accel_bias_valid = pending.cycle.sensorSnapshot.accelBiasValid ? 1U : 0U;
        _mainLogRow.imu_gyro_x = pending.cycle.sensorSnapshot.imuBackLeft.gyroX;
        _mainLogRow.imu_gyro_y = pending.cycle.sensorSnapshot.imuBackLeft.gyroY;
        _mainLogRow.imu_gyro_z = pending.cycle.sensorSnapshot.imuBackLeft.gyroZ;
        _mainLogRow.imu_accel_x = pending.cycle.sensorSnapshot.imuBackLeft.accelX;
        _mainLogRow.imu_accel_y = pending.cycle.sensorSnapshot.imuBackLeft.accelY;
        _mainLogRow.imu_accel_z = pending.cycle.sensorSnapshot.imuBackLeft.accelZ;
        _mainLogRow.imu_temp = pending.cycle.sensorSnapshot.imuBackLeft.temp;
        _mainLogRow.gyro_raw_radps = pending.cycle.sensorSnapshot.gyroRawRadps;
        _mainLogRow.gyro_bias_radps = pending.cycle.sensorSnapshot.gyroBiasRadps;
        _mainLogRow.gyro_radps = pending.cycle.sensorSnapshot.gyroRadps;
        _mainLogRow.accel_body_x_mps2 = pending.cycle.sensorSnapshot.accelBodyXMps2;
        _mainLogRow.accel_body_y_mps2 = pending.cycle.sensorSnapshot.accelBodyYMps2;
        _mainLogRow.planar_accel_mps2 = pending.cycle.planarAccelMps2;
        _mainLogRow.front_timestamp_us = pending.cycle.sensorSnapshot.frontTiming.observationReadyUs;
        _mainLogRow.left_timestamp_us = pending.cycle.sensorSnapshot.leftTiming.observationReadyUs;
        _mainLogRow.right_timestamp_us = pending.cycle.sensorSnapshot.rightTiming.observationReadyUs;
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
        _mainLogRow.fan_duty_cycle = pending.cycle.fanDutyCycle;

        if (_runtime.LogUtilityDataRow(_mainLogRow))
        {
            return true;
        }

        RecordMainLogFailure();
        return false;
    };

    switch (pending.kind)
    {
    case PendingLogKind::TimingSample:
        return !_timingLogOpen || emitTimingSample();

    case PendingLogKind::TimingFault:
        if (_timingLogOpen && !emitTimingSample())
        {
            return false;
        }
        return !_timingLogOpen || LogTimingFault(
            pending.cycle,
            pending.faultCode,
            pending.controlHalted,
            pending.extra0,
            pending.extra1);

    case PendingLogKind::MainSample:
        return !_mainLogOpen || emitMainSample();

    case PendingLogKind::MainFault:
        if (_mainLogOpen && !emitMainSample())
        {
            return false;
        }
        return !_mainLogOpen || LogMainFault(
            pending.labels,
            pending.cycle,
            pending.faultCode,
            pending.controlHalted,
            pending.extra0,
            pending.extra1);

    case PendingLogKind::None:
    default:
        return true;
    }
}

bool OpenFloorMeasurementController::EmitPendingLog(MazeMap::App::Internal::LoopController::TickServices& services)
{
    if (EmitPendingLog())
    {
        return true;
    }

    services.Fault("Open-floor pending log write failed");
    return false;
}

void OpenFloorMeasurementController::ApplyPublishedTiming(OpenFloorMeasurementCycle& cycle) const noexcept
{
    const LoopController::TimingDiagnostics& timing = _loopController.LastDiagnostics();
    cycle.controlTiming = timing.controlTiming;
}

void OpenFloorMeasurementController::StageTimingSample(const OpenFloorMeasurementCycle& cycle) noexcept
{
    _pendingLog = PendingLog{};
    _pendingLog.kind = PendingLogKind::TimingSample;
    _pendingLog.cycle = cycle;
}

void OpenFloorMeasurementController::StageTimingFault(
    const OpenFloorMeasurementCycle& cycle,
    const MazeMap::OpenFloorFaultCode faultCode,
    const bool controlHalted,
    const std::uint32_t extra0,
    const std::uint32_t extra1) noexcept
{
    _pendingLog = PendingLog{};
    _pendingLog.kind = PendingLogKind::TimingFault;
    _pendingLog.cycle = cycle;
    _pendingLog.faultCode = faultCode;
    _pendingLog.controlHalted = controlHalted;
    _pendingLog.extra0 = extra0;
    _pendingLog.extra1 = extra1;
}

void OpenFloorMeasurementController::StageMainSample(
    const OpenFloorMeasurementLabels& labels,
    const OpenFloorMeasurementCycle& cycle) noexcept
{
    _pendingLog = PendingLog{};
    _pendingLog.kind = PendingLogKind::MainSample;
    _pendingLog.labels = labels;
    _pendingLog.cycle = cycle;
}

void OpenFloorMeasurementController::StageMainFault(
    const OpenFloorMeasurementLabels& labels,
    const OpenFloorMeasurementCycle& cycle,
    const MazeMap::OpenFloorFaultCode faultCode,
    const bool controlHalted,
    const std::uint32_t extra0,
    const std::uint32_t extra1) noexcept
{
    _pendingLog = PendingLog{};
    _pendingLog.kind = PendingLogKind::MainFault;
    _pendingLog.labels = labels;
    _pendingLog.cycle = cycle;
    _pendingLog.faultCode = faultCode;
    _pendingLog.controlHalted = controlHalted;
    _pendingLog.extra0 = extra0;
    _pendingLog.extra1 = extra1;
}

bool OpenFloorMeasurementController::BeginTimingLog()
{
    _timingOverflowed = false;
    _timingWriteFailed = false;
    auto failTimingStep = [this](const char* step) { return FailLogSetupStep("Timing log", step); };
    auto failDataMetadata = [&failTimingStep](const char* key)
    {
        char step[96] = {};
        const int length = snprintf(
            step,
            sizeof(step),
            "data metadata %s",
            (key != nullptr && key[0] != '\0') ? key : "key");
        if (length <= 0 || length >= static_cast<int>(sizeof(step)))
        {
            return failTimingStep("data metadata");
        }
        return failTimingStep(step);
    };
    auto failTextMetadata = [&failTimingStep](const char* key)
    {
        char step[96] = {};
        const int length = snprintf(
            step,
            sizeof(step),
            "text metadata %s",
            (key != nullptr && key[0] != '\0') ? key : "key");
        if (length <= 0 || length >= static_cast<int>(sizeof(step)))
        {
            return failTimingStep("text metadata");
        }
        return failTimingStep(step);
    };

    if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorTimingFileName))
    {
        return failTimingStep("file open");
    }

    if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return failDataMetadata("mode");
    if (!_runtime.WriteUtilityDataLogMetadata("stream_type", "open_floor_timing")) return failDataMetadata("stream_type");
    if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return failDataMetadata("format_version");
    if (!_runtime.WriteUtilityDataLogMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return failDataMetadata("logging_format_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("primitive_schedule_revision", MazeMap::kOpenFloorPrimitiveScheduleRevision)) return failDataMetadata("primitive_schedule_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("phase_binning_revision", MazeMap::kOpenFloorPhaseBinningRevision)) return failDataMetadata("phase_binning_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("start_marker_definitions_revision", MazeMap::kOpenFloorStartMarkerDefinitionsRevision)) return failDataMetadata("start_marker_definitions_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", "pins_27_28_shorted_at_boot")) return failDataMetadata("boot_reason");
    if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return failDataMetadata("format_spec");
    if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return failDataMetadata("endianness");
    if (_runId[0] != '\0' && !_runtime.WriteUtilityDataLogMetadata("run_id", _runId)) return failDataMetadata("run_id");
    if (!_runtime.WriteUtilityDataLogMetadataFloat("battery_voltage_start", _batteryVoltageStart, 3)) return failDataMetadata("battery_voltage_start");
    if (!_runtime.WriteUtilityDataLogMetadataFloat("fan_duty_cycle_start", _fanDutyCycleStart, 3)) return failDataMetadata("fan_duty_cycle_start");
    if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return failDataMetadata("control_period_us");

    _timingLogRow = {};
    if (!_runtime.BeginUtilityDataLogSchema(_timingLogRow))
    {
        return failTimingStep("schema begin");
    }

    if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return failTextMetadata("file");
    if (!_runtime.WriteTextLogMetadata("data_file", MazeMap::kOpenFloorTimingFileName)) return failTextMetadata("data_file");
    if (!_runtime.WriteTextLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return failTextMetadata("mode");
    if (!_runtime.WriteTextLogMetadata("stream_type", "open_floor_timing_control_log")) return failTextMetadata("stream_type");
    if (!_runtime.WriteTextLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return failTextMetadata("format_version");
    if (!_runtime.WriteTextLogMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return failTextMetadata("logging_format_revision");
    if (!_runtime.WriteTextLogMetadata("primitive_schedule_revision", MazeMap::kOpenFloorPrimitiveScheduleRevision)) return failTextMetadata("primitive_schedule_revision");
    if (!_runtime.WriteTextLogMetadata("phase_binning_revision", MazeMap::kOpenFloorPhaseBinningRevision)) return failTextMetadata("phase_binning_revision");
    if (!_runtime.WriteTextLogMetadata("start_marker_definitions_revision", MazeMap::kOpenFloorStartMarkerDefinitionsRevision)) return failTextMetadata("start_marker_definitions_revision");
    if (!_runtime.WriteTextLogMetadata("boot_reason", "pins_27_28_shorted_at_boot")) return failTextMetadata("boot_reason");
    if (_runId[0] != '\0' && !_runtime.WriteTextLogMetadata("run_id", _runId)) return failTextMetadata("run_id");
    if (!_runtime.WriteTextLogMetadata("pins_27_28_shorted_at_boot", _pinsLatchedAtBoot ? "true" : "false")) return failTextMetadata("pins_27_28_shorted_at_boot");
    if (!_runtime.WriteTextLogEntry(micros(), "run_state", _pinsLatchedAtBoot ? "pins_27_28_shorted_at_boot=true" : "pins_27_28_shorted_at_boot=false")) return failTimingStep("text run_state entry");
    return true;
}

bool OpenFloorMeasurementController::LogTimingFault(
    const OpenFloorMeasurementCycle& cycle,
    MazeMap::OpenFloorFaultCode faultCode,
    bool controlHalted,
    uint32_t extra0,
    uint32_t extra1)
{
    char message[256] = {};
    const int length = snprintf(
        message,
        sizeof(message),
        "fault=%s;section_id=%s;control_halted=%u;tick=%lu;dt_us=%lu;extra0=%lu;extra1=%lu",
        MazeMap::OpenFloorFaultName(faultCode),
        MazeMap::OpenFloorSectionName(MazeMap::OpenFloorSectionId::Sec00Timing),
        controlHalted ? 1U : 0U,
        static_cast<unsigned long>(cycle.controlTickSequence),
        static_cast<unsigned long>(cycle.dtUs),
        static_cast<unsigned long>(extra0),
        static_cast<unsigned long>(extra1));
    if (length <= 0)
    {
        return false;
    }
    message[sizeof(message) - 1U] = '\0';
    return _runtime.WriteTextLogEntry(micros(), "fault", message);
}

void OpenFloorMeasurementController::ServiceLogs()
{
    if (!_runtime.ServiceUtilityDataLog())
    {
        if (_timingLogOpen)
        {
            RecordTimingLogFailure();
        }
        if (_mainLogOpen)
        {
            RecordMainLogFailure();
        }
    }
}

void OpenFloorMeasurementController::CloseTimingLog()
{
    if (!_runtime.CloseUtilityDataLog())
    {
        RecordTimingLogFailure();
    }
}

void OpenFloorMeasurementController::RecordTimingLogFailure() noexcept
{
    _runtime.CaptureUtilityDataLogFailure(_timingOverflowed, _timingWriteFailed);
}

bool OpenFloorMeasurementController::BeginMainLog()
{
    _mainOverflowed = false;
    _mainWriteFailed = false;
    auto failMainStep = [this](const char* step) { return FailLogSetupStep("Main log", step); };
    auto failDataMetadata = [&failMainStep](const char* key)
    {
        char step[96] = {};
        const int length = snprintf(
            step,
            sizeof(step),
            "data metadata %s",
            (key != nullptr && key[0] != '\0') ? key : "key");
        if (length <= 0 || length >= static_cast<int>(sizeof(step)))
        {
            return failMainStep("data metadata");
        }
        return failMainStep(step);
    };
    auto failTextMetadata = [&failMainStep](const char* key)
    {
        char step[96] = {};
        const int length = snprintf(
            step,
            sizeof(step),
            "text metadata %s",
            (key != nullptr && key[0] != '\0') ? key : "key");
        if (length <= 0 || length >= static_cast<int>(sizeof(step)))
        {
            return failMainStep("text metadata");
        }
        return failMainStep(step);
    };

    if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorMainFileName))
    {
        return failMainStep("file open");
    }

    if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return failDataMetadata("mode");
    if (!_runtime.WriteUtilityDataLogMetadata("stream_type", "open_floor_main")) return failDataMetadata("stream_type");
    if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return failDataMetadata("format_version");
    if (!_runtime.WriteUtilityDataLogMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return failDataMetadata("logging_format_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("primitive_schedule_revision", MazeMap::kOpenFloorPrimitiveScheduleRevision)) return failDataMetadata("primitive_schedule_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("phase_binning_revision", MazeMap::kOpenFloorPhaseBinningRevision)) return failDataMetadata("phase_binning_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("active_imu_id", MazeMap::kOpenFloorActiveImuId)) return failDataMetadata("active_imu_id");
    if (!_runtime.WriteUtilityDataLogMetadata("imu_extrinsics_revision", MazeMap::kOpenFloorImuExtrinsicsRevision)) return failDataMetadata("imu_extrinsics_revision");
    if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return failDataMetadata("format_spec");
    if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return failDataMetadata("endianness");
    if (_runId[0] != '\0' && !_runtime.WriteUtilityDataLogMetadata("run_id", _runId)) return failDataMetadata("run_id");
    if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return failDataMetadata("control_period_us");
    if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return failDataMetadata("imu_gyro_mdps_per_lsb");
    if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return failDataMetadata("imu_accel_mg_per_lsb");

    _mainLogRow = {};
    if (!_runtime.BeginUtilityDataLogSchema(_mainLogRow))
    {
        return failMainStep("schema begin");
    }

    if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return failTextMetadata("file");
    if (!_runtime.WriteTextLogMetadata("data_file", MazeMap::kOpenFloorMainFileName)) return failTextMetadata("data_file");
    if (!_runtime.WriteTextLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return failTextMetadata("mode");
    if (!_runtime.WriteTextLogMetadata("stream_type", "open_floor_main_control_log")) return failTextMetadata("stream_type");
    if (!_runtime.WriteTextLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return failTextMetadata("format_version");
    if (!_runtime.WriteTextLogMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return failTextMetadata("logging_format_revision");
    if (!_runtime.WriteTextLogMetadata("primitive_schedule_revision", MazeMap::kOpenFloorPrimitiveScheduleRevision)) return failTextMetadata("primitive_schedule_revision");
    if (!_runtime.WriteTextLogMetadata("phase_binning_revision", MazeMap::kOpenFloorPhaseBinningRevision)) return failTextMetadata("phase_binning_revision");
    if (_runId[0] != '\0' && !_runtime.WriteTextLogMetadata("run_id", _runId)) return failTextMetadata("run_id");
    return true;
}

bool OpenFloorMeasurementController::LogMainFault(
    const OpenFloorMeasurementLabels& labels,
    const OpenFloorMeasurementCycle& cycle,
    MazeMap::OpenFloorFaultCode faultCode,
    bool controlHalted,
    uint32_t extra0,
    uint32_t extra1)
{
    char message[384] = {};
    const int length = snprintf(
        message,
        sizeof(message),
        "fault=%s;section_id=%s;primitive_id=%s;direction=%s;phase_id=%s;speed_bin=%s;start_marker=%s;repeat_index=%u;mirrored=%u;control_halted=%u;extra0=%lu;extra1=%lu",
        MazeMap::OpenFloorFaultName(faultCode),
        MazeMap::OpenFloorSectionName(labels.sectionId),
        MazeMap::OpenFloorPrimitiveName(labels.primitiveId),
        MazeMap::OpenFloorDirectionName(labels.directionId),
        MazeMap::OpenFloorPhaseName(labels.phaseId),
        MazeMap::OpenFloorSpeedBinName(labels.speedBin),
        MazeMap::OpenFloorMarkerName(labels.startMarkerId),
        static_cast<unsigned>(labels.repeatIndex),
        MazeMap::OpenFloorPrimitiveIsMirrored(labels.primitiveId) ? 1U : 0U,
        controlHalted ? 1U : 0U,
        static_cast<unsigned long>(extra0),
        static_cast<unsigned long>(extra1));
    if (length <= 0)
    {
        return false;
    }
    message[sizeof(message) - 1U] = '\0';
    return _runtime.WriteTextLogEntry(micros(), "fault", message);
}

bool OpenFloorMeasurementController::WriteMainSectionMarker(
    const char* type,
    const OpenFloorMeasurementLabels& labels,
    const char* reason)
{
    char message[256] = {};
    const int length = snprintf(
        message,
        sizeof(message),
        "section_id=%s;primitive_id=%s;direction=%s;start_marker=%s;repeat_index=%u;speed_bin=%s%s%s",
        MazeMap::OpenFloorSectionName(labels.sectionId),
        MazeMap::OpenFloorPrimitiveName(labels.primitiveId),
        MazeMap::OpenFloorDirectionName(labels.directionId),
        MazeMap::OpenFloorMarkerName(labels.startMarkerId),
        static_cast<unsigned>(labels.repeatIndex),
        MazeMap::OpenFloorSpeedBinName(labels.speedBin),
        (reason != nullptr && reason[0] != '\0') ? ";reason=" : "",
        (reason != nullptr && reason[0] != '\0') ? reason : "");
    if (length <= 0 || length >= static_cast<int>(sizeof(message)))
    {
        return false;
    }
    return _runtime.WriteTextLogEntry(micros(), type, message);
}

bool OpenFloorMeasurementController::BeginMainSection(const OpenFloorMeasurementLabels& labels)
{
    return WriteMainSectionMarker("section_start", labels, nullptr);
}

bool OpenFloorMeasurementController::EndMainSection(const OpenFloorMeasurementLabels& labels)
{
    return WriteMainSectionMarker("section_end", labels, nullptr);
}

bool OpenFloorMeasurementController::AbortMainSection(const OpenFloorMeasurementLabels& labels, const char* reason)
{
    return WriteMainSectionMarker("abort", labels, reason);
}

bool OpenFloorMeasurementController::WriteMainEvent(const char* type, const char* message)
{
    return _runtime.WriteTextLogEntry(micros(), type, message);
}

bool OpenFloorMeasurementController::WriteFaultDumpTextEntry(const char* type, const char* message)
{
    return _runtime.WriteTextLogEntry(micros(), type, message);
}

bool OpenFloorMeasurementController::DumpUkfFaultToTextLog(const char* reason)
{
    char header[224] = {};
    const int length = snprintf(
        header,
        sizeof(header),
        "reason=%s;estimator_fault=%u;estimator_fault_reason=%s",
        (reason != nullptr && reason[0] != '\0') ? reason : "unknown",
        _drive.HasEstimatorFault() ? 1U : 0U,
        _drive.GetEstimatorFaultReason());
    if (length <= 0 || length >= static_cast<int>(sizeof(header)))
    {
        return false;
    }

    if (!WriteFaultDumpTextEntry("ukf_dump_begin", header))
    {
        return false;
    }

    if (!_drive.WriteUkfDebugTextDump(
            [this](const char* type, const char* message) noexcept -> bool
            {
                return WriteFaultDumpTextEntry(type, message);
            }))
    {
        return false;
    }

    return WriteFaultDumpTextEntry("ukf_dump_end", "complete=true");
}

void OpenFloorMeasurementController::CloseMainLog()
{
    if (!_runtime.CloseUtilityDataLog())
    {
        RecordMainLogFailure();
    }
    _runtime.CloseTextLog();
}

void OpenFloorMeasurementController::RecordMainLogFailure() noexcept
{
    _runtime.CaptureUtilityDataLogFailure(_mainOverflowed, _mainWriteFailed);
}

bool OpenFloorMeasurementController::Begin()
{
    _faulted = false;
    _timingLogOpen = false;
    _mainLogOpen = false;
    _phaseFn = nullptr;
    _pauseAction = PauseAction::None;
    ResetPendingLog();
    _timingBlockState = TimingBlockState{};
    _staticHoldState = StaticHoldState{};
    _recoveryState = RecoveryState{};
    _launchPulseState = LaunchPulseState{};
    _straightSectionState = StraightSectionState{};
    _sectionSettleState = SectionSettleState{};
    _turnSectionState = TurnSectionState{};
    _smoothTurnState = SmoothTurnState{};
    _launchSequenceState = LaunchSequenceState{};
    _straightSequenceState = StraightSequenceState{};
    _yawSequenceState = YawSequenceState{};
    _smoothSequenceState = SmoothSequenceState{};
    _loopSequenceState = LoopSequenceState{};
    if (!_runtime.RegisterModeFaultHandler(&OpenFloorMeasurementController::HandleRuntimeFault, this, "open_floor_measurement"))
    {
        return false;
    }
    if (!SetupHardware())
    {
        return Fail("Hardware setup failed");
    }
    ResetStartupTrace("mode:open_floor_measurement");
    if (!_drive.Begin())
    {
        return Fail("Drive base init failed");
    }
    _drive.UseNominalWheelControlProfile();
    SetMissionLevelFanEnabled(true);
    gWallDistanceCalibration.Clear();
    if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
    {
        return Fail("Measurement sensor init failed");
    }

    snprintf(_runId, sizeof(_runId), "ofm_%lu", static_cast<unsigned long>(micros()));
    _controlTickSequence = 0UL;
    _pinsLatchedAtBoot = MazeMap::App::IsBootModeSelectorActive(MazeMap::App::BootModeId::PrimaryDiagnostic);
    ConfigurePrimaryDiagnosticSelectorMonitor();
    _batteryVoltageStart = ReadBatteryVoltage();
    _fanDutyCycleStart = GetMissionFanDutyCycle();
    char runStartMessage[256] = {};
    const int runStartLength = snprintf(
        runStartMessage,
        sizeof(runStartMessage),
        "run_id=%s;pins_27_28_shorted_at_boot=%u;battery_voltage_start=%.3f;fan_duty_cycle_start=%.3f",
        _runId,
        _pinsLatchedAtBoot ? 1U : 0U,
        _batteryVoltageStart,
        _fanDutyCycleStart);
    if (runStartLength <= 0 ||
        runStartLength >= static_cast<int>(sizeof(runStartMessage)) ||
        !_runtime.WriteTextLogEntry("open_floor_measurement", micros(), "run_start", runStartMessage))
    {
        return Fail("Open-floor run metadata logging failed");
    }

    SeedPoseAtMarker(MazeMap::OpenFloorMarkerId::C);
    return true;
}

void OpenFloorMeasurementController::Run()
{
    if (_faulted)
    {
        return;
    }

    bool ok = StartTimingBlockPhase();
    if (ok)
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &OpenFloorMeasurementController::ModeWorkThunk;
        callbacks.context = this;
        ResetPendingLog();
        _pauseAction = PauseAction::None;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            _phaseFn = nullptr;
            ok = Fail("Open-floor loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            const bool emitOk = EmitPendingLog();
            _phaseFn = nullptr;
            _pauseAction = PauseAction::None;
            ResetPendingLog();
            if (!emitOk)
            {
                ok = Fail("Open-floor pending log emission failed");
            }
            else
            {
                ok = (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
            }
        }
    }

    _drive.Brake();
    _drive.UseNominalWheelControlProfile();
    if (_timingLogOpen)
    {
        CloseTimingLog();
        _timingLogOpen = false;
    }
    if (_mainLogOpen)
    {
        CloseMainLog();
        _mainLogOpen = false;
    }
    ReleasePrimaryDiagnosticSelectorMonitor();
    if (ok)
    {
        AppendStartupTrace("open_floor_measurement:complete");
    }
    SetMissionLevelFanEnabled(false);
}

MotionLimits OpenFloorMeasurementController::MeasurementLimits(float maxSpeedMps)
{
    MotionLimits limits{};
    limits.maxSpeedMps = maxSpeedMps;
    limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
    limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
    limits.maxAngularSpeedRadps = DiagnosticConfig::kTurnMaxOmegaRadps;
    limits.angularAccelRadps2 = DiagnosticConfig::kTurnAccelRadps2;
    return limits;
}

uint32_t OpenFloorMeasurementController::ReadCycleCounter()
{
#if defined(ARDUINO_TEENSY41)
    return ARM_DWT_CYCCNT;
#else
    return 0UL;
#endif
}

MazeMap::OpenFloorPrimitiveId OpenFloorMeasurementController::PrimitiveIdForSmoothCode(MazeMap::ManeuverCode code)
{
    switch (code)
    {
    case MazeMap::S45SS:
        return MazeMap::OpenFloorPrimitiveId::S45ss;
    case MazeMap::S45SS_M:
        return MazeMap::OpenFloorPrimitiveId::S45ssM;
    case MazeMap::S90SS:
        return MazeMap::OpenFloorPrimitiveId::S90ss;
    case MazeMap::S90SS_M:
        return MazeMap::OpenFloorPrimitiveId::S90ssM;
    case MazeMap::S135SS:
        return MazeMap::OpenFloorPrimitiveId::S135ss;
    case MazeMap::S135SS_M:
        return MazeMap::OpenFloorPrimitiveId::S135ssM;
    default:
        return MazeMap::OpenFloorPrimitiveId::None;
    }
}

unsigned long OpenFloorMeasurementController::FailureTimeoutMs(unsigned long requestedTimeoutMs)
{
    return (std::max)(requestedTimeoutMs, kMinimumFailureTimeoutMs);
}

MazeMap::OpenFloorPhaseId OpenFloorMeasurementController::StraightPhaseForProgress(float progress)
{
    if (progress < 0.25f)
    {
        return MazeMap::OpenFloorPhaseId::Accel;
    }
    if (progress > 0.80f)
    {
        return MazeMap::OpenFloorPhaseId::Brake;
    }
    return MazeMap::OpenFloorPhaseId::Cruise;
}

MazeMap::OpenFloorPhaseId OpenFloorMeasurementController::TurnPhaseForProgress(float progress)
{
    if (progress < 0.20f)
    {
        return MazeMap::OpenFloorPhaseId::Startup;
    }
    if (progress > 0.85f)
    {
        return MazeMap::OpenFloorPhaseId::Stop;
    }
    return MazeMap::OpenFloorPhaseId::SteadyRotation;
}

bool OpenFloorMeasurementController::Fail(const char* message)
{
    return _runtime.FailActiveMode(message);
}

bool OpenFloorMeasurementController::FailLogSetupStep(const char* logName, const char* step)
{
    char message[256] = {};
    const char* const detail = _runtime.LastRuntimeLogError();
    const bool hasDetail = (detail != nullptr) && (detail[0] != '\0');
    const int length = snprintf(
        message,
        sizeof(message),
        "%s %s failed%s%s",
        (logName != nullptr && logName[0] != '\0') ? logName : "Log",
        (step != nullptr && step[0] != '\0') ? step : "step",
        hasDetail ? ": " : "",
        hasDetail ? detail : "");
    if (length <= 0 || length >= static_cast<int>(sizeof(message)))
    {
        char fallback[96] = {};
        const int fallbackLength = snprintf(
            fallback,
            sizeof(fallback),
            "%s setup failed",
            (logName != nullptr && logName[0] != '\0') ? logName : "Log");
        if (fallbackLength > 0 && fallbackLength < static_cast<int>(sizeof(fallback)))
        {
            return Fail(fallback);
        }
        return Fail("Log setup failed");
    }
    return Fail(message);
}

void OpenFloorMeasurementController::HandleRuntimeFault(void* context, const char* reason) noexcept
{
    if (context == nullptr)
    {
        return;
    }

    static_cast<OpenFloorMeasurementController*>(context)->OnRuntimeFault(reason);
}

void OpenFloorMeasurementController::OnRuntimeFault(const char* message) noexcept
{
    _faulted = true;
    ReleasePrimaryDiagnosticSelectorMonitor();
    if (message != nullptr && message[0] != '\0')
    {
        AppendStartupTrace(message);
    }

    if (!DumpUkfFaultToTextLog(message))
    {
        AppendStartupTrace("open_floor_ukf_dump_failed");
    }
}

void OpenFloorMeasurementController::SeedPoseAtMarker(MazeMap::OpenFloorMarkerId markerId)
{
    _drive.SetPose(
        MazeMap::OpenFloorMarkerXMeters(markerId),
        MazeMap::OpenFloorMarkerYMeters(markerId),
        DirectionToYawRad(MazeMap::GetOpenFloorMarker(markerId).heading));
}

void OpenFloorMeasurementController::ConfigurePrimaryDiagnosticSelectorMonitor() noexcept
{
    ReleasePrimaryDiagnosticSelectorMonitor();
    if (!_pinsLatchedAtBoot)
    {
        return;
    }

    const MazeMap::App::BootModeRegistryEntry* const entry =
        MazeMap::App::FindBootModeRegistryEntry(MazeMap::App::BootModeId::PrimaryDiagnostic);
    if (entry == nullptr || entry->selector.kind != MazeMap::App::BootModeSelectorKind::PinPair)
    {
        return;
    }

    _primaryDiagnosticSelectorDrivePin = entry->selector.pinA;
    _primaryDiagnosticSelectorSensePin = entry->selector.pinB;
    BeginPinPairStrapMonitor(_primaryDiagnosticSelectorDrivePin, _primaryDiagnosticSelectorSensePin);
    _primaryDiagnosticSelectorMonitorArmed = true;
}

void OpenFloorMeasurementController::ReleasePrimaryDiagnosticSelectorMonitor() noexcept
{
    if (_primaryDiagnosticSelectorMonitorArmed)
    {
        EndPinPairStrapMonitor(_primaryDiagnosticSelectorDrivePin, _primaryDiagnosticSelectorSensePin);
    }

    _primaryDiagnosticSelectorInactiveStartMs = 0UL;
    _primaryDiagnosticSelectorDrivePin = 0U;
    _primaryDiagnosticSelectorSensePin = 0U;
    _primaryDiagnosticSelectorMonitorArmed = false;
    _primaryDiagnosticSelectorInactive = false;
}

bool OpenFloorMeasurementController::HasPrimaryDiagnosticSelectorRemovalFault() noexcept
{
    if (!_primaryDiagnosticSelectorMonitorArmed)
    {
        return false;
    }

    if (IsPinPairStrapMonitorClosed(_primaryDiagnosticSelectorSensePin))
    {
        _primaryDiagnosticSelectorInactiveStartMs = 0UL;
        _primaryDiagnosticSelectorInactive = false;
        return false;
    }

    const unsigned long nowMs = millis();
    if (!_primaryDiagnosticSelectorInactive)
    {
        _primaryDiagnosticSelectorInactiveStartMs = nowMs;
        _primaryDiagnosticSelectorInactive = true;
        return false;
    }

    return MazeMap::HasOpenFloorSelectorRemovalFaultDelayElapsed(
        _primaryDiagnosticSelectorInactiveStartMs,
        nowMs);
}

float OpenFloorMeasurementController::ReadBatteryVoltage() const
{
    return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
}

float OpenFloorMeasurementController::ReadBoardTemperatureC(const SensorSnapshot& snapshot) const
{
    return 25.0f + (static_cast<float>(snapshot.imuBackLeft.temp) / 256.0f);
}

void OpenFloorMeasurementController::PopulateCycleFromState(
    const MazeMap::App::Internal::LoopController::ModeState& state,
    OpenFloorMeasurementCycle& cycle)
{
    cycle.masterTimeUs = state.tickStartUs;
    cycle.controlTickSequence = ++_controlTickSequence;
    cycle.dtUs = state.dtUs;
    cycle.driveTelemetry = state.driveTelemetry;
    cycle.sensorSnapshot = state.sensors;
    cycle.measuredLinearSpeedMps = state.measured.linearSpeedMps;
    cycle.measuredAngularSpeedRadps = state.measured.angularSpeedRadps;
    cycle.planarAccelMps2 = cycle.sensorSnapshot.planarAccelMps2;
    cycle.batteryVoltage = ReadBatteryVoltage();
    cycle.boardTemperatureC = ReadBoardTemperatureC(cycle.sensorSnapshot);
    cycle.fanDutyCycle = GetMissionFanDutyCycle();
    cycle.selectorJumperRemoved = HasPrimaryDiagnosticSelectorRemovalFault();
    cycle.estimatorFault = !state.estimatorHealthy;
}

bool OpenFloorMeasurementController::QueueMeasurementCaptureFault(
    OpenFloorMeasurementLabels& labels,
    OpenFloorMeasurementCycle& cycle)
{
    labels.abortMarker = true;
    if (cycle.estimatorFault)
    {
        StageMainFault(
            labels,
            cycle,
            MazeMap::OpenFloorFaultCode::EstimatorFault,
            true);
        return true;
    }
    if (cycle.selectorJumperRemoved)
    {
        StageMainFault(
            labels,
            cycle,
            MazeMap::OpenFloorFaultCode::SelectorJumperRemoved,
            true,
            static_cast<std::uint32_t>(MazeMap::kOpenFloorSelectorRemovalFaultDelayMs));
        return true;
    }
    return false;
}

MazeMap::App::Internal::LoopController::PauseDisposition OpenFloorMeasurementController::OnPauseGranted(
    const MazeMap::App::Internal::LoopController::PauseContext& pause)
{
    (void)pause;

    switch (_pauseAction)
    {
    case PauseAction::TimingToMain:
        if (!TransitionFromTimingToMain())
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor timing-to-main log transition failed");
        }
        _pauseAction = PauseAction::None;
        if (!StartStaticSectionPhase())
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor static section start failed");
        }
        return LoopController::PauseDisposition::Resume();

    case PauseAction::None:
    default:
        return LoopController::PauseDisposition::StopByRuntime(
            "Open-floor pause granted without a pending pause action");
    }
}

bool OpenFloorMeasurementController::TransitionFromTimingToMain()
{
    if (_timingLogOpen)
    {
        CloseTimingLog();
        _timingLogOpen = false;
    }

    if (_mainLogOpen)
    {
        return true;
    }

    if (!BeginMainLog())
    {
        return false;
    }

    _mainLogOpen = true;
    return true;
}

bool OpenFloorMeasurementController::StartTimingBlockPhase()
{
    if (!_timingLogOpen)
    {
        if (!BeginTimingLog())
        {
            return false;
        }
        _timingLogOpen = true;
    }

    _timingBlockState = TimingBlockState{};
    _phaseFn = &OpenFloorMeasurementController::RunTimingBlockTick;
    return true;
}

bool OpenFloorMeasurementController::StartTimingToMainPausePhase()
{
    _pauseAction = PauseAction::TimingToMain;
    _phaseFn = &OpenFloorMeasurementController::TimingToMainPauseTick;
    return true;
}

bool OpenFloorMeasurementController::StartStaticSectionPhase()
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = MazeMap::OpenFloorSectionId::Sec10Static;
    labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    labels.primitiveId = MazeMap::OpenFloorPrimitiveId::StaticHold;
    labels.phaseId = MazeMap::OpenFloorPhaseId::Hold;
    labels.repeatIndex = 1U;

    _staticHoldState = StaticHoldState{};
    _staticHoldState.labels = labels;
    return StartRecoveryPhase(
        labels,
        labels.startMarkerId,
        DiagnosticConfig::kCharacterizationRecoverySpeedMps,
        2500UL,
        true);
}

bool OpenFloorMeasurementController::StartRecoveryPhase(
    const OpenFloorMeasurementLabels& labels,
    const MazeMap::OpenFloorMarkerId markerId,
    const float maxSpeedMps,
    const unsigned long timeoutMs,
    const bool emitSectionMarkers)
{
    OpenFloorMeasurementLabels recoveryLabels = labels;
    recoveryLabels.startMarkerId = markerId;
    recoveryLabels.primitiveId = MazeMap::OpenFloorPrimitiveId::Recovery;
    recoveryLabels.directionId = MazeMap::OpenFloorDirectionId::None;
    recoveryLabels.phaseId = MazeMap::OpenFloorPhaseId::Recovery;
    if (emitSectionMarkers && _mainLogOpen && !BeginMainSection(recoveryLabels))
    {
        return false;
    }

    const MazeMap::OpenFloorMarkerPose& marker = MazeMap::GetOpenFloorMarker(markerId);
    const float targetX = MazeMap::OpenFloorMarkerXMeters(markerId);
    const float targetY = MazeMap::OpenFloorMarkerYMeters(markerId);
    const PoseEstimate startPose = _drive.GetPose();

    _recoveryState = RecoveryState{};
    _recoveryState.labels = recoveryLabels;
    _recoveryState.targetX = targetX;
    _recoveryState.targetY = targetY;
    _recoveryState.targetHeading = DirectionToUnitVector(marker.heading);
    _recoveryState.limits = MeasurementLimits(maxSpeedMps);
    _recoveryState.deadlineMs = millis() + FailureTimeoutMs(timeoutMs);
    _recoveryState.initialDistanceOutsideZoneM = MazeMap::OpenFloorRecoveryDistanceOutsideAcceptanceZoneM(
        targetX - startPose.xMeters,
        targetY - startPose.yMeters);
    _phaseFn = &OpenFloorMeasurementController::RecoverToMarkerTick;
    return true;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::RunTimingBlockTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (cycle.estimatorFault)
    {
        StageTimingFault(cycle, MazeMap::OpenFloorFaultCode::EstimatorFault, true);
        services.Fault("Estimator fault during timing capture");
        return LoopController::ControlVector::Brake;
    }
    if (cycle.selectorJumperRemoved)
    {
        StageTimingFault(
            cycle,
            MazeMap::OpenFloorFaultCode::SelectorJumperRemoved,
            true,
            static_cast<std::uint32_t>(MazeMap::kOpenFloorSelectorRemovalFaultDelayMs));
        services.Fault("Primary diagnostic selector jumper removed during timing capture");
        return LoopController::ControlVector::Brake;
    }

    StageTimingSample(cycle);
    ++_timingBlockState.tickIndex;
    if (_timingBlockState.tickIndex >= DiagnosticConfig::kTimingCaptureCycles)
    {
        if (!StartTimingToMainPausePhase())
        {
            services.Fault("Failed to stage open-floor timing pause transition");
        }
    }

    return LoopController::ControlVector::Brake;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::TimingToMainPauseTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    (void)state;

    LoopController::PauseRequest request{};
    request.onPauseGranted = &OpenFloorMeasurementController::PauseThunk;
    request.reason = "open_floor_timing_to_main";
    request.flushLogsBeforeGrant = true;
    request.resetClockOnResume = true;
    services.RequestPause(request);
    return LoopController::ControlVector::Brake;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::RunStaticSectionTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_staticHoldState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    StageMainSample(_staticHoldState.labels, cycle);
    if (static_cast<long>(_staticHoldState.deadlineMs - millis()) <= 0)
    {
        if (!EndMainSection(_staticHoldState.labels))
        {
            services.Fault("Failed to write static section end marker");
            return LoopController::ControlVector::Brake;
        }

        _launchSequenceState = LaunchSequenceState{};
        if (!AdvanceLaunchSequence())
        {
            services.Fault("Failed to advance open-floor launch sequence");
        }
    }

    return LoopController::ControlVector::Brake;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::RecoverToMarkerTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_recoveryState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    const PoseEstimate& pose = state.estimate;
    const float dx = _recoveryState.targetX - pose.xMeters;
    const float dy = _recoveryState.targetY - pose.yMeters;
    const Eigen::Vector2f travelHeading = pose.headingUnit;
    const float recoveryLongitudinalErrorM =
        MazeMap::OpenFloorRecoverySignedLongitudinalDistanceToAcceptanceZoneM(travelHeading, dx, dy);
    const float recoveryLateralErrorM =
        MazeMap::OpenFloorRecoverySignedLateralMissToAcceptanceZoneM(travelHeading, dx, dy);
    const float targetHeadingErrorRad = HeadingErrorRad(_recoveryState.targetHeading, pose.headingUnit);
    const bool positionArrived = MazeMap::OpenFloorRecoveryWithinAcceptanceRadius(dx, dy);
    const float recoveryDistanceOutsideZoneM =
        MazeMap::OpenFloorRecoveryDistanceOutsideAcceptanceZoneM(dx, dy);
    _recoveryState.labels.phaseId = MazeMap::OpenFloorPhaseId::Recovery;
    _recoveryState.labels.progressNorm = (_recoveryState.initialDistanceOutsideZoneM > 0.0f) ?
        (std::clamp)(
            1.0f - (recoveryDistanceOutsideZoneM / _recoveryState.initialDistanceOutsideZoneM),
            0.0f,
            1.0f) :
        1.0f;

    if (positionArrived &&
        std::fabs(targetHeadingErrorRad) <= MazeMap::kOpenFloorRecoveryArrivalHeadingToleranceRad &&
        std::fabs(pose.linearSpeedMps) <= Config::kSpeedToleranceMps &&
        std::fabs(pose.angularSpeedRadps) <= Config::kAngularSpeedToleranceRadps)
    {
        StageMainSample(_recoveryState.labels, cycle);
        if (_mainLogOpen && !EndMainSection(_recoveryState.labels))
        {
            services.Fault("Failed to write recovery section end marker");
            return LoopController::ControlVector::Brake;
        }

        switch (_recoveryState.labels.sectionId)
        {
        case MazeMap::OpenFloorSectionId::Sec10Static:
            if (!BeginMainSection(_staticHoldState.labels))
            {
                services.Fault("Failed to write static section start marker");
                return LoopController::ControlVector::Brake;
            }
            _staticHoldState.deadlineMs = millis() + DiagnosticConfig::kStaticHoldMs;
            _phaseFn = &OpenFloorMeasurementController::RunStaticSectionTick;
            break;

        case MazeMap::OpenFloorSectionId::Sec60LoopCw:
        case MazeMap::OpenFloorSectionId::Sec70LoopCcw:
            if (!AdvanceLoopAfterRecovery())
            {
                services.Fault("Failed to continue open-floor loop section after recovery");
                return LoopController::ControlVector::Brake;
            }
            break;

        default:
            services.Fault("Open-floor recovery completed without a valid continuation");
            return LoopController::ControlVector::Brake;
        }

        return LoopController::ControlVector::Brake;
    }

    if (static_cast<long>(_recoveryState.deadlineMs - millis()) <= 0)
    {
        _recoveryState.labels.abortMarker = true;
        cycle.watchdogFlags |= kWatchdogFlagRecoveryTimeout;
        StageMainFault(
            _recoveryState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::RecoveryTimedOut,
            true);
        services.Fault("Recovery to marker timed out");
        return LoopController::ControlVector::Brake;
    }

    const float linearCommandMps = positionArrived ?
        0.0f :
        (std::clamp)(4.0f * recoveryLongitudinalErrorM, -_recoveryState.limits.maxSpeedMps, _recoveryState.limits.maxSpeedMps);
    float angularCommandRadps = positionArrived ?
        ((Config::kStraightHeadingKp * targetHeadingErrorRad) - (Config::kStraightYawD * pose.angularSpeedRadps)) :
        (-(3.0f * recoveryLateralErrorM) - (Config::kStraightYawD * pose.angularSpeedRadps));
    angularCommandRadps = (std::clamp)(
        angularCommandRadps,
        -_recoveryState.limits.maxAngularSpeedRadps,
        _recoveryState.limits.maxAngularSpeedRadps);

    StageMainSample(_recoveryState.labels, cycle);
    return _drive.PointControlVector(
        linearCommandMps,
        angularCommandRadps,
        MazeMap::CommandPD::StateWheelOmegaPD);
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ExecuteLaunchPulseTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_launchPulseState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    _launchPulseState.stationaryCheckState.SetStateVector(_drive.GetEstimatorStateVector());
    const bool estimatorStationary = _launchPulseState.stationaryCheckState.IsStationary();

    if (_launchPulseState.phase == LaunchPulseState::Phase::LaunchPulse)
    {
        _launchPulseState.labels.phaseId = MazeMap::OpenFloorPhaseId::LaunchPulse;
        if (_launchPulseState.previousStationary && !estimatorStationary)
        {
            _launchPulseState.launchFlippedStationary = true;
        }
        _launchPulseState.previousStationary = estimatorStationary;
        StageMainSample(_launchPulseState.labels, cycle);
        if (static_cast<long>(_launchPulseState.pulseDeadlineMs - millis()) <= 0)
        {
            if (_launchPulseState.launchFlippedStationary)
            {
                _launchPulseState.phase = LaunchPulseState::Phase::BrakeToStop;
            }
            else
            {
                if (!EndMainSection(_launchPulseState.labels))
                {
                    services.Fault("Failed to write launch section end marker");
                    return LoopController::ControlVector::Brake;
                }
                if (!AdvanceLaunchSequence())
                {
                    services.Fault("Failed to advance open-floor launch sequence");
                }
            }
            return LoopController::ControlVector::Brake;
        }

        return LoopController::ControlVector::RawMotorPwm(
            _launchPulseState.signedDriveCommand,
            _launchPulseState.signedDriveCommand);
    }

    const unsigned long nowMs = millis();
    if (!estimatorStationary)
    {
        _launchPulseState.phase = LaunchPulseState::Phase::BrakeToStop;
        _launchPulseState.settleStartMs = 0UL;
        _launchPulseState.labels.phaseId = MazeMap::OpenFloorPhaseId::Brake;
        _launchPulseState.labels.progressNorm = 0.0f;
    }
    else
    {
        if (_launchPulseState.phase != LaunchPulseState::Phase::HoldSettled)
        {
            _launchPulseState.phase = LaunchPulseState::Phase::HoldSettled;
            _launchPulseState.settleStartMs = nowMs;
        }
        _launchPulseState.labels.phaseId = MazeMap::OpenFloorPhaseId::Hold;
        _launchPulseState.labels.progressNorm = (MazeMap::kOpenFloorLaunchSettleMs > 0UL) ?
            (std::clamp)(
                static_cast<float>(nowMs - _launchPulseState.settleStartMs) /
                    static_cast<float>(MazeMap::kOpenFloorLaunchSettleMs),
                0.0f,
                1.0f) :
            1.0f;
    }

    StageMainSample(_launchPulseState.labels, cycle);
    if (estimatorStationary && ((nowMs - _launchPulseState.settleStartMs) >= MazeMap::kOpenFloorLaunchSettleMs))
    {
        if (!EndMainSection(_launchPulseState.labels))
        {
            services.Fault("Failed to write launch section end marker");
            return LoopController::ControlVector::Brake;
        }
        if (!AdvanceLaunchSequence())
        {
            services.Fault("Failed to advance open-floor launch sequence");
        }
    }
    return LoopController::ControlVector::Brake;
}

bool OpenFloorMeasurementController::StartLaunchPulsePhase(const float signedDriveCommand, const uint16_t repeatIndex)
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = MazeMap::OpenFloorSectionId::Sec20Launch;
    labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    labels.primitiveId = MazeMap::OpenFloorPrimitiveId::OpenLoopLaunch;
    labels.directionId =
        (signedDriveCommand >= 0.0f) ?
        MazeMap::OpenFloorDirectionId::Positive :
        MazeMap::OpenFloorDirectionId::Negative;
    labels.repeatIndex = repeatIndex;
    if (!BeginMainSection(labels))
    {
        return false;
    }

    _launchPulseState = LaunchPulseState{};
    _launchPulseState.labels = labels;
    _launchPulseState.signedDriveCommand = signedDriveCommand;
    _launchPulseState.pulseDeadlineMs = millis() + MazeMap::kOpenFloorLaunchPulseMs;
    _launchPulseState.stationaryCheckState.SetStateVector(_drive.GetEstimatorStateVector());
    _launchPulseState.previousStationary = _launchPulseState.stationaryCheckState.IsStationary();
    _phaseFn = &OpenFloorMeasurementController::ExecuteLaunchPulseTick;
    return true;
}

bool OpenFloorMeasurementController::AdvanceLaunchSequence()
{
    while (_launchSequenceState.magnitudeIndex < MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
    {
        if (_launchSequenceState.repeatIteration >= DiagnosticConfig::kLaunchRepeatsPerMagnitude)
        {
            _launchSequenceState.repeatIteration = 0U;
            _launchSequenceState.negativeNext = false;
            ++_launchSequenceState.magnitudeIndex;
            continue;
        }

        const float magnitude = MazeMap::kOpenFloorLaunchDriveMagnitudes[_launchSequenceState.magnitudeIndex];
        const bool negative = _launchSequenceState.negativeNext;
        const std::uint16_t repeatIndex = static_cast<std::uint16_t>(_launchSequenceState.nextRepeatIndex + 1U);
        _launchSequenceState.nextRepeatIndex = repeatIndex;
        if (_launchSequenceState.negativeNext)
        {
            _launchSequenceState.negativeNext = false;
            ++_launchSequenceState.repeatIteration;
        }
        else
        {
            _launchSequenceState.negativeNext = true;
        }

        return StartLaunchPulsePhase(negative ? -magnitude : magnitude, repeatIndex);
    }

    _straightSequenceState = StraightSequenceState{};
    return AdvanceStraightSequence();
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ExecuteStraightDistanceTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_straightSectionState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _straightSectionState.startDistanceM);
    const float remainingM = (std::max)(0.0f, _straightSectionState.distanceM - traveledM);
    _straightSectionState.labels.progressNorm = (std::clamp)(traveledM / _straightSectionState.distanceM, 0.0f, 1.0f);
    _straightSectionState.labels.phaseId = StraightPhaseForProgress(_straightSectionState.labels.progressNorm);

    if ((remainingM <= Config::kDistanceToleranceM) &&
        (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
    {
        StageMainSample(_straightSectionState.labels, cycle);
        const bool inLoopSection =
            (_straightSectionState.labels.sectionId == MazeMap::OpenFloorSectionId::Sec60LoopCw) ||
            (_straightSectionState.labels.sectionId == MazeMap::OpenFloorSectionId::Sec70LoopCcw);
        if (inLoopSection)
        {
            if (!AdvanceLoopAfterStraight())
            {
                services.Fault("Failed to continue open-floor loop straight segment");
            }
        }
        else
        {
            if (!EndMainSection(_straightSectionState.labels))
            {
                services.Fault("Failed to write straight section end marker");
                return LoopController::ControlVector::Brake;
            }
            if (!AdvanceStraightSequence())
            {
                services.Fault("Failed to advance open-floor straight sequence");
            }
        }
        return LoopController::ControlVector::Brake;
    }
    if (_straightSectionState.translationWatchdog.Stalled(
            traveledM,
            _straightSectionState.commandedSpeedMagnitudeMps,
            remainingM,
            millis()))
    {
        cycle.watchdogFlags |= kWatchdogFlagTranslationStall;
    }
    if (static_cast<long>(_straightSectionState.timeoutMs - millis()) <= 0)
    {
        _straightSectionState.labels.abortMarker = true;
        cycle.watchdogFlags |= kWatchdogFlagSectionTimeout;
        StageMainFault(
            _straightSectionState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::StraightSectionTimedOut,
            true);
        services.Fault("Straight section timed out");
        return LoopController::ControlVector::Brake;
    }

    const float accelLimitedSpeedMps =
        (std::min)(
            _straightSectionState.cruiseSpeedMps,
            _straightSectionState.commandedSpeedMagnitudeMps + (_straightSectionState.limits.accelMps2 * state.dtSeconds));
    const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, _straightSectionState.limits.decelMps2);
    _straightSectionState.commandedSpeedMagnitudeMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);
    const float headingErrorRad = HeadingErrorRad(_straightSectionState.targetHeading, state.estimate.headingUnit);
    const float angularCommandRadps =
        (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * state.estimate.angularSpeedRadps);

    StageMainSample(_straightSectionState.labels, cycle);
    return _drive.PointControlVector(
        _straightSectionState.straightDirectionSign * _straightSectionState.commandedSpeedMagnitudeMps,
        angularCommandRadps,
        MazeMap::CommandPD::StateWheelOmegaPD);
}

bool OpenFloorMeasurementController::StartStraightDistancePhase(
    const MazeMap::OpenFloorSectionId sectionId,
    const MazeMap::OpenFloorMarkerId markerId,
    const MazeMap::OpenFloorDirectionId directionId,
    const float distanceM,
    const float cruiseSpeedMps,
    const uint16_t repeatIndex,
    const MazeMap::OpenFloorSpeedBin speedBin,
    const MazeMap::OpenFloorPrimitiveId primitiveId,
    const bool emitSectionMarkers)
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = sectionId;
    labels.startMarkerId = markerId;
    labels.directionId = directionId;
    labels.primitiveId = primitiveId;
    labels.repeatIndex = repeatIndex;
    labels.speedBin = speedBin;
    if (emitSectionMarkers && !BeginMainSection(labels))
    {
        return false;
    }

    _straightSectionState = StraightSectionState{};
    _straightSectionState.labels = labels;
    _straightSectionState.distanceM = distanceM;
    _straightSectionState.cruiseSpeedMps = cruiseSpeedMps;
    _straightSectionState.limits = MeasurementLimits(cruiseSpeedMps);
    _straightSectionState.straightDirectionSign =
        (directionId == MazeMap::OpenFloorDirectionId::Southbound) ? -1.0f : 1.0f;
    _straightSectionState.startDistanceM = _drive.GetAverageDistanceMeters();
    _straightSectionState.targetHeading = _drive.GetPose().headingUnit;
    _straightSectionState.translationWatchdog.Reset(0.0f, millis());
    _straightSectionState.timeoutMs = millis() +
        FailureTimeoutMs(static_cast<unsigned long>(2500.0f + (6000.0f * distanceM)));
    _phaseFn = &OpenFloorMeasurementController::ExecuteStraightDistanceTick;
    return true;
}

bool OpenFloorMeasurementController::AdvanceStraightSequence()
{
    const float straightDistanceM = MazeMap::OpenFloorStrEquivalentDistanceMeters(4U);
    while (_straightSequenceState.speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size())
    {
        if (_straightSequenceState.repeatIteration >= DiagnosticConfig::kStraightRepeatsPerSpeed)
        {
            _straightSequenceState.repeatIteration = 0U;
            _straightSequenceState.southboundNext = false;
            ++_straightSequenceState.speedIndex;
            continue;
        }

        const bool southbound = _straightSequenceState.southboundNext;
        const std::uint16_t repeatIndex = static_cast<std::uint16_t>(_straightSequenceState.nextRepeatIndex + 1U);
        _straightSequenceState.nextRepeatIndex = repeatIndex;
        if (_straightSequenceState.southboundNext)
        {
            _straightSequenceState.southboundNext = false;
            ++_straightSequenceState.repeatIteration;
        }
        else
        {
            _straightSequenceState.southboundNext = true;
        }

        return StartStraightDistancePhase(
            MazeMap::OpenFloorSectionId::Sec30Straight,
            southbound ? MazeMap::OpenFloorMarkerId::S : MazeMap::OpenFloorMarkerId::N,
            southbound ? MazeMap::OpenFloorDirectionId::Southbound : MazeMap::OpenFloorDirectionId::Northbound,
            straightDistanceM,
            MazeMap::kOpenFloorStraightSpeedBinsMps[_straightSequenceState.speedIndex],
            repeatIndex,
            SpeedBinForIndex(_straightSequenceState.speedIndex),
            MazeMap::OpenFloorPrimitiveId::Str4,
            true);
    }

    _yawSequenceState = YawSequenceState{};
    OpenFloorMeasurementLabels settleLabels{};
    settleLabels.sectionId = MazeMap::OpenFloorSectionId::Sec40Yaw;
    settleLabels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    settleLabels.phaseId = MazeMap::OpenFloorPhaseId::Brake;
    return StartSectionSettlePhase(settleLabels, SectionSettleState::Destination::Yaw);
}

bool OpenFloorMeasurementController::StartSectionSettlePhase(
    const OpenFloorMeasurementLabels& labels,
    const SectionSettleState::Destination destination)
{
    if (!BeginMainSection(labels))
    {
        return false;
    }

    _sectionSettleState = SectionSettleState{};
    _sectionSettleState.labels = labels;
    _sectionSettleState.destination = destination;
    _phaseFn = &OpenFloorMeasurementController::ExecuteSectionSettleTick;
    return true;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ExecuteSectionSettleTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_sectionSettleState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    _sectionSettleState.stationaryCheckState.SetStateVector(_drive.GetEstimatorStateVector());
    const bool estimatorStationary = _sectionSettleState.stationaryCheckState.IsStationary();
    const unsigned long nowMs = millis();
    if (!estimatorStationary)
    {
        _sectionSettleState.holdingSettled = false;
        _sectionSettleState.settleStartMs = 0UL;
        _sectionSettleState.labels.phaseId = MazeMap::OpenFloorPhaseId::Brake;
        _sectionSettleState.labels.progressNorm = 0.0f;
    }
    else
    {
        if (!_sectionSettleState.holdingSettled)
        {
            _sectionSettleState.holdingSettled = true;
            _sectionSettleState.settleStartMs = nowMs;
        }
        _sectionSettleState.labels.phaseId = MazeMap::OpenFloorPhaseId::Hold;
        _sectionSettleState.labels.progressNorm = (MazeMap::kOpenFloorLaunchSettleMs > 0UL) ?
            (std::clamp)(
                static_cast<float>(nowMs - _sectionSettleState.settleStartMs) /
                    static_cast<float>(MazeMap::kOpenFloorLaunchSettleMs),
                0.0f,
                1.0f) :
            1.0f;
    }

    StageMainSample(_sectionSettleState.labels, cycle);
    if (estimatorStationary &&
        ((_sectionSettleState.holdingSettled && ((nowMs - _sectionSettleState.settleStartMs) >= MazeMap::kOpenFloorLaunchSettleMs)) ||
         (MazeMap::kOpenFloorLaunchSettleMs == 0UL)))
    {
        if (!EndMainSection(_sectionSettleState.labels))
        {
            services.Fault("Failed to write section settle end marker");
            return LoopController::ControlVector::Brake;
        }

        switch (_sectionSettleState.destination)
        {
        case SectionSettleState::Destination::Yaw:
            if (!AdvanceYawSequence())
            {
                services.Fault("Failed to advance open-floor yaw sequence after settle");
            }
            break;

        case SectionSettleState::Destination::None:
        default:
            services.Fault("Open-floor section settle destination was not configured");
            break;
        }
    }

    return LoopController::ControlVector::Brake;
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ExecuteInPlaceTurnTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_turnSectionState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    const float errorRad = AngleErrorRad(_turnSectionState.targetYawRad, state.estimate.yawRad);
    _turnSectionState.labels.progressNorm = (_turnSectionState.targetMagnitude > 0.0f) ?
        (std::clamp)(1.0f - (std::fabs(errorRad) / _turnSectionState.targetMagnitude), 0.0f, 1.0f) :
        1.0f;
    _turnSectionState.labels.phaseId = TurnPhaseForProgress(_turnSectionState.labels.progressNorm);

    if (MazeMap::IsInPlaceTurnComplete(errorRad, state.estimate.angularSpeedRadps, _turnSectionState.turnProfile))
    {
        StageMainSample(_turnSectionState.labels, cycle);
        const bool inLoopSection =
            (_turnSectionState.labels.sectionId == MazeMap::OpenFloorSectionId::Sec60LoopCw) ||
            (_turnSectionState.labels.sectionId == MazeMap::OpenFloorSectionId::Sec70LoopCcw);
        if (inLoopSection)
        {
            const bool finalLoopComplete =
                !_loopSequenceState.clockwise &&
                (_loopSequenceState.legIndex == 3U) &&
                (_loopSequenceState.repeatIndex == DiagnosticConfig::kLoopRepeats);
            if (finalLoopComplete)
            {
                if (!EndMainSection(_loopSequenceState.loopLabels))
                {
                    services.Fault("Failed to write final loop section end marker");
                    return LoopController::ControlVector::Brake;
                }
                services.RequestEndLoop();
            }
            else if (!AdvanceLoopAfterTurn())
            {
                services.Fault("Failed to continue open-floor loop turn sequence");
            }
        }
        else
        {
            if (!EndMainSection(_turnSectionState.labels))
            {
                services.Fault("Failed to write yaw section end marker");
                return LoopController::ControlVector::Brake;
            }
            if (!AdvanceYawSequence())
            {
                services.Fault("Failed to advance open-floor yaw sequence");
            }
        }
        return LoopController::ControlVector::Brake;
    }
    if (static_cast<long>(_turnSectionState.timeoutMs - millis()) <= 0)
    {
        _turnSectionState.labels.abortMarker = true;
        cycle.watchdogFlags |= kWatchdogFlagSectionTimeout;
        StageMainFault(
            _turnSectionState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::YawSectionTimedOut,
            true);
        services.Fault("Yaw section timed out");
        return LoopController::ControlVector::Brake;
    }

    float angularCommandRadps = 0.0f;
    if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
            errorRad,
            state.estimate.angularSpeedRadps,
            _turnSectionState.turnProfile,
            angularCommandRadps))
    {
        _turnSectionState.labels.abortMarker = true;
        StageMainFault(
            _turnSectionState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::YawProfileInvalid,
            true);
        services.Fault("Yaw profile became invalid");
        return LoopController::ControlVector::Brake;
    }

    StageMainSample(_turnSectionState.labels, cycle);
    return _drive.PointControlVector(
        0.0f,
        angularCommandRadps,
        MazeMap::CommandPD::StateWheelOmegaPD);
}

bool OpenFloorMeasurementController::StartInPlaceTurnPhase(
    const MazeMap::OpenFloorPrimitiveId primitiveId,
    const MazeMap::OpenFloorDirectionId directionId,
    const float angleRad,
    const float maxOmegaRadps,
    const uint16_t repeatIndex,
    const MazeMap::OpenFloorSpeedBin speedBin,
    const bool emitSectionMarkers)
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId =
        emitSectionMarkers ? MazeMap::OpenFloorSectionId::Sec40Yaw : _loopSequenceState.loopLabels.sectionId;
    labels.startMarkerId =
        emitSectionMarkers ? MazeMap::OpenFloorMarkerId::C : _loopSequenceState.loopLabels.startMarkerId;
    labels.primitiveId = primitiveId;
    labels.directionId = directionId;
    labels.repeatIndex = repeatIndex;
    labels.speedBin = speedBin;
    if (emitSectionMarkers && !BeginMainSection(labels))
    {
        return false;
    }

    MotionLimits limits = MeasurementLimits(0.0f);
    limits.maxAngularSpeedRadps = maxOmegaRadps;
    _turnSectionState = TurnSectionState{};
    _turnSectionState.labels = labels;
    _turnSectionState.turnProfile = BuildSharedInPlaceTurnProfile(limits);
    _turnSectionState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
    _turnSectionState.targetMagnitude = std::fabs(angleRad);
    _turnSectionState.timeoutMs = millis() + FailureTimeoutMs(3000UL);
    _phaseFn = &OpenFloorMeasurementController::ExecuteInPlaceTurnTick;
    return true;
}

bool OpenFloorMeasurementController::AdvanceYawSequence()
{
    while (_yawSequenceState.speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size())
    {
        if (_yawSequenceState.repeatIteration >= DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed)
        {
            _yawSequenceState.repeatIteration = 0U;
            _yawSequenceState.primitiveIndex = 0U;
            ++_yawSequenceState.speedIndex;
            continue;
        }

        const std::uint8_t primitiveIndex = _yawSequenceState.primitiveIndex;
        const std::uint16_t repeatIndex = static_cast<std::uint16_t>(_yawSequenceState.nextRepeatIndex + 1U);
        _yawSequenceState.nextRepeatIndex = repeatIndex;
        ++_yawSequenceState.primitiveIndex;
        if (_yawSequenceState.primitiveIndex >= 3U)
        {
            _yawSequenceState.primitiveIndex = 0U;
            ++_yawSequenceState.repeatIteration;
        }

        MazeMap::OpenFloorPrimitiveId primitiveId = MazeMap::OpenFloorPrimitiveId::None;
        MazeMap::OpenFloorDirectionId directionId = MazeMap::OpenFloorDirectionId::None;
        float angleRad = 0.0f;
        switch (primitiveIndex)
        {
        case 0U:
            primitiveId = MazeMap::OpenFloorPrimitiveId::Ip90;
            directionId = MazeMap::OpenFloorDirectionId::Clockwise;
            angleRad = HALF_PI_F;
            break;

        case 1U:
            primitiveId = MazeMap::OpenFloorPrimitiveId::Ip90M;
            directionId = MazeMap::OpenFloorDirectionId::CounterClockwise;
            angleRad = -HALF_PI_F;
            break;

        case 2U:
        default:
            primitiveId = MazeMap::OpenFloorPrimitiveId::Ip180;
            directionId = MazeMap::OpenFloorDirectionId::Flip;
            angleRad = PI_F;
            break;
        }

        return StartInPlaceTurnPhase(
            primitiveId,
            directionId,
            angleRad,
            MazeMap::kOpenFloorYawOmegaBinsRadps[_yawSequenceState.speedIndex],
            repeatIndex,
            SpeedBinForIndex(_yawSequenceState.speedIndex),
            true);
    }

    _smoothSequenceState = SmoothSequenceState{};
    return AdvanceSmoothSequence();
}

MazeMap::App::Internal::LoopController::ControlVector OpenFloorMeasurementController::ExecuteSmoothTurnTick(
    std::uint32_t loopEndTimeUs,
    const MazeMap::App::Internal::LoopController::ModeState& state,
    MazeMap::App::Internal::LoopController::TickServices& services)
{
    (void)loopEndTimeUs;
    OpenFloorMeasurementCycle cycle{};
    PopulateCycleFromState(state, cycle);
    if (QueueMeasurementCaptureFault(_smoothTurnState.labels, cycle))
    {
        services.Fault(cycle.estimatorFault ?
            "Estimator fault during open-floor measurement" :
            "Primary diagnostic selector jumper removed during open-floor measurement");
        return LoopController::ControlVector::Brake;
    }

    const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _smoothTurnState.startDistanceM);
    const float remainingM = (std::max)(0.0f, _smoothTurnState.totalDistanceM - traveledM);
    _smoothTurnState.labels.progressNorm =
        (_smoothTurnState.totalDistanceM > 0.0f) ?
        (std::clamp)(traveledM / _smoothTurnState.totalDistanceM, 0.0f, 1.0f) :
        0.0f;
    _smoothTurnState.labels.phaseId =
        (_smoothTurnState.labels.progressNorm < 0.33f) ? MazeMap::OpenFloorPhaseId::Entry :
        (_smoothTurnState.labels.progressNorm > 0.66f) ? MazeMap::OpenFloorPhaseId::Exit :
        MazeMap::OpenFloorPhaseId::Middle;

    if (remainingM <= Config::kDistanceToleranceM)
    {
        StageMainSample(_smoothTurnState.labels, cycle);
        if (!EndMainSection(_smoothTurnState.labels))
        {
            services.Fault("Failed to write smooth-turn section end marker");
            return LoopController::ControlVector::Brake;
        }
        if (!AdvanceSmoothSequence())
        {
            services.Fault("Failed to advance open-floor smooth-turn sequence");
        }
        return LoopController::ControlVector::Brake;
    }
    if (_smoothTurnState.translationWatchdog.Stalled(traveledM, _smoothTurnState.cruiseSpeed, remainingM, millis()))
    {
        cycle.watchdogFlags |= kWatchdogFlagTranslationStall;
    }
    if (static_cast<long>(_smoothTurnState.timeoutMs - millis()) <= 0)
    {
        _smoothTurnState.labels.abortMarker = true;
        cycle.watchdogFlags |= kWatchdogFlagSectionTimeout;
        StageMainFault(
            _smoothTurnState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::SmoothSectionTimedOut,
            true);
        services.Fault("Smooth-turn section timed out");
        return LoopController::ControlVector::Brake;
    }

    MazeMap::ManeuverPoint point{};
    if (!_smoothTurnState.maneuver.TryGetManeuverPoint(
            traveledM,
            _smoothTurnState.cruiseSpeed,
            point,
            Config::kCellSizeM))
    {
        _smoothTurnState.labels.abortMarker = true;
        StageMainFault(
            _smoothTurnState.labels,
            cycle,
            MazeMap::OpenFloorFaultCode::SmoothTargetInvalid,
            true);
        services.Fault("Maneuver point became invalid");
        return LoopController::ControlVector::Brake;
    }
    const float yawRateCorrectionRadps = MazeMap::ComputeSmoothTurnYawRatePdCorrection(
        point.Omega,
        state.estimate.angularSpeedRadps,
        state.dtSeconds,
        Config::kSmoothTurnYawRateKp,
        Config::kSmoothTurnYawRateKd,
        _smoothTurnState.yawRateController);
    point.Omega = point.Omega + yawRateCorrectionRadps;
    point.Omega = (std::clamp)(
        point.Omega,
        -_smoothTurnState.limits.maxAngularSpeedRadps,
        _smoothTurnState.limits.maxAngularSpeedRadps);

    StageMainSample(_smoothTurnState.labels, cycle);
    return _drive.PointControlVector(
        point,
        MazeMap::CommandPD::StateWheelOmegaPD);
}

bool OpenFloorMeasurementController::StartSmoothTurnPhase(
    const MazeMap::ManeuverCode code,
    const float cruiseSpeed,
    const uint16_t repeatIndex,
    const MazeMap::OpenFloorSpeedBin speedBin)
{
    const MazeMap::ManeuverInstance maneuver(code, MazeMap::DirectionalLocation());
    if (!maneuver.SupportsPointTracking())
    {
        return false;
    }
    const float totalDistanceM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM);
    if (!(totalDistanceM > 0.0f))
    {
        return false;
    }

    OpenFloorMeasurementLabels labels{};
    labels.sectionId = MazeMap::OpenFloorSectionId::Sec50Smooth;
    labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    labels.primitiveId = PrimitiveIdForSmoothCode(code);
    labels.directionId =
        ((code & MazeMap::MIRRORED_MANEUVER_FLAG) == MazeMap::MIRRORED_MANEUVER_FLAG) ?
        MazeMap::OpenFloorDirectionId::Left :
        MazeMap::OpenFloorDirectionId::Right;
    labels.repeatIndex = repeatIndex;
    labels.speedBin = speedBin;
    if ((labels.primitiveId == MazeMap::OpenFloorPrimitiveId::None) || !BeginMainSection(labels))
    {
        return false;
    }

    _smoothTurnState = SmoothTurnState{};
    _smoothTurnState.labels = labels;
    _smoothTurnState.maneuver = maneuver;
    _smoothTurnState.cruiseSpeed = cruiseSpeed;
    _smoothTurnState.totalDistanceM = totalDistanceM;
    _smoothTurnState.startDistanceM = _drive.GetAverageDistanceMeters();
    _smoothTurnState.translationWatchdog.Reset(0.0f, millis());
    _smoothTurnState.timeoutMs = millis() +
        FailureTimeoutMs(static_cast<unsigned long>(2500.0f + (5000.0f * totalDistanceM)));
    _smoothTurnState.limits = MeasurementLimits(cruiseSpeed);
    _phaseFn = &OpenFloorMeasurementController::ExecuteSmoothTurnTick;
    return true;
}

bool OpenFloorMeasurementController::AdvanceSmoothSequence()
{
    static constexpr MazeMap::ManeuverCode kSmoothCodes[] = {
        MazeMap::S45SS,
        MazeMap::S45SS_M,
        MazeMap::S90SS,
        MazeMap::S90SS_M,
        MazeMap::S135SS,
        MazeMap::S135SS_M,
    };

    while (_smoothSequenceState.speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size())
    {
        if (_smoothSequenceState.codeIndex >= std::size(kSmoothCodes))
        {
            _smoothSequenceState.codeIndex = 0U;
            _smoothSequenceState.repeatIteration = 0U;
            ++_smoothSequenceState.speedIndex;
            continue;
        }

        if (_smoothSequenceState.repeatIteration >= DiagnosticConfig::kSmoothRepeatsPerPrimitiveSpeed)
        {
            _smoothSequenceState.repeatIteration = 0U;
            ++_smoothSequenceState.codeIndex;
            continue;
        }

        const MazeMap::ManeuverCode code = kSmoothCodes[_smoothSequenceState.codeIndex];
        const std::uint16_t repeatIndex = static_cast<std::uint16_t>(_smoothSequenceState.nextRepeatIndex + 1U);
        _smoothSequenceState.nextRepeatIndex = repeatIndex;
        ++_smoothSequenceState.repeatIteration;
        if (_smoothSequenceState.repeatIteration >= DiagnosticConfig::kSmoothRepeatsPerPrimitiveSpeed)
        {
            _smoothSequenceState.repeatIteration = 0U;
            ++_smoothSequenceState.codeIndex;
        }

        return StartSmoothTurnPhase(
            code,
            MazeMap::kOpenFloorSmoothSpeedBinsMps[_smoothSequenceState.speedIndex],
            repeatIndex,
            SpeedBinForIndex(_smoothSequenceState.speedIndex));
    }

    return StartLoopSequence(true);
}

bool OpenFloorMeasurementController::StartLoopSequence(const bool clockwise)
{
    _loopSequenceState = LoopSequenceState{};
    _loopSequenceState.clockwise = clockwise;
    _loopSequenceState.repeatIndex = 1U;
    return AdvanceLoopSequenceAfterRepeat();
}

bool OpenFloorMeasurementController::AdvanceLoopSequenceAfterRepeat()
{
    const MazeMap::OpenFloorMarkerId markerId =
        _loopSequenceState.clockwise ? MazeMap::OpenFloorMarkerId::CW : MazeMap::OpenFloorMarkerId::CCW;
    const MazeMap::OpenFloorSectionId sectionId =
        _loopSequenceState.clockwise ? MazeMap::OpenFloorSectionId::Sec60LoopCw : MazeMap::OpenFloorSectionId::Sec70LoopCcw;

    _loopSequenceState.legIndex = 0U;
    _loopSequenceState.loopLabels = OpenFloorMeasurementLabels{};
    _loopSequenceState.loopLabels.sectionId = sectionId;
    _loopSequenceState.loopLabels.startMarkerId = markerId;
    _loopSequenceState.loopLabels.repeatIndex = _loopSequenceState.repeatIndex;
    _loopSequenceState.recoveryLabels = _loopSequenceState.loopLabels;
    _loopSequenceState.recoveryLabels.primitiveId = MazeMap::OpenFloorPrimitiveId::Recovery;
    _loopSequenceState.recoveryLabels.directionId = MazeMap::OpenFloorDirectionId::None;
    _loopSequenceState.recoveryLabels.phaseId = MazeMap::OpenFloorPhaseId::Recovery;
    return StartRecoveryPhase(
        _loopSequenceState.loopLabels,
        markerId,
        DiagnosticConfig::kCharacterizationRecoverySpeedMps,
        2500UL,
        true);
}

bool OpenFloorMeasurementController::AdvanceLoopAfterRecovery()
{
    if (!BeginMainSection(_loopSequenceState.loopLabels))
    {
        return false;
    }

    return StartStraightDistancePhase(
        _loopSequenceState.loopLabels.sectionId,
        _loopSequenceState.loopLabels.startMarkerId,
        _loopSequenceState.clockwise ?
            MazeMap::OpenFloorDirectionId::Clockwise :
            MazeMap::OpenFloorDirectionId::CounterClockwise,
        MazeMap::OpenFloorStrEquivalentDistanceMeters(2U),
        MazeMap::kOpenFloorStraightSpeedBinsMps[0],
        _loopSequenceState.repeatIndex,
        MazeMap::OpenFloorSpeedBin::Low,
        MazeMap::OpenFloorPrimitiveId::Str2,
        false);
}

bool OpenFloorMeasurementController::AdvanceLoopAfterStraight()
{
    return StartInPlaceTurnPhase(
        _loopSequenceState.clockwise ?
            MazeMap::OpenFloorPrimitiveId::Ip90 :
            MazeMap::OpenFloorPrimitiveId::Ip90M,
        _loopSequenceState.clockwise ?
            MazeMap::OpenFloorDirectionId::Clockwise :
            MazeMap::OpenFloorDirectionId::CounterClockwise,
        _loopSequenceState.clockwise ? HALF_PI_F : -HALF_PI_F,
        MazeMap::kOpenFloorYawOmegaBinsRadps[1],
        _loopSequenceState.repeatIndex,
        MazeMap::OpenFloorSpeedBin::Medium,
        false);
}

bool OpenFloorMeasurementController::AdvanceLoopAfterTurn()
{
    if (_loopSequenceState.legIndex < 3U)
    {
        ++_loopSequenceState.legIndex;
        return StartStraightDistancePhase(
            _loopSequenceState.loopLabels.sectionId,
            _loopSequenceState.loopLabels.startMarkerId,
            _loopSequenceState.clockwise ?
                MazeMap::OpenFloorDirectionId::Clockwise :
                MazeMap::OpenFloorDirectionId::CounterClockwise,
            MazeMap::OpenFloorStrEquivalentDistanceMeters(2U),
            MazeMap::kOpenFloorStraightSpeedBinsMps[0],
            _loopSequenceState.repeatIndex,
            MazeMap::OpenFloorSpeedBin::Low,
            MazeMap::OpenFloorPrimitiveId::Str2,
            false);
    }

    if (!EndMainSection(_loopSequenceState.loopLabels))
    {
        return false;
    }

    if (_loopSequenceState.repeatIndex < DiagnosticConfig::kLoopRepeats)
    {
        ++_loopSequenceState.repeatIndex;
        return AdvanceLoopSequenceAfterRepeat();
    }

    return _loopSequenceState.clockwise ? StartLoopSequence(false) : false;
}

namespace MazeMap::App::Internal
{
    IApplicationMode& GetDiagnosticMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PrimaryDiagnostic,
            BootModeCategory::Utility,
            "primary_diagnostic",
            "Run the open-floor measurement battery for the primary diagnostic mode.",
            "logging.txt; open-floor timing and main mmlog",
            &GetDiagnosticMode,
            "GetDiagnosticMode",
            "OpenFloorMeasurementController.cpp",
            "timing capture; static hold; launch; straight; settle; yaw; smooth turn; CW loop; CCW loop",
            "DiagnosticConfig; OpenFloorMeasurementSpec; mission drive/sensor tuning",
            "Open-floor workspace, repeats, speed bins, and primitives are diagnostic-local.",
            "open_floor_timing.mmlog; open_floor_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}

