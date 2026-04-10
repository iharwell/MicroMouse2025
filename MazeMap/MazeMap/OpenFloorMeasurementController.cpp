#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
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
    X(float,         ukf_state_px_m)               \
    X(float,         ukf_state_py_m)               \
    X(float,         ukf_state_psi_rad)            \
    X(float,         ukf_state_u_mps)              \
    X(float,         ukf_state_v_mps)              \
    X(float,         ukf_state_r_radps)            \
    X(float,         ukf_state_omega_l_radps)      \
    X(float,         ukf_state_omega_r_radps)      \
    X(float,         ukf_state_bgz_radps)          \
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
inline constexpr std::uint16_t kOpenFloorMeasurementFlagWorkspaceViolation = 1u << 1;
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
    if (cycle.workspaceViolation)
    {
        flags |= kOpenFloorMeasurementFlagWorkspaceViolation;
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
    static constexpr uint16_t kWatchdogFlagTranslationStall = 1u << 0;
    static constexpr uint16_t kWatchdogFlagSectionTimeout = 1u << 1;
    static constexpr uint16_t kWatchdogFlagRecoveryTimeout = 1u << 2;
    // Policy: no watchdog timer in this codebase may trigger in under 90 seconds.
    static constexpr unsigned long kMinimumFailureTimeoutMs = 90000UL;

    SharedRobotRuntime& _runtime;
    MazeMap::Vehicle& _vehicle;
    DiagnosticSensorSuite& _sensors;
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
    unsigned long _lastControlMicros;
    unsigned long _workspaceOutOfBoundsStartMs;
    uint16_t _timingTickIndex;
    uint32_t _controlTickSequence;
    char _runId[24];
    bool _workspaceOutOfBoundsActive;

    static MotionLimits MeasurementLimits(float maxSpeedMps);
    static void HandleRuntimeFault(void* context, const char* reason) noexcept;
    static uint32_t ReadCycleCounter();
    static bool TryGetSmoothTurnExecutionProfileMeters(MazeMap::ManeuverCode code, MazeMap::SmoothTurnExecutionProfile& profile);
    static MazeMap::OpenFloorPrimitiveId PrimitiveIdForSmoothCode(MazeMap::ManeuverCode code);
    static unsigned long FailureTimeoutMs(unsigned long requestedTimeoutMs);
    static MazeMap::OpenFloorPhaseId StraightPhaseForProgress(float progress);
    static MazeMap::OpenFloorPhaseId TurnPhaseForProgress(float progress);

    bool BeginTimingLog();
    bool LogTimingSample(const OpenFloorMeasurementCycle& cycle);
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
    bool LogMainSample(const OpenFloorMeasurementLabels& labels, const OpenFloorMeasurementCycle& cycle);
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
    bool WriteFaultDumpTextEntryAndFlush(const char* type, const char* message);
    bool DumpUkfFaultToTextLog(const char* reason);
    void CloseMainLog();
    void RecordMainLogFailure() noexcept;
    bool FailLogSetupStep(const char* logName, const char* step);

    bool Fail(const char* message);
    bool LogTimingFaultAndFail(
        OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        const char* message,
        uint32_t extra0 = 0UL,
        uint32_t extra1 = 0UL);
    bool LogSectionFaultAndFail(
        OpenFloorMeasurementLabels& labels,
        OpenFloorMeasurementCycle& cycle,
        MazeMap::OpenFloorFaultCode faultCode,
        const char* message,
        uint16_t watchdogFlags = 0U,
        uint16_t clippingFlags = 0U,
        uint32_t extra0 = 0UL,
        uint32_t extra1 = 0UL);
    bool HandleMeasurementCaptureFault(OpenFloorMeasurementLabels& labels, OpenFloorMeasurementCycle& cycle);
    void SeedPoseAtMarker(MazeMap::OpenFloorMarkerId markerId);
    bool IsWithinBoundary() const;
    bool HasWorkspaceViolationFault();
    float ReadBatteryVoltage() const;
    float ReadBoardTemperatureC(const DiagnosticSensorSnapshot& snapshot) const;
    bool CaptureCycle(bool stationary, OpenFloorMeasurementCycle& cycle);
    void FinalizeCycle(OpenFloorMeasurementCycle& cycle);
    bool LogCycle(const OpenFloorMeasurementLabels& labels, OpenFloorMeasurementCycle& cycle);
    bool TraverseToMarker(OpenFloorMeasurementLabels& labels, MazeMap::OpenFloorMarkerId markerId);
    bool RecoverToMarker(OpenFloorMeasurementLabels& labels, MazeMap::OpenFloorMarkerId markerId, float maxSpeedMps, unsigned long timeoutMs);

    bool RunTimingBlock();
    bool RunStaticSection();
    bool RunLaunchSection();
    bool RunStraightSection();
    bool RunYawSection();
    bool RunSmoothSection();
    bool RunLoopSection(bool clockwise);

    bool ExecuteLaunchPulse(float signedDriveCommand, uint16_t repeatIndex);
    bool ExecuteStraightDistance(
        MazeMap::OpenFloorSectionId sectionId,
        MazeMap::OpenFloorMarkerId markerId,
        MazeMap::OpenFloorDirectionId directionId,
        float distanceM,
        float cruiseSpeedMps,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin,
        MazeMap::OpenFloorPrimitiveId primitiveId,
        bool emitSectionMarkers = true,
        bool snapToStartMarker = true);
    bool ExecuteInPlaceTurn(
        MazeMap::OpenFloorPrimitiveId primitiveId,
        MazeMap::OpenFloorDirectionId directionId,
        float angleRad,
        float maxOmegaRadps,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin,
        bool emitSectionMarkers = true,
        bool snapToStartMarker = true);
    bool ExecuteSmoothTurn(
        MazeMap::ManeuverCode code,
        float cruiseSpeed,
        uint16_t repeatIndex,
        MazeMap::OpenFloorSpeedBin speedBin);
};

OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
    : _runtime(runtime)
    , _vehicle(runtime.SpeedVehicle())
    , _sensors(runtime.DiagnosticSensors())
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
    , _lastControlMicros(0UL)
    , _workspaceOutOfBoundsStartMs(0UL)
    , _timingTickIndex(0U)
    , _controlTickSequence(0UL)
    , _workspaceOutOfBoundsActive(false)
{
    _runId[0] = '\0';
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

    OpenFloorTimingRow row{};
    if (!_runtime.BeginUtilityDataLogSchema(row))
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

bool OpenFloorMeasurementController::LogTimingSample(const OpenFloorMeasurementCycle& cycle)
{
    OpenFloorTimingRow row{};
    row.mono_time_us = cycle.masterTimeUs;
    row.control_tick_sequence = cycle.controlTickSequence;
    row.dt_us = cycle.dtUs;
    row.section_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
    row.control_start_us = cycle.controlTiming.controlStartUs;
    row.control_end_us = cycle.controlTiming.controlEndUs;
    row.pwm_latch_us = cycle.controlTiming.pwmLatchUs;
    row.encoder_latch_us = cycle.controlTiming.encoderLatchUs;
    row.encoder_read_done_us = cycle.controlTiming.encoderReadDoneUs;
    row.ukf_predict_start_us = cycle.controlTiming.ukfPredictStartUs;
    row.ukf_predict_end_us = cycle.controlTiming.ukfPredictEndUs;
    row.ukf_predict_duration_us = cycle.controlTiming.ukfPredictDurationUs;
    row.ukf_update_start_us = cycle.controlTiming.ukfUpdateStartUs;
    row.ukf_update_end_us = cycle.controlTiming.ukfUpdateEndUs;
    row.ukf_update_duration_us = cycle.controlTiming.ukfUpdateDurationUs;
    row.imu_drdy_us = cycle.sensorSnapshot.imuTiming.drdyUs;
    row.imu_read_start_us = cycle.sensorSnapshot.imuTiming.readStartUs;
    row.imu_read_done_us = cycle.sensorSnapshot.imuTiming.readDoneUs;
    row.front_led_on_us = cycle.sensorSnapshot.frontTiming.ledOnCommandUs;
    row.front_adc_on_us = cycle.sensorSnapshot.frontTiming.adcOnSampleUs;
    row.front_led_off_us = cycle.sensorSnapshot.frontTiming.ledOffCommandUs;
    row.front_adc_off_us = cycle.sensorSnapshot.frontTiming.adcOffSampleUs;
    row.front_ready_us = cycle.sensorSnapshot.frontTiming.observationReadyUs;
    row.left_led_on_us = cycle.sensorSnapshot.leftTiming.ledOnCommandUs;
    row.left_adc_on_us = cycle.sensorSnapshot.leftTiming.adcOnSampleUs;
    row.left_led_off_us = cycle.sensorSnapshot.leftTiming.ledOffCommandUs;
    row.left_adc_off_us = cycle.sensorSnapshot.leftTiming.adcOffSampleUs;
    row.left_ready_us = cycle.sensorSnapshot.leftTiming.observationReadyUs;
    row.right_led_on_us = cycle.sensorSnapshot.rightTiming.ledOnCommandUs;
    row.right_adc_on_us = cycle.sensorSnapshot.rightTiming.adcOnSampleUs;
    row.right_led_off_us = cycle.sensorSnapshot.rightTiming.ledOffCommandUs;
    row.right_adc_off_us = cycle.sensorSnapshot.rightTiming.adcOffSampleUs;
    row.right_ready_us = cycle.sensorSnapshot.rightTiming.observationReadyUs;
    row.cycle_counter_start = cycle.controlTiming.cycleCounterStart;
    row.cycle_counter_end = cycle.controlTiming.cycleCounterEnd;

    if (!_runtime.LogUtilityDataRow(row))
    {
        RecordTimingLogFailure();
        return false;
    }
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

    OpenFloorMainRow row{};
    if (!_runtime.BeginUtilityDataLogSchema(row))
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

bool OpenFloorMeasurementController::LogMainSample(
    const OpenFloorMeasurementLabels& labels,
    const OpenFloorMeasurementCycle& cycle)
{
    const bool encoderValid = cycle.driveTelemetry.encoderObservationValid;
    const bool imuValid = std::isfinite(cycle.sensorSnapshot.gyroRawRadps);
    const float maxRangeM = MazeMap::PlantParams::Default().noHitRangeM;
    MazeMap::WallObs frontLeftObs{};
    MazeMap::WallObs frontRightObs{};
    DriveBase::BuildLoggedFrontPairObservations(cycle.sensorSnapshot, maxRangeM, frontLeftObs, frontRightObs);
    const MazeMap::WallObs leftObs = DriveBase::BuildLoggedLeftSideObservation(cycle.sensorSnapshot, maxRangeM);
    const MazeMap::WallObs rightObs = DriveBase::BuildLoggedRightSideObservation(cycle.sensorSnapshot, maxRangeM);
    const MazeMap::VehicleState::StateVector& estimatorState = _drive.GetEstimatorStateVector();

    OpenFloorMainRow row{};
    row.master_time_us = cycle.masterTimeUs;
    row.control_tick_sequence = cycle.controlTickSequence;
    row.dt_us = cycle.dtUs;
    row.section_id = static_cast<std::uint8_t>(labels.sectionId);
    row.primitive_id = static_cast<std::uint8_t>(labels.primitiveId);
    row.primitive_family = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveFamilyForId(labels.primitiveId));
    row.direction_id = static_cast<std::uint8_t>(labels.directionId);
    row.phase_id = static_cast<std::uint8_t>(labels.phaseId);
    row.speed_bin = static_cast<std::uint8_t>(labels.speedBin);
    row.start_marker_id = static_cast<std::uint8_t>(labels.startMarkerId);
    row.repeat_index = labels.repeatIndex;
    row.progress_norm = labels.progressNorm;
    row.mode_flags = cycle.driveTelemetry.modeFlags;
    row.clipping_flags = cycle.clippingFlags;
    row.saturation_flags = cycle.driveTelemetry.saturationFlags;
    row.watchdog_flags = cycle.watchdogFlags;
    row.measurement_flags = MazeMap::App::Internal::Runtime::BuildOpenFloorMeasurementFlags(
        labels,
        cycle,
        encoderValid,
        imuValid,
        frontLeftObs,
        frontRightObs,
        leftObs,
        rightObs);
    row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
    row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
    row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
    row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
    row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
    row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
    row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
    row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
    row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
    row.measured_linear_speed_mps = cycle.measuredLinearSpeedMps;
    row.measured_angular_speed_radps = cycle.measuredAngularSpeedRadps;
    row.cmd_linear_mps = _drive.GetLastLinearCommandMps();
    row.cmd_angular_radps = _drive.GetLastAngularCommandRadps();
    row.left_drive_command = cycle.driveTelemetry.leftDriveCommand;
    row.right_drive_command = cycle.driveTelemetry.rightDriveCommand;
    row.left_feedforward_command = cycle.driveTelemetry.leftFeedforwardCommand;
    row.right_feedforward_command = cycle.driveTelemetry.rightFeedforwardCommand;
    row.left_feedback_command = cycle.driveTelemetry.leftFeedbackCommand;
    row.right_feedback_command = cycle.driveTelemetry.rightFeedbackCommand;
    row.left_target_velocity_mps = cycle.driveTelemetry.leftTargetVelocityMps;
    row.right_target_velocity_mps = cycle.driveTelemetry.rightTargetVelocityMps;
    row.left_launch_assist_floor = cycle.driveTelemetry.leftLaunchAssistFloor;
    row.right_launch_assist_floor = cycle.driveTelemetry.rightLaunchAssistFloor;
    row.encoder_timestamp_us = cycle.controlTiming.encoderReadDoneUs;
    row.left_encoder_count = cycle.driveTelemetry.leftEncoderCount;
    row.right_encoder_count = cycle.driveTelemetry.rightEncoderCount;
    row.left_encoder_omega_radps = cycle.driveTelemetry.leftEncoderOmegaRadps;
    row.right_encoder_omega_radps = cycle.driveTelemetry.rightEncoderOmegaRadps;
    row.left_encoder_distance_m = cycle.driveTelemetry.leftDistanceM;
    row.right_encoder_distance_m = cycle.driveTelemetry.rightDistanceM;
    row.left_encoder_velocity_mps = cycle.driveTelemetry.leftVelocityMps;
    row.right_encoder_velocity_mps = cycle.driveTelemetry.rightVelocityMps;
    row.imu_timestamp_us = cycle.sensorSnapshot.imuTiming.readDoneUs;
    row.imu_status = cycle.sensorSnapshot.imuBackLeft.status;
    row.imu_interrupt_high = cycle.sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
    row.accel_bias_valid = cycle.sensorSnapshot.accelBiasValid ? 1U : 0U;
    row.imu_gyro_x = cycle.sensorSnapshot.imuBackLeft.gyroX;
    row.imu_gyro_y = cycle.sensorSnapshot.imuBackLeft.gyroY;
    row.imu_gyro_z = cycle.sensorSnapshot.imuBackLeft.gyroZ;
    row.imu_accel_x = cycle.sensorSnapshot.imuBackLeft.accelX;
    row.imu_accel_y = cycle.sensorSnapshot.imuBackLeft.accelY;
    row.imu_accel_z = cycle.sensorSnapshot.imuBackLeft.accelZ;
    row.imu_temp = cycle.sensorSnapshot.imuBackLeft.temp;
    row.gyro_raw_radps = cycle.sensorSnapshot.gyroRawRadps;
    row.gyro_bias_radps = cycle.sensorSnapshot.gyroBiasRadps;
    row.gyro_radps = cycle.sensorSnapshot.gyroRadps;
    row.accel_body_x_mps2 = cycle.sensorSnapshot.accelBodyXMps2;
    row.accel_body_y_mps2 = cycle.sensorSnapshot.accelBodyYMps2;
    row.planar_accel_mps2 = cycle.planarAccelMps2;
    row.front_timestamp_us = cycle.sensorSnapshot.frontTiming.observationReadyUs;
    row.left_timestamp_us = cycle.sensorSnapshot.leftTiming.observationReadyUs;
    row.right_timestamp_us = cycle.sensorSnapshot.rightTiming.observationReadyUs;
    row.front_left_obs_class = static_cast<std::uint8_t>(frontLeftObs.cls);
    row.front_right_obs_class = static_cast<std::uint8_t>(frontRightObs.cls);
    row.left_obs_class = static_cast<std::uint8_t>(leftObs.cls);
    row.right_obs_class = static_cast<std::uint8_t>(rightObs.cls);
    row.front_left_obs_rho_m = frontLeftObs.rho;
    row.front_right_obs_rho_m = frontRightObs.rho;
    row.left_obs_rho_m = leftObs.rho;
    row.right_obs_rho_m = rightObs.rho;
    row.front_left_obs_confidence = frontLeftObs.confidence;
    row.front_right_obs_confidence = frontRightObs.confidence;
    row.left_obs_confidence = leftObs.confidence;
    row.right_obs_confidence = rightObs.confidence;
    row.fan_duty_cycle = cycle.fanDutyCycle;

    if (!_runtime.LogUtilityDataRow(row))
    {
        RecordMainLogFailure();
        return false;
    }
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

bool OpenFloorMeasurementController::WriteFaultDumpTextEntryAndFlush(const char* type, const char* message)
{
    if (!_runtime.WriteTextLogEntry(micros(), type, message))
    {
        return false;
    }

    _runtime.FlushTextLog();
    return true;
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

    if (!WriteFaultDumpTextEntryAndFlush("ukf_dump_begin", header))
    {
        return false;
    }

    if (!_drive.WriteUkfDebugTextDump(
            [this](const char* type, const char* message) noexcept -> bool
            {
                return WriteFaultDumpTextEntryAndFlush(type, message);
            }))
    {
        return false;
    }

    return WriteFaultDumpTextEntryAndFlush("ukf_dump_end", "complete=true");
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
    _workspaceOutOfBoundsStartMs = 0UL;
    _workspaceOutOfBoundsActive = false;
    _pinsLatchedAtBoot = IsPrimaryDiagnosticModeRequested();
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
    _lastControlMicros = micros();
    return true;
}

void OpenFloorMeasurementController::Run()
{
    if (_faulted)
    {
        return;
    }

    bool ok = true;
    ok = ok && RunTimingBlock();
    if (ok)
    {
        CloseTimingLog();
        _timingLogOpen = false;
        if (!BeginMainLog())
        {
            ok = false;
        }
        else
        {
            _mainLogOpen = true;
        }
    }
    ok = ok && RunStaticSection();
    ok = ok && RunLaunchSection();
    ok = ok && RunStraightSection();
    ok = ok && RunYawSection();
    ok = ok && RunSmoothSection();
    ok = ok && RunLoopSection(true);
    ok = ok && RunLoopSection(false);

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

bool OpenFloorMeasurementController::TryGetSmoothTurnExecutionProfileMeters(
    MazeMap::ManeuverCode code,
    MazeMap::SmoothTurnExecutionProfile& profile)
{
    profile = MazeMap::SmoothTurnExecutionProfile{};
    if ((code == MazeMap::MC_NONE) || IsStraightCode(code))
    {
        return false;
    }

    MazeMap::SmoothTurnExecutionProfile profileInCells{};
    if (!MazeMap::ManeuverSet::GetSet()[code].TryGetSmoothTurnExecutionProfile(profileInCells))
    {
        return false;
    }

    profile = MazeMap::ScaleSmoothTurnExecutionProfile(profileInCells, Config::kCellSizeM);
    profile.radians = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
    return profile.IsValid();
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
    if (message != nullptr && message[0] != '\0')
    {
        AppendStartupTrace(message);
    }

    if (!DumpUkfFaultToTextLog(message))
    {
        AppendStartupTrace("open_floor_ukf_dump_failed");
    }

    _timingLogOpen = false;
    _mainLogOpen = false;
}

bool OpenFloorMeasurementController::LogTimingFaultAndFail(
    OpenFloorMeasurementCycle& cycle,
    MazeMap::OpenFloorFaultCode faultCode,
    const char* message,
    uint32_t extra0,
    uint32_t extra1)
{
    _drive.Brake();
    FinalizeCycle(cycle);
    if (_timingLogOpen)
    {
        if (!LogTimingSample(cycle))
        {
            return Fail("Failed to write timing sample");
        }
        if (!LogTimingFault(cycle, faultCode, true, extra0, extra1))
        {
            return Fail("Failed to write timing fault row");
        }
    }
    return Fail(message);
}

bool OpenFloorMeasurementController::LogSectionFaultAndFail(
    OpenFloorMeasurementLabels& labels,
    OpenFloorMeasurementCycle& cycle,
    MazeMap::OpenFloorFaultCode faultCode,
    const char* message,
    uint16_t watchdogFlags,
    uint16_t clippingFlags,
    uint32_t extra0,
    uint32_t extra1)
{
    labels.abortMarker = true;
    cycle.watchdogFlags |= watchdogFlags;
    cycle.clippingFlags |= clippingFlags;
    _drive.Brake();
    FinalizeCycle(cycle);
    if (_mainLogOpen)
    {
        if (!LogMainSample(labels, cycle))
        {
            return Fail("Failed to write open-floor main sample");
        }
        if (!LogMainFault(labels, cycle, faultCode, true, extra0, extra1))
        {
            return Fail("Failed to write open-floor main fault row");
        }
    }
    return Fail(message);
}

bool OpenFloorMeasurementController::HandleMeasurementCaptureFault(
    OpenFloorMeasurementLabels& labels,
    OpenFloorMeasurementCycle& cycle)
{
    if (cycle.estimatorFault)
    {
        return LogSectionFaultAndFail(
            labels,
            cycle,
            MazeMap::OpenFloorFaultCode::EstimatorFault,
            "Estimator fault during open-floor measurement");
    }
    if (cycle.workspaceViolation)
    {
        return LogSectionFaultAndFail(
            labels,
            cycle,
            MazeMap::OpenFloorFaultCode::WorkspaceViolation,
            "Workspace violation during open-floor measurement");
    }
    return Fail("Open-floor control-cycle capture failed");
}

void OpenFloorMeasurementController::SeedPoseAtMarker(MazeMap::OpenFloorMarkerId markerId)
{
    _drive.SetPose(
        MazeMap::OpenFloorMarkerXMeters(markerId),
        MazeMap::OpenFloorMarkerYMeters(markerId),
        DirectionToYawRad(MazeMap::GetOpenFloorMarker(markerId).heading));
    _lastControlMicros = micros();
    _workspaceOutOfBoundsStartMs = 0UL;
    _workspaceOutOfBoundsActive = false;
}

bool OpenFloorMeasurementController::IsWithinBoundary() const
{
    return MazeMap::IsPoseInsideOpenFloorWorkspace(_drive.GetPose());
}

bool OpenFloorMeasurementController::HasWorkspaceViolationFault()
{
    if (IsWithinBoundary())
    {
        _workspaceOutOfBoundsStartMs = 0UL;
        _workspaceOutOfBoundsActive = false;
        return false;
    }

    const unsigned long nowMs = millis();
    if (!_workspaceOutOfBoundsActive)
    {
        _workspaceOutOfBoundsStartMs = nowMs;
        _workspaceOutOfBoundsActive = true;
        return false;
    }

    return MazeMap::HasOpenFloorOutOfBoundsGraceElapsed(_workspaceOutOfBoundsStartMs, nowMs);
}

float OpenFloorMeasurementController::ReadBatteryVoltage() const
{
    return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
}

float OpenFloorMeasurementController::ReadBoardTemperatureC(const DiagnosticSensorSnapshot& snapshot) const
{
    return 25.0f + (static_cast<float>(snapshot.imuBackLeft.temp) / 256.0f);
}

bool OpenFloorMeasurementController::CaptureCycle(bool stationary, OpenFloorMeasurementCycle& cycle)
{
    if ((micros() - _lastControlMicros) < DiagnosticConfig::kControlPeriodUs)
    {
        ServiceLogs();
        while ((micros() - _lastControlMicros) < DiagnosticConfig::kControlPeriodUs)
        {
            delayMicroseconds(20);
        }
    }

    cycle.controlTiming.controlStartUs = micros();
    cycle.controlTiming.cycleCounterStart = ReadCycleCounter();
    cycle.masterTimeUs = cycle.controlTiming.controlStartUs;
    cycle.controlTickSequence = ++_controlTickSequence;
    cycle.dtUs = static_cast<uint32_t>(cycle.controlTiming.controlStartUs - _lastControlMicros);
    _lastControlMicros = cycle.controlTiming.controlStartUs;

    cycle.controlTiming.encoderLatchUs = micros();
    cycle.driveTelemetry = _drive.GetTelemetry();
    cycle.controlTiming.encoderReadDoneUs = micros();
    const float dtSeconds = static_cast<float>(cycle.dtUs) * 1.0e-6f;
    cycle.sensorSnapshot = _sensors.Capture(
        stationary,
        _drive.GetPose(),
        [this, dtSeconds, &cycle](DiagnosticSensorSnapshot& captureSnapshot, auto&& serviceWallRead, auto&& captureImu) noexcept
        {
            _drive.UpdateOdometry(
                dtSeconds,
                captureSnapshot,
                nullptr,
                &cycle.controlTiming,
                [&serviceWallRead]() noexcept
                {
                    serviceWallRead();
                },
                [&captureImu]() noexcept
                {
                    captureImu();
                });
        });
    if (_drive.HasEstimatorFault())
    {
        cycle.estimatorFault = true;
        return false;
    }
    const DriveBase::MeasuredKinematics measuredKinematics = _drive.GetMeasuredKinematics(cycle.sensorSnapshot.gyroRadps);
    cycle.measuredLinearSpeedMps = measuredKinematics.linearSpeedMps;
    cycle.measuredAngularSpeedRadps = measuredKinematics.angularSpeedRadps;
    cycle.planarAccelMps2 = _sensors.GetPlanarAccelMps2(cycle.sensorSnapshot);
    cycle.batteryVoltage = ReadBatteryVoltage();
    cycle.boardTemperatureC = ReadBoardTemperatureC(cycle.sensorSnapshot);
    cycle.fanDutyCycle = GetMissionFanDutyCycle();
    cycle.workspaceViolation = HasWorkspaceViolationFault();
    return !cycle.workspaceViolation;
}

void OpenFloorMeasurementController::FinalizeCycle(OpenFloorMeasurementCycle& cycle)
{
    cycle.controlTiming.pwmLatchUs = micros();
    cycle.controlTiming.controlEndUs = cycle.controlTiming.pwmLatchUs;
    cycle.controlTiming.cycleCounterEnd = ReadCycleCounter();
}

bool OpenFloorMeasurementController::LogCycle(const OpenFloorMeasurementLabels& labels, OpenFloorMeasurementCycle& cycle)
{
    FinalizeCycle(cycle);
    if (!_mainLogOpen)
    {
        return true;
    }
    return LogMainSample(labels, cycle) ? true : Fail("Failed to write open-floor main sample");
}

bool OpenFloorMeasurementController::TraverseToMarker(
    OpenFloorMeasurementLabels& labels,
    MazeMap::OpenFloorMarkerId markerId)
{
    labels.startMarkerId = markerId;
    OpenFloorMeasurementLabels recoveryLabels = labels;
    recoveryLabels.primitiveId = MazeMap::OpenFloorPrimitiveId::Recovery;
    recoveryLabels.directionId = MazeMap::OpenFloorDirectionId::None;
    recoveryLabels.phaseId = MazeMap::OpenFloorPhaseId::Recovery;
    if (_mainLogOpen && !BeginMainSection(recoveryLabels))
    {
        return Fail("Failed to write recovery section start marker");
    }
    if (!RecoverToMarker(
        recoveryLabels,
        markerId,
        DiagnosticConfig::kCharacterizationRecoverySpeedMps,
        2500UL))
    {
        return false;
    }
    if (_mainLogOpen && !EndMainSection(recoveryLabels))
    {
        return Fail("Failed to write recovery section end marker");
    }
    return true;
}

bool OpenFloorMeasurementController::RecoverToMarker(
    OpenFloorMeasurementLabels& labels,
    MazeMap::OpenFloorMarkerId markerId,
    float maxSpeedMps,
    unsigned long timeoutMs)
{
    OpenFloorMeasurementLabels recoveryLabels = labels;
    recoveryLabels.primitiveId = MazeMap::OpenFloorPrimitiveId::Recovery;
    recoveryLabels.directionId = MazeMap::OpenFloorDirectionId::None;
    const MazeMap::OpenFloorMarkerPose& marker = MazeMap::GetOpenFloorMarker(markerId);
    const float targetX = MazeMap::OpenFloorMarkerXMeters(markerId);
    const float targetY = MazeMap::OpenFloorMarkerYMeters(markerId);
    const Eigen::Vector2f targetHeading = DirectionToUnitVector(marker.heading);
    const Eigen::Vector2f leftUnit(-targetHeading.y(), targetHeading.x());
    const unsigned long deadline = millis() + FailureTimeoutMs(timeoutMs);
    const PoseEstimate startPose = _drive.GetPose();
    const float initialLongitudinalError = std::fabs(
        ((targetX - startPose.xMeters) * targetHeading.x()) +
        ((targetY - startPose.yMeters) * targetHeading.y()));

    while (true)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(false, cycle))
        {
            return HandleMeasurementCaptureFault(recoveryLabels, cycle);
        }

        const PoseEstimate pose = _drive.GetPose();
        const float dx = targetX - pose.xMeters;
        const float dy = targetY - pose.yMeters;
        const float longitudinalErrorM = (dx * targetHeading.x()) + (dy * targetHeading.y());
        const float lateralErrorM = (dx * leftUnit.x()) + (dy * leftUnit.y());
        const float headingErrorRad = HeadingErrorRad(targetHeading, pose.headingUnit);
        recoveryLabels.phaseId = MazeMap::OpenFloorPhaseId::Recovery;
        recoveryLabels.progressNorm = (initialLongitudinalError > Config::kDistanceToleranceM) ?
            (std::clamp)(1.0f - (std::fabs(longitudinalErrorM) / initialLongitudinalError), 0.0f, 1.0f) :
            1.0f;

        if (std::fabs(longitudinalErrorM) <= Config::kDistanceToleranceM &&
            std::fabs(lateralErrorM) <= Config::kDistanceToleranceM &&
            std::fabs(pose.linearSpeedMps) <= Config::kSpeedToleranceMps &&
            std::fabs(pose.angularSpeedRadps) <= 0.25f)
        {
            _drive.Brake();
            return LogCycle(recoveryLabels, cycle);
        }

        if (static_cast<long>(deadline - millis()) <= 0)
        {
            return LogSectionFaultAndFail(
                recoveryLabels,
                cycle,
                MazeMap::OpenFloorFaultCode::RecoveryTimedOut,
                "Recovery to marker timed out",
                kWatchdogFlagRecoveryTimeout);
        }

        const float dtSeconds = static_cast<float>(cycle.dtUs) * 1.0e-6f;
        const float linearCommandMps = (std::clamp)(4.0f * longitudinalErrorM, -maxSpeedMps, maxSpeedMps);
        const float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * pose.angularSpeedRadps) -
            (3.0f * lateralErrorM);
        _drive.CommandVelocity(linearCommandMps, angularCommandRadps, dtSeconds);

        if (!LogCycle(recoveryLabels, cycle))
        {
            return false;
        }
    }
}

bool OpenFloorMeasurementController::RunTimingBlock()
{
    if (!BeginTimingLog())
    {
        return false;
    }
    _timingLogOpen = true;
    for (_timingTickIndex = 0U; _timingTickIndex < DiagnosticConfig::kTimingCaptureCycles; ++_timingTickIndex)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(true, cycle))
        {
            if (cycle.estimatorFault)
            {
                return LogTimingFaultAndFail(
                    cycle,
                    MazeMap::OpenFloorFaultCode::EstimatorFault,
                    "Estimator fault during timing capture");
            }
            if (cycle.workspaceViolation)
            {
                return LogTimingFaultAndFail(
                    cycle,
                    MazeMap::OpenFloorFaultCode::WorkspaceViolation,
                    "Workspace violation during timing capture");
            }
            return Fail("Open-floor timing capture failed");
        }

        _drive.Brake();
        FinalizeCycle(cycle);
        if (!LogTimingSample(cycle))
        {
            return Fail("Failed to write timing sample");
        }
    }
    return true;
}

bool OpenFloorMeasurementController::RunStaticSection()
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = MazeMap::OpenFloorSectionId::Sec10Static;
    labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    labels.primitiveId = MazeMap::OpenFloorPrimitiveId::StaticHold;
    labels.phaseId = MazeMap::OpenFloorPhaseId::Hold;
    labels.repeatIndex = 1U;
    if (!TraverseToMarker(labels, labels.startMarkerId))
    {
        return false;
    }
    if (!BeginMainSection(labels))
    {
        return Fail("Failed to write section start marker");
    }

    const unsigned long deadline = millis() + DiagnosticConfig::kStaticHoldMs;
    while (static_cast<long>(deadline - millis()) > 0)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(true, cycle))
        {
            return HandleMeasurementCaptureFault(labels, cycle);
        }
        _drive.Brake();
        if (!LogCycle(labels, cycle))
        {
            return false;
        }
    }

    return EndMainSection(labels);
}

bool OpenFloorMeasurementController::ExecuteLaunchPulse(float signedDriveCommand, uint16_t repeatIndex)
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
        return Fail("Failed to write section start marker");
    }

    const unsigned long pulseDeadline = millis() + MazeMap::kOpenFloorLaunchPulseMs;
    const float launchBoundM = MazeMap::OpenFloorHalfStepMeters() + Config::kDistanceToleranceM;
    unsigned long launchOutOfBoundsStartMs = 0UL;
    bool launchOutOfBoundsActive = false;
    while (static_cast<long>(pulseDeadline - millis()) > 0)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(false, cycle))
        {
            return HandleMeasurementCaptureFault(labels, cycle);
        }

        labels.phaseId = MazeMap::OpenFloorPhaseId::LaunchPulse;
        const PoseEstimate pose = _drive.GetPose();
        const float dx = pose.xMeters - MazeMap::OpenFloorMarkerXMeters(labels.startMarkerId);
        const float dy = pose.yMeters - MazeMap::OpenFloorMarkerYMeters(labels.startMarkerId);
        const bool launchOutOfBounds = std::sqrt((dx * dx) + (dy * dy)) > launchBoundM;
        if (!launchOutOfBounds)
        {
            launchOutOfBoundsStartMs = 0UL;
            launchOutOfBoundsActive = false;
        }
        else
        {
            const unsigned long nowMs = millis();
            if (!launchOutOfBoundsActive)
            {
                launchOutOfBoundsStartMs = nowMs;
                launchOutOfBoundsActive = true;
            }
            else if (MazeMap::HasOpenFloorOutOfBoundsGraceElapsed(launchOutOfBoundsStartMs, nowMs))
            {
                return LogSectionFaultAndFail(
                    labels,
                    cycle,
                    MazeMap::OpenFloorFaultCode::LaunchBoundExceeded,
                    "Launch bound exceeded");
            }
        }
        _drive.CommandOpenLoopRaw(signedDriveCommand, signedDriveCommand);

        if (!LogCycle(labels, cycle))
        {
            return false;
        }
    }

    _drive.Brake();
    if (!RecoverToMarker(labels, labels.startMarkerId, DiagnosticConfig::kCharacterizationRecoverySpeedMps, 2500UL))
    {
        return false;
    }
    return EndMainSection(labels);
}

bool OpenFloorMeasurementController::RunLaunchSection()
{
    OpenFloorMeasurementLabels transitionLabels{};
    transitionLabels.sectionId = MazeMap::OpenFloorSectionId::Sec20Launch;
    transitionLabels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    if (!TraverseToMarker(transitionLabels, transitionLabels.startMarkerId))
    {
        return false;
    }
    uint16_t repeatIndex = 0U;
    for (float magnitude : MazeMap::kOpenFloorLaunchDriveMagnitudes)
    {
        for (uint8_t repeat = 0U; repeat < DiagnosticConfig::kLaunchRepeatsPerMagnitude; ++repeat)
        {
            ++repeatIndex;
            if (!ExecuteLaunchPulse(magnitude, repeatIndex))
            {
                return false;
            }
            ++repeatIndex;
            if (!ExecuteLaunchPulse(-magnitude, repeatIndex))
            {
                return false;
            }
        }
    }
    return true;
}

bool OpenFloorMeasurementController::ExecuteStraightDistance(
    MazeMap::OpenFloorSectionId sectionId,
    MazeMap::OpenFloorMarkerId markerId,
    MazeMap::OpenFloorDirectionId directionId,
    float distanceM,
    float cruiseSpeedMps,
    uint16_t repeatIndex,
    MazeMap::OpenFloorSpeedBin speedBin,
    MazeMap::OpenFloorPrimitiveId primitiveId,
    bool emitSectionMarkers,
    bool snapToStartMarker)
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = sectionId;
    labels.startMarkerId = markerId;
    labels.directionId = directionId;
    labels.primitiveId = primitiveId;
    labels.repeatIndex = repeatIndex;
    labels.speedBin = speedBin;
    if (snapToStartMarker && !TraverseToMarker(labels, markerId))
    {
        return false;
    }
    if (emitSectionMarkers && !BeginMainSection(labels))
    {
        return Fail("Failed to write section start marker");
    }

    const MotionLimits limits = MeasurementLimits(cruiseSpeedMps);
    const float startDistanceM = _drive.GetAverageDistanceMeters();
    const Eigen::Vector2f targetHeading = _drive.GetPose().headingUnit;
    float commandedSpeedMps = 0.0f;
    EncoderProgressWatchdog translationWatchdog{};
    translationWatchdog.Reset(0.0f, millis());
    const unsigned long timeoutMs = millis() +
        FailureTimeoutMs(static_cast<unsigned long>(2500.0f + (6000.0f * distanceM)));

    while (true)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(false, cycle))
        {
            return HandleMeasurementCaptureFault(labels, cycle);
        }

        const float dtSeconds = static_cast<float>(cycle.dtUs) * 1.0e-6f;
        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const float remainingM = (std::max)(0.0f, distanceM - traveledM);
        labels.progressNorm = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
        labels.phaseId = StraightPhaseForProgress(labels.progressNorm);

        if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            _drive.Brake();
            if (!LogCycle(labels, cycle))
            {
                return false;
            }
            break;
        }
        if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
        {
            cycle.watchdogFlags |= kWatchdogFlagTranslationStall;
        }
        if (static_cast<long>(timeoutMs - millis()) <= 0)
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::StraightSectionTimedOut,
                "Straight section timed out",
                kWatchdogFlagSectionTimeout);
        }

        const float accelLimitedSpeedMps = (std::min)(cruiseSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
        const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
        commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);
        const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
        const float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
        _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

        if (!LogCycle(labels, cycle))
        {
            return false;
        }
    }

    return !emitSectionMarkers || EndMainSection(labels);
}

bool OpenFloorMeasurementController::RunStraightSection()
{
    uint16_t repeatIndex = 0U;
    const float straightDistanceM = MazeMap::OpenFloorStrEquivalentDistanceMeters(4U);
    for (size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size(); ++speedIndex)
    {
        const MazeMap::OpenFloorSpeedBin speedBin =
            (speedIndex == 0U) ? MazeMap::OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? MazeMap::OpenFloorSpeedBin::Medium :
            MazeMap::OpenFloorSpeedBin::High;
        for (uint8_t repeat = 0U; repeat < DiagnosticConfig::kStraightRepeatsPerSpeed; ++repeat)
        {
            ++repeatIndex;
            if (!ExecuteStraightDistance(
                    MazeMap::OpenFloorSectionId::Sec30Straight,
                    MazeMap::OpenFloorMarkerId::N,
                    MazeMap::OpenFloorDirectionId::Northbound,
                    straightDistanceM,
                    MazeMap::kOpenFloorStraightSpeedBinsMps[speedIndex],
                    repeatIndex,
                    speedBin,
                    MazeMap::OpenFloorPrimitiveId::Str4))
            {
                return false;
            }
            ++repeatIndex;
            if (!ExecuteStraightDistance(
                    MazeMap::OpenFloorSectionId::Sec30Straight,
                    MazeMap::OpenFloorMarkerId::S,
                    MazeMap::OpenFloorDirectionId::Southbound,
                    straightDistanceM,
                    MazeMap::kOpenFloorStraightSpeedBinsMps[speedIndex],
                    repeatIndex,
                    speedBin,
                    MazeMap::OpenFloorPrimitiveId::Str4))
            {
                return false;
            }
        }
    }
    return true;
}

bool OpenFloorMeasurementController::ExecuteInPlaceTurn(
    MazeMap::OpenFloorPrimitiveId primitiveId,
    MazeMap::OpenFloorDirectionId directionId,
    float angleRad,
    float maxOmegaRadps,
    uint16_t repeatIndex,
    MazeMap::OpenFloorSpeedBin speedBin,
    bool emitSectionMarkers,
    bool snapToStartMarker)
{
    OpenFloorMeasurementLabels labels{};
    labels.sectionId = MazeMap::OpenFloorSectionId::Sec40Yaw;
    labels.startMarkerId = MazeMap::OpenFloorMarkerId::C;
    labels.primitiveId = primitiveId;
    labels.directionId = directionId;
    labels.repeatIndex = repeatIndex;
    labels.speedBin = speedBin;
    if (snapToStartMarker && !TraverseToMarker(labels, labels.startMarkerId))
    {
        return false;
    }
    if (emitSectionMarkers && !BeginMainSection(labels))
    {
        return Fail("Failed to write section start marker");
    }

    MotionLimits limits = MeasurementLimits(0.0f);
    limits.maxAngularSpeedRadps = maxOmegaRadps;
    const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(limits);
    const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
    const float targetMagnitude = std::fabs(angleRad);
    float commandedOmegaRadps = 0.0f;
    const unsigned long timeoutMs = millis() + FailureTimeoutMs(3000UL);

    while (true)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(false, cycle))
        {
            return HandleMeasurementCaptureFault(labels, cycle);
        }

        const float dtSeconds = static_cast<float>(cycle.dtUs) * 1.0e-6f;
        const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        labels.progressNorm = (targetMagnitude > 0.0f) ?
            (std::clamp)(1.0f - (std::fabs(errorRad) / targetMagnitude), 0.0f, 1.0f) :
            1.0f;
        labels.phaseId = TurnPhaseForProgress(labels.progressNorm);

        if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
        {
            _drive.Brake();
            if (!LogCycle(labels, cycle))
            {
                return false;
            }
            break;
        }
        if (static_cast<long>(timeoutMs - millis()) <= 0)
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::YawSectionTimedOut,
                "Yaw section timed out",
                kWatchdogFlagSectionTimeout);
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                errorRad,
                _drive.GetPose().angularSpeedRadps,
                dtSeconds,
                turnProfile,
                commandedOmegaRadps,
                angularCommandRadps))
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::YawProfileInvalid,
                "Yaw profile became invalid");
        }
        _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);

        if (!LogCycle(labels, cycle))
        {
            return false;
        }
    }

    return !emitSectionMarkers || EndMainSection(labels);
}

bool OpenFloorMeasurementController::RunYawSection()
{
    uint16_t repeatIndex = 0U;
    for (size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size(); ++speedIndex)
    {
        const MazeMap::OpenFloorSpeedBin speedBin =
            (speedIndex == 0U) ? MazeMap::OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? MazeMap::OpenFloorSpeedBin::Medium :
            MazeMap::OpenFloorSpeedBin::High;
        for (uint8_t repeat = 0U; repeat < DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed; ++repeat)
        {
            ++repeatIndex;
            if (!ExecuteInPlaceTurn(
                    MazeMap::OpenFloorPrimitiveId::Ip90,
                    MazeMap::OpenFloorDirectionId::Clockwise,
                    HALF_PI_F,
                    MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex],
                    repeatIndex,
                    speedBin))
            {
                return false;
            }
            ++repeatIndex;
            if (!ExecuteInPlaceTurn(
                    MazeMap::OpenFloorPrimitiveId::Ip90M,
                    MazeMap::OpenFloorDirectionId::CounterClockwise,
                    -HALF_PI_F,
                    MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex],
                    repeatIndex,
                    speedBin))
            {
                return false;
            }
            ++repeatIndex;
            if (!ExecuteInPlaceTurn(
                    MazeMap::OpenFloorPrimitiveId::Ip180,
                    MazeMap::OpenFloorDirectionId::Flip,
                    PI_F,
                    MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex],
                    repeatIndex,
                    speedBin))
            {
                return false;
            }
        }
    }
    return true;
}

bool OpenFloorMeasurementController::ExecuteSmoothTurn(
    MazeMap::ManeuverCode code,
    float cruiseSpeed,
    uint16_t repeatIndex,
    MazeMap::OpenFloorSpeedBin speedBin)
{
    MazeMap::SmoothTurnExecutionProfile profile{};
    if (!TryGetSmoothTurnExecutionProfileMeters(code, profile))
    {
        return Fail("Smooth-turn geometry unavailable");
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
    if (labels.primitiveId == MazeMap::OpenFloorPrimitiveId::None)
    {
        return Fail("Smooth-turn primitive mapping unavailable");
    }
    if (!TraverseToMarker(labels, labels.startMarkerId))
    {
        return false;
    }
    if (!BeginMainSection(labels))
    {
        return Fail("Failed to write section start marker");
    }

    const float startDistanceM = _drive.GetAverageDistanceMeters();
    EncoderProgressWatchdog translationWatchdog{};
    translationWatchdog.Reset(0.0f, millis());
    const unsigned long timeoutMs = millis() +
        FailureTimeoutMs(static_cast<unsigned long>(2500.0f + (5000.0f * profile.totalDistance)));
    MotionLimits limits = MeasurementLimits(cruiseSpeed);
    MazeMap::SmoothTurnYawRateControllerState yawRateController{};

    while (true)
    {
        OpenFloorMeasurementCycle cycle{};
        if (!CaptureCycle(false, cycle))
        {
            return HandleMeasurementCaptureFault(labels, cycle);
        }
        const float dtSeconds = static_cast<float>(cycle.dtUs) * 1.0e-6f;
        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const float remainingM = (std::max)(0.0f, profile.totalDistance - traveledM);
        labels.progressNorm = (std::clamp)(traveledM / profile.totalDistance, 0.0f, 1.0f);
        labels.phaseId =
            (labels.progressNorm < 0.33f) ? MazeMap::OpenFloorPhaseId::Entry :
            (labels.progressNorm > 0.66f) ? MazeMap::OpenFloorPhaseId::Exit :
            MazeMap::OpenFloorPhaseId::Middle;

        if (remainingM <= Config::kDistanceToleranceM)
        {
            _drive.Brake();
            if (!LogCycle(labels, cycle))
            {
                return false;
            }
            break;
        }
        if (translationWatchdog.Stalled(traveledM, cruiseSpeed, remainingM, millis()))
        {
            cycle.watchdogFlags |= kWatchdogFlagTranslationStall;
        }
        if (static_cast<long>(timeoutMs - millis()) <= 0)
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::SmoothSectionTimedOut,
                "Smooth-turn section timed out",
                kWatchdogFlagSectionTimeout);
        }

        float yawOffsetRad = 0.0f;
        float nominalOmegaRadps = 0.0f;
        if (!MazeMap::TryComputeSmoothTurnTarget(profile, traveledM, cruiseSpeed, yawOffsetRad, nominalOmegaRadps))
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::SmoothTargetInvalid,
                "Smooth-turn target became invalid");
        }
        const float yawRateCorrectionRadps = MazeMap::ComputeSmoothTurnYawRatePdCorrection(
            nominalOmegaRadps,
            _drive.GetPose().angularSpeedRadps,
            dtSeconds,
            Config::kSmoothTurnYawRateKp,
            Config::kSmoothTurnYawRateKd,
            yawRateController);
        float angularCommandRadps = nominalOmegaRadps + yawRateCorrectionRadps;
        angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
        _drive.CommandVelocity(cruiseSpeed, angularCommandRadps, dtSeconds);

        if (!LogCycle(labels, cycle))
        {
            return false;
        }
    }

    return EndMainSection(labels);
}

bool OpenFloorMeasurementController::RunSmoothSection()
{
    static const MazeMap::ManeuverCode kSmoothCodes[] = {
        MazeMap::S45SS,
        MazeMap::S45SS_M,
        MazeMap::S90SS,
        MazeMap::S90SS_M,
        MazeMap::S135SS,
        MazeMap::S135SS_M,
    };

    uint16_t repeatIndex = 0U;
    for (size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size(); ++speedIndex)
    {
        const MazeMap::OpenFloorSpeedBin speedBin =
            (speedIndex == 0U) ? MazeMap::OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? MazeMap::OpenFloorSpeedBin::Medium :
            MazeMap::OpenFloorSpeedBin::High;
        for (MazeMap::ManeuverCode code : kSmoothCodes)
        {
            for (uint8_t repeat = 0U; repeat < DiagnosticConfig::kSmoothRepeatsPerPrimitiveSpeed; ++repeat)
            {
                ++repeatIndex;
                if (!ExecuteSmoothTurn(code, MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex], repeatIndex, speedBin))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

bool OpenFloorMeasurementController::RunLoopSection(bool clockwise)
{
    const MazeMap::OpenFloorMarkerId markerId = clockwise ? MazeMap::OpenFloorMarkerId::CW : MazeMap::OpenFloorMarkerId::CCW;
    const MazeMap::OpenFloorSectionId sectionId = clockwise ? MazeMap::OpenFloorSectionId::Sec60LoopCw : MazeMap::OpenFloorSectionId::Sec70LoopCcw;
    const MazeMap::OpenFloorPrimitiveId turnPrimitiveId =
        clockwise ? MazeMap::OpenFloorPrimitiveId::Ip90 : MazeMap::OpenFloorPrimitiveId::Ip90M;
    const MazeMap::OpenFloorDirectionId loopDirection =
        clockwise ? MazeMap::OpenFloorDirectionId::Clockwise : MazeMap::OpenFloorDirectionId::CounterClockwise;
    const float turnAngleRad = clockwise ? HALF_PI_F : -HALF_PI_F;

    for (uint16_t repeatIndex = 1U; repeatIndex <= DiagnosticConfig::kLoopRepeats; ++repeatIndex)
    {
        OpenFloorMeasurementLabels loopLabels{};
        loopLabels.sectionId = sectionId;
        loopLabels.startMarkerId = markerId;
        loopLabels.repeatIndex = repeatIndex;
        if (!TraverseToMarker(loopLabels, markerId))
        {
            return false;
        }
        if (!BeginMainSection(loopLabels))
        {
            return Fail("Failed to write section start marker");
        }

        for (uint8_t leg = 0U; leg < 4U; ++leg)
        {
            if (!ExecuteStraightDistance(
                    sectionId,
                    markerId,
                    loopDirection,
                    MazeMap::OpenFloorStrEquivalentDistanceMeters(2U),
                    MazeMap::kOpenFloorStraightSpeedBinsMps[0],
                    repeatIndex,
                    MazeMap::OpenFloorSpeedBin::Low,
                    MazeMap::OpenFloorPrimitiveId::Str2,
                    false,
                    false))
            {
                return false;
            }
            if (!ExecuteInPlaceTurn(
                    turnPrimitiveId,
                    loopDirection,
                    turnAngleRad,
                    MazeMap::kOpenFloorYawOmegaBinsRadps[1],
                    repeatIndex,
                    MazeMap::OpenFloorSpeedBin::Medium,
                    false,
                    false))
            {
                return false;
            }
        }

        if (!EndMainSection(loopLabels))
        {
            return Fail("Failed to write section end marker");
        }
    }
    return true;
}

namespace MazeMap::App::Internal
{
    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}

