#include "pch.h"
#include "TopSpeedMeasurementMode.h"

#include "BootUtilityModeFramework.h"
#include "BootModeRegistry.h"
#include "DiagnosticConfig.h"
#include "DriveBase.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapRuntimeSensors.h"
#include "MazeMapSharedRuntime.h"
#include "PinPairStrap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
    constexpr const char* kTopSpeedMeasurementStableId = "top_speed_measurement";
    constexpr const char* kTopSpeedMeasurementMainFileName = "top_speed_measurement_main.mmlog";
    constexpr const char* kTopSpeedMeasurementFormatVersion = "top_speed_measurement_rev_g";
    constexpr const char* kTopSpeedMeasurementDataStreamType = "top_speed_main";
    constexpr const char* kTopSpeedMeasurementTextStreamType = "top_speed_main_control_log";
    constexpr float kTopSpeedMeasurementForwardAccelerationMps2 = 8.5f;
    constexpr std::uint32_t kTopSpeedMeasurementAccelerationDurationUs = 1000000U;
    constexpr float kTopSpeedMeasurementReverseImpactThresholdMps2 = -6.0f;
    constexpr float kTopSpeedMeasurementGyroXySpikeThresholdDps = 150.0f;
    constexpr float kTopSpeedMeasurementImpactMinimumSpeedMps = 0.30f;
    constexpr float kTopSpeedMeasurementStoppedEncoderThresholdMps = Config::kWheelRestLaunchSpeedThresholdMps;
    constexpr std::uint8_t kTopSpeedMeasurementStoppedEncoderSettledTicks = 3U;

    float TopSpeedHeadingErrorRad(
        const float targetYawRad,
        const float measuredYawRad) noexcept
    {
        if (!(std::isfinite(targetYawRad) && std::isfinite(measuredYawRad)))
        {
            return 0.0f;
        }

        return
            HeadingErrorRad(
                HeadingUnitFromYawRad(targetYawRad),
                HeadingUnitFromYawRad(measuredYawRad));
    }

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
}

namespace MazeMap::App::Internal
{
    TopSpeedMeasurementMode::TopSpeedMeasurementMode(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
    {
        ResetRunState();
    }

    bool TopSpeedMeasurementMode::Begin()
    {
        ResetRunState();
        if (!_runtime.RegisterModeFaultHandler(&TopSpeedMeasurementMode::HandleRuntimeFault, this, kTopSpeedMeasurementStableId))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }

        ResetStartupTrace("mode:top_speed_measurement");
        (void)_runtime.AppendTextLogLine("Top speed measurement mode");
        (void)_runtime.AppendTextLogLine("Enter by shorting pins 26-27 at boot.");
        (void)_runtime.AppendTextLogLine(
            "This mode commands 8.5 m/s^2 straight ahead for up to 1 second while servoing back to the launch heading, or until strong reverse acceleration or an IMU X/Y gyro spike suggests impact, then brakes until wheel motion settles or the jumper is removed.");

        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }

        _drive.UseNominalWheelControlProfile();
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            return Fail("Top speed measurement sensor init failed");
        }

        const int runIdLength = std::snprintf(
            _runId,
            sizeof(_runId),
            "tsm_%lu",
            static_cast<unsigned long>(micros()));
        if (runIdLength <= 0 || runIdLength >= static_cast<int>(sizeof(_runId)))
        {
            return Fail("Top speed measurement run id generation failed");
        }

        _pinsLatchedAtBoot = MazeMap::App::IsBootModeSelectorActive(MazeMap::App::BootModeId::TopSpeedMeasurement);
        ConfigureSelectorMonitor();
        _batteryVoltageStart = ReadBatteryVoltage();
        _fanDutyCycleStart = GetMissionFanDutyCycle();
        if (!BeginLog())
        {
            return Fail("Top speed measurement log open failed");
        }
        if (!WriteRunStartEvent())
        {
            return Fail("Top speed measurement run metadata logging failed");
        }

        return true;
    }

    void TopSpeedMeasurementMode::Run()
    {
        if (_faulted)
        {
            _loopController.EndSession();
            return;
        }

        bool ok = true;
        std::uint32_t completedTicks = 0U;

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &TopSpeedMeasurementMode::ModeWorkThunk;
        callbacks.context = this;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            ok = Fail("Top speed measurement loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            completedTicks = result.tickCount;
            ok = (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
        }

        if (_logOpen)
        {
            const bool summaryOk = WriteResultSummary(completedTicks);
            ok = ok && summaryOk;
        }

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        ReleaseSelectorMonitor();
        if (ok)
        {
            AppendStartupTrace("top_speed_measurement:complete");
            (void)_runtime.AppendTextLogFormatted(
                "Top speed measurement complete, log saved to %s",
                kTopSpeedMeasurementMainFileName);
        }
        CloseLog();
        SetMissionLevelFanEnabled(false);
    }

    void TopSpeedMeasurementMode::HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context != nullptr)
        {
            static_cast<TopSpeedMeasurementMode*>(context)->OnRuntimeFault(reason);
        }
    }

    LoopController::ControlVector TopSpeedMeasurementMode::ModeWorkThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto* const self = static_cast<TopSpeedMeasurementMode*>(context);
        if (self == nullptr)
        {
            services.Fault("Top-speed measurement context was not installed");
            return LoopController::ControlVector::BrakeCommand();
        }

        return self->RunTick(state, services);
    }

    LoopController::SessionOptions TopSpeedMeasurementMode::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    bool TopSpeedMeasurementMode::BeginLog()
    {
        if (!_runtime.OpenUtilityDataLogFile(kTopSpeedMeasurementMainFileName))
        {
            return false;
        }

        if (!_runtime.WriteUtilityDataLogMetadata("mode", kTopSpeedMeasurementStableId)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", kTopSpeedMeasurementDataStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", kTopSpeedMeasurementFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("run_id", _runId)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("forward_accel_mps2", kTopSpeedMeasurementForwardAccelerationMps2, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned(
                "accel_duration_ms",
                static_cast<unsigned long>(kTopSpeedMeasurementAccelerationDurationUs / 1000U))) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("control_strategy", "direct_longitudinal_accel_request_with_heading_hold")) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat(
                "reverse_impact_threshold_mps2",
                kTopSpeedMeasurementReverseImpactThresholdMps2,
                3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat(
                "gyro_xy_spike_threshold_dps",
                kTopSpeedMeasurementGyroXySpikeThresholdDps,
                3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat(
                "encoder_stop_threshold_mps",
                kTopSpeedMeasurementStoppedEncoderThresholdMps,
                3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned(
                "encoder_stop_settled_ticks",
                static_cast<unsigned long>(kTopSpeedMeasurementStoppedEncoderSettledTicks))) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("wall_updates_enabled", "false")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("pins_26_27_shorted_at_boot", _pinsLatchedAtBoot ? "true" : "false")) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("battery_voltage_start", _batteryVoltageStart, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("fan_duty_cycle_start", _fanDutyCycleStart, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;
        {
            const unsigned long imuSampleRateHz =
                MazeMap::GetUiImuSampleRateHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if ((imuSampleRateHz > 0UL) &&
                !_runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz))
            {
                return false;
            }
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                DiagnosticConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if ((imuAccelLpf2CutoffHz > 0.0f) &&
                !_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3))
            {
                return false;
            }
        }
        {
            const float imuGyroLpf1ReferenceHz =
                MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if ((imuGyroLpf1ReferenceHz > 0.0f) &&
                !_runtime.WriteUtilityDataLogMetadataFloat(
                    "imu_gyro_lpf1_cut213_datasheet_ref_hz",
                    imuGyroLpf1ReferenceHz,
                    3))
            {
                return false;
            }
        }

        TopSpeedMeasurementRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return false;
        if (!_runtime.WriteTextLogMetadata("data_file", kTopSpeedMeasurementMainFileName)) return false;
        if (!_runtime.WriteTextLogMetadata("mode", kTopSpeedMeasurementStableId)) return false;
        if (!_runtime.WriteTextLogMetadata("stream_type", kTopSpeedMeasurementTextStreamType)) return false;
        if (!_runtime.WriteTextLogMetadata("format_version", kTopSpeedMeasurementFormatVersion)) return false;
        if (!_runtime.WriteTextLogMetadata("wall_updates_enabled", "false")) return false;
        if (!_runtime.WriteTextLogMetadata("run_id", _runId)) return false;

        _logOpen = true;
        return true;
    }

    void TopSpeedMeasurementMode::CloseLog() noexcept
    {
        if (_logOpen)
        {
            (void)_runtime.CloseUtilityDataLog();
        }
        _logOpen = false;
    }

    bool TopSpeedMeasurementMode::WriteEvent(const char* type, const char* message)
    {
        return _runtime.WriteTextLogEntry(micros(), type, message);
    }

    bool TopSpeedMeasurementMode::WriteRunStartEvent()
    {
        char message[384] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "run_id=%s;pins_26_27_shorted_at_boot=%u;battery_voltage_start=%.3f;fan_duty_cycle_start=%.3f;forward_accel_mps2=%.3f;accel_duration_ms=%lu;reverse_impact_threshold_mps2=%.3f;gyro_xy_spike_threshold_dps=%.3f",
            _runId,
            _pinsLatchedAtBoot ? 1U : 0U,
            _batteryVoltageStart,
            _fanDutyCycleStart,
            kTopSpeedMeasurementForwardAccelerationMps2,
            static_cast<unsigned long>(kTopSpeedMeasurementAccelerationDurationUs / 1000U),
            kTopSpeedMeasurementReverseImpactThresholdMps2,
            kTopSpeedMeasurementGyroXySpikeThresholdDps);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }

        return
            WriteEvent(
                "summary",
                "Top-speed measurement runs straight ahead with LoopController at 8.5 m/s^2 for up to 1 second while servoing back to the launch heading, then transitions to braking on timer expiry, strong reverse acceleration, an IMU X/Y gyro spike, or selector removal.") &&
            WriteEvent(
                "summary",
                "This mode logs a mode-specific telemetry row and disables wall-sensor estimator updates while still capturing the diagnostic sensor snapshot.") &&
            WriteEvent("run_start", message);
    }

    bool TopSpeedMeasurementMode::WriteResultSummary(const std::uint32_t completedTicks)
    {
        const char* const completionReasonName =
            (_completionReason == CompletionReason::SettledStop) ? "settled_stop" :
            (_completionReason == CompletionReason::SelectorRemoved) ? "selector_removed" :
            "none";
        const char* const brakeTriggerName =
            (_brakeTrigger == BrakeTrigger::TimedWindowElapsed) ? "timed_window_elapsed" :
            (_brakeTrigger == BrakeTrigger::ImpactDetected) ? "impact_detected" :
            (_brakeTrigger == BrakeTrigger::GyroSpikeDetected) ? "gyro_xy_spike_detected" :
            (_brakeTrigger == BrakeTrigger::SelectorRemoved) ? "selector_removed" :
            "none";
        char message[384] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "completion_reason=%s;brake_trigger=%s;impact_detected=%u;peak_measured_speed_mps=%.6f;peak_planar_accel_mps2=%.6f;peak_heading_error_deg=%.3f;most_negative_forward_accel_mps2=%.6f;impact_speed_mps=%.6f;impact_forward_accel_mps2=%.6f;impact_imu_gyro_x_lsb=%d;impact_imu_gyro_y_lsb=%d;completed_ticks=%lu",
            completionReasonName,
            brakeTriggerName,
            _impactDetected ? 1U : 0U,
            _peakMeasuredSpeedMps,
            _peakPlanarAccelMps2,
            RAD_TO_DEG_F * _peakHeadingErrorRad,
            _mostNegativeForwardAccelMps2,
            _impactSpeedMps,
            _impactForwardAccelMps2,
            static_cast<int>(_impactGyroXRawLsb),
            static_cast<int>(_impactGyroYRawLsb),
            static_cast<unsigned long>(completedTicks));
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }

        return WriteEvent("result", message);
    }

    bool TopSpeedMeasurementMode::LogSample(const LoopController::ModeState& state)
    {
        if (!_logOpen)
        {
            return false;
        }

        const DiagnosticSensorSnapshot& snapshot = state.diagnosticSensors;
        const MazeMap::VehicleState::StateVector& estimatorState = _drive.GetEstimatorStateVector();
        TopSpeedMeasurementRow row{};
        row.master_time_us = state.tickStartUs;
        row.control_tick_sequence = ++_controlTickSequence;
        row.dt_us = state.dtUs;
        row.elapsed_test_us = state.tickStartUs - _measurementStartUs;
        row.braking = (_phase == RunPhase::Braking) ? 1U : 0U;
        row.impact_detected = _impactDetected ? 1U : 0U;
        row.selector_removed = SelectorRemoved() ? 1U : 0U;
        row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        row.measured_linear_speed_mps = state.measured.linearSpeedMps;
        row.measured_angular_speed_radps = state.measured.angularSpeedRadps;
        row.cmd_linear_mps = _lastCommandInputLinearSpeedMps;
        row.cmd_angular_radps = _lastCommandInputAngularRateRadps;
        row.heading_error_rad = _lastHeadingErrorRad;
        row.cmd_linear_accel_mps2 =
            (_phase == RunPhase::Accelerating) ? kTopSpeedMeasurementForwardAccelerationMps2 : 0.0f;
        row.left_drive_command = state.driveTelemetry.leftDriveCommand;
        row.right_drive_command = state.driveTelemetry.rightDriveCommand;
        row.left_feedforward_command = state.driveTelemetry.leftFeedforwardCommand;
        row.right_feedforward_command = state.driveTelemetry.rightFeedforwardCommand;
        row.left_feedback_command = state.driveTelemetry.leftFeedbackCommand;
        row.right_feedback_command = state.driveTelemetry.rightFeedbackCommand;
        row.left_target_velocity_mps = state.driveTelemetry.leftTargetVelocityMps;
        row.right_target_velocity_mps = state.driveTelemetry.rightTargetVelocityMps;
        row.left_launch_assist_floor = state.driveTelemetry.leftLaunchAssistFloor;
        row.right_launch_assist_floor = state.driveTelemetry.rightLaunchAssistFloor;
        row.left_encoder_count = state.driveTelemetry.leftEncoderCount;
        row.right_encoder_count = state.driveTelemetry.rightEncoderCount;
        row.left_encoder_omega_radps = state.driveTelemetry.leftEncoderOmegaRadps;
        row.right_encoder_omega_radps = state.driveTelemetry.rightEncoderOmegaRadps;
        row.left_encoder_distance_m = state.driveTelemetry.leftDistanceM;
        row.right_encoder_distance_m = state.driveTelemetry.rightDistanceM;
        row.left_encoder_velocity_mps = state.driveTelemetry.leftVelocityMps;
        row.right_encoder_velocity_mps = state.driveTelemetry.rightVelocityMps;
        row.imu_timestamp_us = snapshot.imuTiming.readDoneUs;
        row.imu_status = snapshot.imuBackLeft.status;
        row.imu_interrupt_high = snapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        row.accel_bias_valid = snapshot.accelBiasValid ? 1U : 0U;
        row.imu_gyro_x = snapshot.imuBackLeft.gyroX;
        row.imu_gyro_y = snapshot.imuBackLeft.gyroY;
        row.imu_gyro_z = snapshot.imuBackLeft.gyroZ;
        row.imu_accel_x = snapshot.imuBackLeft.accelX;
        row.imu_accel_y = snapshot.imuBackLeft.accelY;
        row.imu_accel_z = snapshot.imuBackLeft.accelZ;
        row.imu_temp = snapshot.imuBackLeft.temp;
        row.gyro_raw_radps = snapshot.gyroRawRadps;
        row.gyro_bias_radps = snapshot.gyroBiasRadps;
        row.gyro_radps = snapshot.gyroRadps;
        row.accel_body_x_mps2 = snapshot.accelBodyXMps2;
        row.accel_body_y_mps2 = snapshot.accelBodyYMps2;
        row.planar_accel_mps2 = state.sensors.planarAccelMps2;
        row.front_timestamp_us = snapshot.frontTiming.observationReadyUs;
        row.left_timestamp_us = snapshot.leftTiming.observationReadyUs;
        row.right_timestamp_us = snapshot.rightTiming.observationReadyUs;
        row.front_left_wall_distance_m = snapshot.frontLeft.distanceM;
        row.front_right_wall_distance_m = snapshot.frontRight.distanceM;
        row.side_left_wall_distance_m = snapshot.sideLeft.distanceM;
        row.side_right_wall_distance_m = snapshot.sideRight.distanceM;
        row.front_left_differential_light = snapshot.frontLeft.differentialLight;
        row.front_right_differential_light = snapshot.frontRight.differentialLight;
        row.side_left_differential_light = snapshot.sideLeft.differentialLight;
        row.side_right_differential_light = snapshot.sideRight.differentialLight;
        row.fan_duty_cycle = GetMissionFanDutyCycle();
        return _runtime.LogUtilityDataRow(row);
    }

    bool TopSpeedMeasurementMode::EnterBrakingPhase(
        const BrakeTrigger trigger,
        const char* const eventMessage,
        LoopController::TickServices& services)
    {
        if (_phase == RunPhase::Braking)
        {
            return true;
        }

        _phase = RunPhase::Braking;
        _brakeTrigger = trigger;
        _settledEncoderTicks = 0U;
        if (eventMessage != nullptr && eventMessage[0] != '\0' && !WriteEvent("transition", eventMessage))
        {
            services.Fault("Top-speed measurement transition logging failed");
            return false;
        }

        return true;
    }

    bool TopSpeedMeasurementMode::SelectorRemoved() const noexcept
    {
        return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
    }

    void TopSpeedMeasurementMode::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        if (!_pinsLatchedAtBoot)
        {
            return;
        }

        const MazeMap::App::BootModeRegistryEntry* const entry =
            MazeMap::App::FindBootModeRegistryEntry(MazeMap::App::BootModeId::TopSpeedMeasurement);
        if (entry == nullptr || entry->selector.kind != MazeMap::App::BootModeSelectorKind::PinPair)
        {
            return;
        }

        _selectorDrivePin = entry->selector.pinA;
        _selectorSensePin = entry->selector.pinB;
        BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        _selectorMonitorArmed = true;
    }

    void TopSpeedMeasurementMode::ReleaseSelectorMonitor() noexcept
    {
        if (_selectorMonitorArmed)
        {
            EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        }

        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
        _selectorMonitorArmed = false;
    }

    bool TopSpeedMeasurementMode::ImpactDetected(const LoopController::ModeState& state) const noexcept
    {
        const float forwardAccelMps2 = state.diagnosticSensors.accelBodyYMps2;
        return state.diagnosticSensors.accelBiasValid &&
            std::isfinite(forwardAccelMps2) &&
            std::isfinite(state.measured.linearSpeedMps) &&
            (state.measured.linearSpeedMps >= kTopSpeedMeasurementImpactMinimumSpeedMps) &&
            (forwardAccelMps2 <= kTopSpeedMeasurementReverseImpactThresholdMps2);
    }

    bool TopSpeedMeasurementMode::GyroSpikeDetected(const LoopController::ModeState& state) const noexcept
    {
        const float gyroMdpsPerLsb = _sensors.GetGyroSensitivityMdpsPerLsb();
        if (!std::isfinite(gyroMdpsPerLsb) || (gyroMdpsPerLsb <= 0.0f))
        {
            return false;
        }

        const float gyroDpsPerLsb = gyroMdpsPerLsb / 1000.0f;
        const float absGyroXDps = std::fabs(static_cast<float>(state.diagnosticSensors.imuBackLeft.gyroX)) * gyroDpsPerLsb;
        const float absGyroYDps = std::fabs(static_cast<float>(state.diagnosticSensors.imuBackLeft.gyroY)) * gyroDpsPerLsb;
        return (absGyroXDps >= kTopSpeedMeasurementGyroXySpikeThresholdDps) ||
            (absGyroYDps >= kTopSpeedMeasurementGyroXySpikeThresholdDps);
    }

    bool TopSpeedMeasurementMode::EncoderMotionSettled(const LoopController::ModeState& state) noexcept
    {
        const bool leftStopped =
            std::isfinite(state.driveTelemetry.leftVelocityMps) &&
            (std::fabs(state.driveTelemetry.leftVelocityMps) <= kTopSpeedMeasurementStoppedEncoderThresholdMps);
        const bool rightStopped =
            std::isfinite(state.driveTelemetry.rightVelocityMps) &&
            (std::fabs(state.driveTelemetry.rightVelocityMps) <= kTopSpeedMeasurementStoppedEncoderThresholdMps);
        if (leftStopped && rightStopped)
        {
            if (_settledEncoderTicks < 0xFFU)
            {
                ++_settledEncoderTicks;
            }
        }
        else
        {
            _settledEncoderTicks = 0U;
        }

        return _settledEncoderTicks >= kTopSpeedMeasurementStoppedEncoderSettledTicks;
    }

    void TopSpeedMeasurementMode::UpdatePeaks(const LoopController::ModeState& state) noexcept
    {
        if (std::isfinite(state.measured.linearSpeedMps))
        {
            _peakMeasuredSpeedMps = (std::max)(_peakMeasuredSpeedMps, state.measured.linearSpeedMps);
        }
        if (std::isfinite(state.sensors.planarAccelMps2))
        {
            _peakPlanarAccelMps2 = (std::max)(_peakPlanarAccelMps2, state.sensors.planarAccelMps2);
        }
        if (std::isfinite(state.estimate.yawRad))
        {
            _peakHeadingErrorRad = (std::max)(
                _peakHeadingErrorRad,
                std::fabs(TopSpeedHeadingErrorRad(_targetYawRad, state.estimate.yawRad)));
        }
        if (std::isfinite(state.diagnosticSensors.accelBodyYMps2))
        {
            _mostNegativeForwardAccelMps2 = (std::min)(_mostNegativeForwardAccelMps2, state.diagnosticSensors.accelBodyYMps2);
        }
    }

    LoopController::ControlVector TopSpeedMeasurementMode::RunTick(
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        if (_measurementStartUs == 0U)
        {
            _measurementStartUs = state.tickStartUs;
            _targetYawRad = state.estimate.yawRad;
            _lastHeadingErrorRad = 0.0f;
            SetLastCommandInputs(0.0f, 0.0f);
            if (!WriteEvent("transition", "state=accelerating"))
            {
                services.Fault("Top-speed measurement start logging failed");
                return LoopController::ControlVector::BrakeCommand();
            }
        }
        else
        {
            _lastHeadingErrorRad =
                std::isfinite(state.estimate.yawRad) ?
                TopSpeedHeadingErrorRad(_targetYawRad, state.estimate.yawRad) :
                0.0f;
        }

        if (!LogSample(state))
        {
            services.Fault("Top-speed measurement log write failed");
            return LoopController::ControlVector::BrakeCommand();
        }

        UpdatePeaks(state);
        if (!state.estimatorHealthy)
        {
            services.Fault(
                (state.faultReason != nullptr) ?
                    state.faultReason :
                    "Top-speed measurement estimator fault");
            return LoopController::ControlVector::BrakeCommand();
        }

        const bool selectorRemoved = SelectorRemoved();
        if (_phase == RunPhase::Accelerating)
        {
            if (selectorRemoved)
            {
                (void)EnterBrakingPhase(
                    BrakeTrigger::SelectorRemoved,
                    "state=braking;trigger=selector_removed",
                    services);
                SetLastCommandInputs(0.0f, 0.0f);
                return LoopController::ControlVector::BrakeCommand();
            }

            if (ImpactDetected(state))
            {
                _impactDetected = true;
                _impactSpeedMps = state.measured.linearSpeedMps;
                _impactForwardAccelMps2 = state.diagnosticSensors.accelBodyYMps2;
                _impactGyroXRawLsb = state.diagnosticSensors.imuBackLeft.gyroX;
                _impactGyroYRawLsb = state.diagnosticSensors.imuBackLeft.gyroY;
                (void)EnterBrakingPhase(
                    BrakeTrigger::ImpactDetected,
                    "state=braking;trigger=impact_detected",
                    services);
                SetLastCommandInputs(0.0f, 0.0f);
                return LoopController::ControlVector::BrakeCommand();
            }

            const std::uint32_t elapsedUs = state.tickStartUs - _measurementStartUs;
            if ((elapsedUs > 0U) && GyroSpikeDetected(state))
            {
                _impactDetected = true;
                _impactSpeedMps = state.measured.linearSpeedMps;
                _impactForwardAccelMps2 = state.diagnosticSensors.accelBodyYMps2;
                _impactGyroXRawLsb = state.diagnosticSensors.imuBackLeft.gyroX;
                _impactGyroYRawLsb = state.diagnosticSensors.imuBackLeft.gyroY;
                (void)EnterBrakingPhase(
                    BrakeTrigger::GyroSpikeDetected,
                    "state=braking;trigger=gyro_xy_spike_detected",
                    services);
                SetLastCommandInputs(0.0f, 0.0f);
                return LoopController::ControlVector::BrakeCommand();
            }

            const std::uint32_t nextCommandElapsedUs = elapsedUs + state.dtUs;
            if (nextCommandElapsedUs >= kTopSpeedMeasurementAccelerationDurationUs)
            {
                (void)EnterBrakingPhase(
                    BrakeTrigger::TimedWindowElapsed,
                    "state=braking;trigger=timed_window_elapsed",
                    services);
                SetLastCommandInputs(0.0f, 0.0f);
                return LoopController::ControlVector::BrakeCommand();
            }

            float commandedAngularRateRadps = 0.0f;
            const MazeMap::OpenLoopDriveCommand driveCommand = _drive.ResolveStraightHeadingHoldAccelerationDriveCommandRaw(
                _targetYawRad,
                state.estimate.yawRad,
                state.measured.linearSpeedMps,
                state.estimate.angularSpeedRadps,
                kTopSpeedMeasurementForwardAccelerationMps2,
                state.dtSeconds,
                &commandedAngularRateRadps);
            SetLastCommandInputs(state.measured.linearSpeedMps, commandedAngularRateRadps);
            return LoopController::ControlVector::OpenLoopCommand(
                driveCommand.leftDriveCommand,
                driveCommand.rightDriveCommand);
        }

        if (selectorRemoved)
        {
            _completionReason = CompletionReason::SelectorRemoved;
            services.RequestEndLoop();
            SetLastCommandInputs(0.0f, 0.0f);
            return LoopController::ControlVector::BrakeCommand();
        }

        if (EncoderMotionSettled(state))
        {
            _completionReason = CompletionReason::SettledStop;
            services.RequestEndLoop();
            SetLastCommandInputs(0.0f, 0.0f);
            return LoopController::ControlVector::BrakeCommand();
        }

        SetLastCommandInputs(0.0f, 0.0f);
        return LoopController::ControlVector::BrakeCommand();
    }

    bool TopSpeedMeasurementMode::Fail(const char* reason)
    {
        return _runtime.FailActiveMode(reason);
    }

    void TopSpeedMeasurementMode::OnRuntimeFault(const char* reason) noexcept
    {
        _faulted = true;
        _logOpen = false;
        ReleaseSelectorMonitor();
        AppendStartupTrace((reason != nullptr && reason[0] != '\0') ? reason : "top_speed_measurement_fault");
        if (reason != nullptr && reason[0] != '\0')
        {
            (void)WriteEvent("fault", reason);
        }
    }

    void TopSpeedMeasurementMode::ResetRunState() noexcept
    {
        ReleaseSelectorMonitor();
        _faulted = false;
        _logOpen = false;
        _pinsLatchedAtBoot = false;
        _phase = RunPhase::Accelerating;
        _brakeTrigger = BrakeTrigger::None;
        _completionReason = CompletionReason::None;
        _measurementStartUs = 0U;
        _controlTickSequence = 0U;
        _batteryVoltageStart = 0.0f;
        _fanDutyCycleStart = 0.0f;
        _peakMeasuredSpeedMps = 0.0f;
        _peakPlanarAccelMps2 = 0.0f;
        _peakHeadingErrorRad = 0.0f;
        _mostNegativeForwardAccelMps2 = 0.0f;
        _impactDetected = false;
        _impactSpeedMps = 0.0f;
        _impactForwardAccelMps2 = 0.0f;
        _targetYawRad = 0.0f;
        _lastHeadingErrorRad = 0.0f;
        _impactGyroXRawLsb = 0;
        _impactGyroYRawLsb = 0;
        _lastCommandInputLinearSpeedMps = 0.0f;
        _lastCommandInputAngularRateRadps = 0.0f;
        _settledEncoderTicks = 0U;
        _runId[0] = '\0';
    }

    void TopSpeedMeasurementMode::SetLastCommandInputs(
        const float linearSpeedMps,
        const float angularRateRadps) noexcept
    {
        _lastCommandInputLinearSpeedMps = linearSpeedMps;
        _lastCommandInputAngularRateRadps = angularRateRadps;
    }

    float TopSpeedMeasurementMode::ReadBatteryVoltage() const noexcept
    {
        return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
    }

    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::TopSpeedMeasurement,
            BootModeCategory::Utility,
            "top_speed_measurement",
            "Measure straight-line peak speed with a fixed forward-acceleration launch and braking recovery.",
            "logging.txt; top-speed telemetry mmlog",
            "GetTopSpeedMeasurementMode",
            "TopSpeedMeasurementMode.cpp",
            "startup; accelerated straight run; impact-or-gyro-or-timer brake transition; brake-to-stop or jumper-exit",
            "DiagnosticConfig control period; shared mission drive and IMU tuning; LoopController diagnostic capture path",
            "Fixed 8.5 m/s^2 forward acceleration target for 1 second with launch-heading servo hold; reverse-accel and IMU X/Y gyro-spike impact braking; wall updates disabled; no wall-based correction",
            "top_speed_measurement_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetTopSpeedMeasurementMode()
    {
        static TopSpeedMeasurementMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
