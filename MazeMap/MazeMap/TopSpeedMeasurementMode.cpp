#include "pch.h"
#include "TopSpeedMeasurementMode.h"

#include "BootUtilityModeFramework.h"
#include "BootModeRegistry.h"
#include "DiagnosticConfig.h"
#include "DriveBase.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
    constexpr const char* kTopSpeedMeasurementStableId = "top_speed_measurement";
    constexpr const char* kTopSpeedMeasurementMainFileName = "top_speed_measurement_main.mmlog";
    constexpr float kTopSpeedMeasurementForwardAccelerationMps2 = 3.0f;
    constexpr std::uint32_t kTopSpeedMeasurementAccelerationDurationUs = 6000000U;
    constexpr std::uint32_t kTopSpeedMeasurementPrelaunchWaitUs = 1000000U;
    constexpr float kTopSpeedMeasurementPrelaunchReverseDriveCommand = -0.4f;
    constexpr std::uint32_t kTopSpeedMeasurementPrelaunchReverseDurationUs = 50000U;
    constexpr std::uint32_t kTopSpeedMeasurementBrakeTriggerArmDelayUs = 50000U;
    constexpr const char* kTopSpeedMeasurementSelectorRemovedFaultReason =
        "Top-speed measurement selector jumper removed";
    constexpr float kTopSpeedMeasurementReverseImpactThresholdMps2 = -12.0f;
    constexpr float kTopSpeedMeasurementGyroXySpikeThresholdDps = 650.0f;
    constexpr float kTopSpeedMeasurementImpactMinimumSpeedMps = 2.50f;
    constexpr float kTopSpeedMeasurementStoppedEncoderThresholdMps = Config::kWheelRestLaunchSpeedThresholdMps;
    constexpr std::uint8_t kTopSpeedMeasurementStoppedEncoderSettledTicks = 3U;

    float ComputeHeadingDeviationRad(
        const std::uint32_t measurementStartUs,
        const float measurementStartYawRad,
        const float currentYawRad) noexcept
    {
        if ((measurementStartUs == 0U) ||
            !std::isfinite(measurementStartYawRad) ||
            !std::isfinite(currentYawRad))
        {
            return 0.0f;
        }

        return MazeMap::VehicleState::NormalizeAngle(currentYawRad - measurementStartYawRad);
    }

    float ResolveBiasCorrectedGyroYawRateRadps(const SensorSnapshot& snapshot) noexcept
    {
        if (std::isfinite(snapshot.gyroRadps))
        {
            return snapshot.gyroRadps;
        }

        if (std::isfinite(snapshot.gyroRawRadps) && std::isfinite(snapshot.gyroBiasRadps))
        {
            return snapshot.gyroRawRadps - snapshot.gyroBiasRadps;
        }

        return 0.0f;
    }

    float ResolveGyroBiasHoldYawRateCommandRadps(
        const SensorSnapshot& snapshot,
        const float dtSeconds,
        MazeMap::SmoothTurnYawRateControllerState& controllerState) noexcept
    {
        const float measuredYawRateRadps = ResolveBiasCorrectedGyroYawRateRadps(snapshot);
        const float yawRateCommandRadps =
            MazeMap::ComputeSmoothTurnYawRatePdCorrection(
                0.0f,
                measuredYawRateRadps,
                dtSeconds,
                Config::kSmoothTurnYawRateKp,
                Config::kSmoothTurnYawRateKd,
                controllerState);
        return std::isfinite(yawRateCommandRadps) ? yawRateCommandRadps : 0.0f;
    }
}

namespace MazeMap::App::Internal
{
    TopSpeedMeasurementMode::TopSpeedMeasurementMode(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _sensors(runtime.Sensors())
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
            "1 s wait, 10 ms -0.4 reverse kick, then 8.5 m/s^2 launch with local gyro-z bias hold until timeout or impact from reverse accel / IMU X-Y gyro spikes. Then brake to rest. Jumper removal faults the run.");

        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }

        _drive.UseNominalWheelControlProfile();
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        _pinsLatchedAtBoot = MazeMap::App::IsBootModeSelectorActive(MazeMap::App::BootModeId::TopSpeedMeasurement);
        ConfigureSelectorMonitor();
        if (!EnsureSelectorStillPresent())
        {
            return false;
        }
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            return Fail("Top speed measurement sensor init failed");
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
        }
        if (!FlushTextLogStep("startup text flush"))
        {
            return false;
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
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

        _batteryVoltageStart = ReadBatteryVoltage();
        _fanDutyCycleStart = GetMissionFanDutyCycle();
        if (!BeginLog())
        {
            return FailLogSetupStep("log open");
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
        }
        if (!FlushTextLogStep("post-schema text flush"))
        {
            return false;
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
        }
        if (!WriteRunStartEvent())
        {
            return FailLogSetupStep("run metadata logging");
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
        }
        if (!FlushTextLogStep("preloop text flush"))
        {
            return false;
        }
        if (!EnsureSelectorStillPresent())
        {
            return false;
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
            return LoopController::ControlVector::Brake;
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

        _logRow = {};
        if (!_runtime.BeginUtilityDataLogSchema(_logRow))
        {
            return false;
        }

        _logOpen = true;
        return true;
    }

    bool TopSpeedMeasurementMode::FailLogSetupStep(const char* step)
    {
        char message[256] = {};
        const char* const detail = _runtime.LastRuntimeLogError();
        const bool hasDetail = (detail != nullptr) && (detail[0] != '\0');
        const int length = std::snprintf(
            message,
            sizeof(message),
            "Top speed measurement %s failed%s%s",
            (step != nullptr && step[0] != '\0') ? step : "log step",
            hasDetail ? ": " : "",
            hasDetail ? detail : "");
        if (length > 0 && length < static_cast<int>(sizeof(message)))
        {
            return Fail(message);
        }

        return Fail("Top speed measurement log setup failed");
    }

    bool TopSpeedMeasurementMode::FlushTextLogStep(const char* step)
    {
        _runtime.FlushTextLog();
        const char* const detail = _runtime.LastRuntimeLogError();
        return ((detail == nullptr) || (detail[0] == '\0')) ? true : FailLogSetupStep(step);
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
        char message[256] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "run=%s;pins=%u;bat0=%.3f;fan0=%.3f;launch_a=%.3f;launch_ms=%lu;wait_ms=%lu;rev_cmd=%.3f;rev_ms=%lu;impact_a=%.3f;gxy_spike_dps=%.3f;gz_hold=1;kp=%.3f;kd=%.6f",
            _runId,
            _pinsLatchedAtBoot ? 1U : 0U,
            _batteryVoltageStart,
            _fanDutyCycleStart,
            kTopSpeedMeasurementForwardAccelerationMps2,
            static_cast<unsigned long>(kTopSpeedMeasurementAccelerationDurationUs / 1000U),
            static_cast<unsigned long>(kTopSpeedMeasurementPrelaunchWaitUs / 1000U),
            kTopSpeedMeasurementPrelaunchReverseDriveCommand,
            static_cast<unsigned long>(kTopSpeedMeasurementPrelaunchReverseDurationUs / 1000U),
            kTopSpeedMeasurementReverseImpactThresholdMps2,
            kTopSpeedMeasurementGyroXySpikeThresholdDps,
            Config::kSmoothTurnYawRateKp,
            Config::kSmoothTurnYawRateKd);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }

        return WriteEvent("run_start", message);
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
        char message[256] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "done=%s;brake=%s;impact=%u;peak_v=%.6f;peak_pa=%.6f;peak_head_deg=%.3f;min_ax=%.6f;impact_v=%.6f;impact_ax=%.6f;impact_gx=%d;impact_gy=%d;ticks=%lu",
            completionReasonName,
            brakeTriggerName,
            _impactDetected ? 1U : 0U,
            _peakMeasuredSpeedMps,
            _peakPlanarAccelMps2,
            _peakHeadingDeviationRad * RAD_TO_DEG_F,
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
        SetLastCommandInputs(0.0f, 0.0f);
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

    bool TopSpeedMeasurementMode::EnsureSelectorStillPresent()
    {
        if (!SelectorRemoved())
        {
            return true;
        }

        return Fail(kTopSpeedMeasurementSelectorRemovedFaultReason);
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
        const float forwardAccelMps2 = state.sensors.accelBodyYMps2;
        return state.sensors.accelBiasValid &&
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
        const float absGyroXDps = std::fabs(static_cast<float>(state.sensors.imuBackLeft.gyroX)) * gyroDpsPerLsb;
        const float absGyroYDps = std::fabs(static_cast<float>(state.sensors.imuBackLeft.gyroY)) * gyroDpsPerLsb;
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
        const float headingDeviationRad =
            std::fabs(
                ComputeHeadingDeviationRad(
                    _measurementStartUs,
                    _measurementStartYawRad,
                    state.estimate.yawRad));
        if (std::isfinite(headingDeviationRad))
        {
            _peakHeadingDeviationRad = (std::max)(_peakHeadingDeviationRad, headingDeviationRad);
        }
        if (std::isfinite(state.sensors.accelBodyYMps2))
        {
            _mostNegativeForwardAccelMps2 = (std::min)(_mostNegativeForwardAccelMps2, state.sensors.accelBodyYMps2);
        }
    }
    int accelcount = 0;
    LoopController::ControlVector TopSpeedMeasurementMode::RunTick(
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        if (_phaseStartUs == 0U)
        {
            _phaseStartUs = state.commandApplyTimeUs;
            SetLastCommandInputs(0.0f, 0.0f);
            if (!WriteEvent("transition", "state=prelaunch_wait"))
            {
                services.Fault("Top-speed measurement start logging failed");
                return LoopController::ControlVector::Brake;
            }
        }

        if (_logOpen)
        {
            const SensorSnapshot& snapshot = state.sensors;
            const MazeMap::VehicleState::StateVector& estimatorState = _drive.GetEstimatorStateVector();
            _logRow = {};
            _logRow.master_time_us = state.tickStartUs;
            _logRow.control_tick_sequence = ++_controlTickSequence;
            _logRow.dt_us = state.dtUs;
            _logRow.elapsed_test_us = (_measurementStartUs != 0U) ? (state.tickStartUs - _measurementStartUs) : 0U;
            _logRow.braking = (_phase == RunPhase::Braking) ? 1U : 0U;
            _logRow.impact_detected = _impactDetected ? 1U : 0U;
            _logRow.selector_removed = SelectorRemoved() ? 1U : 0U;
            _logRow.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
            _logRow.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
            _logRow.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
            _logRow.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
            _logRow.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
            _logRow.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
            _logRow.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
            _logRow.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
            _logRow.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
            _logRow.measured_linear_speed_mps = state.measured.linearSpeedMps;
            _logRow.measured_angular_speed_radps = state.measured.angularSpeedRadps;
            _logRow.cmd_linear_mps = _lastCommandInputLinearSpeedMps;
            _logRow.cmd_angular_radps = _lastCommandInputAngularRateRadps;
            _logRow.heading_error_rad =
                ComputeHeadingDeviationRad(
                    _measurementStartUs,
                    _measurementStartYawRad,
                    state.estimate.yawRad);
            _logRow.cmd_linear_accel_mps2 =
                ((_phase == RunPhase::Running) && (_measurementStartUs != 0U)) ? kTopSpeedMeasurementForwardAccelerationMps2 : 0.0f;
            _logRow.left_drive_command = state.driveTelemetry.leftDriveCommand;
            _logRow.right_drive_command = state.driveTelemetry.rightDriveCommand;
            _logRow.left_feedforward_command = state.driveTelemetry.leftFeedforwardCommand;
            _logRow.right_feedforward_command = state.driveTelemetry.rightFeedforwardCommand;
            _logRow.left_feedback_command = state.driveTelemetry.leftFeedbackCommand;
            _logRow.right_feedback_command = state.driveTelemetry.rightFeedbackCommand;
            _logRow.left_target_velocity_mps = state.driveTelemetry.leftTargetVelocityMps;
            _logRow.right_target_velocity_mps = state.driveTelemetry.rightTargetVelocityMps;
            _logRow.left_launch_assist_floor = state.driveTelemetry.leftLaunchAssistFloor;
            _logRow.right_launch_assist_floor = state.driveTelemetry.rightLaunchAssistFloor;
            _logRow.left_encoder_count = state.driveTelemetry.leftEncoderCount;
            _logRow.right_encoder_count = state.driveTelemetry.rightEncoderCount;
            _logRow.left_encoder_omega_radps = state.driveTelemetry.leftEncoderOmegaRadps;
            _logRow.right_encoder_omega_radps = state.driveTelemetry.rightEncoderOmegaRadps;
            _logRow.left_encoder_distance_m = state.driveTelemetry.leftDistanceM;
            _logRow.right_encoder_distance_m = state.driveTelemetry.rightDistanceM;
            _logRow.left_encoder_velocity_mps = state.driveTelemetry.leftVelocityMps;
            _logRow.right_encoder_velocity_mps = state.driveTelemetry.rightVelocityMps;
            _logRow.imu_timestamp_us = snapshot.imuTiming.readDoneUs;
            _logRow.imu_status = snapshot.imuBackLeft.status;
            _logRow.imu_interrupt_high = snapshot.imuBackLeft.interruptHigh ? 1U : 0U;
            _logRow.accel_bias_valid = snapshot.accelBiasValid ? 1U : 0U;
            _logRow.imu_gyro_x = snapshot.imuBackLeft.gyroX;
            _logRow.imu_gyro_y = snapshot.imuBackLeft.gyroY;
            _logRow.imu_gyro_z = snapshot.imuBackLeft.gyroZ;
            _logRow.imu_accel_x = snapshot.imuBackLeft.accelX;
            _logRow.imu_accel_y = snapshot.imuBackLeft.accelY;
            _logRow.imu_accel_z = snapshot.imuBackLeft.accelZ;
            _logRow.imu_temp = snapshot.imuBackLeft.temp;
            _logRow.gyro_raw_radps = snapshot.gyroRawRadps;
            _logRow.gyro_bias_radps = snapshot.gyroBiasRadps;
            _logRow.gyro_radps = snapshot.gyroRadps;
            _logRow.accel_body_x_mps2 = snapshot.accelBodyXMps2;
            _logRow.accel_body_y_mps2 = snapshot.accelBodyYMps2;
            _logRow.planar_accel_mps2 = state.sensors.planarAccelMps2;
            _logRow.front_timestamp_us = snapshot.frontTiming.observationReadyUs;
            _logRow.left_timestamp_us = snapshot.leftTiming.observationReadyUs;
            _logRow.right_timestamp_us = snapshot.rightTiming.observationReadyUs;
            _logRow.front_left_wall_distance_m = snapshot.frontLeft.distanceM;
            _logRow.front_right_wall_distance_m = snapshot.frontRight.distanceM;
            _logRow.side_left_wall_distance_m = snapshot.sideLeft.distanceM;
            _logRow.side_right_wall_distance_m = snapshot.sideRight.distanceM;
            _logRow.front_left_differential_light = snapshot.frontLeft.differentialLight;
            _logRow.front_right_differential_light = snapshot.frontRight.differentialLight;
            _logRow.side_left_differential_light = snapshot.sideLeft.differentialLight;
            _logRow.side_right_differential_light = snapshot.sideRight.differentialLight;
            _logRow.fan_duty_cycle = GetMissionFanDutyCycle();
            if (!_runtime.LogUtilityDataRow(_logRow))
            {
                services.Fault("Top-speed measurement log write failed");
                return LoopController::ControlVector::Brake;
            }
        }

        if (_measurementStartUs != 0U)
        {
            UpdatePeaks(state);
        }
        if (!state.estimatorHealthy)
        {
            services.Fault(
                (state.faultReason != nullptr) ?
                    state.faultReason :
                    "Top-speed measurement estimator fault");
            return LoopController::ControlVector::Brake;
        }

        const bool selectorRemoved = SelectorRemoved();
        if (_phase == RunPhase::PrelaunchWait)
        {
            SetLastCommandInputs(0.0f, 0.0f);
            if (selectorRemoved)
            {
                services.Fault(kTopSpeedMeasurementSelectorRemovedFaultReason);
                return LoopController::ControlVector::Brake;
            }

            const std::uint32_t nextPhaseElapsedUs = state.commandApplyTimeUs - _phaseStartUs;
            if (nextPhaseElapsedUs >= kTopSpeedMeasurementPrelaunchWaitUs)
            {
                _phase = RunPhase::Running;
                _phaseStartUs = state.commandApplyTimeUs;
                _measurementStartUs = 0U;
                if (!WriteEvent("transition", "state=running;stage=reverse_kick"))
                {
                    services.Fault("Top-speed measurement reverse-kick logging failed");
                    return LoopController::ControlVector::Brake;
                }
                return LoopController::ControlVector::RawMotorPwm(
                    kTopSpeedMeasurementPrelaunchReverseDriveCommand,
                    kTopSpeedMeasurementPrelaunchReverseDriveCommand);
            }

            return LoopController::ControlVector::Brake;
        }

        if (_phase == RunPhase::Running)
        {
            const auto issueForwardAccelerationCommand = [&]() noexcept
            {
                const float resolvedLinearSpeedMps =
                    std::isfinite(state.measured.linearSpeedMps) ?
                        state.measured.linearSpeedMps :
                        0.0f;
                const float measuredYawRateRadps =
            ResolveBiasCorrectedGyroYawRateRadps(state.sensors);
                const float yawRateCorrectionRadps =
                    ResolveGyroBiasHoldYawRateCommandRadps(
            state.sensors,
                        state.dtSeconds,
                        _gyroZHoldControllerState);
                const float responseTimeS =
                    (std::isfinite(MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS) &&
                     (MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS > 0.0f)) ?
                    MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS :
                    0.0f;
                const float desiredYawAccelRadps2 =
                    (responseTimeS > 0.0f) ?
                    ((yawRateCorrectionRadps - measuredYawRateRadps) / responseTimeS) :
                    0.0f;

                const MazeMap::OpenLoopDriveCommand driveCommand =
                    _drive.DeltaCommand(
                        resolvedLinearSpeedMps,
                        kTopSpeedMeasurementForwardAccelerationMps2,
                        measuredYawRateRadps,
                        desiredYawAccelRadps2,
                        MazeMap::CommandPD::RawCommand);

                SetLastCommandInputs(resolvedLinearSpeedMps, yawRateCorrectionRadps);
                return LoopController::ControlVector::RawMotorPwm(
                    driveCommand.leftDriveCommand,
                    driveCommand.rightDriveCommand);
            };

            if (_measurementStartUs == 0U)
            {
                SetLastCommandInputs(0.0f, 0.0f);
                if (selectorRemoved)
                {
                    services.Fault(kTopSpeedMeasurementSelectorRemovedFaultReason);
                    return LoopController::ControlVector::Brake;
                }

                const std::uint32_t nextPhaseElapsedUs = state.commandApplyTimeUs - _phaseStartUs;
                if (nextPhaseElapsedUs < kTopSpeedMeasurementPrelaunchReverseDurationUs)
                {
                    return LoopController::ControlVector::RawMotorPwm(
                        kTopSpeedMeasurementPrelaunchReverseDriveCommand,
                        kTopSpeedMeasurementPrelaunchReverseDriveCommand);
                }

                _measurementStartUs = state.commandApplyTimeUs;
                _gyroZHoldControllerState.Reset();
                _measurementStartYawRad =
                    std::isfinite(state.estimate.yawRad) ?
                    state.estimate.yawRad :
                    0.0f;
                if (!WriteEvent("transition", "state=running;stage=forward_run"))
                {
                    services.Fault("Top-speed measurement forward-run logging failed");
                    return LoopController::ControlVector::Brake;
                }
                return issueForwardAccelerationCommand();
            }

            const std::uint32_t elapsedUs = state.tickStartUs - _measurementStartUs;
            const bool brakeTriggersArmed = elapsedUs >= kTopSpeedMeasurementBrakeTriggerArmDelayUs;
            if (selectorRemoved)
            {
                services.Fault(kTopSpeedMeasurementSelectorRemovedFaultReason);
                return LoopController::ControlVector::Brake;
            }

            if (brakeTriggersArmed && ImpactDetected(state))
            {
                _impactDetected = true;
                _impactSpeedMps = state.measured.linearSpeedMps;
                _impactForwardAccelMps2 = state.sensors.accelBodyYMps2;
                _impactGyroXRawLsb = state.sensors.imuBackLeft.gyroX;
                _impactGyroYRawLsb = state.sensors.imuBackLeft.gyroY;
                (void)EnterBrakingPhase(
                    BrakeTrigger::ImpactDetected,
                    "state=braking;trigger=impact_detected",
                    services);
                return LoopController::ControlVector::Brake;
            }

            if (brakeTriggersArmed && GyroSpikeDetected(state))
            {
                _impactDetected = true;
                _impactSpeedMps = state.measured.linearSpeedMps;
                _impactForwardAccelMps2 = state.sensors.accelBodyYMps2;
                _impactGyroXRawLsb = state.sensors.imuBackLeft.gyroX;
                _impactGyroYRawLsb = state.sensors.imuBackLeft.gyroY;
                (void)EnterBrakingPhase(
                    BrakeTrigger::GyroSpikeDetected,
                    "state=braking;trigger=gyro_xy_spike_detected",
                    services);
                return LoopController::ControlVector::Brake;
            }

            const std::uint32_t nextCommandElapsedUs = state.commandApplyTimeUs - _measurementStartUs;
            if (accelcount>=(kTopSpeedMeasurementAccelerationDurationUs/1000))
            {
                (void)EnterBrakingPhase(
                    BrakeTrigger::TimedWindowElapsed,
                    "state=braking;trigger=timed_window_elapsed",
                    services);
                return LoopController::ControlVector::Brake;
            }

            accelcount++;
            return issueForwardAccelerationCommand();
        }

        if (selectorRemoved)
        {
            services.Fault(kTopSpeedMeasurementSelectorRemovedFaultReason);
            SetLastCommandInputs(0.0f, 0.0f);
            return LoopController::ControlVector::Brake;
        }

        if (EncoderMotionSettled(state))
        {
            _completionReason = CompletionReason::SettledStop;
            services.RequestEndLoop();
            SetLastCommandInputs(0.0f, 0.0f);
            return LoopController::ControlVector::Brake;
        }

        SetLastCommandInputs(0.0f, 0.0f);
        return LoopController::ControlVector::Brake;
    }

    bool TopSpeedMeasurementMode::Fail(const char* reason)
    {
        return _runtime.FailActiveMode(reason);
    }

    void TopSpeedMeasurementMode::OnRuntimeFault(const char* reason) noexcept
    {
        _faulted = true;
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
        _phase = RunPhase::PrelaunchWait;
        _brakeTrigger = BrakeTrigger::None;
        _completionReason = CompletionReason::None;
        _phaseStartUs = 0U;
        _measurementStartUs = 0U;
        _controlTickSequence = 0U;
        _batteryVoltageStart = 0.0f;
        _fanDutyCycleStart = 0.0f;
        _measurementStartYawRad = 0.0f;
        _peakMeasuredSpeedMps = 0.0f;
        _peakPlanarAccelMps2 = 0.0f;
        _peakHeadingDeviationRad = 0.0f;
        _mostNegativeForwardAccelMps2 = 0.0f;
        _impactDetected = false;
        _impactSpeedMps = 0.0f;
        _impactForwardAccelMps2 = 0.0f;
        _impactGyroXRawLsb = 0;
        _impactGyroYRawLsb = 0;
        _lastCommandInputLinearSpeedMps = 0.0f;
        _lastCommandInputAngularRateRadps = 0.0f;
        _gyroZHoldControllerState.Reset();
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
            "Measure straight-line peak speed with reverse-kick launch and impact brake.",
            "logging.txt; top-speed telemetry mmlog",
            &GetTopSpeedMeasurementMode,
            "GetTopSpeedMeasurementMode",
            "TopSpeedMeasurementMode.cpp",
            "startup; 1 s wait; 10 ms reverse kick; accel run; impact-or-time brake; settle",
            "DiagnosticConfig period; shared mission drive/IMU tuning; LoopController capture",
            "Fixed 8.5 m/s^2 launch, 10 ms -0.4 reverse kick, local gyro-z bias hold, impact thresholds, wall updates disabled, selector removal faults.",
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
