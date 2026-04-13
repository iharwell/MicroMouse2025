#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

class AuxMeasurementController : public IApplicationMode, public MazeMap::App::Internal::LoopController::IMode
{
public:
    explicit AuxMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _loopRuntime{ runtime, runtime.Drive(), nullptr, &runtime.DiagnosticSensors(), nullptr, nullptr, nullptr, nullptr }
        , _faulted(false)
        , _fanEnabled(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        if (!_runtime.RegisterModeFaultHandler(&AuxMeasurementController::HandleRuntimeFault, this, "aux_measurement"))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:aux_measurement");
        (void)_runtime.AppendTextLogLine("Auxiliary measurement mode");
        if (!_runtime.Drive().Begin())
        {
            return Fail("Drive base init failed");
        }
        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            _runtime.Drive().SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        }
        else
        {
            _runtime.Drive().UseNominalWheelControlProfile();
        }
        SetFanEnabled(false);
        gWallDistanceCalibration.Clear();
        if (!_runtime.DiagnosticSensors().Begin(AuxMeasurementConfig::kControlPeriodUs))
        {
            return Fail("Auxiliary sensor init failed");
        }
        if (!BeginLog())
        {
            return Fail("Auxiliary measurement log open failed");
        }

        _runtime.Drive().SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        MazeMap::App::Internal::LoopController::SessionConfig loopConfig{};
        loopConfig.bootModeId = MazeMap::App::BootModeId::AuxiliaryMeasurement;
        loopConfig.sessionName = "aux_measurement";
        loopConfig.controlPeriodUs = AuxMeasurementConfig::kControlPeriodUs;
        loopConfig.startupCommandPolicy =
            MazeMap::App::Internal::LoopController::SessionConfig::StartupCommandPolicy::Brake;
        loopConfig.actuationPolicy =
            MazeMap::App::Internal::LoopController::SessionConfig::ActuationPolicy::VelocityBrakeOpenLoop;
        loopConfig.allowDynamicCaptureOverride = false;
        loopConfig.serviceWaitState = true;
        loopConfig.serviceSlackState = true;
        return _loopController.BeginSession(loopConfig, _loopRuntime, *this);
    }

    void Run() override
    {
        if (_faulted)
        {
            return;
        }

        (void)_runtime.AppendTextLogLine("Entered by shorting pins 28-29 at boot.");
        (void)_runtime.AppendTextLogLine("This mode uses internal sensors only; edit AuxMeasurementConfig::kRoutine for other one-off measurements.");

        const bool ok = RunSelectedRoutine();

        _runtime.Drive().Brake();
        _runtime.Drive().UseNominalWheelControlProfile();
        SetFanEnabled(false);
        _loopController.EndSession();
        if (ok)
        {
            (void)_runtime.AppendTextLogFormatted("Auxiliary measurement complete, log saved to %s", GetLogFileName());
        }
        CloseLog();
    }

    bool OnSessionBegin(const MazeMap::App::Internal::LoopController::VehicleState& initial) override
    {
        (void)initial;
        return true;
    }

    MazeMap::App::Internal::LoopController::ControlVector Step(
        std::uint32_t availableComputeUs,
        const MazeMap::App::Internal::LoopController::VehicleState& state,
        MazeMap::App::Internal::LoopController::TickServices& services) override
    {
        if (_tickCallback == nullptr)
        {
            services.Fault("Aux measurement tick callback was not installed");
            return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
        }

        return _tickCallback(_tickCallbackContext, availableComputeUs, state, services);
    }

    void OnSessionEnd(const MazeMap::App::Internal::LoopController::SessionResult& result) override
    {
        (void)result;
        _tickCallbackContext = nullptr;
        _tickCallback = nullptr;
    }

    void ServiceWaitState() override
    {
        ServiceLog();
    }

    void ServiceSlackState() override
    {
        ServiceLog();
    }

private:
    template <typename Callback>
    bool RunControlTick(Callback&& callback)
    {
        _tickCallbackContext = &callback;
        _tickCallback = [](void* context,
                           std::uint32_t availableComputeUs,
                           const MazeMap::App::Internal::LoopController::VehicleState& state,
                           MazeMap::App::Internal::LoopController::TickServices& services)
            -> MazeMap::App::Internal::LoopController::ControlVector
        {
            return (*static_cast<Callback*>(context))(availableComputeUs, state, services);
        };

        const MazeMap::App::Internal::LoopController::SessionResult result = _loopController.RunOneTick();
        _tickCallbackContext = nullptr;
        _tickCallback = nullptr;
        return result.status == MazeMap::App::Internal::LoopController::SessionResult::Status::Running;
    }

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<AuxMeasurementController*>(context)->OnRuntimeFault(reason);
    }

    SharedRobotRuntime& _runtime;
    MazeMap::App::Internal::LoopController& _loopController;
    MazeMap::App::Internal::LoopController::RuntimeBundle _loopRuntime;
    char _logFileName[64];
    bool _faulted;
    bool _fanEnabled;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    using TickCallback = MazeMap::App::Internal::LoopController::ControlVector (*)(
        void* context,
        std::uint32_t availableComputeUs,
        const MazeMap::App::Internal::LoopController::VehicleState& state,
        MazeMap::App::Internal::LoopController::TickServices& services);
    void* _tickCallbackContext = nullptr;
    TickCallback _tickCallback = nullptr;

    bool BeginLog()
    {
        _phaseId = 0UL;
        _sampleCount = 0UL;
        if (!_runtime.OpenUtilityDataLog(
                _logFileName,
                sizeof(_logFileName),
                nullptr,
                "aux%03u.mmlog",
                "aux_measurement_log.mmlog"))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", "aux_measurement")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("routine", AuxMeasurementRoutineName(AuxMeasurementConfig::kRoutine))) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", AuxMeasurementConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !_runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                AuxMeasurementConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("startup_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs))) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _runtime.DiagnosticSensors().GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _runtime.DiagnosticSensors().GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _runtime.DiagnosticSensors().GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_runtime.DiagnosticSensors())) return false;
        if (!WriteEvent("summary", "Enter by shorting pins 28 and 29 at boot. Those pins only select this mode; they are not measurement inputs.")) return false;
        if (!WriteEvent("summary", "Edit AuxMeasurementConfig::kRoutine and RunSelectedRoutine() to repurpose this mode for one-off internal measurements.")) return false;
        if (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteEvent("summary", "The default routine enables the mission fan, ramps circle speed without a software ceiling, and if speed plateaus before slip it tightens curvature until sustained encoder-vs-gyro/IMU mismatch indicates traction loss.")) return false;
            if (!WriteEvent("summary", "Use traction_limit_result and the last steady samples before it to estimate the maximum sustainable circle speed, yaw rate, and lateral acceleration.")) return false;
        }
        else
        {
            if (!WriteEvent("summary", "The default routine logs stationary fan-off, fan-on, and recovery phases so you can quantify fan-induced sensor and vibration shifts.")) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;

        AuxMeasurementLogRow row{};
        return _runtime.BeginUtilityDataLogSchema(row);
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;
        return _runtime.WriteTextLogPhase(_phaseId, micros(), name);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        return _runtime.WriteTextLogEntry(micros(), type, message);
    }

    bool LogSample(
        bool stationary,
        bool fanEnabled,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot,
        float planarAccelMps2)
    {
        AuxMeasurementLogRow row{};
        MazeMap::App::Internal::Runtime::PopulateAuxMeasurementLogRow(
            row,
            _sampleCount,
            _phaseId,
            stationary,
            fanEnabled,
            timestampUs,
            dtUs,
            pose,
            drive,
            driveTelemetry,
            sensorSnapshot,
            planarAccelMps2);
        if (!_runtime.LogUtilityDataRow(row))
        {
            return false;
        }

        ++_sampleCount;
        return true;
    }

    void ServiceLog()
    {
        (void)_runtime.ServiceUtilityDataLog();
    }

    void CloseLog()
    {
        (void)_runtime.CloseUtilityDataLog();
        _runtime.CloseTextLog();
    }

    const char* GetLogFileName() const
    {
        return _logFileName;
    }

    bool RunSelectedRoutine()
    {
        switch (AuxMeasurementConfig::kRoutine)
        {
        case AuxMeasurementConfig::Routine::FanStaticSurvey:
            return RunFanStaticSurvey();
        case AuxMeasurementConfig::Routine::TurningTractionSweep:
            return RunTurningTractionSweep();
        default:
            return Fail("Unknown auxiliary measurement routine");
        }
    }

    bool RunFanStaticSurvey()
    {
        bool ok = true;
        ok = ok && HoldPhase("startup_settle", AuxMeasurementConfig::kStartupSettleMs, true, false);
        ok = ok && HoldPhase("fan_off_baseline", AuxMeasurementConfig::kBaselineHoldMs, true, false);
        ok = ok && HoldPhase("fan_on_hold", AuxMeasurementConfig::kFanHoldMs, true, true);
        ok = ok && HoldPhase("fan_off_recovery", AuxMeasurementConfig::kRecoveryHoldMs, true, false);
        return ok;
    }

    bool RunTurningTractionSweep()
    {
        bool ok = true;
        ok = ok && HoldPhase("startup_settle", AuxMeasurementConfig::kStartupSettleMs, true, false);
        ok = ok && HoldPhase("fan_spinup", AuxMeasurementConfig::kTurningTractionSweepFanSettleMs, true, true);
        if (!ok)
        {
            return false;
        }

        if (!BeginPhase("turning_traction_sweep"))
        {
            return Fail("Failed to begin turning traction sweep phase");
        }

        SetFanEnabled(true);
        _runtime.Drive().SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        const float directionSign = AuxMeasurementConfig::kTurningTractionSweepClockwise ? 1.0f : -1.0f;
        const float circleRadiusM = AuxMeasurementConfig::kTurningTractionSweepRadiusM;
        float commandedSpeedMps = AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps;
        float heldSpeedMps = commandedSpeedMps;
        float commandedCurvatureMInv = (circleRadiusM > 1.0e-6f) ? (1.0f / circleRadiusM) : 0.0f;
        float targetYawRad = _runtime.Drive().GetPose().yawRad;
        const unsigned long phaseStartMs = millis();
        unsigned long saturationPlateauStartMs = 0UL;
        unsigned long slipCandidateStartMs = 0UL;
        bool slipCandidateActive = false;
        bool tighteningTurn = false;
        MazeMap::TurningTractionMetrics lastMetrics{};
        float lastPlanarAccelMps2 = 0.0f;
        float lastCommandedOmegaRadps = 0.0f;
        float saturationReferenceSpeedMps = 0.0f;
        bool phaseFinished = false;
        bool phaseSucceeded = false;

        while (!_faulted && !phaseFinished)
        {
            if (!RunControlTick(
                    [&](std::uint32_t,
                        const MazeMap::App::Internal::LoopController::VehicleState& state,
                        MazeMap::App::Internal::LoopController::TickServices& services)
                    {
                        const unsigned long nowMs = millis();
                        if (static_cast<unsigned long>(nowMs - phaseStartMs) >= AuxMeasurementConfig::kTurningTractionSweepTimeoutMs)
                        {
                            phaseSucceeded = WriteTurningTractionResult(
                                "timeout",
                                false,
                                static_cast<unsigned long>(nowMs - phaseStartMs),
                                commandedSpeedMps,
                                lastCommandedOmegaRadps,
                                lastMetrics,
                                lastPlanarAccelMps2);
                            if (!phaseSucceeded)
                            {
                                services.Fault("Failed to write turning traction timeout result");
                            }
                            phaseFinished = true;
                            return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                        }

                        const float dtSeconds = state.dtSeconds;
                        const DiagnosticSensorSnapshot& sensorSnapshot = state.diagnosticSensors;
                        if (!tighteningTurn)
                        {
                            commandedSpeedMps += AuxMeasurementConfig::kTurningTractionSweepAccelMps2 * dtSeconds;
                            if constexpr (AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps > 0.0f)
                            {
                                commandedSpeedMps = (std::min)(AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps, commandedSpeedMps);
                            }
                            heldSpeedMps = commandedSpeedMps;
                        }
                        else
                        {
                            commandedSpeedMps = heldSpeedMps;
                            commandedCurvatureMInv += AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec * dtSeconds;
                        }

                        const float nominalOmegaRadps = directionSign * (commandedSpeedMps * commandedCurvatureMInv);
                        targetYawRad = WrapAngleRad(targetYawRad + (nominalOmegaRadps * dtSeconds));
                        lastCommandedOmegaRadps = MazeMap::ComputeTurningTractionAngularCommand(
                            nominalOmegaRadps,
                            targetYawRad,
                            state.estimate.yawRad,
                            state.estimate.angularSpeedRadps,
                            Config::kArcHeadingKp,
                            Config::kArcYawD,
                            AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps);
                        const float effectiveTrackWidthM =
                            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(commandedSpeedMps, lastCommandedOmegaRadps);
                        const float planarAccelMps2 = _runtime.DiagnosticSensors().GetPlanarAccelMps2(sensorSnapshot);
                        const MazeMap::TurningTractionMetrics metrics = MazeMap::ComputeTurningTractionMetrics(
                            state.driveTelemetry.leftVelocityMps,
                            state.driveTelemetry.rightVelocityMps,
                            effectiveTrackWidthM,
                            sensorSnapshot.gyroRadps,
                            planarAccelMps2);
                        lastMetrics = metrics;
                        lastPlanarAccelMps2 = planarAccelMps2;

                        if (!LogSample(
                                false,
                                _fanEnabled,
                                state.tickStartUs,
                                state.dtUs,
                                state.estimate,
                                _runtime.Drive(),
                                state.driveTelemetry,
                                sensorSnapshot,
                                planarAccelMps2))
                        {
                            services.Fault("Failed to write turning traction sample");
                            return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                        }

                        const bool slipDetected = MazeMap::IsTurningTractionLossDetected(
                            metrics,
                            AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps,
                            AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2,
                            AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor,
                            AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor);
                        if (slipDetected)
                        {
                            if (!slipCandidateActive)
                            {
                                slipCandidateStartMs = nowMs;
                                slipCandidateActive = true;
                            }
                            else if (static_cast<unsigned long>(nowMs - slipCandidateStartMs) >= AuxMeasurementConfig::kTurningTractionSlipConfirmMs)
                            {
                                phaseSucceeded = WriteTurningTractionResult(
                                    "traction_loss",
                                    true,
                                    static_cast<unsigned long>(nowMs - phaseStartMs),
                                    commandedSpeedMps,
                                    lastCommandedOmegaRadps,
                                    metrics,
                                    planarAccelMps2);
                                if (!phaseSucceeded)
                                {
                                    services.Fault("Failed to write turning traction loss result");
                                }
                                phaseFinished = true;
                                return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                            }
                        }
                        else
                        {
                            slipCandidateActive = false;
                        }

                        const float maxWheelCommandMagnitude = (std::max)(
                            std::fabs(state.driveTelemetry.leftDriveCommand),
                            std::fabs(state.driveTelemetry.rightDriveCommand));
                        if (!tighteningTurn)
                        {
                            const bool actuatorLimited =
                                (metrics.encoderLinearSpeedMps >= AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps) &&
                                (maxWheelCommandMagnitude >= AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand);

                            if (!actuatorLimited)
                            {
                                saturationPlateauStartMs = 0UL;
                                saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                            }
                            else if (saturationPlateauStartMs == 0UL)
                            {
                                saturationPlateauStartMs = nowMs;
                                saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                            }
                            else if (metrics.encoderLinearSpeedMps >= (saturationReferenceSpeedMps + AuxMeasurementConfig::kTurningTractionPlateauDeltaMps))
                            {
                                saturationPlateauStartMs = nowMs;
                                saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                            }
                            else if (static_cast<unsigned long>(nowMs - saturationPlateauStartMs) >= AuxMeasurementConfig::kTurningTractionPlateauWindowMs)
                            {
                                tighteningTurn = true;
                                heldSpeedMps = (std::max)(commandedSpeedMps, metrics.encoderLinearSpeedMps);

                                char message[160] = {};
                                const int messageLength = snprintf(
                                    message,
                                    sizeof(message),
                                    "reason=speed_plateau;hold_v_mps=%.3f;curvature_m_inv=%.3f;outer_cmd=%.3f",
                                    heldSpeedMps,
                                    commandedCurvatureMInv,
                                    maxWheelCommandMagnitude);
                                if (messageLength <= 0 || messageLength >= static_cast<int>(sizeof(message)))
                                {
                                    services.Fault("Failed to format turning traction mode event");
                                    return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                                }
                                if (!WriteEvent("turning_traction_mode", message))
                                {
                                    services.Fault("Failed to write turning traction mode event");
                                    return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                                }
                            }
                        }

                        if (static_cast<unsigned long>(nowMs - phaseStartMs) < AuxMeasurementConfig::kTurningTractionLaunchMs)
                        {
                            const MazeMap::TurningLaunchCommands launchCommands = MazeMap::ComputeTurningLaunchCommands(
                                commandedSpeedMps,
                                lastCommandedOmegaRadps,
                                effectiveTrackWidthM,
                                Config::kWheelRestLaunchDriveCommand);
                            return MazeMap::App::Internal::LoopController::ControlVector::OpenLoopCommand(
                                launchCommands.leftCommand,
                                launchCommands.rightCommand);
                        }

                        return MazeMap::App::Internal::LoopController::ControlVector::VelocityCommand(
                            commandedSpeedMps,
                            lastCommandedOmegaRadps);
                    }))
            {
                return false;
            }
        }

        return phaseSucceeded;
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary, bool fanEnabled)
    {
        if (!BeginPhase(phaseName))
        {
            return Fail("Failed to begin auxiliary measurement phase");
        }

        SetFanEnabled(fanEnabled);
        const unsigned long startMs = millis();
        while (!_faulted && static_cast<unsigned long>(millis() - startMs) < durationMs)
        {
            if (!RunControlTick(
                    [&, stationary](std::uint32_t,
                                    const MazeMap::App::Internal::LoopController::VehicleState& state,
                                    MazeMap::App::Internal::LoopController::TickServices& services)
                    {
                        const float planarAccelMps2 = _runtime.DiagnosticSensors().GetPlanarAccelMps2(state.diagnosticSensors);
                        if (!LogSample(
                                stationary,
                                _fanEnabled,
                                state.tickStartUs,
                                state.dtUs,
                                state.estimate,
                                _runtime.Drive(),
                                state.driveTelemetry,
                                state.diagnosticSensors,
                                planarAccelMps2))
                        {
                            services.Fault("Failed to write auxiliary measurement sample");
                        }
                        return MazeMap::App::Internal::LoopController::ControlVector::BrakeCommand();
                    }))
            {
                return false;
            }
        }

        return !_faulted;
    }

    void SetFanEnabled(bool enabled)
    {
        if (_fanEnabled == enabled)
        {
            return;
        }

        _fanEnabled = enabled;
        SetMissionLevelFanEnabled(enabled);
    }

    bool WriteTurningTractionResult(
        const char* reason,
        bool slipDetected,
        unsigned long elapsedMs,
        float commandedSpeedMps,
        float commandedOmegaRadps,
        const MazeMap::TurningTractionMetrics& metrics,
        float planarAccelMps2)
    {
        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "reason=%s;slip=%u;elapsed_ms=%lu;cmd_v_mps=%.3f;cmd_w_radps=%.3f;enc_v_mps=%.3f;enc_w_radps=%.3f;pred_lat_mps2=%.3f;planar_accel_mps2=%.3f;yaw_ratio=%.3f;planar_ratio=%.3f",
            (reason != nullptr) ? reason : "unknown",
            slipDetected ? 1U : 0U,
            elapsedMs,
            commandedSpeedMps,
            commandedOmegaRadps,
            metrics.encoderLinearSpeedMps,
            metrics.encoderOmegaRadps,
            metrics.predictedLateralAccelMps2,
            planarAccelMps2,
            metrics.yawCoherence,
            metrics.planarCoherence);

        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Turning traction result event overflowed");
        }
        if (WriteEvent("traction_limit_result", message))
        {
            return true;
        }
        return Fail("Failed to write turning traction result");
    }

    bool Fail(const char* reason)
    {
        return _runtime.FailActiveMode(reason);
    }

    void OnRuntimeFault(const char* reason) noexcept
    {
        _faulted = true;
        _fanEnabled = false;
        if (reason != nullptr && reason[0] != '\0')
        {
            (void)WriteEvent("fault", reason);
        }
        AppendStartupTrace((reason != nullptr) ? reason : "aux_measurement_fault");
    }

    static MazeMap::WheelControlProfile BuildTurningTractionWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }
};

namespace MazeMap::App::Internal
{
    const BootModeDescriptor& GetAuxMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::AuxiliaryMeasurement,
            BootModeCategory::Utility,
            "auxiliary_measurement",
            "Run the selected one-off auxiliary measurement routine.",
            "logging.txt; auxiliary measurement mmlog",
            "GetAuxMeasurementMode",
            "AuxMeasurementController.cpp",
            "startup settle; selected auxiliary routine; final log close",
            "AuxMeasurementConfig; shared mission drive and sensor tuning",
            "AuxMeasurementConfig::kRoutine selects the auxiliary scenario",
            "aux%03u.mmlog or aux_measurement_log.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetAuxMeasurementMode()
    {
        static AuxMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}

