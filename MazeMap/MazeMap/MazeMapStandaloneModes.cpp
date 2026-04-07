#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorLoggingV2Support.h"
#include "OpenFloorMainLoggerV2.h"
#include "OpenFloorTimingLoggerV2.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;
using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

namespace OpenFloorLoggingV2 = MazeMap::App::Internal::Runtime::OpenFloorLoggingV2;

#define FRONT_WALL_CHARACTERIZATION_LOG_FIELDS(X) \
    X(std::uint32_t, index) \
    X(float,         distance_m) \
    X(float,         front_left_ambient) \
    X(float,         front_left_lit) \
    X(float,         front_left_delta) \
    X(float,         front_right_ambient) \
    X(float,         front_right_lit) \
    X(float,         front_right_delta)

MMLOG_DEFINE_ROW(FrontWallCharacterizationLogRow, FRONT_WALL_CHARACTERIZATION_LOG_FIELDS);

class AuxMeasurementController : public IApplicationMode
{
public:
    explicit AuxMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _faulted(false)
        , _fanEnabled(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _lastControlMicros(0UL)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Auxiliary measurement mode");

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:aux_measurement");
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
        if (!_runtime.TelemetrySensors().Begin(AuxMeasurementConfig::kControlPeriodUs))
        {
            return Fail("Auxiliary sensor init failed");
        }
        if (!BeginLog())
        {
            return Fail("Auxiliary measurement log open failed");
        }

        _runtime.Drive().SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _lastControlMicros = micros();
        return true;
    }

    void Run() override
    {
        if (_faulted)
        {
            return;
        }

        Serial.println("Entered by shorting pins 28-29 at boot.");
        Serial.println("This mode uses internal sensors only; edit AuxMeasurementConfig::kRoutine for other one-off measurements.");

        const bool ok = RunSelectedRoutine();

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        SetFanEnabled(false);
        FlushLog();
        if (ok)
        {
            Serial.print("Auxiliary measurement complete, log saved to ");
            Serial.println(GetLogFileName());
        }
        CloseLog();
    }

private:
	SharedRobotRuntime& _runtime;
    //MazeMap::mmlog::MmLogLogger _sampleLog;
    //MazeMap::App::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    char _logFileName[64];
    bool _faulted;
    bool _fanEnabled;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    unsigned long _lastControlMicros;
    MazeMap::mmlog::MmLogLogger& getDataLogger() { return _runtime.GetDataLogger(); }
    MazeMap::CoreFileExport& getLog() { return _runtime.GetLoggingFile(); }
    bool BeginLog()
    {
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        _phaseId = 0UL;
        _sampleCount = 0UL;
        _logFileName[0] = '\0';
        _runtime.GetDataLogger().close();
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                _logFileName,
                sizeof(_logFileName),
                nullptr,
                "aux%03u.mmlog",
                "aux_measurement_log.mmlog"))
        {
            return false;
        }
        if (!_runtime.GetDataLogger().open(_logFileName))
        {
            return false;
        }
        _runtime.GetLoggingFile();
        if (!_runtime.GetDataLogger().writeMetadata("file", _logFileName)) return false;
        if (!getDataLogger().writeMetadata("mode", "aux_measurement")) return false;
        if (!getDataLogger().writeMetadata("routine", AuxMeasurementRoutineName(AuxMeasurementConfig::kRoutine))) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(getDataLogger(), "control_period_us", AuxMeasurementConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(getDataLogger(), "imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                AuxMeasurementConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(getDataLogger(), "imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(getDataLogger(), "imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(getDataLogger(), "startup_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs))) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(getDataLogger(), "imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(getDataLogger(), "imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(getDataLogger(), "mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogAccelBiasMetadata(getDataLogger(), _sensors)) return false;
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
        if (!getDataLogger().writeMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;

        AuxMeasurementLogRow row{};
        return getDataLogger().begin(row);
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;
        return getDataLogger().WritePhase(_phaseId, micros(), name);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        char line[384] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%s [%lu] %s%s%s",
            (type != nullptr && type[0] != '\0') ? source : "runtime",
            micros(),
            (type != nullptr && type[0] != '\0') ? type : "event",
            (message != nullptr && message[0] != '\0') ? ": " : "",
            (message != nullptr) ? message : "");
        if (length <= 0)
        {
            return false;
        }

        line[sizeof(line) - 1U] = '\0';
        return AppendRuntimeControlLogLine(line);
        return getLog().Write(line);
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
        if (!getDataLogger().log(row))
        {
            return false;
        }

        ++_sampleCount;
        return true;
    }

    void ServiceLog()
    {
        (void)getDataLogger().service();
    }

    void FlushLog()
    {
        (void)getDataLogger().flush();
        getLog().Flush();
    }

    void CloseLog()
    {
        (void)getDataLogger().close();
        getLog().Close();
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

        while (!_faulted)
        {
            const unsigned long nowMs = millis();
            if (static_cast<unsigned long>(nowMs - phaseStartMs) >= AuxMeasurementConfig::kTurningTractionSweepTimeoutMs)
            {
                _runtime.Drive().Brake();
                return WriteTurningTractionResult(
                    "timeout",
                    false,
                    static_cast<unsigned long>(nowMs - phaseStartMs),
                    commandedSpeedMps,
                    lastCommandedOmegaRadps,
                    lastMetrics,
                    lastPlanarAccelMps2);
            }

            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);

            const DiagnosticSensorSnapshot sensorSnapshot = _runtime.TelemetrySensors().Capture(false, _runtime.Drive().GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _runtime.Drive().UpdateOdometry(dtSeconds, sensorSnapshot);
            if (_runtime.Drive().HasEstimatorFault())
            {
                return Fail(_runtime.Drive().GetEstimatorFaultReason());
            }
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
                _runtime.Drive().GetPose().yawRad,
                _runtime.Drive().GetPose().angularSpeedRadps,
                Config::kArcHeadingKp,
                Config::kArcYawD,
                AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps);
            const float effectiveTrackWidthM =
                MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(commandedSpeedMps, lastCommandedOmegaRadps);
            if (static_cast<unsigned long>(nowMs - phaseStartMs) < AuxMeasurementConfig::kTurningTractionLaunchMs)
            {
                const MazeMap::TurningLaunchCommands launchCommands = MazeMap::ComputeTurningLaunchCommands(
                    commandedSpeedMps,
                    lastCommandedOmegaRadps,
                    effectiveTrackWidthM,
                    Config::kWheelRestLaunchDriveCommand);
                _runtime.Drive().CommandOpenLoopRaw(launchCommands.leftCommand, launchCommands.rightCommand);
            }
            else
            {
                _runtime.Drive().CommandVelocity(commandedSpeedMps, lastCommandedOmegaRadps, dtSeconds);
            }

            const DriveTelemetry driveTelemetry = _runtime.Drive().GetTelemetry();
            const float planarAccelMps2 = _runtime.Drive().GetPlanarAccelMps2(sensorSnapshot);
            const MazeMap::TurningTractionMetrics metrics = MazeMap::ComputeTurningTractionMetrics(
                driveTelemetry.leftVelocityMps,
                driveTelemetry.rightVelocityMps,
                effectiveTrackWidthM,
                sensorSnapshot.gyroRadps,
                planarAccelMps2);
            lastMetrics = metrics;
            lastPlanarAccelMps2 = planarAccelMps2;

            if (!LogSample(
                false,
                _fanEnabled,
                timestampUs,
                dtUs,
                _runtime.Drive().GetPose(),
                _runtime.Drive(),
                driveTelemetry,
                sensorSnapshot,
                planarAccelMps2))
            {
                return Fail("Failed to write turning traction sample");
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
                    _runtime.Drive().Brake();
                    return WriteTurningTractionResult(
                        "traction_loss",
                        true,
                        static_cast<unsigned long>(nowMs - phaseStartMs),
                        commandedSpeedMps,
                        lastCommandedOmegaRadps,
                        metrics,
                        planarAccelMps2);
                }
            }
            else
            {
                slipCandidateActive = false;
            }

            const float maxWheelCommandMagnitude = (std::max)(
                std::fabs(driveTelemetry.leftDriveCommand),
                std::fabs(driveTelemetry.rightDriveCommand));
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
                        return Fail("Failed to format turning traction mode event");
                    }
                    if (!WriteEvent("turning_traction_mode", message))
                    {
                        return Fail("Failed to write turning traction mode event");
                    }
                }
            }
        }

        return false;
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
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);

            _runtime.Drive().Brake();
            const DiagnosticSensorSnapshot sensorSnapshot = _runtime.TelemetrySensors().Capture(stationary, _runtime.Drive().GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _runtime.Drive().UpdateOdometry(dtSeconds, sensorSnapshot);
            if (_runtime.Drive().HasEstimatorFault())
            {
                return Fail(_runtime.Drive().GetEstimatorFaultReason());
            }
            const DriveTelemetry driveTelemetry = _runtime.Drive().GetTelemetry();
            const float planarAccelMps2 = _runtime.TelemetrySensors().GetPlanarAccelMps2(sensorSnapshot);
            if (!LogSample(
                stationary,
                _fanEnabled,
                timestampUs,
                dtUs,
                _runtime.Drive().GetPose(),
                _runtime.Drive(),
                driveTelemetry,
                sensorSnapshot,
                planarAccelMps2))
            {
                return Fail("Failed to write auxiliary measurement sample");
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

    void WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)
    {
        while ((micros() - _lastControlMicros) < AuxMeasurementConfig::kControlPeriodUs)
        {
            ServiceLog();
            delayMicroseconds(50);
        }

        timestampUs = micros();
        dtUs = static_cast<uint32_t>(timestampUs - _lastControlMicros);
        _lastControlMicros = timestampUs;
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
        _faulted = true;
        _runtime.Drive().Brake();
        SetFanEnabled(false);
        Serial.print("Auxiliary measurement fault: ");
        Serial.println((reason != nullptr) ? reason : "unknown");
        if (reason != nullptr && reason[0] != '\0')
        {
            WriteEvent("fault", reason);
        }
        AppendStartupTrace((reason != nullptr) ? reason : "aux_measurement_fault");
        return false;
    }

    static MazeMap::WheelControlProfile BuildTurningTractionWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }
};

class FrontWallCharacterizationController : public IApplicationMode
{
public:
    explicit FrontWallCharacterizationController(SharedRobotRuntime& runtime)
        : _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
        , _lastControlMicros(0UL)
    {
    }

    bool Begin() override
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Front wall characterization mode");
        Serial.println("Enter by shorting pins 39-40 at boot.");
        Serial.println("Place the robot with its nose touching a wall, keep the area dark, then power on.");
        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        (void)ResetStartupTrace("mode:front_wall_characterization");
        AppendStartupTrace("front_wall_characterization:begin");
        AppendStartupTrace("front_wall_characterization:sd_ready_wait_begin");
        Serial.print("SD card ready; waiting ");
        Serial.print(FrontWallCharacterizationConfig::kPostSdReadyDelayMs);
        Serial.println(" ms before starting.");
        delay(FrontWallCharacterizationConfig::kPostSdReadyDelayMs);
        AppendStartupTrace("front_wall_characterization:sd_ready_wait_complete");

        SetMissionLevelFanEnabled(false);

        const bool driveOk = _drive.Begin();
        const bool sensorsOk = _sensors.Begin(FrontWallCharacterizationConfig::kControlPeriodUs);
        _drive.UseNominalWheelControlProfile();
        _lastControlMicros = micros();

        MazeMap::FrontWallCharacterizationStorage storedCurve{};
        if (TryReadPersistedFrontWallCharacterization(storedCurve))
        {
            char line[160] = {};
            snprintf(
                line,
                sizeof(line),
                "front_wall_characterization:existing_curve_loaded,samples=%u,terminal_distance_m=%.4f,reverse_speed_mps=%.3f",
                static_cast<unsigned>(storedCurve.sampleCount),
                storedCurve.terminalDistanceM,
                storedCurve.commandedReverseSpeedMps);
            AppendStartupTrace(line);
            Serial.println("Existing persisted front curve found; it will be replaced on success.");
        }

        if (!driveOk)
        {
            return Fail("Drive initialization failed");
        }
        if (!sensorsOk)
        {
            return Fail("Sensor initialization failed");
        }
        return true;
    }

    void Run() override
    {
        MazeMap::FrontWallCharacterizationStorage storage{};
        const bool ok =
            HoldStationary("startup_settle", FrontWallCharacterizationConfig::kStartupSettleMs) &&
            CaptureCurve(storage) &&
            PersistCurve(storage) &&
            ExportCurveToSd(storage) &&
            HoldStationary("post_capture_settle", FrontWallCharacterizationConfig::kPostCaptureSettleMs);

        _drive.Brake();
        SetMissionLevelFanEnabled(false);
        if (ok)
        {
            Serial.println("Front wall characterization complete and persisted.");
        }
    }

private:
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    unsigned long _lastControlMicros;

    bool HoldStationary(const char* phaseName, uint16_t durationMs)
    {
        if (phaseName != nullptr && phaseName[0] != '\0')
        {
            char line[96] = {};
            snprintf(line, sizeof(line), "front_wall_characterization:phase=%s", phaseName);
            AppendStartupTrace(line);
        }

        const unsigned long startMs = millis();
        while (static_cast<unsigned long>(millis() - startMs) < durationMs)
        {
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);
            (void)timestampUs;
            const DiagnosticSensorSnapshot snapshot = _sensors.Capture(true, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, snapshot);
            _drive.Brake();
        }

        return true;
    }

    bool CaptureCurve(MazeMap::FrontWallCharacterizationStorage& storage)
    {
        storage = {};
        storage.distanceStepM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        storage.commandedReverseSpeedMps = FrontWallCharacterizationConfig::kReverseSpeedMps;
        storage.zeroThresholdDifferentialLight = FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold;

        const Eigen::Vector2f targetHeading = _drive.GetPose().headingUnit;
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const DiagnosticSensorSnapshot initialSnapshot = _sensors.Capture(true, _drive.GetPose());
        StoreCurveSample(storage, 0.0f, initialSnapshot);

        float commandedSpeedMps = 0.0f;
        float nextStoredDistanceM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        uint8_t collapsedConsecutiveSamples = 0U;
        const unsigned long startMs = millis();
        const unsigned long timeoutMs =
            static_cast<unsigned long>(2000.0f +
                ((1000.0f * FrontWallCharacterizationConfig::kMaxReverseTravelM) /
                    (std::max)(FrontWallCharacterizationConfig::kReverseSpeedMps, 0.01f)));
        bool elapsedBudgetLogged = false;
        const char* completionReason = "unknown";

        while (true)
        {
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);
            (void)timestampUs;

            const DiagnosticSensorSnapshot snapshot = _sensors.Capture(false, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, snapshot);

            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if ((storage.sampleCount < MazeMap::kFrontWallCharacterizationMaxStoredSamples) &&
                ((traveledDistanceM + Config::kDistanceToleranceM) >= nextStoredDistanceM))
            {
                StoreCurveSample(storage, traveledDistanceM, snapshot);
                nextStoredDistanceM += FrontWallCharacterizationConfig::kStoredDistanceStepM;
            }

            const bool collapsedToZero =
                (snapshot.frontLeft.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold) &&
                (snapshot.frontRight.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold);
            if (traveledDistanceM >= FrontWallCharacterizationConfig::kMinimumTravelBeforeCollapseCheckM && collapsedToZero)
            {
                ++collapsedConsecutiveSamples;
            }
            else
            {
                collapsedConsecutiveSamples = 0U;
            }

            if (storage.sampleCount >= MazeMap::kFrontWallCharacterizationMaxStoredSamples)
            {
                completionReason = "storage_full";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (collapsedConsecutiveSamples >= FrontWallCharacterizationConfig::kCollapsedConsecutiveSamples)
            {
                completionReason = "collapsed_to_zero";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (traveledDistanceM >= FrontWallCharacterizationConfig::kMaxReverseTravelM)
            {
                completionReason = "max_reverse_travel";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (!elapsedBudgetLogged &&
                static_cast<unsigned long>(millis() - startMs) >= timeoutMs)
            {
                char timeoutLine[192] = {};
                snprintf(
                    timeoutLine,
                    sizeof(timeoutLine),
                    "front_wall_characterization:elapsed_budget_reached,travel_m=%.4f,samples=%u,timeout_ms=%lu",
                    traveledDistanceM,
                    static_cast<unsigned>(storage.sampleCount),
                    timeoutMs);
                AppendStartupTrace(timeoutLine);
                elapsedBudgetLogged = true;
            }

            commandedSpeedMps = (std::min)(
                FrontWallCharacterizationConfig::kReverseSpeedMps,
                commandedSpeedMps + (FrontWallCharacterizationConfig::kReverseAccelMps2 * dtSeconds));

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps =
                (Config::kStraightHeadingKp * headingErrorRad) -
                (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -FrontWallCharacterizationConfig::kMaxAngularCommandRadps,
                FrontWallCharacterizationConfig::kMaxAngularCommandRadps);
            _drive.CommandVelocity(-commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        _drive.Brake();
        if (storage.sampleCount < 4U)
        {
            return Fail("Front wall characterization captured too few samples");
        }

        MazeMap::FinalizeFrontWallCharacterizationStorage(storage);

        char summary[224] = {};
        snprintf(
            summary,
            sizeof(summary),
            "front_wall_characterization:captured,reason=%s,samples=%u,terminal_distance_m=%.4f,fl_start=%.6f,fl_end=%.6f,fr_start=%.6f,fr_end=%.6f",
            completionReason,
            static_cast<unsigned>(storage.sampleCount),
            storage.terminalDistanceM,
            storage.frontLeftDifferentialLight[0],
            storage.frontLeftDifferentialLight[storage.sampleCount - 1U],
            storage.frontRightDifferentialLight[0],
            storage.frontRightDifferentialLight[storage.sampleCount - 1U]);
        AppendStartupTrace(summary);
        Serial.println(summary);
        return true;
    }

    static void StoreCurveSample(
        MazeMap::FrontWallCharacterizationStorage& storage,
        float traveledDistanceM,
        const DiagnosticSensorSnapshot& snapshot)
    {
        if (storage.sampleCount >= MazeMap::kFrontWallCharacterizationMaxStoredSamples ||
            !std::isfinite(traveledDistanceM) ||
            traveledDistanceM < 0.0f)
        {
            return;
        }

        const uint16_t index = storage.sampleCount;
        storage.distanceM[index] = traveledDistanceM;
        storage.frontLeftAmbientLight[index] = snapshot.frontLeft.ambientLight;
        storage.frontLeftLitLight[index] = snapshot.frontLeft.litLight;
        storage.frontLeftDifferentialLight[index] = snapshot.frontLeft.differentialLight;
        storage.frontRightAmbientLight[index] = snapshot.frontRight.ambientLight;
        storage.frontRightLitLight[index] = snapshot.frontRight.litLight;
        storage.frontRightDifferentialLight[index] = snapshot.frontRight.differentialLight;
        ++storage.sampleCount;
    }

    bool PersistCurve(const MazeMap::FrontWallCharacterizationStorage& storage)
    {
        if (!WritePersistedFrontWallCharacterization(storage))
        {
            return Fail("Failed to persist front wall characterization");
        }

        MazeMap::FrontWallCharacterizationStorage verify{};
        if (!TryReadPersistedFrontWallCharacterization(verify))
        {
            return Fail("Failed to verify persisted front wall characterization");
        }

        char line[160] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:persisted,samples=%u,terminal_distance_m=%.4f",
            static_cast<unsigned>(verify.sampleCount),
            verify.terminalDistanceM);
        AppendStartupTrace(line);
        Serial.println(line);
        return true;
    }

    bool ExportCurveToSd(const MazeMap::FrontWallCharacterizationStorage& storage)
    {
        if (!MazeMap::IsValidFrontWallCharacterizationStorage(storage))
        {
            return Fail("Invalid front wall characterization cannot be exported");
        }

        char fileName[32] = {};
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "fwc%03u.mmlog",
                "front_wall_characterization.mmlog"))
        {
            return Fail("Front wall characterization log name unavailable");
        }

        MazeMap::mmlog::MmLogLogger log;
        if (!log.open(fileName))
        {
            return Fail("Front wall characterization log open failed");
        }

        if (!log.writeMetadata("file", fileName)) return Fail("Front wall characterization log metadata failed");
        if (!log.writeMetadata("mode", "front_wall_characterization")) return Fail("Front wall characterization log metadata failed");
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(log, "samples", static_cast<unsigned long>(storage.sampleCount))) return Fail("Front wall characterization log metadata failed");
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(log, "distance_step_m", storage.distanceStepM, 6)) return Fail("Front wall characterization log metadata failed");
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(log, "reverse_speed_mps", storage.commandedReverseSpeedMps, 6)) return Fail("Front wall characterization log metadata failed");
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(log, "zero_threshold_differential_light", storage.zeroThresholdDifferentialLight, 6)) return Fail("Front wall characterization log metadata failed");
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(log, "terminal_distance_m", storage.terminalDistanceM, 6)) return Fail("Front wall characterization log metadata failed");
        if (!log.writeMetadata("format_spec", "micromouse_logging_spec_rev_g")) return Fail("Front wall characterization log metadata failed");
        if (!log.writeMetadata("endianness", "little")) return Fail("Front wall characterization log metadata failed");

        FrontWallCharacterizationLogRow row{};
        if (!log.begin(row))
        {
            return Fail("Front wall characterization log open failed");
        }

        for (uint16_t index = 0U; index < storage.sampleCount; ++index)
        {
            row.index = index;
            row.distance_m = storage.distanceM[index];
            row.front_left_ambient = storage.frontLeftAmbientLight[index];
            row.front_left_lit = storage.frontLeftLitLight[index];
            row.front_left_delta = storage.frontLeftDifferentialLight[index];
            row.front_right_ambient = storage.frontRightAmbientLight[index];
            row.front_right_lit = storage.frontRightLitLight[index];
            row.front_right_delta = storage.frontRightDifferentialLight[index];
            if (!log.log(row))
            {
                return Fail("Front wall characterization log write failed");
            }
        }

        if (!log.flush() || !log.close())
        {
            return Fail("Front wall characterization log write failed");
        }

        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:log_exported,file=%s,samples=%u",
            fileName,
            static_cast<unsigned>(storage.sampleCount));
        AppendStartupTrace(line);
        Serial.println(line);
        return true;
    }

    void WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)
    {
        while ((micros() - _lastControlMicros) < FrontWallCharacterizationConfig::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        timestampUs = micros();
        dtUs = static_cast<uint32_t>(timestampUs - _lastControlMicros);
        _lastControlMicros = timestampUs;
    }

    bool Fail(const char* reason)
    {
        _drive.Brake();
        SetMissionLevelFanEnabled(false);
        Serial.print("Front wall characterization fault: ");
        Serial.println((reason != nullptr) ? reason : "unknown");
        if (reason != nullptr && reason[0] != '\0')
        {
            AppendStartupTrace(reason);
        }
        return false;
    }
};

class WallSensorLedCalibrationController : public IApplicationMode
{
public:
    bool Begin() override
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Wall sensor LED calibration mode");
        ResetStartupTrace("mode:wall_sensor_led_calibration");

        pinMode(Pins::LED_Ctrl_Forward_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Forward_Right, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Right, OUTPUT);
        SetFrontLeds(false);
        SetSideLeds(false);
        BeginJumperMonitor();

        Serial.println("Front calibration active; side LEDs held off");
        PrintFrequency("Front LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft));
        Serial.println("Remove jumper on pins 38-39 to switch to side calibration");
        return true;
    }

    void Run() override
    {
        const uint32_t frontHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft);
        const uint32_t sideHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft);
        RunFrontCalibration(frontHalfPeriodUs);

        SetFrontLeds(false);
        SetSideLeds(false);
        Serial.println("Side calibration active; front LEDs held off");
        PrintFrequency("Side LED square wave (Hz): ", sideHalfPeriodUs);
        RunSideCalibration(sideHalfPeriodUs);
        SetFrontLeds(false);
        SetSideLeds(false);
        Serial.println("Wall sensor LED calibration complete");
    }

private:
    static void BeginJumperMonitor()
    {
        pinMode(LedCalibrationConfig::kModeSelectPinA, OUTPUT);
        digitalWriteFast(LedCalibrationConfig::kModeSelectPinA, LOW);
        pinMode(LedCalibrationConfig::kModeSelectPinB, INPUT_PULLUP);
    }

    static bool IsCalibrationJumperInstalled()
    {
        return digitalReadFast(LedCalibrationConfig::kModeSelectPinB) == LOW;
    }

    static void SetFrontLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Forward_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Forward_Right, enabled ? HIGH : LOW);
    }

    static void SetSideLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Side_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Side_Right, enabled ? HIGH : LOW);
    }

    static void RunFrontCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Front, IsCalibrationJumperInstalled()) ==
               WallSensorLedCalibrationPhase::Front)
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetFrontLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void RunSideCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Side, IsCalibrationJumperInstalled()) ==
               WallSensorLedCalibrationPhase::Side)
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetSideLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void PrintFrequency(const char* label, uint32_t halfPeriodUs)
    {
        Serial.print(label);
        if (halfPeriodUs == 0U)
        {
            Serial.println(0.0f, 3);
            return;
        }

        Serial.println(1000000.0f / (2.0f * static_cast<float>(halfPeriodUs)), 3);
    }
};

class DiagnosticController : public IApplicationMode
{
public:
    explicit DiagnosticController(SharedRobotRuntime& runtime)
        : _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
        , _sampleLog()
        , _eventLog()
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _lastControlMicros(0UL)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Micromouse diagnostic setup");

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:primary_diagnostic");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        _drive.SetWheelControlProfile(BuildDiagnosticWheelControlProfile());
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            return Fail("Diagnostic sensor init failed");
        }
        if (!BeginLog())
        {
            return Fail("Diagnostic log open failed");
        }

        _drive.SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _startX = _drive.GetPose().xMeters;
        _startY = _drive.GetPose().yMeters;
        _lastControlMicros = micros();
        return true;
    }

    void Run() override
    {
        if (_faulted)
        {
            return;
        }

        bool ok = true;
        ok = ok && HoldPhase("startup_settle", DiagnosticConfig::kStartupSettleMs, true);
        ok = ok && HoldPhase("baseline_idle", DiagnosticConfig::kBaselineHoldMs, true);
        ok = ok && ExecuteKickoffSweep();
        ok = ok && ExecuteForwardSweep();
        ok = ok && HoldPhase("characterization_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteTurnPhase("turn_cw_90_1", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_1", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_90_2", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_2", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_180", PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_180", -PI_F);
        ok = ok && HoldPhase("turn_sweep_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float shortReturnDistanceM = DiagnosticConfig::kShortStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_short_forward", DiagnosticConfig::kShortStraightDistanceM, DiagnosticConfig::kSlowStraightSpeedMps, &shortReturnDistanceM);
        shortReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(DiagnosticConfig::kShortStraightDistanceM, shortReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_short_turnaround", PI_F);
        ok = ok && ExecuteStraightPhase("straight_short_return", shortReturnDistanceM, DiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_short_reset_heading", PI_F);
        ok = ok && HoldPhase("straight_short_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float longReturnDistanceM = DiagnosticConfig::kLongStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_long_forward", DiagnosticConfig::kLongStraightDistanceM, DiagnosticConfig::kFastStraightSpeedMps, &longReturnDistanceM);
        longReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(DiagnosticConfig::kLongStraightDistanceM, longReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_long_turnaround", PI_F);
        ok = ok && ExecuteStraightPhase("straight_long_return", longReturnDistanceM, DiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_long_reset_heading", PI_F);
        ok = ok && HoldPhase("straight_long_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteCircleSpeedSweep("slow", DiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("medium", DiagnosticConfig::kCircleMediumSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("fast", DiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteSquareLoop("square_cw", HALF_PI_F);
        ok = ok && HoldPhase("square_cw_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteSquareLoop("square_ccw", -HALF_PI_F);
        ok = ok && HoldPhase("final_idle", DiagnosticConfig::kBaselineHoldMs / 2U, true);

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        FlushLog();

        if (ok)
        {
            Serial.print("Diagnostic complete, log saved to ");
            Serial.println(GetLogFileName());
            Serial.println("Use the # event,summary lines in the log header to map phases to tunables.");
        }

        CloseLog();
        SetMissionLevelFanEnabled(false);
    }

private:
    struct StraightPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
    };

    struct TurnPhaseMetrics
    {
        float peakOmegaRadps = 0.0f;
        float maxYawErrorRad = 0.0f;
    };

    struct ArcPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float peakOmegaRadps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
        float durationSeconds = 0.0f;
        float omegaIntegralRad = 0.0f;
        float speedIntegralMpsSeconds = 0.0f;
        float planarAccelIntegralMps2Seconds = 0.0f;
        float peakPlanarAccelMps2 = 0.0f;
    };

    MazeMap::Vehicle& _vehicle;
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    MazeMap::mmlog::MmLogLogger _sampleLog;
    MazeMap::App::Internal::Runtime::OptionalRuntimeEventLog _eventLog;
    char _logFileName[64];
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    unsigned long _lastControlMicros;

    bool BeginLog()
    {
        _phaseId = 0UL;
        _sampleCount = 0UL;
        _eventLog.Close();
        _logFileName[0] = '\0';
        (void)_sampleLog.close();
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                _logFileName,
                sizeof(_logFileName),
                nullptr,
                "diag%03u.mmlog",
                "diagnostic_log.mmlog"))
        {
            return false;
        }
        if (!_eventLog.BeginSibling(_logFileName))
        {
            return false;
        }
        if (!_sampleLog.open(_logFileName))
        {
            _eventLog.Close();
            return false;
        }

        if (!_sampleLog.writeMetadata("file", _logFileName)) return false;
        if (_eventLog.IsEnabled() && !_sampleLog.writeMetadata("control_log_file", _eventLog.GetFileName())) return false;
        if (!_sampleLog.writeMetadata("mode", "diagnostic")) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(_sampleLog, "control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(_sampleLog, "imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                DiagnosticConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_sampleLog, "mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteMmLogAccelBiasMetadata(_sampleLog, _sensors)) return false;
        if (!_sampleLog.writeMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_sampleLog.writeMetadata("endianness", "little")) return false;
        if (!MazeMap::App::Internal::Runtime::WriteDiagnosticTuningEvents(_eventLog)) return false;

        DiagnosticLogRow row{};
        if (!_sampleLog.begin(row))
        {
            _eventLog.Close();
            return false;
        }

        if (_eventLog.IsEnabled())
        {
            (void)_eventLog.WriteMetadata("file", _eventLog.GetFileName());
            (void)_eventLog.WriteMetadata("data_file", _logFileName);
            (void)_eventLog.WriteMetadata("mode", "diagnostic");
        }
        return MazeMap::App::Internal::Runtime::WriteDiagnosticSummaryInstructions(_eventLog);
    }

    bool WriteLogEvent(const char* type, const char* message)
    {
        return _eventLog.WriteEvent(micros(), type, message);
    }

    bool WritePhaseMarker(const char* name)
    {
        ++_phaseId;
        return _eventLog.WritePhase(_phaseId, micros(), name);
    }

    void ServiceLog()
    {
        (void)_sampleLog.service();
    }

    void FlushLog()
    {
        (void)_sampleLog.flush();
        _eventLog.Flush();
    }

    void CloseLog()
    {
        (void)_sampleLog.close();
        _eventLog.Close();
    }

    const char* GetLogFileName() const
    {
        return _logFileName;
    }

    bool WriteStraightResult(
        const char* phaseName,
        float distanceM,
        float cruiseSpeedMps,
        float traveledM,
        const Eigen::Vector2f& targetHeading,
        const StraightPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float stopErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;distance_m=%.3f;cruise_mps=%.3f;peak_speed_mps=%.3f;max_heading_err_deg=%.2f;stop_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            cruiseSpeedMps,
            metrics.peakSpeedMps,
            RAD_TO_DEG_F * metrics.maxHeadingErrorRad,
            stopErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format straight diagnostic result");
        }
        return WriteEventOrFail("straight_result", message, "Failed to write straight diagnostic result");
    }

    bool WriteTurnResult(const char* phaseName, float angleRad, const TurnPhaseMetrics& metrics, float targetYawRad)
    {
        char message[176] = {};
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;angle_deg=%.1f;peak_omega_radps=%.3f;peak_yaw_err_deg=%.2f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            RAD_TO_DEG_F * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG_F * metrics.maxYawErrorRad,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format turn diagnostic result");
        }
        return WriteEventOrFail("turn_result", message, "Failed to write turn diagnostic result");
    }

    bool WriteArcResult(
        const char* phaseName,
        float distanceM,
        float angleRad,
        float traveledM,
        float targetYawRad,
        const ArcPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float distanceErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;dist_m=%.3f;ang_deg=%.1f;peak_w_radps=%.3f;max_head_err_deg=%.2f;dist_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            RAD_TO_DEG_F * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG_F * metrics.maxHeadingErrorRad,
            distanceErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format arc diagnostic result");
        }
        return WriteEventOrFail("arc_result", message, "Failed to write arc diagnostic result");
    }

    bool WriteCircleResult(
        const char* phaseName,
        float cruiseSpeedMps,
        const DriveTelemetry& startTelemetry,
        const ArcPhaseMetrics& metrics)
    {
        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const long leftCountDelta = static_cast<long>(endTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount);
        const long rightCountDelta = static_cast<long>(endTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount);
        const float leftDistanceDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDistanceDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;
        const float averageOmegaRadps = (metrics.durationSeconds > 0.0f) ? (metrics.omegaIntegralRad / metrics.durationSeconds) : 0.0f;
        const float averageSpeedMps = (metrics.durationSeconds > 0.0f) ? (metrics.speedIntegralMpsSeconds / metrics.durationSeconds) : 0.0f;
        const float effectiveTrackWidthM =
            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(averageSpeedMps, averageOmegaRadps);
        const float encoderYawDeg =
            (effectiveTrackWidthM > 0.0f)
            ? (RAD_TO_DEG_F * ((rightDistanceDeltaM - leftDistanceDeltaM) / effectiveTrackWidthM))
            : 0.0f;
        const float estimatedLateralAccelMps2 = std::fabs(averageSpeedMps * averageOmegaRadps);
        const float averageLateralAccelMps2 = (metrics.durationSeconds > 0.0f) ? (metrics.planarAccelIntegralMps2Seconds / metrics.durationSeconds) : 0.0f;

        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cruise_mps=%.3f;l_cnt=%ld;r_cnt=%ld;enc_yaw_deg=%.1f;avg_speed_mps=%.3f;avg_omega_radps=%.3f;est_lat_mps2=%.3f;avg_lat_mps2=%.3f;peak_lat_mps2=%.3f",
            (phaseName != nullptr) ? phaseName : "",
            cruiseSpeedMps,
            leftCountDelta,
            rightCountDelta,
            encoderYawDeg,
            averageSpeedMps,
            averageOmegaRadps,
            estimatedLateralAccelMps2,
            averageLateralAccelMps2,
            metrics.peakPlanarAccelMps2);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format circle diagnostic result");
        }
        return WriteEventOrFail("circle_result", message, "Failed to write circle diagnostic result");
    }

    bool WriteClosureResult(const char* type, const char* phaseName, const PoseEstimate& startPose, const char* failMessage)
    {
        char message[160] = {};
        const PoseEstimate& pose = _drive.GetPose();
        const float deltaXM = pose.xMeters - startPose.xMeters;
        const float deltaYM = pose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, pose.yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;closure_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            closureErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format diagnostic closure result");
        }
        return WriteEventOrFail(type, message, failMessage);
    }

    static void AccumulateArcMetrics(ArcPhaseMetrics& total, const ArcPhaseMetrics& segment)
    {
        total.peakSpeedMps = (std::max)(total.peakSpeedMps, segment.peakSpeedMps);
        total.peakOmegaRadps = (std::max)(total.peakOmegaRadps, segment.peakOmegaRadps);
        total.maxHeadingErrorRad = (std::max)(total.maxHeadingErrorRad, segment.maxHeadingErrorRad);
        total.durationSeconds += segment.durationSeconds;
        total.omegaIntegralRad += segment.omegaIntegralRad;
        total.speedIntegralMpsSeconds += segment.speedIntegralMpsSeconds;
        total.planarAccelIntegralMps2Seconds += segment.planarAccelIntegralMps2Seconds;
        total.peakPlanarAccelMps2 = (std::max)(total.peakPlanarAccelMps2, segment.peakPlanarAccelMps2);
    }

    static MotionLimits DiagnosticLimits(float maxSpeedMps)
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = DiagnosticConfig::kTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = DiagnosticConfig::kTurnAccelRadps2;
        return limits;
    }

    bool WriteEventOrFail(const char* type, const char* message, const char* failMessage)
    {
        if (WriteLogEvent(type, message))
        {
            return true;
        }

        return Fail(failMessage);
    }

    static void BuildDriveCommandLabel(const char* prefix, float driveCommand, char* buffer, size_t bufferSize)
    {
        const unsigned drivePercent = static_cast<unsigned>((100.0f * driveCommand) + 0.5f);
        snprintf(buffer, bufferSize, "%s_%03u", (prefix != nullptr) ? prefix : "cmd", drivePercent);
    }

    static MazeMap::WheelControlProfile BuildDiagnosticWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }

    bool Fail(const char* message)
    {
        _faulted = true;
        SetMissionLevelFanEnabled(false);
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        Serial.print("DIAGNOSTIC FAULT: ");
        Serial.println(message);
        WriteLogEvent("fault", message);
        FlushLog();
        return false;
    }

    bool StartPhase(const char* name)
    {
        Serial.print("Diagnostic phase: ");
        Serial.println(name);
        if (WritePhaseMarker(name))
        {
            return true;
        }
        return Fail("Failed to write diagnostic phase marker");
    }

    bool IsWithinBoundary() const
    {
        const PoseEstimate& pose = _drive.GetPose();
        return (std::fabs(pose.xMeters - _startX) <= DiagnosticConfig::kBoundaryHalfSpanM) &&
            (std::fabs(pose.yMeters - _startY) <= DiagnosticConfig::kBoundaryHalfSpanM);
    }

    bool TickControl(bool stationary, float& dtSeconds, uint32_t& timestampUs, DiagnosticSensorSnapshot& snapshot)
    {
        while ((micros() - _lastControlMicros) < DiagnosticConfig::kControlPeriodUs)
        {
            ServiceLog();
            delayMicroseconds(20);
        }

        timestampUs = micros();
        dtSeconds = static_cast<float>(timestampUs - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = timestampUs;

        snapshot = _sensors.Capture(
            stationary,
            _drive.GetPose(),
            [this, dtSeconds](DiagnosticSensorSnapshot& captureSnapshot, auto&& serviceWallRead, auto&& captureImu) noexcept
            {
                _drive.UpdateOdometry(
                    dtSeconds,
                    captureSnapshot,
                    nullptr,
                    nullptr,
                    [&serviceWallRead]() noexcept
                    {
                        serviceWallRead();
                    },
                    [&captureImu]() noexcept
                    {
                        captureImu();
                    });
            },
            [this]() noexcept
            {
                FlushLog();
            });
        if (_drive.HasEstimatorFault())
        {
            return Fail(_drive.GetEstimatorFaultReason());
        }

        if (!IsWithinBoundary())
        {
            return Fail("Diagnostic boundary exceeded");
        }

        return true;
    }

    bool LogSample(bool stationary, uint32_t timestampUs, float dtSeconds, const DiagnosticSensorSnapshot& snapshot)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        const uint32_t dtUs = static_cast<uint32_t>(dtSeconds * 1.0e6f);
        DiagnosticLogRow row{};
        MazeMap::App::Internal::Runtime::PopulateDiagnosticLogRow(
            row,
            _sampleCount,
            _phaseId,
            stationary,
            timestampUs,
            dtUs,
            _drive.GetPose(),
            _drive,
            telemetry,
            snapshot);
        if (_sampleLog.log(row))
        {
            ++_sampleCount;
            return true;
        }
        return Fail("Failed to write diagnostic sample");
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const unsigned long deadline = millis() + durationMs;
        while (static_cast<long>(deadline - millis()) > 0)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(stationary, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            _drive.Brake();
            if (!LogSample(true, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteStraightPhase(const char* phaseName, float distanceM, float cruiseSpeedMps, float* outTraveledDistanceM = nullptr)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const MotionLimits limits = DiagnosticLimits(cruiseSpeedMps);
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const Eigen::Vector2f targetHeading = _drive.GetPose().headingUnit;
        float commandedSpeedMps = 0.0f;
        float traveledM = 0.0f;
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (6000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        StraightPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            metrics.peakSpeedMps = (std::max)(metrics.peakSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Straight diagnostic encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Straight diagnostic phase timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(limits.maxSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            metrics.maxHeadingErrorRad = (std::max)(metrics.maxHeadingErrorRad, std::fabs(headingErrorRad));
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        if (outTraveledDistanceM != nullptr)
        {
            *outTraveledDistanceM = traveledM;
        }

        return WriteStraightResult(phaseName, distanceM, cruiseSpeedMps, traveledM, targetHeading, metrics);
    }

    bool RecoverCharacterizationSample(const char* label, float traveledDistanceM)
    {
        char phaseName[48] = {};
        if (traveledDistanceM <= DiagnosticConfig::kKickoffSweepMoveThresholdM)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
            return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
        }

        snprintf(phaseName, sizeof(phaseName), "%s_turnaround", label);
        if (!ExecuteTurnPhase(phaseName, PI_F))
        {
            return false;
        }

        // Recover characterization samples with the same forward-drive path used elsewhere in diagnostics.
        // This avoids the poorly controlled reverse leg that can drift far past the available space.
        snprintf(phaseName, sizeof(phaseName), "%s_return", label);
        if (!ExecuteStraightPhase(phaseName, traveledDistanceM, DiagnosticConfig::kCharacterizationRecoverySpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_reset_heading", label);
        if (!ExecuteTurnPhase(phaseName, PI_F))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        _drive.SetPose(pose.xMeters, pose.yMeters, DirectionToYawRad(MazeMap::Up));
        _lastControlMicros = micros();

        snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
        return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
    }

    bool ExecuteKickoffCharacterizationSample(float driveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("kickoff", driveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const unsigned long pulseDeadlineMs = millis() + DiagnosticConfig::kKickoffSweepPulseMs;
        const unsigned long settleDeadlineMs = pulseDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        const float travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        float maxSpeedMps = 0.0f;
        bool travelLimited = false;
        unsigned long travelLimitSettleDeadlineMs = 0UL;

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const unsigned long nowMs = millis();
            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if (!travelLimited && travelLimitM > 0.0f && traveledDistanceM >= travelLimitM)
            {
                travelLimited = true;
                travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
            }

            const bool pulseActive = !travelLimited && static_cast<long>(pulseDeadlineMs - nowMs) > 0;
            if (travelLimited)
            {
                _drive.Brake();
            }
            else if (pulseActive)
            {
                _drive.CommandOpenLoopRaw(driveCommand, driveCommand);
            }
            else
            {
                _drive.Brake();
            }

            maxSpeedMps = (std::max)(maxSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }

            if (travelLimited &&
                static_cast<long>(travelLimitSettleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }

            if (!travelLimited &&
                !pulseActive &&
                static_cast<long>(settleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }
        }

        _drive.Brake();
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const bool moved =
            (traveledDistanceM >= DiagnosticConfig::kKickoffSweepMoveThresholdM) ||
            (maxSpeedMps >= DiagnosticConfig::kKickoffSweepMoveThresholdMps);

        char message[192] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cmd=%.2f;dist_m=%.4f;max_speed_mps=%.3f;moved=%u;travel_limited=%u",
            label,
            driveCommand,
            traveledDistanceM,
            maxSpeedMps,
            moved ? 1U : 0U,
            travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format kickoff characterization result");
        }
        if (!WriteEventOrFail("kickoff_result", message, "Failed to write kickoff characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, traveledDistanceM);
    }

    bool ExecuteForwardCharacterizationSample(float forwardDriveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("forward", forwardDriveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const unsigned long kickoffDeadlineMs = millis() + DiagnosticConfig::kForwardSweepKickoffMs;
        const unsigned long holdDeadlineMs = kickoffDeadlineMs + DiagnosticConfig::kForwardSweepHoldMs;
        const unsigned long settleDeadlineMs = holdDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        const float travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        float maxSpeedMps = 0.0f;
        float holdStartDistanceM = 0.0f;
        float holdEndDistanceM = 0.0f;
        float holdElapsedSeconds = 0.0f;
        bool holdStarted = false;
        bool holdComplete = false;
        bool travelLimited = false;
        unsigned long travelLimitSettleDeadlineMs = 0UL;

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const unsigned long nowMs = millis();
            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if (!travelLimited && travelLimitM > 0.0f && traveledDistanceM >= travelLimitM)
            {
                travelLimited = true;
                travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
                if (holdStarted && !holdComplete)
                {
                    holdComplete = true;
                    holdEndDistanceM = _drive.GetAverageDistanceMeters();
                }
            }

            if (travelLimited)
            {
                _drive.Brake();
            }
            else if (static_cast<long>(kickoffDeadlineMs - nowMs) > 0)
            {
                _drive.CommandOpenLoopRaw(
                    DiagnosticConfig::kForwardSweepKickoffDriveCommand,
                    DiagnosticConfig::kForwardSweepKickoffDriveCommand);
            }
            else if (static_cast<long>(holdDeadlineMs - nowMs) > 0)
            {
                if (!holdStarted)
                {
                    holdStarted = true;
                    holdStartDistanceM = _drive.GetAverageDistanceMeters();
                }
                holdElapsedSeconds += dtSeconds;
                _drive.CommandOpenLoopRaw(forwardDriveCommand, forwardDriveCommand);
            }
            else
            {
                if (!holdComplete)
                {
                    holdComplete = true;
                    holdEndDistanceM = _drive.GetAverageDistanceMeters();
                }
                _drive.Brake();
            }

            maxSpeedMps = (std::max)(maxSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }

            if (travelLimited &&
                static_cast<long>(travelLimitSettleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }

            if (!travelLimited &&
                holdComplete &&
                static_cast<long>(settleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }
        }

        _drive.Brake();
        if (!holdComplete)
        {
            holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        const float totalDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const float holdDistanceM = holdStarted ? std::fabs(holdEndDistanceM - holdStartDistanceM) : 0.0f;
        const float averageHoldSpeedMps = (holdElapsedSeconds > 0.0f) ? (holdDistanceM / holdElapsedSeconds) : 0.0f;
        const bool carried =
            (averageHoldSpeedMps >= DiagnosticConfig::kForwardSweepCarryThresholdMps) ||
            (holdDistanceM >= DiagnosticConfig::kForwardSweepCarryThresholdM);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;kickoff=%.2f;hold=%.2f;hold_dist_m=%.4f;hold_avg_speed_mps=%.3f;total_dist_m=%.4f;max_speed_mps=%.3f;carried=%u;travel_limited=%u",
            label,
            DiagnosticConfig::kForwardSweepKickoffDriveCommand,
            forwardDriveCommand,
            holdDistanceM,
            averageHoldSpeedMps,
            totalDistanceM,
            maxSpeedMps,
            carried ? 1U : 0U,
            travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format forward characterization result");
        }
        if (!WriteEventOrFail("forward_result", message, "Failed to write forward characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, totalDistanceM);
    }

    bool ExecuteKickoffSweep()
    {
        if (!HoldPhase("kickoff_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand = DiagnosticConfig::kKickoffSweepMinDriveCommand;
            driveCommand <= (DiagnosticConfig::kKickoffSweepMaxDriveCommand + 0.0001f);
            driveCommand += DiagnosticConfig::kKickoffSweepStepDriveCommand)
        {
            if (!ExecuteKickoffCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteForwardSweep()
    {
        if (!HoldPhase("forward_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand = DiagnosticConfig::kForwardSweepMinDriveCommand;
            driveCommand <= (DiagnosticConfig::kForwardSweepMaxDriveCommand + 0.0001f);
            driveCommand += DiagnosticConfig::kForwardSweepStepDriveCommand)
        {
            if (!ExecuteForwardCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteTurnPhase(const char* phaseName, float angleRad)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_vehicle);
        float commandedOmegaRadps = 0.0f;
        const unsigned long timeoutMs = millis() + 3000UL;
        TurnPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            const float remainingRad = std::fabs(errorRad);
            metrics.maxYawErrorRad = (std::max)(metrics.maxYawErrorRad, remainingRad);
            metrics.peakOmegaRadps = (std::max)(metrics.peakOmegaRadps, std::fabs(_drive.GetPose().angularSpeedRadps));
            if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Turn diagnostic phase timed out");
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
                return Fail("Turn diagnostic phase profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return WriteTurnResult(phaseName, angleRad, metrics, targetYawRad);
    }

    bool ExecuteArcPhase(const char* phaseName, float distanceM, float angleRad, float cruiseSpeedMps, ArcPhaseMetrics* outMetrics = nullptr)
    {
        if (distanceM <= 0.0f)
        {
            return Fail("Diagnostic arc distance must be positive");
        }
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const MotionLimits limits = DiagnosticLimits(cruiseSpeedMps);
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const float targetYawRad = WrapAngleRad(startYawRad + angleRad);
        const float curvature = angleRad / distanceM;
        float commandedSpeedMps = 0.0f;
        float traveledM = 0.0f;
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        ArcPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            metrics.peakSpeedMps = (std::max)(metrics.peakSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            metrics.peakOmegaRadps = (std::max)(metrics.peakOmegaRadps, std::fabs(_drive.GetPose().angularSpeedRadps));
            metrics.durationSeconds += dtSeconds;
            metrics.omegaIntegralRad += _drive.GetPose().angularSpeedRadps * dtSeconds;
            metrics.speedIntegralMpsSeconds += std::fabs(_drive.GetPose().linearSpeedMps) * dtSeconds;
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(snapshot);
            metrics.planarAccelIntegralMps2Seconds += planarAccelMps2 * dtSeconds;
            metrics.peakPlanarAccelMps2 = (std::max)(metrics.peakPlanarAccelMps2, planarAccelMps2);
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Arc diagnostic encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Arc diagnostic phase timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float progress = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
            const float phaseTargetYawRad = WrapAngleRad(startYawRad + (angleRad * progress));
            const float headingErrorRad = AngleErrorRad(phaseTargetYawRad, _drive.GetPose().yawRad);
            metrics.maxHeadingErrorRad = (std::max)(metrics.maxHeadingErrorRad, std::fabs(headingErrorRad));
            float angularCommandRadps = (curvature * commandedSpeedMps) + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        if (outMetrics != nullptr)
        {
            *outMetrics = metrics;
        }
        return WriteArcResult(phaseName, distanceM, angleRad, traveledM, targetYawRad, metrics);
    }

    bool ExecuteArcCircle(const char* namePrefix, float halfCircleAngleRad, float halfCircleDistanceM, float cruiseSpeedMps)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        ArcPhaseMetrics totalMetrics{};
        ArcPhaseMetrics segmentMetrics{};

        snprintf(phaseName, sizeof(phaseName), "%s_half_1", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        snprintf(phaseName, sizeof(phaseName), "%s_half_2", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        if (!WriteCircleResult(namePrefix, cruiseSpeedMps, startTelemetry, totalMetrics))
        {
            return false;
        }

        return WriteClosureResult("arc_circle_result", namePrefix, startPose, "Failed to write arc circle diagnostic result");
    }

    bool ExecuteCircleSpeedSweep(const char* speedLabel, float cruiseSpeedMps)
    {
        char phaseName[48] = {};

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, PI_F, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_ccw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, -PI_F, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        return HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true);
    }

    bool ExecuteSquareLoop(const char* namePrefix, float turnAngleRad)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        for (uint8_t leg = 0; leg < 4U; ++leg)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_leg_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteStraightPhase(phaseName, DiagnosticConfig::kSquareLegDistanceM, DiagnosticConfig::kSlowStraightSpeedMps))
            {
                return false;
            }

            snprintf(phaseName, sizeof(phaseName), "%s_turn_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteTurnPhase(phaseName, turnAngleRad))
            {
                return false;
            }
        }

        return WriteClosureResult("square_result", namePrefix, startPose, "Failed to write square diagnostic result");
    }
};

class OpenFloorMeasurementController : public IApplicationMode
{
public:
    explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime);

    bool Begin() override;
    void Run() override;

private:
    static constexpr uint16_t kWatchdogFlagTranslationStall = 1u << 0;
    static constexpr uint16_t kWatchdogFlagSectionTimeout = 1u << 1;
    static constexpr uint16_t kWatchdogFlagRecoveryTimeout = 1u << 2;
    static constexpr unsigned long kMinimumFailureTimeoutMs = 60000UL;

    MazeMap::Vehicle& _vehicle;
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    //MazeMap::mmlog::MmLogLogger _timingLog;
    //MazeMap::App::Internal::Runtime::OptionalRuntimeEventLog _timingEventLog;
    //bool _timingOverflowed;
    //bool _timingWriteFailed;
    //MazeMap::mmlog::MmLogLogger _mainLog;
    //MazeMap::App::Internal::Runtime::OptionalRuntimeEventLog _mainEventLog;
    //bool _mainOverflowed;
    //bool _mainWriteFailed;
    //OpenFloorRunManifestWriter _manifestWriter;
    bool _faulted;
    bool _timingLogOpen;
    bool _mainLogOpen;
    unsigned long _lastControlMicros;
    uint16_t _timingTickIndex;
    uint32_t _controlTickSequence;
    char _runId[24];

    static MotionLimits MeasurementLimits(float maxSpeedMps);
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
    void ServiceTimingLog();
    void FlushTimingLog();
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
    void ServiceMainLog();
    void FlushMainLog();
    void CloseMainLog();
    void RecordMainLogFailure() noexcept;

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
    : _vehicle(runtime.SpeedVehicle())
    , _sensors(runtime.DiagnosticSensors())
    , _drive(runtime.Drive())
    , _timingLog()
    , _timingEventLog()
    , _timingOverflowed(false)
    , _timingWriteFailed(false)
    , _mainLog()
    , _mainEventLog()
    , _mainOverflowed(false)
    , _mainWriteFailed(false)
    , _manifestWriter()
    , _faulted(false)
    , _timingLogOpen(false)
    , _mainLogOpen(false)
    , _lastControlMicros(0UL)
    , _timingTickIndex(0U)
    , _controlTickSequence(0UL)
{
    _runId[0] = '\0';
}

bool OpenFloorMeasurementController::BeginTimingLog()
{
    _timingOverflowed = false;
    _timingWriteFailed = false;
    _timingEventLog.Close();
    (void)_timingLog.close();

    if (!_timingEventLog.BeginSibling(MazeMap::kOpenFloorTimingFileName))
    {
        return Fail("Timing log open failed1");
    }

    if (!_timingLog.open(MazeMap::kOpenFloorTimingFileName))
    {
        _timingEventLog.Close();
        return Fail("Timing log open failed2");
    }

    if (!_timingLog.writeMetadata("file", MazeMap::kOpenFloorTimingFileName)) return false;
    if (_timingEventLog.IsEnabled() && !_timingLog.writeMetadata("control_log_file", _timingEventLog.GetFileName())) return false;
    if (!_timingLog.writeMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
    if (!_timingLog.writeMetadata("stream_type", "open_floor_timing")) return false;
    if (!_timingLog.writeMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return false;
    if (!_timingLog.writeMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
    if (!_timingLog.writeMetadata("endianness", "little")) return false;
    if (_runId[0] != '\0' && !_timingLog.writeMetadata("run_id", _runId)) return false;
    if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(_timingLog, "control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;

    OpenFloorTimingRow row{};
    if (!_timingLog.begin(row))
    {
        _timingEventLog.Close();
        return Fail("Timing log open failed3");
    }

    if (_timingEventLog.IsEnabled())
    {
        (void)_timingEventLog.WriteMetadata("file", _timingEventLog.GetFileName());
        (void)_timingEventLog.WriteMetadata("data_file", MazeMap::kOpenFloorTimingFileName);
        (void)_timingEventLog.WriteMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName);
        (void)_timingEventLog.WriteMetadata("stream_type", "open_floor_timing_control_log");
        (void)_timingEventLog.WriteMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision);
        if (_runId[0] != '\0')
        {
            (void)_timingEventLog.WriteMetadata("run_id", _runId);
        }
    }
    return true;
}

bool OpenFloorMeasurementController::LogTimingSample(const OpenFloorMeasurementCycle& cycle)
{
    OpenFloorTimingRow row{};
    row.mono_time_us = cycle.masterTimeUs;
    row.control_tick_sequence = cycle.controlTickSequence;
    row.dt_us = cycle.dtUs;
    row.section_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
    row.logger_flags = OpenFloorLoggingV2::LoggerFlags(_timingOverflowed, _timingWriteFailed);
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

    if (!_timingLog.log(row))
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
    return _timingEventLog.WriteEvent(micros(), "fault", message);
}

void OpenFloorMeasurementController::ServiceTimingLog()
{
    if (!_timingLog.service())
    {
        RecordTimingLogFailure();
    }
}

void OpenFloorMeasurementController::FlushTimingLog()
{
    if (!_timingLog.flush())
    {
        RecordTimingLogFailure();
    }
    _timingEventLog.Flush();
}

void OpenFloorMeasurementController::CloseTimingLog()
{
    if (!_timingLog.close())
    {
        RecordTimingLogFailure();
    }
    _timingEventLog.Close();
}

void OpenFloorMeasurementController::RecordTimingLogFailure() noexcept
{
    MazeMap::App::Internal::Runtime::CaptureMmLogFailure(_timingLog, _timingOverflowed, _timingWriteFailed);
}

bool OpenFloorMeasurementController::BeginMainLog()
{
    _mainOverflowed = false;
    _mainWriteFailed = false;
    _mainEventLog.Close();
    (void)_mainLog.close();

    if (!_mainEventLog.BeginSibling(MazeMap::kOpenFloorMainFileName))
    {
        return false;
    }

    if (!_mainLog.open(MazeMap::kOpenFloorMainFileName))
    {
        _mainEventLog.Close();
        return false;
    }

    if (!_mainLog.writeMetadata("file", MazeMap::kOpenFloorMainFileName)) return false;
    if (_mainEventLog.IsEnabled() && !_mainLog.writeMetadata("control_log_file", _mainEventLog.GetFileName())) return false;
    if (!_mainLog.writeMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
    if (!_mainLog.writeMetadata("stream_type", "open_floor_main")) return false;
    if (!_mainLog.writeMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return false;
    if (!_mainLog.writeMetadata("active_imu_id", MazeMap::kOpenFloorActiveImuId)) return false;
    if (!_mainLog.writeMetadata("imu_extrinsics_revision", MazeMap::kOpenFloorImuExtrinsicsRevision)) return false;
    if (!_mainLog.writeMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
    if (!_mainLog.writeMetadata("endianness", "little")) return false;
    if (_runId[0] != '\0' && !_mainLog.writeMetadata("run_id", _runId)) return false;
    if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataUnsigned(_mainLog, "control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
    if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_mainLog, "imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
    if (!MazeMap::App::Internal::Runtime::WriteMmLogMetadataFloat(_mainLog, "imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;

    OpenFloorMainRow row{};
    if (!_mainLog.begin(row))
    {
        _mainEventLog.Close();
        return false;
    }

    if (_mainEventLog.IsEnabled())
    {
        (void)_mainEventLog.WriteMetadata("file", _mainEventLog.GetFileName());
        (void)_mainEventLog.WriteMetadata("data_file", MazeMap::kOpenFloorMainFileName);
        (void)_mainEventLog.WriteMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName);
        (void)_mainEventLog.WriteMetadata("stream_type", "open_floor_main_control_log");
        (void)_mainEventLog.WriteMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision);
        if (_runId[0] != '\0')
        {
            (void)_mainEventLog.WriteMetadata("run_id", _runId);
        }
    }
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
    row.start_marker_id = static_cast<std::uint16_t>(labels.startMarkerId);
    row.mirrored = MazeMap::OpenFloorPrimitiveIsMirrored(labels.primitiveId) ? 1U : 0U;
    row.repeat_index = labels.repeatIndex;
    row.progress_norm = labels.progressNorm;
    row.mode_flags = cycle.driveTelemetry.modeFlags;
    row.clipping_flags = cycle.clippingFlags;
    row.saturation_flags = cycle.driveTelemetry.saturationFlags;
    row.logger_flags = OpenFloorLoggingV2::LoggerFlags(_mainOverflowed, _mainWriteFailed);
    row.watchdog_flags = cycle.watchdogFlags;
    row.measurement_flags = OpenFloorLoggingV2::MeasurementFlags(
        labels,
        cycle,
        encoderValid,
        imuValid,
        frontLeftObs,
        frontRightObs,
        leftObs,
        rightObs);
    row.pose_x_m = _drive.GetPose().xMeters;
    row.pose_y_m = _drive.GetPose().yMeters;
    row.pose_yaw_rad = _drive.GetPose().yawRad;
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

    if (!_mainLog.log(row))
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
    return _mainEventLog.WriteEvent(micros(), "fault", message);
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
    return _mainEventLog.WriteEvent(micros(), type, message);
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

/*bool OpenFloorMeasurementController::WriteMainEvent(const char* type, const char* message)
{
    return _mainEventLog.WriteEvent(micros(), type, message);
}

void OpenFloorMeasurementController::ServiceMainLog()
{
    if (!_mainLog.service())
    {
        RecordMainLogFailure();
    }
}

void OpenFloorMeasurementController::FlushMainLog()
{
    if (!_mainLog.flush())
    {
        RecordMainLogFailure();
    }
    _mainEventLog.Flush();
}

void OpenFloorMeasurementController::CloseMainLog()
{
    if (!_mainLog.close())
    {
        RecordMainLogFailure();
    }
    _mainEventLog.Close();
}

void OpenFloorMeasurementController::RecordMainLogFailure() noexcept
{
    MazeMap::App::Internal::Runtime::CaptureMmLogFailure(_mainLog, _mainOverflowed, _mainWriteFailed);
}*/

bool OpenFloorMeasurementController::Begin()
{
    _faulted = false;
    _timingLogOpen = false;
    _mainLogOpen = false;
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
    if (!_manifestWriter.WriteManifest(
            _runId,
            IsPrimaryDiagnosticModeRequested(),
            ReadBatteryVoltage(),
            GetMissionFanDutyCycle()))
    {
        return Fail("Run manifest write failed");
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
            ok = Fail("Main log open failed");
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
        FlushTimingLog();
        CloseTimingLog();
        _timingLogOpen = false;
    }
    if (_mainLogOpen)
    {
        FlushMainLog();
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
    _faulted = true;
    SetMissionLevelFanEnabled(false);
    _drive.Brake();
    _drive.UseNominalWheelControlProfile();
    if (message != nullptr && message[0] != '\0')
    {
        AppendStartupTrace(message);
    }
    if (_timingLogOpen)
    {
        FlushTimingLog();
    }
    if (_mainLogOpen)
    {
        FlushMainLog();
    }
    return false;
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
        FlushTimingLog();
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
        FlushMainLog();
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
}

bool OpenFloorMeasurementController::IsWithinBoundary() const
{
    return MazeMap::IsPoseInsideOpenFloorWorkspace(_drive.GetPose());
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
    while ((micros() - _lastControlMicros) < DiagnosticConfig::kControlPeriodUs)
    {
        ServiceTimingLog();
        ServiceMainLog();
        delayMicroseconds(20);
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
        },
        [this]() noexcept
        {
            FlushTimingLog();
            FlushMainLog();
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
    cycle.workspaceViolation = !IsWithinBoundary();
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
    if (LogMainSample(labels, cycle))
    {
        return true;
    }
    return Fail("Failed to write open-floor main sample");
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
        return Fail("Timing log open failed");
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

    const unsigned long pulseDeadline = millis() + DiagnosticConfig::kKickoffSweepPulseMs;
    const float launchBoundM = MazeMap::OpenFloorHalfStepMeters() + Config::kDistanceToleranceM;
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
        if (std::sqrt((dx * dx) + (dy * dy)) > launchBoundM)
        {
            return LogSectionFaultAndFail(
                labels,
                cycle,
                MazeMap::OpenFloorFaultCode::LaunchBoundExceeded,
                "Launch bound exceeded");
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
    IApplicationMode& GetAuxMeasurementMode()
    {
        static AuxMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }

    IApplicationMode& GetFrontWallCharacterizationMode()
    {
        static FrontWallCharacterizationController mode(GetSharedRobotRuntime());
        return mode;
    }

    IApplicationMode& GetWallSensorLedCalibrationMode()
    {
        static WallSensorLedCalibrationController mode;
        return mode;
    }

    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}



