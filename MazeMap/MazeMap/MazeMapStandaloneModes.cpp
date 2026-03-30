#include "MazeMapApplicationPrivate.h"
#include "MazeMapControllerRegistry.h"
#include "MazeMapSharedRuntime.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMapApp::Internal::GetSharedRobotRuntime;
using MazeMapApp::Internal::SharedRobotRuntime;

class AuxMeasurementController : public IApplicationMode
{
public:
    explicit AuxMeasurementController(SharedRobotRuntime& runtime)
        : _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
        , _logger()
        , _faulted(false)
        , _fanEnabled(false)
        , _lastControlMicros(0UL)
    {
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
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            _drive.SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        }
        else
        {
            _drive.UseNominalWheelControlProfile();
        }
        SetFanEnabled(false);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(AuxMeasurementConfig::kControlPeriodUs))
        {
            return Fail("Auxiliary sensor init failed");
        }
        if (!_logger.Begin(_sensors, AuxMeasurementConfig::kRoutine))
        {
            return Fail("Auxiliary measurement log open failed");
        }

        _drive.SnapTo(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
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
        _logger.Flush();
        if (ok)
        {
            Serial.print("Auxiliary measurement complete, log saved to ");
            Serial.println(_logger.GetFileName());
        }
        _logger.Close();
    }

private:
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    AuxMeasurementLogger _logger;
    bool _faulted;
    bool _fanEnabled;
    unsigned long _lastControlMicros;

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

        if (!_logger.BeginPhase("turning_traction_sweep"))
        {
            return Fail("Failed to begin turning traction sweep phase");
        }

        SetFanEnabled(true);
        _drive.SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        const float directionSign = AuxMeasurementConfig::kTurningTractionSweepClockwise ? -1.0f : 1.0f;
        const float circleRadiusM = AuxMeasurementConfig::kTurningTractionSweepRadiusM;
        float commandedSpeedMps = AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps;
        float heldSpeedMps = commandedSpeedMps;
        float commandedCurvatureMInv = (circleRadiusM > 1.0e-6f) ? (1.0f / circleRadiusM) : 0.0f;
        float targetYawRad = _drive.GetPose().yawRad;
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
                _drive.Brake();
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

            const DiagnosticSensorSnapshot sensorSnapshot = _sensors.Capture(false, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, sensorSnapshot);
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
                _drive.GetPose().yawRad,
                _drive.GetPose().angularSpeedRadps,
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
                _drive.CommandOpenLoopRaw(launchCommands.leftCommand, launchCommands.rightCommand);
            }
            else
            {
                _drive.CommandVelocity(commandedSpeedMps, lastCommandedOmegaRadps, dtSeconds);
            }

            const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(sensorSnapshot);
            const MazeMap::TurningTractionMetrics metrics = MazeMap::ComputeTurningTractionMetrics(
                driveTelemetry.leftVelocityMps,
                driveTelemetry.rightVelocityMps,
                effectiveTrackWidthM,
                sensorSnapshot.gyroRadps,
                planarAccelMps2);
            lastMetrics = metrics;
            lastPlanarAccelMps2 = planarAccelMps2;

            if (!_logger.LogSample(
                false,
                _fanEnabled,
                timestampUs,
                dtUs,
                _drive.GetPose(),
                _drive,
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
                    _drive.Brake();
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
                    if (!_logger.WriteEvent("turning_traction_mode", message))
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
        if (!_logger.BeginPhase(phaseName))
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

            _drive.Brake();
            const DiagnosticSensorSnapshot sensorSnapshot = _sensors.Capture(stationary, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, sensorSnapshot);
            const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(sensorSnapshot);
            if (!_logger.LogSample(
                stationary,
                _fanEnabled,
                timestampUs,
                dtUs,
                _drive.GetPose(),
                _drive,
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
        if (_logger.WriteEvent("traction_limit_result", message))
        {
            return true;
        }
        return Fail("Failed to write turning traction result");
    }

    bool Fail(const char* reason)
    {
        _faulted = true;
        _drive.Brake();
        SetFanEnabled(false);
        Serial.print("Auxiliary measurement fault: ");
        Serial.println((reason != nullptr) ? reason : "unknown");
        if (reason != nullptr && reason[0] != '\0')
        {
            _logger.WriteEvent("fault", reason);
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

        const MazeMap::Vectorf<2> targetHeading = _drive.GetPose().headingUnit;
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
        if (!MazeMapApp::Internal::Runtime::SelectSequentialCsvFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "fwc%03u.csv",
                "front_wall_characterization.csv"))
        {
            return Fail("Front wall characterization csv name unavailable");
        }

        MazeMap::CoreFileExport file;
        if (!file.Open(fileName))
        {
            return Fail("Front wall characterization csv open failed");
        }

        auto writeLine = [&](const char* line) -> bool
        {
            return
                (line != nullptr) &&
                file.Write(line) &&
                file.WriteChar('\n');
        };

        char line[224] = {};
        snprintf(line, sizeof(line), "file,%s", fileName);
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        if (!writeLine("mode,front_wall_characterization")) return Fail("Front wall characterization csv write failed");
        snprintf(line, sizeof(line), "samples,%u", static_cast<unsigned>(storage.sampleCount));
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        snprintf(line, sizeof(line), "distance_step_m,%.6f", storage.distanceStepM);
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        snprintf(line, sizeof(line), "reverse_speed_mps,%.6f", storage.commandedReverseSpeedMps);
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        snprintf(line, sizeof(line), "zero_threshold_differential_light,%.6f", storage.zeroThresholdDifferentialLight);
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        snprintf(line, sizeof(line), "terminal_distance_m,%.6f", storage.terminalDistanceM);
        if (!writeLine(line)) return Fail("Front wall characterization csv write failed");
        if (!writeLine("index,distance_m,fl_ambient,fl_lit,fl_delta,fr_ambient,fr_lit,fr_delta"))
        {
            return Fail("Front wall characterization csv write failed");
        }

        for (uint16_t index = 0U; index < storage.sampleCount; ++index)
        {
            snprintf(
                line,
                sizeof(line),
                "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                static_cast<unsigned>(index),
                storage.distanceM[index],
                storage.frontLeftAmbientLight[index],
                storage.frontLeftLitLight[index],
                storage.frontLeftDifferentialLight[index],
                storage.frontRightAmbientLight[index],
                storage.frontRightLitLight[index],
                storage.frontRightDifferentialLight[index]);
            if (!writeLine(line))
            {
                return Fail("Front wall characterization csv write failed");
            }
        }

        file.Flush();
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:csv_exported,file=%s,samples=%u",
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
        , _logger()
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _lastControlMicros(0UL)
    {
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
        if (!_logger.Begin(_sensors))
        {
            return Fail("Diagnostic log open failed");
        }

        _drive.SnapTo(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
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
        ok = ok && ExecuteTurnPhase("turn_cw_90_1", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_1", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_90_2", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_2", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_180", -PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_180", PI_F);
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
        ok = ok && ExecuteSquareLoop("square_cw", -HALF_PI_F);
        ok = ok && HoldPhase("square_cw_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteSquareLoop("square_ccw", HALF_PI_F);
        ok = ok && HoldPhase("final_idle", DiagnosticConfig::kBaselineHoldMs / 2U, true);

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        _logger.Flush();

        if (ok)
        {
            Serial.print("Diagnostic complete, log saved to ");
            Serial.println(_logger.GetFileName());
            Serial.println("Use the # event,summary lines in the log header to map phases to tunables.");
        }

        _logger.Close();
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
    DiagnosticLogger _logger;
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _lastControlMicros;

    bool WriteStraightResult(
        const char* phaseName,
        float distanceM,
        float cruiseSpeedMps,
        float traveledM,
        const MazeMap::Vectorf<2>& targetHeading,
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
        if (_logger.WriteEvent(type, message))
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
        _logger.WriteEvent("fault", message);
        _logger.Flush();
        return false;
    }

    bool StartPhase(const char* name)
    {
        Serial.print("Diagnostic phase: ");
        Serial.println(name);
        if (_logger.BeginPhase(name))
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
            delayMicroseconds(20);
        }

        timestampUs = micros();
        dtSeconds = static_cast<float>(timestampUs - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = timestampUs;

        snapshot = _sensors.Capture(stationary, _drive.GetPose());
        _drive.UpdateOdometry(dtSeconds, snapshot);

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
        if (_logger.LogSample(stationary, timestampUs, dtUs, _drive.GetPose(), _drive, telemetry, snapshot))
        {
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
        const MazeMap::Vectorf<2> targetHeading = _drive.GetPose().headingUnit;
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
        if (!ExecuteArcCircle(phaseName, -PI_F, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_ccw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, PI_F, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
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
namespace MazeMapApp::Internal
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
        static DiagnosticController mode(GetSharedRobotRuntime());
        return mode;
    }
}
