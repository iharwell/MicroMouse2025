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

class AuxMeasurementController : public IApplicationMode
{
public:
    explicit AuxMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _faulted(false)
        , _fanEnabled(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        _faulted = false;
        _fanEnabled = false;
        _phaseFn = nullptr;
        _holdPhaseState = HoldPhaseState{};
        _turningTractionState = TurningTractionState{};
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
        if (!_runtime.Sensors().Begin(AuxMeasurementConfig::kControlPeriodUs))
        {
            return Fail("Auxiliary sensor init failed");
        }
        if (!BeginLog())
        {
            return Fail("Auxiliary measurement log open failed");
        }

        _runtime.Drive().SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        return true;
    }

    void Run() override
    {
        if (_faulted)
        {
            return;
        }

        (void)_runtime.AppendTextLogLine("Entered by shorting pins 28-29 at boot.");
        (void)_runtime.AppendTextLogLine("Internal-sensor auxiliary mode; change AuxMeasurementConfig::kRoutine for other one-offs.");

        const bool ok = RunSelectedRoutine();

        _runtime.Drive().Brake();
        _runtime.Drive().UseNominalWheelControlProfile();
        SetFanEnabled(false);
        if (ok)
        {
            (void)_runtime.AppendTextLogFormatted("Auxiliary measurement complete, log saved to %s", GetLogFileName());
        }
        CloseLog();
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    using PhaseFn = LoopController::ControlVector (AuxMeasurementController::*)(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    enum class RoutineStep : std::uint8_t
    {
        None,
        FanStartupSettle,
        FanOffBaseline,
        FanOnHold,
        FanOffRecovery,
        TurningStartupSettle,
        TurningFanSpinup,
        TurningSweep
    };

    struct HoldPhaseState final
    {
        RoutineStep nextStep{ RoutineStep::None };
        bool stationary{};
        bool fanEnabled{};
        std::uint16_t durationMs{};
        unsigned long startMs{};
        bool started{};
    };

    struct TurningTractionState final
    {
        float directionSign{};
        float commandedSpeedMps{};
        float heldSpeedMps{};
        float commandedCurvatureMInv{};
        float targetYawRad{};
        unsigned long phaseStartMs{};
        unsigned long saturationPlateauStartMs{};
        unsigned long slipCandidateStartMs{};
        bool slipCandidateActive{};
        bool tighteningTurn{};
        MazeMap::TurningTractionMetrics lastMetrics{};
        float lastPlanarAccelMps2{};
        float lastCommandedOmegaRadps{};
        float saturationReferenceSpeedMps{};
        bool phaseSucceeded{};
        bool started{};
    };

    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<AuxMeasurementController*>(context);
        if ((self == nullptr) || (self->_phaseFn == nullptr))
        {
            services.Fault("Aux measurement phase callback was not installed");
            return LoopController::ControlVector::Brake;
        }

        const SensorSnapshot& sensorSnapshot = state.sensors;
        self->_logRow = {};
        self->_logRow.sample = static_cast<std::uint32_t>(self->_sampleCount);
        self->_logRow.phase_id = static_cast<std::uint32_t>(self->_phaseId);
        self->_logRow.t_us = state.tickStartUs;
        self->_logRow.dt_us = state.dtUs;
        self->_logRow.stationary =
            (self->_phaseFn == &AuxMeasurementController::HoldPhaseTick && self->_holdPhaseState.stationary) ? 1U : 0U;
        self->_logRow.fan_enabled = self->_fanEnabled ? 1U : 0U;
        self->_logRow.pose_x_m = state.estimate.xMeters;
        self->_logRow.pose_y_m = state.estimate.yMeters;
        self->_logRow.yaw_rad = state.estimate.yawRad;
        self->_logRow.linear_speed_mps = state.estimate.linearSpeedMps;
        self->_logRow.angular_speed_radps = state.estimate.angularSpeedRadps;
        self->_logRow.planar_accel_mps2 = sensorSnapshot.planarAccelMps2;
        self->_logRow.cmd_linear_mps = self->_runtime.Drive().GetLastLinearCommandMps();
        self->_logRow.cmd_angular_radps = self->_runtime.Drive().GetLastAngularCommandRadps();
        self->_logRow.left_drive_cmd = state.driveTelemetry.leftDriveCommand;
        self->_logRow.right_drive_cmd = state.driveTelemetry.rightDriveCommand;
        self->_logRow.left_encoder_count = state.driveTelemetry.leftEncoderCount;
        self->_logRow.right_encoder_count = state.driveTelemetry.rightEncoderCount;
        self->_logRow.left_distance_m = state.driveTelemetry.leftDistanceM;
        self->_logRow.right_distance_m = state.driveTelemetry.rightDistanceM;
        self->_logRow.left_velocity_mps = state.driveTelemetry.leftVelocityMps;
        self->_logRow.right_velocity_mps = state.driveTelemetry.rightVelocityMps;
        self->_logRow.imu_fr_status = sensorSnapshot.imuFrontRight.status;
        self->_logRow.imu_fr_gyro_x = sensorSnapshot.imuFrontRight.gyroX;
        self->_logRow.imu_fr_gyro_y = sensorSnapshot.imuFrontRight.gyroY;
        self->_logRow.imu_fr_gyro_z = sensorSnapshot.imuFrontRight.gyroZ;
        self->_logRow.imu_fr_accel_x = sensorSnapshot.imuFrontRight.accelX;
        self->_logRow.imu_fr_accel_y = sensorSnapshot.imuFrontRight.accelY;
        self->_logRow.imu_fr_accel_z = sensorSnapshot.imuFrontRight.accelZ;
        self->_logRow.imu_fr_temp = sensorSnapshot.imuFrontRight.temp;
        self->_logRow.imu_fr_int = sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U;
        self->_logRow.imu_bl_status = sensorSnapshot.imuBackLeft.status;
        self->_logRow.imu_bl_gyro_x = sensorSnapshot.imuBackLeft.gyroX;
        self->_logRow.imu_bl_gyro_y = sensorSnapshot.imuBackLeft.gyroY;
        self->_logRow.imu_bl_gyro_z = sensorSnapshot.imuBackLeft.gyroZ;
        self->_logRow.imu_bl_accel_x = sensorSnapshot.imuBackLeft.accelX;
        self->_logRow.imu_bl_accel_y = sensorSnapshot.imuBackLeft.accelY;
        self->_logRow.imu_bl_accel_z = sensorSnapshot.imuBackLeft.accelZ;
        self->_logRow.imu_bl_temp = sensorSnapshot.imuBackLeft.temp;
        self->_logRow.imu_bl_int = sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        self->_logRow.ws_fl_ambient = sensorSnapshot.frontLeft.ambientLight;
        self->_logRow.ws_fl_lit = sensorSnapshot.frontLeft.litLight;
        self->_logRow.ws_fl_delta = sensorSnapshot.frontLeft.differentialLight;
        self->_logRow.ws_fl_raw_distance_m = sensorSnapshot.frontLeft.rawDistanceM;
        self->_logRow.ws_fl_distance_m = sensorSnapshot.frontLeft.distanceM;
        self->_logRow.ws_fr_ambient = sensorSnapshot.frontRight.ambientLight;
        self->_logRow.ws_fr_lit = sensorSnapshot.frontRight.litLight;
        self->_logRow.ws_fr_delta = sensorSnapshot.frontRight.differentialLight;
        self->_logRow.ws_fr_raw_distance_m = sensorSnapshot.frontRight.rawDistanceM;
        self->_logRow.ws_fr_distance_m = sensorSnapshot.frontRight.distanceM;
        self->_logRow.ws_sl_ambient = sensorSnapshot.sideLeft.ambientLight;
        self->_logRow.ws_sl_lit = sensorSnapshot.sideLeft.litLight;
        self->_logRow.ws_sl_delta = sensorSnapshot.sideLeft.differentialLight;
        self->_logRow.ws_sl_raw_distance_m = sensorSnapshot.sideLeft.rawDistanceM;
        self->_logRow.ws_sl_distance_m = sensorSnapshot.sideLeft.distanceM;
        self->_logRow.ws_sr_ambient = sensorSnapshot.sideRight.ambientLight;
        self->_logRow.ws_sr_lit = sensorSnapshot.sideRight.litLight;
        self->_logRow.ws_sr_delta = sensorSnapshot.sideRight.differentialLight;
        self->_logRow.ws_sr_raw_distance_m = sensorSnapshot.sideRight.rawDistanceM;
        self->_logRow.ws_sr_distance_m = sensorSnapshot.sideRight.distanceM;
        self->_logRow.front_wall = sensorSnapshot.frontWall ? 1U : 0U;
        self->_logRow.left_wall = sensorSnapshot.leftWall ? 1U : 0U;
        self->_logRow.right_wall = sensorSnapshot.rightWall ? 1U : 0U;
        self->_logRow.corridor_error_m = sensorSnapshot.corridorErrorM;
        self->_logRow.front_skew_m = sensorSnapshot.frontSkewM;
        self->_logRow.gyro_bias_radps = sensorSnapshot.gyroBiasRadps;
        self->_logRow.gyro_raw_radps = sensorSnapshot.gyroRawRadps;
        self->_logRow.gyro_radps = sensorSnapshot.gyroRadps;
        if (!self->_runtime.LogUtilityDataRow(self->_logRow))
        {
            services.Fault("Failed to write auxiliary measurement sample");
            return LoopController::ControlVector::Brake;
        }
        ++self->_sampleCount;

        return (self->*self->_phaseFn)(loopEndTimeUs, state, services);
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
    LoopController& _loopController;
    char _logFileName[64];
    bool _faulted;
    bool _fanEnabled;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    AuxMeasurementLogRow _logRow{};
    PhaseFn _phaseFn{};
    HoldPhaseState _holdPhaseState{};
    TurningTractionState _turningTractionState{};

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = AuxMeasurementConfig::kControlPeriodUs;
        return options;
    }

    bool RunRoutineSession(const RoutineStep initialStep)
    {
        if (!StartRoutineStep(initialStep))
        {
            return false;
        }

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &AuxMeasurementController::ModeWorkThunk;
        callbacks.context = this;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            _phaseFn = nullptr;
            return Fail("Aux measurement loop session start failed");
        }

        const LoopController::SessionResult result = _loopController.Run();
        _phaseFn = nullptr;
        return (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
    }

    bool StartHoldPhase(
        const char* phaseName,
        uint16_t durationMs,
        bool stationary,
        bool fanEnabled,
        RoutineStep nextStep)
    {
        if (!BeginPhase(phaseName))
        {
            return Fail("Failed to begin auxiliary measurement phase");
        }

        SetFanEnabled(fanEnabled);
        _holdPhaseState = HoldPhaseState{};
        _holdPhaseState.nextStep = nextStep;
        _holdPhaseState.stationary = stationary;
        _holdPhaseState.fanEnabled = fanEnabled;
        _holdPhaseState.durationMs = durationMs;
        _phaseFn = &AuxMeasurementController::HoldPhaseTick;
        return true;
    }

    bool StartTurningTractionSweepPhase()
    {
        if (!BeginPhase("turning_traction_sweep"))
        {
            return Fail("Failed to begin turning traction sweep phase");
        }

        SetFanEnabled(true);
        _runtime.Drive().SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        _turningTractionState = TurningTractionState{};
        _turningTractionState.directionSign = AuxMeasurementConfig::kTurningTractionSweepClockwise ? 1.0f : -1.0f;
        _turningTractionState.commandedSpeedMps = AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps;
        _turningTractionState.heldSpeedMps = _turningTractionState.commandedSpeedMps;
        _turningTractionState.commandedCurvatureMInv =
            (AuxMeasurementConfig::kTurningTractionSweepRadiusM > 1.0e-6f) ?
            (1.0f / AuxMeasurementConfig::kTurningTractionSweepRadiusM) :
            0.0f;
        _turningTractionState.targetYawRad = _runtime.Drive().GetPose().yawRad;
        _turningTractionState.phaseStartMs = millis();
        _phaseFn = &AuxMeasurementController::TurningTractionTick;
        return true;
    }

    bool StartRoutineStep(const RoutineStep step)
    {
        switch (step)
        {
        case RoutineStep::FanStartupSettle:
            return StartHoldPhase(
                "startup_settle",
                AuxMeasurementConfig::kStartupSettleMs,
                true,
                false,
                RoutineStep::FanOffBaseline);

        case RoutineStep::FanOffBaseline:
            return StartHoldPhase(
                "fan_off_baseline",
                AuxMeasurementConfig::kBaselineHoldMs,
                true,
                false,
                RoutineStep::FanOnHold);

        case RoutineStep::FanOnHold:
            return StartHoldPhase(
                "fan_on_hold",
                AuxMeasurementConfig::kFanHoldMs,
                true,
                true,
                RoutineStep::FanOffRecovery);

        case RoutineStep::FanOffRecovery:
            return StartHoldPhase(
                "fan_off_recovery",
                AuxMeasurementConfig::kRecoveryHoldMs,
                true,
                false,
                RoutineStep::None);

        case RoutineStep::TurningStartupSettle:
            return StartHoldPhase(
                "startup_settle",
                AuxMeasurementConfig::kStartupSettleMs,
                true,
                false,
                RoutineStep::TurningFanSpinup);

        case RoutineStep::TurningFanSpinup:
            return StartHoldPhase(
                "fan_spinup",
                AuxMeasurementConfig::kTurningTractionSweepFanSettleMs,
                true,
                true,
                RoutineStep::TurningSweep);

        case RoutineStep::TurningSweep:
            return StartTurningTractionSweepPhase();

        case RoutineStep::None:
        default:
            return false;
        }
    }

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
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _runtime.Sensors().GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _runtime.Sensors().GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _runtime.Sensors().GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_runtime.Sensors())) return false;
        if (!WriteEvent("summary", "Enter by shorting pins 28-29 at boot; those pins select the mode only.")) return false;
        if (!WriteEvent("summary", "Change AuxMeasurementConfig::kRoutine and RunSelectedRoutine() to repurpose this mode.")) return false;
        if (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteEvent("summary", "Default routine enables the fan, ramps circle speed, then tightens curvature after speed plateaus until encoder-vs-gyro/IMU mismatch marks traction loss.")) return false;
            if (!WriteEvent("summary", "Use traction_limit_result and the last steady samples to estimate max sustainable circle speed, yaw rate, and lateral acceleration.")) return false;
        }
        else
        {
            if (!WriteEvent("summary", "Default routine logs stationary fan-off, fan-on, and recovery phases to measure fan-induced sensor/vibration shifts.")) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;

        _logRow = {};
        return _runtime.BeginUtilityDataLogSchema(_logRow);
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
            return RunRoutineSession(RoutineStep::FanStartupSettle);
        case AuxMeasurementConfig::Routine::TurningTractionSweep:
            return RunRoutineSession(RoutineStep::TurningStartupSettle);
        default:
            return Fail("Unknown auxiliary measurement routine");
        }
    }

    LoopController::ControlVector HoldPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        if (!_holdPhaseState.started)
        {
            _holdPhaseState.started = true;
            _holdPhaseState.startMs = millis();
        }

        if (static_cast<unsigned long>(millis() - _holdPhaseState.startMs) >= _holdPhaseState.durationMs)
        {
            if (_holdPhaseState.nextStep == RoutineStep::None)
            {
                services.RequestEndLoop();
            }
            else if (!StartRoutineStep(_holdPhaseState.nextStep))
            {
                services.Fault("Failed to advance auxiliary measurement routine");
            }
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector TurningTractionTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const unsigned long nowMs = millis();
        if (!_turningTractionState.started)
        {
            _turningTractionState.started = true;
            _turningTractionState.phaseStartMs = nowMs;
        }

        if (static_cast<unsigned long>(nowMs - _turningTractionState.phaseStartMs) >= AuxMeasurementConfig::kTurningTractionSweepTimeoutMs)
        {
            _turningTractionState.phaseSucceeded = WriteTurningTractionResult(
                "timeout",
                false,
                static_cast<unsigned long>(nowMs - _turningTractionState.phaseStartMs),
                _turningTractionState.commandedSpeedMps,
                _turningTractionState.lastCommandedOmegaRadps,
                _turningTractionState.lastMetrics,
                _turningTractionState.lastPlanarAccelMps2);
            if (!_turningTractionState.phaseSucceeded)
            {
                services.Fault("Failed to write turning traction timeout result");
            }
            services.RequestEndLoop();
            return LoopController::ControlVector::Brake;
        }

        const SensorSnapshot& sensorSnapshot = state.sensors;
        if (!_turningTractionState.tighteningTurn)
        {
            _turningTractionState.commandedSpeedMps += AuxMeasurementConfig::kTurningTractionSweepAccelMps2 * state.dtSeconds;
            if constexpr (AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps > 0.0f)
            {
                _turningTractionState.commandedSpeedMps = (std::min)(
                    AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps,
                    _turningTractionState.commandedSpeedMps);
            }
            _turningTractionState.heldSpeedMps = _turningTractionState.commandedSpeedMps;
        }
        else
        {
            _turningTractionState.commandedSpeedMps = _turningTractionState.heldSpeedMps;
            _turningTractionState.commandedCurvatureMInv +=
                AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec * state.dtSeconds;
        }

        const float nominalOmegaRadps =
            _turningTractionState.directionSign *
            (_turningTractionState.commandedSpeedMps * _turningTractionState.commandedCurvatureMInv);
        _turningTractionState.targetYawRad =
            WrapAngleRad(_turningTractionState.targetYawRad + (nominalOmegaRadps * state.dtSeconds));
        _turningTractionState.lastCommandedOmegaRadps = MazeMap::ComputeTurningTractionAngularCommand(
            nominalOmegaRadps,
            _turningTractionState.targetYawRad,
            state.estimate.yawRad,
            state.estimate.angularSpeedRadps,
            Config::kArcHeadingKp,
            Config::kArcYawD,
            AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps);
        const float effectiveTrackWidthM =
            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(
                _turningTractionState.commandedSpeedMps,
                _turningTractionState.lastCommandedOmegaRadps);
        const float planarAccelMps2 = sensorSnapshot.planarAccelMps2;
        const MazeMap::TurningTractionMetrics metrics = MazeMap::ComputeTurningTractionMetrics(
            state.driveTelemetry.leftVelocityMps,
            state.driveTelemetry.rightVelocityMps,
            effectiveTrackWidthM,
            sensorSnapshot.gyroRadps,
            planarAccelMps2);
        _turningTractionState.lastMetrics = metrics;
        _turningTractionState.lastPlanarAccelMps2 = planarAccelMps2;

        const bool slipDetected = MazeMap::IsTurningTractionLossDetected(
            metrics,
            AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps,
            AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2,
            AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor,
            AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor);
        if (slipDetected)
        {
            if (!_turningTractionState.slipCandidateActive)
            {
                _turningTractionState.slipCandidateStartMs = nowMs;
                _turningTractionState.slipCandidateActive = true;
            }
            else if (static_cast<unsigned long>(nowMs - _turningTractionState.slipCandidateStartMs) >= AuxMeasurementConfig::kTurningTractionSlipConfirmMs)
            {
                _turningTractionState.phaseSucceeded = WriteTurningTractionResult(
                    "traction_loss",
                    true,
                    static_cast<unsigned long>(nowMs - _turningTractionState.phaseStartMs),
                    _turningTractionState.commandedSpeedMps,
                    _turningTractionState.lastCommandedOmegaRadps,
                    metrics,
                    planarAccelMps2);
                if (!_turningTractionState.phaseSucceeded)
                {
                    services.Fault("Failed to write turning traction loss result");
                }
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;
            }
        }
        else
        {
            _turningTractionState.slipCandidateActive = false;
        }

        const float maxWheelCommandMagnitude = (std::max)(
            std::fabs(state.driveTelemetry.leftDriveCommand),
            std::fabs(state.driveTelemetry.rightDriveCommand));
        if (!_turningTractionState.tighteningTurn)
        {
            const bool actuatorLimited =
                (metrics.encoderLinearSpeedMps >= AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps) &&
                (maxWheelCommandMagnitude >= AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand);

            if (!actuatorLimited)
            {
                _turningTractionState.saturationPlateauStartMs = 0UL;
                _turningTractionState.saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
            }
            else if (_turningTractionState.saturationPlateauStartMs == 0UL)
            {
                _turningTractionState.saturationPlateauStartMs = nowMs;
                _turningTractionState.saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
            }
            else if (metrics.encoderLinearSpeedMps >=
                (_turningTractionState.saturationReferenceSpeedMps + AuxMeasurementConfig::kTurningTractionPlateauDeltaMps))
            {
                _turningTractionState.saturationPlateauStartMs = nowMs;
                _turningTractionState.saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
            }
            else if (static_cast<unsigned long>(nowMs - _turningTractionState.saturationPlateauStartMs) >=
                AuxMeasurementConfig::kTurningTractionPlateauWindowMs)
            {
                _turningTractionState.tighteningTurn = true;
                _turningTractionState.heldSpeedMps = (std::max)(
                    _turningTractionState.commandedSpeedMps,
                    metrics.encoderLinearSpeedMps);

                char message[160] = {};
                const int messageLength = snprintf(
                    message,
                    sizeof(message),
                    "reason=speed_plateau;hold_v_mps=%.3f;curvature_m_inv=%.3f;outer_cmd=%.3f",
                    _turningTractionState.heldSpeedMps,
                    _turningTractionState.commandedCurvatureMInv,
                    maxWheelCommandMagnitude);
                if (messageLength <= 0 || messageLength >= static_cast<int>(sizeof(message)))
                {
                    services.Fault("Failed to format turning traction mode event");
                    return LoopController::ControlVector::Brake;
                }
                if (!WriteEvent("turning_traction_mode", message))
                {
                    services.Fault("Failed to write turning traction mode event");
                    return LoopController::ControlVector::Brake;
                }
            }
        }

        if (static_cast<unsigned long>(nowMs - _turningTractionState.phaseStartMs) < AuxMeasurementConfig::kTurningTractionLaunchMs)
        {
            const MazeMap::TurningLaunchCommands launchCommands = MazeMap::ComputeTurningLaunchCommands(
                _turningTractionState.commandedSpeedMps,
                _turningTractionState.lastCommandedOmegaRadps,
                effectiveTrackWidthM,
                Config::kWheelRestLaunchDriveCommand);
            return LoopController::ControlVector::RawMotorPwm(
                launchCommands.leftCommand,
                launchCommands.rightCommand);
        }

        return _runtime.Drive().PointControlVector(
            _turningTractionState.commandedSpeedMps,
            _turningTractionState.lastCommandedOmegaRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
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
    IApplicationMode& GetAuxMeasurementMode();

    const BootModeDescriptor& GetAuxMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::AuxiliaryMeasurement,
            BootModeCategory::Utility,
            "auxiliary_measurement",
            "Run the selected one-off auxiliary measurement routine.",
            "logging.txt; auxiliary measurement mmlog",
            &GetAuxMeasurementMode,
            "GetAuxMeasurementMode",
            "AuxMeasurementController.cpp",
            "startup settle; selected auxiliary routine; final log close",
            "AuxMeasurementConfig; shared mission drive and sensor tuning",
            "AuxMeasurementConfig::kRoutine selects the scenario",
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

