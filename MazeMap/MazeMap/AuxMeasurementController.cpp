#include "pch.h"
#include "MazeMapApplicationPrivate.h"

#include "BootModeDescriptor.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

using MazeMap::App::Internal::Drive;
using MazeMap::App::Internal::GetSharedRobotRuntime;
using CommandVector = MazeMap::App::Internal::CommandVector;
using LoopController = MazeMap::App::Internal::LoopController;
using MazeMap::App::Internal::SharedRobotRuntime;
using MazeMap::App::Internal::StartupCalibration;

namespace
{
    constexpr const char* kAuxMeasurementStableId = "auxiliary_measurement";

    MotionLimits BuildAuxiliaryDriveLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = vehicle.GetMaxSpeed();
        limits.accelMps2 = vehicle.GetMaxForwardAcceleration();
        limits.decelMps2 = vehicle.GetMaxForwardAcceleration();
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }

    void PopulateAuxMeasurementLogRow(
        AuxMeasurementLogRow& row,
        const std::uint32_t sample,
        const std::uint32_t phaseId,
        const bool stationary,
        const bool fanEnabled,
        const MazeMap::VehicleState& state,
        const DriveBase& drive)
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = drive.GetTelemetry();
        row = {};
        row.sample = sample;
        row.phase_id = phaseId;
        row.stationary = stationary ? 1U : 0U;
        row.fan_enabled = fanEnabled ? 1U : 0U;
        row.pose_x_m = state.GetPositionX();
        row.pose_y_m = state.GetPositionY();
        row.yaw_rad = state.GetOrientation();
        row.linear_speed_mps = state.GetVelocity();
        row.angular_speed_radps = state.GetRotationalVelocity();
        row.planar_accel_mps2 = sensors.planarAccelMps2;
        row.cmd_linear_mps = drive.GetLastLinearCommandMps();
        row.cmd_angular_radps = drive.GetLastAngularCommandRadps();
        row.left_drive_cmd = driveTelemetry.leftDriveCommand;
        row.right_drive_cmd = driveTelemetry.rightDriveCommand;
        row.left_encoder_count = driveTelemetry.leftEncoderCount;
        row.right_encoder_count = driveTelemetry.rightEncoderCount;
        row.left_distance_m = driveTelemetry.leftDistanceM;
        row.right_distance_m = driveTelemetry.rightDistanceM;
        row.left_velocity_mps = driveTelemetry.leftVelocityMps;
        row.right_velocity_mps = driveTelemetry.rightVelocityMps;
        row.front_wall = sensors.frontWall ? 1U : 0U;
        row.left_wall = sensors.leftWall ? 1U : 0U;
        row.right_wall = sensors.rightWall ? 1U : 0U;
        row.corridor_error_m = sensors.corridorErrorM;
        row.front_skew_m = sensors.frontSkewM;
        row.gyro_bias_radps = sensors.gyroBiasRadps;
        row.gyro_raw_radps = sensors.gyroRawRadps;
        row.gyro_radps = sensors.gyroRadps;
    }
}

class AuxMeasurementController final : public IApplicationMode
{
private:
    enum class Phase : std::uint8_t
    {
        Idle,
        LaunchStartupCalibration,
        RunStartupCalibration,
        LaunchBaselineHold,
        RunBaselineHold,
        LaunchFanOnHold,
        RunFanOnHold,
        LaunchRecoveryHold,
        RunRecoveryHold,
        LaunchStartupHold,
        RunStartupHold,
        LaunchFanSpinupHold,
        RunFanSpinupHold,
        TurningTractionSweep,
        Complete
    };

public:
    explicit AuxMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
        _logFileName[0] = '\0';
    }

    void SetupMode() override
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(&AuxMeasurementController::TeardownOnRuntimeFault, this, kAuxMeasurementStableId))
        {
            _runtime.FailActiveMode("Auxiliary measurement fault handler registration failed");
        }
        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Auxiliary measurement hardware setup failed");
        }

        (void)ResetStartupTrace("mode:aux_measurement");
        (void)_runtime.AppendTextLogLine("Auxiliary measurement mode");
        (void)_runtime.AppendTextLogFormatted(
            "Routine: %s",
            AuxMeasurementRoutineName(AuxMeasurementConfig::kRoutine));

        if (!_drive.Begin())
        {
            _runtime.FailActiveMode("Auxiliary measurement drive base init failed");
        }

        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            _drive.SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
            _startupCalibration.SetIsInMaze(false);
            _drive.SetPose(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM, DirectionToYawRad(MazeMap::Up));
        }
        else
        {
            _drive.UseNominalWheelControlProfile();
            _startupCalibration.SetIsInMaze(true);
        }

        if (!_startupCalibration.BringUp())
        {
            _runtime.FailActiveMode("Auxiliary measurement startup bring-up failed");
        }
        if (!BeginLog())
        {
            _runtime.FailActiveMode("Auxiliary measurement log open failed");
        }

        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            _phase = Phase::LaunchStartupHold;
        }
        else
        {
            _phase = Phase::LaunchStartupCalibration;
        }
        _loopController.StageNextSessionState(BuildLoopOptions());
    }

private:
    static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        auto* const self = static_cast<AuxMeasurementController*>(context);
        if (self == nullptr)
        {
            return;
        }

        (void)reason;
        self->_runtimeFaulted = true;
        self->_phase = Phase::Idle;
        self->_startupCalibration.Cancel();
        self->_drive.Brake();
        self->SetFanEnabled(false);
    }

    LoopController::SessionOptions BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        const auto& runtimeState = _runtime.RuntimeState();
        options.controlPeriodUs = AuxMeasurementConfig::kControlPeriodUs;
        options.SessionStartPointX = runtimeState.GetPositionX();
        options.SessionStartPointY = runtimeState.GetPositionY();
        return options;
    }

    void ResetState() noexcept
    {
        _phase = Phase::Idle;
        _runtimeFaulted = false;
        _fanEnabled = false;
        _phaseId = 0U;
        _sampleCount = 0U;
        _logFileName[0] = '\0';
        _logRow = {};
        _logRowBuffered = false;
        ResetTurningTractionState();
        _startupCalibration.Cancel();
    }

    void ResetTurningTractionState() noexcept
    {
        _turningTractionDirectionSign = 0.0f;
        _turningTractionCommandedSpeedMps = 0.0f;
        _turningTractionPhaseStartMs = 0UL;
        _turningTractionStarted = false;
    }

    bool BeginLog()
    {
        return
            _runtime.OpenUtilityDataLog(_logFileName, sizeof(_logFileName), nullptr, "aux%03u.mmlog", "aux_measurement_log.mmlog") &&
            _runtime.WriteUtilityDataLogMetadata("mode", "auxiliary_measurement") &&
            _runtime.WriteUtilityDataLogMetadata("routine", AuxMeasurementRoutineName(AuxMeasurementConfig::kRoutine)) &&
            _runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", static_cast<unsigned long>(AuxMeasurementConfig::kControlPeriodUs)) &&
            _runtime.WriteUtilityDataLogMetadataUnsigned("startup_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs)) &&
            _runtime.WriteUtilityDataLogAccelBiasMetadata(_runtime.Sensors()) &&
            _runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g") &&
            _runtime.WriteUtilityDataLogMetadata("endianness", "little") &&
            _runtime.BeginUtilityDataLogSchema(_logRow);
    }

    bool WriteEvent(const char* type, const char* message)
    {
        return _runtime.WriteTextLogEntry(micros(), type, message);
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;
        return _runtime.WriteTextLogPhase(_phaseId, micros(), name);
    }

    bool PhaseIsStationary() const noexcept
    {
        switch (_phase)
        {
        case Phase::RunBaselineHold:
        case Phase::RunFanOnHold:
        case Phase::RunRecoveryHold:
        case Phase::RunStartupHold:
        case Phase::RunFanSpinupHold:
            return true;
        default:
            return false;
        }
    }

    bool LogSample(const MazeMap::VehicleState& state)
    {
        if (_logRowBuffered)
        {
            const LoopController::TimingDiagnostics& timing = _loopController.LastDiagnostics();
            _logRow.t_us = timing.tickStartUs;
            _logRow.dt_us = timing.dtUs;
            if (!_runtime.LogUtilityDataRow(_logRow))
            {
                return false;
            }
            _logRowBuffered = false;
        }

        PopulateAuxMeasurementLogRow(
            _logRow,
            static_cast<std::uint32_t>(_sampleCount),
            static_cast<std::uint32_t>(_phaseId),
            PhaseIsStationary(),
            _fanEnabled,
            state,
            _drive);
        ++_sampleCount;
        _logRowBuffered = true;
        return true;
    }

    void SetFanEnabled(const bool enabled)
    {
        if (_fanEnabled == enabled)
        {
            return;
        }

        _fanEnabled = enabled;
        SetMissionLevelFanEnabled(enabled);
    }

    CommandVector FinishMode(LoopController& loopController)
    {
        loopController.RequestEndSession(
            +[](void* const context, LoopController& boundaryLoopController)
            {
                auto* const self = static_cast<AuxMeasurementController*>(context);
                if (self == nullptr)
                {
                    GetSharedRobotRuntime().FailActiveMode(
                        "Auxiliary measurement completion callback context was null");
                    return;
                }

                if (self->_logRowBuffered)
                {
                    const LoopController::TimingDiagnostics& timing = self->_loopController.LastDiagnostics();
                    self->_logRow.t_us = timing.tickStartUs;
                    self->_logRow.dt_us = timing.dtUs;
                    if (!self->_runtime.LogUtilityDataRow(self->_logRow))
                    {
                        self->_runtime.FailActiveMode(
                            "Failed to write final auxiliary measurement sample");
                    }
                    self->_logRowBuffered = false;
                }

                (void)self->_runtime.AppendTextLogFormatted(
                    "Auxiliary measurement complete, log saved to %s",
                    self->_logFileName);
                self->_startupCalibration.Cancel();
                self->_drive.Brake();
                self->_drive.UseNominalWheelControlProfile();
                self->SetFanEnabled(false);
                self->_phase = Phase::Idle;
                boundaryLoopController.HaltExecutionEndProgram();
            },
            this);
        return CommandVector::Brake();
    }

    CommandVector PollDriveHold(const Phase nextPhase)
    {
        bool done = false;
        const CommandVector control = _driveService.GetNextControls(done);
        if (!done)
        {
            return control;
        }

        _phase = nextPhase;
        return CommandVector::Brake();
    }

    CommandVector RunTurningTractionSweep(
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)state;
        const unsigned long nowMs = millis();
        if (!_turningTractionStarted)
        {
            _turningTractionStarted = true;
            _turningTractionPhaseStartMs = nowMs;
        }

        if (static_cast<unsigned long>(nowMs - _turningTractionPhaseStartMs) >= AuxMeasurementConfig::kTurningTractionSweepTimeoutMs)
        {
            (void)WriteEvent("summary", "Turning traction sweep finished; analyze the logged run offline.");
            _phase = Phase::Complete;
            return FinishMode(loopController);
        }

        _turningTractionCommandedSpeedMps +=
            AuxMeasurementConfig::kTurningTractionSweepAccelMps2 *
            (std::max)(0.0f, static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f);
        if constexpr (AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps > 0.0f)
        {
            _turningTractionCommandedSpeedMps =
                (std::min)(AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps, _turningTractionCommandedSpeedMps);
        }

        const float yawRateRadps =
            (AuxMeasurementConfig::kTurningTractionSweepRadiusM > 1.0e-6f) ?
            (_turningTractionDirectionSign * (_turningTractionCommandedSpeedMps / AuxMeasurementConfig::kTurningTractionSweepRadiusM)) :
            0.0f;
        return _drive.PointControlVector(
            _turningTractionCommandedSpeedMps,
            yawRateRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    CommandVector RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController) override
    {
        (void)loopEndTimeUs;
        if (!LogSample(state))
        {
            _runtime.FailActiveMode("Failed to write auxiliary measurement sample");
            return CommandVector::Brake();
        }

        switch (_phase)
        {
        case Phase::LaunchStartupCalibration:
            if (!BeginPhase("startup_calibration"))
            {
                _runtime.FailActiveMode("Failed to log auxiliary startup calibration phase");
                return CommandVector::Brake();
            }
            _startupCalibration.Start();
            if (!_startupCalibration.Active())
            {
                _runtime.FailActiveMode("Auxiliary measurement startup calibration could not start");
            }
            else
            {
                _phase = Phase::RunStartupCalibration;
            }
            return CommandVector::Brake();

        case Phase::RunStartupCalibration:
        {
            bool done = false;
            const CommandVector control = _startupCalibration.GetNextControls(done);
            if (!done)
            {
                return control;
            }
            _phase = Phase::LaunchBaselineHold;
            return CommandVector::Brake();
        }

        case Phase::LaunchBaselineHold:
            if (!BeginPhase("fan_off_baseline")) { _runtime.FailActiveMode("Failed to log auxiliary baseline phase"); return CommandVector::Brake(); }
            SetFanEnabled(false);
            _driveService.SetLimits(BuildAuxiliaryDriveLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(AuxMeasurementConfig::kBaselineHoldMs, true);
            _phase = Phase::RunBaselineHold;
            return CommandVector::Brake();

        case Phase::RunBaselineHold:
            return PollDriveHold(Phase::LaunchFanOnHold);

        case Phase::LaunchFanOnHold:
            if (!BeginPhase("fan_on_hold")) { _runtime.FailActiveMode("Failed to log auxiliary fan-on phase"); return CommandVector::Brake(); }
            SetFanEnabled(true);
            _driveService.SetLimits(BuildAuxiliaryDriveLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(AuxMeasurementConfig::kFanHoldMs, true);
            _phase = Phase::RunFanOnHold;
            return CommandVector::Brake();

        case Phase::RunFanOnHold:
            return PollDriveHold(Phase::LaunchRecoveryHold);

        case Phase::LaunchRecoveryHold:
            if (!BeginPhase("fan_off_recovery")) { _runtime.FailActiveMode("Failed to log auxiliary recovery phase"); return CommandVector::Brake(); }
            SetFanEnabled(false);
            _driveService.SetLimits(BuildAuxiliaryDriveLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(AuxMeasurementConfig::kRecoveryHoldMs, true);
            _phase = Phase::RunRecoveryHold;
            return CommandVector::Brake();

        case Phase::RunRecoveryHold:
            return PollDriveHold(Phase::Complete);

        case Phase::LaunchStartupHold:
            if (!BeginPhase("startup_settle")) { _runtime.FailActiveMode("Failed to log auxiliary startup phase"); return CommandVector::Brake(); }
            SetFanEnabled(false);
            _driveService.SetLimits(BuildAuxiliaryDriveLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(AuxMeasurementConfig::kStartupSettleMs, true);
            _phase = Phase::RunStartupHold;
            return CommandVector::Brake();

        case Phase::RunStartupHold:
            return PollDriveHold(Phase::LaunchFanSpinupHold);

        case Phase::LaunchFanSpinupHold:
            if (!BeginPhase("fan_spinup")) { _runtime.FailActiveMode("Failed to log auxiliary fan-spinup phase"); return CommandVector::Brake(); }
            SetFanEnabled(true);
            _driveService.SetLimits(BuildAuxiliaryDriveLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(AuxMeasurementConfig::kTurningTractionSweepFanSettleMs, true);
            _phase = Phase::RunFanSpinupHold;
            return CommandVector::Brake();

        case Phase::RunFanSpinupHold:
        {
            const CommandVector control = PollDriveHold(Phase::TurningTractionSweep);
            if (_phase == Phase::TurningTractionSweep)
            {
                if (!BeginPhase("turning_traction_sweep"))
                {
                    _runtime.FailActiveMode("Failed to log turning traction sweep phase");
                    return CommandVector::Brake();
                }
                ResetTurningTractionState();
                _turningTractionDirectionSign = AuxMeasurementConfig::kTurningTractionSweepClockwise ? 1.0f : -1.0f;
                _turningTractionCommandedSpeedMps = AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps;
                (void)WriteEvent(
                    "summary",
                    "Turning traction sweep now uses DriveBase point commands only; any traction-loss interpretation is offline from the log.");
            }
            return control;
        }

        case Phase::TurningTractionSweep:
            return RunTurningTractionSweep(state, loopController);

        case Phase::Complete:
            return FinishMode(loopController);

        case Phase::Idle:
        default:
            _runtime.FailActiveMode("Auxiliary measurement phase was not initialized");
            return CommandVector::Brake();
        }
    }

    static MazeMap::WheelControlProfile BuildTurningTractionWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _vehicle;
    DriveBase& _drive;
    Drive& _driveService;
    StartupCalibration& _startupCalibration;
    char _logFileName[64];
    Phase _phase{ Phase::Idle };
    bool _runtimeFaulted{};
    bool _fanEnabled{};
    unsigned long _phaseId{};
    unsigned long _sampleCount{};
    AuxMeasurementLogRow _logRow{};
    bool _logRowBuffered{};

    float _turningTractionDirectionSign{};
    float _turningTractionCommandedSpeedMps{};
    unsigned long _turningTractionPhaseStartMs{};
    bool _turningTractionStarted{};
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
            "Run the selected auxiliary survey or traction sweep on the shared startup and Drive path.",
            "logging.txt; auxiliary measurement mmlog",
            &GetAuxMeasurementMode,
            "GetAuxMeasurementMode",
            "AuxMeasurementController.cpp",
            "shared startup path; Drive hold phases; optional traction sweep",
            "AuxMeasurementConfig plus shared StartupCalibration and Drive services",
            "TurningTractionSweep uses DriveBase point commands only; traction-loss inference is offline",
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
