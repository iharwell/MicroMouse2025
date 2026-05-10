#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "DriveTelemetry.h"
#include "EncoderProgressWatchdog.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "SharedRobotRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "RuntimeBinaryLogSupport.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"
#include "WallDistanceCalibration.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using CommandVector = MazeMap::App::Internal::CommandVector;
using MazeMap::App::Internal::SharedRobotRuntime;

namespace
{
    // This legacy controller is no longer registered as the primary diagnostic mode; keep
    // its old sweep constants private so DiagnosticConfig only exposes active open-floor dials.
    namespace LegacyDiagnosticConfig
    {
        constexpr float kShortStraightDistanceM = 0.18f;
        constexpr float kLongStraightDistanceM = 0.27f;
        constexpr float kSquareLegDistanceM = 0.18f;
        constexpr float kArcHalfCircleDistanceM = 0.20f;
        constexpr float kSlowStraightSpeedMps = 0.4f;
        constexpr float kCircleMediumSpeedMps = 0.6f;
        constexpr float kFastStraightSpeedMps = 0.8f;
        constexpr float kKickoffSweepMoveThresholdM = 0.03f;
        constexpr float kKickoffSweepMoveThresholdMps = 0.03f;
        constexpr uint16_t kKickoffSweepPulseMs = 250U;
        constexpr float kForwardSweepKickoffDriveCommand = 0.35f;
        constexpr uint16_t kForwardSweepKickoffMs = 120U;
        constexpr float kForwardSweepMinDriveCommand = 0.10f;
        constexpr float kForwardSweepMaxDriveCommand = 0.40f;
        constexpr float kForwardSweepStepDriveCommand = 0.01f;
        constexpr uint16_t kForwardSweepHoldMs = 220U;
        constexpr float kForwardSweepCarryThresholdMps = 0.05f;
        constexpr float kForwardSweepCarryThresholdM = 0.180f;
    }

}

class DiagnosticController : public IApplicationMode
{
    using LoopController = MazeMap::App::Internal::LoopController;

public:
    explicit DiagnosticController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _pendingReturnDistanceM(0.0f)
    {
        _logFileName[0] = '\0';
    }

    void SetupMode() override
    {
        ResetModeState();

        if (!_runtime.RegisterModeFaultHandler(&DiagnosticController::HandleRuntimeFault, this, "diagnostic"))
        {
            _runtime.FailActiveMode("Diagnostic fault handler registration failed");
        }

        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Hardware setup failed");
        }
        ResetStartupTrace("mode:primary_diagnostic");
        (void)_runtime.AppendTextLogLine("Micromouse diagnostic setup");
        if (!_drive.Begin())
        {
            _runtime.FailActiveMode("Drive base init failed");
        }
        _drive.SetWheelControlProfile(BuildDiagnosticWheelControlProfile());
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            _runtime.FailActiveMode("Diagnostic sensor init failed");
        }
        if (!BeginLog())
        {
            _runtime.FailActiveMode("Diagnostic log open failed");
        }

        _drive.SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _startX = _runtime.RuntimeState().GetPositionX();
        _startY = _runtime.RuntimeState().GetPositionY();

        if (!StartScenarioStep(ScenarioStep::StartupSettle))
        {
            _runtime.FailActiveMode("Diagnostic scenario initialization failed");
        }

        _loopController.StageNextSessionState(BuildLoopOptions());
    }

    CommandVector RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController) override
    {
        if (!WriteBufferedSample())
        {
            _runtime.FailActiveMode("Failed to write diagnostic sample");
            return CommandVector::Brake();
        }

        if (_phaseFn == nullptr)
        {
            _runtime.FailActiveMode("Diagnostic phase callback was not installed");
            return CommandVector::Brake();
        }

        if (_runtime.Estimator().HasFault())
        {
            _runtime.FailActiveMode(_runtime.Estimator().FaultReason());
            return CommandVector::Brake();
        }

        if (!IsWithinBoundary(state))
        {
            _runtime.FailActiveMode("Diagnostic boundary exceeded");
            return CommandVector::Brake();
        }

        BufferCurrentSample(state);
        ++_sampleCount;

        return (this->*_phaseFn)(loopEndTimeUs, state, loopController);
    }

private:
    bool WriteBufferedSample()
    {
        if (!_logRowBuffered)
        {
            return true;
        }

        const LoopController::TimingDiagnostics& timing = _loopController.LastDiagnostics();
        _logRow.t_us = timing.tickStartUs;
        _logRow.dt_us = timing.dtUs;
        if (!_runtime.LogUtilityDataRow(_logRow))
        {
            return false;
        }

        _logRowBuffered = false;
        return true;
    }

    void BufferCurrentSample(const MazeMap::VehicleState& state)
    {
        const SensorSnapshot& snapshot = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
        _logRow = {};
        _logRow.sample = static_cast<std::uint32_t>(_sampleCount);
        _logRow.phase_id = static_cast<std::uint32_t>(_phaseId);
        _logRow.stationary =
            (_phaseFn == &DiagnosticController::HoldPhaseTick && _holdStationary) ? 1U : 0U;
        _logRow.pose_x_m = state.GetPositionX();
        _logRow.pose_y_m = state.GetPositionY();
        _logRow.yaw_rad = state.GetOrientation();
        _logRow.linear_speed_mps = state.GetVelocity();
        _logRow.angular_speed_radps = state.GetRotationalVelocity();
        // LoopController refreshes runtime state before calling the mode callback, so these
        // command fields match the sampled state for this log row.
        _logRow.cmd_linear_mps = driveTelemetry.commandedLinearSpeedMps;
        _logRow.cmd_angular_radps = driveTelemetry.commandedAngularSpeedRadps;
        _logRow.left_drive_cmd = driveTelemetry.leftDriveCommand;
        _logRow.right_drive_cmd = driveTelemetry.rightDriveCommand;
        _logRow.left_encoder_count = driveTelemetry.leftEncoderCount;
        _logRow.right_encoder_count = driveTelemetry.rightEncoderCount;
        _logRow.left_distance_m = driveTelemetry.leftDistanceM;
        _logRow.right_distance_m = driveTelemetry.rightDistanceM;
        _logRow.left_velocity_mps = driveTelemetry.leftVelocityMps;
        _logRow.right_velocity_mps = driveTelemetry.rightVelocityMps;
        _logRow.imu_fr_status = snapshot.imuFrontRight.status;
        _logRow.imu_fr_gyro_x = snapshot.imuFrontRight.gyroX;
        _logRow.imu_fr_gyro_y = snapshot.imuFrontRight.gyroY;
        _logRow.imu_fr_gyro_z = snapshot.imuFrontRight.gyroZ;
        _logRow.imu_fr_accel_x = snapshot.imuFrontRight.accelX;
        _logRow.imu_fr_accel_y = snapshot.imuFrontRight.accelY;
        _logRow.imu_fr_accel_z = snapshot.imuFrontRight.accelZ;
        _logRow.imu_fr_temp = snapshot.imuFrontRight.temp;
        _logRow.imu_fr_int = snapshot.imuFrontRight.interruptHigh ? 1U : 0U;
        _logRow.imu_bl_status = snapshot.imuBackLeft.status;
        _logRow.imu_bl_gyro_x = snapshot.imuBackLeft.gyroX;
        _logRow.imu_bl_gyro_y = snapshot.imuBackLeft.gyroY;
        _logRow.imu_bl_gyro_z = snapshot.imuBackLeft.gyroZ;
        _logRow.imu_bl_accel_x = snapshot.imuBackLeft.accelX;
        _logRow.imu_bl_accel_y = snapshot.imuBackLeft.accelY;
        _logRow.imu_bl_accel_z = snapshot.imuBackLeft.accelZ;
        _logRow.imu_bl_temp = snapshot.imuBackLeft.temp;
        _logRow.imu_bl_int = snapshot.imuBackLeft.interruptHigh ? 1U : 0U;
        _logRow.ws_fl_ambient = snapshot.frontLeft.ambientLight;
        _logRow.ws_fl_lit = snapshot.frontLeft.litLight;
        _logRow.ws_fl_delta = snapshot.frontLeft.differentialLight;
        _logRow.ws_fl_raw_distance_m = snapshot.frontLeft.rawDistanceM;
        _logRow.ws_fl_distance_m = snapshot.frontLeft.distanceM;
        _logRow.ws_fr_ambient = snapshot.frontRight.ambientLight;
        _logRow.ws_fr_lit = snapshot.frontRight.litLight;
        _logRow.ws_fr_delta = snapshot.frontRight.differentialLight;
        _logRow.ws_fr_raw_distance_m = snapshot.frontRight.rawDistanceM;
        _logRow.ws_fr_distance_m = snapshot.frontRight.distanceM;
        _logRow.ws_sl_ambient = snapshot.sideLeft.ambientLight;
        _logRow.ws_sl_lit = snapshot.sideLeft.litLight;
        _logRow.ws_sl_delta = snapshot.sideLeft.differentialLight;
        _logRow.ws_sl_raw_distance_m = snapshot.sideLeft.rawDistanceM;
        _logRow.ws_sl_distance_m = snapshot.sideLeft.distanceM;
        _logRow.ws_sr_ambient = snapshot.sideRight.ambientLight;
        _logRow.ws_sr_lit = snapshot.sideRight.litLight;
        _logRow.ws_sr_delta = snapshot.sideRight.differentialLight;
        _logRow.ws_sr_raw_distance_m = snapshot.sideRight.rawDistanceM;
        _logRow.ws_sr_distance_m = snapshot.sideRight.distanceM;
        _logRow.front_wall = snapshot.frontWall ? 1U : 0U;
        _logRow.left_wall = snapshot.leftWall ? 1U : 0U;
        _logRow.right_wall = snapshot.rightWall ? 1U : 0U;
        _logRow.corridor_error_m = snapshot.corridorErrorM;
        _logRow.front_skew_m = snapshot.frontSkewM;
        _logRow.gyro_bias_radps = snapshot.gyroBiasRadps;
        _logRow.gyro_raw_radps = snapshot.gyroRawRadps;
        _logRow.gyro_radps = snapshot.gyroRadps;
        _logRowBuffered = true;
    }
    using PhaseFn = CommandVector (DiagnosticController::*)(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController);

    enum class ScenarioStep : std::uint8_t
    {
        None,
        StartupSettle,
        BaselineIdle,
        KickoffSweepPrep,
        KickoffSweepSample,
        RecoveryTurnaround,
        RecoveryReturn,
        RecoveryResetHeading,
        RecoverySettle,
        ForwardSweepPrep,
        ForwardSweepSample,
        CharacterizationSettle,
        TurnCw90_1,
        TurnCcw90_1,
        TurnCw90_2,
        TurnCcw90_2,
        TurnCw180,
        TurnCcw180,
        TurnSweepSettle,
        StraightShortForward,
        StraightShortTurnaround,
        StraightShortReturn,
        StraightShortResetHeading,
        StraightShortSettle,
        StraightLongForward,
        StraightLongTurnaround,
        StraightLongReturn,
        StraightLongResetHeading,
        StraightLongSettle,
        CircleSlowCwHalf1,
        CircleSlowCwHalf2,
        CircleSlowCwSettle,
        CircleSlowCcwHalf1,
        CircleSlowCcwHalf2,
        CircleSlowSettle,
        CircleMediumCwHalf1,
        CircleMediumCwHalf2,
        CircleMediumCwSettle,
        CircleMediumCcwHalf1,
        CircleMediumCcwHalf2,
        CircleMediumSettle,
        CircleFastCwHalf1,
        CircleFastCwHalf2,
        CircleFastCwSettle,
        CircleFastCcwHalf1,
        CircleFastCcwHalf2,
        CircleFastSettle,
        SquareCwLeg1,
        SquareCwTurn1,
        SquareCwLeg2,
        SquareCwTurn2,
        SquareCwLeg3,
        SquareCwTurn3,
        SquareCwLeg4,
        SquareCwTurn4,
        SquareCwSettle,
        SquareCcwLeg1,
        SquareCcwTurn1,
        SquareCcwLeg2,
        SquareCcwTurn2,
        SquareCcwLeg3,
        SquareCcwTurn3,
        SquareCcwLeg4,
        SquareCcwTurn4,
        FinalIdle,
        Complete
    };

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<DiagnosticController*>(context)->OnRuntimeFault(reason);
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
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
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    MazeMap::App::Internal::Drive& _driveService;
    char _logFileName[64];
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    DiagnosticLogRow _logRow{};
    bool _logRowBuffered{};
    PhaseFn _phaseFn{};

    bool _holdStationary{};
    unsigned long _holdDeadlineMs{};
    ScenarioStep _holdNextStep{ ScenarioStep::None };

    struct StraightPhaseState final
    {
        const char* phaseName{};
        float distanceM{};
        float cruiseSpeedMps{};
        float startDistanceM{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        float commandedSpeedMps{};
        float traveledM{};
        unsigned long timeoutMs{};
        EncoderProgressWatchdog translationWatchdog{};
        ScenarioStep nextStep{ ScenarioStep::None };
        bool selectReturnDistance{};
        float peakSpeedMps{};
        float maxHeadingErrorRad{};
    } _straightPhaseState{};

    struct KickoffPhaseState final
    {
        char label[24]{};
        float driveCommand{};
        float startDistanceM{};
        unsigned long pulseDeadlineMs{};
        unsigned long settleDeadlineMs{};
        float travelLimitM{};
        float maxSpeedMps{};
        bool travelLimited{};
        unsigned long travelLimitSettleDeadlineMs{};
    } _kickoffPhaseState{};

    std::size_t _kickoffSweepNextMagnitudeIndex{};

    struct ForwardPhaseState final
    {
        char label[24]{};
        float forwardDriveCommand{};
        float startDistanceM{};
        unsigned long kickoffDeadlineMs{};
        unsigned long holdDeadlineMs{};
        unsigned long settleDeadlineMs{};
        float travelLimitM{};
        float maxSpeedMps{};
        float holdStartDistanceM{};
        float holdEndDistanceM{};
        float holdElapsedSeconds{};
        bool holdStarted{};
        bool holdComplete{};
        bool travelLimited{};
        unsigned long travelLimitSettleDeadlineMs{};
    } _forwardPhaseState{};

    std::uint16_t _forwardSweepNextSampleIndex{};

    struct TurnPhaseState final
    {
        const char* phaseName{};
        float angleRad{};
        float targetYawRad{};
        unsigned long timeoutMs{};
        ScenarioStep nextStep{ ScenarioStep::None };
        bool writeSquareClosure{};
        float peakOmegaRadps{};
        float maxYawErrorRad{};
    } _turnPhaseState{};

    struct ArcPhaseState final
    {
        const char* phaseName{};
        float distanceM{};
        float angleRad{};
        float cruiseSpeedMps{};
        MotionLimits limits{};
        float startDistanceM{};
        float startYawRad{};
        float targetYawRad{};
        float curvature{};
        float commandedSpeedMps{};
        float traveledM{};
        unsigned long timeoutMs{};
        EncoderProgressWatchdog translationWatchdog{};
        ArcPhaseMetrics metrics{};
        ScenarioStep nextStep{ ScenarioStep::None };
        bool writeCircleSummary{};
    } _arcPhaseState{};

    char _characterizationRecoveryLabel[24]{};
    float _characterizationRecoveryTraveledDistanceM{};
    ScenarioStep _characterizationRecoveryNextStep{ ScenarioStep::None };

    char _circleSequenceNamePrefix[48]{};
    float _circleSequenceCruiseSpeedMps{};
    float _circleSequenceStartX{};
    float _circleSequenceStartY{};
    float _circleSequenceStartYawRad{};
    DriveTelemetry _circleSequenceStartTelemetry{};
    ArcPhaseMetrics _circleSequenceTotalMetrics{};

    char _squareSequenceNamePrefix[24]{};
    float _squareSequenceStartX{};
    float _squareSequenceStartY{};
    float _squareSequenceStartYawRad{};

    float _pendingReturnDistanceM;
    bool _cleanupComplete{};

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        const auto& runtimeState = _runtime.RuntimeState();
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.SessionStartPointX = runtimeState.GetPositionX();
        options.SessionStartPointY = runtimeState.GetPositionY();
        return options;
    }

    bool AdvanceToNextStep(
        const ScenarioStep nextStep,
        LoopController& loopController,
        const char* const failureMessage)
    {
        if (nextStep == ScenarioStep::Complete)
        {
            CompleteMode(loopController);
            return true;
        }

        if (StartScenarioStep(nextStep))
        {
            return true;
        }

        _runtime.FailActiveMode(failureMessage);
        return false;
    }

    void ResetModeState() noexcept
    {
        _phaseFn = nullptr;
        _faulted = false;
        _cleanupComplete = false;
        _pendingReturnDistanceM = 0.0f;
        _logRowBuffered = false;
    }

    void CleanupMode() noexcept
    {
        if (_cleanupComplete)
        {
            return;
        }

        _phaseFn = nullptr;
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        CloseLog();
        SetMissionLevelFanEnabled(false);
        _cleanupComplete = true;
    }

    void CompleteMode(LoopController& loopController) noexcept
    {
        loopController.RequestEndSession(
            +[](void* const context, LoopController& boundaryLoopController)
            {
                auto* const self = static_cast<DiagnosticController*>(context);
                if (self == nullptr)
                {
                    GetSharedRobotRuntime().FailActiveMode(
                        "Diagnostic completion callback context was null");
                    return;
                }

                if (!self->WriteBufferedSample())
                {
                    self->_runtime.FailActiveMode("Failed to write final diagnostic sample");
                }

                (void)self->_runtime.AppendTextLogFormatted(
                    "Diagnostic complete, log saved to %s",
                    self->GetLogFileName());
                (void)self->_runtime.AppendTextLogLine(
                    "Use the # event,summary lines in the log header to map phases to tunables.");
                self->CleanupMode();
                boundaryLoopController.HaltExecutionEndProgram();
            },
            this);
    }

    bool StartHoldPhase(
        const char* const phaseName,
        const uint16_t durationMs,
        const bool stationary,
        const ScenarioStep nextStep)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _holdStationary = stationary;
        _holdDeadlineMs = millis() + durationMs;
        _holdNextStep = nextStep;
        _phaseFn = &DiagnosticController::HoldPhaseTick;
        return true;
    }

    bool StartStraightPhase(
        const char* const phaseName,
        const float distanceM,
        const float cruiseSpeedMps,
        const ScenarioStep nextStep,
        const bool selectReturnDistance = false)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _straightPhaseState = StraightPhaseState{};
        _straightPhaseState.phaseName = phaseName;
        _straightPhaseState.distanceM = distanceM;
        _straightPhaseState.cruiseSpeedMps = cruiseSpeedMps;
        _straightPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _straightPhaseState.targetHeading = _runtime.RuntimeState().GetHeadingUnit();
        _straightPhaseState.timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (6000.0f * distanceM));
        _straightPhaseState.translationWatchdog.Reset(0.0f, millis());
        _straightPhaseState.nextStep = nextStep;
        _straightPhaseState.selectReturnDistance = selectReturnDistance;
        _phaseFn = &DiagnosticController::StraightPhaseTick;
        return true;
    }

    bool StartKickoffCharacterizationSample(const float driveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("kickoff", driveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _kickoffPhaseState = KickoffPhaseState{};
        snprintf(_kickoffPhaseState.label, sizeof(_kickoffPhaseState.label), "%s", label);
        _kickoffPhaseState.driveCommand = driveCommand;
        _kickoffPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _kickoffPhaseState.pulseDeadlineMs = millis() + LegacyDiagnosticConfig::kKickoffSweepPulseMs;
        _kickoffPhaseState.settleDeadlineMs = _kickoffPhaseState.pulseDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        _kickoffPhaseState.travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        _phaseFn = &DiagnosticController::KickoffCharacterizationTick;
        return true;
    }

    bool StartNextKickoffCharacterizationSample()
    {
        if (_kickoffSweepNextMagnitudeIndex >= MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
        {
            return StartScenarioStep(ScenarioStep::ForwardSweepPrep);
        }

        const float driveCommand = MazeMap::kOpenFloorLaunchDriveMagnitudes[_kickoffSweepNextMagnitudeIndex];
        ++_kickoffSweepNextMagnitudeIndex;
        return StartKickoffCharacterizationSample(driveCommand);
    }

    bool FinishKickoffCharacterizationSample()
    {
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - _kickoffPhaseState.startDistanceM);
        const bool moved =
            (traveledDistanceM >= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdM) ||
            (_kickoffPhaseState.maxSpeedMps >= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdMps);

        char message[192] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cmd=%.2f;dist_m=%.4f;max_speed_mps=%.3f;moved=%u;travel_limited=%u",
            _kickoffPhaseState.label,
            _kickoffPhaseState.driveCommand,
            traveledDistanceM,
            _kickoffPhaseState.maxSpeedMps,
            moved ? 1U : 0U,
            _kickoffPhaseState.travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            _runtime.FailActiveMode("Failed to format kickoff characterization result");
            return false;
        }
        if (!WriteEventOrFail("kickoff_result", message, "Failed to write kickoff characterization result"))
        {
            return false;
        }

        return StartCharacterizationRecovery(
            _kickoffPhaseState.label,
            traveledDistanceM,
            ScenarioStep::KickoffSweepSample);
    }

    bool StartForwardCharacterizationSample(const float forwardDriveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("forward", forwardDriveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _forwardPhaseState = ForwardPhaseState{};
        snprintf(_forwardPhaseState.label, sizeof(_forwardPhaseState.label), "%s", label);
        _forwardPhaseState.forwardDriveCommand = forwardDriveCommand;
        _forwardPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _forwardPhaseState.kickoffDeadlineMs = millis() + LegacyDiagnosticConfig::kForwardSweepKickoffMs;
        _forwardPhaseState.holdDeadlineMs = _forwardPhaseState.kickoffDeadlineMs + LegacyDiagnosticConfig::kForwardSweepHoldMs;
        _forwardPhaseState.settleDeadlineMs = _forwardPhaseState.holdDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        _forwardPhaseState.travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        _phaseFn = &DiagnosticController::ForwardCharacterizationTick;
        return true;
    }

    bool StartNextForwardCharacterizationSample()
    {
        const float driveCommand =
            LegacyDiagnosticConfig::kForwardSweepMinDriveCommand +
            (LegacyDiagnosticConfig::kForwardSweepStepDriveCommand * static_cast<float>(_forwardSweepNextSampleIndex));
        if (driveCommand > (LegacyDiagnosticConfig::kForwardSweepMaxDriveCommand + 0.0001f))
        {
            return StartScenarioStep(ScenarioStep::CharacterizationSettle);
        }

        ++_forwardSweepNextSampleIndex;
        return StartForwardCharacterizationSample(driveCommand);
    }

    bool FinishForwardCharacterizationSample()
    {
        if (!_forwardPhaseState.holdComplete)
        {
            _forwardPhaseState.holdComplete = true;
            _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        const float totalDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - _forwardPhaseState.startDistanceM);
        const float holdDistanceM =
            _forwardPhaseState.holdStarted ?
            std::fabs(_forwardPhaseState.holdEndDistanceM - _forwardPhaseState.holdStartDistanceM) :
            0.0f;
        const float averageHoldSpeedMps =
            (_forwardPhaseState.holdElapsedSeconds > 0.0f) ?
            (holdDistanceM / _forwardPhaseState.holdElapsedSeconds) :
            0.0f;
        const bool carried =
            (averageHoldSpeedMps >= LegacyDiagnosticConfig::kForwardSweepCarryThresholdMps) ||
            (holdDistanceM >= LegacyDiagnosticConfig::kForwardSweepCarryThresholdM);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;kickoff=%.2f;hold=%.2f;hold_dist_m=%.4f;hold_avg_speed_mps=%.3f;total_dist_m=%.4f;max_speed_mps=%.3f;carried=%u;travel_limited=%u",
            _forwardPhaseState.label,
            LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand,
            _forwardPhaseState.forwardDriveCommand,
            holdDistanceM,
            averageHoldSpeedMps,
            totalDistanceM,
            _forwardPhaseState.maxSpeedMps,
            carried ? 1U : 0U,
            _forwardPhaseState.travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            _runtime.FailActiveMode("Failed to format forward characterization result");
            return false;
        }
        if (!WriteEventOrFail("forward_result", message, "Failed to write forward characterization result"))
        {
            return false;
        }

        return StartCharacterizationRecovery(
            _forwardPhaseState.label,
            totalDistanceM,
            ScenarioStep::ForwardSweepSample);
    }

    bool StartTurnPhase(
        const char* const phaseName,
        const float angleRad,
        const ScenarioStep nextStep,
        const bool writeSquareClosure = false)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _turnPhaseState = TurnPhaseState{};
        _turnPhaseState.phaseName = phaseName;
        _turnPhaseState.angleRad = angleRad;
        _turnPhaseState.targetYawRad = WrapAngleRad(_runtime.RuntimeState().GetOrientation() + angleRad);
        _turnPhaseState.timeoutMs = millis() + 3000UL;
        _turnPhaseState.nextStep = nextStep;
        _turnPhaseState.writeSquareClosure = writeSquareClosure;
        _driveService.SetLimits(DiagnosticLimits(0.0f));
        _driveService.SetOperationMode(MazeMap::App::Internal::Drive::OperationMode::OpenFloor);
        _driveService.StartTurn(angleRad);
        _phaseFn = &DiagnosticController::TurnPhaseTick;
        return true;
    }

    bool StartArcPhase(
        const char* const phaseName,
        const float distanceM,
        const float angleRad,
        const float cruiseSpeedMps,
        const ScenarioStep nextStep,
        const bool writeCircleSummary)
    {
        if (distanceM <= 0.0f)
        {
            _runtime.FailActiveMode("Diagnostic arc distance must be positive");
            return false;
        }
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _arcPhaseState = ArcPhaseState{};
        _arcPhaseState.phaseName = phaseName;
        _arcPhaseState.distanceM = distanceM;
        _arcPhaseState.angleRad = angleRad;
        _arcPhaseState.cruiseSpeedMps = cruiseSpeedMps;
        _arcPhaseState.limits = DiagnosticLimits(cruiseSpeedMps);
        _arcPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _arcPhaseState.startYawRad = _runtime.RuntimeState().GetOrientation();
        _arcPhaseState.targetYawRad = WrapAngleRad(_arcPhaseState.startYawRad + angleRad);
        _arcPhaseState.curvature = angleRad / distanceM;
        _arcPhaseState.timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        _arcPhaseState.translationWatchdog.Reset(0.0f, millis());
        _arcPhaseState.nextStep = nextStep;
        _arcPhaseState.writeCircleSummary = writeCircleSummary;
        _phaseFn = &DiagnosticController::ArcPhaseTick;
        return true;
    }

    void ResetCircleSequence(const char* const namePrefix, const float cruiseSpeedMps)
    {
        snprintf(_circleSequenceNamePrefix, sizeof(_circleSequenceNamePrefix), "%s", (namePrefix != nullptr) ? namePrefix : "");
        _circleSequenceCruiseSpeedMps = cruiseSpeedMps;
        _circleSequenceStartX = _runtime.RuntimeState().GetPositionX();
        _circleSequenceStartY = _runtime.RuntimeState().GetPositionY();
        _circleSequenceStartYawRad = _runtime.RuntimeState().GetOrientation();
        _circleSequenceStartTelemetry = _drive.GetTelemetry();
        _circleSequenceTotalMetrics = ArcPhaseMetrics{};
    }

    void ResetSquareSequence(const char* const namePrefix)
    {
        snprintf(_squareSequenceNamePrefix, sizeof(_squareSequenceNamePrefix), "%s", (namePrefix != nullptr) ? namePrefix : "");
        _squareSequenceStartX = _runtime.RuntimeState().GetPositionX();
        _squareSequenceStartY = _runtime.RuntimeState().GetPositionY();
        _squareSequenceStartYawRad = _runtime.RuntimeState().GetOrientation();
    }

    bool StartCharacterizationRecovery(
        const char* const label,
        const float traveledDistanceM,
        const ScenarioStep nextStepAfterRecovery)
    {
        snprintf(_characterizationRecoveryLabel, sizeof(_characterizationRecoveryLabel), "%s", (label != nullptr) ? label : "");
        _characterizationRecoveryTraveledDistanceM = traveledDistanceM;
        _characterizationRecoveryNextStep = nextStepAfterRecovery;

        if (traveledDistanceM <= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdM)
        {
            return StartScenarioStep(ScenarioStep::RecoverySettle);
        }

        return StartScenarioStep(ScenarioStep::RecoveryTurnaround);
    }

    bool StartScenarioStep(const ScenarioStep step)
    {
        char phaseName[48] = {};
        switch (step)
        {
        case ScenarioStep::StartupSettle:
            return StartHoldPhase(
                "startup_settle",
                DiagnosticConfig::kStartupSettleMs,
                true,
                ScenarioStep::BaselineIdle);

        case ScenarioStep::BaselineIdle:
            return StartHoldPhase(
                "baseline_idle",
                DiagnosticConfig::kBaselineHoldMs,
                true,
                ScenarioStep::KickoffSweepPrep);

        case ScenarioStep::KickoffSweepPrep:
            _kickoffSweepNextMagnitudeIndex = 0;
            return StartHoldPhase(
                "kickoff_sweep_prep",
                DiagnosticConfig::kCharacterizationSettleMs,
                true,
                ScenarioStep::KickoffSweepSample);

        case ScenarioStep::KickoffSweepSample:
            return StartNextKickoffCharacterizationSample();

        case ScenarioStep::RecoveryTurnaround:
            snprintf(phaseName, sizeof(phaseName), "%s_turnaround", _characterizationRecoveryLabel);
            return StartTurnPhase(phaseName, PI_F, ScenarioStep::RecoveryReturn);

        case ScenarioStep::RecoveryReturn:
            snprintf(phaseName, sizeof(phaseName), "%s_return", _characterizationRecoveryLabel);
            return StartStraightPhase(
                phaseName,
                _characterizationRecoveryTraveledDistanceM,
                DiagnosticConfig::kCharacterizationRecoverySpeedMps,
                ScenarioStep::RecoveryResetHeading);

        case ScenarioStep::RecoveryResetHeading:
            snprintf(phaseName, sizeof(phaseName), "%s_reset_heading", _characterizationRecoveryLabel);
            return StartTurnPhase(phaseName, PI_F, ScenarioStep::RecoverySettle);

        case ScenarioStep::RecoverySettle:
            snprintf(phaseName, sizeof(phaseName), "%s_settle", _characterizationRecoveryLabel);
            return StartHoldPhase(
                phaseName,
                DiagnosticConfig::kCharacterizationSettleMs,
                true,
                _characterizationRecoveryNextStep);

        case ScenarioStep::ForwardSweepPrep:
            _forwardSweepNextSampleIndex = 0;
            return StartHoldPhase(
                "forward_sweep_prep",
                DiagnosticConfig::kCharacterizationSettleMs,
                true,
                ScenarioStep::ForwardSweepSample);

        case ScenarioStep::ForwardSweepSample:
            return StartNextForwardCharacterizationSample();

        case ScenarioStep::CharacterizationSettle:
            return StartHoldPhase(
                "characterization_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::TurnCw90_1);

        case ScenarioStep::TurnCw90_1:
            return StartTurnPhase("turn_cw_90_1", HALF_PI_F, ScenarioStep::TurnCcw90_1);

        case ScenarioStep::TurnCcw90_1:
            return StartTurnPhase("turn_ccw_90_1", -HALF_PI_F, ScenarioStep::TurnCw90_2);

        case ScenarioStep::TurnCw90_2:
            return StartTurnPhase("turn_cw_90_2", HALF_PI_F, ScenarioStep::TurnCcw90_2);

        case ScenarioStep::TurnCcw90_2:
            return StartTurnPhase("turn_ccw_90_2", -HALF_PI_F, ScenarioStep::TurnCw180);

        case ScenarioStep::TurnCw180:
            return StartTurnPhase("turn_cw_180", PI_F, ScenarioStep::TurnCcw180);

        case ScenarioStep::TurnCcw180:
            return StartTurnPhase("turn_ccw_180", -PI_F, ScenarioStep::TurnSweepSettle);

        case ScenarioStep::TurnSweepSettle:
            return StartHoldPhase(
                "turn_sweep_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::StraightShortForward);

        case ScenarioStep::StraightShortForward:
            return StartStraightPhase(
                "straight_short_forward",
                LegacyDiagnosticConfig::kShortStraightDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::StraightShortTurnaround,
                true);

        case ScenarioStep::StraightShortTurnaround:
            return StartTurnPhase("straight_short_turnaround", PI_F, ScenarioStep::StraightShortReturn);

        case ScenarioStep::StraightShortReturn:
            return StartStraightPhase(
                "straight_short_return",
                _pendingReturnDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::StraightShortResetHeading);

        case ScenarioStep::StraightShortResetHeading:
            return StartTurnPhase("straight_short_reset_heading", PI_F, ScenarioStep::StraightShortSettle);

        case ScenarioStep::StraightShortSettle:
            return StartHoldPhase(
                "straight_short_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::StraightLongForward);

        case ScenarioStep::StraightLongForward:
            return StartStraightPhase(
                "straight_long_forward",
                LegacyDiagnosticConfig::kLongStraightDistanceM,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::StraightLongTurnaround,
                true);

        case ScenarioStep::StraightLongTurnaround:
            return StartTurnPhase("straight_long_turnaround", PI_F, ScenarioStep::StraightLongReturn);

        case ScenarioStep::StraightLongReturn:
            return StartStraightPhase(
                "straight_long_return",
                _pendingReturnDistanceM,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::StraightLongResetHeading);

        case ScenarioStep::StraightLongResetHeading:
            return StartTurnPhase("straight_long_reset_heading", PI_F, ScenarioStep::StraightLongSettle);

        case ScenarioStep::StraightLongSettle:
            return StartHoldPhase(
                "straight_long_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleSlowCwHalf1);

        case ScenarioStep::CircleSlowCwHalf1:
            ResetCircleSequence("circle_cw_slow", LegacyDiagnosticConfig::kSlowStraightSpeedMps);
            return StartArcPhase(
                "circle_cw_slow_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::CircleSlowCwHalf2,
                false);

        case ScenarioStep::CircleSlowCwHalf2:
            return StartArcPhase(
                "circle_cw_slow_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::CircleSlowCwSettle,
                true);

        case ScenarioStep::CircleSlowCwSettle:
            return StartHoldPhase(
                "circle_cw_slow_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleSlowCcwHalf1);

        case ScenarioStep::CircleSlowCcwHalf1:
            ResetCircleSequence("circle_ccw_slow", LegacyDiagnosticConfig::kSlowStraightSpeedMps);
            return StartArcPhase(
                "circle_ccw_slow_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::CircleSlowCcwHalf2,
                false);

        case ScenarioStep::CircleSlowCcwHalf2:
            return StartArcPhase(
                "circle_ccw_slow_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::CircleSlowSettle,
                true);

        case ScenarioStep::CircleSlowSettle:
            return StartHoldPhase(
                "circle_slow_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleMediumCwHalf1);

        case ScenarioStep::CircleMediumCwHalf1:
            ResetCircleSequence("circle_cw_medium", LegacyDiagnosticConfig::kCircleMediumSpeedMps);
            return StartArcPhase(
                "circle_cw_medium_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kCircleMediumSpeedMps,
                ScenarioStep::CircleMediumCwHalf2,
                false);

        case ScenarioStep::CircleMediumCwHalf2:
            return StartArcPhase(
                "circle_cw_medium_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kCircleMediumSpeedMps,
                ScenarioStep::CircleMediumCwSettle,
                true);

        case ScenarioStep::CircleMediumCwSettle:
            return StartHoldPhase(
                "circle_cw_medium_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleMediumCcwHalf1);

        case ScenarioStep::CircleMediumCcwHalf1:
            ResetCircleSequence("circle_ccw_medium", LegacyDiagnosticConfig::kCircleMediumSpeedMps);
            return StartArcPhase(
                "circle_ccw_medium_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kCircleMediumSpeedMps,
                ScenarioStep::CircleMediumCcwHalf2,
                false);

        case ScenarioStep::CircleMediumCcwHalf2:
            return StartArcPhase(
                "circle_ccw_medium_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kCircleMediumSpeedMps,
                ScenarioStep::CircleMediumSettle,
                true);

        case ScenarioStep::CircleMediumSettle:
            return StartHoldPhase(
                "circle_medium_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleFastCwHalf1);

        case ScenarioStep::CircleFastCwHalf1:
            ResetCircleSequence("circle_cw_fast", LegacyDiagnosticConfig::kFastStraightSpeedMps);
            return StartArcPhase(
                "circle_cw_fast_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::CircleFastCwHalf2,
                false);

        case ScenarioStep::CircleFastCwHalf2:
            return StartArcPhase(
                "circle_cw_fast_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                PI_F,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::CircleFastCwSettle,
                true);

        case ScenarioStep::CircleFastCwSettle:
            return StartHoldPhase(
                "circle_cw_fast_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::CircleFastCcwHalf1);

        case ScenarioStep::CircleFastCcwHalf1:
            ResetCircleSequence("circle_ccw_fast", LegacyDiagnosticConfig::kFastStraightSpeedMps);
            return StartArcPhase(
                "circle_ccw_fast_half_1",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::CircleFastCcwHalf2,
                false);

        case ScenarioStep::CircleFastCcwHalf2:
            return StartArcPhase(
                "circle_ccw_fast_half_2",
                LegacyDiagnosticConfig::kArcHalfCircleDistanceM,
                -PI_F,
                LegacyDiagnosticConfig::kFastStraightSpeedMps,
                ScenarioStep::CircleFastSettle,
                true);

        case ScenarioStep::CircleFastSettle:
            return StartHoldPhase(
                "circle_fast_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::SquareCwLeg1);

        case ScenarioStep::SquareCwLeg1:
            ResetSquareSequence("square_cw");
            return StartStraightPhase(
                "square_cw_leg_1",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCwTurn1);

        case ScenarioStep::SquareCwTurn1:
            return StartTurnPhase("square_cw_turn_1", HALF_PI_F, ScenarioStep::SquareCwLeg2);

        case ScenarioStep::SquareCwLeg2:
            return StartStraightPhase(
                "square_cw_leg_2",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCwTurn2);

        case ScenarioStep::SquareCwTurn2:
            return StartTurnPhase("square_cw_turn_2", HALF_PI_F, ScenarioStep::SquareCwLeg3);

        case ScenarioStep::SquareCwLeg3:
            return StartStraightPhase(
                "square_cw_leg_3",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCwTurn3);

        case ScenarioStep::SquareCwTurn3:
            return StartTurnPhase("square_cw_turn_3", HALF_PI_F, ScenarioStep::SquareCwLeg4);

        case ScenarioStep::SquareCwLeg4:
            return StartStraightPhase(
                "square_cw_leg_4",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCwTurn4);

        case ScenarioStep::SquareCwTurn4:
            return StartTurnPhase("square_cw_turn_4", HALF_PI_F, ScenarioStep::SquareCwSettle, true);

        case ScenarioStep::SquareCwSettle:
            return StartHoldPhase(
                "square_cw_settle",
                DiagnosticConfig::kInterTestHoldMs,
                true,
                ScenarioStep::SquareCcwLeg1);

        case ScenarioStep::SquareCcwLeg1:
            ResetSquareSequence("square_ccw");
            return StartStraightPhase(
                "square_ccw_leg_1",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCcwTurn1);

        case ScenarioStep::SquareCcwTurn1:
            return StartTurnPhase("square_ccw_turn_1", -HALF_PI_F, ScenarioStep::SquareCcwLeg2);

        case ScenarioStep::SquareCcwLeg2:
            return StartStraightPhase(
                "square_ccw_leg_2",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCcwTurn2);

        case ScenarioStep::SquareCcwTurn2:
            return StartTurnPhase("square_ccw_turn_2", -HALF_PI_F, ScenarioStep::SquareCcwLeg3);

        case ScenarioStep::SquareCcwLeg3:
            return StartStraightPhase(
                "square_ccw_leg_3",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCcwTurn3);

        case ScenarioStep::SquareCcwTurn3:
            return StartTurnPhase("square_ccw_turn_3", -HALF_PI_F, ScenarioStep::SquareCcwLeg4);

        case ScenarioStep::SquareCcwLeg4:
            return StartStraightPhase(
                "square_ccw_leg_4",
                LegacyDiagnosticConfig::kSquareLegDistanceM,
                LegacyDiagnosticConfig::kSlowStraightSpeedMps,
                ScenarioStep::SquareCcwTurn4);

        case ScenarioStep::SquareCcwTurn4:
            return StartTurnPhase("square_ccw_turn_4", -HALF_PI_F, ScenarioStep::FinalIdle, true);

        case ScenarioStep::FinalIdle:
            return StartHoldPhase(
                "final_idle",
                DiagnosticConfig::kBaselineHoldMs / 2U,
                true,
                ScenarioStep::Complete);

        case ScenarioStep::Complete:
        case ScenarioStep::None:
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
                "diag%03u.mmlog",
                "diagnostic_log.mmlog"))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", "diagnostic")) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !_runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                DiagnosticConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;
        if (!MazeMap::App::Internal::Runtime::WriteDiagnosticTuningEvents(
                [this](const char* type, const char* message) -> bool
                {
                    return _runtime.WriteTextLogEntry(micros(), type, message);
                })) return false;

        _logRow = {};
        if (!_runtime.BeginUtilityDataLogSchema(_logRow))
        {
            return false;
        }

        if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return false;
        if (!_runtime.WriteTextLogMetadata("data_file", _logFileName)) return false;
        if (!_runtime.WriteTextLogMetadata("mode", "diagnostic")) return false;
        return MazeMap::App::Internal::Runtime::WriteDiagnosticSummaryInstructions(
            [this](const char* type, const char* message) -> bool
            {
                return _runtime.WriteTextLogEntry(micros(), type, message);
            });
    }

    bool WriteLogEvent(const char* type, const char* message)
    {
        return _runtime.WriteTextLogEntry(micros(), type, message);
    }

    bool WritePhaseMarker(const char* name)
    {
        ++_phaseId;
        return _runtime.WriteTextLogPhase(_phaseId, micros(), name);
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

    bool WriteStraightResult(
        const char* phaseName,
        float distanceM,
        float cruiseSpeedMps,
        float traveledM,
        const Eigen::Vector2f& targetHeading,
        float peakSpeedMps,
        float maxHeadingErrorRad,
        const MazeMap::VehicleState& state)
    {
        char message[192] = {};
        const float stopErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * HeadingErrorRad(targetHeading, state.GetHeadingUnit());
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;distance_m=%.3f;cruise_mps=%.3f;peak_speed_mps=%.3f;max_heading_err_deg=%.2f;stop_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            cruiseSpeedMps,
            peakSpeedMps,
            RAD_TO_DEG_F * maxHeadingErrorRad,
            stopErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            _runtime.FailActiveMode("Failed to format straight diagnostic result");
            return false;
        }
        return WriteEventOrFail("straight_result", message, "Failed to write straight diagnostic result");
    }

    bool WriteTurnResult(
        const char* phaseName,
        float angleRad,
        float peakOmegaRadps,
        float maxYawErrorRad,
        float targetYawRad,
        const MazeMap::VehicleState& state)
    {
        char message[176] = {};
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, state.GetOrientation());
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;angle_deg=%.1f;peak_omega_radps=%.3f;peak_yaw_err_deg=%.2f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            RAD_TO_DEG_F * angleRad,
            peakOmegaRadps,
            RAD_TO_DEG_F * maxYawErrorRad,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            _runtime.FailActiveMode("Failed to format turn diagnostic result");
            return false;
        }
        return WriteEventOrFail("turn_result", message, "Failed to write turn diagnostic result");
    }

    bool WriteArcResult(
        const char* phaseName,
        float distanceM,
        float angleRad,
        float traveledM,
        float targetYawRad,
        const ArcPhaseMetrics& metrics,
        const MazeMap::VehicleState& state)
    {
        char message[192] = {};
        const float distanceErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, state.GetOrientation());
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
            _runtime.FailActiveMode("Failed to format arc diagnostic result");
            return false;
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
            _runtime.FailActiveMode("Failed to format circle diagnostic result");
            return false;
        }
        return WriteEventOrFail("circle_result", message, "Failed to write circle diagnostic result");
    }

    bool WriteClosureResult(
        const char* type,
        const char* phaseName,
        const float startXM,
        const float startYM,
        const float startYawRad,
        const MazeMap::VehicleState& state,
        const char* failMessage)
    {
        char message[160] = {};
        const float deltaXM = state.GetPositionX() - startXM;
        const float deltaYM = state.GetPositionY() - startYM;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startYawRad, state.GetOrientation());
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;closure_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            closureErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            _runtime.FailActiveMode("Failed to format diagnostic closure result");
            return false;
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

    static void BuildDriveCommandLabel(const char* prefix, float driveCommand, char* buffer, size_t bufferSize)
    {
        const unsigned drivePercent = static_cast<unsigned>((100.0f * driveCommand) + 0.5f);
        snprintf(buffer, bufferSize, "%s_%03u", (prefix != nullptr) ? prefix : "cmd", drivePercent);
    }

    static MazeMap::WheelControlProfile BuildDiagnosticWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }

    bool WriteEventOrFail(const char* type, const char* message, const char* failMessage)
    {
        if (WriteLogEvent(type, message))
        {
            return true;
        }

        _runtime.FailActiveMode(failMessage);
        return false;
    }

    void OnRuntimeFault(const char* message) noexcept
    {
        _faulted = true;
        (void)WriteLogEvent("fault", message);
        CleanupMode();
    }

    bool StartPhase(const char* name)
    {
        (void)_runtime.AppendTextLogFormatted("Diagnostic phase: %s", (name != nullptr) ? name : "unknown");
        if (WritePhaseMarker(name))
        {
            return true;
        }
        _runtime.FailActiveMode("Failed to write diagnostic phase marker");
        return false;
    }

    bool IsWithinBoundary(const MazeMap::VehicleState& state) const
    {
        return (std::fabs(state.GetPositionX() - _startX) <= DiagnosticConfig::kBoundaryHalfSpanM) &&
            (std::fabs(state.GetPositionY() - _startY) <= DiagnosticConfig::kBoundaryHalfSpanM);
    }

    CommandVector HoldPhaseTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)state;
        if (static_cast<long>(_holdDeadlineMs - millis()) <= 0)
        {
            (void)AdvanceToNextStep(
                _holdNextStep,
                loopController,
                "Failed to advance diagnostic hold phase");
        }

        return CommandVector::Brake();
    }

    CommandVector StraightPhaseTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        const MotionLimits limits = DiagnosticLimits(_straightPhaseState.cruiseSpeedMps);
        _straightPhaseState.traveledM =
            std::fabs(_drive.GetAverageDistanceMeters() - _straightPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _straightPhaseState.distanceM - _straightPhaseState.traveledM);
        _straightPhaseState.peakSpeedMps = (std::max)(
            _straightPhaseState.peakSpeedMps,
            std::fabs(state.GetVelocity()));
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            if (_straightPhaseState.selectReturnDistance)
            {
                _pendingReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(
                    _straightPhaseState.distanceM,
                    _straightPhaseState.traveledM);
            }
            if (!WriteStraightResult(
                    _straightPhaseState.phaseName,
                    _straightPhaseState.distanceM,
                _straightPhaseState.cruiseSpeedMps,
                _straightPhaseState.traveledM,
                _straightPhaseState.targetHeading,
                _straightPhaseState.peakSpeedMps,
                _straightPhaseState.maxHeadingErrorRad,
                state))
            {
                _runtime.FailActiveMode("Failed to write straight diagnostic result");
                return CommandVector::Brake();
            }
            (void)AdvanceToNextStep(
                _straightPhaseState.nextStep,
                loopController,
                "Failed to advance diagnostic straight phase");
            return CommandVector::Brake();
        }
        if (_straightPhaseState.translationWatchdog.Stalled(
                _straightPhaseState.traveledM,
                _straightPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            _runtime.FailActiveMode("Straight diagnostic encoder progress stalled");
            return CommandVector::Brake();
        }
        if (static_cast<long>(_straightPhaseState.timeoutMs - millis()) <= 0)
        {
            _runtime.FailActiveMode("Straight diagnostic phase timed out");
            return CommandVector::Brake();
        }

        const float accelLimitedSpeedMps = (std::min)(
            limits.maxSpeedMps,
            _straightPhaseState.commandedSpeedMps +
            (limits.accelMps2 * (static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f)));
        const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
        _straightPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float headingErrorRad = HeadingErrorRad(_straightPhaseState.targetHeading, state.GetHeadingUnit());
        _straightPhaseState.maxHeadingErrorRad = (std::max)(
            _straightPhaseState.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * state.GetRotationalVelocity());
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -limits.maxAngularSpeedRadps,
            limits.maxAngularSpeedRadps);

        return _drive.PointControlVector(
            _straightPhaseState.commandedSpeedMps,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    CommandVector KickoffCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)loopController;
        const unsigned long nowMs = millis();
        const float traveledDistanceM =
            std::fabs(_drive.GetAverageDistanceMeters() - _kickoffPhaseState.startDistanceM);
        if (!_kickoffPhaseState.travelLimited &&
            (_kickoffPhaseState.travelLimitM > 0.0f) &&
            (traveledDistanceM >= _kickoffPhaseState.travelLimitM))
        {
            _kickoffPhaseState.travelLimited = true;
            _kickoffPhaseState.travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
        }

        const bool pulseActive =
            !_kickoffPhaseState.travelLimited &&
            (static_cast<long>(_kickoffPhaseState.pulseDeadlineMs - nowMs) > 0);
        const CommandVector command =
            _kickoffPhaseState.travelLimited ?
            CommandVector::Brake() :
            pulseActive ?
            CommandVector(
                _kickoffPhaseState.driveCommand,
                _kickoffPhaseState.driveCommand) :
            CommandVector::Brake();

        _kickoffPhaseState.maxSpeedMps = (std::max)(
            _kickoffPhaseState.maxSpeedMps,
            std::fabs(state.GetVelocity()));
        if (_kickoffPhaseState.travelLimited &&
            (static_cast<long>(_kickoffPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            if (!FinishKickoffCharacterizationSample())
            {
                _runtime.FailActiveMode("Failed to complete kickoff characterization sample");
            }
            return CommandVector::Brake();
        }

        if (!_kickoffPhaseState.travelLimited &&
            !pulseActive &&
            (static_cast<long>(_kickoffPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            if (!FinishKickoffCharacterizationSample())
            {
                _runtime.FailActiveMode("Failed to complete kickoff characterization sample");
            }
            return CommandVector::Brake();
        }

        return command;
    }

    CommandVector ForwardCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)loopController;
        const unsigned long nowMs = millis();
        const float traveledDistanceM =
            std::fabs(_drive.GetAverageDistanceMeters() - _forwardPhaseState.startDistanceM);
        if (!_forwardPhaseState.travelLimited &&
            (_forwardPhaseState.travelLimitM > 0.0f) &&
            (traveledDistanceM >= _forwardPhaseState.travelLimitM))
        {
            _forwardPhaseState.travelLimited = true;
            _forwardPhaseState.travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
            if (_forwardPhaseState.holdStarted && !_forwardPhaseState.holdComplete)
            {
                _forwardPhaseState.holdComplete = true;
                _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
            }
        }

        CommandVector command = CommandVector::Brake();
        if (_forwardPhaseState.travelLimited)
        {
            command = CommandVector::Brake();
        }
        else if (static_cast<long>(_forwardPhaseState.kickoffDeadlineMs - nowMs) > 0)
        {
            command = CommandVector(
                LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand,
                LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand);
        }
        else if (static_cast<long>(_forwardPhaseState.holdDeadlineMs - nowMs) > 0)
        {
            if (!_forwardPhaseState.holdStarted)
            {
                _forwardPhaseState.holdStarted = true;
                _forwardPhaseState.holdStartDistanceM = _drive.GetAverageDistanceMeters();
            }
            _forwardPhaseState.holdElapsedSeconds +=
                static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f;
            command = CommandVector(
                _forwardPhaseState.forwardDriveCommand,
                _forwardPhaseState.forwardDriveCommand);
        }
        else if (!_forwardPhaseState.holdComplete)
        {
            _forwardPhaseState.holdComplete = true;
            _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        _forwardPhaseState.maxSpeedMps = (std::max)(
            _forwardPhaseState.maxSpeedMps,
            std::fabs(state.GetVelocity()));
        if (_forwardPhaseState.travelLimited &&
            (static_cast<long>(_forwardPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            if (!FinishForwardCharacterizationSample())
            {
                _runtime.FailActiveMode("Failed to complete forward characterization sample");
            }
            return CommandVector::Brake();
        }

        if (!_forwardPhaseState.travelLimited &&
            _forwardPhaseState.holdComplete &&
            (static_cast<long>(_forwardPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            if (!FinishForwardCharacterizationSample())
            {
                _runtime.FailActiveMode("Failed to complete forward characterization sample");
            }
            return CommandVector::Brake();
        }

        return command;
    }

    CommandVector TurnPhaseTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        const float errorRad = AngleErrorRad(_turnPhaseState.targetYawRad, state.GetOrientation());
        const float remainingRad = std::fabs(errorRad);
        _turnPhaseState.maxYawErrorRad = (std::max)(_turnPhaseState.maxYawErrorRad, remainingRad);
        _turnPhaseState.peakOmegaRadps = (std::max)(
            _turnPhaseState.peakOmegaRadps,
            std::fabs(state.GetRotationalVelocity()));
        bool done = false;
        const CommandVector control = _driveService.GetNextControls(done);
        if (done)
        {
            if (!WriteTurnResult(
                    _turnPhaseState.phaseName,
                    _turnPhaseState.angleRad,
                    _turnPhaseState.peakOmegaRadps,
                    _turnPhaseState.maxYawErrorRad,
                    _turnPhaseState.targetYawRad,
                    state))
            {
                _runtime.FailActiveMode("Failed to write turn diagnostic result");
                return CommandVector::Brake();
            }
            if (_turnPhaseState.nextStep == ScenarioStep::RecoverySettle)
            {
                _drive.SetPose(state.GetPositionX(), state.GetPositionY(), DirectionToYawRad(MazeMap::Up));
            }
            if (_turnPhaseState.writeSquareClosure &&
                !WriteClosureResult(
                    "square_result",
                    _squareSequenceNamePrefix,
                    _squareSequenceStartX,
                    _squareSequenceStartY,
                    _squareSequenceStartYawRad,
                    state,
                    "Failed to write square diagnostic result"))
            {
                _runtime.FailActiveMode("Failed to write square diagnostic result");
                return CommandVector::Brake();
            }
            (void)AdvanceToNextStep(
                _turnPhaseState.nextStep,
                loopController,
                "Failed to advance diagnostic turn phase");
            return CommandVector::Brake();
        }
        if (static_cast<long>(_turnPhaseState.timeoutMs - millis()) <= 0)
        {
            _runtime.FailActiveMode("Turn diagnostic phase timed out");
            return CommandVector::Brake();
        }

        return control;
    }

    CommandVector ArcPhaseTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        _arcPhaseState.traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _arcPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _arcPhaseState.distanceM - _arcPhaseState.traveledM);
        _arcPhaseState.metrics.peakSpeedMps = (std::max)(
            _arcPhaseState.metrics.peakSpeedMps,
            std::fabs(state.GetVelocity()));
        _arcPhaseState.metrics.peakOmegaRadps = (std::max)(
            _arcPhaseState.metrics.peakOmegaRadps,
            std::fabs(state.GetRotationalVelocity()));
        const float dtSeconds = static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f;
        _arcPhaseState.metrics.durationSeconds += dtSeconds;
        _arcPhaseState.metrics.omegaIntegralRad += state.GetRotationalVelocity() * dtSeconds;
        _arcPhaseState.metrics.speedIntegralMpsSeconds += std::fabs(state.GetVelocity()) * dtSeconds;
        const float planarAccelMps2 = state.GetSensorSnapshot().planarAccelMps2;
        _arcPhaseState.metrics.planarAccelIntegralMps2Seconds += planarAccelMps2 * dtSeconds;
        _arcPhaseState.metrics.peakPlanarAccelMps2 = (std::max)(
            _arcPhaseState.metrics.peakPlanarAccelMps2,
            planarAccelMps2);
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.GetVelocity()) <= Config::kSpeedToleranceMps))
        {
            AccumulateArcMetrics(_circleSequenceTotalMetrics, _arcPhaseState.metrics);
            if (!WriteArcResult(
                    _arcPhaseState.phaseName,
                    _arcPhaseState.distanceM,
                    _arcPhaseState.angleRad,
                    _arcPhaseState.traveledM,
                    _arcPhaseState.targetYawRad,
                    _arcPhaseState.metrics,
                    state))
            {
                _runtime.FailActiveMode("Failed to write arc diagnostic result");
                return CommandVector::Brake();
            }
            if (_arcPhaseState.writeCircleSummary)
            {
                if (!WriteCircleResult(
                        _circleSequenceNamePrefix,
                        _circleSequenceCruiseSpeedMps,
                        _circleSequenceStartTelemetry,
                        _circleSequenceTotalMetrics))
                {
                    _runtime.FailActiveMode("Failed to write circle diagnostic result");
                    return CommandVector::Brake();
                }
                if (!WriteClosureResult(
                        "arc_circle_result",
                        _circleSequenceNamePrefix,
                        _circleSequenceStartX,
                        _circleSequenceStartY,
                        _circleSequenceStartYawRad,
                        state,
                        "Failed to write arc circle diagnostic result"))
                {
                    _runtime.FailActiveMode("Failed to write arc circle diagnostic result");
                    return CommandVector::Brake();
                }
            }
            (void)AdvanceToNextStep(
                _arcPhaseState.nextStep,
                loopController,
                "Failed to advance diagnostic arc phase");
            return CommandVector::Brake();
        }
        if (_arcPhaseState.translationWatchdog.Stalled(
                _arcPhaseState.traveledM,
                _arcPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            _runtime.FailActiveMode("Arc diagnostic encoder progress stalled");
            return CommandVector::Brake();
        }
        if (static_cast<long>(_arcPhaseState.timeoutMs - millis()) <= 0)
        {
            _runtime.FailActiveMode("Arc diagnostic phase timed out");
            return CommandVector::Brake();
        }

        const float accelLimitedSpeedMps = (std::min)(
            _arcPhaseState.cruiseSpeedMps,
            _arcPhaseState.commandedSpeedMps +
            (_arcPhaseState.limits.accelMps2 * (static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f)));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(0.0f, remainingM, _arcPhaseState.limits.decelMps2);
        _arcPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float progress = (std::clamp)(_arcPhaseState.traveledM / _arcPhaseState.distanceM, 0.0f, 1.0f);
        const float phaseTargetYawRad = WrapAngleRad(_arcPhaseState.startYawRad + (_arcPhaseState.angleRad * progress));
        const float headingErrorRad = AngleErrorRad(phaseTargetYawRad, state.GetOrientation());
        _arcPhaseState.metrics.maxHeadingErrorRad = (std::max)(
            _arcPhaseState.metrics.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (_arcPhaseState.curvature * _arcPhaseState.commandedSpeedMps) +
            (Config::kArcHeadingKp * headingErrorRad) -
            (Config::kArcYawD * state.GetRotationalVelocity());
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -_arcPhaseState.limits.maxAngularSpeedRadps,
            _arcPhaseState.limits.maxAngularSpeedRadps);

        return _drive.PointControlVector(
            _arcPhaseState.commandedSpeedMps,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

};

