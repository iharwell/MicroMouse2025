#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
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

    MazeMap::App::Internal::LoopController::ControlVector MakeWheelOmegaRawMotorPwmCommand(
        DriveBase& drive,
        const float linearSpeedMps,
        const float angularSpeedRadps) noexcept
    {
        const MazeMap::OpenLoopDriveCommand driveCommand =
            drive.PointCommand(
                linearSpeedMps,
                angularSpeedRadps,
                MazeMap::CommandPD::StateWheelOmegaPD);
        return MazeMap::App::Internal::LoopController::ControlVector::RawMotorPwm(
            driveCommand.leftDriveCommand,
            driveCommand.rightDriveCommand);
    }
}

class DiagnosticController : public IApplicationMode
{
public:
    explicit DiagnosticController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _pendingReturnDistanceM(0.0f)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        if (!_runtime.RegisterModeFaultHandler(&DiagnosticController::HandleRuntimeFault, this, "diagnostic"))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:primary_diagnostic");
        (void)_runtime.AppendTextLogLine("Micromouse diagnostic setup");
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

        return true;
    }

    void Run() override
    {
        if (_faulted)
        {
            _loopController.EndSession();
            return;
        }

        bool ok = StartScenarioStep(ScenarioStep::StartupSettle);
        if (ok)
        {
            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &DiagnosticController::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                _phaseFn = nullptr;
                ok = Fail("Diagnostic loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                _phaseFn = nullptr;
                ok = (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
            }
        }
        else
        {
            _phaseFn = nullptr;
        }

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();

        if (ok)
        {
            (void)_runtime.AppendTextLogFormatted("Diagnostic complete, log saved to %s", GetLogFileName());
            (void)_runtime.AppendTextLogLine("Use the # event,summary lines in the log header to map phases to tunables.");
        }

        CloseLog();
        SetMissionLevelFanEnabled(false);
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    using PhaseFn = LoopController::ControlVector (DiagnosticController::*)(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

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
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    char _logFileName[64];
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    PhaseFn _phaseFn{};

    struct HoldPhaseState final
    {
        bool stationary{};
        unsigned long deadlineMs{};
        ScenarioStep nextStep{ ScenarioStep::None };
    } _holdPhaseState{};

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
        StraightPhaseMetrics metrics{};
        ScenarioStep nextStep{ ScenarioStep::None };
        bool selectReturnDistance{};
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

    struct KickoffSweepSequenceState final
    {
        std::size_t nextMagnitudeIndex{};
    } _kickoffSweepSequenceState{};

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

    struct ForwardSweepSequenceState final
    {
        std::uint16_t nextSampleIndex{};
    } _forwardSweepSequenceState{};

    struct TurnPhaseState final
    {
        const char* phaseName{};
        float angleRad{};
        float targetYawRad{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        unsigned long timeoutMs{};
        TurnPhaseMetrics metrics{};
        ScenarioStep nextStep{ ScenarioStep::None };
        bool writeSquareClosure{};
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

    struct CharacterizationRecoveryState final
    {
        char label[24]{};
        float traveledDistanceM{};
        ScenarioStep nextStep{ ScenarioStep::None };
    } _characterizationRecoveryState{};

    struct CircleSequenceState final
    {
        char namePrefix[48]{};
        float cruiseSpeedMps{};
        PoseEstimate startPose{};
        DriveTelemetry startTelemetry{};
        ArcPhaseMetrics totalMetrics{};
    } _circleSequenceState{};

    struct SquareSequenceState final
    {
        char namePrefix[24]{};
        PoseEstimate startPose{};
    } _squareSequenceState{};

    float _pendingReturnDistanceM;

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        return options;
    }

    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<DiagnosticController*>(context);
        if ((self == nullptr) || (self->_phaseFn == nullptr))
        {
            services.Fault("Diagnostic phase callback was not installed");
            return LoopController::ControlVector::Brake;
        }

        if (!state.estimatorHealthy)
        {
            services.Fault((state.faultReason != nullptr) ? state.faultReason : "Diagnostic estimator fault");
            return LoopController::ControlVector::Brake;
        }

        if (!self->IsWithinBoundary(state.estimate))
        {
            services.Fault("Diagnostic boundary exceeded");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_phaseFn)(loopEndTimeUs, state, services);
    }

    bool AdvanceToNextStep(
        const ScenarioStep nextStep,
        LoopController::TickServices& services,
        const char* const failureMessage)
    {
        if (nextStep == ScenarioStep::Complete)
        {
            services.RequestEndLoop();
            return true;
        }

        if (StartScenarioStep(nextStep))
        {
            return true;
        }

        services.Fault(failureMessage);
        return false;
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

        _holdPhaseState = HoldPhaseState{};
        _holdPhaseState.stationary = stationary;
        _holdPhaseState.deadlineMs = millis() + durationMs;
        _holdPhaseState.nextStep = nextStep;
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
        _straightPhaseState.targetHeading = _drive.GetPose().headingUnit;
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
        if (_kickoffSweepSequenceState.nextMagnitudeIndex >= MazeMap::kOpenFloorLaunchDriveMagnitudes.size())
        {
            return StartScenarioStep(ScenarioStep::ForwardSweepPrep);
        }

        const float driveCommand = MazeMap::kOpenFloorLaunchDriveMagnitudes[_kickoffSweepSequenceState.nextMagnitudeIndex];
        ++_kickoffSweepSequenceState.nextMagnitudeIndex;
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
            return Fail("Failed to format kickoff characterization result");
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
            (LegacyDiagnosticConfig::kForwardSweepStepDriveCommand * static_cast<float>(_forwardSweepSequenceState.nextSampleIndex));
        if (driveCommand > (LegacyDiagnosticConfig::kForwardSweepMaxDriveCommand + 0.0001f))
        {
            return StartScenarioStep(ScenarioStep::CharacterizationSettle);
        }

        ++_forwardSweepSequenceState.nextSampleIndex;
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
            return Fail("Failed to format forward characterization result");
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
        _turnPhaseState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        _turnPhaseState.turnProfile = BuildSharedInPlaceTurnProfile(_vehicle);
        _turnPhaseState.timeoutMs = millis() + 3000UL;
        _turnPhaseState.nextStep = nextStep;
        _turnPhaseState.writeSquareClosure = writeSquareClosure;
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
            return Fail("Diagnostic arc distance must be positive");
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
        _arcPhaseState.startYawRad = _drive.GetPose().yawRad;
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
        _circleSequenceState = CircleSequenceState{};
        snprintf(_circleSequenceState.namePrefix, sizeof(_circleSequenceState.namePrefix), "%s", (namePrefix != nullptr) ? namePrefix : "");
        _circleSequenceState.cruiseSpeedMps = cruiseSpeedMps;
        _circleSequenceState.startPose = _drive.GetPose();
        _circleSequenceState.startTelemetry = _drive.GetTelemetry();
    }

    void ResetSquareSequence(const char* const namePrefix)
    {
        _squareSequenceState = SquareSequenceState{};
        snprintf(_squareSequenceState.namePrefix, sizeof(_squareSequenceState.namePrefix), "%s", (namePrefix != nullptr) ? namePrefix : "");
        _squareSequenceState.startPose = _drive.GetPose();
    }

    bool StartCharacterizationRecovery(
        const char* const label,
        const float traveledDistanceM,
        const ScenarioStep nextStepAfterRecovery)
    {
        _characterizationRecoveryState = CharacterizationRecoveryState{};
        snprintf(
            _characterizationRecoveryState.label,
            sizeof(_characterizationRecoveryState.label),
            "%s",
            (label != nullptr) ? label : "");
        _characterizationRecoveryState.traveledDistanceM = traveledDistanceM;
        _characterizationRecoveryState.nextStep = nextStepAfterRecovery;

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
            _kickoffSweepSequenceState = KickoffSweepSequenceState{};
            return StartHoldPhase(
                "kickoff_sweep_prep",
                DiagnosticConfig::kCharacterizationSettleMs,
                true,
                ScenarioStep::KickoffSweepSample);

        case ScenarioStep::KickoffSweepSample:
            return StartNextKickoffCharacterizationSample();

        case ScenarioStep::RecoveryTurnaround:
            snprintf(phaseName, sizeof(phaseName), "%s_turnaround", _characterizationRecoveryState.label);
            return StartTurnPhase(phaseName, PI_F, ScenarioStep::RecoveryReturn);

        case ScenarioStep::RecoveryReturn:
            snprintf(phaseName, sizeof(phaseName), "%s_return", _characterizationRecoveryState.label);
            return StartStraightPhase(
                phaseName,
                _characterizationRecoveryState.traveledDistanceM,
                DiagnosticConfig::kCharacterizationRecoverySpeedMps,
                ScenarioStep::RecoveryResetHeading);

        case ScenarioStep::RecoveryResetHeading:
            snprintf(phaseName, sizeof(phaseName), "%s_reset_heading", _characterizationRecoveryState.label);
            return StartTurnPhase(phaseName, PI_F, ScenarioStep::RecoverySettle);

        case ScenarioStep::RecoverySettle:
            snprintf(phaseName, sizeof(phaseName), "%s_settle", _characterizationRecoveryState.label);
            return StartHoldPhase(
                phaseName,
                DiagnosticConfig::kCharacterizationSettleMs,
                true,
                _characterizationRecoveryState.nextStep);

        case ScenarioStep::ForwardSweepPrep:
            _forwardSweepSequenceState = ForwardSweepSequenceState{};
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

        DiagnosticLogRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
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
        return _runtime.FailActiveMode(message);
    }

    void OnRuntimeFault(const char* message) noexcept
    {
        _faulted = true;
        (void)WriteLogEvent("fault", message);
    }

    bool StartPhase(const char* name)
    {
        (void)_runtime.AppendTextLogFormatted("Diagnostic phase: %s", (name != nullptr) ? name : "unknown");
        if (WritePhaseMarker(name))
        {
            return true;
        }
        return Fail("Failed to write diagnostic phase marker");
    }

    bool IsWithinBoundary(const PoseEstimate& pose) const
    {
        return (std::fabs(pose.xMeters - _startX) <= DiagnosticConfig::kBoundaryHalfSpanM) &&
            (std::fabs(pose.yMeters - _startY) <= DiagnosticConfig::kBoundaryHalfSpanM);
    }

    bool LogSample(bool stationary, uint32_t timestampUs, float dtSeconds, const SensorSnapshot& snapshot)
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
        if (_runtime.LogUtilityDataRow(row))
        {
            ++_sampleCount;
            return true;
        }
        return Fail("Failed to write diagnostic sample");
    }

    LoopController::ControlVector HoldPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        if (!LogSample(_holdPhaseState.stationary, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        if (static_cast<long>(_holdPhaseState.deadlineMs - millis()) <= 0)
        {
            (void)AdvanceToNextStep(
                _holdPhaseState.nextStep,
                services,
                "Failed to advance diagnostic hold phase");
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector StraightPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const MotionLimits limits = DiagnosticLimits(_straightPhaseState.cruiseSpeedMps);
        _straightPhaseState.traveledM =
            std::fabs(_drive.GetAverageDistanceMeters() - _straightPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _straightPhaseState.distanceM - _straightPhaseState.traveledM);
        _straightPhaseState.metrics.peakSpeedMps = (std::max)(
            _straightPhaseState.metrics.peakSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::Brake;
            }

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
                    _straightPhaseState.metrics))
            {
                services.Fault("Failed to write straight diagnostic result");
                return LoopController::ControlVector::Brake;
            }
            (void)AdvanceToNextStep(
                _straightPhaseState.nextStep,
                services,
                "Failed to advance diagnostic straight phase");
            return LoopController::ControlVector::Brake;
        }
        if (_straightPhaseState.translationWatchdog.Stalled(
                _straightPhaseState.traveledM,
                _straightPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            services.Fault("Straight diagnostic encoder progress stalled");
            return LoopController::ControlVector::Brake;
        }
        if (static_cast<long>(_straightPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Straight diagnostic phase timed out");
            return LoopController::ControlVector::Brake;
        }

        const float accelLimitedSpeedMps = (std::min)(
            limits.maxSpeedMps,
            _straightPhaseState.commandedSpeedMps + (limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
        _straightPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float headingErrorRad = HeadingErrorRad(_straightPhaseState.targetHeading, state.estimate.headingUnit);
        _straightPhaseState.metrics.maxHeadingErrorRad = (std::max)(
            _straightPhaseState.metrics.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * state.estimate.angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -limits.maxAngularSpeedRadps,
            limits.maxAngularSpeedRadps);

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        return MakeWheelOmegaRawMotorPwmCommand(
            _drive,
            _straightPhaseState.commandedSpeedMps,
            angularCommandRadps);
    }

    LoopController::ControlVector KickoffCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
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
        const LoopController::ControlVector command =
            _kickoffPhaseState.travelLimited ?
            LoopController::ControlVector::Brake :
            pulseActive ?
            LoopController::ControlVector::RawMotorPwm(
                _kickoffPhaseState.driveCommand,
                _kickoffPhaseState.driveCommand) :
            LoopController::ControlVector::Brake;

        _kickoffPhaseState.maxSpeedMps = (std::max)(
            _kickoffPhaseState.maxSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        if (_kickoffPhaseState.travelLimited &&
            (static_cast<long>(_kickoffPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!FinishKickoffCharacterizationSample())
            {
                services.Fault("Failed to complete kickoff characterization sample");
            }
            return LoopController::ControlVector::Brake;
        }

        if (!_kickoffPhaseState.travelLimited &&
            !pulseActive &&
            (static_cast<long>(_kickoffPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!FinishKickoffCharacterizationSample())
            {
                services.Fault("Failed to complete kickoff characterization sample");
            }
            return LoopController::ControlVector::Brake;
        }

        return command;
    }

    LoopController::ControlVector ForwardCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
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

        LoopController::ControlVector command = LoopController::ControlVector::Brake;
        if (_forwardPhaseState.travelLimited)
        {
            command = LoopController::ControlVector::Brake;
        }
        else if (static_cast<long>(_forwardPhaseState.kickoffDeadlineMs - nowMs) > 0)
        {
            command = LoopController::ControlVector::RawMotorPwm(
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
            _forwardPhaseState.holdElapsedSeconds += state.dtSeconds;
            command = LoopController::ControlVector::RawMotorPwm(
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
            std::fabs(state.estimate.linearSpeedMps));
        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        if (_forwardPhaseState.travelLimited &&
            (static_cast<long>(_forwardPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!FinishForwardCharacterizationSample())
            {
                services.Fault("Failed to complete forward characterization sample");
            }
            return LoopController::ControlVector::Brake;
        }

        if (!_forwardPhaseState.travelLimited &&
            _forwardPhaseState.holdComplete &&
            (static_cast<long>(_forwardPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!FinishForwardCharacterizationSample())
            {
                services.Fault("Failed to complete forward characterization sample");
            }
            return LoopController::ControlVector::Brake;
        }

        return command;
    }

    LoopController::ControlVector TurnPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const float errorRad = AngleErrorRad(_turnPhaseState.targetYawRad, state.estimate.yawRad);
        const float remainingRad = std::fabs(errorRad);
        _turnPhaseState.metrics.maxYawErrorRad = (std::max)(_turnPhaseState.metrics.maxYawErrorRad, remainingRad);
        _turnPhaseState.metrics.peakOmegaRadps = (std::max)(
            _turnPhaseState.metrics.peakOmegaRadps,
            std::fabs(state.estimate.angularSpeedRadps));
        if (MazeMap::IsInPlaceTurnComplete(errorRad, state.estimate.angularSpeedRadps, _turnPhaseState.turnProfile))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::Brake;
            }

            if (!WriteTurnResult(
                    _turnPhaseState.phaseName,
                    _turnPhaseState.angleRad,
                    _turnPhaseState.metrics,
                    _turnPhaseState.targetYawRad))
            {
                services.Fault("Failed to write turn diagnostic result");
                return LoopController::ControlVector::Brake;
            }
            if (_turnPhaseState.nextStep == ScenarioStep::RecoverySettle)
            {
                const PoseEstimate& pose = _drive.GetPose();
                _drive.SetPose(pose.xMeters, pose.yMeters, DirectionToYawRad(MazeMap::Up));
            }
            if (_turnPhaseState.writeSquareClosure &&
                !WriteClosureResult(
                    "square_result",
                    _squareSequenceState.namePrefix,
                    _squareSequenceState.startPose,
                    "Failed to write square diagnostic result"))
            {
                services.Fault("Failed to write square diagnostic result");
                return LoopController::ControlVector::Brake;
            }
            (void)AdvanceToNextStep(
                _turnPhaseState.nextStep,
                services,
                "Failed to advance diagnostic turn phase");
            return LoopController::ControlVector::Brake;
        }
        if (static_cast<long>(_turnPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Turn diagnostic phase timed out");
            return LoopController::ControlVector::Brake;
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                errorRad,
                state.estimate.angularSpeedRadps,
                _turnPhaseState.turnProfile,
                angularCommandRadps))
        {
            services.Fault("Turn diagnostic phase profile became invalid");
            return LoopController::ControlVector::Brake;
        }

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        return MakeWheelOmegaRawMotorPwmCommand(_drive, 0.0f, angularCommandRadps);
    }

    LoopController::ControlVector ArcPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        _arcPhaseState.traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _arcPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _arcPhaseState.distanceM - _arcPhaseState.traveledM);
        _arcPhaseState.metrics.peakSpeedMps = (std::max)(
            _arcPhaseState.metrics.peakSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        _arcPhaseState.metrics.peakOmegaRadps = (std::max)(
            _arcPhaseState.metrics.peakOmegaRadps,
            std::fabs(state.estimate.angularSpeedRadps));
        _arcPhaseState.metrics.durationSeconds += state.dtSeconds;
        _arcPhaseState.metrics.omegaIntegralRad += state.estimate.angularSpeedRadps * state.dtSeconds;
        _arcPhaseState.metrics.speedIntegralMpsSeconds += std::fabs(state.estimate.linearSpeedMps) * state.dtSeconds;
        const float planarAccelMps2 = state.sensors.planarAccelMps2;
        _arcPhaseState.metrics.planarAccelIntegralMps2Seconds += planarAccelMps2 * state.dtSeconds;
        _arcPhaseState.metrics.peakPlanarAccelMps2 = (std::max)(
            _arcPhaseState.metrics.peakPlanarAccelMps2,
            planarAccelMps2);
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::Brake;
            }

            AccumulateArcMetrics(_circleSequenceState.totalMetrics, _arcPhaseState.metrics);
            if (!WriteArcResult(
                    _arcPhaseState.phaseName,
                    _arcPhaseState.distanceM,
                    _arcPhaseState.angleRad,
                    _arcPhaseState.traveledM,
                    _arcPhaseState.targetYawRad,
                    _arcPhaseState.metrics))
            {
                services.Fault("Failed to write arc diagnostic result");
                return LoopController::ControlVector::Brake;
            }
            if (_arcPhaseState.writeCircleSummary)
            {
                if (!WriteCircleResult(
                        _circleSequenceState.namePrefix,
                        _circleSequenceState.cruiseSpeedMps,
                        _circleSequenceState.startTelemetry,
                        _circleSequenceState.totalMetrics))
                {
                    services.Fault("Failed to write circle diagnostic result");
                    return LoopController::ControlVector::Brake;
                }
                if (!WriteClosureResult(
                        "arc_circle_result",
                        _circleSequenceState.namePrefix,
                        _circleSequenceState.startPose,
                        "Failed to write arc circle diagnostic result"))
                {
                    services.Fault("Failed to write arc circle diagnostic result");
                    return LoopController::ControlVector::Brake;
                }
            }
            (void)AdvanceToNextStep(
                _arcPhaseState.nextStep,
                services,
                "Failed to advance diagnostic arc phase");
            return LoopController::ControlVector::Brake;
        }
        if (_arcPhaseState.translationWatchdog.Stalled(
                _arcPhaseState.traveledM,
                _arcPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            services.Fault("Arc diagnostic encoder progress stalled");
            return LoopController::ControlVector::Brake;
        }
        if (static_cast<long>(_arcPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Arc diagnostic phase timed out");
            return LoopController::ControlVector::Brake;
        }

        const float accelLimitedSpeedMps = (std::min)(
            _arcPhaseState.cruiseSpeedMps,
            _arcPhaseState.commandedSpeedMps + (_arcPhaseState.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(0.0f, remainingM, _arcPhaseState.limits.decelMps2);
        _arcPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float progress = (std::clamp)(_arcPhaseState.traveledM / _arcPhaseState.distanceM, 0.0f, 1.0f);
        const float phaseTargetYawRad = WrapAngleRad(_arcPhaseState.startYawRad + (_arcPhaseState.angleRad * progress));
        const float headingErrorRad = AngleErrorRad(phaseTargetYawRad, state.estimate.yawRad);
        _arcPhaseState.metrics.maxHeadingErrorRad = (std::max)(
            _arcPhaseState.metrics.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (_arcPhaseState.curvature * _arcPhaseState.commandedSpeedMps) +
            (Config::kArcHeadingKp * headingErrorRad) -
            (Config::kArcYawD * state.estimate.angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -_arcPhaseState.limits.maxAngularSpeedRadps,
            _arcPhaseState.limits.maxAngularSpeedRadps);

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.sensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::Brake;
        }

        return MakeWheelOmegaRawMotorPwmCommand(
            _drive,
            _arcPhaseState.commandedSpeedMps,
            angularCommandRadps);
    }

};

