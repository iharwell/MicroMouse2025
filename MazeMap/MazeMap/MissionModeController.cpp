#include "pch.h"
#include "MissionModeController.h"

#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "AuxMeasurementModeSupport.h"
#include "ManeuverExecutor.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "MazeMapSharedRuntime.h"

using MazeMap::App::Internal::SharedRobotRuntime;
using namespace MazeMap::App::Internal::AuxMeasurementModeSupport;

namespace
{
    constexpr MazeMap::CommandPD kMissionDriveBaseTrackingCommandPd =
        MazeMap::CommandPD::StateWheelOmegaPD |
        MazeMap::CommandPD::IMUYaw;
}

class MazeMap::App::Internal::MissionModeController::Implementation final
{
public:
    explicit Implementation(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _speedVehicle(runtime.SpeedVehicle())
        , _mappingVehicle(runtime.SearchVehicle())
        , _maze(runtime.Maze())
        , _searchPathFinder(runtime.SearchPathFinder())
        , _speedPathFinder(runtime.SpeedPathFinder())
        , _wallBeliefMap(runtime.WallBeliefMap())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _currentCell(0, 0)
        , _currentDirection(MazeMap::Up)
        , _currentDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _goalPauseComplete(false)
        , _missionComplete(false)
        , _faulted(false)
        , _telemetryLoggingEnabled(false)
        , _missionTextLoggingEnabled(false)
        , _missionMazeSnapshotWritten(false)
        , _frontWallCharacterization()
        , _frontWallCharacterizationAvailable(false)
        , _lastWallTouchStandoffEstimateM(0.0f)
        , _hasWallTouchStandoffEstimate(false)
        , _activeModeFaultSource("mission")
        , _telemetryPhaseId(0UL)
        , _telemetrySampleCount(0UL)
    {
        _telemetryLogFileName[0] = '\0';
    }

    Implementation(const Implementation&) = delete;
    Implementation& operator=(const Implementation&) = delete;
    Implementation(Implementation&&) = delete;
    Implementation& operator=(Implementation&&) = delete;

    bool BeginMissionRunMode()
    {
        ResetForMode(true, "mission");
        if (!Initialize("mode:mission", "Micromouse mission setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        return true;
    }

    void RunMissionRunMode()
    {
        if (_missionComplete || _faulted)
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Exploration start"))
        {
            return;
        }
        _drive.SetWheelControlProfile(BuildMappingWheelControlProfile());
        if (!ExploreFullMaze())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return;
        }
        if (!ReturnToStart())
        {
            return;
        }
        _drive.UseNominalWheelControlProfile();

        _maze.PreCalculate();

        if (!EmitMissionControllerLineOrFail("Speed run 1 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        if (!HandleInterRunServiceCycle())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Speed run 2 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        (void)WriteMissionMazeSnapshot("mission_complete");

        SetRacingFanEnabled(false);
        _drive.Brake();
        _missionComplete = true;
        (void)EmitMissionControllerLine("Mission complete");
        CloseMissionTextLog();
    }

    bool BeginCorridorRepeatabilityMode()
    {
        ResetForMode(false, "corridor_repeatability");
        if (!Initialize("mode:corridor_repeatability", "Corridor repeatability setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_sensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.mmlog",
                "corridor_repeatability.mmlog"))
        {
            return Fail("Unable to choose corridor repeatability log file");
        }
        if (!BeginTelemetryLog(fileName, "corridor_repeatability"))
        {
            return Fail("Unable to open corridor repeatability log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("corridor_repeatability:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        if (!LogCorridorRepeatabilityMetadataImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                }))
        {
            return false;
        }
        AppendStartupTrace("corridor_repeatability:metadata_written");
        return true;
    }

    void RunCorridorRepeatabilityMode()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("corridor_repeatability:run_entered");
        const bool ok = RunCorridorRepeatabilityPasses();
        ShutdownTelemetryMode(false);
        if (ok)
        {
            AppendStartupTrace("corridor_repeatability:complete");
            (void)EmitMissionControllerLine("Corridor repeatability sweep complete");
        }
        CloseMissionTextLog();
    }

    bool BeginPositionAccuracyAuditMode()
    {
        ResetForMode(false, "position_accuracy_audit");
        if (!Initialize("mode:position_accuracy_audit", "Position accuracy audit setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_sensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.mmlog",
                "position_accuracy_audit.mmlog"))
        {
            return Fail("Unable to choose position accuracy audit log file");
        }
        if (!BeginTelemetryLog(fileName, "position_accuracy_audit"))
        {
            return Fail("Unable to open position accuracy audit log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("position_accuracy_audit:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        const PositionAuditFixtureGeometry positionAuditGeometry = BuildPositionAuditFixtureGeometry();
        if (!LogPositionAccuracyAuditMetadataImpl(
                positionAuditGeometry,
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                }))
        {
            return false;
        }
        AppendStartupTrace("position_accuracy_audit:metadata_written");
        return true;
    }

    void RunPositionAccuracyAuditMode()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("position_accuracy_audit:run_entered");
        const bool ok = RunPositionAccuracyAuditPasses();
        ShutdownTelemetryMode(true);
        if (ok)
        {
            AppendStartupTrace("position_accuracy_audit:complete");
            (void)EmitMissionControllerLine("Position accuracy audit complete");
        }
        CloseMissionTextLog();
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    static constexpr const char* kMissionControllerTextLogSource = "mission_controller";
    static constexpr const char* kMissionTraceTextLogSource = "mission_trace";

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _speedVehicle;
    MazeMap::Vehicle& _mappingVehicle;
    MazeMap::Maze& _maze;
    MazeMap::FloodFillPathFinder& _searchPathFinder;
    MazeMap::ManeuverPathFinder& _speedPathFinder;
    MazeMap::WallBeliefMap& _wallBeliefMap;
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    MazeMap::CellCoordinates _currentCell;
    MazeMap::Direction _currentDirection;
    MazeMap::DirectionalLocation _currentDirectionalLocation;
    bool _goalPauseComplete;
    bool _missionComplete;
    bool _faulted;
    bool _telemetryLoggingEnabled;
    bool _missionTextLoggingEnabled;
    bool _missionMazeSnapshotWritten;
    MazeMap::FrontWallCharacterizationStorage _frontWallCharacterization;
    bool _frontWallCharacterizationAvailable;
    float _lastWallTouchStandoffEstimateM;
    bool _hasWallTouchStandoffEstimate;
    const char* _activeModeFaultSource;
    char _telemetryLogFileName[64];
    unsigned long _telemetryPhaseId;
    unsigned long _telemetrySampleCount;
    DiagnosticLogRow _telemetryLogRow{};
    using ActiveLoopTickFn = LoopController::ControlVector (Implementation::*)(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    struct WallTouchPoseResetTarget;

    struct SharedManeuverExecutorLoopState final
    {
        void* nextState{};
        ActiveLoopTickFn nextTickFn{};
    };

    struct InterRunServicePauseLoopState final
    {
    };

    struct QueuedManeuverLoopState final
    {
        MazeMap::ManeuverQueue* queue{};
        MotionLimits limits{};
        bool snapToExpectedLocation{};
        const char* completionHoldPhaseName{};
        SharedManeuverExecutorLoopState completionHold{};
    };

    struct StartupStationaryHoldLoopState final
    {
        unsigned long stationaryStartMs{};
        unsigned long lastResetTraceMs{};
        bool stationaryWindowActive{};
        DriveTelemetry stationaryStartTelemetry{};
    };

    struct ObservationCaptureLoopState final
    {
        MazeMap::CellCoordinates observedCell{};
        MazeMap::Direction observedDirection{ MazeMap::None };
        SensorSnapshot* outputSnapshot{};
        SensorSnapshot samples[Config::kSearchRollingObservationSampleCount]{};
        float frontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float frontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        std::uint8_t nextSampleIndex{};
    };

    struct FrontCalibrationSweepLoopState final
    {
        FrontCalibrationSpinSampleSet<Config::kStartupWallCalibrationFrontSpinMaxSamples> sweepSamples{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        MotionLimits limits{};
        float targetSweepAngleRad{};
        float captureStepRad{};
        float accumulatedSweepAngleRad{};
        float lastStoredSweepAngleRad{};
        float previousYawRad{};
        unsigned long expectedCompletionDeadlineMs{};
        bool durationLogged{};
        bool* storedBands{};
        SharedManeuverExecutorLoopState settleHold{};
    };

    struct SearchStraightLoopState final
    {
        MazeMap::Direction direction{ MazeMap::None };
        MazeMap::CellCoordinates startCell{};
        MazeMap::CellCoordinates destination{};
        std::uint16_t cellCount{};
        float entrySpeedMps{};
        float cruiseSpeedMps{};
        float exitSpeedMps{};
        bool observeWhileRolling{};
        bool stoppedForReplan{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        float targetXMeters{};
        float targetYMeters{};
        float distanceToTargetM{};
        float commandedSpeedMps{};
        std::uint16_t rollingObservationCount{};
        MazeMap::CellCoordinates nextRollingObservationCell{};
        float rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount]{};
        SensorSnapshot rollingObservationSamples[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationFrontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationFrontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationSideResetTriggerTravelM{};
        std::uint8_t rollingObservationNextSampleIndex{};
        bool rollingObservationSideResetPending{};
        bool rollingObservationPlanInitialized{};
        bool stallLogged{};
        bool durationLogged{};
        float previousCorridorErrorM{};
        float filteredCorridorErrorRateMps{};
        bool previousCorridorErrorValid{};
        MazeMap::CellCoordinates replanObservedCell{};
        float replanProjectedTravelM{};
        std::uint16_t replanFrontVoteCount{};
        unsigned long expectedCompletionDeadlineMs{};
        EncoderProgressWatchdog translationWatchdog{};
        SharedManeuverExecutorLoopState completionSettle{};
    };

    struct WallTouchLoopState final
    {
        float targetYawRad{};
        float minLatchTravelM{};
        float maxApproachTravelM{};
        bool allowPassThroughNoWall{};
        const WallTouchPoseResetTarget* poseResetTarget{};
        float* seatedYawErrorRad{};
        MazeMap::App::Internal::Runtime::WallTouchExecutionResult result{};
        DriveTelemetry lastMotionTelemetry{};
        unsigned long touchStartMs{};
        unsigned long stateStartMs{};
        unsigned long contactCandidateStartMs{};
        unsigned long contactConfirmedStartMs{};
        unsigned long frontSignalMissingStartMs{};
        unsigned long lastMotionMs{};
        float startDistanceM{};
        float approachDriveCommand{};
        float ditherTurnFraction{};
        float previousCycleFrontSkewMagnitudeM{};
        float currentCycleStartYawRad{};
        float currentCycleMaxFrontSkewMagnitudeM{};
        float currentCycleMaxResidualYawRateRadps{};
        bool currentCycleFrontSignalValid{};
        bool haveSquareSample{};
        unsigned long lastHalfCycleIndex{};
        float lastSquareYawRad{};
        float lastSquareFrontSkewM{};
        float lastSquareYawRateRadps{};
        bool lastSquareFrontSignalValid{};
        std::uint8_t completedHalfCycles{};
        std::uint8_t consecutiveGoodFullCycles{};
        bool contactCandidateActive{};
        bool seatedResetApplied{};
        bool passThroughCompletionPending{};
        MazeMap::App::Internal::Runtime::WallTouchState runtimeState{
            MazeMap::App::Internal::Runtime::WallTouchState::EntryConditioning
        };
        SharedManeuverExecutorLoopState passThroughSettle{};
    };

    void* _activeLoopState{};
    ActiveLoopTickFn _activeLoopTickFn{};

    bool BeginTelemetryLog(const char* fileName, const char* modeName)
    {
        const char* resolvedFileName = (fileName != nullptr && fileName[0] != '\0') ? fileName : "telemetry.mmlog";
        const char* resolvedModeName = (modeName != nullptr && modeName[0] != '\0') ? modeName : "telemetry";
        _telemetryLogFileName[0] = '\0';
        snprintf(_telemetryLogFileName, sizeof(_telemetryLogFileName), "%s", resolvedFileName);
        return MazeMap::App::Internal::Runtime::BeginDiagnosticUtilityTelemetryLog(
            _runtime,
            _sensors,
            _telemetryLogRow,
            _telemetryLogFileName,
            resolvedModeName,
            _telemetryPhaseId,
            _telemetrySampleCount);
    }

    bool WriteTelemetryEvent(const char* type, const char* message)
    {
        return !_telemetryLoggingEnabled || _runtime.WriteTextLogEntry(micros(), type, message);
    }

    void ServiceTelemetryLog()
    {
        (void)_runtime.ServiceUtilityDataLog();
    }

    void FlushTelemetryLog()
    {
        _runtime.FlushTextLog();
    }

    void CloseTelemetryLog()
    {
        (void)_runtime.CloseUtilityDataLog();
    }

    static void SetRacingFanEnabled(bool enabled)
    {
        if (enabled)
        {
            RampFanDutyCycle(Config::kRacingFanDutyCycle);
            return;
        }

        WriteFanDutyCycle(0.0f);
    }

    MotionLimits SearchLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kSearchMaxSpeedMps;
        limits.accelMps2 = Config::kSearchAccelMps2;
        limits.decelMps2 = Config::kSearchDecelMps2;
        limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration();
        limits.angleToleranceRad = Config::kMappingAngleToleranceRad;
        limits.angularSpeedToleranceRadps = Config::kMappingAngularSpeedToleranceRadps;
        return limits;
    }

    MotionLimits FinalLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = _speedVehicle.GetMaxSpeed() * Config::kSpeedRunScale;
        limits.accelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.decelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity() * Config::kSpeedRunScale;
        limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration() * Config::kSpeedRunScale;
        return limits;
    }

    static MotionLimits StartupWallCalibrationLimits()
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kStartupWallCalibrationSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationDecelMps2;
        limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kStartupWallCalibrationTurnAccelRadps2;
        return limits;
    }

    static MotionLimits StartupWallCalibrationCenteringLimits()
    {
        MotionLimits limits = StartupWallCalibrationLimits();
        limits.maxSpeedMps = Config::kStartupWallCalibrationCenteringSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationCenteringAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationCenteringDecelMps2;
        return limits;
    }

    static MotionLimits StartupWallCalibrationFrontSweepLimits()
    {
        MotionLimits limits = StartupWallCalibrationLimits();
        limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationFrontSweepMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kStartupWallCalibrationFrontSweepAccelRadps2;
        return limits;
    }

    float SearchUnmappedCruiseSpeedMps() const
    {
        const float frontSensorForwardOffsetM = (std::min)(
            _speedVehicle.FrontLeft.GetPosition().x(),
            _speedVehicle.FrontRight.GetPosition().x());
        float frontWallOnThresholdM = Config::kFrontWallOnThresholdM;
        float frontWallOffThresholdM = Config::kFrontWallOffThresholdM;
        gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            frontWallOnThresholdM,
            frontWallOffThresholdM);

        const float safeCruiseSpeedMps = (std::min)(
            SearchLimits().maxSpeedMps,
            MazeMap::ComputeSafeUnmappedCruiseSpeed(
                SearchLimits().decelMps2,
                frontWallOnThresholdM,
                frontSensorForwardOffsetM,
                Config::kWallTouchContactStandoffM,
                Config::kDistanceToleranceM));
        return MazeMap::ApplyMinimumCruiseSpeedFloor(
            safeCruiseSpeedMps,
            Config::kMinimumAllowedCruiseSpeedMps,
            SearchLimits().maxSpeedMps);
    }

    static MazeMap::WallBeliefConfig BuildWallBeliefConfig()
    {
        MazeMap::WallBeliefConfig config{};
        config.hitLogOdds = Config::kWallBeliefHitLogOdds;
        config.missLogOdds = Config::kWallBeliefMissLogOdds;
        config.contradictoryMissLogOdds = Config::kWallBeliefContradictoryMissLogOdds;
        config.setThreshold = Config::kWallBeliefSetThreshold;
        config.clearThreshold = Config::kWallBeliefClearThreshold;
        config.saturationMagnitude = Config::kWallBeliefSaturationMagnitude;
        return config;
    }

    void SeedWallBeliefsFromKnownMaze()
    {
        _wallBeliefMap.Reset();
        const MazeMap::WallBeliefConfig beliefConfig = BuildWallBeliefConfig();
        constexpr MazeMap::Direction kDirections[] = {
            MazeMap::Up,
            MazeMap::Down,
            MazeMap::Left,
            MazeMap::Right
        };

        for (uint8_t x = 0U; x < 16U; ++x)
        {
            for (uint8_t y = 0U; y < 16U; ++y)
            {
                const MazeMap::CellCoordinates cell(x, y);
                const MazeMap::Cell& knownCell = _maze[cell];
                for (MazeMap::Direction direction : kDirections)
                {
                    const MazeMap::WallState hardState = knownCell.GetWall(direction);
                    if (hardState != MazeMap::Unknown)
                    {
                        _wallBeliefMap.SeedKnownState(cell, direction, hardState, beliefConfig);
                    }
                }
            }
        }
    }

    void SnapToStartPose()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SetStartPoint(_currentDirectionalLocation);
    }

    void PrimeKnownMissionStartCell()
    {
        _maze.SetWall(0, 0, MazeMap::Up, MazeMap::NoWall);
        _maze.SetWall(0, 0, MazeMap::Down, MazeMap::Wall);
        _maze.SetWall(0, 0, MazeMap::Left, MazeMap::Wall);
        _maze.SetWall(0, 0, MazeMap::Right, MazeMap::Wall);
        SeedWallBeliefsFromKnownMaze();
    }

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<Implementation*>(context)->OnRuntimeFault(reason);
    }

    void ResetForMode(bool enableMissionTextLogging, const char* activeModeFaultSource)
    {
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = enableMissionTextLogging;
        _missionMazeSnapshotWritten = false;
        _goalPauseComplete = false;
        _missionComplete = false;
        _faulted = false;
        _activeModeFaultSource =
            (activeModeFaultSource != nullptr && activeModeFaultSource[0] != '\0') ? activeModeFaultSource : "mission";
        _runtime.FlushTextLog();
        _hasWallTouchStandoffEstimate = false;
    }

    void ShutdownTelemetryMode(bool disableFan)
    {
        if (disableFan)
        {
            SetRacingFanEnabled(false);
        }

        _drive.Brake();
        FlushTelemetryLog();
        _telemetryLoggingEnabled = false;
        CloseTelemetryLog();
    }

    bool OpenMissionTextLog()
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        return _runtime.EnsureTextLogOpen();
    }

    void FlushMissionTextLog()
    {
        if (_missionTextLoggingEnabled)
        {
            _runtime.FlushTextLog();
        }
    }

    void CloseMissionTextLog()
    {
        _runtime.FlushTextLog();
    }

    bool WriteMissionTextLineIfEnabled(const char* message)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (message == nullptr)
        {
            return false;
        }

        return _runtime.WriteTextLogEntry(
            kMissionTraceTextLogSource,
            micros(),
            "trace",
            message);
    }

    void DisableMissionTextLogging(const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return;
        }

        if (traceLabel != nullptr && traceLabel[0] != '\0')
        {
            AppendStartupTrace(traceLabel);
            char message[192] = {};
            const int written = snprintf(
                message,
                sizeof(message),
                "Mission text logging disabled: %s",
                traceLabel);
            if (written > 0 && written < static_cast<int>(sizeof(message)))
            {
                (void)_runtime.WriteTextLogEntry(
                    kMissionControllerTextLogSource,
                    micros(),
                    "status",
                    message);
            }
        }

        CloseMissionTextLog();
        _missionTextLoggingEnabled = false;
    }

    bool WriteMissionTraceLineBestEffort(const char* message, const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (WriteMissionTextLineIfEnabled(message))
        {
            return true;
        }

        DisableMissionTextLogging(traceLabel);
        return true;
    }

    static const char* FrontObservationSourceName(const SensorSnapshot& snapshot)
    {
        if (snapshot.frontWallUsesFallbackDetection)
        {
            return "front_pair_fallback";
        }
        if (snapshot.frontLeftWall && snapshot.frontRightWall)
        {
            return "front_left+front_right";
        }
        if (snapshot.frontLeftWall)
        {
            return "front_left";
        }
        if (snapshot.frontRightWall)
        {
            return "front_right";
        }
        return "front_left+front_right";
    }

    static const char* FrontObservationModeName(const SensorSnapshot& snapshot)
    {
        if (snapshot.frontWallUsesFallbackDetection)
        {
            return "fallback";
        }
        if (snapshot.frontWallUsesCharacterizationDetection)
        {
            return "characterized";
        }
        return "differential";
    }

    void LoadPersistedFrontWallCharacterization()
    {
        _frontWallCharacterization = MazeMap::FrontWallCharacterizationStorage{};
        _frontWallCharacterizationAvailable =
            TryReadPersistedFrontWallCharacterization(_frontWallCharacterization);

        if (_frontWallCharacterizationAvailable)
        {
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "initialize:front_characterization_loaded,samples=%u,terminal_distance_m=%.4f,reverse_speed_mps=%.3f",
                static_cast<unsigned>(_frontWallCharacterization.sampleCount),
                _frontWallCharacterization.terminalDistanceM,
                _frontWallCharacterization.commandedReverseSpeedMps);
            AppendStartupTrace(traceLine);
            (void)EmitMissionControllerLine("Loaded persisted front wall characterization.");
        }
        else
        {
            AppendStartupTrace("initialize:front_characterization_unavailable");
        }
    }

    static bool DoesFrontWallCharacterizationMatchIndicateWall(
        const MazeMap::FrontWallCharacterizationMatch& match)
    {
        return
            match.valid &&
            (match.sampleCount >= Config::kFrontWallCharacterizationMinMatchSamples) &&
            std::isfinite(match.scale) &&
            std::isfinite(match.normalizedCorrelation) &&
            std::isfinite(match.relativeResidual) &&
            (match.scale >= Config::kFrontWallCharacterizationMinScale) &&
            (match.normalizedCorrelation >= Config::kFrontWallCharacterizationMinCorrelation) &&
            (match.relativeResidual <= Config::kFrontWallCharacterizationMaxRelativeResidual);
    }

    bool TryApplyFrontWallCharacterizationToObservation(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        const char* observationOrigin,
        const SensorSnapshot* samples,
        const float* frontLeftCandidateDistanceM,
        const float* frontRightCandidateDistanceM,
        uint8_t sampleCount,
        SensorSnapshot& combinedSnapshot)
    {
        if (!_frontWallCharacterizationAvailable ||
            samples == nullptr ||
            frontLeftCandidateDistanceM == nullptr ||
            frontRightCandidateDistanceM == nullptr ||
            sampleCount == 0U)
        {
            return false;
        }

        float frontLeftBaseline = 0.0f;
        float frontRightBaseline = 0.0f;
        const bool haveFrontLeftBaseline =
            gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
                WallSensorId::FrontLeft,
                frontLeftBaseline);
        const bool haveFrontRightBaseline =
            gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
                WallSensorId::FrontRight,
                frontRightBaseline);

        float frontLeftMeasured[Config::kSearchRollingObservationSampleCount] = {};
        float frontRightMeasured[Config::kSearchRollingObservationSampleCount] = {};
        for (uint8_t index = 0U; index < sampleCount; ++index)
        {
            frontLeftMeasured[index] = samples[index].frontLeftDifferentialLight;
            frontRightMeasured[index] = samples[index].frontRightDifferentialLight;
        }

        MazeMap::FrontWallCharacterizationMatch frontLeftMatch{};
        MazeMap::FrontWallCharacterizationMatch frontRightMatch{};
        const bool haveFrontLeftMatch =
            haveFrontLeftBaseline &&
            MazeMap::TryMatchFrontWallCharacterizationChannel(
                _frontWallCharacterization,
                false,
                frontLeftMeasured,
                frontLeftCandidateDistanceM,
                sampleCount,
                frontLeftBaseline,
                frontLeftMatch);
        const bool haveFrontRightMatch =
            haveFrontRightBaseline &&
            MazeMap::TryMatchFrontWallCharacterizationChannel(
                _frontWallCharacterization,
                true,
                frontRightMeasured,
                frontRightCandidateDistanceM,
                sampleCount,
                frontRightBaseline,
                frontRightMatch);

        if (!(haveFrontLeftMatch || haveFrontRightMatch))
        {
            return false;
        }

        const bool frontLeftDetected =
            haveFrontLeftMatch &&
            DoesFrontWallCharacterizationMatchIndicateWall(frontLeftMatch);
        const bool frontRightDetected =
            haveFrontRightMatch &&
            DoesFrontWallCharacterizationMatchIndicateWall(frontRightMatch);
        combinedSnapshot.frontLeftWall = frontLeftDetected;
        combinedSnapshot.frontRightWall = frontRightDetected;
        combinedSnapshot.frontWall = frontLeftDetected || frontRightDetected;
        combinedSnapshot.frontWallObservationValid = true;
        combinedSnapshot.frontWallUsesFallbackDetection = false;
        combinedSnapshot.frontWallUsesCharacterizationDetection = true;

        AppendMissionTraceFormatted(
            "mission_front_curve_fit,cell=(%d,%d),abs=%s,origin=%s,left_fit=%u,left_n=%u,left_scale=%.3f,left_corr=%.3f,left_rr=%.3f,left_hit=%u,right_fit=%u,right_n=%u,right_scale=%.3f,right_corr=%.3f,right_rr=%.3f,right_hit=%u",
            observedCell.GetX(),
            observedCell.GetY(),
            DirectionName(observedDirection),
            (observationOrigin != nullptr) ? observationOrigin : "unknown",
            haveFrontLeftMatch ? 1U : 0U,
            static_cast<unsigned>(frontLeftMatch.sampleCount),
            frontLeftMatch.scale,
            frontLeftMatch.normalizedCorrelation,
            frontLeftMatch.relativeResidual,
            frontLeftDetected ? 1U : 0U,
            haveFrontRightMatch ? 1U : 0U,
            static_cast<unsigned>(frontRightMatch.sampleCount),
            frontRightMatch.scale,
            frontRightMatch.normalizedCorrelation,
            frontRightMatch.relativeResidual,
            frontRightDetected ? 1U : 0U);
        return true;
    }

    LoopController::ControlVector ObservationCaptureLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& capture = *static_cast<ObservationCaptureLoopState*>(rawState);

        if (capture.nextSampleIndex < Config::kSearchRollingObservationSampleCount)
        {
            capture.samples[capture.nextSampleIndex] = state.sensors;
            float frontLeftDistanceM = NAN;
            float frontRightDistanceM = NAN;
            (void)TryComputeFrontWallCandidateDistancesForPose(
                _drive.GetPose(),
                _speedVehicle,
                capture.observedCell,
                capture.observedDirection,
                frontLeftDistanceM,
                frontRightDistanceM);
            capture.frontLeftCandidateDistanceM[capture.nextSampleIndex] = frontLeftDistanceM;
            capture.frontRightCandidateDistanceM[capture.nextSampleIndex] = frontRightDistanceM;
            ++capture.nextSampleIndex;
        }

        if (capture.nextSampleIndex < Config::kSearchRollingObservationSampleCount)
        {
            return LoopController::ControlVector::Brake;
        }

        RollingObservationVoteSummary voteSummary{};
        if (!BuildEvidenceObservationSnapshot(
                capture.samples,
                Config::kSearchRollingObservationSampleCount,
                *capture.outputSnapshot,
                voteSummary))
        {
            return FaultLoopPhase(services, "Stationary observation majority snapshot is invalid");
        }

        if (!TryApplyFrontWallCharacterizationToObservation(
                capture.observedCell,
                capture.observedDirection,
                "stationary",
                capture.samples,
                capture.frontLeftCandidateDistanceM,
                capture.frontRightCandidateDistanceM,
                Config::kSearchRollingObservationSampleCount,
                *capture.outputSnapshot))
        {
            AppendMissionTraceFormatted(
                "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=stationary,fallback_valid=%u",
                capture.observedCell.GetX(),
                capture.observedCell.GetY(),
                DirectionName(capture.observedDirection),
                capture.outputSnapshot->frontWallObservationValid ? 1U : 0U);
        }

        AppendMissionTraceFormatted(
            "mission_observation_stationary,cell=(%d,%d),abs=%s,samples=%u,front_valid=%u,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
            capture.observedCell.GetX(),
            capture.observedCell.GetY(),
            DirectionName(capture.observedDirection),
            static_cast<unsigned>(voteSummary.sampleCount),
            capture.outputSnapshot->frontWallObservationValid ? 1U : 0U,
            static_cast<unsigned>(voteSummary.frontWallVotes),
            static_cast<unsigned>(voteSummary.leftWindowValidVotes),
            static_cast<unsigned>(voteSummary.leftWallVotes),
            static_cast<unsigned>(voteSummary.rightWindowValidVotes),
            static_cast<unsigned>(voteSummary.rightWallVotes));
        return EndLoopPhase(services);
    }

    bool CaptureStationaryObservationSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        SensorSnapshot& observationSnapshot)
    {
        ObservationCaptureLoopState capture{};
        capture.observedCell = observedCell;
        capture.observedDirection = observedDirection;
        capture.outputSnapshot = &observationSnapshot;
        return RunLoopSession(&capture, &Implementation::ObservationCaptureLoopTick);
    }

    bool LogWallObservationDecision(
        const MazeMap::CellCoordinates& observedCell,
        const char* relativeDirectionName,
        MazeMap::Direction absoluteDirection,
        MazeMap::WallState observedState,
        const char* sensorSource,
        const char* sensorMode,
        float primaryDistanceM,
        float secondaryDistanceM,
        bool primaryDetected,
        bool secondaryDetected,
        const SensorSnapshot& snapshot,
        MazeMap::WallState beliefState,
        float beliefLogOdds)
    {
        char line[256] = {};
        const bool haveSecondaryDistance = std::isfinite(secondaryDistanceM);
        const int written =
            haveSecondaryDistance ?
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,obs_state=%s,belief=%s,log_odds=%.3f,sensor=%s,mode=%s,primary_hit=%u,secondary_hit=%u,primary_m=%.4f,secondary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                WallStateName(beliefState),
                beliefLogOdds,
                (sensorSource != nullptr) ? sensorSource : "unknown",
                (sensorMode != nullptr) ? sensorMode : "unknown",
                primaryDetected ? 1U : 0U,
                secondaryDetected ? 1U : 0U,
                primaryDistanceM,
                secondaryDistanceM) :
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,obs_state=%s,belief=%s,log_odds=%.3f,sensor=%s,primary_hit=%u,primary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                WallStateName(beliefState),
                beliefLogOdds,
                (sensorSource != nullptr) ? sensorSource : "unknown",
                primaryDetected ? 1U : 0U,
                primaryDistanceM);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return Fail("Unable to format wall observation log");
        }

        AppendStartupTrace(line);
        (void)WriteMissionTraceLineBestEffort(line, "mission_text_logging:wall_observation_write_failed");
        if (_telemetryLoggingEnabled && !WriteTelemetryEvent("wall_observation", line))
        {
            return Fail("Unable to write wall observation log");
        }
        (void)snapshot;
        return true;
    }

    bool EmitMissionControllerLine(const char* message)
    {
        if (message == nullptr)
        {
            return false;
        }

        if (_missionTextLoggingEnabled)
        {
            if (_runtime.WriteTextLogEntry(
                    kMissionControllerTextLogSource,
                    micros(),
                    "status",
                    message))
            {
                return true;
            }

            DisableMissionTextLogging("mission_text_logging:controller_write_failed");
        }

        return _runtime.WriteTextLogEntry(
            kMissionControllerTextLogSource,
            micros(),
            "status",
            message);
    }

    bool EmitMissionControllerFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLine(line);
    }

    bool EmitMissionControllerLineOrFail(const char* message)
    {
        return EmitMissionControllerLine(message);
    }

    bool EmitMissionControllerFormattedOrFail(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLineOrFail(line);
    }

    void AppendMissionTraceLine(const char* message)
    {
        if (message == nullptr)
        {
            return;
        }

        AppendStartupTrace(message);
        (void)WriteMissionTraceLineBestEffort(message, "mission_text_logging:trace_write_failed");
    }

    void AppendMissionTraceFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return;
        }

        char line[320] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return;
        }

        AppendMissionTraceLine(line);
    }

    bool WriteMissionMazeSnapshot(const char* trigger)
    {
        if (!_missionTextLoggingEnabled || _missionMazeSnapshotWritten)
        {
            return true;
        }

        const bool ok = MazeMap::ExportMazeSnapshot(_maze, "maze.txt");
        AppendStartupTrace(ok ? "mission_maze_snapshot:maze.txt" : "mission_maze_snapshot:write_failed");
        if (ok)
        {
            _missionMazeSnapshotWritten = true;
            (void)EmitMissionControllerFormatted("Maze snapshot written to maze.txt after %s", (trigger != nullptr) ? trigger : "unknown");
        }
        else
        {
            (void)EmitMissionControllerFormatted("Maze snapshot write failed after %s", (trigger != nullptr) ? trigger : "unknown");
        }

        return ok;
    }

    void AppendStartupCalibrationStateTrace(const char* label)
    {
        const PoseEstimate& pose = _drive.GetPose();
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[320] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_state:%s,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f,left_v=%.4f,right_v=%.4f",
            (label != nullptr) ? label : "unknown",
            pose.xMeters,
            pose.yMeters,
            pose.yawRad * RAD_TO_DEG_F,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationMoveTrace(
        const char* axis,
        float startMeters,
        float targetMeters,
        float signedTravelMeters)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_move:%s,start=%.4f,target=%.4f,signed=%.4f",
            (axis != nullptr) ? axis : "unknown",
            startMeters,
            targetMeters,
            signedTravelMeters);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTurnTrace(const char* label, float currentYawRad, float targetYawRad, float angleRad)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_turn:%s,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f",
            (label != nullptr) ? label : "unknown",
            currentYawRad * RAD_TO_DEG_F,
            targetYawRad * RAD_TO_DEG_F,
            angleRad * RAD_TO_DEG_F);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchPlanTrace(
        CalibrationWall wall,
        float expectedTravelM,
        float minLatchTravelM,
        float maxApproachTravelM,
        float targetYawRad)
    {
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch_plan:wall=%s,expected=%.4f,min_latch=%.4f,max_travel=%.4f,target_yaw_deg=%.2f",
            CalibrationWallName(wall),
            expectedTravelM,
            minLatchTravelM,
            maxApproachTravelM,
            targetYawRad * RAD_TO_DEG_F);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchTrace(
        CalibrationWall wall,
        float traveledDistanceM,
        float expectedTravelM,
        float minLatchTravelM,
        float finalYawErrorRad)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch:wall=%s,travel=%.4f,expected=%.4f,min_latch=%.4f,final_yaw_err_deg=%.2f,left_v=%.4f,right_v=%.4f",
            CalibrationWallName(wall),
            traveledDistanceM,
            expectedTravelM,
            minLatchTravelM,
            finalYawErrorRad * RAD_TO_DEG_F,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationSampleTrace(
        WallSensorId sensorId,
        CalibrationWall wall,
        float measuredValue,
        float fallbackDistanceM,
        float actualDistanceM)
    {
        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_sample:sensor=%s,wall=%s,measured=%.6f,fallback=%.4f,actual=%.4f",
            WallSensorIdName(sensorId),
            CalibrationWallName(wall),
            measuredValue,
            fallbackDistanceM,
            actualDistanceM);
        AppendStartupTrace(line);
    }

    bool ReseatMissionStartPoseWithPhasePrefix(const char* phasePrefix, uint16_t settleMs)
    {
        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[64] = {};

        if (!RetreatCalibrationPoseFromSideWallForSafeRotation(Config::kCellSizeM, centeringLimits, phasePrefix))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_south", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Down, CalibrationWall::South))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_west", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Left, CalibrationWall::West))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center_x", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownX(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_rotate_up", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, limits))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        snprintf(phaseName, sizeof(phaseName), "%s_settle", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        return HoldPosition(settleMs, phaseName);
    }

    bool ReseatCorridorRepeatabilityStartPose(uint8_t speedIndex, float centerOffsetFromTouchM)
    {
        (void)centerOffsetFromTouchM;
        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "corridor_%u_reseat", static_cast<unsigned>(speedIndex));
        return ReseatMissionStartPoseWithPhasePrefix(
            phasePrefix,
            AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs);
    }

    bool RunSingleCorridorRepeatabilityPass(uint8_t speedIndex, float cruiseSpeedMps, float outDistanceM, float centerOffsetFromTouchM)
    {
        const MotionLimits limits = CorridorRepeatabilityLimits(cruiseSpeedMps);
        const MotionLimits touchLimits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[48] = {};
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f southHeading = DirectionToUnitVector(MazeMap::Down);
        const float farCellCenterYM = (0.5f * Config::kCellSizeM) + outDistanceM;
        const float corridorSpanYM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount);
        const float farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            corridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        const Eigen::Vector2f farCellCenter(0.5f * Config::kCellSizeM, farCellCenterYM);
        const Eigen::Vector2f startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, farWallTouchYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(farCellCenterYM, centeringLimits, corridorSpanYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_home", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        if (!WriteCorridorRepeatabilityResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                speedIndex,
                cruiseSpeedMps,
                startPose,
                startTelemetry,
                _drive.GetPose(),
                _drive.GetTelemetry()))
        {
            return false;
        }

        return ReseatCorridorRepeatabilityStartPose(speedIndex, centerOffsetFromTouchM);
    }

    bool RunCorridorRepeatabilityPasses()
    {
        if constexpr (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount < 2U)
        {
            return Fail("Corridor repeatability row must be at least two cells long");
        }

        const float outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U);
        const float centerOffsetFromTouchM = MazeMap::ComputeMissionStartCenterAdvanceM(
            Config::kCellSizeM,
            Config::kMissionStartRearWallInsetM);
        if (centerOffsetFromTouchM <= 0.0f)
        {
            return Fail("Invalid corridor repeatability start-cell center offset");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            if (!RunSingleCorridorRepeatabilityPass(
                    speedIndex,
                    AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex],
                    outDistanceM,
                    centerOffsetFromTouchM))
            {
                return false;
            }
        }

        return true;
    }

    bool RunSinglePositionStraightAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t speedIndex,
        float cruiseSpeedMps)
    {
        _maze = geometry.maze;
        const MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        const MotionLimits turnLimits = PositionAccuracyAuditTurnLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f southHeading = DirectionToUnitVector(MazeMap::Down);
        const Eigen::Vector2f farCellCenter(0.5f * Config::kCellSizeM, geometry.farCellCenterYM);
        const Eigen::Vector2f startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        const DriveTelemetry outTelemetry = _drive.GetTelemetry();
        const float northStopErrorM = geometry.farCellCenterYM - poseBeforeTouch.yMeters;
        const float encoderOutDistanceM =
            0.5f *
            ((outTelemetry.leftDistanceM - startTelemetry.leftDistanceM) +
                (outTelemetry.rightDistanceM - startTelemetry.rightDistanceM));
        const float encoderOutErrorM = encoderOutDistanceM - geometry.outDistanceM;

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float northTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, geometry.farWallTouchYM, &northTouchCorrectionM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(geometry.farCellCenterYM, centeringLimits, geometry.northCorridorSpanYM))
        {
            return false;
        }

        const DriveTelemetry turnStartTelemetry = _drive.GetTelemetry();
        const float turnStartYawRad = _drive.GetPose().yawRad;
        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, turnLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            return false;
        }
        const DriveTelemetry turnEndTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - turnStartYawRad);
        const float leftTurnDeltaM = turnEndTelemetry.leftDistanceM - turnStartTelemetry.leftDistanceM;
        const float rightTurnDeltaM = turnEndTelemetry.rightDistanceM - turnStartTelemetry.rightDistanceM;
        if (!WritePositionInPlaceTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                MazeMap::Down,
                northTouchCorrectionM,
                leftTurnDeltaM,
                rightTurnDeltaM,
                yawChangeRad,
                _drive.GetPose().yawRad))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        if (!WritePositionStraightAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                speedIndex,
                cruiseSpeedMps,
                northStopErrorM,
                northTouchCorrectionM,
                encoderOutErrorM,
                startPose,
                _drive.GetPose()))
        {
            return false;
        }

        return true;
    }

    bool RunSinglePositionInPlaceTurnAuditPass(uint8_t turnIndex, MazeMap::Direction targetDirection)
    {
        if (!(targetDirection == MazeMap::Right || targetDirection == MazeMap::Left))
        {
            return Fail("Position audit in-place turn direction is invalid");
        }

        const MotionLimits turnLimits = PositionAccuracyAuditTurnLimits();
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_start", static_cast<unsigned>(turnIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        const float startYawRad = _drive.GetPose().yawRad;
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(startYawRad, DirectionToYawRad(targetDirection), angleRad))
        {
            return Fail("Position audit in-place turn angle is invalid");
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_turn", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteTurnProfile(angleRad, turnLimits))
        {
            return false;
        }

        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - startYawRad);
        const float leftDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;

        float touchCoordinateM = 0.0f;
        CalibrationWall touchWall = CalibrationWall::West;
        if (targetDirection == MazeMap::Right)
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::East;
        }
        else
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::West;
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_touch", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float touchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(targetDirection, touchWall, touchCoordinateM, &touchCorrectionM))
        {
            return false;
        }

        if (!WritePositionInPlaceTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                targetDirection,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                yawChangeRad,
                _drive.GetPose().yawRad))
        {
            return false;
        }

        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "position_ip_turn_%u_reseat", static_cast<unsigned>(turnIndex));
        return ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs);
    }

    bool RunSinglePositionSmoothTurnAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t codeIndex,
        MazeMap::ManeuverCode code,
        uint8_t speedIndex,
        float requestedCruiseSpeedMps)
    {
        _maze = geometry.maze;
        MazeMap::ManeuverPath forwardPath;
        MazeMap::ManeuverPath reversePath;
        uint8_t launchHalfSteps = 0U;
        uint8_t postStraightHalfSteps = 0U;
        if (!TryBuildPositionAuditSmoothTurnPaths(code, forwardPath, reversePath, launchHalfSteps, postStraightHalfSteps))
        {
            return Fail("Position audit smooth turn path is invalid");
        }

        const MazeMap::DirectionalLocation auditStart(
            MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0U, 0U)),
            MazeMap::Up);
        MazeMap::DirectionalLocation finalLocation;
        if (!TryValidatePositionAuditPath(geometry.maze, forwardPath, auditStart, finalLocation))
        {
            return Fail("Position audit smooth turn path does not fit fixture");
        }
        const MazeMap::DirectionalLocation returnStart(finalLocation.GetLocation(), -finalLocation.GetDirection());
        MazeMap::DirectionalLocation returnEnd;
        if (!TryValidatePositionAuditPath(geometry.maze, reversePath, returnStart, returnEnd))
        {
            return Fail("Position audit smooth turn reverse path does not fit fixture");
        }
        if (!(returnEnd.GetLocation() == auditStart.GetLocation()) || returnEnd.GetDirection() != MazeMap::Down)
        {
            return Fail("Position audit smooth turn reverse path does not return to start");
        }

        const MazeMap::ManeuverInstance turnManeuver(code, auditStart);
        const float nominalRadiusM = turnManeuver.GetNominalTurnRadiusMeters(Config::kCellSizeM);
        const MotionLimits straightLimits = PositionAccuracyAuditStraightLimits(requestedCruiseSpeedMps);
        const MotionLimits cornerLimits = PositionAccuracyAuditCornerLimits(requestedCruiseSpeedMps, nominalRadiusM);
        const MotionLimits calibrationLimits = PositionAccuracyAuditTurnLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const float turnCruiseSpeedMps =
            _runtime.ManeuverExecutorService().ComputeManeuverSpeedLimit(code, cornerLimits);
        if (!(turnCruiseSpeedMps > 0.0f))
        {
            return Fail("Position audit smooth turn speed is invalid");
        }

        const MazeMap::DirectionalLocation launchLocation = auditStart.MoveForward(launchHalfSteps);
        const float postStraightDistanceM = 0.5f * Config::kCellSizeM * static_cast<float>(postStraightHalfSteps);

        float finalTargetXM = 0.0f;
        float finalTargetYM = 0.0f;
        finalLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, finalTargetXM, finalTargetYM);
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f finalHeading = DirectionToUnitVector(finalLocation.GetDirection());
        float launchXM = 0.0f;
        float launchYM = 0.0f;
        launchLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, launchXM, launchYM);
        const Eigen::Vector2f launchPosition(launchXM, launchYM);
        const Eigen::Vector2f finalPosition(finalTargetXM, finalTargetYM);
        const float launchDistanceM = launchYM - (0.5f * Config::kCellSizeM);
        const float maneuverExitSpeedMps = turnCruiseSpeedMps;
        const MazeMap::ManeuverInstance trackedManeuver(code, launchLocation, turnCruiseSpeedMps, maneuverExitSpeedMps);
        const float maneuverDistanceM = trackedManeuver.GetTravelDistanceMeters(Config::kCellSizeM);
        const float maneuverAngleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        const bool hasTrackedManeuver = trackedManeuver.SupportsPointTracking();
        char phaseName[64] = {};

        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled)
        {
            SetRacingFanEnabled(true);
        }

        bool ok = false;
        do
        {
            snprintf(
                phaseName,
                sizeof(phaseName),
                "position_turn_%u_%u_start",
                static_cast<unsigned>(codeIndex),
                static_cast<unsigned>(speedIndex));
            if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
            {
                break;
            }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_launch",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!ExecuteStraightProfile(
                launchDistanceM,
                0.0f,
                requestedCruiseSpeedMps,
                turnCruiseSpeedMps,
                straightLimits,
                true,
                &northHeading,
                &launchPosition))
        {
            break;
        }

        const DriveTelemetry arcStartTelemetry = _drive.GetTelemetry();
        const float arcStartYawRad = _drive.GetPose().yawRad;
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_arc",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (hasTrackedManeuver)
        {
            if (!ExecuteSmoothTurnProfile(
                    trackedManeuver,
                    turnCruiseSpeedMps,
                    cornerLimits))
            {
                break;
            }
        }
        else if (!ExecuteArcProfile(
                maneuverDistanceM,
                maneuverAngleRad,
                turnCruiseSpeedMps,
                maneuverExitSpeedMps,
                turnCruiseSpeedMps,
                cornerLimits))
        {
            break;
        }
        const DriveTelemetry arcEndTelemetry = _drive.GetTelemetry();
        const float arcEndYawRad = _drive.GetPose().yawRad;
        const float leftArcDeltaM = arcEndTelemetry.leftDistanceM - arcStartTelemetry.leftDistanceM;
        const float rightArcDeltaM = arcEndTelemetry.rightDistanceM - arcStartTelemetry.rightDistanceM;
        const float yawChangeRad = WrapAngleRad(arcEndYawRad - arcStartYawRad);

        if (postStraightDistanceM > 0.0f)
        {
            snprintf(
                phaseName,
                sizeof(phaseName),
                "position_turn_%u_%u_post",
                static_cast<unsigned>(codeIndex),
                static_cast<unsigned>(speedIndex));
            if (!BeginTelemetryPhase(phaseName))
            {
                break;
            }
            if (!ExecuteStraightProfile(
                    postStraightDistanceM,
                    turnCruiseSpeedMps,
                    requestedCruiseSpeedMps,
                    0.0f,
                    straightLimits,
                    true,
                    &finalHeading,
                    &finalPosition))
            {
                break;
            }
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        SensorSnapshot snapshotBeforeTouch{};
        _sensors.Capture(true, _drive.GetPose(), snapshotBeforeTouch);
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(DirectionToYawRad(finalLocation.GetDirection()), poseBeforeTouch.yawRad);

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_touch_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        float eastTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Right, CalibrationWall::East, geometry.eastWallTouchXM, &eastTouchCorrectionM))
        {
            break;
        }

        if (!WritePositionSmoothTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                code,
                speedIndex,
                turnCruiseSpeedMps,
                nominalRadiusM,
                snapshotBeforeTouch.corridorErrorM,
                eastTouchCorrectionM,
                leftArcDeltaM,
                rightArcDeltaM,
                yawChangeRad,
                yawErrorDeg))
        {
            break;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_center_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!DriveCalibrationPoseToKnownX(finalTargetXM, centeringLimits))
        {
            break;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_face_left",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!RotateCalibrationTo(MazeMap::Left, calibrationLimits))
        {
            break;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            break;
        }
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_return",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }

            _currentDirectionalLocation = returnStart;
            _currentDirection = _currentDirectionalLocation.GetDirection();
            _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());
            MazeMap::ManeuverQueue queue(reversePath, _currentDirectionalLocation);
            queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
            _runtime.ManeuverExecutorService().ApplyAsymmetricQueueLimits(
                queue,
                cornerLimits,
                _speedVehicle,
                0.0f,
                0.0f);
            ok = ExecuteQueuedManeuvers(queue, cornerLimits, false);
        }
        while (false);

        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled)
        {
            SetRacingFanEnabled(false);
        }
        return ok;
    }

    bool RunPositionAccuracyAuditPasses()
    {
        if constexpr (AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount < 3U)
        {
            return Fail("Position accuracy audit north corridor must be at least three cells");
        }
        if constexpr (AuxMeasurementConfig::kPositionAuditEastBranchCellCount < 1U)
        {
            return Fail("Position accuracy audit east extension must be at least one cell");
        }

        const PositionAuditFixtureGeometry geometry = BuildPositionAuditFixtureGeometry();

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            char phasePrefix[56] = {};
            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase1", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionStraightAuditPass(
                    geometry,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase2", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    0U,
                    MazeMap::S90SS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase3", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    1U,
                    MazeMap::S90LS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }
        }

        return true;
    }

    void SeedStartupWallCalibrationPoseFromSouthWall()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SetPose(0.5f * Config::kCellSizeM, Config::kMissionStartRearWallInsetM, DirectionToYawRad(MazeMap::Up));
        AppendStartupCalibrationStateTrace("seed_south_wall_start");
    }

    bool RotateCalibrationTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Startup calibration turn angle is invalid");
        }
        AppendStartupCalibrationTurnTrace("rotate_begin", _drive.GetPose().yawRad, targetYawRad, angleRad);
        if (!ExecuteTurnProfile(angleRad, limits))
        {
            return false;
        }

        AppendStartupCalibrationStateTrace("rotate_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownX(float targetXMeters, const MotionLimits& limits)
    {
        // Audit reseat can target a global fixture x beyond one cell after an east-wall touch.
        if (!MazeMap::IsValidCalibrationCenterCoordinateM(targetXMeters))
        {
            return Fail("Startup calibration target x is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingX = startPose.headingUnit.x();
        if (std::fabs(headingX) < 0.5f)
        {
            return Fail("Startup calibration x reposition requires east-west heading");
        }

        const float deltaXMeters = targetXMeters - startPose.xMeters;
        const float signedTravelMeters = deltaXMeters / headingX;
        const Eigen::Vector2f targetHeading = startPose.headingUnit;
        const Eigen::Vector2f targetPosition(targetXMeters, startPose.yMeters);
        AppendStartupCalibrationMoveTrace("x", startPose.xMeters, targetXMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        AppendStartupCalibrationStateTrace("x_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits, float maxAllowedYMeters)
    {
        if (!(std::isfinite(targetYMeters) &&
            std::isfinite(maxAllowedYMeters) &&
            targetYMeters >= 0.0f &&
            maxAllowedYMeters >= 0.0f &&
            targetYMeters <= maxAllowedYMeters))
        {
            return Fail("Startup calibration target y is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingY = startPose.headingUnit.y();
        if (std::fabs(headingY) < 0.5f)
        {
            return Fail("Startup calibration y reposition requires north-south heading");
        }

        const float deltaYMeters = targetYMeters - startPose.yMeters;
        const float signedTravelMeters = deltaYMeters / headingY;
        const Eigen::Vector2f targetHeading = startPose.headingUnit;
        const Eigen::Vector2f targetPosition(startPose.xMeters, targetYMeters);
        AppendStartupCalibrationMoveTrace("y", startPose.yMeters, targetYMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        AppendStartupCalibrationStateTrace("y_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits)
    {
        return DriveCalibrationPoseToKnownY(targetYMeters, limits, Config::kCellSizeM);
    }

    float ComputeCalibrationSideRotationClearanceM() const
    {
        return Config::kWallCalibrationWallClearanceM + Config::kDistanceToleranceM;
    }

    float ComputeCalibrationSafeMinCenterXForWestWallRotationM() const
    {
        return MazeMap::ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    float ComputeCalibrationSafeMaxCenterXForEastWallRotationM(float spanXMeters) const
    {
        return MazeMap::ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(
            spanXMeters,
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    bool RetreatCalibrationPoseFromSideWallForSafeRotation(
        float spanXMeters,
        const MotionLimits& limits,
        const char* phasePrefix = nullptr)
    {
        if (!(std::isfinite(spanXMeters) && spanXMeters > 0.0f))
        {
            return Fail("Startup calibration side-clear span is invalid");
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float headingX = pose.headingUnit.x();
        if (std::fabs(headingX) < 0.5f)
        {
            return true;
        }

        const float safeMinCenterXM = ComputeCalibrationSafeMinCenterXForWestWallRotationM();
        const float safeMaxCenterXM = ComputeCalibrationSafeMaxCenterXForEastWallRotationM(spanXMeters);
        if (!(safeMinCenterXM > 0.0f &&
            safeMaxCenterXM > safeMinCenterXM &&
            safeMaxCenterXM < spanXMeters))
        {
            return Fail("Startup calibration side-clear target is invalid");
        }

        float targetXMeters = pose.xMeters;
        if (headingX > 0.5f)
        {
            targetXMeters = (std::min)(pose.xMeters, safeMaxCenterXM);
        }
        else
        {
            targetXMeters = (std::max)(pose.xMeters, safeMinCenterXM);
        }

        if (std::fabs(targetXMeters - pose.xMeters) <= Config::kDistanceToleranceM)
        {
            return true;
        }

        if (phasePrefix != nullptr)
        {
            char phaseName[64] = {};
            snprintf(phaseName, sizeof(phaseName), "%s_clear_side", phasePrefix);
            if (!BeginTelemetryPhase(phaseName))
            {
                return false;
            }
        }

        return DriveCalibrationPoseToKnownX(targetXMeters, limits);
    }

    struct WallTouchPoseResetTarget
    {
        float xMeters = 0.0f;
        float yMeters = 0.0f;
        float yawRad = 0.0f;
        bool enabled = false;
    };

    bool ExecuteWallTouchOff(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        const WallTouchPoseResetTarget* poseResetTarget,
        WallTouchOutcome& outcome,
        float& traveledDistanceM,
        float* seatedYawErrorRad = nullptr)
    {
        return ExecuteWallTouchOffLoopDriven(
            targetYawRad,
            minLatchTravelM,
            maxApproachTravelM,
            allowPassThroughNoWall,
            poseResetTarget,
            outcome,
            traveledDistanceM,
            seatedYawErrorRad);
    }

    LoopController::ControlVector WallTouchLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& wallTouch = *static_cast<WallTouchLoopState*>(rawState);

        const float clampedMinLatchTravelM = (std::max)(0.0f, wallTouch.minLatchTravelM);
        const float clampedMaxApproachTravelM =
            (std::max)(clampedMinLatchTravelM, wallTouch.maxApproachTravelM);
        if (!(std::isfinite(clampedMaxApproachTravelM) && (clampedMaxApproachTravelM > 0.0f)))
        {
            return FaultLoopPhase(services, "Wall touch-off max travel is invalid");
        }

        auto appendTraceLine = [this](const char* line)
        {
            if (line != nullptr)
            {
                AppendStartupTrace(line);
            }
        };
        auto traceStateTransition =
            [&wallTouch, &appendTraceLine](MazeMap::App::Internal::Runtime::WallTouchState fromState,
                                           MazeMap::App::Internal::Runtime::WallTouchState toState,
                                           float traveledDistanceM)
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(fromState),
                MazeMap::App::Internal::Runtime::WallTouchStateName(toState),
                static_cast<unsigned long>(millis() - wallTouch.touchStartMs),
                traveledDistanceM);
            appendTraceLine(line);
        };

        const MazeMap::App::Internal::Runtime::WallTouchObservation observation =
            MazeMap::App::Internal::Runtime::MakeWallTouchObservation(state.sensors);
        const unsigned long nowMs = millis();
        const unsigned long elapsedMs = nowMs - wallTouch.touchStartMs;
        const unsigned long stateElapsedMs = nowMs - wallTouch.stateStartMs;
        const PoseEstimate& pose = _drive.GetPose();
        const DriveTelemetry& telemetry = state.driveTelemetry;
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - wallTouch.startDistanceM);
        const bool frontSignalActive =
            observation.frontWall ||
            observation.frontLeftWall ||
            observation.frontRightWall;
        wallTouch.result.finalTravelM = traveledDistanceM;

        if ((wallTouch.runtimeState != MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease) &&
            (wallTouch.runtimeState != MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance) &&
            (traveledDistanceM >= clampedMaxApproachTravelM))
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:max_travel,state=%s,travel=%.4f,expected=%.4f,max=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(wallTouch.runtimeState),
                traveledDistanceM,
                clampedMinLatchTravelM,
                clampedMaxApproachTravelM);
            appendTraceLine(line);
            if (wallTouch.allowPassThroughNoWall && (wallTouch.contactConfirmedStartMs == 0UL))
            {
                wallTouch.result.outcome = WallTouchOutcome::PassedThroughNoWall;
                if (!BeginSharedBrakedSettlePhase(
                        wallTouch.passThroughSettle,
                        "Wall touch-off failed to settle after pass-through",
                        Config::kStartupWallCalibrationSettleMs,
                        0U,
                        nullptr,
                        nullptr,
                        services))
                {
                    return FaultLoopPhase(services, "Failed to begin wall-touch pass-through settle phase");
                }
                return LoopController::ControlVector::Brake;
            }
            return FaultLoopPhase(services, "Wall touch-off exceeded max travel");
        }

        if (MazeMap::App::Internal::Runtime::HasWallTouchEncoderMotion(
                wallTouch.lastMotionTelemetry,
                telemetry,
                Config::kWallTouchProgressStallDistanceM))
        {
            wallTouch.lastMotionMs = nowMs;
            wallTouch.lastMotionTelemetry = telemetry;
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek)
        {
            wallTouch.approachDriveCommand =
                MazeMap::App::Internal::Runtime::ComputeWallTouchApproachDriveCommand(
                    traveledDistanceM,
                    clampedMinLatchTravelM);
            LoopController::ControlVector command = LoopController::ControlVector::Brake;
            if (MazeMap::App::Internal::Runtime::ShouldBrakeWallTouchApproachForEncoderSpeed(telemetry))
            {
                command = LoopController::ControlVector::Brake;
            }
            else
            {
                wallTouch.approachDriveCommand =
                    MazeMap::App::Internal::Runtime::LimitWallTouchApproachDriveCommandByEncoderSpeed(
                        wallTouch.approachDriveCommand,
                        telemetry);
                command = LoopController::ControlVector::RawMotorPwm(
                    wallTouch.approachDriveCommand,
                    wallTouch.approachDriveCommand);
            }

            const bool motionCollapseIndicator = MazeMap::IsWallTouchContactSample(
                traveledDistanceM,
                pose.linearSpeedMps,
                Config::kWallTouchMinApproachDistanceM,
                clampedMinLatchTravelM,
                Config::kMotionSettleSpeedThresholdMps,
                elapsedMs,
                Config::kWallTouchMinCommandTimeMs);
            const bool progressStallIndicator =
                (elapsedMs >= Config::kWallTouchMinCommandTimeMs) &&
                ((nowMs - wallTouch.lastMotionMs) >= Config::kWallTouchProgressStallWindowMs);
            const std::uint8_t indicatorCount = MazeMap::CountWallTouchContactIndicators(
                frontSignalActive,
                motionCollapseIndicator,
                progressStallIndicator);
            if (indicatorCount >= 2U)
            {
                if (!wallTouch.contactCandidateActive)
                {
                    wallTouch.contactCandidateStartMs = nowMs;
                    wallTouch.contactCandidateActive = true;
                }
                else if (MazeMap::HasWallTouchConfirmedContact(
                    nowMs - wallTouch.contactCandidateStartMs,
                    Config::kWallTouchContactConfirmationMs,
                    indicatorCount))
                {
                    wallTouch.contactConfirmedStartMs = wallTouch.contactCandidateStartMs;
                    wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp;
                    wallTouch.stateStartMs = nowMs;
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:contact_confirmed,travel=%.4f,elapsed_ms=%lu,front=%u,collapse=%u,stall=%u",
                        traveledDistanceM,
                        elapsedMs,
                        frontSignalActive ? 1U : 0U,
                        motionCollapseIndicator ? 1U : 0U,
                        progressStallIndicator ? 1U : 0U);
                    appendTraceLine(line);
                    traceStateTransition(
                        MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek,
                        wallTouch.runtimeState,
                        traveledDistanceM);
                }
            }
            else
            {
                wallTouch.contactCandidateActive = false;
            }

            return command;
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp)
        {
            const float rampAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchSeatRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchSeatRampMs, static_cast<std::uint16_t>(1U)));
            const float seatDriveCommand =
                wallTouch.approachDriveCommand +
                ((Config::kWallTouchSeatRampMaxDriveCommand - wallTouch.approachDriveCommand) * rampAlpha);
            if (stateElapsedMs >= Config::kWallTouchSeatRampMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(seatDriveCommand, seatDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell)
        {
            if (stateElapsedMs >= Config::kWallTouchInitialSeatDwellMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither;
                wallTouch.stateStartMs = nowMs;
                wallTouch.currentCycleStartYawRad = pose.yawRad;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                wallTouch.currentCycleFrontSignalValid = true;
                wallTouch.haveSquareSample = false;
                wallTouch.completedHalfCycles = 0U;
                wallTouch.result.completedFullCycles = 0U;
                wallTouch.consecutiveGoodFullCycles = 0U;
                wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
                wallTouch.frontSignalMissingStartMs = 0UL;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither)
        {
            const unsigned long contactDurationMs =
                (wallTouch.contactConfirmedStartMs > 0UL) ?
                (nowMs - wallTouch.contactConfirmedStartMs) :
                0UL;
            wallTouch.result.confirmedContactMs = contactDurationMs;
            if (!frontSignalActive)
            {
                if (wallTouch.frontSignalMissingStartMs == 0UL)
                {
                    wallTouch.frontSignalMissingStartMs = nowMs;
                }
                else if ((nowMs - wallTouch.frontSignalMissingStartMs) >= Config::kWallTouchContactConfirmationMs)
                {
                    char line[192] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:front_signal_invalid,elapsed_ms=%lu,travel=%.4f",
                        contactDurationMs,
                        traveledDistanceM);
                    appendTraceLine(line);
                    return FaultLoopPhase(services, "Wall touch-off front sensors invalid during square-up");
                }
            }
            else
            {
                wallTouch.frontSignalMissingStartMs = 0UL;
            }

            const MazeMap::OpenLoopDriveCommand ditherCommand = MazeMap::ComputeOpenLoopYawDitherCommand(
                Config::kWallTouchSeatRampMaxDriveCommand,
                stateElapsedMs,
                Config::kWallTouchSeatWiggleHalfPeriodMs,
                Config::kWallTouchSeatWiggleBlendMs,
                wallTouch.ditherTurnFraction,
                Config::kWallTouchSeatWiggleRetainedForwardFraction);

            const unsigned long halfCycleIndex =
                stateElapsedMs /
                (std::max)(Config::kWallTouchSeatWiggleHalfPeriodMs, static_cast<std::uint16_t>(1U));
            if (wallTouch.haveSquareSample && (halfCycleIndex != wallTouch.lastHalfCycleIndex))
            {
                ++wallTouch.completedHalfCycles;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = (std::max)(
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                    std::fabs(wallTouch.lastSquareFrontSkewM));
                wallTouch.currentCycleMaxResidualYawRateRadps = (std::max)(
                    wallTouch.currentCycleMaxResidualYawRateRadps,
                    std::fabs(wallTouch.lastSquareYawRateRadps));
                wallTouch.currentCycleFrontSignalValid =
                    wallTouch.currentCycleFrontSignalValid && wallTouch.lastSquareFrontSignalValid;

                char halfCycleLine[256] = {};
                snprintf(
                    halfCycleLine,
                    sizeof(halfCycleLine),
                    "startup_cal_touch:half_cycle,index=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,turn_fraction=%.3f",
                    static_cast<unsigned>(wallTouch.completedHalfCycles),
                    std::fabs(wallTouch.lastSquareFrontSkewM),
                    std::fabs(wallTouch.lastSquareYawRateRadps),
                    wallTouch.ditherTurnFraction);
                appendTraceLine(halfCycleLine);

                if ((wallTouch.completedHalfCycles & 1U) == 0U)
                {
                    ++wallTouch.result.completedFullCycles;
                    const float netYawChangeMagnitudeRad =
                        std::fabs(AngleErrorRad(wallTouch.currentCycleStartYawRad, wallTouch.lastSquareYawRad));
                    const bool cycleGood = MazeMap::IsWallTouchSquareCycleGood(
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        Config::kWallTouchSquareFrontSkewThresholdM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        Config::kWallTouchSquareResidualYawRateThresholdRadps,
                        netYawChangeMagnitudeRad,
                        Config::kWallTouchSquareNetYawChangeThresholdRad,
                        wallTouch.currentCycleFrontSignalValid);
                    wallTouch.consecutiveGoodFullCycles =
                        cycleGood ? static_cast<std::uint8_t>(wallTouch.consecutiveGoodFullCycles + 1U) : 0U;

                    char cycleLine[320] = {};
                    snprintf(
                        cycleLine,
                        sizeof(cycleLine),
                        "startup_cal_touch:full_cycle,index=%u,good=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,net_yaw_deg=%.2f,contact_ms=%lu,turn_fraction=%.3f",
                        static_cast<unsigned>(wallTouch.result.completedFullCycles),
                        cycleGood ? 1U : 0U,
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        RAD_TO_DEG_F * netYawChangeMagnitudeRad,
                        contactDurationMs,
                        wallTouch.ditherTurnFraction);
                    appendTraceLine(cycleLine);

                    if (!cycleGood &&
                        (wallTouch.ditherTurnFraction < Config::kWallTouchSeatWiggleMaxTurnFraction) &&
                        (MazeMap::HasWallTouchSquareUpSaturated(
                            wallTouch.previousCycleFrontSkewMagnitudeM,
                            wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                            Config::kWallTouchSquareImprovementSaturationThresholdM) ||
                            (wallTouch.result.completedFullCycles >= Config::kWallTouchSeatMinimumFullCycles)))
                    {
                        wallTouch.ditherTurnFraction = MazeMap::ComputeWallTouchSeatWiggleTurnFraction(
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatWiggleTurnFraction,
                            Config::kWallTouchSeatWiggleTurnFractionStep,
                            Config::kWallTouchSeatWiggleMaxTurnFraction);
                    }

                    wallTouch.previousCycleFrontSkewMagnitudeM = wallTouch.currentCycleMaxFrontSkewMagnitudeM;
                    wallTouch.currentCycleStartYawRad = wallTouch.lastSquareYawRad;
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                    wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                    wallTouch.currentCycleFrontSignalValid = true;

                    if (MazeMap::IsWallTouchSquareSuccessEligible(
                            contactDurationMs,
                            Config::kWallTouchMinimumConfirmedContactMs,
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatMinimumFullCycles,
                            wallTouch.consecutiveGoodFullCycles,
                            Config::kWallTouchSeatRequiredGoodFullCycles))
                    {
                        wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold;
                        wallTouch.stateStartMs = nowMs;
                        wallTouch.result.seatedTravelM = traveledDistanceM;
                        wallTouch.result.seatedYawErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
                        traceStateTransition(
                            MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither,
                            wallTouch.runtimeState,
                            traveledDistanceM);
                    }
                }
            }

            if (contactDurationMs >= Config::kWallTouchSquareUpTimeoutMs)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:square_timeout,contact_ms=%lu,turn_fraction=%.3f,cycles=%u",
                    contactDurationMs,
                    wallTouch.ditherTurnFraction,
                    static_cast<unsigned>(wallTouch.result.completedFullCycles));
                appendTraceLine(line);
                return FaultLoopPhase(services, "Wall touch-off square-up timed out");
            }

            wallTouch.haveSquareSample = true;
            wallTouch.lastHalfCycleIndex = halfCycleIndex;
            wallTouch.lastSquareYawRad = pose.yawRad;
            wallTouch.lastSquareFrontSkewM = observation.frontSkewM;
            wallTouch.lastSquareYawRateRadps = pose.angularSpeedRadps;
            wallTouch.lastSquareFrontSignalValid = frontSignalActive;
            return LoopController::ControlVector::RawMotorPwm(
                ditherCommand.leftDriveCommand,
                ditherCommand.rightDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold)
        {
            if (!wallTouch.seatedResetApplied &&
                (stateElapsedMs >= (Config::kWallTouchPostSquareHoldMs / 2U)))
            {
                wallTouch.seatedResetApplied = true;
                if (wallTouch.poseResetTarget != nullptr && wallTouch.poseResetTarget->enabled)
                {
                    _drive.SetPose(
                        wallTouch.poseResetTarget->xMeters,
                        wallTouch.poseResetTarget->yMeters,
                        wallTouch.poseResetTarget->yawRad);
                    AppendStartupCalibrationStateTrace("touch_pose_set");
                }
            }
            if (stateElapsedMs >= Config::kWallTouchPostSquareHoldMs)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:reset_pose,x=%.4f,y=%.4f,yaw_deg=%.2f,travel=%.4f",
                    _drive.GetPose().xMeters,
                    _drive.GetPose().yMeters,
                    RAD_TO_DEG_F * _drive.GetPose().yawRad,
                    wallTouch.result.seatedTravelM);
                appendTraceLine(line);
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease)
        {
            const float releaseAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchReleaseRampMs, static_cast<std::uint16_t>(1U)));
            const float forwardPreloadCommand =
                Config::kWallTouchSeatRampMaxDriveCommand * (1.0f - releaseAlpha);
            float reverseCommand = 0.0f;
            if (Config::kWallTouchReleaseReverseOverlapMs >= Config::kWallTouchReleaseRampMs)
            {
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * releaseAlpha;
            }
            else if (stateElapsedMs >= (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs))
            {
                const unsigned long reverseElapsedMs =
                    stateElapsedMs - (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs);
                const float reverseAlpha =
                    static_cast<float>((std::min)(reverseElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseReverseOverlapMs))) /
                    static_cast<float>((std::max)(Config::kWallTouchReleaseReverseOverlapMs, static_cast<std::uint16_t>(1U)));
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * reverseAlpha;
            }

            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            if ((wallTouch.result.reverseDistanceM >= Config::kDistanceToleranceM) && !frontSignalActive)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:release_clear,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            else if (stateElapsedMs >= Config::kWallTouchReleaseRampMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                forwardPreloadCommand - reverseCommand,
                forwardPreloadCommand - reverseCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance)
        {
            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            const float headingErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
            float angularCommandRadps =
                Config::kStraightHeadingKp * headingErrorRad;
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -Config::kWallTouchReverseMaxAngularCommandRadps,
                Config::kWallTouchReverseMaxAngularCommandRadps);

            if (wallTouch.result.reverseDistanceM >= Config::kWallTouchFrontClearanceDistanceM)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_reached,reverse_m=%.4f,target_m=%.4f",
                    wallTouch.result.reverseDistanceM,
                    Config::kWallTouchFrontClearanceDistanceM);
                appendTraceLine(line);
                return EndLoopPhase(services);
            }

            if ((stateElapsedMs >= Config::kMotionSettleTimeoutMs) && frontSignalActive)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_failed,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                return FaultLoopPhase(services, "Wall touch-off failed to establish front-wall clearance");
            }

            return _drive.PointControlVector(
                -Config::kWallTouchReverseSpeedMps,
                angularCommandRadps,
                kMissionDriveBaseTrackingCommandPd);
        }

        return EndLoopPhase(services);
    }

    bool ExecuteWallTouchOffLoopDriven(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        const WallTouchPoseResetTarget* poseResetTarget,
        WallTouchOutcome& outcome,
        float& traveledDistanceM,
        float* seatedYawErrorRad = nullptr)
    {
        outcome = WallTouchOutcome::SeatedContact;
        traveledDistanceM = 0.0f;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = 0.0f;
        }

        WallTouchLoopState wallTouch{};
        wallTouch.targetYawRad = targetYawRad;
        wallTouch.minLatchTravelM = minLatchTravelM;
        wallTouch.maxApproachTravelM = maxApproachTravelM;
        wallTouch.allowPassThroughNoWall = allowPassThroughNoWall;
        wallTouch.poseResetTarget = poseResetTarget;
        wallTouch.seatedYawErrorRad = seatedYawErrorRad;
        wallTouch.startDistanceM = _drive.GetAverageDistanceMeters();
        wallTouch.touchStartMs = millis();
        wallTouch.stateStartMs = wallTouch.touchStartMs;
        wallTouch.lastMotionMs = wallTouch.touchStartMs;
        wallTouch.lastMotionTelemetry = _drive.GetTelemetry();
        wallTouch.approachDriveCommand = Config::kWallTouchDriveCommand;
        wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
        wallTouch.previousCycleFrontSkewMagnitudeM = std::numeric_limits<float>::infinity();
        wallTouch.currentCycleStartYawRad = _drive.GetPose().yawRad;
        wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek;
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(
                    MazeMap::App::Internal::Runtime::WallTouchState::EntryConditioning),
                MazeMap::App::Internal::Runtime::WallTouchStateName(wallTouch.runtimeState),
                0UL,
                0.0f);
            AppendStartupTrace(line);
        }

        if (!RunLoopSession(&wallTouch, &Implementation::WallTouchLoopTick))
        {
            return false;
        }

        outcome = wallTouch.result.outcome;
        traveledDistanceM = wallTouch.result.seatedTravelM;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = wallTouch.result.seatedYawErrorRad;
        }
        return true;
    }

    bool TryTouchWallAndMaybeSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        bool allowPassThroughNoWall,
        WallTouchOutcome& outcome,
        float* traveledDistanceM = nullptr)
    {
        outcome = WallTouchOutcome::SeatedContact;
        if (!(std::isfinite(targetCoordinateM) && targetCoordinateM >= 0.0f))
        {
            return Fail("Startup calibration touch coordinate is invalid");
        }

        const MotionLimits limits = StartupWallCalibrationLimits();
        if (!RotateCalibrationTo(facingDirection, limits))
        {
            return false;
        }
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        float xMeters = pose.xMeters;
        float yMeters = pose.yMeters;
        float expectedTravelM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.xMeters - xMeters);
            break;
        case CalibrationWall::East:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(xMeters - pose.xMeters);
            break;
        case CalibrationWall::South:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.yMeters - yMeters);
            break;
        case CalibrationWall::North:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(yMeters - pose.yMeters);
            break;
        default:
            break;
        }

        const float targetYawRad = DirectionToYawRad(facingDirection);
        const float minLatchTravelM = MazeMap::ComputeWallTouchMinimumLatchTravelM(
            expectedTravelM,
            Config::kWallTouchMinApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        const float maxApproachTravelM = MazeMap::ComputeWallTouchMaximumApproachDistanceM(
            expectedTravelM,
            Config::kWallTouchBaseMaxApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        AppendStartupCalibrationTouchPlanTrace(wall, expectedTravelM, minLatchTravelM, maxApproachTravelM, targetYawRad);

        float localTravelM = 0.0f;
        float finalYawErrorRad = 0.0f;
        const WallTouchPoseResetTarget poseResetTarget{
            xMeters,
            yMeters,
            DirectionToYawRad(facingDirection),
            true
        };
        if (!ExecuteWallTouchOff(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                &poseResetTarget,
                outcome,
                localTravelM,
                &finalYawErrorRad))
        {
            return false;
        }

        if (outcome == WallTouchOutcome::SeatedContact)
        {
            AppendStartupCalibrationTouchTrace(wall, localTravelM, expectedTravelM, minLatchTravelM, finalYawErrorRad);
        }

        if (traveledDistanceM != nullptr)
        {
            *traveledDistanceM = localTravelM;
        }
        return true;
    }

    bool TouchWallAndSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        float* traveledDistanceM = nullptr)
    {
        WallTouchOutcome outcome = WallTouchOutcome::SeatedContact;
        if (!TryTouchWallAndMaybeSetKnownWallCoordinate(
                facingDirection,
                wall,
                targetCoordinateM,
                false,
                outcome,
                traveledDistanceM))
        {
            return false;
        }

        return true;
    }

    bool TouchWallAndSetPose(MazeMap::Direction facingDirection, CalibrationWall wall, float* traveledDistanceM = nullptr)
    {
        float targetCoordinateM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::East:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::South:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromSouthWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::North:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        default:
            return Fail("Startup calibration wall touch is invalid");
        }

        return TouchWallAndSetKnownWallCoordinate(
            facingDirection,
            wall,
            targetCoordinateM,
            traveledDistanceM);
    }

    bool TryComputeCalibrationReferenceDistanceM(const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM) const
    {
        const PoseEstimate& pose = _drive.GetPose();
        switch (wall)
        {
        case CalibrationWall::West:
            return TryDistanceToWestWall(pose, sensor, actualDistanceM);
        case CalibrationWall::East:
            return TryDistanceToEastWall(pose, sensor, actualDistanceM);
        case CalibrationWall::South:
            return TryDistanceToSouthWall(pose, sensor, actualDistanceM);
        case CalibrationWall::North:
            return false;
        default:
            return false;
        }
    }

    bool StoreWallCalibrationPoint(
        WallSensorId sensorId,
        CalibrationWall wall,
        float actualDistanceM,
        const WallSensorCalibrationInput& input,
        const RobustSignalBand* differentialLightBand = nullptr)
    {
        if (!(std::isfinite(actualDistanceM) && actualDistanceM > 0.0f))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        if (!gWallDistanceCalibration.AddPoint(sensorId, input.measuredValue, actualDistanceM, input.ambientLight))
        {
            return Fail("Unable to store startup wall calibration point");
        }
        if ((sensorId == WallSensorId::SideLeft) || (sensorId == WallSensorId::SideRight))
        {
            if (std::isfinite(input.differentialLight) && input.differentialLight > 0.0f)
            {
                gWallDistanceCalibration.SetSideWallReferenceDifferentialLight(sensorId, input.differentialLight);
                gWallDistanceCalibration.SetSideWallReferenceDistanceM(sensorId, actualDistanceM);
                if ((differentialLightBand != nullptr) &&
                    std::isfinite(differentialLightBand->low) &&
                    std::isfinite(differentialLightBand->high) &&
                    differentialLightBand->low > 0.0f &&
                    differentialLightBand->high >= differentialLightBand->low)
                {
                    gWallDistanceCalibration.SetSideWallReferenceDifferentialLightBand(
                        sensorId,
                        differentialLightBand->low,
                        differentialLightBand->high);
                }
            }
        }
        else if (IsFrontWallSensor(sensorId) &&
            (differentialLightBand != nullptr) &&
            std::isfinite(input.measuredValue) &&
            input.measuredValue > 0.0f &&
            std::isfinite(differentialLightBand->low) &&
            std::isfinite(differentialLightBand->high) &&
            differentialLightBand->low > 0.0f &&
            differentialLightBand->high >= differentialLightBand->low)
        {
            gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
                sensorId,
                input.measuredValue,
                differentialLightBand->low,
                differentialLightBand->high);
        }
        AppendStartupCalibrationSampleTrace(sensorId, wall, input.measuredValue, input.fallbackDistanceM, actualDistanceM);

        return true;
    }

    bool AddWallCalibrationPoint(WallSensorId sensorId, const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM)
    {
        if (!TryComputeCalibrationReferenceDistanceM(sensor, wall, actualDistanceM))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        const WallSensorCalibrationCapture capture = SampleWallCalibrationCaptureAverageRaw(sensorId, sensor);
        return StoreWallCalibrationPoint(
            sensorId,
            wall,
            actualDistanceM,
            capture.input,
            capture.haveDifferentialLightBand ? &capture.differentialLightBand : nullptr);
    }

    template <size_t MaxSamples>
    bool TryStoreFrontCalibrationSpinSensorBands(
        WallSensorId sensorId,
        const std::array<float, MaxSamples>& openSamples,
        uint16_t openSampleCount,
        const std::array<float, MaxSamples>& wallSamples,
        const std::array<float, MaxSamples>& wallDistanceSamples,
        uint16_t wallSampleCount)
    {
        constexpr float kCollapsedFrontSweepSignalEpsilon = 1.0e-4f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        float onTargetDistanceM = 0.0f;
        float offTargetDistanceM = 0.0f;
        if (!TryComputeFrontWallObservationThresholdDistancesM(
                _speedVehicle,
                sensorId,
                Config::kFrontWallReleaseHysteresisM,
                onTargetDistanceM,
                offTargetDistanceM))
        {
            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_observation_geometry_unavailable,sensor=%s",
                WallSensorIdName(sensorId));
            AppendStartupTrace(traceLine);
            return false;
        }

        const auto logOpenPool = [&](const RobustSignalBand* openBand) noexcept
        {
            float minSignal = INFINITY;
            float maxSignal = 0.0f;
            uint16_t validCount = 0U;
            uint16_t collapsedCount = 0U;
            for (uint16_t index = 0U; index < openSampleCount; ++index)
            {
                const float signal = openSamples[index];
                if (!std::isfinite(signal) || signal < 0.0f)
                {
                    continue;
                }

                minSignal = (signal < minSignal) ? signal : minSignal;
                maxSignal = (signal > maxSignal) ? signal : maxSignal;
                if (signal <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++collapsedCount;
                }
                ++validCount;
            }

            char traceLine[320] = {};
            if (openBand != nullptr)
            {
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_front_sweep_open_pool,sensor=%s,count=%u,valid=%u,collapsed=%u,min=%.6f,max=%.6f,median=%.6f,low=%.6f,high=%.6f",
                    WallSensorIdName(sensorId),
                    static_cast<unsigned>(openSampleCount),
                    static_cast<unsigned>(validCount),
                    static_cast<unsigned>(collapsedCount),
                    std::isfinite(minSignal) ? minSignal : 0.0f,
                    maxSignal,
                    openBand->median,
                    openBand->low,
                    openBand->high);
            }
            else
            {
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_front_sweep_open_pool,sensor=%s,count=%u,valid=%u,collapsed=%u,min=%.6f,max=%.6f",
                    WallSensorIdName(sensorId),
                    static_cast<unsigned>(openSampleCount),
                    static_cast<unsigned>(validCount),
                    static_cast<unsigned>(collapsedCount),
                    std::isfinite(minSignal) ? minSignal : 0.0f,
                    maxSignal);
            }
            AppendStartupTrace(traceLine);
        };

        const auto logWallPoolAndProbe = [&](const char* label, float targetDistanceM) noexcept
        {
            float minDistanceM = INFINITY;
            float maxDistanceM = 0.0f;
            float minSignal = INFINITY;
            float maxSignal = 0.0f;
            uint16_t validCount = 0U;
            uint16_t withinTargetCount = 0U;
            uint16_t collapsedCount = 0U;
            uint16_t withinTargetCollapsedCount = 0U;
            float selectedSignals[MaxSamples] = {};
            float selectedDistances[MaxSamples] = {};
            float selectedErrors[MaxSamples] = {};
            for (uint16_t index = 0U; index < wallSampleCount; ++index)
            {
                const float signal = wallSamples[index];
                const float distanceM = wallDistanceSamples[index];
                if (!std::isfinite(signal) ||
                    signal < 0.0f ||
                    !std::isfinite(distanceM) ||
                    distanceM <= 0.0f)
                {
                    continue;
                }

                minDistanceM = (distanceM < minDistanceM) ? distanceM : minDistanceM;
                maxDistanceM = (distanceM > maxDistanceM) ? distanceM : maxDistanceM;
                minSignal = (signal < minSignal) ? signal : minSignal;
                maxSignal = (signal > maxSignal) ? signal : maxSignal;
                if (signal <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++collapsedCount;
                }
                const float errorM = std::fabs(distanceM - targetDistanceM);
                if (errorM <= Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM)
                {
                    ++withinTargetCount;
                    if (signal <= kCollapsedFrontSweepSignalEpsilon)
                    {
                        ++withinTargetCollapsedCount;
                    }
                }

                selectedSignals[validCount] = signal;
                selectedDistances[validCount] = distanceM;
                selectedErrors[validCount] = errorM;
                ++validCount;
            }

            for (uint16_t index = 1U; index < validCount; ++index)
            {
                const float signal = selectedSignals[index];
                const float distanceM = selectedDistances[index];
                const float errorM = selectedErrors[index];
                uint16_t insertIndex = index;
                while ((insertIndex > 0U) && (selectedErrors[insertIndex - 1U] > errorM))
                {
                    selectedSignals[insertIndex] = selectedSignals[insertIndex - 1U];
                    selectedDistances[insertIndex] = selectedDistances[insertIndex - 1U];
                    selectedErrors[insertIndex] = selectedErrors[insertIndex - 1U];
                    --insertIndex;
                }

                selectedSignals[insertIndex] = signal;
                selectedDistances[insertIndex] = distanceM;
                selectedErrors[insertIndex] = errorM;
            }

            const uint16_t retainedCount =
                (validCount < Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount) ?
                validCount :
                Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount;
            float retainedMinDistanceM = INFINITY;
            float retainedMaxDistanceM = 0.0f;
            float retainedMinSignal = INFINITY;
            float retainedMaxSignal = 0.0f;
            uint16_t retainedCollapsedCount = 0U;
            for (uint16_t index = 0U; index < retainedCount; ++index)
            {
                retainedMinDistanceM = (selectedDistances[index] < retainedMinDistanceM) ? selectedDistances[index] : retainedMinDistanceM;
                retainedMaxDistanceM = (selectedDistances[index] > retainedMaxDistanceM) ? selectedDistances[index] : retainedMaxDistanceM;
                retainedMinSignal = (selectedSignals[index] < retainedMinSignal) ? selectedSignals[index] : retainedMinSignal;
                retainedMaxSignal = (selectedSignals[index] > retainedMaxSignal) ? selectedSignals[index] : retainedMaxSignal;
                if (selectedSignals[index] <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++retainedCollapsedCount;
                }
            }

            char traceLine[384] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_match_probe,sensor=%s,label=%s,target_m=%.4f,valid=%u,collapsed=%u,within=%u,within_collapsed=%u,retain=%u,retain_collapsed=%u,nearest_err=%.4f,last_err=%.4f,dist_min=%.4f,dist_max=%.4f,sel_dist_min=%.4f,sel_dist_max=%.4f,sel_sig_min=%.6f,sel_sig_max=%.6f",
                WallSensorIdName(sensorId),
                (label != nullptr) ? label : "unknown",
                targetDistanceM,
                static_cast<unsigned>(validCount),
                static_cast<unsigned>(collapsedCount),
                static_cast<unsigned>(withinTargetCount),
                static_cast<unsigned>(withinTargetCollapsedCount),
                static_cast<unsigned>(retainedCount),
                static_cast<unsigned>(retainedCollapsedCount),
                (validCount > 0U) ? selectedErrors[0U] : 0.0f,
                (retainedCount > 0U) ? selectedErrors[retainedCount - 1U] : 0.0f,
                std::isfinite(minDistanceM) ? minDistanceM : 0.0f,
                maxDistanceM,
                std::isfinite(retainedMinDistanceM) ? retainedMinDistanceM : 0.0f,
                retainedMaxDistanceM,
                std::isfinite(retainedMinSignal) ? retainedMinSignal : 0.0f,
                retainedMaxSignal);
            AppendStartupTrace(traceLine);

            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_wall_pool,sensor=%s,label=%s,count=%u,collapsed=%u,dist_min=%.4f,dist_max=%.4f,sig_min=%.6f,sig_max=%.6f",
                WallSensorIdName(sensorId),
                (label != nullptr) ? label : "unknown",
                static_cast<unsigned>(wallSampleCount),
                static_cast<unsigned>(collapsedCount),
                std::isfinite(minDistanceM) ? minDistanceM : 0.0f,
                maxDistanceM,
                std::isfinite(minSignal) ? minSignal : 0.0f,
                maxSignal);
            AppendStartupTrace(traceLine);
        };

        RobustSignalBand openBand{};
        const bool haveOpenBand = MazeMap::TryComputeRobustSignalBandFromSamples(
            openSamples,
            openSampleCount,
            Config::kWallCalibrationScaledMadMultiplier,
            openBand.median,
            openBand.low,
            openBand.high);
        logOpenPool(haveOpenBand ? &openBand : nullptr);
        logWallPoolAndProbe("on", onTargetDistanceM);
        RobustSignalBand wallBand{};
        const bool haveWallBand = MazeMap::TryComputeRobustSignalBandFromSamples(
            wallSamples,
            wallSampleCount,
            Config::kWallCalibrationScaledMadMultiplier,
            wallBand.median,
            wallBand.low,
            wallBand.high);
        RobustSignalBand onDistanceBand{};
        const bool haveOnDistanceBand = MazeMap::TryComputeRobustDistanceMatchedSignalBandFromSamples(
            wallSamples,
            wallDistanceSamples,
            wallSampleCount,
            onTargetDistanceM,
            Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMinSamples,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM,
            Config::kWallCalibrationScaledMadMultiplier,
            onDistanceBand.median,
            onDistanceBand.low,
            onDistanceBand.high);
        logWallPoolAndProbe("off", offTargetDistanceM);
        RobustSignalBand offDistanceBand{};
        const bool haveOffDistanceBand = MazeMap::TryComputeRobustDistanceMatchedSignalBandFromSamples(
            wallSamples,
            wallDistanceSamples,
            wallSampleCount,
            offTargetDistanceM,
            Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMinSamples,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM,
            Config::kWallCalibrationScaledMadMultiplier,
            offDistanceBand.median,
            offDistanceBand.low,
            offDistanceBand.high);
        if (!haveOpenBand ||
            !haveOnDistanceBand ||
            !haveOffDistanceBand ||
            !std::isfinite(openBand.median) ||
            !std::isfinite(openBand.low) ||
            !std::isfinite(openBand.high) ||
            !std::isfinite(onDistanceBand.median) ||
            !std::isfinite(onDistanceBand.low) ||
            !std::isfinite(onDistanceBand.high) ||
            !std::isfinite(offDistanceBand.median) ||
            !std::isfinite(offDistanceBand.low) ||
            !std::isfinite(offDistanceBand.high) ||
            openBand.median < 0.0f ||
            openBand.low < 0.0f ||
            openBand.high < openBand.low ||
            onDistanceBand.median <= 0.0f ||
            onDistanceBand.low <= 0.0f ||
            onDistanceBand.high < onDistanceBand.low ||
            offDistanceBand.median <= 0.0f ||
            offDistanceBand.low <= 0.0f ||
            offDistanceBand.high < offDistanceBand.low)
        {
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_distance_match_unavailable,sensor=%s,on_m=%.4f,off_m=%.4f,open=%u,on=%u,off=%u",
                WallSensorIdName(sensorId),
                onTargetDistanceM,
                offTargetDistanceM,
                haveOpenBand ? 1U : 0U,
                haveOnDistanceBand ? 1U : 0U,
                haveOffDistanceBand ? 1U : 0U);
            AppendStartupTrace(traceLine);

            if (haveOpenBand &&
                haveWallBand &&
                std::isfinite(openBand.median) &&
                std::isfinite(openBand.low) &&
                std::isfinite(openBand.high) &&
                std::isfinite(wallBand.median) &&
                std::isfinite(wallBand.low) &&
                std::isfinite(wallBand.high) &&
                openBand.median >= 0.0f &&
                openBand.low >= 0.0f &&
                openBand.high >= openBand.low &&
                wallBand.median > 0.0f &&
                wallBand.low > 0.0f &&
                wallBand.high >= wallBand.low)
            {
                float wallBucketBaseline = 0.0f;
                float wallBucketOnRiseThreshold = 0.0f;
                float wallBucketOffRiseThreshold = 0.0f;
                if (MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
                        openBand.low,
                        openBand.high,
                        wallBand.low,
                        wallBand.high,
                        Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
                        Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold,
                        wallBucketBaseline))
                {
                    gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(sensorId, openBand.median);
                    gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(sensorId, openBand.low, openBand.high);
                    gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
                        sensorId,
                        wallBand.median,
                        wallBand.low,
                        wallBand.high);
                    gWallDistanceCalibration.SetFrontDirectRiseThresholds(
                        sensorId,
                        wallBucketBaseline,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold);
                    snprintf(
                        traceLine,
                        sizeof(traceLine),
                        "startup_front_sweep_wall_bucket_match,sensor=%s,baseline=%.6f,wall_low=%.6f,wall_high=%.6f,on_rise=%.6f,off_rise=%.6f",
                        WallSensorIdName(sensorId),
                        wallBucketBaseline,
                        wallBand.low,
                        wallBand.high,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold);
                    AppendStartupTrace(traceLine);
                    return true;
                }
            }

            return false;
        }

        float signalBaseline = openBand.high;
        float onSignal = onDistanceBand.low;
        float offSignal = offDistanceBand.high;
        if (!(std::isfinite(signalBaseline) &&
            std::isfinite(onSignal) &&
            std::isfinite(offSignal) &&
            signalBaseline >= 0.0f &&
            onSignal > signalBaseline &&
            offSignal > signalBaseline &&
            offSignal < onSignal))
        {
            signalBaseline = openBand.median;
            onSignal = onDistanceBand.median;
            offSignal = offDistanceBand.median;
            if (!(std::isfinite(signalBaseline) &&
                std::isfinite(onSignal) &&
                std::isfinite(offSignal) &&
                signalBaseline >= 0.0f &&
                onSignal > signalBaseline &&
                offSignal > signalBaseline &&
                offSignal < onSignal))
            {
                return false;
            }

            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_distance_match_using_medians,sensor=%s,open_hi=%.6f,on_lo=%.6f,off_hi=%.6f",
                WallSensorIdName(sensorId),
                openBand.high,
                onDistanceBand.low,
                offDistanceBand.high);
            AppendStartupTrace(traceLine);
        }

        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(sensorId, openBand.median);
        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(sensorId, openBand.low, openBand.high);
        gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
            sensorId,
            onDistanceBand.median,
            onDistanceBand.low,
            onDistanceBand.high);
        gWallDistanceCalibration.SetFrontDirectRiseThresholds(
            sensorId,
            signalBaseline,
            onSignal - signalBaseline,
            offSignal - signalBaseline);
        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_distance_match,sensor=%s,on_m=%.4f,off_m=%.4f,baseline=%.6f,on_signal=%.6f,off_signal=%.6f",
            WallSensorIdName(sensorId),
            onTargetDistanceM,
            offTargetDistanceM,
            signalBaseline,
            onSignal,
            offSignal);
        AppendStartupTrace(traceLine);
        return true;
    }

    LoopController::ControlVector FrontCalibrationSweepLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& sweep = *static_cast<FrontCalibrationSweepLoopState*>(rawState);

        const PoseEstimate& pose = _drive.GetPose();
        const float deltaYawRad = WrapAngleRad(pose.yawRad - sweep.previousYawRad);
        sweep.previousYawRad = pose.yawRad;
        sweep.accumulatedSweepAngleRad += (std::max)(0.0f, deltaYawRad);
        sweep.accumulatedSweepAngleRad = (std::min)(sweep.accumulatedSweepAngleRad, sweep.targetSweepAngleRad);

        if ((sweep.accumulatedSweepAngleRad - sweep.lastStoredSweepAngleRad) >= sweep.captureStepRad)
        {
            RawWallSensorSample frontLeftSample{};
            RawWallSensorSample frontRightSample{};
            float frontLeftWallDistanceM = NAN;
            float frontRightWallDistanceM = NAN;
            SampleWallSensorPairRaw(
                WallSensorId::FrontLeft,
                _speedVehicle.FrontLeft,
                WallSensorId::FrontRight,
                _speedVehicle.FrontRight,
                frontLeftSample,
                frontRightSample);
            (void)TryComputeNearestStartCellWallDistanceM(pose, _speedVehicle.FrontLeft, frontLeftWallDistanceM);
            (void)TryComputeNearestStartCellWallDistanceM(pose, _speedVehicle.FrontRight, frontRightWallDistanceM);
            sweep.sweepSamples.Push(
                MazeMap::ClassifyFrontCalibrationSpinHeadingFromNorth(
                    pose.yawRad,
                    Config::kStartupWallCalibrationFrontNorthOpenHalfWidthRad,
                    Config::kStartupWallCalibrationFrontWallMinEastOfNorthRad,
                    Config::kStartupWallCalibrationFrontWallMaxEastOfNorthRad),
                frontLeftSample.differentialLight,
                frontRightSample.differentialLight,
                frontLeftWallDistanceM,
                frontRightWallDistanceM);
            sweep.lastStoredSweepAngleRad = sweep.accumulatedSweepAngleRad;
        }

        const float remainingRad = sweep.targetSweepAngleRad - sweep.accumulatedSweepAngleRad;
        if (MazeMap::IsInPlaceTurnComplete(remainingRad, pose.angularSpeedRadps, sweep.turnProfile))
        {
            if (!BeginSharedHoldPhase(
                    sweep.settleHold,
                    Config::kStartupWallCalibrationSettleMs,
                    true,
                    nullptr,
                    nullptr,
                    services))
            {
                return FaultLoopPhase(services, "Failed to begin startup front-sweep settle hold");
            }
            return LoopController::ControlVector::Brake;
        }
        if (!sweep.durationLogged && static_cast<long>(sweep.expectedCompletionDeadlineMs - millis()) <= 0)
        {
            sweep.durationLogged = true;
            AppendStartupTrace("startup_wall_calibration:front_sweep_elapsed_budget_exceeded");
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                remainingRad,
                pose.angularSpeedRadps,
                sweep.turnProfile,
                angularCommandRadps))
        {
            return FaultLoopPhase(services, "Startup front calibration sweep profile became invalid");
        }

        return _drive.PointControlVector(
            0.0f,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    bool CaptureAndStoreFrontCalibrationSweep(const MotionLimits& limits, bool& storedBands)
    {
        storedBands = false;
        FrontCalibrationSweepLoopState sweep{};
        sweep.limits = limits;
        sweep.turnProfile = BuildSharedInPlaceTurnProfile(limits);
        sweep.targetSweepAngleRad =
            static_cast<float>(Config::kStartupWallCalibrationFrontSpinTurnCount) * TWO_PI_F;
        sweep.captureStepRad = Config::kStartupWallCalibrationFrontSpinCaptureStepRad;
        sweep.lastStoredSweepAngleRad = -sweep.captureStepRad;
        sweep.previousYawRad = _drive.GetPose().yawRad;
        sweep.expectedCompletionDeadlineMs =
            millis() +
            static_cast<unsigned long>(
                2500.0f +
                (1000.0f * sweep.targetSweepAngleRad / (std::max)(0.25f, limits.maxAngularSpeedRadps)));
        sweep.storedBands = &storedBands;

        if (!RunLoopSession(&sweep, &Implementation::FrontCalibrationSweepLoopTick))
        {
            return false;
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_samples,fl_open=%u,fl_wall=%u,fr_open=%u,fr_wall=%u",
            static_cast<unsigned>(sweep.sweepSamples.frontLeftOpenCount),
            static_cast<unsigned>(sweep.sweepSamples.frontLeftWallCount),
            static_cast<unsigned>(sweep.sweepSamples.frontRightOpenCount),
            static_cast<unsigned>(sweep.sweepSamples.frontRightWallCount));
        AppendStartupTrace(traceLine);

        const bool storedFrontLeftBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontLeft,
            sweep.sweepSamples.frontLeftOpenSamples,
            sweep.sweepSamples.frontLeftOpenCount,
            sweep.sweepSamples.frontLeftWallSamples,
            sweep.sweepSamples.frontLeftWallDistanceSamples,
            sweep.sweepSamples.frontLeftWallCount);
        const bool storedFrontRightBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontRight,
            sweep.sweepSamples.frontRightOpenSamples,
            sweep.sweepSamples.frontRightOpenCount,
            sweep.sweepSamples.frontRightWallSamples,
            sweep.sweepSamples.frontRightWallDistanceSamples,
            sweep.sweepSamples.frontRightWallCount);

        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_bands,fl=%u,fr=%u",
            storedFrontLeftBands ? 1U : 0U,
            storedFrontRightBands ? 1U : 0U);
        AppendStartupTrace(traceLine);
        storedBands = storedFrontLeftBands && storedFrontRightBands;
        return true;
    }

    bool CaptureAndStoreFrontOpenBaselineAtCurrentPose()
    {
        WallSensorCalibrationCapture frontLeftCapture{};
        WallSensorCalibrationCapture frontRightCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::FrontLeft,
            _speedVehicle.FrontLeft,
            WallSensorId::FrontRight,
            _speedVehicle.FrontRight,
            frontLeftCapture,
            frontRightCapture);

        const auto storeBaseline = [&](WallSensorId sensorId, const WallSensorCalibrationCapture& capture) -> bool
        {
            if (!(std::isfinite(capture.input.differentialLight) &&
                capture.input.differentialLight >= 0.0f &&
                capture.haveDifferentialLightBand &&
                std::isfinite(capture.differentialLightBand.low) &&
                std::isfinite(capture.differentialLightBand.high) &&
                capture.differentialLightBand.low >= 0.0f &&
                capture.differentialLightBand.high >= capture.differentialLightBand.low))
            {
                return false;
            }

            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(
                sensorId,
                capture.input.differentialLight);
            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(
                sensorId,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_open_baseline,sensor=%s,median=%.6f,low=%.6f,high=%.6f",
                WallSensorIdName(sensorId),
                capture.input.differentialLight,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
            AppendStartupTrace(traceLine);
            return true;
        };

        return
            storeBaseline(WallSensorId::FrontLeft, frontLeftCapture) &&
            storeBaseline(WallSensorId::FrontRight, frontRightCapture);
    }

    // WARNING: Keep this procedure aligned with the validated hardware sequence unless it is re-proven on the robot.
    // The startup wall calibration must remain:
    // 1. start north with the rear touching the south wall
    // 2. complete the stationary settle
    // 3. drive forward to the start-cell center
    // 4. face west and calibrate the left side sensor against the south-wall-referenced distance
    // 5. face east and calibrate the right side sensor against the south-wall-referenced distance
    // 6. reseat to the start-cell center facing north and capture the per-run front open-scene baseline used to
    //    adapt the persisted dark-room characterization to the current ambient
    // 7. stay in the start-cell center facing north and exit
    bool RunStartupWallCalibration()
    {
        if (!EmitMissionControllerLineOrFail("Startup wall calibration"))
        {
            return false;
        }
        gWallDistanceCalibration.Clear();
        _hasWallTouchStandoffEstimate = false;
        SeedStartupWallCalibrationPoseFromSouthWall();
        if (!WaitForMissionStartupStationaryHold())
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:begin");

        const float targetCenterXM = 0.5f * Config::kCellSizeM;
        const float targetCenterYM = 0.5f * Config::kCellSizeM;
        char geometryTraceLine[256] = {};
        snprintf(
            geometryTraceLine,
            sizeof(geometryTraceLine),
            "startup_cal_geometry:cell_pitch=%.4f,cell_clear=%.4f,center_x=%.4f,center_y=%.4f",
            Config::kCellSizeM,
            Config::kCellClearSpanM,
            targetCenterXM,
            targetCenterYM);
        AppendStartupTrace(geometryTraceLine);

        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        AppendStartupTrace("startup_wall_calibration:settle_at_south_wall_start");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:move_forward_to_center");
        if (!DriveCalibrationPoseToKnownY(targetCenterYM, centeringLimits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_after_forward_move");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        float actualDistanceM = 0.0f;
        float sideDistanceSumM = 0.0f;
        uint8_t sideDistanceCount = 0U;

        AppendStartupTrace("startup_wall_calibration:rotate_west_for_left_side_sample");
        if (!RotateCalibrationTo(MazeMap::Left, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_left_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_left_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideLeft, _speedVehicle.SideLeft, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture rightSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideRight, _speedVehicle.SideRight);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            rightSideBaselineCapture.input.differentialLight);
        if (rightSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideRight,
                rightSideBaselineCapture.differentialLightBand.low,
                rightSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:rotate_east_for_right_side_sample");
        if (!RotateCalibrationTo(MazeMap::Right, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_right_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_right_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideRight, _speedVehicle.SideRight, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture leftSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideLeft, _speedVehicle.SideLeft);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            leftSideBaselineCapture.input.differentialLight);
        if (leftSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideLeft,
                leftSideBaselineCapture.differentialLightBand.low,
                leftSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:reseat_before_front_baseline");
        if (!ReseatMissionStartPoseWithPhasePrefix("startup_front_baseline", Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_front_open_baseline");
        if (!CaptureAndStoreFrontOpenBaselineAtCurrentPose())
        {
            return false;
        }

        if (sideDistanceCount > 0U)
        {
            gWallDistanceCalibration.SetExpectedSideWallDistanceM(sideDistanceSumM / static_cast<float>(sideDistanceCount));
        }

        AppendStartupTrace("startup_wall_calibration:complete");
        AppendStartupCalibrationStateTrace("startup_complete");
        SnapToStartPose();
        return HoldPosition(Config::kStartupWallCalibrationSettleMs);
    }

    bool Initialize(const char* startupTraceLine, const char* banner, bool observeCurrentCellAfterInit)
    {
        if (!_runtime.RegisterModeFaultHandler(&Implementation::HandleRuntimeFault, this, _activeModeFaultSource))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace(
            (startupTraceLine != nullptr && startupTraceLine[0] != '\0') ?
                startupTraceLine :
                "mode:mission");
        if (!OpenMissionTextLog())
        {
            AppendStartupTrace("initialize:logging_txt_open_failed");
            DisableMissionTextLogging("initialize:mission_text_log_unavailable");
        }
        if (!EmitMissionControllerLine(banner))
        {
            AppendStartupTrace("initialize:banner_log_failed");
            return false;
        }
        AppendStartupTrace("initialize:setup_hardware_ok");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        AppendStartupTrace("initialize:drive_ok");
        if (!_sensors.Begin())
        {
            return Fail("Sensor init failed");
        }
        AppendStartupTrace("initialize:sensors_ok");

        if (!RunStartupWallCalibration())
        {
            return false;
        }
        AppendStartupTrace("initialize:startup_wall_calibration_ok");
        LoadPersistedFrontWallCharacterization();

        if (observeCurrentCellAfterInit && !ObserveCurrentCell())
        {
            return false;
        }
        if (observeCurrentCellAfterInit)
        {
            AppendStartupTrace("initialize:observed_current_cell");
        }

        return true;
    }

    bool BeginTelemetryPhase(const char* name)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }
        ++_telemetryPhaseId;
        if (_runtime.WriteTextLogPhase(_telemetryPhaseId, micros(), name))
        {
            return true;
        }
        return Fail("Failed to write maneuver test phase marker");
    }

    bool LogWallCalibrationMetadata()
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char line[128] = {};
        snprintf(
            line,
            sizeof(line),
            "calibration_average_samples,%u",
            static_cast<unsigned>(Config::kWallCalibrationAverageSampleCount));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "detection_window_cycles,%u",
            static_cast<unsigned>(Config::kWallDetectionAverageWindowCycles));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_turn_count,%u",
            static_cast<unsigned>(Config::kStartupWallCalibrationFrontSpinTurnCount));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_capture_step_deg,%.1f",
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontSpinCaptureStepRad);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_heading_bands_deg,%.1f,%.1f,%.1f",
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontNorthOpenHalfWidthRad,
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontWallMinEastOfNorthRad,
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontWallMaxEastOfNorthRad);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "expected_side_distance_m,%.6f",
            gWallDistanceCalibration.GetExpectedSideWallDistanceM());
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "configured_touch_standoff_m,%.6f",
            Config::kWallTouchContactStandoffM);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        if (_hasWallTouchStandoffEstimate)
        {
            snprintf(
                line,
                sizeof(line),
                "estimated_touch_standoff_m,%.6f",
                _lastWallTouchStandoffEstimateM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        if (gWallDistanceCalibration.TryComputeSideWallDistanceThresholds(
                Config::kSideWallDistanceLatchFractionOfCalibration,
                Config::kSideWallDistanceReleaseFractionOfCalibration,
                sideWallOnThresholdM,
                sideWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_thresholds_m,%.6f,%.6f",
                sideWallOnThresholdM,
                sideWallOffThresholdM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLight = 0.0f;
        float sideRightReferenceDifferentialLight = 0.0f;
        const bool haveSideLeftReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLight);
        const bool haveSideRightReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLight);
        if (haveSideLeftReferenceDifferentialLight || haveSideRightReferenceDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff,%.6f,%.6f",
                sideLeftReferenceDifferentialLight,
                sideRightReferenceDifferentialLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLightLow = 0.0f;
        float sideLeftReferenceDifferentialLightHigh = 0.0f;
        float sideRightReferenceDifferentialLightLow = 0.0f;
        float sideRightReferenceDifferentialLightHigh = 0.0f;
        const bool haveSideLeftReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLightLow,
            sideLeftReferenceDifferentialLightHigh);
        const bool haveSideRightReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLightLow,
            sideRightReferenceDifferentialLightHigh);
        if (haveSideLeftReferenceDifferentialLightBand || haveSideRightReferenceDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftReferenceDifferentialLightLow,
                sideLeftReferenceDifferentialLightHigh,
                sideRightReferenceDifferentialLightLow,
                sideRightReferenceDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDistanceM = 0.0f;
        float sideRightReferenceDistanceM = 0.0f;
        const bool haveSideLeftReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideLeft,
            sideLeftReferenceDistanceM);
        const bool haveSideRightReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideRight,
            sideRightReferenceDistanceM);
        if (haveSideLeftReferenceDistanceM || haveSideRightReferenceDistanceM)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_distance_m,%.6f,%.6f",
                sideLeftReferenceDistanceM,
                sideRightReferenceDistanceM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLight = 0.0f;
        float sideRightBaselineDifferentialLight = 0.0f;
        const bool haveSideLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLight);
        const bool haveSideRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLight);
        if (haveSideLeftBaselineDifferentialLight || haveSideRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff,%.6f,%.6f",
                sideLeftBaselineDifferentialLight,
                sideRightBaselineDifferentialLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLightLow = 0.0f;
        float sideLeftBaselineDifferentialLightHigh = 0.0f;
        float sideRightBaselineDifferentialLightLow = 0.0f;
        float sideRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveSideLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLightLow,
            sideLeftBaselineDifferentialLightHigh);
        const bool haveSideRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLightLow,
            sideRightBaselineDifferentialLightHigh);
        if (haveSideLeftBaselineDifferentialLightBand || haveSideRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftBaselineDifferentialLightLow,
                sideLeftBaselineDifferentialLightHigh,
                sideRightBaselineDifferentialLightLow,
                sideRightBaselineDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftOnMeasuredThreshold = 0.0f;
        float sideLeftOffMeasuredThreshold = 0.0f;
        float sideRightOnMeasuredThreshold = 0.0f;
        float sideRightOffMeasuredThreshold = 0.0f;
        float sideLeftSignalBaseline = 0.0f;
        float sideRightSignalBaseline = 0.0f;
        const bool haveSideLeftMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideLeft,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideLeftOnMeasuredThreshold,
            sideLeftOffMeasuredThreshold,
            sideLeftSignalBaseline);
        const bool haveSideRightMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideRight,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideRightOnMeasuredThreshold,
            sideRightOffMeasuredThreshold,
            sideRightSignalBaseline);
        if (haveSideLeftMeasuredThreshold || haveSideRightMeasuredThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                sideLeftOnMeasuredThreshold,
                sideLeftOffMeasuredThreshold,
                sideRightOnMeasuredThreshold,
                sideRightOffMeasuredThreshold);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontWallOnThresholdM = 0.0f;
        float frontWallOffThresholdM = 0.0f;
        if (gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
                _speedVehicle,
                Config::kFrontWallReleaseHysteresisM,
                frontWallOnThresholdM,
                frontWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_thresholds_m,%.6f,%.6f",
                frontWallOnThresholdM,
                frontWallOffThresholdM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftOnMeasuredThreshold = 0.0f;
        float frontLeftOffMeasuredThreshold = 0.0f;
        float frontLeftSignalBaseline = 0.0f;
        float frontRightOnMeasuredThreshold = 0.0f;
        float frontRightOffMeasuredThreshold = 0.0f;
        float frontRightSignalBaseline = 0.0f;
        float frontLeftReferenceAmbientLight = 0.0f;
        float frontRightReferenceAmbientLight = 0.0f;
        const bool haveFrontLeftAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontLeft,
            frontLeftReferenceAmbientLight);
        const bool haveFrontRightAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontRight,
            frontRightReferenceAmbientLight);
        const bool haveFrontLeftThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontLeft,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontLeftAmbient ? frontLeftReferenceAmbientLight : NAN,
            frontLeftOnMeasuredThreshold,
            frontLeftOffMeasuredThreshold,
            frontLeftSignalBaseline);
        const bool haveFrontRightThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontRight,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontRightAmbient ? frontRightReferenceAmbientLight : NAN,
            frontRightOnMeasuredThreshold,
            frontRightOffMeasuredThreshold,
            frontRightSignalBaseline);
        if (haveFrontLeftAmbient || haveFrontRightAmbient)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_reference_ambient,%.6f,%.6f",
                frontLeftReferenceAmbientLight,
                frontRightReferenceAmbientLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        if (haveFrontLeftThreshold || haveFrontRightThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                frontLeftOnMeasuredThreshold,
                frontLeftOffMeasuredThreshold,
                frontRightOnMeasuredThreshold,
                frontRightOffMeasuredThreshold);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        const bool haveFrontLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontLeft,
            frontLeftSignalBaseline);
        const bool haveFrontRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontRight,
            frontRightSignalBaseline);
        if (haveFrontLeftBaselineDifferentialLight || haveFrontRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff,%.6f,%.6f",
                frontLeftSignalBaseline,
                frontRightSignalBaseline);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftBaselineDifferentialLightLow = 0.0f;
        float frontLeftBaselineDifferentialLightHigh = 0.0f;
        float frontRightBaselineDifferentialLightLow = 0.0f;
        float frontRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftBaselineDifferentialLightLow,
            frontLeftBaselineDifferentialLightHigh);
        const bool haveFrontRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightBaselineDifferentialLightLow,
            frontRightBaselineDifferentialLightHigh);
        if (haveFrontLeftBaselineDifferentialLightBand || haveFrontRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftBaselineDifferentialLightLow,
                frontLeftBaselineDifferentialLightHigh,
                frontRightBaselineDifferentialLightLow,
                frontRightBaselineDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftWeakestDifferentialLightLow = 0.0f;
        float frontLeftWeakestDifferentialLightHigh = 0.0f;
        float frontRightWeakestDifferentialLightLow = 0.0f;
        float frontRightWeakestDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftWeakestDifferentialLightLow,
            frontLeftWeakestDifferentialLightHigh);
        const bool haveFrontRightWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightWeakestDifferentialLightLow,
            frontRightWeakestDifferentialLightHigh);
        if (haveFrontLeftWeakestDifferentialLightBand || haveFrontRightWeakestDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_weakest_calibration_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftWeakestDifferentialLightLow,
                frontLeftWeakestDifferentialLightHigh,
                frontRightWeakestDifferentialLightLow,
                frontRightWeakestDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        for (uint8_t sensorIndex = 0U; sensorIndex < static_cast<uint8_t>(WallSensorId::Count); ++sensorIndex)
        {
            const WallSensorId sensorId = static_cast<WallSensorId>(sensorIndex);
            const MazeMap::WallSensorCalibrationCurve& curve = gWallDistanceCalibration.GetCurve(sensorId);
            snprintf(
                line,
                sizeof(line),
                "%s,measurement,%s,count,%u",
                WallSensorIdName(sensorId),
                WallSensorCalibrationMeasurementName(sensorId),
                static_cast<unsigned>(curve.GetCount()));
            if (!WriteTelemetryEvent("wall_calibration_curve", line))
            {
                return Fail("Unable to write wall calibration curve metadata");
            }

            for (uint8_t pointIndex = 0U; pointIndex < curve.GetCount(); ++pointIndex)
            {
                const MazeMap::WallSensorCalibrationCurve::Point& point = curve.GetPoint(pointIndex);
                snprintf(
                    line,
                    sizeof(line),
                    "%s,%s,%u,%.6f,%.6f",
                    WallSensorIdName(sensorId),
                    WallSensorCalibrationMeasurementName(sensorId),
                    static_cast<unsigned>(pointIndex),
                    point.measuredValue,
                    point.actualDistanceM);
                if (!WriteTelemetryEvent("wall_calibration_point", line))
                {
                    return Fail("Unable to write wall calibration point metadata");
                }
            }
        }

        return true;
    }

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = Config::kControlPeriodUs;
        return options;
    }

    static LoopController::ControlVector ActiveLoopThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<Implementation*>(context);
        if ((self == nullptr) || (self->_activeLoopState == nullptr) || (self->_activeLoopTickFn == nullptr))
        {
            services.Fault("Mission loop callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        const bool shouldLogSample =
            (self->_activeLoopTickFn == &Implementation::ObservationCaptureLoopTick) ||
            (self->_activeLoopTickFn == &Implementation::WallTouchLoopTick) ||
            (self->_activeLoopTickFn == &Implementation::FrontCalibrationSweepLoopTick) ||
            (self->_activeLoopTickFn == &Implementation::StartupStationaryHoldLoopTick) ||
            (self->_activeLoopTickFn == &Implementation::SearchStraightLoopTick);
        if (shouldLogSample)
        {
            const bool stationary =
                (self->_activeLoopTickFn == &Implementation::ObservationCaptureLoopTick) ||
                (self->_activeLoopTickFn == &Implementation::StartupStationaryHoldLoopTick);
            if (!Implementation::ManeuverExecutorSampleHook(self, stationary, state))
            {
                services.Fault("Failed to write maneuver test sample");
                return LoopController::ControlVector::Brake;
            }
        }

        return (self->*self->_activeLoopTickFn)(self->_activeLoopState, loopEndTimeUs, state, services);
    }

    static LoopController::PauseDisposition InterRunServicePauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<Implementation*>(context);
        if (self == nullptr)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Mission inter-run service pause callback context was null");
        }

        return self->OnInterRunServicePauseGranted(pause);
    }

    LoopController::ModeCallbacks PrepareLoopContinuation(
        void* nextState,
        const ActiveLoopTickFn nextTickFn) noexcept
    {
        _activeLoopState = nextState;
        _activeLoopTickFn = nextTickFn;

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &Implementation::ActiveLoopThunk;
        callbacks.context = this;
        return callbacks;
    }

    void UpdateDirectionalLocation(const MazeMap::DirectionalLocation& location) noexcept
    {
        _currentDirectionalLocation = location;
        _currentDirection = location.GetDirection();
        _currentCell = static_cast<MazeMap::CellCoordinates>(location.GetLocation());
    }

    static bool ManeuverExecutorSampleHook(
        void* context,
        bool stationary,
        const LoopController::ModeState& state)
    {
        auto* const self = static_cast<Implementation*>(context);
        if (self == nullptr || !self->_telemetryLoggingEnabled)
        {
            return self != nullptr;
        }

        Runtime::PopulateDiagnosticLogRow(
            self->_telemetryLogRow,
            static_cast<std::uint32_t>(self->_telemetrySampleCount),
            static_cast<std::uint32_t>(self->_telemetryPhaseId),
            stationary,
            state,
            self->_drive);
        if (self->_runtime.LogUtilityDataRow(self->_telemetryLogRow))
        {
            ++self->_telemetrySampleCount;
            return true;
        }
        return self->Fail("Failed to write maneuver test sample");
    }

    static bool ManeuverExecutorQueueEntryBeginHook(
        void* context,
        std::uint16_t index,
        const MazeMap::ManeuverInstance& entry,
        const MazeMap::DirectionalLocation& location)
    {
        auto* const self = static_cast<Implementation*>(context);
        if (self == nullptr)
        {
            return false;
        }

        self->UpdateDirectionalLocation(location);
        const MazeMap::ManeuverCode code = entry.getCode();
        const float entrySpeed = entry.getEntrySpeed();
        const float exitSpeed = entry.getExitSpeed();
        char codeName[24] = {};
        FormatManeuverCodeName(code, codeName, sizeof(codeName));

        self->AppendMissionTraceFormatted(
            "mission_maneuver:begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
            static_cast<unsigned>(index),
            codeName,
            self->_currentCell.GetX(),
            self->_currentCell.GetY(),
            DirectionName(self->_currentDirection),
            entrySpeed,
            exitSpeed);
        return true;
    }

    static bool ManeuverExecutorQueueEntryCompleteHook(
        void* context,
        std::uint16_t index,
        const MazeMap::ManeuverInstance& entry,
        const MazeMap::DirectionalLocation& location)
    {
        auto* const self = static_cast<Implementation*>(context);
        if (self == nullptr)
        {
            return false;
        }

        self->UpdateDirectionalLocation(location);
        char codeName[24] = {};
        FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
        self->AppendMissionTraceFormatted(
            "mission_maneuver:end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
            static_cast<unsigned>(index),
            codeName,
            self->_currentCell.GetX(),
            self->_currentCell.GetY(),
            DirectionName(self->_currentDirection),
            self->_drive.GetPose().xMeters,
            self->_drive.GetPose().yMeters,
            RAD_TO_DEG_F * self->_drive.GetPose().yawRad);
        return true;
    }

    ManeuverExecutor::Hooks BuildManeuverExecutorHooks(const bool includeQueueHooks) noexcept
    {
        ManeuverExecutor::Hooks hooks{};
        hooks.context = this;
        hooks.onSample = &Implementation::ManeuverExecutorSampleHook;
        if (includeQueueHooks)
        {
            hooks.onQueueEntryBegin = &Implementation::ManeuverExecutorQueueEntryBeginHook;
            hooks.onQueueEntryComplete = &Implementation::ManeuverExecutorQueueEntryCompleteHook;
        }
        return hooks;
    }

    LoopController::ControlVector QueuedManeuverLaunchTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        auto& queuedManeuvers = *static_cast<QueuedManeuverLoopState*>(rawState);
        if (queuedManeuvers.queue == nullptr)
        {
            return FaultLoopPhase(services, "Queued maneuver session state was invalid");
        }
        if (!EmitMissionControllerFormattedOrFail(
                "Queued maneuvers: %u",
                static_cast<unsigned>(queuedManeuvers.queue->size())))
        {
            return FaultLoopPhase(services, "Failed to emit queued maneuver summary");
        }

        const LoopController::ModeCallbacks continuation =
            PrepareLoopContinuation(rawState, &Implementation::QueuedManeuverFinalHoldTick);
        if (!_runtime.ManeuverExecutorService().ProceedToManeuverExecutionRoutine(
                *queuedManeuvers.queue,
                queuedManeuvers.limits,
                queuedManeuvers.snapToExpectedLocation,
                _currentDirectionalLocation,
                continuation,
                services,
                BuildManeuverExecutorHooks(true)))
        {
            return FaultLoopPhase(services, "Failed to launch queued maneuver routine");
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector QueuedManeuverFinalHoldTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        auto& queuedManeuvers = *static_cast<QueuedManeuverLoopState*>(rawState);
        if ((queuedManeuvers.completionHoldPhaseName != nullptr) &&
            !BeginTelemetryPhase(queuedManeuvers.completionHoldPhaseName))
        {
            return FaultLoopPhase(services, "Failed to begin queued maneuver completion hold phase");
        }

        if (!BeginSharedHoldPhase(
                queuedManeuvers.completionHold,
                50U,
                true,
                rawState,
                &Implementation::SharedManeuverRoutineCompleteTick,
                services))
        {
            return FaultLoopPhase(services, "Failed to begin queued maneuver completion hold");
        }
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector SharedManeuverRoutineCompleteTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)rawState;
        (void)loopEndTimeUs;
        (void)state;
        return EndLoopPhase(services);
    }

    void TransitionLoopPhase(
        void* nextState,
        const ActiveLoopTickFn nextTickFn,
        LoopController::TickServices& services) noexcept
    {
        services.SetNextModeWorkCallbacks(PrepareLoopContinuation(nextState, nextTickFn));
    }

    LoopController::ControlVector EndLoopPhase(LoopController::TickServices& services) noexcept
    {
        services.RequestEndLoop();
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector FaultLoopPhase(
        LoopController::TickServices& services,
        const char* reason) noexcept
    {
        services.Fault(reason);
        return LoopController::ControlVector::Brake;
    }

    bool RunLoopSession(void* initialState, const ActiveLoopTickFn initialTickFn)
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &Implementation::ActiveLoopThunk;
        callbacks.context = this;
        _activeLoopState = initialState;
        _activeLoopTickFn = initialTickFn;

        const bool began = _loopController.BeginSession(BuildLoopOptions(), callbacks);
        if (!began)
        {
            _activeLoopState = nullptr;
            _activeLoopTickFn = nullptr;
            return Fail("Mission loop controller session could not start");
        }

        const LoopController::SessionResult result = _loopController.Run();
        _loopController.EndSession();
        _activeLoopState = nullptr;
        _activeLoopTickFn = nullptr;

        if (_faulted)
        {
            return false;
        }
        if (result.status != LoopController::SessionResult::Status::Completed)
        {
            return false;
        }

        return true;
    }

    LoopController::ControlVector SharedManeuverExecutorPhaseTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto& phase = *static_cast<SharedManeuverExecutorLoopState*>(rawState);
        auto& executor = _runtime.ManeuverExecutorService();
        const LoopController::ControlVector command =
            executor.DriveActivePhase(loopEndTimeUs, state, services);
        if (!executor.Active())
        {
            if (executor.ActivePhaseFaulted())
            {
                return LoopController::ControlVector::Brake;
            }

            if (phase.nextTickFn != nullptr)
            {
                TransitionLoopPhase(phase.nextState, phase.nextTickFn, services);
                return LoopController::ControlVector::Brake;
            }

            return EndLoopPhase(services);
        }

        return command;
    }

    void PrepareSharedManeuverExecutorContinuation(
        SharedManeuverExecutorLoopState& phase,
        void* nextState,
        const ActiveLoopTickFn nextTickFn) noexcept
    {
        phase = SharedManeuverExecutorLoopState{};
        phase.nextState = nextState;
        phase.nextTickFn = nextTickFn;
    }

    bool BeginSharedHoldPhase(
        SharedManeuverExecutorLoopState& phase,
        const std::uint16_t durationMs,
        const bool stationary,
        void* nextState,
        const ActiveLoopTickFn nextTickFn,
        LoopController::TickServices& services)
    {
        if (!_runtime.ManeuverExecutorService().BeginHoldPhase(
                durationMs,
                stationary,
                BuildManeuverExecutorHooks(false)))
        {
            return false;
        }

        PrepareSharedManeuverExecutorContinuation(phase, nextState, nextTickFn);
        TransitionLoopPhase(&phase, &Implementation::SharedManeuverExecutorPhaseTick, services);
        return true;
    }

    bool BeginSharedBrakedSettlePhase(
        SharedManeuverExecutorLoopState& phase,
        const char* timeoutMessage,
        const std::uint16_t stationaryHoldMs,
        const std::uint16_t timeoutMs,
        void* nextState,
        const ActiveLoopTickFn nextTickFn,
        LoopController::TickServices& services)
    {
        if (!_runtime.ManeuverExecutorService().BeginBrakedSettlePhase(
                timeoutMessage,
                stationaryHoldMs,
                timeoutMs,
                BuildManeuverExecutorHooks(false)))
        {
            return false;
        }

        PrepareSharedManeuverExecutorContinuation(phase, nextState, nextTickFn);
        TransitionLoopPhase(&phase, &Implementation::SharedManeuverExecutorPhaseTick, services);
        return true;
    }

    bool RunSharedManeuverExecutorPhase()
    {
        SharedManeuverExecutorLoopState phase{};
        const bool ok = RunLoopSession(&phase, &Implementation::SharedManeuverExecutorPhaseTick);
        if (!ok && _runtime.ManeuverExecutorService().Active())
        {
            _runtime.ManeuverExecutorService().CancelActivePhase();
        }
        return ok;
    }

    LoopController::PauseDisposition OnInterRunServicePauseGranted(
        const LoopController::PauseContext& pause)
    {
        (void)pause;

        if (!EmitMissionControllerLineOrFail("Install 34-35 jumper before lifting for tire service"))
        {
            return _faulted ?
                LoopController::PauseDisposition::Complete() :
                LoopController::PauseDisposition::StopByRuntime(
                    "Mission inter-run service install prompt failed");
        }

        _drive.Brake();
        while (!IsInterRunServiceJumperInstalled())
        {
            delay(Config::kInterRunServicePollMs);
        }

        if (!EmitMissionControllerLineOrFail("Service jumper detected; place robot back at start facing up and remove jumper"))
        {
            return _faulted ?
                LoopController::PauseDisposition::Complete() :
                LoopController::PauseDisposition::StopByRuntime(
                    "Mission inter-run service remove prompt failed");
        }

        while (IsInterRunServiceJumperInstalled())
        {
            _drive.Brake();
            delay(Config::kInterRunServicePollMs);
        }

        return LoopController::PauseDisposition::Complete();
    }

    LoopController::ControlVector InterRunServicePauseTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)rawState;
        (void)loopEndTimeUs;
        (void)state;

        LoopController::PauseRequest request{};
        request.onPauseGranted = &Implementation::InterRunServicePauseThunk;
        request.reason = "mission_inter_run_service";
        request.flushLogsBeforeGrant = true;
        services.RequestPause(request);
        return LoopController::ControlVector::Brake;
    }

    bool Fail(const char* message)
    {
        return _runtime.FailActiveMode(message);
    }

    void OnRuntimeFault(const char* message) noexcept
    {
        _faulted = true;
        AppendStartupCalibrationStateTrace("fault_state");
        char traceMessage[128] = {};
        snprintf(traceMessage, sizeof(traceMessage), "fault:%s", (message != nullptr) ? message : "unknown");
        AppendStartupTrace(traceMessage);
        (void)EmitMissionControllerFormatted("FAULT: %s", (message != nullptr) ? message : "unknown");
        if (_missionTextLoggingEnabled && !_missionMazeSnapshotWritten)
        {
            (void)WriteMissionMazeSnapshot("mission_fault");
        }
        if (_telemetryLoggingEnabled)
        {
            (void)WriteTelemetryEvent("fault", message);
            FlushTelemetryLog();
            CloseTelemetryLog();
            _telemetryLoggingEnabled = false;
        }
        FlushMissionTextLog();
    }

    bool HoldPosition(uint16_t durationMs, const char* phaseName = nullptr)
    {
        if (phaseName != nullptr && !BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        if (!_runtime.ManeuverExecutorService().BeginHoldPhase(
                durationMs,
                true,
                BuildManeuverExecutorHooks(false)))
        {
            return Fail("Failed to begin shared hold phase");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool HoldBrakedUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        if (!_runtime.ManeuverExecutorService().BeginBrakedSettlePhase(
                timeoutMessage,
                stationaryHoldMs,
                timeoutMs,
                BuildManeuverExecutorHooks(false)))
        {
            return Fail("Failed to begin shared braked settle phase");
        }

        return RunSharedManeuverExecutorPhase();
    }

    LoopController::ControlVector StartupStationaryHoldLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& hold = *static_cast<StartupStationaryHoldLoopState*>(rawState);

        const unsigned long nowMs = millis();
        if (!hold.stationaryWindowActive)
        {
            hold.stationaryStartMs = nowMs;
            hold.stationaryStartTelemetry = state.driveTelemetry;
            hold.stationaryWindowActive = true;
            return LoopController::ControlVector::Brake;
        }

        const bool stationary = MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            state.driveTelemetry.leftDistanceM - hold.stationaryStartTelemetry.leftDistanceM,
            state.driveTelemetry.rightDistanceM - hold.stationaryStartTelemetry.rightDistanceM,
            static_cast<float>(nowMs - hold.stationaryStartMs) * 1.0e-3f,
            state.sensors.gyroRadps,
            Config::kMissionStartupStationarySpeedThresholdMps,
            Config::kMissionStartupStationaryMaxAbsYawRateRadps);
        if (!stationary)
        {
            if ((nowMs - hold.lastResetTraceMs) >= 1000UL)
            {
                char traceLine[160];
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_stationary_hold:reset,left_dm=%.5f,right_dm=%.5f,gyro=%.5f",
                    state.driveTelemetry.leftDistanceM - hold.stationaryStartTelemetry.leftDistanceM,
                    state.driveTelemetry.rightDistanceM - hold.stationaryStartTelemetry.rightDistanceM,
                    state.sensors.gyroRadps);
                AppendStartupTrace(traceLine);
                hold.lastResetTraceMs = nowMs;
            }
            hold.stationaryStartMs = nowMs;
            hold.stationaryStartTelemetry = state.driveTelemetry;
            return LoopController::ControlVector::Brake;
        }

        if ((nowMs - hold.stationaryStartMs) >= Config::kMissionStartupStationaryHoldMs)
        {
            AppendStartupTrace("startup_stationary_hold:complete");
            return EndLoopPhase(services);
        }

        return LoopController::ControlVector::Brake;
    }

    bool WaitForMissionStartupStationaryHold()
    {
        if (!EmitMissionControllerLineOrFail("Waiting for 2 s stationary start"))
        {
            return false;
        }
        AppendStartupTrace("startup_stationary_hold:waiting");

        StartupStationaryHoldLoopState hold{};
        return RunLoopSession(&hold, &Implementation::StartupStationaryHoldLoopTick);
    }

    bool ExecuteReverseStraightProfile(
        float distanceM,
        const MotionLimits& limits,
        const Eigen::Vector2f* targetHeadingOverride = nullptr,
        const Eigen::Vector2f* targetPositionOverride = nullptr)
    {
        if (!(std::isfinite(distanceM) && distanceM > 0.0f))
        {
            return true;
        }

        if (!_runtime.ManeuverExecutorService().BeginReverseStraightPhase(
                distanceM,
                limits,
                BuildManeuverExecutorHooks(false),
                targetHeadingOverride,
                targetPositionOverride))
        {
            return Fail("Failed to begin shared reverse-straight execution");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool ExecuteQueuedManeuvers(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        bool snapToExpectedLocation,
        const char* completionHoldPhaseName = nullptr)
    {
        if (queue.empty())
        {
            return HoldPosition(50);
        }

        QueuedManeuverLoopState queuedManeuvers{};
        queuedManeuvers.queue = &queue;
        queuedManeuvers.limits = limits;
        queuedManeuvers.snapToExpectedLocation = snapToExpectedLocation;
        queuedManeuvers.completionHoldPhaseName = completionHoldPhaseName;
        return RunLoopSession(&queuedManeuvers, &Implementation::QueuedManeuverLaunchTick);
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, bool snapToExpectedLocation)
    {
        return ExecuteQueuedManeuvers(queue, FinalLimits(), snapToExpectedLocation, nullptr);
    }

    bool ObserveCellFromSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        const SensorSnapshot& snapshot,
        bool* outForwardWallCommittedFromUnknown = nullptr)
    {
        if (outForwardWallCommittedFromUnknown != nullptr)
        {
            *outForwardWallCommittedFromUnknown = false;
        }

        MazeMap::WallState knownWallState = MazeMap::WallState::Unknown;
        if (MazeMap::TryGetKnownMissionStartWallState(observedCell, MazeMap::Up, knownWallState))
        {
            PrimeKnownMissionStartCell();
            AppendStartupTrace("observe_current_cell:used_known_start_cell_topology");
            return true;
        }

        MazeMap::Cell& cell = _maze[observedCell];
        const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
        const MazeMap::Direction leftDirection = observedDirection + MazeMap::Left90;
        const MazeMap::Direction rightDirection = observedDirection + MazeMap::Right90;
        const bool forwardUnknown = cell.GetWall(forwardDirection) == MazeMap::WallState::Unknown;
        const bool leftUnknown = cell.GetWall(leftDirection) == MazeMap::WallState::Unknown;
        const bool rightUnknown = cell.GetWall(rightDirection) == MazeMap::WallState::Unknown;
        if (!(forwardUnknown || leftUnknown || rightUnknown))
        {
            AppendStartupTrace("observe_current_cell:skipped_known_walls");
            return true;
        }

        const MazeMap::WallBeliefConfig beliefConfig = BuildWallBeliefConfig();
        const uint32_t beliefTick = millis();
        const auto applyBeliefQualifiedObservation =
            [&](const char* relativeDirectionName,
                MazeMap::Direction absoluteDirection,
                MazeMap::WallState observedState,
                const char* sensorSource,
                const char* sensorMode,
                float primaryDistanceM,
                float secondaryDistanceM,
                bool primaryDetected,
                bool secondaryDetected,
                MazeMap::WallBeliefUpdate* outBeliefUpdate = nullptr) -> bool
        {
            const MazeMap::WallBeliefUpdate beliefUpdate =
                _wallBeliefMap.ApplyObservation(
                    observedCell,
                    absoluteDirection,
                    (observedState == MazeMap::Wall) ?
                        MazeMap::WallSampleClassification::WallHit :
                        MazeMap::WallSampleClassification::WallMiss,
                    beliefConfig,
                    beliefTick);
            if (beliefUpdate.valid && beliefUpdate.hardState != MazeMap::Unknown)
            {
                _maze.SetWall(cell, absoluteDirection, beliefUpdate.hardState);
            }
            if (outBeliefUpdate != nullptr)
            {
                *outBeliefUpdate = beliefUpdate;
            }

            return LogWallObservationDecision(
                observedCell,
                relativeDirectionName,
                absoluteDirection,
                observedState,
                sensorSource,
                sensorMode,
                primaryDistanceM,
                secondaryDistanceM,
                primaryDetected,
                secondaryDetected,
                snapshot,
                beliefUpdate.valid ? beliefUpdate.hardState : MazeMap::WallState::Unknown,
                beliefUpdate.valid ? beliefUpdate.logOdds : 0.0f);
        };

        if (forwardUnknown)
        {
            if (!snapshot.frontWallObservationValid)
            {
                AppendMissionTraceFormatted(
                    "mission_front_wall_update_skipped,cell=(%d,%d),abs=%s,reason=insufficient_evidence",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(forwardDirection));
            }
            else
            {
                MazeMap::WallState observedState = snapshot.frontWall ? MazeMap::Wall : MazeMap::NoWall;
                const char* sensorSource = FrontObservationSourceName(snapshot);
                const char* sensorMode = FrontObservationModeName(snapshot);
                MazeMap::WallBeliefUpdate forwardBeliefUpdate{};
                if (!applyBeliefQualifiedObservation(
                        "forward",
                        forwardDirection,
                        observedState,
                        sensorSource,
                        sensorMode,
                        snapshot.frontLeftDistanceM,
                        snapshot.frontRightDistanceM,
                        snapshot.frontLeftWall,
                        snapshot.frontRightWall,
                        &forwardBeliefUpdate))
                {
                    return false;
                }
                if (outForwardWallCommittedFromUnknown != nullptr &&
                    observedState == MazeMap::Wall &&
                    forwardBeliefUpdate.valid &&
                    forwardBeliefUpdate.hardState == MazeMap::Wall)
                {
                    *outForwardWallCommittedFromUnknown = true;
                }
            }
        }
        if (leftUnknown)
        {
            if (snapshot.leftWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.leftWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                if (!applyBeliefQualifiedObservation(
                        "left",
                        leftDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideLeft),
                        nullptr,
                        snapshot.sideLeftDistanceM,
                        NAN,
                        snapshot.leftWallObservation,
                        false))
                {
                    return false;
                }
            }
            else
            {
                AppendMissionTraceFormatted(
                    "mission_side_wall_update_skipped,cell=(%d,%d),abs=%s,reason=%s",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(leftDirection),
                    snapshot.leftTransitionDetected ? "transition_ambiguous" : "outside_window_or_ambiguous");
            }
        }
        if (rightUnknown)
        {
            if (snapshot.rightWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.rightWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                if (!applyBeliefQualifiedObservation(
                        "right",
                        rightDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideRight),
                        nullptr,
                        snapshot.sideRightDistanceM,
                        NAN,
                        snapshot.rightWallObservation,
                        false))
                {
                    return false;
                }
            }
            else
            {
                AppendMissionTraceFormatted(
                    "mission_side_wall_update_skipped,cell=(%d,%d),abs=%s,reason=%s",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(rightDirection),
                    snapshot.rightTransitionDetected ? "transition_ambiguous" : "outside_window_or_ambiguous");
            }
        }
        return true;
    }

    bool ObserveCurrentCellFromSnapshot(const SensorSnapshot& snapshot)
    {
        return ObserveCellFromSnapshot(_currentCell, _currentDirection, snapshot);
    }

    bool ObserveCurrentCell()
    {
        if (!HoldPosition(Config::kObservationSettleMs))
        {
            return false;
        }

        SensorSnapshot snapshot{};
        if (!CaptureStationaryObservationSnapshot(_currentCell, _currentDirection, snapshot))
        {
            return false;
        }
        _drive.Brake();
        return ObserveCurrentCellFromSnapshot(snapshot);
    }

    bool HandleSearchWallMapUpdateStop(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        float projectedTravelM,
        uint16_t frontVoteCount,
        bool* outStoppedForReplan = nullptr)
    {
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = false;
        }

        _currentCell = observedCell;
        _currentDirection = observedDirection;

        _drive.Brake();

        MazeMap::Path<PATH_SIZE> replannedPath;
        _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, replannedPath);
        const MazeMap::SearchReplanResponse replan = MazeMap::PlanSearchReplanResponse(replannedPath, _currentDirection);
        AppendMissionTraceFormatted(
            "mission_observation_replan,cell=(%d,%d),abs=%s,travel_m=%.4f,front_votes=%u,path_size=%u,next_abs=%s,requires_turn=%u",
            _currentCell.GetX(),
            _currentCell.GetY(),
            DirectionName(_currentDirection),
            projectedTravelM,
            static_cast<unsigned>(frontVoteCount),
            static_cast<unsigned>(replannedPath.GetSize()),
            DirectionName(replan.nextDirection),
            replan.requiresTurn ? 1U : 0U);

        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
        {
            return false;
        }

        if (replan.requiresTurn && !OrientTo(replan.nextDirection, SearchLimits()))
        {
            return false;
        }

        _currentDirectionalLocation = MazeMap::DirectionalLocation(
            MazeMap::MazeLocation::CellCenter(_currentCell),
            _currentDirection);
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = true;
        }
        return true;
    }

    LoopController::ControlVector SearchStraightLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& search = *static_cast<SearchStraightLoopState*>(rawState);

        const PoseEstimate& livePose = _drive.GetPose();
        float projectedRemainingM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                livePose.xMeters,
                livePose.yMeters,
                search.targetXMeters,
                search.targetYMeters,
                search.targetHeading.x(),
                search.targetHeading.y(),
                projectedRemainingM))
        {
            return FaultLoopPhase(services, "Search straight target projection is invalid");
        }
        const float remainingM = (std::max)(0.0f, projectedRemainingM);
        const float projectedTravelM =
            (std::clamp)(search.distanceToTargetM - projectedRemainingM, 0.0f, search.distanceToTargetM);
        const float sideSensorForwardOffsetM =
            (std::max)(_speedVehicle.SideLeft.GetPosition().x(), _speedVehicle.SideRight.GetPosition().x());

        auto resetRollingObservationPlan = [&search]()
        {
            search.rollingObservationNextSampleIndex = 0U;
            search.rollingObservationPlanInitialized = false;
            search.rollingObservationSideResetTriggerTravelM = 0.0f;
            search.rollingObservationSideResetPending = false;
            memset(search.rollingObservationTriggerTravelM, 0, sizeof(search.rollingObservationTriggerTravelM));
            memset(search.rollingObservationSamples, 0, sizeof(search.rollingObservationSamples));
            for (std::uint8_t sampleIndex = 0U;
                sampleIndex < Config::kSearchRollingObservationSampleCount;
                ++sampleIndex)
            {
                search.rollingObservationFrontLeftCandidateDistanceM[sampleIndex] = NAN;
                search.rollingObservationFrontRightCandidateDistanceM[sampleIndex] = NAN;
            }
        };

        auto initializeRollingObservationPlan = [this, &search, sideSensorForwardOffsetM]() -> const char*
        {
            if (!search.observeWhileRolling || search.rollingObservationCount >= search.cellCount)
            {
                return nullptr;
            }
            if (search.rollingObservationPlanInitialized)
            {
                return nullptr;
            }

            const PoseEstimate startPose = _drive.GetPose();
            for (std::uint8_t sampleIndex = 0U;
                sampleIndex < Config::kSearchRollingObservationSampleCount;
                ++sampleIndex)
            {
                float targetObservationXMeters = 0.0f;
                float targetObservationYMeters = 0.0f;
                if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
                        search.nextRollingObservationCell,
                        search.direction,
                        Config::kCellSizeM,
                        Config::kMazeWallThicknessM,
                        sideSensorForwardOffsetM,
                        Config::kSideWallSegmentCenterFraction,
                        sampleIndex,
                        Config::kSearchRollingObservationSampleCount,
                        targetObservationXMeters,
                        targetObservationYMeters))
                {
                    return "Search straight rolling observation sample pose is invalid";
                }

                float triggerTravelM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                        startPose.xMeters,
                        startPose.yMeters,
                        targetObservationXMeters,
                        targetObservationYMeters,
                        search.targetHeading.x(),
                        search.targetHeading.y(),
                        triggerTravelM))
                {
                    return "Search straight rolling observation sample trigger is invalid";
                }
                if (sampleIndex > 0U &&
                    triggerTravelM < (search.rollingObservationTriggerTravelM[sampleIndex - 1U] - Config::kDistanceToleranceM))
                {
                    AppendMissionTraceFormatted(
                        "mission_observation_trigger_recovered,cell=(%d,%d),abs=%s,sample=%u,prev_m=%.4f,raw_m=%.4f",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        static_cast<unsigned>(sampleIndex),
                        search.rollingObservationTriggerTravelM[sampleIndex - 1U],
                        triggerTravelM);
                    triggerTravelM = search.rollingObservationTriggerTravelM[sampleIndex - 1U];
                }

                search.rollingObservationTriggerTravelM[sampleIndex] = triggerTravelM;
            }

            float targetResetXMeters = 0.0f;
            float targetResetYMeters = 0.0f;
            if (!MazeMap::TryComputeSideWallTravelFractionPoseM(
                    search.nextRollingObservationCell,
                    search.direction,
                    Config::kCellSizeM,
                    sideSensorForwardOffsetM,
                    Config::kSideWallStateResetCellEntryFraction,
                    targetResetXMeters,
                    targetResetYMeters))
            {
                return "Search straight side reset trigger pose is invalid";
            }

            float resetTriggerTravelM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    startPose.xMeters,
                    startPose.yMeters,
                    targetResetXMeters,
                    targetResetYMeters,
                    search.targetHeading.x(),
                    search.targetHeading.y(),
                    resetTriggerTravelM))
            {
                return "Search straight side reset trigger is invalid";
            }
            if (resetTriggerTravelM > (search.rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM))
            {
                AppendMissionTraceFormatted(
                    "mission_side_reset_trigger_recovered,cell=(%d,%d),abs=%s,raw_m=%.4f,first_sample_m=%.4f",
                    search.nextRollingObservationCell.GetX(),
                    search.nextRollingObservationCell.GetY(),
                    DirectionName(search.direction),
                    resetTriggerTravelM,
                    search.rollingObservationTriggerTravelM[0]);
                resetTriggerTravelM =
                    (std::max)(0.0f, search.rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM);
            }

            search.rollingObservationSideResetTriggerTravelM = resetTriggerTravelM;
            search.rollingObservationSideResetPending = true;
            search.rollingObservationPlanInitialized = true;
            return nullptr;
        };

        if (search.observeWhileRolling)
        {
            while (search.rollingObservationCount < search.cellCount)
            {
                if (const char* error = initializeRollingObservationPlan())
                {
                    return FaultLoopPhase(services, error);
                }

                if (search.rollingObservationSideResetPending &&
                    (projectedTravelM + Config::kDistanceToleranceM) >= search.rollingObservationSideResetTriggerTravelM)
                {
                    _sensors.ResetSideWallMemory();
                    search.rollingObservationSideResetPending = false;
                    AppendMissionTraceFormatted(
                        "mission_side_reset,cell=(%d,%d),abs=%s,travel_m=%.4f,trigger_m=%.4f",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        projectedTravelM,
                        search.rollingObservationSideResetTriggerTravelM);
                    break;
                }

                while (search.rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount &&
                    (projectedTravelM + Config::kDistanceToleranceM) >=
                        search.rollingObservationTriggerTravelM[search.rollingObservationNextSampleIndex])
                {
                    search.rollingObservationSamples[search.rollingObservationNextSampleIndex] = state.sensors;
                    float frontLeftCandidateDistanceM = NAN;
                    float frontRightCandidateDistanceM = NAN;
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontLeft,
                        search.nextRollingObservationCell,
                        search.direction,
                        frontLeftCandidateDistanceM);
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontRight,
                        search.nextRollingObservationCell,
                        search.direction,
                        frontRightCandidateDistanceM);
                    search.rollingObservationFrontLeftCandidateDistanceM[search.rollingObservationNextSampleIndex] =
                        frontLeftCandidateDistanceM;
                    search.rollingObservationFrontRightCandidateDistanceM[search.rollingObservationNextSampleIndex] =
                        frontRightCandidateDistanceM;
                    ++search.rollingObservationNextSampleIndex;
                }

                if (search.rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount)
                {
                    break;
                }

                SensorSnapshot majoritySnapshot{};
                RollingObservationVoteSummary voteSummary{};
                if (!BuildEvidenceObservationSnapshot(
                        search.rollingObservationSamples,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot,
                        voteSummary))
                {
                    return FaultLoopPhase(services, "Search straight rolling observation majority snapshot is invalid");
                }
                if (!TryApplyFrontWallCharacterizationToObservation(
                        search.nextRollingObservationCell,
                        search.direction,
                        "rolling",
                        search.rollingObservationSamples,
                        search.rollingObservationFrontLeftCandidateDistanceM,
                        search.rollingObservationFrontRightCandidateDistanceM,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot))
                {
                    AppendMissionTraceFormatted(
                        "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=rolling,fallback_valid=%u",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        majoritySnapshot.frontWallObservationValid ? 1U : 0U);
                }

                AppendMissionTraceFormatted(
                    "mission_observation_timed,cell=(%d,%d),abs=%s,samples=%u,start_m=%.4f,end_m=%.4f,travel_m=%.4f,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
                    search.nextRollingObservationCell.GetX(),
                    search.nextRollingObservationCell.GetY(),
                    DirectionName(search.direction),
                    static_cast<unsigned>(voteSummary.sampleCount),
                    search.rollingObservationTriggerTravelM[0],
                    search.rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount - 1U],
                    projectedTravelM,
                    static_cast<unsigned>(voteSummary.frontWallVotes),
                    static_cast<unsigned>(voteSummary.leftWindowValidVotes),
                    static_cast<unsigned>(voteSummary.leftWallVotes),
                    static_cast<unsigned>(voteSummary.rightWindowValidVotes),
                    static_cast<unsigned>(voteSummary.rightWallVotes));
                bool forwardWallCommittedFromUnknown = false;
                if (!ObserveCellFromSnapshot(
                        search.nextRollingObservationCell,
                        search.direction,
                        majoritySnapshot,
                        &forwardWallCommittedFromUnknown))
                {
                    return FaultLoopPhase(services, "Search straight observation commit failed");
                }

                if (forwardWallCommittedFromUnknown)
                {
                    search.stoppedForReplan = true;
                    search.replanObservedCell = search.nextRollingObservationCell;
                    search.replanProjectedTravelM = projectedTravelM;
                    search.replanFrontVoteCount = voteSummary.frontWallVotes;
                    if (!BeginSharedBrakedSettlePhase(
                            search.completionSettle,
                            nullptr,
                            Config::kMotionSettleHoldMs,
                            0U,
                            nullptr,
                            nullptr,
                            services))
                    {
                        return FaultLoopPhase(services, "Failed to begin search replan settle phase");
                    }
                    return LoopController::ControlVector::Brake;
                }

                ++search.rollingObservationCount;
                if (search.rollingObservationCount < search.cellCount)
                {
                    search.nextRollingObservationCell = search.nextRollingObservationCell >> search.direction;
                }
                resetRollingObservationPlan();
            }
        }

        const bool stoppingAtEndpoint = search.exitSpeedMps <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            if (!BeginSharedBrakedSettlePhase(
                    search.completionSettle,
                    nullptr,
                    Config::kMotionSettleHoldMs,
                    0U,
                    nullptr,
                    nullptr,
                    services))
            {
                return FaultLoopPhase(services, "Failed to begin search completion settle phase");
            }
            return LoopController::ControlVector::Brake;
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - search.exitSpeedMps) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return EndLoopPhase(services);
        }

        const unsigned long nowMs = millis();
        if (!search.stallLogged &&
            search.translationWatchdog.Stalled(projectedTravelM, search.commandedSpeedMps, remainingM, nowMs))
        {
            search.stallLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=search_straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                projectedTravelM,
                remainingM,
                search.commandedSpeedMps);
        }
        if (!search.durationLogged && static_cast<long>(search.expectedCompletionDeadlineMs - nowMs) <= 0)
        {
            search.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=search_straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                projectedTravelM,
                remainingM,
                search.commandedSpeedMps);
        }

        const MotionLimits searchLimits = SearchLimits();
        const float accelLimitedSpeedMps = (std::min)(
            search.cruiseSpeedMps,
            search.commandedSpeedMps + (searchLimits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(search.exitSpeedMps, remainingM, searchLimits.decelMps2);
        search.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        float wallOmegaRadps = 0.0f;
        float signalCorridorErrorM = 0.0f;
        if (Runtime::TryComputeWallGroundedCorridorErrorM(
                _maze,
                _speedVehicle,
                _currentDirectionalLocation,
                state.estimate,
                state.sensors,
                signalCorridorErrorM))
        {
            wallOmegaRadps += Runtime::ComputeWallCenterPdOmegaRadps(
                signalCorridorErrorM,
                search.commandedSpeedMps,
                state.dtSeconds,
                search.previousCorridorErrorM,
                search.filteredCorridorErrorRateMps,
                search.previousCorridorErrorValid);
        }
        else
        {
            search.filteredCorridorErrorRateMps = 0.0f;
            search.previousCorridorErrorValid = false;
        }
        if (stoppingAtEndpoint &&
            std::isfinite(state.sensors.frontLeftDistanceM) &&
            std::isfinite(state.sensors.frontRightDistanceM) &&
            state.sensors.frontLeftDistanceM < Config::kFrontWallOnThresholdM &&
            state.sensors.frontRightDistanceM < Config::kFrontWallOnThresholdM &&
            remainingM < 0.07f)
        {
            wallOmegaRadps += Config::kFrontSkewGain * state.sensors.frontSkewM;
        }

        const float headingErrorRad = HeadingErrorRad(search.targetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) +
            wallOmegaRadps;
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -searchLimits.maxAngularSpeedRadps,
            searchLimits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            search.commandedSpeedMps,
            angularCommandRadps,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteSearchStraightCellsLoopDriven(
        MazeMap::Direction direction,
        uint16_t cellCount,
        float entrySpeedMps,
        float cruiseSpeedMps,
        float exitSpeedMps,
        bool snapAtEnd,
        bool observeWhileRolling = false,
        bool* outStoppedForReplan = nullptr)
    {
        (void)snapAtEnd;
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = false;
        }
        if (cellCount == 0U)
        {
            return true;
        }

        SearchStraightLoopState search{};
        search.direction = direction;
        search.cellCount = cellCount;
        search.entrySpeedMps = entrySpeedMps;
        search.cruiseSpeedMps = cruiseSpeedMps;
        search.exitSpeedMps = exitSpeedMps;
        search.observeWhileRolling = observeWhileRolling;
        search.startCell = _currentCell;
        search.destination = _currentCell;
        for (std::uint16_t index = 0U; index < cellCount; ++index)
        {
            search.destination = search.destination >> direction;
        }
        search.targetHeading = DirectionToUnitVector(direction);
        MazeMap::MazeLocation::CellCenter(search.destination).GetPhysicalLocation(
            Config::kCellSizeM,
            search.targetXMeters,
            search.targetYMeters);

        const PoseEstimate startPose = _drive.GetPose();
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                startPose.xMeters,
                startPose.yMeters,
                search.targetXMeters,
                search.targetYMeters,
                search.targetHeading.x(),
                search.targetHeading.y(),
                search.distanceToTargetM))
        {
            return Fail("Search straight target distance is invalid");
        }
        if (search.distanceToTargetM < -Config::kDistanceToleranceM)
        {
            return Fail("Search straight target fell behind the current pose");
        }

        search.nextRollingObservationCell = search.startCell;
        if (observeWhileRolling)
        {
            search.nextRollingObservationCell = search.nextRollingObservationCell >> direction;
        }
        search.commandedSpeedMps = (std::max)(entrySpeedMps, 0.0f);
        search.translationWatchdog.Reset(0.0f, millis());
        search.expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2500.0f + (5000.0f * search.distanceToTargetM));

        if (!RunLoopSession(&search, &Implementation::SearchStraightLoopTick))
        {
            return false;
        }

        if (search.stoppedForReplan)
        {
            return HandleSearchWallMapUpdateStop(
                search.replanObservedCell,
                direction,
                search.replanProjectedTravelM,
                search.replanFrontVoteCount,
                outStoppedForReplan);
        }

        _currentCell = search.destination;
        _currentDirectionalLocation =
            MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    bool ExploreFullMaze()
    {
        while (!_maze.IsComplete())
        {
            if (_maze.HasFoundGoal() && !_goalPauseComplete)
            {
                if (!EmitMissionControllerLineOrFail("Goal discovered"))
                {
                    return false;
                }
                if (!DriveToGoalAndPause())
                {
                    return false;
                }
                continue;
            }

            MazeMap::Path<PATH_SIZE> path;
            _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);

            if (path.GetSize() < 2)
            {
                if (!ObserveCurrentCell())
                {
                    return false;
                }
                _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);
                if (path.GetSize() < 2 && !_maze.IsComplete())
                {
                    return Fail("Search path stalled before maze completion");
                }
                continue;
            }

            if (!ExecuteSearchPath(path, true))
            {
                return false;
            }
        }

        return true;
    }

    bool DriveToGoalAndPause()
    {
        while (!IsInGoalCell(_currentCell))
        {
            MazeMap::Path<PATH_SIZE> goalPath;
            _searchPathFinder.PathToGoal(_currentCell, _currentDirection, goalPath);
            if (goalPath.GetSize() < 2)
            {
                return Fail("Unable to drive to goal after detection");
            }
            if (!ExecuteSearchPath(goalPath, false))
            {
                return false;
            }
        }

        if (!EmitMissionControllerLineOrFail("Holding in goal for 2 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kGoalPauseMs))
        {
            return false;
        }
        _goalPauseComplete = true;
        return ObserveCurrentCell();
    }

    bool ReturnToStart()
    {
        const MazeMap::CellCoordinates start(0, 0);
        while (_currentCell != start)
        {
            MazeMap::HalfStepPath<PATH_SIZE * 2> returnHalfStepPath;
            _searchPathFinder.HalfStepPathFromTo(_currentCell, _currentDirection, start, returnHalfStepPath);

            MazeMap::ManeuverPath maneuverPath;
            if (returnHalfStepPath.GetSize() > 1U
                && MazeMap::ManeuverPath::FromHalfStep(returnHalfStepPath, _currentDirectionalLocation, maneuverPath)
                && maneuverPath.GetSize() > 0U)
            {
                char traceLine[112] = {};
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "return_to_start:using_search_halfstep_maneuvers,halfsteps=%u,maneuvers=%u",
                    static_cast<unsigned>(returnHalfStepPath.GetSize()),
                    static_cast<unsigned>(maneuverPath.GetSize()));
                AppendStartupTrace(traceLine);

                MazeMap::ManeuverQueue queue(maneuverPath, _currentDirectionalLocation);
                queue.ComputeSpeeds(_mappingVehicle, 0.0f, 0.0f);
                const MotionLimits returnLimits = SearchLimits();
                _runtime.ManeuverExecutorService().ApplyAsymmetricQueueLimits(
                    queue,
                    returnLimits,
                    _mappingVehicle,
                    0.0f,
                    0.0f);
                if (!ExecuteQueuedManeuvers(queue, returnLimits, false))
                {
                    return false;
                }
            }
            else
            {
                MazeMap::Path<PATH_SIZE> path;
                _searchPathFinder.PathFromTo(_currentCell, _currentDirection, start, path);
                if (path.GetSize() < 2)
                {
                    return Fail("Unable to return to start");
                }
                AppendStartupTrace("return_to_start:fallback_search_path");
                if (!ExecuteSearchPath(path, false))
                {
                    return false;
                }
            }
        }

        if (!OrientTo(MazeMap::Up, SearchLimits()))
        {
            return false;
        }

        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return ObserveCurrentCell();
    }

    bool HandleInterRunServiceCycle()
    {
        InterRunServicePauseLoopState pauseState{};
        return RunLoopSession(&pauseState, &Implementation::InterRunServicePauseTick) &&
            PrepareForSecondSpeedRun();
    }

    bool PrepareForSecondSpeedRun()
    {
        if (!_sensors.Begin())
        {
            return Fail("Sensor reset failed after inter-run service");
        }

        return RunStartupWallCalibration();
    }

    bool FinishSpeedRunAndReturnToStart()
    {
        if (!EmitMissionControllerLineOrFail("Holding at finish for 3 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kSpeedRunFinishPauseMs))
        {
            return false;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return false;
        }
        return ReturnToStart();
    }

    bool ExecuteRacingRunCycle()
    {
        SetRacingFanEnabled(true);
        const bool ok = RunSpeedRun() && FinishSpeedRunAndReturnToStart();
        SetRacingFanEnabled(false);
        return ok;
    }

    bool RunSpeedRun()
    {
        MazeMap::ManeuverPath path;
        _speedPathFinder.ManeuverPathToGoal(_currentCell, _currentDirection, path);
        if (path.GetSize() == 0)
        {
            return Fail("ManeuverPathFinder returned an empty path");
        }

        MazeMap::ManeuverQueue queue(path, _currentDirectionalLocation);
        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        _runtime.ManeuverExecutorService().ApplyAsymmetricQueueLimits(
            queue,
            FinalLimits(),
            0.0f,
            0.0f);
        return ExecuteQueuedManeuvers(queue, true);
    }

    bool ExecuteSearchStraightCells(
        MazeMap::Direction direction,
        uint16_t cellCount,
        float entrySpeedMps,
        float cruiseSpeedMps,
        float exitSpeedMps,
        bool snapAtEnd,
        bool observeWhileRolling = false,
        bool* outStoppedForReplan = nullptr)
    {
        return ExecuteSearchStraightCellsLoopDriven(
            direction,
            cellCount,
            entrySpeedMps,
            cruiseSpeedMps,
            exitSpeedMps,
            snapAtEnd,
            observeWhileRolling,
            outStoppedForReplan);
    }

    bool ExecuteSearchPath(const MazeMap::Path<PATH_SIZE>& path, bool observeFinalCell)
    {
        if (path.GetSize() < 2)
        {
            return observeFinalCell ? ObserveCurrentCell() : true;
        }

        const MotionLimits searchLimits = SearchLimits();
        const float cautiousCruiseSpeedMps = SearchUnmappedCruiseSpeedMps();
        uint16_t pathIndex = 1U;

        while (pathIndex < path.GetSize())
        {
            const MazeMap::SearchStraightPlan plan = MazeMap::PlanSearchStraightSegment(_maze, path, pathIndex);
            if (plan.direction == MazeMap::None || plan.TotalCellCount() == 0U)
            {
                return Fail("Search path contained an invalid segment");
            }

            if (!OrientTo(plan.direction, searchLimits))
            {
                return false;
            }

            float rollingEntrySpeedMps = 0.0f;
            if (plan.fullSpeedCellCount > 0U)
            {
                const float exitSpeedMps = (plan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                if (!ExecuteSearchStraightCells(
                    plan.direction,
                    plan.fullSpeedCellCount,
                    0.0f,
                    searchLimits.maxSpeedMps,
                    exitSpeedMps,
                    plan.cautiousCellCount == 0U,
                    false))
                {
                    return false;
                }
                rollingEntrySpeedMps = exitSpeedMps;
            }

            if (plan.cautiousCellCount > 0U)
            {
                if (!observeFinalCell)
                {
                    const float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            plan.cautiousCellCount,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            0.0f,
                            true))
                    {
                        return false;
                    }
                    pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
                    continue;
                }

                float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? rollingEntrySpeedMps : 0.0f;
                while (true)
                {
                    bool stoppedForReplan = false;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            1U,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            cautiousCruiseSpeedMps,
                            false,
                            true,
                            &stoppedForReplan))
                    {
                        return false;
                    }
                    if (stoppedForReplan)
                    {
                        return true;
                    }

                    cautiousEntrySpeedMps = cautiousCruiseSpeedMps;
                    MazeMap::Path<PATH_SIZE> continuingPath;
                    _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, continuingPath);
                    if (continuingPath.GetSize() < 2U)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    const MazeMap::SearchStraightPlan nextPlan = MazeMap::PlanSearchStraightSegment(_maze, continuingPath, 1U);
                    if (nextPlan.direction == MazeMap::None || nextPlan.TotalCellCount() == 0U)
                    {
                        return Fail("Search path contained an invalid continuation");
                    }

                    if (nextPlan.direction != plan.direction)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    if (nextPlan.fullSpeedCellCount > 0U)
                    {
                        const float exitSpeedMps = (nextPlan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                        if (!ExecuteSearchStraightCells(
                                plan.direction,
                                nextPlan.fullSpeedCellCount,
                                cautiousEntrySpeedMps,
                                searchLimits.maxSpeedMps,
                                exitSpeedMps,
                                nextPlan.cautiousCellCount == 0U,
                                false))
                        {
                            return false;
                        }
                        if (nextPlan.cautiousCellCount == 0U)
                        {
                            return observeFinalCell ? ObserveCurrentCell() : true;
                        }

                        cautiousEntrySpeedMps = exitSpeedMps;
                    }
                }
            }

            pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
        }

        return observeFinalCell ? ObserveCurrentCell() : true;
    }

    bool ApplyWallGroundedCorridorPoseCorrection(const SensorSnapshot& snapshot)
    {
        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!Runtime::TryComputeWallGroundedCorridorCoordinateM(
                _maze,
                _speedVehicle,
                _currentDirectionalLocation,
                _drive.GetPose(),
                snapshot,
                corridorCoordinateM,
                correctsXAxis))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(corridorCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(corridorCoordinateM);
        }

        if (std::fabs(corridorCoordinateM - priorCoordinateM) >= 0.001f)
        {
            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "mission_pose_snap,axis=%s,from=%.4f,to=%.4f,cell=(%d,%d)",
                correctsXAxis ? "x" : "y",
                priorCoordinateM,
                corridorCoordinateM,
                _currentCell.GetX(),
                _currentCell.GetY());
            AppendStartupTrace(traceLine);
        }
        return true;
    }

    bool TryComputeTurnWallEdgeCoordinateM(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker,
        float& coordinateM,
        bool& correctsXAxis,
        const char*& sourceName) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        sourceName = "none";

        // The center post in the 2x2 goal can create side-sensor rising edges that do not correspond to a usable
        // corridor wall boundary, so suppress turn-edge grounding inside the goal area.
        if (IsInGoalCell(_currentCell))
        {
            sourceName = "goal_suppressed";
            return false;
        }

        switch (targetDirection)
        {
        case MazeMap::Up:
        case MazeMap::Down:
            correctsXAxis = true;
            break;
        case MazeMap::Left:
        case MazeMap::Right:
            correctsXAxis = false;
            break;
        default:
            return false;
        }

        const MazeMap::Cell& cell = _maze[_currentCell];
        const MazeMap::Direction leftWallDirection = targetDirection + MazeMap::Left90;
        const MazeMap::Direction rightWallDirection = targetDirection + MazeMap::Right90;
        const PoseEstimate& pose = _drive.GetPose();
        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate =
            edgeTracker.leftWallRose &&
            snapshot.leftDistanceValidForControl &&
            (cell.GetWall(leftWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                _currentCell,
                leftWallDirection,
                leftCoordinateM);
        bool haveRightCoordinate =
            edgeTracker.rightWallRose &&
            snapshot.rightDistanceValidForControl &&
            (cell.GetWall(rightWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                _currentCell,
                rightWallDirection,
                rightCoordinateM);

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        if (haveLeftCoordinate && haveRightCoordinate)
        {
            coordinateM = 0.5f * (leftCoordinateM + rightCoordinateM);
            sourceName = "left+right";
        }
        else if (haveLeftCoordinate)
        {
            coordinateM = leftCoordinateM;
            sourceName = "left";
        }
        else
        {
            coordinateM = rightCoordinateM;
            sourceName = "right";
        }

        return std::isfinite(coordinateM);
    }

    bool ApplyTurnWallEdgePoseCorrection(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker)
    {
        float correctedCoordinateM = 0.0f;
        bool correctsXAxis = false;
        const char* sourceName = "none";
        if (!TryComputeTurnWallEdgeCoordinateM(
                targetDirection,
                snapshot,
                edgeTracker,
                correctedCoordinateM,
                correctsXAxis,
                sourceName))
        {
            if (IsInGoalCell(_currentCell) && (edgeTracker.leftWallRose || edgeTracker.rightWallRose))
            {
                AppendMissionTraceFormatted(
                    "mission_turn_edge_snap:suppressed,cell=(%d,%d),dir=%s,left_rose=%u,right_rose=%u",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    DirectionName(targetDirection),
                    edgeTracker.leftWallRose ? 1U : 0U,
                    edgeTracker.rightWallRose ? 1U : 0U);
            }
            return true;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(correctedCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(correctedCoordinateM);
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn_edge_snap,axis=%s,from=%.4f,to=%.4f,dir=%s,source=%s,cell=(%d,%d)",
            correctsXAxis ? "x" : "y",
            priorCoordinateM,
            correctedCoordinateM,
            DirectionName(targetDirection),
            sourceName,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        (void)WriteMissionTraceLineBestEffort(traceLine, "mission_text_logging:turn_edge_snap_write_failed");

        return true;
    }

    bool OrientTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const MazeMap::RelativeDirection relative = targetDirection - _currentDirection;
        if (relative == MazeMap::Forward)
        {
            return true;
        }

        if (RelativeToInPlaceCode(relative) == MazeMap::MC_NONE)
        {
            return Fail("Unsupported in-place turn requested");
        }

        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Mission turn angle is invalid");
        }
        char traceLine[160] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:begin,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f,cell=(%d,%d)",
            RAD_TO_DEG_F * _drive.GetPose().yawRad,
            RAD_TO_DEG_F * targetYawRad,
            RAD_TO_DEG_F * angleRad,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        MazeMap::TurnWallEdgeTracker wallEdgeTracker{};
        if (!ExecuteTurnProfile(angleRad, limits, &wallEdgeTracker))
        {
            return false;
        }

        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:end,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f",
            _drive.GetPose().xMeters,
            _drive.GetPose().yMeters,
            RAD_TO_DEG_F * _drive.GetPose().yawRad,
            _drive.GetPose().linearSpeedMps,
            _drive.GetPose().angularSpeedRadps);
        AppendStartupTrace(traceLine);
        _currentDirection = targetDirection;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    bool ExecuteStraightProfile(
        float distanceM,
        float entrySpeed,
        float cruiseSpeed,
        float exitSpeed,
        const MotionLimits& limits,
        bool useWallCentering,
        const Eigen::Vector2f* targetHeadingOverride = nullptr,
        const Eigen::Vector2f* targetPositionOverride = nullptr)
    {
        if (!(std::isfinite(distanceM) && (distanceM > 0.0f)))
        {
            return true;
        }

        if (!_runtime.ManeuverExecutorService().BeginStraightPhase(
                distanceM,
                entrySpeed,
                cruiseSpeed,
                exitSpeed,
                limits,
                useWallCentering,
                useWallCentering ? &_currentDirectionalLocation : nullptr,
                BuildManeuverExecutorHooks(false),
                targetHeadingOverride,
                targetPositionOverride))
        {
            return Fail("Failed to begin shared straight execution");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool ExecuteTurnProfile(
        float angleRad,
        const MotionLimits& limits,
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr)
    {
        if (!_runtime.ManeuverExecutorService().BeginTurnPhase(
                angleRad,
                limits,
                BuildManeuverExecutorHooks(false),
                wallEdgeTracker))
        {
            return Fail("Failed to begin shared turn execution");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool ExecuteArcProfile(float distanceM, float angleRad, float entrySpeed, float exitSpeed, float cruiseSpeed, const MotionLimits& limits)
    {
        if (!_runtime.ManeuverExecutorService().BeginArcPhase(
                distanceM,
                angleRad,
                entrySpeed,
                exitSpeed,
                cruiseSpeed,
                limits,
                BuildManeuverExecutorHooks(false)))
        {
            return Fail("Failed to begin shared arc execution");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool ExecuteSmoothTurnProfile(
        const MazeMap::ManeuverInstance& maneuver,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        if (!_runtime.ManeuverExecutorService().BeginSmoothTurnPhase(
                maneuver,
                cruiseSpeed,
                limits,
                BuildManeuverExecutorHooks(false)))
        {
            return Fail("Failed to begin shared smooth-turn execution");
        }

        return RunSharedManeuverExecutorPhase();
    }

    bool IsInGoalCell(MazeMap::CellCoordinates coords) const
    {
        if (!_maze.HasFoundGoal())
        {
            return false;
        }

        const MazeMap::CellCoordinates goal = _maze.GetGoalLowerLeft();
        const bool xMatch = (coords.GetX() == goal.GetX()) || (coords.GetX() == static_cast<uint8_t>(goal.GetX() + 1));
        const bool yMatch = (coords.GetY() == goal.GetY()) || (coords.GetY() == static_cast<uint8_t>(goal.GetY() + 1));
        return xMatch && yMatch;
    }

};

namespace MazeMap::App::Internal
{
    MissionModeController::MissionModeController(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    MissionModeController::~MissionModeController() = default;

    bool MissionModeController::BeginMissionRunMode()
    {
        return _impl->BeginMissionRunMode();
    }

    void MissionModeController::RunMissionRunMode()
    {
        _impl->RunMissionRunMode();
    }

    bool MissionModeController::BeginCorridorRepeatabilityMode()
    {
        return _impl->BeginCorridorRepeatabilityMode();
    }

    void MissionModeController::RunCorridorRepeatabilityMode()
    {
        _impl->RunCorridorRepeatabilityMode();
    }

    bool MissionModeController::BeginPositionAccuracyAuditMode()
    {
        return _impl->BeginPositionAccuracyAuditMode();
    }

    void MissionModeController::RunPositionAccuracyAuditMode()
    {
        _impl->RunPositionAccuracyAuditMode();
    }
}
