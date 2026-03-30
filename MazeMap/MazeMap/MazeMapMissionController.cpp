#include "MazeMapApplicationPrivate.h"
#include "MazeMapControllerRegistry.h"
#include "MazeMapSharedRuntime.h"

using MazeMapApp::Internal::GetSharedRobotRuntime;
using MazeMapApp::Internal::SharedRobotRuntime;

class MissionController final : public IMissionModeHost
{
public:
    explicit MissionController(SharedRobotRuntime& runtime)
        : _speedVehicle(runtime.SpeedVehicle())
        , _mappingVehicle(runtime.SearchVehicle())
        , _maze(runtime.Maze())
        , _searchPathFinder(runtime.SearchPathFinder())
        , _speedPathFinder(runtime.SpeedPathFinder())
        , _wallBeliefMap(runtime.WallBeliefMap())
        , _sensors(runtime.MissionSensors())
        , _telemetrySensors(runtime.TelemetrySensors())
        , _drive(runtime.Drive())
        , _currentCell(0, 0)
        , _currentDirection(MazeMap::Up)
        , _currentDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _goalPauseComplete(false)
        , _missionComplete(false)
        , _faulted(false)
        , _maneuverTestMode(false)
        , _telemetryLoggingEnabled(false)
        , _missionTextLoggingEnabled(false)
        , _missionMazeSnapshotWritten(false)
        , _frontWallCharacterization()
        , _frontWallCharacterizationAvailable(false)
        , _lastWallTouchStandoffEstimateM(0.0f)
        , _hasWallTouchStandoffEstimate(false)
        , _lastControlMicros(0UL)
    {
    }

    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;
    MissionController(MissionController&&) = delete;
    MissionController& operator=(MissionController&&) = delete;

    void CommandOpenLoop(const MazeMap::OpenLoopDriveCommand& command)
    {
        _drive.CommandOpenLoop(command);
    }

    void CommandOpenLoopRaw(const MazeMap::OpenLoopDriveCommand& command)
    {
        _drive.CommandOpenLoopRaw(command);
    }

    bool BeginMissionRunMode() override
    {
        ResetForMode(false, true);
        if (!Initialize("Micromouse mission setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        return true;
    }

    void RunMissionRunMode() override
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

    bool BeginManeuverFileTestMode() override
    {
        ResetForMode(true, false);
        if (!Initialize("Micromouse maneuver test setup", false))
        {
            return false;
        }

        SeedWallBeliefsFromKnownMaze();
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, "maneuver_test.csv", Config::kControlPeriodUs, "maneuver_test"))
        {
            AppendStartupTrace("maneuver_test:telemetry_logger_open_failed");
            Serial.println("Maneuver test telemetry log unavailable; continuing without telemetry file");
            _telemetryLoggingEnabled = false;
            return true;
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("maneuver_test:telemetry_logger_opened");
        if (!_telemetryLogger.WriteEvent("source", "test.txt"))
        {
            AppendStartupTrace("maneuver_test:source_metadata_write_failed");
            Serial.println("Maneuver test source metadata write failed; disabling telemetry file logging");
            ShutdownTelemetryMode(false);
            return true;
        }
        AppendStartupTrace("maneuver_test:source_metadata_written");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        AppendStartupTrace("maneuver_test:wall_calibration_logged");
        return true;
    }

    void RunManeuverFileTestMode() override
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("maneuver_test:run_entered");
        MazeMap::ManeuverQueue queue;
        if (!LoadManeuverQueueFromSd("test.txt", queue))
        {
            return;
        }
        AppendStartupTrace("maneuver_test:queue_loaded");

        Serial.print("Loaded maneuver test queue with ");
        Serial.print(queue.size());
        Serial.println(" maneuvers");

        if (!HoldPosition(Config::kObservationSettleMs, "startup_settle"))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        if (!ExecuteQueuedManeuvers(queue, false))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        if (!HoldPosition(50, "final_hold"))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        ShutdownTelemetryMode(false);
        AppendStartupTrace("maneuver_test:complete");
        Serial.println("Maneuver file test complete");
    }

    bool BeginCorridorRepeatabilityMode() override
    {
        ResetForMode(false, false);
        if (!Initialize("Corridor repeatability setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMapApp::Internal::Runtime::SelectSequentialCsvFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.csv",
                "corridor_repeatability.csv"))
        {
            return Fail("Unable to choose corridor repeatability log file");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, fileName, Config::kControlPeriodUs, "corridor_repeatability"))
        {
            return Fail("Unable to open corridor repeatability log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("corridor_repeatability:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        if (!LogCorridorRepeatabilityMetadata())
        {
            return false;
        }
        AppendStartupTrace("corridor_repeatability:metadata_written");
        return true;
    }

    void RunCorridorRepeatabilityMode() override
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
            Serial.println("Corridor repeatability sweep complete");
        }
    }

    bool BeginPositionAccuracyAuditMode() override
    {
        ResetForMode(false, false);
        if (!Initialize("Position accuracy audit setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMapApp::Internal::Runtime::SelectSequentialCsvFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.csv",
                "position_accuracy_audit.csv"))
        {
            return Fail("Unable to choose position accuracy audit log file");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, fileName, Config::kControlPeriodUs, "position_accuracy_audit"))
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
        if (!LogPositionAccuracyAuditMetadata(positionAuditGeometry))
        {
            return false;
        }
        AppendStartupTrace("position_accuracy_audit:metadata_written");
        return true;
    }

    void RunPositionAccuracyAuditMode() override
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
            Serial.println("Position accuracy audit complete");
        }
    }

private:
    static void SetKnownMazeCellWalls(
        MazeMap::Maze& maze,
        const MazeMap::CellCoordinates& cellCoordinates,
        MazeMap::WallState up,
        MazeMap::WallState down,
        MazeMap::WallState left,
        MazeMap::WallState right)
    {
        MazeMap::Cell& cell = maze[cellCoordinates];
        maze.SetWall(cell, MazeMap::Up, up);
        maze.SetWall(cell, MazeMap::Down, down);
        maze.SetWall(cell, MazeMap::Left, left);
        maze.SetWall(cell, MazeMap::Right, right);
    }

    struct PositionAuditFixtureGeometry
    {
        MazeMap::Maze maze;
        uint8_t northCorridorCellCount = 0U;
        uint8_t eastExtensionCellCount = 0U;
        uint8_t eastTotalCellCount = 0U;
        float northCorridorSpanYM = 0.0f;
        float eastBranchSpanXM = 0.0f;
        float outDistanceM = 0.0f;
        float farCellCenterYM = 0.0f;
        float farWallTouchYM = 0.0f;
        float eastWallTouchXM = 0.0f;
    };

    static MazeMap::Maze BuildPositionAuditMazeFixture(uint8_t northCorridorCellCount, uint8_t eastExtensionCellCount)
    {
        MazeMap::Maze maze;

        for (uint8_t y = 0U; y < northCorridorCellCount; ++y)
        {
            const MazeMap::CellCoordinates cell(0U, y);
            SetKnownMazeCellWalls(
                maze,
                cell,
                (y + 1U < northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall,
                (y > 0U) ? MazeMap::NoWall : MazeMap::Wall,
                MazeMap::Wall,
                (y + 1U == northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        for (uint8_t x = 1U; x <= eastExtensionCellCount; ++x)
        {
            const MazeMap::CellCoordinates cell(
                x,
                static_cast<uint8_t>(northCorridorCellCount - 1U));
            SetKnownMazeCellWalls(
                maze,
                cell,
                MazeMap::Wall,
                MazeMap::Wall,
                MazeMap::NoWall,
                (x < eastExtensionCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        return maze;
    }

    static PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry()
    {
        PositionAuditFixtureGeometry geometry{};
        geometry.northCorridorCellCount = AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount;
        geometry.eastExtensionCellCount = AuxMeasurementConfig::kPositionAuditEastBranchCellCount;
        geometry.eastTotalCellCount = static_cast<uint8_t>(geometry.eastExtensionCellCount + 1U);
        geometry.northCorridorSpanYM = Config::kCellSizeM * static_cast<float>(geometry.northCorridorCellCount);
        geometry.eastBranchSpanXM = Config::kCellSizeM * static_cast<float>(geometry.eastTotalCellCount);
        geometry.outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(geometry.northCorridorCellCount - 1U);
        geometry.farCellCenterYM =
            (static_cast<float>(geometry.northCorridorCellCount) - 0.5f) *
            Config::kCellSizeM;
        geometry.farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            geometry.northCorridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.eastWallTouchXM = MazeMap::ComputeWallTouchPoseFromEastWallM(
            geometry.eastBranchSpanXM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.maze = BuildPositionAuditMazeFixture(
            geometry.northCorridorCellCount,
            geometry.eastExtensionCellCount);
        return geometry;
    }

    static uint8_t CountClearForwardHalfStepsUntilBlocked(
        const MazeMap::Maze& maze,
        const MazeMap::DirectionalLocation& start,
        uint8_t maxHalfSteps = 31U)
    {
        MazeMap::DirectionalLocation cursor = start;
        uint8_t clearHalfSteps = 0U;
        while (clearHalfSteps < maxHalfSteps)
        {
            cursor = cursor.MoveForward(1U);
            if (!maze.IsAccessibleLocation(cursor.GetLocation()))
            {
                break;
            }

            ++clearHalfSteps;
        }

        return clearHalfSteps;
    }

    static bool TryResolvePositionAuditSmoothTurnLaunchLocation(
        const PositionAuditFixtureGeometry& geometry,
        MazeMap::ManeuverCode code,
        MazeMap::DirectionalLocation& launchLocation)
    {
        const uint8_t corridorCenterHalfX = 1U;
        const uint8_t maxHalfY = static_cast<uint8_t>(
            (geometry.northCorridorCellCount << 1U) - 1U);
        const MazeMap::Maze& auditMaze = geometry.maze;

        for (uint8_t halfY = maxHalfY; halfY > 0U; --halfY)
        {
            const MazeMap::DirectionalLocation candidate(
                MazeMap::MazeLocation(corridorCenterHalfX, halfY),
                MazeMap::Up);
            if (!auditMaze.IsAccessibleLocation(candidate.GetLocation()))
            {
                continue;
            }
            if (!MazeMap::ManeuverSet::GetSet().IsValidMove(code, candidate, auditMaze))
            {
                continue;
            }

            const MazeMap::DirectionalLocation maneuverEnd = MazeMap::ManeuverSet::GetSet().Move(code, candidate);
            if (!auditMaze.IsAccessibleLocation(maneuverEnd.GetLocation()))
            {
                continue;
            }
            if (CountClearForwardHalfStepsUntilBlocked(auditMaze, maneuverEnd) == 0U)
            {
                continue;
            }

            launchLocation = candidate;
            return true;
        }

        launchLocation = MazeMap::DirectionalLocation();
        return false;
    }

    static bool TryBuildReverseManeuverPath(
        const MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath)
    {
        reversePath.clear();
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (int index = static_cast<int>(forwardPath.GetSize()) - 1; index >= 0; --index)
        {
            if (!reversePath.push_back(maneuverSet.GetReverseCode(forwardPath[static_cast<uint16_t>(index)])))
            {
                reversePath.clear();
                return false;
            }
        }

        return true;
    }

    static bool TryResolvePositionAuditSmoothTurnHalfSteps(
        MazeMap::ManeuverCode code,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
    {
        switch (code)
        {
        case MazeMap::S90SS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps;
            return true;
        case MazeMap::S90LS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps;
            return true;
        default:
            preTurnHalfSteps = 0U;
            postTurnHalfSteps = 0U;
            return false;
        }
    }

    static bool TryBuildPositionAuditSmoothTurnPaths(
        MazeMap::ManeuverCode code,
        MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
    {
        forwardPath.clear();
        reversePath.clear();
        if (!TryResolvePositionAuditSmoothTurnHalfSteps(code, preTurnHalfSteps, postTurnHalfSteps))
        {
            return false;
        }

        if (!forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(preTurnHalfSteps)) ||
            !forwardPath.push_back(code) ||
            !forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(postTurnHalfSteps)))
        {
            forwardPath.clear();
            return false;
        }

        return TryBuildReverseManeuverPath(forwardPath, reversePath);
    }

    static bool TryValidatePositionAuditPath(
        const MazeMap::Maze& maze,
        const MazeMap::ManeuverPath& path,
        MazeMap::DirectionalLocation start,
        MazeMap::DirectionalLocation& end)
    {
        MazeMap::DirectionalLocation current = start;
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (uint16_t index = 0U; index < path.GetSize(); ++index)
        {
            const MazeMap::ManeuverCode code = path[index];
            if (!maneuverSet.IsValidMove(code, current, maze))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }

            current = maneuverSet.Move(code, current);
            if (!maze.IsAccessibleLocation(current.GetLocation()))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }
        }

        end = current;
        return true;
    }

    MazeMap::Vehicle& _speedVehicle;
    MazeMap::Vehicle& _mappingVehicle;
    MazeMap::Maze& _maze;
    MazeMap::FloodFillPathFinder& _searchPathFinder;
    MazeMap::ManeuverPathFinder& _speedPathFinder;
    MazeMap::WallBeliefMap& _wallBeliefMap;
    SensorSuite& _sensors;
    DiagnosticSensorSuite& _telemetrySensors;
    DiagnosticLogger _telemetryLogger;
    MazeMap::CoreFileExport _missionTextLogFile;
    DriveBase& _drive;
    MazeMap::CellCoordinates _currentCell;
    MazeMap::Direction _currentDirection;
    MazeMap::DirectionalLocation _currentDirectionalLocation;
    bool _goalPauseComplete;
    bool _missionComplete;
    bool _faulted;
    bool _maneuverTestMode;
    bool _telemetryLoggingEnabled;
    bool _missionTextLoggingEnabled;
    bool _missionMazeSnapshotWritten;
    MazeMap::FrontWallCharacterizationStorage _frontWallCharacterization;
    bool _frontWallCharacterizationAvailable;
    float _lastWallTouchStandoffEstimateM;
    bool _hasWallTouchStandoffEstimate;
    unsigned long _lastControlMicros;

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
            _speedVehicle.FrontLeft.GetPosition().GetX(),
            _speedVehicle.FrontRight.GetPosition().GetX());
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
        _drive.SnapTo(_currentDirectionalLocation);
        _lastControlMicros = micros();
    }

    void PrimeKnownMissionStartCell()
    {
        MazeMap::Cell& cell = _maze[MazeMap::CellCoordinates(0, 0)];
        _maze.SetWall(cell, MazeMap::Up, MazeMap::NoWall);
        _maze.SetWall(cell, MazeMap::Down, MazeMap::Wall);
        _maze.SetWall(cell, MazeMap::Left, MazeMap::Wall);
        _maze.SetWall(cell, MazeMap::Right, MazeMap::Wall);
        SeedWallBeliefsFromKnownMaze();
    }

    void ResetForMode(bool maneuverTestMode, bool enableMissionTextLogging)
    {
        _maneuverTestMode = maneuverTestMode;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = enableMissionTextLogging;
        _missionMazeSnapshotWritten = false;
        _goalPauseComplete = false;
        _missionComplete = false;
        _faulted = false;
        _missionTextLogFile.Close();
        _hasWallTouchStandoffEstimate = false;
    }

    void ShutdownTelemetryMode(bool disableFan)
    {
        if (disableFan)
        {
            SetRacingFanEnabled(false);
        }

        _drive.Brake();
        _telemetryLogger.Flush();
        _telemetryLoggingEnabled = false;
        _telemetryLogger.Close();
    }

    bool OpenMissionTextLog()
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        return _missionTextLogFile.Open("logging.txt");
    }

    void FlushMissionTextLog()
    {
        if (_missionTextLoggingEnabled)
        {
            _missionTextLogFile.Flush();
        }
    }

    void CloseMissionTextLog()
    {
        _missionTextLogFile.Close();
    }

    bool WriteMissionTextLineIfEnabled(const char* message)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (message == nullptr || !_missionTextLogFile.IsOpen())
        {
            return false;
        }

        if (!_missionTextLogFile.Write(message) || !_missionTextLogFile.WriteChar('\n'))
        {
            return false;
        }

        _missionTextLogFile.Flush();
        return true;
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
            Serial.print("Mission text logging disabled: ");
            Serial.println(traceLabel);
        }

        FlushMissionTextLog();
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
            Serial.println("Loaded persisted front wall characterization.");
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

    bool CaptureStationaryObservationSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        SensorSnapshot& observationSnapshot)
    {
        SensorSnapshot samples[Config::kSearchRollingObservationSampleCount] = {};
        float frontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount] = {};
        float frontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount] = {};
        for (uint8_t index = 0U; index < Config::kSearchRollingObservationSampleCount; ++index)
        {
            float dtSeconds = 0.0f;
            if (!TickControl(true, dtSeconds, samples[index]))
            {
                return false;
            }
            _drive.Brake();

            float frontLeftDistanceM = NAN;
            float frontRightDistanceM = NAN;
            (void)TryComputeFrontWallCandidateDistancesForPose(
                _drive.GetPose(),
                _speedVehicle,
                observedCell,
                observedDirection,
                frontLeftDistanceM,
                frontRightDistanceM);
            frontLeftCandidateDistanceM[index] = frontLeftDistanceM;
            frontRightCandidateDistanceM[index] = frontRightDistanceM;
        }

        RollingObservationVoteSummary voteSummary{};
        if (!BuildEvidenceObservationSnapshot(
                samples,
                Config::kSearchRollingObservationSampleCount,
                observationSnapshot,
                voteSummary))
        {
            return Fail("Stationary observation majority snapshot is invalid");
        }

        if (!TryApplyFrontWallCharacterizationToObservation(
                observedCell,
                observedDirection,
                "stationary",
                samples,
                frontLeftCandidateDistanceM,
                frontRightCandidateDistanceM,
                Config::kSearchRollingObservationSampleCount,
                observationSnapshot))
        {
            AppendMissionTraceFormatted(
                "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=stationary,fallback_valid=%u",
                observedCell.GetX(),
                observedCell.GetY(),
                DirectionName(observedDirection),
                observationSnapshot.frontWallObservationValid ? 1U : 0U);
        }

        AppendMissionTraceFormatted(
            "mission_observation_stationary,cell=(%d,%d),abs=%s,samples=%u,front_valid=%u,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
            observedCell.GetX(),
            observedCell.GetY(),
            DirectionName(observedDirection),
            static_cast<unsigned>(voteSummary.sampleCount),
            observationSnapshot.frontWallObservationValid ? 1U : 0U,
            static_cast<unsigned>(voteSummary.frontWallVotes),
            static_cast<unsigned>(voteSummary.leftWindowValidVotes),
            static_cast<unsigned>(voteSummary.leftWallVotes),
            static_cast<unsigned>(voteSummary.rightWindowValidVotes),
            static_cast<unsigned>(voteSummary.rightWallVotes));
        return true;
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
        if (_telemetryLoggingEnabled && !_telemetryLogger.WriteEvent("wall_observation", line))
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
            if (WriteMissionTextLineIfEnabled(message))
            {
                return true;
            }

            DisableMissionTextLogging("mission_text_logging:controller_write_failed");
        }

        Serial.println(message);
        return true;
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

    bool LogCorridorRepeatabilityMetadata()
    {
        char line[160] = {};
        if (!_telemetryLogger.WriteEvent(
            "summary",
            "Place the robot in a 5-cell enclosed row like a mission start. This routine runs startup wall calibration, drives to the far end and back at several speeds, and logs closure error at the start cell."))
        {
            return Fail("Unable to write corridor repeatability summary");
        }

        snprintf(line, sizeof(line), "row_cell_count,%u", static_cast<unsigned>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount));
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        const float outDistanceM =
            (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount > 0U) ?
            (Config::kCellSizeM * static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)) :
            0.0f;
        snprintf(line, sizeof(line), "out_distance_m,%.6f", outDistanceM);
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;turn_max_omega_radps,%.6f;turn_accel_radps2,%.6f",
            AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityTurnMaxOmegaRadps,
            AuxMeasurementConfig::kCorridorRepeatabilityTurnAccelRadps2);
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("corridor_repeatability_speed", line))
            {
                return Fail("Unable to write corridor repeatability speed metadata");
            }
        }

        return true;
    }

    bool LogPositionAccuracyAuditMetadata(const PositionAuditFixtureGeometry& geometry)
    {
        char line[320] = {};
        snprintf(
            line,
            sizeof(line),
            "Build a one-cell-wide fixture: normal mission start, a %u-cell north corridor including the start and corner cells, and a %u-cell east extension beyond that corner with solid side walls. All following phases reuse this same fixed geometry.",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount));
        if (!_telemetryLogger.WriteEvent("summary", line))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_straight_result isolates wheel-diameter, straight feedforward, and stop-distance error through north_touch_correction_m, enc_out_err_m, closure_m, and yaw_err_deg."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_in_place_turn_result isolates the shared in-place turn profile through yaw_err_deg, effective_track_width_m, and wall_touch_correction_m."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_smooth_turn_result compares S90SS and S90LS against nominal_radius_m, measured_radius_m, effective_track_width_m, corridor_err_m, and east_touch_correction_m to expose radius-dependent feedforward error."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 1 runs S8, centers in the north corner, turns in place to face down, and runs S8 back to start."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 2 reseats at start, runs S7 + S90SS + S7, centers at the east end, turns to face left, and returns on the reversed maneuver path."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 3 reseats at start, runs S6 + S90LS + S6, recenters at the east end, and returns on the reversed maneuver path."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled &&
            !_telemetryLogger.WriteEvent(
                "summary",
                "Smooth-turn phases run with the mission fan enabled; the existing 2 s ramp to 80% completes before motion begins so high-speed S90 data reflects the intended downforce state."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }

        snprintf(
            line,
            sizeof(line),
            "north_corridor_cells,%u;east_extension_cells,%u;east_total_cells,%u",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount),
            static_cast<unsigned>(geometry.eastTotalCellCount));
        if (!_telemetryLogger.WriteEvent("position_audit", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;start_settle_ms,%u",
            AuxMeasurementConfig::kPositionAuditAccelMps2,
            AuxMeasurementConfig::kPositionAuditDecelMps2,
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditStartSettleMs));
        if (!_telemetryLogger.WriteEvent("position_audit", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "smooth_turn_fan_enabled,%u;kRacingFanDutyCycle,%.6f;kRacingFanRampMs,%u",
            AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled ? 1U : 0U,
            Config::kRacingFanDutyCycle,
            static_cast<unsigned>(Config::kRacingFanRampMs));
        if (!_telemetryLogger.WriteEvent("position_audit", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=1;forward_half_steps=%u;turn=IP180;return_half_steps=%u",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=2;forward=%u,S90SS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=3;forward=%u,S90LS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("position_audit_straight_speed", line))
            {
                return Fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditCornerSpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("position_audit_corner_speed", line))
            {
                return Fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (uint8_t codeIndex = 0U; codeIndex < AuxMeasurementConfig::kPositionAuditSmoothTurnCodeCount; ++codeIndex)
        {
            const MazeMap::ManeuverCode code = AuxMeasurementConfig::kPositionAuditSmoothTurnCodes[codeIndex];
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));
            snprintf(
                line,
                sizeof(line),
                "code=%s;nominal_radius_m=%.6f;distance_m=%.6f",
                codeName,
                MazeMap::ManeuverSet::GetSet()[code].GetNominalTurnRadiusInCells() * Config::kCellSizeM,
                ManeuverDistanceMeters(code));
            if (!_telemetryLogger.WriteEvent("position_audit_turn_code", line))
            {
                return Fail("Unable to write position accuracy audit turn metadata");
            }
        }

        return true;
    }

    MotionLimits CorridorRepeatabilityLimits(float cruiseSpeedMps) const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = AuxMeasurementConfig::kCorridorRepeatabilityTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = AuxMeasurementConfig::kCorridorRepeatabilityTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditStraightLimits(float cruiseSpeedMps) const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kPositionAuditAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kPositionAuditDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditTurnLimits() const
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(0.0f);
        limits.maxSpeedMps = 0.0f;
        return limits;
    }

    MotionLimits PositionAccuracyAuditCornerLimits(float cruiseSpeedMps, float nominalRadiusM) const
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        (void)nominalRadiusM;
        limits.maxAngularSpeedRadps = AuxMeasurementConfig::kPositionAuditCornerMaxOmegaRadps;
        return limits;
    }

    bool WriteCorridorRepeatabilityResult(
        uint8_t speedIndex,
        float cruiseSpeedMps,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry)
    {
        const PoseEstimate& finalPose = _drive.GetPose();
        const DriveTelemetry& finalTelemetry = _drive.GetTelemetry();
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "speed_index=%u;cruise_mps=%.3f;dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f;"
            "left_delta_m=%.6f;right_delta_m=%.6f;left_delta_cnt=%ld;right_delta_cnt=%ld",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg,
            finalTelemetry.leftDistanceM - startTelemetry.leftDistanceM,
            finalTelemetry.rightDistanceM - startTelemetry.rightDistanceM,
            static_cast<long>(finalTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount),
            static_cast<long>(finalTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount));

        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Corridor repeatability result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("corridor_repeatability_result", message))
        {
            return true;
        }
        return Fail("Unable to write corridor repeatability result");
    }

    bool WritePositionStraightAuditResult(
        uint8_t speedIndex,
        float cruiseSpeedMps,
        float northStopErrorM,
        float northTouchCorrectionM,
        float encoderOutErrorM,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry)
    {
        (void)startTelemetry;
        const PoseEstimate& finalPose = _drive.GetPose();
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "speed_idx=%u;v=%.3f;stop_err_m=%.6f;touch_correction_m=%.6f;enc_out_err_m=%.6f;"
            "dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            northStopErrorM,
            northTouchCorrectionM,
            encoderOutErrorM,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position straight result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_straight_result", message))
        {
            return true;
        }
        return Fail("Unable to write position straight result");
    }

    bool WritePositionInPlaceTurnAuditResult(
        MazeMap::Direction targetDirection,
        float touchCorrectionM,
        float leftDeltaM,
        float rightDeltaM,
        float yawChangeRad)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftDeltaM,
            rightDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(DirectionToYawRad(targetDirection), _drive.GetPose().yawRad);

        char message[224] = {};
        const int length =
            haveTrackWidth ?
            snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=%.6f",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG_F * yawChangeRad,
                effectiveTrackWidthM) :
            snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=nan",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG_F * yawChangeRad);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position in-place turn result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_in_place_turn_result", message))
        {
            return true;
        }
        return Fail("Unable to write position in-place turn result");
    }

    bool WritePositionSmoothTurnAuditResult(
        MazeMap::ManeuverCode code,
        uint8_t speedIndex,
        float cruiseSpeedMps,
        float nominalRadiusM,
        float corridorErrorM,
        float eastTouchCorrectionM,
        float leftArcDeltaM,
        float rightArcDeltaM,
        float yawChangeRad,
        float yawErrorDeg)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        float measuredRadiusM = 0.0f;
        const bool haveMeasuredRadius = TryComputeEffectiveTurnRadiusM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            measuredRadiusM);

        char codeName[24] = {};
        FormatManeuverCodeName(code, codeName, sizeof(codeName));
        char measuredRadiusText[24] = {};
        char effectiveTrackWidthText[24] = {};
        if (haveMeasuredRadius)
        {
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "%.6f", measuredRadiusM);
        }
        else
        {
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "nan");
        }
        if (haveTrackWidth)
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "%.6f", effectiveTrackWidthM);
        }
        else
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "nan");
        }

        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "code=%s;speed_idx=%u;v=%.3f;nominal_radius_m=%.6f;measured_radius_m=%s;"
            "effective_track_width_m=%s;yaw_err_deg=%.3f;corridor_err_m=%.6f;east_touch_correction_m=%.6f",
            codeName,
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            nominalRadiusM,
            measuredRadiusText,
            effectiveTrackWidthText,
            yawErrorDeg,
            corridorErrorM,
            eastTouchCorrectionM);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position smooth turn result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_smooth_turn_result", message))
        {
            return true;
        }
        return Fail("Unable to write position smooth turn result");
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

        SnapToStartPose();
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
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> southHeading = DirectionToUnitVector(MazeMap::Down);
        const float farCellCenterYM = (0.5f * Config::kCellSizeM) + outDistanceM;
        const float corridorSpanYM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount);
        const float farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            corridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        const MazeMap::Vectorf<2> farCellCenter(0.5f * Config::kCellSizeM, farCellCenterYM);
        const MazeMap::Vectorf<2> startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);

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

        if (!WriteCorridorRepeatabilityResult(speedIndex, cruiseSpeedMps, startPose, startTelemetry))
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
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> southHeading = DirectionToUnitVector(MazeMap::Down);
        const MazeMap::Vectorf<2> farCellCenter(0.5f * Config::kCellSizeM, geometry.farCellCenterYM);
        const MazeMap::Vectorf<2> startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
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
        if (!WritePositionInPlaceTurnAuditResult(
                MazeMap::Down,
                northTouchCorrectionM,
                leftTurnDeltaM,
                rightTurnDeltaM,
                yawChangeRad))
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

        if (!WritePositionStraightAuditResult(
                speedIndex,
                cruiseSpeedMps,
                northStopErrorM,
                northTouchCorrectionM,
                encoderOutErrorM,
                startPose,
                startTelemetry))
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

        if (!WritePositionInPlaceTurnAuditResult(targetDirection, touchCorrectionM, leftDeltaM, rightDeltaM, yawChangeRad))
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

        const float nominalRadiusM = MazeMap::ManeuverSet::GetSet()[code].GetNominalTurnRadiusInCells() * Config::kCellSizeM;
        const MotionLimits straightLimits = PositionAccuracyAuditStraightLimits(requestedCruiseSpeedMps);
        const MotionLimits cornerLimits = PositionAccuracyAuditCornerLimits(requestedCruiseSpeedMps, nominalRadiusM);
        const MotionLimits calibrationLimits = PositionAccuracyAuditTurnLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const float turnCruiseSpeedMps = ManeuverSpeedLimit(code, cornerLimits);
        if (!(turnCruiseSpeedMps > 0.0f))
        {
            return Fail("Position audit smooth turn speed is invalid");
        }

        const MazeMap::DirectionalLocation launchLocation = auditStart.MoveForward(launchHalfSteps);
        const float postStraightDistanceM = 0.5f * Config::kCellSizeM * static_cast<float>(postStraightHalfSteps);

        float finalTargetXM = 0.0f;
        float finalTargetYM = 0.0f;
        finalLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, finalTargetXM, finalTargetYM);
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> finalHeading = DirectionToUnitVector(finalLocation.GetDirection());
        float launchXM = 0.0f;
        float launchYM = 0.0f;
        launchLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, launchXM, launchYM);
        const MazeMap::Vectorf<2> launchPosition(launchXM, launchYM);
        const MazeMap::Vectorf<2> finalPosition(finalTargetXM, finalTargetYM);
        const float launchDistanceM = launchYM - (0.5f * Config::kCellSizeM);
        const float maneuverDistanceM = ManeuverDistanceMeters(code);
        const float maneuverAngleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
        const bool hasSmoothTurnProfile = TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile);
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
        const float maneuverExitSpeedMps = turnCruiseSpeedMps;
        if (hasSmoothTurnProfile)
        {
            if (!ExecuteSmoothTurnProfile(
                    code,
                    turnCruiseSpeedMps,
                    maneuverExitSpeedMps,
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
        const SensorSnapshot snapshotBeforeTouch = _sensors.Capture(true, _drive.GetPose());
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

        if (!WritePositionSmoothTurnAuditResult(
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
            ApplyAsymmetricQueueLimits(queue, cornerLimits, _speedVehicle, 0.0f, 0.0f);
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
        _lastControlMicros = micros();
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

        _lastControlMicros = micros();
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
        const float headingX = startPose.headingUnit.GetX();
        if (std::fabs(headingX) < 0.5f)
        {
            return Fail("Startup calibration x reposition requires east-west heading");
        }

        const float deltaXMeters = targetXMeters - startPose.xMeters;
        const float signedTravelMeters = deltaXMeters / headingX;
        const MazeMap::Vectorf<2> targetHeading = startPose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(targetXMeters, startPose.yMeters);
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

        _lastControlMicros = micros();
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
        const float headingY = startPose.headingUnit.GetY();
        if (std::fabs(headingY) < 0.5f)
        {
            return Fail("Startup calibration y reposition requires north-south heading");
        }

        const float deltaYMeters = targetYMeters - startPose.yMeters;
        const float signedTravelMeters = deltaYMeters / headingY;
        const MazeMap::Vectorf<2> targetHeading = startPose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(startPose.xMeters, targetYMeters);
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

        _lastControlMicros = micros();
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
        const float headingX = pose.headingUnit.GetX();
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
        outcome = WallTouchOutcome::SeatedContact;
        traveledDistanceM = 0.0f;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = 0.0f;
        }

        MazeMapApp::Internal::Runtime::WallTouchExecutionResult result{};
        const auto tickWallTouch =
            [this](bool stationary, float& dtSeconds, MazeMapApp::Internal::Runtime::WallTouchObservation& observation) -> bool
        {
            SensorSnapshot snapshot{};
            if (!TickControl(stationary, dtSeconds, snapshot))
            {
                return false;
            }

            observation = MazeMapApp::Internal::Runtime::MakeWallTouchObservation(snapshot);
            return true;
        };
        const auto appendTraceLine =
            [this](const char* line)
        {
            if (line != nullptr)
            {
                AppendStartupTrace(line);
            }
        };
        const auto finishWallTouch =
            [this](const char* timeoutMessage) -> bool
        {
            _drive.Brake();
            return HoldBrakedUntilDriveSettles(timeoutMessage, Config::kStartupWallCalibrationSettleMs, 0U);
        };
        const auto onSeatedHold =
            [this, poseResetTarget, seatedYawErrorRad](const MazeMapApp::Internal::Runtime::WallTouchExecutionResult& touchResult) -> bool
        {
            if (seatedYawErrorRad != nullptr)
            {
                *seatedYawErrorRad = touchResult.seatedYawErrorRad;
            }
            if ((poseResetTarget == nullptr) || !poseResetTarget->enabled)
            {
                return true;
            }

            _drive.SetPose(poseResetTarget->xMeters, poseResetTarget->yMeters, poseResetTarget->yawRad);
            _lastControlMicros = micros();
            AppendStartupCalibrationStateTrace("touch_pose_set");
            return true;
        };

        if (!MazeMapApp::Internal::Runtime::ExecuteSharedWallTouchOff(
                _drive,
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                tickWallTouch,
                appendTraceLine,
                finishWallTouch,
                [this](const char* message) -> bool { return Fail(message); },
                onSeatedHold,
                result))
        {
            return false;
        }

        outcome = result.outcome;
        traveledDistanceM = result.seatedTravelM;
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

    bool CaptureAndStoreFrontCalibrationSweep(const MotionLimits& limits, bool& storedBands)
    {
        storedBands = false;
        FrontCalibrationSpinSampleSet<Config::kStartupWallCalibrationFrontSpinMaxSamples> sweepSamples{};
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(limits);
        const float targetSweepAngleRad = static_cast<float>(Config::kStartupWallCalibrationFrontSpinTurnCount) * TWO_PI_F;
        const float captureStepRad = Config::kStartupWallCalibrationFrontSpinCaptureStepRad;
        float commandedOmegaRadps = 0.0f;
        float accumulatedSweepAngleRad = 0.0f;
        float lastStoredSweepAngleRad = -captureStepRad;
        float previousYawRad = _drive.GetPose().yawRad;
        const unsigned long expectedCompletionDeadlineMs =
            millis() +
            static_cast<unsigned long>(
                2500.0f +
                (1000.0f * targetSweepAngleRad / (std::max)(0.25f, limits.maxAngularSpeedRadps)));
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const PoseEstimate& pose = _drive.GetPose();
            const float deltaYawRad = WrapAngleRad(pose.yawRad - previousYawRad);
            previousYawRad = pose.yawRad;
            accumulatedSweepAngleRad += (std::max)(0.0f, deltaYawRad);
            accumulatedSweepAngleRad = (std::min)(accumulatedSweepAngleRad, targetSweepAngleRad);

            if ((accumulatedSweepAngleRad - lastStoredSweepAngleRad) >= captureStepRad)
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
                sweepSamples.Push(
                    MazeMap::ClassifyFrontCalibrationSpinHeadingFromNorth(
                        pose.yawRad,
                        Config::kStartupWallCalibrationFrontNorthOpenHalfWidthRad,
                        Config::kStartupWallCalibrationFrontWallMinEastOfNorthRad,
                        Config::kStartupWallCalibrationFrontWallMaxEastOfNorthRad),
                    frontLeftSample.differentialLight,
                    frontRightSample.differentialLight,
                    frontLeftWallDistanceM,
                    frontRightWallDistanceM);
                lastStoredSweepAngleRad = accumulatedSweepAngleRad;
            }

            const float remainingRad = targetSweepAngleRad - accumulatedSweepAngleRad;
            if (MazeMap::IsInPlaceTurnComplete(remainingRad, pose.angularSpeedRadps, turnProfile))
            {
                _drive.Brake();
                break;
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - millis()) <= 0)
            {
                durationLogged = true;
                AppendStartupTrace("startup_wall_calibration:front_sweep_elapsed_budget_exceeded");
            }

            float angularCommandRadps = 0.0f;
            if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                    remainingRad,
                    pose.angularSpeedRadps,
                    dtSeconds,
                    turnProfile,
                    commandedOmegaRadps,
                    angularCommandRadps))
            {
                return Fail("Startup front calibration sweep profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);
        }

        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_samples,fl_open=%u,fl_wall=%u,fr_open=%u,fr_wall=%u",
            static_cast<unsigned>(sweepSamples.frontLeftOpenCount),
            static_cast<unsigned>(sweepSamples.frontLeftWallCount),
            static_cast<unsigned>(sweepSamples.frontRightOpenCount),
            static_cast<unsigned>(sweepSamples.frontRightWallCount));
        AppendStartupTrace(traceLine);

        const bool storedFrontLeftBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontLeft,
            sweepSamples.frontLeftOpenSamples,
            sweepSamples.frontLeftOpenCount,
            sweepSamples.frontLeftWallSamples,
            sweepSamples.frontLeftWallDistanceSamples,
            sweepSamples.frontLeftWallCount);
        const bool storedFrontRightBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontRight,
            sweepSamples.frontRightOpenSamples,
            sweepSamples.frontRightOpenCount,
            sweepSamples.frontRightWallSamples,
            sweepSamples.frontRightWallDistanceSamples,
            sweepSamples.frontRightWallCount);

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

    bool Initialize(const char* banner, bool observeCurrentCellAfterInit)
    {
        Serial.begin(115200);
        delay(1000);

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace(_maneuverTestMode ? "mode:maneuver_file_test" : "mode:mission");
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

        SnapToStartPose();
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
        if (_telemetryLogger.BeginPhase(name))
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
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "detection_window_cycles,%u",
            static_cast<unsigned>(Config::kWallDetectionAverageWindowCycles));
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_turn_count,%u",
            static_cast<unsigned>(Config::kStartupWallCalibrationFrontSpinTurnCount));
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_capture_step_deg,%.1f",
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontSpinCaptureStepRad);
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "expected_side_distance_m,%.6f",
            gWallDistanceCalibration.GetExpectedSideWallDistanceM());
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "configured_touch_standoff_m,%.6f",
            Config::kWallTouchContactStandoffM);
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
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
            if (!_telemetryLogger.WriteEvent("wall_calibration_curve", line))
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
                if (!_telemetryLogger.WriteEvent("wall_calibration_point", line))
                {
                    return Fail("Unable to write wall calibration point metadata");
                }
            }
        }

        return true;
    }

    bool LogTelemetrySample(bool stationary, uint32_t timestampUs, uint32_t dtUs)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        const DiagnosticSensorSnapshot telemetrySnapshot = _telemetrySensors.Capture(stationary, _drive.GetPose());
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        if (_telemetryLogger.LogSample(stationary, timestampUs, dtUs, _drive.GetPose(), _drive, telemetry, telemetrySnapshot))
        {
            return true;
        }
        return Fail("Failed to write maneuver test sample");
    }

    bool Fail(const char* message)
    {
        _faulted = true;
        SetRacingFanEnabled(false);
        _drive.Brake();
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
            _telemetryLogger.WriteEvent("fault", message);
            _telemetryLogger.Flush();
        }
        FlushMissionTextLog();
        if (_missionTextLoggingEnabled)
        {
            CloseMissionTextLog();
        }
        return false;
    }

    bool TickControl(bool stationary, float& dtSeconds, SensorSnapshot& snapshot)
    {
        while ((micros() - _lastControlMicros) < Config::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        const unsigned long now = micros();
        dtSeconds = static_cast<float>(now - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = now;
        snapshot = _sensors.Capture(stationary, _drive.GetPose());
        _drive.UpdateOdometry(dtSeconds, snapshot, &_maze);
        return LogTelemetrySample(stationary, now, static_cast<uint32_t>(dtSeconds * 1.0e6f));
    }

    bool HoldPosition(uint16_t durationMs, const char* phaseName = nullptr)
    {
        if (phaseName != nullptr && !BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        const unsigned long deadline = millis() + durationMs;
        _drive.Brake();
        while (static_cast<long>(deadline - millis()) > 0)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            _drive.Brake();
        }

        return true;
    }

    bool IsDriveMotionSettled() const
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        return MazeMap::IsMissionStartupStationarySample(
            _drive.GetPose().linearSpeedMps,
            _drive.GetPose().angularSpeedRadps,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    bool HoldBrakedUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        const unsigned long startMs = millis();
        unsigned long stationaryStartMs = 0UL;
        bool stationaryWindowActive = false;
        _drive.Brake();
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            _drive.Brake();
            const bool settled = IsDriveMotionSettled();
            const unsigned long nowMs = millis();
            if (!settled)
            {
                stationaryWindowActive = false;
            }
            else
            {
                if (!stationaryWindowActive)
                {
                    stationaryStartMs = nowMs;
                    stationaryWindowActive = true;
                }
                if ((nowMs - stationaryStartMs) >= stationaryHoldMs)
                {
                    return true;
                }
            }

            if ((timeoutMs > 0U) && ((nowMs - startMs) >= timeoutMs))
            {
                return Fail(timeoutMessage);
            }
        }
    }

    bool HoldZeroVelocityUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        const unsigned long startMs = millis();
        unsigned long stationaryStartMs = 0UL;
        bool stationaryWindowActive = false;
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            _drive.CommandVelocity(0.0f, 0.0f, dtSeconds);
            const bool settled = IsDriveMotionSettled();
            const unsigned long nowMs = millis();
            if (!settled)
            {
                stationaryWindowActive = false;
            }
            else
            {
                if (!stationaryWindowActive)
                {
                    stationaryStartMs = nowMs;
                    stationaryWindowActive = true;
                }
                if ((nowMs - stationaryStartMs) >= stationaryHoldMs)
                {
                    _drive.Brake();
                    return true;
                }
            }

            if ((timeoutMs > 0U) && ((nowMs - startMs) >= timeoutMs))
            {
                return Fail(timeoutMessage);
            }
        }
    }

    bool WaitForMissionStartupStationaryHold()
    {
        if (!EmitMissionControllerLineOrFail("Waiting for 2 s stationary start"))
        {
            return false;
        }
        AppendStartupTrace("startup_stationary_hold:waiting");

        unsigned long stationaryStartMs = 0UL;
        bool stationaryWindowActive = false;
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const DriveTelemetry telemetry = _drive.GetTelemetry();
            const bool stationary = MazeMap::IsMissionStartupStationarySample(
                _drive.GetPose().linearSpeedMps,
                _drive.GetPose().angularSpeedRadps,
                telemetry.leftVelocityMps,
                telemetry.rightVelocityMps,
                Config::kMissionStartupStationarySpeedThresholdMps,
                Config::kMissionStartupStationaryMaxAbsYawRateRadps);

            _drive.Brake();
            if (!stationary)
            {
                stationaryWindowActive = false;
                continue;
            }

            const unsigned long nowMs = millis();
            if (!stationaryWindowActive)
            {
                stationaryStartMs = nowMs;
                stationaryWindowActive = true;
            }

            if ((nowMs - stationaryStartMs) >= Config::kMissionStartupStationaryHoldMs)
            {
                AppendStartupTrace("startup_stationary_hold:complete");
                return true;
            }
        }
    }

    bool ExecuteReverseStraightProfile(
        float distanceM,
        const MotionLimits& limits,
        const MazeMap::Vectorf<2>* targetHeadingOverride = nullptr,
        const MazeMap::Vectorf<2>* targetPositionOverride = nullptr)
    {
        if (!(std::isfinite(distanceM) && distanceM > 0.0f))
        {
            return true;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        float commandedSpeedMps = 0.0f;
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        bool projectionFallbackLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if (targetPositionOverride != nullptr)
            {
                const PoseEstimate& pose = _drive.GetPose();
                float projectedRemainingM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    pose.xMeters,
                    pose.yMeters,
                    targetPositionOverride->GetX(),
                    targetPositionOverride->GetY(),
                    -targetHeading.GetX(),
                    -targetHeading.GetY(),
                    projectedRemainingM))
                {
                    if (!projectionFallbackLogged)
                    {
                        projectionFallbackLogged = true;
                        AppendStartupTrace("reverse_profile:projection_fallback_to_encoder_distance");
                    }
                }
                else
                {
                    remainingM = (std::max)(0.0f, projectedRemainingM);
                }
            }
            if ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled())
            {
                _drive.Brake();
                return HoldPosition(Config::kMotionSettleHoldMs);
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                AppendStartupTrace("reverse_profile:encoder_progress_stalled_holding_position");
                return HoldPosition(Config::kMotionSettleHoldMs);
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                _drive.Brake();
                AppendStartupTrace("reverse_profile:elapsed_budget_reached_holding_position");
                return HoldPosition(Config::kMotionSettleHoldMs);
            }

            const float accelLimitedSpeedMps = (std::min)(limits.maxSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(-commandedSpeedMps, angularCommandRadps, dtSeconds);
        }
    }

    bool LoadManeuverQueueFromSd(const char* fileName, MazeMap::ManeuverQueue& queue)
    {
#if defined(ARDUINO_TEENSY41)
        File file = SD.open(fileName, FILE_READ);
        if (!file)
        {
            AppendStartupTrace("maneuver_test:test_file_unavailable");
            Serial.println("Maneuver file unavailable; skipping maneuver-file test");
            return false;
        }
        AppendStartupTrace("maneuver_test:test_txt_opened");

        MazeMap::ManeuverPath path;
        char line[128] = {};
        uint16_t lineNumber = 0U;
        while (file.available())
        {
            const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
            line[lineLength] = '\0';
            ++lineNumber;

            char* hashComment = strchr(line, '#');
            if (hashComment != nullptr)
            {
                *hashComment = '\0';
            }

            char* slashComment = strstr(line, "//");
            if (slashComment != nullptr)
            {
                *slashComment = '\0';
            }

            for (char* token = strtok(line, ", \t\r;"); token != nullptr; token = strtok(nullptr, ", \t\r;"))
            {
                MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                if (!TryParseManeuverCodeToken(token, code))
                {
                    char message[96] = {};
                    snprintf(message, sizeof(message), "Maneuver file token issue on line %u: %s", lineNumber, token);
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_parse_issue");
                    Serial.println(message);
                    return false;
                }
                if (!path.push_back(code))
                {
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_path_capacity_reached");
                    Serial.println("Maneuver file exceeded path capacity; skipping maneuver-file test");
                    return false;
                }
            }
        }

        file.close();

        if (path.GetSize() == 0)
        {
            AppendStartupTrace("maneuver_test:test_file_empty");
            Serial.println("Maneuver file did not contain any maneuvers");
            return false;
        }
        AppendStartupTrace("maneuver_test:path_parsed");

        queue.clear();
        if (!queue.push_back(path, _currentDirectionalLocation))
        {
            AppendStartupTrace("maneuver_test:queue_build_issue");
            Serial.println("Maneuver file could not be converted into a queue");
            return false;
        }
        AppendStartupTrace("maneuver_test:queue_built");

        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
        AppendStartupTrace("maneuver_test:speeds_computed");
        return LogLoadedManeuverQueue(queue);
#else
        (void)fileName;
        (void)queue;
        AppendStartupTrace("maneuver_test:teensy_target_required");
        Serial.println("Maneuver-file test mode requires the Teensy target");
        return false;
#endif
    }

    bool LogLoadedManeuverQueue(const MazeMap::ManeuverQueue& queue)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char message[128] = {};
        snprintf(message, sizeof(message), "count,%u", static_cast<unsigned>(queue.size()));
        if (!_telemetryLogger.WriteEvent("queue", message))
        {
            AppendStartupTrace("maneuver_test:queue_logging_disabled");
            Serial.println("Maneuver queue logging unavailable; continuing without queue metadata");
            _telemetryLogger.Flush();
            _telemetryLogger.Close();
            _telemetryLoggingEnabled = false;
            return true;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            char codeName[24] = {};
            char queueLine[160] = {};
            FormatManeuverCodeName(queue[i].GetCode(), codeName, sizeof(codeName));
            snprintf(
                queueLine,
                sizeof(queueLine),
                "%u,%s,%.6f,%.6f",
                static_cast<unsigned>(i),
                codeName,
                queue[i].GetEntrySpeed(),
                queue[i].GetExitSpeed());

            if (!_telemetryLogger.WriteEvent("queue_entry", queueLine))
            {
                AppendStartupTrace("maneuver_test:queue_logging_disabled");
                Serial.println("Maneuver queue entry logging unavailable; continuing without queue metadata");
                _telemetryLogger.Flush();
                _telemetryLogger.Close();
                _telemetryLoggingEnabled = false;
                return true;
            }
        }

        return true;
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, bool snapToExpectedLocation)
    {
        if (!EmitMissionControllerFormattedOrFail("Queued maneuvers: %u", static_cast<unsigned>(queue.size())))
        {
            return false;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            const MazeMap::ManeuverInstance& entry = queue[i];
            const MazeMap::ManeuverCode code = entry.GetCode();
            const float entrySpeed = entry.GetEntrySpeed();
            const float exitSpeed = entry.GetExitSpeed();
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));

            AppendMissionTraceFormatted(
                "mission_maneuver:begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                entrySpeed,
                exitSpeed);

            if (_maneuverTestMode)
            {
                char phaseName[48] = {};
                snprintf(phaseName, sizeof(phaseName), "maneuver_%u_%s", static_cast<unsigned>(i), codeName);
                if (!BeginTelemetryPhase(phaseName))
                {
                    return false;
                }
            }

            bool ok = false;
            if (IsStraightCode(code))
            {
                ok = ExecuteStraightProfile(
                    0.5f * Config::kCellSizeM * static_cast<float>(static_cast<uint8_t>(code)),
                    entrySpeed,
                    limits.maxSpeedMps,
                    exitSpeed,
                    limits,
                    true);
            }
            else
            {
                const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
                MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
                if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
                {
                    const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                    ok = ExecuteSmoothTurnProfile(code, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                }
                else
                {
                    const float distanceM = ManeuverDistanceMeters(code);
                    if (distanceM <= 0.0f)
                    {
                        ok = ExecuteTurnProfile(angleRad, limits);
                    }
                    else
                    {
                        const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                        ok = ExecuteArcProfile(distanceM, angleRad, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                    }
                }
            }

            if (!ok)
            {
                return false;
            }

            _currentDirectionalLocation = entry.GetEnd();
            _currentDirection = _currentDirectionalLocation.GetDirection();
            _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());

            AppendMissionTraceFormatted(
                "mission_maneuver:end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                _drive.GetPose().xMeters,
                _drive.GetPose().yMeters,
            RAD_TO_DEG_F * _drive.GetPose().yawRad);

            if (snapToExpectedLocation)
            {
                _drive.SnapTo(_currentDirectionalLocation);
            }
        }

        return HoldPosition(50);
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, bool snapToExpectedLocation)
    {
        return ExecuteQueuedManeuvers(queue, FinalLimits(), snapToExpectedLocation);
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
                ApplyAsymmetricQueueLimits(queue, returnLimits, _mappingVehicle, 0.0f, 0.0f);
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
        if (!EmitMissionControllerLineOrFail("Install 34-35 jumper before lifting for tire service"))
        {
            return false;
        }
        _drive.Brake();
        while (!IsInterRunServiceJumperInstalled())
        {
            delay(Config::kInterRunServicePollMs);
        }

        if (!EmitMissionControllerLineOrFail("Service jumper detected; place robot back at start facing up and remove jumper"))
        {
            return false;
        }
        while (IsInterRunServiceJumperInstalled())
        {
            _drive.Brake();
            delay(Config::kInterRunServicePollMs);
        }

        return PrepareForSecondSpeedRun();
    }

    bool PrepareForSecondSpeedRun()
    {
        if (!_sensors.Begin())
        {
            return Fail("Sensor reset failed after inter-run service");
        }

        SnapToStartPose();
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
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
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
        (void)snapAtEnd;
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = false;
        }

        if (cellCount == 0U)
        {
            return true;
        }

        const MazeMap::CellCoordinates startCell = _currentCell;
        MazeMap::CellCoordinates destination = startCell;
        for (uint16_t i = 0U; i < cellCount; ++i)
        {
            destination = destination >> direction;
        }

        const MazeMap::Vectorf<2> targetHeading = DirectionToUnitVector(direction);
        float targetXMeters = 0.0f;
        float targetYMeters = 0.0f;
        MazeMap::MazeLocation::CellCenter(destination).GetPhysicalLocation(Config::kCellSizeM, targetXMeters, targetYMeters);

        const PoseEstimate startPose = _drive.GetPose();
        float distanceToTargetM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
            startPose.xMeters,
            startPose.yMeters,
            targetXMeters,
            targetYMeters,
            targetHeading.GetX(),
            targetHeading.GetY(),
            distanceToTargetM))
        {
            return Fail("Search straight target distance is invalid");
        }
        if (distanceToTargetM < -Config::kDistanceToleranceM)
        {
            return Fail("Search straight target fell behind the current pose");
        }

        const float sideSensorForwardOffsetM =
            (std::max)(_speedVehicle.SideLeft.GetPosition().GetX(), _speedVehicle.SideRight.GetPosition().GetX());
        uint16_t rollingObservationCount = 0U;
        MazeMap::CellCoordinates nextRollingObservationCell = startCell;
        float rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount] = {};
        SensorSnapshot rollingObservationSamples[Config::kSearchRollingObservationSampleCount] = {};
        float rollingObservationFrontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount] = {};
        float rollingObservationFrontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount] = {};
        float rollingObservationSideResetTriggerTravelM = 0.0f;
        uint8_t rollingObservationNextSampleIndex = 0U;
        bool rollingObservationSideResetPending = false;
        bool rollingObservationPlanInitialized = false;
        if (observeWhileRolling)
        {
            nextRollingObservationCell = nextRollingObservationCell >> direction;
        }

        const auto resetRollingObservationPlan = [&]()
        {
            rollingObservationNextSampleIndex = 0U;
            rollingObservationPlanInitialized = false;
            rollingObservationSideResetTriggerTravelM = 0.0f;
            rollingObservationSideResetPending = false;
            memset(rollingObservationTriggerTravelM, 0, sizeof(rollingObservationTriggerTravelM));
            memset(rollingObservationSamples, 0, sizeof(rollingObservationSamples));
            for (uint8_t sampleIndex = 0U; sampleIndex < Config::kSearchRollingObservationSampleCount; ++sampleIndex)
            {
                rollingObservationFrontLeftCandidateDistanceM[sampleIndex] = NAN;
                rollingObservationFrontRightCandidateDistanceM[sampleIndex] = NAN;
            }
        };

        const auto initializeRollingObservationPlan = [&]() -> bool
        {
            if (!observeWhileRolling || rollingObservationCount >= cellCount)
            {
                return true;
            }
            if (rollingObservationPlanInitialized)
            {
                return true;
            }

            // WARNING: Mapping observation here is intentionally one constant-velocity traversal through the cell.
            // Do not add any parallel traversal mechanism, extra pass, or speed-shape change here. The only permitted
            // refinement is to move the observation timing inside the target region while the chassis keeps the same
            // steady mapping straight.
            for (uint8_t sampleIndex = 0U; sampleIndex < Config::kSearchRollingObservationSampleCount; ++sampleIndex)
            {
                float targetObservationXMeters = 0.0f;
                float targetObservationYMeters = 0.0f;
                if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
                        nextRollingObservationCell,
                        direction,
                        Config::kCellSizeM,
                        Config::kMazeWallThicknessM,
                        sideSensorForwardOffsetM,
                        Config::kSideWallSegmentCenterFraction,
                        sampleIndex,
                        Config::kSearchRollingObservationSampleCount,
                        targetObservationXMeters,
                        targetObservationYMeters))
                {
                    return Fail("Search straight rolling observation sample pose is invalid");
                }

                float triggerTravelM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                        startPose.xMeters,
                        startPose.yMeters,
                        targetObservationXMeters,
                        targetObservationYMeters,
                        targetHeading.GetX(),
                        targetHeading.GetY(),
                        triggerTravelM))
                {
                    return Fail("Search straight rolling observation sample trigger is invalid");
                }
                if (sampleIndex > 0U &&
                    triggerTravelM < (rollingObservationTriggerTravelM[sampleIndex - 1U] - Config::kDistanceToleranceM))
                {
                    AppendMissionTraceFormatted(
                        "mission_observation_trigger_recovered,cell=(%d,%d),abs=%s,sample=%u,prev_m=%.4f,raw_m=%.4f",
                        nextRollingObservationCell.GetX(),
                        nextRollingObservationCell.GetY(),
                        DirectionName(direction),
                        static_cast<unsigned>(sampleIndex),
                        rollingObservationTriggerTravelM[sampleIndex - 1U],
                        triggerTravelM);
                    triggerTravelM = rollingObservationTriggerTravelM[sampleIndex - 1U];
                }

                rollingObservationTriggerTravelM[sampleIndex] = triggerTravelM;
            }

            float targetResetXMeters = 0.0f;
            float targetResetYMeters = 0.0f;
            if (!MazeMap::TryComputeSideWallTravelFractionPoseM(
                    nextRollingObservationCell,
                    direction,
                    Config::kCellSizeM,
                    sideSensorForwardOffsetM,
                    Config::kSideWallStateResetCellEntryFraction,
                    targetResetXMeters,
                    targetResetYMeters))
            {
                return Fail("Search straight side reset trigger pose is invalid");
            }

            float resetTriggerTravelM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    startPose.xMeters,
                    startPose.yMeters,
                    targetResetXMeters,
                    targetResetYMeters,
                    targetHeading.GetX(),
                    targetHeading.GetY(),
                    resetTriggerTravelM))
            {
                return Fail("Search straight side reset trigger is invalid");
            }
            if (resetTriggerTravelM > (rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM))
            {
                AppendMissionTraceFormatted(
                    "mission_side_reset_trigger_recovered,cell=(%d,%d),abs=%s,raw_m=%.4f,first_sample_m=%.4f",
                    nextRollingObservationCell.GetX(),
                    nextRollingObservationCell.GetY(),
                    DirectionName(direction),
                    resetTriggerTravelM,
                    rollingObservationTriggerTravelM[0]);
                resetTriggerTravelM = (std::max)(0.0f, rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM);
            }

            rollingObservationSideResetTriggerTravelM = resetTriggerTravelM;
            rollingObservationSideResetPending = true;

            rollingObservationPlanInitialized = true;
            return true;
        };

        bool rollingObservationStoppedForReplan = false;
        const auto tryObserveRollingCells = [&](float projectedTravelM, const PoseEstimate& livePose, const SensorSnapshot& liveSnapshot) -> bool
        {
            if (!observeWhileRolling)
            {
                return true;
            }

            while (rollingObservationCount < cellCount)
            {
                if (!initializeRollingObservationPlan())
                {
                    return false;
                }

                if (rollingObservationSideResetPending &&
                    (projectedTravelM + Config::kDistanceToleranceM) >= rollingObservationSideResetTriggerTravelM)
                {
                    _sensors.ResetSideWallMemory();
                    rollingObservationSideResetPending = false;
                    AppendMissionTraceFormatted(
                        "mission_side_reset,cell=(%d,%d),abs=%s,travel_m=%.4f,trigger_m=%.4f",
                        nextRollingObservationCell.GetX(),
                        nextRollingObservationCell.GetY(),
                        DirectionName(direction),
                        projectedTravelM,
                        rollingObservationSideResetTriggerTravelM);
                    break;
                }

                while (rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount &&
                    (projectedTravelM + Config::kDistanceToleranceM) >= rollingObservationTriggerTravelM[rollingObservationNextSampleIndex])
                {
                    rollingObservationSamples[rollingObservationNextSampleIndex] = liveSnapshot;
                    float frontLeftCandidateDistanceM = NAN;
                    float frontRightCandidateDistanceM = NAN;
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontLeft,
                        nextRollingObservationCell,
                        direction,
                        frontLeftCandidateDistanceM);
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontRight,
                        nextRollingObservationCell,
                        direction,
                        frontRightCandidateDistanceM);
                    rollingObservationFrontLeftCandidateDistanceM[rollingObservationNextSampleIndex] = frontLeftCandidateDistanceM;
                    rollingObservationFrontRightCandidateDistanceM[rollingObservationNextSampleIndex] = frontRightCandidateDistanceM;
                    ++rollingObservationNextSampleIndex;
                }

                if (rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount)
                {
                    break;
                }

                SensorSnapshot majoritySnapshot{};
                RollingObservationVoteSummary voteSummary{};
                if (!BuildEvidenceObservationSnapshot(
                        rollingObservationSamples,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot,
                        voteSummary))
                {
                    return Fail("Search straight rolling observation majority snapshot is invalid");
                }
                if (!TryApplyFrontWallCharacterizationToObservation(
                        nextRollingObservationCell,
                        direction,
                        "rolling",
                        rollingObservationSamples,
                        rollingObservationFrontLeftCandidateDistanceM,
                        rollingObservationFrontRightCandidateDistanceM,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot))
                {
                    AppendMissionTraceFormatted(
                        "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=rolling,fallback_valid=%u",
                        nextRollingObservationCell.GetX(),
                        nextRollingObservationCell.GetY(),
                        DirectionName(direction),
                        majoritySnapshot.frontWallObservationValid ? 1U : 0U);
                }

                AppendMissionTraceFormatted(
                    "mission_observation_timed,cell=(%d,%d),abs=%s,samples=%u,start_m=%.4f,end_m=%.4f,travel_m=%.4f,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
                    nextRollingObservationCell.GetX(),
                    nextRollingObservationCell.GetY(),
                    DirectionName(direction),
                    static_cast<unsigned>(voteSummary.sampleCount),
                    rollingObservationTriggerTravelM[0],
                    rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount - 1U],
                    projectedTravelM,
                    static_cast<unsigned>(voteSummary.frontWallVotes),
                    static_cast<unsigned>(voteSummary.leftWindowValidVotes),
                    static_cast<unsigned>(voteSummary.leftWallVotes),
                    static_cast<unsigned>(voteSummary.rightWindowValidVotes),
                    static_cast<unsigned>(voteSummary.rightWallVotes));
                bool forwardWallCommittedFromUnknown = false;
                if (!ObserveCellFromSnapshot(
                        nextRollingObservationCell,
                        direction,
                        majoritySnapshot,
                        &forwardWallCommittedFromUnknown))
                {
                    return false;
                }

                if (forwardWallCommittedFromUnknown)
                {
                    if (!HandleSearchWallMapUpdateStop(
                            nextRollingObservationCell,
                            direction,
                            projectedTravelM,
                            voteSummary.frontWallVotes,
                            outStoppedForReplan))
                    {
                        return false;
                    }
                    rollingObservationStoppedForReplan = true;
                    return true;
                }

                ++rollingObservationCount;
                if (rollingObservationCount < cellCount)
                {
                    nextRollingObservationCell = nextRollingObservationCell >> direction;
                }
                resetRollingObservationPlan();
            }

            return true;
        };

        const MotionLimits searchLimits = SearchLimits();
        float commandedSpeedMps = (std::max)(entrySpeedMps, 0.0f);
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceToTargetM));
        bool stallLogged = false;
        bool durationLogged = false;
        float previousCorridorErrorM = 0.0f;
        float filteredCorridorErrorRateMps = 0.0f;
        bool previousCorridorErrorValid = false;
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }

            const PoseEstimate& livePose = _drive.GetPose();
            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    livePose.xMeters,
                    livePose.yMeters,
                    targetXMeters,
                    targetYMeters,
                    targetHeading.GetX(),
                    targetHeading.GetY(),
                    projectedRemainingM))
            {
                return Fail("Search straight target projection is invalid");
            }
            const float remainingM = (std::max)(0.0f, projectedRemainingM);
            const float projectedTravelM = (std::clamp)(distanceToTargetM - projectedRemainingM, 0.0f, distanceToTargetM);

            if (!tryObserveRollingCells(projectedTravelM, livePose, snapshot))
            {
                return false;
            }
            if (rollingObservationStoppedForReplan)
            {
                return true;
            }

            const bool stoppingAtEndpoint = exitSpeedMps <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeedMps) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }

            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(projectedTravelM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=search_straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    projectedTravelM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=search_straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    projectedTravelM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeedMps, commandedSpeedMps + (searchLimits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeedMps, remainingM, searchLimits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            float wallOmegaRadps = 0.0f;
            float signalCorridorErrorM = 0.0f;
            if (TryComputeWallGroundedCorridorErrorM(snapshot, signalCorridorErrorM))
            {
                wallOmegaRadps += ComputeWallCenterPdOmegaRadps(
                    signalCorridorErrorM,
                    commandedSpeedMps,
                    dtSeconds,
                    previousCorridorErrorM,
                    filteredCorridorErrorRateMps,
                    previousCorridorErrorValid);
            }
            else
            {
                filteredCorridorErrorRateMps = 0.0f;
                previousCorridorErrorValid = false;
            }
            if (stoppingAtEndpoint &&
                std::isfinite(snapshot.frontLeftDistanceM) &&
                std::isfinite(snapshot.frontRightDistanceM) &&
                snapshot.frontLeftDistanceM < Config::kFrontWallOnThresholdM &&
                snapshot.frontRightDistanceM < Config::kFrontWallOnThresholdM &&
                remainingM < 0.07f)
            {
                wallOmegaRadps += Config::kFrontSkewGain * snapshot.frontSkewM;
            }

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps) + wallOmegaRadps;
            angularCommandRadps = (std::clamp)(angularCommandRadps, -searchLimits.maxAngularSpeedRadps, searchLimits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeedMps <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }

        _currentCell = destination;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
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

    struct MapQualifiedSideWallReference
    {
        bool useWall = false;
        MazeMap::CellCoordinates cell = MazeMap::CellCoordinates(0U, 0U);
        MazeMap::Direction wallDirection = MazeMap::None;
    };

    bool TryResolveMapQualifiedSideWallReference(
        const PoseEstimate& pose,
        const MazeMap::WallSensor& sensor,
        bool distanceValidForControl,
        MapQualifiedSideWallReference& reference) const
    {
        // Exclusively for the purpose of centering.
        reference = MapQualifiedSideWallReference{};
        if (!distanceValidForControl)
        {
            return false;
        }

        float alongWallCoordinateM = 0.0f;
        if (!TryComputeSideWallAimCoordinateM(pose, sensor, alongWallCoordinateM))
        {
            return false;
        }

        const MazeMap::Vectorf<2> worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
        const float sensorXM = pose.xMeters + worldOffset.GetX();
        const float sensorYM = pose.yMeters + worldOffset.GetY();
        const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);

        int cellX = -1;
        int cellY = -1;
        MazeMap::Direction wallDirection = MazeMap::None;
        if (std::fabs(sensorFacing.GetX()) >= std::fabs(sensorFacing.GetY()))
        {
            if (!std::isfinite(sensorXM) || !std::isfinite(alongWallCoordinateM))
            {
                return false;
            }

            cellX = static_cast<int>(std::floor(sensorXM / Config::kCellSizeM));
            cellY = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
            wallDirection = (sensorFacing.GetX() >= 0.0f) ? MazeMap::Right : MazeMap::Left;
        }
        else
        {
            if (!std::isfinite(sensorYM) || !std::isfinite(alongWallCoordinateM))
            {
                return false;
            }

            cellX = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
            cellY = static_cast<int>(std::floor(sensorYM / Config::kCellSizeM));
            wallDirection = (sensorFacing.GetY() >= 0.0f) ? MazeMap::Up : MazeMap::Down;
        }

        if (cellX < 0 || cellY < 0 ||
            cellX >= static_cast<int>(_maze.GetXSize()) ||
            cellY >= static_cast<int>(_maze.GetYSize()))
        {
            return false;
        }

        const MazeMap::Cell& observedCell = _maze.Index(cellX, cellY);
        if (observedCell.GetWall(wallDirection) != MazeMap::Wall)
        {
            return false;
        }

        reference.useWall = true;
        reference.cell = MazeMap::CellCoordinates(static_cast<uint8_t>(cellX), static_cast<uint8_t>(cellY));
        reference.wallDirection = wallDirection;
        return true;
    }

    void ResolveMapQualifiedSideWalls(const SensorSnapshot& snapshot, bool& useLeftWall, bool& useRightWall) const
    {
        // Exclusively for the purpose of centering.
        const PoseEstimate& pose = _drive.GetPose();
        MapQualifiedSideWallReference leftReference{};
        MapQualifiedSideWallReference rightReference{};
        useLeftWall = TryResolveMapQualifiedSideWallReference(
            pose,
            _speedVehicle.SideLeft,
            snapshot.leftDistanceValidForControl,
            leftReference);
        useRightWall = TryResolveMapQualifiedSideWallReference(
            pose,
            _speedVehicle.SideRight,
            snapshot.rightDistanceValidForControl,
            rightReference);
    }

    bool TryComputeWallGroundedCorridorCoordinateM(const SensorSnapshot& snapshot, float& coordinateM, bool& correctsXAxis) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        switch (_currentDirection)
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

        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate = false;
        bool haveRightCoordinate = false;
        const PoseEstimate& pose = _drive.GetPose();
        MapQualifiedSideWallReference leftReference{};
        MapQualifiedSideWallReference rightReference{};

        if (TryResolveMapQualifiedSideWallReference(
                pose,
                _speedVehicle.SideLeft,
                snapshot.leftDistanceValidForControl,
                leftReference))
        {
            haveLeftCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                leftReference.cell,
                leftReference.wallDirection,
                leftCoordinateM);
        }

        if (TryResolveMapQualifiedSideWallReference(
                pose,
                _speedVehicle.SideRight,
                snapshot.rightDistanceValidForControl,
                rightReference))
        {
            haveRightCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                rightReference.cell,
                rightReference.wallDirection,
                rightCoordinateM);
        }

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        coordinateM = haveLeftCoordinate && haveRightCoordinate ?
            (0.5f * (leftCoordinateM + rightCoordinateM)) :
            (haveLeftCoordinate ? leftCoordinateM : rightCoordinateM);
        return std::isfinite(coordinateM);
    }

    bool TryComputeWallGroundedCorridorErrorM(const SensorSnapshot& snapshot, float& corridorErrorM) const
    {
        corridorErrorM = 0.0f;

        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
        {
            return false;
        }

        float centerXM = 0.0f;
        float centerYM = 0.0f;
        if (!TryGetCellCenterMeters(_currentCell, centerXM, centerYM))
        {
            return false;
        }

        const float errorXM = correctsXAxis ? (corridorCoordinateM - centerXM) : 0.0f;
        const float errorYM = correctsXAxis ? 0.0f : (corridorCoordinateM - centerYM);
        const MazeMap::Vectorf<2> heading = DirectionToUnitVector(_currentDirection);
        corridorErrorM = (heading.GetY() * errorXM) - (heading.GetX() * errorYM);
        return std::isfinite(corridorErrorM);
    }

    static float ComputeWallCenterPdOmegaRadps(
        float corridorErrorM,
        float forwardSpeedMps,
        float dtSeconds,
        float& previousCorridorErrorM,
        float& filteredCorridorErrorRateMps,
        bool& previousCorridorErrorValid)
    {
        // Exclusively for the purpose of centering.
        float corridorErrorRateMps = filteredCorridorErrorRateMps;
        if (previousCorridorErrorValid && (dtSeconds > 0.0f))
        {
            const float rawCorridorErrorRateMps = (corridorErrorM - previousCorridorErrorM) / dtSeconds;
            const float derivativeAlpha =
                dtSeconds / (Config::kWallCenterDerivativeFilterTauSeconds + dtSeconds);
            corridorErrorRateMps += derivativeAlpha * (rawCorridorErrorRateMps - corridorErrorRateMps);
        }
        else
        {
            corridorErrorRateMps = 0.0f;
        }

        previousCorridorErrorM = corridorErrorM;
        filteredCorridorErrorRateMps = corridorErrorRateMps;
        previousCorridorErrorValid = true;
        const float rawWallCenterOmegaRadps =
            (Config::kWallCenterGain * corridorErrorM) +
            (Config::kWallCenterD * corridorErrorRateMps);
        const float maxCenteringCurvatureMInv =
            (2.0f * Config::kWallCenterMaxClosurePerCellM) /
            (Config::kCellSizeM * Config::kCellSizeM);
        const float maxWallCenterOmegaRadps = std::fabs(forwardSpeedMps) * maxCenteringCurvatureMInv;
        return (std::clamp)(
            rawWallCenterOmegaRadps,
            -maxWallCenterOmegaRadps,
            maxWallCenterOmegaRadps);
    }

    bool ApplyWallGroundedCorridorPoseCorrection(const SensorSnapshot& snapshot)
    {
        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
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
        const MazeMap::Vectorf<2>* targetHeadingOverride = nullptr,
        const MazeMap::Vectorf<2>* targetPositionOverride = nullptr)
    {
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        const bool diagonalHeading = IsApproximatelyDiagonalHeadingUnit(targetHeading);
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        bool stallLogged = false;
        bool durationLogged = false;
        float previousCorridorErrorM = 0.0f;
        float filteredCorridorErrorRateMps = 0.0f;
        bool previousCorridorErrorValid = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if (targetPositionOverride != nullptr)
            {
                const PoseEstimate& pose = _drive.GetPose();
                float projectedRemainingM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    pose.xMeters,
                    pose.yMeters,
                    targetPositionOverride->GetX(),
                    targetPositionOverride->GetY(),
                    targetHeading.GetX(),
                    targetHeading.GetY(),
                    projectedRemainingM))
                {
                    return Fail("Straight target projection is invalid");
                }
                remainingM = (std::max)(0.0f, projectedRemainingM);
            }
            const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }
            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            float wallOmegaRadps = 0.0f;
            if (useWallCentering)
            {
                if (diagonalHeading)
                {
                    wallOmegaRadps += ComputeDiagonalWallCenterOmegaRadps(
                        gWallDistanceCalibration,
                        snapshot.sideLeftDifferentialLight,
                        snapshot.sideRightDifferentialLight);
                }
                else
                {
                    float signalCorridorErrorM = 0.0f;
                    if (TryComputeWallGroundedCorridorErrorM(snapshot, signalCorridorErrorM))
                    {
                        wallOmegaRadps += ComputeWallCenterPdOmegaRadps(
                            signalCorridorErrorM,
                            commandedSpeedMps,
                            dtSeconds,
                            previousCorridorErrorM,
                            filteredCorridorErrorRateMps,
                            previousCorridorErrorValid);
                    }
                    else
                    {
                        filteredCorridorErrorRateMps = 0.0f;
                        previousCorridorErrorValid = false;
                    }
                    if (stoppingAtEndpoint &&
                        std::isfinite(snapshot.frontLeftDistanceM) &&
                        std::isfinite(snapshot.frontRightDistanceM) &&
                        snapshot.frontLeftDistanceM < Config::kFrontWallOnThresholdM &&
                        snapshot.frontRightDistanceM < Config::kFrontWallOnThresholdM &&
                        remainingM < 0.07f)
                    {
                        wallOmegaRadps += Config::kFrontSkewGain * snapshot.frontSkewM;
                    }
                }
            }
            else
            {
                filteredCorridorErrorRateMps = 0.0f;
                previousCorridorErrorValid = false;
            }

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps) + wallOmegaRadps;
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeed <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }
        return true;
    }

    bool ExecuteTurnProfile(
        float angleRad,
        const MotionLimits& limits,
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr)
    {
        const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(limits);
        float commandedOmegaRadps = 0.0f;
        const unsigned long expectedCompletionDeadlineMs = millis() + 2500UL;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            if (wallEdgeTracker != nullptr)
            {
                MazeMap::ObserveTurnWallStates(*wallEdgeTracker, snapshot.leftWall, snapshot.rightWall);
            }

            const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
            {
                break;
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - millis()) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=turn,reason=elapsed_budget_exceeded,cell=(%d,%d),yaw_err_deg=%.2f,w_radps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    RAD_TO_DEG_F * errorRad,
                    _drive.GetPose().angularSpeedRadps);
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
                return Fail("Turn profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);
        }

        if (!HoldZeroVelocityUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
        {
            return false;
        }
        return true;
    }

    bool ExecuteArcProfile(float distanceM, float angleRad, float entrySpeed, float exitSpeed, float cruiseSpeed, const MotionLimits& limits)
    {
        if (distanceM <= 0.0f)
        {
            return ExecuteTurnProfile(angleRad, limits);
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const float curvature = angleRad / distanceM;
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }
            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=arc,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=arc,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float progress = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
            const float targetYawRad = WrapAngleRad(startYawRad + (angleRad * progress));
            const float headingErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            float angularCommandRadps = (curvature * commandedSpeedMps) + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeed <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }
        return true;
    }

    bool ExecuteSmoothTurnProfile(
        MazeMap::ManeuverCode code,
        float entrySpeed,
        float exitSpeed,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        MazeMap::SmoothTurnExecutionProfile profile{};
        if (!TryGetSmoothTurnExecutionProfileMeters(code, profile))
        {
            return Fail("Smooth turn geometry is unavailable");
        }

        float maneuverSpeedMps = cruiseSpeed;
        if (!(maneuverSpeedMps > 0.0f))
        {
            maneuverSpeedMps = (std::max)(entrySpeed, exitSpeed);
        }
        if (!(maneuverSpeedMps > 0.0f))
        {
            return Fail("Smooth turn speed is invalid");
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * profile.totalDistance));
        MazeMap::SmoothTurnYawRateControllerState yawRateController{};
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, profile.totalDistance - traveledM);
            if (remainingM <= Config::kDistanceToleranceM)
            {
                break;
            }

            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, maneuverSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=smooth_turn,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    maneuverSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=smooth_turn,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    maneuverSpeedMps);
            }

            float yawOffsetRad = 0.0f;
            float nominalOmegaRadps = 0.0f;
            if (!MazeMap::TryComputeSmoothTurnTarget(profile, traveledM, maneuverSpeedMps, yawOffsetRad, nominalOmegaRadps))
            {
                return Fail("Smooth turn target became invalid");
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
            _drive.CommandVelocity(maneuverSpeedMps, angularCommandRadps, dtSeconds);
        }

        return true;
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

    static bool TryGetSmoothTurnExecutionProfileMeters(MazeMap::ManeuverCode code, MazeMap::SmoothTurnExecutionProfile& profile)
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

    static float ManeuverDistanceMeters(MazeMap::ManeuverCode code)
    {
        MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
        if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
        {
            return smoothTurnProfile.totalDistance;
        }
        return 0.5f * Config::kCellSizeM * static_cast<float>(MazeMap::ManeuverSet::GetSet().DistanceTravelled(code));
    }

    static float ManeuverSpeedLimit(MazeMap::ManeuverCode code, const MotionLimits& limits, const MazeMap::Vehicle& vehicle)
    {
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (IsStraightCode(code))
        {
            return limits.maxSpeedMps;
        }
        return (std::min)(limits.maxSpeedMps, MazeMap::ManeuverSet::GetSet()[code].GetVMax(vehicle));
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code, const MotionLimits& limits) const
    {
        return ManeuverSpeedLimit(code, limits, _speedVehicle);
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code) const
    {
        return ManeuverSpeedLimit(code, FinalLimits());
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, const MazeMap::Vehicle& vehicle, float initialEntrySpeed, float finalExitSpeed)
    {
        if (queue.empty())
        {
            return;
        }

        float boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits, vehicle);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(boundarySpeed, speedLimit);
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)((std::min)(entry.GetEntrySpeed(), boundarySpeed), speedLimit);
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }

        float requiredExitSpeed = (std::max)(finalExitSpeed, 0.0f);
        for (int i = static_cast<int>(queue.size()) - 1; i >= 0; --i)
        {
            MazeMap::ManeuverInstance& entry = queue[static_cast<uint16_t>(i)];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits, vehicle);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(requiredExitSpeed, speedLimit));
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(exitSpeed, distanceM, limits.decelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                requiredExitSpeed = entrySpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(requiredExitSpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                requiredExitSpeed = maneuverSpeed;
            }
        }

        boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits, vehicle);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, float initialEntrySpeed, float finalExitSpeed)
    {
        ApplyAsymmetricQueueLimits(queue, FinalLimits(), _speedVehicle, initialEntrySpeed, finalExitSpeed);
    }
};
namespace MazeMapApp::Internal
{
    IMissionModeHost& GetMissionModeHost()
    {
        static MissionController mission(GetSharedRobotRuntime());
        return mission;
    }
}
