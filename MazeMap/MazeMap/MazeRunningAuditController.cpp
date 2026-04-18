#include "pch.h"
#include "MazeRunningAuditController.h"

#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "AuxMeasurementModeSupport.h"
#include "ManeuverExecutor.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallTouchRoutine.h"

using MazeMap::App::Internal::SharedRobotRuntime;
using namespace MazeMap::App::Internal::AuxMeasurementModeSupport;

namespace
{
    constexpr MazeMap::CommandPD kMazeRunningDriveBaseTrackingCommandPd =
        MazeMap::CommandPD::StateWheelOmegaPD |
        MazeMap::CommandPD::IMUYaw;
}

class MazeMap::App::Internal::MazeRunningAuditController::Implementation final
{
public:
    explicit Implementation(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _speedVehicle(runtime.SpeedVehicle())
        , _mappingVehicle(runtime.SearchVehicle())
        , _maze(runtime.Maze())
        , _wallBeliefMap(runtime.WallBeliefMap())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _wallTouchRoutine(_drive)
        , _currentCell(0, 0)
        , _currentDirection(MazeMap::Up)
        , _currentDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _faulted(false)
        , _telemetryLoggingEnabled(false)
        , _textLoggingEnabled(false)
        , _mazeSnapshotWritten(false)
        , _lastWallTouchStandoffEstimateM(0.0f)
        , _hasWallTouchStandoffEstimate(false)
        , _activeModeFaultSource("utility_audit")
        , _telemetryPhaseId(0UL)
        , _telemetrySampleCount(0UL)
    {
        _telemetryLogFileName[0] = '\0';
    }

    Implementation(const Implementation&) = delete;
    Implementation& operator=(const Implementation&) = delete;
    Implementation(Implementation&&) = delete;
    Implementation& operator=(Implementation&&) = delete;

public:

    bool BeginCorridorRepeatabilityRoutine()
    {
        ResetForMode(false, "corridor_repeatability");
        if (!Initialize("mode:corridor_repeatability", "Corridor repeatability setup"))
        {
            return false;
        }

        PrimeKnownStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
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
        if (!LogCorridorRepeatabilityMetadata(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this))
        {
            return false;
        }
        AppendStartupTrace("corridor_repeatability:metadata_written");
        return true;
    }

    void RunCorridorRepeatabilityRoutine()
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
            (void)EmitControllerLine("Corridor repeatability sweep complete");
        }
        CloseTextLog();
    }

    bool BeginPositionAccuracyAuditRoutine()
    {
        ResetForMode(false, "position_accuracy_audit");
        if (!Initialize("mode:position_accuracy_audit", "Position accuracy audit setup"))
        {
            return false;
        }

        PrimeKnownStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
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
        if (!LogPositionAccuracyAuditMetadata(
                positionAuditGeometry,
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this))
        {
            return false;
        }
        AppendStartupTrace("position_accuracy_audit:metadata_written");
        return true;
    }

    void RunPositionAccuracyAuditRoutine()
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
            (void)EmitControllerLine("Position accuracy audit complete");
        }
        CloseTextLog();
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    static constexpr const char* kMazeRunningControllerTextLogSource = "maze_running_audit";
    static constexpr const char* kMazeRunningTraceTextLogSource = "maze_running_trace";

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _speedVehicle;
    MazeMap::Vehicle& _mappingVehicle;
    MazeMap::Maze& _maze;
    MazeMap::WallBeliefMap& _wallBeliefMap;
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    MazeMap::App::Internal::WallTouchRoutine _wallTouchRoutine;
    MazeMap::CellCoordinates _currentCell;
    MazeMap::Direction _currentDirection;
    MazeMap::DirectionalLocation _currentDirectionalLocation;
    bool _faulted;
    bool _telemetryLoggingEnabled;
    bool _textLoggingEnabled;
    bool _mazeSnapshotWritten;
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

    struct InterRunServicePauseLoopState final
    {
    };

    struct QueuedManeuverLoopState final
    {
        MazeMap::ManeuverQueue* queue{};
        MotionLimits limits{};
        bool snapToExpectedLocation{};
        const char* completionHoldPhaseName{};
    };

    struct StartupStationaryHoldLoopState final
    {
        unsigned long stationaryStartMs{};
        unsigned long lastResetTraceMs{};
        bool stationaryWindowActive{};
        DriveTelemetry stationaryStartTelemetry{};
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
    };

    struct SharedRoutineLoopState final
    {
        enum class Routine : std::uint8_t
        {
            Hold,
            BrakedSettle,
            ReverseStraight,
            Straight,
            Turn,
            Arc,
            SmoothTurn
        };

        Routine routine{ Routine::Hold };
        const char* failureReason{};
        ManeuverExecutor::Hooks hooks{};
        std::uint16_t durationMs{};
        bool stationary{};
        const char* timeoutMessage{};
        std::uint16_t stationaryHoldMs{};
        std::uint16_t timeoutMs{};
        float distanceM{};
        MotionLimits limits{};
        bool useWallCentering{};
        float entrySpeed{};
        float cruiseSpeed{};
        float exitSpeed{};
        MazeMap::DirectionalLocation* currentLocation{};
        float angleRad{};
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker{};
        const MazeMap::ManeuverInstance* maneuver{};
        const Eigen::Vector2f* targetHeadingOverride{};
        const Eigen::Vector2f* targetPositionOverride{};
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

    void PrimeKnownStartCell()
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

    static bool WriteAuxMeasurementEventCallback(
        void* context,
        const char* type,
        const char* message) noexcept
    {
        auto* const self = static_cast<Implementation*>(context);
        return (self != nullptr) && self->WriteTelemetryEvent(type, message);
    }

    static bool FailAuxMeasurementCallback(void* context, const char* message) noexcept
    {
        auto* const self = static_cast<Implementation*>(context);
        return (self != nullptr) && self->Fail(message);
    }

    void ResetForMode(bool enableTextLogging, const char* activeModeFaultSource)
    {
        _telemetryLoggingEnabled = false;
        _textLoggingEnabled = enableTextLogging;
        _mazeSnapshotWritten = false;
        _faulted = false;
        _activeModeFaultSource =
            (activeModeFaultSource != nullptr && activeModeFaultSource[0] != '\0') ? activeModeFaultSource : "utility_audit";
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

    bool OpenTextLog()
    {
        if (!_textLoggingEnabled)
        {
            return true;
        }

        return _runtime.EnsureTextLogOpen();
    }

    void FlushTextLog()
    {
        if (_textLoggingEnabled)
        {
            _runtime.FlushTextLog();
        }
    }

    void CloseTextLog()
    {
        _runtime.FlushTextLog();
    }

    bool WriteTextLineIfEnabled(const char* message)
    {
        if (!_textLoggingEnabled)
        {
            return true;
        }

        if (message == nullptr)
        {
            return false;
        }

        return _runtime.WriteTextLogEntry(
            kMazeRunningTraceTextLogSource,
            micros(),
            "trace",
            message);
    }

    void DisableTextLogging(const char* traceLabel)
    {
        if (!_textLoggingEnabled)
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
                "Text logging disabled: %s",
                traceLabel);
            if (written > 0 && written < static_cast<int>(sizeof(message)))
            {
                (void)_runtime.WriteTextLogEntry(
                    kMazeRunningControllerTextLogSource,
                    micros(),
                    "status",
                    message);
            }
        }

        CloseTextLog();
        _textLoggingEnabled = false;
    }

    bool WriteTraceLineBestEffort(const char* message, const char* traceLabel)
    {
        if (!_textLoggingEnabled)
        {
            return true;
        }

        if (WriteTextLineIfEnabled(message))
        {
            return true;
        }

        DisableTextLogging(traceLabel);
        return true;
    }
    bool EmitControllerLine(const char* message)
    {
        if (message == nullptr)
        {
            return false;
        }

        if (_textLoggingEnabled)
        {
            if (_runtime.WriteTextLogEntry(
                    kMazeRunningControllerTextLogSource,
                    micros(),
                    "status",
                    message))
            {
                return true;
            }

            DisableTextLogging("maze_running_text_logging:controller_write_failed");
        }

        return _runtime.WriteTextLogEntry(
            kMazeRunningControllerTextLogSource,
            micros(),
            "status",
            message);
    }

    bool EmitControllerFormatted(const char* format, ...)
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

        return EmitControllerLine(line);
    }

    bool EmitControllerLineOrFail(const char* message)
    {
        return EmitControllerLine(message);
    }

    bool EmitControllerFormattedOrFail(const char* format, ...)
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

        return EmitControllerLineOrFail(line);
    }

    void AppendTraceLine(const char* message)
    {
        if (message == nullptr)
        {
            return;
        }

        AppendStartupTrace(message);
        (void)WriteTraceLineBestEffort(message, "maze_running_text_logging:trace_write_failed");
    }

    void AppendTraceFormatted(const char* format, ...)
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

        AppendTraceLine(line);
    }

    bool WriteMazeSnapshot(const char* trigger)
    {
        if (!_textLoggingEnabled || _mazeSnapshotWritten)
        {
            return true;
        }

        const bool ok = MazeMap::ExportMazeSnapshot(_maze, "maze.txt");
        AppendStartupTrace(ok ? "maze_running_maze_snapshot:maze.txt" : "maze_running_maze_snapshot:write_failed");
        if (ok)
        {
            _mazeSnapshotWritten = true;
            (void)EmitControllerFormatted("Maze snapshot written to maze.txt after %s", (trigger != nullptr) ? trigger : "unknown");
        }
        else
        {
            (void)EmitControllerFormatted("Maze snapshot write failed after %s", (trigger != nullptr) ? trigger : "unknown");
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

    bool ReseatStartPoseWithPhasePrefix(const char* phasePrefix, uint16_t settleMs)
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

        PrimeKnownStartCell();
        snprintf(phaseName, sizeof(phaseName), "%s_settle", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        return HoldPosition(settleMs, phaseName);
    }

    bool ReseatCorridorRepeatabilityStartPose(uint8_t speedIndex, float centerOffsetFromTouchM)
    {
        (void)centerOffsetFromTouchM;
        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "corridor_%u_reseat", static_cast<unsigned>(speedIndex));
        return ReseatStartPoseWithPhasePrefix(
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

        if (!WriteCorridorRepeatabilityResult(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this,
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
        if (!WritePositionInPlaceTurnAuditResult(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this,
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

        if (!WritePositionStraightAuditResult(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this,
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

        if (!WritePositionInPlaceTurnAuditResult(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this,
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
        return ReseatStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs);
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

        if (!WritePositionSmoothTurnAuditResult(
                &Implementation::WriteAuxMeasurementEventCallback,
                &Implementation::FailAuxMeasurementCallback,
                this,
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
            if (!ReseatStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
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
            if (!ReseatStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
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
            if (!ReseatStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
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

    bool ExecuteWallTouchRoutine(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        const MazeMap::App::Internal::Runtime::WallTouchPoseResetTarget* poseResetTarget,
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

        MazeMap::App::Internal::Runtime::WallTouchExecutionResult result{};
        MazeMap::App::Internal::WallTouchRoutine::Hooks hooks{};
        hooks.context = this;
        hooks.onSample = &Implementation::ManeuverExecutorSampleHook;
        hooks.onTraceLine = [](void* context, const char* line) noexcept
        {
            auto* const self = static_cast<Implementation*>(context);
            if ((self != nullptr) && (line != nullptr))
            {
                AppendStartupTrace(line);
            }
        };
        hooks.onPoseReset = [](void* context) noexcept
        {
            auto* const self = static_cast<Implementation*>(context);
            if (self != nullptr)
            {
                self->AppendStartupCalibrationStateTrace("touch_pose_set");
            }
        };

        LoopController::ModeCallbacks initialCallbacks{};
        if (!_wallTouchRoutine.BeginSession(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                poseResetTarget,
                &result,
                PrepareLoopContinuation(this, &Implementation::WallTouchRoutineCompleteTick),
                initialCallbacks,
                hooks))
        {
            return Fail("Failed to prepare wall-touch routine");
        }
        if (!RunLoopSession(initialCallbacks))
        {
            _wallTouchRoutine.CancelActiveRoutine();
            return false;
        }

        outcome = result.outcome;
        traveledDistanceM = result.seatedTravelM;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = result.seatedYawErrorRad;
        }
        return true;
    }

    LoopController::ControlVector WallTouchRoutineCompleteTick(
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
        const MazeMap::App::Internal::Runtime::WallTouchPoseResetTarget poseResetTarget{
            xMeters,
            yMeters,
            DirectionToYawRad(facingDirection),
            true
        };
        if (!ExecuteWallTouchRoutine(
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
            if (!_runtime.ManeuverExecutorService().BeginHoldRoutine(
                    Config::kStartupWallCalibrationSettleMs,
                    true,
                    PrepareLoopContinuation(rawState, &Implementation::SharedRoutineCompleteTick),
                    services,
                    BuildManeuverExecutorHooks(false)))
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
        if (!EmitControllerLineOrFail("Startup wall calibration"))
        {
            return false;
        }
        gWallDistanceCalibration.Clear();
        _hasWallTouchStandoffEstimate = false;
        SeedStartupWallCalibrationPoseFromSouthWall();
        if (!WaitForStartupStationaryHold())
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
        if (!ReseatStartPoseWithPhasePrefix("startup_front_baseline", Config::kStartupWallCalibrationSettleMs))
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

    bool Initialize(const char* startupTraceLine, const char* banner)
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
                "mode:maze_running_audit");
        if (!OpenTextLog())
        {
            AppendStartupTrace("initialize:logging_txt_open_failed");
            DisableTextLogging("initialize:text_log_unavailable");
        }
        if (!EmitControllerLine(banner))
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
            services.Fault("Maze-running loop callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        const bool shouldLogSample =
            (self->_activeLoopTickFn == &Implementation::FrontCalibrationSweepLoopTick) ||
            (self->_activeLoopTickFn == &Implementation::StartupStationaryHoldLoopTick);
        if (shouldLogSample)
        {
            const bool stationary =
                self->_activeLoopTickFn == &Implementation::StartupStationaryHoldLoopTick;
            if (!Implementation::ManeuverExecutorSampleHook(self, stationary, state))
            {
                services.Fault("Failed to write maneuver test sample");
                return LoopController::ControlVector::Brake;
            }
        }

        return (self->*self->_activeLoopTickFn)(self->_activeLoopState, loopEndTimeUs, state, services);
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

        self->AppendTraceFormatted(
            "maze_running_maneuver:begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
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
        self->AppendTraceFormatted(
            "maze_running_maneuver:end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
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
        if (!EmitControllerFormattedOrFail(
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

        if (!_runtime.ManeuverExecutorService().BeginHoldRoutine(
                50U,
                true,
                PrepareLoopContinuation(rawState, &Implementation::SharedRoutineCompleteTick),
                services,
                BuildManeuverExecutorHooks(false)))
        {
            return FaultLoopPhase(services, "Failed to begin queued maneuver completion hold");
        }
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector SharedRoutineLaunchTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        auto& routine = *static_cast<SharedRoutineLoopState*>(rawState);
        const LoopController::ModeCallbacks continuation =
            PrepareLoopContinuation(rawState, &Implementation::SharedRoutineCompleteTick);

        bool begun = false;
        switch (routine.routine)
        {
        case SharedRoutineLoopState::Routine::Hold:
            begun = _runtime.ManeuverExecutorService().BeginHoldRoutine(
                routine.durationMs,
                routine.stationary,
                continuation,
                services,
                routine.hooks);
            break;

        case SharedRoutineLoopState::Routine::BrakedSettle:
            begun = _runtime.ManeuverExecutorService().BeginBrakedSettleRoutine(
                routine.timeoutMessage,
                routine.stationaryHoldMs,
                routine.timeoutMs,
                continuation,
                services,
                routine.hooks);
            break;

        case SharedRoutineLoopState::Routine::ReverseStraight:
            begun = _runtime.ManeuverExecutorService().BeginReverseStraightRoutine(
                routine.distanceM,
                routine.limits,
                continuation,
                services,
                routine.hooks,
                routine.targetHeadingOverride,
                routine.targetPositionOverride);
            break;

        case SharedRoutineLoopState::Routine::Straight:
            begun = _runtime.ManeuverExecutorService().BeginStraightRoutine(
                routine.distanceM,
                routine.entrySpeed,
                routine.cruiseSpeed,
                routine.exitSpeed,
                routine.limits,
                routine.useWallCentering,
                routine.currentLocation,
                continuation,
                services,
                routine.hooks,
                routine.targetHeadingOverride,
                routine.targetPositionOverride);
            break;

        case SharedRoutineLoopState::Routine::Turn:
            begun = _runtime.ManeuverExecutorService().BeginTurnRoutine(
                routine.angleRad,
                routine.limits,
                continuation,
                services,
                routine.hooks,
                routine.wallEdgeTracker);
            break;

        case SharedRoutineLoopState::Routine::Arc:
            begun = _runtime.ManeuverExecutorService().BeginArcRoutine(
                routine.distanceM,
                routine.angleRad,
                routine.entrySpeed,
                routine.exitSpeed,
                routine.cruiseSpeed,
                routine.limits,
                continuation,
                services,
                routine.hooks);
            break;

        case SharedRoutineLoopState::Routine::SmoothTurn:
            begun = (routine.maneuver != nullptr) &&
                _runtime.ManeuverExecutorService().BeginSmoothTurnRoutine(
                    *routine.maneuver,
                    routine.cruiseSpeed,
                    routine.limits,
                    continuation,
                    services,
                    routine.hooks);
            break;
        }

        if (!begun)
        {
            return FaultLoopPhase(
                services,
                (routine.failureReason != nullptr) ?
                    routine.failureReason :
                    "Failed to launch shared routine");
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector SharedRoutineCompleteTick(
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

    bool RunLoopSession(const LoopController::ModeCallbacks& initialCallbacks)
    {
        const bool began = _loopController.BeginSession(BuildLoopOptions(), initialCallbacks);
        if (!began)
        {
            _activeLoopState = nullptr;
            _activeLoopTickFn = nullptr;
            return Fail("Maze-running loop controller session could not start");
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

    bool RunLoopSession(void* initialState, const ActiveLoopTickFn initialTickFn)
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &Implementation::ActiveLoopThunk;
        callbacks.context = this;
        _activeLoopState = initialState;
        _activeLoopTickFn = initialTickFn;

        return RunLoopSession(callbacks);
    }

    bool RunSharedRoutine(SharedRoutineLoopState& routine)
    {
        return RunLoopSession(&routine, &Implementation::SharedRoutineLaunchTick);
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
        (void)EmitControllerFormatted("FAULT: %s", (message != nullptr) ? message : "unknown");
        if (_textLoggingEnabled && !_mazeSnapshotWritten)
        {
            (void)WriteMazeSnapshot("maze_running_fault");
        }
        if (_telemetryLoggingEnabled)
        {
            (void)WriteTelemetryEvent("fault", message);
            FlushTelemetryLog();
            CloseTelemetryLog();
            _telemetryLoggingEnabled = false;
        }
        FlushTextLog();
    }

    bool HoldPosition(uint16_t durationMs, const char* phaseName = nullptr)
    {
        if (phaseName != nullptr && !BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::Hold;
        routine.failureReason = "Failed to begin shared hold routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.durationMs = durationMs;
        routine.stationary = true;
        return RunSharedRoutine(routine);
    }

    bool HoldBrakedUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::BrakedSettle;
        routine.failureReason = "Failed to begin shared braked-settle routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.timeoutMessage = timeoutMessage;
        routine.stationaryHoldMs = stationaryHoldMs;
        routine.timeoutMs = timeoutMs;
        return RunSharedRoutine(routine);
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

    bool WaitForStartupStationaryHold()
    {
        if (!EmitControllerLineOrFail("Waiting for 2 s stationary start"))
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

        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::ReverseStraight;
        routine.failureReason = "Failed to begin shared reverse-straight routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.distanceM = distanceM;
        routine.limits = limits;
        routine.targetHeadingOverride = targetHeadingOverride;
        routine.targetPositionOverride = targetPositionOverride;
        return RunSharedRoutine(routine);
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

        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::Straight;
        routine.failureReason = "Failed to begin shared straight routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.distanceM = distanceM;
        routine.entrySpeed = entrySpeed;
        routine.cruiseSpeed = cruiseSpeed;
        routine.exitSpeed = exitSpeed;
        routine.limits = limits;
        routine.useWallCentering = useWallCentering;
        routine.currentLocation = useWallCentering ? &_currentDirectionalLocation : nullptr;
        routine.targetHeadingOverride = targetHeadingOverride;
        routine.targetPositionOverride = targetPositionOverride;
        return RunSharedRoutine(routine);
    }

    bool ExecuteTurnProfile(
        float angleRad,
        const MotionLimits& limits,
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr)
    {
        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::Turn;
        routine.failureReason = "Failed to begin shared turn routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.angleRad = angleRad;
        routine.limits = limits;
        routine.wallEdgeTracker = wallEdgeTracker;
        return RunSharedRoutine(routine);
    }

    bool ExecuteArcProfile(float distanceM, float angleRad, float entrySpeed, float exitSpeed, float cruiseSpeed, const MotionLimits& limits)
    {
        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::Arc;
        routine.failureReason = "Failed to begin shared arc routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.distanceM = distanceM;
        routine.angleRad = angleRad;
        routine.entrySpeed = entrySpeed;
        routine.exitSpeed = exitSpeed;
        routine.cruiseSpeed = cruiseSpeed;
        routine.limits = limits;
        return RunSharedRoutine(routine);
    }

    bool ExecuteSmoothTurnProfile(
        const MazeMap::ManeuverInstance& maneuver,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        SharedRoutineLoopState routine{};
        routine.routine = SharedRoutineLoopState::Routine::SmoothTurn;
        routine.failureReason = "Failed to begin shared smooth-turn routine";
        routine.hooks = BuildManeuverExecutorHooks(false);
        routine.maneuver = &maneuver;
        routine.cruiseSpeed = cruiseSpeed;
        routine.limits = limits;
        return RunSharedRoutine(routine);
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
    MazeRunningAuditController::MazeRunningAuditController(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    MazeRunningAuditController::~MazeRunningAuditController() = default;

    bool MazeRunningAuditController::BeginCorridorRepeatabilityRoutine()
    {
        return _impl->BeginCorridorRepeatabilityRoutine();
    }

    void MazeRunningAuditController::RunCorridorRepeatabilityRoutine()
    {
        _impl->RunCorridorRepeatabilityRoutine();
    }

    bool MazeRunningAuditController::BeginPositionAccuracyAuditRoutine()
    {
        return _impl->BeginPositionAccuracyAuditRoutine();
    }

    void MazeRunningAuditController::RunPositionAccuracyAuditRoutine()
    {
        _impl->RunPositionAccuracyAuditRoutine();
    }
}










