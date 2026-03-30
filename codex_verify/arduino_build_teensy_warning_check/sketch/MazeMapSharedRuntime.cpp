#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapSharedRuntime.cpp"
#include "pch.h"
#include "MazeMapSharedRuntime.h"
#include "MazeMapApplicationPrivate.h"

namespace MazeMapApp::Internal
{
    class SharedRobotRuntime::Implementation final
    {
    public:
        Implementation()
            : speedVehicle()
            , searchVehicle()
            , maze()
            , searchPathFinder(maze, speedVehicle)
            , speedPathFinder(maze, speedVehicle)
            , wallBeliefMap()
            , drive()
            , missionSensors(speedVehicle, gWallDistanceCalibration)
            , telemetrySensors(speedVehicle, gWallDistanceCalibration)
            , diagnosticSensors(speedVehicle, gWallDistanceCalibration)
        {
            // Search-mode queue timing stays intentionally conservative even though the shared
            // physical vehicle model is reused everywhere else.
            searchVehicle.SetMaxSpeed(Config::kSearchMaxSpeedMps);
            searchVehicle.SetMaxForwardAcceleration(Config::kSearchAccelMps2);
            searchVehicle.SetMaxLateralAcceleration(Config::kSearchMaxLateralAccelerationMps2);
        }

        MazeMap::Vehicle speedVehicle;
        MazeMap::Vehicle searchVehicle;
        MazeMap::Maze maze;
        MazeMap::FloodFillPathFinder searchPathFinder;
        MazeMap::ManeuverPathFinder speedPathFinder;
        MazeMap::WallBeliefMap wallBeliefMap;
        DriveBase drive;
        SensorSuite missionSensors;
        DiagnosticSensorSuite telemetrySensors;
        DiagnosticSensorSuite diagnosticSensors;
    };

    SharedRobotRuntime::SharedRobotRuntime()
        : _impl(std::make_unique<Implementation>())
    {
    }

    SharedRobotRuntime::~SharedRobotRuntime() = default;

    MazeMap::Vehicle& SharedRobotRuntime::SpeedVehicle() noexcept
    {
        return _impl->speedVehicle;
    }

    const MazeMap::Vehicle& SharedRobotRuntime::SpeedVehicle() const noexcept
    {
        return _impl->speedVehicle;
    }

    MazeMap::Vehicle& SharedRobotRuntime::SearchVehicle() noexcept
    {
        return _impl->searchVehicle;
    }

    const MazeMap::Vehicle& SharedRobotRuntime::SearchVehicle() const noexcept
    {
        return _impl->searchVehicle;
    }

    MazeMap::Maze& SharedRobotRuntime::Maze() noexcept
    {
        return _impl->maze;
    }

    const MazeMap::Maze& SharedRobotRuntime::Maze() const noexcept
    {
        return _impl->maze;
    }

    MazeMap::FloodFillPathFinder& SharedRobotRuntime::SearchPathFinder() noexcept
    {
        return _impl->searchPathFinder;
    }

    MazeMap::ManeuverPathFinder& SharedRobotRuntime::SpeedPathFinder() noexcept
    {
        return _impl->speedPathFinder;
    }

    MazeMap::WallBeliefMap& SharedRobotRuntime::WallBeliefMap() noexcept
    {
        return _impl->wallBeliefMap;
    }

    DriveBase& SharedRobotRuntime::Drive() noexcept
    {
        return _impl->drive;
    }

    SensorSuite& SharedRobotRuntime::MissionSensors() noexcept
    {
        return _impl->missionSensors;
    }

    DiagnosticSensorSuite& SharedRobotRuntime::TelemetrySensors() noexcept
    {
        return _impl->telemetrySensors;
    }

    DiagnosticSensorSuite& SharedRobotRuntime::DiagnosticSensors() noexcept
    {
        return _impl->diagnosticSensors;
    }

    SharedRobotRuntime& GetSharedRobotRuntime()
    {
        static SharedRobotRuntime runtime;
        return runtime;
    }
}
