#pragma once
// Declares the shared runtime object that wires together vehicles, planners, sensors, and drive services for the app.

#include "Defines.h"

#include <memory>

class DriveBase;
class SensorSuite;
class DiagnosticSensorSuite;

namespace MazeMap
{
    class Vehicle;
    class Maze;
    class FloodFillPathFinder;
    class ManeuverPathFinder;
    class WallBeliefMap;
}

namespace MazeMap::App::Internal
{
    // Owns the heavyweight runtime infrastructure that should be shared across all startup modes.
    // Mode implementations stay small by borrowing references from this hub instead of allocating
    // duplicate vehicle, estimator, pathfinding, and sensor state.
    class EXPORT SharedRobotRuntime final
    {
    public:
        SharedRobotRuntime();
        ~SharedRobotRuntime();

        SharedRobotRuntime(const SharedRobotRuntime&) = delete;
        SharedRobotRuntime& operator=(const SharedRobotRuntime&) = delete;
        SharedRobotRuntime(SharedRobotRuntime&&) = delete;
        SharedRobotRuntime& operator=(SharedRobotRuntime&&) = delete;

        MazeMap::Vehicle& SpeedVehicle() noexcept;
        const MazeMap::Vehicle& SpeedVehicle() const noexcept;

        MazeMap::Vehicle& SearchVehicle() noexcept;
        const MazeMap::Vehicle& SearchVehicle() const noexcept;

        MazeMap::Maze& Maze() noexcept;
        const MazeMap::Maze& Maze() const noexcept;

        MazeMap::FloodFillPathFinder& SearchPathFinder() noexcept;
        MazeMap::ManeuverPathFinder& SpeedPathFinder() noexcept;
        MazeMap::WallBeliefMap& WallBeliefMap() noexcept;

        DriveBase& Drive() noexcept;
        SensorSuite& MissionSensors() noexcept;
        DiagnosticSensorSuite& TelemetrySensors() noexcept;
        DiagnosticSensorSuite& DiagnosticSensors() noexcept;

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    EXPORT SharedRobotRuntime& GetSharedRobotRuntime();
}

