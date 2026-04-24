#pragma once

#include "Defines.h"
#include "WallDetectionThresholds.h"

struct SensorSnapshot;

namespace MazeMap
{
    class Maze;
    class Vehicle;
    class VehicleState;
    class DirectionalLocation;
}

namespace MazeMap::App::Internal::Runtime
{
    // Computes the usable reflected-light rise above a calibrated open-space baseline.
    EXPORT float ComputeSignalRiseAboveBaseline(float measuredDifferentialLight, float signalBaseline) noexcept;

    // Applies the shared hysteresis-based wall-signal latch used by both runtime sensor pipelines.
    EXPORT bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized) noexcept;

    // Computes lateral corridor error from whichever side-wall observations are still valid for control.
    EXPORT float ComputeCorridorError(
        float leftDistanceM,
        float rightDistanceM,
        bool leftDistanceValidForControl,
        bool rightDistanceValidForControl,
        float expectedSideWallDistanceM) noexcept;

    // Resolves the map-qualified corridor coordinate from valid side-wall observations.
    EXPORT bool TryComputeWallGroundedCorridorCoordinateM(
        const MazeMap::Maze& maze,
        const MazeMap::Vehicle& vehicle,
        const MazeMap::DirectionalLocation& currentLocation,
        const MazeMap::VehicleState& state,
        const SensorSnapshot& snapshot,
        float& coordinateM,
        bool& correctsXAxis);

    // Computes signed corridor error in the robot heading frame using map-qualified side walls.
    EXPORT bool TryComputeWallGroundedCorridorErrorM(
        const MazeMap::Maze& maze,
        const MazeMap::Vehicle& vehicle,
        const MazeMap::DirectionalLocation& currentLocation,
        const MazeMap::VehicleState& state,
        const SensorSnapshot& snapshot,
        float& corridorErrorM);

    // Shared PD wall-centering controller used by both search motion and maneuver execution.
    EXPORT float ComputeWallCenterPdOmegaRadps(
        float corridorErrorM,
        float forwardSpeedMps,
        float dtSeconds,
        float& previousCorridorErrorM,
        float& filteredCorridorErrorRateMps,
        bool& previousCorridorErrorValid);
}

