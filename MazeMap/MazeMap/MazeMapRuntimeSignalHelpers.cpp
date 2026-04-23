#include "pch.h"
#include "MazeMapRuntimeSignalHelpers.h"

#include "SensorSnapshot.h"

#include <algorithm>
#include <cmath>

namespace MazeMap::App::Internal::Runtime
{
    namespace
    {
        struct MapQualifiedSideWallReference final
        {
            bool useWall = false;
            MazeMap::CellCoordinates cell = MazeMap::CellCoordinates(0U, 0U);
            MazeMap::Direction wallDirection = MazeMap::None;
        };

        bool TryResolveMapQualifiedSideWallReference(
            const MazeMap::Maze& maze,
            const PoseEstimate& pose,
            const MazeMap::WallSensor& sensor,
            const bool distanceValidForControl,
            MapQualifiedSideWallReference& reference)
        {
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

            const Eigen::Vector2f worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
            const float sensorXM = pose.xMeters + worldOffset.x();
            const float sensorYM = pose.yMeters + worldOffset.y();
            const Eigen::Vector2f sensorFacing = SensorWorldFacing(pose, sensor);

            int cellX = -1;
            int cellY = -1;
            MazeMap::Direction wallDirection = MazeMap::None;
            if (std::fabs(sensorFacing.x()) >= std::fabs(sensorFacing.y()))
            {
                if (!std::isfinite(sensorXM) || !std::isfinite(alongWallCoordinateM))
                {
                    return false;
                }

                cellX = static_cast<int>(std::floor(sensorXM / Config::kCellSizeM));
                cellY = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
                wallDirection = (sensorFacing.x() >= 0.0f) ? MazeMap::Right : MazeMap::Left;
            }
            else
            {
                if (!std::isfinite(sensorYM) || !std::isfinite(alongWallCoordinateM))
                {
                    return false;
                }

                cellX = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
                cellY = static_cast<int>(std::floor(sensorYM / Config::kCellSizeM));
                wallDirection = (sensorFacing.y() >= 0.0f) ? MazeMap::Up : MazeMap::Down;
            }

            if ((cellX < 0) ||
                (cellY < 0) ||
                (cellX >= static_cast<int>(maze.GetXSize())) ||
                (cellY >= static_cast<int>(maze.GetYSize())))
            {
                return false;
            }

            const MazeMap::Cell& observedCell = maze.Index(cellX, cellY);
            if (observedCell.GetWall(wallDirection) != MazeMap::Wall)
            {
                return false;
            }

            reference.useWall = true;
            reference.cell = MazeMap::CellCoordinates(static_cast<std::uint8_t>(cellX), static_cast<std::uint8_t>(cellY));
            reference.wallDirection = wallDirection;
            return true;
        }
    }

    float ComputeSignalRiseAboveBaseline(float measuredDifferentialLight, float signalBaseline) noexcept
    {
        if (!std::isfinite(measuredDifferentialLight) ||
            !std::isfinite(signalBaseline))
        {
            return 0.0f;
        }

        return (measuredDifferentialLight > signalBaseline) ?
            (measuredDifferentialLight - signalBaseline) :
            0.0f;
    }

    bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized) noexcept
    {
        filteredSignal = measuredDifferentialLight;
        initialized = true;
        currentState = MazeMap::HysteresisSignalHigh(
            currentState,
            measuredDifferentialLight,
            onMeasuredThreshold,
            offMeasuredThreshold);
        return currentState;
    }

    float ComputeCorridorError(
        float leftDistanceM,
        float rightDistanceM,
        bool leftDistanceValidForControl,
        bool rightDistanceValidForControl,
        float expectedSideWallDistanceM) noexcept
    {
        if (leftDistanceValidForControl && rightDistanceValidForControl)
        {
            return 0.5f * (leftDistanceM - rightDistanceM);
        }
        if (leftDistanceValidForControl)
        {
            return leftDistanceM - expectedSideWallDistanceM;
        }
        if (rightDistanceValidForControl)
        {
            return expectedSideWallDistanceM - rightDistanceM;
        }
        return 0.0f;
    }

    bool TryComputeWallGroundedCorridorCoordinateM(
        const MazeMap::Maze& maze,
        const MazeMap::Vehicle& vehicle,
        const MazeMap::DirectionalLocation& currentLocation,
        const PoseEstimate& pose,
        const SensorSnapshot& snapshot,
        float& coordinateM,
        bool& correctsXAxis)
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        const MazeMap::Direction currentDirection = currentLocation.GetDirection();
        switch (currentDirection)
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
        MapQualifiedSideWallReference leftReference{};
        MapQualifiedSideWallReference rightReference{};

        if (TryResolveMapQualifiedSideWallReference(
                maze,
                pose,
                vehicle.SideLeft,
                snapshot.leftDistanceValidForControl,
                leftReference))
        {
            haveLeftCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                vehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                leftReference.cell,
                leftReference.wallDirection,
                leftCoordinateM);
        }

        if (TryResolveMapQualifiedSideWallReference(
                maze,
                pose,
                vehicle.SideRight,
                snapshot.rightDistanceValidForControl,
                rightReference))
        {
            haveRightCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                vehicle.SideRight,
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

    bool TryComputeWallGroundedCorridorErrorM(
        const MazeMap::Maze& maze,
        const MazeMap::Vehicle& vehicle,
        const MazeMap::DirectionalLocation& currentLocation,
        const PoseEstimate& pose,
        const SensorSnapshot& snapshot,
        float& corridorErrorM)
    {
        corridorErrorM = 0.0f;

        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(
                maze,
                vehicle,
                currentLocation,
                pose,
                snapshot,
                corridorCoordinateM,
                correctsXAxis))
        {
            return false;
        }

        float centerXM = 0.0f;
        float centerYM = 0.0f;
        if (!TryGetCellCenterMeters(
                static_cast<MazeMap::CellCoordinates>(currentLocation.GetLocation()),
                centerXM,
                centerYM))
        {
            return false;
        }

        const float errorXM = correctsXAxis ? (corridorCoordinateM - centerXM) : 0.0f;
        const float errorYM = correctsXAxis ? 0.0f : (corridorCoordinateM - centerYM);
        const Eigen::Vector2f heading = DirectionToUnitVector(currentLocation.GetDirection());
        corridorErrorM = (heading.y() * errorXM) - (heading.x() * errorYM);
        return std::isfinite(corridorErrorM);
    }

    float ComputeWallCenterPdOmegaRadps(
        float corridorErrorM,
        float forwardSpeedMps,
        float dtSeconds,
        float& previousCorridorErrorM,
        float& filteredCorridorErrorRateMps,
        bool& previousCorridorErrorValid)
    {
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
}

