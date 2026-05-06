#include "pch.h"
#include "WallGeometryModel.h"

#include "PlantModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr std::uint8_t kWallPredictionRadiusCells = 2U;

    Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yaw, s, c);
        return Eigen::Vector2f(s, c);
    }

    Eigen::Vector2f RightUnitFromHeading(const Eigen::Vector2f& heading) noexcept
    {
        return Eigen::Vector2f(heading.y(), -heading.x());
    }

    Eigen::Vector2f RotateBodyVectorToWorld(
        const Eigen::Vector2f& vectorBody,
        const Eigen::Vector2f& heading) noexcept
    {
        const Eigen::Vector2f right = RightUnitFromHeading(heading);
        return Eigen::Vector2f(
            (right.x() * vectorBody.x()) + (heading.x() * vectorBody.y()),
            (right.y() * vectorBody.x()) + (heading.y() * vectorBody.y()));
    }

}

namespace MazeMap
{
    Eigen::Vector2f WallGeometryModel::sensorOriginWorld(
        const VehicleState::StateVector& state,
        const SensorMount& sensorMount) const noexcept
    {
        return sensorOriginWorld(buildStateFrame(state), sensorMount);
    }

    Eigen::Vector2f WallGeometryModel::sensorOriginWorld(
        const GeometryStateFrame& frame,
        const SensorMount& sensorMount) const noexcept
    {
        return frame.positionWorldM + RotateBodyVectorToWorld(sensorMount.positionBodyM(), frame.heading);
    }

    Eigen::Vector2f WallGeometryModel::sensorDirectionWorld(
        const VehicleState::StateVector& state,
        const SensorMount& sensorMount) const noexcept
    {
        return sensorDirectionWorld(buildStateFrame(state), sensorMount);
    }

    Eigen::Vector2f WallGeometryModel::sensorDirectionWorld(
        const GeometryStateFrame& frame,
        const SensorMount& sensorMount) const noexcept
    {
        return RotateBodyVectorToWorld(sensorMount.SensorForwardBody(), frame.heading);
    }

    WallGeometryModel::GeometryStateFrame WallGeometryModel::buildStateFrame(
        const VehicleState::StateVector& state) const noexcept
    {
        GeometryStateFrame frame{};
        frame.positionWorldM = Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy));
        frame.heading = HeadingUnitFromYaw(state(VehicleState::kPsi));
        frame.centerCell = WorldToCell(frame.positionWorldM.x(), frame.positionWorldM.y());
        return frame;
    }

    CellCoordinates WallGeometryModel::WorldToCell(float xMeters, float yMeters) noexcept
    {
        const float safeCellSize = Config::kCellSizeM;
        int x = static_cast<int>(std::floor(xMeters / safeCellSize));
        int y = static_cast<int>(std::floor(yMeters / safeCellSize));
        x = (std::clamp)(x, 0, 15);
        y = (std::clamp)(y, 0, 15);
        return CellCoordinates(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
    }

    bool WallGeometryModel::intersectRayAabb(
        const Eigen::Vector2f& origin,
        const Eigen::Vector2f& direction,
        const Eigen::Vector2f& minCorner,
        const Eigen::Vector2f& maxCorner,
        float& rangeM) noexcept
    {
        rangeM = 0.0f;
        float tMin = 0.0f;
        float tMax = (std::numeric_limits<float>::max)();

        for (int axis = 0; axis < 2; ++axis)
        {
            const float originComponent = origin[axis];
            const float directionComponent = direction[axis];
            const float minComponent = minCorner[axis];
            const float maxComponent = maxCorner[axis];

            if (std::fabs(directionComponent) <= 1.0e-6f)
            {
                if ((originComponent < minComponent) || (originComponent > maxComponent))
                {
                    return false;
                }
                continue;
            }

            float t1 = (minComponent - originComponent) / directionComponent;
            float t2 = (maxComponent - originComponent) / directionComponent;
            if (t1 > t2)
            {
                const float swap = t1;
                t1 = t2;
                t2 = swap;
            }
            tMin = (std::max)(tMin, t1);
            tMax = (std::min)(tMax, t2);
            if (tMax < tMin)
            {
                return false;
            }
        }

        if (tMax <= 0.0f)
        {
            return false;
        }

        rangeM = (tMin > 0.0f) ? tMin : tMax;
        return std::isfinite(rangeM) && (rangeM > 0.0f);
    }

    void WallGeometryModel::testUniqueWall(
        const Eigen::Vector2f& rayOrigin,
        const Eigen::Vector2f& rayDirection,
        const CellCoordinates& cell,
        Direction direction,
        WallState state,
        GeometryPrediction& best) noexcept
    {
        if (state != WallState::Wall)
        {
            return;
        }

        const float cellX = static_cast<float>(cell.GetX()) * Config::kCellSizeM;
        const float cellY = static_cast<float>(cell.GetY()) * Config::kCellSizeM;

        Eigen::Vector2f minCorner = Eigen::Vector2f::Zero();
        Eigen::Vector2f maxCorner = Eigen::Vector2f::Zero();
        switch (direction)
        {
        case Direction::Up:
            minCorner = Eigen::Vector2f(
                cellX,
                cellY + Config::kCellSizeM - (0.5f * Config::kMazeWallThicknessM));
            maxCorner = Eigen::Vector2f(
                cellX + Config::kCellSizeM,
                cellY + Config::kCellSizeM + (0.5f * Config::kMazeWallThicknessM));
            break;
        case Direction::Down:
            minCorner = Eigen::Vector2f(cellX, cellY - (0.5f * Config::kMazeWallThicknessM));
            maxCorner = Eigen::Vector2f(
                cellX + Config::kCellSizeM,
                cellY + (0.5f * Config::kMazeWallThicknessM));
            break;
        case Direction::Left:
            minCorner = Eigen::Vector2f(cellX - (0.5f * Config::kMazeWallThicknessM), cellY);
            maxCorner = Eigen::Vector2f(
                cellX + (0.5f * Config::kMazeWallThicknessM),
                cellY + Config::kCellSizeM);
            break;
        case Direction::Right:
        default:
            minCorner = Eigen::Vector2f(
                cellX + Config::kCellSizeM - (0.5f * Config::kMazeWallThicknessM),
                cellY);
            maxCorner = Eigen::Vector2f(
                cellX + Config::kCellSizeM + (0.5f * Config::kMazeWallThicknessM),
                cellY + Config::kCellSizeM);
            break;
        }

        float rangeM = 0.0f;
        if (!intersectRayAabb(rayOrigin, rayDirection, minCorner, maxCorner, rangeM))
        {
            return;
        }
        if (!(rangeM > 0.0f) || !(rangeM < best.rangeM))
        {
            return;
        }

        best.hit = true;
        best.type = GeometryHitType::WallFace;
        best.rangeM = rangeM;
        best.pointWorldM = rayOrigin + (rayDirection * rangeM);
        best.cell = cell;
        best.edge = direction;
    }

    GeometryPrediction WallGeometryModel::predictRay(
        const VehicleState::StateVector& state,
        const SensorMount& sensorMount,
        const Maze& maze) const noexcept
    {
        return predictRay(state, sensorMount, maze, PlantParams::Default().noHitRangeM);
    }

    GeometryPrediction WallGeometryModel::predictRay(
        const GeometryStateFrame& frame,
        const SensorMount& sensorMount,
        const Maze& maze) const noexcept
    {
        return predictRay(frame, sensorMount, maze, PlantParams::Default().noHitRangeM);
    }

    GeometryPrediction WallGeometryModel::predictRay(
        const VehicleState::StateVector& state,
        const SensorMount& sensorMount,
        const Maze& maze,
        float maxRangeM) const noexcept
    {
        return predictRay(buildStateFrame(state), sensorMount, maze, maxRangeM);
    }

    GeometryPrediction WallGeometryModel::predictRay(
        const GeometryStateFrame& frame,
        const SensorMount& sensorMount,
        const Maze& maze,
        float maxRangeM) const noexcept
    {
        GeometryPrediction best{};
        best.rangeM = (maxRangeM > 0.0f) ? maxRangeM : PlantParams::Default().noHitRangeM;

        const Eigen::Vector2f rayOrigin = sensorOriginWorld(frame, sensorMount);
        const Eigen::Vector2f rayDirection = sensorDirectionWorld(frame, sensorMount);
        const CellCoordinates centerCell = frame.centerCell;

        const int minX = (std::max)(0, static_cast<int>(centerCell.GetX()) - static_cast<int>(kWallPredictionRadiusCells));
        const int maxX = (std::min)(15, static_cast<int>(centerCell.GetX()) + static_cast<int>(kWallPredictionRadiusCells));
        const int minY = (std::max)(0, static_cast<int>(centerCell.GetY()) - static_cast<int>(kWallPredictionRadiusCells));
        const int maxY = (std::min)(15, static_cast<int>(centerCell.GetY()) + static_cast<int>(kWallPredictionRadiusCells));

        for (int x = minX; x <= maxX; ++x)
        {
            for (int y = minY; y <= maxY; ++y)
            {
                const CellCoordinates cell(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
                const Cell& mazeCell = maze[cell];
                testUniqueWall(rayOrigin, rayDirection, cell, Direction::Up, mazeCell.GetUp(), best);
                testUniqueWall(rayOrigin, rayDirection, cell, Direction::Right, mazeCell.GetRight(), best);
                if (y == 0)
                {
                    testUniqueWall(rayOrigin, rayDirection, cell, Direction::Down, mazeCell.GetDown(), best);
                }
                if (x == 0)
                {
                    testUniqueWall(rayOrigin, rayDirection, cell, Direction::Left, mazeCell.GetLeft(), best);
                }
            }
        }

        for (int gridX = minX; gridX <= maxX + 1; ++gridX)
        {
            for (int gridY = minY; gridY <= maxY + 1; ++gridY)
            {
                const float gridXM = static_cast<float>(gridX) * Config::kCellSizeM;
                const float gridYM = static_cast<float>(gridY) * Config::kCellSizeM;
                const float postHalfWidthM = 0.5f * Config::kMazeWallThicknessM;
                const Eigen::Vector2f minCorner(gridXM - postHalfWidthM, gridYM - postHalfWidthM);
                const Eigen::Vector2f maxCorner(gridXM + postHalfWidthM, gridYM + postHalfWidthM);
                float rangeM = 0.0f;
                if (!intersectRayAabb(rayOrigin, rayDirection, minCorner, maxCorner, rangeM))
                {
                    continue;
                }
                if (!(rangeM > 0.0f) || !(rangeM < best.rangeM))
                {
                    continue;
                }

                best.hit = true;
                best.type = GeometryHitType::Post;
                best.rangeM = rangeM;
                best.pointWorldM = rayOrigin + (rayDirection * rangeM);
                best.postGridX = static_cast<uint8_t>(gridX);
                best.postGridY = static_cast<uint8_t>(gridY);
                best.edge = Direction::None;
            }
        }

        return best;
    }
}
