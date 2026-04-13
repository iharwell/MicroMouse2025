#pragma once
// Declares wall-geometry prediction helpers used by the UKF wall-measurement updates.

#include "Cell.h"
#include "CellCoordinates.h"
#include "Direction.h"
#include "Maze.h"
#include "VehicleState.h"

#include <cstdint>

namespace MazeMap
{
    // Classifies the map feature hit by a wall-sensor ray cast.
    enum class GeometryHitType : uint8_t
    {
        None = 0U,
        WallFace = 1U,
        Post = 2U
    };

    // Read-only map view used by UKF wall prediction.
    struct LocalMapView
    {
        const Maze* maze = nullptr;
        float cellSizeM = Maze::GetCellDimension();
        float wallThicknessM = WALL_THICKNESS;
        float postHalfWidthM = 0.5f * WALL_THICKNESS;
        float noHitRangeM = 0.30f;
        uint8_t radiusCells = 2U;
        bool freezeMapMutation = false;

        bool IsValid() const noexcept
        {
            return maze != nullptr;
        }
    };

    // One predicted wall-sensor ray-cast result.
    struct GeometryPrediction
    {
        bool hit = false;
        GeometryHitType type = GeometryHitType::None;
        float rangeM = 0.0f;
        Eigen::Vector2f pointWorldM = Eigen::Vector2f::Zero();
        CellCoordinates cell{};
        Direction edge = Direction::None;
        uint8_t postGridX = 0U;
        uint8_t postGridY = 0U;
    };

    // Projects wall-sensor rays into the local maze geometry around the estimated pose.
    class EXPORT WallGeometryModel
    {
    public:
        // Cached pose frame shared by a group of ray casts from the same sigma point.
        struct GeometryStateFrame
        {
            Eigen::Vector2f positionWorldM = Eigen::Vector2f::Zero();
            Eigen::Vector2f heading = Eigen::Vector2f(0.0f, 1.0f);
            CellCoordinates centerCell{};
        };

        GeometryPrediction predictRay(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics,
            const LocalMapView& map) const noexcept;

        GeometryPrediction predictRay(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics,
            const LocalMapView& map) const noexcept;

        Eigen::Vector2f sensorOriginWorld(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorOriginWorld(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorDirectionWorld(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorDirectionWorld(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        GeometryStateFrame buildStateFrame(
            const VehicleState::StateVector& state,
            const LocalMapView& map) const noexcept;

        static CellCoordinates WorldToCell(float xMeters, float yMeters, float cellSizeM) noexcept;

    private:
        static bool intersectRayAabb(
            const Eigen::Vector2f& origin,
            const Eigen::Vector2f& direction,
            const Eigen::Vector2f& minCorner,
            const Eigen::Vector2f& maxCorner,
            float& rangeM) noexcept;
        static void testUniqueWall(
            const Eigen::Vector2f& rayOrigin,
            const Eigen::Vector2f& rayDirection,
            const CellCoordinates& cell,
            Direction direction,
            WallState state,
            const LocalMapView& map,
            GeometryPrediction& best) noexcept;
    };
}
