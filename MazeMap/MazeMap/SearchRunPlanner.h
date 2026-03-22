#pragma once

#include "Defines.h"
#include "Maze.h"
#include "Path.h"
#include <cmath>
#include <stdint.h>

namespace MazeMap
{
    struct SearchStraightPlan
    {
        Direction direction = Direction::None;
        uint16_t segmentEndIndex = 0U;
        uint16_t fullSpeedCellCount = 0U;
        uint16_t cautiousCellCount = 0U;

        MAZEMAP_INLINE uint16_t TotalCellCount() const
        {
            return static_cast<uint16_t>(fullSpeedCellCount + cautiousCellCount);
        }
    };

    template<int Size>
    MAZEMAP_INLINE SearchStraightPlan PlanSearchStraightSegment(const Maze& maze, const Path<Size>& path, uint16_t startIndex)
    {
        SearchStraightPlan plan{};
        if (startIndex == 0U || startIndex >= path.GetSize())
        {
            return plan;
        }

        const Direction direction = path[startIndex - 1U].DirectionTo(path[startIndex]);
        if (direction == Direction::None)
        {
            return plan;
        }

        uint16_t segmentEndIndex = startIndex;
        while ((segmentEndIndex + 1U) < path.GetSize() && path[segmentEndIndex].DirectionTo(path[segmentEndIndex + 1U]) == direction)
        {
            ++segmentEndIndex;
        }

        const uint16_t totalCellCount = static_cast<uint16_t>(segmentEndIndex - startIndex + 1U);
        plan.direction = direction;
        plan.segmentEndIndex = segmentEndIndex;

        if (!maze[path[segmentEndIndex]].IsFullyKnown())
        {
            plan.fullSpeedCellCount = static_cast<uint16_t>(totalCellCount - 1U);
            plan.cautiousCellCount = 1U;
            return plan;
        }

        plan.fullSpeedCellCount = totalCellCount;
        return plan;
    }

    MAZEMAP_INLINE float ComputeSafeUnmappedCruiseSpeed(
        float maxDecelMps2,
        float frontWallDetectionThresholdM,
        float frontSensorForwardOffsetM,
        float stopStandoffM,
        float distanceToleranceM = 0.0f)
    {
        if (!(maxDecelMps2 > 0.0f))
        {
            return 0.0f;
        }

        const float brakingDistanceM =
            frontWallDetectionThresholdM +
            frontSensorForwardOffsetM -
            stopStandoffM -
            ((distanceToleranceM > 0.0f) ? distanceToleranceM : 0.0f);

        if (!(brakingDistanceM > 0.0f))
        {
            return 0.0f;
        }

        return sqrtf(2.0f * maxDecelMps2 * brakingDistanceM);
    }
}
