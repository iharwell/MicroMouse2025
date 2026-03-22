#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MotionTargetProjection.h"
#pragma once

#include "Defines.h"
#include <cmath>

namespace MazeMap
{
    MAZEMAP_INLINE bool TryComputeProjectedDistanceToTargetM(
        float currentXM,
        float currentYM,
        float targetXM,
        float targetYM,
        float directionX,
        float directionY,
        float& projectedDistanceM) noexcept
    {
        projectedDistanceM = 0.0f;
        if (!std::isfinite(currentXM) ||
            !std::isfinite(currentYM) ||
            !std::isfinite(targetXM) ||
            !std::isfinite(targetYM) ||
            !std::isfinite(directionX) ||
            !std::isfinite(directionY))
        {
            return false;
        }

        const float directionMagnitudeSq = (directionX * directionX) + (directionY * directionY);
        if (!(directionMagnitudeSq > 0.25f))
        {
            return false;
        }

        projectedDistanceM = ((targetXM - currentXM) * directionX) + ((targetYM - currentYM) * directionY);
        return std::isfinite(projectedDistanceM);
    }
}
