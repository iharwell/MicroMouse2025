#pragma once

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    inline float ApplyMinimumCruiseSpeedFloor(
        float requestedSpeedMps,
        float minimumCruiseSpeedMps,
        float maximumCruiseSpeedMps) noexcept
    {
        if (!std::isfinite(requestedSpeedMps) ||
            !std::isfinite(minimumCruiseSpeedMps) ||
            !std::isfinite(maximumCruiseSpeedMps) ||
            maximumCruiseSpeedMps <= 0.0f)
        {
            return 0.0f;
        }

        if (requestedSpeedMps <= 0.0f)
        {
            return 0.0f;
        }

        const float clampedMaximumCruiseSpeedMps = (std::max)(0.0f, maximumCruiseSpeedMps);
        const float clampedMinimumCruiseSpeedMps = (std::clamp)(minimumCruiseSpeedMps, 0.0f, clampedMaximumCruiseSpeedMps);
        const float clampedRequestedSpeedMps = (std::clamp)(requestedSpeedMps, 0.0f, clampedMaximumCruiseSpeedMps);
        return (std::max)(clampedRequestedSpeedMps, clampedMinimumCruiseSpeedMps);
    }
}
