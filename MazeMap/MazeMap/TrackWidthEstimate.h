#pragma once

#include <cmath>

namespace MazeMap
{
    inline bool TryComputeEffectiveTrackWidthM(
        float leftDistanceM,
        float rightDistanceM,
        float yawChangeRad,
        float& effectiveTrackWidthM) noexcept
    {
        if (!std::isfinite(leftDistanceM) ||
            !std::isfinite(rightDistanceM) ||
            !std::isfinite(yawChangeRad) ||
            std::fabs(yawChangeRad) < 1.0e-4f)
        {
            effectiveTrackWidthM = 0.0f;
            return false;
        }

        effectiveTrackWidthM = std::fabs((rightDistanceM - leftDistanceM) / yawChangeRad);
        return std::isfinite(effectiveTrackWidthM) && (effectiveTrackWidthM > 0.0f);
    }
}
