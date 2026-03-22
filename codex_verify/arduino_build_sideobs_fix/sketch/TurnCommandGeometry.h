#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\TurnCommandGeometry.h"
#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
    inline bool TryComputeSignedTurnAngleRad(float currentYawRad, float targetYawRad, float& angleRad) noexcept
    {
        angleRad = 0.0f;
        if (!std::isfinite(currentYawRad) || !std::isfinite(targetYawRad))
        {
            return false;
        }

        angleRad = std::remainder(targetYawRad - currentYawRad, TWO_PI);
        return std::isfinite(angleRad);
    }
}
