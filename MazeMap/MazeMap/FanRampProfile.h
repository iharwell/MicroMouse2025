#pragma once

#include "Defines.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    inline float ComputeFanRampDutyCycle(float targetDutyCycle, unsigned long elapsedMs, unsigned long rampDurationMs)
    {
        const float clampedTargetDutyCycle = std::isfinite(targetDutyCycle)
            ? (std::clamp)(targetDutyCycle, 0.0f, 1.0f)
            : 0.0f;
        if (clampedTargetDutyCycle <= 0.0f)
        {
            return 0.0f;
        }

        if (rampDurationMs == 0UL)
        {
            return clampedTargetDutyCycle;
        }

        const float progress = (std::clamp)(
            static_cast<float>(elapsedMs) / static_cast<float>(rampDurationMs),
            0.0f,
            1.0f);
        return clampedTargetDutyCycle * progress;
    }
}
