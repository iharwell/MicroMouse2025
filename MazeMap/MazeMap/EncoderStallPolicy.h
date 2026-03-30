#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
    inline float ComputeAverageEncoderAbsSpeedMps(float leftVelocityMps, float rightVelocityMps)
    {
        if (!std::isfinite(leftVelocityMps) || !std::isfinite(rightVelocityMps))
        {
            return 0.0f;
        }

        return 0.5f * (std::fabs(leftVelocityMps) + std::fabs(rightVelocityMps));
    }

    inline bool IsEncoderProgressWatchdogArmed(
        float commandedSpeedMps,
        float remainingM,
        unsigned long activeMotionMs,
        float commandThresholdMps,
        float distanceToleranceM,
        unsigned long startupGraceMs)
    {
        if (!std::isfinite(commandedSpeedMps) ||
            !std::isfinite(remainingM) ||
            !std::isfinite(commandThresholdMps) ||
            !std::isfinite(distanceToleranceM) ||
            commandThresholdMps <= 0.0f ||
            distanceToleranceM < 0.0f)
        {
            return false;
        }

        if ((commandedSpeedMps < commandThresholdMps) || (remainingM <= distanceToleranceM))
        {
            return false;
        }

        return activeMotionMs >= startupGraceMs;
    }
}
