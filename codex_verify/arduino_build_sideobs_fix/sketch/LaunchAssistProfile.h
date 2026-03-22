#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\LaunchAssistProfile.h"
#pragma once

#include "Defines.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    inline float ComputeLaunchAssistDriveFloor(
        float initialDriveCommand,
        float maxDriveCommand,
        unsigned long elapsedMs,
        unsigned long rampMs)
    {
        if (!std::isfinite(initialDriveCommand) || !std::isfinite(maxDriveCommand))
        {
            return 0.0f;
        }

        const float startCommand = (std::clamp)(initialDriveCommand, 0.0f, 1.0f);
        const float endCommand = (std::clamp)(maxDriveCommand, startCommand, 1.0f);
        if (endCommand <= startCommand)
        {
            return endCommand;
        }

        if (rampMs == 0UL)
        {
            return endCommand;
        }

        const float alpha = (elapsedMs >= rampMs) ? 1.0f : (static_cast<float>(elapsedMs) / static_cast<float>(rampMs));
        return startCommand + ((endCommand - startCommand) * alpha);
    }
}
