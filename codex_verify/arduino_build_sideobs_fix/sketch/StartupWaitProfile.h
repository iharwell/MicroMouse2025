#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\StartupWaitProfile.h"
#pragma once

namespace MazeMap
{
    inline bool IsStartupWaitIndicatorEnabled(unsigned long elapsedMs, unsigned long blinkPeriodMs)
    {
        if (blinkPeriodMs < 2UL)
        {
            return false;
        }

        const unsigned long halfPeriodMs = blinkPeriodMs / 2UL;
        return (elapsedMs % blinkPeriodMs) < halfPeriodMs;
    }
}
