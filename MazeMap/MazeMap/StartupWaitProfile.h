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
