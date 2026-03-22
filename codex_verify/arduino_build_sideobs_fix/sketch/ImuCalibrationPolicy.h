#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ImuCalibrationPolicy.h"
#pragma once

#include <cmath>
#include <cstdint>

namespace MazeMap
{
    struct EncoderCountPair
    {
        int32_t left = 0;
        int32_t right = 0;
    };

    constexpr unsigned long ComputeMinimumSampleCountForDurationMs(
        unsigned long sampleIntervalMs,
        unsigned long minimumDurationMs) noexcept
    {
        if (sampleIntervalMs == 0UL)
        {
            return 0UL;
        }

        return (minimumDurationMs + sampleIntervalMs - 1UL) / sampleIntervalMs;
    }

    constexpr unsigned long ComputeGyroBiasSampleCount(
        unsigned long configuredSamples,
        unsigned long sampleIntervalMs,
        unsigned long minimumDurationMs) noexcept
    {
        const unsigned long minimumSamples =
            ComputeMinimumSampleCountForDurationMs(sampleIntervalMs, minimumDurationMs);
        return (configuredSamples >= minimumSamples) ? configuredSamples : minimumSamples;
    }

    constexpr bool HaveEncoderCountsChanged(
        const EncoderCountPair& start,
        const EncoderCountPair& current) noexcept
    {
        return (start.left != current.left) || (start.right != current.right);
    }

    inline bool IsAccelSelfTestDeltaValidMg(float deltaMg) noexcept
    {
        const float absoluteDeltaMg = std::fabs(deltaMg);
        return std::isfinite(absoluteDeltaMg) &&
            (absoluteDeltaMg >= 50.0f) &&
            (absoluteDeltaMg <= 1700.0f);
    }

    inline bool IsGyroSelfTestDeltaValidDps(float deltaDps, float fullScaleDps) noexcept
    {
        const float absoluteDeltaDps = std::fabs(deltaDps);
        if (!std::isfinite(absoluteDeltaDps) || !std::isfinite(fullScaleDps))
        {
            return false;
        }

        if (std::fabs(fullScaleDps - 250.0f) < 0.5f)
        {
            return (absoluteDeltaDps >= 20.0f) && (absoluteDeltaDps <= 80.0f);
        }

        if (std::fabs(fullScaleDps - 2000.0f) < 0.5f)
        {
            return (absoluteDeltaDps >= 150.0f) && (absoluteDeltaDps <= 700.0f);
        }

        return false;
    }
}
