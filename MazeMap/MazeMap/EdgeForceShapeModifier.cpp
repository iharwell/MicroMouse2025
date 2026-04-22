#include "pch.h"
#include "EdgeForceShapeModifier.h"

#include <algorithm>
#include <cmath>

namespace
{
    inline float Clamp01(float value) noexcept
    {
        if (!std::isfinite(value))
        {
            return 0.0f;
        }

        return (std::clamp)(value, 0.0f, 1.0f);
    }

    inline float SmoothStep(float value) noexcept
    {
        const float clamped = Clamp01(value);
        return clamped * clamped * (3.0f - (2.0f * clamped));
    }
}

namespace MazeMap
{
    float EdgeForceShapeModifier::ComputeUtilizationScale(
        float utilization,
        float bankMemory,
        float frozenStrength,
        bool holdoffActive,
        bool recoveryActive) noexcept
    {
        const float memoryWeight = Clamp01(bankMemory);
        const float strength = Clamp01(frozenStrength);
        const float saturationRegion = SmoothStep((utilization - 0.65f) / 0.35f);
        float modifier =
            1.0f -
            (0.20f * strength * saturationRegion) -
            (0.10f * memoryWeight * saturationRegion);

        if (holdoffActive)
        {
            modifier -= 0.05f * strength;
        }
        if (recoveryActive)
        {
            modifier -= 0.10f * (0.5f + (0.5f * strength));
        }

        return (std::clamp)(modifier, 0.60f, 1.00f);
    }

    float EdgeForceShapeModifier::ComputeCapacityScale(
        float bankMemory,
        float frozenStrength,
        bool holdoffActive,
        bool recoveryActive) noexcept
    {
        float modifier =
            1.0f -
            (0.10f * Clamp01(bankMemory)) -
            (0.08f * Clamp01(frozenStrength));
        if (holdoffActive)
        {
            modifier -= 0.04f;
        }
        if (recoveryActive)
        {
            modifier -= 0.08f;
        }

        return (std::clamp)(modifier, 0.65f, 1.00f);
    }
}
