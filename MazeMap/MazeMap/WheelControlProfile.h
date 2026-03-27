#pragma once

#include <cmath>

namespace MazeMap
{
    struct WheelControlProfile
    {
        float velocityKpScale = 1.0f;
        float velocityKiScale = 1.0f;
        float integralLimitScale = 1.0f;
        float accelerationResponseScale = 0.0f;
    };

    inline float NormalizeWheelControlScale(float scale) noexcept
    {
        return (std::isfinite(scale) && (scale > 0.0f)) ? scale : 1.0f;
    }

    inline WheelControlProfile NormalizeWheelControlProfile(const WheelControlProfile& profile) noexcept
    {
        WheelControlProfile normalized{};
        normalized.velocityKpScale = NormalizeWheelControlScale(profile.velocityKpScale);
        normalized.velocityKiScale = NormalizeWheelControlScale(profile.velocityKiScale);
        normalized.integralLimitScale = NormalizeWheelControlScale(profile.integralLimitScale);
        normalized.accelerationResponseScale =
            (std::isfinite(profile.accelerationResponseScale) && (profile.accelerationResponseScale >= 0.0f)) ?
            profile.accelerationResponseScale :
            0.0f;
        return normalized;
    }

    inline float ScaleWheelControlValue(float baseValue, float scale) noexcept
    {
        if (!std::isfinite(baseValue) || (baseValue <= 0.0f))
        {
            return 0.0f;
        }

        return baseValue * NormalizeWheelControlScale(scale);
    }
}
