#pragma once

#include <cmath>
#include <algorithm>

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

    inline float ClampWheelDriveCommand(float command) noexcept
    {
        if (!std::isfinite(command))
        {
            return 0.0f;
        }

        return (std::clamp)(command, -1.0f, 1.0f);
    }

    inline bool ShouldAccumulateWheelVelocityIntegral(
        float unclampedCommand,
        float clampedCommand,
        float errorMps) noexcept
    {
        if (!std::isfinite(unclampedCommand) ||
            !std::isfinite(clampedCommand) ||
            !std::isfinite(errorMps))
        {
            return false;
        }

        if (std::fabs(unclampedCommand - clampedCommand) <= 1.0e-6f)
        {
            return true;
        }

        if (clampedCommand >= 1.0f)
        {
            return errorMps < 0.0f;
        }

        if (clampedCommand <= -1.0f)
        {
            return errorMps > 0.0f;
        }

        return true;
    }
}
