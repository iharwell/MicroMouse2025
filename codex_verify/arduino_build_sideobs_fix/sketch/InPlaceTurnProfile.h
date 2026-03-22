#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\InPlaceTurnProfile.h"
#pragma once

#include "Defines.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    struct InPlaceTurnProfile
    {
        float maxAngularSpeedRadps = 0.0f;
        float angularAccelRadps2 = 0.0f;
        float headingKp = 0.0f;
        float yawD = 0.0f;
        float angleToleranceRad = 0.0f;
        float angularSpeedToleranceRadps = 0.0f;
    };

    inline bool IsInPlaceTurnProfileValid(const InPlaceTurnProfile& profile) noexcept
    {
        return
            std::isfinite(profile.maxAngularSpeedRadps) &&
            std::isfinite(profile.angularAccelRadps2) &&
            std::isfinite(profile.headingKp) &&
            std::isfinite(profile.yawD) &&
            std::isfinite(profile.angleToleranceRad) &&
            std::isfinite(profile.angularSpeedToleranceRadps) &&
            (profile.maxAngularSpeedRadps > 0.0f) &&
            (profile.angularAccelRadps2 > 0.0f) &&
            (profile.headingKp >= 0.0f) &&
            (profile.yawD >= 0.0f) &&
            (profile.angleToleranceRad > 0.0f) &&
            (profile.angularSpeedToleranceRadps > 0.0f);
    }

    inline bool IsInPlaceTurnComplete(
        float remainingRad,
        float angularSpeedRadps,
        const InPlaceTurnProfile& profile) noexcept
    {
        return
            IsInPlaceTurnProfileValid(profile) &&
            std::isfinite(remainingRad) &&
            std::isfinite(angularSpeedRadps) &&
            (std::fabs(remainingRad) <= profile.angleToleranceRad) &&
            (std::fabs(angularSpeedRadps) <= profile.angularSpeedToleranceRadps);
    }

    inline bool TryComputeInPlaceTurnCommandRadps(
        float remainingRad,
        float angularSpeedRadps,
        float dtSeconds,
        const InPlaceTurnProfile& profile,
        float& commandedOmegaRadps,
        float& angularCommandRadps) noexcept
    {
        if (!IsInPlaceTurnProfileValid(profile) ||
            !std::isfinite(remainingRad) ||
            !std::isfinite(angularSpeedRadps) ||
            !std::isfinite(dtSeconds) ||
            !std::isfinite(commandedOmegaRadps) ||
            dtSeconds <= 0.0f)
        {
            return false;
        }

        const float accelLimitedOmega = (std::min)(
            profile.maxAngularSpeedRadps,
            (std::max)(0.0f, commandedOmegaRadps + (profile.angularAccelRadps2 * dtSeconds)));
        const float decelLimitedOmega = sqrtf((std::max)(0.0f, 2.0f * profile.angularAccelRadps2 * std::fabs(remainingRad)));
        commandedOmegaRadps = (std::min)(accelLimitedOmega, decelLimitedOmega);

        const float direction = (remainingRad >= 0.0f) ? 1.0f : -1.0f;
        angularCommandRadps =
            (direction * commandedOmegaRadps) +
            (profile.headingKp * remainingRad) -
            (profile.yawD * angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -profile.maxAngularSpeedRadps,
            profile.maxAngularSpeedRadps);
        return std::isfinite(angularCommandRadps);
    }
}
