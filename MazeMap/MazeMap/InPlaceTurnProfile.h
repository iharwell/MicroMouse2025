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
        const InPlaceTurnProfile& profile,
        float& angularCommandRadps) noexcept
    {
        if (!IsInPlaceTurnProfileValid(profile) ||
            !std::isfinite(remainingRad) ||
            !std::isfinite(angularSpeedRadps) ||
            !std::isfinite(profile.angularAccelRadps2))
        {
            return false;
        }

        // The turn helper owns only the heading-derived yaw-rate setpoint. DriveBase owns the
        // present-state transition shaping through the shared velocity-target response path.
        const float decelLimitedOmega = sqrtf((std::max)(0.0f, 2.0f * profile.angularAccelRadps2 * std::fabs(remainingRad)));
        const float desiredOmegaRadps = (std::min)(profile.maxAngularSpeedRadps, decelLimitedOmega);

        const float direction = (remainingRad >= 0.0f) ? 1.0f : -1.0f;
        angularCommandRadps =
            (direction * desiredOmegaRadps) +
            (profile.headingKp * remainingRad) -
            (profile.yawD * angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -profile.maxAngularSpeedRadps,
            profile.maxAngularSpeedRadps);
        return std::isfinite(angularCommandRadps);
    }
}
