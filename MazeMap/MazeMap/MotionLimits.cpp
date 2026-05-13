#include "pch.h"
#include "MotionLimits.h"

#include <cmath>

MotionLimits::MotionLimits() noexcept = default;

MotionLimits::MotionLimits(
    const float maxSpeedMps,
    const float accelMps2,
    const float decelMps2,
    const float maxAngularSpeedRadps,
    const float angularAccelRadps2,
    const float angleToleranceRad) noexcept
    :
    _maxSpeedMps(maxSpeedMps),
    _accelMps2(accelMps2),
    _decelMps2(decelMps2),
    _maxAngularSpeedRadps(maxAngularSpeedRadps),
    _angularAccelRadps2(angularAccelRadps2),
    _angleToleranceRad(angleToleranceRad)
{
}

MotionLimits& MotionLimits::SetMaxSpeedMps(const float value) noexcept
{
    _maxSpeedMps = value;
    return *this;
}

MotionLimits& MotionLimits::SetAccelMps2(const float value) noexcept
{
    _accelMps2 = value;
    return *this;
}

MotionLimits& MotionLimits::SetDecelMps2(const float value) noexcept
{
    _decelMps2 = value;
    return *this;
}

MotionLimits& MotionLimits::SetMaxAngularSpeedRadps(const float value) noexcept
{
    _maxAngularSpeedRadps = value;
    return *this;
}

MotionLimits& MotionLimits::SetAngularAccelRadps2(const float value) noexcept
{
    _angularAccelRadps2 = value;
    return *this;
}

MotionLimits& MotionLimits::SetAngleToleranceRad(const float value) noexcept
{
    _angleToleranceRad = value;
    return *this;
}

float MotionLimits::GetEffectiveAngleToleranceRad(const float fallbackAngleToleranceRad) const noexcept
{
    const float fallback =
        std::isfinite(fallbackAngleToleranceRad) ?
        fallbackAngleToleranceRad :
        0.0f;
    return std::fabs(std::isfinite(_angleToleranceRad) ? _angleToleranceRad : fallback);
}

float MotionLimits::ComputeMinimumTurnDurationSeconds(const float angleRad) const noexcept
{
    const float absoluteAngleRad = std::fabs(std::isfinite(angleRad) ? angleRad : 0.0f);
    const float angularAccelRadps2 =
        std::isfinite(_angularAccelRadps2) ?
        std::fabs(_angularAccelRadps2) :
        0.0f;
    if (!(absoluteAngleRad > 0.0f) || !(angularAccelRadps2 > 0.0f))
    {
        return 0.0f;
    }

    const float maxAngularSpeedRadps =
        std::isfinite(_maxAngularSpeedRadps) ?
        std::fabs(_maxAngularSpeedRadps) :
        0.0f;
    if (!(maxAngularSpeedRadps > 0.0f))
    {
        return 2.0f * std::sqrt(absoluteAngleRad / angularAccelRadps2);
    }

    const float accelAndDecelAngleRad =
        (maxAngularSpeedRadps * maxAngularSpeedRadps) / angularAccelRadps2;
    if (absoluteAngleRad <= accelAndDecelAngleRad)
    {
        return 2.0f * std::sqrt(absoluteAngleRad / angularAccelRadps2);
    }

    return
        (2.0f * maxAngularSpeedRadps / angularAccelRadps2) +
        ((absoluteAngleRad - accelAndDecelAngleRad) / maxAngularSpeedRadps);
}

MotionLimits& MotionLimits::SetLinearEnvelope(
    const float maxSpeedMps,
    const float accelMps2,
    const float decelMps2) noexcept
{
    _maxSpeedMps = maxSpeedMps;
    _accelMps2 = accelMps2;
    _decelMps2 = decelMps2;
    return *this;
}

MotionLimits& MotionLimits::SetAngularEnvelope(
    const float maxAngularSpeedRadps,
    const float angularAccelRadps2,
    const float angleToleranceRad) noexcept
{
    _maxAngularSpeedRadps = maxAngularSpeedRadps;
    _angularAccelRadps2 = angularAccelRadps2;
    _angleToleranceRad = angleToleranceRad;
    return *this;
}
