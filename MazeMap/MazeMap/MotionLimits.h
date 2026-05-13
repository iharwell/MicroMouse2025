#pragma once

#include "CoreConfig.h"
#include "Defines.h"

class EXPORT MotionLimits final
{
public:
    MotionLimits() noexcept;
    MotionLimits(
        float maxSpeedMps,
        float accelMps2,
        float decelMps2,
        float maxAngularSpeedRadps,
        float angularAccelRadps2,
        float angleToleranceRad = MazeMap::Config::kAngleToleranceRad) noexcept;

    float GetMaxSpeedMps() const noexcept { return _maxSpeedMps; }
    float GetAccelMps2() const noexcept { return _accelMps2; }
    float GetDecelMps2() const noexcept { return _decelMps2; }
    float GetMaxAngularSpeedRadps() const noexcept { return _maxAngularSpeedRadps; }
    float GetAngularAccelRadps2() const noexcept { return _angularAccelRadps2; }
    float GetAngleToleranceRad() const noexcept { return _angleToleranceRad; }
    float GetEffectiveAngleToleranceRad(
        float fallbackAngleToleranceRad = MazeMap::Config::kAngleToleranceRad) const noexcept;
    float ComputeMinimumTurnDurationSeconds(float angleRad) const noexcept;

    MotionLimits& SetMaxSpeedMps(float value) noexcept;
    MotionLimits& SetAccelMps2(float value) noexcept;
    MotionLimits& SetDecelMps2(float value) noexcept;
    MotionLimits& SetMaxAngularSpeedRadps(float value) noexcept;
    MotionLimits& SetAngularAccelRadps2(float value) noexcept;
    MotionLimits& SetAngleToleranceRad(float value) noexcept;

    MotionLimits& SetLinearEnvelope(float maxSpeedMps, float accelMps2, float decelMps2) noexcept;
    MotionLimits& SetAngularEnvelope(
        float maxAngularSpeedRadps,
        float angularAccelRadps2,
        float angleToleranceRad = MazeMap::Config::kAngleToleranceRad) noexcept;

private:
    float _maxSpeedMps = 0.0f;
    float _accelMps2 = 0.0f;
    float _decelMps2 = 0.0f;
    float _maxAngularSpeedRadps = 0.0f;
    float _angularAccelRadps2 = 0.0f;
    float _angleToleranceRad = MazeMap::Config::kAngleToleranceRad;
};
