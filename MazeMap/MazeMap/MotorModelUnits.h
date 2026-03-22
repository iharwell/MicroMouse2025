#pragma once

#include "Defines.h"

namespace MazeMap
{
    inline constexpr float MilliAmpsToAmps(float milliamps) noexcept
    {
        return milliamps * 1.0e-3f;
    }

    inline constexpr float MilliNewtonMetersToNewtonMeters(float milliNewtonMeters) noexcept
    {
        return milliNewtonMeters * 1.0e-3f;
    }

    inline constexpr float RpmPerVoltToRadPerSecondPerVolt(float rpmPerVolt) noexcept
    {
        return rpmPerVolt * ((2.0f * PI_F) / 60.0f);
    }

    inline constexpr float RpmToRadPerSecond(float rpm) noexcept
    {
        return rpm * ((2.0f * PI_F) / 60.0f);
    }

    inline constexpr float ComputeMotorSpeedConstantRadpsPerVolt(
        float noLoadSpeedRpm,
        float nominalVoltageV,
        float noLoadCurrentA,
        float terminalResistanceOhms) noexcept
    {
        const float effectiveBackEmfVoltageV = nominalVoltageV - (noLoadCurrentA * terminalResistanceOhms);
        return (effectiveBackEmfVoltageV > 0.0f)
            ? (RpmToRadPerSecond(noLoadSpeedRpm) / effectiveBackEmfVoltageV)
            : 0.0f;
    }
}
