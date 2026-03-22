#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
    struct TurningTractionMetrics
    {
        float encoderLinearSpeedMps = 0.0f;
        float encoderOmegaRadps = 0.0f;
        float predictedLateralAccelMps2 = 0.0f;
        float yawCoherence = 0.0f;
        float planarCoherence = 0.0f;
    };

    struct TurningLaunchCommands
    {
        float leftCommand = 0.0f;
        float rightCommand = 0.0f;
    };

    inline float WrapSignedAngleRad(float angleRad) noexcept
    {
        if (!std::isfinite(angleRad))
        {
            return 0.0f;
        }

        while (angleRad > PI_F)
        {
            angleRad -= (2.0f * PI_F);
        }

        while (angleRad < -PI_F)
        {
            angleRad += (2.0f * PI_F);
        }

        return angleRad;
    }

    inline float ComputePositiveCoherenceRatio(float measuredMagnitude, float predictedMagnitude)
    {
        if (!std::isfinite(measuredMagnitude) || !std::isfinite(predictedMagnitude) || predictedMagnitude <= 1.0e-6f)
        {
            return 0.0f;
        }

        return (measuredMagnitude > 0.0f) ? (measuredMagnitude / predictedMagnitude) : 0.0f;
    }

    inline TurningTractionMetrics ComputeTurningTractionMetrics(
        float leftVelocityMps,
        float rightVelocityMps,
        float trackWidthM,
        float gyroOmegaRadps,
        float planarAccelMps2)
    {
        TurningTractionMetrics metrics{};
        if (!std::isfinite(leftVelocityMps) ||
            !std::isfinite(rightVelocityMps) ||
            !std::isfinite(trackWidthM) ||
            trackWidthM <= 0.0f)
        {
            return metrics;
        }

        metrics.encoderLinearSpeedMps = std::fabs(0.5f * (leftVelocityMps + rightVelocityMps));
        metrics.encoderOmegaRadps = (rightVelocityMps - leftVelocityMps) / trackWidthM;
        metrics.predictedLateralAccelMps2 = std::fabs(metrics.encoderLinearSpeedMps * metrics.encoderOmegaRadps);
        metrics.yawCoherence = ComputePositiveCoherenceRatio(std::fabs(gyroOmegaRadps), std::fabs(metrics.encoderOmegaRadps));
        metrics.planarCoherence = ComputePositiveCoherenceRatio(std::fabs(planarAccelMps2), metrics.predictedLateralAccelMps2);
        return metrics;
    }

    inline TurningLaunchCommands ComputeTurningLaunchCommands(
        float linearSpeedMps,
        float angularSpeedRadps,
        float trackWidthM,
        float outerWheelLaunchCommand)
    {
        TurningLaunchCommands commands{};
        if (!std::isfinite(linearSpeedMps) ||
            !std::isfinite(angularSpeedRadps) ||
            !std::isfinite(trackWidthM) ||
            !std::isfinite(outerWheelLaunchCommand) ||
            trackWidthM <= 0.0f ||
            outerWheelLaunchCommand <= 0.0f)
        {
            return commands;
        }

        const float leftTargetMps = linearSpeedMps - (0.5f * trackWidthM * angularSpeedRadps);
        const float rightTargetMps = linearSpeedMps + (0.5f * trackWidthM * angularSpeedRadps);
        const float maxTargetMagnitudeMps = (std::max)(std::fabs(leftTargetMps), std::fabs(rightTargetMps));
        if (maxTargetMagnitudeMps <= 1.0e-6f)
        {
            return commands;
        }

        const float scale = outerWheelLaunchCommand / maxTargetMagnitudeMps;
        commands.leftCommand = (std::clamp)(leftTargetMps * scale, -1.0f, 1.0f);
        commands.rightCommand = (std::clamp)(rightTargetMps * scale, -1.0f, 1.0f);
        return commands;
    }

    inline float ComputeTurningTractionAngularCommand(
        float nominalAngularSpeedRadps,
        float targetYawRad,
        float measuredYawRad,
        float measuredYawRateRadps,
        float headingKp,
        float yawD,
        float maxAngularSpeedRadps) noexcept
    {
        if (!std::isfinite(nominalAngularSpeedRadps) ||
            !std::isfinite(targetYawRad) ||
            !std::isfinite(measuredYawRad) ||
            !std::isfinite(measuredYawRateRadps) ||
            !std::isfinite(headingKp) ||
            !std::isfinite(yawD) ||
            headingKp < 0.0f ||
            yawD < 0.0f)
        {
            return 0.0f;
        }

        const float headingErrorRad = WrapSignedAngleRad(targetYawRad - measuredYawRad);
        const float commandedAngularSpeedRadps =
            nominalAngularSpeedRadps +
            (headingKp * headingErrorRad) -
            (yawD * measuredYawRateRadps);
        if (std::isfinite(maxAngularSpeedRadps) && (maxAngularSpeedRadps > 0.0f))
        {
            return (std::clamp)(commandedAngularSpeedRadps, -maxAngularSpeedRadps, maxAngularSpeedRadps);
        }

        return commandedAngularSpeedRadps;
    }

    inline bool IsTurningTractionLossDetected(
        const TurningTractionMetrics& metrics,
        float minLinearSpeedMps,
        float minPredictedLateralAccelMps2,
        float yawCoherenceFloor,
        float planarCoherenceFloor)
    {
        if (!std::isfinite(minLinearSpeedMps) ||
            !std::isfinite(minPredictedLateralAccelMps2) ||
            !std::isfinite(yawCoherenceFloor) ||
            !std::isfinite(planarCoherenceFloor) ||
            minLinearSpeedMps <= 0.0f ||
            minPredictedLateralAccelMps2 <= 0.0f ||
            yawCoherenceFloor <= 0.0f ||
            planarCoherenceFloor <= 0.0f)
        {
            return false;
        }

        return
            (metrics.encoderLinearSpeedMps >= minLinearSpeedMps) &&
            (metrics.predictedLateralAccelMps2 >= minPredictedLateralAccelMps2) &&
            (metrics.yawCoherence < yawCoherenceFloor) &&
            (metrics.planarCoherence < planarCoherenceFloor);
    }
}
