#pragma once

#include <cmath>
#include <stdint.h>

namespace MazeMap
{
    inline float ComputeAverageEncoderAbsSpeedMps(float leftVelocityMps, float rightVelocityMps)
    {
        if (!std::isfinite(leftVelocityMps) || !std::isfinite(rightVelocityMps))
        {
            return 0.0f;
        }

        return 0.5f * (std::fabs(leftVelocityMps) + std::fabs(rightVelocityMps));
    }

    inline bool IsWallTapMotionEstablished(
        float peakEncoderSpeedMps,
        float traveledDistanceM,
        float minimumMotionSpeedMps,
        float minimumMotionDistanceM)
    {
        if (!std::isfinite(peakEncoderSpeedMps) ||
            !std::isfinite(traveledDistanceM) ||
            !std::isfinite(minimumMotionSpeedMps) ||
            !std::isfinite(minimumMotionDistanceM) ||
            peakEncoderSpeedMps < 0.0f ||
            traveledDistanceM < 0.0f ||
            minimumMotionSpeedMps <= 0.0f ||
            minimumMotionDistanceM < 0.0f)
        {
            return false;
        }

        return
            (peakEncoderSpeedMps >= minimumMotionSpeedMps) ||
            (traveledDistanceM >= minimumMotionDistanceM);
    }

    inline bool HasSharpEncoderVelocityDecline(
        float peakEncoderSpeedMps,
        float currentEncoderSpeedMps,
        float minimumPeakEncoderSpeedMps,
        float maximumCurrentPeakRatio,
        float minimumDropMps)
    {
        if (!std::isfinite(peakEncoderSpeedMps) ||
            !std::isfinite(currentEncoderSpeedMps) ||
            !std::isfinite(minimumPeakEncoderSpeedMps) ||
            !std::isfinite(maximumCurrentPeakRatio) ||
            !std::isfinite(minimumDropMps) ||
            peakEncoderSpeedMps < 0.0f ||
            currentEncoderSpeedMps < 0.0f ||
            minimumPeakEncoderSpeedMps <= 0.0f ||
            maximumCurrentPeakRatio <= 0.0f ||
            maximumCurrentPeakRatio >= 1.0f ||
            minimumDropMps <= 0.0f)
        {
            return false;
        }

        if (peakEncoderSpeedMps < minimumPeakEncoderSpeedMps)
        {
            return false;
        }

        const float speedDropMps = peakEncoderSpeedMps - currentEncoderSpeedMps;
        return
            (speedDropMps >= minimumDropMps) &&
            (currentEncoderSpeedMps <= (peakEncoderSpeedMps * maximumCurrentPeakRatio));
    }

    inline bool HasPlanarAccelContactSpike(
        float baselinePlanarAccelMps2,
        float currentPlanarAccelMps2,
        float minimumSpikeMps2)
    {
        if (!std::isfinite(baselinePlanarAccelMps2) ||
            !std::isfinite(currentPlanarAccelMps2) ||
            !std::isfinite(minimumSpikeMps2) ||
            baselinePlanarAccelMps2 < 0.0f ||
            currentPlanarAccelMps2 < 0.0f ||
            minimumSpikeMps2 <= 0.0f)
        {
            return false;
        }

        return currentPlanarAccelMps2 >= (baselinePlanarAccelMps2 + minimumSpikeMps2);
    }

    inline bool ShouldArmBoundaryImpactWatch(float distanceToBoundaryTouchM, float armDistanceM)
    {
        if (!std::isfinite(distanceToBoundaryTouchM) ||
            !std::isfinite(armDistanceM) ||
            armDistanceM <= 0.0f)
        {
            return false;
        }

        return distanceToBoundaryTouchM <= armDistanceM;
    }

    inline bool HasClearedBoundaryWithoutImpact(float distanceToBoundaryTouchM, float clearMarginM)
    {
        if (!std::isfinite(distanceToBoundaryTouchM) ||
            !std::isfinite(clearMarginM) ||
            clearMarginM < 0.0f)
        {
            return false;
        }

        return distanceToBoundaryTouchM <= -clearMarginM;
    }

    inline bool ShouldRetryWallTapAfterNoMotion(
        unsigned long elapsedMs,
        float peakEncoderSpeedMps,
        float traveledDistanceM,
        unsigned long noMotionTimeoutMs,
        float minimumMotionSpeedMps,
        float minimumMotionDistanceM)
    {
        if (elapsedMs < noMotionTimeoutMs)
        {
            return false;
        }

        return !IsWallTapMotionEstablished(
            peakEncoderSpeedMps,
            traveledDistanceM,
            minimumMotionSpeedMps,
            minimumMotionDistanceM);
    }
}
