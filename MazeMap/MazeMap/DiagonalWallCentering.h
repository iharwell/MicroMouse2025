#pragma once

#include <cmath>

namespace MazeMap
{
    inline bool TryComputeNormalizedWallSignalBalanceError(
        float leftMeasuredSignal,
        float leftReferenceSignal,
        float rightMeasuredSignal,
        float rightReferenceSignal,
        float minNormalizedSignal,
        float& balanceError)
    {
        balanceError = 0.0f;
        if (!std::isfinite(leftMeasuredSignal) ||
            !std::isfinite(leftReferenceSignal) ||
            !std::isfinite(rightMeasuredSignal) ||
            !std::isfinite(rightReferenceSignal) ||
            !std::isfinite(minNormalizedSignal) ||
            leftMeasuredSignal < 0.0f ||
            leftReferenceSignal <= 0.0f ||
            rightMeasuredSignal < 0.0f ||
            rightReferenceSignal <= 0.0f ||
            minNormalizedSignal < 0.0f)
        {
            return false;
        }

        const float leftNormalizedSignal = leftMeasuredSignal / leftReferenceSignal;
        const float rightNormalizedSignal = rightMeasuredSignal / rightReferenceSignal;
        if (!std::isfinite(leftNormalizedSignal) ||
            !std::isfinite(rightNormalizedSignal) ||
            leftNormalizedSignal < minNormalizedSignal ||
            rightNormalizedSignal < minNormalizedSignal)
        {
            return false;
        }

        const float normalizedSignalSum = leftNormalizedSignal + rightNormalizedSignal;
        if (!(normalizedSignalSum > 0.0f) || !std::isfinite(normalizedSignalSum))
        {
            return false;
        }

        balanceError = (leftNormalizedSignal - rightNormalizedSignal) / normalizedSignalSum;
        return std::isfinite(balanceError);
    }
}
