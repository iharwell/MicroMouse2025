#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
    inline float SelectDiagnosticReturnDistanceM(float nominalDistanceM, float measuredDistanceM)
    {
        if (std::isfinite(measuredDistanceM) && measuredDistanceM > 0.0f)
        {
            return measuredDistanceM;
        }

        if (std::isfinite(nominalDistanceM) && nominalDistanceM > 0.0f)
        {
            return nominalDistanceM;
        }

        return 0.0f;
    }

    inline float ComputeDiagnosticCharacterizationTravelLimitM(float boundaryHalfSpanM, float reserveM)
    {
        if (!std::isfinite(boundaryHalfSpanM) || boundaryHalfSpanM <= 0.0f)
        {
            return 0.0f;
        }

        if (!std::isfinite(reserveM) || reserveM < 0.0f)
        {
            reserveM = 0.0f;
        }

        const float limitM = boundaryHalfSpanM - reserveM;
        return (limitM > 0.0f) ? limitM : 0.0f;
    }
}
