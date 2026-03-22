#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
    inline bool ShouldUpdateGyroBiasFromStationarySample(float rawGyroRadps, float maxAbsUpdateRateRadps)
    {
        if (!std::isfinite(rawGyroRadps) || !std::isfinite(maxAbsUpdateRateRadps) || maxAbsUpdateRateRadps <= 0.0f)
        {
            return false;
        }

        return std::fabs(rawGyroRadps) <= maxAbsUpdateRateRadps;
    }
}
