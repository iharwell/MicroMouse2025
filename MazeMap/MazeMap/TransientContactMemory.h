#pragma once

#include "Defines.h"
#include "EstimatorGeometry.h"

namespace MazeMap
{
    struct TransientContactMemoryConfig
    {
        float riseRatePerS = 8.0f;
        float decayRatePerS = 1.25f;
        float maximumMemory = 1.0f;
    };

    class EXPORT TransientContactMemory
    {
    public:
        static TransientContactMemoryState Advance(
            const TransientContactMemoryState& previousState,
            const GripUtilizationSnapshot& utilization,
            float dtS,
            const TransientContactMemoryConfig& config = {}) noexcept;
    };
}
