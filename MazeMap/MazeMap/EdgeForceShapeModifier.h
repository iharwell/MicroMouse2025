#pragma once

#include "Defines.h"

namespace MazeMap
{
    class EXPORT EdgeForceShapeModifier
    {
    public:
        static float ComputeUtilizationScale(
            float utilization,
            float bankMemory,
            float frozenStrength,
            bool holdoffActive,
            bool recoveryActive) noexcept;

        static float ComputeCapacityScale(
            float bankMemory,
            float frozenStrength,
            bool holdoffActive,
            bool recoveryActive) noexcept;
    };
}
