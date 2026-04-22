#pragma once

#include "EstimatorGeometry.h"
#include "TransientContactMemory.h"

namespace MazeMap
{
    struct RegripRecoveryTuning
    {
        float holdoffThreshold = 0.700f;
        float releaseThreshold = 0.350f;
        float holdoffDwellS = 0.080f;
        float scoreRiseTauS = 0.050f;
        float scoreDecayTauS = 0.400f;
    };

    class EXPORT RegripRecoveryMonitor
    {
    public:
        static RegripRecoveryState Advance(
            const RegripRecoveryState& prior,
            const GripUtilizationSnapshot& utilization,
            const TransientContactMemoryState& memory,
            float dtS,
            const RegripRecoveryTuning& tuning = RegripRecoveryTuning{}) noexcept;

        static bool IsHoldoffActive(const RegripRecoveryState& state) noexcept;
        static bool IsHoldoffActiveLeft(const RegripRecoveryState& state) noexcept;
        static bool IsHoldoffActiveRight(const RegripRecoveryState& state) noexcept;
    };
}
