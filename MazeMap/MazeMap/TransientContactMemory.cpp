#include "pch.h"
#include "TransientContactMemory.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    namespace
    {
        float Clamp01(float value) noexcept
        {
            if (!std::isfinite(value))
            {
                return 0.0f;
            }

            if (value <= 0.0f)
            {
                return 0.0f;
            }

            if (value >= 1.0f)
            {
                return 1.0f;
            }

            return value;
        }

        float BlendTowards(float prior, float target, float dtS, float tauS) noexcept
        {
            if (!(std::isfinite(dtS) && (dtS > 0.0f)) || !(std::isfinite(tauS) && (tauS > 0.0f)))
            {
                return Clamp01(prior);
            }

            const float alpha = Clamp01(dtS / tauS);
            return Clamp01(prior + ((target - prior) * alpha));
        }
    }

    TransientContactMemoryState TransientContactMemory::Advance(
        const TransientContactMemoryState& previousState,
        const GripUtilizationSnapshot& utilization,
        float dtS,
        const TransientContactMemoryConfig& config) noexcept
    {
        TransientContactMemoryState next{};
        const float leftInput =
            (std::max)(
                Clamp01(utilization.leftBankAnomalySeverity),
                Clamp01(utilization.leftBankPreProjectionUtilization));
        const float rightInput =
            (std::max)(
                Clamp01(utilization.rightBankAnomalySeverity),
                Clamp01(utilization.rightBankPreProjectionUtilization));

        const float riseTauS =
            (std::isfinite(config.riseRatePerS) && (config.riseRatePerS > 0.0f)) ?
            (1.0f / config.riseRatePerS) :
            0.125f;
        const float decayTauS =
            (std::isfinite(config.decayRatePerS) && (config.decayRatePerS > 0.0f)) ?
            (1.0f / config.decayRatePerS) :
            0.8f;

        next.leftBankMemory =
            (leftInput >= previousState.leftBankMemory) ?
            BlendTowards(previousState.leftBankMemory, leftInput, dtS, riseTauS) :
            BlendTowards(previousState.leftBankMemory, leftInput, dtS, decayTauS);
        next.rightBankMemory =
            (rightInput >= previousState.rightBankMemory) ?
            BlendTowards(previousState.rightBankMemory, rightInput, dtS, riseTauS) :
            BlendTowards(previousState.rightBankMemory, rightInput, dtS, decayTauS);

        const float maximumMemory = Clamp01(config.maximumMemory);
        next.leftBankMemory = (std::min)(next.leftBankMemory, maximumMemory);
        next.rightBankMemory = (std::min)(next.rightBankMemory, maximumMemory);
        return next;
    }
}
