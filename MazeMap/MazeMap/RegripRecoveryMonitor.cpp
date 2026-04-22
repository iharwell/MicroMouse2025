#include "pch.h"
#include "RegripRecoveryMonitor.h"

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

        float ResolveSeverity(float anomalySeverity, float memorySeverity) noexcept
        {
            return Clamp01((std::max)(Clamp01(anomalySeverity), Clamp01(memorySeverity)));
        }
    }

    RegripRecoveryState RegripRecoveryMonitor::Advance(
        const RegripRecoveryState& prior,
        const GripUtilizationSnapshot& utilization,
        const TransientContactMemoryState& memory,
        float dtS,
        const RegripRecoveryTuning& tuning) noexcept
    {
        RegripRecoveryState next = prior;
        const float leftSeverity = ResolveSeverity(utilization.leftBankAnomalySeverity, memory.leftBankMemory);
        const float rightSeverity = ResolveSeverity(utilization.rightBankAnomalySeverity, memory.rightBankMemory);

        const bool leftHoldoffTrigger = leftSeverity >= Clamp01(tuning.holdoffThreshold);
        const bool rightHoldoffTrigger = rightSeverity >= Clamp01(tuning.holdoffThreshold);

        next.leftBankRecoveryScore = leftHoldoffTrigger ?
            BlendTowards(prior.leftBankRecoveryScore, leftSeverity, dtS, tuning.scoreRiseTauS) :
            BlendTowards(prior.leftBankRecoveryScore, 0.0f, dtS, tuning.scoreDecayTauS);
        next.rightBankRecoveryScore = rightHoldoffTrigger ?
            BlendTowards(prior.rightBankRecoveryScore, rightSeverity, dtS, tuning.scoreRiseTauS) :
            BlendTowards(prior.rightBankRecoveryScore, 0.0f, dtS, tuning.scoreDecayTauS);

        if (leftHoldoffTrigger)
        {
            next.leftBankInRecovery = true;
            next.leftBankRecoveryTimeRemainingS = (std::max)(0.0f, tuning.holdoffDwellS);
        }
        else
        {
            next.leftBankRecoveryTimeRemainingS =
                (std::isfinite(dtS) && (dtS > 0.0f)) ?
                (std::max)(0.0f, prior.leftBankRecoveryTimeRemainingS - dtS) :
                Clamp01(prior.leftBankRecoveryTimeRemainingS);
            if ((next.leftBankRecoveryTimeRemainingS <= 0.0f) &&
                (next.leftBankRecoveryScore <= Clamp01(tuning.releaseThreshold)))
            {
                next.leftBankInRecovery = false;
            }
        }

        if (rightHoldoffTrigger)
        {
            next.rightBankInRecovery = true;
            next.rightBankRecoveryTimeRemainingS = (std::max)(0.0f, tuning.holdoffDwellS);
        }
        else
        {
            next.rightBankRecoveryTimeRemainingS =
                (std::isfinite(dtS) && (dtS > 0.0f)) ?
                (std::max)(0.0f, prior.rightBankRecoveryTimeRemainingS - dtS) :
                Clamp01(prior.rightBankRecoveryTimeRemainingS);
            if ((next.rightBankRecoveryTimeRemainingS <= 0.0f) &&
                (next.rightBankRecoveryScore <= Clamp01(tuning.releaseThreshold)))
            {
                next.rightBankInRecovery = false;
            }
        }

        next.leftBankRecoveryScore = Clamp01(next.leftBankRecoveryScore);
        next.rightBankRecoveryScore = Clamp01(next.rightBankRecoveryScore);
        return next;
    }

    bool RegripRecoveryMonitor::IsHoldoffActive(const RegripRecoveryState& state) noexcept
    {
        return IsHoldoffActiveLeft(state) || IsHoldoffActiveRight(state);
    }

    bool RegripRecoveryMonitor::IsHoldoffActiveLeft(const RegripRecoveryState& state) noexcept
    {
        return state.leftBankInRecovery && (state.leftBankRecoveryTimeRemainingS > 0.0f);
    }

    bool RegripRecoveryMonitor::IsHoldoffActiveRight(const RegripRecoveryState& state) noexcept
    {
        return state.rightBankInRecovery && (state.rightBankRecoveryTimeRemainingS > 0.0f);
    }
}
