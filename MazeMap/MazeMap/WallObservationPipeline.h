#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MazeMap
{
    enum class WallSampleClassification : uint8_t
    {
        Unknown = 0U,
        WallHit,
        WallMiss
    };

    enum class WallMeasurementRejectReason : uint8_t
    {
        None = 0U,
        InvalidRange,
        OutsideDecisionWindow,
        TransitionAmbiguous,
        CharacterizationUnavailable
    };

    struct WallEvidenceConfig
    {
        float hitWeight = 0.55f;
        float missWeight = 0.55f;
        float transitionMissWeight = 0.35f;
        float unknownDecay = 0.08f;
        float commitThreshold = 1.00f;
    };

    class WallDecisionAccumulator
    {
    public:
        void Reset() noexcept
        {
            _score = 0.0f;
            _hitCount = 0U;
            _missCount = 0U;
            _unknownCount = 0U;
        }

        void Update(
            WallSampleClassification classification,
            float hitWeight,
            float missWeight,
            float unknownDecay) noexcept
        {
            switch (classification)
            {
            case WallSampleClassification::WallHit:
                if (std::isfinite(hitWeight) && hitWeight > 0.0f)
                {
                    _score += hitWeight;
                }
                ++_hitCount;
                break;
            case WallSampleClassification::WallMiss:
                if (std::isfinite(missWeight) && missWeight > 0.0f)
                {
                    _score -= missWeight;
                }
                ++_missCount;
                break;
            case WallSampleClassification::Unknown:
            default:
                if (std::isfinite(unknownDecay) && unknownDecay > 0.0f)
                {
                    if (_score > 0.0f)
                    {
                        _score = (std::max)(0.0f, _score - unknownDecay);
                    }
                    else if (_score < 0.0f)
                    {
                        _score = (std::min)(0.0f, _score + unknownDecay);
                    }
                }
                ++_unknownCount;
                break;
            }
        }

        void InjectMissImpulse(float missWeight) noexcept
        {
            if (std::isfinite(missWeight) && missWeight > 0.0f)
            {
                _score -= missWeight;
            }
        }

        WallSampleClassification FinalClassification(float commitThreshold) const noexcept
        {
            if (!(std::isfinite(commitThreshold) && commitThreshold > 0.0f))
            {
                return WallSampleClassification::Unknown;
            }

            if (_score >= commitThreshold)
            {
                return WallSampleClassification::WallHit;
            }
            if (_score <= -commitThreshold)
            {
                return WallSampleClassification::WallMiss;
            }

            return WallSampleClassification::Unknown;
        }

        float Score() const noexcept { return _score; }
        uint8_t HitCount() const noexcept { return _hitCount; }
        uint8_t MissCount() const noexcept { return _missCount; }
        uint8_t UnknownCount() const noexcept { return _unknownCount; }

    private:
        float _score = 0.0f;
        uint8_t _hitCount = 0U;
        uint8_t _missCount = 0U;
        uint8_t _unknownCount = 0U;
    };

    inline const char* WallMeasurementRejectReasonName(WallMeasurementRejectReason reason) noexcept
    {
        switch (reason)
        {
        case WallMeasurementRejectReason::None:
            return "none";
        case WallMeasurementRejectReason::InvalidRange:
            return "invalid_range";
        case WallMeasurementRejectReason::OutsideDecisionWindow:
            return "outside_window";
        case WallMeasurementRejectReason::TransitionAmbiguous:
            return "transition_ambiguous";
        case WallMeasurementRejectReason::CharacterizationUnavailable:
            return "characterization_unavailable";
        default:
            return "unknown";
        }
    }
}
