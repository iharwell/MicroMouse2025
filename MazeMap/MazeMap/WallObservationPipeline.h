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

    // Classified wall observations remain distinct from raw sensor snapshots because downstream
    // consumers need the normalized range, confidence, and semantic interpretation together.
    enum class ObsClass : uint8_t
    {
        WallLike = 0U,
        PostLike = 1U,
        OpenLike = 2U,
        Ambiguous = 3U
    };

    // One canonical wall observation emitted by the pipeline. It survives as a value type because
    // preprocessing, estimation, and map evidence all consume the same validated observation shape.
    struct WallObs
    {
        bool valid = false;
        float rho = 0.0f;
        float confidence = 0.0f;
        ObsClass cls = ObsClass::Ambiguous;
    };

    inline WallObs MakeWallObs(float rho, float confidence, ObsClass cls) noexcept
    {
        WallObs observation{};
        observation.valid = true;
        observation.rho = rho;
        observation.confidence = confidence;
        observation.cls = cls;
        return observation;
    }

    inline void BuildFrontWallObservations(
        bool frontWallObservationValid,
        bool frontWall,
        bool frontWallUsesFallbackDetection,
        bool frontWallUsesCharacterizationDetection,
        float frontLeftDistanceM,
        float frontRightDistanceM,
        float maxRangeM,
        WallObs& left,
        WallObs& right) noexcept
    {
        left = WallObs{};
        right = WallObs{};
        if (!frontWallObservationValid ||
            !frontWall ||
            !(std::isfinite(frontLeftDistanceM) && (frontLeftDistanceM > 0.0f)) ||
            !(std::isfinite(frontRightDistanceM) && (frontRightDistanceM > 0.0f)))
        {
            return;
        }

        const float confidence =
            frontWallUsesCharacterizationDetection ? 0.90f :
            (frontWallUsesFallbackDetection ? 0.60f : 0.80f);
        left = MakeWallObs((std::clamp)(frontLeftDistanceM, 0.01f, maxRangeM), confidence, ObsClass::WallLike);
        right = MakeWallObs((std::clamp)(frontRightDistanceM, 0.01f, maxRangeM), confidence, ObsClass::WallLike);
    }

    inline WallObs BuildSideWallObservation(
        bool distanceValidForControl,
        bool transitionDetected,
        bool wallObservation,
        float sideDistanceM,
        float maxRangeM) noexcept
    {
        WallObs observation{};
        if (!distanceValidForControl ||
            transitionDetected ||
            !wallObservation ||
            !(std::isfinite(sideDistanceM) && (sideDistanceM > 0.0f)))
        {
            return observation;
        }

        observation = MakeWallObs((std::clamp)(sideDistanceM, 0.01f, maxRangeM), 0.80f, ObsClass::WallLike);
        return observation;
    }
}
