#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MazeMap
{
    inline constexpr float kDefaultWallObservationMaxRangeM = 0.30f;

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
    class WallObs final
    {
    public:
        WallObs() noexcept = default;
        WallObs(
            float rho,
            float confidence,
            ObsClass cls,
            float measurementNoiseSigmaM = 0.0f) noexcept
            : _valid(true),
            _rho(rho),
            _confidence(confidence),
            _measurementNoiseSigmaM(measurementNoiseSigmaM),
            _class(cls)
        {
        }

        bool IsValid() const noexcept { return _valid; }
        float Rho() const noexcept { return _rho; }
        float Confidence() const noexcept { return _confidence; }
        float MeasurementNoiseSigmaM() const noexcept { return _measurementNoiseSigmaM; }
        ObsClass Class() const noexcept { return _class; }

        static void BuildFrontWallObservations(
            bool frontWallObservationValid,
            bool frontWall,
            bool frontWallUsesFallbackDetection,
            bool frontWallUsesCharacterizationDetection,
            float frontLeftDistanceM,
            float frontRightDistanceM,
            float maxRangeM,
            WallObs& left,
            WallObs& right,
            float frontLeftMeasurementNoiseSigmaM = 0.0f,
            float frontRightMeasurementNoiseSigmaM = 0.0f,
            float frontLeftConfidence = -1.0f,
            float frontRightConfidence = -1.0f,
            ObsClass frontLeftClass = ObsClass::WallLike,
            ObsClass frontRightClass = ObsClass::WallLike) noexcept;

        static WallObs BuildSideWallObservation(
            bool distanceValidForControl,
            bool transitionDetected,
            bool wallObservation,
            float sideDistanceM,
            float maxRangeM,
            float measurementNoiseSigmaM = 0.0f,
            float confidence = 0.80f,
            ObsClass observationClass = ObsClass::WallLike) noexcept;

    private:
        bool _valid = false;
        float _rho = 0.0f;
        float _confidence = 0.0f;
        float _measurementNoiseSigmaM = 0.0f;
        ObsClass _class = ObsClass::Ambiguous;
    };

    inline void WallObs::BuildFrontWallObservations(
        const bool frontWallObservationValid,
        const bool frontWall,
        const bool frontWallUsesFallbackDetection,
        const bool frontWallUsesCharacterizationDetection,
        const float frontLeftDistanceM,
        const float frontRightDistanceM,
        const float maxRangeM,
        WallObs& left,
        WallObs& right,
        const float frontLeftMeasurementNoiseSigmaM,
        const float frontRightMeasurementNoiseSigmaM,
        const float frontLeftConfidence,
        const float frontRightConfidence,
        const ObsClass frontLeftClass,
        const ObsClass frontRightClass) noexcept
    {
        (void)maxRangeM;
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
        const float leftConfidence =
            std::isfinite(frontLeftConfidence) && (frontLeftConfidence >= 0.0f) ?
            (std::clamp)(frontLeftConfidence, 0.0f, 1.0f) :
            confidence;
        const float rightConfidence =
            std::isfinite(frontRightConfidence) && (frontRightConfidence >= 0.0f) ?
            (std::clamp)(frontRightConfidence, 0.0f, 1.0f) :
            confidence;
        left = WallObs(
            frontLeftDistanceM,
            leftConfidence,
            frontLeftClass,
            frontLeftMeasurementNoiseSigmaM);
        right = WallObs(
            frontRightDistanceM,
            rightConfidence,
            frontRightClass,
            frontRightMeasurementNoiseSigmaM);
    }

    inline WallObs WallObs::BuildSideWallObservation(
        const bool distanceValidForControl,
        const bool transitionDetected,
        const bool wallObservation,
        const float sideDistanceM,
        const float maxRangeM,
        const float measurementNoiseSigmaM,
        const float confidence,
        const ObsClass observationClass) noexcept
    {
        (void)maxRangeM;
        WallObs observation{};
        if (!distanceValidForControl ||
            transitionDetected ||
            (!wallObservation && (observationClass != ObsClass::OpenLike)) ||
            !(std::isfinite(sideDistanceM) && (sideDistanceM > 0.0f)))
        {
            return observation;
        }

        const float resolvedConfidence =
            std::isfinite(confidence) ?
            (std::clamp)(confidence, 0.0f, 1.0f) :
            0.80f;
        observation = WallObs(
            sideDistanceM,
            resolvedConfidence,
            observationClass,
            measurementNoiseSigmaM);
        return observation;
    }
}
