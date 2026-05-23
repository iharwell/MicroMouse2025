#include "pch.h"
#include "WallSensorPreprocessor.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    WallSensorPreprocessor::WallSensorPreprocessor() noexcept = default;

    WallObs WallSensorPreprocessor::process(
        const WallSensor& sensor,
        const float ledOffLevel,
        const float ledOnLevel,
        const float measuredRangeM,
        const float supportSpanM,
        const float multiSensorCoherence,
        const float incidenceCosine,
        const float derivativeConsistency) const noexcept
    {
        const float differential =
            (std::max)(
                0.0f,
                ((sensor.DifferentialLightLevel(ledOffLevel, ledOnLevel) -
                    _zeroOffset) *
                    _gain));
        if (!std::isfinite(differential))
        {
            return WallObs{};
        }

        const float pseudoRangeM = sensor.DistanceFromDifferentialLight(differential);
        float lowNoiseRangeM = pseudoRangeM;
        float highNoiseRangeM = pseudoRangeM;
        const float differentialNoise =
            (std::isfinite(_noiseFloor) && (_noiseFloor > 0.0f)) ?
            _noiseFloor :
            0.0f;
        if (differentialNoise > 0.0f)
        {
            lowNoiseRangeM =
                sensor.DistanceFromDifferentialLight(
                    (std::max)(0.0f, differential - differentialNoise));
            highNoiseRangeM =
                sensor.DistanceFromDifferentialLight(
                    differential + differentialNoise);
        }

        const float observationRangeM =
            std::isfinite(measuredRangeM) ?
            measuredRangeM :
            pseudoRangeM;
        if (!(std::isfinite(observationRangeM) && (observationRangeM > 0.0f)))
        {
            return WallObs{};
        }

        const float ambientMagnitude = (std::max)(std::fabs(ledOffLevel), _noiseFloor);
        const float snr = differential / ambientMagnitude;
        const float snrScore = (std::clamp)((snr - 1.0f) / 7.0f, 0.0f, 1.0f);
        const float supportScore =
            (std::clamp)(supportSpanM / _wallSupportSpanM, 0.0f, 1.0f);
        const float coherenceScore = (std::clamp)(multiSensorCoherence, 0.0f, 1.0f);
        const float incidenceScore = (std::clamp)(incidenceCosine, 0.0f, 1.0f);
        const float derivativeScore = (std::clamp)(derivativeConsistency, 0.0f, 1.0f);

        const float measurementNoiseSigmaM =
            (std::max)(
                0.0f,
                (std::max)(
                    std::fabs(lowNoiseRangeM - pseudoRangeM),
                    std::fabs(highNoiseRangeM - pseudoRangeM)));
        float confidence =
            (0.30f * snrScore) +
            (0.20f * supportScore) +
            (0.20f * coherenceScore) +
            (0.15f * incidenceScore) +
            (0.15f * derivativeScore);
        confidence =
            std::isfinite(confidence) ?
            (std::clamp)(confidence, 0.0f, 1.0f) :
            0.0f;

        ObsClass observationClass = ObsClass::Ambiguous;
        if (confidence < _minConfidence)
        {
            observationClass = ObsClass::Ambiguous;
        }
        else if (observationRangeM >= _openLikeRangeM)
        {
            observationClass = ObsClass::OpenLike;
        }
        else if (supportSpanM <= _postSupportSpanM)
        {
            observationClass = ObsClass::PostLike;
        }
        else if (observationRangeM <= _wallLikeRangeM)
        {
            observationClass = ObsClass::WallLike;
        }

        return WallObs(
            observationRangeM,
            confidence,
            observationClass,
            measurementNoiseSigmaM);
    }
}
