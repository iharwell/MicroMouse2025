#include "pch.h"
#include "WallSensorPreprocessor.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    WallSensorPreprocessor::WallSensorPreprocessor() noexcept
        : WallSensorPreprocessor(Config{})
    {
    }

    WallSensorPreprocessor::WallSensorPreprocessor(
        const Config& config) noexcept
        : _config(config)
    {
    }

    const WallSensorPreprocessor::Config& WallSensorPreprocessor::config() const noexcept
    {
        return _config;
    }

    WallObs WallSensorPreprocessor::process(
        const WallSensor& sensor,
        const Input& input) const noexcept
    {
        WallObs observation{};
        const float differential =
            (std::max)(0.0f, ((sensor.DifferentialLightLevel(input.ledOffLevel, input.ledOnLevel) - _config.zeroOffset) * _config.gain));
        if (!std::isfinite(differential))
        {
            return observation;
        }

        float pseudoRangeM = sensor.DistanceFromDifferentialLight(differential);
        if (_config.calibration.GetCount() > 0U)
        {
            pseudoRangeM = _config.calibration.Apply(pseudoRangeM, _config.calibrationMode);
        }

        if (!(std::isfinite(pseudoRangeM) &&
            (pseudoRangeM >= _config.minPseudoRangeM) &&
            (pseudoRangeM <= _config.maxPseudoRangeM)))
        {
            return observation;
        }

        const float ambientMagnitude = (std::max)(std::fabs(input.ledOffLevel), _config.noiseFloor);
        const float snr = differential / ambientMagnitude;
        const float snrScore = (std::clamp)((snr - 1.0f) / 7.0f, 0.0f, 1.0f);
        const float supportScore = (std::clamp)(input.supportSpanM / _config.wallSupportSpanM, 0.0f, 1.0f);
        const float coherenceScore = (std::clamp)(input.multiSensorCoherence, 0.0f, 1.0f);
        const float incidenceScore = (std::clamp)(input.incidenceCosine, 0.0f, 1.0f);
        const float derivativeScore = (std::clamp)(input.derivativeConsistency, 0.0f, 1.0f);
        const float saturationScore = input.saturated ? 0.15f : 1.0f;

        observation.valid = true;
        observation.rho = pseudoRangeM;
        observation.confidence =
            (0.30f * snrScore) +
            (0.20f * supportScore) +
            (0.20f * coherenceScore) +
            (0.15f * incidenceScore) +
            (0.10f * derivativeScore) +
            (0.05f * saturationScore);
        observation.confidence = (std::clamp)(observation.confidence, 0.0f, 1.0f);

        if (observation.confidence < _config.minConfidence)
        {
            observation.cls = ObsClass::Ambiguous;
        }
        else if (pseudoRangeM >= _config.openLikeRangeM)
        {
            observation.cls = ObsClass::OpenLike;
        }
        else if (input.supportSpanM <= _config.postSupportSpanM)
        {
            observation.cls = ObsClass::PostLike;
        }
        else if (pseudoRangeM <= _config.wallLikeRangeM)
        {
            observation.cls = ObsClass::WallLike;
        }
        else
        {
            observation.cls = ObsClass::Ambiguous;
        }

        return observation;
    }
}
