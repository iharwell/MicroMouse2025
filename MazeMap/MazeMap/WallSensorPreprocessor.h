#pragma once
// Declares the wall-sensor preprocessing stage that converts raw LED on/off samples into estimator observations.

#include "WallObservationPipeline.h"
#include "WallSensor.h"

namespace MazeMap
{
    // Converts raw wall-sensor light readings into typed estimator wall observations.
    class EXPORT WallSensorPreprocessor
    {
    public:
        WallSensorPreprocessor() noexcept;

        WallObs process(
            const WallSensor& sensor,
            float ledOffLevel,
            float ledOnLevel,
            float measuredRangeM,
            float supportSpanM = 0.05f,
            float multiSensorCoherence = 1.0f,
            float incidenceCosine = 1.0f,
            float derivativeConsistency = 1.0f) const noexcept;

    private:
        float _zeroOffset = 0.0f;
        float _gain = 1.0f;
        float _wallLikeRangeM = 0.11f;
        float _openLikeRangeM = 0.16f;
        float _postSupportSpanM = 0.020f;
        float _wallSupportSpanM = 0.040f;
        float _minConfidence = 0.25f;
        float _noiseFloor = 0.02f;
    };
}
