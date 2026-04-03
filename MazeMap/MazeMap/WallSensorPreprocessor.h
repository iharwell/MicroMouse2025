#pragma once
// Declares the wall-sensor preprocessing stage that converts raw LED on/off samples into UKF observations.

#include "VehicleState.h"
#include "WallSensor.h"
#include "WallSensorCalibration.h"

namespace MazeMap
{
    // Raw wall-sensor evidence collected around a single on/off measurement pair.
    struct WallPreprocessorInput
    {
        float ledOffLevel = 0.0f;
        float ledOnLevel = 0.0f;
        float supportSpanM = 0.05f;
        float multiSensorCoherence = 1.0f;
        float incidenceCosine = 1.0f;
        float derivativeConsistency = 1.0f;
        bool saturated = false;
    };

    // Thresholds and calibration data for converting a raw wall reading into a UKF wall observation.
    struct WallSensorPreprocessorConfig
    {
        float zeroOffset = 0.0f;
        float gain = 1.0f;
        float minPseudoRangeM = 0.01f;
        float maxPseudoRangeM = 0.25f;
        float wallLikeRangeM = 0.11f;
        float openLikeRangeM = 0.16f;
        float postSupportSpanM = 0.020f;
        float wallSupportSpanM = 0.040f;
        float minConfidence = 0.25f;
        float noiseFloor = 0.02f;
        WallSensorCalibrationMode calibrationMode = WallSensorCalibrationMode::DistanceOffset;
        WallSensorCalibrationCurve calibration{};
    };

    // Converts raw wall-sensor light readings into typed UKF wall observations.
    class EXPORT WallSensorPreprocessor
    {
    public:
        explicit WallSensorPreprocessor(
            const WallSensorPreprocessorConfig& config = WallSensorPreprocessorConfig{}) noexcept;

        const WallSensorPreprocessorConfig& config() const noexcept;

        WallObs process(
            const WallSensor& sensor,
            const WallPreprocessorInput& input) const noexcept;

    private:
        WallSensorPreprocessorConfig _config;
    };
}
