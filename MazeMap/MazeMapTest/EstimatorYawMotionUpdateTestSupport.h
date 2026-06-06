#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct MovingPredictResidualScenario final
        {
            bool resetAccepted = false;
            bool predictAccepted = false;
            StateVector initialState = StateVector::Zero();
            StateVector predictedState = NanState();
        };

        struct LaunchEncoderSample final
        {
            float dtSeconds = 0.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
        };

        inline constexpr int kLaunchEncoderSampleCount = 10;

        struct LaunchEncoderPredictionScenario final
        {
            bool resetAccepted = false;
            int completedPredictSamples = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
            bool predictionEncoderInputObserved = false;
            SensorSnapshot::EncoderObs finalEncoder = SensorSnapshot{}.EncoderObservation();
            IndexedDifference measuredForwardSpeedDifference{};
            IndexedDifference measuredYawRateDifference{};
            float dumpLeftWheelSpeedRadps = std::numeric_limits<float>::quiet_NaN();
            float dumpRightWheelSpeedRadps = std::numeric_limits<float>::quiet_NaN();
        };

        MovingPredictResidualScenario RunMovingPredictResidualScenario();
        LaunchEncoderPredictionScenario RunLaunchEncoderPredictionScenario();
    }
}
