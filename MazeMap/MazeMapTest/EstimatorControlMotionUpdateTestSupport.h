#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct StationarySplitCommandPrediction final
        {
            StateVector state = NanState();
            int completedSteps = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
        };

        struct ActiveCommandStorageScenario final
        {
            bool resetPoseAccepted = false;
            bool firstPredictAccepted = false;
            bool secondPredictAccepted = false;
            CommandVector commandAfterReset{};
            CommandVector activeCommand{};
            CommandVector commandAfterFirstPredict{};
            CommandVector nextCommand{};
            CommandVector commandAfterSecondPredict{};
        };

        struct IterativeMotionScenario final
        {
            int completedSteps = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
            StateVector state = NanState();
        };

        struct SplitDrivePredictScenario final
        {
            bool resetAccepted = false;
            int completedSteps = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
            StateVector initialState = StateVector::Zero();
            StateVector state = NanState();
        };

        struct ControlDirectionScenario final
        {
            int completedStationarySteps = 0;
            int completedTrackingSteps = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
            StateVector state = NanState();
        };

        ActiveCommandStorageScenario RunActiveCommandStorageScenario();
        IterativeMotionScenario RunOpposedControlScenario();
        IterativeMotionScenario RunUnopposedControlScenario();
        SplitDrivePredictScenario RunSplitDrivePredictScenario();
        StationarySplitCommandPrediction PredictStationarySplitCommandStateAfterPivotPredictSequence();
        ControlDirectionScenario RunControlDirectionScenario(bool stationaryFirst);
    }
}
