#pragma once

#include "EstimatorEncoderMotionUpdateTestSupport.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct FeedforwardRuntimeScenario final
        {
            bool resetAccepted = false;
            bool predictAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = false;
            bool updateRecordedAccepted = false;
            EncoderObs encoder{};
            StateVector beforeEncoderState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforeEncoderCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
            EncoderPairExpectation expectation{};
            float actualNis = std::numeric_limits<float>::quiet_NaN();
            float runtimeLeftWheelSpeedBeforeUpdateRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeRightWheelSpeedBeforeUpdateRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeLeftWheelSpeedAfterUpdateRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeRightWheelSpeedAfterUpdateRadps =
                std::numeric_limits<float>::quiet_NaN();
        };

        struct TorqueRefreshScenario final
        {
            bool resetAccepted = false;
            bool firstPredictAccepted = false;
            bool firstEncoderAccepted = false;
            bool firstControlResetAccepted = false;
            bool secondControlResetAccepted = false;
            bool firstControlPredictAccepted = false;
            bool secondControlPredictAccepted = false;
            float responseDelta = std::numeric_limits<float>::quiet_NaN();
        };

        FeedforwardRuntimeScenario RunFeedforwardRuntimeScenario();
        TorqueRefreshScenario RunTorqueRefreshScenario();
    }
}
