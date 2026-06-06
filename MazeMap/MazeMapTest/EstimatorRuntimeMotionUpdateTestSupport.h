#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct FeedforwardRuntimeScenario final
        {
            bool resetAccepted = false;
            bool predictAccepted = false;
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            StateVector beforePredictState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforePredictCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
            float measuredForwardVelocityMps = std::numeric_limits<float>::quiet_NaN();
            float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN();
            float travelForwardVelocityMps = std::numeric_limits<float>::quiet_NaN();
            float travelYawRateRadps = std::numeric_limits<float>::quiet_NaN();
            float runtimeLeftWheelSpeedBeforePredictRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeRightWheelSpeedBeforePredictRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeLeftWheelSpeedAfterPredictRadps =
                std::numeric_limits<float>::quiet_NaN();
            float runtimeRightWheelSpeedAfterPredictRadps =
                std::numeric_limits<float>::quiet_NaN();
        };

        struct TorqueRefreshScenario final
        {
            bool resetAccepted = false;
            bool firstPredictAccepted = false;
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
