#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct WallUpdateScenario final
        {
            bool primaryPredictionHit = false;
            bool secondaryPredictionHit = false;
            bool resetAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = false;
            bool updateRecordedAccepted = false;
            StateVector beforeState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforeCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
        };

        WallUpdateScenario RunFrontWallScenario();
        WallUpdateScenario RunLeftWallScenario();
    }
}
