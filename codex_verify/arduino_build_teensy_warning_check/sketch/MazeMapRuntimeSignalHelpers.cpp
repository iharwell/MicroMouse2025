#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapRuntimeSignalHelpers.cpp"
#include "MazeMapRuntimeSignalHelpers.h"

#include <cmath>

namespace MazeMapApp::Internal::Runtime
{
    float ComputeSignalRiseAboveBaseline(float measuredDifferentialLight, float signalBaseline) noexcept
    {
        if (!std::isfinite(measuredDifferentialLight) ||
            !std::isfinite(signalBaseline))
        {
            return 0.0f;
        }

        return (measuredDifferentialLight > signalBaseline) ?
            (measuredDifferentialLight - signalBaseline) :
            0.0f;
    }

    bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized) noexcept
    {
        filteredSignal = measuredDifferentialLight;
        initialized = true;
        currentState = MazeMap::HysteresisSignalHigh(
            currentState,
            measuredDifferentialLight,
            onMeasuredThreshold,
            offMeasuredThreshold);
        return currentState;
    }

    float ComputeCorridorError(
        float leftDistanceM,
        float rightDistanceM,
        bool leftDistanceValidForControl,
        bool rightDistanceValidForControl,
        float expectedSideWallDistanceM) noexcept
    {
        if (leftDistanceValidForControl && rightDistanceValidForControl)
        {
            return 0.5f * (leftDistanceM - rightDistanceM);
        }
        if (leftDistanceValidForControl)
        {
            return leftDistanceM - expectedSideWallDistanceM;
        }
        if (rightDistanceValidForControl)
        {
            return expectedSideWallDistanceM - rightDistanceM;
        }
        return 0.0f;
    }
}
