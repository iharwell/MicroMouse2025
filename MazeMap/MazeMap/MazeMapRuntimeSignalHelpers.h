#pragma once

#include "Defines.h"
#include "WallDetectionThresholds.h"

namespace MazeMapApp::Internal::Runtime
{
    // Computes the usable reflected-light rise above a calibrated open-space baseline.
    EXPORT float ComputeSignalRiseAboveBaseline(float measuredDifferentialLight, float signalBaseline) noexcept;

    // Applies the shared hysteresis-based wall-signal latch used by both runtime sensor pipelines.
    EXPORT bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized) noexcept;

    // Computes lateral corridor error from whichever side-wall observations are still valid for control.
    EXPORT float ComputeCorridorError(
        float leftDistanceM,
        float rightDistanceM,
        bool leftDistanceValidForControl,
        bool rightDistanceValidForControl,
        float expectedSideWallDistanceM) noexcept;
}
