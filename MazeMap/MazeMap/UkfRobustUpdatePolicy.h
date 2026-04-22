#pragma once

#include "EstimatorGeometry.h"

namespace MazeMap
{
    class EXPORT UkfRobustUpdatePolicy
    {
    public:
        static FrozenCycleSchedule BuildFrozenCycleSchedule(
            const GripUtilizationSnapshot& utilization,
            const TransientContactMemoryState& memory,
            const RegripRecoveryState& regrip,
            bool exactStationaryLock,
            bool planarAccelForwardUpdateEnabled,
            bool planarAccelLateralUpdateEnabled,
            bool softOdometryEnabled,
            bool lowSpeedLaunchWindowActive = false,
            bool inconsistencyWindowActive = false) noexcept;
    };
}
