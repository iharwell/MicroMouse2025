#include "pch.h"
#include "UkfRobustUpdatePolicy.h"

#include "RegripRecoveryMonitor.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    namespace
    {
        float Clamp01(float value) noexcept
        {
            if (!std::isfinite(value))
            {
                return 0.0f;
            }

            if (value <= 0.0f)
            {
                return 0.0f;
            }

            if (value >= 1.0f)
            {
                return 1.0f;
            }

            return value;
        }

        float ClampScale(float value) noexcept
        {
            if (!std::isfinite(value))
            {
                return 1.0f;
            }

            return (std::max)(0.25f, (std::min)(value, 8.0f));
        }

    }

    FrozenCycleSchedule UkfRobustUpdatePolicy::BuildFrozenCycleSchedule(
        const GripUtilizationSnapshot& utilization,
        const TransientContactMemoryState& memory,
        const RegripRecoveryState& regrip,
        bool exactStationaryLock,
        bool planarAccelForwardUpdateEnabled,
        bool planarAccelLateralUpdateEnabled,
        bool softOdometryEnabled,
        bool lowSpeedLaunchWindowActive,
        bool inconsistencyWindowActive) noexcept
    {
        FrozenCycleSchedule schedule{};
        schedule.exactStationaryLock = exactStationaryLock;
        schedule.planarAccelForwardUpdateEnabled = planarAccelForwardUpdateEnabled && !exactStationaryLock;
        schedule.planarAccelLateralUpdateEnabled = planarAccelLateralUpdateEnabled && !exactStationaryLock;
        schedule.softOdometryEnabled = softOdometryEnabled && !exactStationaryLock;

        const float leftSeverity =
            (std::max)(Clamp01(utilization.leftBankAnomalySeverity), Clamp01(memory.leftBankMemory));
        const float rightSeverity =
            (std::max)(Clamp01(utilization.rightBankAnomalySeverity), Clamp01(memory.rightBankMemory));
        const float meanMemory = 0.5f * (Clamp01(memory.leftBankMemory) + Clamp01(memory.rightBankMemory));
        const float meanRecovery = 0.5f * (Clamp01(regrip.leftBankRecoveryScore) + Clamp01(regrip.rightBankRecoveryScore));
        const float motionSeverity =
            (std::max)(Clamp01(utilization.longitudinalClosureSeverity),
                (std::max)(Clamp01(utilization.differentialClosureSeverity),
                    (std::max)(Clamp01(utilization.lateralAccelerationSeverity), Clamp01(utilization.yawConsistencySeverity))));

        schedule.closureCovarianceScaleLeft =
            ClampScale(1.0f + (0.75f * leftSeverity) + (0.50f * meanMemory) + (0.50f * meanRecovery));
        schedule.closureCovarianceScaleRight =
            ClampScale(1.0f + (0.75f * rightSeverity) + (0.50f * meanMemory) + (0.50f * meanRecovery));
        schedule.lateralPseudoMeasurementCovarianceScale =
            ClampScale(1.0f + (0.60f * motionSeverity) + (0.25f * meanMemory) + (0.25f * meanRecovery));

        schedule.forwardSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.80f * Clamp01(utilization.longitudinalClosureSeverity)) + (0.30f * meanMemory));
        schedule.lateralSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.80f * Clamp01(utilization.lateralAccelerationSeverity)) + (0.30f * meanMemory));
        schedule.yawRateProcessNoiseScale =
            ClampScale(1.0f + (0.80f * Clamp01(utilization.differentialClosureSeverity)) + (0.30f * Clamp01(utilization.yawConsistencySeverity)));
        schedule.leftWheelSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.90f * leftSeverity) + (0.20f * meanRecovery));
        schedule.rightWheelSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.90f * rightSeverity) + (0.20f * meanRecovery));

        schedule.leftEdgeShapeStrength =
            Clamp01(0.50f * Clamp01(memory.leftBankMemory) + 0.50f * Clamp01(regrip.leftBankRecoveryScore));
        schedule.rightEdgeShapeStrength =
            Clamp01(0.50f * Clamp01(memory.rightBankMemory) + 0.50f * Clamp01(regrip.rightBankRecoveryScore));

        schedule.leftBankHoldoffActive = RegripRecoveryMonitor::IsHoldoffActiveLeft(regrip);
        schedule.rightBankHoldoffActive = RegripRecoveryMonitor::IsHoldoffActiveRight(regrip);
        if (lowSpeedLaunchWindowActive)
        {
            schedule.lateralPseudoMeasurementCovarianceScale =
                (std::min)(schedule.lateralPseudoMeasurementCovarianceScale, 0.50f);
        }
        if (inconsistencyWindowActive || schedule.leftBankHoldoffActive || schedule.rightBankHoldoffActive)
        {
            schedule.lateralPseudoMeasurementCovarianceScale =
                ClampScale((2.0f * schedule.lateralPseudoMeasurementCovarianceScale) + 1.0f);
        }

        const bool severeTwoBankEdge =
            schedule.leftBankHoldoffActive &&
            schedule.rightBankHoldoffActive &&
            (meanMemory >= 0.60f) &&
            (meanRecovery >= 0.60f) &&
            ((std::max)(
                std::isfinite(utilization.leftBankPreProjectionUtilization) ? utilization.leftBankPreProjectionUtilization : 0.0f,
                std::isfinite(utilization.rightBankPreProjectionUtilization) ? utilization.rightBankPreProjectionUtilization : 0.0f) >= 1.0f);
        if (severeTwoBankEdge)
        {
            schedule.lateralPseudoMeasurementCovarianceScale = 64.0f;
        }
        return schedule;
    }
}
