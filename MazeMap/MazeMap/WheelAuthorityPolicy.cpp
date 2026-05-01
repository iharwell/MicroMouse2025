#include "pch.h"
#include "WheelAuthorityPolicy.h"

#include "PlantModel.h"

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

        float SideRecoveryPenalty(Side side, const RegripRecoveryState& regrip) noexcept
        {
            const float leftPenalty = regrip.leftBankInRecovery ? (0.5f + 0.5f * Clamp01(regrip.leftBankRecoveryScore)) : 0.0f;
            const float rightPenalty = regrip.rightBankInRecovery ? (0.5f + 0.5f * Clamp01(regrip.rightBankRecoveryScore)) : 0.0f;
            return (side == Side::Left) ? leftPenalty : rightPenalty;
        }
    }

    WheelOnlyMeasurementPrediction WheelAuthorityPolicy::PredictWheelOnlyMeasurement(
        const VehicleState::StateVector& state,
        const EstimatorGeometry& geometry) noexcept
    {
        WheelOnlyMeasurementPrediction prediction{};
        const float wheelRadiusM =
            (std::isfinite(geometry.effectiveWheelRadiusM) && (geometry.effectiveWheelRadiusM > 0.0f)) ?
            geometry.effectiveWheelRadiusM :
            0.0f;
        const float trackWidthM =
            (std::isfinite(geometry.effectiveTrackWidthM) && (geometry.effectiveTrackWidthM > 0.0f)) ?
            geometry.effectiveTrackWidthM :
            0.0f;
        const float forwardSpeedMps = std::isfinite(state(VehicleState::kU)) ? state(VehicleState::kU) : 0.0f;
        const float yawRateRadps = std::isfinite(state(VehicleState::kR)) ? state(VehicleState::kR) : 0.0f;

        prediction.forwardSpeedMps = forwardSpeedMps;
        prediction.yawRateRadps = yawRateRadps;
        prediction.leftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardSpeedMps + (0.5f * trackWidthM * yawRateRadps)) / wheelRadiusM) :
            0.0f;
        prediction.rightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardSpeedMps - (0.5f * trackWidthM * yawRateRadps)) / wheelRadiusM) :
            0.0f;
        return prediction;
    }

    float WheelAuthorityPolicy::ClosurePseudoMeasurementScale(
        Side side,
        const GripUtilizationSnapshot& utilization,
        const RegripRecoveryState& regrip,
        const FrozenCycleSchedule& schedule) noexcept
    {
        const float anomalySeverity =
            (side == Side::Left) ?
            Clamp01(utilization.leftBankAnomalySeverity) :
            Clamp01(utilization.rightBankAnomalySeverity);
        const float memoryPenalty =
            (side == Side::Left) ?
            Clamp01(schedule.leftEdgeShapeStrength) :
            Clamp01(schedule.rightEdgeShapeStrength);
        const float recoveryPenalty = SideRecoveryPenalty(side, regrip);
        const float holdoffPenalty =
            (side == Side::Left) ?
            (schedule.leftBankHoldoffActive ? 1.0f : 0.0f) :
            (schedule.rightBankHoldoffActive ? 1.0f : 0.0f);

        return 1.0f + (0.75f * anomalySeverity) + (0.50f * memoryPenalty) + recoveryPenalty + holdoffPenalty;
    }

    float WheelAuthorityPolicy::LateralPseudoMeasurementScale(
        const GripUtilizationSnapshot& utilization,
        const RegripRecoveryState& regrip,
        const FrozenCycleSchedule& schedule) noexcept
    {
        const float utilizationSeverity =
            (std::max)(
                Clamp01(utilization.lateralAccelerationSeverity),
                Clamp01(utilization.yawConsistencySeverity));
        const float memorySeverity =
            0.5f * (Clamp01(schedule.leftEdgeShapeStrength) + Clamp01(schedule.rightEdgeShapeStrength));
        const float recoverySeverity =
            0.5f * (Clamp01(regrip.leftBankRecoveryScore) + Clamp01(regrip.rightBankRecoveryScore));
        return 1.0f + (0.75f * utilizationSeverity) + (0.35f * memorySeverity) + (0.35f * recoverySeverity);
    }
}
