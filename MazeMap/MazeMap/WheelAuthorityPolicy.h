#pragma once

#include "EstimatorGeometry.h"
#include "VehicleState.h"

namespace MazeMap
{
    struct WheelOnlyMeasurementPrediction
    {
        float leftWheelSpeedRadps = 0.0f;
        float rightWheelSpeedRadps = 0.0f;
        float forwardSpeedMps = 0.0f;
        float yawRateRadps = 0.0f;
    };

    class EXPORT WheelAuthorityPolicy
    {
    public:
        static WheelOnlyMeasurementPrediction PredictWheelOnlyMeasurement(
            const VehicleState::StateVector& state,
            const EstimatorGeometry& geometry) noexcept;

        static float ClosurePseudoMeasurementScale(
            Side side,
            const GripUtilizationSnapshot& utilization,
            const RegripRecoveryState& regrip,
            const FrozenCycleSchedule& schedule) noexcept;

        static float LateralPseudoMeasurementScale(
            const GripUtilizationSnapshot& utilization,
            const RegripRecoveryState& regrip,
            const FrozenCycleSchedule& schedule) noexcept;
    };
}
