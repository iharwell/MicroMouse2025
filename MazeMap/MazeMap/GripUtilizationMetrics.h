#pragma once

#include "EstimatorGeometry.h"
#include "VehicleState.h"

namespace MazeMap
{
    struct PlantPreparedParams;

    struct GripUtilizationInputs
    {
        float fanDutyCycle = 0.80f;
        float longitudinalClosureSeverity = 0.0f;
        float differentialClosureSeverity = 0.0f;
        float lateralAccelerationSeverity = 0.0f;
        float yawConsistencySeverity = 0.0f;

        float leftBankAnomalySeverity = 0.0f;
        float rightBankAnomalySeverity = 0.0f;
    };

    class EXPORT GripUtilizationMetrics
    {
    public:
        static GripUtilizationSnapshot Compute(
            const VehicleState::StateVector& currentState,
            const AppliedTorqueEstimate& appliedTorque,
            const PlantPreparedParams& params,
            const GripUtilizationInputs& inputs = GripUtilizationInputs{}) noexcept;
    };
}
