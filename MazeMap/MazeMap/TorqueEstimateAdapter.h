#pragma once

#include "EstimatorGeometry.h"
#include "VehicleState.h"

namespace MazeMap
{
    class PlantModel;
    struct PlantPreparedParams;

    class EXPORT TorqueEstimateAdapter
    {
    public:
        static AppliedTorqueEstimate Estimate(
            const PlantModel& plant,
            const VehicleState::StateVector& currentState,
            const ControlInput& control,
            const PlantPreparedParams& params,
            float batteryVoltageV = 0.0f) noexcept;
    };
}
