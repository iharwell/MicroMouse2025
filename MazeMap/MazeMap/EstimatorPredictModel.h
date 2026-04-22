#pragma once

#include "Defines.h"
#include "EstimatorGeometry.h"
#include "PlantModel.h"

namespace MazeMap
{
    class EXPORT EstimatorPredictModel
    {
    public:
        using StateVector = VehicleState::StateVector;

        struct PredictInput
        {
            StateVector currentState = StateVector::Zero();
            float leftAppliedBankTorqueNm = 0.0f;
            float rightAppliedBankTorqueNm = 0.0f;
            float fanDutyCycle = 0.80f;
            float dtS = 0.0f;
            const ModelCycleContext* cycleContext = nullptr;
        };

        struct PredictOutput
        {
            StateVector nextState = StateVector::Zero();
            PlantDerivatives evaluatedStep{};
        };

        PlantDerivatives EvaluateStep(
            const PredictInput& input,
            const PlantModel::PreparedParams& prepared) const noexcept;

        PredictOutput Integrate(
            const PredictInput& input,
            const PlantModel::PreparedParams& prepared) const noexcept;

        static StateVector SemiImplicitAdvance(
            const StateVector& currentState,
            const PlantDerivatives& evaluatedStep,
            float dtS) noexcept;
    };
}
