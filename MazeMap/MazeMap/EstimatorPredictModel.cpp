#include "pch.h"
#include "EstimatorPredictModel.h"

#include <cmath>

namespace MazeMap
{
    namespace
    {
        constexpr float kStationaryBodyDecayTauS = 0.075f;
        constexpr float kStationaryWheelDecayTauS = 0.050f;

        float ResolveStationaryDecayAlpha(float dtS, float tauS) noexcept
        {
            if (!(std::isfinite(dtS) && (dtS > 0.0f) && std::isfinite(tauS) && (tauS > 0.0f)))
            {
                return 0.0f;
            }

            return (std::clamp)(std::exp(-dtS / tauS), 0.0f, 1.0f);
        }
    }

    EstimatorPredictModel::PredictOutput EstimatorPredictModel::Integrate(
        const PredictInput& input,
        const PlantModel::PreparedParams& prepared) const noexcept
    {
        PredictOutput output{};
        output.nextState = input.currentState;

        if (!(std::isfinite(input.dtS) && (input.dtS > 0.0f)))
        {
            return output;
        }

        const bool exactStationaryLock =
            (input.cycleContext != nullptr) && input.cycleContext->schedule.exactStationaryLock;
        if (exactStationaryLock)
        {
            const float bodyDecayAlpha = ResolveStationaryDecayAlpha(input.dtS, kStationaryBodyDecayTauS);
            const float wheelDecayAlpha = ResolveStationaryDecayAlpha(input.dtS, kStationaryWheelDecayTauS);
            output.nextState(VehicleState::kPx) = input.currentState(VehicleState::kPx);
            output.nextState(VehicleState::kPy) = input.currentState(VehicleState::kPy);
            output.nextState(VehicleState::kPsi) = VehicleState::NormalizeAngle(input.currentState(VehicleState::kPsi));
            output.nextState(VehicleState::kU) = bodyDecayAlpha * input.currentState(VehicleState::kU);
            output.nextState(VehicleState::kV) = bodyDecayAlpha * input.currentState(VehicleState::kV);
            output.nextState(VehicleState::kR) = bodyDecayAlpha * input.currentState(VehicleState::kR);
            output.nextState(VehicleState::kOmegaL) = wheelDecayAlpha * input.currentState(VehicleState::kOmegaL);
            output.nextState(VehicleState::kOmegaR) = wheelDecayAlpha * input.currentState(VehicleState::kOmegaR);
            output.nextState(VehicleState::kBgz) = input.currentState(VehicleState::kBgz);
            output.evaluatedStep.stateDot =
                (output.nextState - input.currentState) / input.dtS;
            output.evaluatedStep.regime = MotionRegime::StoppedHold;
            return output;
        }

        output.evaluatedStep = EvaluateStep(input, prepared);
        output.nextState = SemiImplicitAdvance(input.currentState, output.evaluatedStep, input.dtS);
        return output;
    }

    PlantDerivatives EstimatorPredictModel::EvaluateStep(
        const PredictInput& input,
        const PlantModel::PreparedParams& prepared) const noexcept
    {
        const PlantModel plantModel{};
        return plantModel.forwardStepFromAppliedBankTorques(
            input.currentState,
            input.leftAppliedBankTorqueNm,
            input.rightAppliedBankTorqueNm,
            prepared,
            input.fanDutyCycle,
            input.cycleContext);
    }

    EstimatorPredictModel::StateVector EstimatorPredictModel::SemiImplicitAdvance(
        const StateVector& currentState,
        const PlantDerivatives& evaluatedStep,
        float dtS) noexcept
    {
        StateVector nextState = currentState;
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return nextState;
        }

        nextState(VehicleState::kOmegaL) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaL);
        nextState(VehicleState::kOmegaR) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaR);

        nextState(VehicleState::kU) += dtS * evaluatedStep.stateDot(VehicleState::kU);
        nextState(VehicleState::kV) += dtS * evaluatedStep.stateDot(VehicleState::kV);
        nextState(VehicleState::kR) += dtS * evaluatedStep.stateDot(VehicleState::kR);

        nextState(VehicleState::kPsi) =
            VehicleState::NormalizeAngle(
                currentState(VehicleState::kPsi) + (dtS * nextState(VehicleState::kR)));

        float sineHeading = 0.0f;
        float cosineHeading = 0.0f;
        sin_cosf(nextState(VehicleState::kPsi), sineHeading, cosineHeading);
        const float worldRightVelocityMps =
            (nextState(VehicleState::kV) * cosineHeading) +
            (nextState(VehicleState::kU) * sineHeading);
        const float worldForwardVelocityMps =
            (-nextState(VehicleState::kV) * sineHeading) +
            (nextState(VehicleState::kU) * cosineHeading);
        nextState(VehicleState::kPx) += dtS * worldRightVelocityMps;
        nextState(VehicleState::kPy) += dtS * worldForwardVelocityMps;

        nextState(VehicleState::kBgz) += dtS * evaluatedStep.stateDot(VehicleState::kBgz);
        nextState(VehicleState::kPsi) = VehicleState::NormalizeAngle(nextState(VehicleState::kPsi));
        return nextState;
    }
}
