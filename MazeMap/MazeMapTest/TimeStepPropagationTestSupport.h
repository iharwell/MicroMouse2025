#pragma once

#include "..\MazeMap\PlantModel.h"

#include <cmath>

namespace MazeMap
{
    struct TimeStepPropagationExpectation final
    {
        float xM = 0.0f;
        float yM = 0.0f;
        float yawRad = 0.0f;
        float forwardVelocityMps = 0.0f;
    };

    inline TimeStepPropagationExpectation EvaluateMidpointTimeStepPropagation(
        const VehicleState::StateVector& initialState,
        float lateralDistanceM,
        float forwardDistanceM,
        float deltaYawRad,
        float dtSeconds) noexcept
    {
        TimeStepPropagationExpectation expectation{};
        expectation.xM = initialState(VehicleState::kPx);
        expectation.yM = initialState(VehicleState::kPy);
        expectation.yawRad = initialState(VehicleState::kPsi);
        expectation.forwardVelocityMps = initialState(VehicleState::kU);

        if (!(std::isfinite(lateralDistanceM) &&
            std::isfinite(forwardDistanceM) &&
            std::isfinite(deltaYawRad)))
        {
            return expectation;
        }

        const float translationYawRad =
            VehicleState::NormalizeAngle(initialState(VehicleState::kPsi) + (0.5f * deltaYawRad));
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(translationYawRad, s, c);

        expectation.xM =
            initialState(VehicleState::kPx) +
            (lateralDistanceM * c) +
            (forwardDistanceM * s);
        expectation.yM =
            initialState(VehicleState::kPy) -
            (lateralDistanceM * s) +
            (forwardDistanceM * c);
        expectation.yawRad =
            VehicleState::NormalizeAngle(initialState(VehicleState::kPsi) + deltaYawRad);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            expectation.forwardVelocityMps = forwardDistanceM / dtSeconds;
        }

        return expectation;
    }

    inline VehicleState::StateVector ApplyTimeStepPropagationExpectation(
        const VehicleState::StateVector& baseState,
        const TimeStepPropagationExpectation& expectation) noexcept
    {
        VehicleState::StateVector propagatedState = baseState;
        propagatedState(VehicleState::kPx) = expectation.xM;
        propagatedState(VehicleState::kPy) = expectation.yM;
        propagatedState(VehicleState::kPsi) = expectation.yawRad;
        propagatedState(VehicleState::kU) = expectation.forwardVelocityMps;
        VehicleState::NormalizeStateVector(propagatedState);
        return propagatedState;
    }

    inline VehicleState::StateVector AdvancePlantPredictionState(
        const PlantModel& plant,
        const PlantModel::PreparedParams& prepared,
        const VehicleState::StateVector& state,
        const App::Internal::CommandVector& control,
        float dtSeconds) noexcept
    {
        const VehicleState::StateVector integratedState =
            plant.integrate(state, control, dtSeconds, prepared);
        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)))
        {
            return integratedState;
        }

        if (!(std::isfinite(prepared.wheelRadiusM) && (prepared.wheelRadiusM > 0.0f)))
        {
            return integratedState;
        }

        const float leftDistanceM =
            0.5f *
            (state(VehicleState::kOmegaL) + integratedState(VehicleState::kOmegaL)) *
            prepared.wheelRadiusM *
            dtSeconds;
        const float rightDistanceM =
            0.5f *
            (state(VehicleState::kOmegaR) + integratedState(VehicleState::kOmegaR)) *
            prepared.wheelRadiusM *
            dtSeconds;
        const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
        const float lateralDistanceM =
            0.5f * (state(VehicleState::kV) + integratedState(VehicleState::kV)) * dtSeconds;
        const float deltaYawRad =
            0.5f * (state(VehicleState::kR) + integratedState(VehicleState::kR)) * dtSeconds;

        return ApplyTimeStepPropagationExpectation(
            integratedState,
            EvaluateMidpointTimeStepPropagation(
                state,
                lateralDistanceM,
                forwardDistanceM,
                deltaYawRad,
                dtSeconds));
    }
}


