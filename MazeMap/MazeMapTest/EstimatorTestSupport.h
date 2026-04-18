#pragma once

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\VehicleState.h"
#include "..\MazeMap\WallGeometryModel.h"

namespace MazeMap
{
    inline VehicleState::StateVector BuildUkfState(
        float xM,
        float yM,
        float yawRad,
        float forwardVelocityMps,
        float lateralVelocityMps,
        float yawRateRadps,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps,
        float gyroBiasRadps = 0.0f)
    {
        VehicleState::StateVector state = VehicleState::StateVector::Zero();
        state(VehicleState::kPx) = xM;
        state(VehicleState::kPy) = yM;
        state(VehicleState::kPsi) = yawRad;
        state(VehicleState::kU) = forwardVelocityMps;
        state(VehicleState::kV) = lateralVelocityMps;
        state(VehicleState::kR) = yawRateRadps;
        state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
        state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
        state(VehicleState::kBgz) = gyroBiasRadps;
        VehicleState::NormalizeStateVector(state);
        return state;
    }

    inline VehicleState::StateMatrix BuildUkfCovariance(
        float positionSigmaM = 0.01f,
        float headingSigmaRad = 0.03f,
        float forwardVelocitySigmaMps = 0.05f,
        float lateralVelocitySigmaMps = 0.05f,
        float yawRateSigmaRadps = 0.10f,
        float wheelSigmaRadps = 0.30f,
        float gyroBiasSigmaRadps = 0.03f)
    {
        VehicleState::StateMatrix covariance = VehicleState::StateMatrix::Zero();
        covariance(VehicleState::kPx, VehicleState::kPx) = positionSigmaM * positionSigmaM;
        covariance(VehicleState::kPy, VehicleState::kPy) = positionSigmaM * positionSigmaM;
        covariance(VehicleState::kPsi, VehicleState::kPsi) = headingSigmaRad * headingSigmaRad;
        covariance(VehicleState::kU, VehicleState::kU) = forwardVelocitySigmaMps * forwardVelocitySigmaMps;
        covariance(VehicleState::kV, VehicleState::kV) = lateralVelocitySigmaMps * lateralVelocitySigmaMps;
        covariance(VehicleState::kR, VehicleState::kR) = yawRateSigmaRadps * yawRateSigmaRadps;
        covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasSigmaRadps * gyroBiasSigmaRadps;
        return covariance;
    }

    inline float DistancePerEncoderCountMeters(const PlantParams& params) noexcept
    {
        return
            (2.0f * PI_F * params.wheelRadiusM) /
            (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
    }
}
