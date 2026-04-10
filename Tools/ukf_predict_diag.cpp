// Host-only diagnostic for UKF predict/update covariance behavior.
// Keep this source in Tools so it is available without unignoring generated verify output.

#include "PlantModel.h"
#include "SrUkfCore.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace
{
    MazeMap::VehicleState::StateVector BuildState(
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
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kPx) = xM;
        state(MazeMap::VehicleState::kPy) = yM;
        state(MazeMap::VehicleState::kPsi) = yawRad;
        state(MazeMap::VehicleState::kU) = forwardVelocityMps;
        state(MazeMap::VehicleState::kV) = lateralVelocityMps;
        state(MazeMap::VehicleState::kR) = yawRateRadps;
        state(MazeMap::VehicleState::kOmegaL) = leftWheelSpeedRadps;
        state(MazeMap::VehicleState::kOmegaR) = rightWheelSpeedRadps;
        state(MazeMap::VehicleState::kBgz) = gyroBiasRadps;
        MazeMap::VehicleState::NormalizeStateVector(state);
        return state;
    }

    MazeMap::VehicleState::StateMatrix BuildCovariance(
        float positionSigmaM,
        float headingSigmaRad,
        float forwardVelocitySigmaMps,
        float lateralVelocitySigmaMps,
        float yawRateSigmaRadps,
        float wheelSigmaRadps,
        float gyroBiasSigmaRadps)
    {
        MazeMap::VehicleState::StateMatrix covariance = MazeMap::VehicleState::StateMatrix::Zero();
        covariance(MazeMap::VehicleState::kPx, MazeMap::VehicleState::kPx) = positionSigmaM * positionSigmaM;
        covariance(MazeMap::VehicleState::kPy, MazeMap::VehicleState::kPy) = positionSigmaM * positionSigmaM;
        covariance(MazeMap::VehicleState::kPsi, MazeMap::VehicleState::kPsi) = headingSigmaRad * headingSigmaRad;
        covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU) = forwardVelocitySigmaMps * forwardVelocitySigmaMps;
        covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV) = lateralVelocitySigmaMps * lateralVelocitySigmaMps;
        covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR) = yawRateSigmaRadps * yawRateSigmaRadps;
        covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz) = gyroBiasSigmaRadps * gyroBiasSigmaRadps;
        return covariance;
    }
}

void PrintCoreDriveState(const char* label, const MazeMap::SrUkfCore& core)
{
    const auto state = core.state();
    const auto covariance = core.covariance();
    std::cout
        << label
        << " py=" << state(MazeMap::VehicleState::kPy)
        << " u=" << state(MazeMap::VehicleState::kU)
        << " omega_l=" << state(MazeMap::VehicleState::kOmegaL)
        << " omega_r=" << state(MazeMap::VehicleState::kOmegaR)
        << " pu=" << covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU)
        << '\n';
}

int main(int argc, char** argv)
{
    const MazeMap::PlantParams params = MazeMap::PlantParams::Default();
    MazeMap::PlantModel plant;
    const int stationarySteps = (argc > 1) ? std::max(0, std::atoi(argv[1])) : 2000;
    const int printInterval = (argc > 2) ? std::max(1, std::atoi(argv[2])) : 100;

    std::cout << std::fixed << std::setprecision(8);
    for (const float v : { -0.30f, -0.15f, -0.05f, -0.02f, -0.005f, 0.005f, 0.02f, 0.05f, 0.15f, 0.30f })
    {
        const auto state = BuildState(0.0f, 0.09f, 0.0f, 0.0f, v, 0.0f, 0.0f, 0.0f);
        const MazeMap::ControlInput control{};
        const MazeMap::PlantDerivatives d = plant.forwardStep(state, control, params);
        const auto next = plant.integrate(state, control, 0.001f, params);
        std::cout
            << "v=" << v
            << " vdot=" << d.stateDot(MazeMap::VehicleState::kV)
            << " nextV=" << next(MazeMap::VehicleState::kV)
            << " regime=" << static_cast<int>(d.regime)
            << " util=" << d.maxContactUtilization
            << '\n';
    }

    MazeMap::SrUkfCore core;
    const auto initialState = BuildState(0.0f, 0.09f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const auto initialCovariance = BuildCovariance(0.001f, 0.01f, 0.005f, 1.0f, 1.0f, 0.05f, 0.02f);
    core.reset(initialState, initialCovariance);

    MazeMap::ControlInput control{};
    MazeMap::EncoderObs encoder{};
    constexpr float dt = 0.001f;

    for (int step = 0; step <= stationarySteps; ++step)
    {
        if ((step % printInterval) == 0)
        {
            const auto covariance = core.covariance();
            const auto state = core.state();
            std::cout
                << "step=" << step
                << " v=" << state(MazeMap::VehicleState::kV)
                << " pv=" << covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV)
                << " pr=" << covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
                << " pu=" << covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU)
                << '\n';
        }

        if (step == stationarySteps)
        {
            break;
        }

        core.predict(dt, control);
        const auto encoderResult = core.updateEncoderPair(encoder, dt);
        if (!encoderResult.accepted)
        {
            std::cout << "encoder rejected at step " << step << '\n';
            break;
        }
    }

    MazeMap::SrUkfCore noEncoderCore;
    MazeMap::ControlInput drive{};
    drive.leftMotorCommand = 0.5f;
    drive.rightMotorCommand = 0.5f;
    drive.fanDutyCycle = 0.80f;
    drive.batteryVoltageV = params.supplyVoltageV;

    MazeMap::VehicleState::StateVector rawDriveState = MazeMap::VehicleState::StateVector::Zero();
    for (int step = 0; step < 200; ++step)
    {
        rawDriveState = plant.integrate(rawDriveState, drive, dt, params);
    }
    std::cout
        << "raw_drive"
        << " py=" << rawDriveState(MazeMap::VehicleState::kPy)
        << " u=" << rawDriveState(MazeMap::VehicleState::kU)
        << " omega_l=" << rawDriveState(MazeMap::VehicleState::kOmegaL)
        << " omega_r=" << rawDriveState(MazeMap::VehicleState::kOmegaR)
        << '\n';

    MazeMap::SrUkfCore noEncoderPredictOnlyCore;
    for (int step = 0; step < 200; ++step)
    {
        noEncoderPredictOnlyCore.predict(dt, drive);
    }
    PrintCoreDriveState("no_encoder_predict_only", noEncoderPredictOnlyCore);

    for (int step = 0; step < 200; ++step)
    {
        noEncoderCore.predict(dt, drive);
        noEncoderCore.updateYawRate(0.0f);
    }
    PrintCoreDriveState("no_encoder_drive_yaw_updates", noEncoderCore);

    MazeMap::SrUkfCore yawOnlyCore;
    yawOnlyCore.reset(
        BuildState(0.02f, 0.14f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
        BuildCovariance(0.01f, 0.04f, 0.02f, 0.02f, 0.30f, 0.10f, 0.30f));
    const auto yawBefore = yawOnlyCore.covariance();
    const auto yawResult = yawOnlyCore.updateYawRate(0.35f);
    const auto yawAfter = yawOnlyCore.covariance();
    const auto yawState = yawOnlyCore.state();
    std::cout
        << "yaw_only_update"
        << " accepted=" << yawResult.accepted
        << " r=" << yawState(MazeMap::VehicleState::kR)
        << " bgz=" << yawState(MazeMap::VehicleState::kBgz)
        << " before_pr=" << yawBefore(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
        << " after_pr=" << yawAfter(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
        << " before_pbgz=" << yawBefore(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz)
        << " after_pbgz=" << yawAfter(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz)
        << '\n';

    const float distancePerCountM =
        (2.0f * PI_F * params.wheelRadiusM) /
        (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
    const float measuredWheelOmegaRadps = distancePerCountM / (params.wheelRadiusM * dt);
    const float measuredLinearSpeedMps = params.wheelRadiusM * measuredWheelOmegaRadps;
    MazeMap::SrUkfCore movingEncoderCore(params);
    movingEncoderCore.reset(
        BuildState(
            0.0f,
            0.09f,
            0.0f,
            measuredLinearSpeedMps,
            0.0f,
            0.0f,
            measuredWheelOmegaRadps,
            measuredWheelOmegaRadps),
        BuildCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f));
    const auto movingBefore = movingEncoderCore.covariance();
    movingEncoderCore.predict(dt, control);
    const auto movingPredicted = movingEncoderCore.covariance();
    MazeMap::EncoderObs movingEncoder{};
    movingEncoder.totalLeftCounts = 1;
    movingEncoder.totalRightCounts = 1;
    movingEncoder.omegaLeftRadps = movingEncoderCore.state()(MazeMap::VehicleState::kOmegaL);
    movingEncoder.omegaRightRadps = movingEncoderCore.state()(MazeMap::VehicleState::kOmegaR);
    const auto movingResult = movingEncoderCore.updateEncoderPair(movingEncoder, dt);
    const auto movingAfter = movingEncoderCore.covariance();
    std::cout
        << "moving_encoder_yaw_variance"
        << " accepted=" << movingResult.accepted
        << " before_pr=" << movingBefore(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
        << " predicted_pr=" << movingPredicted(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
        << " after_pr=" << movingAfter(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
        << '\n';
}
