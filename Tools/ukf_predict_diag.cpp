// Host-only diagnostic for estimator predict/update behavior.
// Keep this source in Tools so it is available without unignoring generated verify output.

#include "CommandVector.h"
#include "EigenCompat.h"
#include "EncoderObs.h"
#include "Estimator.h"
#include "PlantModel.h"
#include "Vehicle.h"
#include "VehicleState.h"

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace
{
    constexpr float kTickSeconds = 0.001f;
    constexpr float kFanDuty = 0.80f;
}

int main(int argc, char** argv)
{
    const int stationarySteps = (argc > 1) ? (std::max)(0, std::atoi(argv[1])) : 2000;
    const int printInterval = (argc > 2) ? (std::max)(1, std::atoi(argv[2])) : 100;
    const MazeMap::App::Internal::CommandVector zeroControl(0.0f, 0.0f);

    std::cout << std::fixed << std::setprecision(8);
    for (const float v : { -0.30f, -0.15f, -0.05f, -0.02f, -0.005f, 0.005f, 0.02f, 0.05f, 0.15f, 0.30f })
    {
        MazeMap::Vehicle vehicle{};
        vehicle.SetFanDuty(kFanDuty);
        MazeMap::VehicleState state{};
        state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
        state.SetOrientation(0.0f);
        state.SetVelocity(0.0f);
        state.SetLateralVelocity(v);
        state.SetRotationalVelocity(0.0f);
        state.SetWheelSpeedLeft(0.0f);
        state.SetWheelSpeedRight(0.0f);
        state.SetGyroBiasZ(0.0f);
        MazeMap::PlantModel plant(vehicle, state);

        const float previousV = state.GetLateralVelocity();
        plant.integrate(zeroControl, kTickSeconds);
        const float nextV = state.GetLateralVelocity();
        std::cout
            << "v=" << v
            << " vdot=" << ((nextV - previousV) / kTickSeconds)
            << " nextV=" << nextV
            << " regime=unavailable"
            << " util=unavailable"
            << '\n';
    }

    MazeMap::Vehicle stationaryVehicle{};
    stationaryVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState stationaryState{};
    stationaryState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
    stationaryState.SetOrientation(0.0f);
    stationaryState.SetVelocity(0.0f);
    stationaryState.SetLateralVelocity(0.0f);
    stationaryState.SetRotationalVelocity(0.0f);
    stationaryState.SetWheelSpeedLeft(0.0f);
    stationaryState.SetWheelSpeedRight(0.0f);
    stationaryState.SetGyroBiasZ(0.0f);
    MazeMap::PlantModel stationaryPlant(stationaryVehicle, stationaryState);
    MazeMap::Estimator stationaryEstimator(stationaryPlant, stationaryState);

    MazeMap::EncoderObs encoder{};

    for (int step = 0; step <= stationarySteps; ++step)
    {
        if ((step % printInterval) == 0)
        {
            std::cout
                << "step=" << step
                << " u=" << stationaryState.GetVelocity()
                << " v=" << stationaryState.GetLateralVelocity()
                << " r=" << stationaryState.GetRotationalVelocity()
                << '\n';
        }

        if (step == stationarySteps)
        {
            break;
        }

        (void)stationaryEstimator.predict(kTickSeconds, zeroControl);
        const auto encoderResult = stationaryEstimator.updateEncoderPair(encoder, kTickSeconds);
        if (!encoderResult.accepted)
        {
            std::cout << "encoder rejected at step " << step << '\n';
            break;
        }
    }

    const MazeMap::App::Internal::CommandVector drive(0.5f, 0.5f);

    MazeMap::Vehicle rawDriveVehicle{};
    rawDriveVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState rawDriveState{};
    MazeMap::PlantModel rawDrivePlant(rawDriveVehicle, rawDriveState);
    for (int step = 0; step < 200; ++step)
    {
        rawDrivePlant.integrate(drive, kTickSeconds);
    }
    std::cout
        << "raw_drive"
        << " py=" << rawDriveState.GetPositionY()
        << " u=" << rawDriveState.GetVelocity()
        << " omega_l=" << rawDriveState.GetWheelSpeedLeft()
        << " omega_r=" << rawDriveState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle noEncoderPredictVehicle{};
    noEncoderPredictVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState noEncoderPredictState{};
    MazeMap::PlantModel noEncoderPredictPlant(noEncoderPredictVehicle, noEncoderPredictState);
    MazeMap::Estimator noEncoderPredictEstimator(noEncoderPredictPlant, noEncoderPredictState);
    for (int step = 0; step < 200; ++step)
    {
        (void)noEncoderPredictEstimator.predict(kTickSeconds, drive);
    }
    std::cout
        << "no_encoder_predict_only"
        << " py=" << noEncoderPredictState.GetPositionY()
        << " u=" << noEncoderPredictState.GetVelocity()
        << " omega_l=" << noEncoderPredictState.GetWheelSpeedLeft()
        << " omega_r=" << noEncoderPredictState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle noEncoderYawVehicle{};
    noEncoderYawVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState noEncoderYawState{};
    MazeMap::PlantModel noEncoderYawPlant(noEncoderYawVehicle, noEncoderYawState);
    MazeMap::Estimator noEncoderYawEstimator(noEncoderYawPlant, noEncoderYawState);
    for (int step = 0; step < 200; ++step)
    {
        (void)noEncoderYawEstimator.predict(kTickSeconds, drive);
        (void)noEncoderYawEstimator.updateYawRate(0.0f);
    }
    std::cout
        << "no_encoder_drive_yaw_updates"
        << " py=" << noEncoderYawState.GetPositionY()
        << " u=" << noEncoderYawState.GetVelocity()
        << " omega_l=" << noEncoderYawState.GetWheelSpeedLeft()
        << " omega_r=" << noEncoderYawState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle yawOnlyVehicle{};
    yawOnlyVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState yawOnlyState{};
    yawOnlyState.SetPosition(Eigen::Vector2f(0.02f, 0.14f));
    yawOnlyState.SetOrientation(0.10f);
    yawOnlyState.SetVelocity(0.0f);
    yawOnlyState.SetLateralVelocity(0.0f);
    yawOnlyState.SetRotationalVelocity(0.0f);
    yawOnlyState.SetWheelSpeedLeft(0.0f);
    yawOnlyState.SetWheelSpeedRight(0.0f);
    yawOnlyState.SetGyroBiasZ(0.0f);
    MazeMap::PlantModel yawOnlyPlant(yawOnlyVehicle, yawOnlyState);
    MazeMap::Estimator yawOnlyEstimator(yawOnlyPlant, yawOnlyState);
    const auto yawResult = yawOnlyEstimator.updateYawRate(0.35f);
    std::cout
        << "yaw_only_update"
        << " accepted=" << yawResult.accepted
        << " r=" << yawOnlyState.GetRotationalVelocity()
        << " bgz=" << yawOnlyState.GetGyroBiasZ()
        << '\n';

    const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
    const float measuredLinearSpeedMps = distancePerCountM / kTickSeconds;
    const float measuredWheelOmegaRadps = MazeMap::Vehicle::WheelOmegaFromLinearVelocity(measuredLinearSpeedMps);
    MazeMap::Vehicle movingEncoderVehicle{};
    movingEncoderVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState movingEncoderState{};
    movingEncoderState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
    movingEncoderState.SetOrientation(0.0f);
    movingEncoderState.SetVelocity(measuredLinearSpeedMps);
    movingEncoderState.SetLateralVelocity(0.0f);
    movingEncoderState.SetRotationalVelocity(0.0f);
    movingEncoderState.SetWheelSpeedLeft(measuredWheelOmegaRadps);
    movingEncoderState.SetWheelSpeedRight(measuredWheelOmegaRadps);
    movingEncoderState.SetGyroBiasZ(0.0f);
    MazeMap::PlantModel movingEncoderPlant(movingEncoderVehicle, movingEncoderState);
    MazeMap::Estimator movingEncoderEstimator(movingEncoderPlant, movingEncoderState);
    (void)movingEncoderEstimator.predict(kTickSeconds, zeroControl);
    MazeMap::EncoderObs movingEncoder{};
    movingEncoder.totalLeftCounts = 1;
    movingEncoder.totalRightCounts = 1;
    movingEncoder.leftDistanceDeltaM = distancePerCountM;
    movingEncoder.rightDistanceDeltaM = distancePerCountM;
    movingEncoder.leftVelocityMps = measuredLinearSpeedMps;
    movingEncoder.rightVelocityMps = measuredLinearSpeedMps;
    movingEncoder.omegaLeftRadps = movingEncoderState.GetWheelSpeedLeft();
    movingEncoder.omegaRightRadps = movingEncoderState.GetWheelSpeedRight();
    const auto movingResult = movingEncoderEstimator.updateEncoderPair(movingEncoder, kTickSeconds);
    std::cout
        << "moving_encoder_update"
        << " accepted=" << movingResult.accepted
        << " u=" << movingEncoderState.GetVelocity()
        << " r=" << movingEncoderState.GetRotationalVelocity()
        << " omega_l=" << movingEncoderState.GetWheelSpeedLeft()
        << " omega_r=" << movingEncoderState.GetWheelSpeedRight()
        << '\n';
}
