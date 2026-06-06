// Host-only diagnostic for estimator predict/update behavior.
// Keep this source in Tools so it is available without unignoring generated verify output.

#include "CommandVector.h"
#include "EigenCompat.h"
#include "Estimator.h"
#include "PlantModel.h"
#include "SensorSnapshot.h"
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
    for (const float lateralVelocityMps : { -0.30f, -0.15f, -0.05f, -0.02f, -0.005f, 0.005f, 0.02f, 0.05f, 0.15f, 0.30f })
    {
        MazeMap::Vehicle vehicle{};
        vehicle.SetFanDuty(kFanDuty);
        MazeMap::VehicleState state{};
        state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
        state.SetHeading(0.0f);
        state.SetForwardVelocity(0.0f);
        state.SetRightwardVelocity(lateralVelocityMps);
        state.SetYawRate(0.0f);
        state.PublishEncoderWheelSpeedsRadps(0.0f, 0.0f);
        MazeMap::PlantModel plant(vehicle, state);

        const float previousVrMps = state.GetRightwardVelocity();
        plant.integrate(zeroControl, kTickSeconds);
        const float nextVrMps = state.GetRightwardVelocity();
        std::cout
            << "vr=" << lateralVelocityMps
            << " vr_dot=" << ((nextVrMps - previousVrMps) / kTickSeconds)
            << " next_vr=" << nextVrMps
            << " regime=unavailable"
            << " util=unavailable"
            << '\n';
    }

    MazeMap::Vehicle stationaryVehicle{};
    stationaryVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState stationaryState{};
    stationaryState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
    stationaryState.SetHeading(0.0f);
    stationaryState.SetForwardVelocity(0.0f);
    stationaryState.SetRightwardVelocity(0.0f);
    stationaryState.SetYawRate(0.0f);
    stationaryState.PublishEncoderWheelSpeedsRadps(0.0f, 0.0f);
    MazeMap::PlantModel stationaryPlant(stationaryVehicle, stationaryState);
    MazeMap::Estimator stationaryEstimator(stationaryVehicle, stationaryPlant, stationaryState);

    SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
    SensorSnapshot stationarySnapshot{};
    stationarySnapshot.PublishEncoderObservation(
        encoder,
        true,
        0,
        0,
        0.0f,
        0.0f);

    for (int step = 0; step <= stationarySteps; ++step)
    {
        if ((step % printInterval) == 0)
        {
            std::cout
                << "step=" << step
                << " vf=" << stationaryState.GetForwardVelocity()
                << " vr=" << stationaryState.GetRightwardVelocity()
                << " yaw_rate=" << stationaryState.GetYawRate()
                << '\n';
        }

        if (step == stationarySteps)
        {
            break;
        }

        stationaryState.SetSensorSnapshot(stationarySnapshot);
        stationaryState.SetTime(stationaryState.GetTime() + kTickSeconds);
        (void)stationaryEstimator.predict(kTickSeconds, zeroControl);
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
        << " vf=" << rawDriveState.GetForwardVelocity()
        << " left_wheel_speed=" << rawDriveState.GetWheelSpeedLeft()
        << " right_wheel_speed=" << rawDriveState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle noEncoderPredictVehicle{};
    noEncoderPredictVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState noEncoderPredictState{};
    MazeMap::PlantModel noEncoderPredictPlant(noEncoderPredictVehicle, noEncoderPredictState);
    MazeMap::Estimator noEncoderPredictEstimator(noEncoderPredictVehicle, noEncoderPredictPlant, noEncoderPredictState);
    for (int step = 0; step < 200; ++step)
    {
        (void)noEncoderPredictEstimator.predict(kTickSeconds, drive);
    }
    std::cout
        << "no_encoder_predict_only"
        << " py=" << noEncoderPredictState.GetPositionY()
        << " vf=" << noEncoderPredictState.GetForwardVelocity()
        << " left_wheel_speed=" << noEncoderPredictState.GetWheelSpeedLeft()
        << " right_wheel_speed=" << noEncoderPredictState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle noEncoderYawVehicle{};
    noEncoderYawVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState noEncoderYawState{};
    MazeMap::PlantModel noEncoderYawPlant(noEncoderYawVehicle, noEncoderYawState);
    MazeMap::Estimator noEncoderYawEstimator(noEncoderYawVehicle, noEncoderYawPlant, noEncoderYawState);
    for (int step = 0; step < 200; ++step)
    {
        (void)noEncoderYawEstimator.predict(kTickSeconds, drive);
        (void)noEncoderYawEstimator.updateYawRate(0.0f);
    }
    std::cout
        << "no_encoder_drive_yaw_updates"
        << " py=" << noEncoderYawState.GetPositionY()
        << " vf=" << noEncoderYawState.GetForwardVelocity()
        << " left_wheel_speed=" << noEncoderYawState.GetWheelSpeedLeft()
        << " right_wheel_speed=" << noEncoderYawState.GetWheelSpeedRight()
        << '\n';

    MazeMap::Vehicle yawOnlyVehicle{};
    yawOnlyVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState yawOnlyState{};
    yawOnlyState.SetPosition(Eigen::Vector2f(0.02f, 0.14f));
    yawOnlyState.SetHeading(0.10f);
    yawOnlyState.SetForwardVelocity(0.0f);
    yawOnlyState.SetRightwardVelocity(0.0f);
    yawOnlyState.SetYawRate(0.0f);
    yawOnlyState.PublishEncoderWheelSpeedsRadps(0.0f, 0.0f);
    MazeMap::PlantModel yawOnlyPlant(yawOnlyVehicle, yawOnlyState);
    MazeMap::Estimator yawOnlyEstimator(yawOnlyVehicle, yawOnlyPlant, yawOnlyState);
    const float correctedYawRateRadps = 0.35f;
    const bool yawAccepted = yawOnlyEstimator.updateYawRate(correctedYawRateRadps);
    std::cout
        << "yaw_only_update"
        << " accepted=" << yawAccepted
        << " corrected_yaw_rate=" << yawOnlyState.GetYawRate()
        << '\n';

    const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
    const float measuredLinearSpeedMps = distancePerCountM / kTickSeconds;
    const float measuredWheelSpeedRadps = MazeMap::Vehicle::WheelSpeedFromLinearVelocity(measuredLinearSpeedMps);
    MazeMap::Vehicle movingEncoderVehicle{};
    movingEncoderVehicle.SetFanDuty(kFanDuty);
    MazeMap::VehicleState movingEncoderState{};
    movingEncoderState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
    movingEncoderState.SetHeading(0.0f);
    movingEncoderState.SetForwardVelocity(measuredLinearSpeedMps);
    movingEncoderState.SetRightwardVelocity(0.0f);
    movingEncoderState.SetYawRate(0.0f);
    movingEncoderState.PublishEncoderWheelSpeedsRadps(measuredWheelSpeedRadps, measuredWheelSpeedRadps);
    MazeMap::PlantModel movingEncoderPlant(movingEncoderVehicle, movingEncoderState);
    MazeMap::Estimator movingEncoderEstimator(movingEncoderVehicle, movingEncoderPlant, movingEncoderState);
    SensorSnapshot::EncoderObs movingEncoder = SensorSnapshot{}.EncoderObservation();
    movingEncoder.SetTotalLeftCounts(1);
    movingEncoder.SetTotalRightCounts(1);
    movingEncoder.SetLeftDistanceDeltaM(distancePerCountM);
    movingEncoder.SetRightDistanceDeltaM(distancePerCountM);
    movingEncoder.SetLeftVelocityMps(measuredLinearSpeedMps);
    movingEncoder.SetRightVelocityMps(measuredLinearSpeedMps);
    movingEncoder.SetLeftWheelSpeedRadps(movingEncoderState.GetWheelSpeedLeft());
    movingEncoder.SetRightWheelSpeedRadps(movingEncoderState.GetWheelSpeedRight());
    SensorSnapshot movingEncoderSnapshot{};
    movingEncoderSnapshot.PublishEncoderObservation(
        movingEncoder,
        true,
        movingEncoder.TotalLeftCounts(),
        movingEncoder.TotalRightCounts(),
        movingEncoder.LeftDistanceDeltaM(),
        movingEncoder.RightDistanceDeltaM());
    movingEncoderState.SetSensorSnapshot(movingEncoderSnapshot);
    movingEncoderState.SetTime(movingEncoderState.GetTime() + kTickSeconds);
    (void)movingEncoderEstimator.predict(kTickSeconds, zeroControl);
    std::cout
        << "moving_encoder_predict"
        << " vf=" << movingEncoderState.GetForwardVelocity()
        << " yaw_rate=" << movingEncoderState.GetYawRate()
        << " left_wheel_speed=" << movingEncoderState.GetWheelSpeedLeft()
        << " right_wheel_speed=" << movingEncoderState.GetWheelSpeedRight()
        << '\n';
}
