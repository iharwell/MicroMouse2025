#include "pch.h"
#include "VehicleState.h"

#include "Vehicle.h"

static float EstimatorStationaryLinearSpeedThresholdMps() noexcept
{
    return 0.002936f;
}

static float EstimatorStationaryYawRateThresholdRadps() noexcept
{
    return 3.0f * 0.0010954451f;
}

static float EstimatorStationaryWheelSpeedThresholdRadps() noexcept
{
    constexpr float wheelRadiusM = MazeMap::Vehicle::GetDriveWheelRadiusM();
    if (!std::isfinite(wheelRadiusM) || !(wheelRadiusM > 0.0f))
    {
        return 0.0f;
    }

    return EstimatorStationaryLinearSpeedThresholdMps() / wheelRadiusM;
}

namespace MazeMap
{
    void VehicleState::PublishEncoderWheelSpeedsRadps(
        const float leftWheelSpeedRadps,
        const float rightWheelSpeedRadps,
        const bool valid) noexcept
    {
        SensorSnapshot::EncoderObs observation = _sensorSnapshot.EncoderObservation();
        observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
        observation.SetWheelLinearVelocityMps(
            Vehicle::WheelLinearVelocityFromWheelSpeed(leftWheelSpeedRadps),
            Vehicle::WheelLinearVelocityFromWheelSpeed(rightWheelSpeedRadps));
        _sensorSnapshot.PublishEncoderObservation(
            observation,
            valid,
            _sensorSnapshot.LeftEncoderTotalCounts(),
            _sensorSnapshot.RightEncoderTotalCounts(),
            _sensorSnapshot.LeftEncoderDistanceM(),
            _sensorSnapshot.RightEncoderDistanceM());
    }

    bool VehicleState::IsStationary() const noexcept
    {
        const float wheelSpeedThresholdRadps = EstimatorStationaryWheelSpeedThresholdRadps();
        return
            std::isfinite(_state(kVf)) &&
            std::isfinite(_state(kVr)) &&
            std::isfinite(_state(kYawRate)) &&
            std::isfinite(GetWheelSpeedLeft()) &&
            std::isfinite(GetWheelSpeedRight()) &&
            (std::fabs(_state(kVf)) <= EstimatorStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kVr)) <= EstimatorStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kYawRate)) <= EstimatorStationaryYawRateThresholdRadps()) &&
            (std::fabs(GetWheelSpeedLeft()) <= wheelSpeedThresholdRadps) &&
            (std::fabs(GetWheelSpeedRight()) <= wheelSpeedThresholdRadps);
    }

}
