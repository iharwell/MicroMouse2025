#include "pch.h"
#include "VehicleState.h"

#include "Vehicle.h"

namespace
{
    float UkfStationaryLinearSpeedThresholdMps() noexcept
    {
        return 0.002936f;
    }

    float UkfStationaryYawRateThresholdRadps() noexcept
    {
        return 3.0f * 0.0010954451f;
    }

    float UkfStationaryWheelSpeedThresholdRadps() noexcept
    {
        constexpr float wheelRadiusM = MazeMap::Vehicle::GetDriveWheelRadiusM();
        if (!std::isfinite(wheelRadiusM) || !(wheelRadiusM > 0.0f))
        {
            return 0.0f;
        }

        return UkfStationaryLinearSpeedThresholdMps() / wheelRadiusM;
    }
}

namespace MazeMap
{
    bool VehicleState::IsStationary() const noexcept
    {
        const float wheelSpeedThresholdRadps = UkfStationaryWheelSpeedThresholdRadps();
        return
            std::isfinite(_state(kU)) &&
            std::isfinite(_state(kV)) &&
            std::isfinite(_state(kR)) &&
            std::isfinite(_state(kOmegaL)) &&
            std::isfinite(_state(kOmegaR)) &&
            (std::fabs(_state(kU)) <= UkfStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kV)) <= UkfStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kR)) <= UkfStationaryYawRateThresholdRadps()) &&
            (std::fabs(_state(kOmegaL)) <= wheelSpeedThresholdRadps) &&
            (std::fabs(_state(kOmegaR)) <= wheelSpeedThresholdRadps);
    }

}
