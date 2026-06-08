#include "pch.h"
#include "DriveBaseTrackingTuning.h"

#include <cmath>
#include <limits>

namespace MazeMap
{
    float DriveAxisTrackingTuning::ComposeAcceleration(
        const float requestedAcceleration,
        const float positionError,
        const float velocityError,
        const float accelerationError) const noexcept
    {
        if (std::isinf(requestedAcceleration))
        {
            return requestedAcceleration;
        }

        bool hasAccelerationObjective = std::isfinite(requestedAcceleration);
        float accelerationObjective = hasAccelerationObjective ? requestedAcceleration : 0.0f;

        if (std::isfinite(_positionErrorToAccelerationGain) &&
            (_positionErrorToAccelerationGain >= 0.0f) &&
            std::isfinite(positionError))
        {
            accelerationObjective += _positionErrorToAccelerationGain * positionError;
            hasAccelerationObjective = true;
        }

        if (std::isfinite(_velocityErrorToAccelerationGain) &&
            (_velocityErrorToAccelerationGain >= 0.0f) &&
            std::isfinite(velocityError))
        {
            accelerationObjective += _velocityErrorToAccelerationGain * velocityError;
            hasAccelerationObjective = true;
        }

        if (std::isfinite(_accelerationErrorToAccelerationGain) &&
            (_accelerationErrorToAccelerationGain >= 0.0f) &&
            std::isfinite(accelerationError))
        {
            accelerationObjective += _accelerationErrorToAccelerationGain * accelerationError;
            hasAccelerationObjective = true;
        }

        return hasAccelerationObjective ?
            accelerationObjective :
            (std::numeric_limits<float>::quiet_NaN)();
    }

    float DriveBaseTrackingTuning::ComposeForwardAccelerationMps2(
        const float requestedForwardAccelMps2,
        const float forwardPositionErrorM,
        const float forwardVelocityErrorMps,
        const float forwardAccelerationErrorMps2) const noexcept
    {
        return _forwardAxis.ComposeAcceleration(
            requestedForwardAccelMps2,
            forwardPositionErrorM,
            forwardVelocityErrorMps,
            forwardAccelerationErrorMps2);
    }

    float DriveBaseTrackingTuning::ComposeYawAccelerationRadps2(
        const float requestedYawAccelRadps2,
        const float yawPositionErrorRad,
        const float yawVelocityErrorRadps,
        const float yawAccelerationErrorRadps2) const noexcept
    {
        return _yawAxis.ComposeAcceleration(
            requestedYawAccelRadps2,
            yawPositionErrorRad,
            yawVelocityErrorRadps,
            yawAccelerationErrorRadps2);
    }
}
