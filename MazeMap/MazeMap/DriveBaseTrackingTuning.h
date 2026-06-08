#pragma once

#include "Defines.h"

namespace MazeMap
{
    class EXPORT DriveAxisTrackingTuning final
    {
    public:
        constexpr DriveAxisTrackingTuning() noexcept = default;

        constexpr DriveAxisTrackingTuning(
            float positionErrorToAccelerationGain,
            float velocityErrorToAccelerationGain,
            float accelerationErrorToAccelerationGain) noexcept
            : _positionErrorToAccelerationGain(positionErrorToAccelerationGain)
            , _velocityErrorToAccelerationGain(velocityErrorToAccelerationGain)
            , _accelerationErrorToAccelerationGain(accelerationErrorToAccelerationGain)
        {
        }

        constexpr float PositionErrorToAccelerationGain() const noexcept { return _positionErrorToAccelerationGain; }
        constexpr float VelocityErrorToAccelerationGain() const noexcept { return _velocityErrorToAccelerationGain; }
        constexpr float AccelerationErrorToAccelerationGain() const noexcept { return _accelerationErrorToAccelerationGain; }

        float ComposeAcceleration(
            float requestedAcceleration,
            float positionError,
            float velocityError,
            float accelerationError) const noexcept;

    private:
        float _positionErrorToAccelerationGain = 0.0f;
        float _velocityErrorToAccelerationGain = 0.0f;
        float _accelerationErrorToAccelerationGain = 0.0f;
    };

    class EXPORT DriveBaseTrackingTuning final
    {
    public:
        constexpr DriveBaseTrackingTuning() noexcept = default;

        constexpr DriveBaseTrackingTuning(
            DriveAxisTrackingTuning forwardAxis,
            DriveAxisTrackingTuning yawAxis) noexcept
            : _forwardAxis(forwardAxis)
            , _yawAxis(yawAxis)
        {
        }

        constexpr const DriveAxisTrackingTuning& ForwardAxis() const noexcept { return _forwardAxis; }
        constexpr const DriveAxisTrackingTuning& YawAxis() const noexcept { return _yawAxis; }

        float ComposeForwardAccelerationMps2(
            float requestedForwardAccelMps2,
            float forwardPositionErrorM,
            float forwardVelocityErrorMps,
            float forwardAccelerationErrorMps2) const noexcept;

        float ComposeYawAccelerationRadps2(
            float requestedYawAccelRadps2,
            float yawPositionErrorRad,
            float yawVelocityErrorRadps,
            float yawAccelerationErrorRadps2) const noexcept;

    private:
        DriveAxisTrackingTuning _forwardAxis{};
        DriveAxisTrackingTuning _yawAxis{};
    };
}
