#pragma once

#include "CommandPD.h"
#include "Defines.h"
#include "ProportionalDerivative.h"

namespace MazeMap
{
    // Owns the canonical named proportional-derivative setups used by drive and maneuver code.
    // Each member name follows `[Controlled][Signal]PD`.
    //
    // Controlled quantity tokens:
    // - `Heading`
    // - `Velocity`
    // - `YawRate`
    // - `LongitudinalAcceleration`
    // - `WheelVelocity`
    // - `YawAcceleration`
    //
    // Signal tokens:
    // - `State`: the matching signal field from the state vector,
    // - `Gyro`: the IMU yaw-rate signal,
    // - `EncoderAverage`: the body-forward encoder average signal,
    // - `EncoderDelta`: the left encoder reading minus the right encoder reading,
    // - `IMUForwardAccel`: the IMU forward-acceleration signal,
    // - `IMULateralAccel`: the IMU lateral-acceleration signal.
    //
    // This type stays deliberately small. It owns the named PD setups and only cluster-wide
    // operations that apply uniformly to every setup. It does not choose maneuver policy or
    // perform signal preprocessing. It does provide controlled-quantity lookup helpers so code
    // such as `DriveBase` can map a `CommandPD` source selection onto the corresponding named
    // setup without hard-coding member names repeatedly.
    class EXPORT ProportionalDerivativeCluster final
    {
    public:
        // Builds a cluster whose named setups all start from their default zero-gain state.
        constexpr ProportionalDerivativeCluster() noexcept = default;

        // Builds the full cluster from explicitly named setups.
        //
        // Parameter mapping is exact and positional: each constructor argument initializes the
        // identically named member in declaration order.
        constexpr ProportionalDerivativeCluster(
            ProportionalDerivative headingStatePD,
            ProportionalDerivative headingGyroPD,
            ProportionalDerivative headingEncoderDeltaPD,
            ProportionalDerivative velocityStatePD,
            ProportionalDerivative velocityEncoderAveragePD,
            ProportionalDerivative yawRateStatePD,
            ProportionalDerivative yawRateGyroPD,
            ProportionalDerivative yawRateEncoderDeltaPD,
            ProportionalDerivative yawRateIMULateralAccelPD,
            ProportionalDerivative longitudinalAccelerationStatePD,
            ProportionalDerivative longitudinalAccelerationIMUForwardAccelPD,
            ProportionalDerivative wheelVelocityStatePD,
            ProportionalDerivative wheelVelocityEncoderPD,
            ProportionalDerivative yawAccelerationStatePD,
            ProportionalDerivative yawAccelerationGyroPD,
            ProportionalDerivative yawAccelerationEncoderDeltaPD) noexcept
            : HeadingStatePD(headingStatePD)
            , HeadingGyroPD(headingGyroPD)
            , HeadingEncoderDeltaPD(headingEncoderDeltaPD)
            , VelocityStatePD(velocityStatePD)
            , VelocityEncoderAveragePD(velocityEncoderAveragePD)
            , YawRateStatePD(yawRateStatePD)
            , YawRateGyroPD(yawRateGyroPD)
            , YawRateEncoderDeltaPD(yawRateEncoderDeltaPD)
            , YawRateIMULateralAccelPD(yawRateIMULateralAccelPD)
            , LongitudinalAccelerationStatePD(longitudinalAccelerationStatePD)
            , LongitudinalAccelerationIMUForwardAccelPD(longitudinalAccelerationIMUForwardAccelPD)
            , WheelVelocityStatePD(wheelVelocityStatePD)
            , WheelVelocityEncoderPD(wheelVelocityEncoderPD)
            , YawAccelerationStatePD(yawAccelerationStatePD)
            , YawAccelerationGyroPD(yawAccelerationGyroPD)
            , YawAccelerationEncoderDeltaPD(yawAccelerationEncoderDeltaPD)
        {
        }

        // Clears the sampled-derivative history in every owned setup.
        void ResetDerivativeHistories() noexcept;

        // Returns the heading-focused setup selected by the supplied `CommandPD` source.
        // Unsupported or ambiguous source selections return an inert zero-gain setup reference.
        const ProportionalDerivative& GetHeadingPD(CommandPD pd) const noexcept;

        // Returns the forward-velocity-focused setup selected by the supplied `CommandPD` source.
        // Unsupported or ambiguous source selections return an inert zero-gain setup reference.
        const ProportionalDerivative& GetVelocityPD(CommandPD pd) const noexcept;

        // Returns the yaw-rate-focused setup selected by the supplied `CommandPD` source.
        // Unsupported or ambiguous source selections return an inert zero-gain setup reference.
        const ProportionalDerivative& GetYawRatePD(CommandPD pd) const noexcept;

        // Returns the longitudinal-acceleration-focused setup selected by the supplied
        // `CommandPD` source. Unsupported or ambiguous source selections return an inert
        // zero-gain setup reference.
        const ProportionalDerivative& GetLongitudinalAccelerationPD(CommandPD pd) const noexcept;

        // Returns the wheel-velocity-focused setup selected by the supplied `CommandPD` source.
        // Unsupported or ambiguous source selections return an inert zero-gain setup reference.
        const ProportionalDerivative& GetWheelVelocityPD(CommandPD pd) const noexcept;

        // Returns the yaw-acceleration-focused setup selected by the supplied `CommandPD` source.
        // Unsupported or ambiguous source selections return an inert zero-gain setup reference.
        const ProportionalDerivative& GetYawAccelerationPD(CommandPD pd) const noexcept;

        // Heading correction using state heading / yaw angle as the feedback signal.
        ProportionalDerivative HeadingStatePD{};

        // Heading correction using the gyro signal path.
        ProportionalDerivative HeadingGyroPD{};

        // Heading correction using encoder delta.
        ProportionalDerivative HeadingEncoderDeltaPD{};

        // Forward-velocity correction using state forward velocity.
        ProportionalDerivative VelocityStatePD{};

        // Forward-velocity correction using encoder average.
        ProportionalDerivative VelocityEncoderAveragePD{};

        // Yaw-rate correction using state yaw rate.
        ProportionalDerivative YawRateStatePD{};

        // Yaw-rate correction using the gyro signal path.
        ProportionalDerivative YawRateGyroPD{};

        // Yaw-rate correction using encoder delta.
        ProportionalDerivative YawRateEncoderDeltaPD{};

        // Yaw-rate correction using IMU lateral acceleration.
        ProportionalDerivative YawRateIMULateralAccelPD{};

        // Longitudinal-acceleration correction using state longitudinal acceleration.
        ProportionalDerivative LongitudinalAccelerationStatePD{};

        // Longitudinal-acceleration correction using IMU forward acceleration.
        ProportionalDerivative LongitudinalAccelerationIMUForwardAccelPD{};

        // Wheel-velocity correction using wheel velocity from state.
        ProportionalDerivative WheelVelocityStatePD{};

        // Wheel-velocity correction using encoder wheel velocity.
        ProportionalDerivative WheelVelocityEncoderPD{};

        // Yaw-acceleration correction using state yaw acceleration.
        ProportionalDerivative YawAccelerationStatePD{};

        // Yaw-acceleration correction using the gyro signal path.
        ProportionalDerivative YawAccelerationGyroPD{};

        // Yaw-acceleration correction using encoder delta.
        ProportionalDerivative YawAccelerationEncoderDeltaPD{};
    };
}
