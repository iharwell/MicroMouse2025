#pragma once

#include "Defines.h"
#include "ProportionalDerivative.h"

namespace MazeMap
{
    // Owns the canonical named proportional-derivative setups used by drive and maneuver code.
    // Member names follow `[ControlledValue][SignalSource]PD`. Encoder-derived signals are named by
    // the body-frame value they produce: wheel average for forward velocity, wheel delta for yaw rate.
    //
    // This type stays deliberately small. It owns the named PD setups and only cluster-wide
    // operations that apply uniformly to every setup. It does not choose maneuver policy or
    // perform signal preprocessing.
    class EXPORT PDCluster final
    {
    public:
        // Builds a cluster whose named setups all start from their default zero-gain state.
        constexpr PDCluster() noexcept = default;

        // Builds the full cluster from explicitly named setups.
        //
        // Parameter mapping is exact and positional: each constructor argument initializes the
        // identically named member in declaration order.
        constexpr PDCluster(
            ProportionalDerivative headingStatePD,
            ProportionalDerivative velocityStatePD,
            ProportionalDerivative velocityEncoderAveragePD,
            ProportionalDerivative yawRateStatePD,
            ProportionalDerivative yawRateGyroPD,
            ProportionalDerivative yawRateEncoderDeltaPD,
            ProportionalDerivative yawRateIMULateralAccelPD,
            ProportionalDerivative longitudinalAccelerationStatePD,
            ProportionalDerivative longitudinalAccelerationIMUForwardAccelPD) noexcept
            : HeadingStatePD(headingStatePD)
            , VelocityStatePD(velocityStatePD)
            , VelocityEncoderAveragePD(velocityEncoderAveragePD)
            , YawRateStatePD(yawRateStatePD)
            , YawRateGyroPD(yawRateGyroPD)
            , YawRateEncoderDeltaPD(yawRateEncoderDeltaPD)
            , YawRateIMULateralAccelPD(yawRateIMULateralAccelPD)
            , LongitudinalAccelerationStatePD(longitudinalAccelerationStatePD)
            , LongitudinalAccelerationIMUForwardAccelPD(longitudinalAccelerationIMUForwardAccelPD)
        {
        }

        // Clears the sampled-derivative history in every owned setup.
        void ResetDerivativeHistories() noexcept;

        // Heading correction using state heading / yaw angle as the feedback signal.
        ProportionalDerivative HeadingStatePD{};

        // Forward-velocity correction using state forward velocity.
        ProportionalDerivative VelocityStatePD{};

        // Forward-velocity correction using encoder wheel average.
        ProportionalDerivative VelocityEncoderAveragePD{};

        // Yaw-rate correction using state yaw rate.
        ProportionalDerivative YawRateStatePD{};

        // Yaw-rate correction using the gyro signal path.
        ProportionalDerivative YawRateGyroPD{};

        // Yaw-rate correction using encoder wheel delta.
        ProportionalDerivative YawRateEncoderDeltaPD{};

        // Yaw-rate correction using IMU lateral acceleration.
        ProportionalDerivative YawRateIMULateralAccelPD{};

        // Longitudinal-acceleration correction using state longitudinal acceleration.
        ProportionalDerivative LongitudinalAccelerationStatePD{};

        // Longitudinal-acceleration correction using IMU forward acceleration.
        ProportionalDerivative LongitudinalAccelerationIMUForwardAccelPD{};
    };
}
