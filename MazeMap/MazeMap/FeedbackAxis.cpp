#include "pch.h"
#include "FeedbackAxis.h"

#include "PDCluster.h"
#include "ProportionalDerivative.h"
#include "SensorSnapshot.h"
#include "Vehicle.h"
#include "VehicleState.h"

namespace
{
    constexpr float kMinimumDerivativeDtSeconds = 1.0e-6f;

    std::uint8_t SourceIndex(const MazeMap::FeedbackSource source) noexcept
    {
        if (source == MazeMap::FeedbackSource::Imu)
        {
            return 1U;
        }
        if (source == MazeMap::FeedbackSource::Encoder)
        {
            return 2U;
        }
        return 0U;
    }
}

MazeMap::FeedbackAxis::FeedbackAxis(
    const VehicleState* const state,
    const bool rotational) noexcept
    : _state(state)
    , _rotational(rotational)
{
    for (std::uint8_t sourceIndex = 0U; sourceIndex < 3U; ++sourceIndex)
    {
        for (std::uint8_t derivativeOrder = 0U; derivativeOrder < 3U; ++derivativeOrder)
        {
            _previousObserved[sourceIndex][derivativeOrder] =
                std::numeric_limits<float>::quiet_NaN();
            _previousObservedTimeS[sourceIndex][derivativeOrder] =
                std::numeric_limits<float>::quiet_NaN();
            _previousObservedDerivative[sourceIndex][derivativeOrder] =
                std::numeric_limits<float>::quiet_NaN();
        }
    }
}

float MazeMap::FeedbackAxis::GetFeedback(
    const std::uint8_t derivativeOrder,
    const float target,
    const FeedbackSource sources,
    const PDCluster& proportionalDerivativeCluster) const noexcept
{
    float sum = 0.0f;
    std::uint8_t count = 0U;
    std::uint8_t bits = static_cast<std::uint8_t>(sources);
    while (bits != 0U)
    {
        const std::uint8_t bit =
            static_cast<std::uint8_t>(bits & static_cast<std::uint8_t>(0U - bits));
        bits = static_cast<std::uint8_t>(bits & static_cast<std::uint8_t>(~bit));

        sum +=
            GetSourceFeedback(
            derivativeOrder,
            static_cast<FeedbackSource>(bit),
            target,
            proportionalDerivativeCluster);
        ++count;
    }

    return (count != 0U) ? (sum / static_cast<float>(count)) : 0.0f;
}

float MazeMap::FeedbackAxis::GetSourceFeedback(
    const std::uint8_t derivativeOrder,
    const FeedbackSource source,
    const float target,
    const PDCluster& proportionalDerivativeCluster) const noexcept
{
    const float observed = Observe(derivativeOrder, source);
    const float observedDerivative = ObserveDerivative(derivativeOrder, source, observed);
    return
        ProportionalDerivativeFor(
            derivativeOrder,
            source,
            proportionalDerivativeCluster).Compute(
                (_rotational && (derivativeOrder == 0U)) ?
                    VehicleState::NormalizeAngle(target - observed) :
                    target - observed,
                -observedDerivative);
}

float MazeMap::FeedbackAxis::Observe(
    const std::uint8_t derivativeOrder,
    const FeedbackSource source) const noexcept
{
    const std::uint8_t order = derivativeOrder;
    const SensorSnapshot& snapshot = _state->GetSensorSnapshot();
    if (!_rotational)
    {
        if (source == FeedbackSource::State)
        {
            if (order == 0U)
            {
                const Eigen::Vector2f position = _state->GetPosition();
                const Eigen::Vector2f heading = _state->GetHeadingUnit();
                return position.dot(heading);
            }
            if (order == 1U)
            {
                return _state->GetVelocity();
            }
            return _state->GetLongitudinalAcceleration();
        }

        if (source == FeedbackSource::Imu)
        {
            if (order == 0U)
            {
                return 0.5f * (snapshot.leftEncoderDistanceM + snapshot.rightEncoderDistanceM);
            }
            if (order == 1U)
            {
                return _state->GetVelocity();
            }
            return snapshot.accelBodyYMps2;
        }

        if (source == FeedbackSource::Encoder)
        {
            if (order == 0U)
            {
                return 0.5f * (snapshot.leftEncoderDistanceM + snapshot.rightEncoderDistanceM);
            }
            if (order == 1U)
            {
                const MazeMap::EncoderObs& encoder = snapshot.encoderObservation;
                return
                    Vehicle::BodyForwardVelocityFromWheelLinear(
                        encoder.leftVelocityMps,
                        encoder.rightVelocityMps);
            }
            const MazeMap::EncoderObs& encoder = snapshot.encoderObservation;
            return
                SampleObservedDerivative(
                    1U,
                    source,
                    Vehicle::BodyForwardVelocityFromWheelLinear(
                        encoder.leftVelocityMps,
                        encoder.rightVelocityMps),
                    _state->GetLongitudinalAcceleration());
        }

        return _state->GetLongitudinalAcceleration();
    }

    if (source == FeedbackSource::State)
    {
        if (order == 0U)
        {
            return _state->GetOrientation();
        }
        if (order == 1U)
        {
            return _state->GetRotationalVelocity();
        }
        return _state->GetYawAcceleration();
    }

    if (source == FeedbackSource::Imu)
    {
        if (order == 0U)
        {
            return _state->GetOrientation();
        }
        if (order == 1U)
        {
            return snapshot.gyroRadps;
        }
        return
            SampleObservedDerivative(
                1U,
                source,
                snapshot.gyroRadps,
                _state->GetYawAcceleration());
    }

    if (source == FeedbackSource::Encoder)
    {
        if (order == 0U)
        {
            return
                Vehicle::BodyYawRateFromWheelLinear(
                    snapshot.leftEncoderDistanceM,
                    snapshot.rightEncoderDistanceM);
        }
        if (order == 1U)
        {
            const MazeMap::EncoderObs& encoder = snapshot.encoderObservation;
            return
                Vehicle::BodyYawRateFromWheelLinear(
                    encoder.leftVelocityMps,
                    encoder.rightVelocityMps);
        }
        const MazeMap::EncoderObs& encoder = snapshot.encoderObservation;
        return
            SampleObservedDerivative(
                1U,
                source,
                Vehicle::BodyYawRateFromWheelLinear(
                    encoder.leftVelocityMps,
                    encoder.rightVelocityMps),
                _state->GetYawAcceleration());
    }

    return _state->GetYawAcceleration();
}

float MazeMap::FeedbackAxis::ObserveDerivative(
    const std::uint8_t derivativeOrder,
    const FeedbackSource source,
    const float observed) const noexcept
{
    if (derivativeOrder < 2U)
    {
        return Observe(static_cast<std::uint8_t>(derivativeOrder + 1U), source);
    }

    return SampleObservedDerivative(derivativeOrder, source, observed, 0.0f);
}

float MazeMap::FeedbackAxis::SampleObservedDerivative(
    const std::uint8_t derivativeOrder,
    const FeedbackSource source,
    const float observed,
    const float firstSampleDerivative) const noexcept
{
    const std::uint8_t sourceIndex = SourceIndex(source);
    const std::uint8_t orderIndex =
        (derivativeOrder < 3U) ? derivativeOrder : 2U;
    const float nowSeconds = _state->GetTime();
    float observedDerivative =
        std::isfinite(firstSampleDerivative) ? firstSampleDerivative : 0.0f;
    const float previousObserved = _previousObserved[sourceIndex][orderIndex];
    const float previousTimeSeconds = _previousObservedTimeS[sourceIndex][orderIndex];
    if (std::isfinite(observed) &&
        std::isfinite(previousObserved) &&
        std::isfinite(nowSeconds) &&
        std::isfinite(previousTimeSeconds))
    {
        const float dtSeconds = nowSeconds - previousTimeSeconds;
        if (dtSeconds > kMinimumDerivativeDtSeconds)
        {
            observedDerivative = (observed - previousObserved) / dtSeconds;
        }
        else if (std::isfinite(_previousObservedDerivative[sourceIndex][orderIndex]))
        {
            observedDerivative = _previousObservedDerivative[sourceIndex][orderIndex];
        }
    }

    if (std::isfinite(observed) && std::isfinite(nowSeconds))
    {
        _previousObserved[sourceIndex][orderIndex] = observed;
        _previousObservedTimeS[sourceIndex][orderIndex] = nowSeconds;
        _previousObservedDerivative[sourceIndex][orderIndex] = observedDerivative;
    }

    return observedDerivative;
}

const MazeMap::ProportionalDerivative& MazeMap::FeedbackAxis::ProportionalDerivativeFor(
    const std::uint8_t derivativeOrder,
    const FeedbackSource source,
    const PDCluster& proportionalDerivativeCluster) const noexcept
{
    if (!_rotational)
    {
        if (derivativeOrder == 1U)
        {
            if (source == FeedbackSource::State)
            {
                return proportionalDerivativeCluster.VelocityStatePD;
            }
            if (source == FeedbackSource::Encoder)
            {
                return proportionalDerivativeCluster.VelocityEncoderAveragePD;
            }
        }

        if (derivativeOrder == 2U)
        {
            if (source == FeedbackSource::State)
            {
                return proportionalDerivativeCluster.LongitudinalAccelerationStatePD;
            }
            if (source == FeedbackSource::Imu)
            {
                return proportionalDerivativeCluster.LongitudinalAccelerationIMUForwardAccelPD;
            }
        }

        if (source == FeedbackSource::Imu)
        {
            return (derivativeOrder == 2U) ?
                proportionalDerivativeCluster.LongitudinalAccelerationIMUForwardAccelPD :
                proportionalDerivativeCluster.VelocityStatePD;
        }

        return (derivativeOrder == 2U) ?
            proportionalDerivativeCluster.LongitudinalAccelerationStatePD :
            proportionalDerivativeCluster.VelocityEncoderAveragePD;
    }

    if (derivativeOrder == 0U)
    {
        return proportionalDerivativeCluster.HeadingStatePD;
    }

    if (derivativeOrder == 1U)
    {
        if (source == FeedbackSource::State)
        {
            return proportionalDerivativeCluster.YawRateStatePD;
        }
        if (source == FeedbackSource::Imu)
        {
            return proportionalDerivativeCluster.YawRateGyroPD;
        }
        if (source == FeedbackSource::Encoder)
        {
            return proportionalDerivativeCluster.YawRateEncoderDeltaPD;
        }
    }

    if (source == FeedbackSource::Imu)
    {
        return proportionalDerivativeCluster.YawRateIMULateralAccelPD;
    }
    if (source == FeedbackSource::Encoder)
    {
        return proportionalDerivativeCluster.YawRateEncoderDeltaPD;
    }
    return proportionalDerivativeCluster.YawRateStatePD;
}
