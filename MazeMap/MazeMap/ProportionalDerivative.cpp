#include "pch.h"
#include "ProportionalDerivative.h"

#include <cmath>

namespace
{
    constexpr float kMinimumDerivativeDtSeconds = 1.0e-6f;

    constexpr MazeMap::ProportionalDerivative kConstexprExample =
        MazeMap::ProportionalDerivative(2.5f, 0.125f);

    static_assert(
        kConstexprExample.GetProportionalGain() == 2.5f,
        "ProportionalDerivative must remain constexpr-constructible.");
    static_assert(
        kConstexprExample.GetDerivativeGain() == 0.125f,
        "ProportionalDerivative getters must remain constexpr-safe.");
}

float MazeMap::ProportionalDerivative::GetProportionalGain() noexcept
{
    return _proportionalGain;
}

float MazeMap::ProportionalDerivative::GetDerivativeGain() noexcept
{
    return _derivativeGain;
}

void MazeMap::ProportionalDerivative::SetProportionalGain(const float proportionalGain) noexcept
{
    _proportionalGain = proportionalGain;
}

void MazeMap::ProportionalDerivative::SetDerivativeGain(const float derivativeGain) noexcept
{
    _derivativeGain = derivativeGain;
}

void MazeMap::ProportionalDerivative::SetGains(
    const float proportionalGain,
    const float derivativeGain) noexcept
{
    _proportionalGain = proportionalGain;
    _derivativeGain = derivativeGain;
}

void MazeMap::ProportionalDerivative::ResetDerivativeHistory() noexcept
{
    _previousErrorForDerivative = std::numeric_limits<float>::quiet_NaN();
}

bool MazeMap::ProportionalDerivative::IsValid() noexcept
{
    return static_cast<const ProportionalDerivative&>(*this).IsValid();
}

bool MazeMap::ProportionalDerivative::IsValid() const noexcept
{
    return
        std::isfinite(_proportionalGain) &&
        std::isfinite(_derivativeGain) &&
        (_proportionalGain >= 0.0f) &&
        (_derivativeGain >= 0.0f);
}

float MazeMap::ProportionalDerivative::Compute(
    const float error,
    const float errorRate) noexcept
{
    return static_cast<const ProportionalDerivative&>(*this).Compute(error, errorRate);
}

float MazeMap::ProportionalDerivative::Compute(
    const float error,
    const float errorRate) const noexcept
{
    if (!IsValid() || !std::isfinite(error) || !std::isfinite(errorRate))
    {
        return 0.0f;
    }

    return (_proportionalGain * error) + (_derivativeGain * errorRate);
}

float MazeMap::ProportionalDerivative::ComputeFromErrorSample(
    const float error,
    const float dtSeconds) noexcept
{
    if (!IsValid() || !std::isfinite(error) || !std::isfinite(dtSeconds))
    {
        ResetDerivativeHistory();
        return 0.0f;
    }

    const float proportionalTerm = _proportionalGain * error;
    if (!std::isfinite(_previousErrorForDerivative))
    {
        _previousErrorForDerivative = error;
        return proportionalTerm;
    }

    if (dtSeconds <= kMinimumDerivativeDtSeconds)
    {
        _previousErrorForDerivative = error;
        return proportionalTerm;
    }

    const float errorRate = (error - _previousErrorForDerivative) / dtSeconds;
    _previousErrorForDerivative = error;
    return proportionalTerm + (_derivativeGain * errorRate);
}

float MazeMap::ProportionalDerivative::ComputeFromMeasurementRate(
    const float error,
    const float measurementRate) noexcept
{
    return static_cast<const ProportionalDerivative&>(*this).ComputeFromMeasurementRate(error, measurementRate);
}

float MazeMap::ProportionalDerivative::ComputeFromMeasurementRate(
    const float error,
    const float measurementRate) const noexcept
{
    if (!IsValid() || !std::isfinite(error) || !std::isfinite(measurementRate))
    {
        return 0.0f;
    }

    return (_proportionalGain * error) - (_derivativeGain * measurementRate);
}
