#pragma once

#include <cstdint>

namespace MazeMap
{
    // Exact wheel-encoder measurement contract consumed by the estimator.
    class EncoderObs final
    {
    public:
        constexpr EncoderObs() noexcept = default;

        constexpr EncoderObs(
            std::int32_t totalLeftCounts,
            std::int32_t totalRightCounts,
            float leftDistanceDeltaM,
            float rightDistanceDeltaM,
            float leftVelocityMps,
            float rightVelocityMps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps) noexcept
            : _totalLeftCounts(totalLeftCounts)
            , _totalRightCounts(totalRightCounts)
            , _leftDistanceDeltaM(leftDistanceDeltaM)
            , _rightDistanceDeltaM(rightDistanceDeltaM)
            , _leftVelocityMps(leftVelocityMps)
            , _rightVelocityMps(rightVelocityMps)
            , _leftWheelSpeedRadps(leftWheelSpeedRadps)
            , _rightWheelSpeedRadps(rightWheelSpeedRadps)
        {
        }

        constexpr std::int32_t TotalLeftCounts() const noexcept { return _totalLeftCounts; }
        constexpr std::int32_t TotalRightCounts() const noexcept { return _totalRightCounts; }
        constexpr float LeftDistanceDeltaM() const noexcept { return _leftDistanceDeltaM; }
        constexpr float RightDistanceDeltaM() const noexcept { return _rightDistanceDeltaM; }
        constexpr float LeftVelocityMps() const noexcept { return _leftVelocityMps; }
        constexpr float RightVelocityMps() const noexcept { return _rightVelocityMps; }
        constexpr float LeftWheelSpeedRadps() const noexcept { return _leftWheelSpeedRadps; }
        constexpr float RightWheelSpeedRadps() const noexcept { return _rightWheelSpeedRadps; }

        constexpr void SetTotalLeftCounts(std::int32_t counts) noexcept { _totalLeftCounts = counts; }
        constexpr void SetTotalRightCounts(std::int32_t counts) noexcept { _totalRightCounts = counts; }
        constexpr void SetLeftDistanceDeltaM(float distanceM) noexcept { _leftDistanceDeltaM = distanceM; }
        constexpr void SetRightDistanceDeltaM(float distanceM) noexcept { _rightDistanceDeltaM = distanceM; }
        constexpr void SetLeftVelocityMps(float velocityMps) noexcept { _leftVelocityMps = velocityMps; }
        constexpr void SetRightVelocityMps(float velocityMps) noexcept { _rightVelocityMps = velocityMps; }
        constexpr void SetLeftWheelSpeedRadps(float wheelSpeedRadps) noexcept { _leftWheelSpeedRadps = wheelSpeedRadps; }
        constexpr void SetRightWheelSpeedRadps(float wheelSpeedRadps) noexcept { _rightWheelSpeedRadps = wheelSpeedRadps; }

        constexpr void SetCounts(std::int32_t leftCounts, std::int32_t rightCounts) noexcept
        {
            _totalLeftCounts = leftCounts;
            _totalRightCounts = rightCounts;
        }

        constexpr void SetDistanceDeltasM(float leftDistanceM, float rightDistanceM) noexcept
        {
            _leftDistanceDeltaM = leftDistanceM;
            _rightDistanceDeltaM = rightDistanceM;
        }

        constexpr void SetWheelLinearVelocityMps(float leftVelocityMps, float rightVelocityMps) noexcept
        {
            _leftVelocityMps = leftVelocityMps;
            _rightVelocityMps = rightVelocityMps;
        }

        constexpr void SetWheelSpeedRadps(float leftWheelSpeedRadps, float rightWheelSpeedRadps) noexcept
        {
            _leftWheelSpeedRadps = leftWheelSpeedRadps;
            _rightWheelSpeedRadps = rightWheelSpeedRadps;
        }

    private:
        std::int32_t _totalLeftCounts = 0;
        std::int32_t _totalRightCounts = 0;
        float _leftDistanceDeltaM = 0.0f;
        float _rightDistanceDeltaM = 0.0f;
        float _leftVelocityMps = 0.0f;
        float _rightVelocityMps = 0.0f;
        float _leftWheelSpeedRadps = 0.0f;
        float _rightWheelSpeedRadps = 0.0f;
    };
}
