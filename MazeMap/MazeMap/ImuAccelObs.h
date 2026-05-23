#pragma once

namespace MazeMap
{
    // Planar accelerometer measurement contract consumed by the estimator.
    class ImuAccelObs final
    {
    public:
        constexpr ImuAccelObs() noexcept = default;

        constexpr ImuAccelObs(
            bool valid,
            float accelBodyForwardMps2,
            float accelBodyRightMps2) noexcept
            : _valid(valid)
            , _accelBodyForwardMps2(accelBodyForwardMps2)
            , _accelBodyRightMps2(accelBodyRightMps2)
        {
        }

        constexpr bool IsValid() const noexcept { return _valid; }
        constexpr float AccelBodyForwardMps2() const noexcept { return _accelBodyForwardMps2; }
        constexpr float AccelBodyRightMps2() const noexcept { return _accelBodyRightMps2; }

        constexpr void SetValid(bool valid) noexcept { _valid = valid; }

        constexpr void SetBodyForwardRightMps2(
            float accelBodyForwardMps2,
            float accelBodyRightMps2) noexcept
        {
            _accelBodyForwardMps2 = accelBodyForwardMps2;
            _accelBodyRightMps2 = accelBodyRightMps2;
        }

    private:
        bool _valid = false;
        float _accelBodyForwardMps2 = 0.0f;
        float _accelBodyRightMps2 = 0.0f;
    };
}
