#pragma once

#include "Defines.h"

#include <cmath>
#include <limits>

namespace MazeMap::App::Internal
{
    // Left/right raw motor-PWM command proposal for the next control tick.
    class EXPORT CommandVector final
    {
    public:
        CommandVector() = default;

        constexpr CommandVector(float leftMotorPwm, float rightMotorPwm) noexcept
            : _leftMotorPwm(leftMotorPwm)
            , _rightMotorPwm(rightMotorPwm)
        {}

        static constexpr CommandVector Brake() noexcept
        {
            return CommandVector(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN());
        }

        static constexpr CommandVector FromAverageAndDifferential(
            float averageMotorPwm,
            float differentialMotorPwm) noexcept
        {
            return CommandVector(
                averageMotorPwm + differentialMotorPwm,
                averageMotorPwm - differentialMotorPwm);
        }

        constexpr float LeftMotorPwm() const noexcept { return _leftMotorPwm; }
        constexpr float RightMotorPwm() const noexcept { return _rightMotorPwm; }

        void SetLeftMotorPwm(float leftMotorPwm) noexcept { _leftMotorPwm = leftMotorPwm; }
        void SetRightMotorPwm(float rightMotorPwm) noexcept { _rightMotorPwm = rightMotorPwm; }

        constexpr float Average() const noexcept { return 0.5f * (_leftMotorPwm + _rightMotorPwm); }
        constexpr float Differential() const noexcept { return 0.5f * (_leftMotorPwm - _rightMotorPwm); }

        void SetAverage(float averageMotorPwm) noexcept
        {
            SetAverageAndDifferential(averageMotorPwm, Differential());
        }

        void SetDifferential(float differentialMotorPwm) noexcept
        {
            SetAverageAndDifferential(Average(), differentialMotorPwm);
        }

        void SetAverageAndDifferential(float averageMotorPwm, float differentialMotorPwm) noexcept
        {
            _leftMotorPwm = averageMotorPwm + differentialMotorPwm;
            _rightMotorPwm = averageMotorPwm - differentialMotorPwm;
        }

        bool IsFinite() const noexcept
        {
            return std::isfinite(_leftMotorPwm) && std::isfinite(_rightMotorPwm);
        }

        CommandVector& operator+=(const CommandVector& rhs) noexcept
        {
            _leftMotorPwm += rhs._leftMotorPwm;
            _rightMotorPwm += rhs._rightMotorPwm;
            return *this;
        }

        CommandVector& operator-=(const CommandVector& rhs) noexcept
        {
            _leftMotorPwm -= rhs._leftMotorPwm;
            _rightMotorPwm -= rhs._rightMotorPwm;
            return *this;
        }

        CommandVector& operator*=(float scalar) noexcept
        {
            _leftMotorPwm *= scalar;
            _rightMotorPwm *= scalar;
            return *this;
        }

        CommandVector& operator/=(float scalar) noexcept
        {
            _leftMotorPwm /= scalar;
            _rightMotorPwm /= scalar;
            return *this;
        }

        constexpr CommandVector operator-() const noexcept
        {
            return CommandVector(-_leftMotorPwm, -_rightMotorPwm);
        }

        friend constexpr bool operator==(const CommandVector& lhs, const CommandVector& rhs) noexcept
        {
            return
                (lhs._leftMotorPwm == rhs._leftMotorPwm) &&
                (lhs._rightMotorPwm == rhs._rightMotorPwm);
        }

        friend constexpr bool operator!=(const CommandVector& lhs, const CommandVector& rhs) noexcept
        {
            return !(lhs == rhs);
        }
    private:
        float _leftMotorPwm{};
        float _rightMotorPwm{};
    };

    inline CommandVector operator+(CommandVector lhs, const CommandVector& rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }

    inline CommandVector operator-(CommandVector lhs, const CommandVector& rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }

    inline CommandVector operator*(CommandVector lhs, float scalar) noexcept
    {
        lhs *= scalar;
        return lhs;
    }

    inline CommandVector operator*(float scalar, CommandVector rhs) noexcept
    {
        rhs *= scalar;
        return rhs;
    }

    inline CommandVector operator/(CommandVector lhs, float scalar) noexcept
    {
        lhs /= scalar;
        return lhs;
    }
}
