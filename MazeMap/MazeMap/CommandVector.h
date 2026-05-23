#pragma once

#include "Defines.h"

#include <cmath>
#include <limits>

namespace MazeMap::App::Internal
{
    // Left/right normalized actuator command active for the state interval being calculated.
    // Producers such as Drive may return one as the caller's proposal for that interval.
    class EXPORT CommandVector final
    {
    public:
        CommandVector() = default;

        constexpr CommandVector(float leftCommand, float rightCommand) noexcept
            : _leftCommand(leftCommand)
            , _rightCommand(rightCommand)
        {}

        static constexpr CommandVector Brake() noexcept
        {
            return CommandVector(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN());
        }

        static constexpr CommandVector FromAverageAndDifferential(
            float averageCommand,
            float differentialCommand) noexcept
        {
            return CommandVector(
                averageCommand + differentialCommand,
                averageCommand - differentialCommand);
        }

        constexpr float LeftCommand() const noexcept { return _leftCommand; }
        constexpr float RightCommand() const noexcept { return _rightCommand; }

        void SetLeftCommand(float leftCommand) noexcept { _leftCommand = leftCommand; }
        void SetRightCommand(float rightCommand) noexcept { _rightCommand = rightCommand; }

        constexpr float Average() const noexcept { return 0.5f * (_leftCommand + _rightCommand); }
        constexpr float Differential() const noexcept { return 0.5f * (_leftCommand - _rightCommand); }

        void SetAverage(float averageCommand) noexcept
        {
            SetAverageAndDifferential(averageCommand, Differential());
        }

        void SetDifferential(float differentialCommand) noexcept
        {
            SetAverageAndDifferential(Average(), differentialCommand);
        }

        void SetAverageAndDifferential(float averageCommand, float differentialCommand) noexcept
        {
            _leftCommand = averageCommand + differentialCommand;
            _rightCommand = averageCommand - differentialCommand;
        }

        void ClampCommand() noexcept
        {
			if (_leftCommand > 1.0f)
			{
				_leftCommand = 1.0f;
			}
			else if (_leftCommand < -1.0f)
			{
				_leftCommand = -1.0f;
			}
			if (_rightCommand > 1.0f)
			{
				_rightCommand = 1.0f;
			}
			else if (_rightCommand < -1.0f)
			{
				_rightCommand = -1.0f;
			}
        }

        bool IsFinite() const noexcept
        {
            return std::isfinite(_leftCommand) && std::isfinite(_rightCommand);
        }

        CommandVector& operator+=(const CommandVector& rhs) noexcept
        {
            _leftCommand += rhs._leftCommand;
            _rightCommand += rhs._rightCommand;
            return *this;
        }

        CommandVector& operator-=(const CommandVector& rhs) noexcept
        {
            _leftCommand -= rhs._leftCommand;
            _rightCommand -= rhs._rightCommand;
            return *this;
        }

        CommandVector& operator*=(float scalar) noexcept
        {
            _leftCommand *= scalar;
            _rightCommand *= scalar;
            return *this;
        }

        CommandVector& operator/=(float scalar) noexcept
        {
            _leftCommand /= scalar;
            _rightCommand /= scalar;
            return *this;
        }

        constexpr CommandVector operator-() const noexcept
        {
            return CommandVector(-_leftCommand, -_rightCommand);
        }

        friend constexpr bool operator==(const CommandVector& lhs, const CommandVector& rhs) noexcept
        {
            return
                (lhs._leftCommand == rhs._leftCommand) &&
                (lhs._rightCommand == rhs._rightCommand);
        }

        friend constexpr bool operator!=(const CommandVector& lhs, const CommandVector& rhs) noexcept
        {
            return !(lhs == rhs);
        }
    private:
        float _leftCommand{};
        float _rightCommand{};
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
