#pragma once

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    struct OpenLoopDriveCommand
    {
        float leftDriveCommand = 0.0f;
        float rightDriveCommand = 0.0f;
    };

    inline constexpr OpenLoopDriveCommand MakeOpenLoopDriveCommand(float leftDriveCommand, float rightDriveCommand) noexcept
    {
        return OpenLoopDriveCommand{ leftDriveCommand, rightDriveCommand };
    }

    inline constexpr OpenLoopDriveCommand MakeSymmetricOpenLoopDriveCommand(float driveCommand) noexcept
    {
        return MakeOpenLoopDriveCommand(driveCommand, driveCommand);
    }

    inline constexpr OpenLoopDriveCommand MakeDifferentialOpenLoopDriveCommand(float forwardDriveCommand, float turnDriveCommand) noexcept
    {
        return MakeOpenLoopDriveCommand(
            forwardDriveCommand - turnDriveCommand,
            forwardDriveCommand + turnDriveCommand);
    }

    inline bool IsFiniteOpenLoopDriveCommand(const OpenLoopDriveCommand& command) noexcept
    {
        return std::isfinite(command.leftDriveCommand) && std::isfinite(command.rightDriveCommand);
    }

    inline OpenLoopDriveCommand ClampOpenLoopDriveCommand(const OpenLoopDriveCommand& command) noexcept
    {
        if (!IsFiniteOpenLoopDriveCommand(command))
        {
            return {};
        }

        return OpenLoopDriveCommand{
            (std::clamp)(command.leftDriveCommand, -1.0f, 1.0f),
            (std::clamp)(command.rightDriveCommand, -1.0f, 1.0f)
        };
    }

    inline OpenLoopDriveCommand ComputeOpenLoopYawWiggleCommand(
        float forwardDriveCommand,
        unsigned long elapsedMs,
        unsigned long wiggleHalfPeriodMs,
        float wiggleTurnFraction,
        float minimumRetainedForwardFraction = 0.0f) noexcept
    {
        const float clampedForwardDriveCommand =
            std::isfinite(forwardDriveCommand) ?
            (std::clamp)(forwardDriveCommand, -1.0f, 1.0f) :
            0.0f;
        if (!(std::isfinite(wiggleTurnFraction) && wiggleTurnFraction > 0.0f) ||
            wiggleHalfPeriodMs == 0UL)
        {
            return MakeSymmetricOpenLoopDriveCommand(clampedForwardDriveCommand);
        }

        const float clampedRetainedForwardFraction =
            std::isfinite(minimumRetainedForwardFraction) ?
            (std::clamp)(minimumRetainedForwardFraction, 0.0f, 1.0f) :
            0.0f;
        const float clampedTurnFraction = (std::clamp)(wiggleTurnFraction, 0.0f, 1.0f);
        const bool biasRight = ((elapsedMs / wiggleHalfPeriodMs) & 1UL) == 0UL;
        const float forwardSign = (clampedForwardDriveCommand >= 0.0f) ? 1.0f : -1.0f;
        const float forwardMagnitude = std::fabs(clampedForwardDriveCommand);
        const float turnMagnitude = clampedTurnFraction * forwardMagnitude;
        const float minimumRetainedMagnitude = clampedRetainedForwardFraction * forwardMagnitude;
        const float lowMagnitude = (std::max)(forwardMagnitude - turnMagnitude, minimumRetainedMagnitude);
        const float highMagnitude = (std::min)(1.0f, lowMagnitude + (2.0f * turnMagnitude));
        const float leftMagnitude = biasRight ? lowMagnitude : highMagnitude;
        const float rightMagnitude = biasRight ? highMagnitude : lowMagnitude;
        return ClampOpenLoopDriveCommand(
            MakeOpenLoopDriveCommand(
                forwardSign * leftMagnitude,
                forwardSign * rightMagnitude));
    }

    inline OpenLoopDriveCommand ComputeOpenLoopYawDitherCommand(
        float forwardDriveCommand,
        unsigned long elapsedMs,
        unsigned long ditherHalfPeriodMs,
        unsigned long signBlendMs,
        float ditherTurnFraction,
        float minimumRetainedForwardFraction = 0.0f) noexcept
    {
        const float clampedForwardDriveCommand =
            std::isfinite(forwardDriveCommand) ?
            (std::clamp)(forwardDriveCommand, -1.0f, 1.0f) :
            0.0f;
        if (!(std::isfinite(ditherTurnFraction) && ditherTurnFraction > 0.0f) ||
            ditherHalfPeriodMs == 0UL)
        {
            return MakeSymmetricOpenLoopDriveCommand(clampedForwardDriveCommand);
        }

        const float clampedRetainedForwardFraction =
            std::isfinite(minimumRetainedForwardFraction) ?
            (std::clamp)(minimumRetainedForwardFraction, 0.0f, 1.0f) :
            0.0f;
        const float clampedTurnFraction = (std::clamp)(ditherTurnFraction, 0.0f, 1.0f);
        const unsigned long halfCycleIndex = elapsedMs / ditherHalfPeriodMs;
        const unsigned long phaseMs = elapsedMs % ditherHalfPeriodMs;
        const bool biasRight = (halfCycleIndex & 1UL) == 0UL;
        float signedTurnFraction = biasRight ? clampedTurnFraction : -clampedTurnFraction;

        if ((signBlendMs > 0UL) && (phaseMs < signBlendMs) && (elapsedMs >= ditherHalfPeriodMs))
        {
            const float blendAlpha =
                static_cast<float>(phaseMs) /
                static_cast<float>((std::max)(signBlendMs, 1UL));
            const float previousSignedTurnFraction = -signedTurnFraction;
            signedTurnFraction =
                previousSignedTurnFraction +
                ((signedTurnFraction - previousSignedTurnFraction) * blendAlpha);
        }

        const float forwardSign = (clampedForwardDriveCommand >= 0.0f) ? 1.0f : -1.0f;
        const float forwardMagnitude = std::fabs(clampedForwardDriveCommand);
        const float turnMagnitude = std::fabs(signedTurnFraction) * forwardMagnitude;
        const float minimumRetainedMagnitude = clampedRetainedForwardFraction * forwardMagnitude;
        const float lowMagnitude = (std::max)(forwardMagnitude - turnMagnitude, minimumRetainedMagnitude);
        const float highMagnitude = (std::min)(1.0f, lowMagnitude + (2.0f * turnMagnitude));
        const float blendSign = (signedTurnFraction >= 0.0f) ? 1.0f : -1.0f;
        const float leftMagnitude = (blendSign >= 0.0f) ? lowMagnitude : highMagnitude;
        const float rightMagnitude = (blendSign >= 0.0f) ? highMagnitude : lowMagnitude;
        return ClampOpenLoopDriveCommand(
            MakeOpenLoopDriveCommand(
                forwardSign * leftMagnitude,
                forwardSign * rightMagnitude));
    }
}
