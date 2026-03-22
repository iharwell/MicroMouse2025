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
}
