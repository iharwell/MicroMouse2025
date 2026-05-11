#pragma once

#include <cstdint>

namespace MazeMap
{
    // Selects which closed-loop command contributors should be layered onto a generated drive command.
    //
    // `RawCommand` leaves the result as pure plant feedforward.
    // All other flags request one additional feedback objective. Multiple flags may be combined.
    enum class CommandPD : std::uint16_t
    {
        RawCommand = 0U,
        StateHeadingPD = 1U,
        StateYawPD = 2U,
        StateVelocityPD = 16U,
        StateAccelerationPD = 32U,
        EncoderVelocity = 64U,
        IMUYaw = 128U,
        IMUForwardAccel = 256U,
        IMULateralAccel = 512U
    };

    inline constexpr CommandPD operator|(CommandPD lhs, CommandPD rhs) noexcept
    {
        return static_cast<CommandPD>(
            static_cast<std::uint16_t>(lhs) |
            static_cast<std::uint16_t>(rhs));
    }

    inline constexpr CommandPD operator&(CommandPD lhs, CommandPD rhs) noexcept
    {
        return static_cast<CommandPD>(
            static_cast<std::uint16_t>(lhs) &
            static_cast<std::uint16_t>(rhs));
    }

    inline constexpr CommandPD& operator|=(CommandPD& lhs, CommandPD rhs) noexcept
    {
        lhs = lhs | rhs;
        return lhs;
    }

    inline constexpr bool HasCommandPD(CommandPD flags, CommandPD flag) noexcept
    {
        return
            (static_cast<std::uint16_t>(flags & flag) != 0U);
    }
}
