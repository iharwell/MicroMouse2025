#pragma once

#include <stdint.h>

enum class CalibrationWall : uint8_t
{
    West,
    East,
    South,
    North
};

enum class WallTouchOutcome : uint8_t
{
    SeatedContact,
    PassedThroughNoWall
};
