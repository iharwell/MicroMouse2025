#pragma once
// Declares the board-level pin assignments shared by Teensy bring-up code and host-side runtime configuration.

#include "Defines.h"

#define MAZEMAP_PINS_NAMESPACE_AVAILABLE 1

namespace MazeMap::Pins
{
    constexpr uint8_t R_MotorA = 5;
    constexpr uint8_t R_MotorB = 6;
    constexpr uint8_t R_EncA = 7;
    constexpr uint8_t R_EncB = 8;

    constexpr uint8_t L_MotorA = 24;
    constexpr uint8_t L_MotorB = 25;
    constexpr uint8_t L_EncA = 2;
    constexpr uint8_t L_EncB = 3;

    constexpr uint8_t Fan_CTRL = 4;

    constexpr uint8_t IMU_CS_A = 36;
    constexpr uint8_t IMU_CS_B = 37;
    constexpr uint8_t IMU_INT_1A = 32;
    constexpr uint8_t IMU_INT_1B = 33;

    constexpr uint8_t WS_Forward_Right = 23;
    constexpr uint8_t LED_Ctrl_Forward_Right = 19;

    constexpr uint8_t WS_Forward_Left = 22;
    constexpr uint8_t LED_Ctrl_Forward_Left = 18;

    constexpr uint8_t WS_Side_Right = 21;
    constexpr uint8_t LED_Ctrl_Side_Right = 17;

    constexpr uint8_t WS_Side_Left = 20;
    constexpr uint8_t LED_Ctrl_Side_Left = 16;
}

namespace Pins = MazeMap::Pins;
