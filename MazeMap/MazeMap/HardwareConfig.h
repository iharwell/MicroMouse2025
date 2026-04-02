#pragma once
// Declares hardware timing, PWM, ADC, and SD bring-up constants shared across the runtime and board support layer.

#include "Defines.h"

#define MAZEMAP_HARDWARE_CONFIG_NAMESPACE_AVAILABLE 1

namespace MazeMap::HardwareConfig
{
    constexpr uint32_t kMotorPwmFrequencyHz = 80000U;
    constexpr uint32_t kFanPwmFrequencyHz = 80000U;

    constexpr uint8_t kPwmBits = 12U;
    constexpr uint8_t kAdcBits = 12U;

    // Front wall sensors use log-amp front ends and need a longer settle after LED transitions.
    constexpr uint32_t kFrontWallSensorSwitchSettleTime_us = 60U;
    // Side wall sensors settle much faster and are trimmed separately on the scope.
    constexpr uint32_t kSideWallSensorSwitchSettleTime_us = 30U;

    // Keep retrying SD init indefinitely so the operator can insert the card after power-up.
    constexpr uint16_t kSdInitRetryDelayMs = 150U;
    constexpr uint16_t kSdWaitBlinkPeriodMs = 1000U;
}

namespace HardwareConfig = MazeMap::HardwareConfig;
