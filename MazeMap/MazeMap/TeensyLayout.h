#pragma once

#ifdef ARDUINO_TEENSY41
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <QuadEncoder.h>

namespace Pins
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

namespace HardwareConfig
{
    constexpr uint32_t kMotorPwmFrequencyHz = 20000U;
    constexpr uint32_t kFanPwmFrequencyHz = 20000U;

    constexpr uint8_t kPwmBits = 12U;
    constexpr uint8_t kAdcBits = 12U;

    constexpr uint32_t kWallSensorLedTurnOnTime_us = 3U;
    constexpr uint32_t kWallSensorAnalogSettleTime_us = 2U;
}

// Hardware quadrature encoder channels.
// Channel numbers must be unique in the range supported by the library.
QuadEncoder g_rightEncoder(1, Pins::R_EncA, Pins::R_EncB, 0);
QuadEncoder g_leftEncoder(2, Pins::L_EncA, Pins::L_EncB, 0);

static void ConfigurePwmOutput(uint8_t pin, uint32_t frequencyHz)
{
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, frequencyHz);
    analogWrite(pin, 0);
}

static void ConfigureDigitalOutputLow(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

bool SetupHardware()
{
    // ADC / PWM global resolution setup.
    analogReadResolution(HardwareConfig::kAdcBits);
    analogWriteResolution(HardwareConfig::kPwmBits);

    // Main motor outputs.
    // Default both DRV8871 inputs low so the outputs start inactive.
    ConfigurePwmOutput(Pins::R_MotorA, HardwareConfig::kMotorPwmFrequencyHz);
    ConfigurePwmOutput(Pins::R_MotorB, HardwareConfig::kMotorPwmFrequencyHz);
    ConfigurePwmOutput(Pins::L_MotorA, HardwareConfig::kMotorPwmFrequencyHz);
    ConfigurePwmOutput(Pins::L_MotorB, HardwareConfig::kMotorPwmFrequencyHz);

    // Vacuum fan control.
    ConfigurePwmOutput(Pins::Fan_CTRL, HardwareConfig::kFanPwmFrequencyHz);

    // IMU SPI chip selects idle high.
    pinMode(Pins::IMU_CS_A, OUTPUT);
    pinMode(Pins::IMU_CS_B, OUTPUT);
    digitalWrite(Pins::IMU_CS_A, HIGH);
    digitalWrite(Pins::IMU_CS_B, HIGH);

    // IMU interrupt pins.
    pinMode(Pins::IMU_INT_1A, INPUT);
    pinMode(Pins::IMU_INT_1B, INPUT);

    // Start SPI on the default hardware SPI pins: MOSI=11, MISO=12, SCK=13.
    SPI.begin();

    // Wall sensor analog inputs.
    pinMode(Pins::WS_Forward_Right, INPUT);
    pinMode(Pins::WS_Forward_Left, INPUT);
    pinMode(Pins::WS_Side_Right, INPUT);
    pinMode(Pins::WS_Side_Left, INPUT);

    // Wall sensor LED control outputs default off.
    ConfigureDigitalOutputLow(Pins::LED_Ctrl_Forward_Right);
    ConfigureDigitalOutputLow(Pins::LED_Ctrl_Forward_Left);
    ConfigureDigitalOutputLow(Pins::LED_Ctrl_Side_Right);
    ConfigureDigitalOutputLow(Pins::LED_Ctrl_Side_Left);

    // Hardware quadrature encoder setup.
    g_rightEncoder.setInitConfig();
    g_rightEncoder.init();
    g_rightEncoder.write(0);

    g_leftEncoder.setInitConfig();
    g_leftEncoder.init();
    g_leftEncoder.write(0);

    // Built-in SD card on Teensy 4.1 uses native SDIO, not SPI.
    if (!SD.begin(BUILTIN_SDCARD))
    {
        return false;
    }

    return true;
}
#endif
