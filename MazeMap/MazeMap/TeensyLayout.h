#pragma once
#include "Pins.h"
#include "HardwareConfig.h"
#include "MazeMapSharedRuntime.h"

#include "StartupWaitProfile.h"

#ifdef ARDUINO_TEENSY41
#include "Defines.h"
#include <IntervalTimer.h>
#include <SPI.h>
#include <SD.h>

namespace MazeMap::Platform
{
    inline IntervalTimer gBuiltinLedBlinkTimer;
    inline volatile bool gBuiltinLedBlinkEnabled = false;
    inline volatile bool gBuiltinLedBlinkState = false;

    inline const char* GetSdioModeName(uint8_t mode)
    {
        return mode == DMA_SDIO ? "DMA_SDIO" : "FIFO_SDIO";
    }

    inline const char* GetVolumeTypeName(uint8_t fatType)
    {
        if (fatType == FAT_TYPE_EXFAT)
        {
            return "exFAT";
        }

        if (fatType != 0U)
        {
            switch (fatType)
            {
            case 12U:
                return "FAT12";
            case 16U:
                return "FAT16";
            case 32U:
                return "FAT32";
            default:
                return "FAT";
            }
        }

        return "none";
    }

    inline bool TryMountSdioVolume(uint8_t sdioMode, uint8_t part)
    {
        if (!SD.sdfs.cardBegin(SdioConfig(sdioMode)))
        {
            return false;
        }

        auto* card = SD.sdfs.card();
        if (card == nullptr)
        {
            return false;
        }

        if (static_cast<FsVolume&>(SD.sdfs).begin(card, true, part))
        {
            (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
                "SD mounted via %s %s%s",
                GetSdioModeName(sdioMode),
                part == 0U ? "sector 0 as " : "partition 1 as ",
                GetVolumeTypeName(SD.sdfs.fatType()));
            return true;
        }

        return false;
    }

    inline bool TryMountPreferredSdVolume()
    {
        // The production runtime now uses one mounted SD filesystem for logging.txt, primary .mmlog,
        // and .sidecar files. Keep the shared logging path on FIFO SDIO so service writes
        // remain synchronous 512-byte sectors instead of DMA transfers with a separate busy window.
        return TryMountSdioVolume(FIFO_SDIO, 1U) ||
               TryMountSdioVolume(FIFO_SDIO, 0U);
    }

    inline void UpdateBuiltinLedBlink()
    {
        if (!gBuiltinLedBlinkEnabled)
        {
            return;
        }

        gBuiltinLedBlinkState = !gBuiltinLedBlinkState;
        digitalWriteFast(LED_BUILTIN, gBuiltinLedBlinkState ? HIGH : LOW);
    }

    inline void StartBuiltinLedBlink(unsigned long blinkPeriodMs)
    {
        if (blinkPeriodMs < 2UL)
        {
            return;
        }

        gBuiltinLedBlinkTimer.end();
        pinMode(LED_BUILTIN, OUTPUT);
        gBuiltinLedBlinkState = MazeMap::IsStartupWaitIndicatorEnabled(0UL, blinkPeriodMs);
        gBuiltinLedBlinkEnabled = true;
        digitalWriteFast(LED_BUILTIN, gBuiltinLedBlinkState ? HIGH : LOW);
        const unsigned long halfPeriodUs = blinkPeriodMs * 1000UL / 2UL;
        gBuiltinLedBlinkTimer.begin(UpdateBuiltinLedBlink, halfPeriodUs);
    }

    inline void StartStartupWaitIndicatorBlink()
    {
        StartBuiltinLedBlink(HardwareConfig::kSdWaitBlinkPeriodMs);
    }

    inline void StartRuntimeFaultIndicatorBlink()
    {
        StartBuiltinLedBlink(HardwareConfig::kFaultIndicatorBlinkPeriodMs);
    }

    inline void StopBuiltinLedBlink()
    {
        gBuiltinLedBlinkTimer.end();
        gBuiltinLedBlinkEnabled = false;
        gBuiltinLedBlinkState = false;
        digitalWriteFast(LED_BUILTIN, LOW);
    }
}

inline void StartRuntimeFaultIndicatorBlink()
{
    MazeMap::Platform::StartRuntimeFaultIndicatorBlink();
}

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

inline bool SetupHardware()
{
    // ADC / PWM global resolution setup.
    analogReadResolution(HardwareConfig::kAdcBits);
    analogWriteResolution(HardwareConfig::kPwmBits);

    // Main motor outputs.
    // Default both DRV8871 inputs low so the outputs start inactive.
    MazeMap::Platform::ConfigureMotorPwmPin(Pins::R_MotorA);
    MazeMap::Platform::ConfigureMotorPwmPin(Pins::R_MotorB);
    MazeMap::Platform::ConfigureMotorPwmPin(Pins::L_MotorA);
    MazeMap::Platform::ConfigureMotorPwmPin(Pins::L_MotorB);

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

    pinMode(LED_BUILTIN, OUTPUT);
    MazeMap::Platform::StopBuiltinLedBlink();

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
    MazeMap::Platform::ConfigureEncoder(1U, Pins::R_EncA, Pins::R_EncB);
    MazeMap::Platform::WriteEncoderCount(1U, 0);

    MazeMap::Platform::ConfigureEncoder(2U, Pins::L_EncA, Pins::L_EncB);
    MazeMap::Platform::WriteEncoderCount(2U, 0);

    // Built-in SD card on Teensy 4.1 uses native SDIO, not SPI. If the card is missing, stay in a visible
    // waiting state instead of failing startup so the operator can insert the card or use this as a manual delay.
    // Do not attempt text logging until a filesystem is actually mounted; the SD path cannot report its own
    // failures before that point, and such attempts would permanently fault the runtime text log state.
    unsigned long waitStartMs = millis();
    bool waitingForSd = false;
    while (true)
    {
        if (MazeMap::Platform::TryMountPreferredSdVolume())
        {
            if (waitingForSd)
            {
                (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
                    "SD init recovered after waiting %lu ms",
                    static_cast<unsigned long>(millis() - waitStartMs));
            }
            MazeMap::Platform::StopBuiltinLedBlink();
            return true;
        }

        if (!waitingForSd)
        {
            waitingForSd = true;
            waitStartMs = millis();
            MazeMap::Platform::StartStartupWaitIndicatorBlink();
        }

        delay(HardwareConfig::kSdInitRetryDelayMs);
    }

}
#else
inline void StartRuntimeFaultIndicatorBlink()
{
}
#endif
