#pragma once
#include "Pins.h"
#include "HardwareConfig.h"

#include "StartupWaitProfile.h"

#ifdef ARDUINO_TEENSY41
#include "Defines.h"
#include <IntervalTimer.h>
#include <SPI.h>
#include <SD.h>

namespace
{
    constexpr uint16_t kBootSectorSignatureOffset = 510U;
    constexpr uint16_t kPartitionTableOffset = 446U;
    constexpr uint8_t kPartitionEntrySize = 16U;

    IntervalTimer gStartupWaitBlinkTimer;
    volatile bool gStartupWaitIndicatorEnabled = false;
    volatile bool gStartupWaitIndicatorState = false;

    uint32_t ReadLe32(const uint8_t* bytes)
    {
        return static_cast<uint32_t>(bytes[0]) |
               (static_cast<uint32_t>(bytes[1]) << 8U) |
               (static_cast<uint32_t>(bytes[2]) << 16U) |
               (static_cast<uint32_t>(bytes[3]) << 24U);
    }

    const char* GetSdCardTypeName(uint8_t type)
    {
        switch (type)
        {
        case SD_CARD_TYPE_SD1:
            return "SD1";
        case SD_CARD_TYPE_SD2:
            return "SD2";
        case SD_CARD_TYPE_SDHC:
            return "SDHC/SDXC";
        default:
            return "unknown";
        }
    }

    const char* GetSdioModeName(uint8_t mode)
    {
        return mode == DMA_SDIO ? "DMA_SDIO" : "FIFO_SDIO";
    }

    void PrintVolumeType(uint8_t fatType)
    {
        if (fatType == FAT_TYPE_EXFAT)
        {
            Serial.print("exFAT");
            return;
        }

        if (fatType != 0U)
        {
            Serial.print("FAT");
            Serial.print(static_cast<unsigned>(fatType));
            return;
        }

        Serial.print("none");
    }

    void PrintBootSectorSummary(const char* label, const uint8_t* sector)
    {
        char oem[9] = {};
        char fat16[9] = {};
        char fat32[9] = {};

        memcpy(oem, sector + 3U, 8U);
        memcpy(fat16, sector + 54U, 8U);
        memcpy(fat32, sector + 82U, 8U);

        Serial.print(label);
        Serial.print(" sig=0x");
        Serial.print(static_cast<unsigned>(sector[kBootSectorSignatureOffset + 1U]), HEX);
        Serial.print(static_cast<unsigned>(sector[kBootSectorSignatureOffset]), HEX);
        Serial.print(" oem='");
        Serial.print(oem);
        Serial.print("' fat16='");
        Serial.print(fat16);
        Serial.print("' fat32='");
        Serial.print(fat32);
        Serial.println("'");
    }

    [[maybe_unused]] void PrintCardLayoutDiagnostics(uint8_t sdioMode)
    {
        if (!SD.sdfs.cardBegin(SdioConfig(sdioMode)))
        {
            Serial.print("SD diagnostic card init failed in ");
            Serial.print(GetSdioModeName(sdioMode));
            Serial.print(": ");
            SD.sdfs.printSdError(&Serial);
            return;
        }

        auto* card = SD.sdfs.card();
        if (card == nullptr)
        {
            Serial.print("SD diagnostic: no card object in ");
            Serial.println(GetSdioModeName(sdioMode));
            return;
        }

        Serial.print("SD raw card mode=");
        Serial.print(GetSdioModeName(sdioMode));
        Serial.print(" type=");
        Serial.print(GetSdCardTypeName(card->type()));
        Serial.print(" sectors=");
        Serial.println(card->sectorCount());

        uint8_t sector0[512] = {};
        if (!card->readSector(0U, sector0))
        {
            Serial.println("SD diagnostic: failed to read sector 0");
            return;
        }

        PrintBootSectorSummary("SD sector0", sector0);

        bool anyPartition = false;
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            const uint8_t* entry = sector0 + kPartitionTableOffset + index * kPartitionEntrySize;
            const uint8_t partitionType = entry[4];
            const uint32_t startSector = ReadLe32(entry + 8U);
            const uint32_t sectorCount = ReadLe32(entry + 12U);
            if (partitionType == 0U && startSector == 0U && sectorCount == 0U)
            {
                continue;
            }

            anyPartition = true;
            Serial.print("SD MBR part ");
            Serial.print(static_cast<unsigned>(index + 1U));
            Serial.print(" type=0x");
            if (partitionType < 0x10U)
            {
                Serial.print('0');
            }
            Serial.print(static_cast<unsigned>(partitionType), HEX);
            Serial.print(" start=");
            Serial.print(startSector);
            Serial.print(" sectors=");
            Serial.println(sectorCount);

            if (startSector == 0U)
            {
                continue;
            }

            uint8_t partitionBootSector[512] = {};
            if (card->readSector(startSector, partitionBootSector))
            {
                PrintBootSectorSummary("SD part boot", partitionBootSector);
            }
        }

        if (!anyPartition)
        {
            Serial.println("SD MBR part table is empty");
        }
    }

    bool TryMountSdioVolume(uint8_t sdioMode, uint8_t part, bool logFailure)
    {
        if (!SD.sdfs.cardBegin(SdioConfig(sdioMode)))
        {
            if (logFailure)
            {
                Serial.print("SD ");
                Serial.print(GetSdioModeName(sdioMode));
                Serial.print(" card init failed: ");
                SD.sdfs.printSdError(&Serial);
            }
            return false;
        }

        auto* card = SD.sdfs.card();
        if (card == nullptr)
        {
            if (logFailure)
            {
                Serial.print("SD ");
                Serial.print(GetSdioModeName(sdioMode));
                Serial.println(" card init returned no card");
            }
            return false;
        }

        if (static_cast<FsVolume&>(SD.sdfs).begin(card, true, part))
        {
            Serial.print("SD mounted via ");
            Serial.print(GetSdioModeName(sdioMode));
            Serial.print(part == 0U ? " sector 0 as " : " partition 1 as ");
            PrintVolumeType(SD.sdfs.fatType());
            Serial.println();
            return true;
        }

        if (logFailure)
        {
            Serial.print("SD ");
            Serial.print(GetSdioModeName(sdioMode));
            Serial.print(part == 0U ? " sector-0 mount failed: " : " partition-1 mount failed: ");
            SD.sdfs.printSdError(&Serial);
        }
        return false;
    }

    bool TryMountPreferredSdVolume(bool logFailure)
    {
        return TryMountSdioVolume(FIFO_SDIO, 1U, logFailure) ||
               TryMountSdioVolume(FIFO_SDIO, 0U, logFailure) ||
               TryMountSdioVolume(DMA_SDIO, 1U, logFailure);
    }

    void UpdateStartupWaitIndicator()
    {
        if (!gStartupWaitIndicatorEnabled)
        {
            return;
        }

        gStartupWaitIndicatorState = !gStartupWaitIndicatorState;
        digitalWriteFast(LED_BUILTIN, gStartupWaitIndicatorState ? HIGH : LOW);
    }

    void StartStartupWaitIndicatorBlink()
    {
        gStartupWaitBlinkTimer.end();
        gStartupWaitIndicatorState = MazeMap::IsStartupWaitIndicatorEnabled(0UL, HardwareConfig::kSdWaitBlinkPeriodMs);
        gStartupWaitIndicatorEnabled = true;
        digitalWriteFast(LED_BUILTIN, gStartupWaitIndicatorState ? HIGH : LOW);
        const unsigned long halfPeriodUs = static_cast<unsigned long>(HardwareConfig::kSdWaitBlinkPeriodMs) * 1000UL / 2UL;
        gStartupWaitBlinkTimer.begin(UpdateStartupWaitIndicator, halfPeriodUs);
    }

    void StopStartupWaitIndicatorBlink()
    {
        gStartupWaitBlinkTimer.end();
        gStartupWaitIndicatorEnabled = false;
        gStartupWaitIndicatorState = false;
        digitalWriteFast(LED_BUILTIN, LOW);
    }
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
    StopStartupWaitIndicatorBlink();

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
    unsigned long waitStartMs = millis();
    bool waitingForSd = false;
    while (true)
    {
        if (TryMountPreferredSdVolume(false))
        {
            if (waitingForSd)
            {
                Serial.print("SD init recovered after waiting ");
                Serial.print(millis() - waitStartMs);
                Serial.println(" ms");
            }
            StopStartupWaitIndicatorBlink();
            return true;
        }

        if (!waitingForSd)
        {
            waitingForSd = true;
            waitStartMs = millis();
            Serial.println("Waiting for SD card");
            StartStartupWaitIndicatorBlink();
        }

        delay(HardwareConfig::kSdInitRetryDelayMs);
    }

}
#endif
