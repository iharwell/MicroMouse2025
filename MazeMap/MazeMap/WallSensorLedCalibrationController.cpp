#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

class WallSensorLedCalibrationController : public IApplicationMode
{
public:
    bool Begin() override
    {
        (void)GetSharedRobotRuntime().AppendTextLogLine("Wall sensor LED calibration mode");
        ResetStartupTrace("mode:wall_sensor_led_calibration");

        pinMode(Pins::LED_Ctrl_Forward_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Forward_Right, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Right, OUTPUT);
        SetFrontLeds(false);
        SetSideLeds(false);
        BeginJumperMonitor();

        (void)GetSharedRobotRuntime().AppendTextLogLine("Front calibration active; side LEDs held off");
        PrintFrequency("Front LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft));
        (void)GetSharedRobotRuntime().AppendTextLogLine("Remove jumper on pins 38-39 to switch to side calibration");
        return true;
    }

    void Run() override
    {
        const uint32_t frontHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft);
        const uint32_t sideHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft);
        RunFrontCalibration(frontHalfPeriodUs);

        SetFrontLeds(false);
        SetSideLeds(false);
        (void)GetSharedRobotRuntime().AppendTextLogLine("Side calibration active; front LEDs held off");
        PrintFrequency("Side LED square wave (Hz): ", sideHalfPeriodUs);
        RunSideCalibration(sideHalfPeriodUs);
        SetFrontLeds(false);
        SetSideLeds(false);
        (void)GetSharedRobotRuntime().AppendTextLogLine("Wall sensor LED calibration complete");
    }

private:
    static void BeginJumperMonitor()
    {
        pinMode(LedCalibrationConfig::kCalibrationJumperPinA, OUTPUT);
        digitalWriteFast(LedCalibrationConfig::kCalibrationJumperPinA, LOW);
        pinMode(LedCalibrationConfig::kCalibrationJumperPinB, INPUT_PULLUP);
    }

    static bool IsCalibrationJumperInstalled()
    {
        return digitalReadFast(LedCalibrationConfig::kCalibrationJumperPinB) == LOW;
    }

    static void SetFrontLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Forward_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Forward_Right, enabled ? HIGH : LOW);
    }

    static void SetSideLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Side_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Side_Right, enabled ? HIGH : LOW);
    }

    static void RunFrontCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Front, IsCalibrationJumperInstalled()) ==
               WallSensorLedCalibrationPhase::Front)
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetFrontLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void RunSideCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Side, IsCalibrationJumperInstalled()) ==
               WallSensorLedCalibrationPhase::Side)
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetSideLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void PrintFrequency(const char* label, uint32_t halfPeriodUs)
    {
        if (halfPeriodUs == 0U)
        {
            (void)GetSharedRobotRuntime().AppendTextLogFormatted("%s%.3f", (label != nullptr) ? label : "", 0.0f);
            return;
        }

        (void)GetSharedRobotRuntime().AppendTextLogFormatted(
            "%s%.3f",
            (label != nullptr) ? label : "",
            1000000.0f / (2.0f * static_cast<float>(halfPeriodUs)));
    }
};

namespace MazeMap::App::Internal
{
    IApplicationMode& GetWallSensorLedCalibrationMode();

    const BootModeDescriptor& GetWallSensorLedCalibrationBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::WallSensorLedCalibration,
            BootModeCategory::Utility,
            "wall_sensor_led_calibration",
            "Drive front and side wall-sensor LEDs for optical calibration.",
            "logging.txt; operator-visible LED square waves",
            &GetWallSensorLedCalibrationMode,
            "GetWallSensorLedCalibrationMode",
            "WallSensorLedCalibrationController.cpp",
            "front LED calibration; side LED calibration",
            "WallSensorLedCalibrationPhase and wall-sensor LED timing helpers",
            "none",
            "calibration frequency trace in logging.txt",
        };
        return descriptor;
    }

    IApplicationMode& GetWallSensorLedCalibrationMode()
    {
        static WallSensorLedCalibrationController mode;
        return mode;
    }
}

