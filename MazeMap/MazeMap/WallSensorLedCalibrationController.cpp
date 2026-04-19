#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootUtilityModeFramework.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapSharedRuntime.h"
#include "PinPairStrap.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

namespace
{
    constexpr const char* kWallSensorLedCalibrationStableId = "wall_sensor_led_calibration";

    enum class LedCalibrationPhase : std::uint8_t
    {
        Front,
        Side,
        Complete
    };

    LedCalibrationPhase AdvanceLedCalibrationPhase(
        const LedCalibrationPhase currentPhase,
        const bool jumperInstalled) noexcept
    {
        switch (currentPhase)
        {
        case LedCalibrationPhase::Front:
            return jumperInstalled ? LedCalibrationPhase::Front : LedCalibrationPhase::Side;
        case LedCalibrationPhase::Side:
            return jumperInstalled ? LedCalibrationPhase::Complete : LedCalibrationPhase::Side;
        case LedCalibrationPhase::Complete:
        default:
            return LedCalibrationPhase::Complete;
        }
    }
}

class WallSensorLedCalibrationController : public IApplicationMode
{
public:
    explicit WallSensorLedCalibrationController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
    {
    }

    bool Begin() override
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(
                &WallSensorLedCalibrationController::TeardownOnRuntimeFault,
                this,
                kWallSensorLedCalibrationStableId))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return _runtime.FailActiveMode("Wall sensor LED calibration hardware setup failed");
        }

        const MazeMap::App::BootModeRegistryEntry* const entry =
            MazeMap::App::FindBootModeRegistryEntry(MazeMap::App::BootModeId::WallSensorLedCalibration);
        if ((entry == nullptr) || (entry->selector.kind != MazeMap::App::BootModeSelectorKind::PinPair))
        {
            return _runtime.FailActiveMode("Wall sensor LED calibration selector pins unavailable");
        }

        _monitorDrivePin = entry->selector.pinA;
        _monitorSensePin = entry->selector.pinB;
        _phase = LedCalibrationPhase::Front;
        _ledEnabled = false;
        _lastToggleUs = micros();

        pinMode(Pins::LED_Ctrl_Forward_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Forward_Right, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Right, OUTPUT);
        SetAllLeds(false);
        BeginPinPairStrapMonitor(_monitorDrivePin, _monitorSensePin);
        _monitorArmed = true;

        (void)_runtime.AppendTextLogLine("Wall sensor LED calibration mode");
        (void)MazeMap::App::Internal::BootUtilityModeFramework::ResetStartupTrace("mode:wall_sensor_led_calibration");
        (void)_runtime.AppendTextLogLine("Front calibration active; side LEDs held off");
        PrintFrequency("Front LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft));
        (void)_runtime.AppendTextLogLine("Remove selector jumper to switch to side calibration");
        return true;
    }

    void Run() override
    {
        bool ok = false;
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &WallSensorLedCalibrationController::ModeWorkThunk;
        callbacks.context = this;
        if (_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            const LoopController::SessionResult result = _loopController.Run();
            ok = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }
        else
        {
            (void)_runtime.FailActiveMode("Wall sensor LED calibration loop session start failed");
        }

        CleanupHardware();
        if (ok)
        {
            (void)_runtime.AppendTextLogLine("Wall sensor LED calibration complete");
        }
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;

    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<WallSensorLedCalibrationController*>(context);
        if (self == nullptr)
        {
            services.Fault("Wall sensor LED calibration context was not installed");
            return LoopController::ControlVector::Brake;
        }

        return self->RunTick(loopEndTimeUs, state, services);
    }

    static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context != nullptr)
        {
            static_cast<WallSensorLedCalibrationController*>(context)->CleanupOnRuntimeFault(reason);
        }
    }

    LoopController::SessionOptions BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = Config::kControlPeriodUs;
        options.workPlan.wallMask = LoopController::WallMask::None;
        options.workPlan.readEncoders = false;
        options.workPlan.readImuBundle = false;
        options.workPlan.useEncoderUpdate = false;
        options.workPlan.useGyroUpdate = false;
        options.workPlan.useAccelUpdate = false;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    static void SetFrontLeds(const bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Forward_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Forward_Right, enabled ? HIGH : LOW);
    }

    static void SetSideLeds(const bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Side_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Side_Right, enabled ? HIGH : LOW);
    }

    static void SetAllLeds(const bool enabled)
    {
        SetFrontLeds(enabled);
        SetSideLeds(enabled);
    }

    void ToggleActiveLeds()
    {
        _ledEnabled = !_ledEnabled;
        if (_phase == LedCalibrationPhase::Front)
        {
            SetFrontLeds(_ledEnabled);
            return;
        }

        SetSideLeds(_ledEnabled);
    }

    void PrintFrequency(const char* label, const uint32_t halfPeriodUs)
    {
        if (halfPeriodUs == 0U)
        {
            (void)_runtime.AppendTextLogFormatted("%s%.3f", (label != nullptr) ? label : "", 0.0f);
            return;
        }

        (void)_runtime.AppendTextLogFormatted(
            "%s%.3f",
            (label != nullptr) ? label : "",
            1000000.0f / (2.0f * static_cast<float>(halfPeriodUs)));
    }

    LoopController::ControlVector RunTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;

        const bool jumperInstalled = IsPinPairStrapMonitorClosed(_monitorSensePin);
        const LedCalibrationPhase nextPhase = AdvanceLedCalibrationPhase(_phase, jumperInstalled);
        if (nextPhase != _phase)
        {
            if (_phase == LedCalibrationPhase::Front)
            {
                SetFrontLeds(false);
                _ledEnabled = false;
                _lastToggleUs = micros();
                (void)_runtime.AppendTextLogLine("Side calibration active; front LEDs held off");
                PrintFrequency("Side LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft));
            }
            else if (_phase == LedCalibrationPhase::Side)
            {
                SetSideLeds(false);
                _ledEnabled = false;
            }

            _phase = nextPhase;
            if (_phase == LedCalibrationPhase::Complete)
            {
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;
            }
        }

        const uint32_t halfPeriodUs =
            (_phase == LedCalibrationPhase::Front) ?
                WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft) :
                WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft);
        const unsigned long nowUs = micros();
        if (static_cast<std::uint32_t>(nowUs - _lastToggleUs) >= halfPeriodUs)
        {
            ToggleActiveLeds();
            _lastToggleUs = nowUs;
        }

        return LoopController::ControlVector::Brake;
    }

    void ResetState() noexcept
    {
        _phase = LedCalibrationPhase::Front;
        _ledEnabled = false;
        _lastToggleUs = 0UL;
        _monitorDrivePin = 0U;
        _monitorSensePin = 0U;
        _monitorArmed = false;
    }

    void CleanupHardware() noexcept
    {
        if (_monitorArmed)
        {
            EndPinPairStrapMonitor(_monitorDrivePin, _monitorSensePin);
            _monitorArmed = false;
        }

        SetAllLeds(false);
        _ledEnabled = false;
    }

    void CleanupOnRuntimeFault(const char* reason) noexcept
    {
        (void)reason;
        CleanupHardware();
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    std::uint8_t _monitorDrivePin{};
    std::uint8_t _monitorSensePin{};
    LedCalibrationPhase _phase{ LedCalibrationPhase::Front };
    bool _ledEnabled{};
    unsigned long _lastToggleUs{};
    bool _monitorArmed{};
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
            "Blink front and side wall-sensor LEDs for optical calibration.",
            "logging.txt; operator-visible LED square waves",
            &GetWallSensorLedCalibrationMode,
            "GetWallSensorLedCalibrationMode",
            "WallSensorLedCalibrationController.cpp",
            "front LED calibration; side LED calibration",
            "none",
            "none",
            "calibration frequency trace in logging.txt",
        };
        return descriptor;
    }

    IApplicationMode& GetWallSensorLedCalibrationMode()
    {
        static WallSensorLedCalibrationController mode(GetSharedRobotRuntime());
        return mode;
    }
}
