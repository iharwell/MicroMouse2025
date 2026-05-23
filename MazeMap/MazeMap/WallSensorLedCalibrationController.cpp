#include "pch.h"
#include "WallSensorLedCalibrationController.h"

#include "MazeMapApplicationPrivate.h"
#include "BootUtilityModeFramework.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "PinPairStrap.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

namespace
{
    constexpr const char* kWallSensorLedCalibrationStableId = "wall_sensor_led_calibration";
}

namespace MazeMap::App::Internal
{
    WallSensorLedCalibrationController::WallSensorLedCalibrationController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
    {
    }

    void WallSensorLedCalibrationController::SetupMode()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(
                &WallSensorLedCalibrationController::TeardownOnRuntimeFault,
                this,
                kWallSensorLedCalibrationStableId))
        {
            _runtime.FailActiveMode("Wall sensor LED calibration fault handler registration failed");
        }

        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Wall sensor LED calibration hardware setup failed");
        }

        const MazeMap::App::BootModeRegistryEntry* const entry =
            MazeMap::App::FindBootModeRegistryEntry(MazeMap::App::BootModeId::WallSensorLedCalibration);
        if ((entry == nullptr) || (entry->selector.kind != MazeMap::App::BootModeSelectorKind::PinPair))
        {
            _runtime.FailActiveMode("Wall sensor LED calibration selector pins unavailable");
        }

        _monitorDrivePin = entry->selector.pinA;
        _monitorSensePin = entry->selector.pinB;
        _phase = LedCalibrationPhase::Front;
        _ledEnabled = false;
        _lastToggleUs = static_cast<std::uint32_t>(micros());
        _pauseRequested = false;
        _runtimeFaultReason = nullptr;

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
        const auto& runtimeState = _runtime.RuntimeState();
        _loopController.StageNextSessionState(
            Config::kControlPeriodUs,
            runtimeState.GetPositionX(),
            runtimeState.GetPositionY(),
            LoopController::WallMask::None,
            false,
            false,
            false,
            false);
    }

    CommandVector WallSensorLedCalibrationController::RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)state;
        return OnModeWork(loopController);
    }

    void WallSensorLedCalibrationController::PauseThunk(
        void* context,
        LoopController& loopController)
    {
        auto* const self = static_cast<WallSensorLedCalibrationController*>(context);
        if (self == nullptr)
        {
            GetSharedRobotRuntime().FailActiveMode(
                "Wall sensor LED calibration pause callback context was null");
        }

        self->OnPauseGranted(loopController);
    }

    void WallSensorLedCalibrationController::TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context != nullptr)
        {
            static_cast<WallSensorLedCalibrationController*>(context)->CleanupOnRuntimeFault(reason);
        }
    }

    CommandVector WallSensorLedCalibrationController::OnModeWork(
        LoopController& loopController)
    {
        if (_pauseRequested)
        {
            _runtime.FailActiveMode(
                "Wall sensor LED calibration unexpectedly resumed after pause request");
            return CommandVector::Brake();
        }

        _pauseRequested = true;
        loopController.RequestPause(&WallSensorLedCalibrationController::PauseThunk, this);
        return CommandVector::Brake();
    }

    void WallSensorLedCalibrationController::OnPauseGranted(
        LoopController& loopController)
    {
        RunCalibrationLoop();
        if (_runtimeFaulted)
        {
            _runtime.FailActiveMode(
                (_runtimeFaultReason != nullptr) ?
                    _runtimeFaultReason :
                    "Wall sensor LED calibration paused execution faulted");
        }

        FinalizeSuccessfulRun();
        loopController.HaltExecutionEndProgram();
    }

    void WallSensorLedCalibrationController::SetFrontLeds(const bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Forward_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Forward_Right, enabled ? HIGH : LOW);
    }

    void WallSensorLedCalibrationController::SetSideLeds(const bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Side_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Side_Right, enabled ? HIGH : LOW);
    }

    void WallSensorLedCalibrationController::SetAllLeds(const bool enabled)
    {
        SetFrontLeds(enabled);
        SetSideLeds(enabled);
    }

    void WallSensorLedCalibrationController::ToggleActiveLeds()
    {
        _ledEnabled = !_ledEnabled;
        if (_phase == LedCalibrationPhase::Front)
        {
            SetFrontLeds(_ledEnabled);
            return;
        }

        SetSideLeds(_ledEnabled);
    }

    void WallSensorLedCalibrationController::PrintFrequency(const char* label, const uint32_t halfPeriodUs)
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

    std::uint32_t WallSensorLedCalibrationController::ActiveHalfPeriodUs() const noexcept
    {
        switch (_phase)
        {
        case LedCalibrationPhase::Front:
            return WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft);
        case LedCalibrationPhase::Side:
            return WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft);
        case LedCalibrationPhase::Complete:
        default:
            return 0U;
        }
    }

    void WallSensorLedCalibrationController::RunCalibrationLoop()
    {
        while (!_runtimeFaulted && _phase != LedCalibrationPhase::Complete)
        {
            AdvancePhase();
            if (_runtimeFaulted || _phase == LedCalibrationPhase::Complete)
            {
                break;
            }

            const std::uint32_t halfPeriodUs = ActiveHalfPeriodUs();
            if (halfPeriodUs == 0U)
            {
                continue;
            }

            const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
            if (static_cast<std::uint32_t>(nowUs - _lastToggleUs) >= halfPeriodUs)
            {
                ToggleActiveLeds();
                _lastToggleUs = nowUs;
                continue;
            }

            const std::uint32_t remainingUs = halfPeriodUs - static_cast<std::uint32_t>(nowUs - _lastToggleUs);
            delayMicroseconds(static_cast<unsigned int>(remainingUs));
        }
    }

    void WallSensorLedCalibrationController::AdvancePhase()
    {
        const bool jumperInstalled = IsPinPairStrapMonitorClosed(_monitorSensePin);
        LedCalibrationPhase nextPhase = _phase;
        switch (_phase)
        {
        case LedCalibrationPhase::Front:
            nextPhase = jumperInstalled ? LedCalibrationPhase::Front : LedCalibrationPhase::Side;
            break;
        case LedCalibrationPhase::Side:
            nextPhase = jumperInstalled ? LedCalibrationPhase::Complete : LedCalibrationPhase::Side;
            break;
        case LedCalibrationPhase::Complete:
        default:
            nextPhase = LedCalibrationPhase::Complete;
            break;
        }
        if (nextPhase != _phase)
        {
            if (_phase == LedCalibrationPhase::Front)
            {
                SetFrontLeds(false);
                _ledEnabled = false;
                _lastToggleUs = static_cast<std::uint32_t>(micros());
                (void)_runtime.AppendTextLogLine("Side calibration active; front LEDs held off");
                PrintFrequency("Side LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft));
            }
            else if (_phase == LedCalibrationPhase::Side)
            {
                SetSideLeds(false);
                _ledEnabled = false;
            }

            _phase = nextPhase;
        }
    }

    void WallSensorLedCalibrationController::ResetState() noexcept
    {
        _phase = LedCalibrationPhase::Front;
        _ledEnabled = false;
        _lastToggleUs = 0U;
        _monitorDrivePin = 0U;
        _monitorSensePin = 0U;
        _monitorArmed = false;
        _runtimeFaulted = false;
        _pauseRequested = false;
        _runtimeFaultReason = nullptr;
    }

    void WallSensorLedCalibrationController::FinalizeSuccessfulRun() noexcept
    {
        _phase = LedCalibrationPhase::Complete;
        CleanupHardware();
        (void)_runtime.AppendTextLogLine("Wall sensor LED calibration complete");
    }

    void WallSensorLedCalibrationController::CleanupHardware() noexcept
    {
        if (_monitorArmed)
        {
            EndPinPairStrapMonitor(_monitorDrivePin, _monitorSensePin);
            _monitorArmed = false;
        }

        SetAllLeds(false);
        _ledEnabled = false;
    }

    void WallSensorLedCalibrationController::CleanupOnRuntimeFault(const char* reason) noexcept
    {
        _runtimeFaulted = true;
        _runtimeFaultReason = reason;
        _phase = LedCalibrationPhase::Complete;
        CleanupHardware();
    }

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
