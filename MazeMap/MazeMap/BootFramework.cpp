#include "pch.h"
#include "BootFramework.h"

#include "BootModeRegistry.h"
#include "IApplicationMode.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

#include <cassert>

namespace MazeMap::App::Internal
{
    BootFramework::BootFramework(SharedRobotRuntime& runtime) noexcept
        : _runtime(runtime)
        , _selectedMode(nullptr)
        , _activeMode(nullptr)
        , _selectorDrivePin(0U)
        , _selectorSensePin(0U)
        , _selectorArmed(false)
        , _selectorAvailable(false)
    {
    }

    [[noreturn]] void BootFramework::RunSelectedBootMode()
    {
        const BootModeRegistryEntry& selectedMode = ResolveSelectedBootMode();
        ValidateSelectedBootMode(selectedMode);
        _selectedMode = &selectedMode;

        const BootModeDescriptor& descriptor = *selectedMode.descriptor;
        IApplicationMode& activeMode = descriptor.entryMode();
        _activeMode = &activeMode;

        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Boot hardware setup failed");
        }

        if (!_runtime.RegisterModeFaultHandler(
                &BootFramework::HandleModeFaultThunk,
                this,
                descriptor.stableId))
        {
            _runtime.FailActiveMode("Boot fault handler registration failed");
        }

        PrepareSelectedSelectorQuery();

        if (!_runtime.WriteTextLogEntry("startup_trace", micros(), "begin", descriptor.stableId))
        {
            _runtime.FailActiveMode("Boot startup trace begin failed");
        }

        BringUpSelectedStartupServices(descriptor);
        activeMode.SetupMode(*this);
        _runtime.ControlLoop().BindApplicationMode(activeMode);
        _runtime.ControlLoop().Run();
        RestoreSelectedSelectorPins();
        _runtime.FinalizeSuccessfulModeExit();
        HaltAfterProgramExit();
    }

    void BootFramework::BringUpSelectedStartupServices(const BootModeDescriptor& descriptor)
    {
        if (!descriptor.requiresStartupCalibrationBringUp)
        {
            return;
        }

        if (!_runtime.StartupCalibrationService().BringUp())
        {
            _runtime.FailActiveMode("Startup calibration bring-up failed");
        }
    }

    bool BootFramework::AppendStartupTrace(const char* line)
    {
        if (line == nullptr || line[0] == '\0')
        {
            return false;
        }

        return _runtime.WriteTextLogEntry(
            "startup_trace",
            micros(),
            "trace",
            line);
    }

    bool BootFramework::IsSelectedModeSelectorInstalled() const noexcept
    {
        if (_selectedMode == nullptr)
        {
            assert(false && "BootFramework selector queried before boot mode selection.");
            return false;
        }

        return _selectorAvailable && _selectorArmed && digitalRead(_selectorSensePin) == LOW;
    }

    void BootFramework::HandleModeFaultThunk(void* context, const char* reason) noexcept
    {
        auto* const framework = static_cast<BootFramework*>(context);
        if (framework == nullptr)
        {
            return;
        }

        if (framework->_activeMode != nullptr)
        {
            framework->_activeMode->OnModeFault(reason);
        }

        framework->RestoreSelectedSelectorPins();
    }

    [[noreturn]] void BootFramework::FailInvalidBootModeSelection(
        const MazeMap::App::BootModeRegistryEntry& selectedMode)
    {
        (void)selectedMode;
        assert(false && "BootModeRegistry selected an invalid boot mode descriptor.");
        _runtime.FailActiveMode("invalid_boot_mode_selection");
    }

    [[noreturn]] void BootFramework::HaltAfterProgramExit() noexcept
    {
        while (true)
        {
            delay(100);
        }
    }

    void BootFramework::PrepareSelectedSelectorQuery() noexcept
    {
        RestoreSelectedSelectorPins();

        if (_selectedMode == nullptr ||
            _selectedMode->selector.kind != BootModeSelectorKind::PinPair)
        {
            return;
        }

        _selectorDrivePin = _selectedMode->selector.pinA;
        _selectorSensePin = _selectedMode->selector.pinB;
        _selectorAvailable = true;

        digitalWrite(_selectorDrivePin, LOW);
        pinMode(_selectorDrivePin, OUTPUT);
        pinMode(_selectorSensePin, INPUT_PULLUP);
        _selectorArmed = true;
    }

    void BootFramework::RestoreSelectedSelectorPins() noexcept
    {
        if (_selectorArmed)
        {
            pinMode(_selectorDrivePin, INPUT_PULLUP);
            pinMode(_selectorSensePin, INPUT_PULLUP);
        }

        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
        _selectorArmed = false;
        _selectorAvailable = false;
    }

    void BootFramework::ValidateSelectedBootMode(const BootModeRegistryEntry& selectedMode)
    {
        if (selectedMode.descriptor == nullptr ||
            selectedMode.descriptor->id != selectedMode.id ||
            selectedMode.descriptor->stableId == nullptr ||
            selectedMode.descriptor->stableId[0] == '\0' ||
            selectedMode.descriptor->entryMode == nullptr)
        {
            FailInvalidBootModeSelection(selectedMode);
        }
    }
}
