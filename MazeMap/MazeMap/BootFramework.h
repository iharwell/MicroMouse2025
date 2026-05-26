#pragma once

#include "Defines.h"

namespace MazeMap::App
{
    struct BootModeRegistryEntry;
}

namespace MazeMap::App::Internal
{
    class IApplicationMode;
    class SharedRobotRuntime;

    class EXPORT BootFramework final
    {
    public:
        explicit BootFramework(SharedRobotRuntime& runtime) noexcept;

        BootFramework(const BootFramework&) = delete;
        BootFramework& operator=(const BootFramework&) = delete;
        BootFramework(BootFramework&&) = delete;
        BootFramework& operator=(BootFramework&&) = delete;

        [[noreturn]] void RunSelectedBootMode();
        bool AppendStartupTrace(const char* line);
        bool IsSelectedModeSelectorInstalled() const noexcept;

    private:
        static void HandleModeFaultThunk(void* context, const char* reason) noexcept;

        [[noreturn]] void FailInvalidBootModeSelection(
            const MazeMap::App::BootModeRegistryEntry& selectedMode);
        [[noreturn]] void HaltAfterProgramExit() noexcept;
        void PrepareSelectedSelectorQuery() noexcept;
        void RestoreSelectedSelectorPins() noexcept;
        void ValidateSelectedBootMode(const MazeMap::App::BootModeRegistryEntry& selectedMode);

        SharedRobotRuntime& _runtime;
        const MazeMap::App::BootModeRegistryEntry* _selectedMode;
        IApplicationMode* _activeMode;
        uint8_t _selectorDrivePin;
        uint8_t _selectorSensePin;
        bool _selectorArmed;
        bool _selectorAvailable;
    };
}
