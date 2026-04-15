#include "pch.h"
#include "MazeMapApplication.h"

#include "BootModeRegistry.h"
#include "MazeMapApplicationRuntime.h"

#include <cassert>
#include <stdexcept>
#include <string>

namespace
{
    [[noreturn]] void FailInvalidBootModeSelection(const MazeMap::App::BootModeRegistryEntry& selectedMode)
    {
        const char* stableId =
            (selectedMode.descriptor != nullptr && selectedMode.descriptor->stableId != nullptr) ?
            selectedMode.descriptor->stableId :
            MazeMap::App::BootModeIdName(selectedMode.id);
#ifdef ARDUINO_TEENSY41
        assert(false && "BootModeRegistry selected a mode without a callable descriptor entry.");
        while (true)
        {
        }
#else
        throw std::logic_error(
            std::string("BootModeRegistry selected a mode without a callable descriptor entry: ") +
            stableId);
#endif
    }
}

namespace MazeMap::App::Internal
{
    IApplicationMode& ResolveActiveApplicationMode()
    {
        const BootModeRegistryEntry& selectedMode = ResolveSelectedBootMode();
        if (selectedMode.descriptor == nullptr || selectedMode.descriptor->entryMode == nullptr)
        {
            FailInvalidBootModeSelection(selectedMode);
        }

        return selectedMode.descriptor->entryMode();
    }
}
