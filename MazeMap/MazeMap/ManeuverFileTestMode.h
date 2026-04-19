#pragma once

#include "BootModeDescriptor.h"

namespace MazeMap::App::Internal
{
    class IApplicationMode;

    IApplicationMode& GetManeuverFileTestMode();
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor();
}
