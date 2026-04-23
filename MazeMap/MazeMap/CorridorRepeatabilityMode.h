#pragma once

#include "BootModeDescriptor.h"
#include "IApplicationMode.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetCorridorRepeatabilityMode();
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor();
}
