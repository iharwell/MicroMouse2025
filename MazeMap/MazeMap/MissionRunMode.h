#pragma once

#include "BootModeDescriptor.h"
#include "IApplicationMode.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetMissionRunMode();
    const BootModeDescriptor& GetMissionRunBootModeDescriptor();
}
