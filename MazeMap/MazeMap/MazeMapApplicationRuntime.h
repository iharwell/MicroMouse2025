#pragma once

#include "Defines.h"
#include "IApplicationMode.h"

namespace MazeMap::App::Internal
{
    EXPORT IApplicationMode& ResolveActiveApplicationMode();
}

