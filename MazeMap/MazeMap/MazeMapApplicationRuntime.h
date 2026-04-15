#pragma once

#include "Defines.h"
#include "MazeMapApplicationMode.h"

namespace MazeMap::App::Internal
{
    EXPORT IApplicationMode& ResolveActiveApplicationMode();
}

