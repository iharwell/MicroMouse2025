#pragma once

#include "Defines.h"

namespace MazeMap::App::Internal::BootUtilityModeFramework
{
    EXPORT bool ResetStartupTrace(const char* firstLine);
    EXPORT bool AppendStartupTrace(const char* line);
}
