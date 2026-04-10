#pragma once

namespace MazeMap::App::Internal::BootUtilityModeFramework
{
    bool ResetStartupTrace(const char* firstLine);
    bool AppendStartupTrace(const char* line);
}
