#include "pch.h"
#include "BootUtilityModeFramework.h"

#include "Defines.h"
#include "SharedRobotRuntime.h"

namespace MazeMap::App::Internal::BootUtilityModeFramework
{
    bool ResetStartupTrace(const char* firstLine)
    {
        if (firstLine == nullptr || firstLine[0] == '\0')
        {
            return false;
        }

        return MazeMap::App::Internal::GetSharedRobotRuntime().WriteTextLogEntry(
            "startup_trace",
            micros(),
            "begin",
            firstLine);
    }

    bool AppendStartupTrace(const char* line)
    {
        if (line == nullptr || line[0] == '\0')
        {
            return false;
        }

        return MazeMap::App::Internal::GetSharedRobotRuntime().WriteTextLogEntry(
            "startup_trace",
            micros(),
            "trace",
            line);
    }
}
