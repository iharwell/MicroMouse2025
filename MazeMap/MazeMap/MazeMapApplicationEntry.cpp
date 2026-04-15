#include "pch.h"
#include "MazeMapApplication.h"
#include "MazeMapApplicationRuntime.h"
#include "MazeMapSharedRuntime.h"
#include "Defines.h"

namespace MazeMap::App
{
    void Application::Setup()
    {
        Internal::SharedRobotRuntime& runtime = Internal::GetSharedRobotRuntime();
        Internal::IApplicationMode& mode = Internal::ResolveActiveApplicationMode();
        if (mode.Begin())
        {
            mode.Run();
            runtime.FinalizeSuccessfulModeExit();
        }
    }

    void Application::Loop()
    {
        delay(100);
    }
}

