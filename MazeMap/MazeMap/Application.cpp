#include "pch.h"
#include "Application.h"
#include "MazeMapApplicationRuntime.h"
#include "SharedRobotRuntime.h"
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

