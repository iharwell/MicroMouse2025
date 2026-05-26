#include "pch.h"
#include "Application.h"
#include "SharedRobotRuntime.h"
#include "Defines.h"

namespace MazeMap::App
{
    void Application::Setup()
    {
        Internal::SharedRobotRuntime& runtime = Internal::GetSharedRobotRuntime();
        runtime.BootFramework().RunSelectedBootMode();
    }

    void Application::Loop()
    {
        delay(100);
    }
}

