#include "pch.h"
#include "Application.h"
#include "MazeMapApplicationRuntime.h"
#include "SharedRobotRuntime.h"
#include "Defines.h"

namespace
{
    [[noreturn]] void HaltAfterProgramExit() noexcept
    {
        while (true)
        {
            delay(100);
        }
    }
}

namespace MazeMap::App
{
    void Application::Setup()
    {
        Internal::SharedRobotRuntime& runtime = Internal::GetSharedRobotRuntime();
        Internal::IApplicationMode& mode = Internal::ResolveActiveApplicationMode();
        mode.SetupMode();
        runtime.ControlLoop().BindApplicationMode(mode);
        runtime.ControlLoop().Run();
        runtime.FinalizeSuccessfulModeExit();
        HaltAfterProgramExit();
    }

    void Application::Loop()
    {
        delay(100);
    }
}

