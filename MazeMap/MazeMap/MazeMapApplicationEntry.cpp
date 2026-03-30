#include "pch.h"
#include "MazeMapApplication.h"
#include "MazeMapApplicationRuntime.h"
#include "Defines.h"

namespace MazeMapApp
{
    void Application::Setup()
    {
        Internal::IApplicationMode& mode = Internal::ResolveActiveApplicationMode();
        if (mode.Begin())
        {
            mode.Run();
        }
    }

    void Application::Loop()
    {
        delay(100);
    }
}
