#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapApplicationEntry.cpp"
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
