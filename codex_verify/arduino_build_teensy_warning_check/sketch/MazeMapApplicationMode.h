#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapApplicationMode.h"
#pragma once

namespace MazeMapApp::Internal
{
    class IApplicationMode
    {
    public:
        virtual ~IApplicationMode() = default;
        virtual bool Begin() = 0;
        virtual void Run() = 0;
    };
}
