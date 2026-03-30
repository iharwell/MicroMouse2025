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
