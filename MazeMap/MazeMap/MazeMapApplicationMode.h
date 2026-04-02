#pragma once
// Declares the application-mode interface used by the desktop and embedded entry points.

namespace MazeMap::App::Internal
{
    // Represents a runnable application mode with explicit begin and run phases.
    class IApplicationMode
    {
    public:
        virtual ~IApplicationMode() = default;
        virtual bool Begin() = 0;
        virtual void Run() = 0;
    };
}

