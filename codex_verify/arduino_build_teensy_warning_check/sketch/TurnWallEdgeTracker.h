#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\TurnWallEdgeTracker.h"
#pragma once

namespace MazeMap
{
    struct TurnWallEdgeTracker
    {
        bool initialized = false;
        bool previousLeftWall = false;
        bool previousRightWall = false;
        bool leftWallRose = false;
        bool rightWallRose = false;
    };

    inline void ObserveTurnWallStates(TurnWallEdgeTracker& tracker, bool leftWall, bool rightWall) noexcept
    {
        if (!tracker.initialized)
        {
            tracker.initialized = true;
            tracker.previousLeftWall = leftWall;
            tracker.previousRightWall = rightWall;
            return;
        }

        tracker.leftWallRose = tracker.leftWallRose || (!tracker.previousLeftWall && leftWall);
        tracker.rightWallRose = tracker.rightWallRose || (!tracker.previousRightWall && rightWall);
        tracker.previousLeftWall = leftWall;
        tracker.previousRightWall = rightWall;
    }
}
