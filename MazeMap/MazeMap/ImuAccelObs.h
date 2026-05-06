#pragma once

namespace MazeMap
{
    // Planar accelerometer measurement contract consumed by the estimator.
    struct ImuAccelObs
    {
        bool valid = false;
        float accelBodyXMps2 = 0.0f;
        float accelBodyYMps2 = 0.0f;
    };
}
