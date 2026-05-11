#pragma once

#include <cstdint>

namespace MazeMap
{
    // Exact wheel-encoder measurement contract consumed by the estimator.
    struct EncoderObs
    {
        std::int32_t totalLeftCounts = 0;
        std::int32_t totalRightCounts = 0;
        float leftDistanceDeltaM = 0.0f;
        float rightDistanceDeltaM = 0.0f;
        float leftVelocityMps = 0.0f;
        float rightVelocityMps = 0.0f;
        float omegaLeftRadps = 0.0f;
        float omegaRightRadps = 0.0f;
    };
}
