#pragma once

#include <cstdint>

namespace MazeMap
{
    // Exact wheel-encoder measurement contract consumed by the estimator.
    struct EncoderObs
    {
        std::int32_t totalLeftCounts = 0;
        std::int32_t totalRightCounts = 0;
        float omegaLeftRadps = 0.0f;
        float omegaRightRadps = 0.0f;
    };
}
