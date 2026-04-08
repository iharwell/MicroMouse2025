#pragma once

#include "MissionHostedModeBase.h"

namespace MazeMap::App::Internal
{
    class CorridorRepeatabilityMode final : public MissionHostedModeBase
    {
    public:
        explicit CorridorRepeatabilityMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };
}

