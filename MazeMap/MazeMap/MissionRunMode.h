#pragma once

#include "MissionHostedModeBase.h"

namespace MazeMap::App::Internal
{
    class MissionRunMode final : public MissionHostedModeBase
    {
    public:
        explicit MissionRunMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    IApplicationMode& GetMissionRunMode();
}

