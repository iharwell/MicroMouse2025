#pragma once

#include "MissionHostedModeBase.h"

namespace MazeMap::App::Internal
{
    class ManeuverFileTestMode final : public MissionHostedModeBase
    {
    public:
        explicit ManeuverFileTestMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    IApplicationMode& GetManeuverFileTestMode();
}

