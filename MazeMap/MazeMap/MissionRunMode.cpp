#include "pch.h"
#include "MissionRunMode.h"

namespace MazeMap::App::Internal
{
    MissionRunMode::MissionRunMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool MissionRunMode::Begin()
    {
        return Host().BeginMissionRunMode();
    }

    void MissionRunMode::Run()
    {
        Host().RunMissionRunMode();
    }
}

