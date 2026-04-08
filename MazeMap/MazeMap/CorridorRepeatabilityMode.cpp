#include "pch.h"
#include "CorridorRepeatabilityMode.h"

namespace MazeMap::App::Internal
{
    CorridorRepeatabilityMode::CorridorRepeatabilityMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool CorridorRepeatabilityMode::Begin()
    {
        return Host().BeginCorridorRepeatabilityMode();
    }

    void CorridorRepeatabilityMode::Run()
    {
        Host().RunCorridorRepeatabilityMode();
    }
}

