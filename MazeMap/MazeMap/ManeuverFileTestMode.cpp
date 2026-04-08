#include "pch.h"
#include "ManeuverFileTestMode.h"

namespace MazeMap::App::Internal
{
    ManeuverFileTestMode::ManeuverFileTestMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool ManeuverFileTestMode::Begin()
    {
        return Host().BeginManeuverFileTestMode();
    }

    void ManeuverFileTestMode::Run()
    {
        Host().RunManeuverFileTestMode();
    }
}

