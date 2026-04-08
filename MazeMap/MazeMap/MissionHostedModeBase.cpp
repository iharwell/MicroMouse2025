#include "pch.h"
#include "MissionHostedModeBase.h"

namespace MazeMap::App::Internal
{
    MissionHostedModeBase::MissionHostedModeBase(IMissionModeHost& host)
        : _host(host)
    {
    }

    IMissionModeHost& MissionHostedModeBase::Host() const
    {
        return _host;
    }
}

