#pragma once

#include "MazeMapApplicationMode.h"
#include "MazeMapMissionModeHost.h"

namespace MazeMap::App::Internal
{
    class MissionHostedModeBase : public IApplicationMode
    {
    public:
        explicit MissionHostedModeBase(IMissionModeHost& host);
        MissionHostedModeBase(const MissionHostedModeBase&) = delete;
        MissionHostedModeBase& operator=(const MissionHostedModeBase&) = delete;
        MissionHostedModeBase(MissionHostedModeBase&&) = delete;
        MissionHostedModeBase& operator=(MissionHostedModeBase&&) = delete;

    protected:
        IMissionModeHost& Host() const;

    private:
        IMissionModeHost& _host;
    };
}
