#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"
#include "MissionModeController.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class MissionRunMode final : public IApplicationMode
    {
    public:
        explicit MissionRunMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        MissionModeController _controller;
    };

    IApplicationMode& GetMissionRunMode();
    const BootModeDescriptor& GetMissionRunBootModeDescriptor();
}
