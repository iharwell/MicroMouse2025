#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"
#include "MissionModeController.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class ManeuverFileTestMode final : public IApplicationMode
    {
    public:
        explicit ManeuverFileTestMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        MissionModeController _controller;
    };

    IApplicationMode& GetManeuverFileTestMode();
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor();
}
