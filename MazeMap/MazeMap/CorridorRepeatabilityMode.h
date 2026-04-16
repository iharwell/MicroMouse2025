#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"
#include "MissionModeController.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class CorridorRepeatabilityMode final : public IApplicationMode
    {
    public:
        explicit CorridorRepeatabilityMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        MissionModeController _controller;
    };

    IApplicationMode& GetCorridorRepeatabilityMode();
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor();
}
