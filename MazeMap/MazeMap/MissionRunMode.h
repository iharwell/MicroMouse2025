#pragma once

#include <memory>

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class MissionRunMode final : public IApplicationMode
    {
    public:
        explicit MissionRunMode(SharedRobotRuntime& runtime);
        ~MissionRunMode();

        bool Begin() override;
        void Run() override;

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    IApplicationMode& GetMissionRunMode();
    const BootModeDescriptor& GetMissionRunBootModeDescriptor();
}
