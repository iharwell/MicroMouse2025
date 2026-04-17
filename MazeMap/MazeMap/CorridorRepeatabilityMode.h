#pragma once

#include <memory>

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class CorridorRepeatabilityMode final : public IApplicationMode
    {
    public:
        explicit CorridorRepeatabilityMode(SharedRobotRuntime& runtime);
        ~CorridorRepeatabilityMode();

        bool Begin() override;
        void Run() override;

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    IApplicationMode& GetCorridorRepeatabilityMode();
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor();
}
