#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"
#include "MissionModeController.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class PositionAccuracyAuditMode final : public IApplicationMode
    {
    public:
        explicit PositionAccuracyAuditMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        MissionModeController _controller;
    };

    IApplicationMode& GetPositionAccuracyAuditMode();
    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor();
}
