#pragma once

#include <memory>

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class PositionAccuracyAuditMode final : public IApplicationMode
    {
    public:
        explicit PositionAccuracyAuditMode(SharedRobotRuntime& runtime);
        ~PositionAccuracyAuditMode();

        bool Begin() override;
        void Run() override;

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    IApplicationMode& GetPositionAccuracyAuditMode();
    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor();
}
