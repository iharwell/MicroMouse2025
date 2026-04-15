#pragma once

#include "MissionHostedModeBase.h"

namespace MazeMap::App::Internal
{
    class PositionAccuracyAuditMode final : public MissionHostedModeBase
    {
    public:
        explicit PositionAccuracyAuditMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    IApplicationMode& GetPositionAccuracyAuditMode();
}

