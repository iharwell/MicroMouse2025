#include "pch.h"
#include "PositionAccuracyAuditMode.h"

namespace MazeMap::App::Internal
{
    PositionAccuracyAuditMode::PositionAccuracyAuditMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool PositionAccuracyAuditMode::Begin()
    {
        return Host().BeginPositionAccuracyAuditMode();
    }

    void PositionAccuracyAuditMode::Run()
    {
        Host().RunPositionAccuracyAuditMode();
    }
}

