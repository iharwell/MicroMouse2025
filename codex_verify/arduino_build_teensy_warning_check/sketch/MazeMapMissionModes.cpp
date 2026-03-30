#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapMissionModes.cpp"
#include "pch.h"
#include "MazeMapMissionModes.h"

namespace MazeMapApp::Internal
{
    MissionHostedModeBase::MissionHostedModeBase(IMissionModeHost& host)
        : _host(host)
    {
    }

    IMissionModeHost& MissionHostedModeBase::Host() const
    {
        return _host;
    }

    MissionRunMode::MissionRunMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool MissionRunMode::Begin()
    {
        return Host().BeginMissionRunMode();
    }

    void MissionRunMode::Run()
    {
        Host().RunMissionRunMode();
    }

    ManeuverFileTestMode::ManeuverFileTestMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool ManeuverFileTestMode::Begin()
    {
        return Host().BeginManeuverFileTestMode();
    }

    void ManeuverFileTestMode::Run()
    {
        Host().RunManeuverFileTestMode();
    }

    CorridorRepeatabilityMode::CorridorRepeatabilityMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool CorridorRepeatabilityMode::Begin()
    {
        return Host().BeginCorridorRepeatabilityMode();
    }

    void CorridorRepeatabilityMode::Run()
    {
        Host().RunCorridorRepeatabilityMode();
    }

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
