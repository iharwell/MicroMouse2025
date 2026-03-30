#pragma once

namespace MazeMapApp::Internal
{
    class IMissionModeHost
    {
    public:
        virtual ~IMissionModeHost() = default;
        virtual bool BeginMissionRunMode() = 0;
        virtual void RunMissionRunMode() = 0;
        virtual bool BeginManeuverFileTestMode() = 0;
        virtual void RunManeuverFileTestMode() = 0;
        virtual bool BeginCorridorRepeatabilityMode() = 0;
        virtual void RunCorridorRepeatabilityMode() = 0;
        virtual bool BeginPositionAccuracyAuditMode() = 0;
        virtual void RunPositionAccuracyAuditMode() = 0;
    };
}
