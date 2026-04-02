#pragma once
// Declares the mission-mode host callbacks used by the standalone mission controller modes.

namespace MazeMap::App::Internal
{
    // Defines the host services that concrete mission modes use to enter and run each mission workflow.
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

