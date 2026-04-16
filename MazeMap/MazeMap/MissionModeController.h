#pragma once

#include <memory>

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class MissionModeController final
    {
    public:
        explicit MissionModeController(SharedRobotRuntime& runtime);
        ~MissionModeController();

        MissionModeController(const MissionModeController&) = delete;
        MissionModeController& operator=(const MissionModeController&) = delete;
        MissionModeController(MissionModeController&&) = delete;
        MissionModeController& operator=(MissionModeController&&) = delete;

        bool BeginMissionRunMode();
        void RunMissionRunMode();

        bool BeginManeuverFileTestMode();
        void RunManeuverFileTestMode();

        bool BeginCorridorRepeatabilityMode();
        void RunCorridorRepeatabilityMode();

        bool BeginPositionAccuracyAuditMode();
        void RunPositionAccuracyAuditMode();

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };
}
