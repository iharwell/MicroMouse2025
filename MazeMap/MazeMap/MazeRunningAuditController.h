#pragma once

#include <memory>

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class MazeRunningAuditController final
    {
    public:
        explicit MazeRunningAuditController(SharedRobotRuntime& runtime);
        ~MazeRunningAuditController();

        MazeRunningAuditController(const MazeRunningAuditController&) = delete;
        MazeRunningAuditController& operator=(const MazeRunningAuditController&) = delete;
        MazeRunningAuditController(MazeRunningAuditController&&) = delete;
        MazeRunningAuditController& operator=(MazeRunningAuditController&&) = delete;

        bool BeginCorridorRepeatabilityRoutine();
        void RunCorridorRepeatabilityRoutine();
        bool BeginPositionAccuracyAuditRoutine();
        void RunPositionAccuracyAuditRoutine();

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };
}
