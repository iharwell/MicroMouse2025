#include "pch.h"
#include "PositionAccuracyAuditMode.h"

#include "MazeMapSharedRuntime.h"
#include "MazeRunningAuditController.h"

namespace MazeMap::App::Internal
{
    class PositionAccuracyAuditMode::Implementation final
    {
    public:
        explicit Implementation(SharedRobotRuntime& runtime)
            : routineController(runtime)
        {
        }

        MazeRunningAuditController routineController;
    };

    PositionAccuracyAuditMode::PositionAccuracyAuditMode(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    PositionAccuracyAuditMode::~PositionAccuracyAuditMode() = default;

    bool PositionAccuracyAuditMode::Begin()
    {
        return _impl->routineController.BeginPositionAccuracyAuditRoutine();
    }

    void PositionAccuracyAuditMode::Run()
    {
        _impl->routineController.RunPositionAccuracyAuditRoutine();
    }

    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PositionAccuracyAudit,
            BootModeCategory::Utility,
            "position_accuracy_audit",
            "Run the fixed-fixture position accuracy audit.",
            "logging.txt; position-audit mmlog",
            &GetPositionAccuracyAuditMode,
            "GetPositionAccuracyAuditMode",
            "MazeRunningAuditController.cpp",
            "mode initialization; log setup; straight, turn, smooth-turn phases",
            "AuxMeasurementConfig position-audit profile; CoreConfig mission tuning; Maneuvers",
            "Fixture geometry, speed points, and fan policy are profile deltas",
            "position_accuracy_audit.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetPositionAccuracyAuditMode()
    {
        static PositionAccuracyAuditMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
