#include "pch.h"
#include "PositionAccuracyAuditMode.h"

#include "MazeMapSharedRuntime.h"

namespace MazeMap::App::Internal
{
    PositionAccuracyAuditMode::PositionAccuracyAuditMode(SharedRobotRuntime& runtime)
        : _controller(runtime)
    {
    }

    bool PositionAccuracyAuditMode::Begin()
    {
        return _controller.BeginPositionAccuracyAuditMode();
    }

    void PositionAccuracyAuditMode::Run()
    {
        _controller.RunPositionAccuracyAuditMode();
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
            "PositionAccuracyAuditMode.cpp",
            "mission-family init; log setup; straight, turn, smooth-turn phases",
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
