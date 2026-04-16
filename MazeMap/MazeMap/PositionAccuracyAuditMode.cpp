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
            "logging.txt; position accuracy telemetry mmlog",
            &GetPositionAccuracyAuditMode,
            "GetPositionAccuracyAuditMode",
            "PositionAccuracyAuditMode.cpp",
            "mission-family initialization; telemetry log setup; straight, turn, and smooth-turn audit phases",
            "AuxMeasurementConfig position-audit profile; CoreConfig mission tuning; Maneuver classes",
            "fixture geometry, speed points, and fan policy are auxiliary-profile deltas",
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
