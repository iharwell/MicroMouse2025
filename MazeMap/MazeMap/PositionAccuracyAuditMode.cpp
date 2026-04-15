#include "pch.h"
#include "PositionAccuracyAuditMode.h"

#include "MazeMapControllerRegistry.h"

namespace MazeMap::App::Internal
{
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
            "PositionAccuracyAuditMode.cpp; MazeMapMissionController.cpp",
            "mission-family initialization; telemetry log setup; straight, turn, and smooth-turn audit phases",
            "AuxMeasurementConfig position-audit profile; CoreConfig mission tuning; Maneuver classes",
            "fixture geometry, speed points, and fan policy are auxiliary-profile deltas",
            "position_accuracy_audit.mmlog",
        };
        return descriptor;
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

    IApplicationMode& GetPositionAccuracyAuditMode()
    {
        static PositionAccuracyAuditMode mode(GetMissionModeHost());
        return mode;
    }
}

