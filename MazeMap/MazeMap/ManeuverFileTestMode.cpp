#include "pch.h"
#include "ManeuverFileTestMode.h"

#include "BootModeDescriptor.h"

namespace MazeMap::App::Internal
{
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ManeuverFileTest,
            BootModeCategory::Utility,
            "maneuver_file_test",
            "Load and execute the maneuver queue stored in test.txt.",
            "logging.txt; maneuver test telemetry mmlog",
            "ManeuverFileTestMode::Begin/Run",
            "ManeuverFileTestMode.cpp; MazeMapMissionController.cpp",
            "mission-family initialization; telemetry log setup; test.txt load; queue execution",
            "CoreConfig mission tuning; Maneuver classes; shared runtime pathfinders",
            "test.txt is the selected input artifact for this utility workflow",
            "maneuver_test.mmlog",
        };
        return descriptor;
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
}

