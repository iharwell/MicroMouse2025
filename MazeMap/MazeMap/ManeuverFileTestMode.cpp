#include "pch.h"
#include "ManeuverFileTestMode.h"

#include "MazeMapSharedRuntime.h"

namespace MazeMap::App::Internal
{
    ManeuverFileTestMode::ManeuverFileTestMode(SharedRobotRuntime& runtime)
        : _controller(runtime)
    {
    }

    bool ManeuverFileTestMode::Begin()
    {
        return _controller.BeginManeuverFileTestMode();
    }

    void ManeuverFileTestMode::Run()
    {
        _controller.RunManeuverFileTestMode();
    }

    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ManeuverFileTest,
            BootModeCategory::Utility,
            "maneuver_file_test",
            "Load and execute the maneuver queue stored in test.txt.",
            "logging.txt; maneuver test telemetry mmlog",
            &GetManeuverFileTestMode,
            "GetManeuverFileTestMode",
            "ManeuverFileTestMode.cpp",
            "mission-family initialization; telemetry log setup; test.txt load; queue execution",
            "CoreConfig mission tuning; Maneuver classes; shared runtime pathfinders",
            "test.txt is the selected input artifact for this utility workflow",
            "maneuver_test.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetManeuverFileTestMode()
    {
        static ManeuverFileTestMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
