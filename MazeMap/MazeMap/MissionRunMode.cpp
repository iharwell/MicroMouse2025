#include "pch.h"
#include "MissionRunMode.h"

#include "MazeMapSharedRuntime.h"

namespace MazeMap::App::Internal
{
    MissionRunMode::MissionRunMode(SharedRobotRuntime& runtime)
        : _controller(runtime)
    {
    }

    bool MissionRunMode::Begin()
    {
        return _controller.BeginMissionRunMode();
    }

    void MissionRunMode::Run()
    {
        _controller.RunMissionRunMode();
    }

    const BootModeDescriptor& GetMissionRunBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::Mission,
            BootModeCategory::Mission,
            "mission",
            "Run the normal exploration and speed-run mission workflow.",
            "logging.txt; mission telemetry; maze snapshot when exported",
            &GetMissionRunMode,
            "GetMissionRunMode",
            "MissionRunMode.cpp",
            "mission initialization; startup wall calibration; exploration; return; speed runs; service cycle",
            "CoreConfig mission tuning; shared runtime pathfinders; persisted front-wall characterization when available",
            "none",
            "telemetry mmlog; maze.txt when exported",
        };
        return descriptor;
    }

    IApplicationMode& GetMissionRunMode()
    {
        static MissionRunMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
