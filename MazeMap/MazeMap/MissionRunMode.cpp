#include "pch.h"
#include "MissionRunMode.h"

#include "MazeMapControllerRegistry.h"

namespace MazeMap::App::Internal
{
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
            "MissionRunMode.cpp; MazeMapMissionController.cpp",
            "mission initialization; startup wall calibration; exploration; return; speed runs; service cycle",
            "CoreConfig mission tuning; shared runtime pathfinders; persisted front-wall characterization when available",
            "none",
            "telemetry mmlog; maze.txt when exported",
        };
        return descriptor;
    }

    MissionRunMode::MissionRunMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool MissionRunMode::Begin()
    {
        return Host().BeginMissionRunMode();
    }

    void MissionRunMode::Run()
    {
        Host().RunMissionRunMode();
    }

    IApplicationMode& GetMissionRunMode()
    {
        static MissionRunMode mode(GetMissionModeHost());
        return mode;
    }
}

