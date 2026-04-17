#include "pch.h"
#include "MissionRunMode.h"

#include "MazeMapSharedRuntime.h"
#include "MissionModeController.h"

namespace MazeMap::App::Internal
{
    class MissionRunMode::Implementation final
    {
    public:
        explicit Implementation(SharedRobotRuntime& runtime)
            : routineController(runtime)
        {
        }

        MissionModeController routineController;
    };

    MissionRunMode::MissionRunMode(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    MissionRunMode::~MissionRunMode() = default;

    bool MissionRunMode::Begin()
    {
        return _impl->routineController.BeginMissionRunRoutine();
    }

    void MissionRunMode::Run()
    {
        _impl->routineController.RunMissionRunRoutine();
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
            "MissionModeController.cpp",
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
