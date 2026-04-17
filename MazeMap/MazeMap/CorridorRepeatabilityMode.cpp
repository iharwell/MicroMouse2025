#include "pch.h"
#include "CorridorRepeatabilityMode.h"

#include "MazeMapSharedRuntime.h"

namespace MazeMap::App::Internal
{
    CorridorRepeatabilityMode::CorridorRepeatabilityMode(SharedRobotRuntime& runtime)
        : _controller(runtime)
    {
    }

    bool CorridorRepeatabilityMode::Begin()
    {
        return _controller.BeginCorridorRepeatabilityMode();
    }

    void CorridorRepeatabilityMode::Run()
    {
        _controller.RunCorridorRepeatabilityMode();
    }

    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::CorridorRepeatability,
            BootModeCategory::Utility,
            "corridor_repeatability",
            "Run the fixed-corridor mapping repeatability sweep.",
            "logging.txt; corridor repeatability mmlog",
            &GetCorridorRepeatabilityMode,
            "GetCorridorRepeatabilityMode",
            "CorridorRepeatabilityMode.cpp",
            "mission-family init; log setup; corridor speed passes",
            "AuxMeasurementConfig corridor profile; CoreConfig mission tuning; Maneuvers",
            "Corridor geometry, speed points, and repeatability limits are profile deltas",
            "corridor_repeatability.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetCorridorRepeatabilityMode()
    {
        static CorridorRepeatabilityMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
