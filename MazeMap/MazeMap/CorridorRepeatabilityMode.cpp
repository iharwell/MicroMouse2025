#include "pch.h"
#include "CorridorRepeatabilityMode.h"

#include "MazeMapControllerRegistry.h"

namespace MazeMap::App::Internal
{
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::CorridorRepeatability,
            BootModeCategory::Utility,
            "corridor_repeatability",
            "Run the fixed-corridor mapping repeatability sweep.",
            "logging.txt; corridor repeatability telemetry mmlog",
            &GetCorridorRepeatabilityMode,
            "GetCorridorRepeatabilityMode",
            "CorridorRepeatabilityMode.cpp; MazeMapMissionController.cpp",
            "mission-family initialization; telemetry log setup; corridor speed passes",
            "AuxMeasurementConfig corridor profile; CoreConfig mission tuning; Maneuver classes",
            "corridor geometry, speed points, and repeatability limits are auxiliary-profile deltas",
            "corridor_repeatability.mmlog",
        };
        return descriptor;
    }

    CorridorRepeatabilityMode::CorridorRepeatabilityMode(IMissionModeHost& host)
        : MissionHostedModeBase(host)
    {
    }

    bool CorridorRepeatabilityMode::Begin()
    {
        return Host().BeginCorridorRepeatabilityMode();
    }

    void CorridorRepeatabilityMode::Run()
    {
        Host().RunCorridorRepeatabilityMode();
    }

    IApplicationMode& GetCorridorRepeatabilityMode()
    {
        static CorridorRepeatabilityMode mode(GetMissionModeHost());
        return mode;
    }
}

