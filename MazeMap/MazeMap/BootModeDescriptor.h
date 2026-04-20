#pragma once

#include "Defines.h"

namespace MazeMap::App::Internal
{
    class IApplicationMode;
}

namespace MazeMap::App
{
    enum class BootModeId : uint8_t
    {
        FrontWallCharacterization,
        WallSensorLedCalibration,
        AuxiliaryMeasurement,
        CorridorRepeatability,
        PositionAccuracyAudit,
        ManeuverFileTest,
        TopSpeedMeasurement,
        OpenFloorMeasurement,
        Mission,
    };

    enum class BootModeCategory : uint8_t
    {
        Mission,
        Utility,
    };

    using BootModeEntryMode = Internal::IApplicationMode& (*)();

    struct BootModeDescriptor
    {
        BootModeId id;
        BootModeCategory category;
        const char* stableId;
        const char* purposeSummary;
        const char* primaryOutputs;
        BootModeEntryMode entryMode;
        const char* entryPoint;
        const char* implementationFile;
        const char* majorPhases;
        const char* sharedTuning;
        const char* explicitTuningOverrides;
        const char* expectedArtifacts;
    };
}
