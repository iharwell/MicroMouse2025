#pragma once

#include "Defines.h"

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
        PrimaryDiagnostic,
        Mission,
    };

    enum class BootModeCategory : uint8_t
    {
        Mission,
        Utility,
    };

    struct BootModeDescriptor
    {
        BootModeId id;
        BootModeCategory category;
        const char* stableId;
        const char* purposeSummary;
        const char* primaryOutputs;
        const char* entryPoint;
        const char* implementationFile;
        const char* majorPhases;
        const char* sharedTuning;
        const char* explicitTuningOverrides;
        const char* expectedArtifacts;
    };
}
