#pragma once

#include "BootModeDescriptor.h"
#include "IApplicationMode.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetAuxMeasurementMode();
    const BootModeDescriptor& GetAuxMeasurementBootModeDescriptor();
    IApplicationMode& GetFrontWallCharacterizationMode();
    const BootModeDescriptor& GetFrontWallCharacterizationBootModeDescriptor();
    IApplicationMode& GetWallSensorLedCalibrationMode();
    const BootModeDescriptor& GetWallSensorLedCalibrationBootModeDescriptor();
    IApplicationMode& GetOpenFloorMeasurementMode();
    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
    IApplicationMode& GetShowcasingDonutMode();
    const BootModeDescriptor& GetShowcasingDonutBootModeDescriptor();
    IApplicationMode& GetTopSpeedMeasurementMode();
    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor();
    IApplicationMode& GetMissionRunMode();
    IApplicationMode& GetManeuverFileTestMode();
    IApplicationMode& GetCorridorRepeatabilityMode();
    IApplicationMode& GetPositionAccuracyAuditMode();
    const BootModeDescriptor& GetMissionRunBootModeDescriptor();
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor();
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor();
    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor();
}
