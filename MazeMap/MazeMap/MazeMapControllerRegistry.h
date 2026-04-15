#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"
#include "MazeMapMissionModeHost.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetAuxMeasurementMode();
    const BootModeDescriptor& GetAuxMeasurementBootModeDescriptor();
    IApplicationMode& GetFrontWallCharacterizationMode();
    const BootModeDescriptor& GetFrontWallCharacterizationBootModeDescriptor();
    IApplicationMode& GetWallSensorLedCalibrationMode();
    const BootModeDescriptor& GetWallSensorLedCalibrationBootModeDescriptor();
    IApplicationMode& GetDiagnosticMode();
    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
    IApplicationMode& GetTopSpeedMeasurementMode();
    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor();
    IApplicationMode& GetMissionRunMode();
    IApplicationMode& GetManeuverFileTestMode();
    IApplicationMode& GetCorridorRepeatabilityMode();
    IApplicationMode& GetPositionAccuracyAuditMode();
    IMissionModeHost& GetMissionModeHost();
    const BootModeDescriptor& GetMissionRunBootModeDescriptor();
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor();
    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor();
    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor();
}
