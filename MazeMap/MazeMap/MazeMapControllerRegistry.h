#pragma once

#include "MazeMapApplicationMode.h"
#include "MazeMapMissionModeHost.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetAuxMeasurementMode();
    IApplicationMode& GetFrontWallCharacterizationMode();
    IApplicationMode& GetWallSensorLedCalibrationMode();
    IApplicationMode& GetDiagnosticMode();
    IMissionModeHost& GetMissionModeHost();
}

