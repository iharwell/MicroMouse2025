#pragma once

#include "MazeMapApplicationMode.h"
#include "MazeMapMissionModeHost.h"

namespace MazeMapApp::Internal
{
    IApplicationMode& GetAuxMeasurementMode();
    IApplicationMode& GetFrontWallCharacterizationMode();
    IApplicationMode& GetWallSensorLedCalibrationMode();
    IApplicationMode& GetDiagnosticMode();
    IMissionModeHost& GetMissionModeHost();
}
