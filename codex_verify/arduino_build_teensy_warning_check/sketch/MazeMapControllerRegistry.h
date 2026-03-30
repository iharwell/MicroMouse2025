#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapControllerRegistry.h"
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
