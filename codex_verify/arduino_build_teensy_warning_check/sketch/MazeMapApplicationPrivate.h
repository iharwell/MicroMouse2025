#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapApplicationPrivate.h"
#pragma once

#include "MazeMapApplication.h"
#include "MazeMapApplicationMode.h"
#include "MazeMapApplicationRuntime.h"
#include "MazeMapMissionModeHost.h"
#include "MazeMapMissionModes.h"
#include "Defines.h"
#include "TeensyLayout.h"
#include "Maze.h"
#include "PathFinder.h"
#include "Maneuver.h"
#include "ManeuverQueue.h"
#include "ManeuverPathFinder.h"
#include "MotorEncoderDrive.h"
#include "SearchRunPlanner.h"
#include "Vehicle.h"
#include "WallSensorCalibration.h"
#include "DirectionalLocation.h"
#include "Kinematics.h"
#include "CoreFileExport.h"
#include "DiagnosticCoverage.h"
#include "DiagnosticLogBudget.h"
#include "DiagnosticMotionPlan.h"
#include "EncoderStallPolicy.h"
#include "FanRampProfile.h"
#include "GyroBiasUpdatePolicy.h"
#include "ImuCalibrationPolicy.h"
#include "ImuSamplingProfile.h"
#include "InPlaceTurnProfile.h"
#include "LaunchAssistProfile.h"
#include "CruiseSpeedFloor.h"
#include "MotionTargetProjection.h"
#include "MissionStartPolicy.h"
#include "MissionMazeExport.h"
#include "MotorModelUnits.h"
#include "MouseUkf.h"
#include "OpenLoopDriveCommand.h"
#include "RollingAverageWindow.h"
#include "SmoothTurnYawRateController.h"
#include "TrackWidthEstimate.h"
#include "TurnCommandGeometry.h"
#include "TurnWallEdgeTracker.h"
#include "DiagonalWallCentering.h"
#include "FrontWallCharacterizationStorage.h"
#include "TractionLimitSweep.h"
#include "WallBeliefMap.h"
#include "WallDetectionThresholds.h"
#include "WallContactDetection.h"
#include "WallObservationPipeline.h"
#include "WheelControlProfile.h"

#if defined(ARDUINO_TEENSY41)
#include <EEPROM.h>
#endif

#include <cstring>
#include <ctype.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using MazeMapApp::Internal::IApplicationMode;
using MazeMapApp::Internal::IMissionModeHost;
using MazeMapApp::Internal::MissionRunMode;
using MazeMapApp::Internal::ManeuverFileTestMode;
using MazeMapApp::Internal::CorridorRepeatabilityMode;
using MazeMapApp::Internal::PositionAccuracyAuditMode;

namespace MazeMap
{
    namespace Platform
    {
        int32_t ReadEncoderCount(uint8_t channel);
    }
}

#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeDrive.h"
#include "MazeMapRuntimeSensors.h"
#include "MazeMapRuntimeInfrastructure.h"
