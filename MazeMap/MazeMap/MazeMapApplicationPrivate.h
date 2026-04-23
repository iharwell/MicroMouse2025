#pragma once

#include "Application.h"
#include "IApplicationMode.h"
#include "MazeMapApplicationRuntime.h"
#include "BootUtilityModeFramework.h"
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
#include "DiagnosticCoverage.h"
#include "DiagnosticLogBudget.h"
#include "DiagnosticMotionPlan.h"
#include "EncoderStallPolicy.h"
#include "FanRampProfile.h"
#include "GyroBiasUpdatePolicy.h"
#include "ImuCalibrationPolicy.h"
#include "ImuSamplingProfile.h"
#include "LaunchAssistProfile.h"
#include "CruiseSpeedFloor.h"
#include "MotionTargetProjection.h"
#include "MissionStartPolicy.h"
#include "MissionMazeExport.h"
#include "MotorModelUnits.h"
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

using MazeMap::App::Internal::IApplicationMode;
using MazeMap::App::Internal::BootUtilityModeFramework::AppendStartupTrace;
using MazeMap::App::Internal::BootUtilityModeFramework::ResetStartupTrace;

namespace MazeMap
{
    namespace Platform
    {
        int32_t ReadEncoderCount(uint8_t channel);
    }
}

#include "MazeMapRuntimeCore.h"
#include "RuntimeSensorSuite.h"

