#pragma once
// Declares the LED-calibration phase state machine used by wall-sensor bring-up and validation routines.

#include <stdint.h>

namespace MazeMap
{
    enum class WallSensorLedCalibrationPhase : uint8_t
    {
        Front = 0U,
        Side = 1U,
        Complete = 2U
    };

    inline WallSensorLedCalibrationPhase AdvanceWallSensorLedCalibrationPhase(
        WallSensorLedCalibrationPhase currentPhase,
        bool jumperInstalled) noexcept
    {
        switch (currentPhase)
        {
        case WallSensorLedCalibrationPhase::Front:
            return jumperInstalled ? WallSensorLedCalibrationPhase::Front : WallSensorLedCalibrationPhase::Side;
        case WallSensorLedCalibrationPhase::Side:
            return jumperInstalled ? WallSensorLedCalibrationPhase::Complete : WallSensorLedCalibrationPhase::Side;
        default:
            return WallSensorLedCalibrationPhase::Complete;
        }
    }
}

using WallSensorLedCalibrationPhase = MazeMap::WallSensorLedCalibrationPhase;
