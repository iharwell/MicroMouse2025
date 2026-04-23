#include "pch.h"
#include "EncoderProgressWatchdog.h"

#include "CoreConfig.h"
#include "EncoderStallPolicy.h"

void EncoderProgressWatchdog::Reset(float traveledM, unsigned long nowMs) noexcept
{
    _lastProgressM = traveledM;
    _lastProgressMs = nowMs;
    _activeMotionStartMs = nowMs;
    _activeMotionCommand = false;
}

bool EncoderProgressWatchdog::Stalled(float traveledM, float commandedSpeedMps, float remainingM, unsigned long nowMs) noexcept
{
    if ((traveledM - _lastProgressM) >= MazeMap::Config::kEncoderProgressEpsilonM)
    {
        _lastProgressM = traveledM;
        _lastProgressMs = nowMs;
    }

    if (!MazeMap::IsEncoderProgressWatchdogArmed(
            commandedSpeedMps,
            remainingM,
            _activeMotionCommand ? (nowMs - _activeMotionStartMs) : 0UL,
            MazeMap::Config::kEncoderStallCommandThresholdMps,
            MazeMap::Config::kDistanceToleranceM,
            MazeMap::Config::kEncoderStallStartupGraceMs))
    {
        if ((commandedSpeedMps >= MazeMap::Config::kEncoderStallCommandThresholdMps) && (remainingM > MazeMap::Config::kDistanceToleranceM))
        {
            if (!_activeMotionCommand)
            {
                _activeMotionStartMs = nowMs;
                _activeMotionCommand = true;
                _lastProgressMs = nowMs;
            }
        }
        else
        {
            _activeMotionStartMs = nowMs;
            _activeMotionCommand = false;
        }

        _lastProgressMs = nowMs;
        return false;
    }

    return static_cast<unsigned long>(nowMs - _lastProgressMs) >= MazeMap::Config::kEncoderStallTimeoutMs;
}
