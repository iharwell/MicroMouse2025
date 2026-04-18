#include "pch.h"
#include "WallTouchRoutine.h"

#include "DriveBase.h"
#include "MissionStartPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr MazeMap::CommandPD kWallTouchTrackingCommandPd =
            MazeMap::CommandPD::StateWheelOmegaPD |
            MazeMap::CommandPD::IMUYaw;

        constexpr float AverageDistanceMeters(const DriveTelemetry& telemetry) noexcept
        {
            return 0.5f * (telemetry.leftDistanceM + telemetry.rightDistanceM);
        }

        bool TryComputeWallTouchLaunchPose(
            const PoseEstimate& pose,
            const MazeMap::CellCoordinates& wallCell,
            const MazeMap::Direction wallDirection,
            CalibrationWall& calibrationWall,
            float& targetCoordinateM,
            float& expectedTravelM,
            float& poseResetXMeters,
            float& poseResetYMeters) noexcept
        {
            targetCoordinateM = 0.0f;
            expectedTravelM = 0.0f;
            poseResetXMeters = pose.xMeters;
            poseResetYMeters = pose.yMeters;
            if (!TryComputeWallTouchTargetCoordinateForCellWall(
                    wallCell,
                    wallDirection,
                    targetCoordinateM,
                    calibrationWall))
            {
                return false;
            }

            switch (calibrationWall)
            {
            case CalibrationWall::West:
            case CalibrationWall::East:
                poseResetXMeters = targetCoordinateM;
                expectedTravelM = std::fabs(targetCoordinateM - pose.xMeters);
                break;
            case CalibrationWall::South:
            case CalibrationWall::North:
                poseResetYMeters = targetCoordinateM;
                expectedTravelM = std::fabs(targetCoordinateM - pose.yMeters);
                break;
            default:
                return false;
            }

            return std::isfinite(targetCoordinateM) &&
                std::isfinite(expectedTravelM) &&
                std::isfinite(poseResetXMeters) &&
                std::isfinite(poseResetYMeters);
        }
    }

    WallTouchRoutine::WallTouchRoutine(DriveBase& drive) noexcept
        : _drive(&drive)
    {
    }

    LoopController::ControlVector WallTouchRoutine::ActiveRoutineThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if ((self == nullptr) || (self->_activeState == nullptr) || (self->_activePhaseTick == nullptr))
        {
            services.Fault("WallTouchRoutine callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_activePhaseTick)(self->_activeState, loopEndTimeUs, state, services);
    }

    bool WallTouchRoutine::Active() const noexcept
    {
        return (_activeState != nullptr) && (_activePhaseTick != nullptr);
    }

    bool WallTouchRoutine::ActivePhaseFaulted() const noexcept
    {
        return _activePhaseFaulted;
    }

    const Runtime::WallTouchExecutionResult& WallTouchRoutine::LastResult() const noexcept
    {
        return _lastResult;
    }

    bool WallTouchRoutine::Begin(
        const MazeMap::CellCoordinates& wallCell,
        const MazeMap::Direction wallDirection,
        const bool allowPassThroughNoWall,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services,
        const Hooks& hooks)
    {
        LoopController::ModeCallbacks initialCallbacks{};
        if (!PrepareWallTouchPhase(
                wallCell,
                wallDirection,
                allowPassThroughNoWall,
                hooks) ||
            !BuildInitialCallbacks(initialCallbacks))
        {
            return false;
        }

        services.SetNextModeWorkCallbacks(initialCallbacks);
        _returnCallbacks = returnCallbacks;
        return true;
    }

    bool WallTouchRoutine::PrepareInitialCallbacks(
        const MazeMap::CellCoordinates& wallCell,
        const MazeMap::Direction wallDirection,
        const bool allowPassThroughNoWall,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::ModeCallbacks& initialCallbacks,
        const Hooks& hooks)
    {
        if (!PrepareWallTouchPhase(
                wallCell,
                wallDirection,
                allowPassThroughNoWall,
                hooks))
        {
            return false;
        }

        _returnCallbacks = returnCallbacks;
        return BuildInitialCallbacks(initialCallbacks);
    }

    void WallTouchRoutine::CancelActiveRoutine() noexcept
    {
        _activePhaseFaulted = false;
        ResetActiveRoutine();
    }

    bool WallTouchRoutine::PrepareWallTouchPhase(
        const MazeMap::CellCoordinates& wallCell,
        const MazeMap::Direction wallDirection,
        const bool allowPassThroughNoWall,
        const Hooks& hooks) noexcept
    {
        float ignoredTargetCoordinateM = 0.0f;
        CalibrationWall ignoredCalibrationWall = CalibrationWall::West;
        if (!CanBeginPhase() ||
            !TryComputeWallTouchTargetCoordinateForCellWall(
                wallCell,
                wallDirection,
                ignoredTargetCoordinateM,
                ignoredCalibrationWall))
        {
            return false;
        }

        _activePhaseFaulted = false;
        _lastResult = Runtime::WallTouchExecutionResult{};
        _hooks = hooks;
        _wallTouchState = Runtime::WallTouchLoopState{};
        _settleState = SettleRoutineState{};
        _wallTouchState.wallCell = wallCell;
        _wallTouchState.wallDirection = wallDirection;
        _wallTouchState.allowPassThroughNoWall = allowPassThroughNoWall;
        ActivatePhase(&_wallTouchState, &WallTouchRoutine::WallTouchRoutineTick);
        return true;
    }

    bool WallTouchRoutine::CaptureLaunchBaseline(
        Runtime::WallTouchLoopState& wallTouch,
        const LoopController::ModeState& state) noexcept
    {
        if ((_drive == nullptr) || wallTouch.launchBaselineCaptured)
        {
            return _drive != nullptr;
        }

        const PoseEstimate& pose = state.estimate;
        float poseResetXMeters = pose.xMeters;
        float poseResetYMeters = pose.yMeters;
        if (!TryComputeWallTouchLaunchPose(
                pose,
                wallTouch.wallCell,
                wallTouch.wallDirection,
                wallTouch.calibrationWall,
                wallTouch.targetCoordinateM,
                wallTouch.expectedTravelM,
                poseResetXMeters,
                poseResetYMeters))
        {
            return false;
        }

        wallTouch.targetYawRad = DirectionToYawRad(wallTouch.wallDirection);
        wallTouch.minLatchTravelM = MazeMap::ComputeWallTouchMinimumLatchTravelM(
            wallTouch.expectedTravelM,
            Config::kWallTouchMinApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        wallTouch.maxApproachTravelM = MazeMap::ComputeWallTouchMaximumApproachDistanceM(
            wallTouch.expectedTravelM,
            Config::kWallTouchBaseMaxApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        wallTouch.poseResetXMeters = poseResetXMeters;
        wallTouch.poseResetYMeters = poseResetYMeters;
        wallTouch.poseResetYawRad = wallTouch.targetYawRad;
        wallTouch.poseResetEnabled = true;
        wallTouch.startDistanceM = AverageDistanceMeters(state.driveTelemetry);
        wallTouch.touchStartMs = millis();
        wallTouch.stateStartMs = wallTouch.touchStartMs;
        wallTouch.lastMotionMs = wallTouch.touchStartMs;
        wallTouch.lastMotionTelemetry = state.driveTelemetry;
        wallTouch.approachDriveCommand = Config::kWallTouchDriveCommand;
        wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
        wallTouch.previousCycleFrontSkewMagnitudeM = std::numeric_limits<float>::infinity();
        wallTouch.currentCycleStartYawRad = pose.yawRad;
        wallTouch.runtimeState = Runtime::WallTouchState::ContactSeek;
        wallTouch.launchBaselineCaptured = true;

        if (_hooks.onTraceLine != nullptr)
        {
            char line[256] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch_plan:wall=%s,expected=%.4f,min_latch=%.4f,max_travel=%.4f,target_yaw_deg=%.2f",
                CalibrationWallName(wallTouch.calibrationWall),
                wallTouch.expectedTravelM,
                wallTouch.minLatchTravelM,
                wallTouch.maxApproachTravelM,
                wallTouch.targetYawRad * RAD_TO_DEG_F);
            _hooks.onTraceLine(_hooks.context, line);

            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                Runtime::WallTouchStateName(Runtime::WallTouchState::EntryConditioning),
                Runtime::WallTouchStateName(wallTouch.runtimeState),
                0UL,
                0.0f);
            _hooks.onTraceLine(_hooks.context, line);
        }

        return std::isfinite(wallTouch.targetYawRad) &&
            std::isfinite(wallTouch.minLatchTravelM) &&
            std::isfinite(wallTouch.maxApproachTravelM);
    }

    bool WallTouchRoutine::CanBeginPhase() const noexcept
    {
        return (_drive != nullptr) && !Active();
    }

    bool WallTouchRoutine::BuildInitialCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept
    {
        if ((_activeState == nullptr) || (_activePhaseTick == nullptr))
        {
            callbacks = {};
            return false;
        }

        callbacks = {};
        callbacks.onModeWork = &WallTouchRoutine::ActiveRoutineThunk;
        callbacks.context = const_cast<WallTouchRoutine*>(this);
        return true;
    }

    bool WallTouchRoutine::BeginPassThroughSettlePhase(
        const char* const timeoutMessage,
        const std::uint16_t stationaryHoldMs,
        const std::uint16_t timeoutMs) noexcept
    {
        if (!Active())
        {
            return false;
        }

        _settleState = SettleRoutineState{};
        _settleState.timeoutMessage = timeoutMessage;
        _settleState.stationaryHoldMs = stationaryHoldMs;
        _settleState.timeoutMs = timeoutMs;
        ActivatePhase(&_settleState, &WallTouchRoutine::SettleRoutineTick);
        return true;
    }

    void WallTouchRoutine::ActivatePhase(
        void* const activeState,
        const ActivePhaseTickFn activePhaseTick) noexcept
    {
        _activeState = activeState;
        _activePhaseTick = activePhaseTick;
    }

    void WallTouchRoutine::ResetActiveRoutine() noexcept
    {
        _activeState = nullptr;
        _activePhaseTick = nullptr;
        _returnCallbacks = LoopController::ModeCallbacks{};
        _hooks = Hooks{};
        _wallTouchState = Runtime::WallTouchLoopState{};
        _settleState = SettleRoutineState{};
    }

    bool WallTouchRoutine::InvokeSampleHook(
        const bool stationary,
        const LoopController::ModeState& state) const
    {
        return (_hooks.onSample == nullptr) || _hooks.onSample(_hooks.context, stationary, state);
    }

    void WallTouchRoutine::PersistResult() noexcept
    {
        _lastResult = _wallTouchState.result;
    }

    bool WallTouchRoutine::IsDriveMotionSettled(
        const DriveTelemetry& stationaryReferenceTelemetry,
        const unsigned long stationaryReferenceMs,
        const DriveTelemetry& telemetry,
        const SensorSnapshot& snapshot,
        const unsigned long nowMs) const noexcept
    {
        const unsigned long elapsedMs = nowMs - stationaryReferenceMs;
        return MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            telemetry.leftDistanceM - stationaryReferenceTelemetry.leftDistanceM,
            telemetry.rightDistanceM - stationaryReferenceTelemetry.rightDistanceM,
            static_cast<float>(elapsedMs) * 1.0e-3f,
            snapshot.gyroRadps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    LoopController::ControlVector WallTouchRoutine::ReturnToContinuation(
        LoopController::TickServices& services) noexcept
    {
        const LoopController::ModeCallbacks callbacks = _returnCallbacks;
        if ((_hooks.onTraceLine != nullptr) &&
            _wallTouchState.launchBaselineCaptured &&
            (_wallTouchState.result.outcome == WallTouchOutcome::SeatedContact))
        {
            const DriveTelemetry telemetry = _drive->GetTelemetry();
            char line[256] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:wall=%s,travel=%.4f,expected=%.4f,min_latch=%.4f,final_yaw_err_deg=%.2f,left_v=%.4f,right_v=%.4f",
                CalibrationWallName(_wallTouchState.calibrationWall),
                _wallTouchState.result.seatedTravelM,
                _wallTouchState.expectedTravelM,
                _wallTouchState.minLatchTravelM,
                _wallTouchState.result.seatedYawErrorRad * RAD_TO_DEG_F,
                telemetry.leftVelocityMps,
                telemetry.rightVelocityMps);
            _hooks.onTraceLine(_hooks.context, line);
        }
        PersistResult();
        ResetActiveRoutine();
        if (callbacks.onModeWork != nullptr)
        {
            services.SetNextModeWorkCallbacks(callbacks);
        }
        else
        {
            services.RequestEndLoop();
        }
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector WallTouchRoutine::CompleteCurrentPhase(
        void* const nextState,
        const ActivePhaseTickFn nextPhaseTick,
        LoopController::TickServices& services) noexcept
    {
        if ((nextState != nullptr) && (nextPhaseTick != nullptr))
        {
            ActivatePhase(nextState, nextPhaseTick);
            return LoopController::ControlVector::Brake;
        }

        return ReturnToContinuation(services);
    }

    LoopController::ControlVector WallTouchRoutine::FaultPhase(
        LoopController::TickServices& services,
        const char* const reason) noexcept
    {
        _activePhaseFaulted = true;
        PersistResult();
        ResetActiveRoutine();
        services.Fault(reason);
        return LoopController::ControlVector::Brake;
    }

    void WallTouchRoutine::AppendTraceLineHook(void* const context, const char* const line) noexcept
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if ((self != nullptr) && (self->_hooks.onTraceLine != nullptr) && (line != nullptr))
        {
            self->_hooks.onTraceLine(self->_hooks.context, line);
        }
    }

    void WallTouchRoutine::PoseResetHook(void* const context) noexcept
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if ((self != nullptr) && (self->_hooks.onPoseReset != nullptr))
        {
            self->_hooks.onPoseReset(self->_hooks.context);
        }
    }

    LoopController::ControlVector WallTouchRoutine::BeginPassThroughSettleHook(
        void* const context,
        void* const rawState,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if ((self == nullptr) || (rawState != &self->_wallTouchState))
        {
            services.Fault("WallTouchRoutine pass-through settle context was not initialized");
            return LoopController::ControlVector::Brake;
        }

        if (!self->BeginPassThroughSettlePhase(
                "Wall touch-off failed to settle after pass-through",
                Config::kStartupWallCalibrationSettleMs,
                0U))
        {
            return self->FaultPhase(services, "Failed to begin wall-touch pass-through settle phase");
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector WallTouchRoutine::FaultHook(
        void* const context,
        LoopController::TickServices& services,
        const char* const reason) noexcept
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if (self == nullptr)
        {
            services.Fault(reason);
            return LoopController::ControlVector::Brake;
        }

        return self->FaultPhase(services, reason);
    }

    LoopController::ControlVector WallTouchRoutine::CompleteHook(
        void* const context,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<WallTouchRoutine*>(context);
        if (self == nullptr)
        {
            services.RequestEndLoop();
            return LoopController::ControlVector::Brake;
        }

        return self->CompleteCurrentPhase(nullptr, nullptr, services);
    }

    LoopController::ControlVector WallTouchRoutine::WallTouchRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& wallTouch = *static_cast<Runtime::WallTouchLoopState*>(rawState);
        if (!CaptureLaunchBaseline(wallTouch, state))
        {
            return FaultPhase(services, "WallTouchRoutine launch baseline is invalid");
        }
        if (!InvokeSampleHook(false, state))
        {
            return FaultPhase(services, "WallTouchRoutine sample hook failed");
        }

        Runtime::WallTouchLoopHooks hooks{};
        hooks.context = this;
        hooks.appendTraceLine = &WallTouchRoutine::AppendTraceLineHook;
        hooks.beginPassThroughSettle = &WallTouchRoutine::BeginPassThroughSettleHook;
        hooks.onPoseReset = &WallTouchRoutine::PoseResetHook;
        hooks.fault = &WallTouchRoutine::FaultHook;
        hooks.complete = &WallTouchRoutine::CompleteHook;

        return Runtime::DriveSharedWallTouchLoopTick(
            *_drive,
            rawState,
            wallTouch,
            state,
            services,
            hooks,
            kWallTouchTrackingCommandPd);
    }

    LoopController::ControlVector WallTouchRoutine::SettleRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)rawState;
        (void)loopEndTimeUs;
        auto& settle = _settleState;
        if (!settle.started)
        {
            settle.started = true;
            settle.startMs = millis();
        }

        if (!InvokeSampleHook(true, state))
        {
            return FaultPhase(services, "WallTouchRoutine sample hook failed");
        }

        const unsigned long nowMs = millis();
        if (!settle.stationaryWindowActive)
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
            settle.stationaryWindowActive = true;
        }
        else if (!IsDriveMotionSettled(
                     settle.stationaryStartTelemetry,
                     settle.stationaryStartMs,
                     state.driveTelemetry,
                     state.sensors,
                     nowMs))
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
        }
        else if ((nowMs - settle.stationaryStartMs) >= settle.stationaryHoldMs)
        {
            return CompleteCurrentPhase(nullptr, nullptr, services);
        }

        if ((settle.timeoutMs > 0U) && ((nowMs - settle.startMs) >= settle.timeoutMs))
        {
            return FaultPhase(
                services,
                (settle.timeoutMessage != nullptr) ?
                    settle.timeoutMessage :
                    "WallTouchRoutine settle timed out");
        }

        return LoopController::ControlVector::Brake;
    }
}
