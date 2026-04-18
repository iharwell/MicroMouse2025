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

    bool WallTouchRoutine::Begin(
        const float targetYawRad,
        const float minLatchTravelM,
        const float maxApproachTravelM,
        const bool allowPassThroughNoWall,
        const Runtime::WallTouchPoseResetTarget* const poseResetTarget,
        Runtime::WallTouchExecutionResult* const resultSink,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services,
        const Hooks& hooks)
    {
        LoopController::ModeCallbacks initialCallbacks{};
        return
            BeginSession(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                poseResetTarget,
                resultSink,
                returnCallbacks,
                initialCallbacks,
                hooks) &&
            (services.SetNextModeWorkCallbacks(initialCallbacks), true);
    }

    bool WallTouchRoutine::BeginSession(
        const float targetYawRad,
        const float minLatchTravelM,
        const float maxApproachTravelM,
        const bool allowPassThroughNoWall,
        const Runtime::WallTouchPoseResetTarget* const poseResetTarget,
        Runtime::WallTouchExecutionResult* const resultSink,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::ModeCallbacks& initialCallbacks,
        const Hooks& hooks)
    {
        return
            PrepareWallTouchPhase(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                poseResetTarget,
                resultSink,
                returnCallbacks,
                hooks) &&
            BuildInitialCallbacks(initialCallbacks);
    }

    void WallTouchRoutine::CancelActiveRoutine() noexcept
    {
        _activePhaseFaulted = false;
        ResetActiveRoutine();
    }

    bool WallTouchRoutine::PrepareWallTouchPhase(
        const float targetYawRad,
        const float minLatchTravelM,
        const float maxApproachTravelM,
        const bool allowPassThroughNoWall,
        const Runtime::WallTouchPoseResetTarget* const poseResetTarget,
        Runtime::WallTouchExecutionResult* const resultSink,
        const LoopController::ModeCallbacks& returnCallbacks,
        const Hooks& hooks) noexcept
    {
        if (!CanBeginPhase())
        {
            return false;
        }

        _activePhaseFaulted = false;
        _resultSink = resultSink;
        if (_resultSink != nullptr)
        {
            *_resultSink = Runtime::WallTouchExecutionResult{};
        }

        _returnCallbacks = returnCallbacks;
        _hooks = hooks;
        _wallTouchState = Runtime::WallTouchLoopState{};
        _settleState = SettleRoutineState{};
        _wallTouchState.targetYawRad = targetYawRad;
        _wallTouchState.minLatchTravelM = minLatchTravelM;
        _wallTouchState.maxApproachTravelM = maxApproachTravelM;
        _wallTouchState.allowPassThroughNoWall = allowPassThroughNoWall;
        _wallTouchState.poseResetTarget = poseResetTarget;
        _wallTouchState.startDistanceM = _drive->GetAverageDistanceMeters();
        _wallTouchState.touchStartMs = millis();
        _wallTouchState.stateStartMs = _wallTouchState.touchStartMs;
        _wallTouchState.lastMotionMs = _wallTouchState.touchStartMs;
        _wallTouchState.lastMotionTelemetry = _drive->GetTelemetry();
        _wallTouchState.approachDriveCommand = Config::kWallTouchDriveCommand;
        _wallTouchState.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
        _wallTouchState.previousCycleFrontSkewMagnitudeM = std::numeric_limits<float>::infinity();
        _wallTouchState.currentCycleStartYawRad = _drive->GetPose().yawRad;
        _wallTouchState.runtimeState = Runtime::WallTouchState::ContactSeek;
        ActivatePhase(&_wallTouchState, &WallTouchRoutine::WallTouchRoutineTick);

        if (_hooks.onTraceLine != nullptr)
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                Runtime::WallTouchStateName(Runtime::WallTouchState::EntryConditioning),
                Runtime::WallTouchStateName(_wallTouchState.runtimeState),
                0UL,
                0.0f);
            _hooks.onTraceLine(_hooks.context, line);
        }

        return true;
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
        _resultSink = nullptr;
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
        if (_resultSink != nullptr)
        {
            *_resultSink = _wallTouchState.result;
        }
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
