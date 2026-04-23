#include "pch.h"
#include "ManeuverExecutor.h"

#include "Drive.h"
#include "DriveBase.h"
#include "SharedRobotRuntime.h"

namespace MazeMap::App::Internal
{
    LoopController::ControlVector ManeuverExecutor::ActiveRoutineThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<ManeuverExecutor*>(context);
        if ((self == nullptr) || (self->_activeState == nullptr) || (self->_activePhaseTick == nullptr))
        {
            services.Fault("ManeuverExecutor callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_activePhaseTick)(self->_activeState, loopEndTimeUs, state, services);
    }

    void ManeuverExecutor::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _drive = &runtime.Drive();
        _driveService = &runtime.DriveService();
    }

    bool ManeuverExecutor::Active() const noexcept
    {
        return (_activeState != nullptr) && (_activePhaseTick != nullptr);
    }

    void ManeuverExecutor::CancelActivePhase() noexcept
    {
        ResetActiveRoutine();
    }

    bool ManeuverExecutor::CanBeginPhase() const noexcept
    {
        return
            (_runtime != nullptr) &&
            (_drive != nullptr) &&
            (_driveService != nullptr) &&
            !Active();
    }

    bool ManeuverExecutor::BeginPhase(
        void* const activeState,
        const ActivePhaseTickFn activePhaseTick) noexcept
    {
        if (!CanBeginPhase() || (activeState == nullptr) || (activePhaseTick == nullptr))
        {
            return false;
        }

        _returnCallbacks = LoopController::ModeCallbacks{};
        ActivatePhase(activeState, activePhaseTick);
        return true;
    }

    bool ManeuverExecutor::BuildRoutineCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept
    {
        if ((_activeState == nullptr) || (_activePhaseTick == nullptr))
        {
            callbacks = {};
            return false;
        }

        callbacks = {};
        callbacks.onModeWork = &ManeuverExecutor::ActiveRoutineThunk;
        callbacks.context = const_cast<ManeuverExecutor*>(this);
        return true;
    }

    bool ManeuverExecutor::InstallRoutineCallbacks(
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services)
    {
        if ((returnCallbacks.onModeWork == nullptr) ||
            (_activeState == nullptr) ||
            (_activePhaseTick == nullptr))
        {
            ResetActiveRoutine();
            return false;
        }

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &ManeuverExecutor::ActiveRoutineThunk;
        callbacks.context = this;
        services.SetNextModeWorkCallbacks(callbacks);
        _returnCallbacks = returnCallbacks;
        return true;
    }

    void ManeuverExecutor::ActivatePhase(
        void* const activeState,
        const ActivePhaseTickFn activePhaseTick) noexcept
    {
        _activeState = activeState;
        _activePhaseTick = activePhaseTick;
    }

    void ManeuverExecutor::ResetActiveRoutine() noexcept
    {
        _activeState = nullptr;
        _activePhaseTick = nullptr;
        _returnCallbacks = LoopController::ModeCallbacks{};
        if ((_driveService != nullptr) && _driveService->Active())
        {
            _driveService->Cancel();
        }
        _delegatedDriveState = DelegatedDriveRoutineState{};
        _queueState = QueueRoutineState{};
    }

    LoopController::ControlVector ManeuverExecutor::ReturnToContinuation(
        LoopController::TickServices& services) noexcept
    {
        const LoopController::ModeCallbacks callbacks = _returnCallbacks;
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

    LoopController::ControlVector ManeuverExecutor::CompleteCurrentPhase(
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

    LoopController::ControlVector ManeuverExecutor::FaultPhase(
        LoopController::TickServices& services,
        const char* const reason) noexcept
    {
        ResetActiveRoutine();
        services.Fault(reason);
        return LoopController::ControlVector::Brake;
    }

    bool ManeuverExecutor::SEND_IT(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        const bool snapToExpectedLocation,
        MazeMap::DirectionalLocation& currentLocation,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services)
    {
        if (returnCallbacks.onModeWork == nullptr)
        {
            return false;
        }

        if (queue.empty())
        {
            services.SetNextModeWorkCallbacks(returnCallbacks);
            return true;
        }

        currentLocation = queue.front().getStart();
        _queueState = QueueRoutineState{};
        _queueState.queue = &queue;
        _queueState.currentLocation = &currentLocation;
        _queueState.limits = limits;
        _queueState.snapToExpectedLocation = snapToExpectedLocation;
        return
            BeginPhase(&_queueState, &ManeuverExecutor::QueueDispatchRoutineTick) &&
            InstallRoutineCallbacks(returnCallbacks, services);
    }

    bool ManeuverExecutor::SEND_IT(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        const bool snapToExpectedLocation,
        MazeMap::DirectionalLocation& currentLocation,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::ModeCallbacks& initialCallbacks)
    {
        initialCallbacks = {};
        if (returnCallbacks.onModeWork == nullptr)
        {
            return false;
        }

        if (queue.empty())
        {
            initialCallbacks = returnCallbacks;
            return true;
        }

        currentLocation = queue.front().getStart();
        _queueState = QueueRoutineState{};
        _queueState.queue = &queue;
        _queueState.currentLocation = &currentLocation;
        _queueState.limits = limits;
        _queueState.snapToExpectedLocation = snapToExpectedLocation;
        if (!BeginPhase(&_queueState, &ManeuverExecutor::QueueDispatchRoutineTick) ||
            !BuildRoutineCallbacks(initialCallbacks))
        {
            ResetActiveRoutine();
            initialCallbacks = {};
            return false;
        }

        _returnCallbacks = returnCallbacks;
        return true;
    }

    LoopController::ControlVector ManeuverExecutor::DelegatedDriveRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& delegated = *static_cast<DelegatedDriveRoutineState*>(rawState);
        if (_driveService == nullptr)
        {
            return FaultPhase(services, "Queued maneuver Drive service was unavailable");
        }
        if (!_driveService->Active())
        {
            return CompleteCurrentPhase(delegated.nextState, delegated.nextPhaseTick, services);
        }

        bool done = false;
        const LoopController::ControlVector control = _driveService->GetNextControls(done);
        if (!done)
        {
            return control;
        }

        return CompleteCurrentPhase(delegated.nextState, delegated.nextPhaseTick, services);
    }

    LoopController::ControlVector ManeuverExecutor::QueueDispatchRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) || (queueState.currentLocation == nullptr))
        {
            return FaultPhase(services, "Queued maneuver routine state was invalid");
        }
        if (queueState.nextIndex >= queueState.queue->size())
        {
            return ReturnToContinuation(services);
        }

        queueState.activeIndex = queueState.nextIndex;
        const MazeMap::ManeuverInstance& entry = (*queueState.queue)[queueState.activeIndex];
        *queueState.currentLocation = entry.getStart();
        if (entry.getCode() == MazeMap::MC_NONE)
        {
            return FaultPhase(services, "Queued maneuver entry was invalid");
        }

        // Temporary bridge: ManeuverExecutor now delegates queued maneuver execution to Drive so
        // Drive remains the sole owner of PD selection and low-level tracking. This is incomplete
        // relative to ManeuverExecutor's intended final role; queue-specific execution behavior
        // still needs to be rebuilt on top of Drive rather than bypassing it.
        _delegatedDriveState = DelegatedDriveRoutineState{};
        _driveService->Cancel();
        _driveService->SetLimits(queueState.limits);
        _driveService->StartManeuver(entry);
        if (!_driveService->Active())
        {
            return FaultPhase(services, "Queued maneuver Drive primitive could not start");
        }

        _delegatedDriveState.nextState = &queueState;
        _delegatedDriveState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
        return CompleteCurrentPhase(
            &_delegatedDriveState,
            &ManeuverExecutor::DelegatedDriveRoutineTick,
            services);
    }

    LoopController::ControlVector ManeuverExecutor::QueueAdvanceRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) ||
            (queueState.currentLocation == nullptr) ||
            (queueState.activeIndex >= queueState.queue->size()))
        {
            return FaultPhase(services, "Queued maneuver advance state was invalid");
        }

        const MazeMap::ManeuverInstance& entry = (*queueState.queue)[queueState.activeIndex];
        *queueState.currentLocation = entry.getEnd();

        if (queueState.snapToExpectedLocation)
        {
            _drive->SetStartPoint(*queueState.currentLocation);
        }

        queueState.nextIndex = static_cast<std::uint16_t>(queueState.activeIndex + 1U);
        if (queueState.nextIndex >= queueState.queue->size())
        {
            return ReturnToContinuation(services);
        }

        return CompleteCurrentPhase(&queueState, &ManeuverExecutor::QueueDispatchRoutineTick, services);
    }
}
