#include "pch.h"
#include "ManeuverExecutor.h"

#include "Drive.h"
#include "DriveBase.h"
#include "SharedRobotRuntime.h"

namespace MazeMap::App::Internal
{
    CommandVector ManeuverExecutor::ActiveRoutineThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        auto* const self = static_cast<ManeuverExecutor*>(context);
        if ((self == nullptr) || (self->_activeState == nullptr) || (self->_activePhaseTick == nullptr))
        {
            if ((self != nullptr) && (self->_runtime != nullptr))
            {
                self->_runtime->FailActiveMode("ManeuverExecutor callback dispatch was not initialized");
            }

            while (true)
            {
            }
        }

        return (self->*self->_activePhaseTick)(self->_activeState, loopEndTimeUs, state, loopController);
    }

    void ManeuverExecutor::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _drive = &runtime.Drive();
        _driveService = &runtime.DriveService();
    }

    [[noreturn]] void ManeuverExecutor::FailInvariant(const char* const reason) const noexcept
    {
        if (_runtime != nullptr)
        {
            _runtime->FailActiveMode(reason);
        }

        while (true)
        {
        }
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
        const ActivePhaseTickFn activePhaseTick,
        const LoopController::ModeWorkCallback continuationCallback,
        void* const continuationContext) noexcept
    {
        if (!CanBeginPhase() || (activeState == nullptr) || (activePhaseTick == nullptr))
        {
            return false;
        }

        if (continuationCallback == nullptr)
        {
            FailInvariant("ManeuverExecutor continuation was not installed");
        }

        _continuation = Continuation{};
        _continuation.callback = continuationCallback;
        _continuation.context = continuationContext;
        ActivatePhase(activeState, activePhaseTick);
        return true;
    }

    bool ManeuverExecutor::InstallRoutineCallback(LoopController& loopController) noexcept
    {
        if ((_activeState == nullptr) || (_activePhaseTick == nullptr) || (_continuation.callback == nullptr))
        {
            ResetActiveRoutine();
            return false;
        }

        loopController.SetNextModeWorkCallback(&ManeuverExecutor::ActiveRoutineThunk, this);
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
        _continuation = Continuation{};
        _delegatedDriveState = DelegatedDriveRoutineState{};
        _queueState = QueueRoutineState{};
    }

    CommandVector ManeuverExecutor::ReturnToContinuation(
        LoopController& loopController) noexcept
    {
        const Continuation continuation = _continuation;
        ResetActiveRoutine();
        if (continuation.callback == nullptr)
        {
            FailInvariant("ManeuverExecutor continuation was not installed");
        }

        loopController.SetNextModeWorkCallback(continuation.callback, continuation.context);
        return CommandVector::Brake();
    }

    CommandVector ManeuverExecutor::CompleteCurrentPhase(
        void* const nextState,
        const ActivePhaseTickFn nextPhaseTick,
        LoopController& loopController) noexcept
    {
        if ((nextState != nullptr) && (nextPhaseTick != nullptr))
        {
            ActivatePhase(nextState, nextPhaseTick);
            return CommandVector::Brake();
        }

        return ReturnToContinuation(loopController);
    }

    CommandVector ManeuverExecutor::FaultPhase(
        const char* const reason) noexcept
    {
        ResetActiveRoutine();
        FailInvariant(reason);
        return CommandVector::Brake();
    }

    bool ManeuverExecutor::SEND_IT(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        const bool snapToExpectedLocation,
        MazeMap::DirectionalLocation& currentLocation,
        const LoopController::ModeWorkCallback continuationCallback,
        void* const continuationContext,
        LoopController& loopController)
    {
        if (continuationCallback == nullptr)
        {
            FailInvariant("ManeuverExecutor continuation was not installed");
        }

        if (queue.empty())
        {
            loopController.SetNextModeWorkCallback(continuationCallback, continuationContext);
            return true;
        }

        currentLocation = queue.front().getStart();
        _queueState = QueueRoutineState{};
        _queueState.queue = &queue;
        _queueState.currentLocation = &currentLocation;
        _queueState.limits = limits;
        _queueState.snapToExpectedLocation = snapToExpectedLocation;
        return
            BeginPhase(
                &_queueState,
                &ManeuverExecutor::QueueDispatchRoutineTick,
                continuationCallback,
                continuationContext) &&
            InstallRoutineCallback(loopController);
    }

    CommandVector ManeuverExecutor::DelegatedDriveRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& delegated = *static_cast<DelegatedDriveRoutineState*>(rawState);
        if (_driveService == nullptr)
        {
            return FaultPhase("Queued maneuver Drive service was unavailable");
        }

        bool done = false;
        const CommandVector control = _driveService->GetNextControls(done);
        if (!done)
        {
            return control;
        }

        return CompleteCurrentPhase(delegated.nextState, delegated.nextPhaseTick, loopController);
    }

    CommandVector ManeuverExecutor::QueueDispatchRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) || (queueState.currentLocation == nullptr))
        {
            return FaultPhase("Queued maneuver routine state was invalid");
        }
        if (queueState.nextIndex >= queueState.queue->size())
        {
            return ReturnToContinuation(loopController);
        }

        queueState.activeIndex = queueState.nextIndex;
        const MazeMap::ManeuverInstance& entry = (*queueState.queue)[queueState.activeIndex];
        *queueState.currentLocation = entry.getStart();
        if (entry.getCode() == MazeMap::MC_NONE)
        {
            return FaultPhase("Queued maneuver entry was invalid");
        }

        // Temporary bridge: ManeuverExecutor now delegates queued maneuver execution to Drive so
        // Drive remains the sole owner of PD selection and low-level tracking. This is incomplete
        // relative to ManeuverExecutor's intended final role; queue-specific execution behavior
        // still needs to be rebuilt on top of Drive rather than bypassing it.
        _delegatedDriveState = DelegatedDriveRoutineState{};
        _driveService->SetLimits(queueState.limits);
        _driveService->StartManeuver(entry);

        _delegatedDriveState.nextState = &queueState;
        _delegatedDriveState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
        return CompleteCurrentPhase(
            &_delegatedDriveState,
            &ManeuverExecutor::DelegatedDriveRoutineTick,
            loopController);
    }

    CommandVector ManeuverExecutor::QueueAdvanceRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        (void)state;

        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) ||
            (queueState.currentLocation == nullptr) ||
            (queueState.activeIndex >= queueState.queue->size()))
        {
            return FaultPhase("Queued maneuver advance state was invalid");
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
            return ReturnToContinuation(loopController);
        }

        return CompleteCurrentPhase(&queueState, &ManeuverExecutor::QueueDispatchRoutineTick, loopController);
    }
}
