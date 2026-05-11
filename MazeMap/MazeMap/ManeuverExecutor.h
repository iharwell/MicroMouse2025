#pragma once

#include "CommandVector.h"
#include "LoopController.h"
#include "ManeuverQueue.h"
#include "MazeMapRuntimeCore.h"

#include <cstdint>

namespace MazeMap
{
    struct TurnWallEdgeTracker;
}

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;

    class EXPORT ManeuverExecutor final
    {
    public:
        ManeuverExecutor() = default;

        bool Active() const noexcept;

        bool SEND_IT(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            bool snapToExpectedLocation,
            MazeMap::DirectionalLocation& currentLocation,
            LoopController::ModeWorkCallback continuationCallback,
            void* continuationContext,
            LoopController& loopController);

        void CancelActivePhase() noexcept;

    private:
        friend class SharedRobotRuntime;

        using ActivePhaseTickFn = CommandVector (ManeuverExecutor::*)(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        struct Continuation final
        {
            LoopController::ModeWorkCallback callback{};
            void* context{};
        };

        struct DelegatedDriveRoutineState final
        {
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
        };

        struct QueueRoutineState final
        {
            MazeMap::ManeuverQueue* queue{};
            MazeMap::DirectionalLocation* currentLocation{};
            MotionLimits limits{};
            std::uint16_t nextIndex{};
            std::uint16_t activeIndex{};
            bool snapToExpectedLocation{};
        };

        static CommandVector ActiveRoutineThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        [[noreturn]] void FailInvariant(const char* reason) const noexcept;
        bool CanBeginPhase() const noexcept;
        bool BeginPhase(
            void* activeState,
            ActivePhaseTickFn activePhaseTick,
            LoopController::ModeWorkCallback continuationCallback,
            void* continuationContext) noexcept;
        bool InstallRoutineCallback(LoopController& loopController) noexcept;
        void ActivatePhase(void* activeState, ActivePhaseTickFn activePhaseTick) noexcept;
        void ResetActiveRoutine() noexcept;

        CommandVector ReturnToContinuation(
            LoopController& loopController) noexcept;
        CommandVector CompleteCurrentPhase(
            void* nextState,
            ActivePhaseTickFn nextPhaseTick,
            LoopController& loopController) noexcept;
        CommandVector FaultPhase(
            const char* reason) noexcept;

        CommandVector DelegatedDriveRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        CommandVector QueueDispatchRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        CommandVector QueueAdvanceRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        SharedRobotRuntime* _runtime{};
        Drive* _driveService{};
        void* _activeState{};
        ActivePhaseTickFn _activePhaseTick{};
        Continuation _continuation{};
        DelegatedDriveRoutineState _delegatedDriveState{};
        QueueRoutineState _queueState{};
    };
}
