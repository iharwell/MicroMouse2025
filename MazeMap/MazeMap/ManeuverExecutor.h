#pragma once

#include "LoopController.h"
#include "ManeuverQueue.h"
#include "MazeMapRuntimeCore.h"

#include <cstdint>

class DriveBase;

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
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services);
        bool SEND_IT(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            bool snapToExpectedLocation,
            MazeMap::DirectionalLocation& currentLocation,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::ModeCallbacks& initialCallbacks);

        void CancelActivePhase() noexcept;

    private:
        friend class SharedRobotRuntime;

        using ActivePhaseTickFn = LoopController::ControlVector (ManeuverExecutor::*)(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

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

        static LoopController::ControlVector ActiveRoutineThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        bool CanBeginPhase() const noexcept;
        bool BeginPhase(
            void* activeState,
            ActivePhaseTickFn activePhaseTick) noexcept;
        bool BuildRoutineCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept;
        bool InstallRoutineCallbacks(
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services);
        void ActivatePhase(void* activeState, ActivePhaseTickFn activePhaseTick) noexcept;
        void ResetActiveRoutine() noexcept;

        LoopController::ControlVector ReturnToContinuation(
            LoopController::TickServices& services) noexcept;
        LoopController::ControlVector CompleteCurrentPhase(
            void* nextState,
            ActivePhaseTickFn nextPhaseTick,
            LoopController::TickServices& services) noexcept;
        LoopController::ControlVector FaultPhase(
            LoopController::TickServices& services,
            const char* reason) noexcept;

        LoopController::ControlVector DelegatedDriveRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector QueueDispatchRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector QueueAdvanceRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController::TickServices& services);

        SharedRobotRuntime* _runtime{};
        DriveBase* _drive{};
        Drive* _driveService{};
        void* _activeState{};
        ActivePhaseTickFn _activePhaseTick{};
        LoopController::ModeCallbacks _returnCallbacks{};
        DelegatedDriveRoutineState _delegatedDriveState{};
        QueueRoutineState _queueState{};
    };
}
