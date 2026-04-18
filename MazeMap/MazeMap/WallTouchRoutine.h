#pragma once

#include "CellCoordinates.h"
#include "Direction.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeInfrastructure.h"

class DriveBase;

namespace MazeMap::App::Internal
{
    class EXPORT WallTouchRoutine final
    {
    public:
        using SampleCallback = bool (*)(
            void* context,
            bool stationary,
            const LoopController::ModeState& state);

        using TraceLineCallback = void (*)(void* context, const char* line) noexcept;
        using NotificationCallback = void (*)(void* context) noexcept;

        struct Hooks final
        {
            constexpr Hooks() noexcept
                : context(nullptr)
                , onSample(nullptr)
                , onTraceLine(nullptr)
                , onPoseReset(nullptr)
            {
            }

            void* context;
            SampleCallback onSample;
            TraceLineCallback onTraceLine;
            NotificationCallback onPoseReset;
        };

        explicit WallTouchRoutine(DriveBase& drive) noexcept;

        bool Active() const noexcept;
        bool ActivePhaseFaulted() const noexcept;
        const Runtime::WallTouchExecutionResult& LastResult() const noexcept;

        bool Begin(
            const MazeMap::CellCoordinates& wallCell,
            MazeMap::Direction wallDirection,
            bool allowPassThroughNoWall,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});

        bool PrepareInitialCallbacks(
            const MazeMap::CellCoordinates& wallCell,
            MazeMap::Direction wallDirection,
            bool allowPassThroughNoWall,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        void CancelActiveRoutine() noexcept;

    private:
        using ActivePhaseTickFn = LoopController::ControlVector (WallTouchRoutine::*)(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        struct SettleRoutineState final
        {
            const char* timeoutMessage{};
            std::uint16_t stationaryHoldMs{ Config::kMotionSettleHoldMs };
            std::uint16_t timeoutMs{ Config::kMotionSettleTimeoutMs };
            unsigned long startMs{};
            unsigned long stationaryStartMs{};
            bool stationaryWindowActive{};
            bool started{};
            DriveTelemetry stationaryStartTelemetry{};
        };

        static LoopController::ControlVector ActiveRoutineThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        bool PrepareWallTouchPhase(
            const MazeMap::CellCoordinates& wallCell,
            MazeMap::Direction wallDirection,
            bool allowPassThroughNoWall,
            const Hooks& hooks) noexcept;
        bool CaptureLaunchBaseline(
            Runtime::WallTouchLoopState& wallTouch,
            const LoopController::ModeState& state) noexcept;

        bool CanBeginPhase() const noexcept;
        bool BuildInitialCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept;
        bool BeginPassThroughSettlePhase(
            const char* timeoutMessage,
            std::uint16_t stationaryHoldMs,
            std::uint16_t timeoutMs) noexcept;
        void ActivatePhase(void* activeState, ActivePhaseTickFn activePhaseTick) noexcept;
        void ResetActiveRoutine() noexcept;
        bool InvokeSampleHook(bool stationary, const LoopController::ModeState& state) const;
        void PersistResult() noexcept;
        bool IsDriveMotionSettled(
            const DriveTelemetry& stationaryReferenceTelemetry,
            unsigned long stationaryReferenceMs,
            const DriveTelemetry& telemetry,
            const SensorSnapshot& snapshot,
            unsigned long nowMs) const noexcept;
        LoopController::ControlVector ReturnToContinuation(LoopController::TickServices& services) noexcept;
        LoopController::ControlVector CompleteCurrentPhase(
            void* nextState,
            ActivePhaseTickFn nextPhaseTick,
            LoopController::TickServices& services) noexcept;
        LoopController::ControlVector FaultPhase(
            LoopController::TickServices& services,
            const char* reason) noexcept;

        static void AppendTraceLineHook(void* context, const char* line) noexcept;
        static void PoseResetHook(void* context) noexcept;
        static LoopController::ControlVector BeginPassThroughSettleHook(
            void* context,
            void* rawState,
            LoopController::TickServices& services);
        static LoopController::ControlVector FaultHook(
            void* context,
            LoopController::TickServices& services,
            const char* reason) noexcept;
        static LoopController::ControlVector CompleteHook(
            void* context,
            LoopController::TickServices& services);

        LoopController::ControlVector WallTouchRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector SettleRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        DriveBase* _drive{};
        void* _activeState{};
        ActivePhaseTickFn _activePhaseTick{};
        bool _activePhaseFaulted{};
        Runtime::WallTouchExecutionResult _lastResult{};
        LoopController::ModeCallbacks _returnCallbacks{};
        Hooks _hooks{};
        Runtime::WallTouchLoopState _wallTouchState{};
        SettleRoutineState _settleState{};
    };
}
