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
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        struct HoldRoutineState final
        {
            std::uint16_t durationMs{};
            unsigned long startMs{};
            bool stationary{ true };
            bool started{};
        };

        struct SettleRoutineState final
        {
            const char* timeoutMessage{};
            std::uint16_t stationaryHoldMs{ Config::kMotionSettleHoldMs };
            std::uint16_t timeoutMs{ Config::kMotionSettleTimeoutMs };
            unsigned long startMs{};
            unsigned long stationaryStartMs{};
            bool stationaryWindowActive{};
            bool started{};
            bool brakeCommand{ true };
            DriveTelemetry stationaryStartTelemetry{};
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
        };

        struct StraightRoutineState final
        {
            MotionLimits limits{};
            Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
            const Eigen::Vector2f* targetPositionOverride{};
            MazeMap::DirectionalLocation* currentLocation{};
            float distanceM{};
            float entrySpeed{};
            float exitSpeed{};
            float cruiseSpeed{};
            float startDistanceM{};
            float commandedSpeedMps{};
            bool useWallCentering{};
            bool diagonalHeading{};
            float previousCorridorErrorM{};
            float filteredCorridorErrorRateMps{};
            bool previousCorridorErrorValid{};
            SettleRoutineState completionSettle{};
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
        };

        struct TurnRoutineState final
        {
            SettleRoutineState completionSettle{};
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
        };

        struct ArcRoutineState final
        {
            MotionLimits limits{};
            float distanceM{};
            float angleRad{};
            float exitSpeed{};
            float cruiseSpeed{};
            float startDistanceM{};
            float startYawRad{};
            float curvature{};
            float commandedSpeedMps{};
            EncoderProgressWatchdog translationWatchdog{};
            SettleRoutineState completionSettle{};
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
        };

        struct SmoothTurnRoutineState final
        {
            MazeMap::ManeuverInstance maneuver{};
            MotionLimits limits{};
            float maneuverSpeedMps{};
            float totalDistanceM{};
            float startDistanceM{};
            void* nextState{};
            ActivePhaseTickFn nextPhaseTick{};
            EncoderProgressWatchdog translationWatchdog{};
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
            const LoopController::ModeState& state,
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

        bool IsDriveMotionSettled(
            const DriveTelemetry& stationaryReferenceTelemetry,
            unsigned long stationaryReferenceMs,
            const DriveTelemetry& telemetry,
            const SensorSnapshot& snapshot,
            unsigned long nowMs) const;
        static float ManeuverSpeedLimit(
            MazeMap::ManeuverCode code,
            const MotionLimits& limits,
            const MazeMap::Vehicle& vehicle);

        LoopController::ControlVector HoldRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector SettleRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector StraightRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector TurnRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector ArcRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector SmoothTurnRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector QueueDispatchRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        LoopController::ControlVector QueueAdvanceRoutineTick(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        SharedRobotRuntime* _runtime{};
        DriveBase* _drive{};
        Drive* _driveService{};
        MazeMap::Vehicle* _speedVehicle{};
        MazeMap::Maze* _maze{};
        void* _activeState{};
        ActivePhaseTickFn _activePhaseTick{};
        LoopController::ModeCallbacks _returnCallbacks{};
        HoldRoutineState _holdState{};
        SettleRoutineState _settleState{};
        StraightRoutineState _straightState{};
        TurnRoutineState _turnState{};
        ArcRoutineState _arcState{};
        SmoothTurnRoutineState _smoothTurnState{};
        QueueRoutineState _queueState{};
    };
}
