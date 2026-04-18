#pragma once

#include "InPlaceTurnProfile.h"
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
    class SharedRobotRuntime;

    class EXPORT ManeuverExecutor final
    {
    public:
        using SampleCallback = bool (*)(
            void* context,
            bool stationary,
            const LoopController::ModeState& state);

        using QueueEntryCallback = bool (*)(
            void* context,
            std::uint16_t index,
            const MazeMap::ManeuverInstance& entry,
            const MazeMap::DirectionalLocation& location);

        struct Hooks final
        {
            constexpr Hooks() noexcept
                : context(nullptr)
                , onSample(nullptr)
                , onQueueEntryBegin(nullptr)
                , onQueueEntryComplete(nullptr)
            {
            }

            void* context;
            SampleCallback onSample;
            QueueEntryCallback onQueueEntryBegin;
            QueueEntryCallback onQueueEntryComplete;
        };

        ManeuverExecutor() = default;

        bool Active() const noexcept;
        bool ActivePhaseFaulted() const noexcept;

        float ComputeManeuverSpeedLimit(
            MazeMap::ManeuverCode code,
            const MotionLimits& limits,
            const MazeMap::Vehicle& vehicle) const;

        float ComputeManeuverSpeedLimit(
            MazeMap::ManeuverCode code,
            const MotionLimits& limits) const;

        bool BeginHoldRoutine(
            std::uint16_t durationMs,
            bool stationary,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});
        bool PrepareHoldRoutineCallbacks(
            std::uint16_t durationMs,
            bool stationary,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        void ApplyAsymmetricQueueLimits(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            const MazeMap::Vehicle& vehicle,
            float initialEntrySpeed,
            float finalExitSpeed) const;

        void ApplyAsymmetricQueueLimits(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            float initialEntrySpeed,
            float finalExitSpeed) const;

        bool ProceedToManeuverExecutionRoutine(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            bool snapToExpectedLocation,
            MazeMap::DirectionalLocation& currentLocation,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});
        bool ProceedToManeuverExecutionRoutine(
            MazeMap::ManeuverQueue& queue,
            const MotionLimits& limits,
            bool snapToExpectedLocation,
            MazeMap::DirectionalLocation& currentLocation,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        bool BeginBrakedSettleRoutine(
            const char* timeoutMessage,
            std::uint16_t stationaryHoldMs,
            std::uint16_t timeoutMs,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});
        bool PrepareBrakedSettleRoutineCallbacks(
            const char* timeoutMessage,
            std::uint16_t stationaryHoldMs,
            std::uint16_t timeoutMs,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        bool BeginReverseStraightRoutine(
            float distanceM,
            const MotionLimits& limits,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);
        bool PrepareReverseStraightRoutineCallbacks(
            float distanceM,
            const MotionLimits& limits,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);

        bool BeginStraightRoutine(
            float distanceM,
            float entrySpeed,
            float cruiseSpeed,
            float exitSpeed,
            const MotionLimits& limits,
            bool useWallCentering,
            MazeMap::DirectionalLocation* currentLocation,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);
        bool PrepareStraightRoutineCallbacks(
            float distanceM,
            float entrySpeed,
            float cruiseSpeed,
            float exitSpeed,
            const MotionLimits& limits,
            bool useWallCentering,
            MazeMap::DirectionalLocation* currentLocation,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);

        bool BeginTurnRoutine(
            float angleRad,
            const MotionLimits& limits,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{},
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr);
        bool PrepareTurnRoutineCallbacks(
            float angleRad,
            const MotionLimits& limits,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{},
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr);

        bool BeginArcRoutine(
            float distanceM,
            float angleRad,
            float entrySpeed,
            float exitSpeed,
            float cruiseSpeed,
            const MotionLimits& limits,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});
        bool PrepareArcRoutineCallbacks(
            float distanceM,
            float angleRad,
            float entrySpeed,
            float exitSpeed,
            float cruiseSpeed,
            const MotionLimits& limits,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        bool BeginSmoothTurnRoutine(
            const MazeMap::ManeuverInstance& maneuver,
            float cruiseSpeed,
            const MotionLimits& limits,
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services,
            const Hooks& hooks = Hooks{});
        bool PrepareSmoothTurnRoutineCallbacks(
            const MazeMap::ManeuverInstance& maneuver,
            float cruiseSpeed,
            const MotionLimits& limits,
            LoopController::ModeCallbacks& initialCallbacks,
            const Hooks& hooks = Hooks{});

        LoopController::ControlVector DriveActivePhase(
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        void CancelActivePhase() noexcept;

    private:
        friend class SharedRobotRuntime;

        using ActivePhaseTickFn = LoopController::ControlVector (ManeuverExecutor::*)(
            void* rawState,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        bool BeginHoldPhase(
            std::uint16_t durationMs,
            bool stationary,
            const Hooks& hooks = Hooks{});

        bool BeginBrakedSettlePhase(
            const char* timeoutMessage,
            std::uint16_t stationaryHoldMs,
            std::uint16_t timeoutMs,
            const Hooks& hooks = Hooks{});

        bool BeginReverseStraightPhase(
            float distanceM,
            const MotionLimits& limits,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);

        bool BeginStraightPhase(
            float distanceM,
            float entrySpeed,
            float cruiseSpeed,
            float exitSpeed,
            const MotionLimits& limits,
            bool useWallCentering,
            MazeMap::DirectionalLocation* currentLocation,
            const Hooks& hooks = Hooks{},
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);

        bool BeginTurnPhase(
            float angleRad,
            const MotionLimits& limits,
            const Hooks& hooks = Hooks{},
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr);

        bool BeginArcPhase(
            float distanceM,
            float angleRad,
            float entrySpeed,
            float exitSpeed,
            float cruiseSpeed,
            const MotionLimits& limits,
            const Hooks& hooks = Hooks{});

        bool BeginSmoothTurnPhase(
            const MazeMap::ManeuverInstance& maneuver,
            float cruiseSpeed,
            const MotionLimits& limits,
            const Hooks& hooks = Hooks{});

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

        struct ReverseStraightRoutineState final
        {
            MotionLimits limits{};
            Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
            const Eigen::Vector2f* targetPositionOverride{};
            float distanceM{};
            float startDistanceM{};
            float commandedSpeedMps{};
            bool projectionFallbackLogged{};
            unsigned long timeoutMs{};
            EncoderProgressWatchdog translationWatchdog{};
            SettleRoutineState completionSettle{};
            HoldRoutineState fallbackHold{};
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
            float targetYawRad{};
            MazeMap::InPlaceTurnProfile turnProfile{};
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker{};
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
            ActivePhaseTickFn activePhaseTick,
            const Hooks& hooks) noexcept;
        bool BuildRoutineCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept;
        bool InstallRoutineCallbacks(
            const LoopController::ModeCallbacks& returnCallbacks,
            LoopController::TickServices& services);
        void ActivatePhase(void* activeState, ActivePhaseTickFn activePhaseTick) noexcept;
        void ResetActiveRoutine() noexcept;

        bool InvokeSampleHook(bool stationary, const LoopController::ModeState& state) const;
        bool InvokeQueueEntryBegin(
            std::uint16_t index,
            const MazeMap::ManeuverInstance& entry,
            const MazeMap::DirectionalLocation& location) const;
        bool InvokeQueueEntryComplete(
            std::uint16_t index,
            const MazeMap::ManeuverInstance& entry,
            const MazeMap::DirectionalLocation& location) const;

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
        LoopController::ControlVector ReverseStraightRoutineTick(
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
        MazeMap::Vehicle* _speedVehicle{};
        MazeMap::Maze* _maze{};
        void* _activeState{};
        ActivePhaseTickFn _activePhaseTick{};
        LoopController::ModeCallbacks _returnCallbacks{};
        Hooks _hooks{};
        HoldRoutineState _holdState{};
        SettleRoutineState _settleState{};
        ReverseStraightRoutineState _reverseStraightState{};
        StraightRoutineState _straightState{};
        TurnRoutineState _turnState{};
        ArcRoutineState _arcState{};
        SmoothTurnRoutineState _smoothTurnState{};
        QueueRoutineState _queueState{};
        bool _activePhaseFaulted{};
    };
}
