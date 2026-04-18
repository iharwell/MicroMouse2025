#pragma once

#include "CommandPD.h"
#include "LoopController.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeCore.h"

#include <cstddef>
#include <cstdint>

class DriveBase;

namespace MazeMap
{
    class Maze;
    struct TurnWallEdgeTracker;
    class Vehicle;
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    // Owns the ordinary shared multi-tick motion primitives that mode code may use while still
    // doing other work inside the same LoopController callback. The mode keeps callback ownership:
    // it arms one primitive through a Start... member, then calls GetNextControls(bool& done) from
    // its callback and decides whether to return Drive's proposed controls or do something else.
    //
    // This is intentionally a simpler, cadence-safe owner than ManeuverExecutor. Drive keeps the
    // primitive-specific state and control law private so most mode logic does not need to care
    // which primitive is currently active.
    class EXPORT Drive final
    {
    public:
        // Selects whether eligible primitives may use maze-wall observations for pose maintenance.
        //
        // Behavior:
        // `Maze` allows wall-grounded correction when the active primitive supports it.
        // `OpenFloor` disables wall-based correction and relies only on non-wall state sources.
        enum class OperationMode : std::uint8_t
        {
            Maze,
            OpenFloor
        };

        // Owner-level feedback-selection defaults reused across subsequent primitive starts.
        // This intentionally borrows DriveBase's CommandPD flag vocabulary instead of introducing
        // another control-settings dialect.
        //
        // `heading`:
        // Flags used when a primitive is correcting a heading target.
        //
        // `yawRate`:
        // Flags used when a primitive is commanding yaw-rate or alpha-driven yaw motion.
        //
        // `distance`:
        // Flags used when a primitive is interpreting translation-placement or distance-style
        // correction terms.
        //
        // `velocity`:
        // Flags used for forward-speed target tracking.
        struct CommandPDSettings final
        {
            MazeMap::CommandPD heading{};  // Used when holding or correcting a heading target.
            MazeMap::CommandPD yawRate{};  // Used when commanding yaw-rate or alpha-driven turn motion.
            MazeMap::CommandPD distance{}; // Used when correcting distance or placement error.
            MazeMap::CommandPD velocity{}; // Used when tracking forward-speed targets.
        };

        Drive();

        // Owner-level configuration follows the "set up once, use forever" model.
        //
        // `SetOperationMode(mode)`:
        // Changes whether eligible primitives may use maze-wall correction.
        //
        // Parameters:
        // `mode`:
        // Operation policy consulted by active execution for primitives that support wall-grounded
        // correction.
        //
        // `GetOperationMode()`:
        // Returns the current wall-usage policy.
        void SetOperationMode(OperationMode mode) noexcept;
        OperationMode GetOperationMode() const noexcept;

        // `SetLimits(limits)`:
        // Installs the owner-level motion envelope.
        //
        // Parameters:
        // `limits`:
        // Owner-level speed, acceleration, deceleration, and angular bounds consulted until changed.
        void SetLimits(const MotionLimits& limits) noexcept;

        // Returns the current owner-level motion envelope.
        const MotionLimits& GetLimits() const noexcept;

        // `SetCommandPDSettings(settings)`:
        // Installs the owner-level feedback-selection flags.
        //
        // Parameters:
        // `settings`:
        // Per-objective CommandPD flag selections consulted until changed.
        void SetCommandPDSettings(const CommandPDSettings& settings) noexcept;

        // Returns the current owner-level feedback-selection defaults.
        const CommandPDSettings& GetCommandPDSettings() const noexcept;

        // Reports whether Drive currently owns an active primitive or maneuver that has not yet been
        // cancelled or completed.
        bool Active() const noexcept;

        // Drops the active primitive immediately and clears Drive's private execution state.
        // This is the explicit escape hatch when a mode decides the currently armed motion should
        // no longer contribute controls on future ticks.
        void Cancel() noexcept;

        // Hold measures stationary time, not raw wall-clock time with brake output forced. If a
        // caller only wants to return ControlVector::Brake for some interval, it should do that in
        // the mode directly instead of routing that trivial case through Drive.
        //
        // Parameters:
        // `durationMs`:
        // Required stationary-time budget before the primitive reports completion.
        //
        // `requireContinuous`:
        // `true` resets progress when motion resumes, so completion requires one uninterrupted
        // stationary span of `durationMs`.
        //
        // `false` accumulates stationary time across interruptions until the total reaches
        // `durationMs`.
        void StartHold(
            std::uint16_t durationMs,
            bool requireContinuous);

        // Arms linear motion. Signed velocity determines forward versus reverse travel.
        // Drive captures the current initiation point and then owns progress tracking, heading
        // hold, and completion behavior privately.
        //
        // Parameters:
        // `distanceM`:
        // Requested travel magnitude along the held heading line.
        //
        // `cruiseSpeed`:
        // Requested steady signed speed target while translation remains in progress.
        //
        // `exitSpeed`:
        // Requested terminal signed speed boundary near completion. Use `0.0f` when the motion
        // should finish by settling to a stop.
        //
        // `targetHeadingOverride`:
        // Optional heading target to hold during the motion. When null, Drive captures the current
        // heading at arm time.
        //
        // `targetPositionOverride`:
        // Optional projected position target used for completion logic instead of pure
        // encoder-distance progress from the initiation point.
        void StartStraight(
            float distanceM,
            float cruiseSpeed,
            float exitSpeed,
            const Eigen::Vector2f* targetHeadingOverride = nullptr,
            const Eigen::Vector2f* targetPositionOverride = nullptr);

        // Arms a pure in-place turn. Translation is not the objective; the primitive exists to
        // rotate the robot about its pose with the project yaw sign convention.
        //
        // Parameters:
        // `angleRad`:
        // Signed turn amount relative to the current yaw. Positive values follow the project
        // convention of clockwise-positive yaw.
        //
        // `wallEdgeTracker`:
        // Optional observer updated while the turn runs. This is observational support for callers
        // that care about wall-edge tracking during the turn; it does not change the public control
        // protocol.
        void StartTurn(
            float angleRad,
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr);

        // Starts the transition-region primitive used between straight and constant-curvature arc
        // regions.
        //
        // Parameters:
        // `distanceM`:
        // Length of the transition region to execute.
        //
        // `dCurvatureDs`:
        // Signed derivative of curvature with respect to traveled distance for that region.
        void StartTurnTransition(
            float distanceM,
            float dCurvatureDs);

        // Starts the constant-curvature region only. This API intentionally does not claim support
        // for caller-directed speed changes through the turn.
        //
        // Parameters:
        // `distanceM`:
        // Arc length to travel through the constant-curvature region.
        //
        // `curvature`:
        // Signed constant curvature for that region.
        void StartArc(
            float distanceM,
            float curvature);

        // Starts canonical maneuver execution when the caller wants the catalogued maneuver
        // vocabulary rather than the simpler Drive-region primitives. This is the higher-end path
        // when the caller wants Drive to preserve maneuver-native geometry and point-tracking
        // behavior instead of spelling the motion out as separate straight, transition, and arc
        // regions itself.
        //
        // Parameters:
        // `maneuver`:
        // Canonical maneuver instance to execute. For point-trackable smooth turns, this preserves
        // maneuver-native geometry and uses maneuver-point evaluation instead of reducing the motion
        // to ad hoc straight/transition/arc parameters.
        void StartManeuver(
            const MazeMap::ManeuverInstance& maneuver);

        // Generic per-tick query used by the active mode callback. The caller may return the
        // proposed controls to LoopController, override them, or ignore them. `done` reports normal
        // primitive completion; internal faults are routed through the shared runtime fault path
        // rather than through a separate public Drive fault protocol.
        //
        // Parameters:
        // `done`:
        // Set to `true` when the active primitive or maneuver completes normally on this tick.
        //
        // Return value:
        // Proposed control vector for the present tick. The mode may return it directly to
        // LoopController, replace it, or ignore it.
        LoopController::ControlVector GetNextControls(bool& done);

    private:
        friend class SharedRobotRuntime;

        // Internal decomposition only. The public model stays at Start... + GetNextControls(...).
        enum class ActivePrimitive : std::uint8_t
        {
            None,
            Hold,
            LinearMotion,
            Turn,
            TurnTransition,
            Arc,
            Maneuver
        };

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        bool CanStart() const noexcept;
        void ResetActivePrimitive() noexcept;
        void SetFault(const char* reason) noexcept;
        bool IsDriveMotionSettled(
            const DriveTelemetry& stationaryReferenceTelemetry,
            unsigned long stationaryReferenceMs,
            const DriveTelemetry& telemetry,
            const SensorSnapshot& snapshot,
            unsigned long nowMs) const;
        const LoopController::ModeState* TryGetLoopState() const noexcept;
        static float ManeuverSpeedLimit(
            MazeMap::ManeuverCode code,
            const MotionLimits& limits,
            const MazeMap::Vehicle& vehicle);

        // Primitive-specific stepping helpers behind the generic public GetNextControls(...).
        LoopController::ControlVector HoldControls(bool& done);
        LoopController::ControlVector LinearMotionControls(
            const LoopController::ModeState& state,
            bool& done);
        LoopController::ControlVector TurnControls(
            const LoopController::ModeState& state,
            bool& done);
        LoopController::ControlVector TurnTransitionControls(
            const LoopController::ModeState& state,
            bool& done);
        LoopController::ControlVector ArcControls(
            const LoopController::ModeState& state,
            bool& done);
        LoopController::ControlVector ManeuverControls(
            const LoopController::ModeState& state,
            bool& done);

        SharedRobotRuntime* _runtime{};     // Shared runtime owner used for fault routing and shared services.
        LoopController* _loopController{};  // LoopController queried internally for current tick state.
        DriveBase* _drive{};                // Concrete low-level drive command sink/helper.
        MazeMap::Vehicle* _speedVehicle{};  // Vehicle facts used for speed-limit derivation.
        MazeMap::Maze* _maze{};             // Maze facts used when maze-mode wall correction is enabled.
        MotionLimits _limits{};                              // Owner-level motion envelope.
        CommandPDSettings _commandPdSettings{};              // Owner-level CommandPD selections.
        OperationMode _operationMode{ OperationMode::Maze }; // Owner-level wall-correction policy.
        ActivePrimitive _activePrimitive{ ActivePrimitive::None }; // Currently armed primitive kind.
        // Private fault bookkeeping only. Drive does not export a public fault protocol; failures
        // are expected to route through the shared runtime fault path.
        const char* _faultReason{};               // Most recent internal fault description routed to runtime.
        bool _faulted{};                          // Sticky internal-fault indicator for the active run.

        // Opaque active-primitive storage. Drive.cpp privately overlays this with the active union
        // state and is responsible for asserting that the chosen union fits in this buffer.
        static constexpr std::size_t PrimitiveStorageWordCount = 10;
        alignas(16) std::uint32_t _primitiveStorageWords[PrimitiveStorageWordCount]{};
    };
}
