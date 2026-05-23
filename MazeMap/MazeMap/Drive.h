#pragma once

#include "CommandVector.h"
#include "ManeuverInstance.h"
#include "MotionLimits.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"

#include <cstddef>
#include <cstdint>


namespace MazeMap
{
    class Maze;
    struct TurnWallEdgeTracker;
    class Vehicle;
    class DriveBase;
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    // Stateful, total command-proposal engine for ordinary shared multi-tick motion
    // primitives that mode code may consult while still doing other work inside the
    // same LoopController callback.
    //
    // Layering contract:
    // DriveBase is the low-level drive mechanism. Drive owns the higher-level primitive
    // semantics used by normal mode code: primitive progression state, retained live
    // execution configuration, command interpretation, and degraded-input behavior.
    //
    // Mode code keeps callback ownership:
    // - A mode arms one primitive or maneuver through a Start... member.
    // - On later ticks, the mode may call GetNextControls(bool& done) when it wants
    //   Drive's current proposal.
    // - The mode may return Drive's proposal, override it, ignore it, or stop querying
    //   Drive without transferring lifecycle ownership to Drive. Once the caller applies
    //   the returned CommandVector, that command is active for the state interval being
    //   calculated.
    //
    // Public state model:
    // - Set... members install retained live Drive configuration. These settings are
    //   normally established during boot-time setup and reused across primitives, but
    //   mode code may change them while running. Active primitives observe changed
    //   settings on later GetNextControls(...) calls when applicable.
    // - Each Start... member supersedes the previous instruction and latches a new
    //   latest instruction. Within Drive, that latest instruction is authoritative
    //   until another Start... call supersedes it. Numeric Start arguments have no
    //   validity precondition:
    //   NaN, infinity, out-of-range, physically impossible, or otherwise ill-posed
    //   numeric requests do not cause Drive to reject the Start call, retain an older
    //   instruction, fault, or become incoherent.
    // - GetNextControls(...) combines the latest latched instruction, the current live
    //   Drive configuration, and the current runtime state to produce one advisory
    //   control proposal.
    //
    // Robustness and semantic-recovery model:
    // Public numeric inputs are requests, not scalar validity preconditions. Drive does
    // not merely discard unexpected numeric values. When a value is not directly usable,
    // Drive may derive meaning from the field's domain, the active primitive, sibling
    // parameters, retained live configuration, captured start state, current robot
    // state, vehicle facts, maze facts, project sign conventions, and any other
    // available local context.
    //
    // Extended numeric values may have domain meaning. For example, an infinite limit
    // may be interpreted as an unbounded constraint when the field semantics support
    // that reading. A negative value in a field normally used as an upper acceleration
    // or speed bound may, when that is the most coherent interpretation, be treated as
    // a signed lower bound or reverse-direction constraint rather than as an immediate
    // error.
    //
    // Only when Drive cannot derive a coherent meaning for a supplied value does it
    // neutralize or ignore the affected term, and then only locally. When multiple
    // coherent interpretations remain and no retained caller signal or local context
    // distinguishes them, Drive still chooses one coherent interpretation instead of
    // treating ambiguity as failure. Degraded evaluation is still evaluation of the
    // latest installed instruction; Drive must not substitute an unrelated primitive
    // or route command generation through a separate fault mode.
    //
    // Malformed numeric input is non-sticky. It will not, under any circumstances
    // within normal C++ object validity, poison Drive's state to the point of loss of
    // usability. Drive remains able to accept later Start... calls, accept later Set...
    // calls, produce coherent control proposals, and return to normal behavior once
    // coherent inputs are supplied.
    //
    // Drive is not a vehicle permission authority. Its responsibility is to produce commands
    // aligned with the best available interpretation of the caller's latest request.
    // Permission logic, stop authority, and decision-level permission to move
    // belong outside this class. Drive respects finite and physically plausible
    // configured constraints because they are part of the interpreted request, not
    // because Drive owns a generic permission boundary.
    //
    // This contract assumes normal C++ validity: intact object storage, intact code,
    // valid non-null caller-owned pointers according to their documented lifetime
    // rules, and no external memory corruption. Ill-posed numeric requests are inside
    // Drive's robustness contract; C++ undefined behavior is not.
    class EXPORT Drive final
    {
    public:
        // Selects the Drive-level live wall-correction mode used by eligible primitives.
        //
        // Behavior:
        // `Maze` allows wall-grounded correction when the active primitive supports it.
        // `OpenFloor` disables wall-based correction and relies only on non-wall state sources.
        enum class OperationMode : std::uint8_t
        {
            Maze,
            OpenFloor
        };

        Drive();
        explicit Drive(float nominalCommandPeriodSeconds);

        // Drive-level configuration is retained and live.
        //
        // These settings are typically installed during boot-time setup and reused across
        // primitives. They may also be changed by mode code while the robot is running;
        // active primitives observe the new values on subsequent GetNextControls(...) calls
        // when the primitive consults that setting.
        //
        // `SetOperationMode(mode)`:
        // Installs the live wall-correction mode used by primitives that support
        // wall-grounded correction.
        //
        // Parameters:
        // `mode`:
        // Wall-correction mode consulted by active execution for eligible primitives.
        //
        // `GetOperationMode()`:
        // Returns the current wall-correction mode.
        void SetOperationMode(OperationMode mode) noexcept;
        OperationMode GetOperationMode() const noexcept;

        // `SetLimits(limits)`:
        // Installs the Drive-level live motion envelope.
        //
        // Limits are retained on Drive rather than passed to individual Start... calls.
        // A mode may change them while a primitive is active; subsequent
        // GetNextControls(...) calls use the current limits wherever that primitive applies
        // motion limiting.
        //
        // Parameters:
        // `limits`:
        // Speed, acceleration, deceleration, and angular bounds consulted until changed.
        // Drive stores the supplied MotionLimits object verbatim. It does not clamp,
        // sanitize, or rewrite the retained configuration at SetLimits(...) time. Limit
        // fields are interpreted defensively at the point of use.
        //
        // Finite and physically plausible limit fields are binding in the direction Drive
        // coherently interprets them and must be respected whenever the corresponding
        // constrained quantity is commanded. Non-finite, contradictory, signed in an
        // unexpected direction, or physically implausible fields are not automatically
        // discarded: Drive first attempts to interpret them according to the field's
        // semantics and the current execution context.
        //
        // Examples of possible interpretations include treating an infinite upper bound as
        // an unbounded constraint, or treating a negative acceleration-like upper bound as a
        // signed lower bound/reverse-direction constraint when that best matches the
        // caller's apparent request. If no coherent meaning can be derived for a limit
        // field, Drive treats that field as unavailable for that particular limiting
        // operation. That does not authorize violation of other usable limits, cancel the
        // active primitive, revive an older instruction, or corrupt Drive's retained state.
        //
        // Malformed or uninterpretable limit fields are non-sticky: they may reduce the
        // constrainedness or quality of proposals while configured, but they will not, under
        // any circumstances within normal C++ object validity, poison Drive's state to the
        // point of loss of usability. Later coherent limits must restore coherent limit
        // behavior without requiring Drive to be reconstructed.
        //
        // These limits express the caller's requested motion envelope as interpreted by
        // Drive. They are not a vehicle permission boundary and do not make Drive responsible
        // for deciding whether the vehicle should be allowed to move.
        void SetLimits(const MotionLimits& limits) noexcept;

        // Returns the current Drive-level live motion envelope exactly as retained by Drive.
        const MotionLimits& GetLimits() const noexcept;

        float GetNominalCommandPeriodSeconds() const noexcept;

        // Convenience completion query.
        //
        // This reports whether Drive is currently acting as though the latest observed
        // `GetNextControls(...)` completion result was `done == true`. Before any command is
        // installed, Drive behaves as though a hold had already completed. Arming a new
        // instruction clears this cached observation until Drive evaluates that instruction.
        bool IsEffectivelyComplete() const noexcept;

        // Start... members are total instruction setters.
        //
        // Each Start... call replaces the previous installed instruction with the newly
        // requested primitive or maneuver. Numeric arguments are accepted as the caller's
        // latest request even when they are NaN, infinite, out of range, or physically
        // unattainable. Such values may require semantic recovery, but they do not cause
        // the Start call to fail, leave the previous instruction active, install a
        // different primitive, or make Drive unusable.
        //
        // Drive may resolve that request into retained internal execution state during the
        // Start call itself. Later GetNextControls(...) calls should then execute and track
        // that retained instruction coherently rather than re-deciding what the caller
        // meant on every tick. Only irrecoverably meaningless terms are neutralized or
        // ignored.
        //
        // Unless explicitly documented as a retained observer pointer, Start arguments are
        // sampled during the call and latched into Drive's active primitive state. Numeric
        // robustness does not relax pointer validity requirements: non-null pointer
        // arguments must satisfy the lifetime and validity rules documented for that
        // parameter.

        // Hold measures stationary time, not raw wall-clock time with brake output forced. If a
        // caller only wants to return CommandVector::Brake() for some interval, it should do that in
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
        // Drive captures the current initiation point and target references, then owns progress
        // tracking and completion behavior while delegating closed-loop heading/speed tracking to
        // DriveBase's canonical command helpers.
        //
        // Parameters:
        // `distanceM`:
        // Requested travel amount along the held heading line.
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
        // heading at arm time. When non-null, the pointed-to numeric values are sampled during this
        // Start call and resolved into the retained instruction under Drive's recovery rules.
        //
        // `targetPositionOverride`:
        // Optional projected position target used for completion logic instead of pure
        // encoder-distance progress from the initiation point. When non-null, the pointed-to
        // numeric values are sampled during this Start call and resolved into the retained
        // instruction under Drive's recovery rules.
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
        // Optional non-owning observer pointer latched into the turn instruction. When non-null,
        // it may be updated while the turn runs and must remain valid until the turn is
        // superseded or no longer queried. This observational support does not change the public
        // control protocol.
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
        // to ad hoc straight/transition/arc parameters. The maneuver request is latched as the
        // latest instruction.
        void StartManeuver(
            const MazeMap::ManeuverInstance& maneuver);

        // Generic per-tick query used by the active mode callback. The caller may return the
        // proposed controls to LoopController, override them, ignore them, or stop querying Drive
        // altogether without transferring lifecycle ownership to the service.
        //
        // Each call evaluates:
        //   latest latched instruction
        //   + current live Drive configuration
        //   + current runtime state
        // to produce the best coherent CommandVector Drive can infer for the state interval
        // the caller is about to calculate.
        //
        // Unexpected or non-finite inputs are handled by semantic recovery rather than by a
        // separate fault path. That recovery should converge onto one retained instruction rather
        // than re-deciding the instruction on every tick. Later calls then combine that retained
        // instruction with current live configuration and runtime state to keep the returned
        // command coherent.
        //
        // The returned proposal must remain aligned with the latest installed instruction as Drive
        // best understands it. Drive may degrade toward neutral/brake behavior when no meaningful
        // progress command can be computed, but it must not revive an older instruction, reset
        // itself to None, or substitute an unrelated primitive merely because the latest request is
        // ill-posed.
        //
        // The returned proposal must respect every applicable finite and physically plausible limit
        // that Drive can coherently interpret. Uninterpretable limit fields remove only their own
        // constraint; they do not release other usable constraints or permit unrelated motion
        // objectives. Drive's responsibility here is request alignment, not global move permission.
        //
        // Parameters:
        // `done`:
        // Set to `true` when the installed primitive or maneuver has reached its completion
        // condition on this tick. When the retained instruction has no finite completion
        // condition, `done` is also reported as `true` so callers that assume commands normally
        // complete can still behave reasonably while Drive continues proposing motion for the
        // retained instruction. This observation is advisory and should not be treated as a
        // validity signal, fault signal, or implicit permission for Drive to revoke
        // or forget the installed instruction.
        //
        // Return value:
        // Proposed active command for the state interval the caller is about to calculate. The
        // mode may return it directly to LoopController, replace it, or ignore it. Drive does not
        // inject an independent fault or independent fallback behavior into this command path.
        CommandVector GetNextControls(bool& done);

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

        void ResetActivePrimitive() noexcept;

        // Primitive-specific stepping helpers behind the generic public GetNextControls(...).
        CommandVector HoldControls(
            const SensorSnapshot& sensors,
            bool& done);
        CommandVector LinearMotionControls(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            bool& done);
        CommandVector TurnControls(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            bool& done);
        CommandVector TurnTransitionControls(
            const MazeMap::VehicleState& state,
            bool& done);
        CommandVector ArcControls(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            bool& done);
        CommandVector ManeuverControls(
            const MazeMap::VehicleState& state,
            bool& done);

        SharedRobotRuntime* _runtime{};     // Canonical runtime owner for live state and services.
        MazeMap::DriveBase* _drive{};                // Concrete low-level drive command proposal sink.
        MazeMap::Vehicle* _vehicle{};       // Canonical vehicle facts used for limit derivation.
        MazeMap::Maze* _maze{};             // Maze facts used when maze-mode wall correction is enabled.
        MotionLimits _limits{};             // Drive-level live motion envelope, interpreted at use.
        float _nominalCommandPeriodSeconds{}; // Next-tick command-shaping horizon for MotionLimits.
        OperationMode _operationMode{ OperationMode::Maze }; // Drive-level live wall-correction mode.
        ActivePrimitive _activePrimitive{ ActivePrimitive::None }; // Latest installed instruction kind.
        bool _effectivelyComplete{ true };  // Latest completion observation, defaulting to the no-command hold-like state.

        // Opaque active-primitive storage. Drive.cpp privately overlays this with the active union
        // state and is responsible for asserting that the chosen union fits in this buffer.
        static constexpr std::size_t PrimitiveStorageWordCount = 16;
        alignas(16) std::uint32_t _primitiveStorageWords[PrimitiveStorageWordCount]{};
    };
}
