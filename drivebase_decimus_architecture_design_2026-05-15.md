# DriveBase Decimus-Class Architecture Design

Date: 2026-05-15

## Purpose

This document defines the proposed replacement architecture for `DriveBase`.

`DriveBase` is currently too broad for the role it should serve. It mixes physical command solving, output-space feedback trim, encoder/IMU cache interpretation, command telemetry, reset hooks, and overlapping command names. The replacement design must make `DriveBase` suitable for a high-performance micromouse that is expected to operate near destructive physical limits.

Decimus 5A is the documented floor to beat, not the target to merely match. Even with its published control detail, characterization, suction/downforce work, and aggressive performance envelope, it still lost. This architecture must therefore treat Decimus-style practice as the minimum admissible seriousness and then leave room for more aggressive, better-instrumented competition behavior. It is not a generic robotics drivetrain abstraction. The drive architecture must assume:

- feedforward-dominant control,
- high acceleration and braking near traction limits,
- fan/downforce-dependent authority,
- saturation and lost authority as normal competitive events,
- empirical tuning from flight-recorder logs,
- clean ownership boundaries that do not hide physics behind compatibility wrappers.

No implementation is included in this document.

## Source Basis

The design is based on:

- Current repository ownership rules in `AGENTS.md`.
- Current `DriveBase` role and caller surface in `MazeMap/MazeMap/DriveBase.h` and `MazeMap/MazeMap/DriveBase.cpp`.
- `Drive.h` as the reference for a clean public protocol and private internal decomposition.
- The OpenFloor measurement regime subsystem as a reference for private role-specific vocabulary and observable physical metadata.
- Decimus and Micromouse Online material:
  - https://micromouseonline.com/2017/04/06/apec-2017-31st-annual-micromouse-contest/
  - https://micromouseusa.com/wp-content/uploads/2018/03/APEC18P.pdf
  - https://micromouseonline.com/2008/04/04/decimus-speed-feed-forward/
  - https://micromouseonline.com/2008/03/23/decimus-doing-circuits/
  - https://micromouseonline.com/2017/08/03/micromouse-hard-acceleration/
  - https://micromouseonline.com/2018/02/18/more-suck-less-slip/
  - https://micromouseonline.com/2013/03/04/speed-affects-micromouse-smooth-turn/
  - https://micromouseonline.com/2011/12/31/rocking-out-of-control/
  - https://micromouseonline.com/2017/06/12/rotation-centre-for-your-robot/

No public Decimus 5A source repository was found. The design uses the published behavior, control approach, and tuning lessons as the minimum bar that a record-seeking system must exceed.

## Design Summary

`DriveBase` must be the single-tick competitive drive effort solver for a robot intended to exceed the Decimus 5A performance and tuning bar.

It owns this contract:

> Given one physically meaningful tick request, current authoritative runtime state, and current authoritative fan/downforce reference, propose left/right normalized actuator commands and record solver-local evidence for that exact proposal.

It must not become:

- a motion planner,
- a maneuver executor,
- a mode framework,
- a safety-limit owner,
- a fan policy owner,
- a sensor capture owner,
- an odometry or estimator owner,
- a hardware lifecycle owner,
- a compatibility facade around the old API.

The cross-review binding decisions are:

- `DriveBase` has one production proposal method, not a family of convenience commands.
- The proposal method does not accept caller-supplied present state.
- `CommandVector` and DriveBase telemetry are normalized actuator command proposals, not hardware PWM.
- Physical feedback correction happens before `PlantModel` inverse dynamics.
- Feedback is acceleration-domain arbitration: velocity, yaw-rate, and heading errors produce acceleration corrections, not mutated velocity/yaw-rate targets.
- A PlantModel-owned combined body-action inverse-dynamics contract is prerequisite work if it does not already exist.
- DriveBase telemetry is mandatory command evidence with join keys and validity flags, not a sketch.
- Old command APIs and old telemetry semantics must be deleted, not wrapped.

## Canonical Ownership Boundaries

### Vehicle

`Vehicle` remains the owner of robot construction facts, physical dimensions, hardware capability facts, motor banks, and fixed device facts.

`DriveBase` consumes facts only through existing canonical owners. It must not copy vehicle facts into public parameter bags or become a second owner of drive geometry.

### PlantModel

`PlantModel` remains the single source of truth for drive plant equations, physical-to-actuator inverse dynamics, wheel target derivation, and plant-owned feasibility evidence.

`DriveBase` must ask `PlantModel` for normalized actuator command proposals for a composed physical request. It must not duplicate plant equations, compose plant fragments ad hoc, or construct a parallel motor model.

Implementation prerequisite:

`PlantModel` must own a combined body-action inverse-dynamics path before the DriveBase redesign is implemented. That contract must make requested motion objectives, measured state, authority references, and composed acceleration commands distinct. DriveBase must not treat requested velocity/yaw-rate scalars as substitutes for present measured state.

Minimum PlantModel-owned inverse solve vocabulary:

- requested forward velocity scalar,
- requested yaw-rate scalar,
- composed forward acceleration scalar,
- composed yaw acceleration scalar,
- left/right command result,
- left/right wheel target result,
- plant evaluation id or deterministic plant solve fingerprint when available,
- plant model revision/fingerprint when available,
- PlantModel-owned feasibility evidence if the model solves a maximize request or detects a plant constraint.

These are vocabulary requirements, not a mandate to introduce a public request wrapper. The motion objective scalars already carry intent through IEEE values, and wrapping them just to regain the same meaning is rejected unless implementation proves the scalar contract is insufficient.

The concrete PlantModel contract must return one solve result for each DriveBase proposal. That result must include the command, wheel targets when PlantModel computes them, the plant evaluation id/fingerprint when available, scalar-intent/constraint status for maximize objectives, and failure flags. DriveBase may copy only the command-evidence fields it owns into `DriveTelemetry`; detailed feasibility evidence stays with PlantModel or a PlantModel-owned log keyed by the evaluation id.

The velocity and yaw-rate scalars are included only when they serve an intentional profile/control objective. They are not a redundant delivery stream for present measured velocity or yaw rate; present state remains available through `VehicleState`/PlantModel ownership. If a velocity or yaw-rate objective is inactive, the caller uses the project-wide scalar objective convention instead of substituting measured state.

`PlantModel` owns the exact use of requested velocity/yaw-rate objectives and present measured state for back-EMF, drag, friction, wheel target generation, ground-strike effects, scrub-mode behavior, fan-seal/downforce modeling, and traction feasibility. The DriveBase-to-PlantModel solve is one same-tick transaction: DriveBase reads the authoritative `VehicleState` snapshot once, uses that same snapshot for feedback observations, and passes that same snapshot or a const reference to it into the PlantModel solve. PlantModel must not independently re-read live runtime state during that solve. If PlantModel cannot declare these semantics, the DriveBase redesign is blocked. If traction, current, voltage, or downforce feasibility evidence is needed for high-speed falsification, it must be returned as PlantModel-owned result evidence or emitted in a PlantModel-owned log keyed by the plant evaluation id; DriveBase must not compute or label that evidence.

Fan/downforce state is slow-changing and centrally available. PlantModel should consume it through the canonical runtime/state owner; if that owner is awkward, moving the reference into `VehicleState` is preferred over adding fan arguments to the DriveBase-to-PlantModel API. Battery voltage is not currently measured; if it is measured later, it belongs in `VehicleState` or the authoritative runtime state path. DriveBase must not add battery voltage to its PlantModel call contract.

DriveBase must not create `correctedForwardMps` or `correctedYawRateRadps` by adding feedback to PlantModel velocity inputs. Feedback changes acceleration-domain requests and leaves velocity/yaw-rate objective semantics intact.

### Drive and Maneuver

`Drive`, `Maneuver`, and related motion owners remain responsible for multi-tick primitive progression, motion profile decisions, requested motion envelopes, and higher-level command sequencing.

`DriveBase` sees only the current tick request.

### MotionLimits

`MotionLimits` remains the owner of requested motion-command envelopes used by motion code.

`DriveBase` does not own max speed, acceleration, deceleration, jerk, or generic safety bounds. It may report that a requested tick command exceeded actuator authority after solving the request, but it does not choose the request envelope.

### LoopController and VehicleState

`LoopController` owns control cadence and hardware application timing.

`VehicleState` and sensor/runtime owners remain authoritative for sensor snapshots, estimator state, encoder observations, IMU observations, and time.

`DriveBase` reads authoritative state to solve and record the current proposal. It must not maintain a duplicate encoder/IMU state cache or shadow estimator.

### SharedRobotRuntime

`SharedRobotRuntime` remains the production owner of the single `DriveBase` instance.

No second production `DriveBase`, drive manager, drive facade, or hidden duplicate command solver is allowed.

### MotorEncoderDrive

`MotorEncoderDrive` owns motor-bank electrical facts, encoder geometry, normalized-command-to-hardware-PWM mapping, and final motor-bank actuation details.

If actuator-space residual trim is needed for motor-bank calibration, the preferred long-term owner is `MotorEncoderDrive` or data owned by `Vehicle` and applied through `MotorEncoderDrive`, not `DriveBase`.

## Shared Scalar Objective Convention

Motion command scalars use IEEE float values as shared project vocabulary. This convention is deliberately chosen because it carries intent without extra representation cost in a memory-constrained system.

For motion objective fields:

| Scalar value | Meaning |
| --- | --- |
| finite | specific numeric objective |
| quiet NaN | objective intentionally inactive/unconstrained for this tick |
| `+inf` | maximize the positive-direction objective under the owning solver's model |
| `-inf` | maximize the negative-direction objective under the owning solver's model |

This is not numeric recovery. These values are command language. `targetForwardMps = NaN` and `targetForwardAccelMps2 = +inf` means the caller does not care about a specific forward velocity objective and is asking for the maximum positive deliverable forward acceleration under the plant model.

The convention is shared by `Drive`, `DriveBase`, `PlantModel`, boot modes, tests, and host replay. `MotionLimits` does not need to interpret this convention, but code paths that accept shared motion requests must not reject these values merely because they are non-finite. Finite-profile calculations may still require finite values when they are explicitly doing finite profile math.

The table above is definitive for motion objective fields. If an owner in the DriveBase path cannot accept an inactive or maximize objective for a field, that owner work is a prerequisite blocker. It must not reinterpret the scalar as malformed input.

Responsibility split:

- `Drive` and boot modes choose the intent values.
- `DriveBase` decodes active feedback objectives and preserves maximize/unconstrained intent.
- `PlantModel` resolves maximize acceleration objectives into deliverable normalized commands under the current plant model.
- Host tools and logs preserve IEEE values rather than collapsing them into invalid numeric input.

## Public API Shape

The public API must use physical tick language. It must not expose implementation names like `PointCommand`, `DeltaCommand`, `ControlVector`, or generic derivative-order feedback commands.

Exact production shape:

```cpp
class DriveBase final
{
public:
    explicit DriveBase(
        const MazeMap::PlantModel& plant,
        const MazeMap::VehicleState& runtimeState,
        const MazeMap::DriveBaseFeedbackTuning& feedbackTuning);

    DriveBase(const DriveBase&) = delete;
    DriveBase& operator=(const DriveBase&) = delete;
    DriveBase(DriveBase&&) = delete;
    DriveBase& operator=(DriveBase&&) = delete;

    void ClearCommandEvidence() noexcept;

    MazeMap::App::Internal::CommandVector ProposeBodyTick(
        float targetForwardMps,
        float targetYawRateRadps,
        float targetForwardAccelMps2,
        float targetYawAccelRadps2,
        float targetYawRad);

    MazeMap::App::Internal::CommandVector ProposeNeutralTick() noexcept;

    const DriveTelemetry& LastTelemetry() const noexcept;
};
```

The public API is intentionally not a convenience surface. All ordinary locomotion goes through `ProposeBodyTick(...)`.

Public concepts:

- one committed body tick proposal,
- neutral tick proposal,
- command evidence clear,
- last proposal telemetry.

Argument semantics:

- `targetForwardMps`: requested body forward velocity objective. Positive follows the project convention of `+Y = forward/up`. IEEE special values follow the shared scalar objective convention.
- `targetYawRateRadps`: requested body yaw-rate objective. Positive follows the project convention of `+Yaw = clockwise`. IEEE special values follow the shared scalar objective convention.
- `targetForwardAccelMps2`: requested body forward acceleration objective. This is not a DriveBase-owned profile or response horizon. `+inf` and `-inf` request a PlantModel maximize solve in the corresponding direction.
- `targetYawAccelRadps2`: requested body yaw acceleration objective. `+inf` and `-inf` request a PlantModel maximize solve in the corresponding yaw direction.
- `targetYawRad`: requested heading objective. A quiet NaN makes heading inactive. Finite values are interpreted using the project clockwise-positive yaw convention. Infinities are invalid unless a later approved shared convention assigns them meaning.

`DriveBase` reads authoritative `VehicleState` exactly once during each proposal for feedback observations and any present-state values needed by the PlantModel solve. The same state snapshot is supplied to PlantModel for that proposal. Public callers must not pass present speed, yaw rate, wheel velocity, encoder velocity, or other measured-state substitutes through `DriveBase`.

Feedback source and blending strategy are DriveBase configuration/tuning, not per-tick command data. A proposal call carries physical intent only.

One-call-per-tick semantics:

`ProposeBodyTick(...)` and `ProposeNeutralTick()` are committed proposal calls. Production mode flow must issue at most one DriveBase proposal for each applied `LoopController` control tick. Unit tests may call multiple proposals outside a loop tick only when explicitly testing solver behavior; the latest proposal always owns `LastTelemetry()`. The telemetry sequence key must make repeated proposals observable.

## Deleted Public Surface

The replacement must remove these compiled public APIs rather than keep aliases or wrappers:

- `PointCommand(...)`
- `PointControlVector(...)`
- `PointCommandWithHeadingTarget(...)`
- `PointControlVectorWithHeadingTarget(...)`
- `DeltaCommand(...)`
- `DeltaYawRateCommand(...)`
- `PointYawRateCommand(...)`
- `GetFeedbackCommand(...)`
- `Begin()`
- `Brake()`
- `ResetControllers()`
- `CurrentControlVector()`
- `GetAverageDistanceMeters()`
- `GetLastLinearCommandMps()`
- `GetLastAngularCommandRadps()`
- `GetLastFeedforward()`
- `GetLastFeedback()`
- public derivative-order feedback composition
- public command variants that only rearrange arguments
- public maneuver-point forwarding overloads in `DriveBase`

Maneuver-point interpretation belongs to `Drive` or maneuver execution owners. `DriveBase` must receive a physical tick request, not a maneuver-domain object.

## Control Law

The default control law must be acceleration-domain correction before inverse dynamics.

DriveBase receives two first-class objective scalars per body axis:

- the velocity/yaw-rate objective for the current tick,
- the acceleration objective for the current tick.

Those facts are not competing commands. A finite velocity or yaw-rate objective participates in feedback. An inactive velocity or yaw-rate objective does not. A finite acceleration objective is a requested derivative. An infinite acceleration objective is a maximize request that `PlantModel` resolves under the current plant model. Feedback must resolve tracking error by modifying the acceleration-domain request, not by creating a second velocity command.

Flow:

1. Start with the caller's physical request.
2. Observe physical error from authoritative state through the configured feedback strategy.
3. Convert feedback into acceleration-domain correction:
   - forward velocity error becomes forward-acceleration correction,
   - yaw-rate error becomes yaw-acceleration correction,
   - heading error becomes yaw-acceleration correction.
4. Compose one physical inverse-dynamics request:
   - forward velocity objective,
   - yaw-rate objective,
   - composed or maximize forward acceleration objective,
   - composed or maximize yaw acceleration objective.
5. Ask `PlantModel` for the normalized actuator command proposal for that composed request.
6. Apply final normalized-command envelope clamping at the actuator proposal boundary.
7. Record solver-local telemetry for that exact proposal.

The old model of `feedforward PWM + feedback PWM` is rejected as the normal locomotion path.

At Decimus-class speeds, actuator-space addition assumes a locally linear and separable plant. That is not a sound default near saturation, fan-assisted traction, braking pitch, speed-dependent turns, or left/right authority imbalance. `PlantModel` exists so the composed acceleration request is resolved through the authoritative plant equations.

DriveBase does not own acceleration policy. `targetForwardAccelMps2` and `targetYawAccelRadps2` are caller-supplied objectives from `Drive`, `Maneuver`, or a characterization mode. If a caller wants a maximum-acceleration behavior, it says so with `+inf` or `-inf`. If a caller wants no velocity objective, it says so with NaN. DriveBase must not infer a response horizon, speed ramp, acceleration limit, or hidden profile.

### Acceleration-Domain Arbitration

For the forward axis:

```text
forwardVelocityErrorMps =
    targetForwardMps - observedForwardMps

forwardFeedbackAccelMps2 =
    kForwardVelocityToAccel * forwardVelocityErrorMps

composedForwardAccelMps2 =
    targetForwardAccelMps2 + forwardFeedbackAccelMps2
```

If `targetForwardMps` is NaN, forward velocity feedback is inactive and the forward velocity error term is not evaluated. If `targetForwardAccelMps2` is infinite, the forward feedback term is treated as a bias on the maximize solve only if the approved PlantModel contract supports that composition; otherwise finite feedback is suppressed for that axis and telemetry records the branch decision. DriveBase must not replace an infinite acceleration objective with a hard-coded finite maximum.

For the rotational axis:

```text
yawRateErrorRadps =
    targetYawRateRadps - observedYawRateRadps

headingErrorRad =
    clockwise_wrap(targetYawRad - observedYawRad)

yawFeedbackAccelRadps2 =
    kYawRateToAccel * yawRateErrorRadps
    + kHeadingToAccel * headingErrorRad

composedYawAccelRadps2 =
    targetYawAccelRadps2 + yawFeedbackAccelRadps2
```

If `targetYawRateRadps` is NaN, yaw-rate feedback is inactive. If `targetYawRad` is NaN, heading feedback is inactive. Heading-only yaw acceleration is not rejected by design; it must instead be validated against the plant's physical damping and scrub behavior. If validation shows overshoot, chatter, or insufficient damping, add an explicit current-yaw-rate damping term using authoritative same-tick yaw rate. Do not revive sampled derivative history to add damping.

The result is one acceleration command per axis. A braking profile with a positive velocity error may legitimately reduce braking or request positive acceleration. An accelerating profile with a negative velocity error may legitimately reduce acceleration or request braking. That is not feedback fighting feedforward; that is the controller spending acceleration authority to recover the trajectory state as fast as the chosen gains and actuator authority allow.

Acceleration measurement feedback is not part of the phase-1 DriveBase public contract. If measured-acceleration feedback is later approved, it must be designed as a bandwidth-limited disturbance or inner acceleration residual around `composedForwardAccelMps2` and `composedYawAccelRadps2`, not as a peer trajectory objective and not as actuator-space trim.

## Actuator-Space Residual Trim

Actuator-space residual trim is not part of the DriveBase public API for this redesign.

Allowed meanings:

- known motor-bank deadband compensation,
- left/right motor or H-bridge asymmetry,
- PWM nonlinearity,
- direction-specific actuator bias,
- temporary characterization perturbation,
- motor-side residual that is not yet represented in `PlantModel`.

Forbidden meanings:

- body velocity feedback,
- yaw feedback,
- heading correction,
- trajectory tracking error,
- motion-envelope enforcement,
- generic safety derating.

Preferred ownership:

- Long-term calibrated actuator residuals belong in `MotorEncoderDrive` or `Vehicle`-owned motor-bank calibration applied through `MotorEncoderDrive`.
- Phase 1 DriveBase telemetry records actuator residual trim as zero and marks it valid only if the authoritative motor-bank owner later exposes an applied residual value.
- A future characterization-only actuator-space path requires a separate approved design. It must not be added opportunistically to DriveBase during this migration.

Do not call actuator-space residual trim "feedback." If it exists in telemetry, name it `actuatorResidualTrim`.

## Feedback Model

Feedback source and blending strategy are DriveBase-owned configuration/tuning. They are not a per-call command input. The result of feedback must be acceleration-domain physical correction.

DriveBase may use estimator state, direct encoder-derived observations, gyro/IMU-derived observations, wall/sensor-derived interaction where approved, source fallback, or weighted combinations when testing shows that improves competition behavior. This is a tunable control dimension, not a headline public API feature. DriveBase still must not own raw sensor interpretation, encoder geometry derivation, gyro bias handling, wall geometry, sample history, or estimator state. Every observation used for feedback must come from an authoritative same-tick owner or snapshot.

DriveBase feedback is stateless. For each proposal, DriveBase computes correction only from:

- the consumed target request,
- the current authoritative measured value,
- a current authoritative same-tick measured rate or derivative only for an approved correction that explicitly needs it,
- configured physical-correction gains.

DriveBase must not:

- retain previous observed signal samples,
- retain previous observed timestamps,
- retain previous target values,
- retain previous error values,
- synthesize derivatives from prior DriveBase samples,
- finite-difference encoder, gyro, yaw, velocity, or acceleration samples,
- call sampled-derivative APIs,
- own resettable feedback controller state,
- include or instantiate `FeedbackAxis`.

The default stateless correction law is an acceleration output from current error:

```text
feedbackAccel = kp_to_accel * (target - measured)
```

For corrections where a target rate and measured rate are both defined, the derivative-style term is still an acceleration output:

```text
feedbackAccel = kp_to_accel * (target - measured)
              + kd_to_accel * (target_rate - measured_rate)
```

For the phase-1 DriveBase law, forward velocity and yaw-rate errors already compare rate-like state variables to their profile targets, so their correction output is acceleration. Heading error compares angle to angle and produces yaw-acceleration correction. If a selected correction later uses a current authoritative measured-rate signal, a missing measured-rate signal makes only that derivative contribution zero and records a branch/validity flag when host replay cannot reconstruct the condition. DriveBase must not fall back to sampled derivative history.

Legal production feedback corrections:

| Correction | Strategy inputs | Legal authoritative observations | Error equation | Correction output | Composed request field |
| --- | --- | --- | --- | --- | --- |
| Forward velocity | configured source, fallback, or weighted blend | state forward velocity, encoder-average body forward velocity, or approved direct body-speed observation | `targetForwardMps - observedForwardMps` when target is finite | `forwardFeedbackAccelMps2` | `composedForwardAccelMps2` |
| Yaw rate | configured source, fallback, or weighted blend | state yaw rate, gyro yaw rate, encoder-delta yaw rate, or approved direct yaw-rate observation | `targetYawRateRadps - observedYawRateRadps` when target is finite | `yawRateFeedbackAccelRadps2` | `composedYawAccelRadps2` |
| Heading | configured source, fallback, or weighted blend | state yaw angle or approved direct heading observation | clockwise-positive wrapped angle error from `targetYawRad` to observed yaw when target is finite | `headingFeedbackAccelRadps2` | `composedYawAccelRadps2` |

Measured-acceleration feedback is excluded from the public DriveBase contract until an authoritative measured acceleration source, latency model, and physical correction tuning are defined with units. Acceleration-domain correction produced from velocity, yaw-rate, and heading error is required and is not the excluded feature. The current migration must not preserve output-space acceleration feedback as a renamed physical correction.

Invalid feedback observation behavior:

- Unsupported or unavailable configured observations are handled by the configured fallback/blend strategy.
- If no configured authoritative observation is valid for an active objective, the affected correction is set to zero and the branch decision is recorded in telemetry only if host replay cannot reconstruct it from source validity logs.
- Other legal corrections remain active.

`FeedbackAxis` as currently shaped is too generic to be the public DriveBase language. During migration, either:

- fold the needed feedback math privately into `DriveBase` with physical names, or
- replace it with a canonical physical-correction owner only if that owner becomes true shared project vocabulary.

Do not preserve `GetFeedbackCommand(...)` as a public DriveBase API. Do not keep `FeedbackAxis` as a hidden DriveBase compatibility helper. If some other subsystem truly needs sampled-derivative signal processing, it must be redesigned under that subsystem's owner with explicit units and reset semantics.

Feedback tuning ownership:

Existing `PDCluster` gain values must not be reused blindly. The current gains were tuned for output-space behavior in places, while this design requires physical correction units. `ProportionalDerivative` should be retained as the underlying stateless math object if it is repurposed with explicit units and no sampled derivative history in the DriveBase path. Each retained DriveBase tuning member must document:

- input error unit,
- measured-rate unit,
- physical correction output unit,
- valid feedback observation/strategy path,
- valid command field.

Production DriveBase attaches to the authoritative tuning at construction. Runtime mutation through public DriveBase setters is forbidden. Tests needing alternate gains must construct a DriveBase with explicit tuning, not mutate the runtime-owned singleton. DriveBase must read only stateless proportional and derivative gains; it must not call `ComputeFromErrorSample(...)`, `ResetDerivativeHistory()`, `ResetDerivativeHistories()`, or any API that mutates derivative history.

The constructor type is `DriveBaseFeedbackTuning`. Implementation may build that owner by renaming/repurposing the existing `PDCluster` only if the resulting public type exposes unit-named physical correction gains and no sampled-derivative/reset semantics. `ProportionalDerivative` remains acceptable as the underlying value/math type.

## Fan and Traction Regime

Fan/downforce matters because authority changes with downforce, fan spin-up, surface state, ground strike, fan-seal behavior, scrub, and wheel loading.

`DriveBase` must not own fan policy or traction-regime policy. Fan/downforce is centrally available and changes slowly relative to the drive proposal. `PlantModel` should consume the authoritative fan/downforce reference through the canonical runtime/state path; storing the commanded fan duty cycle on `Vehicle` behind accessors is acceptable if it is treated as a robot fact/commanded hardware reference rather than mode policy.

The first implementation may consume the existing canonical mission fan duty path only if that path is explicitly named in the implementation plan and remains owned outside DriveBase. DriveBase may record the consumed regime id only when that id is part of the solver transaction and cannot be recovered from the joined PlantModel/runtime logs. DriveBase must not call broad fan-policy helpers except through the approved authoritative source.

Battery voltage is not measured today. Do not add it to the DriveBase-to-PlantModel API. If it is measured later, it belongs in `VehicleState` or the authoritative runtime state path and is consumed by `PlantModel` through that owner.

`DriveBase` must not own:

- fan control,
- fan spin-up policy,
- traction envelopes,
- generic safety limits,
- cooldown rules,
- run suppression,
- solver failure handling,
- motion rejection policy.

The DriveBase telemetry responsibility is: "This tick was solved using this authoritative regime reference, and the final proposal had this much actuator authority deficit."

It is not: "DriveBase decided the robot is safe or unsafe to move."

Actuator authority deficit is not traction loss. Saturation proves lost normalized actuator authority, not skid. Slip, lift, touchdown, and surface-state diagnosis remain host-analysis or mode-diagnostic concerns.

## Telemetry Contract

DriveBase telemetry must contain only solver-local evidence that cannot be safely reconstructed from raw sensor logs or host-side analysis.

The hostile standard is:

> If the field is raw physical state, sensor history, estimator state, offline diagnosis, or host-derivable arithmetic, do not put it in DriveBase telemetry. If the field records a non-derivable DriveBase branch decision, the exact scalar objective sent to PlantModel, or the exact command returned by PlantModel/DriveBase, keep it.

### Keep In DriveBase Telemetry

Required command evidence:

- `proposalSequenceId`,
- plant evaluation id or result fingerprint when available,
- `feedbackTuningRevisionId`,
- feedback strategy revision or id,
- original consumed body request, preserving IEEE special values,
- composed body inverse-dynamics request sent to `PlantModel`,
- left and right PlantModel command for the composed request,
- left and right actuator residual trim only if supplied by an approved MotorEncoderDrive/Vehicle-owned calibration path,
- left and right final command,
- non-derivable feedback strategy branch/fallback/blend decisions,
- scalar-intent, validity, and solver failure flags,
- stale/no-fresh-proposal flags.

Timing, loop tick identity, state sample identity, and run/static metadata are not DriveBase responsibilities. A mode log such as OpenFloor should use its existing `control_tick_sequence`, timing rows, sidecar metadata, and runtime/state logs to join against DriveBase evidence rather than forcing DriveBase to expose timing fields.

### Do Not Keep In DriveBase Telemetry

These belong to `VehicleState`, sensor telemetry, mode logs, or host analysis:

- control-loop sequence numbers,
- loop timing and timestamps,
- runtime state sample ids,
- plant model revisions or fingerprints when they are run/static metadata,
- encoder counts,
- wheel distances,
- raw wheel speeds,
- gyro yaw rate,
- estimator forward velocity,
- estimator yaw rate,
- raw IMU acceleration,
- fan measured RPM or fan hardware telemetry,
- slip suspicion labels,
- lift suspicion labels,
- touchdown suspicion labels,
- vague confidence scores,
- generic capability or safety status.

### On Unclamped PWM

Do not log `unclamped PWM`. DriveBase does not own hardware PWM. Do not log `preLimitCommand` as a separate field if final command composition is:

```text
PlantModel command + actuatorResidualTrim
```

and those fields are already logged.

The host can reconstruct the pre-limit command arithmetically. If future command composition becomes non-additive, introduce one explicitly named pre-limit command field then. Do not log redundant arithmetic fields by default.

### Telemetry Truth Requirement

Every public proposal method must atomically update telemetry for that exact proposal before returning. `LastTelemetry()` must never expose a command vector from one proposal with request/correction/authority evidence from another.

Old telemetry or mmlog field names must not be reused for changed semantics. Do not keep old and new parallel names for the same value. The concrete migration results for this codebase are:

| Current/proposed concept | Existing field or schema name | Required result |
| --- | --- | --- |
| DriveBase proposal sequence | none; not `control_tick_sequence` | Introduce `proposalSequenceId` only as DriveBase command evidence. Do not use it for loop timing. |
| PlantModel solve identity | none; not OpenFloor `revisions` | Introduce `plantEvaluationId` only if PlantModel exposes a per-solve id/fingerprint. Static plant revisions remain run metadata. |
| Feedback tuning identity | none | Introduce `feedbackTuningRevisionId` only if the tuning owner exposes it. Do not encode it in `modeFlags`. |
| Feedback strategy identity | none | Introduce `feedbackStrategyRevisionId` only if the strategy owner exposes it. |
| Requested forward velocity objective | `DriveTelemetry::commandedLinearSpeedMps`, OpenFloor `cmd_linear_mps` | Replace in DriveBase telemetry with `requestedForwardMps`. Existing mode logs may keep `cmd_linear_mps` only if it remains a mode/Drive command field. If the field becomes DriveBase evidence, rename it to `requested_forward_mps`. |
| Requested yaw-rate objective | `DriveTelemetry::commandedAngularSpeedRadps`, OpenFloor `cmd_angular_radps` | Replace in DriveBase telemetry with `requestedYawRateRadps`. If logged as DriveBase evidence, use `requested_yaw_rate_radps`. |
| Requested forward acceleration objective | no DriveTelemetry field; old API arguments such as `desiredLongitudinalAccelMps2` | Introduce `requestedForwardAccelMps2` as genuinely new DriveBase evidence. Prefer `Forward` to old mixed `linear`/`longitudinal` naming. |
| Requested yaw acceleration objective | no DriveTelemetry field; old API arguments such as `desiredYawAccelRadps2` | Introduce `requestedYawAccelRadps2` as genuinely new DriveBase evidence. |
| Requested heading objective | no DriveTelemetry field; old API argument `targetYawRad` | Introduce `requestedYawRad` as genuinely new DriveBase evidence. |
| Post-feedback forward acceleration sent to PlantModel | none | Use `composedForwardAccelMps2`, not `commandedForwardAccelMps2`. It is the composed PlantModel request, not a generic command field. |
| Post-feedback yaw acceleration sent to PlantModel | none | Use `composedYawAccelRadps2`, not `commandedYawAccelRadps2`. |
| PlantModel command | `leftFeedforwardCommand`, `rightFeedforwardCommand`; mmlog `left_feedforward_command`, `right_feedforward_command` | Replace old feedforward fields. Use `leftPlantCommand` / `rightPlantCommand` in C++ and `left_plant_command` / `right_plant_command` in mmlog if logged. The old `feedforward` names would now lie. |
| Actuator residual trim | tempting old `leftFeedbackCommand`, `rightFeedbackCommand` | Do not introduce in phase 1. If later supplied by MotorEncoderDrive/Vehicle, use `leftActuatorTrimCommand` / `rightActuatorTrimCommand`. Never reuse `feedback`. |
| Final proposed command | `leftDriveCommand`, `rightDriveCommand`; mmlog `left_drive_command`, `right_drive_command`, `left_drive_cmd`, `right_drive_cmd` | Prefer keeping `leftDriveCommand` / `rightDriveCommand` in C++ if they remain final DriveBase command proposal. If explicit proposal wording is required, use `leftProposedCommand` / `rightProposedCommand`. Existing mmlog schemas may keep old drive-command field names only if they continue to mean final command. Do not log both names in one row. |
| Scalar intent evidence | none | Introduce `scalarIntentFlags` as genuinely new. It is not a replacement for `modeFlags`. |
| Non-derivable feedback branch/blend decision | none; not `leftFeedbackCommand`/`rightFeedbackCommand` | Introduce `feedbackBranchFlags` only for non-derivable branch decisions. Do not use it for arithmetic feedback values. |
| DriveBase evidence validity | superficial overlap with `encoderObservationValid` | Introduce `telemetryValidFlags`. Delete `encoderObservationValid` from DriveTelemetry; sensor validity stays in `SensorSnapshot`. |
| Solver failure evidence | none | Introduce `solverFailureFlags` as genuinely new. |
| Stale/no-fresh-proposal state | old `modeFlags`/`kModeBraking` | Encode through `telemetryValidFlags` or a small DriveBase command-kind flag if needed. Do not preserve `modeFlags` as DriveBase mode state. |
| Wheel target velocities | `leftTargetVelocityMps`, `rightTargetVelocityMps`; mmlog `left_target_velocity_mps`, `right_target_velocity_mps` | Remove from DriveBase. If a mode needs wheel target logging, source it from PlantModel and keep the existing mmlog names only if the meaning is unchanged. Otherwise rename once to `left_wheel_target_velocity_mps` / `right_wheel_target_velocity_mps`. |
| Encoder counts/distances/velocities/omega | DriveTelemetry encoder fields; OpenFloor encoder fields | Delete from DriveTelemetry. Source from `SensorSnapshot`, `VehicleState`, or mode-owned anchors. If a mode switches from DriveBase-relative values to raw totals, rename counts to `left_encoder_total_count` / `right_encoder_total_count`, or keep a mode-owned anchor and name deltas explicitly. |
| Mode flags | `DriveTelemetry::modeFlags`, mmlog `mode_flags` | Delete from DriveTelemetry. Existing mmlog `mode_flags` may remain only as mode-owned phase/command status. |
| Saturation flags | `DriveTelemetry::saturationFlags`, mmlog `saturation_flags` | Delete from DriveTelemetry. DriveBase saturation/authority deficit is derivable from PlantModel command, optional residual trim, and final proposed command. Existing `saturation_flags` should be removed unless a non-DriveBase owner still defines it. |

Do not log a field merely because it is useful to see in a spreadsheet when host replay can derive it exactly.

## Mandatory DriveTelemetry Shape

This is the mandatory command-evidence schema unless implementation discovers a blocker and updates this design before coding:

```cpp
struct DriveTelemetry
{
    std::uint32_t proposalSequenceId;
    std::uint32_t plantEvaluationId;
    std::uint32_t feedbackTuningRevisionId;
    std::uint32_t feedbackStrategyRevisionId;

    float requestedForwardMps;
    float requestedYawRateRadps;
    float requestedForwardAccelMps2;
    float requestedYawAccelRadps2;
    float requestedYawRad;

    float composedForwardAccelMps2;
    float composedYawAccelRadps2;

    float leftPlantCommand;
    float rightPlantCommand;
    float leftDriveCommand;
    float rightDriveCommand;

    std::uint16_t commandKindFlags;
    std::uint16_t scalarIntentFlags;
    std::uint16_t feedbackBranchFlags;
    std::uint16_t telemetryValidFlags;
    std::uint16_t solverFailureFlags;
};
```

IEEE special values are valid command-language values and must be preserved in request fields. Zero is also a valid physical value and must not be used as an implicit "not applicable" sentinel. Field applicability is defined only by `scalarIntentFlags` and `telemetryValidFlags`. If an id is unavailable because the authoritative owner does not expose it yet, the field is set to zero and the corresponding valid flag is clear.

DriveTelemetry is allowed as a public value type only as the stable command-evidence record shared by DriveBase consumers and log population code. Do not add a second public request, result, authority, traction, or telemetry-context family.

`leftDriveCommand` and `rightDriveCommand` keep the existing final-command meaning in DriveBase telemetry. In schemas such as OpenFloor that already have `left_drive_command` and `right_drive_command`, keep the old fields with the same final-command meaning. Do not add `left_proposed_command` or any `normalized` spelling beside them. The same rule applies to wheel target fields, mode flags, and command velocity fields that already exist in a mode schema.

Required flag semantics:

- `commandKindFlags`: at minimum distinguishes fresh body proposal, fresh neutral proposal, stale/no-fresh-proposal evidence, and solver-failure neutral proposal.
- `scalarIntentFlags`: records inactive/maximize/finite interpretation for each motion objective field.
- `telemetryValidFlags`: records whether ids and command evidence fields are valid.
- `solverFailureFlags`: records PlantModel non-finite result, unsupported scalar intent for a field, or other solver-local failure that forced neutral output.
- `feedbackBranchFlags`: records only non-derivable feedback strategy branches.

Mode/runtime logs that need to join command evidence to timing/state data must copy `proposalSequenceId` into their own rows beside `control_tick_sequence`, runtime sample ids, or timing fields. DriveBase does not own those timing/state fields.

## Authority Formula

DriveBase reports command-space actuator authority only.

Definitions, per side:

```text
preLimitCommand = plantCommand + actuatorTrimCommand
finalCommand = clamp(preLimitCommand, -1.0, 1.0)
authorityDeficit = preLimitCommand - finalCommand
```

For phase 1, actuator trim is zero and not logged in the mandatory telemetry shape. If an approved MotorEncoderDrive/Vehicle-owned trim is later applied after PlantModel, telemetry must include the applied trim or a join key to it so the host can reconstruct the pre-limit command.

Do not log saturation flags or authority deficit fields when the host can derive them from logged PlantModel command, residual trim when present, and final proposed command. Positive derived deficit means the pre-limit command exceeded positive authority; negative derived deficit means it exceeded negative authority.

DriveBase must not compute traction loss, skid, lift, or safety status from authority deficit.

## Scalar Intent Decoding

`DriveBase` must decode motion scalar intent before numeric solving. Non-finite values are not automatically malformed; they are valid command language where this design assigns meaning.

Intent rules:

| Field | NaN | +inf / -inf | DriveBase behavior | PlantModel input |
| --- | --- | --- | --- | --- |
| `targetForwardMps` | forward velocity objective inactive | maximize positive/negative forward velocity objective | no forward velocity-error feedback when inactive | preserve scalar intent |
| `targetYawRateRadps` | yaw-rate objective inactive | maximize positive/negative yaw-rate objective | no yaw-rate feedback when inactive | preserve scalar intent |
| `targetForwardAccelMps2` | forward acceleration objective inactive | maximize positive/negative forward acceleration under PlantModel | preserve maximize intent; do not replace with finite max | preserve scalar intent |
| `targetYawAccelRadps2` | yaw acceleration objective inactive | maximize clockwise/counterclockwise yaw acceleration under PlantModel | preserve maximize intent; do not replace with finite max | preserve scalar intent |
| `targetYawRad` | heading objective inactive | maximize positive/negative heading objective is reserved by the shared convention and currently unsupported by DriveBase until an owner assigns useful semantics | no heading feedback when inactive | preserve scalar intent or reject only because no owner supports this field's maximize meaning |
| unavailable configured feedback observation | affected correction disabled or alternate configured blend/fallback branch used | same | record branch only if not host-derivable | composed acceleration excludes unavailable correction unless strategy supplies alternate |
| non-finite PlantModel output | final proposal becomes neutral command | final proposal becomes neutral command | solver failure flag set | neutral command telemetry with plant-output failure flag |

Invalid objective values never revive an older request and never report as full-authority success. Valid IEEE intent values do not set recovery flags because there is nothing to recover.

## Clear And Neutral

`ClearCommandEvidence()` exact postcondition:

- clears telemetry to "no fresh proposal",
- increments no proposal sequence,
- does not reset feedback state because DriveBase owns none,
- sets stale/no-fresh-proposal valid flags,
- sets last proposed command to neutral,
- does not call `LoopController`,
- does not apply hardware,
- does not imply brake or coast on the physical robot.

`ProposeNeutralTick()` exact behavior:

- returns `CommandVector(0.0f, 0.0f)`,
- records fresh telemetry with command kind `Neutral`,
- records requested and composed physical terms as zero/inactive according to valid flags,
- records plant command and final command as zero,
- does not update feedback derivative history because DriveBase owns none,
- does not call `LoopController`,
- does not apply hardware.

This follows the robust shape already documented in `Drive.h`, while keeping the effect local to a single tick.

## Saturation and Authority

Silent clamping is rejected.

When a proposed command exceeds the command envelope:

- clamp final command,
- preserve enough solver evidence for the host to derive side-specific saturation and authority deficit,
- preserve the composed physical request and PlantModel command in telemetry,
- do not report the proposal as fully achieved.

`DriveBase` does not decide whether the mode should recover, stop, or continue. Command authority loss is derived from PlantModel command, optional actuator trim, and final command. Embedded consumers that need an immediate authority-loss decision must derive it from those fields or from a deliberately introduced narrow command-validity/clamped bit; do not revive old `saturationFlags`.

## Private Internals

Private internal helpers are allowed when they keep the public API small and the implementation readable. The spec does not pre-approve helper names. New private terms should be introduced only when they represent genuinely new implementation concepts, not cleaner aliases for existing project vocabulary.

Private helpers must remain private nested types or file-local helpers unless one becomes stable shared project vocabulary. Do not publish a public family of DriveBase companion structs merely to organize implementation details.

Any non-template class or file-local block over 100 lines must follow the repository rules: same-named authoritative header and implementation, with the appropriate export macro where needed.

The names above are examples, not permission to grow a private mini-framework. If any private helper becomes large enough to trigger repository file-local size rules, implementation must stop and re-evaluate ownership rather than automatically publishing a new DriveBase helper subsystem.

## Interaction With Drive

`Drive` must migrate first.

Current `Drive` translates primitives into acceleration deltas and calls `DriveBase::DeltaCommand(...)`. After the redesign, `Drive` should produce the current tick's physical request and call the canonical DriveBase proposal method.

`Drive` remains responsible for:

- retained primitive state,
- maneuver execution,
- motion profile progression,
- motion limits,
- wall correction policy,
- completion,
- caller-facing total primitive semantics.

`DriveBase` remains responsible for:

- single-tick physical correction,
- PlantModel request,
- actuator proposal,
- command evidence telemetry.

## Interaction With Measurement Modes

Measurement modes must use `Drive` when they are asking for ordinary motion behavior.

They may call `DriveBase` directly only when the purpose is drive characterization or actuator/plant measurement for a single tick request.

OpenFloor and similar logs must join DriveBase command evidence with runtime/sensor logs during host analysis. DriveBase must not duplicate raw sensor state merely to make one log row self-contained.

## Rejected Designs

### Generic Drivetrain Abstraction

Rejected because it does not meet, much less exceed, the Decimus-class floor. A generic "body velocity in, wheel command out" abstraction hides authority, fan regime, saturation, and tuning evidence.

### Feedforward PWM Plus Feedback PWM

Rejected as the normal control law. Feedback must correct physical request inputs before `PlantModel` inverse dynamics.

### Public Request/Result Helper Family

Rejected unless the types become genuine shared project vocabulary. Public helper families would recreate DriveBase sprawl under cleaner names. Do not wrap IEEE motion objective scalars merely to express finite/inactive/maximize intent; the scalar value already carries that meaning.

### Compatibility Wrappers

Rejected. The old API must not remain compiled beside the new API.

### DriveBase-Owned Sensor Cache

Rejected. Sensor capture, encoder totals, IMU observations, and estimator state belong to runtime sensing and `VehicleState`.

### DriveBase-Owned Limits Or Safety Policy

Rejected. Requested motion envelopes belong to `MotionLimits` and motion owners. Physical facts belong to `Vehicle`. Plant equations belong to `PlantModel`. DriveBase reports authority; it does not own permission to move.

### Slip/Lift/Touchdown Labels In DriveBase Telemetry

Rejected. These are physical diagnoses requiring sensor history, state history, and often external evidence. They belong in host analysis or mode-specific diagnostics, not in DriveBase solver telemetry.

## Migration Plan

The migration must use copy-delete-stitch.

1. Copy current `DriveBase.h` and `DriveBase.cpp` to a temporary non-compiled reference outside `MazeMap/MazeMap`, outside any Teensy/sketch input directory, and outside project compile metadata. Prefer a `.md` or `.txt` reference under a non-build documentation path.
2. Delete the old compiled command API immediately.
3. Complete prerequisite owner work: PlantModel combined body-action inverse dynamics, feedback strategy/tuning ownership, and command-evidence telemetry schema.
4. Move progress/distance users off DriveBase telemetry before deleting encoder/distance fields.
5. Migrate `Drive`.
6. Migrate direct mode/controller callers.
7. Redesign `DriveTelemetry` and affected mmlog schemas intentionally.
8. Rewrite tests around physical invariants and command evidence in the same compiled cut.
9. Delete the temporary non-compiled reference before completion.

Do not stage the new API beside old wrappers.

### Caller Migration Requirements

Before implementation, enumerate every compiled `DriveBase`, `DriveTelemetry`, and old command API consumer with `rg`. The required migration set includes at least:

- `Drive.cpp`,
- `WallTouch`,
- `DiagnosticController`,
- `AuxMeasurementController`,
- `OpenFloorMeasurementController`,
- `ShowcasingDonutController`,
- `FrontWallCharacterizationController`,
- `TopSpeedMeasurementMode`,
- `MissionRunMode`,
- `ManeuverFileTestMode`,
- `CorridorRepeatabilityMode`,
- `PositionAccuracyAuditMode`,
- `SharedRobotRuntime`,
- `DriveManeuverTests`,
- `SharedRuntimeTest`,
- `DriveBaseTest`,
- `EstimatorTestSupport`.

`WallTouch` and `Drive` must be migrated away from DriveBase telemetry as a distance/speed source before deleting DriveTelemetry encoder distance/velocity fields. Those uses must move to `VehicleState`, `SensorSnapshot`, `LoopController` last-applied command, or the owning routine's explicit distance anchor.

Old method replacement mapping:

- `Begin()`: setup owner or `ClearCommandEvidence()`, depending on whether the call wanted hardware setup or command evidence clear.
- `Brake()`: hardware brake sentinel/application remains outside DriveBase. `ProposeNeutralTick()` means zero command proposal with fresh telemetry, not brake. A call site that truly needs brake must call the hardware/application owner path; a call site that only needs no drive effort uses `ProposeNeutralTick()`.
- `ResetControllers()`: deleted. DriveBase owns no resettable feedback controller state.
- `CurrentControlVector()`: `LoopController` last-applied command or latest returned proposal owned by the caller.
- `GetAverageDistanceMeters()`: `VehicleState`, `SensorSnapshot`, or routine-owned distance anchor.
- `GetLastLinearCommandMps()` / `GetLastAngularCommandRadps()`: DriveBase command evidence telemetry if the caller needs the latest proposal; otherwise Drive state.
- `GetLastFeedforward()` / `GetLastFeedback()`: deleted. Use PlantModel command evidence and host-derived feedback reconstruction.
- `GetTelemetry()`: `LastTelemetry()` for command evidence only; sensor fields must come from runtime state/sensor logs.
- `SetProportionalDerivativeCluster()` / `GetProportionalDerivativeCluster()`: deleted public APIs. Tests needing alternate gains construct DriveBase with explicit tuning.

The exact order may change during implementation if compile dependencies require it, but `Drive` must migrate before behavior can be validated.

Additional required migration targets:

- `CommandVector`: rename `LeftMotorPwm()` / `RightMotorPwm()` and storage/constructor names to command-space names such as `LeftCommand()` / `RightCommand()`. Reserve `Pwm` for actual hardware PWM paths.
- `Drive.h` private control helpers and telemetry fallback reads.
- `AuxMeasurementModeSupport` and any measurement support that consumes old DriveBase telemetry.
- `SrUkfCore` runtime-context/saturation inputs, including any `saturationFlags` dependency.
- OpenFloor, Showcasing, Diagnostic, and Aux mmlog schemas that currently mirror DriveTelemetry names.

### Build And Log Metadata

If implementation adds, removes, or splits files, update `MazeMap.vcxproj`, `MazeMap.vcxproj.filters`, `MazeMapTest.vcxproj`, and `MazeMapTest.vcxproj.filters` in the same change.

If feedback strategy vocabulary is split out of `FeedbackAxis.h`, add that header deliberately and verify host and Teensy include paths. Do not keep `FeedbackAxis` compiled only to preserve an enum or configuration name.

Mmlog schema migration is not field reuse. Schemas that currently contain `feedback_command`, `feedforward_command`, drive command, encoder count, encoder distance, or encoder velocity fields must either keep the old source/meaning or introduce renamed/versioned fields. Any mmlog schema that keeps encoder fields must source them from `SensorSnapshot`/`VehicleState`, not DriveBase telemetry.

### Static Migration Gates

Before verification, these searches must prove no compiled source still depends on the old design:

- no compiled declaration or call of `PointCommand`,
- no compiled declaration or call of `PointControlVector`,
- no compiled declaration or call of `DeltaCommand`,
- no compiled declaration or call of `DeltaYawRateCommand`,
- no compiled declaration or call of `PointYawRateCommand`,
- no compiled declaration or call of `GetFeedbackCommand`,
- no compiled DriveBase public `Begin`, `Brake`, `ResetControllers`, `CurrentControlVector`, `GetAverageDistanceMeters`, `GetLastFeedforward`, or `GetLastFeedback`,
- no DriveBase encoder/IMU cache fields,
- no DriveBase include of `MotionLimits`,
- no DriveBase include or instantiation of `FeedbackAxis`,
- no DriveBase fields for previous feedback samples, previous targets, previous errors, or previous timestamps,
- no DriveBase calls to sampled-derivative or derivative-history reset APIs,
- no additive feedback-command telemetry fields,
- no old temporary DriveBase reference file under compiled or sketch paths.

## Validation Plan

Validation must prove that `DriveBase` is a deterministic single-tick effort solver and disprove slides into planning, sensing ownership, hidden limits, or silent authority loss.

The validation plan does not claim to prove record-setting physical robot performance. It proves the software architecture can expose and survive the command-authority conditions required to pursue performance beyond the Decimus 5A floor.

### Unit Tests

Required coverage:

- exactly one production proposal path exists,
- zero-error velocity/yaw/heading proposal leaves commanded accelerations equal to caller-supplied feedforward accelerations,
- finite zero acceleration objectives remain zero unless caller or scalar intent requests otherwise,
- NaN velocity/yaw/heading objectives disable the corresponding feedback objective,
- infinite acceleration objectives reach PlantModel as maximize intent and are not replaced with finite constants,
- forward velocity feedback changes only the composed forward acceleration before `PlantModel`, not the forward velocity objective,
- yaw-rate feedback changes only the composed yaw acceleration before `PlantModel`, not the yaw-rate objective,
- heading error becomes yaw-acceleration correction before `PlantModel`,
- PlantModel command matches `PlantModel` for the composed inverse-dynamics request,
- parameter perturbation or an instrumented PlantModel proves DriveBase does not duplicate plant equations,
- same request plus same authoritative state plus same tuning plus same fan/downforce id produces identical correction and command regardless of prior proposal calls,
- proposal results for a given input tuple are independent of call order,
- `ClearCommandEvidence()` between identical proposals does not change the second proposal except telemetry freshness/sequence fields,
- missing measured-rate signals produce zero derivative contribution and a validity/branch marker only when the condition is not host-derivable,
- heading derivative uses current measured yaw rate, not sampled heading error,
- velocity and yaw-rate feedback do not finite-difference encoder, gyro, yaw, velocity, or acceleration samples inside DriveBase,
- actuator residual trim is absent from normal mission commands,
- clipping is derivable from PlantModel command, residual trim, and final proposed command when those fields are logged,
- scalar intent decoding records only non-derivable branch/failure decisions and does not poison later proposals,
- `LastTelemetry()` always matches the most recent proposal,
- mode/runtime logs, not DriveBase telemetry, provide timing and runtime-sample join keys where needed,
- telemetry proves each axis has one acceleration command after arbitration and no feedback-mutated velocity objective,
- old `PointCommand`/`DeltaCommand` APIs are not present.

### Simulation Tests

Required scenarios:

- step forward velocity at low, medium, and near-Decimus acceleration,
- step yaw rate at rest,
- step yaw rate at speed,
- combined forward/yaw acceleration near traction limit,
- fan-on 80 percent authority cases around the project's documented 16.5 m/s^2 lateral capability,
- reduced-traction/fan-off authority loss reported as authority loss, not success,
- encoder mismatch and IMU disagreement under configured feedback strategy,
- high-frequency feedback jitter and alternating saturation checks.
- sustained high-speed turn window with no alternating saturation chatter above the chosen threshold.

Quantitative gates must be attached to these tests before implementation. Required gates include:

- fan-on 80 percent cases covering the project reference of `16.5 m/s^2` lateral authority,
- combined acceleration/yaw cases within 5-10 percent of normalized command saturation,
- derived side-specific authority deficit signs and magnitudes within replay tolerance,
- maximum alternating saturation count over the high-speed turn window,
- solver execution time below the active control-loop budget on Teensy O2+LTO,
- zero dynamic allocation in the proposal path.

### Log Falsification

A failed high-speed run log must explain whether the likely failure was:

- model/feedforward error,
- physical feedback correction instability,
- actuator saturation,
- fan/downforce regime mismatch,
- scalar intent decoding or solver failure,
- sensor/runtime state problem.

If DriveBase command evidence cannot support that distinction when joined with runtime/sensor logs, the telemetry contract is insufficient.

Negative falsification cases must intentionally create:

- model/feedforward error without saturation,
- saturation with correct model,
- wrong fan/downforce reference,
- wrong feedback strategy or non-derivable feedback branch,
- scalar intent or solver failure on one axis only,
- stale or missing runtime state sample id,
- sensor disagreement outside DriveBase ownership.

A passing log must distinguish these cases without DriveBase duplicating raw sensor state.

### Release Verification

Before testing, verify active binaries are based on the latest changes by timestamp/artifact checks. Build only if the active artifacts are stale or if rebuilding is necessary to solve a specific task problem.

Verification should run release-mode unit tests where supported, including:

- `DriveBaseTest`,
- `SharedRuntimeTest`,
- `DriveManeuverTests`,
- host build verification,
- Teensy build verification when required by the task.

For this redesign, Teensy verification is required if any edited files affect embedded build inputs. Required embedded gates:

- Teensy O2+LTO compile,
- bounded stack growth for DriveBase proposal path,
- no dynamic allocation in the proposal path,
- no new large public header fan-out from `DriveBase.h`,
- proposal path measured or statically bounded under the control-loop deadline.

## Approval Conditions

The design is approved only if implementation satisfies all of the following:

- `DriveBase` remains the single public owner of low-level drive effort proposal.
- The public API uses physical tick language.
- The public API has one production proposal method.
- Public callers do not supply present operating state to DriveBase.
- DriveBase outputs and telemetry use project command-space language, not hardware PWM language.
- Feedback acts on physical request inputs before `PlantModel`.
- Velocity, yaw-rate, and heading feedback produce acceleration-domain corrections; DriveBase does not create feedback-mutated velocity or yaw-rate targets.
- IEEE scalar objective semantics are preserved across Drive, DriveBase, PlantModel, boot modes, tests, and host replay.
- Actuator residual trim is absent from the DriveBase public API.
- Every proposal atomically updates command evidence telemetry.
- Telemetry contains solver evidence with validity flags and join keys, not duplicated sensor state.
- Saturation and authority deficit are reconstructible from logged solver evidence when not logged directly.
- Fan/downforce reference is consumed through a named authoritative owner outside DriveBase.
- Old public APIs are deleted, not wrapped.
- `Drive` remains the owner of multi-tick motion semantics.
- `PlantModel`, `Vehicle`, `MotionLimits`, `LoopController`, and `VehicleState` retain their canonical ownership.

## Hard Rejection Checks

Reject the implementation if any of these occur:

- `DriveBase` owns motion limits.
- `DriveBase` owns fan policy.
- `DriveBase` starts or stops loop sessions.
- `DriveBase` applies hardware directly.
- `DriveBase` stores duplicate encoder/IMU state caches.
- `DriveBase` publishes public helper families around its internals.
- additive PWM feedback is used for normal mission locomotion.
- DriveBase computes plant equations or actuator authority from first principles instead of using PlantModel-owned behavior.
- DriveBase accepts caller-supplied present state.
- DriveBase reuses output-domain PD gains as physical correction gains without redefining units and tuning ownership.
- DriveBase changes both velocity objective and acceleration command from the same feedback error.
- DriveBase treats measured-acceleration feedback as a peer trajectory objective instead of a separately approved disturbance/inner residual.
- silent clamping remains.
- DriveBase treats IEEE scalar command intent as malformed numeric input.
- old command APIs remain compiled.
- old mmlog field names are populated with changed semantics.
- tests preserve noncanonical old names instead of validating the new physical contract.
- telemetry cannot distinguish requested scalar intent, PlantModel input, PlantModel output, residual trim when present, and final command evidence.
