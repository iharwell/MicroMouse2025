# LoopController Target Design

## Purpose

`LoopController` is the strict periodic timing authority for the robot control loop.

Its job is to:

- own the control-period schedule,
- apply the currently latched command at each tick boundary,
- capture granular timing diagnostics for the full tick,
- expose those diagnostics read-only to consumers that need them,
- provide a controlled pause path when a mode must temporarily leave the strict loop.

It is not responsible for:

- logging,
- boot-mode metadata,
- per-mode result formatting,
- fault publication authority.

`SharedRobotRuntime` remains the owner of runtime-wide faults and shared logging.

## Ownership Rules

`LoopController` should be owned by `SharedRobotRuntime`, but its public API should not expose runtime plumbing.

That means:

- no public logger handles,
- no public `RuntimeBundle`,
- no public raw subsystem pointers,
- no public boot-mode identifier, descriptor, or session-name fields.

If the loop must expose acquisition and estimator-work choices, it should expose those as low-level work-plan knobs rather than as a mode/pipeline identity.

That means:

- the loop should not publicly expose "mission pipeline" versus "diagnostic pipeline",
- the loop should publicly expose which sensor observations are acquired,
- the loop should publicly expose which estimator update stages are enabled,
- the loop should not publicly reference a concrete UKF type.

## Public API

The intended public shape is small:

```cpp
class LoopController final
{
public:
    struct SessionOptions;
    struct SessionResult;
    struct ModeCallbacks;
    struct ModeState;
    struct TimingDiagnostics;
    struct ControlVector;
    class TickServices;

    bool BeginSession(const SessionOptions& options, const ModeCallbacks& callbacks);
    SessionResult Run();
    void EndSession();

    bool SessionActive() const noexcept;
    const TimingDiagnostics& LastDiagnostics() const noexcept;
    const ControlVector& LastAppliedCommand() const noexcept;
};
```

### SessionOptions

`SessionOptions` should contain only loop-owned execution choices.

The default expectation is:

```cpp
enum class WallMask : std::uint8_t
{
    None = 0x00,
    Front = 0x01,
    Left = 0x02,
    Right = 0x04,
    All = 0x07
};

struct SensorWorkPlan final
{
    WallMask wallMask{ WallMask::All };

    bool readEncoders{ true };
    bool readImuBundle{ true };

    bool useEncoderUpdate{ true };
    bool useGyroUpdate{ true };
    bool useAccelUpdate{ true };
    bool useWallUpdates{ true };
};

struct SessionOptions final
{
    std::uint32_t controlPeriodUs{};
    SensorWorkPlan workPlan{};
};
```

Notes:

- The loop always starts in active brake.
- There is no startup-command policy.
- There is no initial-command field.
- There is no idle-sleep or wait-state policy in the public session contract.
- `SensorWorkPlan` is fixed for the life of a session.
- If the work plan needs to change, end the current loop session and start a new one with new options.
- `readImuBundle` is one acquisition switch because the gyro-Z and accel-X/accel-Y signals arrive from the same IMU read.
- The `use*Update` fields describe which estimator-update stages are executed during the interlaced capture/update pass.
- The loop does not need to mutate the work plan in response to bad observations; the runtime-owned estimator path is expected to reject invalid data during update as needed.
- If the loop must ever support another genuinely loop-owned execution choice, it should be added only if it changes the loop's own scheduling/capture behavior rather than mode behavior.

### SessionResult

`SessionResult` should describe loop control-flow outcome, not canonical fault details.

A tight shape is:

```cpp
struct SessionResult final
{
    enum class Status : std::uint8_t
    {
        Completed,
        StoppedByRuntime
    } status{ Status::Completed };

    std::uint32_t tickCount{};
};
```

Notes:

- Runtime fault details remain owned by `SharedRobotRuntime`.
- `LoopController` may detect a stop condition, but it should not become the authoritative fault record.

## Runtime Sensing/Update Tie-In

The loop should not directly depend on a concrete UKF type or on a concrete estimator object type.

The sensor-acquisition and estimator-update path is interlaced:

- encoders are acquired and used,
- the IMU observation bundle is acquired and used,
- wall observations are acquired and used,
- UKF predict and update phases occur between and around those acquisition points.

Because of that, `LoopController` should depend on one runtime-owned sensing/update tie-in rather than on direct references to a concrete estimator implementation.

That tie-in should:

- accept the current `SensorWorkPlan`,
- accept the current `stationaryHint`,
- accept the current tick `dt`,
- write timing markers directly into the loop-owned working diagnostics slot,
- produce the observed state the loop needs for mode callbacks,
- internally decide how to sequence predict/update work against the runtime-owned estimator.

Conceptually:

```cpp
struct RuntimeSensingTieIn final
{
    bool (*execute)(
        void* context,
        const SensorWorkPlan& workPlan,
        bool stationaryHint,
        float dtSeconds,
        TimingDiagnostics& timing,
        void* observedStateOut) = nullptr;

    void* context{};
};
```

The exact syntax may differ, but the architectural rule should remain:

- `LoopController` owns schedule and timing publication,
- the runtime tie-in owns concrete sensor/UKF sequencing,
- UKF changes should not force public `LoopController` API churn.

This tie-in should be private or internal-only.

It should not become another public framework family.

## Command Semantics

`LoopController` holds two command concepts:

- `LastAppliedCommand()`: the command actually applied at the start of the current/most recent tick,
- the internally queued next command: the command returned by the mode callback and latched for the next tick boundary.

At session start:

- both commands are brake,
- the first tick starts with active brake,
- no mode may bypass that startup behavior.

The mode callback always returns a command for the next application point, never for the current tick retroactively.

## Timing Diagnostics Surface

The timing diagnostic API should be based on the data needed by open-floor timing capture.

That means the published diagnostic snapshot should include:

```cpp
struct TimingDiagnostics final
{
    std::uint32_t sequence{};
    std::uint32_t tickStartUs{};
    std::uint32_t dtUs{};

    std::uint32_t controlStartUs{};
    std::uint32_t controlEndUs{};
    std::uint32_t pwmLatchUs{};
    std::uint32_t encoderLatchUs{};
    std::uint32_t encoderReadDoneUs{};

    std::uint32_t ukfPredictStartUs{};
    std::uint32_t ukfPredictEndUs{};
    std::uint32_t ukfPredictDurationUs{};
    std::uint32_t ukfUpdateStartUs{};
    std::uint32_t ukfUpdateEndUs{};
    std::uint32_t ukfUpdateDurationUs{};

    std::uint32_t imuDrdyUs{};
    std::uint32_t imuReadStartUs{};
    std::uint32_t imuReadDoneUs{};

    std::uint32_t frontLedOnUs{};
    std::uint32_t frontAdcOnUs{};
    std::uint32_t frontLedOffUs{};
    std::uint32_t frontAdcOffUs{};
    std::uint32_t frontReadyUs{};

    std::uint32_t leftLedOnUs{};
    std::uint32_t leftAdcOnUs{};
    std::uint32_t leftLedOffUs{};
    std::uint32_t leftAdcOffUs{};
    std::uint32_t leftReadyUs{};

    std::uint32_t rightLedOnUs{};
    std::uint32_t rightAdcOnUs{};
    std::uint32_t rightLedOffUs{};
    std::uint32_t rightAdcOffUs{};
    std::uint32_t rightReadyUs{};

    std::uint32_t cycleCounterStart{};
    std::uint32_t cycleCounterEnd{};

    std::uint16_t tActuationAppliedUs{};
    std::uint16_t tModeReturnUs{};
    std::uint16_t tPostServiceDoneUs{};
    std::uint16_t overrunUs{};

    std::uint8_t flags{};
};
```

This struct exists so diagnostic consumers can read precise loop timing without `LoopController` owning any log format.

## Two-Slot Timing Buffer

The timing diagnostics implementation should use a two-element ring buffer, like a double-buffered framebuffer; the obvious implication is that external readers see only the previous completed tick's timing until the current tick is finalized and published.

One slot is the published, read-only snapshot exposed through `LastDiagnostics()`.
The other slot is the private write target for the in-progress tick.

Suggested private members:

```cpp
TimingDiagnostics _timingBuffers[2]{};
std::uint8_t _publishedTimingIndex{ 0U };
std::uint8_t _workingTimingIndex{ 1U };
```

Suggested helpers:

```cpp
TimingDiagnostics& WorkingTiming() noexcept
{
    return _timingBuffers[_workingTimingIndex];
}

const TimingDiagnostics& PublishedTiming() const noexcept
{
    return _timingBuffers[_publishedTimingIndex];
}

void PublishWorkingTiming() noexcept
{
    _publishedTimingIndex = _workingTimingIndex;
    _workingTimingIndex ^= 1U;
}
```

Implementation rules:

- Do not build a stack-local timing snapshot for the tick.
- Write timing fields directly into the inactive working slot.
- Zero/reset the working slot at the beginning of each tick.
- Publish only after the tick's timing is fully finalized.
- `LastDiagnostics()` always returns the published slot, never the in-progress slot.

Benefits:

- no per-tick timing copy-out,
- no public exposure of mutable timing storage,
- clear ownership of "currently being written" versus "safe to read",
- low stack pressure and predictable memory access,
- very readable publish semantics.

## ModeState Semantics

The mode callback should not receive a state vector describing the past application point.

It should receive a state vector that is representative of the moment when the command it returns is expected to apply.

That requirement should be explicit in the code comments and the header contract.

Suggested shape:

```cpp
struct PoseState final
{
    float xMeters{};
    float yMeters{};
    float yawRad{};
    float linearSpeedMps{};
    float angularSpeedRadps{};
};

struct EncoderState final
{
    float leftVelocityMps{};
    float rightVelocityMps{};
    bool observationValid{};
};

struct ImuState final
{
    float gyroZRadps{};
    float accelXMps2{};
    float accelYMps2{};
    bool accelBiasValid{};
};

struct WallState final
{
    float frontLeftDistanceM{};
    float frontRightDistanceM{};
    float sideLeftDistanceM{};
    float sideRightDistanceM{};
    bool frontWall{};
    bool leftWall{};
    bool rightWall{};
};

struct ModeState final
{
    std::uint32_t sequence{};
    std::uint32_t commandApplyTimeUs{};
    std::uint32_t dtUs{};

    PoseState pose{};
    EncoderState encoders{};
    ImuState imu{};
    WallState walls{};
    bool estimatorHealthy{ true };
    bool overrun{};
};
```

Public loop-facing state should use loop-owned value types like the above rather than directly exposing broad project/runtime structs such as `PoseEstimate`, `SensorSnapshot`, or `DiagnosticSensorSnapshot`.

The exact field list should be the smallest stable subset the migrated modes actually need. The point is not to mirror every internal runtime struct; the point is to publish a minimal stable state contract for mode logic.

Required documentation sentence:

> The `ModeState` passed to `ModeCallbacks::onModeWork(...)` represents the best loop-owned estimate of robot state at the next command-application boundary. The `ControlVector` returned by that callback is latched for that boundary and is not applied retroactively to the current tick.

### How To Build ModeState

The desired implementation is:

1. apply the currently queued command at the current tick boundary,
2. capture sensors and update odometry/estimation for the current tick,
3. form the current observed state,
4. determine the next nominal command-application time,
5. project the observed state produced by the runtime sensing/update tie-in forward to that application time,
6. pass that projected state into `ModeCallbacks::onModeWork(...)`.

This is important.

If the callback is choosing the command for the next tick boundary, the callback should not need to mentally undo a one-tick delay. The loop should provide the state in the same temporal frame as the returned command.

### Overrun Case

If the loop has already consumed the full period before the callback state is built:

- set the overrun flag,
- clamp the forward projection interval to zero,
- pass the best current post-capture estimate into the mode,
- record the overrun precisely in `TimingDiagnostics`.

This keeps the state contract honest while still making the schedule violation explicit.

## Mode Callback Surface

The desired mode contract is a narrow callback set rather than a full interface object.

The primary callback is the in-loop "mode work can happen now" callback.

Suggested shape:

```cpp
using ModeWorkCallback = ControlVector (*)(
    void* context,
    std::uint32_t loopEndTimeUs,
    const ModeState& state,
    TickServices& services);

using PauseCallback = PauseDisposition (*)(
    void* context,
    const PauseContext& pause);

struct ModeCallbacks final
{
    ModeWorkCallback onModeWork{};
    void* context{};
};
```

Notes:

- `onModeWork` is the primary mode-side callback inside the loop.
- It runs after the control/sensor/UKF block has completed for the current tick.
- It is where mode logic consumes loop-produced state and decides the next command.
- `loopEndTimeUs` is an absolute timestamp in the loop's microsecond timebase, not a relative duration.
- It tells the mode the tick-end deadline it must finish before so the loop can still handle any optional post-mode log service and the final sync step.
- A full virtual interface is unnecessary for this purpose.

The loop may keep internal helpers around this contract, but the public session API should stay callback-oriented.

Ownership and lifetime rules:

- `BeginSession()` copies `ModeCallbacks` by value into loop-owned session state.
- The intended `context` is the durable mode instance that exists for the life of the program, or at minimum for the full active session.
- The normal pattern is a static/free thunk function that casts `context` back to that durable mode instance and calls the relevant phase method.
- Callback function targets and `context` must remain valid for the full active session.

Phase-transition rule:

- mode logic may retarget the next `onModeWork` callback to a different phase function while keeping the same durable `context`,
- that retargeting applies only to subsequent callbacks, never to the currently executing callback,
- this exists to let one durable mode instance advance between phase handlers without reconfiguring the whole session.

Budget rule:

- `onModeWork` executes inside loop-owned time.
- Modes must stay within the budget the loop leaves available.
- The timing getter remains the public read path outside the callback.

## End-Of-Tick Policy

The loop should have exactly one per-tick wait block, and it should be the final sync wait at the end of the tick.

The intended per-tick order is:

1. run the control/sensor/UKF block,
2. run mode work,
3. service runtime-owned buffered logs only if sufficient pre-sync slack remains,
4. wait once for the next sync moment.

This matters for three reasons:

- buffered log service should never steal time the control path does not actually have,
- a single end-of-tick wait block is the only place that should absorb normal timing uncertainty,
- the loop structure stays readable because all discretionary slack handling happens in one place.

Clarification:

- the "if sufficient pre-sync slack remains" rule applies to normal buffered-log servicing,
- fault-path logging/export is priority work, not discretionary housekeeping,
- pause/end/stop requests are latched outcomes handled at the loop's next safe control-flow point rather than immediate mid-tick preemption.

The loop should not:

- service logs speculatively before mode work,
- split waiting across multiple independent boundary-wait phases,
- add a second per-tick wait at the beginning of the next iteration,
- hide schedule uncertainty behind repeated wait/service loops.

## TickServices

`TickServices` should stay narrow.

It should support:

- explicit end-of-session request,
- explicit pause request,
- optional retargeting of the next `onModeWork` callback,
- runtime stop request forwarding when the mode detects a condition that should escalate to `SharedRobotRuntime`.

There should be no generic idle or wait-state service knobs here.

Suggested pause direction:

```cpp
struct PauseRequest final
{
    PauseCallback onPauseGranted{};
};

services.RequestPause(pauseRequest);
```

Pause rules:

- `PauseRequest::onPauseGranted` must never be null.
- A pause request without a callback has no useful meaning and should be represented as end-of-session or some other explicit control-flow action instead.
- The pause callback uses the same durable mode `context` already stored in the active session callback set.

Suggested phase-switch direction:

```cpp
services.SetNextModeWorkCallback(nextPhaseCallback);
```

The actual heavy-work body is supplied through the pause callback passed into `RequestPause(...)`. The important architectural rule is:

- leaving the strict periodic loop must be explicit,
- a pause request is a request to be honored at a safe point, not an immediate demand,
- it must not happen through silent caller-driven gaps between ticks,
- it must not happen through ad hoc idle-sleep configuration.

## Run Loop

`Run()` should own the entire active cadence.

The production flow should be:

1. validate `SessionOptions`,
2. arm the session,
3. set queued/applied command to brake,
4. initialize the first sync target,
5. loop while session remains active.

For each tick:

1. reset the working timing slot,
2. stamp tick start and sequence directly into the working slot,
3. apply the currently queued command,
4. stamp actuation-applied timing,
5. execute the runtime sensing/update tie-in using the current `SensorWorkPlan`, with that tie-in stamping acquisition/update timing directly into the working slot,
6. compute the next application time,
7. build `ModeState` projected to that application time,
8. compute the absolute `loopEndTimeUs` deadline for the current tick,
9. call `ModeCallbacks::onModeWork(...)`,
10. stamp mode-return timing,
11. resolve any latched pause/end/runtime-stop/fault request into the tick's control-flow outcome,
12. compute how much time remains before the next sync target,
13. if a fault path needs runtime-owned log flush/export work, prioritize that logging path,
14. else if enough pre-sync slack remains and the tick remains on the normal cadence path, service runtime-owned buffered logs,
15. stamp post-service timing,
16. finalize control-end, PWM-latch, cycle-counter-end, and overrun timing,
17. publish the working timing slot,
18. latch the returned command for the next tick or override it with the required controlled-stop/brake command according to the resolved control-flow outcome,
19. advance the next sync target by exactly one period,
20. wait once for that sync target.

Important schedule rule:

- each tick gets one normal wait block, and it is the final sync wait,
- the next sync target is derived from the previous sync target plus one period,
- not from the wall-clock time at the end of the current tick,
- and if the tick overruns that target, the wait collapses to zero while the overrun is recorded explicitly.

That prevents schedule drift.

## Pause Behavior

Pause is the only legitimate way to leave the strict periodic loop for non-periodic work.

Desired pause flow:

1. mode requests pause,
2. the loop latches that request and continues normal tick ownership until it reaches a safe point to honor it,
3. the loop brakes and settles to rest,
4. the loop publishes the final settled timing/state snapshot if needed,
5. strict cadence is suspended,
6. the pause callback supplied with `RequestPause(...)` executes,
7. the pause result chooses resume or complete,
8. resume returns to the loop with brake as the active command until a later tick requests otherwise.

There should be no public idle-sleep configuration used as a substitute for pause.

## Fault Ownership

`LoopController` may detect local stop conditions, but `SharedRobotRuntime` remains the fault authority.

That means:

- runtime owns the canonical fault record,
- runtime owns fault publication,
- runtime owns fault-related logging,
- runtime owns any broader cleanup/distribution behavior.

`LoopController` should translate local problems into "stop this session through runtime" rather than becoming a second fault owner.

Additional rule:

- a runtime stop or fault escalation is a latched control-flow outcome, not an immediate mid-tick abort,
- the loop must preserve the control action needed to bring the vehicle to rest cleanly,
- fault-path logging/export is the highest-priority logging work because it may be the only path to get diagnostic data out of the system.

## Open-Floor Use

Open-floor timing capture should consume the loop timing through the loop-owned diagnostics storage, using the in-loop mode-work callback when it needs the current tick's budgeted execution point and the diagnostics getter when it needs the latest published snapshot.

That means:

- `LoopController` owns timestamp generation,
- open-floor owns row formatting and log emission,
- the mode callback sees published timing from the previous completed tick, not the current in-progress tick,
- no open-floor code should stamp loop timing fields such as `controlEndUs`, `pwmLatchUs`, or `cycleCounterEnd`.

The timing row becomes a pure readout of loop-owned diagnostics.

## Migration Direction

The implementation should converge in this order:

1. remove public runtime/logging exposure from `LoopController`,
2. reduce the session-start contract to minimal loop-owned options plus `SensorWorkPlan`,
3. bind `LoopController` privately to `SharedRobotRuntime`,
4. introduce the internal runtime sensing/update tie-in so the loop stops depending on concrete UKF-shaped calls,
5. add the two-slot `TimingDiagnostics` buffer,
6. stamp all loop timing directly into the working slot,
7. make the published diagnostics getter authoritative,
8. document and implement `ModeState` as "state at next command application time" using loop-owned value types rather than broad runtime structs,
9. replace the full mode interface with the narrow callback-oriented contract centered on the in-loop mode-work callback,
10. remove remaining mode-local loop timing stamping,
11. remove public `RunOneTick()` from the production API.

## Summary

The target `LoopController` is:

- brake-first,
- schedule-owning,
- runtime-owned,
- logger-independent,
- fault-non-authoritative,
- explicit about pause,
- explicit about command timing semantics,
- explicit about the meaning of mode callback state,
- and authoritative for granular loop timing through a published two-slot diagnostic buffer.
