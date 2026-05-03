# OpenFloorMeasurementController Restructure Notes

## Purpose

Capture the current implementation proposal for converging `OpenFloorMeasurementController` into a scalable, reconfigurable open-floor test-battery owner without depending on thread context.

This note is intended to be strong enough to hand to another agent for implementation, including an initial approved verification target.

## Scope

This proposal is intentionally **internal-first**.

- `OpenFloorMeasurementController` remains the single authoritative owner of the open-floor measurement battery.
- The first convergence happens **inside this controller**, not by extracting a shared framework immediately.
- The goal is to separate **what** the battery runs from **how** it executes, while preserving the existing external logging contract.

## Immediate Ownership Decisions

This handoff note assumes the implementation makes these ownership choices now rather than leaving them open:

- `OpenFloorMeasurementController` remains the only public owner for this mode.
- `TimingStage` and `MainStage` are internal controller-owned execution stages, not new public subsystem owners.
- Logging mechanics should move into controller-owned stage/logging support inside the open-floor controller ownership boundary, not into a new shared framework.
- Open-floor row/schema ownership should become explicit within the controller's own authoritative files rather than remaining buried as incidental large file-local implementation detail.

## Current Problem

The current controller mixes together:

- battery definition and section ordering
- per-phase execution mechanics
- logging/session plumbing

The present shape scales poorly because it duplicates:

- per-phase runtime structs
- per-phase start functions
- per-phase tick functions
- per-phase transition logic
- per-phase log/fault boilerplate

## Core Direction

`OpenFloorMeasurementController` should become a direct owner with:

- one internal compiled battery plan for the main measurement battery
- one stable stage boundary between the direct `LoopController` callback and executable test segments
- one stable executor interface for compiled segments

The important internal split is:

- **What**: define and compile the open-floor battery into an ordered list of executable main-stage segments before execution begins.
- **How**: run those segments through a stable stage/executor path that does not change when new phases are added.

## Stage Boundary

The most important architectural decision is to place **one layer** between the direct `LoopController` callback and the executable test segments.

That layer should be a **stream/stage** split, not another phase split.

### The two stages

- `TimingStage`
- `MainStage`

### TimingStage

`TimingStage` is **outside the compiled segment model**.

It exists because timing capture:

- uses a different schema
- serves a different purpose
- has its own explicit transition into the main stream

It should be implemented as stage-owned behavior, not as a compiled segment family.

### MainStage

`MainStage` owns the compiled flat segment plan for the actual open-floor battery.

Everything after timing capture belongs to `MainStage`.

### Stable callback shape

The direct `LoopController` callback should dispatch only to the active stage:

```cpp
return (this->*_activeStageTick)(state, services);
```

Within `MainStage`, execution proceeds by dispatching to the current compiled segment executor:

```cpp
return _activeSegment.executor->tick(
    *this,
    _activeRuntime,
    *_activeSegment,
    state,
    services);
```

Adding a new battery phase should not require changing either dispatch site.

## Why This Stage Boundary Matters

This boundary lets the controller move **logging mechanics** off of the individual phases/segments without inventing another unstable switch-based phase machine.

The stage owners can cleanly own:

- stream open/setup
- metadata writing
- schema begin
- row population
- pending-row staging
- pending-row flush
- stage-specific fault handling boilerplate
- the timing-to-main transition

The segments then supply only:

- semantic sample identity
- command/execution behavior
- their own local progress/runtime state

## Compiled Battery Plan

The main-stage battery should be compiled into a flat ordered segment list during setup.

That compiled plan should expand:

- repeats
- sign alternation
- inter-segment holds
- section-to-section holds

The runtime engine should then advance linearly through the compiled list.

This removes the need for schedule-as-control-flow machinery such as:

- `HasRemainingLaunchSamples()`
- `StartNextLaunchSample()`
- `HasRemainingStraightSamples()`
- `StartNextStraightSample()`
- `HasRemainingYawSamples()`
- `StartNextYawSample()`
- `StartNextSmoothEntry()`
- `StartNextLoopEntry()`
- `HoldContinuation`

Those are signs that battery definition is still encoded in control flow instead of plan data.

## Executor Families, Not Named Phases

The correct boundary is **executor family**, not **named phase**.

Bad model:

- one state type for launch
- one state type for straight
- one state type for yaw
- one state type for smooth
- one state type for loop

Better model:

- one immutable payload shape per reusable execution family
- one mutable runtime shape per reusable execution family

This means a new phase that is "another straight test with different parameters" reuses the existing straight family instead of inventing another controller-local state type.

## Recommended Main-Stage Families

The current proposal supports these families cleanly:

### 1. Stationary hold

Used for static holds and inter-segment/inter-section holds once the controller is in `MainStage`.

### 2. Wheel-command profile

This should be generalized from the current launch pulse idea.

It should cover open-loop wheel-command profiles such as:

- symmetric launch pulse
- symmetric launch ramp
- anti-symmetric in-place turn launch pulse
- anti-symmetric in-place turn ramp

This family is important because likely future tests reuse open-loop wheel-space execution while changing only the profile data.

### 3. Drive primitive

Used for `Drive`-backed closed-loop primitives such as:

- straight tests
- in-place turn tests
- maneuver-based segments when the `Drive` service remains the executor

### 4. Curvature profile

Needed for a constant-curvature circular path with changing speed over time.

This is a real new family because current `Drive::StartArc(...)` does not represent caller-directed speed changes through the turn.

## Logging Model

The controller rewrite should move **logging mechanics** off of the segments and into the two stage owners.

### What TimingStage should own

- timing stream open/setup
- timing metadata writing
- timing schema begin
- timing-row population from runtime state
- timing-row staging
- timing-row flush
- timing fault checks
- deciding when timing capture is complete
- requesting the pause/handoff into main

### What MainStage should own

- main stream open/setup
- main metadata writing
- main schema begin
- main-row population from runtime state plus sample identity
- main-row staging
- main-row flush
- main fault checks
- advancing through the compiled segment list
- final flush/end behavior

### What stays with segment execution

The current segment/executor still has to provide the semantic identity for the sample:

- `section_id`
- `primitive_id`
- `phase_id`
- `speed_bin`
- `repeat_index`

That identity is the stable external row-level contract for `OpenFloorMainRow`.

Segments should not own:

- stream setup
- metadata writes
- schema begin
- staged-row flush logic
- timing/main transition logic

## Logging Semantics Contract

The implementation must preserve this meaning for each logged main row:

- the command that was active during that tick
- the phase/section/primitive/repeat/speed-bin identity associated with that active command for that tick
- the state information captured for that tick while that command was active
- the timing information for that same tick

If those are all true, it does not matter when the row is actually written to the logger buffer.

### Consequence

The delayed-write model is acceptable, but only if the staged row still preserves the active-command/state/timing meaning above when it is finalized and written.

### Stage tick contract

For implementation purposes, each main-stage tick should follow this semantic contract:

1. Start from the tick-start state that is visible to the active stage/segment.
2. Treat the logged row for that tick as describing the command already active during that tick and the identity associated with that active command.
3. Allow the active segment executor to choose the next command for the following tick without changing the meaning of the row being staged for the current tick.
4. Allow the actual write/flush timing to vary, so long as the committed row still means the same active command, identity, state inputs, and timing data for that tick.

This is the behavioral contract that matters. The physical buffer-write moment is not the contract.

## Logging Contract Constraint

The main row format should be treated as a fixed external contract.

Do **not** assume this rewrite can add new row fields such as `segment_id`.

The design must work while keeping the current row identity contract:

- `section_id`
- `primitive_id`
- `phase_id`
- `speed_bin`
- `repeat_index`

Any richer run-wide description should be considered sidecar metadata, not a row-layout change.

## Future-Test Stress Check

The current proposal was checked against these likely expansions:

### A. Repeating smooth ramp-up of raw forward command

Fits the `wheel-command profile` family.

No new controller architecture is required if the profile is represented as data.

### B. Launch-parallel open-loop in-place turn tests

Also fits the same `wheel-command profile` family.

The only change is the wheel command mapping:

- forward launch uses symmetric wheel commands
- in-place turn launch uses anti-symmetric wheel commands

### C. Slowly accelerating constant-curvature circular path across radii

Requires a real new executor family, `curvature profile`.

This is acceptable and does **not** invalidate the design because it still adds:

- one new reusable family
- one new payload/runtime shape

instead of another controller-local phase machine.

## Effective Stop Condition

This rewrite is done when all of the following are true:

1. `OpenFloorMeasurementController` is the real owner, rather than a thin shell over a hidden mega-state implementation.
2. Timing capture is stage-owned behavior outside the compiled segment model.
3. Main-stage battery execution is driven by a compiled flat segment plan.
4. The direct `LoopController` callback only dispatches to `TimingStage` or `MainStage`.
5. `MainStage` dispatches to compiled segment executors without a growing phase-name switch.
6. The current per-phase scheduling helpers and continuation graph are removed.
7. Logging mechanics are owned by the two stages rather than individual phases/segments.
8. The main row schema and row-level identity contract remain unchanged.
9. The emitted rows still mean: command active during the tick, identity associated with that command for that tick, state captured for that tick, and timing for that same tick.
10. The current battery ordering and coarse external behavior are preserved:
    - same timing-first / main-second split
    - same major section order
    - same repeat numbering semantics
    - same hold insertion semantics
    - same timing-to-main pause/handoff semantics
    - same tooling-facing row identity vocabulary

## What This Test-Bed Change Should Include

This rewrite **should** include:

- replacing the current per-phase machine with the internal test-bed architecture described here
- removing schedule-as-control-flow logic in favor of a compiled main-stage segment plan
- moving logging/session mechanics into `TimingStage` and `MainStage`
- keeping the implementation entirely inside the `OpenFloorMeasurementController` ownership boundary

## What This Test-Bed Change Should Not Include

This rewrite **should not** include:

- extracting a shared framework for other modes yet
- changing the external main row layout
- inventing a second public subsystem owner for open-floor execution
- broad Drive redesign unrelated to making the open-floor test bed work
- speculative support for modes or schemas not needed to prove the design inside this controller

## Constraints and Caveats

### 1. Internal-first only

This proposal should be proven out entirely inside `OpenFloorMeasurementController` before any extraction elsewhere.

### 2. No partial migration

Do not leave the old per-phase controller machine and a new plan-driven machine side by side.

If this rewrite happens, it should replace the current approach in one convergent change.

### 3. Logging mechanics can move, but semantic sample identity cannot disappear

Segments still need to supply the row identity used by tooling.

### 4. The delayed-write timing model must remain correct

The logger currently stages rows and patches timing fields from loop diagnostics before commit.

Any logging cleanup must preserve the logging semantics contract described above.

### 5. Timing/main stream handoff remains explicit

The runtime uses separate schemas/files for timing and main.

That transition is real and must stay correct even if the code is reorganized.

### 6. Row-schema ownership is still local today

The current open-floor row definitions are private to `OpenFloorMeasurementController.cpp`.

As part of this rewrite, row/schema ownership should become more explicit inside the controller’s own authoritative file/header structure.

### 7. Current log vocabulary may eventually become the limiting factor

The execution design can scale farther than the present coarse open-floor row identity vocabulary.

That is a separate observability constraint from the controller architecture itself.

## Verification

This handoff now includes an initial approved verification target.

### Required first-pass tests

- `TimingStage` stays outside the compiled plan: advance through timing capture only and verify only timing-schema rows are produced, no main-plan segments are consumed, and the handoff to main occurs only when timing completes.
- Heterogeneous compiled main segments execute in flat order: inject a short plan containing a hold, a wheel-command-profile segment, and a `Drive`-backed segment, then verify command sequence and row identities follow compiled order exactly.
- Active-command logging semantics survive delayed flush: use a state-dependent segment, capture the command active for a tick, perturb state/diagnostics before flush, and verify the committed row still matches that active command, identity, state inputs, and timing rather than a newly produced next-tick command.
- Main row schema stays unchanged: verify the rewritten controller opens the same main schema and emits the same field set/layout as the current `OpenFloorMainRow` contract.
- Completion honors the intended stop condition: run a short full main plan and verify the session ends only after the last segment completes and any pending row is flushed, with no extra commands or rows afterward.
- Timing-to-main handoff preserves both stream lifecycles: verify timing finalization, main schema startup, and correctness of the first main-row identity/timing across the transition.
- Flattened plan preserves coarse external identity behavior: verify `section_id`, `primitive_id`, `phase_id`, `repeat_index`, and `speed_bin` advance only at the expected boundaries in a repeated multi-bin sweep.
- Fault handling remains stage-owned: inject a fault during main execution and verify the last committed row still reflects the last valid active command/identity for its tick, followed by a clean stop or recovery transition without partial next-segment logging.

### High-value follow-on tests

- Data-defined holds replace continuation logic correctly.
- One executor family can cover multiple future wheel-command-profile variants without new dispatch structure.
- No rows are dropped or duplicated across delayed flush boundaries, including completion, pause, fault, and stage-switch edges.
- A deterministic identity-trace regression for the current nominal battery preserves externally relevant section/primitive/phase/repeat structure.
- Invalid compiled-plan shapes fail early and clearly at the compile/validate boundary.

### Recommended test seams

- a fake runtime logger that records schema changes and committed rows
- a deterministic tick/diagnostics harness
- a small plan-injection seam for tests
- a fake or stub `Drive` path so compiled main-stage behavior can be exercised without depending on full motion execution

## Summary

The present recommendation is:

1. Keep `OpenFloorMeasurementController` as the sole owner.
2. Rewrite it around a compiled flat main-stage battery plan.
3. Put one stable stage layer between the direct `LoopController` callback and the executable test segments.
4. Use `TimingStage` and `MainStage` as the logging/session boundary.
5. Keep timing outside the compiled segment model.
6. Dispatch from `MainStage` to compiled segment executors by stable executor reference, not by a growing `switch`.
7. Use executor-family payload/runtime shapes, not one struct per named phase.
8. Move logging infrastructure into the two stage owners.
9. Preserve the current main-row logging contract and active-command logging semantics.
10. Treat this controller as the proving ground before any wider extraction.
