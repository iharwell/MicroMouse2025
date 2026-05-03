# OpenFloorMeasurementController Restructure Notes

## Purpose

Capture the corrected implementation target for converging `OpenFloorMeasurementController` into a scalable, maintainable open-floor measurement battery owner without depending on thread context.

This note replaces the previous segment-family interpretation. It is intended to be strong enough to hand to another agent for implementation and verification.

## Scope

This proposal remains intentionally **internal-first**.

- `OpenFloorMeasurementController` remains the single authoritative public owner of the open-floor measurement battery.
- The first convergence happens **inside this controller**, not by extracting a shared framework immediately.
- The goal is to separate **stage infrastructure and sequencing** from **measurement-phase command generation**, while preserving the current timing/main split and the existing Main log format.

## Authoritative Ownership Decisions

This handoff note assumes the implementation makes these ownership choices now rather than leaving them open:

- `OpenFloorMeasurementController` remains the only public owner for this mode.
- `TimingStage` and `MainStage` are internal controller-owned execution stages, not new public subsystem owners.
- `TimingStage` owns timing-capture execution and timing-stream lifecycle only.
- `MainStage` owns Main-battery sequencing, Main-row labeling, hold policy, row staging/flush, and infrastructure faults.
- Each measurement phase is a private implementation class under controller ownership.
- Logging mechanics stay inside controller-owned stage infrastructure. Do not invent a new shared logging framework here.
- This controller does not own `logging.txt`, does not open the text logger, and does not close logs.

## Correction to the Previous Direction

The earlier direction was wrong because it still centered the design on:

- closed-set execution families
- central `variant`-based dispatch
- phase-owned row identity
- per-phase fault text
- phase-local settle-hold policy

That shape is rejected even if the types are private.

The extension unit must be:

- one measurement phase class

not:

- one central execution-family alternative

## Current Problem

The controller is still too easy to pull back toward a closed-set central dispatcher.

That happens when:

- every new test requires editing core controller-owned type families
- sequencing state lives inside phase/test objects
- row labels are treated as phase-owned runtime data
- hold policy is embedded inside phase execution
- infrastructure faults are interpreted by the active phase

That shape is still hard to extend and still keeps too much architectural knowledge centralized in the wrong place.

## Corrected Core Direction

`OpenFloorMeasurementController` should become a direct owner with:

- one stage-owned timing path
- one stage-owned Main battery sequencer
- one flat scheduled Main execution list
- one narrow private contract for arbitrary Main-log-compatible measurement phases

The important internal split is:

- **Infrastructure**: stages, sequencing, row labeling, holds, logging lifecycle, row staging/flush, and runtime fault handling
- **Measurement phase**: define cases, start a selected case, generate commands each tick, report completion

## Stage Boundary

The most important architectural decision remains the stage split:

- `TimingStage`
- `MainStage`

That split is correct and should be preserved.

### TimingStage

`TimingStage` stays outside the Main measurement battery.

It exists because timing capture:

- uses a different schema
- serves a different purpose
- has its own explicit transition into the Main stream

`TimingStage` should own:

- timing stream open/setup
- timing metadata writing
- timing schema begin
- timing-row population
- timing-row staging
- timing-row flush
- timing completion detection
- timing-to-main handoff
- timing-stage infrastructure faults

### MainStage

`MainStage` owns everything after timing capture.

It is not an executor-family dispatcher. It is the battery sequencer and Main logging owner.

`MainStage` should own:

- battery registration order
- phase-case compilation into flat scheduled work
- active phase selection
- active case selection
- `phase_id`, `primitive_id`, `speed_bin`, and `repeat_index` for the current Main row
- inter-case hold policy
- inter-phase hold policy
- Main stream open/setup
- Main metadata writing
- Main schema begin
- Main-row population from runtime state plus stage-owned active-case metadata
- Main-row staging
- Main-row flush
- infrastructure faults such as selector removal, estimator faults, and log write failures
- linearly advancing through scheduled work

## Measurement Phase Contract

Each Main-stage measurement phase should be one proper private class.

After compilation succeeds, a measurement phase must be total during execution.

That means the phase should not have:

- a runtime failure channel
- fault text
- stop-policy ownership
- mutable row-label sequencing state
- hold policy ownership

It should only need to do work equivalent to:

- define or compile its case space
- own any phase-local compiled artifacts it needs
- begin a selected case
- generate commands each tick for that selected case
- report completion

If a phase cannot justify an independent behaviorful class, it should not survive as a named type.

## Scheduling Model

The Main battery should be compiled by `MainStage`.

That compilation should become:

1. register phases in battery order
2. ask each phase to contribute its cases
3. let `MainStage` expand repetitions and attach row-label coordinates
4. finalize one flat scheduled execution list

At runtime, `MainStage` should:

1. run either a stage-owned hold or one active phase case
2. stamp the Main row from stage-owned active-case metadata
3. advance linearly through scheduled work

Holds are infrastructure, not measurement phases.

Settle holds must not be implemented inside phase classes.

## Row Labeling and Main Log Contract

The Main row format is an external contract and should stay fixed.

The important correction is ownership:

- `phase_id`
- `primitive_id`
- `speed_bin`
- `repeat_index`

belong to MainStage sequencing infrastructure, not to the measurement phase as mutable runtime state.

A measurement phase may define the available case metadata needed for those labels, but `MainStage` should own:

- case ordering
- repeat expansion
- active-case coordinates
- row stamping

The Main row for a tick must still mean:

- the command active during that tick
- the active phase/case coordinates associated with that tick
- the state captured for that tick
- the timing information for that same tick

If delayed flush is used, that semantic meaning must remain correct.

## Fault Ownership

Infrastructure faults belong to the stage/controller boundary, not to the measurement phase.

Examples include:

- selector removal
- estimator faults
- log open failures
- log write failures
- invalid timing/main handoff state

Measurement phases should not produce human-readable fault text and should not decide stop policy.

If something can fail before execution, that is a compile/setup/controller concern.

If something can fail during execution, that is a stage/infrastructure concern.

## Terminology

Internal `section` terminology is drift and should not be used as the controller's architectural vocabulary.

Use:

- `phase`

internally.

Only preserve legacy `section` vocabulary at an external boundary if some existing non-controller contract truly still requires it.

Do not let internal phase identity be built by casting or aliasing `OpenFloorSectionId`.

## Phase-Local Compiled Artifacts

If a measurement phase needs compile-time artifacts, those artifacts belong inside that phase.

Examples:

- smooth maneuver queues
- loop maneuver queues
- any compiled maneuver storage needed only by one phase

Those artifacts should not be promoted into `MainStage` just because they must survive across ticks.

`MainStage` should know only generic scheduled work, not the internals of every phase's compiled data.

## Rejected Shapes

The following shapes are explicitly rejected:

- central execution-family `variant` dispatch
- central `Plan` / `ExecutionState` type families
- one `TickXxxExecution(...)` method per family
- phase-owned row identity runtime objects
- per-phase fault text
- phase-local settle-hold policy
- data-only structs that exist only to carry execution internals
- new public wrappers, facades, helpers, managers, or compatibility shims

Private visibility does not make those shapes acceptable.

## Effective Stop Condition

This rewrite is done when all of the following are true:

1. `OpenFloorMeasurementController` is the real owner, rather than a shell over a hidden central dispatcher.
2. `TimingStage` remains stage-owned timing behavior outside the Main battery.
3. `MainStage` clearly owns sequencing, row labeling, holds, staging/flush, and infrastructure faults.
4. Main-stage execution is driven by a flat scheduled work list.
5. Measurement phases are proper private classes whose runtime behavior is only command generation plus completion.
6. No central closed-set execution-family dispatcher remains.
7. No per-phase fault text or runtime failure channel remains.
8. The current timing-to-main split and handoff semantics remain correct.
9. The Main row format and row meaning remain correct.
10. Adding a new Main-log-compatible measurement phase requires:
    - one new phase class
    - one reference in the primary battery-registration location
11. Adding that new phase does not require editing:
    - a central `variant`
    - a central dispatcher
    - a row-identity type
    - a fault-text method
    - a family-specific execution lattice

## Migration Guidance

The clean migration direction is:

- keep the `TimingStage` / `MainStage` split
- delete the segment/family architecture rather than renaming it
- move sequencing ownership harder into `MainStage`
- move phase-local compiled data into the corresponding phase classes
- delete phase-owned row-label runtime state
- delete phase fault semantics
- move hold policy into infrastructure

The current compile loops are already the right ownership area for sequencing, but the wrong abstraction for execution.

Use that fact to converge toward:

- stage-owned scheduling
- phase-owned command generation

not toward another closed-set central family system.

## What This Rewrite Should Include

This rewrite should include:

- replacing the current segment/family architecture with the phase-class architecture described here
- moving Main-row coordinates into `MainStage`
- moving hold policy into `MainStage`
- keeping phase-local compiled artifacts with their owning phases
- preserving the timing/main stream split inside the controller boundary

## What This Rewrite Should Not Include

This rewrite should not include:

- extracting a shared framework for other modes yet
- changing the external Main row layout
- introducing a second public subsystem owner
- inventing a new logging owner
- broad unrelated `Drive` redesign
- staged compatibility architecture that leaves old and new ownership in parallel

## Verification

This handoff includes an approved first-pass verification target.

### Required first-pass tests

- `OpenFloorMeasurementController_TimingFaultStopsBeforeMainStage`
- `OpenFloorMeasurementController_TimingToMainHandoffCreatesSeparateStreamsAndCommitsFinalTimingRow`
- `OpenFloorMeasurementController_FirstMainSegmentIdentityIsStaticHoldAndFaultDoesNotAdvancePastIt`

### Required verification path

- before testing, verify the active binaries actually reflect the latest edits
- do not build from scratch unless necessary to solve a specific problem
- verify through Release-mode unit tests
- perform a direct Teensy compile

### High-value follow-on checks

- a new phase can be added as one class plus one battery-registration reference
- no rows are dropped or duplicated across delayed flush boundaries
- hold insertion remains infrastructure-owned and occurs at the intended boundaries
- row-label sequencing still advances only at intended boundaries
- phase-local compiled artifacts stay local and do not leak back into `MainStage`

## Summary

The present recommendation is:

1. Keep `OpenFloorMeasurementController` as the sole public owner.
2. Keep the `TimingStage` / `MainStage` split.
3. Make `MainStage` the owner of sequencing, row labeling, holds, Main-row staging/flush, and infrastructure faults.
4. Make each measurement phase one proper private class.
5. Let phases compile cases, begin a selected case, tick commands, and report completion.
6. Remove central execution-family variants and dispatch lattices.
7. Remove phase-owned row identity and phase fault semantics.
8. Keep timing outside the Main battery.
9. Preserve the Main log format and row meaning.
10. Treat arbitrary measurement-phase support as the acceptance threshold, not mere reduction of the existing mess.
