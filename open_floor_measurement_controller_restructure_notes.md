# OpenFloorMeasurementController Restructure Notes

## Purpose

Capture the corrected implementation target for converging `OpenFloorMeasurementController` into a scalable, maintainable open-floor measurement battery owner without depending on thread context.

This note replaces the earlier wrong-shape direction. It is intended to be strong enough to hand to another agent for implementation and verification.

## Scope

This proposal remains intentionally **internal-first**.

- `OpenFloorMeasurementController` remains the single authoritative public owner of the open-floor measurement battery.
- The first convergence happens **inside this controller**, not by extracting a shared framework immediately.
- The goal is to separate **stage infrastructure and sequencing** from **measurement-phase command generation**, while preserving the current timing/main split and the existing Main log format.

## Non-Negotiable Ownership Decisions

- `OpenFloorMeasurementController` remains the only public owner for this mode.
- `TimingStage` and `MainStage` are internal controller-owned stages, not new public subsystem owners.
- `TimingStage` owns timing-capture execution and timing-stream lifecycle only.
- `MainStage` owns Main-battery sequencing, iteration state, Main-row labeling, hold policy, row staging/flush, and infrastructure faults.
- Each measurement phase is a private implementation class under controller ownership.
- Logging mechanics stay inside controller-owned stage infrastructure. Do not invent a new shared logging framework here.
- This controller does not own `logging.txt`, does not open the text logger, and does not close logs.

## What Was Wrong Before

The rejected direction still organized the controller around:

- central execution-family dispatch
- central type families and variants
- phase-owned row-label state
- phase-owned fault semantics
- phase-owned hold policy
- extra scheduling scaffolding such as explicit work-item, segment, or case-registration machinery

That shape is rejected even if the types are private.

The extension unit must be:

- one measurement phase class

not:

- one new alternative in a central dispatcher
- one new registration framework concept
- one new schedule-entry family

## Core Direction

The controller should have:

- one stage-owned timing path
- one stage-owned Main battery sequencer
- one ordered list of measurement phases
- one infrastructure-owned iteration cursor over the active Main battery

The important split is:

- **Infrastructure**: stages, sequencing, iteration, row labeling, holds, logging lifecycle, row staging/flush, and runtime fault handling
- **Measurement phase**: command generation for the currently selected measurement slot until complete

## Stage Boundary

The stage split remains correct:

- `TimingStage`
- `MainStage`

That boundary should be preserved.

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

It is not an executor-family dispatcher. It is not a case-registration system. It is not a segment scheduler.

It is the battery sequencer and Main logging owner.

`MainStage` should own:

- ordered measurement-phase registration
- active phase index
- active primitive index
- active speed index
- active repeat index
- stage-owned hold state
- `phase_id`, `primitive_id`, `speed_bin`, and `repeat_index` for the current Main row
- Main stream open/setup
- Main metadata writing
- Main schema begin
- Main-row population from runtime state plus infrastructure-owned active-slot metadata
- Main-row staging
- Main-row flush
- infrastructure faults such as selector removal, estimator faults, and log write failures
- generic advancement across the battery

## MainStage Iteration Model

The Main battery should be driven by infrastructure-owned iteration, not by phase-authored scheduled work objects.

The core model should be:

1. `MainStage` owns the ordered list of measurement phases.
2. Each phase reports the iteration dimensions it wants the infrastructure to run.
3. `MainStage` owns the active cursor across those dimensions.
4. `MainStage` owns when to advance primitive, speed, repeat, phase, and holds.
5. The active phase is only told which primitive/speed selection is active and is then ticked until complete.

The important point is ownership:

- the phase may report counts and label mappings
- the infrastructure owns sequencing

That means the phase may define things equivalent to:

- how many primitive variants it has
- how many speed variants it has
- how many repetitions it wants
- how primitive indices map to external `primitive_id`
- how speed indices map to external `speed_bin`

But the phase must not own:

- the active repeat counter
- row-label sequencing state
- advancement policy
- hold insertion policy

If some phase needs only one primitive or one speed, the infrastructure dimension is just size `1`.

If some phase has a more complex internal command sequence, that complexity stays inside that one phase class after the infrastructure selects the current primitive/speed slot.

## Measurement Phase Contract

Each Main-stage measurement phase should be one proper private class.

After setup/compilation succeeds, a measurement phase must be total during execution.

That means the phase should not have:

- a runtime failure channel
- fault text
- stop-policy ownership
- mutable row-label sequencing state
- repeat-counter ownership
- hold policy ownership

It should only need to do work equivalent to:

- optional phase-local setup or precomputation
- report its iteration dimensions and label mappings to `MainStage`
- begin the currently selected primitive/speed slot
- generate commands each tick for that active slot
- report completion

If a phase cannot justify an independent behaviorful class, it should not survive as a named type.

## Holds Are Infrastructure

Holds are not measurement phases.

Holds are not phase-owned settle logic.

Holds are infrastructure-owned pauses between measurement slots or between phases.

That means:

- inter-slot holds belong to `MainStage`
- inter-phase holds belong to `MainStage`
- settle behavior must not be implemented by stuffing hold logic into each phase class

## Row Labeling and Main Log Contract

The Main row format is an external contract and should stay fixed.

The important correction is ownership:

- `phase_id`
- `primitive_id`
- `speed_bin`
- `repeat_index`

belong to `MainStage` infrastructure, not to the measurement phase as mutable runtime state.

The phase may supply stable label mappings and dimension sizes, but `MainStage` owns:

- active row-label coordinates
- repeat expansion
- sequencing boundaries
- row stamping

The Main row for a tick must still mean:

- the command active during that tick
- the active phase/primitive/speed/repeat coordinates associated with that tick
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

If something can fail before execution, that is a setup/controller concern.

If something can fail during execution, that is a stage/infrastructure concern.

## Phase-Local Compiled Artifacts

If a measurement phase needs compile-time artifacts, those artifacts belong inside that phase.

Examples:

- smooth maneuver queues
- loop maneuver queues
- any compiled maneuver storage needed only by one phase

Those artifacts should not be promoted into `MainStage` just because they must survive across ticks.

`MainStage` should know only:

- which phase is active
- which primitive index is active
- which speed index is active
- which repeat index is active
- whether it is currently in a hold

It should not know phase internals.

## Terminology

Internal `section` terminology is drift and should not be used as the controller's architectural vocabulary.

Use:

- `phase`

internally.

Only preserve legacy `section` vocabulary at an external boundary if some existing non-controller contract truly still requires it.

Do not let internal phase identity be built by casting or aliasing `OpenFloorSectionId`.

## Rejected Shapes

The following shapes are explicitly rejected:

- central execution-family `variant` dispatch
- central `Plan` / `ExecutionState` type families
- one `TickXxxExecution(...)` method per family
- phase-owned row identity runtime objects
- per-phase fault text
- phase-local settle-hold policy
- explicit work-item, segment, or case-registration frameworks that recreate the same scaffolding under new names
- data-only structs that exist only to carry execution internals
- new public wrappers, facades, helpers, managers, or compatibility shims

Private visibility does not make those shapes acceptable.

## Effective Stop Condition

This rewrite is done when all of the following are true:

1. `OpenFloorMeasurementController` is the real owner, rather than a shell over a hidden central dispatcher.
2. `TimingStage` remains stage-owned timing behavior outside the Main battery.
3. `MainStage` clearly owns sequencing, iteration state, row labeling, holds, staging/flush, and infrastructure faults.
4. Main-stage execution is driven by generic infrastructure iteration over an ordered phase list.
5. Measurement phases are proper private classes whose runtime behavior is only command generation plus completion.
6. No central closed-set execution-family dispatcher remains.
7. No per-phase fault text or runtime failure channel remains.
8. No phase-owned row-label sequencing state remains.
9. The current timing-to-main split and handoff semantics remain correct.
10. The Main row format and row meaning remain correct.
11. Adding a new Main-log-compatible measurement phase requires:
    - one new phase class
    - one registration reference in the primary phase list
12. Adding that new phase does not require editing:
    - a central `variant`
    - a central dispatcher
    - a row-identity type
    - a fault-text method
    - a case-registration framework
    - a family-specific execution lattice

## Migration Guidance

The clean migration direction is:

- keep the `TimingStage` / `MainStage` split
- delete the segment/family architecture rather than renaming it
- move sequencing ownership harder into `MainStage`
- replace phase-authored scheduled work with infrastructure-owned iteration
- move phase-local compiled data into the corresponding phase classes
- delete phase-owned row-label runtime state
- delete phase fault semantics
- move hold policy into infrastructure

The current compile loops are already the right ownership area for sequencing, but the wrong abstraction for execution.

Use that fact to converge toward:

- stage-owned iteration
- phase-owned command generation

not toward another closed-set central family system under cleaner names.

## What This Rewrite Should Include

This rewrite should include:

- replacing the current segment/family architecture with the phase-list-plus-iteration architecture described here
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
- new scaffolding layers that make adding a phase harder than adding one class plus one registration reference

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

- a new phase can be added as one class plus one registration reference
- no rows are dropped or duplicated across delayed flush boundaries
- hold insertion remains infrastructure-owned and occurs at the intended boundaries
- row-label sequencing still advances only at intended boundaries
- phase-local compiled artifacts stay local and do not leak back into `MainStage`

## Summary

The present recommendation is:

1. Keep `OpenFloorMeasurementController` as the sole public owner.
2. Keep the `TimingStage` / `MainStage` split.
3. Make `MainStage` the owner of sequencing, iteration, row labeling, holds, Main-row staging/flush, and infrastructure faults.
4. Make each measurement phase one proper private class.
5. Let phases report dimensions and label mappings, then generate commands for the currently selected primitive/speed slot until complete.
6. Remove central execution-family variants and dispatch lattices.
7. Remove phase-owned row identity, phase fault semantics, and phase-owned hold logic.
8. Keep timing outside the Main battery.
9. Preserve the Main log format and row meaning.
10. Treat arbitrary measurement-phase support as the acceptance threshold.
