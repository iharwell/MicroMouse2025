# OpenFloorMeasurementController Main-Stage Regime Migration Plan

## Goal

Move the remaining `OpenFloorMeasurementController` main-stage regimes onto the narrowed `MainStage` architecture without reintroducing hidden traversal loops, shadow repeat state, or regime-local scheduler machinery.

## Architectural rules

- `MainStage` owns traversal of regime, speed, primitive, and repeat axes.
- Add `MainMeasurementRegime::GroupPrimitives()` with default `false`.
- Default traversal order is:
  - repeat
  - primitive
  - speed
  - regime
- Grouped traversal order is:
  - primitive
  - repeat
  - speed
  - regime
- Grouping applies only within the active speed bin.
- Regimes must not hide private tracker loops inside `Tick(...)`.
- Regimes should use `Drive::IsEffectivelyComplete()` as the entry/completion boundary whenever possible.
- Regimes still ask `Drive` for controls every tick; some completion-edge ticks only add a start call before that.
- Regimes should propagate real completion from the final `Drive`-owned state by returning `GetNextControls(done)` on the terminal path.
- Regime-specific constants belong in the regime, not in shared config/spec owners, unless they are already shared for an intentional reason.

## Regime targets

### Yaw

- Make `YawMeasurementRegime` match the `StraightMeasurementRegime` structure:
  - if `IsEffectivelyComplete()`, arm either the turn or the closing hold
  - otherwise return `GetNextControls(done)`
- Keep the post-segment hold as part of repetition completion.
- Remove unnecessary shadow state if possible.

### Smooth

- Keep `MainStage` axes fully visible in the logs.
- Do not flatten traversal into regime-private loops.
- Keep the regime small and `Drive`-centric:
  - if `IsEffectivelyComplete()`, arm the current primitive or the final regime-exit hold
  - otherwise return `GetNextControls(done)`
- Store the opening straight as `S2`, and only log/build it as `S1` on the first slot by subtracting `((speedIndex == 0) && (primitiveIndex == 0))`.
- Preserve explicit logged primitives, including the connector straight between speed bins.
- No holds between smooth primitives or between grouped repeats.
- Only hold before progression to the next regime.

### Loop

- Override `GroupPrimitives()` to return `true`.
- Remove shadow repeat tracking.
- Let one repeat mean one full grouped circuit at the current speed bin.
- Keep the regime in the same minimalist `Drive`-completion shape as smooth.
- No holds between primitives or between grouped repeats.
- Only hold before progression to the next regime.

### Launch / Yaw Launch

- Not primary edit targets.
- Preserve their minimalist shape.
- Accept only small cleanup that reduces indirection or uses shared hold constants where appropriate.

## MainStage changes

- Extend the regime interface with `GroupPrimitives()`.
- Update `AdvanceIndices()` so grouped regimes traverse primitives before repeats.
- Keep `MainStage` as the only owner of traversal semantics.

## Sanity checks while editing

- A regime should usually not need more than one tiny piece of local state.
- If a regime grows explicit entered/holding/repeat/selection flags, re-check whether `IsEffectivelyComplete()` already solves the problem.
- Confirm terminal completion comes from the final `Drive` call using the real `done` output.
- Preserve the invariant that the next regime enters with `IsEffectivelyComplete() == true`.

## Verification plan

- Review the resulting `Tick(...)` shapes for yaw, smooth, and loop against the yaw-launch/straight pattern.
- Build the affected project/test targets in Release.
- Run the relevant unit-test path in Release after confirming the binaries are built from the latest source.
