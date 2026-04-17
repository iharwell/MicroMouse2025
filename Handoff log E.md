# Handoff Log E

Date: 2026-04-17

## Work Completed

Inspected the current `ManeuverExecutor` surface and the three maze-running mode files to determine which remaining maneuver-execution wrappers or scaffolding should move into `ManeuverExecutor` next.

Scope was limited to source inspection and dirty-tree audit for:

- `MazeMap/MazeMap/ManeuverExecutor.h`
- `MazeMap/MazeMap/ManeuverExecutor.cpp`
- `MazeMap/MazeMap/MissionRunMode.h`
- `MazeMap/MazeMap/MissionRunMode.cpp`
- `MazeMap/MazeMap/CorridorRepeatabilityMode.h`
- `MazeMap/MazeMap/CorridorRepeatabilityMode.cpp`
- `MazeMap/MazeMap/PositionAccuracyAuditMode.h`
- `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp`
- `MazeMap/MazeMap/MissionModeController.h`
- `MazeMap/MazeMap/MissionModeController.cpp`
- `MazeMap/MazeMap/MazeRunningAuditController.h`
- `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- `MazeMap/MazeMap/ManeuverFileTestMode.cpp`

No files were edited in this inspection pass. No build or tests were run in this pass.

## Main Finding

The three maze-running mode files are already thin wrappers and should not be the next place to move maneuver logic from:

- `MissionRunMode.cpp`
- `CorridorRepeatabilityMode.cpp`
- `PositionAccuracyAuditMode.cpp`

All three currently just delegate to controller owners. The remaining maneuver-execution wrapper debt is concentrated in:

- `MissionModeController.cpp`
- `MazeRunningAuditController.cpp`

## What Already Exists In ManeuverExecutor

`ManeuverExecutor` already has routine-form public entry points for the shared maneuver execution primitives:

- `BeginHoldRoutine(...)`
- `BeginBrakedSettleRoutine(...)`
- `BeginReverseStraightRoutine(...)`
- `BeginStraightRoutine(...)`
- `BeginTurnRoutine(...)`
- `BeginArcRoutine(...)`
- `BeginSmoothTurnRoutine(...)`
- `ProceedToManeuverExecutionRoutine(...)`

It also already owns:

- continuation installation
- active routine state
- phase-to-phase transition inside the executor
- queue dispatch / queue advance for maneuver queues
- queue begin / queue complete hooks

That means the next convergence cut is not to add more controller-local scaffolding. It is to migrate remaining controller call sites off the older public `Begin*Phase(...)` path and onto the routine-form API that already exists.

## Remaining Wrapper Layer To Remove Next

The next maneuver-execution wrappers to eliminate are the one-for-one blocking shims still present in both controllers:

- `HoldPosition(...)`
- `HoldBrakedUntilDriveSettles(...)`
- `ExecuteReverseStraightProfile(...)`
- `ExecuteStraightProfile(...)`
- `ExecuteTurnProfile(...)`
- `ExecuteArcProfile(...)`
- `ExecuteSmoothTurnProfile(...)`

Current pattern in both controllers:

1. begin a public executor `Phase`
2. call `RunSharedManeuverExecutorPhase()`
3. optionally bolt on controller-local continuation glue

That is exactly the wrapper/scaffolding layer that should disappear next.

## Concrete API Recommendations

1. Migrate controller call sites from public `Begin*Phase(...)` methods to the existing routine-form APIs:
   - `BeginHoldRoutine(...)`
   - `BeginBrakedSettleRoutine(...)`
   - `BeginReverseStraightRoutine(...)`
   - `BeginStraightRoutine(...)`
   - `BeginTurnRoutine(...)`
   - `BeginArcRoutine(...)`
   - `BeginSmoothTurnRoutine(...)`
   - `ProceedToManeuverExecutionRoutine(...)`

2. After callers are moved, make the older public `Begin*Phase(...)` methods private implementation detail inside `ManeuverExecutor`.

3. Delete controller-local `RunSharedManeuverExecutorPhase()` as each call path is migrated. It is nested-session scaffolding and conflicts with the convergence note's direction.

4. Keep the caller-owned continuation model used in `ManeuverFileTestMode.cpp` as the reference shape for the remaining mode/controller migrations.

5. For queued maneuvers, keep using `ProceedToManeuverExecutionRoutine(...)` as the authoritative shared queue owner. Do not reintroduce queue-launch wrappers elsewhere.

6. If a small API cleanup is desired to remove more wrappers cleanly, make `BeginStraightRoutine(...)` and `BeginReverseStraightRoutine(...)` treat nonpositive distance as immediate continuation success so controller-side "distance <= 0 means return true" wrappers can disappear.

## Call-Site Recommendations

Recommended next call-site replacements inside `MissionModeController.cpp` and `MazeRunningAuditController.cpp`:

- `HoldPosition(...)`
  - stop calling `BeginHoldPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginHoldRoutine(...)` with a caller-owned continuation instead

- `HoldBrakedUntilDriveSettles(...)`
  - stop calling `BeginBrakedSettlePhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginBrakedSettleRoutine(...)`

- `ExecuteReverseStraightProfile(...)`
  - stop calling `BeginReverseStraightPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginReverseStraightRoutine(...)`

- `ExecuteStraightProfile(...)`
  - stop calling `BeginStraightPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginStraightRoutine(...)`

- `ExecuteTurnProfile(...)`
  - stop calling `BeginTurnPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginTurnRoutine(...)`

- `ExecuteArcProfile(...)`
  - stop calling `BeginArcPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginArcRoutine(...)`

- `ExecuteSmoothTurnProfile(...)`
  - stop calling `BeginSmoothTurnPhase(...)` + `RunSharedManeuverExecutorPhase()`
  - call `BeginSmoothTurnRoutine(...)`

For queued maneuvers, the remaining controller-local scaffolding is:

- queued-maneuver launch
- optional final hold
- completion tick that just ends the outer loop session

That should shrink so the caller owns only its policy-specific continuation and any optional final hold, while `ManeuverExecutor` remains the shared routine owner of queue execution itself.

## What Should Not Move Into ManeuverExecutor

These do not look like executor ownership and should not be the next things moved there:

- `SearchStraightLoopState`
- `WallTouchLoopState`
- `FrontCalibrationSweepLoopState`
- telemetry phase naming
- controller-specific logging helpers
- `BuildManeuverExecutorHooks(...)`

Those are routine- or mode-specific concerns, not shared maneuver-execution ownership.

## Dirty-Tree Hazards

Current hazards noted during the inspection:

1. `MissionModeController` still exposes utility-mode entry points while `MazeRunningAuditController` also owns the utility workflows. That leaves parallel controller ownership.
2. `MazeRunningAuditController.cpp` and `.h` are currently untracked local files but are already wired into `MazeMap.vcxproj` and `.filters`.
3. `MissionModeController.cpp` and `MazeRunningAuditController.cpp` still contain near-duplicate maneuver wrapper code, so changing one without the other will increase divergence immediately.
4. `ManeuverExecutor` currently exposes both public `Phase` and public `Routine` APIs for the same responsibilities. Expanding that overlap further would be the wrong direction.

## Practical Next Step

The next convergent maneuver-execution cut should be:

1. finish migrating controller call sites onto the routine-form `ManeuverExecutor` API
2. delete `RunSharedManeuverExecutorPhase()` usage as those paths disappear
3. collapse public `Phase` entry points back to executor-private implementation detail
4. only after that, continue with separate routine extraction for wall touch, search straight, and mapping
