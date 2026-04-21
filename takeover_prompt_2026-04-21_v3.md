# Takeover Prompt 2026-04-21 v3

You are taking over the feedforward investigation in `C:\Users\thene\source\repos\MicroMouse2025`.

Before doing anything else:

1. Read `AGENTS.md` now.
2. Read it again after every context compaction.

Then read these handoff files in this order:

- `feedforward_handoff_2026-04-21.md`
- `agent_handoffs_2026-04-21.md`
- `feedforward_handoff_2026-04-21_v2.md`
- `agent_handoffs_2026-04-21_v2.md`
- `feedforward_handoff_2026-04-21_v3.md`
- `agent_handoffs_2026-04-21_v3.md`

## User Goal

Build a non-iterative / algebraic feedforward system that matches the actual prediction behavior within about 10%, while remaining fault-free and returning reasonable results.

The user also explicitly wants:

- tests that fail in exactly one obvious way per test path
- no confusing multi-assert ambiguity when a test fails
- care around `forwardStep(...)` semantics versus repeated predict/update semantics

## Current State

The previous direct blocker has been fixed.

The reduced-overload `solveDriveCommands(...)` path was previously constructing a fake stopped-wheel operating point because:

- reduced overloads populated finite zero wheel speeds
- `BuildDriveCommandOperatingState(...)` treated those as observed state
- fallback rolling wheel speeds were therefore not used

That is now fixed in:

- `MazeMap/MazeMap/PlantModel.cpp`
- `BuildReducedDriveCommandOperatingState(...)`

## Important Facts Already Established

- `PlantModel` remains the authoritative owner.
- Do not restore the old heuristic yaw-bias / trim path as the main solution.
- The touched direct operating-point tests should remain on `forwardStep(...)`.
- The touched tests should keep one explicit failure path/message.
- The old direct inverse vs `forwardStep(...)` mismatch is no longer the main blocker.

These direct blocker tests now pass in the fixed host Release state:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`

## Last Completed Verify Explicitly Run In The Prior Thread

Use:

- `codex_verify/logs/verify_latest_build_20260421_044025_023.txt`

Result:

- `592` tests
- `529` passed
- `63` failed

Treat that as the last completed explicit verify from the prior thread, not automatically as the newest final result.

## Important User Note About Current Runtime State

After that explicit verify, the user said:

- things should be building again
- no build has happened yet in the new cycle
- the tests are still running

Before making the next substantive code change:

1. inspect `codex_verify/logs` for anything newer than the handoff verify,
2. confirm whether the currently running tests completed,
3. only then decide which log is authoritative.

## Most Important Remaining Failures

### 1. Repeated-predict steady-state hold still drifts badly

Most useful current failure:

- `PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict`

Most useful message fragment:

- `predicted_u/r=0.143630,1.867198`
- `immediate_u_dd/r_dd=0.000006,0.000000`

Interpretation:

- the immediate operating point is nearly correct
- the repeated prediction loop still diverges from that operating point

### 2. Near-zero lateral / yaw perturbation stability is still broken

- `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
- latest known message:
  - `Expected:<0> Actual:<0.302125>`

This is currently the best low-level plant stability signal.

### 3. Downstream velocity-target and predict-matching tests remain broken

Examples:

- `PlantModelSolveDriveCommandsForVelocityTarget...`
- `FeedforwardAgreesWithPredict`
- `SrUkfCoreControlDirectionsCorrect`
- `SrUkfCoreControlDirectionsCorrectAfterStationary`

Treat these as downstream of the remaining repeated-predict / low-speed plant stability issue until proven otherwise.

## Code Areas To Inspect Next

- `MazeMap/MazeMap/PlantModel.cpp`
  - `BuildReducedDriveCommandOperatingState(...)`
  - `PlantModel::forwardStep(...)`
  - `PlantModel::integrate(...)`
  - `ShouldSnapToZeroFast(...)`
  - `ShouldReportStoppedDiagnosticsFast(...)`
  - low-speed `motionWeight` / lateral-scrub handling
- `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
  - `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
  - steady-state hold tests
  - velocity-target tests
- `MazeMap/MazeMapTest/PlantModelTestSupport.h`
  - `AdvancePlantPredictionState(...)`
- `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h`
  - `RunPredictionMatchingCycle(...)`

## Current Best Next Step

Do not go back to the old direct inverse mismatch as the first target.

The highest-signal next work is:

1. wait for the current tests to finish,
2. inspect the newest completed logs,
3. confirm that the direct blocker tests remain fixed,
4. debug the near-zero lateral/yaw plant stability path,
5. then re-evaluate the steady-state hold and velocity-target feedforward failures

## Current Dirty-Tree Caution

Do not casually revert unrelated user or other-thread work in:

- `MazeMap/MazeMap/BootModeDescriptor.h`
- `MazeMap/MazeMap/BootModeRegistry.cpp`
- `MazeMap/MazeMap/MazeMap.vcxproj`
- `MazeMap/MazeMap/MazeMap.vcxproj.filters`
- `MazeMap/MazeMap/MazeMapControllerRegistry.h`
- `showcasing_donut_controller_spec.md`
- `staging/`

Also note deleted unrelated files were present:

- `MazeMap/MazeMap/ShowcasingDonutController.cpp`
- `MazeMap/MazeMap/ShowcasingDonutController.h`
