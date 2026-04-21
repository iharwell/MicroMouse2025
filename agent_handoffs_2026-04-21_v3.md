# Agent Handoffs 2026-04-21 v3

This file records the effective handoff state for a fresh thread after the direct reduced-overload inverse fix.

## New Agent Activity In This Continuation

Two subagents were used during this continuation.

### James

- independently confirmed that the direct `PlantModel` inverse mismatch is no longer the main blocker
- pointed to the reduced operating-state wheel-speed fallback as the real root cause
- confirmed the direct blocker tests pass on the fixed host Release run
- concluded the remaining failures have shifted downstream to repeated predict / velocity-target behavior

### Poincare

- was tasked with inspecting the newest runtime/test artifacts
- did not return a final result before closeout

## Findings That Still Hold From Earlier Agent Work

- `PlantModel` remains the correct owner.
- The old heuristic yaw-assist / yaw-bias / trim path should not be restored as the main solution.
- The touched direct `solveDriveCommands(...)` operating-point tests should stay on `forwardStep(...)`, not UKF predict/update semantics.
- The direct operating-point tests should keep one explicit failure path/message.

## Current Thread Findings

### 1. The reduced overload bug was the real direct blocker

The reduced `solveDriveCommands(...)` path was constructing an operating state with finite zero wheel speeds.

That caused `BuildDriveCommandOperatingState(...)` to treat the wheel state as:

- observed
- stopped

instead of falling back to rolling wheel speeds implied by the body state.

That bogus operating point was enough to poison the direct inverse solve.

### 2. The direct `solveDriveCommands(...)` vs `forwardStep(...)` mismatch is fixed

These previously important failures now pass in the fixed host Release state:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`

### 3. The latest completed verify from this thread improved by one pass

Last completed explicit wrapper verify from this thread:

- `codex_verify/logs/verify_latest_build_20260421_044025_023.txt`

Result:

- `529 passed / 63 failed`

This is better than the earlier:

- `528 passed / 64 failed`

### 4. The current blocker is now repeated-predict stability, not the old inverse mismatch

Most useful remaining signal:

- `PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict`
- failure text includes:
  - `predicted_u/r=0.143630,1.867198`
  - `immediate_u_dd/r_dd=0.000006,0.000000`

Interpretation:

- the immediate operating point is basically correct
- repeated prediction still drifts badly

### 5. The low-speed / near-zero lateral stability path is still broken

Still important:

- `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
- latest failure still reports:
  - `Expected:<0> Actual:<0.302125>`

That is now the strongest low-level plant signal for the next agent.

### 6. User state changed after the last explicit verify

The user later said:

- things should be building again
- no building has happened yet in the new cycle
- the tests are still running

So the next agent must check for fresher logs before assuming this handoff contains the newest final count.

## Best Next Move For A Fresh Agent

Do not restart from the old direct inverse theory first.

The next agent should:

1. reread `AGENTS.md`,
2. read the new handoff files,
3. inspect `codex_verify/logs` for anything newer than `verify_latest_build_20260421_044025_023.txt`,
4. confirm whether the in-progress user test run completed,
5. if the direct blocker tests still pass, shift focus to:
   - `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
   - steady-state hold in predict
   - downstream velocity-target drift

## Current Key References

- last completed explicit verify from this thread:
  - `codex_verify/logs/verify_latest_build_20260421_044025_023.txt`
- current main code:
  - `MazeMap/MazeMap/PlantModel.cpp`
- touched tests:
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- low-speed plant stability test:
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
- prediction helpers:
  - `MazeMap/MazeMapTest/PlantModelTestSupport.h`
  - `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h`

## Current Dirty-Tree / User-Work Caution

When this handoff was prepared, unrelated user or other-thread work was present in:

- `MazeMap/MazeMap/BootModeDescriptor.h`
- `MazeMap/MazeMap/BootModeRegistry.cpp`
- `MazeMap/MazeMap/MazeMap.vcxproj`
- `MazeMap/MazeMap/MazeMap.vcxproj.filters`
- `MazeMap/MazeMap/MazeMapControllerRegistry.h`
- `showcasing_donut_controller_spec.md`
- `staging/`

Deleted unrelated files were also present:

- `MazeMap/MazeMap/ShowcasingDonutController.cpp`
- `MazeMap/MazeMap/ShowcasingDonutController.h`

Do not casually revert that work while continuing the plant/feedforward investigation.
