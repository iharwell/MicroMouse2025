# Feedforward Handoff 2026-04-21 v3

This handoff captures the state after fixing the direct `solveDriveCommands(...)` inverse mismatch that was the main blocker in the prior handoff.

## User Goal

- Keep `PlantModel` as the canonical owner.
- Build a non-iterative / algebraic feedforward system that matches the actual prediction path within about 10%.
- Avoid faulting out of feedforward solve paths.
- Preserve host and Teensy verification paths.
- Keep touched tests failing in one obvious way per path.

## AGENTS.md Requirements Reconfirmed

I reread `AGENTS.md` before continuing this work.

The constraints that mattered most here were:

- `PlantModel` remains the authoritative owner.
- Preserve buildability and host/Teensy verification paths.
- Do not introduce wrappers or parallel owners.
- Before testing, verify current binaries are actually newer than the edited sources.

At the next compaction or takeover, reread `AGENTS.md` again before proceeding.

## What Changed In This Continuation

### 1. Fixed the reduced operating-state bug in the canonical owner

The reduced `solveDriveCommands(...)` / velocity-target overloads were constructing a state with:

- `kU = forwardVelocityMps`
- `kR = yawRateRadps`
- `kOmegaL = 0`
- `kOmegaR = 0`

Because those zero wheel speeds were finite, `BuildDriveCommandOperatingState(...)` treated them as observed stopped-wheel state instead of falling back to rolling-kinematic wheel speeds.

That meant the inverse was solving against a bogus operating point:

- body already moving
- wheel state effectively stopped

The reduced helper now marks the wheel speeds unknown so `BuildDriveCommandOperatingState(...)` uses its rolling fallback instead.

Current fix location:

- `MazeMap/MazeMap/PlantModel.cpp`
- `BuildReducedDriveCommandOperatingState(...)`

### 2. The direct inverse mismatch against `forwardStep(...)` is now fixed

The two highest-signal direct blocker tests from the prior handoff now pass:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`

Interpretation:

- the reduced inverse path is no longer solving against the bogus stopped-wheel baseline
- the returned operating point is again self-consistent with `forwardStep(...)` for those cases

### 3. The remaining failure shape has moved downstream

The remaining plant/feedforward problems are no longer best explained by the old direct inverse mismatch.

The strongest remaining signal is now:

- immediate operating-point checks can be correct
- but repeated predict/integrate behavior still drifts

Most useful current example:

- `PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict`
- failure text:
  - `predict steady-state hold failed predicted_u/r=0.143630,1.867198 immediate_u_dd/r_dd=0.000006,0.000000 ...`

That means:

- the immediate operating point is effectively correct
- the repeated prediction loop is still not holding that operating point

### 4. The next low-level blocker is now near-zero lateral / yaw stability

`PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest` is still failing.

That matters because it lines up with the new downstream symptom pattern:

- repeated predict drift
- steady-speed hold drift
- velocity-target feedforward drift

So the next debugging target should be the low-speed / near-zero lateral-yaw plant stability path rather than another rewrite of the direct inverse.

## Current Code State

### `MazeMap/MazeMap/PlantModel.cpp`

Important current behavior:

- `BuildReducedDriveCommandOperatingState(...)` now marks reduced-overload wheel speeds unknown instead of implicitly forcing stopped-wheel state
- reduced `solveDriveCommands(...)` calls now solve against the proper rolling operating point fallback
- the earlier direct inverse vs `forwardStep(...)` mismatch is no longer the main blocker

### `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`

Important current behavior:

- the touched direct operating-point tests still use `forwardStep(...)`
- the touched tests still fail with one explicit meaning per test path
- the direct blocker tests now pass on the fresh host Release run from this thread

## What I Verified

### Last completed authoritative verify from this thread

Use:

- `codex_verify/logs/verify_latest_build_20260421_044025_023.txt`

Result:

- `592` total
- `529` passed
- `63` failed

This is the last completed `test_latest_binaries.cmd` verify I explicitly ran in this thread against fresh host Release binaries after the reduced-state fix.

### Important caution about newer results

The user later said:

- things should be building again
- no build has happened yet in the new cycle
- tests are still running

So the next agent should **not** assume the above count is the newest available result.

Before making the next substantive change:

1. inspect `codex_verify/logs` for anything newer than this handoff,
2. confirm whether the in-progress test run completed,
3. only then decide which log is authoritative.

## Important Verify Results

### Direct inverse blockers fixed

These prior blockers are now resolved in the last completed host verify from this thread:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`

### Steady-state hold still fails in repeated predict

Most useful current failure:

- `PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict`
- failure text includes:
  - `predicted_u/r=0.143630,1.867198`
  - `immediate_u_dd/r_dd=0.000006,0.000000`

Interpretation:

- the returned operating point is nearly steady in the instantaneous plant derivative
- but the repeated predict/update path still spins up yaw and loses forward speed

### Near-zero lateral perturbation stability still broken

- `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
- latest failure still reports:
  - `Expected:<0> Actual:<0.302125>`

### Velocity-target feedforward remains downstream-broken

The exact horizon values may change once the currently running test pass completes, but the current state is:

- open-loop and traction-limited velocity-target cases still fail badly
- `FeedforwardAgreesWithPredict` still fails
- `SrUkfCoreControlDirectionsCorrect` and `...AfterStationary` still fail on excessive lateral velocity

Interpretation:

- the downstream feedforward layer is still wrong
- but the next rational fix target is the low-speed / repeated-predict plant stability path, not the already-fixed direct inverse mismatch

## Current Hypothesis

The next real blocker is likely in the low-speed transition / repeated-predict path, not in direct inverse algebra.

Most likely places to inspect next:

1. `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
2. `AdvancePlantPredictionState(...)`
3. `PlantModel::integrate(...)`
4. low-speed stop/snap gating in `PlantModel.cpp`
5. `motionWeight` / lateral-scrub handling near zero lateral velocity and small yaw

This is still a hypothesis, not yet a proven root cause.

## Best Next Step

Do not go back to the old direct inverse investigation first.

Instead:

1. Wait for the currently running tests to finish.
2. Inspect any newer `codex_verify/logs/*verify*` or `*build_and_verify*` output.
3. Reconfirm whether the direct blocker tests still pass in the newest completed run.
4. If they do, debug:
   - `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
   - `PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict`
5. Only after repeated-predict stability is corrected should the agent retune the broader velocity-target feedforward behavior.

## References

- current direct-fix code:
  - `MazeMap/MazeMap/PlantModel.cpp`
- touched drive-command tests:
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- low-speed plant stability test:
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
- prediction helpers:
  - `MazeMap/MazeMapTest/PlantModelTestSupport.h`
  - `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h`
- last completed verify I explicitly ran:
  - `codex_verify/logs/verify_latest_build_20260421_044025_023.txt`

## Incoming User Data

The user explicitly said:

- things should be building again
- no building has happened yet in the current cycle
- the tests are still running

That means the next agent should check for fresher results before trusting any count in this handoff as the newest available state.
