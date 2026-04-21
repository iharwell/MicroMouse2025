# Feedforward Handoff 2026-04-21 v2

This handoff captures the state after continuing the original 2026-04-21 feedforward investigation.

## User Goal

- Keep `PlantModel` as the canonical owner.
- Produce a non-iterative / algebraic feedforward path that matches actual prediction behavior within about 10%.
- Avoid faulting out of feedforward solve paths.
- Keep host and Teensy verification paths intact.
- Ensure test failures are immediately meaningful, with exactly one obvious failure path per test.

## AGENTS.md Requirements Reconfirmed

I reread `AGENTS.md` while continuing this work.

The constraints that mattered most in this continuation were:

- `PlantModel` remains the authoritative owner
- preserve buildability and host/Teensy verification paths
- do not introduce wrappers or parallel owners
- before testing, verify current binaries are actually newer than edited sources

At the next compaction or takeover, reread `AGENTS.md` again before proceeding.

## What Changed In This Continuation

### 1. Reworked the touched `PlantModelDriveCommandTest.cpp` cases back to correct semantics

The previous thread temporarily converted several direct `solveDriveCommands(...)` operating-point tests to a 5-step `integrate(...)` helper.

That was wrong for those tests.

Those specific tests are validating instantaneous reported operating-point telemetry, so they now use `forwardStep(...)` again instead of multi-step prediction.

I also changed those touched tests to one explicit failure path per test:

- they now build a single `failure` message string
- they call `Assert::Fail(...)` once if any condition is violated
- failure text now directly states what the mismatch means

This was applied to the touched direct operating-point cases including:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
- `PlantModelSolveDriveCommandsForVelocityTargetReportsReturnedControlPredictionAtOperatingPoint`
- `PlantModelSolveDriveCommandsValidationUsesFullCurrentState`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`
- `PlantModelSolveDriveCommandsBeyondNominalReturnsClippedFiniteSolution`
- `PlantModelSolveDriveCommandsBeyondPeakRemainsStable`

### 2. Rejected the body-only exact-solver experiment

The previous handoff suggested trying a body-only exact solve by replacing the two

- `ResolveModeAffineNetDriveTorqueNm(...)`

calls in `ResolveVelocityTargetExactControl(...)` with

- `ResolveModeNetDriveTorqueNm(...)`

That experiment had already been applied before this continuation.

Host verification showed that it materially worsened the velocity-target yaw overshoot.

During this continuation I restored the original wheel-terminal constrained exact solve:

- target wheel-rate terms are back
- the exact branch again uses `ResolveModeAffineNetDriveTorqueNm(...)`

### 3. Fixed exact-candidate selection so it is no longer accepted merely because it is finite

I added:

- `ShouldPreferExactVelocityTargetCandidate(...)`

Current intent:

- exact candidate is only returned if its response metric is finite and better than the base solution’s response metric
- exact candidate is no longer returned just because the metric is finite

This is important because the previous logic could still return an exact candidate even when it was not actually the better solution.

### 4. Synced manual-build output into the verify-wrapper path

The user manually built the host binaries because the full wrapper is slower.

In the latest cycle:

- fresh `MazeMap.dll` landed in
  `MazeMap/MazeMap/x64/Release/MazeMap.dll`
- verify wrapper uses
  `MazeMap/x64/Release/MazeMap.dll`

I copied the fresh host `MazeMap.dll` and matching PDB into the wrapper-visible path before running the verify wrapper.

`MazeMapTest.dll` was already being emitted into the wrapper-visible path by the test project build.

## Current Code State

### `MazeMap/MazeMap/PlantModel.cpp`

Current working-tree changes still present include:

- `UpdateVelocityTargetSolutionPrediction(...)`
- `ResolveVelocityTargetResponseTolerance(...)`
- `ComputeVelocityTargetResponseErrorMetric(...)`
- `ShouldPreferExactVelocityTargetCandidate(...)`
- expanded `VelocityTargetExactSolution` telemetry/error fields
- exact-candidate selection wiring in `SolveVelocityTargetFeedforward(...)`
- current public velocity-target state/reduced overloads still prefer the exact algebraic branch when not traction-limited

Important current behavior:

- the harmful body-only exact-solver experiment is no longer active
- exact-candidate selection is now gated on improving the response metric

### `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`

Important current behavior:

- the touched direct operating-point tests are back on `forwardStep(...)`
- the touched tests now fail with one explicit meaning per test

## What I Verified

### Binary freshness and wrapper path issue

I verified that the user’s manual host build produced fresh binaries, but not all of them landed where the verify wrapper expects.

Before the final verify run:

- `PlantModel.cpp` was newer than `MazeMap/x64/Release/MazeMap.dll`
- fresh `MazeMap.dll` existed at `MazeMap/MazeMap/x64/Release/MazeMap.dll`
- I synced that DLL and PDB into `MazeMap/x64/Release`

### Current authoritative verify run

Use:

- `codex_verify/logs/verify_latest_build_20260421_040112_931.txt`

This is the best current host-verification snapshot for the present working tree.

Result:

- `592` total
- `528` passed
- `64` failed

The wrapper reported:

- host verification ready
- host tests ran
- firmware image stale relative to the latest source edit

That firmware warning is expected because the user only rebuilt the host side.

## Important Verify Results

### Direct inverse still disagrees with `forwardStep(...)`

These are now the clearest failures:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
  - `forwardStep yaw accel mismatch reported/achieved=4.500000,-93.903145`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`
  - `forwardStep yaw accel mismatch reported/achieved=18.000002,-102.752533`

Interpretation:

- `solveDriveCommands(...)` still reports a positive yaw acceleration target
- but the returned operating point, when evaluated through `forwardStep(...)`, produces a large negative yaw acceleration

This is the most important unresolved plant inconsistency.

### Velocity-target feedforward is back to the pre-body-only exact state

That is actually useful because it confirms the bad experiment is gone.

Latest key values:

- open-loop `500 ms`:
  - predicted `U,R,V = 0.292059, 5.103136, -0.002114`
- traction-limited `500 ms`:
  - predicted `U,R,V = 0.301445, 4.851594, -0.002722`

Expected target in those checks:

- `U,R = 0.200000, 0.600000`

So the system is not fixed, but it is back to the earlier known-bad regime rather than the even worse body-only-exact regime.

### Other still-important failures

- `PlantModelTractionLimitedVelocityTargetKeepsTenPercentTractionReserveWhenLimited`
  - `limited=false reserve_accel=8.000001 limited_accel=8.000001 expected_reserved=7.200001`
- `FeedforwardAgreesWithPredict`
  - `Vehicle speed shouldn't be too high.`
- `PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest`
  - still failing with `Actual:<0.302125>`
- many maneuver tests remain broadly broken

## What Improved In This Continuation

The touched operating-point tests are much more useful now.

Several touched checks that were failing under the incorrect multi-step test semantics are now passing again, and the remaining touched failures now tell you exactly what they mean:

- they are no longer ambiguous short-horizon transient mismatches
- they now clearly indicate a direct algebraic inverse versus `forwardStep(...)` inconsistency

That narrows the next debugging target substantially.

## Current Hypothesis

The current biggest issue is not the velocity-target exact branch itself.

The bigger blocker is lower-level:

- `solveDriveCommands(...)` and `forwardStep(...)` still do not agree on the yaw response of the returned operating point

Likely reasons to investigate:

1. `solveDriveCommands(...)` appears to treat the baseline front/rear lateral-yaw contribution as fixed while changing the longitudinal bank forces.
2. `forwardStep(...)` recomputes the contact forces through `EvaluateRollingState(...)` and `EvaluateSplitContactForces(...)`, where friction-ellipse clamping can change the lateral force distribution when longitudinal force rises.
3. The direct inverse may still be mixing “incremental around baseline” reasoning with “absolute operating-point force” reasoning.

In other words:

- the returned wheel speeds / forces may not actually correspond to the same force decomposition that `forwardStep(...)` then reconstructs.

## Best Next Step

Do not go back to the body-only exact solver.

Instead:

1. Trace `solveDriveCommands(...)` against `forwardStep(...)` for the two direct yaw mismatch tests.
2. Focus on:
   - `EvaluateSplitContactForces(...)`
   - `EvaluateRollingState(...)`
   - `PlantModel::forwardStep(...)`
   - `PlantModel::solveDriveCommands(...)`
3. Determine whether the longitudinal command in `solveDriveCommands(...)` should be interpreted as:
   - total bank longitudinal force
   - or incremental force relative to the current rolling state
4. Confirm whether the front/rear lateral-force contribution used in the inverse must be recomputed after longitudinal-force clamping rather than treated as fixed baseline.
5. Only after the direct inverse matches `forwardStep(...)` again should you return to tuning the velocity-target layer.

## References

- current takeover prompt:
  - `takeover_prompt_2026-04-21_v2.md`
- prior handoffs:
  - `feedforward_handoff_2026-04-21.md`
  - `agent_handoffs_2026-04-21.md`
- latest verify:
  - `codex_verify/logs/verify_latest_build_20260421_040112_931.txt`
- current main code:
  - `MazeMap/MazeMap/PlantModel.cpp`
- touched tests:
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`

## Incoming User Data

The user said they are going to run the system and the result should be ready for the next agent in the `TestResults` folder.

That means the next agent should:

1. inspect `TestResults` for anything newer than this handoff
2. compare the new real-world behavior against the current host failures before making another major plant/feedforward change
