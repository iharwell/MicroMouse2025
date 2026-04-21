# Agent Handoffs 2026-04-21 v2

This file records the effective handoff state for a fresh thread after the second continuation pass.

## New Agent Activity In This Continuation

No new subagents were spawned during this continuation.

The useful information below is a synthesis of:

- the earlier agent handoff file
- the prior feedforward handoff
- the current thread’s code changes and verify results

## Findings That Still Hold From The Earlier Agent Work

### Goodall

- The old heuristic yaw-assist / yaw-bias / common-trim path is not the right thing to restore.
- The body-only exact-solver idea was worth testing once, but should not be treated as the default fix.

### Aristotle

- UKF predict+measurement-update loops are the wrong validation shape for direct feedforward acceleration telemetry.
- Those semantics belong in estimator tests, not in direct `solveDriveCommands(...)` operating-point checks.

### Godel / Faraday / Galileo

- The exact algebraic path is still the right long-term direction.
- `PlantModel` remains the correct owner.
- The old heuristic shaping path should not be resurrected as the main solution.

### Sartre

- Real-world rolling-turn data still suggests the exact path should not be globally discarded based only on host-side acceptance logic.
- But that does not change the fact that the direct inverse currently needs to be made self-consistent first.

## Current Thread Findings

### 1. The touched direct operating-point tests were previously on the wrong semantics

That is now fixed.

The touched direct `solveDriveCommands(...)` tests have been moved back to instantaneous `forwardStep(...)` validation and each now has one explicit failure path/message.

This means the remaining failures are much more trustworthy.

### 2. The body-only exact-solver experiment was harmful and is no longer active

The body-only exact solve made the velocity-target yaw overshoot materially worse:

- it pushed open-loop yaw to around `9.055712 rad/s` at `500 ms`

That experiment has been undone.

Current open-loop/traction-limited numbers are back to the earlier known-bad regime:

- open-loop `500 ms`:
  - `0.292059, 5.103136, -0.002114`
- traction-limited `500 ms`:
  - `0.301445, 4.851594, -0.002722`

### 3. Exact-candidate selection is now less obviously wrong

Current code no longer returns the exact candidate merely because its error metric is finite.

It now requires the exact candidate to improve the response metric over the base solution.

This is good, but it did not fix the feedforward system.

### 4. The main blocker is now clearly below the velocity-target layer

Latest authoritative host verify:

- `codex_verify/logs/verify_latest_build_20260421_040112_931.txt`
- `528 passed / 64 failed`

Most important remaining direct-inverse mismatches:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
  - `reported/achieved yaw accel = 4.500000 / -93.903145`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`
  - `reported/achieved yaw accel = 18.000002 / -102.752533`

Interpretation:

- `solveDriveCommands(...)` is still not self-consistent with `forwardStep(...)`
- until that is fixed, velocity-target feedforward tuning is downstream of a broken base inverse

### 5. Manual-build path nuance matters for verify

Latest manual build behavior:

- fresh host `MazeMap.dll` landed in
  `MazeMap/MazeMap/x64/Release/MazeMap.dll`
- wrapper verify uses
  `MazeMap/x64/Release/MazeMap.dll`

If the user manually builds again, check whether the fresh host DLL needs to be copied into the wrapper-visible release directory before running `test_latest_binaries.cmd`.

`MazeMapTest.dll` was already landing in the wrapper-visible release directory in the latest manual build.

## Best Next Move For A Fresh Agent

Do not ask the user for more information first.

The next agent should:

1. Reread `AGENTS.md`.
2. Read the old and new handoff files.
3. Keep `PlantModel` as the owner.
4. Debug the direct mismatch between:
   - `PlantModel::solveDriveCommands(...)`
   - `PlantModel::forwardStep(...)`
5. Compare how these functions handle:
   - longitudinal bank forces
   - baseline lateral/yaw moment
   - friction-ellipse clamping and front/rear right-force redistribution
6. Only after that mismatch is resolved should the agent return to velocity-target feedforward tuning.

## Current Key References

- latest takeover prompt:
  - `takeover_prompt_2026-04-21_v2.md`
- latest detailed handoff:
  - `feedforward_handoff_2026-04-21_v2.md`
- current main code:
  - `MazeMap/MazeMap/PlantModel.cpp`
- touched tests:
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- latest verify:
  - `codex_verify/logs/verify_latest_build_20260421_040112_931.txt`
