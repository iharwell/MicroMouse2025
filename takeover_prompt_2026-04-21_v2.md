# Takeover Prompt 2026-04-21 v2

You are taking over the feedforward investigation in `C:\Users\thene\source\repos\MicroMouse2025`.

Before doing anything else:

1. Read `AGENTS.md` now.
2. Read it again after every context compaction.

Then read these handoff files in this order:

- `feedforward_handoff_2026-04-21.md`
- `agent_handoffs_2026-04-21.md`
- `feedforward_handoff_2026-04-21_v2.md`
- `agent_handoffs_2026-04-21_v2.md`

## User Goal

Build a non-iterative / algebraic feedforward system that matches the actual predictions we make within about 10%, while remaining fault-free and returning reasonable results.

The user also explicitly wants:

- tests that fail in exactly one obvious way per test path
- no confusing multi-assert ambiguity when a test fails
- care around `forwardStep(...)` semantics versus multi-step prediction/update semantics

## Current State

Working-tree files currently modified or untracked include:

- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- `MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp`
- `codex_verify/ARDUINO_BUILD_NOTES.md`
- `codex_verify/build_and_verify_latest.ps1`
- `tooling/README.md`
- `tooling/analyze_open_floor.py`
- several `tooling/__pycache__/*` files
- the prior handoff markdown files

Do not casually revert:

- `codex_verify/ARDUINO_BUILD_NOTES.md`
- `codex_verify/build_and_verify_latest.ps1`
- `tooling/README.md`
- `tooling/analyze_open_floor.py`
- `MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp`

Those include user or other-thread work.

## Important Facts Already Established

- `PlantModel` is still the authoritative owner.
- The body-only exact-solver experiment in `ResolveVelocityTargetExactControl(...)` was tried and rejected. It made the host yaw overshoot materially worse.
- The touched direct `solveDriveCommands(...)` operating-point tests should use `forwardStep(...)`, not UKF predict+update and not `integrate(...)`.
- `forwardStep(...)` is valid for instantaneous operating-point validation only. It is not a drop-in replacement for multi-step predict/update behavior.
- The touched `PlantModelDriveCommandTest.cpp` cases now use one explicit failure message per test instead of many ambiguous asserts.

## Current Authoritative Verify Result

Use:

- `codex_verify/logs/verify_latest_build_20260421_040112_931.txt`

Result:

- `592` tests
- `528` passed
- `64` failed

Important note:

- That verify was run after syncing the fresh host `MazeMap.dll` from
  `MazeMap/MazeMap/x64/Release/MazeMap.dll`
  into the wrapper-visible path
  `MazeMap/x64/Release/MazeMap.dll`
- The verify wrapper still reported the firmware image stale because the latest source edit happened after the last Teensy build.
- Host results are still useful and should be treated as the current authoritative host state.

## Expected New Real-World Data

The user said a fresh run should be available for the next agent in the `TestResults` folder.

Before making the next substantive plant/feedforward change, check `TestResults` for any newer datalog or analysis artifacts produced after this handoff and compare them against the current host-side failure regime.

## Most Important Remaining Failures

### 1. Direct inverse mismatch inside `solveDriveCommands(...)`

These are now the clearest blocker:

- `PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget`
  - failure message:
    `forwardStep yaw accel mismatch reported/achieved=4.500000,-93.903145`
- `PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited`
  - failure message:
    `forwardStep yaw accel mismatch reported/achieved=18.000002,-102.752533`

That means the direct algebraic inverse in `solveDriveCommands(...)` is still not self-consistent with `forwardStep(...)` at the returned operating point.

### 2. Velocity-target feedforward still overshoots yaw badly

Latest key values:

- open-loop velocity target at `500 ms`:
  - predicted `U,R,V = 0.292059, 5.103136, -0.002114`
  - expected `U,R = 0.200000, 0.600000`
- traction-limited velocity target at `500 ms`:
  - predicted `U,R,V = 0.301445, 4.851594, -0.002722`
  - expected `U,R = 0.200000, 0.600000`

So the velocity-target layer is still wrong, but do not treat it as the first thing to change again until the direct inverse is reconciled with `forwardStep(...)`.

### 3. `FeedforwardAgreesWithPredict` is still failing

- latest message:
  - `Vehicle speed shouldn't be too high.`

### 4. Traction-limited reserve behavior still not correct

- `PlantModelTractionLimitedVelocityTargetKeepsTenPercentTractionReserveWhenLimited`
  - latest message:
    `limited=false reserve_accel=8.000001 limited_accel=8.000001 expected_reserved=7.200001`

## Current Best Next Step

Do not go back to the body-only exact solver.

The highest-signal next work is:

1. Compare `PlantModel::solveDriveCommands(...)` against `PlantModel::forwardStep(...)` for the failing yaw operating-point cases.
2. Focus on whether `solveDriveCommands(...)` is treating longitudinal bank-force demand as total force while still assuming a fixed baseline lateral/yaw contribution from `EvaluateRollingState(...)`.
3. Inspect how friction-ellipse clamping in `EvaluateSplitContactForces(...)` changes the front/rear lateral force distribution once large longitudinal force is applied.
4. Only after the direct inverse matches `forwardStep(...)` again should you revisit the velocity-target layer.

## Code Areas To Inspect

- `MazeMap/MazeMap/PlantModel.cpp`
  - `EvaluateSplitContactForces(...)`
  - `EvaluateRollingState(...)`
  - `PlantModel::forwardStep(...)`
  - `PlantModel::solveDriveCommands(...)`
  - `SolveVelocityTargetFeedforward(...)`

Key line anchors in the current working tree:

- `SolveVelocityTargetFeedforward(...)` around line `1555`
- `PlantModel::forwardStep(...)` around line `1884`
- `PlantModel::solveDriveCommands(...)` around line `2279`

## Manual Build / Verify Nuance

If the user performs a manual host build again, check the binary locations before verify:

- the fresh host `MazeMap.dll` may land at
  `MazeMap/MazeMap/x64/Release/MazeMap.dll`
- the verify wrapper expects
  `MazeMap/x64/Release/MazeMap.dll`

In the latest manual-build cycle:

- `MazeMapTest.dll` was already written to the wrapper-visible path
- `MazeMap.dll` had to be copied into the wrapper-visible path before `test_latest_binaries.cmd` used the fresh build

## Caution

- Do not casually revert existing user/other-thread changes.
- Do not “fix” the touched direct operating-point tests by pushing them back to `integrate(...)` or UKF predict/update semantics.
- The touched tests now intentionally fail with one explicit meaning each. Preserve that property if you edit them again.
