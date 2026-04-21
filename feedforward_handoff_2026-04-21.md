# Feedforward Handoff 2026-04-21

This file is a context-preservation note for the current feedforward investigation.
It captures what was changed, what was verified, what data was used, and the next step I was about to take.

## User Goal

- Develop a non-iterative / algebraic feedforward system that matches the actual predictions we make within about 10%.
- Do not allow the feedforward block to fail; it must return reasonable results.
- Avoid the previous heuristic contamination called out in `Feedforward analysis 1`.
- Be careful about transient slip / oscillation behavior when acceleration demand jumps.
- Move any single-tick host tests to 5-tick predict+measurement-update tests using encoder-pair + gyro updates.

## AGENTS.md Requirements Reconfirmed

I reread `AGENTS.md` before writing this handoff.
The key constraints that matter here are:

- `PlantModel` is the authoritative owner for this work.
- Preserve buildability and both host + Teensy verification paths.
- Do not introduce parallel owners or wrapper layers.
- Before testing, verify that active binaries are current rather than blindly rebuilding.

At the next compaction, reread `AGENTS.md` again before proceeding.

## Current Working Tree

Current modified files observed in `git status --short`:

- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- `codex_verify/ARDUINO_BUILD_NOTES.md`
- `codex_verify/build_and_verify_latest.ps1`
- `tooling/analyze_open_floor.py`

Important ownership note:

- The `codex_verify/ARDUINO_BUILD_NOTES.md` change is from another thread and should not be reverted casually.
- `tooling/analyze_open_floor.py` is also user/other-thread work and should not be reverted casually.

## What I Changed

### 1. PlantModel feedforward path

Current working-tree changes in `MazeMap/MazeMap/PlantModel.cpp`:

- Added `UpdateVelocityTargetSolutionPrediction(...)` to revalidate a returned feedforward command by rebuilding a validation state and running `forwardStep(...)`.
- Added `ResolveVelocityTargetResponseTolerance(...)`.
- Added `ComputeVelocityTargetResponseErrorMetric(...)`.
- Expanded `VelocityTargetExactSolution` with:
  - `commandedLongitudinalAccelMps2`
  - `commandedYawAccelRadps2`
  - `longitudinalAccelErrorMps2`
  - `yawAccelErrorRadps2`
  - `converged`
- Modified `ResolveVelocityTargetExactControl(...)` so it records commanded acceleration/error fields and requires finite commands / finite prediction fields / motor commands under `0.999`.
- Modified `ApplyVelocityTargetExactControl(...)` so it:
  - returns `bool`
  - takes both a solve state and a validation state
  - copies exact fields into a `DriveCommandSolution`
  - calls `UpdateVelocityTargetSolutionPrediction(...)`
- Modified `SolveVelocityTargetFeedforward(...)` so it:
  - takes `operatingState`
  - takes `validationState`
  - takes `exactState`
  - takes `useObservedWheelState`
  - takes `preferExactAlgebraicControl`
  - recomputes prediction telemetry on the base solution
  - marks `converged` from a short-horizon response metric
  - prefers the exact candidate whenever that candidate is finite
- Modified the public velocity-target state overload so it currently wires:
  - `operatingState` into validation
  - `operatingState` into exact solve
  - `useObservedWheelState = true`
  - `preferExactAlgebraicControl = true`
- Modified the reduced overload so it currently uses reduced state for all three states and `preferExactAlgebraicControl = true`.
- Left traction-limited velocity-target overloads on `preferExactAlgebraicControl = false`.

### 2. Host tests

Current working-tree changes in `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`:

- Added `kShortPredictUpdateSteps = 5`.
- Added `ShortPredictObservation`.
- Added `ObserveShortPredictResponse(...)`, which:
  - resets `SrUkfCore`
  - performs 5 `RunPredictionMatchingCycle(...)` steps
  - synthesizes encoder-pair + yaw-rate updates
  - returns final state plus average `U` and `R` accelerations across the horizon
- Converted several direct `forwardStep(...)` single-tick tests into 5-tick predict+update tests.
- There are currently no remaining `forwardStep(... solution.control ...)` checks in that file.

### 3. Teensy build wrapper

I patched `codex_verify/build_and_verify_latest.ps1` to fix the Arduino build-path/output-dir mixup:

- Changed:
  - `$buildPath = Join-Path $canonicalBuildPath 'build'`
  - `$firmwareOutputDir = Join-Path $canonicalBuildPath 'firmware'`
- Added:
  - `--output-dir $firmwareOutputDir`
  - while keeping `--build-path $buildPath`

Why:

- The last full build log showed Arduino/Teensy failing with:
  - `ar.exe: unable to copy file '...codex_verify\arduino_build\firmware\core\core.a'; reason: No such file or directory`
- `arduino-cli compile --help` confirms `--build-path` and `--output-dir` are separate concepts.
- The current repo-local build tree already matches the split layout:
  - `codex_verify/arduino_build/build/core/core.a`
  - `codex_verify/arduino_build/firmware/sketch/...`

I also parsed the modified PowerShell file with the PowerShell parser and it is syntactically valid.

## What I Verified

### Binary freshness

I checked timestamps before testing:

- `MazeMap/MazeMap/PlantModel.cpp` -> `2026-04-21 01:51:29`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp` -> `2026-04-21 02:03:23`
- `MazeMap/x64/Release/MazeMap.dll` -> newer than `PlantModel.cpp`
- `MazeMap/x64/Release/MazeMapTest.dll` -> newer than `PlantModelDriveCommandTest.cpp`

That means the release host DLLs used for verification matched the edited sources at the time of the last verify run.

### Host verification command run

I ran:

- `.\codex_verify\test_latest_binaries.cmd --no-pause`

Latest verify log used:

- `codex_verify/logs/verify_latest_build_20260421_021241_385.txt`

Result:

- Release host verification ran successfully against current binaries.
- Total tests: `592`
- Passed: `526`
- Failed: `66`

### Most important current host failures

Feedforward-specific failures from the latest verify log:

- `PlantModelSolveDriveCommandsForVelocityTargetReportsTractionLimitAtCanonicalResponseTime`
- `PlantModelTractionLimitedVelocityTargetKeepsTenPercentTractionReserveWhenLimited`
- `PlantModelSolveDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAfterOneSecond`
- `PlantModelSolveDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAfterOneSecond`
- `PlantModelSolveDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons`
- `PlantModelSolveDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons`
- `PlantModelSolveDriveCommandsForVelocityTargetReportsReturnedControlPredictionAtOperatingPoint`
- `FeedforwardAgreesWithPredict`

Key failure values from that log:

- Open-loop reduced/state velocity-target at `500 ms`:
  - predicted `U,R,V = 0.292059, 5.103136, -0.002114`
  - expected `U,R = 0.200000, 0.600000`
- Traction-limited reduced/state velocity-target at `500 ms`:
  - predicted `U,R,V = 0.301445, 4.851594, -0.002722`
  - expected `U,R = 0.200000, 0.600000`
- New 5-tick operating-point tests also showed very large mismatches:
  - expected `1.2`, actual `42.2539`
  - expected `3.2`, actual `108.682`
  - expected `18`, actual `3205.92`

Broader unrelated failures remain in maneuver tests and a plant perturbation test.

## Data Used

### 1. `Feedforward analysis 1`

Key conclusion from that note:

- The old heuristic velocity-target path is contaminated by:
  - yaw assist scaling
  - explicit yaw bias
  - explicit negative common command trim
- The note argues against restoring that heuristic contamination.
- The note does **not** argue against using an exact algebraic branch.

### 2. Open-floor datalog comparison

Agent review of `TestResults` concluded:

- `TestResults/mmlog_decode_2026-04-21_01-09-34/open_floor_main.csv`
  looks materially better than
  `TestResults/mmlog_decode_2026-04-21_00-16-10/open_floor_main.csv`
  for rolling-turn behavior.
- Rolling-turn angular MAE improved substantially.
- Pure yaw was not materially better overall.

Implication:

- Do not globally reject the exact path based only on host-side rejection logic.
- Pure-yaw startup/stop is still a weak regime.

### 3. UKF replay report

Used:

- `TestResults/open_floor_ukf_replay_direct_20260420_203605/report.md`

Key conclusion:

- The replay report itself says its numbers are prediction-vs-observable consistency checks, not external ground truth.
- The worst mismatch buckets are concentrated in yaw.

### 4. Teensy build log

Used:

- `codex_verify/logs/build_and_verify_latest_20260421_011513_172.txt`

Key failure:

- `ar.exe` could not copy `codex_verify\arduino_build\firmware\core\core.a` because that path did not exist.

This is what drove the `build_and_verify_latest.ps1` fix.

## Agent Findings

### Agent finding: exact solver aggressiveness

The best concrete recommendation from the agent review was:

- Keep the exact algebraic branch.
- Stop enforcing terminal wheel-speed targets inside the exact branch.
- Specifically, inside `ResolveVelocityTargetExactControl(...)`, replace:
  - `ResolveModeAffineNetDriveTorqueNm(...)`
- with:
  - `ResolveModeNetDriveTorqueNm(...)`

Reasoning:

- The exact branch currently solves for both:
  - terminal body-rate target
  - terminal slip-bearing wheel-rate target
- But the command is not actually held for the full response horizon.
- It is recomputed every 1 ms while the target horizon stays at 25 ms.
- That makes the wheel terminal constraint too aggressive, especially in yaw.

This was the next code change I was about to make.

### Agent finding: exact branch should use real operating state

Another agent review confirmed:

- The exact solve should use the real operating state with observed wheel speeds where available.
- That is already how the current state overload is wired in the working tree.
- The heuristic yaw/common-trim branch should not be restored.

## Current Hypothesis

The main host-side problem is not that algebraic exact control is inherently wrong.
The problem is that the current exact branch is solving the wrong algebraic problem for a receding-horizon 1 ms usage pattern.

More specifically:

- `ResolveVelocityTargetExactControl(...)` currently enforces target wheel arrival over the full response horizon.
- That is likely front-loading torque and producing large transient yaw acceleration.
- A body-only exact solve should preserve the algebraic approach while removing the most aggressive wheel-target constraint.

## Important Open Question About The 5-Tick Tests

The new 5-tick tests are consistent with the user request in structure:

- they use predict
- they synthesize encoder-pair updates
- they synthesize gyro updates

However, the very large observed acceleration values suggest one of two things:

1. the exact command is genuinely far too aggressive in the host prediction/update loop, or
2. some of the converted assertions are still measuring the wrong quantity over such a short, quantized encoder horizon

I have not resolved that question yet.
The converted 5-tick tests remain in the working tree, but they are not yet validated as the final correct assertion shape.

## Immediate Next Steps

If continuing from this handoff, the next recommended sequence is:

1. Reread `AGENTS.md`.
2. Keep `PlantModel` as the owner.
3. In `MazeMap/MazeMap/PlantModel.cpp`, change the exact branch from wheel+body terminal enforcement to body-only terminal enforcement:
   - replace the two `ResolveModeAffineNetDriveTorqueNm(...)` calls in `ResolveVelocityTargetExactControl(...)`
   - use `ResolveModeNetDriveTorqueNm(...)`
   - remove the now-unused target wheel-rate block
4. Ask the user to run the full build again using the now-fixed wrapper so they can get a fresh Teensy datalog.
5. Compare:
   - host verify results
   - fresh real-world datalog
6. If real-world behavior remains improved while host prediction is still poor, narrow the mismatch to test formulation or estimator semantics rather than undoing the exact branch.

## Current File/Log References

- Feedforward note:
  - `Feedforward analysis 1`
- Main code:
  - `MazeMap/MazeMap/PlantModel.cpp`
- Main tests:
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h`
- Latest verify log:
  - `codex_verify/logs/verify_latest_build_20260421_021241_385.txt`
- Teensy build-failure log:
  - `codex_verify/logs/build_and_verify_latest_20260421_011513_172.txt`
- Datalogs used:
  - `TestResults/mmlog_decode_2026-04-21_01-09-34/open_floor_main.csv`
  - `TestResults/mmlog_decode_2026-04-21_00-16-10/open_floor_main.csv`
- Replay report used:
  - `TestResults/open_floor_ukf_replay_direct_20260420_203605/report.md`

