# Takeover Prompt 2026-04-21

You are taking over a feedforward investigation in `C:\Users\thene\source\repos\MicroMouse2025`. Use of agents is encouraged but not required.

Before doing anything else:

1. Read `AGENTS.md` now.
2. Read it again after every context compaction.

Then read these handoff files first:

- `feedforward_handoff_2026-04-21.md`
- `agent_handoffs_2026-04-21.md`

## User Goal

Build a non-iterative / algebraic feedforward system that matches the actual predictions we make within about 10%, while remaining fault-free and returning reasonable results.

The user specifically warned about:

- sudden acceleration demand causing extra slip and strange tire-dynamics transients
- needing real-world reasonableness, not just host-side synthetic success

## Current State

Working-tree files currently modified:

- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
- `codex_verify/ARDUINO_BUILD_NOTES.md`
- `codex_verify/build_and_verify_latest.ps1`
- `tooling/analyze_open_floor.py`

Do not casually revert:

- `codex_verify/ARDUINO_BUILD_NOTES.md`
- `tooling/analyze_open_floor.py`

Those include other-thread/user work.

## Important Facts Already Established

- `PlantModel` is the authoritative owner for this task.
- `Feedforward analysis 1` argues against the old heuristic yaw-assist / yaw-bias / common-trim path.
- The current exact velocity-target path appears too aggressive in host prediction.
- Real-world datalog evidence suggests the exact path materially improved rolling turns even though pure yaw remains somewhat weak with 90 degree turns pivoting on the right bank and 180 turns operating well.
- The Teensy build failure was in the build wrapper pathing, not in `PlantModel`.

## Current Host Verification Result

Latest verify run used:

- `codex_verify/logs/verify_latest_build_20260421_021241_385.txt`

Result:

- `592` tests
- `526` passed
- `66` failed

Most important feedforward failures:

- open-loop velocity-target misses target badly at 500 ms / 1 s / extended horizons
- traction-limited velocity-target also misses badly
- `FeedforwardAgreesWithPredict` fails
- the new 5-tick tests currently show extremely large mismatches

## Current Best Next Step

The highest-signal next code change is:

- In `MazeMap/MazeMap/PlantModel.cpp`
- inside `ResolveVelocityTargetExactControl(...)`
- replace the two `ResolveModeAffineNetDriveTorqueNm(...)` calls with `ResolveModeNetDriveTorqueNm(...)`

Rationale:

- The exact solver currently enforces both body-rate and terminal wheel-rate targets over the full response horizon.
- But the command is recomputed every 1 ms while the default response horizon is 25 ms.
- That likely front-loads torque and causes the host overshoot.
- Switching to body-only exact enforcement keeps the approach algebraic while removing the most suspicious overconstraint.

## Extra Caution

Do not assume the converted 5-tick tests are all correct.
Agent review suggests they may be mixing UKF measurement-update semantics with feedforward acceleration validation.
If you touch those tests, keep the user’s requirement in mind:

- single-tick host tests should move to 5-tick predict + synthesized encoder-pair + synthesized gyro updates due to how the UKF and plant model presently work.

But do not confuse that with validating the returned plant operating-point acceleration telemetry.

## Teensy Build

`codex_verify/build_and_verify_latest.ps1` was patched so Arduino uses:

- `--build-path codex_verify/arduino_build/build`
- `--output-dir codex_verify/arduino_build/firmware`

That was done to fix the previous `core.a` failure from:

- `codex_verify/logs/build_and_verify_latest_20260421_011513_172.txt`

If the user builds again and it still fails, inspect the new build log first before changing anything else.

## Suggested Work Sequence

1. Read `AGENTS.md`.
2. Read the two handoff markdown files.
3. Inspect the current diff in `PlantModel.cpp` and `PlantModelDriveCommandTest.cpp`.
4. Preserve the Teensy wrapper fix unless a new log proves it is wrong.
5. Implement the body-only exact-solver change in `PlantModel.cpp`.
6. Re-check the latest binaries before testing.
7. Ask the user to build if needed.
8. Run `codex_verify/test_latest_binaries.cmd --no-pause`.
9. If Teensy build is good, ask the user for a new real-world datalog.
10. Compare host behavior against the new datalog before deciding whether the remaining issue is real or synthetic.

