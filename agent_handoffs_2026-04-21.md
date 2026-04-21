# Agent Handoffs 2026-04-21

This file records what the existing subagents reported so a fresh thread can pick up their work without losing context.

## Goodall

- Inspected:
  - `Feedforward analysis 1`
  - `MazeMap/MazeMap/PlantModel.cpp`
  - `MazeMap/MazeMap/PlantModel.h`
  - `MazeMap/MazeMap/DriveBase.cpp`
  - `MazeMap/MazeMap/MazeMapRuntimeCore.h`
  - `MazeMap/MazeMap/DiagnosticConfig.h`
- Key finding:
  - The deleted heuristic yaw-assist / yaw-bias / common-trim path is not the present overshoot source.
  - The current exact branch enforces both body-rate and terminal wheel-rate targets over `responseTimeS`.
  - That exact solve is recomputed every 1 ms while the default response horizon is 25 ms.
  - The wheel terminal constraint is the most likely source of front-loaded torque and host overshoot.
- Recommended next step:
  - In `ResolveVelocityTargetExactControl(...)`, replace the two `ResolveModeAffineNetDriveTorqueNm(...)` calls with `ResolveModeNetDriveTorqueNm(...)` so the exact branch enforces body-rate targets only.

## Aristotle

- Inspected:
  - `codex_verify/build_and_verify_latest.ps1`
  - `codex_verify/build_and_verify_latest.cmd`
  - `codex_verify/test_latest_binaries.cmd`
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
  - `MazeMap/MazeMapTest/DriveBaseTest.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp`
  - `MazeMap/MazeMapTest/DriveManeuverTests.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h`
- Key finding:
  - The new 5-tick `ObserveShortPredictResponse(...)` tests are not measuring a pure fixed-feedforward plant response.
  - They run a full UKF loop with synthetic encoder and yaw updates every tick, then infer average acceleration from filtered-state deltas.
  - That does not match the semantics of `DriveCommandSolution.commandedLongitudinalAccelMps2` / `commandedYawAccelRadps2`, which come from plant operating-point prediction.
- Recommended next step:
  - Do not use `RunPredictionMatchingCycle(...)` for exact feedforward acceleration checks.
  - Use direct `forwardStep(...)` validation or pure `PlantModel` integration for those assertions.
  - Keep `RunPredictionMatchingCycle(...)` for UKF predict/update consistency tests only.

## Godel

- Inspected:
  - `Feedforward analysis 1`
  - `MazeMap/MazeMap/PlantModel.cpp`
  - `MazeMap/MazeMap/PlantModel.h`
  - `MazeMap/MazeMap/DriveBase.cpp`
  - `MazeMap/MazeMap/Drive.cpp`
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
  - `tooling/open_floor_plant_fit.py`
  - `tooling/competition_feedforward.py`
- Key finding:
  - `PlantModel::solveDriveCommands(...)` is already an algebraic, self-consistent inverse.
  - The velocity-target mismatch is introduced in `SolveVelocityTargetFeedforward(...)`, which historically solved a base command and then post-processed it with heuristic shaping.
  - The exact algebraic velocity-target path already exists and should be the canonical basis for non-iterative matching work.
- Recommended next step:
  - Prefer the exact algebraic path for public non-traction-limited velocity-target feedforward.
  - Keep the heuristic path only as fallback.
  - Leave traction-limited reserve overloads alone so they keep reserve behavior.

## Faraday

- Inspected:
  - `Feedforward analysis 1`
  - `MazeMap/MazeMap/PlantModel.cpp`
  - `MazeMap/MazeMap/DriveBase.cpp`
  - `MazeMap/MazeMap/Drive.cpp`
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
  - `codex_verify/build_and_verify_latest.ps1`
  - `codex_verify/logs/build_and_verify_latest_20260421_011513_172.txt`
- Key finding:
  - The public velocity-target feedforward path is still heuristic-heavy.
  - `PlantModel` already contains the exact algebraic inverse that fits the user’s “match predictions without iteration” goal.
  - `DriveBase` adds feedback on top of the base feedforward, so feedforward alignment should be checked against the base feedforward path and telemetry, not the final closed-loop command.
  - The Teensy `core.a` failure came from the wrapper pathing, not from `PlantModel`.
- Recommended next step:
  - Make the exact algebraic velocity-target path primary when valid and not traction-limited.
  - Stop applying post-solve yaw/common heuristics on that exact path.
  - Rebuild and verify through the canonical wrapper.

## Sartre

- Inspected:
  - `Feedforward analysis 1`
  - `MazeMap/MazeMap/PlantModel.cpp`
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
  - `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp`
  - open-floor datalogs under `TestResults/mmlog_decode_2026-04-21_*`
  - replay report under `TestResults/open_floor_ukf_replay_direct_20260420_203605/report.md`
  - verify logs
- Key finding:
  - The round-trip problem predates the newest exact wiring.
  - The current exact-path selection likely adds a separate regression.
  - The real-world datalog comparison suggests the exact path materially improved rolling turns while pure yaw startup/stop remained weak.
  - Host-side acceptance should not globally reject exact control.
- Recommended next step:
  - Fix the rejected-candidate mutation / selection problem first.
  - Then use regime-aware exact selection:
    - prefer exact in rolling cases when validated prediction improves
    - be conservative in static / high-slip yaw startup-stop cases

## Galileo

- Inspected:
  - `Feedforward analysis 1`
  - `MazeMap/MazeMap/PlantModel.cpp`
  - `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`
  - `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp`
  - latest verify logs
- Key finding:
  - The note argues against the old heuristic yaw/common-trim branch, not against the exact algebraic path.
  - The exact solve should use the real operating state with observed wheel speed where available.
  - The host-side one-step metric should not be the sole veto for exact control.
- Recommended next step:
  - Keep the exact path primary or near-primary in the state overload using the real operating state and observed wheels.
  - Do not restore the old heuristic branch.

## Overall Synthesis

- Broad agreement:
  - Do not restore the deleted yaw/common-trim heuristic path.
  - Keep `PlantModel` as the canonical owner.
  - Preserve traction-limited reserve behavior separately.
  - The exact algebraic path is the right direction.
- Main disagreement / caution:
  - The new 5-tick predict+update tests may not be the right assertion shape for feedforward acceleration validation.
  - Real-world rolling-turn data suggests the exact path should not be discarded based only on current host-side rejection logic.
- Highest-signal next experiment:
  - Change the exact solver from body+wheel terminal enforcement to body-only terminal enforcement.

