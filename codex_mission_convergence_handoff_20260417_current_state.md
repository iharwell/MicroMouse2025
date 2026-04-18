# Mission Convergence Handoff

Date: 2026-04-17

This is the only active handoff note for the mission-convergence slice.

## Current State

- The work is not done.
- The audit is not clean.
- The stop condition remains a clean audit against:
  - `AGENTS.md`
  - `project_vocabulary.md`
  - `execution_model_guide.md`
- The secondary size target remains `340 KB` of Teensy flash code.

## What Landed In The Current Tree

- `WallTouchRoutine` remains the shared owner for wall-touch execution:
  - [MazeMap/MazeMap/WallTouchRoutine.h](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\WallTouchRoutine.h)
  - [MazeMap/MazeMap/WallTouchRoutine.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\WallTouchRoutine.cpp)
- The wall-touch routine no longer writes through a caller-provided result sink.
- `WallTouchRoutine` now owns its completed result and exposes `LastResult()`.
- Mission and audit now read wall-touch completion data from the routine-owned result instead of passing a sink pointer:
  - [MazeMap/MazeMap/MissionModeController.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MissionModeController.cpp)
  - [MazeMap/MazeMap/MazeRunningAuditController.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeRunningAuditController.cpp)
- The bridge API name `WallTouchRoutine::BeginSession(...)` was removed.
- The current bridge helper is now `WallTouchRoutine::PrepareInitialCallbacks(...)` to make it explicit that it prepares callback transfer state but does not start a `LoopController` session.
- `ManeuverExecutor` bridge helpers were renamed to `Prepare*RoutineCallbacks(...)` so the callback-transfer bridge path is named honestly instead of looking like the canonical non-blocking routine entrypoint.

## Execution-Model And Vocabulary Corrections Landed

The repository documentation now reflects the clarified session and callback model:

- [AGENTS.md](C:\Users\thene\source\repos\MicroMouse2025\AGENTS.md)
- [project_vocabulary.md](C:\Users\thene\source\repos\MicroMouse2025\project_vocabulary.md)
- [execution_model_guide.md](C:\Users\thene\source\repos\MicroMouse2025\execution_model_guide.md)

The important wording now in force is:

- a boot cycle normally gets one `LoopController` session,
- a second session is allowed only when vehicle state is known to be discontinuous, such as user service or a physical lift that invalidates UKF continuity,
- `LoopController` must not be paused, started, or stopped unless the vehicle is stationary or the runtime is faulting,
- `SetNextModeWorkCallbacks(...)` / `SetNextModeWorkCallback(...)` are the control-transfer mechanisms inside an active session,
- pauses are for blocking, no-motion calculation, and other non-real-time work.

## Verified Result

Latest verification log:

- [codex_verify/logs/build_and_verify_latest_20260417_213645_679.txt](C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260417_213645_679.txt)

Result:

- Teensy compile succeeded.
- Host `Release|x64` build succeeded.
- Release tests ran.
- Test result was `512 total / 511 passed / 1 failed`.
- The single failing test remained the known pre-existing failure:
  - [MazeMap/MazeMapTest/DriveBaseTest.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DriveBaseTest.cpp:426)
  - `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`

Size snapshot from that verify run:

- `FLASH: code:401728`
- `RAM1: code:398776`

## Explicit Remaining Drift

Residual execution-model drift is documented in:

- [codex_cleanup_execution_model_20260417.md](C:\Users\thene\source\repos\MicroMouse2025\codex_cleanup_execution_model_20260417.md)

Current highest-signal residue:

- `MissionModeController.cpp` no longer uses the old one-cycle shared-motion launch tick, but it still starts dedicated `LoopController` sessions around routine bridges.
- `MazeRunningAuditController.cpp` still carries the older `SharedRoutineLaunchTick` / `RunSharedRoutine(...)` pattern above the shared owners.
- `WallTouchRoutine::PrepareInitialCallbacks(...)` remains bridge debt. The converged target is still `WallTouchRoutine::Begin(...)` as the only public entry path.
- `WallTouchRoutine::Begin(...)` / `PrepareInitialCallbacks(...)` still accept physical execution parameters (`targetYawRad`, `minLatchTravelM`, `maxApproachTravelM`, pose-reset target) instead of maze-grid intent.
- `WallTouchRoutine::PrepareWallTouchPhase(...)` still samples vehicle launch baseline too early. It captures state before the routine owns a tick, which violates the clarified execution model.
- Mission and audit still each own duplicated startup wall-touch policy and wall-calibration derivation above the shared routine.

## Important Design Correction Agreed In This Thread

The next agent should treat the following as authoritative for the wall-touch API:

- The parameters of `Begin(...)` should only describe what the routine should do.
- `Begin(...)` must not sample vehicle state that belongs to the caller's current tick.
- The routine does not own the tick in which `Begin(...)` is called.
- Launch baselines must be captured on the first routine-owned tick, not during callback installation.
- Completed wall-touch results should be obtained from a getter on the routine owner, not through a sink parameter.
- The wall-touch command surface should be maze-grid intent:
  - which wall
  - from which side
- Both of those inputs should use maze-grid/domain types, not an external physical-parameter bag.

## Important Constraint For The Next Pass

- Do not justify any wrapper, duplicate path, or partial migration because it is small, temporary, local, or easy to review.
- If a migration cannot be completed into the final owner, stop and record the blocker instead of leaving ambiguous compiled ownership behind.
- Do not preserve the current physical-parameter wall-touch API as the final design.

## Recommended Next Cut

The next convergent cut should be the wall-touch API and startup-calibration intent cleanup, not another naming-only pass.

Required target shape:

- `WallTouchRoutine::Begin(...)` should accept maze-grid intent only.
- Wall-touch result access should remain routine-owned through `LastResult()`.
- The first routine-owned tick should capture the real launch baseline.
- Shared wall-touch policy and wall-selection derivation should move inward toward the shared owner as far as possible without inventing a wrapper family.
- Mode code should retain only workflow policy, phase labels, and interpretation of the result.

Explicitly avoid:

- reintroducing sink-style result plumbing,
- passing physical target coordinates / yaw / travel windows as the final public wall-touch API,
- claiming routine ownership of the caller's current tick,
- using a second execution model,
- preserving the dedicated-session bridge code as the final architecture.

## Recommended Next File Focus

- `MazeMap/MazeMap/WallTouchRoutine.h`
- `MazeMap/MazeMap/WallTouchRoutine.cpp`
- `MazeMap/MazeMap/MissionModeController.cpp`
- `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `AGENTS.md`
- `project_vocabulary.md`
- `execution_model_guide.md`

## Dirty Tree Note

The current worktree intentionally includes active edits in:

- `AGENTS.md`
- `project_vocabulary.md`
- `execution_model_guide.md`
- `codex_cleanup_execution_model_20260417.md`
- `MazeMap/MazeMap/ManeuverExecutor.h`
- `MazeMap/MazeMap/ManeuverExecutor.cpp`
- `MazeMap/MazeMap/WallTouchRoutine.h`
- `MazeMap/MazeMap/WallTouchRoutine.cpp`
- `MazeMap/MazeMap/MissionModeController.cpp`
- `MazeMap/MazeMap/MazeRunningAuditController.cpp`

There may also be unrelated dirty files elsewhere in the workspace. Re-audit before broadening the task.

## Verification Rules For The Next Pass

- Before testing, check timestamps so the binaries being tested are newer than the edited sources.
- Use the repo scripts:
  - `codex_verify/build_and_verify_latest.cmd --no-pause`
  - or the paired verify script if only verification is needed.
- If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately.
- Do not try to fix that condition with `Clean`, `Rebuild`, or more artifact deletion.
- Record whether any failure is new or matches the known pre-existing `DriveBaseTest.cpp:426` failure.

## Best Short Instruction For The Next Agent

Continue from the verified callback-driven baseline, keep the corrected session/pause rules intact, and converge `WallTouchRoutine` so its public entrypoint takes maze-grid intent only, captures launch baseline on the first routine-owned tick, and remains the shared owner of completed wall-touch result state without sink-based plumbing.
