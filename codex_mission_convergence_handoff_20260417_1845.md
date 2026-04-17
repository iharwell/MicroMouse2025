# Mission Convergence Handoff

Date: 2026-04-17

## Task state

- The work is not done.
- The audit is not clean.
- The stop condition remains a clean audit.
- The secondary size target is now `340 KB` of Teensy flash code.

## User clarifications that must be treated as authoritative

- `SharedRobotRuntime` is the convergence point for shared boot-mode infrastructure, even if some older docs understate that.
- A `Routine` in this project is callback-driven `LoopController` work, following the `ManeuverExecutor` model.
- Synchronous or blocking routine wrappers are not allowed.
- Any routine that does not explicitly pause `LoopController` must return control before the active tick deadline.
- If `LoopController` is paused, the robot must not move.
- Re-read `AGENTS.md` after each context compression.

## What was learned and corrected in this pass

- The first attempt to centralize simple maneuver helpers through blocking `Runtime::Run*Routine(...)` helpers was architecturally wrong for this project.
- That blocking surface was removed.
- `AGENTS.md` and `project_vocabulary.md` were updated so the callback-driven routine rule and the paused-motion prohibition are now explicit.
- The corrected direction is:
  - keep routine launch callback-owned,
  - keep `ManeuverExecutor` as the authoritative shared maneuver owner,
  - keep `SharedRobotRuntime` as the convergence point for shared boot-mode infrastructure,
  - do not create a second execution model just to reduce edits.

## What is landed right now

- `MissionModeController.cpp` and `MazeRunningAuditController.cpp` no longer call `ManeuverExecutor` public `Begin*Phase(...)` entry points.
- `ManeuverExecutor` phase entry points remain internal/private detail.
- The controllers launch simple hold, settle, reverse-straight, straight, turn, arc, and smooth-turn routines from the first loop tick via `ManeuverExecutor::Begin*Routine(...)`.
- The temporary blocking helper surface was removed from:
  - `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
  - `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.cpp`
- The callback-driven rule was documented in:
  - `AGENTS.md`
  - `project_vocabulary.md`

## Current verified result

- Latest verification log:
  - `codex_verify/logs/build_and_verify_latest_20260417_184021_869.txt`
- Result:
  - Teensy compile succeeded.
  - Host Release build succeeded.
  - Release tests ran.
  - Test result was `512 total / 511 passed / 1 failed`.
  - The single failure remained the known pre-existing failure:
    - `MazeMap/MazeMapTest/DriveBaseTest.cpp:426`
    - `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`
- Latest Teensy size from that log:
  - `FLASH: code:402304`
  - `RAM1: code:399352`
- This is still far above the `340 KB` target.

## Highest-value clean-audit blockers still remaining

1. `MissionModeController.cpp` and `MazeRunningAuditController.cpp` still each own a duplicated controller-local `LoopController` mini-framework.
2. Startup wall calibration and wall-touch are still duplicated across mission and audit and remain the best next flash/convergence cut.
3. Mission search and mapping execution are still trapped inside `MissionModeController.cpp` instead of a shared runtime-owned callback-driven routine surface.
4. Controller-local logging/bootstrap lifecycle is still duplicated above the runtime-owned logger.
5. Boot modes are still thinner wrappers than they should be, but that is lower priority than the motion and session duplication.

## Recommended next cut order

1. `WallTouchRoutine`
- Move shared wall-touch and startup wall-calibration execution out of both controllers.
- Keep mode/controller code responsible only for workflow policy, labels, and result interpretation.
- This is the best immediate target for both audit convergence and flash recovery.

2. Shared session and logging convergence
- Remove the duplicated controller-local `LoopController` harness and duplicated logging lifecycle code.
- Replace them with one callback-driven shared surface owned by runtime infrastructure.
- Do not introduce another wrapper class.

3. `SearchStraightRoutine`
- Extract the loop-state and straight-search execution that still keeps `MissionModeController` as the owner of reusable maze-running motion behavior.
- Keep caller-owned continuation and caller-owned result storage.

4. `MappingRoutine`
- Move observation capture, rolling observation, and fusion work out of mission policy code.
- Leave goal choice, retry policy, and mission progression in mission ownership.

## Concrete file focus for the next agent

- `MazeMap/MazeMap/MissionModeController.cpp`
- `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.cpp`
- `MazeMap/MazeMap/MazeMapSharedRuntime.h`
- `MazeMap/MazeMap/MazeMapSharedRuntime.cpp`
- `MazeMap/MazeMap/ManeuverExecutor.h`
- `MazeMap/MazeMap/ManeuverExecutor.cpp`
- `AGENTS.md`
- `project_vocabulary.md`

## Important cautions

- Do not reintroduce synchronous or blocking routine wrappers.
- Do not widen the duplicated controller-local maneuver launcher into a permanent shared facade.
- Do not add another boot-mode/session wrapper family.
- Keep `MissionRunMode`, `CorridorRepeatabilityMode`, and `PositionAccuracyAuditMode` as the public boot-mode owners unless a larger convergent change explicitly replaces that structure.
- Continue to treat `ManeuverExecutor` as the authoritative shared owner of maneuver execution.
- Keep `SharedRobotRuntime` as the convergence point for shared boot-mode infrastructure.
- The repo is dirty beyond this task. Do not revert unrelated changes.
- `MazeMap/MazeMap/MazeRunningAuditController.cpp` and `.h` are untracked in git status but are part of the active build.

## Verification rules to follow next

- Before testing, check timestamps so the binaries being tested are actually newer than the edited sources.
- Use the repo scripts:
  - `codex_verify/build_and_verify_latest.cmd --no-pause`
  - or the paired verify script if only verification is needed.
- If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately.
- Do not “fix” that condition with `Clean`, `Rebuild`, or more artifact deletion.

## Best short instruction for the next agent

Continue from the verified callback-driven state, keep the stop condition as a clean audit, and take `WallTouchRoutine` / startup wall-calibration duplication out of mission and audit first without inventing a second execution model.
