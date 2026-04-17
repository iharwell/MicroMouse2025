# Handoff Log B

Date: 2026-04-17

## Scope

This pass focused on stabilizing the maze-running mode/controller split after the earlier partial inline migration left the tree in a confused state.

Primary goals handled in this pass:

- recover one clear authoritative mission owner,
- keep corridor/position utility owners separate from mission,
- restore thin boot-mode wrappers,
- get the repo back through the standard verify path,
- reduce duplicated code pressure on the Teensy build.

## What I Changed

- Re-audited the current convergence state using:
  - `AGENTS.md`
  - `project_vocabulary.md`
  - `codex_mission_convergence_next_steps.md`
  - the repo-root handoff notes from the earlier agents
- Restored the active mission path to a single mission-only controller:
  - `MazeMap/MazeMap/MissionModeController.h`
  - `MazeMap/MazeMap/MissionModeController.cpp`
- Kept `MissionRunMode` as a thin mode wrapper over the mission controller:
  - `MazeMap/MazeMap/MissionRunMode.h`
  - `MazeMap/MazeMap/MissionRunMode.cpp`
- Kept the utility modes thin and pointed them at the utility-side controller split:
  - `MazeMap/MazeMap/CorridorRepeatabilityMode.h`
  - `MazeMap/MazeMap/CorridorRepeatabilityMode.cpp`
  - `MazeMap/MazeMap/PositionAccuracyAuditMode.h`
  - `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp`
  - `MazeMap/MazeMap/MazeRunningAuditController.h`
  - `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- Fixed project wiring so the active build matches the current owner split:
  - `MazeMap/MazeMap/MazeMap.vcxproj`
  - `MazeMap/MazeMap/MazeMap.vcxproj.filters`

## Architectural Outcome

Current active owner shape:

- `MissionRunMode` is a thin boot-selected mission mode wrapper.
- `MissionModeController` is the current mission-only implementation owner and uses routine-style names:
  - `BeginMissionRunRoutine()`
  - `RunMissionRunRoutine()`
- `CorridorRepeatabilityMode` is a thin utility-mode wrapper.
- `PositionAccuracyAuditMode` is a thin utility-mode wrapper.
- `MazeRunningAuditController` is the current utility-side implementation owner for:
  - corridor repeatability
  - position accuracy audit

This is still not the final converged architecture, but it is materially better than the broken state where multiple mode files had absorbed large duplicated controller bodies.

## Key Finding From The Audit

The highest-value step-3 work is still the same:

1. finish moving the remaining maneuver-session scaffolding out of the mode/controller owners and into `ManeuverExecutor`,
2. then extract shared routine owners for:
   - wall touch
   - search straight
   - mapping / observation capture

The main illegal ownership still present is the nested loop/session scaffolding and routine implementation logic inside the controller bodies. The current pass did not solve that deeper extraction; it stabilized the owner split and build surface so those cuts can proceed cleanly.

## Verification

Latest successful full verify path to actionable result:

- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260417_095928_377.txt`

Results:

- Teensy compile succeeded.
- Host Release build succeeded.
- Release tests ran to completion.
- Test result was:
  - 512 total
  - 511 passed
  - 1 failed

Known remaining test failure:

- `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`
- file: `MazeMap/MazeMapTest/DriveBaseTest.cpp:426`

This appears to be the same pre-existing failing Release test already called out in earlier handoff notes, not a new failure introduced by this pass.

## Teensy Size

Latest size from the verify log:

- `FLASH: code:409152`
- `RAM1: code:406200`

Comparison to the earlier state seen during this thread:

- earlier observed Teensy flash code: `412224`
- latest observed Teensy flash code: `409152`

So this pass recovered about 3 KB of flash by removing some duplicated active ownership/build surface, but the build is still above the softer `~380 KB` target and far above the long-term `~330 KB` goal.

## Important Current Workspace State

- `MazeRunningAuditController.cpp` and `MazeRunningAuditController.h` are active in the current build shape.
- Those two files are still untracked in git in the current worktree.
- Do not delete them casually; the utility mode wrappers currently depend on them.
- There are other unrelated dirty-tree changes outside this pass. Do not blindly revert the tree.

## Recommended Next Steps

1. Continue the step-3 convergence inside the controllers, not by adding wrappers.
2. Finish the remaining `ManeuverExecutor` convergence first.
3. Remove the controller-local maneuver session helpers that still wrap `ManeuverExecutor`.
4. Extract `WallTouchRoutine`.
5. Extract `SearchStraightRoutine`.
6. Extract `MappingRoutine`.
7. Re-run `build_and_verify_latest.cmd --no-pause` after each convergent cut.

