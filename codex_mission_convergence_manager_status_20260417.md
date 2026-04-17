# Mission Convergence Manager Status

Date: 2026-04-17

## Current verified state

- `MissionModeController.cpp` and `MazeRunningAuditController.cpp` no longer call `ManeuverExecutor` public `Begin*Phase(...)` entry points.
- Both controllers now launch maneuver execution through controller-local request state plus the routine-form `ManeuverExecutor` APIs.
- External callers of `ManeuverExecutor` phase entry points were eliminated in this pass.
- `ManeuverExecutor` now exposes the phase entry points only as internal/private implementation detail.
- The controller-local launcher still exists as duplicated transitional debt in both controllers. Do not widen it into a shared wrapper layer.

## Verification

- `build_and_verify_latest.cmd --no-pause`
  - log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260417_162258_121.txt`
  - result:
    - Teensy compile succeeded.
    - Host Release build reached `MazeMap.dll`, `MazeSimulation.exe`, and `MazeMapTest.dll`.
    - build then failed during the `MazeMapTest.vcxproj` post-build copy step because `MazeMap.dll` in `MazeMap\x64\Release` was locked by another local process.
- `verify_latest_build.cmd --no-pause`
  - log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\verify_latest_build_20260417_162711_265.txt`
  - result:
    - verified latest firmware and Release host artifacts were newer than source inputs.
    - Release tests ran to completion.
    - test result: 512 total, 511 passed, 1 failed.
    - failing test matched the known pre-existing failure:
      - `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`
      - `MazeMap/MazeMapTest/DriveBaseTest.cpp:426`

## Size snapshot

- Teensy compile from `build_and_verify_latest_20260417_162258_121.txt`
  - `FLASH: code:410304`
  - `RAM1: code:407352`

This pass did not recover flash. The build remains well above the current `340 KB` secondary target.

## Practical blocker note

- There is a long-lived local `vstest.console.exe` / Visual Studio environment holding `MazeMap.dll` during the `MazeMapTest.vcxproj` post-build copy step.
- The up-to-date artifacts in `MazeMap\x64\Release` were still usable for `verify_latest_build`.
- If the copy-step failure remains, fix the local file-lock situation rather than bypassing the repo verify scripts with ad hoc build/test commands.

## Recommended next extraction order

1. `WallTouchRoutine`
   - move shared wall-touch and startup-calibration execution out of both controllers.
   - keep mode/controller owners responsible only for workflow policy, labels, and result interpretation.
2. `SearchStraightRoutine`
   - extract the loop-state and straight-search execution that currently keeps `MissionModeController` as the owner of reusable maze-running motion behavior.
   - design it as a routine with caller-owned continuation and caller-owned result storage.
3. `MappingRoutine`
   - move observation capture / rolling observation / fusion work out of mission policy code.
   - leave goal choice, retry policy, and mission progression inside `MissionRunMode` / mission ownership.

## Cautions for the next pass

- Do not turn the duplicated controller-local maneuver launcher into a permanent shared facade.
- Keep `MissionRunMode`, `CorridorRepeatabilityMode`, and `PositionAccuracyAuditMode` as the public boot-mode owners.
- Continue using `ManeuverExecutor` as the authoritative shared owner of maneuver execution.
- Re-read `AGENTS.md` after any context compression before continuing the extraction work.

## 18:43 update

- User clarification: `SharedRobotRuntime` is the convergence point for shared boot-mode infrastructure in this codebase.
- User clarification: callback-driven routine ownership is mandatory. A `Routine` follows the `ManeuverExecutor` model:
  - it advances only through `LoopController` callbacks,
  - it must return control before the active tick deadline unless the loop is explicitly paused,
  - the robot must not move while `LoopController` is paused.
- The temporary blocking `Runtime::Run*Routine(...)` helper surface was removed after that clarification.
- `MissionModeController.cpp` and `MazeRunningAuditController.cpp` now launch their simple hold/settle/straight/turn/arc/smooth-turn routines through controller-local callback-owned loop state again, using `ManeuverExecutor::Begin*Routine(...)` from the first session tick instead of blocking runtime wrappers.
- Documentation was updated in:
  - `AGENTS.md`
  - `project_vocabulary.md`

## Latest verification

- `build_and_verify_latest.cmd --no-pause`
  - log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260417_184021_869.txt`
  - result:
    - Teensy compile succeeded.
    - Host Release build succeeded.
    - Release tests ran to completion.
    - test result: 512 total, 511 passed, 1 failed.
    - failing test remained the known pre-existing failure:
      - `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`
      - `MazeMap/MazeMapTest/DriveBaseTest.cpp:426`

## Latest size snapshot

- Teensy compile from `build_and_verify_latest_20260417_184021_869.txt`
  - `FLASH: code:402304`
  - `RAM1: code:399352`

## Current clean-audit blockers

1. `MissionModeController.cpp` and `MazeRunningAuditController.cpp` still each own a duplicated `LoopController` mini-framework and still start controller-local sessions.
2. Startup wall calibration and wall-touch remain the biggest duplicated executable body across mission and audit and are still the best next flash/convergence cut.
3. Mission mapping/search execution remains trapped inside `MissionModeController.cpp` rather than a shared runtime-owned callback-driven routine surface.
4. Controller-local logging/bootstrap lifecycle is still duplicated above the runtime-owned logger.
