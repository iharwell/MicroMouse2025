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

## What Landed In This Pass

- Added [MazeMap/MazeMap/WallTouchRoutine.h](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\WallTouchRoutine.h).
- Added [MazeMap/MazeMap/WallTouchRoutine.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\WallTouchRoutine.cpp).
- Moved shared wall-touch execution ownership out of:
  - [MazeMap/MazeMap/MissionModeController.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MissionModeController.cpp)
  - [MazeMap/MazeMap/MazeRunningAuditController.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeRunningAuditController.cpp)
- Both controllers now hand wall-touch execution to `WallTouchRoutine` instead of carrying controller-local wall-touch loop implementations.
- `WallTouchRoutine` owns the pass-through settle behavior internally instead of routing that behavior through `ManeuverExecutor`.
- Project wiring was updated in:
  - [MazeMap/MazeMap/MazeMap.vcxproj](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMap.vcxproj)
  - [MazeMap/MazeMap/MazeMap.vcxproj.filters](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMap.vcxproj.filters)

## Policy Updates Landed

- [AGENTS.md](C:\Users\thene\source\repos\MicroMouse2025\AGENTS.md) now explicitly says:
  - there are no size-based exceptions to convergence rules,
  - a partial migration that preserves the old owner is worse than not migrating,
  - wrappers/redirection shells are not acceptable migration steps,
  - the preferred migration sequence is:
    - copy to an external non-compiled reference file,
    - delete from compiled source immediately,
    - move into the final authoritative destination.

## Verified Result

Latest verification log:

- [codex_verify/logs/build_and_verify_latest_20260417_201658_182.txt](C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260417_201658_182.txt)

Result:

- Teensy compile succeeded.
- Host `Release|x64` build succeeded.
- Release tests ran.
- Test result was `512 total / 511 passed / 1 failed`.
- The single failing test remained the known pre-existing failure:
  - [MazeMap/MazeMapTest/DriveBaseTest.cpp](C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DriveBaseTest.cpp:426)
  - `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`

Size snapshot from that verify run:

- `FLASH: code:403200`
- `RAM1: code:400248`

## Explicit Remaining Drift

Residual execution-model drift is documented in:

- [codex_cleanup_execution_model_20260417.md](C:\Users\thene\source\repos\MicroMouse2025\codex_cleanup_execution_model_20260417.md)

Current documented residue:

- `MissionModeController.cpp` still has `SharedMotionRoutineLaunchTick` / `RunSharedMotionRoutine(...)`.
- `MazeRunningAuditController.cpp` still has `SharedRoutineLaunchTick` / `RunSharedRoutine(...)`.
- Those helpers still use the older launch-tick/session pattern for maneuver-related routine start.
- `WallTouchRoutine::BeginSession(...)` remains only as bridge debt until that larger controller/session rewrite is completed.

## Important Constraint For The Next Pass

- Do not justify any wrapper, duplicate path, or partial migration because it is small, temporary, local, or easy to review.
- If a migration cannot be completed into the final owner, stop and record the blocker instead of leaving ambiguous compiled ownership behind.

## Dirty Tree Note

This workspace still contains unrelated dirty/untracked items outside the wall-touch change, including:

- `MazeMap/MazeMap/MazeMapSharedRuntime.cpp`
- `MazeMap/MazeMap/MazeMapSharedRuntime.h`
- `project_vocabulary.md`
- `codex_verify/build_and_verify_latest.ps1`

Do not treat those as part of the wall-touch convergence unless they are deliberately re-audited.
- The temporary blocking helper surface introduced earlier was removed.

## What Landed In This Pass

The execution-model and vocabulary correction is now explicit and durable in repository documentation:

- `AGENTS.md`
- `project_vocabulary.md`
- `execution_model_guide.md`

This pass corrected the earlier wording drift that implied a routine could span phases. That wording is now superseded.

## What Did Not Land In This Pass

- The shared wall-touch / startup wall-calibration convergence is **not** landed.
- The duplicated controller-local wall-touch execution glue is still present.
- Mission and audit still each own controller-local wall-touch setup and callback plumbing around:
  - `ExecuteWallTouchOffLoopDriven(...)`
  - `WallTouchLoopTick(...)`
  - `TryTouchWallAndMaybeSetKnownWallCoordinate(...)`
- The broader duplicated controller-local `LoopController` mini-framework is still present in both controllers.
- Search/mapping extraction did not move.
- Shared session/logging convergence did not move.

## Important Non-Landed Attempt

During this pass, a partial code attempt explored moving wall-touch initialization/tick helper logic toward `ManeuverExecutor`.

That attempt was **backed out** and must be treated as non-landed. The repository should not be understood as having a finished shared wall-touch routine yet.

The rejected shape was not sufficient because it only moved helper pieces while leaving the controller-local routine/session scaffolding in place. That is not the converged cut.

## Current Highest-Value Clean-Audit Blockers

1. `MissionModeController.cpp` and `MazeRunningAuditController.cpp` still each own duplicated controller-local `LoopController` mini-framework code.
2. Startup wall calibration and wall-touch are still duplicated across mission and audit and remain the best immediate convergence and flash-recovery target.
3. Mission search and mapping execution are still trapped inside `MissionModeController.cpp` instead of a shared callback-driven owner.
4. Controller-local logging/bootstrap lifecycle is still duplicated above the runtime-owned logger.
5. Boot modes are still thinner wrappers than they should be, but that remains lower priority than motion/session duplication.

## Immediate Next Cut

The next cut should still be the shared wall-touch / startup wall-calibration extraction, but it must follow the clarified execution model.

### Required target shape

- The mode owns the phase boundary, labels, and policy.
- The shared owner hosts a reusable wall-touch routine or equivalent callback-driven behavioral block.
- That shared routine owns all ticks from launch until completion.
- The mode regains control only through the supplied continuation callback.
- The routine API must not accept or own phase labels as though phases were routine concepts.
- Keep controller code responsible only for workflow policy, labels, target selection, and result interpretation.

### Explicitly avoid

- Blocking `Run*Routine(...)` wrappers.
- A helper split that only moves initialization/tick code without eliminating the duplicated controller-local routine/session scaffolding.
- A second execution model.
- A new wrapper family around `SharedRobotRuntime` or `ManeuverExecutor`.
- Naming or contracts that imply a routine spans phases.

## Recommended Next File Focus

- `MazeMap/MazeMap/MissionModeController.cpp`
- `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- `MazeMap/MazeMap/ManeuverExecutor.h`
- `MazeMap/MazeMap/ManeuverExecutor.cpp`
- `MazeMap/MazeMap/MazeMapSharedRuntime.h`
- `MazeMap/MazeMap/MazeMapSharedRuntime.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.cpp`
- `AGENTS.md`
- `project_vocabulary.md`
- `execution_model_guide.md`

## Verification Rules For The Next Pass

- Before testing, check timestamps so the binaries being tested are newer than the edited sources.
- Use the repo scripts:
  - `codex_verify/build_and_verify_latest.cmd --no-pause`
  - or the paired verify script if only verification is needed.
- If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately.
- Do not try to fix that condition with `Clean`, `Rebuild`, or more artifact deletion.
- Record whether any failure is new or matches the known pre-existing `DriveBaseTest.cpp:426` failure.

## Best Short Instruction For The Next Agent

Continue from the verified callback-driven baseline, treat `AGENTS.md`, `project_vocabulary.md`, and `execution_model_guide.md` as authoritative for every individual step, and extract the shared wall-touch / startup wall-calibration behavior without inventing a second execution model or implying that routines span phases.
