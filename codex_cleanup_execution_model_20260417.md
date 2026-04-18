# Execution Model Cleanup Note

This file records intentional cleanup residue that remains after the wall-touch routine migration so it does not get lost.

## Completed convergence

- `WallTouchRoutine` is now the authoritative owner for shared wall-touch execution.
- The duplicated controller-local wall-touch loop implementations were removed from:
  - `MazeMap/MazeMap/MissionModeController.cpp`
  - `MazeMap/MazeMap/MazeRunningAuditController.cpp`
- `ManeuverExecutor` no longer carries wall-touch behavior.

## Remaining execution-model drift

- `MazeRunningAuditController.cpp` still has `SharedRoutineLaunchTick` / `RunSharedRoutine(...)`.
- `MissionModeController.cpp` now prepares maneuver-executor entry callbacks before starting a dedicated session, which removes the one-cycle launch tick but still preserves session-level bridge debt.
- `MazeRunningAuditController.cpp` still uses the older dedicated-session launch-tick pattern to start maneuver-related routines.

## Why it remains

- Removing the wall-touch duplication was the convergent change for this pass.
- Reworking the remaining bridge code still requires a broader controller/session rewrite so routine entry happens directly from the active callback chain inside the one boot-owned session.

## Deletion target

- Delete the controller-level launch-tick helpers and dedicated-session bridge helpers above once the mission/audit controllers are converted to direct routine handoff.
- At that point, remove `WallTouchRoutine::PrepareInitialCallbacks(...)` as well so `WallTouchRoutine::Begin(...)` is the only public entry path.
