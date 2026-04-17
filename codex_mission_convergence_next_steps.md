# Mission Convergence Next Steps

This note records the next convergent cuts after the current `ManeuverInstance` / `ManeuverPoint` work.

## Current State

- Documentation now explicitly says:
  - one boot-selected top-level mode owner owns one `LoopController` session for the life of that mode,
  - only the active callback runs while motion is active,
  - waits are only allowed inside `LoopController::RequestPause(...)` callbacks,
  - mission mode owns goals and replans, not shared motion execution,
  - `ManeuverInstance` / `ManeuverPoint` are the execution vocabulary, not `SmoothTurnExecutionProfile`.
- Production code now samples maneuver points from the maneuver owners and feeds them through `DriveBase`.
- `MissionModeController` still owns too much motion/session machinery and still starts nested loop sessions internally.

## Immediate Blocker

1. Restore a usable host Release incremental state.
2. Re-run `build_and_verify_latest.cmd --no-pause`.
3. Stop again if the script reports `HOST_INTERMEDIATE_STATE_BROKEN`.
4. If the Release intermediates are healthy but the host link still fails, fix the concrete host build issue before pushing deeper refactors.

## Refactor Order

### 1. Move session ownership outward

The top-level mode owners must become the only places that start the active `LoopController` session:

- `MissionRunMode`
- `ManeuverFileTestMode`
- `CorridorRepeatabilityMode`
- `PositionAccuracyAuditMode`

Required cut:

- move `RunLoopSession(...)` ownership out of `MissionModeController`,
- keep callback bodies, but stop letting subordinate helpers launch their own sessions,
- make session start mean mode startup and session end mean mode shutdown.

### 2. Replace outer-stack mission sequencing with callback-owned phase flow

Replace helper-driven mini-session flow with one callback graph owned by the active session.

Target shape:

- `MissionStartupCallback`
- `MappingModeCallback`
- `SearchPathCallback`
- `QueuedManeuverCallback`
- `ReturnToStartCallback`
- `GoalPauseCallback`
- `RacingRunCallback`
- `InterRunServiceCallback`
- `MissionCompleteCallback`

Required cut:

- remove outer-stack sequencing that assumes normal C++ execution resumes after a motion helper returns,
- move in-motion replans, mapping updates, and maneuver dispatch decisions into callbacks or shared services called by callbacks.

### 3. Delete remaining mission-owned simple motion executors

These are shared motion primitives and should not remain mission-owned:

- `SettleLoopTick`
- `ReverseStraightLoopTick`
- `StraightLoopTick`
- `TurnLoopTick`

Required cut:

- move their state/logic into shared drive execution ownership,
- update mission callers to request those primitives instead of owning them,
- delete the mission-side implementations instead of preserving parallel copies.

### 4. Collapse maneuver execution onto one shared owner

The remaining maneuver execution layer still lives in the wrong place.

Delete from mission ownership:

- `ExecuteQueuedManeuvers`
- `ExecuteSearchPath`
- `ExecuteArcProfile`
- `ExecuteSmoothTurnProfile`
- maneuver-speed / queue-shaping helpers that only exist to support that mission-owned executor

Replace with:

- one shared maneuver executor rooted with the drive runtime,
- consuming `ManeuverInstance`,
- using `ManeuverSet` / `ManeuverPoint` for derived execution targets,
- exposing one canonical path for arc/smooth/in-place/straight maneuver progression.

### 5. Remove mission-owned state that belongs with shared execution

As motion ownership moves, delete or relocate:

- per-motion loop state structs,
- motion watchdog state that belongs to shared execution,
- maneuver geometry helpers duplicated in mission code,
- mission-owned pose/motion correction helpers that are really shared execution/fusion behavior.

Mission mode should keep:

- goal selection,
- replan policy,
- mission progression,
- mission success/failure decisions.

### 6. Keep pause behavior narrow

As session ownership moves, preserve these constraints:

- no waits for control progress outside `RequestPause(...)`,
- no generic pause dispatch hub,
- each pause callback stays local to the phase that requested it,
- pauses exist only for operator input, settling, or non-real-time work.

## Verification Steps After Each Cut

1. Check timestamps to confirm edited sources are newer than active host binaries.
2. Run the smallest relevant Release unit coverage first if available.
3. Run `build_and_verify_latest.cmd --no-pause`.
4. If the script reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately.
5. Record the blocking log path in the work note before continuing later.

## Done When

This cleanup is not done when the code merely compiles.

It is done when:

- top-level mode owners are the only session owners,
- no nested `RunLoopSession(...)` launches remain in `MissionModeController`,
- no waiting for control progress exists outside pause callbacks,
- mission mode no longer owns shared motion primitives or maneuver execution,
- `ManeuverInstance` / `ManeuverPoint` are the only execution vocabulary for maneuver tracking,
- the remaining mission code reads as policy/scheduling instead of as a drive executor.
