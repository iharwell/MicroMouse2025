# Handoff Log D

Date: 2026-04-17

## Scope

This pass did not edit code. It audited `MissionModeController` directly against:

- `AGENTS.md`
- `project_vocabulary.md`
- `codex_mission_convergence_next_steps.md`
- `ManeuverExecutor` as the reference for routine vocabulary

The goal of the pass was to identify the minimal `MissionModeController` subset that is needed only for corridor repeatability and position accuracy audit workflows, and to separate that from mission-only logic and shared infrastructure that should not be copied into a utility-specific owner.

## Work Completed

- Re-read `AGENTS.md`.
- Inspected `MissionModeController.h` and `MissionModeController.cpp` directly.
- Inspected `ManeuverExecutor.h` and `ManeuverExecutor.cpp` to align with the repo's `Routine` vocabulary.
- Mapped exact line/function ranges that are utility-only versus mission-only.
- Identified the shared startup/calibration and maneuver-session helpers that the audit workflows currently depend on but should not own.

## Main Finding

`MissionModeController` is still mixing three different categories of behavior:

1. mission-only policy and search/speed-run ownership,
2. utility-audit workflow code,
3. shared startup/calibration and maneuver-session infrastructure.

The minimal utility slice is much smaller than the full file. The clean split is not "copy the controller and rename it." The clean split is:

- move only the utility-audit workflow surface and utility-only telemetry lifecycle into a dedicated private audit owner,
- keep mission policy out,
- and avoid re-owning shared startup/calibration or shared maneuver-session infrastructure inside that audit owner.

## Minimal Utility Move Set

### Public surface

- `MissionModeController.h:23-27`
- `MissionModeController.cpp:6441-6458`

This is the corridor and position utility public surface only:

- `BeginCorridorRepeatabilityMode()`
- `RunCorridorRepeatabilityMode()`
- `BeginPositionAccuracyAuditMode()`
- `RunPositionAccuracyAuditMode()`

### Utility entry routines

- `MissionModeController.cpp:139-269`

This covers:

- `BeginCorridorRepeatabilityMode()`
- `RunCorridorRepeatabilityMode()`
- `BeginPositionAccuracyAuditMode()`
- `RunPositionAccuracyAuditMode()`

### Utility-only telemetry lifecycle and callbacks

- `MissionModeController.cpp:439-472`
- `MissionModeController.cpp:633-645`
- `MissionModeController.cpp:662-673`
- `MissionModeController.cpp:3971-4425`

This covers:

- `BeginTelemetryLog()`
- `WriteTelemetryEvent()`
- `FlushTelemetryLog()`
- `CloseTelemetryLog()`
- `WriteAuxMeasurementEventCallback()`
- `FailAuxMeasurementCallback()`
- `ShutdownTelemetryMode()`
- `LogWallCalibrationMetadata()`

### Corridor repeatability routine body

- `MissionModeController.cpp:1394-1548`

This covers:

- `ReseatCorridorRepeatabilityStartPose()`
- `RunSingleCorridorRepeatabilityPass()`
- `RunCorridorRepeatabilityPasses()`

### Position accuracy audit routine body

- `MissionModeController.cpp:1550-1673`
- `MissionModeController.cpp:1762-2104`

This covers:

- `RunSinglePositionStraightAuditPass()`
- `RunSinglePositionSmoothTurnAuditPass()`
- `RunPositionAccuracyAuditPasses()`

### Utility-only state worth moving

- `MissionModeController.cpp:292`
- `MissionModeController.cpp:300-303`

This is:

- `_telemetryLoggingEnabled`
- `_telemetryLogFileName`
- `_telemetryPhaseId`
- `_telemetrySampleCount`
- `_telemetryLogRow`

## Shared Dependencies Used By The Audits But Not Audit-Owned

These are currently used by the audit workflows, but they should not be copied into a utility-specific audit controller as if they were audit-specific infrastructure:

- `MissionModeController.cpp:614-621`
- `MissionModeController.cpp:1328-1391`
- `MissionModeController.cpp:2115-3058`
- `MissionModeController.cpp:3769-3955`
- `MissionModeController.cpp:4434-4763`
- `MissionModeController.cpp:4846-4994`
- `MissionModeController.cpp:6323-6405`

This bucket includes:

- known-start and start-pose reseating helpers,
- wall-touch and calibration motion helpers,
- startup wall calibration,
- loop/session dispatch,
- `ManeuverExecutor` hook wiring,
- shared hold/queue/profile launch helpers.

Important naming drift noted during the audit:

- `PrimeKnownMissionStartCell()` and `ReseatMissionStartPoseWithPhasePrefix()` are not mission-only despite their names.
- They are currently shared startup/calibration helpers used by the utility workflows too.

## Mission-Only Code That Should Stay Out

### Mission top-level flow

- `MissionModeController.cpp:65-137`
- `MissionModeController.cpp:6431-6438`

This is:

- `BeginMissionRunMode()`
- `RunMissionRunMode()`

### Mission search, mapping, return, and speed-run ownership

- `MissionModeController.cpp:486-563`
- `MissionModeController.cpp:4997-6080`
- `MissionModeController.cpp:6272-6320`

This bucket includes:

- `SearchLimits()`
- `FinalLimits()`
- `SearchUnmappedCruiseSpeedMps()`
- wall observation and mapping updates,
- search-straight execution and replan handling,
- exploration, goal pause, return-to-start, and speed-run flow,
- `OrientTo()`

### Mission-only state

- `MissionModeController.cpp:281-283`
- `MissionModeController.cpp:289-290`
- `MissionModeController.cpp:295-296`
- `MissionModeController.cpp:332-341`
- `MissionModeController.cpp:358-394`

This is:

- `_searchPathFinder`
- `_speedPathFinder`
- `_wallBeliefMap`
- `_goalPauseComplete`
- `_missionComplete`
- `_frontWallCharacterization`
- `_frontWallCharacterizationAvailable`
- `ObservationCaptureLoopState`
- `SearchStraightLoopState`

## Dead Drift Found

### Unused utility-specific function

- `MissionModeController.cpp:1675-1760`

`RunSinglePositionInPlaceTurnAuditPass()` is utility-specific but is not called by `RunPositionAccuracyAuditPasses()`. It is not part of the minimal required move set and should not be copied forward blindly.

### Unused helper

- `MissionModeController.cpp:460-463`

`ServiceTelemetryLog()` is utility-related but currently unused.

## Vocabulary / Convergence Notes

- The public boot-selected owners should remain `MissionRunMode`, `CorridorRepeatabilityMode`, and `PositionAccuracyAuditMode`.
- A dedicated audit implementation owner should read as a private utility-side owner for audit `Routine`s, not as another public mode surface.
- The audit split should not introduce more `BeginXMode()` / `RunXMode()` surface area than necessary.
- The deeper step-3 convergence still points toward shared routine extraction for startup/calibration and search/mapping execution rather than another copied controller body.

## Verification

No files were edited in this audit pass before writing this handoff note.

No build or tests were run in this pass.
