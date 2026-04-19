# Mode Service Refactor Progress

Date: 2026-04-18

## Task

Refactor all mode implementations against the shared `StartupCalibration`, `WallTouch`, and `Drive` services without modifying those three classes. Stop condition per mode: clean audit against the service APIs, `AGENTS.md`, project vocabulary, and the callback-owned `LoopController` execution model.

User clarification: noncompliant modes should generally be rewritten because the project has substantial architectural drift.
User clarification: pimpl setups are not acceptable in this embedded project; do not use dynamic-allocation ownership indirection or pimpl-style compile-time escape hatches.
User clarification: modes do not own fault handling. `SharedRobotRuntime` owns the fault path; mode-registered callbacks may perform teardown only.
User clarification: a real hold uses `Drive` hold semantics. Repeatedly returning `ControlVector::Brake` for a duration is not a `Drive` hold and must not be named or treated as one.

## Crash Recovery / Compaction Checklist

1. Reread the latest user prompt.
2. Reread `AGENTS.md`.
3. Reread this document.
4. Reconfirm the protected service-owner constraint:
   - do not modify `MazeMap/MazeMap/StartupCalibration.*`
   - do not modify `MazeMap/MazeMap/WallTouch.*`
   - do not modify `MazeMap/MazeMap/Drive.*`
5. Resume mode work from the status table below.

## Audit Criteria

- Modes own top-level boot selection behavior only through `BootModeRegistry` and their descriptors.
- Modes own the active `LoopController` session and callback chain for that boot.
- Shared multi-tick startup work goes through `StartupCalibration`.
- Shared multi-tick wall-seating / wall-contact work goes through `WallTouch`.
- Shared multi-tick ordinary motion primitives go through `Drive`.
- No mode-local replacement layers, wrapper owners, duplicate logging systems, or alternate motion/state machines remain.
- If a mode does not need one of the three services, the audit still confirms that the mode does not reimplement that service's responsibility.

## Mode Inventory

| Mode | File | Status | Notes |
| --- | --- | --- | --- |
| FrontWallCharacterization | `MazeMap/MazeMap/FrontWallCharacterizationController.cpp` | auditing | Uses `StartupCalibration` and `Drive`; needs deeper audit for any remaining duplicated mode-local execution logic. |
| WallSensorLedCalibration | `MazeMap/MazeMap/WallSensorLedCalibrationController.cpp` | rewritten | Clean boot-mode shape confirmed. No duplicated `StartupCalibration` / `WallTouch` / `Drive` responsibility; mode only owns LED toggling, selector monitoring, and session teardown. |
| AuxiliaryMeasurement | `MazeMap/MazeMap/AuxMeasurementController.cpp` | rewritten | Rewritten around shared startup and `Drive` hold phases. Turning sweep is now just a `DriveBase` point-command schedule with offline analysis; legacy traction-loss heuristics were pulled out of compiled mode and parked below for reference. |
| CorridorRepeatability | `MazeMap/MazeMap/CorridorRepeatabilityMode.cpp` | rewritten | Local service start/poll wrappers removed; mode now drives `StartupCalibration`, `WallTouch`, and `Drive` directly from the active callback. |
| PositionAccuracyAudit | `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp` | rewritten | Uses shared startup, `WallTouch`, and `Drive` directly in the active callback; mode-owned fault-state bookkeeping was removed in favor of runtime teardown callbacks. |
| ManeuverFileTest | `MazeMap/MazeMap/ManeuverFileTestMode.cpp` | auditing | Uses `StartupCalibration` and `Drive`. |
| TopSpeedMeasurement | `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp` | rewritten | Uses shared open-floor startup and `Drive`; mode-owned fault bookkeeping was removed in favor of runtime teardown callbacks. |
| PrimaryDiagnostic / OpenFloorMeasurement | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | rewritten | Uses shared open-floor startup and `Drive`; selector cleanup now runs through a runtime teardown callback instead of mode-owned fault state. |
| Mission | `MazeMap/MazeMap/MissionRunMode.cpp` | rewritten | Reduced to the clean shared startup path with a real `Drive` hold and no mode-owned fault state. |

## Early Observations

- `BootModeRegistry` currently exposes 7 active selector entries, but there are 9 mode implementation files because the auxiliary selector may resolve to `AuxiliaryMeasurement`, `CorridorRepeatability`, or `PositionAccuracyAudit`.
- `StartupCalibration`, `WallTouch`, and `Drive` are explicitly designed as subordinate shared multi-tick services that stay under mode callback ownership.
- The strongest clear mismatch so far is `AuxMeasurementController.cpp`, which still drives behavior through direct `DriveBase` access and a legacy mode-local execution style.
- Several newer modes already follow the target pattern and may only need simplification or deletion of residual duplicated logic.

## Parked Legacy Turning-Traction Heuristic

Source: pre-refactor `HEAD` version of `MazeMap/MazeMap/AuxMeasurementController.cpp`. Removed from compiled mode on 2026-04-18 so the active auxiliary traction sweep stays a clean `DriveBase` point-command schedule with offline analysis.

- Launch assist stage: for `kTurningTractionLaunchMs`, the legacy mode returned raw PWM from `ComputeTurningLaunchCommands(...)`.
- Speed-ramp stage: increase commanded speed by `kTurningTractionSweepAccelMps2` until actuator saturation plateaus.
- Plateau-to-curvature transition: when the outer wheel command stayed near `kTurningTractionActuatorCeilingCommand` without at least `kTurningTractionPlateauDeltaMps` speed gain over `kTurningTractionPlateauWindowMs`, the legacy mode held speed and tightened curvature by `kTurningTractionCurvatureRampMInvPerSec`.
- Traction-loss inference: compute `TurningTractionMetrics` with `ComputeTurningTractionMetrics(...)`, then call `IsTurningTractionLossDetected(...)` using the `kTurningTractionSlip*` thresholds and require `kTurningTractionSlipConfirmMs` of sustained mismatch.
- Legacy result event: the compiled mode used to emit `traction_limit_result` text rows for `reason=traction_loss` or `reason=timeout`, plus a `turning_traction_mode` text row when it switched into the curvature-tightening stage.
- Legacy angular-command helper: `ComputeTurningTractionAngularCommand(...)` added local heading/yaw correction on top of the nominal circle-rate request.

## Work Log

- 2026-04-18 22:11 CDT: Inventoried boot registry, descriptors, protected service APIs, and current mode files.
- 2026-04-18 22:11 CDT: Confirmed clean worktree before starting edits.
- 2026-04-18 22:18 CDT: User clarified that noncompliant modes should generally be rewritten rather than preserved.
- 2026-04-18 22:25 CDT: User clarified that pimpl patterns are not acceptable in this embedded codebase.
- 2026-04-18 22:34 CDT: User clarified that modes do not own fault handling; runtime-registered callbacks may perform teardown only.
- 2026-04-18 22:39 CDT: User clarified that `Drive` holds are distinct from repeated brake returns; audit remaining modes for false "hold" language/behavior drift.
- 2026-04-18 22:31 CDT: `WallSensorLedCalibrationController.cpp` rewritten into a clean boot-mode shape with explicit fault registration and idempotent teardown; no shared-service responsibility duplication found.
- 2026-04-18 22:32 CDT: `CorridorRepeatabilityMode.cpp` rewritten to drive shared services directly inside the active callback instead of through mode-local service wrappers.
- 2026-04-18 22:32 CDT: Worker build attempts report a current compile blocker in `MazeMap/MazeMap/FrontWallCharacterizationController.cpp` where `ModeWorkThunk` references missing `RunTick`.
- 2026-04-18 22:30 CDT: Pulled the legacy auxiliary turning-traction loss heuristic out of compiled mode and parked its shape in this document for later reference.
- 2026-04-18 22:46 CDT: Touched `MazeMap/MazeMap/MissionRunMode.cpp` so the host `x64 Release` build would regenerate a stale `MissionRunMode.obj` that had previously been compiled as `x86`.
- 2026-04-18 22:48 CDT: `build_and_verify_latest.cmd --no-pause` completed the Teensy compile and host `Release|x64` build successfully after the stale `MissionRunMode.obj` issue was corrected.
- 2026-04-18 22:49 CDT: Release unit tests ran through `MazeMapTest.dll`; 492 passed and 1 failed. The failing test was `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists` in `MazeMap/MazeMapTest/DriveBaseTest.cpp:406`, outside the edited mode files and outside the protected service-class constraint for this task.
- 2026-04-18 22:52 CDT: Reran `build_and_verify_latest.cmd --no-pause` after the final wall-sensor cleanup rename so verification matched the exact current sources. Teensy compile passed, host `Release|x64` build passed, and the same single Release unit test still failed: `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists` in `MazeMap/MazeMapTest/DriveBaseTest.cpp:406`.
- 2026-04-18 22:53 CDT: Updated `AGENTS.md` so audit failure is explicit when a substantive owner, top-level mode implementation, or any class/cohesive implementation block over 100 lines is kept file-local in a `.cpp` without a same-named authoritative header.
- 2026-04-18 23:06 CDT: Tightened the `AGENTS.md` rule again to remove the judgment word `substantive`: any non-template class, top-level mode owner, or file-local implementation block over 100 lines must be declared in a same-named authoritative header, implemented in a `.cpp`, and use the appropriate `EXPORT` macro where required. Template code remains the explicit exception.
- 2026-04-18 23:09 CDT: Removed the remaining `substantive` wording from the file/header policy so the 100-line header/`.cpp`/`EXPORT` rule is objective rather than judgment-based.
