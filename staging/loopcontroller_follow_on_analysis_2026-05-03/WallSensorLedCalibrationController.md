# WallSensorLedCalibrationController

## Fundamental rule

Rebuild this mode directly to the final narrower pause contract. Do not preserve its current “escape the loop through pause terminal actions” model behind helper wrappers.

## Current dependency

- `Run()` duplicates the session lifecycle.
- The controller immediately escapes through `services.RequestPause(...)`.
- The pause callback depends on `PauseDisposition::Complete()` and `PauseDisposition::StopByRuntime(...)`.
- The controller also depends on `services.Fault(...)`.

## Final-form changes required

- Keep the stop-motion-then-calibrate behavior inside the narrower direct `LoopController` pause contract.
- Remove `PauseDisposition::Complete()` and `PauseDisposition::StopByRuntime(...)`.
- After calibration succeeds, request terminal loop/program end through the final direct terminal loop-owned request.
- Fail by calling direct `SharedRobotRuntime::FailActiveMode(...)`.
- Remove dependence on `TickServices`.

## Why minimal edits are toxic

This mode is the clearest proof that pause became a general escape hatch. Any wrapper-based fix would preserve that architecture.

## Dependencies and risks

The rewrite must preserve deterministic hardware cleanup and the current low-sensor-work session options.
