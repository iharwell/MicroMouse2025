# OpenFloorMeasurementController

## Fundamental rule

This controller must be rebuilt directly to the final narrower `LoopController` contract. Do not preserve its current pause-heavy control flow behind a translation layer.

## Current dependency

- `Run()` duplicates the session lifecycle.
- The controller is deeply coupled to the rejected pause surface: `PauseRequest` policy fields, `services.RequestPause(...)`, and `PauseDisposition::StopByRuntime(...)` / `Resume()`.
- The controller also faults through `services.Fault(...)`.
- It legitimately depends on loop timing diagnostics for measurement logs.
- The current timing-to-main handoff requests pause and asks for a clock reset before resuming the same session.

## Final-form changes required

- Stop modeling the timing-to-main boundary as a pause-with-resume case.
- Rebuild that boundary as an explicit session break:
  - request end session
  - stage the next session state through `StageNextSessionState`
  - let the next session begin after the end-session callback returns
- Remove pause reasons, flush policy, reset-clock policy, and pause terminal actions from this handoff.
- Replace failure during handoff with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Preserve timing diagnostics and current tick timestamp reads.

## Why minimal edits are toxic

This controller built a real session-boundary problem on top of invalid pause capabilities. A wrapper-based cleanup would keep that invalid contract alive in practice.

## Dependencies and risks

This is a high-risk cleanup because the timing-to-main transition is the clearest current case where session discontinuity and pause continuity are being conflated.
