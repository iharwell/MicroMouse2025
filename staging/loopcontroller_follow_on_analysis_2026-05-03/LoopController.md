# LoopController

## Fundamental rule

The fix is subtraction plus consumer redesign. Minimal-churn cleanup is poisonous because it would preserve the same invalid capabilities behind smaller names.

## Current dependency

The commented problems point to invalid public capabilities:

- `TickServices`
- `PauseDisposition`
- `PauseRequest` policy fields
- implicit callback-context carry-forward
- public runtime-stop result semantics

The current class also muddles three different lifecycle concepts:

- pausing with continuity
- ending a session while remaining inside loop infrastructure
- ending the entire loop/program by returning from `Run()`

## Final-form changes required

- Remove the invalid public capability surface from `LoopController`.
- Keep legitimate loop ownership centralized here:
  - cadence
  - tick timing
  - command application
  - active callback dispatch
  - sensing as presently wound into the loop
- Make the direct control surface express the intended lifecycle split:
  - `RequestPause(...)`
  - `RequestEndSession(...)`
  - `StageNextSessionState(...)`
  - a clearly terminal whole-program end request, conceptually closer to `HaltExecutionEndProgram` than the current `RequestEndLoop()`
- Treat returning from `Run()` as whole-program termination inside infrastructure, not as ordinary consumer control flow.

## Why minimal edits are toxic

If the same behaviors survive behind wrappers, helper objects, or renamed methods, the architecture does not improve. The poison is the consumer assumption that those capabilities are valid at all.

## Dependencies and risks

- Every `TickServices` consumer needs real control-flow redesign, not a rename.
- Pause-heavy modes are the highest-risk migration sites.
- Session-handoff users need explicit `RequestEndSession` / `StageNextSessionState` design instead of pause abuse.
- Shared routine owners need explicit callback-transfer cleanup once implicit context carry-forward is gone.
