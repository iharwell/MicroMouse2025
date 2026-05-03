# IApplicationMode

## Fundamental rule

Do not answer the `LoopController` cleanup by widening `IApplicationMode` into a new wrapper layer.

## Current dependency

Many modes currently use their own `Run()` implementation as a place to manually sequence `LoopController::BeginSession(...)`, `Run()`, and `EndSession()`.

## Final-form changes required

- Keep `IApplicationMode` small.
- Stop treating individual mode implementations as the place that owns ordinary `LoopController::Run()` execution.
- Restructure loop-backed modes so infrastructure owns the terminal `Run()` boundary and modes own the callback logic, session-boundary requests, and staged successor-session state only where appropriate.
- Do not hide the cleanup inside a new mode/session abstraction here.

## Why minimal edits are toxic

If `IApplicationMode` becomes a wrapper over the current caller-owned loop model, the same architecture survives behind a different name.

## Dependencies and risks

This file is not the destination for the cleanup; the real work is in the mode implementations and the infrastructure boundary that already exists.
