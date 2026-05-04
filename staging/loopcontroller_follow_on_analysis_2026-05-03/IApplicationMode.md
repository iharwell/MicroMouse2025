# IApplicationMode

## Fundamental rule

Do not answer the `LoopController` cleanup by widening `IApplicationMode` into a new wrapper layer.

## Current dependency

Many modes currently use their own `Run()` implementation as a place to manually sequence `LoopController::BeginSession(...)`, `Run()`, and `EndSession()`.

## Final-form changes required

- Keep `IApplicationMode` small.
- Replace the current top-level mode contract with:
  - `SetupMode()` for pre-loop preparation
  - `RunTick(...)` for the active per-tick mode callback
- Make `RunTick(...)` the uniform initial callback for every mode.
- Make the mode object itself the uniform initial callback context.
- Stop treating individual mode implementations as the place that owns ordinary `LoopController::Run()` execution.
- Restructure loop-backed modes so infrastructure owns the terminal `Run()` boundary, wires `RunTick(...)` to `LoopController`, and modes own preparation, per-tick logic, session-boundary requests, and staged successor-session state only where appropriate.
- Do not hide the cleanup inside a new mode/session abstraction here.

## Why minimal edits are toxic

If `IApplicationMode` becomes a wrapper over the current caller-owned loop model, the same architecture survives behind a different name. The point is to collapse startup divergence, not to standardize it behind one more layer.

## Dependencies and risks

This file carries the startup contract, but the real work is still in the mode implementations and the infrastructure boundary that already exists. Adopting `SetupMode()` plus `RunTick(...)` also implies that the current mode-local `Run()` implementations should disappear.
