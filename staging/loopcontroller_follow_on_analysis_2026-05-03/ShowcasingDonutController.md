# ShowcasingDonutController

## Fundamental rule

Rewrite this controller directly to the final narrower `LoopController` contract. Do not keep the current phase machine alive behind renamed fault/completion helpers.

## Current dependency

- `Run()` open-codes the session lifecycle.
- The controller legitimately reads current tick timing for control and logging.
- The active callback depends on `services.Fault(...)` and `services.RequestEndLoop()`.

## Final-form changes required

- Preserve the current timing reads.
- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep top-level completion as terminal loop/program end semantics, but through the direct final loop-owned request rather than `TickServices`.
- Rewrite the callback path so loop control goes through direct `LoopController` methods rather than `TickServices`.

## Why minimal edits are toxic

This controller mixes legitimate timing reads with illegitimate loop mutation helpers. A minimal rewrite would preserve that mixed model.

## Dependencies and risks

The log format depends directly on loop timing fields, so those reads must survive while the invalid control surface is deleted.
