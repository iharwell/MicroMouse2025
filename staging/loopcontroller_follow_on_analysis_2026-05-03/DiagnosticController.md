# DiagnosticController

## Fundamental rule

Rewrite this file directly to the final narrower `LoopController` contract. A staged approach would be especially toxic here because the helper graph is large enough to preserve the old model almost unchanged.

## Current dependency

- `Run()` open-codes the session lifecycle.
- The diagnostic tick path and many subordinate helpers depend on `services.Fault(...)`.
- Scenario completion depends on `services.RequestEndLoop()`.
- The controller legitimately reads current tick timing for logs and control calculations.

## Final-form changes required

- Preserve the current timing reads.
- Replace every `services.Fault(...)` site with direct `SharedRobotRuntime::FailActiveMode(...)` or a local helper that ends there immediately.
- Keep scenario completion as terminal loop/program end semantics, but through the final direct loop-owned request rather than `TickServices`.
- Rewrite the helper graph so callbacks use direct `LoopController` methods instead of `TickServices`.
- Remove the current mode-local ownership of the terminal `LoopController::Run()` boundary.

## Why minimal edits are toxic

Local adapters would keep loop faulting and completion as ambient helper capabilities across the diagnostic scenarios. That is the architecture being removed.

## Dependencies and risks

This is one of the largest migration sites because many helpers assume loop-fault helper availability today.
