# LoopController Follow-On Analysis

## Fundamental rule

- Any recommendation optimized for minimal alterations is poisonous.
- These findings assume direct final-form restructuring around a simpler `LoopController`.
- Do not preserve the old model through wrappers, compatibility shims, or staged caller patches.
- Do not externalize `LoopController` behavior unless an already-existing destination can take it without needing any destination changes.
- Existing direct destinations relied on here are:
  - direct `LoopController` methods for loop control
  - direct `SharedRobotRuntime::FailActiveMode(...)` for mode faulting
- Sensors may remain wound into `LoopController`; that is not the problem being solved here.

## Lifecycle split used by these findings

- `pause`: continuity preserved
- `request end session`: continuity intentionally broken, but execution remains inside loop infrastructure
- `stage next session state`: prepare the starting state of the next session that will run after the end-session callback returns
- terminal loop end: whole-program termination by returning from infrastructure-owned `Run()`

The current names do not express that clearly. These findings therefore use:

- `RequestEndSession` as the target meaning for the present `EndSession` concept
- `StageNextSessionState` as the target name for staging the successor session
- `HaltExecutionEndProgram` as the target meaning for what the current `RequestEndLoop()` actually does when it causes `Run()` to return

## Scope

Each class file describes what must change once invalid `LoopController` capabilities such as `TickServices`, overpowered pause APIs, implicit callback-context carry-forward, and public runtime-stop result handling are removed, while honoring the lifecycle split above.
