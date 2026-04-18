# Execution Model Guide

This guide records the intended `LoopController` execution model for boot modes, mode phases, and shared routines.

Use this document together with [AGENTS.md](AGENTS.md) and [project_vocabulary.md](project_vocabulary.md). If older code or notes imply a different model, prefer this guide.

## Core Model

- A boot-selected mode owns the active `LoopController` session for the boot.
- A boot cycle normally gets one `LoopController` session.
- A second `LoopController` session is allowed only when the vehicle state is known to be discontinuous, such as after user service or a physical lift that invalidates UKF continuity.
- The mode defines the phase boundaries for its workflow.
- A routine is a reusable callback-driven behavioral block that a mode uses while constructing a phase.
- When a mode hands control to a routine, that routine owns the active `LoopController` callback for its entire execution.
- The mode does not get ordinary control back until the routine completes or faults.
- Because of that callback ownership rule, a routine does not span phases.

## Ownership Rules

### Mode ownership

- Start and end the top-level `LoopController` session.
- Do not start a second `LoopController` session for an individual phase, helper, or routine inside the same boot cycle unless the vehicle state is known to be discontinuous and the new session is intentionally re-establishing a valid estimator context.
- Choose phase boundaries, labels, sequencing, and policy decisions.
- Decide when to install a routine and which continuation callback receives control after routine completion.
- Own mode-specific logging labels, artifacts, and result interpretation.

### Routine ownership

- Implement one reusable callback-driven behavior block.
- Consume every control tick from launch until completion.
- Return control through the continuation callback supplied by the mode or shared owner that launched it.
- Stay non-blocking: if a routine does not explicitly pause `LoopController`, it must return before the active tick deadline.

### Shared execution owner ownership

- Shared execution owners such as `ManeuverExecutor` may host reusable routines.
- They may own reusable per-tick state, continuation plumbing, and shared drive behavior.
- They must not invent separate phase systems or claim ownership of mode phase semantics.

## What A Phase Is

- A phase is a dynamic execution block inside one mode.
- Phases are mode concepts.
- A phase may be built from direct mode callbacks, one shared routine, or several routine invocations separated by mode callbacks.
- The infrastructure may record phase transitions, but it should not need to understand what a phase means semantically.

## What A Routine Is

- A routine is not a blocking wrapper.
- A routine is not a phase.
- A routine is not a hidden boot-mode selector.
- A routine is not a second top-level session.
- A routine is a modular callback-owned behavior block used by a mode while the mode is inside a phase.

## Callback Handoff

Typical control flow looks like this:

1. The mode callback runs inside the active `LoopController` session.
2. The mode decides to enter or continue a phase.
3. The mode installs a routine and supplies a continuation callback.
4. The routine receives all subsequent ticks.
5. The routine completes and returns control through the supplied continuation callback.
6. The mode resumes and decides the next phase step.

That means the mode cannot keep doing other phase work in parallel while the routine is active. If something must happen during motion, it must live in the active routine or in shared services called by that routine's callback.

A routine entrypoint is non-blocking. It uses `SetNextModeWorkCallbacks(...)` or `SetNextModeWorkCallback(...)` to transfer control to the routine work callback, stores the continuation callback it should restore on completion, and then returns.

## Pauses

- `LoopController::RequestPause(...)` is the only sanctioned way to break strict callback cadence for non-real-time work.
- Pauses exist for blocking, no-motion calculation, and other non-real-time work that cannot be completed inside the active control tick.
- A pause does not return ordinary control to the mode callback; it is a sanctioned interruption inside the same callback-owned execution flow.
- If the loop is paused, the robot must not move.
- `LoopController` must not be paused, started, or stopped unless the vehicle is stationary or the runtime is already faulting.
- Callback setters are the control-transfer mechanism; pauses are not a second callback-routing API.
- Do not sleep, spin, or wait for routine completion outside pause handling.

## Disallowed Patterns

- Synchronous or blocking `Run*Routine(...)` wrappers.
- Starting a fresh `LoopController` session around one routine or one phase inside a boot-selected mode without a known state discontinuity that justifies a new estimator context.
- A routine API that accepts or owns phase labels as though phases were routine concepts.
- A mode callback that expects to regain control before the launched routine completes.
- Nested `LoopController` sessions started by subordinate helpers or controllers.
- A shared execution owner that creates its own parallel mode machine.

## Design Test

Before adding or refactoring a routine, confirm all of the following are true:

- The mode still owns the phase boundary and decides when the routine begins.
- The routine will own every tick until it returns through its continuation callback.
- Nothing outside the active routine expects to advance during that time.
- The routine API is expressed as reusable behavior, not as a phase scheduler.
- The design preserves the single active `LoopController` callback chain.

## Concrete Mental Model

Think of a phase as a mode-level chapter and a routine as one reusable block the mode can install while writing that chapter.

Once the mode hands the callback stream to the routine, the routine owns the whole chapter fragment until it returns control. The mode can then decide whether the current phase is complete, whether another routine is needed inside the same phase, or whether it should advance to the next phase.
