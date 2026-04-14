# LoopController Design Follow-Ups

These notes capture remaining implementation ambiguities in
[codex_loop_controller_target_design.md](/C:/Users/thene/source/repos/MicroMouse2025/codex_loop_controller_target_design.md:1)
that should be resolved before treating the design as a fully frozen handoff document.

## 1. Current-Tick Timing Visibility During `onModeWork(...)`

Status: resolved in
[codex_loop_controller_target_design.md](/C:/Users/thene/source/repos/MicroMouse2025/codex_loop_controller_target_design.md:269).

The original ambiguity was about what timing data was visible while
`ModeCallbacks::onModeWork(...)` was running.

Relevant sections:

- the earlier draft passed a current-tick timing view into `onModeWork(...)`,
- the current tick is only fully finalized and published later,
- `LastDiagnostics()` is the public getter for the published slot.

The design now explicitly states:

- the diagnostics buffer is framebuffer-style,
- external readers only see the previous completed tick's timing until the current tick is finalized and published,
- the callback receives a deadline primitive rather than a reference to the in-progress timing buffer.

## 2. Pause/Stop Precedence Over Buffered Log Service

Status: resolved in
[codex_loop_controller_target_design.md](/C:/Users/thene/source/repos/MicroMouse2025/codex_loop_controller_target_design.md:460).

The original ambiguity was about what happens if mode work requests:

- pause,
- end-of-session,
- runtime stop/fault escalation.

The design now explicitly states:

- pause/end/stop/fault outcomes are latched rather than immediate mid-tick preemption,
- normal buffered-log servicing is still discretionary and uses only pre-sync slack on the normal cadence path,
- fault-path log flush/export is priority work,
- controlled stop/brake behavior must be preserved rather than bypassed by an early exit.

## 3. Callback Lifetime and Ownership

Status: resolved in
[codex_loop_controller_target_design.md](/C:/Users/thene/source/repos/MicroMouse2025/codex_loop_controller_target_design.md:423).

The original ambiguity was that `ModeCallbacks` uses raw function pointers and a `void* context`, and the document did not define the lifetime or ownership rules for those values.

The design now explicitly states:

- `BeginSession()` copies `ModeCallbacks` by value into loop-owned session state,
- `context` is intended to be the durable mode instance,
- callback targets and `context` must remain valid for the full active session,
- any pause request must carry a non-null pause callback through `RequestPause(...)`,
- and mode logic may retarget the next `onModeWork` callback to another phase function while keeping the same durable `context`.
