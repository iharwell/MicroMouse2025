# LoopController Implementation Worklog

## Scope

Implement the callback-based `LoopController` design from
[codex_loop_controller_target_design.md](/C:/Users/thene/source/repos/MicroMouse2025/codex_loop_controller_target_design.md:1)
without changing the control/sensor/UKF operation ordering.

## Constraints

- Preserve the sensitive interlaced control/sensor/UKF sequencing.
- Do not introduce compatibility wrappers once the authoritative shape is clear.
- Keep `SharedRobotRuntime` as the fault and logging authority.
- Keep normal per-tick flow as:
  1. control/sensor/UKF block
  2. mode work
  3. conditional log servicing if slack allows
  4. one final sync wait

## Audit Notes

- Current `LoopController` still exposes `RuntimeBundle`, `SessionConfig`, `IMode`, `RunOneTick()`, and `RunOneTickWithCallback()`.
- Current implementation still has two wait/service phases: `WaitForTickBoundaryAndService()` at tick start and a second slack/wait loop at tick end.
- Current modes (`OpenFloorMeasurementController`, `DiagnosticController`, `AuxMeasurementController`, `FrontWallCharacterizationController`) all still use `RunOneTickWithCallback(...)` with lambdas.
- Open-floor still stamps `controlEndUs`, `pwmLatchUs`, and `cycleCounterEnd` itself instead of consuming finalized loop-owned timing.
- Existing modes read the following old `VehicleState` fields:
  - `sequence`, `tickStartUs`, `dtUs`, `dtSeconds`
  - `estimate`
  - `diagnosticSensors`
  - `driveTelemetry`
  - `estimatorHealthy`
  - `faultReason`

## Planned Sequence

1. Refactor `LoopController` header/public API to the callback-based session model.
2. Rebuild `LoopController` internals around:
   - private runtime binding,
   - double-buffered `TimingDiagnostics`,
   - projected `ModeState`,
   - latched control-flow outcomes,
   - one final sync wait per tick.
3. Update `SharedRobotRuntime`/runtime tie-in plumbing to support the new loop owner shape.
4. Migrate non-open-floor controllers from `IMode`/`RunOneTickWithCallback(...)` to durable-instance callback thunks.
5. Migrate open-floor to phase-callback retargeting and loop-owned timing diagnostics.
6. Update design docs to match code and verify with release build/tests.

## Progress

- [x] Design note reviewed and tightened before coding.
- [x] Current loop/runtime/mode dependencies audited.
- [ ] `LoopController` API refactor in code.
- [ ] Runtime binding cleanup.
- [ ] Mode migrations.
- [ ] Verification.

## 2026-04-13 Takeover Checkpoint

- Compile-stabilized the callback/session refactor without reintroducing `RunOneTick`:
  - fixed `LoopController` diagnostic-snapshot bridging so it no longer fabricates observation-validity fields that the diagnostic pipeline does not own,
  - fixed `FrontWallCharacterizationController` runtime-fault latching so its phase-session result handling matches the other migrated controllers,
  - finished migrating the stranded `DiagnosticController` phase lambdas onto durable phase tick methods,
  - fixed `OpenFloorMeasurementController` namespace-scope member definitions that were incorrectly using the class-local `LoopController` alias in their out-of-class signatures.
- Verified the current tree with the approved release flow:
  - `build_and_verify_latest.cmd --no-pause` succeeded on 2026-04-13,
  - Teensy firmware compiled,
  - host `Release|x64` build completed,
  - release unit tests passed,
  - verification log: `codex_verify/logs/build_and_verify_latest_20260413_222008_055.txt`.
- Current spec/status assessment:
  - the tree is buildable and release tests pass,
  - but the controller migrations are still using per-phase `RunPhaseSession(...)` boundaries that end and restart the loop between phases,
  - that is now known to be an intermediate checkpoint rather than the acceptable final shape.
- Important design constraint re-confirmed before compaction:
  - `codex_loop_controller_target_design.md` explicitly says the durable mode `context` should stay alive for the full active session and that mode logic may retarget `onModeWork` with `services.SetNextModeWorkCallback(...)`,
  - user clarification during takeover: ending the loop interrupts state significantly; swapping callbacks without ending is preferred generally.
- In progress at compaction point:
  - refactoring away from per-phase session endings toward continuous in-session callback handoff,
  - first target under review is `OpenFloorMeasurementController` because it is the most timing-sensitive consumer of continuous loop state,
  - open question already identified from the design doc: open-floor likely needs callback retargeting plus the explicit pause path for non-periodic transitions such as switching from timing-log capture to main-log capture without silently leaving the loop.

## 2026-04-13 Continuous-Session Progress Checkpoint

- `OpenFloorMeasurementController.cpp` now runs its full open-floor workflow inside one `LoopController` session instead of restarting the loop between phases:
  - timing capture now hands off to a dedicated bridge tick and then uses the explicit pause path only for the timing-log to main-log transition,
  - static hold, launch, straight, yaw, smooth, and loop sections now hand off by retargeting the controller-owned phase callback inside the same session,
  - the old per-phase `RunPhaseSession(...)`, `RunTimingBlock(...)`, `RunStaticSection(...)`, `RunLaunchSection(...)`, `RunStraightSection(...)`, `RunYawSection(...)`, `RunSmoothSection(...)`, `RunLoopSection(...)`, `TraverseToMarker(...)`, and `RecoverToMarker(...)` wrappers were removed from open-floor.
- `FrontWallCharacterizationController.cpp` now also uses one continuous loop session:
  - startup hold transitions directly into reverse capture,
  - the controller uses one explicit pause only for the non-periodic persist/export work after capture completes,
  - post-capture settle resumes in the same session and then ends the loop only once at final completion.
- Verification after those two controller refactors:
  - `build_and_verify_latest.cmd --no-pause` succeeded again on 2026-04-13,
  - verification log: `codex_verify/logs/build_and_verify_latest_20260413_232847_332.txt`.
- Current in-progress work before the next compaction:
  - `AuxMeasurementController.cpp` is mid-refactor from per-phase `RunPhaseSession(...)` to one routine-owned session,
  - the new shape under edit is a controller-owned routine-step sequencer that advances hold phases and the turning-traction sweep by callback handoff inside one session,
  - `DiagnosticController.cpp` has not yet been converted away from per-phase session endings in this checkpoint.

## 2026-04-13 Additional Stable Checkpoint

- `AuxMeasurementController.cpp` now runs each selected routine inside one continuous `LoopController` session:
  - the fan static survey now advances between startup settle, fan-off baseline, fan-on hold, and recovery holds by controller-owned step handoff instead of restarting the loop per hold,
  - the turning-traction routine now advances from startup settle to fan spinup to the traction sweep inside one session and only ends the loop at the final result-producing sweep completion.
- Verification after the aux-controller refactor:
  - `build_and_verify_latest.cmd --no-pause` succeeded again on 2026-04-13,
  - verification log: `codex_verify/logs/build_and_verify_latest_20260413_234917_816.txt`.
- Remaining spec gap after this checkpoint:
  - `DiagnosticController.cpp` still uses per-phase `RunPhaseSession(...)` boundaries and remains the only known controller in this migration slice that still ends and restarts the loop between phases instead of handing off inside one continuous session.

## 2026-04-14 Diagnostic Migration Kickoff

- Re-read the current target design before editing `DiagnosticController.cpp`.
- Design lines driving this refactor:
  - `codex_loop_controller_target_design.md:454-457`
    - `BeginSession()` copies callbacks by value and the callback targets plus durable mode `context` must remain valid for the full active session.
  - `codex_loop_controller_target_design.md:459-463`
    - mode logic should advance between phase handlers without reconfiguring the whole session.
  - `codex_loop_controller_target_design.md:507-510`
    - `TickServices` is limited to end-of-session, pause, next-callback retargeting, and runtime-stop forwarding.
  - `codex_loop_controller_target_design.md:539-542`
    - leaving the strict periodic loop must be explicit rather than happening through silent caller-driven gaps between ticks.
- Current `DiagnosticController.cpp` audit:
  - `Run()` still sequences the diagnostic schedule by repeatedly calling `HoldPhase(...)`, `ExecuteStraightPhase(...)`, `ExecuteTurnPhase(...)`, `ExecuteArcPhase(...)`, and characterization helpers that each invoke `RunPhaseSession(...)`.
  - That means the strict loop is still ending and restarting between every diagnostic phase instead of remaining active for the full scenario.
- Intended convergence for this task:
  - keep `LoopController` untouched unless a code/spec mismatch is proven,
  - migrate `DiagnosticController` onto one durable session,
  - keep the mode as the owner of the diagnostic schedule, metrics, summaries, and schema,
  - move phase transitions to controller-owned in-session state handoff instead of per-phase session restarts.

## 2026-04-14 Diagnostic Continuous-Session Checkpoint

- `MazeMap/MazeMap/DiagnosticController.cpp` now starts one `LoopController` session in `Run()` and drives the full diagnostic scenario through controller-owned `ScenarioStep` handoff.
- Removed the per-phase `RunPhaseSession(...)` restart path from `DiagnosticController`.
- The controller now keeps the diagnostic schedule inside one durable mode instance and advances between:
  - hold phases,
  - kickoff and forward characterization samples plus recovery,
  - turn sweeps,
  - short/long straight sequences,
  - circle sweeps,
  - square loops,
  - final idle.
- Result/event emission stayed mode-owned:
  - straight/turn/arc/circle/square result summaries are still written by `DiagnosticController`,
  - characterization result events still stay in the mode,
  - the mode still owns the diagnostic schema and sample logging.
- `LoopController` was not modified in this slice because the controller migration could be completed using the existing durable-session contract without finding a code/spec mismatch that required loop changes.
- Next step: verify the current source against the repo’s release build/test flow after checking that the active artifacts are not stale relative to this edit.
- First verification run after the refactor failed in Teensy compile with a local refactor mistake:
  - `_pendingReturnDistanceM` was added to the constructor and phase logic but the member declaration was missing.
  - Fix applied immediately in `DiagnosticController.cpp`; rerun of the same approved build-and-verify flow is next.
- Second verification attempt behavior:
  - Teensy compile completed and the host `MazeMap.dll` timestamp advanced,
  - but the `build_and_verify_latest` flow stopped emitting log output immediately after launching the host `msbuild` command,
  - and the spawned host-build `cmd` process remained resident long past the expected step duration.
- Recovery action in progress:
  - terminate that stuck verification process,
  - rerun the same approved `build_and_verify_latest.cmd --no-pause` flow once with a longer timeout to distinguish a real hang from a slow host build/test phase.
- Final verification result for this slice:
  - reran `build_and_verify_latest.cmd --no-pause` with a longer timeout,
  - Teensy compile succeeded,
  - host `Release|x64` build completed,
  - release unit tests completed successfully,
  - verification log: `codex_verify/logs/build_and_verify_latest_20260414_005542_079.txt`.
- Observed verification nuance:
  - the host release build step was much slower than expected in this environment but did complete successfully when allowed to run long enough,
  - no `LoopController` source changes were required to complete the `DiagnosticController` migration.
