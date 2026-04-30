# Drive.cpp Size-Reduction Audit

## Scope

- Target file only: `MazeMap/MazeMap/Drive.cpp`
- `MazeMap/MazeMap/Drive.h` public declarations and documentation are fixed and authoritative.
- Tests are secondary evidence only. They do not override the `Drive.h` contract.
- Any implementation variant that requires editing a file other than `Drive.cpp` is rejected for this task.
- Any implementation variant that shifts `Drive` behavior into `DriveBase`, `SharedRobotRuntime`, `Maneuver*`, a new helper file, or a new header is rejected for this task.

## Contract checkpoints

Every accepted option must preserve these documented behaviors from `Drive.h`:

1. Live configuration stays live across ticks. `SetOperationMode(...)`, `SetLimits(...)`, and `SetCommandPDSettings(...)` remain retained on `Drive` and are consulted on later `GetNextControls(...)` calls.
2. `Start...` calls are total instruction setters. Ill-posed numeric inputs do not fail the start, retain the old instruction, or install a different primitive.
3. Start-time sampling stays at start time. Pointer-supplied numeric inputs and any recovered retained instruction state must continue to be latched during the `Start...` call, not deferred to `GetNextControls(...)`.
4. `GetNextControls(...)` continues to evaluate retained instruction + live config + current runtime state each tick.
5. Ill-posed requests remain non-sticky. Later coherent requests must restore coherent behavior without reconstructing `Drive`.
6. Non-finite completion conditions keep the current documented behavior where `done == true` may still accompany continued motion proposals.
7. `SetLimits(...)` continues to retain the supplied `MotionLimits` verbatim and interpret them at use time rather than sanitizing them on assignment.

## Additional acceptance gate

A proposal is accepted only if its justification survives this stronger objection:

`You have not considered every edge-case family that affects the things you would change on this item.`

For this audit, that means:

1. The changed surface for the option must be identified narrowly.
2. Every semantic edge-case family crossing that surface must be named or reduced to an invariant argument.
3. If that proof is not tight enough, the option is rejected even if it still looks attractive on code-size grounds.

## Summary

| Option | Area | Verdict | Notes |
| --- | --- | --- | --- |
| 1 | Shared finite turn-to-heading path | Accept | Best size win and lowest behavior risk |
| 2 | Shared straight-exit completion path | Accept | Good size win, edge cases are well-bounded |
| 3 | Shared traveled-distance and settle helpers | Accept | Small but very safe |
| 4 | Collapse straight-start helper cluster | Reject | Too many affected start-time recovery families for the current proof burden |
| 5 | Compress quarter-turn and arc recovery logic | Reject | Too many malformed-input recovery families for the current proof burden |
| 6 | Shared primitive-arming helper | Accept | Mechanical reduction, low semantic risk |
| 7 | Split `ManeuverControls(...)` into more helpers | Reject as a size-reduction tactic | Likely reduces local visual bulk more than total code size |

## Detailed audit

### 1. Merge the duplicated finite turn-to-heading path

- Locations:
  - `Drive::TurnControls(...)` finite-turn branch in `Drive.cpp`
  - zero-distance turn branch inside `Drive::ManeuverControls(...)`
- Verdict: Accept
- Why it shrinks code:
  - Both paths repeat the same sequence: resolve target yaw, compute `remainingRad`, run `IsTurnComplete(...)`, compute feedforward yaw rate from `angularAccelRadps2`, and emit `PointControlVectorWithHeadingTarget(...)`.
- Externally visible behavior risks:
  - Low, as long as the helper takes the already-retained target inputs and does not change which caller supplies them.
- Edge cases to preserve:
  - `NaN` or non-finite yaw state still falls back through the same `FallbackFinite(...)` path.
  - Completed turn still returns `Brake`.
  - The same heading-target command path remains in use.
- Why these remain unchanged:
  - This option is a pure extraction of an already-duplicated block. The helper would receive the same already-resolved inputs that the two call sites use today.
  - The decision sequence stays the same: target yaw resolution, remaining-angle computation, completion check, feedforward yaw-rate computation, then heading-target emission.
  - No recovery timing changes. The retained turn target is still decided before this helper runs, exactly as it is now.
- Why this edge-case list is sufficient:
  - The behavioral surface of this option is narrow. It only touches the shared turn-to-heading subroutine, not start-time recovery, live limit retention, storage layout, or maneuver point tracking.
  - The listed cases cover the only decision pivots in the duplicated block:
    - non-finite measured yaw
    - completed vs incomplete turn
    - control-path selection
  - Saying "you haven't considered every edge case" is true in the abstract for the entire class, but not a useful objection here unless it identifies another edge-case family whose behavior depends on this exact duplicated block. No such additional family is exposed by this option.
- Rejected variants:
  - Reject any version that moves this logic into `Drive.h`, `DriveBase`, or another file.
  - Reject any version that changes finite-turn completion into a generic callback or policy object outside `Drive.cpp`.

### 2. Merge the duplicated straight-exit completion path

- Locations:
  - exit-speed branch in `Drive::LinearMotionControls(...)`
  - straight fallback branch in `Drive::ManeuverControls(...)`
- Verdict: Accept
- Why it shrinks code:
  - Both branches do the same completion policy split:
    - if exit speed is effectively zero, require stationary settling
    - otherwise, compare measured velocity against the desired terminal speed
- Externally visible behavior risks:
  - Low, provided the helper only computes completion and does not alter how desired speed is chosen.
- Edge cases to preserve:
  - Fan-duty-dependent stationary threshold
  - zero-exit-speed stationary completion
  - advisory `done` semantics only
  - no revocation of the retained instruction after completion
- Why these remain unchanged:
  - The proposed helper only centralizes the completion predicate after the desired terminal speed has already been selected by the surrounding path.
  - The same inputs still drive the result: current wheel speeds, gyro rate, fan-duty-scaled settle threshold, current state velocity, exit speed magnitude, and `Config` tolerances.
  - The helper does not change command generation or retained state. It only answers the same "is this complete?" question in one place instead of two.
- Why this edge-case list is sufficient:
  - This option does not alter how straight or maneuver fallback speed is computed, how limits are interpreted, or how malformed inputs are recovered. It only touches terminal completion semantics.
  - The listed cases cover the completion branch split completely:
    - stop-to-settle path
    - nonzero terminal-speed match path
    - advisory-only meaning of `done`
    - retained-instruction persistence after completion
  - The objection "you haven't considered every edge case" would matter only if another completion-affecting input exists in these branches. In the current code, it does not.
- Rejected variants:
  - Reject any version that merges the speed-selection logic too aggressively and changes when `desiredSpeedMps` is computed or clamped.

### 3. Add tiny shared helpers for traveled distance and stationary settling

- Locations:
  - traveled-distance calculations in linear, transition, arc, and maneuver control paths
  - stationary-settle checks in hold, straight completion, and maneuver straight completion
- Verdict: Accept
- Why it shrinks code:
  - This removes repeated `fabs(AverageDistanceMeters(...) - startDistanceM)` and repeated threshold plumbing for the same stationary predicate.
- Externally visible behavior risks:
  - Very low if the helpers are pure wrappers around the current expressions.
- Edge cases to preserve:
  - null `DriveBase` fallback behavior
  - current fan-duty scaling in `ResolveMotionSettleSpeedThresholdMps()`
  - same tolerance constants and same velocity sources
- Why these remain unchanged:
  - The proposed helpers are wrappers around existing expressions, not new policies.
  - Traveled distance still comes from the same `AverageDistanceMeters(...)` or `SafeAverageDistanceMeters(...)` source and the same absolute-difference computation.
  - Stationary settling still calls the same sensor-based predicate with the same thresholds and the same telemetry/sensor fields.
- Why this edge-case list is sufficient:
  - This option is intentionally mechanical. It does not change branch ordering, retained instruction state, recovery timing, or control emission.
  - The only externally meaningful risk is accidental substitution of a different source value or threshold. The listed cases cover those substitution hazards exactly.
  - "Not every edge case" is not a strong criticism here because the helper boundary is so small. If inputs, thresholds, and formulas are unchanged, the edge-case behavior is unchanged by construction.
- Rejected variants:
  - Reject any version that changes which telemetry fields or sensor fields feed the stationary decision.

### 4. Collapse the straight-start helper cluster into one higher-level start resolver

- Locations:
  - heading and target-position helpers used only by `Drive::StartStraight(...)`
- Verdict: Reject
- Why it shrinks code:
  - The current implementation is spread across several one-caller microhelpers. A single higher-level resolver for straight-start latching can remove top-level helper bulk.
- Externally visible behavior risks:
  - High enough to reject under the stricter acceptance gate. This area is sensitive because the contract requires start-time sampling and retained interpretation of pointer-based inputs.
- Edge cases to preserve:
  - runtime present vs absent during start-time capture
  - captured orientation non-finite fallback
  - zero-vector heading override falls back to captured heading
  - infinite heading components still act as directional hints
  - non-finite heading components that do not resolve to a direction
  - null vs non-null target position override
  - infinite target-position coordinates remain functional
  - projected target distance when one axis direction contribution is zero
  - projected target distance when infinite contributions disagree in sign
  - projected target distance rejection when current position inputs are non-finite
  - requested distance remaining authoritative when target-position projection is unavailable
  - explicit straight direction must remain preserved against signed limits
  - the retained straight instruction must be decided at `StartStraight(...)`, not re-derived every tick
- Why this edge-case list is sufficient:
  - The list identifies the affected start-time recovery families, which is exactly why this option is rejected.
  - The problem is not failure to notice the families. The problem is that too much of `StartStraight(...)` would be rearranged at once for the current proof to remain tight.
  - In particular, the combined surface would include heading recovery, target-position projection, fallback-to-requested-distance behavior, and direction preservation against signed limits. That is more semantic density than this audit is willing to accept for a code-size-only change.
- Rejected variants:
  - Reject this option for the current task.
  - Reject any narrower variant that delays straight-target recovery until `GetNextControls(...)`.
  - Reject any narrower variant that adds a new class, header, or public struct.

### 5. Compress quarter-turn recovery and `RecoverArcGeometry(...)`

- Locations:
  - contextual quarter-turn recovery chain
  - `RecoverArcGeometry(...)`
- Verdict: Reject
- Why it shrinks code:
  - The current implementation repeats similar branch shapes for maze-based and sensor-based side-opening recovery, and `RecoverArcGeometry(...)` repeats nominal-radius and recovered-angle math across several descriptor cases.
- Externally visible behavior risks:
  - Too high for acceptance under the stricter gate. This is where many malformed-input edge cases live.
- Edge cases to preserve:
  - maze-mode-only contextual recovery
  - exact left/right/front tie-break ordering
  - deterministic choice when all direct inputs are ambiguous
  - finite requested turn angle bypassing contextual recovery entirely
  - infinite descriptors being treated as directional or unbounded hints rather than generic failure
  - degraded-but-coherent arc recovery when distance and/or curvature are `NaN` or infinite
  - descriptor-presence distinctions among distance-only, curvature-only, both-present, and neither-present requests
  - nominal-radius unavailability causing recovery failure instead of silent substitution
  - no substitution of a different primitive when recovery is incomplete
- Why this edge-case list is sufficient:
  - The list is sufficient to justify rejection.
  - It shows that this option touches multiple dense recovery families at once, and the code-size benefit is not large enough to justify the proof burden needed to show invariant behavior across all of them.
  - In other words, the objection "you have not considered every edge case that affects what you would change here" cannot be answered tightly enough for this option, so the option fails the acceptance gate.
- Rejected variants:
  - Reject this option for the current task.
  - Reject any attempt to move these heuristics into `Maze`, `DriveBase`, `ManeuverInstance`, or a separate recovery subsystem.
  - Reject any refactor that changes when a degenerate request becomes `done == true` versus `done == false`.

### 6. Add a shared file-local primitive-arming helper

- Locations:
  - repeated `ResetActivePrimitive(); placement-new; _activePrimitive = ...; _effectivelyComplete = false;` pattern across all `Start...` methods
- Verdict: Accept
- Why it shrinks code:
  - This is repeated mechanical scaffolding, and collapsing it removes real lines with little semantic coupling.
- Externally visible behavior risks:
  - Low if the helper is only a thin wrapper over the existing sequence.
- Edge cases to preserve:
  - same reset timing
  - same placement-new storage use
  - same `_effectivelyComplete` reset semantics
- Why these remain unchanged:
  - The helper would not own any recovery logic. It would only package the existing arm sequence after all primitive-specific retained-state values have already been computed.
  - That means the same primitive state bytes are still written, in the same storage, after the same reset point, with the same active-primitive tag and completion flag reset.
- Why this edge-case list is sufficient:
  - The helper boundary is purely mechanical. The only plausible behavior changes are ordering mistakes:
    - resetting too early or too late
    - constructing in the wrong storage
    - setting completion state differently
  - The list covers exactly those ordering hazards.
  - The criticism "you haven't considered every edge case" is weak here because this option does not introduce any new semantic branch surface. It only centralizes repeated scaffolding.
- Rejected variants:
  - Reject any version that changes the order of recovery work relative to `ResetActivePrimitive()` in methods where state is sampled before the reset.

### 7. Split `ManeuverControls(...)` into more file-local helpers

- Location:
  - `Drive::ManeuverControls(...)`
- Verdict: Reject as a size-reduction tactic
- Why it is rejected:
  - As phrased, this mainly shortens one function by adding more helper bodies elsewhere in the same file. That improves local readability, but it does not reliably reduce total code size.
  - The maneuver path already becomes smaller indirectly once options 1, 2, and 3 remove shared duplicated logic from it.
- Externally visible behavior risks:
  - Higher than the accepted options because this method mixes:
    - cached point-tracking state
    - zero-distance in-place turn fallback
    - non-point straight/curved fallback
    - finite and non-finite completion behavior
- Acceptable narrower variant:
  - Reuse accepted shared helpers from options 1, 2, and 3 inside `ManeuverControls(...)` without otherwise splitting the method into a family of new local helpers.
- Why this rejection remains justified:
  - This option increases the number of internal seams in the most behavior-dense method without a clear total line-count win.
  - The edge-case families mixed in this method are too numerous to justify helper splitting as a size-only change after the simpler shared extractions already exist.
- Why this edge-case list is sufficient:
  - The rejection does not require proving that every maneuver edge case has been enumerated. It requires showing that this option has poor size-reduction leverage relative to its semantic density.
  - The listed families are enough to prove that point:
    - point tracking state retention
    - in-place turn fallback
    - straight/curved fallback
    - finite/non-finite completion semantics
  - In other words, the burden here is not exhaustive edge-case cataloging. It is showing that the option is the wrong tool for the stated goal. The list is sufficient for that conclusion.
- Rejected variants:
  - Reject any version that degrades maneuver-native point tracking into generic straight/arc decomposition
  - Reject any version that moves maneuver execution policy outside `Drive.cpp`

## Explicit rejections for this task

These changes are out of scope and should be rejected immediately:

- Any edit to `Drive.h`
- Any helper class, new header, or new `.cpp` file
- Any attempt to move malformed-input recovery or primitive semantics into `DriveBase`
- Any attempt to move contextual maze/sensor recovery into `Maze` or another owner
- Any attempt to sanitize retained limits in `SetLimits(...)`
- Any attempt to shift start-time interpretation into `GetNextControls(...)`

## Why this audit does not need every theoretical edge case

The assertion "you haven't considered every edge case" is always literally available against any non-formal review of a class this large. That is not the right standard for these options.

The correct standard for this audit is narrower and stronger:

1. Identify the semantic boundary each option actually changes.
2. Identify the edge-case families that cross that boundary.
3. Prove that the option either:
   - leaves those families unchanged because it is a pure extraction of existing logic, or
   - is rejected because that proof would be too weak for the size benefit offered.

That is what the per-option lists above do.

For accepted options, the argument is invariance:

- same retained inputs
- same branch ordering
- same fallback rules
- same live configuration sources
- same control emission path

For rejected options, the argument is insufficiency:

- too much semantic density
- too little line-count benefit
- too much risk of accidentally changing retained-instruction or malformed-input behavior

So the right response to "you haven't considered every edge case" is:

- Correct, not every theoretical case in the whole class was enumerated.
- Yes, every edge-case family materially affected by each proposed option was considered.
- Where that proof is tight, the option is accepted.
- Where that proof is not tight enough for the payoff, the option is rejected.

Applied to this document, that stricter rule is why options 4 and 5 are now rejected rather than merely guarded.

## Recommended implementation order

1. Option 1
2. Option 2
3. Option 3
4. Option 6
5. Do not pursue options 4, 5, or 7 for this task

This ordering captures most of the likely line-count reduction first while minimizing risk to the contract-heavy malformed-input and retained-instruction behaviors.
