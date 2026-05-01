# Drive.cpp Short-Function Scope Audit

## Scope

- File: `MazeMap/MazeMap/Drive.cpp`
- Population: every `Drive.cpp` function or method estimated at fewer than 15 logical code lines
- Counting rule: logical statements/branches/returns, not physical file lines
- `Drive.h` remains authoritative for public API obligations

## Governing assertion

Each short function is assessed against this test:

`Centralizing the logic on that point is only useful if you can tell what all would be affected by changes to that method. If the wrapper does not make code readable and does not make changes easier to make at the intended scope, it adds bloat without sufficient legibility or functional benefit to justify it.`

## Additional automatic-fail screen

For file-local helpers and private implementation helpers, also apply this rule:

`Reject all methods that require more characters to use than the equivalent inlined form.`

This is treated as an automatic failure condition because it is a strong sign that the abstraction adds neither call-site compression nor scope clarity.

This screen does **not** override fixed `Drive.h` public API obligations. Public `Drive::` methods that exist because the contract requires them are not judged against private-field access or illegal contract-breaking inlined substitutes.

## Additional helper-specific screen

For helpers, also apply this rule:

`A helper must justify itself against the assertion: "The purpose of this function would be just as easily achieved by commenting the code."`

If a helper only saves a short inline expression, ternary, or fallback/defaulting snippet that could stay inline with a local comment and without materially increasing inconsistency risk, that helper fails this screen.

This screen also does **not** apply to fixed `Drive.h` public API methods. A comment is not a substitute for required contract surface.

## Drive-specificity and no-copied-owner screen

For helpers in `Drive.cpp`, also apply this rule:

`Things in the .cpp file should relate specifically to this class, and if the function exists outside of this class as well, it absolutely should not be copied here.`

In audit terms, that means:

- a helper in `Drive.cpp` must either encode a Drive-specific recovery, completion, latching, or command-emission rule, or
- isolate a low-level hazard that exists specifically because of `Drive`'s implementation shape.

If the operative logic is already owned elsewhere, such as by the standard library, `MazeMapRuntimeCore`, `MissionStartPolicy`, or `DriveBase`, then a local helper fails unless it adds a clearly Drive-specific policy boundary on top of that existing owner.

## Verdict meanings

- `Pass`: the function has a coherent semantic scope, materially improves readability, or exists because the fixed public contract requires it.
- `Borderline`: the function is defensible, but it is close enough to the line that future consolidation could reasonably absorb it.
- `Fail`: the function does not justify itself against the assertion and is a valid removal or inline candidate.

## Summary

### Fails

- `FallbackFinite`
- `HasFiniteNonZeroMagnitude`
- `FiniteMagnitudeOrZero`
- `HasFiniteLimit`
- `ResolveDistanceRequestMagnitude`
- `ResolveRecoveredTurnAngleRad`
- `ReachableSpeedWithConfiguredLimit`
- `SafeAverageDistanceMeters`
- `TraveledDistanceMeters`
- `ResolveStraightTargetYawRad`
- `ResolveCommandTargetYawRad`
- `ResolveInitialLinearSpeedMps`
- `ResolveInitialYawRateRadps`

### Borderline

- `ResolveRequestedDirection`

Everything else in this audit passes.

## Audit Table

| Function / Method | Approx logical lines | Verdict | Assessment against the assertion |
| --- | ---: | --- | --- |
| `FallbackFinite` | ~1 | Fail | Too generic. A change would hit yaw fallback, state capture, tolerance evaluation, command sanitization, and feedforward stabilization at once. The name does not expose that blast radius, and the readability gain over an inline finite test is weak. It also fails the comment-substitution screen, and it fails the no-copied-owner screen because the real operative logic is standard-library finiteness testing rather than a specifically Drive-owned rule. |
| `HasFiniteNonZeroMagnitude` | ~1 | Fail | This names a recurring predicate, but the helper still only replaces one short inline condition. A call site comment such as “require a finite nonzero curvature” would preserve the same readability with less abstraction overhead, and the helper is still only a local restatement of standard-library numeric checks rather than a Drive-specific owner. |
| `FiniteMagnitudeOrZero` | ~1 | Fail | This is a local repackaging of `std::isfinite` plus `std::fabs`, not a Drive-specific policy. The file does use that numeric notion often, but the helper itself does not relate specifically enough to `Drive` to justify being copied into this file. |
| `HasFiniteLimit` | ~1 | Fail | This is effectively a renamed `std::isfinite` with no stronger scope story. Changing it would affect many unrelated limit sites, while the name adds little legibility. It also fails the character-count screen because `HasFiniteLimit(x)` is not shorter than `std::isfinite(x)`, and it fails the no-copied-owner screen because the real owner is the standard library. |
| `ResolveSignedPreference` | ~5 | Pass | Encodes a real precedence rule: first usable signed hint wins, with `NaN` ignored. The affected scope is coherent and visible from the name. |
| `ResolveRequestedDirection` | ~2 | Borderline | It does encode a real policy: derive direction from signed hints and default positive when none are usable. That policy is more Drive-specific than a pure std wrapper, but the body is still small enough that the same rule could be expressed inline with one comment after `ResolveSignedPreference(...)`, so the helper stays only narrowly justified. |
| `ResolveRequestedMagnitude` | ~10 | Pass | This is a real recovery routine, not a wrapper. It centralizes precedence among primary, secondary, and fallback magnitudes, including infinity handling. |
| `ResolveDistanceRequestMagnitude` | ~1 | Fail | Too thin. It is only a fixed-argument alias over `ResolveRequestedMagnitude(...)`, and its existence does not make impact easier to trace. It does not fail the character-count screen, but it still fails on insufficient semantic scope. |
| `ResolveNearestCardinalDirectionFromYawRad` | ~10 | Pass | This is domain logic, not glue. It owns one clear transformation from yaw to nearest cardinal direction. |
| `TryGetCurrentMazeCell` | ~12 | Pass | It owns one coherent conversion and validation step: current pose to maze cell coordinates. Call-site impact is easy to understand. |
| `TryResolveMazeSideOpeningQuarterTurnAngleRad` | ~10 | Pass | This is a real maze-recovery decision tree with a clear scope. |
| `TryResolveSensorSideOpeningQuarterTurnAngleRad` | ~5 | Pass | Same as above for sensor-based side-opening recovery. |
| `TryResolveContextualQuarterTurnAngleRad` | ~2 | Pass | Thin, but it still owns one coherent gating decision: contextual recovery only in maze mode, with maze source preferred over sensor source. |
| `ResolveRecoveredQuarterTurnAngleRad` | ~6 | Pass | Real recovery policy. It combines context resolution with preferred-sign precedence and deterministic fallback. |
| `ResolveRecoveredTurnAngleRad` | ~2 | Fail | Too little independent meaning. It is just a finite short-circuit in front of quarter-turn recovery, and the one call site could absorb it without losing scope clarity. It does not fail the character-count screen, but it still fails on insufficient independent scope. |
| `LimitByConfiguredMagnitude` | ~4 | Pass | Centralizes one coherent signed-clamp rule with finite-limit fallback behavior. |
| `LimitMagnitudeByConfiguredMagnitude` | ~3 | Pass | Same for magnitude-only clamping. It expresses a real rule rather than merely renaming a library call. |
| `WithinConfiguredTolerance` | ~2 | Pass | Centralizes one tolerance policy, including fallback tolerance resolution and non-finite error handling. |
| `ReachableSpeedWithConfiguredLimit` | ~1 | Fail | The policy is coherent, but the helper only wraps one ternary around `ReachableSpeedWithBoundary(...)`, which already exists in `MazeMapRuntimeCore`. An inline expression plus a comment such as “only apply reachable-speed math when accel limit is usable” would achieve the same purpose just as easily, so this does not clear either the comment-substitution screen or the no-copied-owner screen. |
| `IsTurnComplete` | ~2 | Pass | It names the exact turn-completion contract and is reused as such. |
| `IsMotionSettled` | ~2 | Pass | Thin, but its scope is coherent and readable: the shared stationary-settle rule, including fan-duty scaling, used by hold and terminal completion paths. It builds on `IsMissionStartupStationaryFromSensors(...)`, but adds a specific Drive completion policy rather than copying that external owner. |
| `StorageAs<T>` | ~1 | Pass | This is generic-looking code, but it exists specifically because `Drive` stores multiple primitive states in one raw storage block. That low-level hazard is part of `Drive`'s implementation shape, so centralizing it here is justified. |
| `StorageAs<const T>` | ~1 | Pass | Same reason as the mutable overload. |
| `SafeAverageDistanceMeters` | ~1 | Fail | Too thin. It is only a null guard around one method call, with very little readability gain and no meaningful policy boundary. It does not fail the character-count screen, but it still fails on weak scope value. |
| `TraveledDistanceMeters` | ~1 | Fail | This is only one repeated arithmetic expression. A local comment like “distance traveled since primitive start” would preserve the same meaning with less indirection, so the helper fails the comment-substitution screen. |
| `TryResolveHeadingYawRadFromComponents` | ~8 | Pass | Real recovery logic for finite and infinite heading hints. The affected scope is specific and easy to reason about. |
| `ResolveStraightTargetYawRad` | ~4 | Fail | Single-call-site composition that does not earn an independent abstraction. The real policy lives in the heading-resolution helper and the caller’s start-time semantics. It does not fail the character-count screen, but it still fails on insufficient standalone scope. |
| `TryResolveProjectedAxisContribution` | ~12 | Pass | Real projection and infinity-handling policy. A change to it has a coherent, narrow effect. |
| `TryResolveStraightTargetDistanceM` | ~10 | Pass | Owns one clear piece of straight-start recovery logic: convert projected target position into retained target distance. |
| `ResolveInitialLinearSpeedMps` | ~3 | Fail | This is a short state-or-last-command fallback pattern. The purpose would be just as easily served by keeping the logic inline with a comment at the small number of start-time call sites, so it no longer clears the helper bar. |
| `ResolveInitialYawRateRadps` | ~3 | Fail | Same judgment as the linear-speed version. It is coherent, but not enough more readable than the commented inline form to justify an extra helper. |
| `ResolveTurnCommandMagnitudeRadps` | ~7 | Pass | Real precedence logic for turn-rate magnitude recovery. |
| `ResolveCommandTargetYawRad` | ~1 | Fail | This helper mainly composes existing owners: `FallbackFinite(...)` and `WrapAngleRad(...)`. It does not add enough uniquely Drive-specific policy to justify standing as its own local abstraction, so under the no-copied-owner rule it should not survive as a separate helper. |
| `ResolveSignedCommandForDriveBase` | ~5 | Pass | Centralizes a nontrivial command-recovery rule including infinity handling and clamping for DriveBase-facing commands. |
| `ResolveRecoveredTranslationSpeedMps` | ~3 | Pass | Small, but it encapsulates one coherent start-time speed-recovery policy rather than a generic wrapper. |
| `MakePointControlVector` | ~2 | Pass | Thin, but it clearly owns the null-safe scalar command emission path to `DriveBase`, including Drive-specific finite fallback and brake-on-missing-drive behavior. That is more than a copied alias. |
| `MakePointControlVectorWithHeadingTarget` | ~2 | Pass | Same, for the heading-target emission path. |
| `IsLinearMotionCompleteAtExit` | ~2 | Pass | This is a shared semantic rule, not a cosmetic wrapper. It centralizes the exact stop-vs-speed-match completion split. |
| `MakeFiniteTurnToHeadingControls` | ~6 | Pass | Owns one shared executor behavior: finite heading-target turn completion plus feedforward command proposal. |
| `Drive::Drive` | ~4 | Pass | Public class construction contract. It installs retained default PD settings and is not optional bloat. |
| `Drive::SetOperationMode` | ~1 | Pass | Fixed public API from `Drive.h`. Its triviality does not undermine its necessity. |
| `Drive::GetOperationMode` | ~1 | Pass | Fixed public API from `Drive.h`. |
| `Drive::SetLimits` | ~1 | Pass | Fixed public API from `Drive.h`, and the contract explicitly requires verbatim retention of the supplied limits. |
| `Drive::GetLimits` | ~1 | Pass | Fixed public API from `Drive.h`. |
| `Drive::SetCommandPDSettings` | ~1 | Pass | Fixed public API from `Drive.h`. |
| `Drive::GetCommandPDSettings` | ~1 | Pass | Fixed public API from `Drive.h`. |
| `Drive::IsEffectivelyComplete` | ~1 | Pass | Fixed public API from `Drive.h`; trivial body is expected. |
| `Drive::StartHold` | ~4 | Pass | Public instruction setter. Even though short, it owns one retained primitive-arm sequence required by the contract. |
| `Drive::StartStraight` | ~13 | Pass | Public instruction setter with start-time latching, direction recovery, and retained straight-state installation. The scope is coherent and contract-driven. |
| `Drive::StartTurn` | ~13 | Pass | Public instruction setter with retained target-yaw and yaw-rate recovery policy. |
| `Drive::StartTurnTransition` | ~8 | Pass | Public instruction setter with one clear retained-state construction path. |
| `Drive::StartArc` | ~10 | Pass | Public instruction setter with arc-geometry recovery and retained-state installation. |
| `Drive::StartManeuver` | ~11 | Pass | Public instruction setter for retained maneuver execution state. |
| `Drive::GetNextControls` | ~11 | Pass | This is the core public tick-time dispatcher. Small only because primitive-specific work is already separated coherently. |
| `Drive::AttachRuntime` | ~8 | Pass | One explicit lifecycle hook for wiring runtime-owned dependencies and initial live limits. |
| `Drive::ResetActivePrimitive` | ~9 | Pass | Centralizes raw-storage destruction and state clearing. Its blast radius is specific and obvious. |
| `Drive::HoldControls` | ~6 | Pass | One primitive executor with a clear scope. |
| `Drive::TurnControls` | ~8 | Pass | One primitive executor with a clear scope. |
| `Drive::TurnTransitionControls` | ~12 | Pass | One primitive executor with a clear scope. |
| `Drive::ArcControls` | ~8 | Pass | One primitive executor with a clear scope. |

## Practical conclusion

If the goal is to reduce `Drive.cpp` bloat without harming the documented behavior model, the most defensible next removals are the `Fail` items first:

1. `FallbackFinite`
2. `HasFiniteNonZeroMagnitude`
3. `FiniteMagnitudeOrZero`
4. `HasFiniteLimit`
5. `ResolveDistanceRequestMagnitude`
6. `ResolveRecoveredTurnAngleRad`
7. `ReachableSpeedWithConfiguredLimit`
8. `SafeAverageDistanceMeters`
9. `TraveledDistanceMeters`
10. `ResolveStraightTargetYawRad`
11. `ResolveCommandTargetYawRad`
12. `ResolveInitialLinearSpeedMps`
13. `ResolveInitialYawRateRadps`

The `Borderline` items are secondary candidates. They still have some semantic naming value, but they no longer have much margin over the “just comment the inline code” objection.

## Effect of the added screens

The character-count screen still most clearly condemns `HasFiniteLimit`, because it is both semantically weak and not shorter than the obvious inlined standard-library form.

The new comment-substitution screen materially changes the audit. It newly condemns helpers whose only strong claim was “this name is a little nicer than the inline expression,” including:

- `HasFiniteNonZeroMagnitude`
- `ReachableSpeedWithConfiguredLimit`
- `TraveledDistanceMeters`
- `ResolveInitialLinearSpeedMps`
- `ResolveInitialYawRateRadps`

The new Drive-specificity screen adds another constraint: `Drive.cpp` helpers are not allowed to be copies of logic already owned by the standard library or existing shared runtime helpers unless they add a specifically Drive-owned policy boundary. Under that test, the clearest additional failures are:

- `FiniteMagnitudeOrZero`
- `ResolveCommandTargetYawRad`

It also confirms the existing failure judgments on helpers that are mostly local repackagings of already-owned logic:

- `FallbackFinite`
- `HasFiniteLimit`
- `ReachableSpeedWithConfiguredLimit`

It downgrades some helpers from comfortable passes to narrow survivors because their policies are real, but the bodies are so small that a commented inline form is a serious alternative:

- `ResolveRequestedDirection`

The helpers that still pass do so because a comment would not preserve the same reusable decision rule, low-level hazard boundary, or domain-specific recovery policy.
