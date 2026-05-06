# Cleanup Architecture Review 2026-05-06

Review target:
- Commit `b309b77785b3a482d88c33b4e2e33fb92ca87930`
- Subject: `Repair post-refactor ownership and timing regressions`
- Review basis: `HEAD` relative to `HEAD^`

## Assessment Frame

This review is grounded in the architectural rules clarified in the discussion around this cleanup.

The current understanding of what this repo is aiming for is:

1. Every real concept must have one authoritative owner.
2. Public APIs must be as narrow as possible for the capability actually being offered.
3. Bags and long parameter lists are both signs that ownership is wrong. Replacing one with the other is not convergence.
4. A sanctioned simplification layer must look like `Drive`: behaviorally coherent, concise, complete, stateful across the full justified simplification space, and singular enough that parallel wrappers become obviously invalid.
5. Existing vocabulary should be reused aggressively. New local dialects are presumed to collide with better existing language unless proven otherwise.
6. `LastDiagnostics()` must mean a completed tick. Live current-tick cadence is a different concern and should not be smuggled through a reporting API.
7. Comments are often preferable to extra abstraction. Lines are not free, and helper/accessor sprawl competes directly with the business logic of the codebase.
8. Public support types need a very high bar. A top-level type with little or no behavior is presumed to be plumbing drift, not architecture.

The findings below are written against that frame, not against a looser “it compiles and the numbers are finite” standard.

## Findings

### P1. `Drive` is consuming completed-tick reporting as if it were live execution cadence

Refs:
- `MazeMap/MazeMap/Drive.cpp:1207`
- `MazeMap/MazeMap/LoopController.h:295`
- `MazeMap/MazeMap/LoopController.cpp:280`

`Drive::GetNextControls(...)` now derives `dtSeconds` from `LoopController::LastDiagnostics().dtUs`. That is wrong against the clarified contract of `LastDiagnostics()`: it is a completed-tick reporting surface, not a live current-tick execution input.

This is a problem for two separate reasons:

- It makes `Drive` execute current primitive progression using stale timing. That is a plain behavioral defect, but it is also an ownership defect because `Drive` is now depending on a reporting record for active control semantics.
- It damages the exact simplification-layer shape that `Drive` is supposed to exemplify. `Drive` is valuable in this repo because it condenses a wide space of stateful `DriveBase` usage into one coherent, retained semantic owner. If `Drive` has to reach sideways into a diagnostic reporting object for live cadence, then execution semantics are no longer cleanly owned.

Against the repo’s present bar, this is not just “wrong field, easy fix.” It means a reporting artifact leaked into the active behavior path, which is precisely the kind of structural confusion the cleanup was supposed to remove.

### P1. `PlantModel` and `SrUkfCore` both act as public drive-command authorities

Refs:
- `MazeMap/MazeMap/PlantModel.h:481`
- `MazeMap/MazeMap/PlantModel.h:512`
- `MazeMap/MazeMap/PlantModel.h:532`
- `MazeMap/MazeMap/PlantModel.h:659`
- `MazeMap/MazeMap/PlantModel.cpp:2810`
- `MazeMap/MazeMap/PlantModel.cpp:2882`
- `MazeMap/MazeMap/PlantModel.cpp:2971`
- `MazeMap/MazeMap/PlantModel.cpp:3624`
- `MazeMap/MazeMap/SrUkfCore.h:317`
- `MazeMap/MazeMap/SrUkfCore.h:330`
- `MazeMap/MazeMap/SrUkfCore.h:345`
- `MazeMap/MazeMap/SrUkfCore.h:351`
- `MazeMap/MazeMap/SrUkfCore.cpp:623`
- `MazeMap/MazeMap/SrUkfCore.cpp:688`
- `MazeMap/MazeMap/SrUkfCore.cpp:742`

The cleanup left `PlantModel` with a public drive-solver surface and then added a second “aligned” public drive-solver/reporting surface on `SrUkfCore`. These are not trivial aliases: the `SrUkfCore` path injects live grip/recovery/policy state while the raw plant path solves with raw plant semantics.

That creates a direct ownership split:

- `PlantModel` is supposed to be the canonical owner of the motion model and shared plant equations.
- `SrUkfCore` is supposed to own estimation/filter behavior.

Once `SrUkfCore` starts exposing a second command-solver vocabulary, callers now have to understand two public semantic spaces:

- raw plant solve semantics
- UKF-conditioned solve semantics

That is exactly the kind of alternate access path the repo is trying to eliminate. It is also the opposite of the `Drive` standard. `Drive` works because it is the one coherent simplification layer over `DriveBase`. Here, the cleanup did not create one sanctioned simplification layer. It created two partially overlapping public authorities with different behavior.

That is not convergence. It is fragmentation disguised as cleanup.

### P1. `DriveBase` now reaches through `Estimator` into UKF internals instead of using a narrow estimator seam

Refs:
- `MazeMap/MazeMap/Estimator.h:33`
- `MazeMap/MazeMap/DriveBase.cpp:411`
- `MazeMap/MazeMap/DriveBase.cpp:454`
- `MazeMap/MazeMap/DriveBase.cpp:556`
- `MazeMap/MazeMap/DriveBase.cpp:609`
- `MazeMap/MazeMap/DriveBase.cpp:635`
- `MazeMap/MazeMap/DriveBase.cpp:709`
- `MazeMap/MazeMap/DriveBase.cpp:735`
- `MazeMap/MazeMap/DriveBase.h:430`
- `MazeMap/MazeMap/DriveBase.h:446`
- `MazeMap/MazeMap/DriveBase.h:541`
- `MazeMap/MazeMap/DriveBase.h:894`
- `MazeMap/MazeMap/SrUkfCore.h:274`
- `MazeMap/MazeMap/SrUkfCore.h:317`

The current shape leaves `Estimator` as a raw escape hatch and lets `DriveBase` pull many distinct capabilities directly from `SrUkfCore`: command solving, technical limits, prepared params, covariance-derived details, feedforward yaw selection, and more.

This is a problem because it broadens rather than narrows the seam:

- `DriveBase` now knows too much about UKF layout and policy details.
- `Estimator` is no longer functioning as a clean estimator boundary.
- `SrUkfCore` is effectively a second public drive/reporting owner.

This is the same architectural mistake as a bag, just expressed in the opposite shape. Instead of a single context object, the design now exposes a large field-and-method surface that consuming code has to reassemble manually. That violates the repo’s clarified rule that both bags and long/split plumbing surfaces are evidence of bad ownership.

The `Drive` exemplar matters here. `Drive` narrows the caller surface and internalizes the complexity. This cleanup did the reverse: it spread the estimator/plant seam across a growing set of UKF methods and direct reads.

### P1. Test support defaults now exercise impossible zero-supply behavior

Refs:
- `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h:257`
- `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h:313`
- `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp:18`
- `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp:727`
- `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp:769`
- `MazeMap/MazeMapTest/SrUkfCoreBiasAndStationaryTest.cpp:214`

`SrUkfCore` test support now defaults `batteryVoltageV` to `0.0f`, and several tests rely on that default rather than explicitly supplying runtime-like values.

This is a real review finding rather than “just a bad test default” because the repo’s testing rules are explicit: tests should preserve access to the canonical architecture and should not be weakened into accepting noncanonical behavior. Here the default test path no longer resembles the real runtime condition. It silently validates an impossible supply state.

That has two bad effects:

- supply-dependent regressions can be masked
- the tests start teaching the wrong shape of the system by normalizing impossible inputs

In a repo that treats architecture as a first-class correctness concern, this is not a cosmetic testing issue. It erodes the ability of the test suite to guard the real canonical runtime path.

### P1. `PlantModelDynamicsTest` helper flow no longer preserves the fan-duty condition it claims to validate

Refs:
- `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp:18`
- `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp:634`

The helper path used by the grip/utilization coverage no longer carries the passed fan-duty condition through to the evaluated plant path, so the call site that appears to be validating a `0.30f` condition is actually validating a different one.

This matters because the repo’s standard is not “keep the test green.” It is “tests must still exercise the canonical behavior through the canonical owner.” Once the helper silently drops a condition like fan duty, the test no longer validates the behavior its name and setup imply. It becomes another example of plumbing code doing a misleading abstraction job rather than preserving directness.

The broader architectural lesson is also relevant: once test helpers become too clever and too indirect, they start hiding owner-boundary mistakes instead of exposing them.

### P2. Several controllers still use completed-tick diagnostics as live active-tick cadence input

Refs:
- `MazeMap/MazeMap/DiagnosticController.cpp:1646`
- `MazeMap/MazeMap/DiagnosticController.cpp:1771`
- `MazeMap/MazeMap/DiagnosticController.cpp:1885`
- `MazeMap/MazeMap/DiagnosticController.cpp:1955`
- `MazeMap/MazeMap/AuxMeasurementController.cpp:364`
- `MazeMap/MazeMap/ShowcasingDonutController.cpp:410`
- `MazeMap/MazeMap/ShowcasingDonutController.cpp:764`

These sites use `LastDiagnostics().dtUs` for live behavior:

- speed ramping
- hold-time accumulation
- arc metric integration
- traction sweep progression
- donut speed ramping

That is wrong even after correcting the `LastDiagnostics()` meaning, because those are not reporting concerns. They are active callback concerns.

Why this is architecturally bad:

- it pushes live cadence semantics onto a reporting API
- it makes the behavior one tick stale
- it forces individual controllers to reason about timing plumbing instead of staying at the level of regime intent

That last point is important against the `Drive` standard. A good simplification layer lets consuming code stay compact and intent-focused. These controllers are at their best when they express regimes and transitions, not when they reconstruct timing semantics from reporting surfaces. This is exactly the kind of code path where the repo wants narrow, capability-shaped APIs rather than cross-cutting plumbing reads.

### P2. `Estimator::updateSideSensor(...)` widens to `RelativeDirection` and then discards most of it

Refs:
- `MazeMap/MazeMap/Estimator.h:115`
- `MazeMap/MazeMap/Estimator.cpp:144`

The cleanup changed the side-sensor API to take the richer canonical `RelativeDirection` vocabulary, but the implementation then maps every non-`Left90` value onto the right sensor.

That is a lossy use of a non-lossy language.

The problem is not merely “future bug if someone passes `Forward`.” It is that the signature now advertises canonical directional language while the behavior still operates on a poorer local side-dialect. That violates one of the strongest lessons from the discussion:

- if the repo already has the vocabulary, use it meaningfully
- do not collapse richer language into a blunter local classifier
- do not create signatures that look canonical while behavior remains parochial

If only `Left90` and `Right90` are valid here, the interface should narrow or reject unsupported cases explicitly. Quietly treating most of the directional language as “right” undermines the exact indexing semantics the repo has already invested in.

### P2. `AppliedTorqueEstimate` is a new public plumbing bag, not a real owner or behavioral type

Refs:
- `MazeMap/MazeMap/AppliedTorqueEstimate.h:5`
- `MazeMap/MazeMap/PlantModel.h:468`
- `MazeMap/MazeMap/SrUkfCore.h:550`
- `MazeMap/MazeMap/SrUkfCore.h:640`

`AppliedTorqueEstimate` is a top-level type with no meaningful behavior of its own. It exists to move internal torque-policy state between `PlantModel` and `SrUkfCore`.

Against the present understanding of the repo, that is almost the textbook definition of a type that should not exist publicly:

- it is not a mature domain vocabulary type
- it is not a behavioral owner
- it is not a narrow centralized contract
- it is a data-moving artifact created by the plumbing shape

This is exactly the kind of type the cleanup should have deleted rather than reintroduced under a different name. If the concept is truly only internal glue between plant and UKF policy state, it should stay internal to the owner that actually needs it. Making it a top-level header just expands the public seam and invites more code to couple to the wrong boundary.

### P2. `modelCycleContext()` was replaced by getter sprawl and duplicated telemetry reconstruction

Refs:
- `MazeMap/MazeMap/SrUkfCore.h:274`
- `MazeMap/MazeMap/SrUkfCore.h:304`
- `MazeMap/MazeMap/DriveBase.h:426`
- `MazeMap/MazeMap/DriveBase.h:512`

The cleanup removed one obvious aggregate shape and replaced it with a broad collection of direct getters plus duplicated telemetry assembly logic in `DriveBase`.

This is a structural regression for two reasons:

- It solves the “bag” problem by atomizing the same internals into a wider surface instead of converging them behind a narrow owner.
- It duplicates the reconstruction logic in more than one place, which means the consumer now has to know the producer’s internal field layout.

This is exactly the failure mode discussed in the prompt thread:

- bags are bad
- but replacing the bag with many direct getters is not success
- and replacing the bag with many direct getters plus duplicated reconstruction code is even worse from a navigation and ownership perspective

The repo wants lines to earn their keep. Here the extra lines are not buying a cleaner capability boundary. They are buying more assembly code and more coupling.

### P2. Command reporting is still split across direct `DriveBase` getters and `DriveTelemetry`

Refs:
- `MazeMap/MazeMap/AuxMeasurementController.cpp:55`
- `MazeMap/MazeMap/AuxMeasurementController.cpp:56`

This controller reads commanded linear and angular speed directly from `DriveBase` while reading other command/reporting fields from `DriveTelemetry`.

That means the cleanup still has not fully converged reporting ownership. If `DriveTelemetry` is the intended reporting surface, consumers should not need parallel direct getters for adjacent reporting concerns. If the direct getters are still necessary, then the telemetry surface is not yet complete.

This matters against the `Drive` standard. A justified simplification or reporting layer should be concise and complete across the whole simplified capability space. If consumers still have to combine it with sibling getters from the underlying owner, then the surface is not actually complete. The presence of both paths means the design is still in a partially split state.

### P2. Several tests were weakened away from owner/path semantics toward “finite output” checks

Refs:
- `MazeMap/MazeMapTest/DriveBaseTest.cpp:1611`
- `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp:1107`
- `MazeMap/MazeMapTest/SrUkfCoreMotionUpdateTest.cpp:863`

The tests in these areas no longer guard the architectural or semantic path they used to assert. Instead, they have drifted toward:

- finite output checks
- value equivalence checks
- shallow getter checks

That is a problem in this repo because the tests are part of how canonical architecture is enforced. If a test used to prove:

- the aligned feedforward path was used
- the fallback path was not used
- the frozen policy state was actually propagated

and now only proves “the numbers are finite,” then the test no longer protects the architecture. It now tolerates exactly the kind of silent owner/path regression the cleanup introduced.

The repo explicitly rejects preserving or adapting tests merely to keep a noncanonical design comfortable. These tests are not neutral after the cleanup. They now under-enforce the intended owner shape.

## Overall Assessment

The current state is materially better than the earlier hub-header/bag sprawl, but it still has unresolved convergence failures.

The dominant remaining problems are:

1. timing/reporting and live execution cadence are still mixed
2. `PlantModel`, `Estimator`, `SrUkfCore`, and `DriveBase` still do not have a clean single-owner command/reporting seam
3. one deleted aggregate was replaced in places by getter sprawl and reconstruction code rather than by a narrower owner
4. tests were weakened in exactly the areas where owner/path semantics most needed protection

The common pattern across these findings is consistent with the discussion that led to this review:

- a bag was deleted
- but the underlying ownership issue was not fully resolved
- so the data resurfaced as duplicated entry points, widened getter surfaces, or reporting/plumbing bleed-through

That is why these are architectural findings rather than mere cleanup nits.
