# DriveBase / PlantModel / SrUkfCore Coupling Audit

Date: 2026-05-06

## Scope

This note inventories the current places where `DriveBase` interacts with implementation-specific details of `PlantModel` or `SrUkfCore`, including direct calls, data-layout coupling, reporting coupling, and test-surface coupling.

The intended-shape description below is grounded in `cleanup_lessons_learned_2026-05-06.md`, especially the guidance that:

- if a design seems to force a choice between a bag and a long parameter list, the structure is wrong and the owner must change;
- deleting one transport shape without converging ownership just creates getter and reconstruction shrapnel;
- a good simplification seam should reduce caller knowledge rather than re-export internals;
- `Drive` is the local exemplar of a coherent, retained, behavioral simplification layer rather than a forwarding or transport layer.

Relevant lessons-learned references:

- `cleanup_lessons_learned_2026-05-06.md:123`
- `cleanup_lessons_learned_2026-05-06.md:177`
- `cleanup_lessons_learned_2026-05-06.md:282`
- `cleanup_lessons_learned_2026-05-06.md:541`
- `cleanup_lessons_learned_2026-05-06.md:556`

## Coupling Inventory

1. `Estimator::ukf()` is the escape hatch that enables the rest of the coupling.
   `DriveBase` reaches through `Estimator` into `SrUkfCore` instead of staying on an estimator-owned capability seam.
   Refs: `MazeMap/MazeMap/Estimator.h:33`, `MazeMap/MazeMap/DriveBase.cpp:411`, `MazeMap/MazeMap/DriveBase.h:373`.

2. Raw feedforward command generation is UKF-owned in the current path.
   `DriveBase::ResolveRawAccelerationCommand(...)` and `DriveBase::ResolveRawVelocityTargetCommand(...)` call `SrUkfCore::solveAlignedDriveCommands(...)` and `SrUkfCore::solveAlignedDriveCommandsForVelocityTarget(...)`.
   Refs: `MazeMap/MazeMap/DriveBase.cpp:677`, `MazeMap/MazeMap/DriveBase.cpp:729`, `MazeMap/MazeMap/SrUkfCore.h:316`, `MazeMap/MazeMap/SrUkfCore.cpp:602`.

3. Those UKF-aligned solve paths are not thin aliases over `PlantModel`.
   `SrUkfCore` builds policy modifiers from grip memory, holdoff, recovery, and utilization state before calling plant feedforward solves, so `DriveBase` is indirectly coupled to those UKF policy internals by using the aligned solve surface.
   Refs: `MazeMap/MazeMap/SrUkfCore.cpp:623`, `MazeMap/MazeMap/SrUkfCore.cpp:742`, `MazeMap/MazeMap/SrUkfCore.cpp:2361`.

4. Feedforward yaw-source selection is currently UKF-owned.
   `DriveBase::GetVelocityCommandOperatingState(...)` asks `SrUkfCore::resolveYawRateForFeedforward(...)` and overwrites `presentState(kR)` with the result.
   Refs: `MazeMap/MazeMap/DriveBase.cpp:454`, `MazeMap/MazeMap/DriveBase.h:551`, `MazeMap/MazeMap/SrUkfCore.cpp:597`.

5. Velocity-target technical-limit selection is also UKF-owned today.
   `DriveBase::ResolveDefaultVelocityTargetCommandEnvelope(...)` calls `SrUkfCore::alignedVelocityTargetTechnicalLimits(...)`, so `DriveBase` depends on UKF-conditioned plant policy rather than a plant-owned technical-limits capability.
   Refs: `MazeMap/MazeMap/DriveBase.h:884`, `MazeMap/MazeMap/SrUkfCore.h:345`, `MazeMap/MazeMap/SrUkfCore.cpp:732`.

6. `DriveBase` rebuilds raw `VehicleState::StateVector` values instead of staying on `VehicleState` getters.
   The anonymous `BuildPlantStateVector(...)`, `GetVelocityCommandOperatingState(...)`, and `CommandContext.presentState` all depend on `VehicleState` index constants and normalization rules.
   Refs: `MazeMap/MazeMap/DriveBase.cpp:10`, `MazeMap/MazeMap/DriveBase.cpp:406`, `MazeMap/MazeMap/DriveBase.h:710`, `MazeMap/MazeMap/VehicleState.h:81`.

7. `DriveBase` depends directly on UKF-owned `PlantParams` and `PreparedParams`.
   It pulls `params()` and `preparedParams()` for encoder omega conversion, wheel radius, track width, fallback kinematics, and wheel-target generation.
   Refs: `MazeMap/MazeMap/DriveBase.h:202`, `MazeMap/MazeMap/DriveBase.cpp:411`, `MazeMap/MazeMap/DriveBase.cpp:556`, `MazeMap/MazeMap/DriveBase.cpp:609`, `MazeMap/MazeMap/DriveBase.cpp:806`.

8. `DriveBase` consumes plant-side implementation helpers, not just one feedforward capability.
   The current path uses `PlantDerivatives`, `forwardStep(...)`, `ComputeBodyAction(...)`, `ComputeBodyActionFromYawRate(...)`, `resolveWheelMotionTargets(...)`, and `PlantModel::kDefaultVelocityTargetResponseTimeS`.
   Refs: `MazeMap/MazeMap/DriveBase.h:713`, `MazeMap/MazeMap/DriveBase.cpp:473`, `MazeMap/MazeMap/DriveBase.cpp:567`, `MazeMap/MazeMap/DriveBase.cpp:635`, `MazeMap/MazeMap/DriveBase.cpp:649`, `MazeMap/MazeMap/DriveBase.cpp:665`, `MazeMap/MazeMap/DriveBase.cpp:609`.

9. `DriveBase` directly indexes UKF covariance and diagnostic state to build reporting.
   `GetGeneratedTelemetry()` and `GetTelemetry()` copy mode, NHC, bias, recovery, grip, process-noise, innovation, applied-torque, and covariance-diagonal fields straight out of `SrUkfCore`.
   Refs: `MazeMap/MazeMap/DriveBase.h:426`, `MazeMap/MazeMap/DriveBase.h:512`, `MazeMap/MazeMap/DriveTelemetry.h:33`, `MazeMap/MazeMap/SrUkfCore.h:224`.

10. That reporting coupling escapes `DriveBase` and becomes indirect runtime coupling elsewhere.
    `DriveTelemetry` carries raw UKF-facing fields, and downstream controllers log or inspect them directly, so `DriveBase` is not merely consuming `SrUkfCore` internals locally; it is exporting them as part of its own reporting contract.
    Refs: `MazeMap/MazeMap/DriveTelemetry.h:33`, `MazeMap/MazeMap/ShowcasingDonutController.cpp:560`, `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1444`.

11. `DriveBase` re-exports a UKF debug surface.
    `WriteUkfDebugTextDump(...)` is a pass-through façade over `SrUkfCore::WriteDebugTextDump(...)`.
    Ref: `MazeMap/MazeMap/DriveBase.h:372`.

12. The test surface currently preserves the same noncanonical seam.
    `DriveBaseTest` asserts against `estimator.ukf().solveAlignedDriveCommandsForVelocityTarget(...)`, which bakes the UKF-facing command path into the drive tests.
    Ref: `MazeMap/MazeMapTest/DriveBaseTest.cpp:1611`.

## Intended Shape

The intended shape, consistent with the lessons-learned note, is:

1. `DriveBase` should use `Estimator` as a narrow estimator boundary and should essentially never call `SrUkfCore` directly.

2. `DriveBase` should read state-dependent feedback signals from `Estimator::RuntimeState()` through `VehicleState` getters and variance getters, not by rebuilding `StateVector` objects, indexing covariance matrices, or rewriting selected state coordinates for feedforward policy reasons.

3. `DriveBase` should ask `PlantModel` for implementation-agnostic feedforward capabilities that are fully contained in `PlantModel`.
   That includes the feedforward solve itself, the velocity-target solve path, and any technical-limit query required to shape those commands.

4. Yaw-source policy for feedforward should not be a `DriveBase` concern.
   If special yaw-selection logic is required, it should be hidden behind a narrow owner-facing capability rather than exposed as `SrUkfCore::resolveYawRateForFeedforward(...)`.

5. `DriveBase` should not carry `PreparedParams`, `PlantDerivatives`, or other plant implementation transport surfaces across the boundary unless those are true stable plant-domain concepts required by the drive owner itself.
   The current pattern is transport-shape leakage, not a deep module boundary.

6. `DriveBase` reporting should not reconstruct or forward UKF internal diagnostic state one field at a time.
   Either those diagnostics belong to an estimator-owned reporting surface, or they should stay estimator-internal.
   `DriveBase` should report drive-owned outputs and drive-owned retained state, not become a telemetry export path for `SrUkfCore`.

7. Tests around `DriveBase` should protect the intended owner/path semantics.
   They should prove that the drive owner uses the canonical plant-facing seam, not that it can still reach through `Estimator` into UKF-aligned solver helpers.

## Summary

The current code does not merely call into `PlantModel` and `SrUkfCore` in a few isolated places. It reconstructs plant operating state, selects estimator policy, solves commands through UKF-owned aligned helpers, imports UKF-owned technical limits and caches, and exports UKF internal diagnostics through drive telemetry.

Relative to the intended architecture, the problem is not just a few bad method calls. The problem is that `DriveBase` currently knows too much about both the plant implementation surface and the UKF implementation surface, and it makes other layers know about those surfaces too.
