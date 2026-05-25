# PlantModel Contact-Continuum Yaw Identification Design Draft

Date: 2026-05-24

Scope: design-only production plan. No production code has been modified by this draft.

## Decision

The current analysis favors a PlantModel patch-force/contact-feature correction, not a residual `Vf/yaw` lookup table and not an aggregate yaw-loss term.

Evidence summary:

- The root authority is `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`. It requires per-contact relative velocity as the primary contact primitive and rejects slip ratio, slip angle, curvature/radius, instantaneous-center-of-rotation denominators, maneuver-mode branches, and command rejection based on force-envelope diagnostics.
- Feature extraction used contact-continuum variables only. It reconstructed per-contact `v_rel_f`, `v_rel_r`, `vbar_rel`, `vbar_lat`, `vbar_yaw`, force-request, and projected-force features without using `ukf_state_*` targets.
- Data quality found 15 fit-authoritative decoded open-floor runs, 9 fit-downweighted open-floor runs, 12 validation-only runs, 13 validation-only-downweighted aux runs, and 21 excluded runs.
- The primary open-floor plateau residual was much lower than entry/exit residual: median absolute residual `0.011055656 Nm` on plateau versus `0.025922618 Nm` on entry+exit, a `57.4%` reduction.
- Candidate plateau fit scores on the primary open-floor authoritative split:
  - `patch_force_contact_features`: weighted R2 `0.7887`, MAE `0.008917 Nm`, RMSE `0.012367 Nm`.
  - `artifact_transition_schema_run_proxy`: weighted R2 `-0.1297`.
  - `yaw_loss_aggregate`: weighted R2 `-2.4536`.
- Plateau validation is mixed but directionally supports patch/contact features:
  - `open_floor_validation_only`: patch R2 `0.5337`, yaw-loss R2 `-0.0052`.
  - `diag_validation_only`: patch R2 `0.9640`, yaw-loss R2 `-0.2184`.
  - `aux_downweighted_validation`: patch R2 `0.9599`, yaw-loss R2 `-2.0139`.
  - `open_floor_fit_downweighted`: patch R2 `-0.3726`, yaw-loss R2 `-7.2052`, which is a validation gap rather than a promotion signal.

Decision wording for production planning: patch-force/contact-feature correction appears favored; aggregate yaw-loss is not supported by the current ablation; validation gaps remain.

## Gate

This is not yet enough evidence to promote fitted coefficients into production as a behavior-changing plant retune. It is enough to define the canonical implementation shape and the exact production touch points.

Recommended gate before behavior change:

- Collect or select targeted yaw-contact validation data with per-row fan duty, saturation/watchdog fields, clean command evidence, and trimmed bad tails.
- Prioritize unsaturated or saturation-labeled high-contact yaw sweeps, both yaw signs, both straight-to-yaw transitions and steady yaw/contact plateaus, plus at least one held-out high-speed arc family.
- If possible, add an independent lateral-velocity reference or a controlled sweep that makes the `Vr=0` reconstruction assumption less confounded.
- Refit the narrow contact correction on fit-authoritative rows only, then require held-out improvement on open-floor validation and no degradation on low-demand straight/symmetric cases.

If implementation is requested before that targeted data exists, bound it to one small PlantModel-only correction path with conservative coefficients and tests that prove finite, continuous, symmetric behavior. Do not add an aggregate yaw-loss coefficient, residual table, new mode branch, or new owner.

## Authoritative Owner

Canonical owner: `PlantModel`.

Reason:

- The changed behavior is the shared plant equation for contact forces and yaw moment.
- `Vehicle` should continue to own physical construction facts only: mass, yaw inertia, track/contact geometry currently used by plant, wheel radius, fan downforce capacity, and physical acceleration references.
- `MotorEncoderDrive` should continue to own motor/encoder/wheel-bank facts only: torque conversion, gear ratio, equivalent wheel inertia, and tire-bank stiffness facts. Do not add yaw-identification coefficients there unless the next evidence proves they are true wheel-bank/tire-bank construction facts rather than empirical plant-equation corrections.

Forbidden shapes for this change:

- No parameter bag, facade, wrapper, helper subsystem, runtime mode branch, safety-limit owner, residual `Vf/yaw` table, slip ratio, slip angle, curvature/radius, or maneuver taxonomy.
- No production logging or runtime ownership changes.
- No new public support type to expose PlantModel internals. Existing PlantModel test-access diagnostics are enough for unit tests.

## Minimal Implementation Shape

The future production edit should stay inside `MazeMap/MazeMap/PlantModel.h` and `MazeMap/MazeMap/PlantModel.cpp`.

Use the existing PlantModel pipeline:

1. Resolve wheel-bank torques from commands and encoder-derived bank rates.
2. Compute per-contact kinematics from body velocity, yaw rate, wheel-bank surface speed, and contact patch locations.
3. Compute normal loads.
4. Compute raw per-contact force requests.
5. Apply any contact-continuum yaw correction as a patch-force correction before force projection.
6. Project contact force through the existing force envelope.
7. Sum per-contact forces into body acceleration and yaw acceleration.

The narrow correction should be expressed as small private PlantModel math over the four existing contact patches. It should adjust raw contact force requests, not append a second yaw model after the fact.

Recommended first production form:

```text
for each contact i:
    r_i = signed right position
    f_i = signed forward position
    q_i = f_i * v_rel_r_i - r_i * v_rel_f_i

q_bar = sum(N_i * q_i) / max(sum(N_i), epsilon)
r2_bar = sum(N_i * (r_i*r_i + f_i*f_i)) / max(sum(N_i), epsilon)

delta_force_f_i = -K_patch_yaw * (N_i / sumN) * (r_i / max(r2_bar, epsilon)) * q_bar
delta_force_r_i =  K_patch_yaw * (N_i / sumN) * (f_i / max(r2_bar, epsilon)) * q_bar

raw_forward_force_i += delta_force_f_i
raw_right_force_i += delta_force_r_i
```

This shape is continuous at zero forward speed and zero yaw rate, uses only per-contact relative velocity and contact load, and is equivalent to a patch-force correction rather than an aggregate yaw-loss branch. Coefficient `K_patch_yaw` must be a private PlantModel constant with units and fit metadata in nearby comments. It must default to the validated fitted value only after the gate above is satisfied. It must not be tuned from the current ablation script output alone.

If the refit shows the correction is better represented as a small scale on existing front/rear right-contact gains, change the existing raw contact force equations directly instead of adding a parallel correction path. That would mean replacing the current scalar `Kr_i * v_rel_r_i` behavior in PlantModel, not creating another model alongside it.

## Existing PlantModel Regions To Change

Current code references are from the inspected worktree on 2026-05-24.

- `MazeMap/MazeMap/PlantModel.h:143-153`
  - Current constants include `kForceEpsilonN`, friction terms, `kRightVelocityDampingNsPerM`, `kYawRateDampingNmsPerRad`, `kFrontLoadFraction`, and tire-friction defaults.
  - Add only private PlantModel constants needed for the contact correction, with units and data-fit metadata.
  - Do not add a public parameter/config type.
  - Do not make `kYawRateDampingNmsPerRad` nonzero. If contact correction is implemented, remove the aggregate yaw-damping constant and its uses unless another current owner still requires it.

- `MazeMap/MazeMap/PlantModel.cpp:170-190`
  - Debug dump emits contact positions using effective track width and longitudinal offset.
  - If a fitted contact correction is promoted, add only compact metadata to the existing debug dump so replay can identify the active plant equation. Do not create new logging ownership.

- `MazeMap/MazeMap/PlantModel.cpp:265-398`
  - `resolveAppliedBankTorques` owns motor command to bank torque behavior and already contains zero torque-correction placeholders at lines 318 and 370.
  - Do not put yaw-contact correction here. These placeholders are torque correction hooks, not contact-yaw physics.

- `MazeMap/MazeMap/PlantModel.cpp:438-733`
  - `evaluateAppliedBankTorqueStep` is the primary production region.
  - Keep normal-load computation at lines 477-490 as the initial static+fan approximation unless targeted data proves load-transfer is required.
  - Keep bank drive-force distribution at lines 492-517 unless fitting identifies load distribution as the actual error source.
  - Replace or extend raw force request computation at lines 569-596. This is the canonical place for patch-force/contact-feature correction.
  - Preserve force projection at lines 598-656, but apply the correction before projection so `contactPreProjectionUtilization` and saturation diagnostics remain meaningful.
  - Preserve yaw moment summation semantics at lines 658-674, using signed contact locations and projected forces.
  - Remove or leave permanently zero the aggregate yaw-damping behavior at lines 675-687. Do not promote it as an aggregate yaw-loss model.

- `MazeMap/MazeMap/PlantModel.cpp:913-970`
  - `wheelKinematics` already computes per-contact forward/right relative velocities in the contact-continuum form.
  - Keep this as the single kinematic source. If signed contact locations are needed by the correction, derive them locally inside PlantModel from `_vehicle.GetTrackWidth()` and `Vehicle::GetDriveWheelLongitudinalOffsetM()`, using the same sign convention.
  - Do not introduce slip ratio, slip angle, curvature, or radius.

- `MazeMap/MazeMap/PlantModel.cpp:1057-1161`
  - `ComputeFeedforward` currently includes `kYawRateDampingNmsPerRad` in requested yaw moment at lines 1097-1100.
  - If aggregate yaw damping is removed, remove this compensation too. Feedforward should not pre-compensate a rejected aggregate yaw-loss model.
  - Do not add maneuver-specific feedforward branches.

- `MazeMap/MazeMap/MotorEncoderDrive.h:20-35`
  - Existing wheel-bank inertia and tire stiffness/gain constants are drive-owned facts.
  - Do not change them for this yaw-identification correction unless the refit explicitly says the authoritative fix is a tire-bank stiffness/gain fact rather than PlantModel contact-equation correction.

- `MazeMap/MazeMap/Vehicle.h:39-83`
  - Existing mass, yaw inertia, track/contact geometry, fan downforce, and wheel radius remain Vehicle-owned facts.
  - Do not add empirical yaw/contact correction coefficients to Vehicle.

## Superseded Behavior

Superseded or blocked by this decision:

- Any nonzero use of `kYawRateDampingNmsPerRad` as aggregate yaw loss.
- Any residual yaw correction indexed by forward velocity and yaw rate.
- Any correction keyed by maneuver mode, in-place-turn label, slip angle, slip ratio, curvature, radius, or stationary/opposed-command thresholds.
- Any command rejection/clamp based only on force-envelope ratio.

Behavior not superseded:

- Motor command to wheel-bank torque through `MotorEncoderDrive`.
- Existing per-contact relative velocity kinematics.
- Existing force-envelope projection as a diagnostic/limiter in the plant equation.
- Residual acceleration state decay semantics.
- Vehicle ownership of construction facts.

## Test Plan

Update existing PlantModel-focused tests only. Do not create wrappers to expose internals.

Recommended test file: `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`

- Add `PlantModelContactContinuumYawCorrectionIsFiniteAcrossZeroForwardSpeed`.
  - Sweep `Vf` negative, zero, positive; `Vr` negative, zero, positive; yaw rates around calibration-turn values; wheel commands same-direction and opposed.
  - Assert finite contact relative velocities, contact forces, yaw acceleration, and integrated state.

- Add `PlantModelContactContinuumYawCorrectionIsContinuousAcrossZeroForwardSpeed`.
  - Use finite differences around `Vf = 0`.
  - Assert bounded yaw-acceleration deltas and no semantic branch change.

- Add `PlantModelContactContinuumYawCorrectionPreservesSymmetricStraightNoYawBias`.
  - Symmetric command, zero yaw rate, zero lateral velocity.
  - Assert near-zero yaw acceleration and near-zero right acceleration.

- Add `PlantModelContactContinuumYawCorrectionPreservesYawSign`.
  - Opposed or differential commands for clockwise and counter-clockwise yaw.
  - Assert correction does not flip expected yaw acceleration sign under ordinary finite commands.

- Update existing `PlantModelYawAccelerationIsSmoothAcrossLowForwardSpeeds` if the implementation changes expected tolerances.

Recommended test file: `MazeMap/MazeMapTest/PlantModelDriveCommandTest.cpp`

- Update feedforward tests only if `kYawRateDampingNmsPerRad` is removed from `ComputeFeedforward`.
- Keep `PlantModelAccelerationFeedforwardReturnsSplitCommandForYawRequest` and related sign tests authoritative for command direction.

Recommended test file: `MazeMap/MazeMapTest/DriveStack_PlantModelPhysicsTest.cpp`

- Preserve axis/sign convention tests:
  - `WheelYawSign_PositiveYawMakesLeftWheelLinearVelocityFaster`.
  - `WheelYawSign_EncoderYawMeasurementPreservesClockwisePositiveSign`.
  - `SymmetricDrive_NoYawAccelerationBias`.
  - finite numeric-stability tests.

Recommended test file: `MazeMap/MazeMapTest/DriveStack_VehiclePlantBoundaryTest.cpp`

- No new Vehicle boundary test unless the implementation changes Vehicle-owned construction facts. It should not.

Recommended test file: `MazeMap/MazeMapTest/MotorEncoderDriveTest.cpp`

- No new MotorEncoderDrive test unless the implementation changes drive-owned tire-bank facts. It should not for the PlantModel-only correction.

## Verification Plan

No tests were run for this design-only draft because production code was not edited.

When implementation starts:

1. Check worktree and active source timestamps before testing:

```powershell
git status --short
Get-Item -LiteralPath MazeMap\MazeMap\PlantModel.cpp,MazeMap\MazeMap\PlantModel.h,MazeMap\MazeMapTest\PlantModelDynamicsTest.cpp,MazeMap\MazeMapTest\PlantModelDriveCommandTest.cpp | Select-Object FullName,LastWriteTime
Get-Item -LiteralPath MazeMap\MazeMap\x64\Release\MazeMap.dll,MazeMap\MazeMapTest\x64\Release\MazeMapTest.dll | Select-Object FullName,LastWriteTime
```

2. If binaries are newer than all touched production/test sources and there is no reason to rebuild, run release verification without building:

```powershell
codex_verify\test_latest_binaries.cmd --no-pause
```

3. If touched sources are newer than active Release binaries, or if timestamp state is ambiguous, run the project verification path without cleaning or rebuilding from scratch:

```powershell
codex_verify\build_and_verify_latest.cmd --no-pause
```

4. Use a timeout of at least 20 minutes for incremental build/test commands. If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately and do not clean/rebuild.

5. Expected verification target is Release x64 unit tests plus the normal Teensy/host verification path handled by `build_and_verify_latest`.

## Implementation Recommendation

Do not implement a fitted production yaw-contact correction from the current ablation outputs alone.

The next behavior-changing production change should be one of these:

1. Preferred: collect/refit targeted data, then implement one PlantModel-local patch-force correction in `evaluateAppliedBankTorqueStep`, with private constants and direct PlantModel tests.
2. If the user accepts the risk now: implement only the minimal PlantModel-local correction described above, remove the aggregate yaw-damping path, and keep the coefficient set narrowly documented and easy to back out through ordinary code review, not through a runtime mode branch.

Do not implement aggregate yaw-loss, residual tables, or maneuver-conditioned plant behavior.
