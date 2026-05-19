# PdTuning

`PdTuning.exe` evaluates candidate `PDCluster` gains against the current C++
`DriveBase` and `PlantModel` implementation. The tool keeps `Vehicle`,
`PlantModel`, feedforward, and physical parameters fixed; only these
`Config::kDriveBasePDCluster` entries are varied:

- `VelocityStatePD.kp`
- `VelocityStatePD.kd`
- `HeadingStatePD.kp`
- `HeadingStatePD.kd`
- `YawRateStatePD.kp`
- `YawRateStatePD.kd`

The evaluator advances the canonical `DriveBase::ProposeBodyTick(...)` plus
`PlantModel::integrate(...)` path at an explicit `0.001 s` tick. JSON output
also reports this as `tick_seconds`.

The evaluator also runs release-test-style acceptance scenarios through
`SharedRobotRuntime`, `Drive`, `DriveBase`, `PlantModel::integrate(...)`, and
sensor snapshot publication. These checks report blocker flags for release-test
contract visibility, but optimizer ranking uses strong continuous score terms
for the underlying metric ratios instead of flat pass/fail penalties. Candidates
therefore get useful scoring pressure while approaching, crossing, and improving
beyond each mission-critical threshold.

Build:

```powershell
msbuild Tools\PdTuning\PdTuning.vcxproj /p:Configuration=Release /p:Platform=x64 /m
```

Evaluate the current baseline:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe
```

Evaluate explicit gains, with omitted values defaulting to
`Config::kDriveBasePDCluster`:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe `
  --velocity-kp 30 --velocity-kd 0.02 `
  --heading-kp 18 --heading-kd 3 `
  --yawrate-kp 45 --yawrate-kd 6
```

Run the built-in bounded coordinate search:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe --search --search-passes 4 --search-grid 9
```

Stdout is JSON. The output includes baseline metrics, candidate metrics, and,
when `--search` is used, the best candidate and top candidate list.

Each evaluation includes `acceptance_scenarios`. A failed acceptance scenario
sets `acceptance_blocked=true`, adds its normalized score contribution, and
makes the process return failure.

## Scenario Envelope

The evaluator stresses `PDCluster` through the same `DriveBase::ProposeBodyTick`
and `PlantModel::integrate` path used by the previous tool, but the scenarios
now cover the real high-performance operating envelope instead of small
perturbations:

- `forward_launch_0_to_4_mps`: zero to 4 m/s forward-speed capture.
- `forward_brake_4_to_0_mps`: 4 m/s braking capture to zero.
- `forward_high_speed_disturbance_4_to_2_mps`: nonzero-speed target disturbance.
- `yaw_rate_max_effort_0_to_9_radps`: in-place maximum-rate turn capture.
- `yaw_rate_reversal_9_to_minus9_radps`: maximum-rate in-place reversal.
- `combined_3mps_16p5mps2_turn`: 3 m/s with 5.5 rad/s yaw rate, representing
  16.5 m/s^2 kinematic lateral acceleration.
- `combined_3mps_heading_correction_15deg`: 15 degree heading capture while
  holding 3 m/s.

Each scenario JSON entry includes a `definition` object with initial conditions,
target commands, and target kinematic lateral acceleration when applicable.
Measured envelope fields include peak forward velocity, yaw rate, forward
acceleration, yaw acceleration, and kinematic lateral acceleration.
Scoring also includes `minimum_abs_error`, `response_reduction_fraction`, and
`response_failure_penalty` so a stable candidate that simply does not respond
cannot beat an active controller.

## Step-Response Metrics

Each scenario reports a `step_response` object. These metrics are normalized and
heavily weighted in the scalar score, not just emitted for inspection:

- `velocity_error_first_500_ticks`: RMS of `targetForwardMps - state.u` over the
  first 500 ticks after a forward velocity step.
- `yaw_rate_error_first_500_ticks`: RMS of `targetYawRateRadps - state.r` over
  the first 500 ticks after a yaw-rate step.
- `forward_accel_error_first_100_ticks`: RMS of
  `(nextU - prevU) / 0.001 - DriveTelemetry.composedForwardAccelMps2` over the
  first 100 ticks after a forward velocity step when the composed acceleration
  objective is finite and active. If the objective is inactive or non-finite,
  fallback samples use RMS of `(nextU - prevU) / 0.001` as undesired forward
  acceleration/roughness.
- `yaw_accel_error_first_100_ticks`: RMS of
  `(nextR - prevR) / 0.001 - DriveTelemetry.composedYawAccelRadps2` over the
  first 100 ticks after a yaw-rate or heading step when the composed yaw
  acceleration objective is finite and active. If the objective is inactive or
  non-finite, fallback samples use RMS of `(nextR - prevR) / 0.001` as
  undesired yaw acceleration/roughness.

The score weights for these normalized terms are 60 for early velocity RMS, 60
for early yaw-rate RMS, 40 for early forward-acceleration RMS, and 40 for early
yaw-acceleration RMS. Existing high-performance envelope, response failure, and
oscillation penalties remain part of the score.

## Acceptance Scenarios

Acceptance scenarios protect release-test contracts that the high-performance
envelope alone does not see:

- `drive_primitive_start_straight_completes`: runs
  `Drive::StartStraight(0.30 m, 0.30 m/s, 0.0 m/s exit)` through
  `SharedRobotRuntime::DriveService()` for up to 6000 exact 0.001 s ticks. It
  emits completion, elapsed ticks, final pose, and average encoder distance.
  Non-completion is a blocker and scores as a completion deficit.
- DriveManeuver in-place coverage: `IP45`, `IP90`, `IP135`, and `IP180`, each
  with completion, command-evidence, shift, heading, and elapsed-time gates.
  Shift must stay below 0.020 m, heading error must stay at or below 3 degrees,
  and elapsed time must stay within 40% of the `MotionLimits` kinematic minimum.
- DriveManeuver smooth coverage: `S45LS`, `S45LD`, `S45SS`, `S45SD`, `S90LS`,
  `S90SS`, `S90SD`, `S135LS`, `S135LD`, `S135SS`, `S135SD`, `S180LS`, and
  `S180SS`, each with completion, command-evidence, velocity-variation,
  yaw-acceleration-variation, yaw-rate-variation, final-position, and
  final-heading gates. Position error must stay at or below 0.030 m and heading
  error must stay at or below 3 degrees.

The maneuver checks use the same smooth-entry, sample extraction, and per-tick
encoder/gyro estimator update pattern as `DriveManeuverTests`, without
importing the test harness. This keeps `PdTuning` focused on the
candidate-dependent `Drive`/`DriveBase`/plant path while exercising the
canonical runtime service and the full maneuver release-test metric matrix.
Each maneuver metric also contributes a normalized squared score using its
release-test tolerance or variation limit, so the search can tune toward lower
shift, heading error, timing error, position error, and command variation even
when a candidate has not yet passed the threshold.

## Oscillation Metrics

Each scenario reports an `oscillation` object:

- `sign_changes_after_first_crossing`: target-error sign changes after the first
  target crossing, with a small deadband to avoid counting numerical noise.
- `late_window_peak_to_peak_error`: error span over the final 25% of the
  scenario, with a minimum 250 ms late window.
- `late_window_rms_error`: RMS error over that same late window.
- `oscillatory`: true when repeated target crossings continue with material
  late-window ringing, or when crossing count is severe even if the late window
  has decayed.
- `penalty`: scalar score penalty from crossing count, late-window residual
  motion, and a large fixed penalty for scenarios classified as oscillatory.

Current `DriveBase` uses `VelocityStatePD` and `YawRateStatePD` with a zero
error-rate argument, so their `kd` values are accepted and reported but are
expected to be score-insensitive until that controller path changes. Heading
`kd` is active through the yaw-rate error term.
