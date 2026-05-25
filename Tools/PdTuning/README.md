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

The evaluator advances the canonical `DriveBase::ProposeBodyTick(...)` path at
an explicit `0.001 s` command tick. For each command tick, the tool evaluates
the command once, then applies ten `0.0001 s` `PlantModel::integrate(...)`
substeps with that same command. JSON output also reports the outer command
cadence as `tick_seconds`.

The evaluator also runs release-test-style acceptance scenarios through
`SharedRobotRuntime`, `Drive`, `DriveBase`, `PlantModel::integrate(...)`, and
sensor snapshot publication. Completion and command-evidence checks report
blocker flags for release-test contract visibility. Tolerance-style maneuver
rows are informational during this tuning pass, and optimizer ranking uses the
continuous `maneuver_tracking_rms` row instead of weighted pass/fail penalties.

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
Tools\PdTuning\x64\Release\PdTuning.exe --search --search-passes 4 --search-points 9 `
  --velocity-kp-min 1 --velocity-kp-max 20 `
  --velocity-kd-min 0 --velocity-kd-max 1 `
  --heading-kp-min 100 --heading-kp-max 2000 `
  --heading-kd-min 0 --heading-kd-max 1 `
  --yawrate-kp-min 100 --yawrate-kp-max 1000 `
  --yawrate-kd-min 0 --yawrate-kd-max 1
```

The search grid is logarithmic for positive gain values and keeps zero as an
explicit candidate. After each coordinate pass, the next range brackets the
current best value using neighboring log-spaced samples. `--search-grid N`
remains accepted as an alias for `--search-points N`. Any parameter with an
explicit search bound must provide both the matching `--*-min` and `--*-max`
options; omitted bounds fall back to the tool's derived compatibility ranges.
The JSON `search.bounds` object reports the effective range and whether it was
explicit or derived.

Stdout is JSON. The output includes baseline metrics, candidate metrics, and,
when `--search` is used, the best candidate and top candidate list.

Each evaluation includes `acceptance_scenarios`. A blocking acceptance scenario
sets `acceptance_blocked=true` and makes the process return failure, but only
the `maneuver_tracking_rms` row contributes acceptance score.

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
  first 100 ticks after a forward velocity step when `DriveTelemetry` publishes
  a finite composed forward-acceleration objective. If the objective is inactive
  or non-finite, fallback samples use RMS of `(nextU - prevU) / 0.001` as
  undesired forward acceleration/roughness.
- `yaw_accel_error_first_100_ticks`: RMS of
  `(nextR - prevR) / 0.001 - DriveTelemetry.composedYawAccelRadps2` over the
  first 100 ticks after a yaw-rate or heading step when `DriveTelemetry`
  publishes a finite composed yaw-acceleration objective. If the objective is
  inactive or non-finite, fallback samples use RMS of `(nextR - prevR) / 0.001`
  as undesired yaw acceleration/roughness.

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
  Non-completion is a blocker but does not contribute acceptance score.
- DriveManeuver in-place coverage: `IP45`, `IP90`, `IP135`, and `IP180`, each
  with completion, command-evidence, shift, heading, and elapsed-time gates.
  Shift is reported against 0.020 m, heading error is reported against
  3 degrees, and elapsed time is reported against the `MotionLimits` kinematic
  minimum. These threshold rows are informational for the current PD ripple
  tuning pass: they do not block candidate acceptance and do not contribute
  score.
- DriveManeuver smooth coverage: `S45LS`, `S45LD`, `S45SS`, `S45SD`, `S90LS`,
  `S90SS`, `S90SD`, `S135LS`, `S135LD`, `S135SS`, `S135SD`, `S180LS`, and
  `S180SS`, each with completion, command-evidence, velocity-variation,
  yaw-acceleration-variation, yaw-rate-variation, final-position, and
  final-heading gates. The variation, final-position, and final-heading
  threshold rows are informational for the current PD ripple tuning pass.

The maneuver checks use the same smooth-entry and sensor publication path as
`DriveManeuverTests`, without importing the test harness. `PdTuning` records one
tracking sample per outer command tick after the plant substeps finish. The
scored maneuver row is `maneuver_tracking_rms`: the sum of squared RMS error
ratios for cumulative distance, instantaneous heading, forward velocity, yaw
rate, forward acceleration, and yaw acceleration.
Smooth maneuver variation rows also report `ripple_oscillatory`. Any normalized
velocity, yaw-acceleration, or yaw-rate variation above `0.001` is treated as
real ripple for tuning and contributes to the aggregate `oscillation_flagged`
value.

## Oscillation Metrics

Each scenario reports an `oscillation` object:

- `sign_changes_after_first_crossing`: target-error sign changes after the first
  target crossing, with a small deadband to avoid counting numerical noise.
- `late_window_peak_to_peak_error`: error span over the final 25% of the
  scenario, with a minimum 250 ms late window.
- `late_window_rms_error`: RMS error over that same late window.
- `oscillatory`: true for a scenario when repeated target crossings continue
  with material late-window ringing, or when crossing count is severe even if
  the late window has decayed.
- `penalty`: scalar score penalty from crossing count, late-window residual
  motion, and a large fixed penalty for scenarios classified as oscillatory.

The top-level `oscillation_flagged` value is broader than a single scenario's
target-crossing classifier. It is true when any scenario is oscillatory or when
any smooth maneuver variation row reports `ripple_oscillatory=true`.

Current `DriveBase` uses `VelocityStatePD` and `YawRateStatePD` with a zero
error-rate argument, so their `kd` values are accepted and reported but are
expected to be score-insensitive until that controller path changes. Heading
`kd` is active through the yaw-rate error term.
