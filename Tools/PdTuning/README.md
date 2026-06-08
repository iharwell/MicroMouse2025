# PdTuning

`PdTuning.exe` evaluates candidate `DriveBaseTrackingTuning` gains against the current C++
`DriveBase` and `PlantModel` implementation. The tool keeps `Vehicle`,
`PlantModel`, feedforward, and physical parameters fixed; only these
`Config::kDriveBaseTrackingTuning` entries are varied:

- forward-axis position-to-acceleration gain
- forward-axis velocity-to-acceleration gain
- forward-axis acceleration-error gain
- yaw-axis position-to-acceleration gain
- yaw-axis velocity-to-acceleration gain
- yaw-axis acceleration-error gain

The evaluator advances the canonical `DriveBase::ProposeBodyTick(...)` path at
an explicit `0.001 s` command tick. For each command tick, the tool evaluates
the command once, then applies ten `0.0001 s` `PlantModel::integrate(...)`
substeps with that same command. JSON output also reports the outer command
cadence as `tick_seconds`.

The evaluator also runs release-test-style scenario coverage through
`SharedRobotRuntime`, `Drive`, `DriveBase`, `PlantModel::integrate(...)`, and
sensor snapshot publication. Completion and command-evidence checks report
blocker flags for release-test contract visibility. Tolerance-style maneuver
rows are informational during this tuning pass, and optimizer ranking uses the
continuous `maneuver_tracking_rms` row instead of weighted pass/fail penalties.

For model tuning, Python model investigations, and data-driven traction/model/
control acceptance, valid evidence must evaluate full-run adherence over
complete eligible runs. Do not accept or reject a controller, traction model,
plant model, or tuning change from isolated samples, one-row checks, short
windows, first-N tick metrics, launch snippets, quick checks, sampled feature
tables, binned short-window summaries, or UKF/estimator-derived targets. Those
outputs are useful diagnostics only. If a diagnostic window is used for tuning
evidence at all, it must cover at least `500 ticks` at the current command tick
length; smaller first-sample or first-N snippets are valid only for smoke/debug
visualization.

This minimum diagnostic-window rule does not constrain ordinary unit tests,
small deterministic unit tests, release-test-style code behavior checks, or
scenario fixtures that verify a specific code path. Those tests may stay as
small as the behavior under test requires; they simply are not standalone
data-driven model-acceptance evidence.

Build:

```powershell
msbuild Tools\PdTuning\PdTuning.vcxproj /p:Configuration=Release /p:Platform=x64 /m
```

Evaluate the current baseline:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe
```

Evaluate explicit gains, with omitted values defaulting to
`Config::kDriveBaseTrackingTuning`:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe `
  --forward-position-to-accel-gain 0 `
  --forward-velocity-to-accel-gain 30 `
  --forward-acceleration-error-gain 0.02 `
  --yaw-position-to-accel-gain 2000 `
  --yaw-velocity-to-accel-gain 45 `
  --yaw-acceleration-error-gain 6
```

Run the built-in bounded coordinate search:

```powershell
Tools\PdTuning\x64\Release\PdTuning.exe --search --search-passes 4 --search-points 9 `
  --forward-position-to-accel-gain-min 0 --forward-position-to-accel-gain-max 20 `
  --forward-velocity-to-accel-gain-min 1 --forward-velocity-to-accel-gain-max 20 `
  --forward-acceleration-error-gain-min 0 --forward-acceleration-error-gain-max 1 `
  --yaw-position-to-accel-gain-min 100 --yaw-position-to-accel-gain-max 2000 `
  --yaw-velocity-to-accel-gain-min 100 --yaw-velocity-to-accel-gain-max 1000 `
  --yaw-acceleration-error-gain-min 0 --yaw-acceleration-error-gain-max 1
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
sets `acceptance_blocked=true` and makes the process return failure, but the
numeric ranking score remains a tuning heuristic, not standalone acceptance
evidence.

## Scenario Envelope

The evaluator stresses `DriveBaseTrackingTuning` through the same `DriveBase::ProposeBodyTick`
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

Each scenario reports a `step_response` object. Step responses can take multiple
samples to appear on sensors, so first-sample, first-window, and short-window
metrics are diagnostic only and cannot be used as model-acceptance evidence.
These metrics are not promoted to release acceptance, and any metric used as
model-tuning evidence must span at least `500 ticks` at the current command tick
length:

- `velocity_error_first_500_ticks`: RMS of `targetForwardMps - state.u` over the
  first 500 ticks after a forward velocity step.
- `yaw_rate_error_first_500_ticks`: RMS of `targetYawRateRadps - state.r` over
  the first 500 ticks after a yaw-rate step.
- `forward_accel_error_first_100_ticks`: RMS of
  `(nextU - prevU) / 0.001 - DriveTelemetry.composedForwardAccelMps2` over the
  first 100 ticks after a forward velocity step when `DriveTelemetry` publishes
  a finite composed forward-acceleration objective. If the objective is inactive
  or non-finite, fallback samples use RMS of `(nextU - prevU) / 0.001` as
  undesired forward acceleration/roughness. This legacy first-100 field is
  smoke/debug-only under the minimum diagnostic-window rule.
- `yaw_accel_error_first_100_ticks`: RMS of
  `(nextR - prevR) / 0.001 - DriveTelemetry.composedYawAccelRadps2` over the
  first 100 ticks after a yaw-rate or heading step when `DriveTelemetry`
  publishes a finite composed yaw-acceleration objective. If the objective is
  inactive or non-finite, fallback samples use RMS of `(nextR - prevR) / 0.001`
  as undesired yaw acceleration/roughness. This legacy first-100 field is
  smoke/debug-only under the minimum diagnostic-window rule.

The tuning-score weights for valid diagnostic terms are 60 for early velocity
RMS and 60 for early yaw-rate RMS. Existing high-performance envelope, response
failure, and oscillation penalties remain part of the diagnostic tuning score.
First-100 acceleration snippets must not be used as tuning evidence until the
tool is updated to emit a minimum `500 tick` diagnostic acceleration window.

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
  minimum. These threshold rows are informational for the current tracking ripple
  tuning pass: they do not block candidate acceptance and do not contribute
  score.
- DriveManeuver smooth coverage: `S45LS`, `S45LD`, `S45SS`, `S45SD`, `S90LS`,
  `S90SS`, `S90SD`, `S135LS`, `S135LD`, `S135SS`, `S135SD`, `S180LS`, and
  `S180SS`, each with completion, command-evidence, velocity-variation,
  yaw-acceleration-variation, yaw-rate-variation, final-position, and
  final-heading gates. The variation, final-position, and final-heading
  threshold rows are informational for the current tracking ripple tuning pass.

The maneuver checks use the same smooth-entry and sensor publication path as
`DriveManeuverTests`, without importing the test harness. `PdTuning` records one
tracking sample per outer command tick after the plant substeps finish. The
diagnostic tuning row is `maneuver_tracking_rms`: the full-run sum of squared
RMS error ratios for cumulative distance, instantaneous heading, forward
velocity, yaw rate, forward acceleration, and yaw acceleration.
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

Current `DriveBase` supplies no forward position error, so the forward-axis
position-to-acceleration gain is accepted and reported but is expected to be
score-insensitive until that controller path changes. Acceleration-error gains
are active only when the corresponding target acceleration is finite.
