# Yaw Launch Compact Plant Identification Evidence

## Source And Boundary

- Primary log: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.csv`
- Sidecar: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.sidecar`
- Phase 20 mapping: `Yaw Launch`
- Fan duty metadata: `0.800`
- Rows read: `145644`; phase counts: `{1: 15605, 2: 13430, 3: 37068, 20: 79541}`
- Yaw-launch command steps: `100`
- Scope: scratch analysis only. Production code and tests were not modified.
- Targets: raw gyro minus independently estimated static bias, filtered only to remove fan/vibration before identification. UKF targets are not used.

## Provisional Fan/Vibration Filter

- No IR-A fan-filter output was present, so this run used a replaceable provisional filter.
- Filter: forward/backward single-pole low-pass at `160.0` Hz over the bias-removed raw gyro. This is offline zero-phase conditioning, not a runtime proposal.
- Static bias: `0.001231439` rad/s from `15605` static rows.
- Static raw std: `0.027680149` rad/s; filtered static std: `0.010630718` rad/s; removed high-frequency std: `0.017153244` rad/s.

## Measurement Protocol Structure

- Known timing: primary command-to-first-gyro-response alignment is `+4` samples; derivative/onset sanity is `+5` samples.
- Quality classes: `{'twitch_static_bristle': 40, 'transitional': 22, 'sustained_launch': 38}`.
- Train/validation split: repeats 3, 7, and 10 are validation; all other repeats train the evidence kernels.
- Amplitudes/signs are preserved. Combined-direction kernels are direction-normalized evidence only and are not presented as runtime tables.

## Compact PlantModel Interpretation

### input_to_gyro_timing

- Inferred range: primary 4 ms; derivative/onset sanity 5 ms
- Evidence: Prior measured alignment plus this filtered replay; same/+1/+2 samples remain baseline and first consistent rise is +4.
- PlantModel implication: Use a compact command/torque or measurement-effective-time delay term; do not amplitude-schedule the delay except for explicit twitch-vs-sustained diagnostics.

### motor_torque_buildup

- Inferred range: initial yaw accel median 143.2 rad/s^2; sustained median 186.6 rad/s^2; effective yaw moment 0.046008104 Nm
- Evidence: Initial acceleration is measured from filtered gyro at +4..+10 ms after command onset.
- PlantModel implication: Represent as a short torque/current rise or launch torque lag before the contact model sees full bank torque.

### static_bristle_twitch_resistance

- Inferred range: 0.50/0.55 commands: peak yaw 1.179 rad/s, steady 0.012 rad/s, command-area 0.0412 rad
- Evidence: Twitch-only pulses move initially but decay/arrest while command remains on.
- PlantModel implication: Add low-speed bristle/static displacement or launch-resistance state/term rather than a scalar gain update.

### breakaway_sustained_resistance

- Inferred range: CW breakaway near command 0.610; CCW breakaway near command 0.650; transitional samples 22
- Evidence: 0.60 is transitional; 0.65/0.70 mostly sustain, with sign-dependent strength.
- PlantModel implication: Use a smooth breakaway/resistance curve over command-derived torque and low wheel speed, not a hard mode branch.

### damping_relaxation

- Inferred range: sustained t63 median 17.3 ms; recovery tau37 median 16.5 ms
- Evidence: Rise and post-command decay are computed from filtered, direction-normalized yaw-rate traces.
- PlantModel implication: Fit yaw/contact damping and relaxation so the model explains both rise and off-recovery, not only steady yaw rate.

### amplitude_sign_asymmetry

- Inferred range: steady CW/CCW ratio: 0.60 transitional 1.72, 0.65 sustained 1.48, 0.70 sustained 0.93
- Evidence: The same measurement sequence alternates command sign at each amplitude/repeat.
- PlantModel implication: Before adding a yaw table, test left/right torque scale, static threshold, and contact-force asymmetry parameters.

### deterministic_error_upper_bound

- Inferred range: validation empirical response derivative RMSE 0.019766833 rad/s vs current PlantModel 0.138108255 rad/s; vs no-response 0.032628416 rad/s; variance explained vs current PlantModel 97.951505%, vs no-response 63.298636%
- Evidence: Condition responses are trained on repeats not in validation and evaluated one-step with the known +4 sample lag.
- PlantModel implication: A compact model that captures the same deterministic response structure plausibly removes a large share of yaw-launch prediction error. The current PlantModel mirror overpredicts these windows enough that even no-response is closer; the empirical response still improves no-response RMSE by 39.418349%.

## Condition Summary

| Amp | Direction | Quality | Steps | Initial accel | Peak yaw | Steady yaw | t63 ms | Recovery tau37 ms |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.50 | CCW | twitch_static_bristle | 10 | 91.776582 | 1.020226 | 0.010710 | nan | nan |
| 0.50 | CW | twitch_static_bristle | 10 | 111.511072 | 1.262358 | 0.011979 | nan | nan |
| 0.55 | CCW | twitch_static_bristle | 10 | 116.228168 | 1.108399 | 0.015319 | nan | nan |
| 0.55 | CW | twitch_static_bristle | 10 | 128.328730 | 1.480807 | 0.023389 | nan | nan |
| 0.60 | CCW | transitional | 10 | 144.973769 | 1.752063 | 0.352652 | nan | 5.000 |
| 0.60 | CW | transitional | 10 | 143.195982 | 1.885011 | 0.605999 | 6.564 | 6.500 |
| 0.65 | CCW | sustained_launch | 8 | 157.016207 | 3.077762 | 1.764144 | 9.074 | 8.000 |
| 0.65 | CCW | transitional | 2 | 166.887075 | 3.719829 | 0.572989 | 5.150 | 5.000 |
| 0.65 | CW | sustained_launch | 10 | 175.852275 | 3.105378 | 2.608923 | 13.502 | 14.500 |
| 0.70 | CCW | sustained_launch | 10 | 206.394061 | 6.590036 | 5.961972 | 119.047 | 23.000 |
| 0.70 | CW | sustained_launch | 10 | 204.689514 | 7.016487 | 5.571876 | 38.697 | 20.000 |

## Prediction Error Upper Bound

- Validation one-step derivative RMSE using current PlantModel mirror: `0.138108255` rad/s.
- Validation one-step derivative RMSE using no-response derivative baseline: `0.032628416` rad/s.
- Validation one-step derivative RMSE using held-out condition response evidence: `0.019766833` rad/s.
- Upper-bound RMSE reduction: `85.687435%`; variance explained: `97.951505%`.
- Interpretation: this measures deterministic structure available to a compact physical model. It is not a recommendation to deploy an impulse response.

## Artifacts

- `per_step_aligned_filtered_traces.csv`: aligned evidence traces, including raw-minus-bias, filtered gyro, removed vibration, encoders, commands, and quality class.
- `per_step_response_metrics.csv`: per-step compact-response metrics.
- `condition_summary_metrics.csv`: per-amplitude/sign/quality aggregates plus repeat/noise trend slopes.
- `per_condition_step_kernels.csv`: averaged direction-normalized response evidence by condition.
- `train_condition_step_kernels.csv`: training-only averaged evidence used for validation prediction.
- `prediction_error_upper_bound.csv`: current PlantModel mirror vs held-out empirical response one-step error estimates.
- `compact_model_parameter_ranges.csv`: compact PlantModel terms and inferred parameter ranges.

## Reproduce

```powershell
python codex_analysis\full_impulse_response_characterization\step_kernels\extract_yaw_launch_compact_identification.py
```
