# Fan/Vibration Filtering For Yaw-Launch Plant Identification

## Source

- Primary log: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.csv`
- Sidecar: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.sidecar`
- Fan duty from sidecar: `0.800`
- Phase counts: `{1: 15605, 2: 13430, 3: 37068, 20: 79541}`
- Static stationary rows used: `15605`
- Yaw-launch rows: `79541`
- Command steps found: `100`
- Sensor target: raw gyro and raw/body accelerometer channels only; UKF targets and UKF state were not used.

## Dominant Fan/Vibration Components

- The stationary phase is dominated by a narrow fan/vibration line near `149.2 Hz`.
- The per-step subtraction uses the stationary-derived fan frequency `149.250 Hz` and fits only local amplitude/phase from each stationary pre-window.
- Per-step gyro fan amplitude median: `0.006580 rad/s`; 10th/90th percentiles: `0.002808`/`0.029358 rad/s`.
- Per-step gyro second-harmonic fit median: `0.000427 rad/s`; it is small relative to the 149 Hz line but kept in the subtraction model.
- Accel X/Y show the same 149 Hz line at much larger physical amplitude, plus a visible harmonic near 298 Hz; encoders have no stationary fan line because stationary encoder rates are zero.

Top stationary gyro peaks:

| Rank | Frequency Hz | ASD | PSD |
| ---: | ---: | ---: | ---: |
| 1 | 149.414062 | 0.027986447 | 0.000783241222 |
| 2 | 151.855469 | 0.005432132 | 0.000029508053 |
| 3 | 146.484375 | 0.003700650 | 0.000013694809 |
| 4 | 154.296875 | 0.002439275 | 0.000005950063 |
| 5 | 144.042969 | 0.002110307 | 0.000004453395 |
| 6 | 141.601562 | 0.001472166 | 0.000002167271 |
| 7 | 156.738281 | 0.001437232 | 0.000002065635 |
| 8 | 174.316406 | 0.001239392 | 0.000001536093 |

Top stationary accel peaks:

| Channel | Rank | Frequency Hz | ASD |
| --- | ---: | ---: | ---: |
| accel_body_x_mps2 | 1 | 149.414062 | 0.229437122 |
| accel_body_x_mps2 | 2 | 151.855469 | 0.042929554 |
| accel_body_x_mps2 | 3 | 146.484375 | 0.029603800 |
| accel_body_x_mps2 | 4 | 154.296875 | 0.019474272 |
| accel_body_x_mps2 | 5 | 144.042969 | 0.016966695 |
| accel_body_x_mps2 | 6 | 141.601562 | 0.011723550 |
| accel_body_y_mps2 | 1 | 149.414062 | 0.265147588 |
| accel_body_y_mps2 | 2 | 151.855469 | 0.049785515 |
| accel_body_y_mps2 | 3 | 146.484375 | 0.034183716 |
| accel_body_y_mps2 | 4 | 154.296875 | 0.022638740 |
| accel_body_y_mps2 | 5 | 144.042969 | 0.019372400 |
| accel_body_y_mps2 | 6 | 156.738281 | 0.013444812 |

## Drift And Aliasing Risk

- Static 2.048 s segment gyro fan estimates span `149.000` to `149.300 Hz`.
- The fan period is about `6.7 ms`, directly overlapping the +4/+5 ms launch-onset region. That does not alias below Nyquist, but it aliases into the step-response measurement problem because fixed sample offsets can land on different fan phases from step to step.
- The 144 to 157 Hz shoulders in finite-window spectra are consistent with short-window leakage plus mild fan-speed/amplitude modulation. They should be treated as fan contamination, not as yaw plant dynamics.

## Filter Recommendation

- Recommended preprocessing for compact PlantModel identification: per-step pre-window sinusoid subtraction on `gyro_raw_radps`, using the stationary-derived 149 Hz fundamental and small 298 Hz harmonic.
- The fit uses only stationary samples before each command edge to estimate local amplitude/phase. It fits bias/trend only to isolate the sinusoid, then subtracts only the sinusoidal components, preserving low-frequency yaw dynamics and command-onset timing.
- This is offline analysis, but it has a causal-equivalent interpretation: estimate fan phase/amplitude from a stationary pre-window and hold those parameters through the measurement window. Do not deploy this as a runtime impulse-response mechanism.
- A broad zero-phase 142-158 Hz notch is not recommended as the primary identification preprocessing because it can remove real launch transient energy. Use it only as an audit if a later worker needs to bound residual narrowband contamination.

Downstream function entry points in `characterize_fan_vibration_filter.py`: `fit_tone_model`, `subtract_tone`, and the generated `yaw_launch_fan_filtered_samples.csv` measurement windows.

## Noise Floor And Onset Detectability

- Initial stationary phase adaptive fan subtraction reduces centered RMS by median `69.2%` on gyro, `72.4%` on accel X, and `71.9%` on accel Y over 2.048 s chunks.
- Median pre-window gyro centered RMS: raw `0.025123 rad/s`, filtered `0.023662 rad/s`, reduction `5.8%`.
- Median pre-window gyro P95 absolute residual: raw `0.037601 rad/s`, filtered `0.037273 rad/s`, reduction `0.9%`.

| Signal | Offset | Median value/threshold | P10 | P90 |
| --- | ---: | ---: | ---: | ---: |
| raw | 0 | -0.051980 | -0.346328 | 0.378627 |
| raw | 4 | 0.461978 | -0.088272 | 1.123520 |
| raw | 5 | 1.568386 | 0.602503 | 3.359573 |
| filtered | 0 | -0.051358 | -0.446148 | 0.424213 |
| filtered | 4 | 0.510572 | -0.100076 | 1.813670 |
| filtered | 5 | 1.757437 | 0.628594 | 5.392151 |

## Compact-Parameter Identifiability

The following uses the direction-normalized gyro slope from +4 through +10 samples after command onset as a compact initial yaw-acceleration proxy. It is not a final plant fit; it is an identifiability check for whether filtering improves a plausible physical-parameter observable.

| Signal | Steps | Slope-vs-command R2 | Mean +4..+10 yaw accel | Std |
| --- | ---: | ---: | ---: | ---: |
| raw | 100 | 0.850461 | 183.602091 | 47.676280 |
| filtered | 100 | 0.844180 | 183.498835 | 47.881430 |

- Filtering clearly removes the stationary fan component and moderately improves the +4/+5 onset threshold margin. It does not materially improve this simple slope-vs-command R2, which means fan removal is necessary cleanup but not by itself a compact PlantModel fit.
- Filtered raw gyro is reliable for estimating yaw launch timing, initial yaw acceleration, twitch/launch threshold, and repeatability of compact yaw/contact terms.
- Filtered accel X/Y are reliable for identifying and auditing vibration contamination and may help with timing corroboration. They are not primary yaw-torque observables here because the fan line is larger than the useful low-speed planar acceleration signal and because yaw-launch accelerometer response mixes body acceleration with IMU placement and contact transients.
- Encoders are reliable for stationary gating and for later drivetrain response checks. They do not show stationary fan contamination in this log.

## Caveats

- The sidecar records fan duty `0.800`; the frequency/amplitude conclusions should be re-estimated for other fan duties.
- The subtraction model is trained on stationary pre-windows. Moving-contact spectra can differ after launch, so residual fan content during motion should be audited before fitting high-frequency model terms.
- Transient contact/bristle breakaway near +4 to +10 ms is real plant content. Do not interpret the filtered signal as pure rigid-body response.
- The subtraction has no phase delay because it subtracts an evaluated sinusoid rather than applying a causal IIR/FIR filter. A conventional causal notch would introduce phase/group-delay concerns unless compensated offline.

## Outputs

- `fan_vibration_component_summary.csv`: stationary spectral peaks.
- `fan_vibration_drift_by_segment.csv`: static-phase 2.048 s fan frequency/amplitude drift.
- `fan_vibration_stationary_filter_check.csv`: adaptive stationary fan-subtraction check by 2.048 s chunk.
- `yaw_launch_fan_filter_step_metrics.csv`: per-step fan fit, noise-floor, onset, and slope metrics.
- `yaw_launch_onset_detectability_same_plus4_plus5.csv`: same/+4/+5 threshold-margin summary.
- `yaw_launch_filter_identifiability_summary.csv`: compact initial-slope repeatability and slope-vs-command audit.
- `yaw_launch_fan_filtered_samples.csv`: per-step measurement windows with raw and fan-filtered gyro/accel channels for downstream physical parameter identification.

## Reproduce

```powershell
python codex_analysis\full_impulse_response_characterization\fan_filter\characterize_fan_vibration_filter.py
```
