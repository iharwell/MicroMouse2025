# Yaw Launch Delay Calibration

## Source

- Primary log: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.csv`
- Sidecar: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.sidecar`
- Phase map: {1: 'static', 2: 'launch', 3: 'straight', 4: 'yaw', 5: 'smooth', 6: 'loop_clockwise', 7: 'loop_counter_clockwise', 20: 'Yaw Launch'}
- Phase row counts: {1: 15605, 2: 13430, 3: 37068, 20: 79541}
- Delay target: off-to-on yaw-launch command edge to measured raw-gyro response.
- Sensor target: `gyro_raw_radps` minus independent stationary bias; no UKF targets used.

## Counts And Quality

- Independent static gyro bias: `0.001231439` rad/s from `15605` static rows; static population std `0.027680149` rad/s.
- Yaw-launch rows: `79541`; command steps: `100`.
- Saturation flags inside yaw-launch phase: {0: 79541}.
- Delay quality flag counts: {'clean_onset': 77, 'no_derivative_onset': 23, 'sustained_launch': 38, 'twitch_only': 43}.
- Pre-baselines are gathered across the phase boundary where needed. The first yaw-launch step has the long stationary pre-window before `phase_id=20`, not just the single phase marker row.

## Method

- For each command edge, the analyzer direction-normalizes the gyro response and subtracts a local stationary baseline.
- Threshold onset uses a robust per-step noise threshold: `max(0.05 rad/s, p99(pre)+0.01, 4*MAD_sigma(pre))`, then requires 3 sustained samples above threshold and reports a fractional crossing.
- Motion onset uses a lower robust threshold: `max(0.03 rad/s, p95(pre)+0.005, 2.5*MAD_sigma(pre))`, then requires 2 sustained samples above threshold. This is intended to estimate first measurable response rather than later high-confidence sustained motion.
- Derivative onset uses robust pre-baseline derivative noise, requires 2 of 3 derivative samples above threshold, and requires a following value response.
- Piecewise ramp onset fits the first 18 samples to a delayed positive ramp and reports the best integer onset.
- Template alignment is computed per step for audit, but the recommendation is based on threshold, derivative, and ramp onset because the template is learned from the same delayed data.

## Same Through +6 Samples

| Amp | Dir | Steps | Sustained | Twitch | Motion samples | Sustained-threshold samples | Deriv samples | Ramp samples | y+0 | y+1 | y+2 | y+3 | y+4 | y+5 | y+6 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.50 | CCW | 10 | 0 | 10 | 8.483 | 13.719 | nan | 2.000 | -0.005376 | -0.017471 | -0.018448 | -0.001710 | 0.043127 | 0.135001 | 0.257419 |
| 0.50 | CW | 10 | 0 | 10 | 6.724 | 7.338 | 5.000 | 2.000 | 0.013561 | 0.005131 | -0.008552 | -0.009041 | 0.025412 | 0.121562 | 0.264993 |
| 0.55 | CCW | 10 | 0 | 10 | 7.902 | 12.968 | 6.000 | 2.000 | -0.002993 | 0.006292 | 0.012156 | 0.009957 | 0.021075 | 0.094990 | 0.242330 |
| 0.55 | CW | 10 | 0 | 10 | 7.121 | 7.731 | 6.000 | 3.000 | -0.009957 | -0.001405 | 0.006658 | 0.012889 | 0.028527 | 0.082039 | 0.218873 |
| 0.60 | CCW | 10 | 0 | 3 | 7.683 | 9.262 | 5.500 | 2.500 | -0.001466 | -0.013806 | -0.014172 | -0.001955 | 0.038607 | 0.143065 | 0.314107 |
| 0.60 | CW | 10 | 0 | 0 | 7.280 | 8.006 | 5.500 | 3.000 | 0.001161 | -0.001772 | -0.003482 | -0.003482 | 0.009957 | 0.090958 | 0.250149 |
| 0.65 | CCW | 10 | 8 | 0 | 6.312 | 8.554 | 5.000 | 2.000 | 0.000672 | 0.003360 | 0.003360 | 0.009957 | 0.077763 | 0.260412 | 0.529559 |
| 0.65 | CW | 10 | 10 | 0 | 6.274 | 6.859 | 5.000 | 3.000 | 0.007330 | -0.004154 | -0.010507 | -0.000855 | 0.047770 | 0.190712 | 0.425895 |
| 0.70 | CCW | 10 | 10 | 0 | 5.484 | 7.042 | 4.000 | 2.000 | -0.005192 | 0.001894 | 0.007391 | 0.007391 | 0.060048 | 0.215574 | 0.509401 |
| 0.70 | CW | 10 | 10 | 0 | 6.085 | 6.733 | 4.000 | 2.000 | 0.003482 | -0.001649 | -0.006414 | -0.003482 | 0.056994 | 0.217651 | 0.495351 |

## Calibration Sets

| Set | Steps | Motion samples | Sustained-threshold samples | Derivative samples | Ramp samples | Recommended offset |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| all_clean | 100 | 7.059 | 7.825 | 5.000 | 2.000 | 4 |
| sustained_only | 38 | 6.127 | 6.853 | 5.000 | 2.000 | 4 |
| above_threshold_0p65_0p70 | 40 | 6.127 | 6.883 | 5.000 | 2.000 | 4 |
| exclude_twitch_only | 57 | 6.646 | 7.278 | 5.000 | 3.000 | 4 |

## Delay Recommendation

- Recommended integer alignment for remaining tuning: `+4` samples.
- Use `+4 ms` as the command-to-first-gyro-response offset at the 1 kHz control/log rate. Treat `+5` as the conservative derivative/motion-threshold onset and `+7` as the high-confidence sustained-threshold crossing.
- Keep this offset global for yaw-launch and similar low-speed command-step replay. Do not make it amplitude-dependent for tuning unless the model explicitly separates below-threshold twitch-only behavior from sustained launch behavior.
- Exclude 0.50 and 0.55 twitch-only steps from sustained-launch alignment calibration. They are useful for static/bristle threshold identification, but their arrested response biases template and threshold fits toward twitch dynamics.

## Gyro Cutoff Context

- A 214 Hz single-pole equivalent has a low-frequency time constant/group-delay scale of about `0.744` ms.
- Observed first nonbaseline response is around +4 samples, while robust sustained threshold and derivative methods center around +5 samples.
- That leaves roughly 3 to 4 ms beyond the simple gyro filter scale, attributable to command/PWM timing, motor current and torque buildup, contact/bristle breakaway, and finite thresholding of a filtered ramp.

## Outputs

- `yaw_launch_delay_per_step.csv`: per-step threshold, derivative, ramp, template, and +0..+15 response samples.
- `yaw_launch_delay_summary.csv`: aggregate delay by amplitude and direction.
- `yaw_launch_delay_recommendation.csv`: calibration-set recommendations.

## Reproduce

```powershell
python codex_analysis\yaw_launch_step_response\estimate_yaw_launch_delay.py
```
