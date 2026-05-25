# Yaw Launch Step Response Extraction

## Source

- Primary log: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.csv`
- Sidecar: `C:\Users\thene\source\repos\MicroMouse2025\TestResults\mmlog_decode_2026-05-04_20-35-47\open_floor_main.sidecar`
- Selection: newest decoded `TestResults/mmlog_decode_*` directory with `open_floor_main.csv`; sidecar maps `phase_id=20` to `Yaw Launch`.
- Raw `D:\open_floor_main.mmlog` was not decoded because the decoded copy is present and matches the expected current yaw-launch source.
- Phase map: {1: 'static', 2: 'launch', 3: 'straight', 4: 'yaw', 5: 'smooth', 6: 'loop_clockwise', 7: 'loop_counter_clockwise', 20: 'Yaw Launch'}
- Phase row counts: {1: 15605, 2: 13430, 3: 37068, 20: 79541}

## Bias And Data Quality

- Independent stationary gyro bias: mean `0.001231439` rad/s from `15605` static phase rows; median `-0.000000000`, population std `0.027680149`.
- Yaw-launch rows: `79541` over `79.540` s.
- Command steps found: `100`; clean aggregate steps: `100`; excluded from aggregate: `0`.
- Quality flag counts: {'impulse_only_not_sustained': 43, 'sustained_launch': 38}.
- Pre-baselines are gathered from the full log, not just inside phase 20, while requiring zero drive commands and stationary encoder evidence.
- Phase-level mode flags: {4: 35000, 8: 44541}; saturation flags: {0: 79541}; stale sensor rows over 3 ms: `0`.

## Alignment

- Direction-normalized yaw-rate delta around onset: +0=0.0000, +1=-0.0025, +2=-0.0033, +3=0.0019, +4=0.0408, +5=0.1551, +6=0.3507, +7=0.5966, +8=0.8198, +9=0.9657.
- Same/+1/+2 samples remain effectively baseline. The first consistent sensor rise is +4 to +6 samples, so +2 is a better command/model alignment than same-sample but still precedes most of the measured gyro step response.

## Aggregate Step Metrics

| Amp | Dir | Steps | Sustained | Impulse-only | Delay ms | Initial accel rad/s^2 | Peak rad/s | Steady rad/s | t63 ms |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.50 | CCW | 10 | 0 | 10 | 5.000 | 47.667886 | 1.047693 | 0.011431 | 3.000 |
| 0.50 | CW | 10 | 0 | 10 | 5.500 | 59.408345 | 1.285258 | 0.010441 | 4.000 |
| 0.55 | CCW | 10 | 0 | 10 | 6.000 | 50.871528 | 1.149068 | 0.016886 | 2.500 |
| 0.55 | CW | 10 | 0 | 10 | 6.000 | 72.966525 | 1.508738 | 0.027013 | 3.500 |
| 0.60 | CCW | 10 | 0 | 3 | 5.500 | 77.778708 | 1.707617 | 0.377472 | 6.000 |
| 0.60 | CW | 10 | 0 | 0 | 6.000 | 91.780589 | 1.917816 | 0.609082 | 7.500 |
| 0.65 | CCW | 10 | 8 | 0 | 5.000 | 92.874834 | 3.461261 | 1.499870 | 8.000 |
| 0.65 | CW | 10 | 10 | 0 | 5.000 | 105.787676 | 3.043346 | 2.505758 | 14.000 |
| 0.70 | CCW | 10 | 10 | 0 | 5.000 | 122.146539 | 6.776945 | 6.046248 | 119.500 |
| 0.70 | CW | 10 | 10 | 0 | 5.000 | 135.238711 | 7.081098 | 5.530946 | 40.500 |

## Threshold Interpretation

- The +/-0.50 and +/-0.55 commands create a repeatable short impulse but do not sustain yaw; their last-50-ms steady yaw is near zero.
- +/-0.60 is transitional: it moves more than 0.55 but usually does not meet the 1 rad/s sustained-launch criterion.
- +/-0.65 is the practical launch boundary in these clean steps: CW sustains in all clean repeats, CCW sustains in most but not all repeats.
- This looks less like a single hard command threshold and more like a static-to-dynamic contact/bristle resistance transition: below the boundary the robot can twitch, then contact resistance arrests the yaw while command remains applied.

## Secondary Model Check

- Prediction check uses the exact command windows with +2 response alignment, raw gyro minus the independent static bias, encoder wheel speeds, and drive commands.
- Old means the mirrored PlantModel path with contact-yaw correction disabled. Tuned means `kContactYawPatchForceGainNsPerM = -6.496619190`.

| Amp | Dir | Samples | Old RMSE | Tuned RMSE | Delta |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0.50 | CCW | 3470 | 0.093122111 | 0.094609427 | 1.597166% |
| 0.50 | CW | 3470 | 0.086930046 | 0.088694172 | 2.029363% |
| 0.55 | CCW | 3470 | 0.124926873 | 0.126582105 | 1.324960% |
| 0.55 | CW | 3470 | 0.130369905 | 0.132094645 | 1.322959% |
| 0.60 | CCW | 3470 | 0.190712211 | 0.197548440 | 3.584578% |
| 0.60 | CW | 3470 | 0.213617259 | 0.219560601 | 2.782239% |
| 0.65 | CCW | 3470 | 0.171592546 | 0.185382402 | 8.036396% |
| 0.65 | CW | 3470 | 0.180146648 | 0.193214818 | 7.254185% |
| 0.70 | CCW | 3470 | 0.095407285 | 0.102370058 | 7.297947% |
| 0.70 | CW | 3470 | 0.076391508 | 0.081485174 | 6.667844% |

## Outputs

- `yaw_launch_step_metrics.csv`: one row per command step.
- `yaw_launch_aggregate.csv`: clean aggregate by amplitude and direction.
- `yaw_launch_alignment_profile.csv`: onset samples 0..15 by amplitude.
- `yaw_launch_model_prediction_plus2.csv`: old vs tuned +2 prediction RMSE by amplitude and direction.
- `yaw_launch_model_prediction_samples_plus2.csv`: small sampled prediction audit.

## Reproduce

```powershell
python codex_analysis\yaw_launch_step_response\extract_yaw_launch_step_response.py
```

## Recommended Next Use

- Use these windows as a launch-specific calibration set, not as a general yaw-surface replacement. Fit static launch, bristle/history, and low-speed contact resistance behavior against the step windows before changing broad contact-continuum gains.
- Keep the +2 command/model alignment for production replay comparisons, but treat first-response timing as a separate sensor/plant delay to model or gate because the gyro response visibly rises after +4 samples.
