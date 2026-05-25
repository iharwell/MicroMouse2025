# Yaw Plant Parameter Fit

## Analysis Path

This scratch analysis uses decoded `open_floor_main.csv` logs directly. It uses raw gyro minus the run's stationary raw-gyro mean, encoder velocities, encoder wheel speeds, and logged drive commands. It does not use UKF state estimates as fit targets.

The fitted one-step yaw model mirrors the yaw-relevant PlantModel terms needed for in-place yaw sections: motor command to wheel-bank torque, static/rolling drive losses, differential drive yaw moment, effective yaw inertia including wheel spin-up, and yaw-rate damping.

Reproduce with:

```powershell
python codex_analysis\yaw_fit\fit_yaw_plant_params.py
```

## Logs And Windows

| Run ID | CSV | Input rows | Kept rows | Fit samples | Cutoff | First dropped tick/time |
| --- | --- | ---: | ---: | ---: | --- | --- |
| `mmlog_decode_2026-04-20_08-38-39` | `TestResults\mmlog_decode_2026-04-20_08-38-39\open_floor_main.csv` | 217867 | 217867 | 63435 | no terminal fault found; kept all rows |  |
| `mmlog_decode_2026-04-20_10-22-09` | `TestResults\mmlog_decode_2026-04-20_10-22-09\open_floor_main.csv` | 99917 | 95512 | 15815 | dropped trailing terminal segment before fault timestamp | 97221 / 108644536 |
| `mmlog_decode_2026-04-20_12-10-58` | `TestResults\mmlog_decode_2026-04-20_12-10-58\open_floor_main.csv` | 86556 | 86402 | 10092 | dropped trailing terminal segment before fault timestamp | 92045 / 115953375 |
| `mmlog_decode_2026-04-21_01-09-34` | `TestResults\mmlog_decode_2026-04-21_01-09-34\open_floor_main.csv` | 101013 | 100776 | 14675 | dropped trailing terminal segment before fault timestamp | 141583 / 155015425 |

Fit rows are limited to `SEC_40_YAW` phase `8` active rotation and phase `9` stop/decay rows, with zero saturation flags, zero watchdog flags when present, same section/primitive/speed/repeat on adjacent samples, and `0.5..3.0 ms` sample intervals.

## Current Vs Proposed

| Parameter | Current | Proposed / fitted |
| --- | ---: | ---: |
| `Vehicle::track_width_m` | 0.084635000 | 0.104595474 |
| `Vehicle::yaw_inertia_kg_m2` | 0.000220000 | 0.000603133 |
| `PlantModel::yaw_rate_damping_nms_per_rad` | 0.000000000 | 0.000000000 |
| fitted yaw denominator including wheel spin-up | 0.000246510 | 0.000643622 |

## RMSE

| Metric | Samples | Current | Proposed |
| --- | ---: | ---: | ---: |
| one-step yaw-rate RMSE (rad/s) | 104017 | 0.226296125 | 0.161693351 |
| implied yaw-accel RMSE (rad/s^2) | 104017 | 226.296 | 161.693 |

## Per-Run One-Step Yaw RMSE

| Run ID | Samples | Current RMSE (rad/s) | Proposed RMSE (rad/s) |
| --- | ---: | ---: | ---: |
| `mmlog_decode_2026-04-20_08-38-39` | 63435 | 0.242043414 | 0.168874485 |
| `mmlog_decode_2026-04-20_10-22-09` | 15815 | 0.223603255 | 0.180994080 |
| `mmlog_decode_2026-04-20_12-10-58` | 10092 | 0.210934786 | 0.154676882 |
| `mmlog_decode_2026-04-21_01-09-34` | 14675 | 0.158916770 | 0.101400332 |

## Fit Counts

- Track-width fit samples: 92562
- Yaw-acceleration fit samples after robust trimming: 75826
