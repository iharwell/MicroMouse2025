# Contact-Continuum Yaw Feature Extraction

Scratch analysis only. No production code was modified.

## Reproduce

```powershell
python codex_analysis\contact_continuum_yaw_identification\features\extract_contact_continuum_features.py --out-dir codex_analysis\contact_continuum_yaw_identification\features --sample-every 25 --min-bin-count 80
```

## Source Basis

The feature pass follows `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`: contact-relative velocity is the primary contact primitive. It does not build a Vf/yaw residual table as the primary semantic object, and it does not compute slip angle, slip ratio, curvature, radius, or maneuver-mode branches.

Targets use sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where stationary rows exist, encoder-derived forward velocity and wheel-bank speeds, logged drive commands, and timestamps. Logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.

The PlantModel mirror includes command-to-bank torque, no-load current, static launch loss, rolling loss, normal-load distribution, longitudinal/right contact force requests, force projection, yaw damping, and the wheel spin-up term in the yaw denominator. The residual convention is `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - model_yaw_moment_nm`; positive yaw is clockwise.

## Feature Definitions

- Contact coordinates are `r>0` right and `f>0` forward: FL=(-half_track,+offset), FR=(+half_track,+offset), RL=(-half_track,-offset), RR=(+half_track,-offset).
- `v_rel_f_i = wheel_surface_velocity_bank - (Vf - omega*r_i)`. The script uses encoder wheel surface velocities and sensor-derived `Vf`.
- `v_rel_r_i = -(Vr + omega*f_i)`. No independent lateral velocity exists in these logs, so `Vr=0` is an explicit `zero_lateral_sensor_unavailable` source assumption.
- `vbar_rel`, `vbar_lat`, and `vbar_yaw` are load-weighted RMS quantities from the root spec. Normal loads are static plus fan load only; load transfer is not reconstructed.
- Force-request columns are the current PlantModel mirror's drive request plus longitudinal/right contact terms before projection. Force columns are after projection.
- `max_force_preprojection_utilization` is the largest raw contact force magnitude divided by its sustained-lateral-acceleration envelope. `max_force_limiter_activity` is zero unless projection scales a contact force.
- `current_proxy_abs_raw_over_unit_command_prior` is a drive-authority proxy against the unit-command current prior, not a measured DRV8871 trip current.

## Output Scope

Discovered 67 candidate logs; 67 produced at least one sample. Input rows scanned: 4456959. Extracted qualifying adjacent samples: 2995068. Final quiescent/bad-tail rows dropped: 7706.

`contact_continuum_feature_sample.csv` keeps a deterministic 1-in-25 sample of qualifying rows, currently 120158 rows. Aggregated tables use all qualifying rows.

| Family | Samples | Runs | Limiter-active fraction | Hardware saturation fraction | Mean vbar_rel m/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| competition_aux | 358483 | 13 | 0.223506 | 0.000000 | 0.070504 |
| competition_diag | 273475 | 4 | 0.441437 | 0.000000 | 0.146086 |
| open_floor | 2362926 | 49 | 0.241950 | 0.053314 | 0.068785 |
| uncertainty_open_floor | 184 | 1 | 0.054348 | 0.043478 | 0.004484 |

## Outputs

- `contact_continuum_feature_sample.csv`: compact per-sample feature rows.
- `contact_continuum_contact_bins.csv`: aggregate residual/contact bins by `vbar_rel`, `vbar_lat`, `vbar_yaw`, and signed yaw-rate bin.
- `contact_continuum_phase_summary.csv`: aggregate rows by family and phase/section labels.
- `contact_continuum_force_bins.csv`: aggregate rows by force-utilization and `vbar_rel` bins.
- `contact_continuum_run_summary.csv`: per-run inclusion, bias, tail, and limitation inventory.
- `plant_mirror_constants.csv`: constants parsed from authoritative `Vehicle`, `PlantModel`, and `MotorEncoderDrive` code.

## Limitations

- Lateral body velocity is not independently measured in these logs; all right-relative contact features assume `Vr=0` and must be treated as reconstruction features, not measured lateral truth.
- Normal-load transfer is not reconstructed; fan load uses row `fan_duty_cycle` when present, competition metadata when present, otherwise 0.8.
- Legacy competition logs lack saturation/watchdog fields and derive wheel omega from encoder velocity and current wheel radius.
- Residual yaw torque differentiates raw gyro, so timing jitter and gyro noise remain visible in single-sample targets; downstream ablation should prefer aggregate or filtered comparisons.
- Saturated/limited rows are retained with evidence columns instead of removed, so consumers must decide whether to train on them.

Aggregate rows written: contact bins 760, phase rows 465, force bins 216.
