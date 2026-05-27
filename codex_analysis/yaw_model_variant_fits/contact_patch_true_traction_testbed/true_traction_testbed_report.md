# Contact-Patch True-Traction Rational Testbed

Analysis-only output. Production code, build metadata, and tests were not modified.

## Recommendation

Viable as a true contact-patch traction formulation, with a qualification: the selected implementation must be expressed as force increments at each contact before yaw moment accumulation. The algebraic version that first computes one yaw residual scalar and then subtracts it remains a residual in disguise and should be rejected for production shape.

The best low-dimensional testbed candidate uses the compact force-only moving-contact branch as per-contact force increments, then blends those increments with a low-speed longitudinal static/yield reserve at each patch. It uses projected/actual contact forces, normal load, contact-relative velocity, and contact geometry. It does not use command/request/preprojection values as traction selectors and does not use UKF state-vector fields.

## Selected Equations

For contact `i`, lateral coordinate `r_i`, longitudinal coordinate `f_i`, normal load `N_i`, projected force `(F_f,i, F_r,i)`, and relative velocity `(v_f,i, v_r,i)`:

`M_i = f_i*F_r,i - r_i*F_f,i`

`drive_i = max_smooth( sign(yawRate) * M_i )`

`Y_i = mu_ref * N_i * sqrt(r_i^2 + f_i^2)` for the selected reserve weighting geometry.

`u_i = drive_i / Y_i`

`v2_i = |Vf|^2 + (rel_weight * sqrt(v_f,i^2 + v_r,i^2))^2`

`G_i = k_v^2/(k_v^2 + v2_i) * u_i^2/(u_i^2 + k_u^2)`

`R_i = speed_fade^2/(speed_fade^2 + v2_i)`

`Delta M_reserve_i = K_slide * (Y_i / sum_j Y_j) * R_i * u_i^2/(u_i^2 + k_u^2)`

`Delta F_reserve_f,i = sign(yawRate) * sign(r_i) * Delta M_reserve_i / |r_i|`, `Delta F_reserve_r,i = 0`

The moving-contact branch is also force-shaped. Coefficients multiplying `gain_long_total_basis`, `gain_right_total_basis`, and `force_moment_opposes_yaw_nm` are applied as per-contact `Delta F_f`, `Delta F_r`, and projected-force-proportional increments, then yaw support is recomputed from geometry.

`Delta F_i = (1 - G_i) * Delta F_C,i + G_i * Delta F_reserve_i`

`M_pred_opposes = sum_i sign(yawRate) * (r_i*Delta F_f,i - f_i*Delta F_r,i)`

That last line is an accumulation of contact-patch forces. A scalar-only implementation of `M_pred_opposes` is not the accepted production interpretation.

## Selected Parameters

| parameter | value |
| --- | --- |
| name | per_patch_full_yaw |
| gate_scope | per_patch |
| reserve_yield | full_yaw |
| speed_knee_mps | 0.500000 |
| util_knee | 0.160000 |
| rel_weight | 1.000000 |
| speed_fade_mps | 0.640000 |
| force_sliding_nm | 0.075000 |

## Candidate Summary

| name | speed_knee_mps | util_knee | rel_weight | force_sliding_nm | primary_rmse_nm | primary_regime_rmse_nm | validation_rmse_nm | validation_rb_rmse_nm | validation_regime_rmse_nm | forward_ge_0p5_rmse_nm | in_place_max_abs_command | score |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| per_patch_full_yaw | 0.500000 | 0.160000 | 1.000000 | 0.075000 | 0.019570 | 0.025638 | 0.028676 | 0.025554 | 0.033963 | 0.031370 | 0.680035 | 0.033740 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 0.750000 | 0.075000 | 0.019656 | 0.025728 | 0.028722 | 0.025552 | 0.033961 | 0.031367 | 0.680276 | 0.033829 |
| per_patch_full_yaw | 0.650000 | 0.160000 | 1.000000 | 0.075000 | 0.019745 | 0.025711 | 0.028707 | 0.025726 | 0.034021 | 0.031497 | 0.680170 | 0.033838 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 1.000000 | 0.075000 | 0.019686 | 0.025733 | 0.028766 | 0.025630 | 0.034024 | 0.031368 | 0.680035 | 0.033840 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 1.000000 | 0.050000 | 0.020145 | 0.025750 | 0.028375 | 0.025951 | 0.034122 | 0.031439 | 0.540942 | 0.033878 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 1.000000 | 0.050000 | 0.020137 | 0.025753 | 0.028393 | 0.025966 | 0.034135 | 0.031438 | 0.540942 | 0.033882 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 0.750000 | 0.050000 | 0.020214 | 0.025786 | 0.028393 | 0.026058 | 0.034135 | 0.031457 | 0.541098 | 0.033918 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 0.750000 | 0.050000 | 0.020207 | 0.025792 | 0.028412 | 0.026073 | 0.034148 | 0.031456 | 0.541098 | 0.033925 |

## Split Metrics

| group | count | baseline_rmse_nm | corrected_rmse_nm | corrected_mae_nm | corrected_median_abs_nm | run_balanced_corrected_rmse_nm | mean_blend |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.019570 | 0.013712 | 0.009729 | 0.019538 | 0.247042 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.035460 | 0.022802 | 0.012008 | 0.032582 | 0.235460 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.009527 | 0.006670 | 0.005160 | 0.009522 | 0.208748 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.025685 | 0.019153 | 0.013935 | 0.025487 | 0.038689 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.027297 | 0.014162 | 0.006477 | 0.027573 | 0.136865 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.028676 | 0.017190 | 0.008905 | 0.025554 | 0.179348 |

## Comparison To Existing Evidence

| group | true_patch_corrected_rmse_nm | rational_residual_corrected_rmse_nm | cubic_force_only_partition_rmse_nm | force_domain_stribeck_rmse_nm |
| --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.019570 | 0.021083 | 0.018341 | 0.027908 |
| open_floor_fit_downweighted | 0.035460 | 0.032823 | 0.033093 | 0.041507 |
| open_floor_validation_only | 0.009527 | 0.011207 | 0.010491 | 0.011394 |
| diag_validation_only | 0.025685 | 0.038489 | 0.029430 | 0.084410 |
| aux_downweighted_validation | 0.027297 | 0.027490 | 0.029397 | 0.048820 |
| validation_non_authoritative | 0.028676 | 0.029680 | 0.028489 |  |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | corrected_mae_nm | mean_blend |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.016965 | 0.012438 | 0.398603 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.011275 | 0.008660 | 0.235272 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.012160 | 0.010235 | 0.145280 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.015120 | 0.011906 | 0.395932 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.016411 | 0.013408 | 0.354691 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.026948 | 0.019608 | 0.153708 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.028314 | 0.020433 | 0.149075 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.044026 | 0.034621 | 0.256207 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.026234 | 0.020032 | 0.037406 |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | run_balanced_corrected_rmse_nm | mean_blend |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.033256 | 0.032032 | 0.238516 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.036727 | 0.037665 | 0.217140 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.031778 | 0.026150 | 0.147017 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.075927 | 0.050027 | 0.042933 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.083039 | 0.083039 | 0.085995 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.013132 | 0.013612 | 0.194969 |
| limiter_active | 31216.000000 | 0.074601 | 0.043779 | 0.044154 | 0.217609 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.055700 | 0.049917 | 0.282340 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.015283 | 0.014404 | 0.343471 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.033256 | 0.022517 | 0.013998 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.036727 | 0.026185 | 0.018880 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.031778 | 0.020767 | 0.012997 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.075927 | 0.044260 | 0.030148 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.083039 | 0.062653 | 0.049334 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.013132 | 0.009226 | 0.007174 |
| limiter_active | 31216.000000 | 0.074601 | 0.043779 | 0.033037 | 0.026261 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.055700 | 0.044970 | 0.038985 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.015283 | 0.011163 | 0.008447 |

## In-Place Command

| extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | max_abs_command | passes_abs_0p6_gate | launch_lock_policy | blend |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.071376 | 0.086170 | 0.680035 | -0.680035 | 0.680035 | True | diagnostic_only | 0.974191 |

The launch check is diagnostic only. It is not used as a score term or candidate-selection gate in this unconstrained run.

## Hardware Cost

Per contact, the selected law needs one local moment, one smooth positive or branch max, one relative-speed square sum, two rational gates, one normal-load weight, and one force blend. For four contacts this is roughly four sqrt calls if exact per-patch relative speed is used, eight divisions unless reciprocals are shared, and no trig, exp, tanh, table lookup, or history state. If `sqrt(v_f^2+v_r^2)` is replaced by the already available `vbar_rel` for the gate, the selected shape drops to one shared speed rational plus per-contact utilization rationals.

## Residual-In-Disguise Check

- Accepted: the force-level blend implemented here. Both moving branch and reserve branch produce `Delta F_f,i`/`Delta F_r,i`, then yaw support is recomputed from `sum_i r_i*F_f - f_i*F_r`.
- Rejected: computing `M_C`, `M_force`, and `M_pred` as yaw scalars and subtracting `M_pred` after the normal yaw moment accumulation. That is the current residual interpretation in different algebra.
- Borderline: using a bank-aggregate gate is physically defensible only if it is computed from actual projected contact force state. It must not be sourced from command, request, preprojection utilization, or UKF state fields.

## Provenance

Feature inputs are the existing selected `yaw_model_variant_fits` shared CSVs. `forward_velocity_mps` is encoder-derived, `yaw_rate_radps` is raw gyro minus stationary bias, residual targets are gyro-differentiated yaw torque residuals against the PlantModel mirror, and contact features are reconstructed from sensor/encoder/drive telemetry. This testbed does not read logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate fields.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\contact_patch_true_traction_testbed\fit_true_traction_testbed.py
```

## Output Files

- `fit_true_traction_testbed.py`
- `true_traction_testbed_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `phase_metrics.csv`
- `risk_metrics.csv`
- `common_range_metrics.csv`
- `baseline_comparison.csv`
- `in_place_1radps_command.csv`
- `prediction_sample.csv`
- `fit_regime_weighting_summary.json`
- `fit_regime_weighting_cells.csv`
- `fit_regime_weighting_marginals.csv`
- `validation_regime_weighting_summary.json`
- `validation_regime_weighting_cells.csv`
- `validation_regime_weighting_marginals.csv`
- `commands_run.txt`
