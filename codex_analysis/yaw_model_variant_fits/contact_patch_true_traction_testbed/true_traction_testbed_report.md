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
| force_sliding_nm | 0.064000 |

## Candidate Summary

| name | speed_knee_mps | util_knee | rel_weight | force_sliding_nm | primary_rmse_nm | primary_regime_rmse_nm | validation_rmse_nm | validation_rb_rmse_nm | validation_regime_rmse_nm | forward_ge_0p5_rmse_nm | in_place_max_abs_command | score |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| per_patch_full_yaw | 0.500000 | 0.160000 | 1.000000 | 0.064000 | 0.019520 | 0.025469 | 0.028416 | 0.025617 | 0.033947 | 0.031394 | 0.618834 | 0.033573 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 1.000000 | 0.067417 | 0.019484 | 0.025485 | 0.028475 | 0.025578 | 0.033938 | 0.031385 | 0.637844 | 0.033586 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 1.000000 | 0.064000 | 0.019561 | 0.025510 | 0.028467 | 0.025658 | 0.033981 | 0.031391 | 0.618834 | 0.033617 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 0.750000 | 0.064000 | 0.019584 | 0.025516 | 0.028438 | 0.025652 | 0.033947 | 0.031399 | 0.619038 | 0.033621 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 0.750000 | 0.067417 | 0.019552 | 0.025542 | 0.028503 | 0.025600 | 0.033936 | 0.031387 | 0.638059 | 0.033644 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 1.000000 | 0.067417 | 0.019545 | 0.025540 | 0.028537 | 0.025629 | 0.033979 | 0.031383 | 0.637844 | 0.033646 |
| per_patch_full_yaw | 0.500000 | 0.160000 | 1.000000 | 0.072000 | 0.019508 | 0.025558 | 0.028585 | 0.025554 | 0.033945 | 0.031376 | 0.663344 | 0.033659 |
| per_patch_longitudinal | 0.500000 | 0.160000 | 0.750000 | 0.064000 | 0.019630 | 0.025565 | 0.028491 | 0.025694 | 0.033982 | 0.031396 | 0.619038 | 0.033672 |

## Split Metrics

| group | count | baseline_rmse_nm | corrected_rmse_nm | corrected_mae_nm | corrected_median_abs_nm | run_balanced_corrected_rmse_nm | mean_blend |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.019520 | 0.013602 | 0.009710 | 0.019354 | 0.247042 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.034837 | 0.021826 | 0.010548 | 0.032073 | 0.235460 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.008849 | 0.005975 | 0.004545 | 0.008983 | 0.208748 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.025703 | 0.019132 | 0.013910 | 0.025505 | 0.038689 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.027896 | 0.014555 | 0.006173 | 0.028252 | 0.136865 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.028416 | 0.016697 | 0.008029 | 0.025617 | 0.179348 |

## Comparison To Existing Evidence

| group | true_patch_corrected_rmse_nm | rational_residual_corrected_rmse_nm | cubic_force_only_partition_rmse_nm | force_domain_stribeck_rmse_nm |
| --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.019520 | 0.021083 | 0.018341 | 0.027908 |
| open_floor_fit_downweighted | 0.034837 | 0.032823 | 0.033093 | 0.041507 |
| open_floor_validation_only | 0.008849 | 0.011207 | 0.010491 | 0.011394 |
| diag_validation_only | 0.025703 | 0.038489 | 0.029430 | 0.084410 |
| aux_downweighted_validation | 0.027896 | 0.027490 | 0.029397 | 0.048820 |
| validation_non_authoritative | 0.028416 | 0.029680 | 0.028489 |  |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | corrected_mae_nm | mean_blend |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.015602 | 0.010886 | 0.398603 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.010144 | 0.008031 | 0.235272 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.012097 | 0.010196 | 0.145280 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.013782 | 0.010753 | 0.395932 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.016373 | 0.013620 | 0.354691 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.026888 | 0.019581 | 0.153708 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.028182 | 0.020356 | 0.149075 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.043145 | 0.033047 | 0.256207 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.026254 | 0.020008 | 0.037406 |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | run_balanced_corrected_rmse_nm | mean_blend |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.032902 | 0.031963 | 0.238516 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.036486 | 0.037634 | 0.217140 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.031755 | 0.026201 | 0.147017 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.075938 | 0.050084 | 0.042933 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.082925 | 0.082925 | 0.085995 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.012857 | 0.013561 | 0.194969 |
| limiter_active | 31216.000000 | 0.074601 | 0.043642 | 0.044710 | 0.217609 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.055547 | 0.050643 | 0.282340 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.014000 | 0.013159 | 0.343471 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.032902 | 0.022040 | 0.013339 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.036486 | 0.025859 | 0.018441 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.031755 | 0.020729 | 0.012944 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.075938 | 0.044273 | 0.029726 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.082925 | 0.062449 | 0.048950 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.012857 | 0.008695 | 0.006414 |
| limiter_active | 31216.000000 | 0.074601 | 0.043642 | 0.032873 | 0.025971 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.055547 | 0.044737 | 0.038651 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.014000 | 0.009922 | 0.007288 |

## In-Place Command

| extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | max_abs_command | passes_abs_0p6_gate | blend |
| --- | --- | --- | --- | --- | --- | --- |
| 0.060933 | 0.075727 | 0.618834 | -0.618834 | 0.618834 | True | 0.974191 |

The hard launch check passes if the selected max absolute command is at least `0.6`; the target reference was about `+0.646/-0.646`.

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
