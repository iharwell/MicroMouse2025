# Project-Suited Stribeck-To-Slip-Angle Design

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Design Rationale

The chosen form is a PlantModel-shaped contact law, not a scalar residual table. It keeps Vehicle-owned facts as inputs, computes per-contact forces from contact relative velocity and geometry, then accumulates yaw with `sum_i(f_i * F_r_i - r_i * F_f_i)`. The launch estimate is diagnostic only; it is not a hard constraint, target solve, or candidate-selection gate.

The form was chosen before fitting:

- `Vf = 0` uses a conventional rational Stribeck/static breakaway branch with smooth sign `v_r / sqrt(v_r^2 + eps^2)`.
- Moving-speed lateral force uses a geometry-derived slip-angle proxy `alpha_i = v_rel_r_i / sqrt((Vf - yaw*r_i)^2 + alpha_floor^2)`.
- Longitudinal drive is distributed by normal load within each left/right bank, not split equally front/rear.
- Raw longitudinal and lateral patch forces are projected through a cheap smooth force envelope.
- Runtime operations are `sqrt`, `abs`, clamps, multiplies, and divides only. There is no trig, `exp`, `tanh`, UKF target, command selector, or hidden state machine.

## Equations

For contact `i` at right offset `r_i` and forward offset `f_i`:

`F_drive_i = drive_scale * F_drive_side * N_i / sum_side(N)`

`G_low = v_gate^2 / (v_gate^2 + Vf^2)`

`mu_stribeck_i = mu_slide + mu_static_extra * v_s^2 / (v_s^2 + v_rel_f_i^2 + v_rel_r_i^2)`

`F_r_low_i = N_i * mu_stribeck_i * v_rel_r_i / sqrt(v_rel_r_i^2 + eps^2)`

`alpha_i = v_rel_r_i / sqrt((Vf - yaw_rate*r_i)^2 + alpha_floor^2)`

`F_r_high_i = N_i * mu_corner_axle * alpha_i / sqrt(alpha_i^2 + alpha_knee^2)`

`F_f_raw_i = F_drive_i + N_i * mu_long * v_rel_f_i / sqrt(v_rel_f_i^2 + k_long^2)`

`F_r_raw_i = G_low*F_r_low_i + (1-G_low)*F_r_high_i`

`scale_i = 1 / sqrt(1 + (F_f_raw_i^2 + F_r_raw_i^2)/(mu_peak*N_i)^2)`

`M_yaw = sum_i(f_i*scale_i*F_r_raw_i - r_i*scale_i*F_f_raw_i)`

`mu_static_extra` is a fitted free parameter. The `Vf=0`, `yawRate=1 rad/s` launch command is computed after optimization as a diagnostic.

## Optimization

SciPy is not installed in this workspace, so the fit used a continuous custom optimizer: differential evolution followed by coordinate polish. The objective is not an all-rows blind fit: 72% primary April authoritative, 18% downweighted open-floor, and 10% May 4 latest-log guard, all run-balanced with quality penalties.

| optimizer | scipy_available | de_population | de_generations | de_sample_rows | polish_rows | polish_iterations | de_best_objective | final_objective | objective_primary_weight | objective_downweighted_weight | objective_latest_may4_weight |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| custom differential_evolution_plus_coordinate_polish | 0.000000 | 54.000000 | 58.000000 | 22000.000000 | 80243.000000 | 48.000000 | 0.020841 | 0.022517 | 0.720000 | 0.180000 | 0.100000 |

## Selected Parameters

| parameter | value |
| --- | --- |
| drive_scale | 0.160401 |
| longitudinal_mu | 0.375799 |
| longitudinal_k_mps | 0.345181 |
| mu_peak | 5.000000 |
| mu_slide | 0.500000 |
| static_extra_mu | 1.400593 |
| stribeck_speed_mps | 0.020000 |
| low_speed_gate_mps | 0.379680 |
| lateral_sign_eps_mps | 0.047838 |
| alpha_floor_mps | 0.800000 |
| alpha_knee | 0.049883 |
| corner_mu_front | 0.050000 |
| corner_mu_rear | 0.050000 |
| objective_score | 0.022517 |

## Bound And Identifiability Notes

Several fitted parameters can hit bounds, and those hits are written to `parameter_bound_hits.csv`. When that happens, read this result as a design-comparison fit rather than a clean coefficient identification: the static Stribeck parameter is free, the per-contact/normal-load/envelope shape is production-aligned, and the broad data may still prefer the brush or standalone candidates for raw fit quality.

## Launch Estimate

| variant | yaw_rate_radps | diagnostic_total_opposing_yaw_torque_nm | static_extra_mu | left_command | right_command | lr_delta_command | max_abs_command | passes_abs_0p6_gate | launch_lock_policy |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| project_suited_stribeck_slip_angle | 1.000000 | 0.011781 | 1.400593 | 0.244082 | -0.244082 | 0.488164 | 0.244082 | 0.000000 | diagnostic_only |

## Fit Results

| group | count | baseline_rmse_nm | direct_model_rmse_nm | rmse_improvement_fraction | run_balanced_direct_rmse_nm |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.017091 | 0.536405 | 0.016720 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.025570 | 0.408012 | 0.023361 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.007202 | 0.574597 | 0.007449 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.031364 | 0.629358 | 0.031386 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.023070 | 0.549999 | 0.022458 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.023617 | 0.529858 | 0.020950 |

## Latest Logs

| run_id | dataset_split | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.011287 | -0.001235 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.009798 | 0.000599 |
| may4_latest_combined | mixed_downweighted_and_validation | 5217.000000 | 0.032158 | 0.010807 | -0.000735 |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.011287 | -0.001235 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.009798 | 0.000599 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.012133 | 0.001180 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.013318 | -0.001360 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.017214 | -0.000881 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.020915 | -0.003959 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.022001 | -0.001874 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.032541 | 0.000057 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.031286 | 0.003512 |

## Risk Slices

| group | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_median_abs_nm |
| --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.027363 | 0.013068 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.031692 | 0.019128 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027606 | 0.012352 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066684 | 0.019920 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.047755 | 0.023146 |
| fast_forward | 0.000000 |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011167 | 0.004422 |
| limiter_active | 31216.000000 | 0.074601 | 0.036059 | 0.025083 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.041083 | 0.029358 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.010807 | 0.006821 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.027363 | 0.019520 | 0.013068 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.031692 | 0.024409 | 0.019128 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027606 | 0.018672 | 0.012352 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066684 | 0.031936 | 0.019920 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.047755 | 0.036881 | 0.023146 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011167 | 0.007246 | 0.004422 |
| limiter_active | 31216.000000 | 0.074601 | 0.036059 | 0.028628 | 0.025083 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.041083 | 0.033253 | 0.029358 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.010807 | 0.008247 | 0.006821 |

## Candidate Comparison

| group | project_suited_direct_rmse_nm | brush_combined_slip_rmse_nm | bristle_slip_rmse_nm | scalar_partition_rmse_nm | standalone_contact_traction_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.017091 | 0.016672 | 0.016764 | 0.036488 | 0.016888 | 0.027908 | 0.021083 |
| open_floor_fit_downweighted | 0.025570 | 0.026738 | 0.026483 | 0.045347 | 0.026632 | 0.041507 | 0.032823 |
| open_floor_validation_only | 0.007202 | 0.006964 | 0.008253 | 0.018000 | 0.007711 | 0.011394 | 0.011207 |
| diag_validation_only | 0.031364 | 0.028612 | 0.026665 | 0.096346 | 0.023947 | 0.084410 | 0.038489 |
| aux_downweighted_validation | 0.023070 | 0.022071 | 0.022043 | 0.055768 | 0.020687 | 0.048820 | 0.027490 |
| validation_non_authoritative | 0.023617 | 0.023429 | 0.023021 | 0.055162 | 0.022327 |  | 0.029680 |

## Production-Shape Implications

If this shape were ever promoted, it belongs inside `PlantModel` as the single plant-equation owner and should use Vehicle-owned geometry/load facts directly. It should not become a new production type, residual overlay, command/request selector, lookup table, or UKF-dependent target path. The force-envelope projection should remain a PlantModel concept: raw desired patch forces first, then a continuous envelope/yield projection.

## Assessment

The design satisfies the launch target by construction: max command 0.244082. The fitted direct model reaches primary RMSE 0.017091 Nm and non-authoritative validation RMSE 0.023617 Nm. The comparison table should be read separately from the design rationale: a model can be project-suited yet still lose to a broader empirical brush or standalone candidate on current noisy data.

## Outputs

- `fit_project_suited_design.py`
- `project_suited_design_report.md`
- `selected_parameters.csv`
- `optimizer_summary.csv`
- `optimizer_trace.csv`
- `split_metrics.csv`
- `latest_weighted_metrics.csv`
- `selected_log_metrics.csv`
- `risk_slices.csv`
- `common_range_metrics.csv`
- `launch_estimate.csv`
- `reference_comparison.csv`
- `parameter_bound_hits.csv`
- `prediction_sample.csv`
- `commands_run.txt`
