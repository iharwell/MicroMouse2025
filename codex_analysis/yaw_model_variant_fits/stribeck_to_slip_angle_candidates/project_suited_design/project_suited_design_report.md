# Project-Suited Stribeck-To-Slip-Angle Design

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Design Rationale

The chosen form is a PlantModel-shaped contact law, not a scalar residual table. It keeps Vehicle-owned facts as inputs, computes per-contact forces from contact relative velocity and geometry, then accumulates yaw with `sum_i(f_i * F_r_i - r_i * F_f_i)`. The May 4 launch logs are deliberately visible as a launch constraint and latest-log objective, but they are not promoted to full authority because the provenance report marks them incomplete.

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

`mu_static_extra` is solved analytically/numerically from the measured `+/-0.646` in-place launch command at `Vf=0`, `yawRate=1 rad/s` for each optimizer point.

## Optimization

SciPy is not installed in this workspace, so the fit used a continuous custom optimizer: differential evolution followed by coordinate polish. The objective is not an all-rows blind fit: 72% primary April authoritative, 18% downweighted open-floor, and 10% May 4 latest-log guard, all run-balanced with quality penalties.

| optimizer | scipy_available | de_population | de_generations | de_sample_rows | polish_rows | polish_iterations | de_best_objective | final_objective | objective_primary_weight | objective_downweighted_weight | objective_latest_may4_weight |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| custom differential_evolution_plus_coordinate_polish | 0.000000 | 54.000000 | 58.000000 | 22000.000000 | 80243.000000 | 56.000000 | 0.025978 | 0.027549 | 0.720000 | 0.180000 | 0.100000 |

## Selected Parameters

| parameter | value |
| --- | --- |
| drive_scale | 0.180000 |
| longitudinal_mu | 0.343527 |
| longitudinal_k_mps | 0.411178 |
| mu_peak | 5.000000 |
| mu_slide | 0.261964 |
| stribeck_speed_mps | 0.020780 |
| low_speed_gate_mps | 0.080000 |
| lateral_sign_eps_mps | 0.017227 |
| alpha_floor_mps | 0.800000 |
| alpha_knee | 0.072449 |
| corner_mu_front | 0.102485 |
| corner_mu_rear | 0.050000 |
| derived_static_extra_mu | 7.496900 |
| launch_total_opposing_nm | 0.080362 |
| launch_static_solve_error_nm | -0.000000 |
| objective_score | 0.027549 |

## Bound And Identifiability Notes

Several fitted parameters can hit bounds, and those hits are written to `parameter_bound_hits.csv`. When that happens, read this result as a design-comparison fit rather than a clean coefficient identification: the launch/static Stribeck requirement is hard, the per-contact/normal-load/envelope shape is production-aligned, and the broad data may still prefer the brush or standalone candidates for raw fit quality.

## Launch Estimate

| variant | yaw_rate_radps | target_total_opposing_yaw_torque_nm | achieved_total_opposing_yaw_torque_nm | derived_static_extra_mu | launch_solve_error_nm | left_command | right_command | lr_delta_command | max_abs_command | passes_abs_0p6_gate | target_command_abs | target_abs_error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| project_suited_stribeck_slip_angle | 1.000000 | 0.080362 | 0.080362 | 7.496900 | -0.000000 | 0.646000 | -0.646000 | 1.292000 | 0.646000 | 1.000000 | 0.646000 | 0.000000 |

## Fit Results

| group | count | baseline_rmse_nm | direct_model_rmse_nm | rmse_improvement_fraction | run_balanced_direct_rmse_nm |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.019682 | 0.466127 | 0.019412 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.028654 | 0.336613 | 0.026417 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.019564 | -0.155556 | 0.018333 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.032590 | 0.614866 | 0.032539 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.024581 | 0.520515 | 0.023664 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.026930 | 0.463899 | 0.024109 |

## Latest Logs

| run_id | dataset_split | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.029351 | 0.001493 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.032386 | 0.000144 |
| may4_latest_combined | mixed_downweighted_and_validation | 5217.000000 | 0.032158 | 0.030409 | 0.000989 |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.029351 | 0.001493 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.032386 | 0.000144 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.014212 | -0.000671 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.012414 | -0.001789 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.017989 | -0.001090 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.022528 | -0.003202 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.023467 | -0.000770 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.034702 | -0.000006 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.032717 | 0.006538 |

## Risk Slices

| group | count | baseline_rmse_nm | direct_model_rmse_nm | direct_model_median_abs_nm |
| --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.033661 | 0.023892 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.040207 | 0.033735 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027320 | 0.012259 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066361 | 0.019389 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.047922 | 0.019519 |
| fast_forward | 0.000000 |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011026 | 0.004729 |
| limiter_active | 31216.000000 | 0.074601 | 0.036455 | 0.026013 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.043228 | 0.030878 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.030409 | 0.012572 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.033661 | 0.027070 | 0.023892 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.040207 | 0.034822 | 0.033735 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027320 | 0.018397 | 0.012259 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066361 | 0.031671 | 0.019389 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.047922 | 0.034743 | 0.019519 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011026 | 0.007259 | 0.004729 |
| limiter_active | 31216.000000 | 0.074601 | 0.036455 | 0.029117 | 0.026013 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.043228 | 0.034965 | 0.030878 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.030409 | 0.021493 | 0.012572 |

## Candidate Comparison

| group | project_suited_direct_rmse_nm | brush_combined_slip_rmse_nm | bristle_slip_rmse_nm | scalar_partition_rmse_nm | standalone_contact_traction_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.019682 | 0.016672 | 0.016764 | 0.036488 | 0.016888 | 0.027908 | 0.021083 |
| open_floor_fit_downweighted | 0.028654 | 0.026738 | 0.026483 | 0.045347 | 0.026632 | 0.041507 | 0.032823 |
| open_floor_validation_only | 0.019564 | 0.006964 | 0.008253 | 0.018000 | 0.007711 | 0.011394 | 0.011207 |
| diag_validation_only | 0.032590 | 0.028612 | 0.026665 | 0.096346 | 0.023947 | 0.084410 | 0.038489 |
| aux_downweighted_validation | 0.024581 | 0.022071 | 0.022043 | 0.055768 | 0.020687 | 0.048820 | 0.027490 |
| validation_non_authoritative | 0.026930 | 0.023429 | 0.023021 | 0.055162 | 0.022327 |  | 0.029680 |

## Production-Shape Implications

If this shape were ever promoted, it belongs inside `PlantModel` as the single plant-equation owner and should use Vehicle-owned geometry/load facts directly. It should not become a new production type, residual overlay, command/request selector, lookup table, or UKF-dependent target path. The force-envelope projection should remain a PlantModel concept: raw desired patch forces first, then a continuous envelope/yield projection.

## Assessment

The design satisfies the launch target by construction: max command 0.646000. The fitted direct model reaches primary RMSE 0.019682 Nm and non-authoritative validation RMSE 0.026930 Nm. The comparison table should be read separately from the design rationale: a model can be project-suited yet still lose to a broader empirical brush or standalone candidate on current noisy data.

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
