# Patch Rational Slip-Angle Stribeck Candidate

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Model

For contact `i` at position `(r_i, f_i)`, the prediction is a physical yaw-opposing torque from summed contact forces:

`M_opp = -sign(yawRate) * sum_i(f_i * F_r_i - r_i * F_f_i)`

Low-speed branch:

`G0 = V0^2 / (V0^2 + Vf^2)`

`A_i = v_i^2 / (v_i^2 + v_y^2)`

`S_i = r_slide + (1-r_slide) * v_s^2 / (v_s^2 + v_i^2)`

`M_low_i = M_static_peak * G0 * A_i * S_i * (N_i * |r_i|) / sum_j(N_j * |r_j|)`

`F_f_i_low = sign(yawRate) * sign(r_i) * M_low_i / |r_i|`, `F_r_i_low = 0`

High-speed branch:

`beta_i = v_rel_r_i / sqrt(v_rel_f_i^2 + v_floor^2)`

`B_i = beta_i / (1 + a*|beta_i| + b*beta_i^2)`

`F_r_i_high = -(1-G0) * N_i * C_axle * B_i`, `F_f_i_high = 0`

The selected law uses only `sqrt`, `abs`, rational divisions, and sign/clamp-style operations. It uses no trig, `atan`, `exp`, `tanh`, command/request selectors, UKF state-vector fields, or residual lookup table.

## Optimization

SciPy was not available in the bundled runtime, so this run used a custom continuous optimizer: differential evolution over nonlinear parameters, then bounded Nelder-Mead refinement. For each nonlinear point, the two nonnegative axle slip-angle gains were solved by weighted two-column NNLS.

| scipy_available | optimizer | de_generations | de_population | de_best_score | nelder_mead_iterations | nelder_mead_stop | final_objective | trace_rows | de_sample_rows | nelder_mead_weighted_rows |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.000000 | custom differential_evolution plus bounded nelder_mead | 64.000000 | 56.000000 | 0.029000 | 120.000000 | max_iter | 0.031629 | 184.000000 | 18000.000000 | 78482.000000 |

## Selected Parameters

| parameter | value |
| --- | --- |
| speed_gate_v0_mps | 0.020041 |
| yaw_activation_mps | 0.040608 |
| stribeck_speed_mps | 0.042820 |
| sliding_ratio | 0.000737 |
| beta_floor_mps | 0.045499 |
| beta_abs_denominator | 2.671881 |
| beta_quad_denominator | 2.084598 |
| high_front_mu_per_beta | 0.000000 |
| high_rear_mu_per_beta | 0.000000 |
| launch_extra_opposing_nm | 0.065568 |
| static_peak_nm | 0.249993 |
| train_weighted_rmse_nm | 0.031629 |
| objective_score | 0.031629 |

## Boundary Behavior

| quantity | value | boundary |
| --- | --- | --- |
| speed_gate_v0_mps | 0.020041 | lower bound was 0.020 m/s |
| high_front_mu_per_beta | 0.000000 | zero gain means no selected front slip-angle branch |
| high_rear_mu_per_beta | 0.000000 | zero gain means no selected rear slip-angle branch |
| static_peak_nm | 0.249993 | regularization knee was 0.25 Nm |

## 1 rad/s In-Place Launch

| variant | base_opposing_yaw_torque_nm | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | lr_delta_command | max_abs_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| patch_rational_slip | 0.014794 | 0.065568 | 0.080362 | 0.646000 | -0.646000 | 1.292000 | 0.646000 | 1.000000 |

## Split Metrics

| group | count | baseline_rmse_nm | corrected_rmse_nm | run_balanced_corrected_rmse_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.036676 | 0.036215 | 0.015890 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.045283 | 0.042938 | 0.013289 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.017373 | 0.016308 | 0.005255 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.096300 | 0.095953 | 0.096186 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.055699 | 0.061212 | 0.018721 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.055071 | 0.052948 | 0.014817 |

## Reference Comparison

| group | patch_rational_slip_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | standalone_contact_traction_rmse_nm | true_patch_testbed_rmse_nm |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036676 | 0.027908 | 0.021083 | 0.016688 | 0.019508 |
| open_floor_fit_downweighted | 0.045283 | 0.041507 | 0.032823 | 0.027952 | 0.035267 |
| open_floor_validation_only | 0.017373 | 0.011394 | 0.011207 | 0.007665 | 0.009294 |
| diag_validation_only | 0.096300 | 0.084410 | 0.038489 | 0.020030 | 0.025685 |
| aux_downweighted_validation | 0.055699 | 0.048820 | 0.027490 | 0.019198 | 0.027446 |
| validation_non_authoritative | 0.055071 |  | 0.029680 | 0.022157 | 0.028585 |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | corrected_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.036160 | 0.008268 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.022864 | 0.000103 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.016035 | -0.002875 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.044906 | 0.009350 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.042078 | 0.011845 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.040367 | 0.008966 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.041048 | 0.003558 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.060091 | 0.000130 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.097241 | -0.014599 |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367.000000 | 0.025842 | 0.026287 | 0.006146 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746.000000 | 0.024100 | 0.024134 | 0.005349 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704.000000 | 0.073503 | 0.083243 | 0.058936 |
| high_forward_vf_ge_0p5 | 3120.000000 | 0.046415 | 0.046421 | 0.012761 |
| high_speed_abs_vf_ge_0p7 | 144.000000 | 0.088793 | 0.088798 | 0.027951 |
| limiter_active | 31014.000000 | 0.074600 | 0.077744 | 0.053158 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.064819 | 0.043117 |

## May 4 Latest Logs

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | corrected_signed_median_nm |
| --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.036160 | 0.008268 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.022864 | 0.000103 |

## Viability

The model passes the hard in-place gate with max command 0.646000, but the optimized slip-angle branch is degenerate: both high-speed axle gains are zero and `V0` is on the lower bound. Primary RMSE is 0.036676 Nm and non-authoritative validation RMSE is 0.055071 Nm, both worse than the main references. The standalone contact-traction testbed remains materially better on validation (0.022157 Nm). Verdict: this exact residual-correction family is not viable; the data prefers either the older force-domain Stribeck residual or the broader standalone contact-traction law over this constrained Stribeck-to-slip-angle handoff.

## Outputs

- `fit_patch_rational_slip.py`
- `patch_rational_slip_report.md`
- `optimizer_trace.csv`
- `optimization_summary.csv`
- `boundary_behavior.csv`
- `selected_parameters.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `may4_latest_log_metrics.csv`
- `risk_metrics.csv`
- `split_reference_comparison.csv`
- `launch_command_estimate.csv`
- `prediction_sample.csv`
- `commands_run.txt`
