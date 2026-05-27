# Relaxation-Free Bristle Slip Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Model

For each contact `i` at `(r_i, f_i)` with normal load `N_i`, longitudinal contact slip `vf_i`, lateral contact slip `vr_i`, and body forward speed `Vf`:

`g0 = 1 / (1 + (abs(Vf) / transition_vf)^2)`

`mu_stribeck_i = mu_slide + mu_static_extra / (1 + (sqrt(vf_i^2 + vr_i^2) / stribeck_speed)^2)`

`Fr_stribeck_i = N_i * mu_stribeck_i * sign(vr_i)`

`alpha_proxy_i = vr_i / (abs(Vf) + alpha_floor)`

`K_alpha_i = alpha_stiff_base + alpha_stiff_speed_gain * abs(Vf) / (abs(Vf) + alpha_stiff_speed_k)`

`Fr_alpha_i = N_i * (K_alpha_i * alpha_proxy_i) / (1 + abs(K_alpha_i * alpha_proxy_i) / alpha_saturation_mu)`

`Fr_i = g0 * Fr_stribeck_i + (1 - g0) * Fr_alpha_i`

`Ff_i = 0.5 * drive_scale * F_drive_side_i + N_i * longitudinal_mu * vf_i / (abs(vf_i) + longitudinal_k)`

`M_yaw = sum_i(f_i * Fr_i - r_i * Ff_i)`

At `Vf=0`, `g0=1`, so the lateral branch is the rational Stribeck/static breakaway law. As `abs(Vf)` rises, the same contact force accumulation transitions into the algebraic slip-angle proxy branch. The law has no state, no command/request selector, no old-force residual branch, and uses no trig, exp, or tanh in the runtime equations.

## Selected Parameters

| parameter | value |
| --- | ---: |
| drive_scale | 0.126938162 |
| longitudinal_mu | 0.524064793 |
| longitudinal_k_mps | 0.6 |
| stribeck_slide_mu | 0.0518527966 |
| stribeck_static_extra_mu | 0.221746759 |
| stribeck_speed_mps | 0.3 |
| transition_vf_mps | 0.0200075013 |
| alpha_stiff_base | 0.972226244 |
| alpha_stiff_speed_gain | 2.06066798e-19 |
| alpha_stiff_speed_k_mps | 1.99389391 |
| alpha_saturation_mu | 0.917386941 |
| alpha_floor_mps | 0.015 |

## Launch

| opposing bristle scrub Nm | left command | right command | max abs command | gate | launch lock policy |
| ---: | ---: | ---: | ---: | --- | --- |
| 0.007785 | 0.534470 | -0.534470 | 0.534470 | fail | diagnostic_only |

## Optimization

Bounded Nelder-Mead was run from deterministic spaced and random seeds. Launch command is diagnostic only; broad RMSE was optimized with run-balanced pseudo-Huber loss on the current 4D-regime-weighted fit rows.

| label | selected | objective | train_rmse_nm | train_robust_loss | launch_max_abs_command | iterations | converged | boundary_hits | collapse_notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| force_domain_launch_heavier | true | 0.000178 | 0.016242 | 0.000178 | 0.534470 | 520.000000 | false | 6.000000 | transition_at_lower_bound |
| standalone_static_yield_inspired | false | 0.000178 | 0.016234 | 0.000178 | 0.497873 | 520.000000 | false | 7.000000 | static_extra_collapsed;transition_at_lower_bound |
| deterministic_random_seed_3 | false | 0.000179 | 0.016269 | 0.000179 | 0.529437 | 520.000000 | false | 6.000000 | static_extra_collapsed;transition_at_lower_bound |
| deterministic_random_seed_2 | false | 0.000179 | 0.016282 | 0.000179 | 0.546823 | 520.000000 | false | 7.000000 | static_extra_collapsed;transition_at_lower_bound |
| deterministic_random_seed_0 | false | 0.000179 | 0.016281 | 0.000179 | 0.547304 | 520.000000 | false | 7.000000 | static_extra_collapsed;transition_at_lower_bound |
| slip_angle_forward_biased | false | 0.000180 | 0.016360 | 0.000180 | 0.666226 | 520.000000 | false | 4.000000 | transition_at_lower_bound |
| deterministic_random_seed_1 | false | 0.000180 | 0.016311 | 0.000180 | 0.593067 | 520.000000 | false | 8.000000 | static_extra_collapsed;transition_at_lower_bound |
| deterministic_random_seed_4 | false | 0.000181 | 0.016264 | 0.000181 | 0.613587 | 520.000000 | false | 4.000000 | static_extra_collapsed |

## Split Metrics

| group | count | baseline_rmse_nm | bristle_slip_rmse_nm | variant_c_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | standalone_contact_traction_rmse_nm | force_level_contact_testbed_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.021468 | 0.028529 | 0.048820 | 0.027490 | 0.020687 | 0.027297 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.025719 | 0.038767 | 0.084410 | 0.038489 | 0.023947 | 0.025685 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.026718 | 0.033093 | 0.041507 | 0.032823 | 0.026632 | 0.035460 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.007876 | 0.014417 | 0.011394 | 0.011207 | 0.007711 | 0.009527 |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.016719 | 0.024120 | 0.027908 | 0.021083 | 0.016888 | 0.019570 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.022834 | 0.030342 |  | 0.029680 | 0.022327 | 0.028676 |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | bristle_slip_rmse_nm | variant_c_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.011400 | 0.028076 | 0.020956 | 0.020128 | 0.011589 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.010182 | 0.019341 | 0.015366 | 0.015839 | 0.009838 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.012324 | 0.014403 | 0.013487 | 0.013203 | 0.012330 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.012501 | 0.018608 | 0.033938 | 0.016861 | 0.012307 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.015895 | 0.023754 | 0.027284 | 0.017657 | 0.016517 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.020483 | 0.027743 | 0.034302 | 0.026878 | 0.020650 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.022220 | 0.029338 | 0.035648 | 0.027977 | 0.022382 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.034451 | 0.041312 | 0.056141 | 0.042487 | 0.034276 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.025458 | 0.038970 | 0.085032 | 0.038712 | 0.023743 |

## Risk Slices

| group | count | baseline_rmse_nm | bristle_slip_rmse_nm | rmse_improvement_fraction |
| --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.026132 | 0.559035 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.028585 | 0.611108 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027088 | 0.490887 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066305 | 0.301163 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.049027 | 0.311356 |
| fast_forward | 0.000000 |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.010889 | 0.548195 |
| limiter_active | 31216.000000 | 0.074601 | 0.034455 | 0.538149 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.043259 | 0.320610 |
| open_floor_all | 93024.000000 | 0.036894 | 0.019775 | 0.464017 |
| diag_all | 11108.000000 | 0.084621 | 0.025719 | 0.696066 |
| aux_all | 14448.000000 | 0.051266 | 0.021468 | 0.581250 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.011004 | 0.657824 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.026132 | 0.018759 | 0.013167 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.028585 | 0.022093 | 0.017986 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027088 | 0.017961 | 0.011612 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066305 | 0.031565 | 0.018844 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.049027 | 0.035849 | 0.020471 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.010889 | 0.007078 | 0.004429 |
| limiter_active | 31216.000000 | 0.074601 | 0.034455 | 0.026895 | 0.023049 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.043259 | 0.035148 | 0.031404 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.011004 | 0.008209 | 0.006311 |

## Viability

This pass is viable as a clean memoryless bristle/slip-angle candidate, but not as the best broad-envelope model. The launch estimate is diagnostic only; broad RMSE should be judged against the standalone contact traction testbed and force-level testbed. If those remain lower on validation, this law is better treated as a simpler interpretable baseline or as a shape for a later stateful LuGre/brush pass.

Unresolved hysteresis remains out of scope by design. Rows with limiter or hardware saturation evidence are reported as risk slices, not fit authority.

## Outputs

- `fit_relaxation_free_bristle_slip.py`
- `optimizer_summary.csv`
- `optimizer_trace.csv`
- `selected_parameters.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_slices.csv`
- `common_range_metrics.csv`
- `launch_estimate.csv`
- `prediction_sample.csv`
- `relaxation_free_bristle_slip_report.md`
