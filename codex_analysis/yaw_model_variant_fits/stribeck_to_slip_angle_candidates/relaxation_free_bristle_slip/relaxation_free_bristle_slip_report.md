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
| drive_scale | 0.10811575 |
| longitudinal_mu | 0.574634245 |
| longitudinal_k_mps | 0.535390261 |
| stribeck_slide_mu | 0.233032501 |
| stribeck_static_extra_mu | 0.0718430492 |
| stribeck_speed_mps | 0.294760746 |
| transition_vf_mps | 0.0305517204 |
| alpha_stiff_base | 0.694772904 |
| alpha_stiff_speed_gain | 0.717583528 |
| alpha_stiff_speed_k_mps | 1.98932134 |
| alpha_saturation_mu | 1.63355498 |
| alpha_floor_mps | 0.0317133188 |

## Launch

| opposing bristle scrub Nm | left command | right command | max abs command | gate | target abs error |
| ---: | ---: | ---: | ---: | --- | ---: |
| 0.008687 | 0.645929 | -0.645929 | 0.645929 | pass | 0.000071 |

## Optimization

Bounded Nelder-Mead was run from deterministic spaced and random seeds. The launch gate was a hard penalty; broad RMSE was optimized with run-balanced pseudo-Huber loss on the primary authoritative split.

| label | selected | objective | train_rmse_nm | train_robust_loss | launch_max_abs_command | iterations | converged | boundary_hits | collapse_notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standalone_static_yield_inspired | false | 0.000178 | 0.016248 | 0.000178 | 0.645073 | 520.000000 | false | 7.000000 | transition_at_lower_bound |
| deterministic_random_seed_0 | false | 0.000179 | 0.016425 | 0.000179 | 0.645532 | 520.000000 | false | 8.000000 | static_extra_collapsed;transition_at_lower_bound |
| force_domain_launch_heavier | false | 0.000179 | 0.016347 | 0.000179 | 0.644246 | 520.000000 | false | 3.000000 | transition_at_lower_bound |
| deterministic_random_seed_1 | false | 0.000179 | 0.016327 | 0.000179 | 0.648076 | 520.000000 | false | 2.000000 | none |
| deterministic_random_seed_2 | true | 0.000179 | 0.016312 | 0.000179 | 0.645929 | 520.000000 | false | 3.000000 | transition_at_lower_bound |
| deterministic_random_seed_4 | false | 0.000180 | 0.016479 | 0.000180 | 0.644433 | 520.000000 | false | 5.000000 | static_extra_collapsed;transition_at_lower_bound |
| deterministic_random_seed_3 | false | 0.000181 | 0.016355 | 0.000181 | 0.646103 | 520.000000 | false | 2.000000 | none |
| slip_angle_forward_biased | false | 0.000181 | 0.016314 | 0.000181 | 0.645422 | 520.000000 | false | 1.000000 | none |

## Split Metrics

| group | count | baseline_rmse_nm | bristle_slip_rmse_nm | variant_c_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | standalone_contact_traction_rmse_nm | force_level_contact_testbed_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.022043 | 0.028529 | 0.048820 | 0.027490 | 0.020687 | 0.027896 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.026665 | 0.038767 | 0.084410 | 0.038489 | 0.023947 | 0.025703 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.026483 | 0.033093 | 0.041507 | 0.032823 | 0.026632 | 0.034837 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.008253 | 0.014417 | 0.011394 | 0.011207 | 0.007711 | 0.008849 |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.016764 | 0.024120 | 0.027908 | 0.021083 | 0.016888 | 0.019520 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.023021 | 0.030342 |  | 0.029680 | 0.022327 | 0.028416 |

## Selected Logs

| run_id | dataset_split | count | baseline_rmse_nm | bristle_slip_rmse_nm | variant_c_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.011849 | 0.028076 | 0.020956 | 0.020128 | 0.011589 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.010537 | 0.019341 | 0.015366 | 0.015839 | 0.009838 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.012436 | 0.014403 | 0.013487 | 0.013203 | 0.012330 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.013113 | 0.018608 | 0.033938 | 0.016861 | 0.012307 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.015908 | 0.023754 | 0.027284 | 0.017657 | 0.016517 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.020796 | 0.027743 | 0.034302 | 0.026878 | 0.020650 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.022386 | 0.029338 | 0.035648 | 0.027977 | 0.022382 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.033684 | 0.041312 | 0.056141 | 0.042487 | 0.034276 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.026401 | 0.038970 | 0.085032 | 0.038712 | 0.023743 |

## Risk Slices

| group | count | baseline_rmse_nm | bristle_slip_rmse_nm | rmse_improvement_fraction |
| --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.026309 | 0.556046 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.029088 | 0.604259 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027125 | 0.490184 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066488 | 0.299230 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.049464 | 0.305212 |
| fast_forward | 0.000000 |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011047 | 0.541627 |
| limiter_active | 31216.000000 | 0.074601 | 0.034681 | 0.535117 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.042707 | 0.329271 |
| open_floor_all | 93024.000000 | 0.036894 | 0.019712 | 0.465708 |
| diag_all | 11108.000000 | 0.084621 | 0.026665 | 0.684894 |
| aux_all | 14448.000000 | 0.051266 | 0.022043 | 0.570024 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.011423 | 0.644777 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range_name | count | baseline_rmse_nm | candidate_rmse_nm | candidate_mae_nm | candidate_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| calibration_low_vf_nonzero_yaw | 41686.000000 | 0.059262 | 0.026309 | 0.018985 | 0.013377 |
| in_place_scrub | 19704.000000 | 0.073503 | 0.029088 | 0.022594 | 0.018395 |
| slow_forward_turn | 19582.000000 | 0.053206 | 0.027125 | 0.018121 | 0.011822 |
| pre_design_turn_speed | 99.000000 | 0.094879 | 0.066488 | 0.031835 | 0.019736 |
| design_turn_speed_and_up | 4.000000 | 0.071194 | 0.049464 | 0.038275 | 0.023687 |
| fast_forward | 0.000000 |  |  |  |  |
| straightish_forward | 21746.000000 | 0.024100 | 0.011047 | 0.007219 | 0.004546 |
| limiter_active | 31216.000000 | 0.074601 | 0.034681 | 0.027274 | 0.023711 |
| hardware_saturation_evidence | 5017.000000 | 0.063673 | 0.042707 | 0.034583 | 0.030699 |
| may4_latest_logs | 5217.000000 | 0.032158 | 0.011423 | 0.008526 | 0.006637 |

## Viability

This pass is viable as a clean memoryless bristle/slip-angle candidate, but not as the best broad-envelope model. It clears the launch gate without command/request conditioning and keeps the contact-patch force structure. Its broad RMSE should be judged against the standalone contact traction testbed and force-level testbed: if those remain lower on validation, this law is better treated as a simpler interpretable baseline or as a shape for a later stateful LuGre/brush pass.

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
