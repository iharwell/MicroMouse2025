# Max/Norm Static-Yield Allocation

Analysis-only output. Production code, build metadata, and tests were not edited.

## Recommendation

Use the sqrt-softplus reserve allocation as the best transition shape from this pass, but treat the coefficient as a validation candidate rather than a production tune. It passes the in-place launch command gate while preserving a meaningful share of Variant C's broad validation advantage; it is materially better than the force-domain Stribeck reference on primary, downweighted, diag, and aux splits, but not on the open-floor validation-only split.

The physical interpretation is that the static-yield branch is a reserve, not a second moving-contact residual. Variant C already accounts for sliding/moving contact; the launch reserve only fills the missing same-sign yaw-opposing torque when the reserve exceeds the already-opposing part of Variant C.

## Selected Equation

`v_t = sqrt((rel_weight * max(vbar_rel, load_weighted_rel, load_weighted_lat))^2 + |Vf|^2)`

`G_v = 1 / (1 + (v_t / v_k)^2)`

`G_u = u_actual^2 / (u_actual^2 + u_k^2)`

`M_launch = M_static_peak * G_v * G_u * clamp(N / N_nom, 0.25, 2.0)`

`C_pos = soft_positive(M_C, eps)`

`M_opp = M_C + soft_positive(M_launch - c_credit * C_pos, eps)`

where `soft_positive(x, eps) = 0.5 * (x + sqrt(x^2 + eps^2))`. The selected form uses no trig, no exp, and no tanh; the only non-polynomial operation is sqrt.

## Coefficients

| parameter | value | unit |
| --- | --- | --- |
| allocation_form | sqrt_softplus_reserve | enum |
| static_peak_nm | 0.100000 | Nm |
| speed_knee_mps | 0.030000 | m/s |
| force_knee_util | 0.450000 | actual force utilization |
| rel_weight | 0.750000 | dimensionless |
| eps_nm | 0.001000 | Nm |
| c_credit | 1.000000 | fraction |
| nominal_load_n | 1.932931 | N |
| nominal_longitudinal_yield_nm | 0.110969 | Nm |

## +1 rad/s In-Place Command

| allocation extra | C extra | total opp | left cmd | right cmd | L-R delta | max abs cmd | gate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.061733 | 0.018241 | 0.076528 | 0.623527 | -0.623527 | 1.247055 | 0.623527 | 1.000000 |

## Split RMSE Versus C And Force-Domain

| split | count | baseline | allocation | C | force-domain | alloc-C | alloc-force |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317.000000 | 0.036866 | 0.025771 | 0.024120 | 0.027908 | 0.001651 | -0.002137 |
| open_floor_fit_downweighted | 31165.000000 | 0.043194 | 0.035025 | 0.033093 | 0.041507 | 0.001932 | -0.006482 |
| open_floor_validation_only | 14542.000000 | 0.016931 | 0.018486 | 0.014417 | 0.011394 | 0.004069 | 0.007093 |
| diag_validation_only | 11108.000000 | 0.084621 | 0.058666 | 0.038767 | 0.084410 | 0.019899 | -0.025744 |
| aux_downweighted_validation | 14448.000000 | 0.051266 | 0.043803 | 0.028529 | 0.048820 | 0.015273 | -0.005017 |
| validation_non_authoritative | 71263.000000 | 0.050234 | 0.039137 | 0.030342 |  | 0.008795 |  |

## Phase RMSE

| split | phase | count | baseline | allocation | improvement % |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | entry | 9719.000000 | 0.045837 | 0.034331 | 25.101228 |
| primary_open_floor_fit_authoritative | plateau | 26778.000000 | 0.028771 | 0.018928 | 34.212485 |
| primary_open_floor_fit_authoritative | exit | 10820.000000 | 0.044807 | 0.030967 | 30.887457 |
| open_floor_fit_downweighted | entry | 8341.000000 | 0.056178 | 0.044180 | 21.356499 |
| open_floor_fit_downweighted | plateau | 14651.000000 | 0.024365 | 0.019916 | 18.260261 |
| open_floor_fit_downweighted | exit | 8173.000000 | 0.053191 | 0.044440 | 16.453338 |
| open_floor_validation_only | entry | 2104.000000 | 0.022225 | 0.023032 | -3.633478 |
| open_floor_validation_only | plateau | 10533.000000 | 0.013257 | 0.014565 | -9.867433 |
| open_floor_validation_only | exit | 1905.000000 | 0.025902 | 0.029153 | -12.550498 |
| diag_validation_only | entry | 3215.000000 | 0.105908 | 0.050241 | 52.561952 |
| diag_validation_only | plateau | 4712.000000 | 0.064485 | 0.058579 | 9.158021 |
| diag_validation_only | exit | 3181.000000 | 0.086655 | 0.066213 | 23.590064 |
| aux_downweighted_validation | entry | 2710.000000 | 0.074083 | 0.044952 | 39.322304 |
| aux_downweighted_validation | plateau | 8708.000000 | 0.037221 | 0.042589 | -14.420928 |
| aux_downweighted_validation | exit | 3030.000000 | 0.060349 | 0.046139 | 23.545859 |

## Selected-Log RMSE Versus C And Force-Domain

| run | split | count | baseline | allocation | C | force-domain | alloc-C | alloc-force |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456.000000 | 0.035846 | 0.034531 | 0.028076 | 0.020956 | 0.006454 | 0.013575 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761.000000 | 0.023278 | 0.026701 | 0.019341 | 0.015366 | 0.007360 | 0.011334 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187.000000 | 0.016217 | 0.017618 | 0.014403 | 0.013487 | 0.003215 | 0.004131 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031.000000 | 0.044975 | 0.018977 | 0.018608 | 0.033938 | 0.000368 | -0.014961 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880.000000 | 0.042584 | 0.026975 | 0.023754 | 0.027284 | 0.003221 | -0.000308 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757.000000 | 0.039824 | 0.028526 | 0.027743 | 0.034302 | 0.000783 | -0.005777 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925.000000 | 0.040355 | 0.030279 | 0.029338 | 0.035648 | 0.000941 | -0.005369 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284.000000 | 0.056225 | 0.042980 | 0.041312 | 0.056141 | 0.001667 | -0.013162 |
| diag003 | diag_validation_only | 5580.000000 | 0.085238 | 0.059252 | 0.038970 | 0.085032 | 0.020281 | -0.025781 |

## Risk Slices

| group | count | baseline | allocation | med abs before | med abs after | RB change % |
| --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367.000000 | 0.025842 | 0.022989 | 0.006082 | 0.005914 | 15.814773 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746.000000 | 0.024100 | 0.013306 | 0.005326 | 0.005061 | 48.561654 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704.000000 | 0.073503 | 0.050063 | 0.054369 | 0.028807 | 29.990430 |
| high_forward_vf_ge_0p5 | 3120.000000 | 0.046415 | 0.029968 | 0.012755 | 0.013077 | 47.338111 |
| limiter_active | 31216.000000 | 0.074601 | 0.046162 | 0.053156 | 0.027618 | 36.202146 |

## Candidate Ranking

| rank | allocation_form | objective_score | static_peak_nm | speed_knee_mps | force_knee_util | c_credit | in_place_max_abs_command | validation_rb_corrected_rmse_nm | primary_rb_corrected_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1.000000 | sqrt_softplus_reserve | 0.051845 | 0.100000 | 0.030000 | 0.450000 | 1.000000 | 0.623527 | 0.038223 | 0.024796 |
| 2.000000 | softmax_reserve | 0.051845 | 0.100000 | 0.030000 | 0.450000 | 1.000000 | 0.623527 | 0.038223 | 0.024796 |
| 3.000000 | norm_reserve | 0.051875 | 0.100000 | 0.030000 | 0.450000 | 1.000000 | 0.651096 | 0.038282 | 0.024662 |
| 4.000000 | norm_reserve | 0.052561 | 0.090000 | 0.045000 | 0.450000 | 1.000000 | 0.628991 | 0.038677 | 0.025210 |
| 5.000000 | sqrt_softplus_reserve | 0.054246 | 0.100000 | 0.045000 | 0.450000 | 1.000000 | 0.672065 | 0.040105 | 0.025800 |
| 6.000000 | softmax_reserve | 0.054246 | 0.100000 | 0.045000 | 0.450000 | 1.000000 | 0.672065 | 0.040105 | 0.025800 |
| 7.000000 | norm_reserve | 0.055089 | 0.100000 | 0.045000 | 0.450000 | 1.000000 | 0.694707 | 0.040207 | 0.025690 |
| 8.000000 | norm_reserve | 10.048616 | 0.090000 | 0.020000 | 0.450000 | 1.000000 | 0.519579 | 0.035433 | 0.023966 |
| 9.000000 | sqrt_softplus_reserve | 10.048664 | 0.090000 | 0.020000 | 0.450000 | 1.000000 | 0.454121 | 0.035414 | 0.024141 |
| 10.000000 | softmax_reserve | 10.048664 | 0.090000 | 0.020000 | 0.450000 | 1.000000 | 0.454121 | 0.035414 | 0.024141 |
| 11.000000 | norm_reserve | 10.049745 | 0.100000 | 0.020000 | 0.450000 | 1.000000 | 0.571049 | 0.036479 | 0.024090 |
| 12.000000 | sqrt_softplus_reserve | 10.049779 | 0.100000 | 0.020000 | 0.450000 | 1.000000 | 0.527091 | 0.036452 | 0.024256 |

## Computational Cost

Per tick, after Variant C is available, the selected allocation adds roughly: one `max` over contact-speed estimates, two squares for `v_t`, one sqrt for `v_t`, two rational gates, one clamp/multiply for load scaling, one sqrt soft-positive for `C_pos`, and one sqrt soft-positive for the reserve. It has no trig table, no exponential, no tanh, and no per-contact history state.

The in-place command estimator still uses the existing analysis motor/launch-friction approximation so it can be compared to prior workers; that estimator is not part of the selected allocation law.

## Caveats

- The actual-force utilization selector is derived from projected/actual contact force and normal load. It does not use command/request values as a traction selector.
- The model does not use UKF state columns.
- Validation RMSE remains worse than pure Variant C because any launch-gate-passing static reserve adds resistance in some rows where C already fit the residual. The allocation shape reduces, but does not erase, that tradeoff.
- The best fit is sensitive to the synthetic +1 rad/s command gate. A targeted in-place launch dataset should replace that synthetic anchor before production tuning.

## Reproduce

```powershell
python codex_analysis\yaw_model_variant_fits\transition_options\max_norm_yield_allocation\fit_max_norm_yield_allocation.py
```

## Output Files

- `fit_max_norm_yield_allocation.py`
- `max_norm_yield_allocation_report.md`
- `selected_parameters.csv`
- `candidate_scores.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `split_comparison_vs_c_force_domain.csv`
- `selected_log_comparison_vs_c_force_domain.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid.csv`
- `prediction_sample.csv`
- `metadata.json`
- `commands_run.txt`
