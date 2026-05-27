# Regime Linear To Unshifted Stribeck Yaw-Support Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Candidate Equation

The selected candidate is a standalone yaw-support law, expressed as an additional yaw-opposing support magnitude for replacement analysis rather than as `old + residual` runtime logic. It uses kinematic/load inputs only: `abs(yawRate)`, `abs(Vf)`, yaw-contact geometry, and total normal load. The old PlantModel and prior fitted models are used only as external baselines for error reporting.

Let `w = abs(yawRate)`, `v = abs(Vf)`, `N_scale = (N / N_nom)^load_exponent`, and `r_yaw` be the kinematic contact-relative speed per rad/s of yaw.

`a = v * k1`

`b = v * k2 + k3`, with `k1 < k2` and `b - a >= 0`.

`vt2 = v^2 + (rel_weight * r_yaw * w)^2`

`R = speed_fade^2 / (speed_fade^2 + vt2)`

`S = stribeck_speed^2 / (stribeck_speed^2 + vt2)`

Raw unshifted Stribeck branch:

`M_stribeck = N_scale * R * (sliding_nm + static_extra_nm * S)`

Boundary Stribeck value at the end of the linear section:

`vt2_a = v^2 + (rel_weight * r_yaw * a)^2`

`M_a = N_scale * R(vt2_a) * (sliding_nm + static_extra_nm * S(vt2_a))`

Linear branch with derived slope:

`M_linear = M_a * w / a` when `a > 0`; if `a = 0`, the linear region has zero width.

Regime blend:

`M_support = M_linear` for `w <= a`

`M_support = (1 - t) * L_boundary + t * M_stribeck`, `t = clamp((w - a) / (b - a), 0, 1)`, `L_boundary = M_a`, for `a < w < b`

`M_support = M_stribeck` for `w >= b`

Additive yaw torque sign convention for evaluation:

`M_additive = -sign(yawRate) * M_support`

The Stribeck branch is intentionally unshifted: the middle section is a linear blend weight into the raw Stribeck curve, with no vertical offset applied to the Stribeck curve. The linear slope is derived from `M_a / a`, so it is not an independent fitted gain and changes with `Vf`. This run uses the `L_boundary` interpretation for the non-Stribeck side of the blend; that preserves the raw peak instead of extending the line above the Stribeck branch.

## Selected Constants

| variant | k1 | k2 | k3_radps | stribeck_speed_mps | speed_fade_mps | rel_weight | load_exponent | sliding_nm | static_extra_nm | train_weighted_rmse_nm | primary_rmse_nm | validation_rmse_nm | launch_max_abs_command |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standard | 0.375777 | 1.280373 | 0.011430 | 0.010246 | 0.638149 | 0.075852 | 0.500000 | 0.000000 | 0.066338 | 0.002893 | 0.037540 | 0.065247 | 0.645931 |
| latest_weighted | 0.720424 | 0.791926 | 0.026871 | 0.010440 | 0.433340 | 0.033889 | 0.750000 | 0.000000 | 0.065666 | 0.003450 | 0.037626 | 0.065623 | 0.645694 |

Nominal load used for scaling: `1.932931 N`.

## In-Place Command Estimate

Synthetic reference: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The command target `+0.646/-0.646` corresponds to extra opposing yaw support `0.065568 Nm` with the shared motor-command helper.

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | max_abs_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- |
| standard | 0.065556 | 0.080350 | 0.645931 | -0.645931 | 0.645931 | True |
| latest_weighted | 0.065516 | 0.080310 | 0.645694 | -0.645694 | 0.645694 | True |

## Standard Split Metrics

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | corrected_mae_nm | corrected_median_abs_nm | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 15 | 0.036866 | 0.037540 | -1.829083 | 0.026210 | 0.017002 | 0.037502 | 0.000568 |
| open_floor_fit_downweighted | 31165 | 9 | 0.043194 | 0.047330 | -9.575262 | 0.030952 | 0.014979 | 0.045401 | 0.000568 |
| open_floor_validation_only | 14542 | 9 | 0.016931 | 0.039543 | -133.555295 | 0.027260 | 0.011041 | 0.035911 | 0.004548 |
| diag_validation_only | 11108 | 3 | 0.084621 | 0.112139 | -32.519126 | 0.099291 | 0.108187 | 0.111702 | 0.022594 |
| aux_downweighted_validation | 14448 | 13 | 0.051266 | 0.070171 | -36.875249 | 0.052311 | 0.043057 | 0.074949 | 0.003929 |
| validation_non_authoritative | 71263 | 34 | 0.050234 | 0.065247 | -29.886362 | 0.045181 | 0.025597 | 0.064310 | 0.001274 |

## Selected-Log Metrics

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_signed_median_nm | corrected_signed_median_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.045728 | -27.568937 | 0.009360 | 0.001701 | 0.028661 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.054841 | -135.594911 | -0.000686 | 0.000665 | 0.061295 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.029771 | -83.578281 | -0.003116 | 0.002667 | 0.000568 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.045241 | -0.589825 | 0.009991 | 0.009011 | 0.000346 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.042077 | 1.191863 | 0.012155 | 0.013614 | 0.002666 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.041182 | -3.409485 | 0.008941 | 0.009835 | 0.000058 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.041650 | -3.210609 | 0.003675 | 0.003614 | 0.000193 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.059769 | -6.303052 | 0.000167 | 0.000143 | 0.000604 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.113289 | -32.908428 | -0.012440 | -0.017505 | 0.025021 |

## Latest-Log Behavior

The standard fit includes May 4 rows only through their normal split membership, chiefly downweighted open-floor rows. In standard training weights, April rows contribute `0.984` of row-weight and the two May 4 requested latest logs contribute `0.016` of row-weight. Therefore the standard constants are still April-dominated, though not April-only.

Latest-weighted rerun: the two requested May 4 logs were raised to at least weight `3.0` regardless of split. This is a sensitivity check, not the selected standard fit.

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.045635 | -27.307786 | 0.028965 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.054450 | -133.913553 | 0.060849 |

## Risk Slices

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 49 | 0.025842 | 0.039312 | -52.127214 | 0.045220 | 0.000584 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 47 | 0.024100 | 0.023883 | 0.900830 | 0.027071 | 0.000158 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 49 | 0.073503 | 0.093500 | -27.205513 | 0.080183 | 0.019269 |
| low_speed_yaw_vf_lt_0p15_yaw_ge_0p5 | 20694 | 45 | 0.072832 | 0.088090 | -20.950039 | 0.085473 | 0.007173 |
| high_forward_vf_ge_0p5 | 3120 | 35 | 0.046415 | 0.046373 | 0.090920 | 0.059472 | 0.000006 |
| high_speed_abs_vf_ge_0p7 | 144 | 17 | 0.088793 | 0.088794 | -0.001831 | 0.089498 | 0.000004 |
| limiter_active | 30265 | 43 | 0.074736 | 0.084280 | -12.770354 | 0.086761 | 0.001250 |
| hardware_saturation_evidence | 5017 | 24 | 0.063673 | 0.065054 | -2.168534 | 0.061445 | 0.000479 |
| may4_latest_logs | 5217 | 2 | 0.032158 | 0.048994 | -52.357194 | 0.050491 | 0.049657 |
| april_rows | 87807 | 31 | 0.037157 | 0.040876 | -10.009694 | 0.038586 | 0.000578 |

## Comparison To Existing References

| group | baseline_rmse_nm | unshifted_corrected_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.037540 | 0.027908 | 0.021083 | 0.019508 | 0.016688 |
| open_floor_fit_downweighted | 0.043194 | 0.047330 | 0.041507 | 0.032823 | 0.035267 | 0.027952 |
| open_floor_validation_only | 0.016931 | 0.039543 | 0.011394 | 0.011207 | 0.009294 | 0.007665 |
| diag_validation_only | 0.084621 | 0.112139 | 0.084410 | 0.038489 | 0.025685 | 0.020030 |
| aux_downweighted_validation | 0.051266 | 0.070171 | 0.048820 | 0.027490 | 0.027446 | 0.019198 |
| validation_non_authoritative | 0.050234 | 0.065247 |  | 0.029680 | 0.028585 | 0.022157 |

Selected-log comparison:

| run_id | dataset_split | baseline_rmse_nm | unshifted_corrected_rmse_nm | force_domain_stribeck_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.045728 | 0.020956 | 0.016473 | 0.011393 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.054841 | 0.015366 | 0.010900 | 0.009662 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.029771 | 0.013487 | 0.012118 | 0.012232 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.045241 | 0.033938 | 0.014672 | 0.011282 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.042077 | 0.027284 | 0.016287 | 0.015434 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.041182 | 0.034302 | 0.026919 | 0.020474 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.041650 | 0.035648 | 0.028266 | 0.022853 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.059769 | 0.056141 | 0.043763 | 0.036549 |
| diag003 | diag_validation_only | 0.085238 | 0.113289 | 0.085032 | 0.026235 | 0.019757 |

## Interpretation

- The unshifted regime law passes the hard launch gate and lands near the requested `+0.646/-0.646` in-place command scale without command/request or preprojection inputs.
- The standard fit improves the primary/open-floor residual target, but broad validation remains materially worse than the standalone contact-traction and true-traction testbeds because this law is still one-sided yaw-opposing support. It cannot remove rows where the current baseline already over-resists yaw.
- The low-yaw branch is slope-matched to the raw Stribeck boundary for each `Vf`; the launch reference at `Vf=0`, `yawRate=1` enters the ordinary raw Stribeck branch for the selected constants.
- The May 4 latest-weighted rerun moves constants only modestly; the main failure mode is model shape, not simply April log dominance.

## Caveats

- This is not a production `PlantModel` change and was not built or unit-tested by design.
- The residual target is derived against the current PlantModel mirror for evaluation, but the candidate itself does not consume old force outputs at runtime.
- Total normal load comes from the shared feature extraction artifacts; yaw support speed is derived from `abs(yawRate)` and geometry. No logged UKF state-vector fields, command/request values, or preprojection force requests are used as selectors.
- Because the Stribeck term is unshifted, a mismatch between the linear branch and raw Stribeck branch can create a slope/value kink at the end of the transition. That kink is the cost of preserving peak raw Stribeck support without vertical offset.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\regime_linear_to_stribeck_unshifted\fit_unshifted_regime_stribeck.py
```

## Output Files

- `fit_unshifted_regime_stribeck.py`
- `unshifted_report.md`
- `selected_parameters.csv`
- `candidate_scores.csv`
- `standard_split_metrics.csv`
- `standard_selected_log_metrics.csv`
- `standard_risk_metrics.csv`
- `standard_phase_metrics.csv`
- `latest_weighted_split_metrics.csv`
- `latest_weighted_selected_log_metrics.csv`
- `latest_weighted_risk_metrics.csv`
- `baseline_comparison.csv`
- `selected_log_comparison.csv`
- `in_place_1radps_command.csv`
- `prediction_sample.csv`
- `commands_run.txt`
