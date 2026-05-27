# Optimized Unshifted Linear-Blend-To-Stribeck Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Model

Inputs used by the selected candidate are only `abs(yawRate)`, `abs(Vf)`, total normal load, and fixed yaw-contact geometry from the shared constants. Logged command/request/preprojection/UKF fields and old core contact-force outputs are not candidate inputs. Prior models are used only as reporting references.

Let `y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v*k1`, and `x2 = v*k2 + k3`, with `k1 < k2` and `x2 > x1`.

Raw unshifted Stribeck branch:

`S_raw = (N/N_nom)^load_exponent * speed_fade^2/(speed_fade^2 + vt2) * (sliding_nm + static_extra_nm * stribeck_speed^2/(stribeck_speed^2 + vt2))`

`vt2 = v^2 + (rel_weight * yaw_contact_offset * y)^2`

The Stribeck branch is never shifted or offset. The search evaluated two derived-slope interpretations:

- `meet_x1`: the zero-anchored line meets raw Stribeck at `x1`; the transition blends from that boundary value into raw Stribeck.
- `reach_x2`: the zero-anchored line is derived from the raw Stribeck value at `x2`; the transition blends the line continuation into raw Stribeck.

No independent low-yaw slope was fitted.

## Optimizer

Initial broad ranges:

{
  "k1": [
    0.00012,
    2.5
  ],
  "delta_k": [
    0.00012,
    5.0
  ],
  "k3_radps": [
    0.004,
    2.0
  ],
  "stribeck_speed_mps": [
    0.00016,
    0.65
  ],
  "speed_fade_mps": [
    0.01,
    5.0
  ],
  "rel_weight": [
    -3.0,
    3.0
  ],
  "load_exponent": [
    0.0,
    1.5
  ]
}

Method: deterministic bounded coordinate-pattern search in transformed parameter space. Positive parameters are optimized in log coordinates; `rel_weight` is optimized as a signed value because it appears only squared in `vt2`, so zero is not an artificial search boundary. `load_exponent` is fixed at `1.0` for physically direct normal-load scaling rather than used as a curve-shape escape hatch. Spaced/random points are used only to choose starting seeds and bounds sanity checks. Each nonlinear shape evaluation solves `sliding_nm` and `static_extra_nm` by weighted least squares with an exact analytic launch equality constraint.

Objective: minimize weighted training RMSE after satisfying launch, nonnegative amplitude, and peak constraints. Standard weights are primary open-floor `1.0` and downweighted open-floor `0.25`; the latest-weighted sensitivity branch raises the two May 4 logs to at least `3.0`.

Analytic launch constraint: the measured `+0.646/-0.646` launch threshold maps to `0.065568 Nm` extra opposing yaw support at `Vf=0`, `Vr=0`, `yawRate=+1`. For each shape, the amplitude solve enforces `launch_basis dot [sliding_nm, static_extra_nm] = 0.065568`.

Boundary policy: any final parameter within `3%` normalized distance of an optimization boundary triggers range expansion and re-optimization. If still boundary-adjacent after the expansion budget, the search is marked unresolved rather than selected.

Optimizer counts:

| variant | optimizer_runs | optimizer_evaluations | best_objective_nm |
| --- | --- | --- | --- |
| latest_weighted | 54 | 26883 | 0.036283 |
| standard | 54 | 21639 | 0.037558 |

Seed scores are in `seed_scores.csv`; optimizer traces are in `optimizer_trace.csv`; convergence summaries are in `optimizer_summary.csv`; range-expansion decisions are in `range_expansion_audit.csv`; final parameter margins are in `boundary_audit.csv`.

## Selected Constants

| variant | interpretation | k1 | k2 | delta_k | k3_radps | stribeck_speed_mps | speed_fade_mps | rel_weight | load_exponent | sliding_nm | static_extra_nm | peak_nm | train_weighted_rmse_nm | primary_rmse_nm | validation_rmse_nm | latest_rmse_nm | launch_max_abs_command | objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standard | reach_x2 | 0.000006 | 0.000016 | 0.000010 | 0.375174 | 0.000431 | 0.060489 | -0.010345 | 1.000000 | 0.014549 | 0.057404 | 0.071954 | 0.037559 | 0.036296 | 0.055220 | 0.032745 | 0.646000 | 0.037559 |
| latest_weighted | reach_x2 | 0.000005 | 0.000010 | 0.000005 | 1.242596 | 0.000665 | 0.049268 | -0.019350 | 1.000000 | 0.019695 | 0.058145 | 0.077840 | 0.036285 | 0.036366 | 0.054976 | 0.030978 | 0.646000 | 0.036285 |

Nominal load used for scaling: `1.932931 N`.

Peak gate: selected nominal peak must be `<= 0.120 Nm`; the standard selected peak is `0.071954 Nm`.

Final standard candidate interior to final reported numeric ranges: `True`.

Accepted selected candidate: `False`.

Search status: `unresolved_boundary_after_expansion`. The optimizer repeatedly pushed `k1` and `delta_k` toward smaller lower bounds during expansion, so this run reports the best constrained optimizer result as diagnostic rather than as an accepted selected model.

## In-Place Command Estimate

Synthetic reference: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The target `+0.646/-0.646` command corresponds to extra opposing yaw support `0.065568 Nm` with the shared motor-command estimate. Launch was a gate/objective only; no giant pseudo-row was added to the coefficient fit.

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | max_abs_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- |
| standard | 0.065568 | 0.080362 | 0.646000 | -0.646000 | 0.646000 | True |
| latest_weighted | 0.065568 | 0.080362 | 0.646000 | -0.646000 | 0.646000 | True |

## Standard Split Metrics

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | corrected_mae_nm | corrected_median_abs_nm | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 15 | 0.036866 | 0.036296 | 1.546231 | 0.025003 | 0.015813 | 0.035871 | 0.001292 |
| open_floor_fit_downweighted | 31165 | 9 | 0.043194 | 0.044465 | -2.942489 | 0.028494 | 0.013552 | 0.042352 | 0.001265 |
| open_floor_validation_only | 14542 | 9 | 0.016931 | 0.020588 | -21.603476 | 0.012329 | 0.006388 | 0.021274 | 0.002172 |
| diag_validation_only | 11108 | 3 | 0.084621 | 0.096737 | -14.317649 | 0.084799 | 0.093327 | 0.096176 | 0.011871 |
| aux_downweighted_validation | 14448 | 13 | 0.051266 | 0.056158 | -9.541956 | 0.037016 | 0.019036 | 0.062569 | 0.001506 |
| validation_non_authoritative | 71263 | 34 | 0.050234 | 0.055220 | -9.925100 | 0.035699 | 0.015369 | 0.053922 | 0.001923 |

## Selected-Log Metrics

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_signed_median_nm | corrected_signed_median_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.034726 | 3.124799 | 0.009360 | 0.007301 | 0.008649 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.028459 | -22.257443 | -0.000686 | 0.000067 | 0.010305 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.016807 | -3.635704 | -0.003116 | -0.000966 | 0.001232 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.044547 | 0.952934 | 0.009991 | 0.008704 | 0.001208 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.041162 | 3.339827 | 0.012155 | 0.012553 | 0.002607 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.040337 | -1.288341 | 0.008941 | 0.009489 | 0.000457 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.041281 | -2.295686 | 0.003675 | 0.003644 | 0.000570 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.058453 | -3.961850 | 0.000167 | 0.000267 | 0.002167 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.098315 | -15.342124 | -0.012440 | -0.015616 | 0.012496 |

## May 4 Latest Logs

The standard fit includes May 4 rows only through normal split weighting. In standard training weights, April rows contribute `0.984` of row-weight and the two May 4 latest logs contribute `0.016` of row-weight.

Latest-weighted sensitivity branch raises the two May 4 logs to at least weight `3.0`; it is reported separately and is not the selected standard fit.

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.033835 | 5.611222 | 0.004046 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.024417 | -4.895511 | 0.003882 |

## Risk Metrics

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 49 | 0.025842 | 0.025834 | 0.031506 | 0.032707 | 0.000488 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 47 | 0.024100 | 0.023878 | 0.923570 | 0.027079 | 0.000226 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 49 | 0.073503 | 0.083315 | -13.348814 | 0.073212 | 0.013143 |
| low_speed_yaw_vf_lt_0p15_yaw_ge_0p5 | 20694 | 45 | 0.072832 | 0.080539 | -10.582965 | 0.079731 | 0.010870 |
| high_forward_vf_ge_0p5 | 3120 | 35 | 0.046415 | 0.046391 | 0.051195 | 0.059490 | 0.000076 |
| high_speed_abs_vf_ge_0p7 | 144 | 17 | 0.088793 | 0.088818 | -0.028521 | 0.089514 | 0.000063 |
| limiter_active | 30265 | 43 | 0.074736 | 0.079406 | -6.249185 | 0.082675 | 0.004415 |
| hardware_saturation_evidence | 5017 | 24 | 0.063673 | 0.064708 | -1.625017 | 0.061133 | 0.002345 |
| may4_latest_logs | 5217 | 2 | 0.032158 | 0.032745 | -1.826221 | 0.031747 | 0.009154 |
| april_rows | 87807 | 31 | 0.037157 | 0.037658 | -1.350134 | 0.034781 | 0.001257 |

## Reference Comparisons

| group | baseline_rmse_nm | unshifted_optimized_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.036296 | 0.027908 | 0.021083 | 0.019508 | 0.016688 |
| open_floor_fit_downweighted | 0.043194 | 0.044465 | 0.041507 | 0.032823 | 0.035267 | 0.027952 |
| open_floor_validation_only | 0.016931 | 0.020588 | 0.011394 | 0.011207 | 0.009294 | 0.007665 |
| diag_validation_only | 0.084621 | 0.096737 | 0.084410 | 0.038489 | 0.025685 | 0.020030 |
| aux_downweighted_validation | 0.051266 | 0.056158 | 0.048820 | 0.027490 | 0.027446 | 0.019198 |
| validation_non_authoritative | 0.050234 | 0.055220 |  | 0.029680 | 0.028585 | 0.022157 |

Selected-log comparison:

| run_id | dataset_split | baseline_rmse_nm | unshifted_optimized_rmse_nm | force_domain_stribeck_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.034726 | 0.020956 | 0.016473 | 0.011393 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.028459 | 0.015366 | 0.010900 | 0.009662 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.016807 | 0.013487 | 0.012118 | 0.012232 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.044547 | 0.033938 | 0.014672 | 0.011282 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.041162 | 0.027284 | 0.016287 | 0.015434 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.040337 | 0.034302 | 0.026919 | 0.020474 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.041281 | 0.035648 | 0.028266 | 0.022853 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.058453 | 0.056141 | 0.043763 | 0.036549 |
| diag003 | diag_validation_only | 0.085238 | 0.098315 | 0.085032 | 0.026235 | 0.019757 |

## Boundary Audit

| parameter | value | range_min | range_max | normalized_margin_to_nearest_boundary | boundary_adjacent | search_status |
| --- | --- | --- | --- | --- | --- | --- |
| k1 | 0.000006 | 0.000001 | 2.500000 | 0.125208 | False | unresolved_boundary_after_expansion |
| delta_k | 0.000010 | 0.000005 | 5.000000 | 0.049560 | False | unresolved_boundary_after_expansion |
| k3_radps | 0.375174 | 0.004000 | 2.000000 | 0.269287 | False | unresolved_boundary_after_expansion |
| stribeck_speed_mps | 0.000431 | 0.000160 | 0.650000 | 0.119345 | False | unresolved_boundary_after_expansion |
| speed_fade_mps | 0.060489 | 0.010000 | 5.000000 | 0.289619 | False | unresolved_boundary_after_expansion |
| rel_weight | -0.010345 | -3.000000 | 3.000000 | 0.498276 | False | unresolved_boundary_after_expansion |
| load_exponent | 1.000000 | 0.000000 | 1.500000 | 0.333333 | False | unresolved_boundary_after_expansion |
| interpretation | reach_x2 | meet_x1;reach_x2 | meet_x1;reach_x2 | 1.000000 | False | unresolved_boundary_after_expansion |

## Interpretation

- The best standard optimizer candidate passes the `|cmd| >= 0.6` launch gate, but it is not accepted as selected because repeated bound expansion kept driving `k1`/`delta_k` toward zero.
- The low-yaw slope is derived from the selected handoff interpretation; no independent linear slope is tuned.
- The best selected shape uses the raw, unshifted Stribeck branch, so peak support is the fitted `sliding_nm + static_extra_nm` at nominal load and is not inflated by a shifted transition.
- This is now a fair continuous optimization attempt for the unshifted family under the corrected slope and launch constraints. The outcome is unresolved, not selected. Even the best constrained diagnostic candidate validates worse than the current force-domain Stribeck, rational residual reference, and standalone contact-traction testbed on most broad splits.

## Reproduce

```powershell
python codex_analysis\yaw_model_variant_fits\regime_linear_blend_stribeck_unshifted_optimized\fit_unshifted_optimized.py
```

## Output Files

- `fit_unshifted_optimized.py`
- `unshifted_optimized_report.md`
- `seed_scores.csv`
- `optimizer_trace.csv`
- `optimizer_summary.csv`
- `selected_parameters.csv`
- `search_decision.csv`
- `standard_split_metrics.csv`
- `standard_selected_log_metrics.csv`
- `standard_risk_metrics.csv`
- `standard_phase_metrics.csv`
- `latest_weighted_split_metrics.csv`
- `latest_weighted_selected_log_metrics.csv`
- `latest_weighted_risk_metrics.csv`
- `latest_weighted_phase_metrics.csv`
- `baseline_comparison.csv`
- `selected_log_comparison.csv`
- `in_place_1radps_command.csv`
- `boundary_audit.csv`
- `range_expansion_audit.csv`
- `prediction_sample.csv`
- `metadata.json`
