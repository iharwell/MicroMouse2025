# Refined Unshifted Linear-Blend-To-Stribeck Fit

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

## Search

Initial broad ranges:

{
  "k1": [
    0.015,
    2.5
  ],
  "delta_k": [
    0.015,
    5.0
  ],
  "k3_radps": [
    0.004,
    2.0
  ],
  "stribeck_speed_mps": [
    0.004,
    0.65
  ],
  "speed_fade_mps": [
    0.05,
    5.0
  ],
  "rel_weight": [
    0.0,
    3.0
  ],
  "load_exponent": [
    0.0,
    1.5
  ]
}

Boundary policy: any final parameter within `3%` normalized distance of a search boundary triggered range expansion and rerun. If still boundary-adjacent after the expansion budget, the search would be marked unresolved rather than selected.

Evaluation counts:

| variant | stages | unique_coarse_evaluations | full_evaluations |
| --- | --- | --- | --- |
| latest_weighted | 3 | 6206 | 330 |
| standard | 3 | 6206 | 330 |

Search stages and ranges are in `search_summary.csv`; expansion decisions and final margins are in `boundary_audit.csv`.

## Selected Constants

| variant | interpretation | k1 | k2 | delta_k | k3_radps | stribeck_speed_mps | speed_fade_mps | rel_weight | load_exponent | sliding_nm | static_extra_nm | peak_nm | train_weighted_rmse_nm | primary_rmse_nm | validation_rmse_nm | latest_rmse_nm | launch_max_abs_command | objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standard | reach_x2 | 0.036374 | 1.252182 | 1.215808 | 1.176190 | 0.005205 | 0.845353 | 0.183215 | 1.301271 | 0.001414 | 0.084185 | 0.085599 | 0.037674 | 0.036556 | 0.054085 | 0.032118 | 0.646000 | 0.038444 |
| latest_weighted | reach_x2 | 0.695212 | 3.924897 | 3.229686 | 1.425027 | 0.004959 | 0.065226 | 0.163871 | 1.242593 | 0.012121 | 0.077143 | 0.089264 | 0.036408 | 0.036393 | 0.055561 | 0.031560 | 0.646000 | 0.037473 |

Nominal load used for scaling: `1.932931 N`.

Peak gate: selected nominal peak must be `<= 0.120 Nm`; the standard selected peak is `0.085599 Nm`.

Final standard candidate interior to searched ranges: `True`.

## In-Place Command Estimate

Synthetic reference: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The target `+0.646/-0.646` command corresponds to extra opposing yaw support `0.065568 Nm` with the shared motor-command estimate. Launch was a gate/objective only; no giant pseudo-row was added to the coefficient fit.

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | max_abs_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- |
| standard | 0.065568 | 0.080362 | 0.646000 | -0.646000 | 0.646000 | True |
| latest_weighted | 0.065568 | 0.080362 | 0.646000 | -0.646000 | 0.646000 | True |

## Standard Split Metrics

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | corrected_mae_nm | corrected_median_abs_nm | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 15 | 0.036866 | 0.036556 | 0.842444 | 0.025187 | 0.015935 | 0.036097 | 0.000284 |
| open_floor_fit_downweighted | 31165 | 9 | 0.043194 | 0.043855 | -1.529767 | 0.027974 | 0.013056 | 0.041892 | 0.000417 |
| open_floor_validation_only | 14542 | 9 | 0.016931 | 0.019213 | -13.482134 | 0.011335 | 0.005366 | 0.017939 | 0.000347 |
| diag_validation_only | 11108 | 3 | 0.084621 | 0.094909 | -12.157455 | 0.082544 | 0.089641 | 0.094589 | 0.005457 |
| aux_downweighted_validation | 14448 | 13 | 0.051266 | 0.054615 | -6.531088 | 0.035656 | 0.017932 | 0.060904 | 0.000638 |
| validation_non_authoritative | 71263 | 34 | 0.050234 | 0.054085 | -7.666616 | 0.034642 | 0.014618 | 0.052511 | 0.000656 |

## Selected-Log Metrics

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_signed_median_nm | corrected_signed_median_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.034391 | 4.058975 | 0.009360 | 0.007747 | 0.003600 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.027109 | -16.457142 | -0.000686 | -0.000079 | 0.004860 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.015796 | 2.594615 | -0.003116 | -0.001901 | 0.000217 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.044843 | 0.293458 | 0.009991 | 0.009381 | 0.000287 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.042075 | 1.195662 | 0.012155 | 0.012276 | 0.000267 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.040030 | -0.517149 | 0.008941 | 0.009072 | 0.001121 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.040798 | -1.098084 | 0.003675 | 0.003665 | 0.000376 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.057402 | -2.093436 | 0.000167 | 0.000204 | 0.000884 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.095773 | -12.359626 | -0.012440 | -0.014404 | 0.005621 |

## May 4 Latest Logs

The standard fit includes May 4 rows only through normal split weighting. In standard training weights, April rows contribute `0.984` of row-weight and the two May 4 latest logs contribute `0.016` of row-weight.

Latest-weighted sensitivity branch raises the two May 4 logs to at least weight `3.0`; it is reported separately and is not the selected standard fit.

| run_id | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.033881 | 5.481313 | 0.004890 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.026417 | -13.485373 | 0.004284 |

## Risk Metrics

| group | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | run_balanced_corrected_rmse_nm | median_support_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 49 | 0.025842 | 0.025687 | 0.597360 | 0.032500 | 0.000066 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 47 | 0.024100 | 0.023875 | 0.934659 | 0.027098 | 0.000040 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 49 | 0.073503 | 0.081635 | -11.064249 | 0.070509 | 0.006126 |
| low_speed_yaw_vf_lt_0p15_yaw_ge_0p5 | 20694 | 45 | 0.072832 | 0.079424 | -9.051880 | 0.077469 | 0.003216 |
| high_forward_vf_ge_0p5 | 3120 | 35 | 0.046415 | 0.046457 | -0.090686 | 0.059578 | 0.000096 |
| high_speed_abs_vf_ge_0p7 | 144 | 17 | 0.088793 | 0.088911 | -0.133154 | 0.089593 | 0.000146 |
| limiter_active | 30265 | 43 | 0.074736 | 0.078473 | -5.000993 | 0.081411 | 0.001413 |
| hardware_saturation_evidence | 5017 | 24 | 0.063673 | 0.064279 | -0.951904 | 0.061342 | 0.001066 |
| may4_latest_logs | 5217 | 2 | 0.032158 | 0.032118 | 0.123004 | 0.030965 | 0.004073 |
| april_rows | 87807 | 31 | 0.037157 | 0.037451 | -0.791499 | 0.034228 | 0.000291 |

## Reference Comparisons

| group | baseline_rmse_nm | unshifted_refined_rmse_nm | force_domain_stribeck_rmse_nm | rational_residual_reference_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.036556 | 0.027908 | 0.021083 | 0.019508 | 0.016688 |
| open_floor_fit_downweighted | 0.043194 | 0.043855 | 0.041507 | 0.032823 | 0.035267 | 0.027952 |
| open_floor_validation_only | 0.016931 | 0.019213 | 0.011394 | 0.011207 | 0.009294 | 0.007665 |
| diag_validation_only | 0.084621 | 0.094909 | 0.084410 | 0.038489 | 0.025685 | 0.020030 |
| aux_downweighted_validation | 0.051266 | 0.054615 | 0.048820 | 0.027490 | 0.027446 | 0.019198 |
| validation_non_authoritative | 0.050234 | 0.054085 |  | 0.029680 | 0.028585 | 0.022157 |

Selected-log comparison:

| run_id | dataset_split | baseline_rmse_nm | unshifted_refined_rmse_nm | force_domain_stribeck_rmse_nm | true_traction_testbed_rmse_nm | standalone_contact_traction_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.034391 | 0.020956 | 0.016473 | 0.011393 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.027109 | 0.015366 | 0.010900 | 0.009662 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.015796 | 0.013487 | 0.012118 | 0.012232 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.044843 | 0.033938 | 0.014672 | 0.011282 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.042075 | 0.027284 | 0.016287 | 0.015434 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.040030 | 0.034302 | 0.026919 | 0.020474 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.040798 | 0.035648 | 0.028266 | 0.022853 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.057402 | 0.056141 | 0.043763 | 0.036549 |
| diag003 | diag_validation_only | 0.085238 | 0.095773 | 0.085032 | 0.026235 | 0.019757 |

## Boundary Audit

| parameter | value | range_min | range_max | normalized_margin_to_nearest_boundary | boundary_adjacent | search_status |
| --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |  |
| k1 | 0.036374 | 0.015000 | 2.500000 | 0.173142 | False | interior_selected |
| delta_k | 1.215808 | 0.015000 | 5.000000 | 0.243414 | False | interior_selected |
| k3_radps | 1.176190 | 0.004000 | 2.000000 | 0.085422 | False | interior_selected |
| stribeck_speed_mps | 0.005205 | 0.004000 | 0.650000 | 0.051727 | False | interior_selected |
| speed_fade_mps | 0.845353 | 0.050000 | 5.000000 | 0.385966 | False | interior_selected |
| rel_weight | 0.183215 | 0.000000 | 3.000000 | 0.061072 | False | interior_selected |
| load_exponent | 1.301271 | 0.000000 | 1.500000 | 0.132486 | False | interior_selected |
| interpretation | reach_x2 | meet_x1;reach_x2 | meet_x1;reach_x2 | 1.000000 | False | interior_selected |

## Interpretation

- The selected standard candidate is interior to the final searched ranges and passes the `|cmd| >= 0.6` launch gate.
- The low-yaw slope is derived from the selected handoff interpretation; no independent linear slope is tuned.
- The best selected shape uses the raw, unshifted Stribeck branch, so peak support is the fitted `sliding_nm + static_extra_nm` at nominal load and is not inflated by a shifted transition.
- The launch target is satisfied without a dominating synthetic row, but this family still validates worse than the current force-domain Stribeck, rational residual reference, and standalone contact-traction testbed on most broad splits.

## Reproduce

```powershell
python codex_analysis\yaw_model_variant_fits\regime_linear_blend_stribeck_unshifted_refined\fit_unshifted_refined.py
```

## Output Files

- `fit_unshifted_refined.py`
- `unshifted_refined_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
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
- `search_summary.csv`
- `prediction_sample.csv`
- `metadata.json`
