# Variant C Combined-Slip Surface Fit

Analysis-only output. Production code, build metadata, and tests were not edited.

## Reproduce

```powershell
python codex_analysis\yaw_model_variant_fits\combined_slip_surface\fit_combined_slip_surface.py
```

## Input Contract Alignment

- Primary input: `codex_analysis\contact_continuum_yaw_identification\ablation\phase_classified_feature_sample.csv`
- Secondary contact/load merge: `codex_analysis\contact_continuum_yaw_identification\features\contact_continuum_feature_sample.csv`
- Fit-authoritative training rows: 47317
- Non-authoritative validation rows: 71263
- All contract-selected logs were present in the primary feature input.

## Model Form

The selected form predicts yaw-aligned residual torque `residual_opposes_yaw_nm`, then converts it back to a raw additive yaw-moment correction with `predicted_raw = -sign(yaw_rate) * predicted_opposes`. The corrected residual is `residual_additive_yaw_torque_nm - predicted_raw`.

Per-contact bases are derived from contact-relative velocities and wheel coordinates: right-force gain terms use `-sign(yaw) * f_i * v_rel_r_i`; longitudinal gain terms use `sign(yaw) * r_i * v_rel_f_i`. The saturation-aware candidate also uses the yaw-opposing difference between requested and projected contact moment. Schedules are continuous scalar multipliers: `low_rel = 1/(1+(vbar_rel/k_rel)^2)`, `high_forward = 1 - 1/(1+(|Vf|/k_fwd)^2)`, smooth force utilization, smooth limiter activity, and total-load delta.

Selected model: `saturation_aware_surface` with `k_rel=0.060 m/s`, `k_fwd=0.700 m/s`, ridge `0.001`, nominal load `1.932931 N`.

## Tuning Summary

This is a narrow tune: the pass compares the three Variant C candidate surfaces at the common `k_rel=0.060 m/s`, `k_fwd=0.700 m/s` schedule point with ridge `0.001`. A wider pure-Python grid was too slow for the shared dataset in this workstream.

| Rank | Candidate | k_rel | k_fwd | Ridge | Objective | Validation corrected RMSE | Straight corrected RMSE |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | saturation_aware_surface | 0.060 | 0.700 | 0.001 | 0.028909 | 0.028909 | 0.019264 |
| 2 | front_rear_gain_surface | 0.060 | 0.700 | 0.001 | 0.030644 | 0.030644 | 0.019575 |
| 3 | compact_gain_surface | 0.060 | 0.700 | 0.001 | 0.030644 | 0.030644 | 0.019575 |

## Split Metrics

| Split | Count | Baseline RMSE | Corrected RMSE | Baseline MAE | Corrected MAE | Median abs before | Median abs after | RB RMSE change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.024120 | 0.025480 | 0.016696 | 0.016045 | 0.011182 | 35.3% |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.033093 | 0.027850 | 0.021126 | 0.013314 | 0.010734 | 24.9% |
| open_floor_validation_only | 14542 | 0.016931 | 0.014417 | 0.010369 | 0.009194 | 0.005201 | 0.004980 | 12.7% |
| diag_validation_only | 11108 | 0.084621 | 0.038767 | 0.073806 | 0.031027 | 0.082360 | 0.028442 | 54.2% |
| aux_downweighted_validation | 14448 | 0.051266 | 0.028529 | 0.033056 | 0.017920 | 0.016360 | 0.010515 | 43.2% |
| validation_non_authoritative | 71263 | 0.050234 | 0.030342 | 0.032502 | 0.019584 | 0.014291 | 0.010204 | 40.9% |

## Selected Log Metrics

| Run | Split | Count | Baseline RMSE | Corrected RMSE | Median signed before | Median signed after | Median abs before | Median abs after |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.028076 | 0.009360 | 0.007324 | 0.025485 | 0.019729 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.019341 | -0.000686 | -0.000457 | 0.013239 | 0.010955 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.014403 | -0.003116 | -0.003581 | 0.011327 | 0.010916 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.018608 | 0.009991 | 0.003156 | 0.026814 | 0.013477 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.023754 | 0.012155 | 0.001644 | 0.034705 | 0.016426 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.027743 | 0.008941 | -0.000540 | 0.019252 | 0.014036 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.029338 | 0.003675 | 0.000334 | 0.015359 | 0.012477 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.041312 | 0.000167 | -0.000841 | 0.036814 | 0.020966 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.038970 | -0.012440 | -0.003248 | 0.085086 | 0.028840 |

## Parent-Context Extra Sanity Logs

| Run | Split | Count | Baseline RMSE | Corrected RMSE | Median abs before | Median abs after |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 2026-04-16_02-43-36 | open_floor_validation_only | 376 | 0.008534 | 0.008815 | 0.005189 | 0.004392 |

## Straight-Line And High-Speed Risk

| Group | Count | Baseline RMSE | Corrected RMSE | Median abs before | Median abs after | RB RMSE change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025842 | 0.015948 | 0.006082 | 0.005327 | 41.1% |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.024100 | 0.012968 | 0.005326 | 0.004793 | 49.3% |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.073503 | 0.039972 | 0.054369 | 0.026326 | 42.5% |
| high_forward_vf_ge_0p5 | 3120 | 0.046415 | 0.030069 | 0.012755 | 0.013231 | 47.2% |
| limiter_active | 31216 | 0.074601 | 0.043163 | 0.053156 | 0.026372 | 44.9% |

## Dominant Coefficients

| Feature | Std coeff Nm | Feature scale | Raw coeff |
| --- | ---: | ---: | ---: |
| req_moment_opposes_yaw_nm__util | -0.013593 | 0.025626 | -0.530458 |
| gain_right_long_basis__low_rel | -0.005850 | 0.001720 | -3.400480 |
| gain_left_long_basis__low_rel | -0.005850 | 0.001720 | -3.400480 |
| gain_front_right_basis__base | 0.005280 | 0.000398 | 13.260170 |
| gain_rear_right_basis__base | 0.005280 | 0.000398 | 13.260170 |
| force_moment_opposes_yaw_nm__high_forward | -0.005244 | 0.001088 | -4.820222 |
| gain_rear_right_basis__util | -0.004192 | 0.000181 | -23.130770 |
| gain_front_right_basis__util | -0.004192 | 0.000181 | -23.130770 |
| gain_right_long_basis__base | -0.003483 | 0.004986 | -0.698421 |
| gain_left_long_basis__base | -0.003483 | 0.004986 | -0.698421 |
| gain_right_long_basis__util | 0.003303 | 0.002280 | 1.448825 |
| gain_left_long_basis__util | 0.003303 | 0.002280 | 1.448825 |

## Assessment

Fit-authoritative run-balanced RMSE changed by 35.3%; non-authoritative validation changed by 40.9%. Because the form is continuous, contact-primitive based, and zeroes out when yaw sign is zero, it is a plausible production-shape candidate. The validation result should still be treated as analysis evidence, not a production tune.

Straight-line risk is bounded mainly by the odd-in-yaw correction convention and by the zero-intercept fit. Near-zero yaw rows can still receive a correction from noisy yaw sign, so production use would need an explicit continuity-preserving deadband or direct dependence on signed contact yaw velocity rather than a raw gyro sign gate.

## Output Files

- `fit_combined_slip_surface.py`
- `variant_c_combined_slip_surface_report.md`
- `candidate_tuning_scores.csv`
- `model_coefficients.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `prediction_sample.csv`
- `extra_parent_log_metrics.csv`
- `commands_run.txt`

