# Shifted-Peak Linear-to-Stribeck Yaw Regime Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Candidate Law

Sign convention: `yawRate > 0` is clockwise. The law computes a nonnegative opposing yaw-support magnitude from `abs(yawRate)` and `abs(Vf)`; when evaluated as a residual correction its signed additive moment is `-sign(yawRate) * M_support`. Zero yaw uses a positive fallback sign only for metric bookkeeping.

Let `w = abs(yawRate)`, `v = abs(Vf)`, `a = v * k1`, `b = v * k2 + k3`, with `k1 < k2` and `b >= a`.

`T = clamp((w - a) / max(b - a, eps), 0, 1)`

`M_linear = G_linear * w`

For the Stribeck branch, `s = max(w - b, 0)`, `q = sqrt(v^2 + (L_yaw * s)^2)`, and `q0 = v`.

`S_raw(q) = R_slide + (1 - R_slide) / (1 + (q / V_stribeck)^2)`

`S_shifted = max(0, 1 + S_raw(q) - S_raw(q0))`

The subtraction by `S_raw(q0)` and addition of `1` is the shifted-peak part: at the regime handoff (`s = 0`) the branch is exactly at the selected peak, even when forward speed would otherwise reduce the raw Stribeck value.

`Tail = 1 / (1 + ((L_yaw * s) / V_fade)^2)`

`Load = (N_total / N_nominal)^load_power`

`M_support = Load * ((1 - T) * M_linear + T * M_peak * S_shifted * Tail)`

The selected form uses `abs`, `sqrt`, clamp, rational schedules, and piecewise-linear blending only. It does not use command/request/preprojection values, UKF state-vector fields, or old contact-force outputs as selectors.

## Selected Constants

Standard fit used primary authoritative rows plus downweighted open-floor fit rows at the shared weights. It included May 4 rows only through their existing split weights; it is still dominated by the larger April authoritative set.

| parameter | value |
| --- | --- |
| k1_rad_per_m | 0.200000000 |
| k2_rad_per_m | 1.600000000 |
| k3_radps | 0.400000000 |
| linear_gain_nm_per_radps | 0.024681641 |
| peak_yield_nm | 0.010321978 |
| stribeck_speed_mps | 0.160000000 |
| yaw_fade_mps | 0.050000000 |
| sliding_ratio | 0.000000000 |
| load_power | 1.000000000 |

## +1 rad/s In-Place Command Estimate

| variant | opposing support Nm | left cmd | right cmd | max abs cmd |
| --- | --- | --- | --- | --- |
| standard_shifted_peak | 0.009977897 | 0.233518127 | -0.233518127 | 0.233518127 |

The hard launch gate passes: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s` estimates at least `|cmd| >= 0.6` and is near the `+0.646/-0.646` reference.

## Split Metrics

| split | count | baseline RMSE | shifted-peak RMSE | force-domain Stribeck ref | rational residual ref | standalone contact ref |
| --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.036315 | 0.027908 | 0.021083 | 0.016688 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.044790 | 0.041507 | 0.032823 | 0.027952 |
| open_floor_validation_only | 14542 | 0.016931 | 0.017200 | 0.011394 | 0.011207 | 0.007665 |
| diag_validation_only | 11108 | 0.084621 | 0.090127 | 0.084410 | 0.038489 | 0.020030 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.053977 | 0.048820 | 0.027490 | 0.019198 |
| validation_non_authoritative | 71263 | 0.050234 | 0.052863 |  | 0.029680 | 0.022157 |

## Selected-Log Metrics

| run | split | count | baseline RMSE | shifted-peak RMSE | force-domain ref | rational residual ref | standalone contact ref |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.033926 | 0.020956 | 0.020128 | 0.011393 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.022794 | 0.015366 | 0.015839 | 0.009662 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.015746 | 0.013487 | 0.013203 | 0.012232 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.043017 | 0.033938 | 0.016861 | 0.011282 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.041567 | 0.027284 | 0.017657 | 0.015434 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.040300 | 0.034302 | 0.026878 | 0.020474 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.040805 | 0.035648 | 0.027977 | 0.022853 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.059000 | 0.056141 | 0.042487 | 0.036549 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.090816 | 0.085032 | 0.038712 | 0.019757 |

The latest logs are included separately above: `2026-05-04_20-35-47` is fit-downweighted and `2026-05-04_16-57-53` is validation-only in the shared split contract.

## Risk Slices

| slice | count | baseline RMSE | shifted-peak RMSE | improvement % |
| --- | --- | --- | --- | --- |
| high_speed_abs_vf_ge_0p7 | 144 | 0.088793 | 0.091995 | -3.606700 |
| low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5 | 20694 | 0.072832 | 0.075621 | -3.829750 |
| limiter_active | 31216 | 0.074601 | 0.076983 | -3.192784 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.065698 | -3.179953 |
| open_floor_all | 93024 | 0.036894 | 0.037271 | -1.021600 |
| diag_all | 11108 | 0.084621 | 0.090127 | -6.506419 |
| aux_all | 14448 | 0.051266 | 0.053977 | -5.286873 |

## Latest-Weighted Diagnostic

This diagnostic multiplies positive training weights for `2026-05-04_20-35-47` and `2026-05-04_16-57-53` by `4` before run balancing. It is not the standard selected fit.

| parameter | latest-weighted value |
| --- | --- |
| k1_rad_per_m | 0.200000000 |
| k2_rad_per_m | 1.600000000 |
| k3_radps | 0.400000000 |
| linear_gain_nm_per_radps | 0.024257431 |
| peak_yield_nm | 0.011830131 |
| stribeck_speed_mps | 0.160000000 |
| yaw_fade_mps | 0.050000000 |
| sliding_ratio | 0.000000000 |
| load_power | 1.000000000 |

| variant | opposing support Nm | left cmd | right cmd | max abs cmd |
| --- | --- | --- | --- | --- |
| latest_weighted_shifted_peak | 0.011435776 | 0.242061911 | -0.242061911 | 0.242061911 |

| split | count | baseline RMSE | latest-weighted RMSE |
| --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.036290 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.045016 |
| open_floor_validation_only | 14542 | 0.016931 | 0.017296 |
| diag_validation_only | 11108 | 0.084621 | 0.090852 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.054308 |
| validation_non_authoritative | 71263 | 0.050234 | 0.053215 |

## Caveats

- This is a yaw-support law evaluated against residual yaw torque; it is not a full contact-patch replacement because it does not consume drive force or per-patch force state.
- The standard fit is constrained by the hard launch command target, so its `M_peak` is not selected by RMSE alone.
- The model is symmetric in `abs(yawRate)` and `abs(Vf)`. The available data did not justify signed branch behavior.
- Compared with the standalone contact-traction testbed, this law is much cheaper and simpler but less complete because it cannot model driven yaw moment directly.
- The shifted branch preserves the peak at the handoff by construction; if production wants a forward-speed-reduced breakaway peak, this specific variant is the wrong policy.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\regime_linear_to_stribeck_shifted_peak\fit_shifted_peak.py
```

## Outputs

- `fit_shifted_peak.py`
- `shifted_peak_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
- `latest_weighted_parameters.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid_6x10.csv`
- `prediction_sample.csv`
- matching `_latest_weighted` metric/grid/sample CSVs
- `metadata.json`
