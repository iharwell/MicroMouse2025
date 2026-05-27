# Round 2 Static-Yield Contact Yaw Model

Analysis-only output. Production code, build metadata, and tests were not edited.

## Model Form

The model predicts yaw-opposing residual torque and converts it back to additive yaw torque with `M_add = -sign(yaw) * M_opp`.

`M_opp = M_yield + M_slide_surface`

`M_yield = A_state(v_yaw_contact, u_force) * R(v_t) * U(u_force) * L(N) * [M_slide + (M_static - M_slide) * exp(-(v_t / v_static)^2)]`

where `A_state` is the smooth union of contact-yaw-relative-velocity activation and actual projected-force-utilization activation, `v_t = sqrt((rel_weight * v_rel)^2 + Vf^2)`, `R(v_t)=1/(1+(v_t/v_fade)^2)`, `U(u)=0.5+0.5*u/(u+u_k)`, and `L(N)=(N/N_nom)^load_exp`.

`M_slide_surface` is a bounded signed ridge residual surface over per-contact velocity bases and actual projected-force bases: `M_slide_surface = M_cap * tanh(M_raw/M_cap)`. Its features are slide-gated and scheduled by contact-relative speed, forward speed, load delta, force utilization, and limiter activity. It has no maneuver-label inputs.

Command/request values are not inputs to the Round2 traction/resistance prediction. They are used only downstream in the motor inverse that asks how much command would be needed to supply the predicted physical opposing torque. The earlier request-gated draft is rejected by this report's rule because identical contact state and forces could have produced different resistance if command differed.

The fit rejects candidates outside the hard gate before scoring RMSE: at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`, both left and right command magnitudes must be at least `0.6`. The reference condition is approximately `+0.646/-0.646`.

## New Mechanism Versus A/C/D-Style Fits

The new mechanism is the explicit static-yield envelope `M_yield`, not the ridge surface. It has a high near-static yield level, a lower sliding level, a continuous velocity transition, contact-state activation, load scaling, and actual-force-utilization scheduling. This branch is fitted with a hard lower bound from the +1 rad/s in-place command before any residual surface is considered.

The low-order ridge portion is only a slide-gated residual corrector on `target - M_yield`. Its contact-velocity and actual-force features are multiplied by sliding schedules, so it is not allowed to be the source of the launch-scale breakaway torque. A/C/D-style low-order surfaces can move RMSE, but they do not contain this static-to-sliding yield envelope and therefore remain C-scale at yaw launch.

## Fitted Parameters

| parameter | value | unit |
| --- | --- | --- |
| yaw_activation_mps | 0.050000 | m/s |
| force_activation_util | 0.300000 | actual force utilization |
| static_speed_mps | 0.035000 | m/s |
| speed_fade_mps | 0.640000 | m/s |
| rel_weight | 0.750000 | dimensionless |
| util_k | 0.200000 | dimensionless utilization |
| slide_ratio | 0.200000 | sliding/static ratio |
| load_exp | 1.000000 | dimensionless |
| static_peak_nm | 0.099836 | Nm |
| sliding_yield_nm | 0.019967 | Nm |
| surface_rel_knee_mps | 0.060000 | m/s |
| surface_fwd_knee_mps | 0.700000 | m/s |
| surface_ridge | 0.001000 | ridge |
| surface_cap_nm | 0.040000 | Nm |
| nominal_load_n | 1.932931 | N |
| hard_gate_abs_command_min | 0.600000 | command |
| hard_gate_left_command_1radps | 0.642715 | command |
| hard_gate_right_command_1radps | -0.642715 | command |
| hard_gate_lr_delta_1radps | 1.285430 | command delta |
| hard_gate_extra_opposing_nm | 0.065007 | Nm |
| hard_gate_pass | 1.000000 | boolean |

## 1 rad/s In-Place Command

Round2 hard-gate result: **PASS** (`|cmd| >= 0.6`; predicted left/right `0.642715/-0.642715`).

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | lr_delta_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- |
| Baseline | 0.000000 | 0.014794 | 0.261744 | -0.261744 | 0.523488 | 0.000000 |
| B_stribeck | 0.065013 | 0.079808 | 0.642750 | -0.642750 | 1.285499 | 1.000000 |
| C_combined_slip | 0.018241 | 0.033035 | 0.368641 | -0.368641 | 0.737282 | 0.000000 |
| Round2_static_yield_contact | 0.065007 | 0.079802 | 0.642715 | -0.642715 | 1.285430 | 1.000000 |

## Split RMSE Versus B/C

| split | baseline_rmse_nm | b_stribeck_corrected_rmse_nm | c_combined_slip_corrected_rmse_nm | round2_static_yield_contact_corrected_rmse_nm |
| --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.028528 | 0.024120 | 0.028110 |
| open_floor_fit_downweighted | 0.043194 | 0.041880 | 0.033093 | 0.036855 |
| open_floor_validation_only | 0.016931 | 0.011372 | 0.014417 | 0.025701 |
| diag_validation_only | 0.084621 | 0.084412 | 0.038767 | 0.088532 |
| aux_downweighted_validation | 0.051266 | 0.048945 | 0.028529 | 0.061266 |
| validation_non_authoritative | 0.050234 |  | 0.030342 | 0.052072 |

## Selected Log RMSE Versus B/C

| run_id | dataset_split | baseline_rmse_nm | b_stribeck_corrected_rmse_nm | c_combined_slip_corrected_rmse_nm | round2_static_yield_contact_corrected_rmse_nm | round2_signed_median_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.021129 | 0.028076 | 0.045155 | -0.003586 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.015289 | 0.019341 | 0.039548 | -0.000433 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.013499 | 0.014403 | 0.022714 | -0.000540 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.034045 | 0.018608 | 0.027211 | 0.001651 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.027835 | 0.023754 | 0.031343 | 0.003866 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.035146 | 0.027743 | 0.028641 | 0.004420 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.036504 | 0.029338 | 0.030144 | 0.001389 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.056260 | 0.041312 | 0.042622 | 0.000314 |
| diag003 | diag_validation_only | 0.085238 | 0.085034 | 0.038970 | 0.089137 | -0.014927 |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | baseline_median_abs_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367.000000 | 0.025842 | 0.030690 | 0.006082 | 0.008238 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746.000000 | 0.024100 | 0.017648 | 0.005326 | 0.007087 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704.000000 | 0.073503 | 0.068860 | 0.054369 | 0.050661 |
| high_forward_vf_ge_0p5 | 3120.000000 | 0.046415 | 0.034894 | 0.012755 | 0.011818 |
| limiter_active | 31216.000000 | 0.074601 | 0.058433 | 0.053156 | 0.036459 |

## Dominant Sliding-Surface Coefficients

| feature | standardized_coefficient_nm | feature_scale | raw_coefficient_nm_per_feature |
| --- | --- | --- | --- |
| force_moment_opposes_yaw_nm__util_slide | -0.040924 | 0.019250 | -2.125901 |
| gain_front_right_basis__slide | -0.016542 | 0.000489 | -33.860393 |
| gain_rear_right_basis__slide | -0.016542 | 0.000489 | -33.860391 |
| gain_rear_right_basis__util_slide | 0.015391 | 0.000236 | 65.181358 |
| gain_front_right_basis__util_slide | 0.015391 | 0.000236 | 65.181354 |
| force_moment_opposes_yaw_nm__limiter_slide | 0.002683 | 0.000000 | 414103441.391178 |
| gain_left_long_basis__slide | 0.001153 | 0.009137 | 0.126241 |
| gain_right_long_basis__slide | 0.001153 | 0.009137 | 0.126220 |
| gain_front_right_basis__high_forward | -0.000308 | 0.000011 | -28.474216 |
| gain_rear_right_basis__high_forward | -0.000308 | 0.000011 | -28.474216 |
| force_abs_contact_moment_nm__limiter_signed_slide | -0.000262 | 0.000000 | -28376189.842309 |

## Evaluation Notes

- Fit-authoritative rows: 47317; non-authoritative validation rows: 71263.
- Accepted hard-gate candidates: 6 of 6.
- Fit-authoritative corrected RMSE: 0.028110 Nm versus baseline 0.036866 Nm.
- Non-authoritative validation corrected RMSE: 0.052072 Nm versus baseline 0.050234 Nm.
- Verdict: this family passes the yaw-launch price of entry, but this bounded fit is not production-ready as an overall replacement. It improves the fit-authoritative split and several motion-risk slices, but broad non-authoritative validation is slightly worse than baseline and materially worse than Variant C.
- The useful result is architectural: static breakaway must be an explicit contact-state yield envelope. The remaining problem is fitting the moving-contact branch without over-resisting validation-only, diag, and aux rows.

## Failure Modes

- The static-yield branch is deliberately constrained by the +1 rad/s in-place gate; it can over-add resistance in logs or cells where the current plant already over-resists yaw.
- The model is memoryless. It does not represent pre-sliding displacement history, hysteresis, surface heating, or stick duration.
- The synthetic command grid replays an approximate contact projection, not the full production force limiter. High-utilization cells are flagged and should not be treated as exact command requirements.
- Near-zero yaw still depends on a continuous yaw/contact-force sign convention. A production implementation would need a continuity-preserving deadband or signed contact-yaw velocity input.
- Rare limiter-scheduled force features have near-zero scales in this dataset; even though the tanh cap bounds output, those coefficients are weak evidence and should not be promoted directly.
- Load scheduling is fitted mostly around the logged fan/load range; extrapolation to very different fan duty or normal load is weak evidence.

## Output Files

- `fit_static_yield_contact.py`
- `static_yield_contact_report.md`
- `static_yield_parameters.csv`
- `static_yield_surface_coefficients.csv`
- `candidate_scores.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `split_comparison_vs_b_c.csv`
- `selected_log_comparison_vs_b_c.csv`
- `one_rad_in_place_command.csv`
- `lr_delta_grid.csv`
- `lr_delta_pivot.md`
- `prediction_sample.csv`
- `commands_run.txt`
