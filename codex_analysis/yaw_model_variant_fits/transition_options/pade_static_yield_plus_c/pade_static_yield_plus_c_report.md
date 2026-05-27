# Pade Static-Yield Plus C Moving-Contact Yaw Model

Analysis-only output. Production code, build metadata, and tests were not edited.

## Model Form

The model predicts yaw-opposing residual torque and converts it back to additive yaw torque with `M_add = -sign(yaw) * M_opp`.

`M_opp = M_yield + M_move_surface`

`M_yield = A_state(v_yaw_contact, u_force) * R_fade(v_t) * R_force(u_force) * L(N) * M0 * [r_slide + (1-r_slide) * S(v_t)]`

where `A_state = 1-(1-v_y^2/(v_y^2+k_y^2))*(1-u^2/(u^2+k_u^2))`, `v_t = sqrt((rel_weight*v_rel)^2 + Vf^2)`, `S(v)=1/(1+x^2+b*x^4)`, `R_fade(v)=1/(1+(v/v_fade)^2)`, `R_force(u)=u/(u+u_k)`, and `L(N)=(N/N_nom)^load_exp`.

`M_move_surface` is a C-style ridge surface over per-contact velocity bases and actual projected-force bases. It is gated in by the rational moving schedule `(1-S(v_t))^2` and bounded by `M_cap * z/sqrt(1+z^2)`, where `z=M_raw/M_cap`. It has no command/request selectors, maneuver-label inputs, trig, tanh, or exp in the selected law.

Command/request values are not inputs to the traction/resistance prediction. They are used only downstream in the motor inverse that asks how much command would be needed to supply the predicted physical opposing torque. The selected moving branch excludes the request/gap features used by some prior diagnostic C-style surfaces.

The fit rejects candidates outside the hard gate before scoring RMSE: at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`, both left and right command magnitudes must be at least `0.6`. The reference condition is approximately `+0.646/-0.646`.

## New Mechanism Versus Prior Fits

The mechanism tested here is the hardware-cheap transition: a rational static/pre-sliding yield envelope hands off to a rational-gated moving-contact surface. The static branch is fitted with a hard lower bound from the +1 rad/s in-place command before the moving surface is considered.

The moving surface is only a residual corrector on `target - M_yield`; it is multiplied by `(1-S)^2`, so it is not allowed to be the source of the launch-scale breakaway torque. This keeps the launch command floor from the static branch while trying to recover C-like validation performance once the contact is sliding.

## Fitted Parameters

| parameter | value | unit |
| --- | --- | --- |
| yaw_activation_mps | 0.008000 | m/s |
| force_activation_util | 0.200000 | actual force utilization |
| static_speed_mps | 0.025000 | m/s |
| speed_fade_mps | 0.640000 | m/s |
| rel_weight | 0.750000 | dimensionless |
| util_k | 0.100000 | dimensionless utilization |
| slide_ratio | 0.000000 | sliding/static ratio |
| static_quartic | 0.200000 | Pade x^4 denominator coefficient |
| load_exp | 1.000000 | dimensionless |
| static_peak_nm | 0.103014 | Nm |
| sliding_yield_nm | 0.000000 | Nm |
| surface_rel_knee_mps | 0.060000 | m/s |
| surface_fwd_knee_mps | 0.700000 | m/s |
| surface_ridge | 0.001000 | ridge |
| surface_cap_nm | 0.040000 | Nm |
| nominal_load_n | 1.932931 | N |
| hard_gate_abs_command_min | 0.600000 | command |
| hard_gate_left_command_1radps | 0.632313 | command |
| hard_gate_right_command_1radps | -0.632313 | command |
| hard_gate_lr_delta_1radps | 1.264626 | command delta |
| hard_gate_extra_opposing_nm | 0.063233 | Nm |
| hard_gate_pass | 1.000000 | boolean |

## 1 rad/s In-Place Command

Pade hard-gate result: **PASS** (`|cmd| >= 0.6`; predicted left/right `0.632313/-0.632313`).

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | lr_delta_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- |
| Baseline | 0.000000 | 0.014794 | 0.261744 | -0.261744 | 0.523488 | 0.000000 |
| ForceDomainStribeck | 0.067115 | 0.081909 | 0.655064 | -0.655064 | 1.310129 | 1.000000 |
| C_combined_slip | 0.018241 | 0.033035 | 0.368641 | -0.368641 | 0.737282 | 0.000000 |
| Pade_static_yield_plus_C | 0.063233 | 0.078027 | 0.632313 | -0.632313 | 1.264626 | 1.000000 |

## Split RMSE Versus B/C

| split | baseline_rmse_nm | force_domain_stribeck_corrected_rmse_nm | c_combined_slip_corrected_rmse_nm | pade_static_yield_plus_c_corrected_rmse_nm |
| --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.027908 | 0.024120 | 0.025762 |
| open_floor_fit_downweighted | 0.043194 | 0.041507 | 0.033093 | 0.033743 |
| open_floor_validation_only | 0.016931 | 0.011394 | 0.014417 | 0.027572 |
| diag_validation_only | 0.084621 | 0.084410 | 0.038767 | 0.078097 |
| aux_downweighted_validation | 0.051266 | 0.048820 | 0.028529 | 0.057118 |
| validation_non_authoritative | 0.050234 | 0.048721 | 0.030342 | 0.047594 |

## Selected Log RMSE Versus B/C

| run_id | dataset_split | baseline_rmse_nm | force_domain_stribeck_corrected_rmse_nm | c_combined_slip_corrected_rmse_nm | pade_static_yield_plus_c_corrected_rmse_nm | pade_signed_median_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.020956 | 0.028076 | 0.041793 | -0.001203 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.015366 | 0.019341 | 0.043658 | -0.000014 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.013487 | 0.014403 | 0.021821 | -0.000014 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.033938 | 0.018608 | 0.027370 | 0.002169 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.027284 | 0.023754 | 0.026355 | 0.003239 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.034302 | 0.027743 | 0.027541 | 0.003824 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.035648 | 0.029338 | 0.029144 | 0.001705 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.056141 | 0.041312 | 0.037997 | 0.000211 |
| diag003 | diag_validation_only | 0.085238 | 0.085032 | 0.038970 | 0.078703 | -0.006401 |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | baseline_median_abs_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367.000000 | 0.025842 | 0.028838 | 0.006082 | 0.006239 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746.000000 | 0.024100 | 0.016356 | 0.005326 | 0.004580 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704.000000 | 0.073503 | 0.060902 | 0.054369 | 0.046984 |
| high_forward_vf_ge_0p5 | 3120.000000 | 0.046415 | 0.034483 | 0.012755 | 0.011635 |
| limiter_active | 31216.000000 | 0.074601 | 0.052912 | 0.053156 | 0.033618 |

## Dominant Moving-Surface Coefficients

| feature | standardized_coefficient_nm | feature_scale | raw_coefficient_nm_per_feature |
| --- | --- | --- | --- |
| force_moment_opposes_yaw_nm__moving | -0.055372 | 0.043591 | -1.270260 |
| force_moment_opposes_yaw_nm__util_moving | 0.014412 | 0.019083 | 0.755242 |
| gain_left_long_basis__moving | 0.001204 | 0.008962 | 0.134363 |
| gain_right_long_basis__moving | 0.001204 | 0.008962 | 0.134345 |
| force_moment_opposes_yaw_nm__high_forward | -0.000489 | 0.001299 | -0.376354 |
| force_abs_contact_moment_nm__limiter_signed_moving | -0.000287 | 0.000000 | -30272813.802636 |
| gain_rear_right_basis__moving | 0.000164 | 0.000497 | 0.329728 |
| gain_front_right_basis__moving | 0.000164 | 0.000497 | 0.329728 |

## Evaluation Notes

- Fit-authoritative rows: 47317; non-authoritative validation rows: 71263.
- Accepted hard-gate candidates: 4 of 4.
- Fit-authoritative corrected RMSE: 0.025762 Nm versus baseline 0.036866 Nm.
- Non-authoritative validation corrected RMSE: 0.047594 Nm versus baseline 0.050234 Nm.
- Verdict: this family passes the yaw-launch price of entry. The selected no-exp/no-tanh rational transition is still not production-ready as an overall replacement unless the split tables show it closes enough of Variant C's validation gap for the intended use.
- Cost estimate: selected runtime needs contact-speed square roots, rational divides for static fade, activation, force/load schedules and moving gate, plus one sqrt soft-cap. It uses no trig, atan, sin/cos, tanh, or exp.

## Failure Modes

- The static-yield branch is deliberately constrained by the +1 rad/s in-place gate; it can over-add resistance in logs or cells where the current plant already over-resists yaw.
- The model is memoryless. It does not represent pre-sliding displacement history, hysteresis, surface heating, or stick duration.
- The synthetic command grid replays an approximate contact projection, not the full production force limiter. High-utilization cells are flagged and should not be treated as exact command requirements.
- Near-zero yaw still depends on a continuous yaw/contact-force sign convention. A production implementation would need a continuity-preserving deadband or signed contact-yaw velocity input.
- Rare limiter-scheduled force features have near-zero scales in this dataset; even though the sqrt soft-cap bounds output, those coefficients are weak evidence and should not be promoted directly.
- Load scheduling is fitted mostly around the logged fan/load range; extrapolation to very different fan duty or normal load is weak evidence.

## Output Files

- `fit_pade_static_yield_plus_c.py`
- `pade_static_yield_plus_c_report.md`
- `pade_static_yield_plus_c_parameters.csv`
- `pade_static_yield_plus_c_surface_coefficients.csv`
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
