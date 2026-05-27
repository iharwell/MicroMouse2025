# Round2 State-Minimal LuGre Yaw Residual Fit

Analysis-only output. Production code, build metadata, and tests were not edited.

## Model Family

The dynamic candidate uses one scalar bristle-fill state `q` per yaw/contact aggregate:

`v_c = sqrt((w_rel * vbar_rel)^2 + |Vf|^2)`

`A = 1 - exp(-(positive(M_req) / M_act)^2)`

`q_eq = A / (1 + tau_fill * v_c / x_slip)`

`dq/dt = (A - q) / tau_fill - (v_c / x_slip) * q`

`R = 1 / (1 + (v_c / v_fade)^2)`

`S = exp(-(v_c / v_s)^2)`

`M_extra = q * R * (K_static * S + K_slide + K_visc * vbar_yaw)`

The additive yaw-torque correction applied to the residual sign convention is `M_add = -sign(yaw) * M_extra`.

The memoryless approximation replaces the replayed state with the algebraic equilibrium `q = q_eq`.

## New Mechanism

This is not another low-order A/C/D-style residual surface with a yaw-launch constraint. The new mechanism is the single bristle-fill state `q`: it is integrated through time, fills under requested yaw moment, is depleted by moving contact slip, and resets on yaw direction reversal. Two rows with identical instantaneous `Vf`, yaw rate, and contact features can therefore predict different breakaway resistance if their recent bristle history differs.

The fitted coefficients only scale the static/Stribeck, sliding, and viscous terms after that state evolution. A ridge/surface fit has no equivalent stored pre-sliding deflection and cannot represent launch hysteresis or direction-reversal breakaway without adding this state or reducing back to the algebraic approximation.

## Command-Invariance Reassessment

New hard rule: traction/resistance must not differ for the same physical contact state and tire/contact forces merely because command values differ. Command may determine actuator torque input, but it must not be an independent traction-model selector or hidden state machine.

Under that rule, the fitted activation `A = 1 - exp(-(positive(M_req) / M_act)^2)` is production-rejected when `M_req` is a requested/pre-projection command-derived yaw moment rather than an actual physical contact-force state. The hard-gated fit remains useful as a diagnostic magnitude target, but the fitted `q` fill law and the memoryless `q_eq` approximation are not production-eligible in their current request-driven form.

Production revision: drive bristle state only from contact slip and actual tire/contact force history. A per-contact form is preferable:

`dot(z_i) = v_t_i - (|v_t_i| / g_i(|v_t_i|, N_i)) * z_i`

`g_i = (F_c_i + (F_s_i - F_c_i) * exp(-( |v_t_i| / v_s )^2)) / sigma0_i`

`F_t_i = clamp(sigma0_i * z_i + sigma1_i * dot(z_i) + sigma2_i * v_t_i, -mu_i * N_i, +mu_i * N_i)`

`M_yaw = sum_i cross(r_i, F_t_i)`

In that revision, command affects the motor torque and therefore the solved physical contact forces, but the friction law sees only `v_t_i`, `N_i`, `z_i`, and actual tangential contact force/load state. For a memoryless fallback, use the steady sliding/Stribeck force from `v_t_i` and `N_i`, or a static-capacity projection based on actual tangential force demand, never on requested command labels or pre-projection command moments.

## Hard Gate

Reference condition: `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`. Acceptance threshold: `|left_command| >= 0.6` and `|right_command| >= 0.6`; measured/calculated reference is approximately `+0.646/-0.646`.

Prior B extra opposing torque at +1 rad/s in-place is `0.065013 Nm` by construction. The state model command is left `0.642750`, right `-0.642750`. The memoryless command is left `0.642750`, right `-0.642750`.

Gate result: state model `PASS`; memoryless approximation `PASS`.

Production eligibility result under the command-invariance rule: `FAIL` for the fitted request-driven state and memoryless forms; `PASS` only as a diagnostic magnitude check. A production candidate must be refit/replayed with the force/slip-driven law above.

## Coefficients

| model | parameter | value | unit |
| --- | --- | --- | --- |
| state_minimal_lugre | req_activation_nm | 0.035 | Nm |
| state_minimal_lugre | stribeck_speed_mps | 0.1 | m/s |
| state_minimal_lugre | speed_fade_mps | 0.64 | m/s |
| state_minimal_lugre | rel_weight | 0.75 | dimensionless |
| state_minimal_lugre | tau_fill_s | 0.005 | s |
| state_minimal_lugre | bristle_slip_distance_m | 0.08 | m |
| state_minimal_lugre | bristle_stribeck_static_nm | 0.001912 | Nm |
| state_minimal_lugre | bristle_sliding_nm | 0.06355 | Nm |
| state_minimal_lugre | bristle_viscous_nm_per_mps | 0 | Nm per (m/s) |
| state_minimal_lugre | fixedpoint_extra_at_1radps_in_place_nm | 0.065013 | Nm |
| state_minimal_lugre | beta_scale_to_gate | 0.994382 | dimensionless |
| state_minimal_lugre | weighted_train_rmse_nm | 0.024836 | Nm |
| state_minimal_lugre | primary_rmse_nm | 0.029042 | Nm |
| state_minimal_lugre | validation_rmse_nm | 0.049128 | Nm |
| memoryless_equilibrium | req_activation_nm | 0.035 | Nm |
| memoryless_equilibrium | stribeck_speed_mps | 0.1 | m/s |
| memoryless_equilibrium | speed_fade_mps | 0.64 | m/s |
| memoryless_equilibrium | rel_weight | 0.75 | dimensionless |
| memoryless_equilibrium | tau_fill_s | 0.005 | s |
| memoryless_equilibrium | bristle_slip_distance_m | 0.08 | m |
| memoryless_equilibrium | bristle_stribeck_static_nm | 0.001595 | Nm |
| memoryless_equilibrium | bristle_sliding_nm | 0.063863 | Nm |
| memoryless_equilibrium | bristle_viscous_nm_per_mps | 0 | Nm per (m/s) |
| memoryless_equilibrium | fixedpoint_extra_at_1radps_in_place_nm | 0.065013 | Nm |
| memoryless_equilibrium | beta_scale_to_gate | 1.000151 | dimensionless |
| memoryless_equilibrium | weighted_train_rmse_nm | 0.024396 | Nm |
| memoryless_equilibrium | primary_rmse_nm | 0.028548 | Nm |
| memoryless_equilibrium | validation_rmse_nm | 0.048891 | Nm |

## +1 Rad/s In-Place Command

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | required_applied_bank_torque_nm | left_command | right_command | lr_delta_command | passes_abs_command_gate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Current baseline | 0 | 0.014794 | 0.002204 | 0.261744 | -0.261744 | 0.523488 | 0 |
| Variant B Stribeck scrub | 0.065013 | 0.079808 | 0.011891 | 0.64275 | -0.64275 | 1.285499 | 1 |
| Variant C combined slip | 0.018241 | 0.033035 | 0.004922 | 0.368641 | -0.368641 | 0.737282 | 0 |
| LuGre_state_steady | 0.065013 | 0.079808 | 0.011891 | 0.64275 | -0.64275 | 1.285499 | 1 |
| LuGre_state_cold20ms | 0.063764 | 0.078558 | 0.011705 | 0.635427 | -0.635427 | 1.270853 | 1 |
| LuGre_memoryless_eq | 0.065013 | 0.079808 | 0.011891 | 0.64275 | -0.64275 | 1.285499 | 1 |

## Split RMSE Versus B/C

| group | baseline_rmse_nm | b_stribeck_rmse_nm | c_combined_slip_rmse_nm | lugre_state_rmse_nm | lugre_memoryless_rmse_nm | lugre_state_delta_vs_b_pct | lugre_state_delta_vs_c_pct |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.028528 | 0.02412 | 0.029042 | 0.028548 | 1.803305 | 20.406524 |
| open_floor_fit_downweighted | 0.043194 | 0.04188 | 0.033093 | 0.042436 | 0.04189 | 1.327855 | 28.233409 |
| open_floor_validation_only | 0.016931 | 0.011372 | 0.014417 | 0.011694 | 0.011372 | 2.832638 | -18.888311 |
| diag_validation_only | 0.084621 | 0.084412 | 0.038767 | 0.084435 | 0.084412 | 0.02742 | 117.802963 |
| aux_downweighted_validation | 0.051266 | 0.048945 | 0.028529 | 0.049003 | 0.048955 | 0.117558 | 71.761454 |
| validation_non_authoritative | 0.050234 | 0.048885 | 0.030342 | 0.049128 | 0.048891 | 0.49617 | 61.915394 |

## Selected-Log RMSE Versus B/C

| run_id | dataset_split | baseline_rmse_nm | b_stribeck_rmse_nm | c_combined_slip_rmse_nm | lugre_state_rmse_nm | lugre_memoryless_rmse_nm | lugre_state_delta_vs_b_pct | lugre_state_delta_vs_c_pct |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.021129 | 0.028076 | 0.02243 | 0.021128 | 6.157245 | -20.112679 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.015289 | 0.019341 | 0.016185 | 0.015289 | 5.858991 | -16.319138 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.013499 | 0.014403 | 0.013739 | 0.013498 | 1.778508 | -4.611472 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.034045 | 0.018608 | 0.034302 | 0.034121 | 0.756622 | 84.336181 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.027835 | 0.023754 | 0.028405 | 0.02784 | 2.048045 | 19.578204 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.035146 | 0.027743 | 0.035598 | 0.035161 | 1.286419 | 28.313555 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.036504 | 0.029338 | 0.036891 | 0.036546 | 1.061663 | 25.744693 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.05626 | 0.041312 | 0.057116 | 0.056258 | 1.52272 | 38.254897 |
| diag003 | diag_validation_only | 0.085238 | 0.085034 | 0.03897 | 0.085058 | 0.085034 | 0.028038 | 118.262273 |

## 6x10 Vf/Yaw Grid Summary

The full grid is in `lr_delta_grid.csv`; the pivot view is in `lr_delta_pivot.md`. Summary:

| variant | cells | min_lr_delta_command | max_lr_delta_command | cells_abs_cmd_gt_1 | cells_contact_util_gt_1 |
| --- | --- | --- | --- | --- | --- |
| B_stribeck | 60 | 0.044086 | 2.190977 | 12 | 36 |
| Baseline | 60 | 0.036749 | 1.442207 | 0 | 36 |
| C_combined_slip | 60 | 0.081505 | 2.816966 | 13 | 36 |
| LuGre_memoryless_eq | 60 | 0.044114 | 2.191569 | 12 | 36 |
| LuGre_state_cold20ms | 60 | 0.043869 | 2.176832 | 12 | 36 |
| LuGre_state_steady | 60 | 0.044064 | 2.190309 | 12 | 36 |

## Risk Checks

| group | count | baseline_rmse_nm | lugre_state_rmse_nm | lugre_memoryless_rmse_nm | baseline_median_abs_nm | lugre_state_median_abs_nm | lugre_memoryless_median_abs_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025842 | 0.021457 | 0.021118 | 0.006082 | 0.005437 | 0.00537 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.0241 | 0.020894 | 0.020683 | 0.005326 | 0.005135 | 0.005126 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.073503 | 0.070705 | 0.070257 | 0.054369 | 0.043409 | 0.042738 |
| high_forward_vf_ge_0p5 | 3120 | 0.046415 | 0.043997 | 0.043858 | 0.012755 | 0.012484 | 0.012447 |
| limiter_active | 31216 | 0.074601 | 0.071747 | 0.071323 | 0.053156 | 0.047697 | 0.047326 |

## Production Implications

- The request-driven fitted `q` law is rejected for production because it can change traction/resistance for identical physical contact state and tire/contact forces when only command/request changes.
- A true production internal state still belongs in `PlantModel`, but it must be driven by contact slip, normal load, and actual tangential force history. If estimator prediction uses it, the estimator needs the same state or a deterministic mirror advanced with the same physical inputs.
- The state must be reset or strongly decayed on yaw direction reversal, physical lift/discontinuous vehicle state, and any `LoopController` session boundary where continuity is not guaranteed.
- The memoryless `q_eq` approximation from this fit is also production-rejected because its fill term is request-driven. A production memoryless fallback may use only steady sliding/Stribeck force from contact slip/load or a static-capacity projection based on actual tangential contact force demand.
- The B-scale `+0.6427/-0.6427` command remains the minimum magnitude target for any force/slip-driven refit; broad RMSE must not be used to accept a lower-command C-like model.

## Output Files

- `fit_state_minimal_lugre.py`
- `round2_state_minimal_lugre_report.md`
- `candidate_tuning_scores.csv`
- `lugre_coefficients.csv`
- `lugre_split_metrics.csv`
- `lugre_selected_log_metrics.csv`
- `split_rmse_comparison.csv`
- `selected_log_rmse_comparison.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid.csv`
- `lr_delta_pivot.md`
- `risk_metrics.csv`
- `production_reassessment.csv`
- `prediction_sample.csv`
- `commands_run.txt`

## Constants Read

- track width: 0.084635 m
- drive wheel longitudinal offset: 0.01475 m
- wheel radius: 0.01261 m
