# Cubic/Quintic Smoothstep Launch-to-C Partition

Analysis-only output. Production code, build metadata, and tests were not edited.

## Recommendation

Use the `partition_low_ref_same_window` shape with `cubic` speed smoothstep as the best candidate from this pass. It enforces the `|cmd| >= 0.6` in-place gate, fades the launch authority to zero at high transition speed, and avoids command/request traction inputs in the selected equation.

The selected moving branch is a compact force-only Variant-C-style fit over projected contact forces and contact-relative velocities. The published Variant C artifact is retained as a comparison reference, but it is not the selected runtime shape here because its strongest prior coefficients include request-derived terms.

## Selected Equations

`v_t = sqrt((rel_weight * vbar_rel)^2 + |Vf|^2)`

`t = clamp((v_t - v0) / (v1 - v0), 0, 1)`

Cubic: `h = t*t*(3 - 2*t)`; quintic option: `h = t^3*(10 - 15*t + 6*t^2)`.

`u = smooth_positive(M_projected_force_opposes_yaw) / M_yield`

`a = smoothstep(clamp(u / util_k, 0, 1))`

`M_launch = K_launch * a`

`M_pred = M_C_force + (1 - h) * (M_launch - (1 - h) * M_C_force)`

That partition subtracts the low-speed reference portion of the C branch instead of adding launch on top of all of C. At `h=0`, it reduces to launch authority; at `h=1`, it is exactly the moving-contact C branch.

## Selected Parameters

| parameter | value | unit |
| --- | --- | --- |
| form | partition_low_ref_same_window | enum |
| smoothstep | cubic | enum |
| transition_variable | speed_hypot | enum |
| v0_mps | 0.0 | m/s |
| v1_mps | 0.08 | m/s |
| rel_weight | 1.0 | dimensionless |
| util_k | 0.55 | yield fraction |
| launch_activation | cubic | enum |
| k_launch_nm | 0.05771872812904492 | Nm |
| k_unconstrained_nm | 0.05736224124965836 | Nm |
| k_gate_min_nm | 0.05771872812904492 | Nm |
| nominal_longitudinal_yield_nm | 0.11096910450167499 | Nm |

## Split Metrics

| dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | compact_c_force_only_corrected_rmse_nm | force_domain_stribeck_rmse_nm | published_variant_c_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | 14448 | 0.05126637392308135 | 0.029397120490599445 | 0.025788824957328874 | 0.048819706958314515 | 0.028529439179585198 |
| diag_validation_only | 11108 | 0.0846209251204878 | 0.029430103867343255 | 0.020043858307783477 | 0.08441017836709906 | 0.03876675590868194 |
| open_floor_fit_downweighted | 31165 | 0.043194073119807955 | 0.03309252577361895 | 0.032807615543679156 | 0.04150718685740355 | 0.033092937468878494 |
| open_floor_validation_only | 14542 | 0.01693070910006944 | 0.010491392588553741 | 0.009500952570880862 | 0.01139376781450519 | 0.014417143137851248 |
| primary_open_floor_fit_authoritative | 47317 | 0.036866176895009706 | 0.018340962145958675 | 0.01815388807456425 | 0.027907821180421354 | 0.024120243843366647 |
| validation_non_authoritative | 71263 | 0.05023383204822011 | 0.028488509193164188 | 0.02620283196090874 | nan | 0.030342 |

## In-Place Gate

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | lr_delta_command | max_abs_command | passes_abs_0p6_gate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| selected_smoothstep_partition | 0.05771872812904492 | 0.07251297812904492 | 0.6 | -0.6 | 1.2 | 0.6 | True |

## Risk Slices

| group | count | baseline_rmse_nm | corrected_rmse_nm | compact_c_force_only_corrected_rmse_nm |
| --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025841853669827997 | 0.0139787688086555 | 0.012084542929370473 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.024100230475114524 | 0.011579091875410735 | 0.011577239692369172 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.07350293824740096 | 0.03461490963209472 | 0.03234038847930374 |
| high_transition_speed_ge_0p5 | 7626 | 0.07600463534157978 | 0.0454022990381 | 0.0454022990381 |
| limiter_active | 31216 | 0.07460143692454368 | 0.04010136172864783 | 0.03967994766351839 |

## Top Candidate Families

| role | form | smoothstep | transition_variable | v0_mps | v1_mps | util_k | k_launch_nm | primary_rmse_nm | validation_non_authoritative_rmse_nm | high_transition_rmse_nm | score |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| selected | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05771872812904492 | 0.018340962145958675 | 0.028488509193164188 | 0.0454022990381 | 0.043071257644571787 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05771872812904492 | 0.018340962145958675 | 0.028488509193164188 | 0.0454022990381 | 0.043071257644571787 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05771872812904492 | 0.0183647431676304 | 0.028521435462233548 | 0.0454022990381 | 0.043111318220142664 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.4 | 0.05771872812904492 | 0.018357089748353244 | 0.028555293742532826 | 0.0454022990381 | 0.04314288047465879 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.4 | 0.05771872812904492 | 0.01838247279136435 | 0.028583686378062207 | 0.0454022990381 | 0.04317888802309151 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.3 | 0.05771872812904492 | 0.01848067318851556 | 0.028694458066597567 | 0.0454022990381 | 0.04331911983077223 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.3 | 0.05771872812904492 | 0.018509561197465323 | 0.02872133367281084 | 0.0454022990381 | 0.04335466183967043 |
| non_additive_grid_top | partition_low_ref_same_window | quintic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05824566039323684 | 0.018448842906385512 | 0.028761085289473437 | 0.0454022990381 | 0.043376197969009087 |
| non_additive_grid_top | partition_low_ref_same_window | quintic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05771872812904492 | 0.018475592656470804 | 0.028793782470401912 | 0.0454022990381 | 0.04341692007496316 |
| non_additive_grid_top | partition_low_ref_same_window | quintic | speed_hypot | 0.0 | 0.08 | 0.4 | 0.05771872812904492 | 0.018473151368355437 | 0.028825891872572704 | 0.0454022990381 | 0.043448297090699334 |
| non_additive_grid_top | partition_low_ref_same_window | quintic | speed_hypot | 0.0 | 0.08 | 0.4 | 0.05771872812904492 | 0.018500744491219426 | 0.028854757390105124 | 0.0454022990381 | 0.04348544054509096 |
| non_additive_grid_top | partition_low_ref_same_window | cubic | speed_hypot | 0.0 | 0.08 | 0.55 | 0.05797600643129943 | 0.01834658502809608 | 0.02897616052081393 | 0.0454022990381 | 0.04356059583686275 |

## Cost

Selected prediction cost after `M_C_force` terms are available:

- comparisons/clamps: 4 to 6 (`smooth_positive` sign-free positive part, utilization clamp, transition clamp, optional force/yaw zero guards).
- multiplies: about 17 for cubic speed partition plus cubic utilization activation and partition composition; quintic adds about 4 multiplies.
- divisions: 2 (`u / util_k`, `(v_t - v0)/(v1-v0)`) if reciprocals are not precomputed; 0 runtime divisions for these if `1/util_k` and `1/(v1-v0)` are constants.
- sqrt calls: 2 (`smooth_positive` and transition `sqrt`). If positive-part is replaced by a cheap branch `max(0, x)`, this drops to 1 sqrt.
- trig/exp/tanh: 0 in the selected form.

The compact moving branch adds 10 coefficients. The launch partition adds four scalar parameters (`v0`, `v1`, `util_k`, `K_launch`) plus the existing `rel_weight` choice.

## Notes

- `additive_no_subtract` candidates were kept in the grid to expose double-counting risk; their low-speed behavior adds launch on top of C and was not selected.
- The `launch_fade_grid.csv` command grid reports the launch fade contribution only; full high-speed command behavior must come from the moving-contact branch replay, not from the static launch envelope.
- The command estimator uses the prior analysis motor inverse for comparability. That inverse contains the existing motor static-friction exponential, but the selected yaw correction equation itself has no exp/tanh/trig.

## Output Files

- `fit_cubic_smoothstep_partition.py`
- `cubic_smoothstep_partition_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
- `moving_contact_coefficients.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `in_place_1radps_command.csv`
- `launch_fade_grid.csv`
- `prediction_sample.csv`
- `metadata.json`
- `commands_run.txt`
