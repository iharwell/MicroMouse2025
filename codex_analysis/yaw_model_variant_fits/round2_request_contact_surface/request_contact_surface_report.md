# Round-2 Force-Coupled Contact-Velocity Surface

Analysis-only output. Production code, build metadata, and tests were not modified.

## Model Family

The selected family is not another low-order residual ridge over A/C/D-style features. The new mechanism is a two-branch yaw-friction law:

1. a force-activated presliding/static reserve that is live before yaw motion develops, and
2. a moving-contact branch that projects bounded per-contact slip tractions into yaw moment.

The moving-contact branch uses each contact's normal load and relative velocity through `N_i * tanh(v_rel_i / v_k)`, so it behaves like a saturated contact-friction/creep mechanism rather than an unbounded polynomial or residual table. The fit tunes branch gains and schedule knees; it does not introduce maneuver labels or a grid lookup.

The model equations are:

`R_total = R_launch + DeltaR_surface`

`R_launch = A_force * F_speed * (K_slide + K_static * exp(-(v_transition / v_s)^2))`

`A_force = 1 - exp(-(abs(M_applied_contact_force) / M_force_knee)^2)`

`v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)`

`F_speed = 1 / (1 + (v_transition / v_fade)^2)`

`F_contact_i = -N_i * [tanh(v_rel_f_i / v_f_k), tanh(v_rel_r_i / v_r_k)]`

`M_contact_opp = -sign(yaw) * sum_i (f_i * F_contact_r_i - r_i * F_contact_f_i)`

`DeltaR_surface = G_move * (beta_c * M_contact_opp + beta_f * abs(M_applied_contact_force) + beta_gap * abs(M_force_gap) + beta_v * vbar_rel + ...)`

`G_move = A_force * (1 - low_rel * low_forward)`

The fitted terms are branch gains on contact-creep, applied contact-force moment, force-gap, yaw-rate, forward-speed, and utilization components. Command values are not features and do not select a traction mode; if two rows have the same contact state, relative velocities, loads, and contact-force inputs, the model returns the same resistance.

Rejected design note: a command/request-gated surface would violate the current rule because identical contact state and tire/contact forces could produce different resistance solely from command metadata. This report excludes that design; motor command is used only after prediction to estimate whether the +1 rad/s command gate is satisfied.

The additive yaw-torque correction applied to the residual convention is `M_corr = -sign(yaw_rate_or_applied_force) * R_total`.

## Fit Basis

- Rows evaluated: 118580
- Training rows with nonzero fit weight: 78482
- Primary fit rows carry full weight; open-floor `fit_downweighted` rows carry 0.25 base weight so May 4 yaw-launch evidence remains visible.
- No maneuver labels are used as features.

## Selected Hyperparameters

- `vrel_knee_mps`: 0.080000
- `fwd_knee_mps`: 0.350000
- `ridge`: 0.01
- fixed launch `K_static`: 0.002391255 Nm
- fixed launch `K_slide`: 0.063031582 Nm

## +1 rad/s In-Place Command

Hard gate result for positive clockwise +1 rad/s, zero forward speed:

| Variant | Extra opposing Nm | Total opposing Nm | Left cmd | Right cmd |
| --- | ---: | ---: | ---: | ---: |
| Current baseline | 0.000000 | 0.014794 | 0.262 | -0.262 |
| Variant B Stribeck scrub | 0.065013 | 0.079808 | 0.643 | -0.643 |
| Variant C combined slip | 0.018241 | 0.033035 | 0.369 | -0.369 |
| Round2_force_contact_surface | 0.061998 | 0.076792 | 0.625 | -0.625 |

Acceptance gate: `min(abs(left_cmd), abs(right_cmd)) >= 0.600` at `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`.
Round-2 maps +1 rad/s in-place to `0.625/-0.625`; pass/fail = `True`.

## Split RMSE Versus B/C

| dataset_split | baseline_rmse_nm | b_corrected_rmse_nm | c_corrected_rmse_nm | round2_corrected_rmse_nm | round2_rmse_improvement_pct |
| --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 0.036866 | 0.028528 | 0.024120 | 0.030451 | 17.401822 |
| open_floor_fit_downweighted | 0.043194 | 0.041880 | 0.033093 | 0.040701 | 5.772251 |
| open_floor_validation_only | 0.016931 | 0.011372 | 0.014417 | 0.019477 | -15.040469 |
| diag_validation_only | 0.084621 | 0.084412 | 0.038767 | 0.063325 | 25.166551 |
| aux_downweighted_validation | 0.051266 | 0.048945 | 0.028529 | 0.044315 | 13.558596 |
| validation_non_authoritative | 0.050234 | nan | 0.030342 | 0.042721 | 14.955935 |

## Selected-Log RMSE Versus B/C

| run_id | dataset_split | baseline_rmse_nm | b_corrected_rmse_nm | c_corrected_rmse_nm | round2_corrected_rmse_nm | round2_signed_median_nm |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.021129 | 0.028076 | 0.037075 | 0.000360 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.015289 | 0.019341 | 0.026669 | -0.000693 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.013499 | 0.014403 | 0.016376 | -0.001507 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.034045 | 0.018608 | 0.023349 | 0.003608 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.027835 | 0.023754 | 0.028405 | 0.004805 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.035146 | 0.027743 | 0.034541 | -0.001289 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.036504 | 0.029338 | 0.035376 | -0.000154 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.056260 | 0.041312 | 0.054380 | 0.000109 |
| diag003 | diag_validation_only | 0.085238 | 0.085034 | 0.038970 | 0.064468 | -0.004108 |

## Dominant Fitted Surface Coefficients

| feature | value | feature_scale | raw_coefficient_nm_per_feature | abs_standardized_coefficient_nm |
| --- | --- | --- | --- | --- |
| contact_creep_tight_nm__gate | 0.197258 | 0.037363 | 5.279496 | 0.197258 |
| contact_creep_tight_nm__gate_util | -0.164172 | 0.022620 | -7.257866 | 0.164172 |
| actual_contact_yaw_authority_nm__gate_util | 0.134838 | 0.015479 | 8.710738 | 0.134838 |
| actual_contact_yaw_authority_nm__gate | -0.131989 | 0.025292 | -5.218704 | 0.131989 |
| contact_creep_wide_nm__gate | -0.128373 | 0.027369 | -4.690397 | 0.128373 |
| contact_creep_wide_nm__gate_util | 0.125974 | 0.016653 | 7.564665 | 0.125974 |
| applied_yaw_force_authority_nm__gate | -0.029179 | 0.044614 | -0.654033 | 0.029179 |
| force_gap_yaw_authority_nm__gate_limiter | -0.014825 | 0.002628 | -5.641142 | 0.014825 |
| applied_yaw_force_authority_nm__gate_util | 0.010093 | 0.026513 | 0.380687 | 0.010093 |
| applied_yaw_force_authority_nm__gate_high_forward | 0.008655 | 0.001657 | 5.222983 | 0.008655 |
| vbar_rel_mps__gate | 0.007154 | 0.079808 | 0.089644 | 0.007154 |
| launch_core_opposes_nm__gate_high_forward | -0.003939 | 0.001007 | -3.911840 | 0.003939 |
| abs_yaw_rate_radps__gate_high_forward | -0.001940 | 0.023462 | -0.082684 | 0.001940 |
| contact_creep_tight_nm__gate_high_forward | -0.000629 | 0.001293 | -0.486764 | 0.000629 |
| actual_contact_yaw_moment_nm__gate_high_forward | -0.000406 | 0.001015 | -0.399842 | 0.000406 |

## 6x10 Vf/Yaw L-R Delta Grid Summary

The full grid is in `lr_delta_grid.csv`; the pivot summary is in `lr_delta_pivot.md`. Key observation: the +1 rad/s in-place neighborhood stays near B, while the surface increasingly diverges from B as forward speed, contact-relative speed, yaw rate, and utilization rise.

## Tuning Candidates

| vrel_knee_mps | fwd_knee_mps | ridge | objective_score | validation_rb_rmse_nm | straight_rb_rmse_nm | low_speed_yaw_rb_rmse_nm | in_place_left_command | hard_gate_pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.080000 | 0.350000 | 0.010000 | 0.055941 | 0.039900 | 0.019407 | 0.046246 | 0.625079 | True |
| 0.100000 | 0.350000 | 0.010000 | 0.057263 | 0.040701 | 0.019785 | 0.048186 | 0.631238 | True |
| 0.100000 | 0.350000 | 0.030000 | 0.057263 | 0.040701 | 0.019786 | 0.048187 | 0.631232 | True |
| 0.100000 | 0.700000 | 0.010000 | 0.058637 | 0.041650 | 0.019842 | 0.050211 | 0.634680 | True |

## Failure Modes

See `failure_modes.md` for the detailed list. The most important caveat is that this is still a residual fit over reconstructed contact features; production eligibility requires full force replay and targeted yaw-launch/low-speed-turn validation.

## Output Files

- `fit_request_contact_surface.py`
- `request_contact_surface_report.md`
- `request_contact_surface_coefficients.csv`
- `candidate_tuning_scores.csv`
- `split_rmse_vs_b_c.csv`
- `selected_log_rmse_vs_b_c.csv`
- `in_place_1radps_command_estimate.csv`
- `lr_delta_grid.csv`
- `lr_delta_pivot.md`
- `failure_modes.md`
- `commands_run.txt`
