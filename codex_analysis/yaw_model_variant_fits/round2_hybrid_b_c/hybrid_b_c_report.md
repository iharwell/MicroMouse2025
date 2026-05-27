# Round 2 Hybrid B/C Yaw Model

Analysis-only output. Production code, build metadata, and tests were not edited.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\round2_hybrid_b_c\fit_round2_hybrid_b_c.py
```

## Model Family

The selected hybrid predicts yaw-opposing residual torque and converts it back to the raw additive yaw moment with:

`M_raw_pred = -d_yaw * M_opp_pred`

where `d_yaw = sign(yaw_rate)` with signed contact-velocity fallback only at near-zero yaw. The residual after correction is `M_raw_residual - M_raw_pred`.

The fitted opposing torque is one coherent hybrid contact-force law with two coupled mechanisms:

`M_opp_pred = G_slide * X_contact(v_contact, F_projected, load, utilization_actual) * beta_contact + B(v_y, N) * R(v_t) * (K_slide + K_static * exp(-(v_t / v_s)^2))`

with:

- `v_t = sqrt((rel_weight * max(vbar_rel, load_weighted_rel))^2 + |Vf|^2)`, `rel_weight=0.750`.
- `v_y = rel_weight * max(vbar_yaw, load_weighted_lat)` and `B(v_y, N) = (N / N_nominal) * (1 - exp(-(v_y / 0.008 m/s)^2))` is the reduced bristle-deflection fill term.
- `R(v_t) = 1 / (1 + (v_t / 0.640 m/s)^2)`.
- `v_s=0.100 m/s` for the Stribeck fade.
- `G_slide = (1 - clamp((1 - exp(-(v_y / v_bristle)^2)) * exp(-(v_t / v_s)^2) * R(v_t), 0, 1))^2.0` fades the moving-contact surface while static adhesion is loaded.
- `X_contact` is a force-state combined-slip basis: per-contact lateral/longitudinal velocity bases, projected contact yaw moment, projected contact-moment magnitude, actual force utilization, normal load delta, low relative speed, and high forward speed.

## New Mechanism Versus A/C/D

The rejected draft used requested yaw moment to load a static reservoir; that violates the rule that traction must not differ for the same physical contact state merely because command/request differs. The revised mechanism is a physical bristle-displacement static adhesion reservoir:

`M_static_capacity = B(v_y, N) * R(v_t) * (K_slide + K_static * exp(-(v_t / v_s)^2))`

This term can produce finite yaw-opposing torque at low contact speed because contact bristles deflect over relative displacement and load, then release continuously with transition speed. That is the missing degree of freedom in the low-order A/C/D-style residual surfaces: they only fit instantaneous residual surfaces, while this family adds a tire/contact micro-state approximation before sliding. No command, requested force, or mode label selects the traction law.

The +1 rad/s in-place B launch torque calibrates the static reservoir magnitude at the reference physical contact state. The moving-contact coefficients are fit only from physical projected-force/contact-velocity features after subtracting the bristle branch. This is not a runtime mode label or residual lookup table.

## Selected Hyperparameters

| vrel_knee_mps | fwd_knee_mps | bristle_velocity_mps | slide_gate_power | moving_surface_source | anchor_extra_nm | nominal_load_n | static_branch_source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.050000 | 0.700000 | 0.008000 | 2.000000 | fit from physical projected-force/contact-velocity features | 0.065013 | 1.932931 | Variant B Stribeck coefficient ratio scaled to physical velocity/load bristle basis |

## Static Branch Coefficients

| feature | std coeff Nm | scale | raw coeff |
| --- | --- | --- | --- |
| bristle_sliding_basis | 0.073552 | 1.000000 | 0.073552 |
| bristle_static_basis | 0.002790 | 1.000000 | 0.002790 |

## Dominant Coefficients

| branch | feature | std coeff Nm | scale | raw coeff |
| --- | --- | --- | --- | --- |
| static_launch | bristle_sliding_basis | 0.073552 | 1.000000 | 0.073552 |
| moving_contact | force_moment_opposes_yaw_nm__base | -0.030656 | 0.034766 | -0.881760 |
| moving_contact | force_moment_opposes_yaw_nm__force_util | -0.008356 | 0.013578 | -0.615367 |
| moving_contact | gain_front_right_basis__base | -0.006086 | 0.000157 | -38.757062 |
| moving_contact | gain_rear_right_basis__base | -0.006086 | 0.000157 | -38.757062 |
| moving_contact | gain_rear_right_basis__low_rel | -0.004218 | 0.000064 | -66.165740 |
| moving_contact | gain_front_right_basis__low_rel | -0.004218 | 0.000064 | -66.165740 |
| moving_contact | gain_left_long_basis__force_util | 0.003903 | 0.001609 | 2.426570 |
| moving_contact | gain_right_long_basis__force_util | 0.003903 | 0.001609 | 2.426570 |
| moving_contact | gain_rear_right_basis__force_util | 0.003398 | 0.000064 | 53.299331 |
| moving_contact | gain_front_right_basis__force_util | 0.003398 | 0.000064 | 53.299331 |
| static_launch | bristle_static_basis | 0.002790 | 1.000000 | 0.002790 |
| moving_contact | force_moment_opposes_yaw_nm__low_rel | 0.002741 | 0.015471 | 0.177199 |
| moving_contact | gain_right_long_basis__low_rel | -0.002519 | 0.001118 | -2.253357 |

## +1 rad/s In-Place Command

| variant | extra Nm | total opp Nm | left cmd | right cmd | L-R delta | max abs cmd | gate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Baseline | 0.000000 | 0.014794 | 0.261744 | -0.261744 | 0.523488 | 0.261744 | False |
| B_stribeck | 0.065013 | 0.079808 | 0.642750 | -0.642750 | 1.285499 | 0.642750 | True |
| C_combined_slip | 0.018241 | 0.033035 | 0.368641 | -0.368641 | 0.737282 | 0.368641 | False |
| Hybrid_BC_adhesion_partition | 0.064633 | 0.079427 | 0.640520 | -0.640520 | 1.281040 | 0.640520 | True |

Hard gate at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`: PASS. Hybrid predicts left/right `0.641/-0.641`, so `max |cmd|=0.641` versus the required `>= 0.600`. The reference measured/calculated command is approximately `+0.646/-0.646`.

## Split RMSE Versus B/C

| split | count | baseline | B RMSE | C RMSE | Hybrid RMSE | Hybrid vs B | Hybrid vs C |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.028528 | 0.024120 | 0.029752 | -4.292251 | -23.350292 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.041880 | 0.033093 | 0.042732 | -2.035132 | -29.128490 |
| open_floor_validation_only | 14542 | 0.016931 | 0.011372 | 0.014417 | 0.025884 | -127.615939 | -79.537486 |
| diag_validation_only | 11108 | 0.084621 | 0.084412 | 0.038767 | 0.073588 | 12.822429 | -89.823283 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.048945 | 0.028529 | 0.040079 | 18.115245 | -40.481299 |
| validation_non_authoritative | 71263 | 0.050234 |  | 0.030342 | 0.045881 |  | -51.213693 |

## Selected Log RMSE Versus B/C

| run | split | count | baseline | B RMSE | C RMSE | Hybrid RMSE | Hybrid vs B | Hybrid vs C |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.021129 | 0.028076 | 0.035766 | -69.275797 | -27.386407 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.015289 | 0.019341 | 0.038394 | -151.118297 | -98.507424 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.013499 | 0.014403 | 0.017239 | -27.707838 | -19.689932 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.034045 | 0.018608 | 0.018099 | 46.837971 | 2.739043 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.027835 | 0.023754 | 0.020593 | 26.018396 | 13.309585 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.035146 | 0.027743 | 0.033818 | 3.779298 | -21.896108 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.036504 | 0.029338 | 0.034917 | 4.347434 | -19.014493 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.056260 | 0.041312 | 0.055896 | 0.646957 | -35.300204 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.085034 | 0.038970 | 0.074455 | 12.440878 | -91.054962 |

## Risk Slices

| group | count | baseline | hybrid | median abs before | median abs after | RB change |
| --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025842 | 0.010778 | 0.006082 | 0.004364 | 67.590534 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.024100 | 0.011341 | 0.005326 | 0.004818 | 57.784815 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.073503 | 0.077146 | 0.054369 | 0.046809 | -7.113477 |
| high_forward_vf_ge_0p5 | 3120 | 0.046415 | 0.028415 | 0.012755 | 0.011107 | 50.247478 |
| limiter_active | 31216 | 0.074601 | 0.057316 | 0.053156 | 0.032357 | 33.819461 |

## 6x10 Vf/Yaw L-R Delta Grid Summary

Hybrid absolute L/R-delta difference from B over the grid: median 0.154, p90 3.100.
Hybrid absolute L/R-delta difference from C over the grid: median 0.395, p90 3.311.
Rows with `|cmd| > 1` for the hybrid grid: 14 of 60; raw contact-utilization > 1: 36 of 60.

## Hybrid_BC_adhesion_partition L/R Delta

| Vf \ yaw | 0.2 | 0.844 | 1.49 | 2.13 | 2.78 | 3.42 | 4.07 | 4.71 | 5.36 | 6 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.000 | -0.059 | 1.148 | 1.493 | 1.622 | 1.736 | 1.848 | 1.961 | 2.075 | 2.196 | 2.329 |
| 0.030 | -0.287 | 1.210 | 1.485 | 1.611 | 1.722 | 1.833 | 1.945 | 2.063 | 2.189 | 2.330 |
| 0.060 | -0.369 | 0.799 | 1.632 | 1.548 | 1.650 | 1.760 | 1.879 | 2.012 | 2.164 | 2.199 |
| 0.090 | -0.410 | 0.690 | 1.011 | 1.629 | 1.425 | 1.535 | 1.681 | 1.610 | 1.287 | 1.041 |
| 0.120 | -0.435 | 0.471 | 0.678 | 0.538 | 0.698 | 0.628 | -0.223 | -0.722 | -1.031 | -1.250 |
| 0.150 | -0.434 | 0.096 | -0.038 | -0.536 | -0.804 | -2.551 | -2.235 | -2.294 | -2.393 | -2.405 |

## Tuning Scores

| gated obj | objective | k_rel | k_fwd | v_bristle | gate_pow | ridge | 1rad |cmd| | gate | validation RB | primary RB | open-fit RB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.040989 | 0.040989 | 0.050000 | 0.700000 | 0.008000 | 2.000000 | 0.100000 | 0.640520 | True | 0.040989 | 0.029382 | 0.038968 |
| 0.040989 | 0.040989 | 0.050000 | 0.700000 | 0.008000 | 2.000000 | 0.030000 | 0.640520 | True | 0.040989 | 0.029382 | 0.038968 |
| 0.040989 | 0.040989 | 0.050000 | 0.700000 | 0.008000 | 2.000000 | 0.010000 | 0.640520 | True | 0.040989 | 0.029382 | 0.038968 |
| 0.040989 | 0.040989 | 0.050000 | 0.700000 | 0.008000 | 2.000000 | 0.003000 | 0.640520 | True | 0.040989 | 0.029382 | 0.038968 |
| 0.041246 | 0.041246 | 0.060000 | 0.700000 | 0.008000 | 2.000000 | 0.100000 | 0.641385 | True | 0.041246 | 0.029607 | 0.039443 |
| 0.041246 | 0.041246 | 0.060000 | 0.700000 | 0.008000 | 2.000000 | 0.030000 | 0.641385 | True | 0.041246 | 0.029607 | 0.039443 |
| 0.041246 | 0.041246 | 0.060000 | 0.700000 | 0.008000 | 2.000000 | 0.010000 | 0.641385 | True | 0.041246 | 0.029607 | 0.039443 |
| 0.041246 | 0.041246 | 0.060000 | 0.700000 | 0.008000 | 2.000000 | 0.003000 | 0.641385 | True | 0.041246 | 0.029607 | 0.039443 |
| 0.041543 | 0.041543 | 0.080000 | 0.700000 | 0.008000 | 2.000000 | 0.100000 | 0.642447 | True | 0.041543 | 0.029846 | 0.040157 |
| 0.041543 | 0.041543 | 0.080000 | 0.700000 | 0.008000 | 2.000000 | 0.030000 | 0.642447 | True | 0.041543 | 0.029846 | 0.040157 |
| 0.041543 | 0.041543 | 0.080000 | 0.700000 | 0.008000 | 2.000000 | 0.010000 | 0.642447 | True | 0.041543 | 0.029846 | 0.040157 |
| 0.041543 | 0.041543 | 0.080000 | 0.700000 | 0.008000 | 2.000000 | 0.003000 | 0.642447 | True | 0.041543 | 0.029846 | 0.040157 |

## Production Risks

- The static reservoir calibration is derived from the B command inversion, not a direct production validation. It should be checked on explicit in-place launch logs before any production tune.
- The near-zero-yaw direction fallback must be implemented as a continuous command/contact direction, not a raw noisy gyro sign branch.
- Moving-contact terms depend on projected contact forces and actual force utilization. Any PlantModel force-projection change invalidates the coefficients and requires a refit.
- Several high-yaw grid cells exceed unit command or raw contact utilization. Those cells need full force-projection replay before treating command magnitudes as feasible.
- The model can add or subtract residual resistance through moving-contact coefficients. That improves broad RMSE but increases sign-convention risk if contact bases are transposed or direction conventions drift.
- This fit uses open-floor/diagnostic feature exports only. Maze wall contact, fan-duty changes outside the sampled envelope, and high-performance maneuver transitions remain out-of-sample.

## Output Files

- `fit_round2_hybrid_b_c.py`
- `hybrid_b_c_report.md`
- `hybrid_model_coefficients.csv`
- `selected_hyperparameters.json`
- `candidate_tuning_scores.csv`
- `split_metrics.csv`
- `split_rmse_comparison_vs_b_c.csv`
- `selected_log_metrics.csv`
- `selected_log_rmse_comparison_vs_b_c.csv`
- `risk_metrics.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid_hybrid_b_c.csv`
- `commands_run.txt`
