# Dynamic LuGre/Bristle Yaw Model

Analysis-only output. Production code, build metadata, and tests were not edited.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\round2_hybrid_b_c\fit_dynamic_bristle_lugre.py
```

## Model

This is a different model family from the rejected gated hybrid. It introduces an internal LuGre/Dahl-style bristle deflection state `z` per log, evolved from physical yaw/contact relative velocity history:

`z_dot = v_y / L - |v_y| * z / (L * g(v_t))`

`g(v_t) = mu_c + (1 - mu_c) * exp(-(v_t / v_s)^2)`

`M_opp = K_z * (N/N0) * max(0, d_yaw*z) + X_force_state(v_contact, F_projected, N, utilization_actual) * beta`

The model does not use command, requested force, requested yaw moment, selector labels, or a residual lookup table as traction inputs. B and C appear only as comparison references and for the physical +1 rad/s calibration target.

## Selected Parameters

| vrel_knee_mps | fwd_knee_mps | bristle_length_m | stribeck_speed_mps | coulomb_fraction | ridge | static_gain_nm | nominal_load_n |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.050000 | 0.700000 | 0.002000 | 0.060000 | 0.650000 | 0.030000 | 0.065783 | 1.932931 |

## +1 rad/s In-Place Command

| variant | extra Nm | total opp Nm | left cmd | right cmd | L-R delta | max abs cmd | gate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Baseline | 0.000000 | 0.014794 | 0.261744 | -0.261744 | 0.523488 | 0.261744 | False |
| B_stribeck | 0.065013 | 0.079808 | 0.642750 | -0.642750 | 1.285499 | 0.642750 | True |
| C_combined_slip | 0.018241 | 0.033035 | 0.368641 | -0.368641 | 0.737282 | 0.368641 | False |
| Dynamic_LuGre_bristle | 0.079622 | 0.094416 | 0.728361 | -0.728361 | 1.456723 | 0.728361 | True |

Hard gate at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`: PASS. Dynamic model predicts left/right `0.728/-0.728`, `max |cmd|=0.728` versus required `>= 0.600`.

## Decision

Candidate status: rejected as a production tune. It is physically compliant and materially different, but it does not retain C-like broad residual performance.
Primary RMSE: dynamic 0.020907 Nm, B 0.028528, C 0.024120. Validation RMSE: dynamic 0.033502 Nm, C 0.030342.

## Split RMSE Versus B/C

| split | count | baseline | B RMSE | C RMSE | Dynamic RMSE | Dynamic vs B | Dynamic vs C |
| --- | --- | --- | --- | --- | --- | --- | --- |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.028528 | 0.024120 | 0.020907 | 26.712225 | 13.319845 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.041880 | 0.033093 | 0.025782 | 38.438522 | 22.092121 |
| open_floor_validation_only | 14542 | 0.016931 | 0.011372 | 0.014417 | 0.010405 | 8.499038 | 27.826440 |
| diag_validation_only | 11108 | 0.084621 | 0.084412 | 0.038767 | 0.057896 | 31.412049 | -49.345639 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.048945 | 0.028529 | 0.037632 | 23.113970 | -31.905498 |
| validation_non_authoritative | 71263 | 0.050234 |  | 0.030342 | 0.033502 |  | -10.415936 |

## Selected Log RMSE Versus B/C

| run | split | count | baseline | B RMSE | C RMSE | Dynamic RMSE | Dynamic vs B | Dynamic vs C |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.021129 | 0.028076 | 0.019369 | 8.326418 | 31.012180 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.015289 | 0.019341 | 0.015478 | -1.238677 | 19.971467 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.013499 | 0.014403 | 0.016696 | -23.684369 | -15.919070 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.034045 | 0.018608 | 0.020870 | 38.696599 | -12.155754 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.027835 | 0.023754 | 0.019439 | 30.164974 | 18.168476 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.035146 | 0.027743 | 0.021576 | 38.609362 | 22.227964 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.036504 | 0.029338 | 0.022416 | 38.592905 | 23.595000 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.056260 | 0.041312 | 0.028119 | 50.018767 | 31.934939 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.085034 | 0.038970 | 0.058828 | 30.817818 | -50.956279 |

## Risk Slices

| group | count | baseline | dynamic | median abs before | median abs after | RB change |
| --- | --- | --- | --- | --- | --- | --- |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025842 | 0.018261 | 0.006082 | 0.005558 | 32.552078 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.024100 | 0.013943 | 0.005326 | 0.005192 | 41.372126 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.073503 | 0.043539 | 0.054369 | 0.028883 | 39.799075 |
| high_forward_vf_ge_0p5 | 3120 | 0.046415 | 0.028103 | 0.012755 | 0.013216 | 48.673382 |
| limiter_active | 31216 | 0.074601 | 0.040628 | 0.053156 | 0.026463 | 46.035576 |

## 6x10 Vf/Yaw L-R Delta Grid Summary

Dynamic absolute L/R-delta difference from B: median 0.543, p90 1.794.
Dynamic absolute L/R-delta difference from C: median 0.514, p90 1.397.
Rows with `|cmd| > 1` for dynamic grid: 22 of 60; raw contact-utilization > 1: 36 of 60.

## Dynamic_LuGre_bristle L/R Delta

| Vf \ yaw | 0.2 | 0.844 | 1.49 | 2.13 | 2.78 | 3.42 | 4.07 | 4.71 | 5.36 | 6 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.000 | 1.657 | 1.477 | 1.434 | 1.511 | 1.717 | 2.054 | 2.524 | 3.127 | 3.865 | 4.556 |
| 0.030 | 1.227 | 1.431 | 1.301 | 1.366 | 1.562 | 1.895 | 2.367 | 2.979 | 3.730 | 4.506 |
| 0.060 | 1.032 | 0.839 | 1.239 | 1.085 | 1.261 | 1.583 | 2.055 | 2.679 | 3.455 | 4.379 |
| 0.090 | 0.911 | 0.692 | 0.579 | 1.159 | 1.035 | 1.338 | 1.800 | 2.426 | 3.216 | 4.166 |
| 0.120 | 0.881 | 0.639 | 0.501 | 0.485 | 0.827 | 1.179 | 1.621 | 2.235 | 3.024 | 3.987 |
| 0.150 | 0.890 | 0.624 | 0.458 | 0.409 | 0.494 | 0.834 | 1.441 | 2.058 | 2.964 | 4.095 |

## Tuning Scores

| gated obj | objective | L | v_s | mu_c | ridge | 1rad |cmd| | gate | validation RB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.034614 | 0.034614 | 0.002000 | 0.060000 | 0.650000 | 0.030000 | 0.728361 | True | 0.034614 |
| 0.034768 | 0.034768 | 0.002000 | 0.060000 | 0.650000 | 0.030000 | 0.792509 | True | 0.034768 |
| 0.035596 | 0.035596 | 0.002000 | 0.100000 | 0.650000 | 0.030000 | 0.659660 | True | 0.035596 |
| 0.035832 | 0.035832 | 0.002000 | 0.100000 | 0.650000 | 0.030000 | 0.720748 | True | 0.035832 |
| 0.036687 | 0.036687 | 0.001000 | 0.060000 | 0.650000 | 0.030000 | 0.623510 | True | 0.036687 |
| 0.036879 | 0.036879 | 0.001000 | 0.060000 | 0.650000 | 0.030000 | 0.683267 | True | 0.036879 |
| 0.037932 | 0.037932 | 0.001000 | 0.100000 | 0.650000 | 0.030000 | 0.617025 | True | 0.037932 |
| 0.038888 | 0.038888 | 0.002000 | 0.060000 | 0.850000 | 0.030000 | 0.649261 | True | 0.038888 |
| 0.039331 | 0.039331 | 0.002000 | 0.100000 | 0.850000 | 0.030000 | 0.624753 | True | 0.039331 |
| 10.037641 | 0.037641 | 0.001000 | 0.100000 | 0.650000 | 0.030000 | 0.559288 | False | 0.037641 |
| 10.038371 | 0.038371 | 0.000500 | 0.060000 | 0.650000 | 0.030000 | 0.524715 | False | 0.038371 |
| 10.038598 | 0.038598 | 0.000500 | 0.060000 | 0.650000 | 0.030000 | 0.582368 | False | 0.038598 |

## Dominant Coefficients

| branch | feature | std coeff Nm | scale | raw coeff |
| --- | --- | --- | --- | --- |
| dynamic_bristle | lugre_bristle_state_basis | 0.065783 | 1.000000 | 0.065783 |
| moving_contact | force_moment_opposes_yaw_nm__base | -0.034932 | 0.050554 | -0.690978 |
| moving_contact | gain_left_long_basis__force_util | 0.014157 | 0.004496 | 3.148723 |
| moving_contact | gain_right_long_basis__force_util | 0.014157 | 0.004496 | 3.148723 |
| moving_contact | gain_front_right_basis__base | -0.012637 | 0.000562 | -22.489634 |
| moving_contact | gain_rear_right_basis__base | -0.012637 | 0.000562 | -22.489634 |
| moving_contact | gain_left_long_basis__base | -0.012454 | 0.009128 | -1.364333 |
| moving_contact | gain_right_long_basis__base | -0.012454 | 0.009128 | -1.364333 |
| moving_contact | gain_front_right_basis__force_util | 0.011102 | 0.000287 | 38.658768 |
| moving_contact | gain_rear_right_basis__force_util | 0.011102 | 0.000287 | 38.658768 |
| moving_contact | force_moment_opposes_yaw_nm__force_util | -0.007035 | 0.022366 | -0.314524 |
| moving_contact | force_moment_opposes_yaw_nm__low_rel | 0.004843 | 0.018457 | 0.262398 |
| moving_contact | gain_front_right_basis__low_rel | -0.002254 | 0.000100 | -22.554869 |
| moving_contact | gain_rear_right_basis__low_rel | -0.002254 | 0.000100 | -22.554869 |
| moving_contact | force_moment_opposes_yaw_nm__high_forward | -0.001179 | 0.001299 | -0.907902 |
| moving_contact | gain_left_long_basis__low_rel | 0.000913 | 0.001285 | 0.710240 |

## Production Risks

- The dynamic state must be integrated at the control-loop boundary with clear reset behavior after lift/service/discontinuous state.
- The current fit uses sparse analysis rows, not a continuous replay of every control tick; state identification should be repeated on full-rate logs before production.
- The +1 rad/s calibration is from the B command inversion/reference condition and should be validated against direct in-place launch measurements.
- The moving force-state surface can still learn large coefficients; coefficient sign and scaling need a stricter physical prior before code implementation.
- B/C comparisons include request-aware reference models only as benchmarks; the dynamic candidate itself does not use request/command traction inputs.

## Output Files

- `fit_dynamic_bristle_lugre.py`
- `dynamic_bristle_lugre_report.md`
- `dynamic_bristle_coefficients.csv`
- `dynamic_selected_hyperparameters.json`
- `dynamic_candidate_tuning_scores.csv`
- `dynamic_split_metrics.csv`
- `dynamic_split_rmse_comparison_vs_b_c.csv`
- `dynamic_selected_log_metrics.csv`
- `dynamic_selected_log_rmse_comparison_vs_b_c.csv`
- `dynamic_risk_metrics.csv`
- `dynamic_in_place_1radps_command.csv`
- `dynamic_lr_delta_grid.csv`
- `dynamic_commands_run.txt`
