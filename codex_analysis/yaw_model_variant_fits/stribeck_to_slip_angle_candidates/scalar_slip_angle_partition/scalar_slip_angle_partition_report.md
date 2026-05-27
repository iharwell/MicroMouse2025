# Scalar Slip-Angle Partition Yaw Support

Analysis-only output. Production code, build metadata, tests, and prior analysis artifacts were not edited.

## Model Form

The model predicts a positive yaw-opposing support torque `M_opp`, then converts it to additive yaw torque with `M_add = -sign(yawRate) * M_opp` for residual evaluation.

Low-speed branch:

`v_y = abs(yawRate) * hypot(track_width/2, drive_wheel_longitudinal_offset)`

`A_y = v_y^2 / (v_y^2 + k_y^2)`

`S = 1 / (1 + (v_y/k_s)^2 + 0.2*(v_y/k_s)^4)`

`M_stribeck = M0 * A_y * (slide_ratio + (1-slide_ratio)*S)`

High-speed branch:

`alpha_proxy = abs(yawRate) * alpha_lever / (abs(Vf) + V_floor)`

`M_alpha = M_cap * (alpha_proxy/alpha_knee) / sqrt(1 + (alpha_proxy/alpha_knee)^2)`

Partition:

`G_low = 1 / (1 + (abs(Vf)/V_gate)^2)`

`M_opp = load_factor * (G_low*M_stribeck + (1-G_low)*M_alpha)`

There is no command, request, preprojection, trig, tanh, exp, lookup table, old-force branch, or residual table in the fitted law. The only exponential in the script is reused in the existing motor static-launch command estimate, outside the model.

## Launch Constraint

The Stribeck peak `M0` is analytically derived so `Vf=0`, `yawRate=+1 rad/s` produces the requested command target `0.646`. The resulting estimate is left/right `0.646000/-0.646000`, extra support 0.065568 Nm, and total opposing yaw torque 0.080362 Nm.

## Selected Parameters

| parameter | value |
| --- | ---: |
| yaw_activation_mps | 0.057682038 |
| stribeck_speed_mps | 0.071469188 |
| slide_ratio | 0.000000000 |
| speed_gate_mps | 0.004883232 |
| v_floor_mps | 0.073053322 |
| alpha_knee_rad | 0.020827971 |
| slip_cap_nm | 0.009510909 |
| alpha_lever_m | 0.027195486 |
| stribeck_quartic_fixed | 0.200000000 |
| derived_static_peak_nm | 0.248071808 |
| nominal_load_n | 1.932931008 |
| static_lever_m | 0.044814432 |

## Optimizer

Bounded Nelder-Mead was run from 12 continuous seeds using primary fit rows plus a light non-authoritative validation guard. Best objective `0.016150808` came from seed `1` after `295` evaluations.
Boundary notes: slide_ratio=near lower bound, speed_gate_mps=near lower bound, alpha_knee_rad=near lower bound.

| seed | objective | evals | static_peak_nm | boundary_flags |
| ---: | ---: | ---: | ---: | --- |
| 1 | 0.016150808 | 295 | 0.248072 | slide_ratio:low;speed_gate_mps:low;alpha_knee_rad:low |
| 8 | 0.016155481 | 228 | 0.248133 | slide_ratio:low;speed_gate_mps:low;v_floor_mps:low |
| 12 | 0.016363968 | 267 | 0.248236 | slide_ratio:low;speed_gate_mps:low;v_floor_mps:high;slip_cap_nm:low;alpha_lever_m:high |
| 7 | 0.016363975 | 391 | 0.249072 | slide_ratio:low;speed_gate_mps:low;v_floor_mps:high;alpha_knee_rad:low;slip_cap_nm:low;alpha_lever_m:high |
| 6 | 0.016364011 | 263 | 0.249939 | slide_ratio:low;speed_gate_mps:low;alpha_knee_rad:high;slip_cap_nm:low |
| 3 | 0.016364030 | 288 | 0.249261 | slide_ratio:low;speed_gate_mps:low;v_floor_mps:high;alpha_knee_rad:high;slip_cap_nm:low;alpha_lever_m:low |
| 2 | 0.016364052 | 281 | 0.248566 | slide_ratio:low;speed_gate_mps:low;alpha_knee_rad:high;slip_cap_nm:low;alpha_lever_m:low |
| 10 | 0.016364511 | 263 | 0.249094 | slide_ratio:low;speed_gate_mps:low;v_floor_mps:high;alpha_knee_rad:high;slip_cap_nm:low |

## Split Metrics

| split | count | baseline RMSE | scalar RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.036488 | 1.03% |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.045347 | -4.98% |
| open_floor_validation_only | 14542 | 0.016931 | 0.018000 | -6.31% |
| diag_validation_only | 11108 | 0.084621 | 0.096346 | -13.86% |
| aux_downweighted_validation | 14448 | 0.051266 | 0.055768 | -8.78% |
| validation_non_authoritative | 71263 | 0.050234 | 0.055162 | -9.81% |

## Comparison

| split | baseline | force Stribeck | rational residual | standalone traction | scalar partition |
| --- | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 0.036866 | 0.027908 | 0.021083 | 0.016688 | 0.036488 |
| open_floor_fit_downweighted | 0.043194 | 0.041507 | 0.032823 | 0.027952 | 0.045347 |
| open_floor_validation_only | 0.016931 | 0.011394 | 0.011207 | 0.007665 | 0.018000 |
| diag_validation_only | 0.084621 | 0.084410 | 0.038489 | 0.020030 | 0.096346 |
| aux_downweighted_validation | 0.051266 | 0.048820 | 0.027490 | 0.019198 | 0.055768 |
| validation_non_authoritative | 0.050234 |  | 0.029680 | 0.022157 | 0.055162 |

## Selected Logs

| run | split | baseline RMSE | scalar RMSE | signed median after |
| --- | --- | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 0.035846 | 0.033483 | 0.008228 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 0.023278 | 0.024582 | 0.000141 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 0.016217 | 0.015841 | -0.002922 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 0.044975 | 0.042861 | 0.007466 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 0.042584 | 0.041092 | 0.011881 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 0.039824 | 0.041762 | 0.008367 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 0.040355 | 0.042105 | 0.003283 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 0.056225 | 0.059806 | 0.000191 |
| diag003 | diag_validation_only | 0.085238 | 0.097092 | -0.015969 |

## May 4 Focus

- `2026-05-04_20-35-47`: baseline 0.035846, force Stribeck 0.020956, rational residual 0.020128, standalone traction 0.011393, scalar partition 0.033483.
- `2026-05-04_16-57-53`: baseline 0.023278, force Stribeck 0.015366, rational residual 0.015839, standalone traction 0.009662, scalar partition 0.024582.

## Risk Slices

| slice | count | baseline RMSE | scalar RMSE | improvement | median abs after |
| --- | ---: | ---: | ---: | ---: | ---: |
| straightish_abs_yaw_lt_0p05 | 35367 | 0.025842 | 0.025882 | -0.16% | 0.006512 |
| straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05 | 21746 | 0.024100 | 0.024147 | -0.19% | 0.005678 |
| low_speed_yaw_vf_lt_0p05_yaw_ge_0p2 | 19704 | 0.073503 | 0.081953 | -11.50% | 0.056616 |
| low_speed_launchish_vf_lt_0p15_yaw_ge_0p5 | 20694 | 0.072832 | 0.080728 | -10.84% | 0.052917 |
| high_forward_vf_ge_0p5 | 3120 | 0.046415 | 0.047590 | -2.53% | 0.012955 |
| high_speed_abs_vf_ge_0p7 | 144 | 0.088793 | 0.090620 | -2.06% | 0.030582 |
| limiter_active | 31216 | 0.074601 | 0.080278 | -7.61% | 0.055043 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.066348 | -4.20% | 0.044518 |
| negative_yaw_opposes_target | 66171 | 0.052718 | 0.059387 | -12.65% | 0.023771 |
| may4_selected_logs | 5217 | 0.032158 | 0.030768 | 4.32% | 0.018075 |

## Assessment

- Fit-authoritative corrected RMSE: `0.036488` Nm versus baseline `0.036866` Nm.
- Non-authoritative validation corrected RMSE: `0.055162` Nm versus baseline `0.050234` Nm.
- Viability: useful as a very cheap launch-preserving scalar support law, but not competitive with the standalone contact-patch traction testbed and generally behind the rational residual reference. The positive-support constraint is the main limitation on splits where the current plant already over-resists yaw.
- Cost estimate: per tick this law needs abs/multiply operations, three rational divides, and two square roots. It uses no trig/atan, tanh, exp, table lookup, or persistent state.

## Outputs

- `fit_scalar_slip_angle_partition.py`
- `scalar_slip_angle_partition_report.md`
- `optimizer_summary.csv`
- `selected_parameters.csv`
- `split_metrics.csv`
- `phase_metrics.csv`
- `selected_log_metrics.csv`
- `risk_slices.csv`
- `split_comparison_vs_references.csv`
- `selected_log_comparison_vs_references.csv`
- `launch_command_estimate.csv`
- `prediction_sample.csv`
- `commands_run.txt`
