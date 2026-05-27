# Brush/Pacejka-Lite Combined-Slip Contact Law

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Model

For each contact patch `i` at position `(r_i, f_i)`, the law predicts contact forces and then accumulates yaw as:

`M_yaw = sum_i(f_i * F_r,i - r_i * F_f,i)`

Longitudinal raw demand:

`F_f,raw = drive_scale * F_drive_side/2 + mu_longitudinal * N_i * v_f / sqrt(v_f^2 + s_long^2)`

Low-speed yaw/Stribeck branch:

`F_r,low = (mu_static_breakaway + mu_low_speed_lateral) * N_i * v_r / sqrt(v_r^2 + s_static^2)`

High-speed slip-angle branch:

`alpha_proxy = v_r / sqrt(|Vf|^2 + v_floor^2)`

`F_r,corner = mu_cornering * N_i * alpha_proxy / sqrt(alpha_proxy^2 + alpha_knee^2)`

Rational speed transition:

`g = Vf^2 / (Vf^2 + speed_gate_knee^2)` and `F_r,raw = (1-g)*F_r,low + g*F_r,corner`

Combined-slip envelope:

`Y_i = mu_peak * N_i`

`scale_i = 1 / sqrt(1 + (((F_f,raw / long_shape)^2 + F_r,raw^2) / Y_i^2))`

`F_f,i = scale_i * F_f,raw`, `F_r,i = scale_i * F_r,raw`

`mu_static_breakaway` is analytically derived for each optimizer step from the selected in-place static opposing moment so the `Vf=0`, `yawRate=1 rad/s` branch reduces to the measured breakaway-style yaw support before the command target penalty is applied.

## Selected Parameters

| parameter | lower | selected | upper |
| --- | ---: | ---: | ---: |
| drive_scale | 0.035 | 0.116304279 | 0.18 |
| mu_peak | 0.65 | 5 | 5 |
| mu_longitudinal | 0 | 0.496484736 | 1.5 |
| mu_low_speed_lateral | 0 | 0.042427306 | 1.5 |
| mu_cornering | 0 | 0.133476205 | 5 |
| alpha_knee | 0.02 | 0.0438386003 | 0.9 |
| long_speed_floor_mps | 0.02 | 1 | 1 |
| speed_gate_knee_mps | 0.03 | 0.0918637141 | 1.5 |
| static_slip_knee_mps | 0.003 | 0.0212199567 | 0.1 |
| longitudinal_slip_knee_mps | 0.01 | 0.7 | 0.7 |
| longitudinal_envelope_shape | 0.45 | 2.5 | 2.5 |
| launch_static_opposing_nm | 0.003 | 0.00934108405 | 0.02 |
| mu_static_breakaway | derived | 0.532840981 | derived |

## Launch Estimate

| total opposing Nm | left command | right command | max abs command | pass |
| ---: | ---: | ---: | ---: | --- |
| 0.009341 | 0.645728 | -0.645728 | 0.645728 | True |

## Convergence

Selected result: restart `100` / `coordinate_stability_polish`, objective `0.002018388`, iterations `74`, evaluations `1179`, converged `True`.

| restart | seed | objective | iterations | evaluations | converged |
| ---: | --- | ---: | ---: | ---: | --- |
| 1 | standalone_like | 0.001752972 | 260 | 363 | False |
| 2 | brush_high_corner | 0.001752314 | 260 | 377 | False |
| 3 | soft_envelope | 0.001759069 | 260 | 378 | False |
| 4 | fast_gate | 0.001753159 | 260 | 364 | False |
| 99 | full_primary_polish | 0.002018527 | 360 | 513 | False |
| 100 | coordinate_stability_polish | 0.002018388 | 74 | 1179 | True |

## Split Metrics

| split | count | baseline RMSE | brush RMSE | improvement | force-domain Stribeck | rational residual | standalone contact | true-traction testbed | Variant C |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.016672 | 0.547774 | 0.027907821180421354 | 0.021083018384621146 | 0.016887630899899024 | 0.019519867783538924 | 0.024120243843366647 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.026738 | 0.380987 | 0.04150718685740355 | 0.03282316561347566 | 0.02663246465575232 | 0.034836888756372 | 0.033092937468878494 |
| open_floor_validation_only | 14542 | 0.016931 | 0.006964 | 0.588685 | 0.01139376781450519 | 0.011206860684196811 | 0.007711279672300156 | 0.008849476125192358 | 0.014417143137851248 |
| diag_validation_only | 11108 | 0.084621 | 0.028612 | 0.661878 | 0.08441017836709906 | 0.03848912769383371 | 0.023947153628180864 | 0.02570299117809136 | 0.03876675590868194 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.022071 | 0.569478 | 0.048819706958314515 | 0.027490242677710965 | 0.020687366391557327 | 0.027895852240170986 | 0.028529439179585198 |
| validation_non_authoritative | 71263 | 0.050234 | 0.023429 | 0.533604 |  | 0.029680153603611304 | 0.022326615396330518 | 0.028415959824015274 | 0.03034172395599833 |

## Selected Logs

| run | split | count | baseline RMSE | brush RMSE | force-domain | rational | standalone | true-traction | Variant C |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.011319 | 0.02095614161732105 | 0.020127808869544805 | 0.011589046852084053 | 0.015602242066314734 | 0.028076443199331575 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.009522 | 0.015366476488899375 | 0.015838560861422477 | 0.00983796163679149 | 0.01014360504287235 | 0.01934116142597289 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.012169 | 0.01348701867968651 | 0.013202689482483838 | 0.012330360089918665 | 0.012097343220894962 | 0.014402919863369421 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.013164 | 0.033937937440801695 | 0.016861102635710126 | 0.01230667486040885 | 0.013782370391656997 | 0.018608488843215733 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.015916 | 0.027283610538090263 | 0.017657454789089182 | 0.016517464699137962 | 0.016372638287973088 | 0.023754464601601764 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.020585 | 0.03430247806398148 | 0.026878339329649562 | 0.020650381351574418 | 0.026887847291071503 | 0.02774307957781035 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.022199 | 0.0356483649477145 | 0.027977442201207736 | 0.02238198379643325 | 0.02818192262657356 | 0.02933811998534102 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.034195 | 0.056141320109295866 | 0.04248692457048695 | 0.0342756822673276 | 0.04314470942081007 | 0.04131229724660652 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.028391 | 0.08503230164347471 | 0.038712164739976 | 0.023743327476242724 | 0.02625386650679142 | 0.03897045903766876 |

## Risk Slices

| slice | count | baseline RMSE | brush RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.027191 | 0.541168 |
| in_place_scrub | 19704 | 0.073503 | 0.030826 | 0.580616 |
| slow_forward_turn | 19582 | 0.053206 | 0.027107 | 0.490540 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066518 | 0.298918 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.049549 | 0.304017 |
| fast_forward | 0 |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.011004 | 0.543418 |
| limiter_active | 31216 | 0.074601 | 0.035776 | 0.520434 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.042747 | 0.328647 |
| may4_latest_logs | 5217 | 0.032158 | 0.010746 | 0.665823 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.027191 | 0.019108 | 0.012276 |
| in_place_scrub | 19704 | 0.073503 | 0.030826 | 0.023423 | 0.017773 |
| slow_forward_turn | 19582 | 0.053206 | 0.027107 | 0.018095 | 0.011787 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066518 | 0.032087 | 0.020149 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.049549 | 0.038386 | 0.023422 |
| fast_forward | 0 |  |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.011004 | 0.007139 | 0.004414 |
| limiter_active | 31216 | 0.074601 | 0.035776 | 0.028145 | 0.024296 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.042747 | 0.034682 | 0.031086 |
| may4_latest_logs | 5217 | 0.032158 | 0.010746 | 0.008158 | 0.006745 |

## Assessment

Physical sanity: `qualified_true`. The selected equation shape is continuous, odd in local slip velocity, uses a rational zero-to-speed gate, and forces longitudinal/lateral patch demands to compete inside a smooth yield envelope. It does not use UKF state-vector fields, command/request/preprojection traction selectors, or an old-plus-residual runtime branch.

Coefficient sanity caveat: `mu_peak, long_speed_floor_mps, longitudinal_slip_knee_mps, longitudinal_envelope_shape` hit optimizer bounds. Treat the family as viable for comparison, but not as an identified production calibration without tighter targeted data or a narrower prior on the envelope/speed-floor terms.

The tradeoff is that the direct brush law is more physically shaped than the rational residual reference, but it is still fitted from noisy yaw-acceleration-derived moments. Validation and operating-range slices are reported, not imposed as hard constraints.

## Outputs

- `fit_brush_combined_slip.py`
- `brush_combined_slip_report.md`
- `selected_parameters.csv`
- `parameter_bound_hits.csv`
- `optimizer_summary.csv`
- `optimizer_trace.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `may4_latest_metrics.csv`
- `risk_slices.csv`
- `common_range_metrics.csv`
- `launch_command_estimate.csv`
- `prediction_sample.csv`
- `commands_run.txt`
