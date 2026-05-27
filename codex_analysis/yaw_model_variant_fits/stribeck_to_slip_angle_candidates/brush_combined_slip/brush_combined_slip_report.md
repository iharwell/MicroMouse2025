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

`mu_static_breakaway` is analytically derived for each optimizer step from the fitted free `launch_static_opposing_nm` parameter. That parameter is optimized only through the data objective; the resulting `Vf=0`, `yawRate=1 rad/s` command is diagnostic.

## Selected Parameters

| parameter | lower | selected | upper |
| --- | ---: | ---: | ---: |
| drive_scale | 0.035 | 0.12152242 | 0.18 |
| mu_peak | 0.65 | 5 | 5 |
| mu_longitudinal | 0 | 0.49488478 | 1.5 |
| mu_low_speed_lateral | 0 | 0.599922184 | 1.5 |
| mu_cornering | 0 | 0.081044553 | 5 |
| alpha_knee | 0.02 | 0.02 | 0.9 |
| long_speed_floor_mps | 0.02 | 0.02 | 1 |
| speed_gate_knee_mps | 0.03 | 0.11058631 | 1.5 |
| static_slip_knee_mps | 0.003 | 0.0317924871 | 0.1 |
| longitudinal_slip_knee_mps | 0.01 | 0.7 | 0.7 |
| longitudinal_envelope_shape | 0.45 | 2.5 | 2.5 |
| launch_static_opposing_nm | 0 | 0.00719132081 | 0.02 |
| mu_static_breakaway | derived | 0.00017029073 | derived |

## Launch Estimate

| total opposing Nm | left command | right command | max abs command | pass | launch lock policy |
| ---: | ---: | ---: | ---: | --- | --- |
| 0.007191 | 0.521845 | -0.521845 | 0.521845 | False | diagnostic_only |

## Convergence

Selected result: restart `100` / `coordinate_stability_polish`, objective `0.002017701`, iterations `55`, evaluations `916`, converged `True`.

| restart | seed | objective | iterations | evaluations | converged |
| ---: | --- | ---: | ---: | ---: | --- |
| 1 | standalone_like | 0.001751392 | 260 | 355 | False |
| 2 | brush_high_corner | 0.001751311 | 260 | 382 | False |
| 3 | soft_envelope | 0.001749547 | 260 | 371 | False |
| 4 | fast_gate | 0.001751903 | 260 | 371 | False |
| 99 | full_primary_polish | 0.002018228 | 360 | 526 | False |
| 100 | coordinate_stability_polish | 0.002017701 | 55 | 916 | True |

## Split Metrics

| split | count | baseline RMSE | brush RMSE | improvement | force-domain Stribeck | rational residual | standalone contact | true-traction testbed | Variant C |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.016691 | 0.547262 | 0.027907821180421354 | 0.021083018384621146 | 0.016887630899899024 | 0.01956954005783589 | 0.024120243843366647 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.026643 | 0.383175 | 0.04150718685740355 | 0.03282316561347566 | 0.02663246465575232 | 0.03545970779268185 | 0.033092937468878494 |
| open_floor_validation_only | 14542 | 0.016931 | 0.006934 | 0.590420 | 0.01139376781450519 | 0.011206860684196811 | 0.007711279672300156 | 0.009527487919849564 | 0.014417143137851248 |
| diag_validation_only | 11108 | 0.084621 | 0.028555 | 0.662556 | 0.08441017836709906 | 0.03848912769383371 | 0.023947153628180864 | 0.025684577398624526 | 0.03876675590868194 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.022122 | 0.568493 | 0.048819706958314515 | 0.027490242677710965 | 0.020687366391557327 | 0.027296975430867794 | 0.028529439179585198 |
| validation_non_authoritative | 71263 | 0.050234 | 0.023379 | 0.534603 |  | 0.029680153603611304 | 0.022326615396330518 | 0.028675902743862298 | 0.03034172395599833 |

## Selected Logs

| run | split | count | baseline RMSE | brush RMSE | force-domain | rational | standalone | true-traction | Variant C |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.011244 | 0.02095614161732105 | 0.020127808869544805 | 0.011589046852084053 | 0.016964678124879697 | 0.028076443199331575 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.009480 | 0.015366476488899375 | 0.015838560861422477 | 0.00983796163679149 | 0.011275094881834886 | 0.01934116142597289 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.012139 | 0.01348701867968651 | 0.013202689482483838 | 0.012330360089918665 | 0.012159883749152972 | 0.014402919863369421 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.013024 | 0.033937937440801695 | 0.016861102635710126 | 0.01230667486040885 | 0.01511997085818717 | 0.018608488843215733 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.015952 | 0.027283610538090263 | 0.017657454789089182 | 0.016517464699137962 | 0.016411486644547828 | 0.023754464601601764 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.020560 | 0.03430247806398148 | 0.026878339329649562 | 0.020650381351574418 | 0.02694791897779314 | 0.02774307957781035 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.022129 | 0.0356483649477145 | 0.027977442201207736 | 0.02238198379643325 | 0.02831442123611043 | 0.02933811998534102 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.034097 | 0.056141320109295866 | 0.04248692457048695 | 0.0342756822673276 | 0.044026014881039896 | 0.04131229724660652 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.028347 | 0.08503230164347471 | 0.038712164739976 | 0.023743327476242724 | 0.026234051082609632 | 0.03897045903766876 |

## Risk Slices

| slice | count | baseline RMSE | brush RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.027122 | 0.542344 |
| in_place_scrub | 19704 | 0.073503 | 0.030731 | 0.581904 |
| slow_forward_turn | 19582 | 0.053206 | 0.027123 | 0.490223 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066547 | 0.298606 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.049456 | 0.305336 |
| fast_forward | 0 |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.010993 | 0.543845 |
| limiter_active | 31216 | 0.074601 | 0.035728 | 0.521080 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.042664 | 0.329958 |
| may4_latest_logs | 5217 | 0.032158 | 0.010681 | 0.667856 |

## Common Range Metrics

These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.

| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.027122 | 0.019018 | 0.012160 |
| in_place_scrub | 19704 | 0.073503 | 0.030731 | 0.023244 | 0.017534 |
| slow_forward_turn | 19582 | 0.053206 | 0.027123 | 0.018128 | 0.011899 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066547 | 0.032094 | 0.019817 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.049456 | 0.038043 | 0.023144 |
| fast_forward | 0 |  |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.010993 | 0.007131 | 0.004402 |
| limiter_active | 31216 | 0.074601 | 0.035728 | 0.028083 | 0.024134 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.042664 | 0.034623 | 0.030811 |
| may4_latest_logs | 5217 | 0.032158 | 0.010681 | 0.008142 | 0.006752 |

## Assessment

Physical sanity: `qualified_true`. The selected equation shape is continuous, odd in local slip velocity, uses a rational zero-to-speed gate, and forces longitudinal/lateral patch demands to compete inside a smooth yield envelope. It does not use UKF state-vector fields, command/request/preprojection traction selectors, or an old-plus-residual runtime branch.

Coefficient sanity caveat: `mu_peak, alpha_knee, long_speed_floor_mps, longitudinal_slip_knee_mps, longitudinal_envelope_shape` hit optimizer bounds. Treat the family as viable for comparison, but not as an identified production calibration without tighter targeted data or a narrower prior on the envelope/speed-floor terms.

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
