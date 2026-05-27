# Standalone Contact-Patch Traction Testbed

Analysis-only output. Production code, build metadata, and tests were not edited.

## Scope

The selected law is standalone: it predicts `observed_yaw_moment_nm` directly from contact-patch relative velocity, normal load, drive force, and contact geometry. It does not use old PlantModel contact-force outputs, old contact gains, old yaw moment, or a residual target as a runtime component. Current baseline, rational residual/reference models, Variant C, and force-domain Stribeck are reported only as external references.

## Selected Law

For each patch `i` at position `(r_i, f_i)`, with `r>0` right and `f>0` forward:

`F_drive_f_i = drive_scale * F_drive_side / 2`

`F_dyn_r_i = lateral_dynamic_mu * N_i * v_rel_r_i / (abs(v_rel_r_i) + v_dyn_k)`

`F_static_r_i = lateral_static_yield_mu * N_i * sign(v_rel_r_i) * G_static_i`

`G_static_i = 1 / (1 + (sqrt(v_rel_f_i^2 + v_rel_r_i^2) / v_static_k)^2) * 1 / (1 + (abs(Vf) / Vf_static_k)^2)`

`F_dyn_f_i = longitudinal_dynamic_mu * N_i * v_rel_f_i / (abs(v_rel_f_i) + v_dyn_k)`

`F_f_i = F_drive_f_i + F_dyn_f_i` and `F_r_i = F_dyn_r_i + F_static_r_i`

`M_yaw = sum_i(f_i * F_r_i - r_i * F_f_i)`

The selected form uses `sqrt`, `abs`, rational schedules, and sign/clamp-style branching only. It has no trigonometry, `exp`, `tanh`, lookup table, command selector, old-force branch, or residual-additive branch.

## Tuned Constants

| parameter | value |
| --- | ---: |
| family | yaw_static_yield_patch |
| track_width_m | 0.084635 |
| v_dyn_k_mps | 0.160000 |
| v_static_k_mps | 0.080000 |
| Vf_static_k_mps | 0.200000 |
| drive_scale | 0.109522488 |
| lateral_dynamic_mu | 0.692024409 |
| lateral_static_yield_mu | 0.149030931 |
| longitudinal_dynamic_mu | 0.273966177 |

## +1 rad/s In-Place Launch

| opposing scrub Nm | left command | right command | max abs command | gate | launch lock policy |
| ---: | ---: | ---: | ---: | --- | --- |
| 0.005775 | 0.484037 | -0.484037 | 0.484037 | fail | diagnostic_only |

## Split Metrics

| split | count | baseline RMSE | standalone RMSE | improvement | Variant C ref | prior force-level ref | force-domain Stribeck ref | rational reserve ref | rational residual ref |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.016865 | 0.542529 | 0.024120243843366647 | 0.01956954005783589 | 0.027907821180421354 | 0.02577081816016731 | 0.021083018384621146 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.026675 | 0.382440 | 0.033092937468878494 | 0.03545970779268185 | 0.04150718685740355 | 0.03502542782372301 | 0.03282316561347566 |
| open_floor_validation_only | 14542 | 0.016931 | 0.007451 | 0.559892 | 0.014417143137851248 | 0.009527487919849564 | 0.01139376781450519 | 0.018486286258366312 | 0.011206860684196811 |
| diag_validation_only | 11108 | 0.084621 | 0.023613 | 0.720953 | 0.03876675590868194 | 0.025684577398624526 | 0.08441017836709906 | 0.05866585738673607 | 0.03848912769383371 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.020441 | 0.601281 | 0.028529439179585198 | 0.027296975430867794 | 0.048819706958314515 | 0.04380279520077506 | 0.027490242677710965 |
| validation_non_authoritative | 71263 | 0.050234 | 0.022229 | 0.557488 | 0.03034172395599833 | 0.028675902743862298 |  | 0.03913695258701344 | 0.029680153603611304 |

## Risk Slices

| slice | count | baseline RMSE | standalone RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.025340 | 0.572409 |
| in_place_scrub | 19704 | 0.073503 | 0.027090 | 0.631439 |
| slow_forward_turn | 19582 | 0.053206 | 0.027234 | 0.488148 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066565 | 0.298425 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.046646 | 0.344794 |
| straightish_forward | 21746 | 0.024100 | 0.011191 | 0.535634 |
| limiter_active | 31216 | 0.074601 | 0.033824 | 0.546604 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.043842 | 0.311447 |
| may4_latest_logs | 5217 | 0.032158 | 0.010715 | 0.666799 |
| open_floor_all | 93024 | 0.036894 | 0.019793 | 0.463534 |
| diag_all | 11108 | 0.084621 | 0.023613 | 0.720953 |
| aux_all | 14448 | 0.051266 | 0.020441 | 0.601281 |

## Common Range Metrics

Shared operating-range definitions are written to `common_range_metrics.csv`; the former `0.7 m/s high-speed` label is intentionally not used.

| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.025340 | 0.018130 | 0.012419 |
| in_place_scrub | 19704 | 0.073503 | 0.027090 | 0.020846 | 0.016848 |
| slow_forward_turn | 19582 | 0.053206 | 0.027234 | 0.018074 | 0.011748 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066565 | 0.031204 | 0.018269 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.046646 | 0.033345 | 0.020490 |
| fast_forward | 0 |  |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.011191 | 0.007602 | 0.005371 |
| limiter_active | 31216 | 0.074601 | 0.033824 | 0.026254 | 0.022275 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.043842 | 0.035629 | 0.031820 |
| may4_latest_logs | 5217 | 0.032158 | 0.010715 | 0.007965 | 0.006175 |

## Selected Logs

| run | split | count | baseline RMSE | standalone RMSE | Variant C ref | prior force-level ref | force-domain ref |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.011312 | 0.028076443199331575 | 0.016964678124879697 | 0.02095614161732105 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.009433 | 0.01934116142597289 | 0.011275094881834886 | 0.015366476488899375 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.012262 | 0.014402919863369421 | 0.012159883749152972 | 0.01348701867968651 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.012167 | 0.018608488843215733 | 0.01511997085818717 | 0.033937937440801695 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.016445 | 0.023754464601601764 | 0.016411486644547828 | 0.027283610538090263 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.020618 | 0.02774307957781035 | 0.02694791897779314 | 0.03430247806398148 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.022285 | 0.02933811998534102 | 0.02831442123611043 | 0.0356483649477145 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.034341 | 0.04131229724660652 | 0.044026014881039896 | 0.056141320109295866 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.023404 | 0.03897045903766876 | 0.026234051082609632 | 0.08503230164347471 |

## Candidate Summary

The top scored candidates are in `candidate_scores.csv`. This run is unconstrained by launch command: the in-place yaw launch estimate is reported as a diagnostic only and is not used as a score term or candidate-selection gate.

## Recommendation

Viable as a standalone baseline contact-patch law. The production shape implied by this pass is a direct patch-force accumulation in `PlantModel`: distribute physical drive force to patches, add dynamic rational slip force and low-speed static yaw-yield force from patch kinematics/load, then accumulate `sum(f*Fr - r*Ff)`. Do not install it as `old + residual` or as an old-force correction branch.

## Outputs

- `candidate_scores.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `common_range_metrics.csv`
- `selected_parameters.csv`
- `in_place_1radps_command.csv`
- `prediction_sample.csv`
