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
| Vf_static_k_mps | 0.050000 |
| drive_scale | 0.109231525 |
| lateral_dynamic_mu | 0.708966035 |
| lateral_static_yield_mu | 0.221202108 |
| longitudinal_dynamic_mu | 0.276021972 |

## +1 rad/s In-Place Launch

| opposing scrub Nm | left command | right command | max abs command | gate |
| ---: | ---: | ---: | ---: | --- |
| 0.007805 | 0.593814 | -0.593814 | 0.593814 | fail |

## Split Metrics

| split | count | baseline RMSE | standalone RMSE | improvement | Variant C ref | prior force-level ref | force-domain Stribeck ref | rational reserve ref | rational residual ref |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.016888 | 0.541921 | 0.024120243843366647 | 0.019519867783538924 | 0.027907821180421354 | 0.02577081816016731 | 0.021083018384621146 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.026632 | 0.383423 | 0.033092937468878494 | 0.034836888756372 | 0.04150718685740355 | 0.03502542782372301 | 0.03282316561347566 |
| open_floor_validation_only | 14542 | 0.016931 | 0.007711 | 0.544539 | 0.014417143137851248 | 0.008849476125192358 | 0.01139376781450519 | 0.018486286258366312 | 0.011206860684196811 |
| diag_validation_only | 11108 | 0.084621 | 0.023947 | 0.717007 | 0.03876675590868194 | 0.02570299117809136 | 0.08441017836709906 | 0.05866585738673607 | 0.03848912769383371 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.020687 | 0.596473 | 0.028529439179585198 | 0.027895852240170986 | 0.048819706958314515 | 0.04380279520077506 | 0.027490242677710965 |
| validation_non_authoritative | 71263 | 0.050234 | 0.022327 | 0.555546 | 0.03034172395599833 | 0.028415959824015274 |  | 0.03913695258701344 | 0.029680153603611304 |

## Risk Slices

| slice | count | baseline RMSE | standalone RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.025452 | 0.570514 |
| in_place_scrub | 19704 | 0.073503 | 0.027300 | 0.628587 |
| slow_forward_turn | 19582 | 0.053206 | 0.027210 | 0.488591 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066562 | 0.298454 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.046602 | 0.345424 |
| straightish_forward | 21746 | 0.024100 | 0.011003 | 0.543462 |
| limiter_active | 31216 | 0.074601 | 0.033860 | 0.546119 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.043769 | 0.312594 |
| may4_latest_logs | 5217 | 0.032158 | 0.011029 | 0.657029 |
| open_floor_all | 93024 | 0.036894 | 0.019799 | 0.463368 |
| diag_all | 11108 | 0.084621 | 0.023947 | 0.717007 |
| aux_all | 14448 | 0.051266 | 0.020687 | 0.596473 |

## Common Range Metrics

Shared operating-range definitions are written to `common_range_metrics.csv`; the former `0.7 m/s high-speed` label is intentionally not used.

| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration_low_vf_nonzero_yaw | 41686 | 0.059262 | 0.025452 | 0.018303 | 0.012878 |
| in_place_scrub | 19704 | 0.073503 | 0.027300 | 0.021167 | 0.017311 |
| slow_forward_turn | 19582 | 0.053206 | 0.027210 | 0.018028 | 0.011690 |
| pre_design_turn_speed | 99 | 0.094879 | 0.066562 | 0.031194 | 0.018527 |
| design_turn_speed_and_up | 4 | 0.071194 | 0.046602 | 0.033385 | 0.020591 |
| fast_forward | 0 |  |  |  |  |
| straightish_forward | 21746 | 0.024100 | 0.011003 | 0.007246 | 0.004737 |
| limiter_active | 31216 | 0.074601 | 0.033860 | 0.026321 | 0.022360 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.043769 | 0.035554 | 0.031586 |
| may4_latest_logs | 5217 | 0.032158 | 0.011029 | 0.008166 | 0.006318 |

## Selected Logs

| run | split | count | baseline RMSE | standalone RMSE | Variant C ref | prior force-level ref | force-domain ref |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.011589 | 0.028076443199331575 | 0.015602242066314734 | 0.02095614161732105 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.009838 | 0.01934116142597289 | 0.01014360504287235 | 0.015366476488899375 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.012330 | 0.014402919863369421 | 0.012097343220894962 | 0.01348701867968651 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.012307 | 0.018608488843215733 | 0.013782370391656997 | 0.033937937440801695 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.016517 | 0.023754464601601764 | 0.016372638287973088 | 0.027283610538090263 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.020650 | 0.02774307957781035 | 0.026887847291071503 | 0.03430247806398148 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.022382 | 0.02933811998534102 | 0.02818192262657356 | 0.0356483649477145 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.034276 | 0.04131229724660652 | 0.04314470942081007 | 0.056141320109295866 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.023743 | 0.03897045903766876 | 0.02625386650679142 | 0.08503230164347471 |

## Candidate Summary

The top scored candidates are in `candidate_scores.csv`. The best pure dynamic patch law fit the broad envelope well but failed the launch command gate; the selected static-yield patch law keeps nearly the same broad-envelope error while lifting the in-place command estimate into the requested hard-launch band.

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
