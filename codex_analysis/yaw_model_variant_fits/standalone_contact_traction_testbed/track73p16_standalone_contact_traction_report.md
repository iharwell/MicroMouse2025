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
| track_width_m | 0.073160 |
| v_dyn_k_mps | 0.160000 |
| v_static_k_mps | 0.080000 |
| Vf_static_k_mps | 0.100000 |
| drive_scale | 0.107125176 |
| lateral_dynamic_mu | 0.859923691 |
| lateral_static_yield_mu | 0.196144372 |
| longitudinal_dynamic_mu | 0.220894015 |

## +1 rad/s In-Place Launch

| opposing scrub Nm | left command | right command | max abs command | gate |
| ---: | ---: | ---: | ---: | --- |
| 0.007478 | 0.647586 | -0.647586 | 0.647586 | pass |

## Split Metrics

| split | count | baseline RMSE | standalone RMSE | improvement | Variant C ref | prior force-level ref | force-domain Stribeck ref | rational reserve ref | rational residual ref |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.016677 | 0.547644 | 0.024120243843366647 | 0.01950812151012949 | 0.027907821180421354 | 0.02577081816016731 | 0.021083018384621146 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.027767 | 0.357153 | 0.033092937468878494 | 0.035267314697039254 | 0.04150718685740355 | 0.03502542782372301 | 0.03282316561347566 |
| open_floor_validation_only | 14542 | 0.016931 | 0.007658 | 0.547715 | 0.014417143137851248 | 0.009293772489079357 | 0.01139376781450519 | 0.018486286258366312 | 0.011206860684196811 |
| diag_validation_only | 11108 | 0.084621 | 0.021038 | 0.751389 | 0.03876675590868194 | 0.02568541911037413 | 0.08441017836709906 | 0.05866585738673607 | 0.03848912769383371 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.019535 | 0.618948 | 0.028529439179585198 | 0.027446247485824863 | 0.048819706958314515 | 0.04380279520077506 | 0.027490242677710965 |
| validation_non_authoritative | 71263 | 0.050234 | 0.022260 | 0.556872 | 0.03034172395599833 | 0.028585353044834484 |  | 0.03913695258701344 | 0.029680153603611304 |

## Risk Slices

| slice | count | baseline RMSE | standalone RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| high_speed_abs_vf_ge_0p7 | 144 | 0.088793 | 0.065624 | 0.260932 |
| low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5 | 20694 | 0.072832 | 0.031489 | 0.567646 |
| limiter_active | 31216 | 0.074601 | 0.033862 | 0.546094 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.046084 | 0.276237 |
| open_floor_all | 93024 | 0.036894 | 0.020222 | 0.451889 |
| diag_all | 11108 | 0.084621 | 0.021038 | 0.751389 |
| aux_all | 14448 | 0.051266 | 0.019535 | 0.618948 |

## Selected Logs

| run | split | count | baseline RMSE | standalone RMSE | Variant C ref | prior force-level ref | force-domain ref |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.011393 | 0.028076443199331575 | 0.01647254388265974 | 0.02095614161732105 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.009652 | 0.01934116142597289 | 0.010900274495035337 | 0.015366476488899375 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.012243 | 0.014402919863369421 | 0.012118358812930297 | 0.01348701867968651 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.011405 | 0.018608488843215733 | 0.01467238488490882 | 0.033937937440801695 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.015489 | 0.023754464601601764 | 0.016287485899994517 | 0.027283610538090263 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.020339 | 0.02774307957781035 | 0.026919442727042133 | 0.03430247806398148 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.022720 | 0.02933811998534102 | 0.02826572451210337 | 0.0356483649477145 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.036229 | 0.04131229724660652 | 0.04376311762102966 | 0.056141320109295866 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.020775 | 0.03897045903766876 | 0.02623535386349149 | 0.08503230164347471 |

## Candidate Summary

The top scored candidates are in `candidate_scores.csv`. The best pure dynamic patch law fit the broad envelope well but failed the launch command gate; the selected static-yield patch law keeps nearly the same broad-envelope error while lifting the in-place command estimate into the requested hard-launch band.

## Recommendation

Viable as a standalone baseline contact-patch law. The production shape implied by this pass is a direct patch-force accumulation in `PlantModel`: distribute physical drive force to patches, add dynamic rational slip force and low-speed static yaw-yield force from patch kinematics/load, then accumulate `sum(f*Fr - r*Ff)`. Do not install it as `old + residual` or as an old-force correction branch.

## Outputs

- `candidate_scores.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `selected_parameters.csv`
- `in_place_1radps_command.csv`
- `prediction_sample.csv`
