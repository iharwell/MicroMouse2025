# Contact Correction Log Evaluation

Scratch analysis only. No production code or tests were modified.

## Method

Targets use actual sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where rows allow it, encoder-derived forward velocity and wheel speeds, logged drive commands, and timestamps. UKF states and logged UKF gyro bias are not used as targets.

The evaluator mirrors the yaw-relevant PlantModel path twice: old/pre-correction contact force requests, then the new contact-continuum patch-force couple before force projection. The mirror uses the current constants parsed from Vehicle, MotorEncoderDrive, and PlantModel. Fidelity assumptions: lateral velocity is unavailable in these logs and is set to zero, normal load transfer is not reconstructed, and legacy competition CSVs lack saturation/watchdog fields and derive wheel omega from linear velocity.

Windows use the prior contact-continuum extractor gating: adjacent valid ticks, same phase/primitive key, raw-gyro differentiated yaw acceleration under 4000 rad/s^2, active command/motion rows, and old-model residual magnitude under 2 Nm to reject derivative spikes.

## Scope

Candidate logs scanned: 66. Runs with samples: 66. Samples: 2994884.

## Aggregate RMSE

| Dataset | Samples | Old RMSE | New RMSE | Delta | Relative delta | Worsened sample fraction |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| all_included | 2994884 | 0.212190004 | 0.212221952 | 0.000031948 | 0.015056% | 0.690150270 |
| open_floor_only | 2362926 | 0.148593814 | 0.148620089 | 0.000026275 | 0.017682% | 0.649151095 |
| competition_only | 631958 | 0.361683931 | 0.361732393 | 0.000048462 | 0.013399% | 0.843448457 |
| fit_authoritative_open_floor | 1182997 | 0.150058239 | 0.150094028 | 0.000035789 | 0.023850% | 0.674173307 |
| fit_downweighted_open_floor | 779760 | 0.174304915 | 0.174325898 | 0.000020983 | 0.012038% | 0.664605776 |
| validation_only_open_floor | 361987 | 0.068655577 | 0.068656215 | 0.000000639 | 0.000930% | 0.578122971 |
| family:competition_aux | 358483 | 0.319136752 | 0.319195009 | 0.000058257 | 0.018255% | 0.776100959 |
| family:competition_diag | 273475 | 0.410836314 | 0.410875582 | 0.000039269 | 0.009558% | 0.931730506 |
| family:open_floor | 2362926 | 0.148593814 | 0.148620089 | 0.000026275 | 0.017682% | 0.649151095 |

## Motion Split

| Family | Motion | Samples | Old RMSE | New RMSE | Delta | Patch RMSE Nm |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| all | in_place_yaw | 493183 | 0.358457371 | 0.358495334 | 0.000037963 | 0.000021284 |
| all | moving_yaw | 604633 | 0.259057883 | 0.259106453 | 0.000048570 | 0.000030250 |
| all | mostly_forward | 1195941 | 0.104577079 | 0.104605575 | 0.000028496 | 0.000013020 |
| all | low_motion_commanded | 701127 | 0.159410197 | 0.159431842 | 0.000021646 | 0.000009532 |
| competition_aux | in_place_yaw | 57801 | 0.541653010 | 0.541735971 | 0.000082961 | 0.000022292 |
| competition_aux | moving_yaw | 56111 | 0.434160993 | 0.434256727 | 0.000095734 | 0.000025710 |
| competition_aux | mostly_forward | 110260 | 0.148098947 | 0.148140683 | 0.000041736 | 0.000010423 |
| competition_aux | low_motion_commanded | 134311 | 0.220963246 | 0.220998754 | 0.000035508 | 0.000009886 |
| competition_diag | in_place_yaw | 133123 | 0.502733160 | 0.502775615 | 0.000042455 | 0.000012206 |
| competition_diag | moving_yaw | 51419 | 0.415869536 | 0.415929364 | 0.000059828 | 0.000014769 |
| competition_diag | mostly_forward | 39023 | 0.093373410 | 0.093387399 | 0.000013989 | 0.000005024 |
| competition_diag | low_motion_commanded | 49910 | 0.256366697 | 0.256385483 | 0.000018785 | 0.000004672 |
| open_floor | in_place_yaw | 302259 | 0.205513948 | 0.205534435 | 0.000020488 | 0.000024052 |
| open_floor | moving_yaw | 497103 | 0.206063378 | 0.206102390 | 0.000039012 | 0.000031873 |
| open_floor | mostly_forward | 1046658 | 0.099300462 | 0.099327705 | 0.000027243 | 0.000013465 |
| open_floor | low_motion_commanded | 516906 | 0.124240335 | 0.124257854 | 0.000017519 | 0.000009785 |

## High-Level Per-Run Result

| Run family | Improved runs | Worsened runs |
| --- | ---: | ---: |
| all | 5 | 61 |
| competition_aux | 0 | 13 |
| competition_diag | 0 | 4 |
| open_floor | 5 | 44 |

## Bin Direction

Signed velocity/yaw bins with at least 80 samples: 713. Improved bins: 85; worsened bins: 628. Full rows are in bin_rmse.csv.

## Baseline Comparison

All-included old/current RMSE in this direct replay is 0.212190004 rad/s versus the prior expanded current baseline 0.272693052 rad/s. New corrected is 0.212221952 rad/s.
Open-floor old/current RMSE in this direct replay is 0.148593814 rad/s versus prior open-floor current 0.258347167 rad/s.
Competition old/current RMSE in this direct replay is 0.361683931 rad/s versus prior competition current 0.308298322 rad/s.
Fit-authoritative open-floor runs contribute 1182997 samples: old 0.150058239 rad/s, new 0.150094028 rad/s.

The earlier fitted/scalar and surface-corrected baselines remain diagnostic comparisons, not production behavior: scalar four-run current 0.226296125 rad/s, scalar fit 0.161693351 rad/s, expanded surface-corrected all/open/competition 0.248282427/0.237744117/0.258216285 rad/s, and validation old/current to surface 0.302028502 -> 0.263993314 rad/s.

## Recommendation

Use the aggregate and motion-split deltas to decide whether the conservative gain should remain. A materially useful coefficient should reduce both in-place yaw and moving-yaw RMSE without broad run/bin regressions; otherwise it should be tuned or disabled pending cleaner targeted runs.
