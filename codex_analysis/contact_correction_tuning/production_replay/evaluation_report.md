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
| all_included | 2994884 | 0.212190004 | 0.204863983 | -0.007326021 | -3.452576% | 0.318353900 |
| open_floor_only | 2362926 | 0.148593814 | 0.142974849 | -0.005618966 | -3.781426% | 0.360982951 |
| competition_only | 631958 | 0.361683931 | 0.349945080 | -0.011738851 | -3.245610% | 0.158961513 |
| fit_authoritative_open_floor | 1182997 | 0.150058239 | 0.142405161 | -0.007653078 | -5.100072% | 0.335346582 |
| fit_downweighted_open_floor | 779760 | 0.174304915 | 0.169765112 | -0.004539804 | -2.604518% | 0.349095876 |
| validation_only_open_floor | 361987 | 0.068655577 | 0.068568535 | -0.000087042 | -0.126780% | 0.427197662 |
| family:competition_aux | 358483 | 0.319136752 | 0.304239349 | -0.014897403 | -4.668031% | 0.226356619 |
| family:competition_diag | 273475 | 0.410836314 | 0.402064371 | -0.008771943 | -2.135143% | 0.070617058 |
| family:open_floor | 2362926 | 0.148593814 | 0.142974849 | -0.005618966 | -3.781426% | 0.360982951 |

## Motion Split

| Family | Motion | Samples | Old RMSE | New RMSE | Delta | Patch RMSE Nm |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| all | in_place_yaw | 493183 | 0.358457371 | 0.349400520 | -0.009056851 | 0.004739230 |
| all | moving_yaw | 604633 | 0.259057883 | 0.248711818 | -0.010346065 | 0.006506257 |
| all | mostly_forward | 1195941 | 0.104577079 | 0.097789369 | -0.006787710 | 0.002916990 |
| all | low_motion_commanded | 701127 | 0.159410197 | 0.154089578 | -0.005320619 | 0.002218114 |
| competition_aux | in_place_yaw | 57801 | 0.541653010 | 0.518901302 | -0.022751708 | 0.006079489 |
| competition_aux | moving_yaw | 56111 | 0.434160993 | 0.413164771 | -0.020996222 | 0.005699999 |
| competition_aux | mostly_forward | 110260 | 0.148098947 | 0.138409204 | -0.009689743 | 0.002397995 |
| competition_aux | low_motion_commanded | 134311 | 0.220963246 | 0.210080221 | -0.010883025 | 0.003073058 |
| competition_diag | in_place_yaw | 133123 | 0.502733160 | 0.493074073 | -0.009659088 | 0.002672398 |
| competition_diag | moving_yaw | 51419 | 0.415869536 | 0.402718983 | -0.013150553 | 0.003152228 |
| competition_diag | mostly_forward | 39023 | 0.093373410 | 0.090650738 | -0.002722672 | 0.001006287 |
| competition_diag | low_motion_commanded | 49910 | 0.256366697 | 0.252565807 | -0.003800890 | 0.000943589 |
| open_floor | in_place_yaw | 302259 | 0.205513948 | 0.201559037 | -0.003954911 | 0.005141421 |
| open_floor | moving_yaw | 497103 | 0.206063378 | 0.197974598 | -0.008088781 | 0.006840546 |
| open_floor | mostly_forward | 1046658 | 0.099300462 | 0.092748045 | -0.006552417 | 0.003013122 |
| open_floor | low_motion_commanded | 516906 | 0.124240335 | 0.120742973 | -0.003497362 | 0.002033152 |

## High-Level Per-Run Result

| Run family | Improved runs | Worsened runs |
| --- | ---: | ---: |
| all | 61 | 5 |
| competition_aux | 13 | 0 |
| competition_diag | 4 | 0 |
| open_floor | 44 | 5 |

## Bin Direction

Signed velocity/yaw bins with at least 80 samples: 713. Improved bins: 604; worsened bins: 109. Full rows are in bin_rmse.csv.

## Baseline Comparison

All-included old/current RMSE in this direct replay is 0.212190004 rad/s versus the prior expanded current baseline 0.272693052 rad/s. New corrected is 0.204863983 rad/s.
Open-floor old/current RMSE in this direct replay is 0.148593814 rad/s versus prior open-floor current 0.258347167 rad/s.
Competition old/current RMSE in this direct replay is 0.361683931 rad/s versus prior competition current 0.308298322 rad/s.
Fit-authoritative open-floor runs contribute 1182997 samples: old 0.150058239 rad/s, new 0.142405161 rad/s.

The earlier fitted/scalar and surface-corrected baselines remain diagnostic comparisons, not production behavior: scalar four-run current 0.226296125 rad/s, scalar fit 0.161693351 rad/s, expanded surface-corrected all/open/competition 0.248282427/0.237744117/0.258216285 rad/s, and validation old/current to surface 0.302028502 -> 0.263993314 rad/s.

## Recommendation

Use the aggregate and motion-split deltas to decide whether the conservative gain should remain. A materially useful coefficient should reduce both in-place yaw and moving-yaw RMSE without broad run/bin regressions; otherwise it should be tuned or disabled pending cleaner targeted runs.
