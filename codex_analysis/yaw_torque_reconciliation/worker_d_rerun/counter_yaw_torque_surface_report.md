# Counter-Yaw Torque Surface Audit

Worker C's scalar track/yaw-inertia fit is treated here as a diagnostic baseline only. This audit estimates a residual torque surface from historical sensor data using current authoritative constants.

Sign convention: `+Yaw` is clockwise. Observed yaw acceleration is a 5-control-sample slope from raw gyro after opening stationary-bias subtraction. The current-constant commanded yaw moment is computed from drive commands and encoder wheel speeds using the motor model, launch torque, rolling torque, current track width, and current yaw denominator. Residual counter-yaw torque is `commanded_yaw_moment - yaw_denominator * observed_yaw_accel`. `opposing_yaw_nm = residual_counter_yaw_nm * sign(gyro_yaw_rate)`, so positive values indicate torque that resists the current yaw rotation.

No UKF targets are used. The script intentionally avoids `ukf_state_*`, `ukf_state_bgz_radps`, `measured_linear_speed_mps`, `measured_angular_speed_radps`, `yaw_consistency_lp_radps`, `yaw_window_mismatch_rad`, and `nhc_*` fields.

Worker C baseline reproduced separately: 104017 one-step samples, current scalar-model RMSE 0.226296125 rad/s, proposed scalar-model RMSE 0.161693351 rad/s, proposed track 0.104595474 m, proposed yaw inertia 0.000603133 kg m^2, yaw damping 0. This remains a diagnostic baseline, not the final recommendation.

## Commands

```powershell
python codex_analysis\yaw_fit\fit_yaw_plant_params.py --report codex_analysis\yaw_fit_validation\worker_c_rerun_report.md
python codex_analysis\yaw_fit_validation\counter_yaw_torque_surface.py
```

## Run Coverage

| Run | Worker C | Rows | Kept | Samples | In-place | Moving | Cut note |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `mmlog_decode_2026-04-21_05-32-06` |  | 234369 | 167763 | 13084 | 13084 | 0 | commanded stall/lift-like window, terminal fault upper bound |
| `mmlog_decode_2026-04-20_08-38-39` | yes | 217867 | 217867 | 33111 | 33111 | 0 |  |
| `mmlog_decode_2026-05-04_20-35-47` |  | 145644 | 110809 | 21945 | 21945 | 0 | commanded stall/lift-like window, terminal fault upper bound |
| `mmlog_decode_2026-05-04_16-57-53` |  | 208375 | 146621 | 4981 | 4981 | 0 | commanded stall/lift-like window, terminal fault upper bound |
| `mmlog_decode_2026-04-21_05-59-46` |  | 100419 | 77132 | 13330 | 13330 | 0 | commanded stall/lift-like window, terminal fault upper bound |
| `mmlog_decode_2026-04-21_00-16-10` |  | 109329 | 109131 | 44418 | 10923 | 33495 | terminal fault upper bound |
| `mmlog_decode_2026-04-21_01-09-34` | yes | 101013 | 100815 | 29230 | 9466 | 19764 | terminal fault upper bound |
| `mmlog_decode_2026-04-20_02-33-07` |  | 27971 | 27772 | 18253 | 0 | 18253 | terminal fault upper bound |
| `mmlog_decode_2026-04-20_12-10-58` | yes | 86556 | 86358 | 24745 | 8443 | 16302 | terminal fault upper bound |
| `mmlog_decode_2026-04-14_04-43-48` |  | 116754 | 116556 | 12671 | 6224 | 6447 | terminal fault upper bound |
| `mmlog_decode_2026-04-20_10-22-09` | yes | 99917 | 99719 | 9165 | 9165 | 0 | terminal fault upper bound |

## RMSE

| Subset | Samples | Current RMSE rad/s | Surface-corrected RMSE rad/s | Surface bins |
| --- | ---: | ---: | ---: | ---: |
| `all_samples_global_surface` | 224933 | 37.869354668 | 28.768104245 | 22 |
| `in_place_global_surface` | 130672 | 35.519763474 | 29.090852260 | 22 |
| `moving_global_surface` | 94261 | 40.903913800 | 28.314601288 | 22 |
| `all_samples_aligned_surface` | 224933 | 37.869354668 | 40.951899711 | 14 |
| `in_place_aligned_surface` | 130672 | 35.519763474 | 39.786900261 | 14 |
| `moving_aligned_surface` | 94261 | 40.903913800 | 42.514143054 | 14 |
| `in_place_own_surface` | 130672 | 35.519763474 | 40.125596994 | 9 |
| `moving_own_surface` | 94261 | 40.903913800 | 41.999033384 | 6 |
| `aligned_samples_aligned_surface` | 75508 | 34.246127067 | 26.554454978 | 14 |
| `leave_run_out:mmlog_decode_2026-04-21_05-32-06` | 13084 | 15.625778062 | 25.635173241 | 14 |
| `leave_run_out:mmlog_decode_2026-04-20_08-38-39` | 33111 | 37.069042385 | 48.260716470 | 14 |
| `leave_run_out:mmlog_decode_2026-05-04_20-35-47` | 21945 | 23.456539660 | 13.886935144 | 14 |
| `leave_run_out:mmlog_decode_2026-05-04_16-57-53` | 4981 | 18.572148773 | 5.214539090 | 14 |
| `leave_run_out:mmlog_decode_2026-04-21_05-59-46` | 13330 | 16.806061834 | 25.903012191 | 14 |
| `leave_run_out:mmlog_decode_2026-04-21_00-16-10` | 44418 | 36.350767400 | 39.590922201 | 12 |
| `leave_run_out:mmlog_decode_2026-04-21_01-09-34` | 29230 | 27.171008279 | 32.751856470 | 12 |
| `leave_run_out:mmlog_decode_2026-04-20_02-33-07` | 18253 | 63.442575486 | 63.114723062 | 13 |
| `leave_run_out:mmlog_decode_2026-04-20_12-10-58` | 24745 | 40.345720597 | 43.579748613 | 12 |
| `leave_run_out:mmlog_decode_2026-04-14_04-43-48` | 12671 | 38.986588257 | 42.557377907 | 14 |
| `leave_run_out:mmlog_decode_2026-04-20_10-22-09` | 9165 | 68.532016303 | 68.096236833 | 14 |

## Discrete Torque Surface

Rows with fewer than 80 samples are retained in the CSV for coverage inspection, but bins below that count are not used by the RMSE correction surface. The aligned rows are the cleaner resistance estimate because commanded yaw moment has the same sign as current yaw rate; all rows include braking and phase-transition dynamics.

| Selection | Regime | Forward m/s | abs yaw rad/s | Samples | Median opposing Nm | Mean opposing Nm | P20 | P80 | Positive fraction |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `aligned` | `in_place` | `-0.5..-0.1` | `0.2..1` | 876 | 1.413956544 | 1.402876457 | 0.708146613 | 2.015448741 | 0.998 |
| `aligned` | `in_place` | `-0.5..-0.1` | `1..3` | 970 | 1.042168517 | 1.089892598 | 0.391132241 | 1.681381669 | 0.985 |
| `aligned` | `in_place` | `-0.5..-0.1` | `3..6` | 128 | 0.453303230 | 0.529453595 | 0.184731336 | 0.833990526 | 1.000 |
| `aligned` | `in_place` | `-0.5..-0.1` | `6..10` | 22 | 0.331724172 | 0.280123062 | 0.106713558 | 0.423618138 | 1.000 |
| `aligned` | `in_place` | `-0.5..-0.1` | `10..15` | 1 | 1.451695487 | 1.451695487 | 1.451695487 | 1.451695487 | 1.000 |
| `aligned` | `in_place` | `-0.5..-0.1` | `15..22` | 4 | 0.650876947 | 0.597344283 | 0.485631756 | 0.816122138 | 1.000 |
| `aligned` | `in_place` | `-0.1..0.1` | `0.2..1` | 21724 | 0.977698939 | 1.299800849 | 0.586716917 | 1.363562684 | 0.996 |
| `aligned` | `in_place` | `-0.1..0.1` | `1..3` | 24656 | 0.894728227 | 0.904103104 | 0.311305383 | 1.312764283 | 0.992 |
| `aligned` | `in_place` | `-0.1..0.1` | `3..6` | 6610 | 0.838973718 | 0.776402857 | 0.515017452 | 1.050669309 | 0.998 |
| `aligned` | `in_place` | `-0.1..0.1` | `6..10` | 1626 | 0.359942771 | 0.390213199 | 0.219235146 | 0.483860014 | 0.993 |
| `aligned` | `in_place` | `-0.1..0.1` | `10..15` | 40 | 1.222221521 | 1.140958129 | 0.433112094 | 1.693797181 | 1.000 |
| `aligned` | `in_place` | `-0.1..0.1` | `15..22` | 4 | 0.244569609 | 0.335598589 | 0.232779081 | 0.256360136 | 1.000 |
| `aligned` | `in_place` | `0.1..0.5` | `0.2..1` | 444 | 1.733108036 | 1.840510912 | 1.108564257 | 2.434127394 | 1.000 |
| `aligned` | `in_place` | `0.1..0.5` | `1..3` | 577 | 1.456336604 | 1.590536269 | 0.730090789 | 2.374281737 | 0.995 |
| `aligned` | `in_place` | `0.1..0.5` | `3..6` | 47 | 0.586362987 | 0.793937070 | 0.151149339 | 1.266544941 | 1.000 |
| `aligned` | `in_place` | `0.1..0.5` | `6..10` | 10 | 0.820444162 | 0.838976274 | 0.390391139 | 1.404591498 | 1.000 |
| `aligned` | `in_place` | `0.1..0.5` | `10..15` | 2 | 1.733525796 | 1.733525796 | 1.182119989 | 2.284931604 | 1.000 |
| `aligned` | `moving` | `-0.1..0.1` | `0.2..1` | 4 | 0.799486674 | 0.756315210 | 0.463586830 | 1.135386517 | 1.000 |
| `aligned` | `moving` | `-0.1..0.1` | `1..3` | 4 | 0.914267650 | 0.949440585 | 0.753281785 | 1.075253515 | 1.000 |
| `aligned` | `moving` | `-0.1..0.1` | `3..6` | 2 | 0.675978032 | 0.675978032 | 0.441206820 | 0.910749243 | 1.000 |
| `aligned` | `moving` | `0.1..0.5` | `0.2..1` | 8064 | 0.679134762 | 1.956814892 | 0.343062676 | 4.121454950 | 1.000 |
| `aligned` | `moving` | `0.1..0.5` | `1..3` | 7728 | 0.426933332 | 0.671255602 | 0.238541962 | 0.808407085 | 0.995 |
| `aligned` | `moving` | `0.1..0.5` | `3..6` | 1343 | 0.219235300 | 0.294805144 | 0.053972730 | 0.484320832 | 0.960 |
| `aligned` | `moving` | `0.1..0.5` | `6..10` | 71 | 0.103563083 | 0.203056136 | 0.022945600 | 0.300482053 | 0.944 |
| `aligned` | `moving` | `0.1..0.5` | `10..15` | 2 | 0.128804588 | 0.128804588 | 0.059166100 | 0.198443077 | 1.000 |
| `aligned` | `moving` | `0.5..1.0` | `0.2..1` | 139 | 1.067634365 | 1.297954968 | 0.523686519 | 2.125891545 | 1.000 |
| `aligned` | `moving` | `0.5..1.0` | `1..3` | 172 | 0.472930888 | 0.749889475 | 0.283752012 | 1.006016005 | 1.000 |
| `aligned` | `moving` | `0.5..1.0` | `3..6` | 76 | 0.607526965 | 0.758597834 | 0.151802431 | 1.395607789 | 1.000 |
| `aligned` | `moving` | `0.5..1.0` | `6..10` | 119 | 0.546854803 | 0.593084544 | 0.244700150 | 0.882629529 | 1.000 |
| `aligned` | `moving` | `0.5..1.0` | `10..15` | 1 | 0.300925806 | 0.300925806 | 0.300925806 | 0.300925806 | 1.000 |
| `aligned` | `moving` | `1.0..1.5` | `0.2..1` | 6 | 1.742770276 | 1.711310345 | 1.469750150 | 1.807121233 | 1.000 |
| `aligned` | `moving` | `1.0..1.5` | `1..3` | 20 | 0.961652709 | 0.965599541 | 0.727797055 | 1.138172353 | 1.000 |
| `aligned` | `moving` | `1.0..1.5` | `3..6` | 16 | 0.683021837 | 0.676470223 | 0.307248910 | 0.968369720 | 1.000 |
| `all` | `in_place` | `-0.5..-0.1` | `0.2..1` | 1830 | -0.200569893 | -0.096229967 | -1.683433880 | 1.586810241 | 0.480 |
| `all` | `in_place` | `-0.5..-0.1` | `1..3` | 3427 | -0.799110488 | -0.664768491 | -1.832500627 | 0.639151891 | 0.284 |
| `all` | `in_place` | `-0.5..-0.1` | `3..6` | 669 | -0.716973300 | -0.935059711 | -1.834308905 | 0.007183368 | 0.206 |
| `all` | `in_place` | `-0.5..-0.1` | `6..10` | 573 | -1.901255447 | -2.202361048 | -3.525186285 | -1.143025824 | 0.038 |
| `all` | `in_place` | `-0.5..-0.1` | `10..15` | 465 | -3.926449955 | -3.753607996 | -5.088248809 | -2.187427513 | 0.002 |
| `all` | `in_place` | `-0.5..-0.1` | `15..22` | 153 | -5.251823897 | -5.119046086 | -6.362574891 | -4.170396386 | 0.026 |
| `all` | `in_place` | `-0.1..0.1` | `0.2..1` | 29079 | 0.758599758 | 0.669446434 | -0.382429133 | 1.194735343 | 0.748 |
| `all` | `in_place` | `-0.1..0.1` | `1..3` | 47698 | 0.047214284 | -0.089797856 | -1.166338768 | 1.129802343 | 0.516 |
| `all` | `in_place` | `-0.1..0.1` | `3..6` | 26112 | -0.434150164 | -0.442981830 | -1.008501683 | 0.530210266 | 0.255 |
| `all` | `in_place` | `-0.1..0.1` | `6..10` | 9264 | -1.690448414 | -1.605483600 | -2.581302691 | -0.422128002 | 0.175 |
| `all` | `in_place` | `-0.1..0.1` | `10..15` | 5447 | -2.861572592 | -2.905875444 | -3.830329670 | -1.846851001 | 0.008 |
| `all` | `in_place` | `-0.1..0.1` | `15..22` | 1581 | -3.253147897 | -3.456562750 | -4.167539332 | -2.710750728 | 0.003 |
| `all` | `in_place` | `0.1..0.5` | `0.2..1` | 1081 | -0.794091839 | -0.289546082 | -2.093493008 | 1.755142645 | 0.412 |
| `all` | `in_place` | `0.1..0.5` | `1..3` | 1839 | -1.021895538 | -0.714387157 | -2.223609238 | 1.111379019 | 0.313 |
| `all` | `in_place` | `0.1..0.5` | `3..6` | 550 | -1.768289786 | -1.826103389 | -2.978388616 | -0.690064136 | 0.085 |
| `all` | `in_place` | `0.1..0.5` | `6..10` | 464 | -3.376963352 | -3.253746305 | -4.312807660 | -2.177877137 | 0.024 |
| `all` | `in_place` | `0.1..0.5` | `10..15` | 413 | -4.611061323 | -4.160826644 | -5.398959373 | -2.311195438 | 0.007 |
| `all` | `in_place` | `0.1..0.5` | `15..22` | 27 | -4.993686944 | -4.549360838 | -6.319416475 | -2.926067604 | 0.000 |
| `all` | `moving` | `-0.1..0.1` | `0.2..1` | 27 | -1.263625668 | -0.975506397 | -1.587447399 | -0.590044168 | 0.148 |
| `all` | `moving` | `-0.1..0.1` | `1..3` | 35 | -1.083682726 | -0.847480563 | -1.382515408 | -0.211735130 | 0.143 |
| `all` | `moving` | `-0.1..0.1` | `3..6` | 10 | -1.344112280 | -1.108879933 | -1.903253553 | -0.886560000 | 0.200 |
| `all` | `moving` | `-0.1..0.1` | `6..10` | 6 | -1.882087303 | -2.049949354 | -2.562259086 | -1.867684948 | 0.000 |
| `all` | `moving` | `-0.1..0.1` | `10..15` | 1 | -2.051088623 | -2.051088623 | -2.051088623 | -2.051088623 | 0.000 |
| `all` | `moving` | `-0.1..0.1` | `15..22` | 1 | -4.443002908 | -4.443002908 | -4.443002908 | -4.443002908 | 0.000 |
| `all` | `moving` | `0.1..0.5` | `0.2..1` | 13963 | 0.258248328 | 0.231138907 | -2.143611484 | 1.997270752 | 0.578 |
| `all` | `moving` | `0.1..0.5` | `1..3` | 15282 | 0.019095413 | -0.371879725 | -1.313786435 | 0.513025082 | 0.505 |
| `all` | `moving` | `0.1..0.5` | `3..6` | 19477 | -1.138554176 | -1.054984323 | -1.605777258 | -0.429731484 | 0.066 |
| ... | ... | ... | ... | 13 more rows in CSV | | | | | |

## Interpretation

Aligned bins show a consistent positive opposing torque, so the historical data can estimate a yaw-resisting surface where the drive command is trying to add yaw in the same direction as current yaw rate. The same surface improves aligned-sample RMSE but worsens many all-sample and leave-run-out cases because braking and phase-transition rows contain different control dynamics. In-place data provides the strongest yaw-rate coverage near zero forward velocity. Moving-yaw data mostly covers 0.1..0.5 m/s and low-to-mid yaw rates; high forward-speed/high-yaw bins are sparse, so current logs are insufficient for a full production-quality two-dimensional tire CoF surface.

Created files: `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_torque_surface_bins.csv`, `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_surface_rmse.csv`, `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_run_summary.csv`, `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_torque_surface_report.md`.
