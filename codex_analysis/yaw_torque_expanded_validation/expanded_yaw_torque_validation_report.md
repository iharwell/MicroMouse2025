# Expanded Yaw Torque Validation

Scratch-only independent validation. No production code was modified.

## Method

Inputs are actual sensors and commands only: raw gyro yaw rate minus an independently estimated stationary bias, encoder-derived forward velocity and wheel speeds, logged drive commands, and timestamps. The extraction does not use `ukf_state_*`, pose, estimator yaw-rate targets, or logged gyro bias as targets.

Torque basis follows the prior reconciled Worker E approach: `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`, where the current model mirror includes motor torque, static/rolling loss, longitudinal tire stiffness, contact force gains, contact projection, and yaw damping. `opposing_yaw_resistance_nm = -sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`.

Only nonzero-forward/nonzero-yaw active adjacent samples are binned: `|Vf| >= 0.05 m/s`, `|yaw| >= 0.25 rad/s`, forward bins 0.10 m/s, yaw bins 0.50 rad/s.

## Constants

| Constant | Value |
| --- | ---: |
| mass kg | 0.140000000 |
| track width m | 0.084635000 |
| yaw inertia kg m^2 | 0.000220000 |
| yaw denominator incl wheel spin-up kg m^2 | 0.000246510 |
| wheel radius m | 0.012610000 |

## Log Set

Discovered 71 candidate CSV logs: 49 `TestResults\mmlog_decode_*\open_floor_main.csv` decoded open-floor logs, 1 extra `TestResults\UncertaintyTestLog\open_floor_main.csv`, 17 competition diagnostic/audit logs, and 4 competition front-wall characterization CSVs.
Used 49 logs with nonzero-Vf yaw samples: 33 decoded open-floor and 16 competition logs.

Full discovered/used/excluded details are in `discovered_logs.csv` and `run_summary.csv`.

Explorer G's inventory matches the local scan: the 49 decoded mmlog open-floor logs contain 3,745,735 rows, the competition `diag000`-`diag003` files contain 318,928 rows, and the competition `aux000`-`aux012` files contain 381,821 rows. `fwc` files were excluded because they are front-wall characterization records, not useful yaw/encoder time series.

## Explorer G Candidate Comparison

Explorer G's moving-yaw counts use a looser row predicate (`|Vf| > 0.02`, `|gyro| > 0.2`). Worker I's extraction is stricter: adjacent same-phase pairs, `|Vf| >= 0.05`, `|yaw| >= 0.25`, active yaw/command evidence, finite yaw acceleration, nonzero rounded bins, and decoded-log saturation flags skipped.

| Run | G-style moving yaw rows | Saturation fraction | Cmd angular zero fraction | Worker I extracted samples | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| `2026-04-20_08-38-39` | 115165 | 0.343160 | 0.003821 | 41332 | Still a major contributor, but saturation removes a large fraction. |
| `2026-04-21_00-16-10` | 54069 | 0.036453 | 0.071187 | 44880 | Best decoded moving-yaw run: broad range and low saturation. |
| `2026-04-20_04-54-09` | 45721 | 0.315654 | 1.000000 | 19376 | Useful, but every G-style row has zero `cmd_angular_radps`; extraction relies on wheel-command delta and sensor yaw. |
| `2026-04-21_05-32-06` | 36420 | 0.034624 | 0.020209 | 20317 | Useful lower-speed coverage with low saturation. |

The comparison table is in `explorer_g_comparison.csv`.

## Competition Contribution

Competition logs do help populate nonzero-Vf bins, especially real maze bins around `Vf=0.10..0.30 m/s`, `|yaw|=0.50..4.50 rad/s`, and a few higher-yaw maneuver bins.

They contributed 91,534 extracted samples. Of 407 reported bins, 177 had competition contribution, 71 were at least approximately 25% competition by per-run counted samples, 17 bins crossed the report threshold because competition data was present, and 6 reported bins were competition-only. Of the 145 bins passing the stricter exploratory support rule used below, 72 had competition contribution.

The competition split is mixed for prediction. `diag` holdouts improved from 0.314918163 to 0.292353413 rad/s over 46,099 samples, while `aux` holdouts worsened from 0.353441789 to 0.368234699 rad/s over 45,435 samples. The combined competition holdout result is nearly flat: 0.334595107 to 0.332192481 rad/s.

That makes `diag000`, `diag001`, and especially `diag003` useful coverage validators, but the `aux` logs are too path/procedure dependent to trust as torque-surface fitting authority without tighter phase labeling and repeat structure.

Competition contribution by bin is in `competition_bin_contribution.csv`.

## Highest-Coverage Nonzero-Vf Bins

| Vf bin m/s | Yaw bin rad/s | Count | Runs | Median residual Nm | Median opposing Nm | IQR Nm | Run spread Nm | Consistency |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.10 | 0.50 | 23315 | 43 | 0.006838926 | -0.006838926 | 0.137543129 | 0.222160815 | cross-run |
| 0.10 | -0.50 | 20502 | 42 | -0.027562556 | -0.027562556 | 0.116916369 | 0.215230332 | cross-run |
| -0.10 | -0.50 | 13646 | 37 | 0.009295287 | 0.009295287 | 0.085551886 | 0.216511531 | cross-run |
| -0.10 | 0.50 | 13468 | 36 | 0.012361551 | -0.012361551 | 0.106082831 | 0.220967954 | cross-run |
| 0.40 | 0.50 | 11059 | 32 | 0.000225178 | -0.000225178 | 0.025006073 | 0.130584335 | cross-run |
| 0.20 | 0.50 | 11041 | 39 | 0.008119138 | -0.008119138 | 0.077480320 | 0.204390246 | cross-run |
| 0.20 | -0.50 | 9691 | 35 | -0.008780367 | -0.008780367 | 0.057595606 | 0.121847600 | cross-run |
| 0.40 | -0.50 | 9395 | 30 | 0.001859664 | 0.001859664 | 0.027323340 | 0.124369200 | cross-run |
| 0.30 | -0.50 | 8885 | 34 | -0.000124812 | -0.000124812 | 0.018236483 | 0.190338743 | cross-run |
| -0.20 | -0.50 | 8865 | 29 | 0.009702608 | 0.009702608 | 0.035921617 | 0.093601336 | cross-run |
| 0.30 | 0.50 | 8574 | 36 | -0.002366578 | 0.002366578 | 0.022978504 | 0.105693255 | cross-run |
| 0.40 | 1.00 | 6188 | 27 | -0.001955067 | 0.001955067 | 0.026116559 | 0.142549997 | cross-run |
| 0.40 | -1.00 | 5602 | 28 | -0.001896866 | -0.001896866 | 0.027834850 | 0.030706881 | cross-run |
| 0.10 | -1.00 | 4362 | 32 | -0.047331986 | -0.047331986 | 0.094631211 | 0.201174152 | cross-run |
| 0.10 | 1.00 | 4279 | 31 | -0.004623791 | 0.004623791 | 0.128294068 | 0.231199146 | cross-run |
| -0.10 | 1.00 | 4088 | 22 | 0.051666732 | -0.051666732 | 0.076126600 | 0.148875309 | cross-run |
| 0.40 | -5.50 | 3833 | 11 | -0.012141063 | -0.012141063 | 0.025857012 | 0.094933893 | cross-run |
| 0.40 | -5.00 | 3752 | 12 | -0.010408566 | -0.010408566 | 0.024517678 | 0.085821851 | cross-run |
| -0.10 | -1.00 | 3719 | 25 | 0.035656915 | 0.035656915 | 0.111958646 | 0.206820997 | cross-run |
| 0.40 | 1.50 | 3619 | 23 | -0.002949622 | 0.002949622 | 0.025935549 | 0.097884640 | cross-run |
| 0.40 | -4.50 | 3540 | 10 | -0.008572925 | -0.008572925 | 0.023312481 | 0.011095478 | cross-run |
| -0.10 | 1.50 | 3352 | 19 | 0.052659580 | -0.052659580 | 0.061197366 | 0.219504948 | cross-run |
| -0.10 | 2.00 | 3329 | 20 | 0.051618412 | -0.051618412 | 0.046057974 | 0.225328738 | cross-run |
| 0.50 | -0.50 | 3302 | 30 | 0.000236930 | 0.000236930 | 0.022095008 | 0.130763820 | cross-run |
| -0.10 | 2.50 | 3294 | 18 | 0.047035327 | -0.047035327 | 0.044176335 | 0.266736144 | cross-run |

## RMSE

Leave-one-run-out correction uses signed-bin median residual torque from all other runs. Units are one-step yaw-rate error in rad/s.

| Holdout | Kind | Samples | Corrected samples | Current RMSE | Corrected RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| `2026-04-10_18-08-20` | decoded_open_floor | 1407 | 1260 | 0.313847862 | 0.305512951 |
| `2026-04-10_18-33-52` | decoded_open_floor | 886 | 886 | 0.305921833 | 0.324831875 |
| `2026-04-11_06-58-25` | decoded_open_floor | 862 | 862 | 0.144449220 | 0.160570808 |
| `2026-04-11_21-03-20` | decoded_open_floor | 174 | 174 | 0.212375911 | 0.222709176 |
| `2026-04-12_05-13-55` | decoded_open_floor | 4 | 4 | 0.363880848 | 0.389441623 |
| `2026-04-12_06-36-32` | decoded_open_floor | 432 | 432 | 0.319614431 | 0.327353892 |
| `2026-04-12_06-44-12` | decoded_open_floor | 432 | 432 | 0.319614431 | 0.327353892 |
| `2026-04-12_20-14-49` | decoded_open_floor | 7344 | 7331 | 0.327063811 | 0.327627015 |
| `2026-04-12_22-27-00` | decoded_open_floor | 10880 | 10880 | 0.311045019 | 0.304333087 |
| `2026-04-13_16-42-46` | decoded_open_floor | 13492 | 13482 | 0.244182252 | 0.234861521 |
| `2026-04-14_02-00-02` | decoded_open_floor | 5457 | 5315 | 0.229011060 | 0.189314962 |
| `2026-04-14_04-43-48` | decoded_open_floor | 13830 | 13812 | 0.273862615 | 0.267952406 |
| `2026-04-14_05-26-35` | decoded_open_floor | 8530 | 8510 | 0.269248514 | 0.259869807 |
| `2026-04-14_16-34-17` | decoded_open_floor | 4204 | 4188 | 0.265442660 | 0.229562961 |
| `2026-04-15_02-09-58` | decoded_open_floor | 8619 | 8374 | 0.306043733 | 0.285434240 |
| `2026-04-16_02-43-36` | decoded_open_floor | 234 | 234 | 0.039058923 | 0.111353683 |
| `2026-04-20_02-33-07` | decoded_open_floor | 20244 | 13610 | 0.380364058 | 0.328535573 |
| `2026-04-20_04-54-09` | decoded_open_floor | 19376 | 18819 | 0.324375240 | 0.296315016 |
| `2026-04-20_08-38-39` | decoded_open_floor | 41332 | 41083 | 0.330182192 | 0.286593152 |
| `2026-04-20_10-22-09` | decoded_open_floor | 10514 | 10344 | 0.319529260 | 0.262907029 |
| `2026-04-20_12-10-58` | decoded_open_floor | 24297 | 23437 | 0.306754816 | 0.235129883 |
| `2026-04-20_23-05-07` | decoded_open_floor | 62 | 62 | 0.108390934 | 0.165391706 |
| `2026-04-21_00-16-10` | decoded_open_floor | 44880 | 43656 | 0.295941786 | 0.175359706 |
| `2026-04-21_01-09-34` | decoded_open_floor | 25854 | 24927 | 0.210806036 | 0.135810906 |
| `2026-04-21_02-46-49` | decoded_open_floor | 13007 | 12484 | 0.277676299 | 0.190279220 |
| `2026-04-21_04-14-05` | decoded_open_floor | 11441 | 11156 | 0.279634240 | 0.163046395 |
| `2026-04-21_05-32-06` | decoded_open_floor | 20317 | 20245 | 0.275808322 | 0.206966549 |
| `2026-04-21_05-59-46` | decoded_open_floor | 14335 | 14289 | 0.248937874 | 0.168214748 |
| `2026-04-21_23-39-12` | decoded_open_floor | 5043 | 4998 | 0.187817681 | 0.204479483 |
| `2026-04-22_01-06-32` | decoded_open_floor | 5043 | 4998 | 0.187817681 | 0.204479483 |
| `2026-04-22_12-10-34` | decoded_open_floor | 3350 | 3350 | 0.207745925 | 0.223171523 |
| `2026-05-04_16-57-53` | decoded_open_floor | 95 | 95 | 0.099929841 | 0.128505316 |
| `2026-05-04_20-35-47` | decoded_open_floor | 1815 | 1815 | 0.282798034 | 0.462690023 |
| `aux000` | competition | 3047 | 3012 | 0.498614792 | 0.538827079 |
| `aux001` | competition | 3632 | 3615 | 0.335189576 | 0.372519212 |
| `aux002` | competition | 5487 | 5310 | 0.291597121 | 0.290225031 |
| `aux003` | competition | 6607 | 6395 | 0.342378197 | 0.338000418 |
| `aux004` | competition | 6552 | 6545 | 0.293220708 | 0.298642070 |
| `aux005` | competition | 2572 | 2571 | 0.367117201 | 0.370915605 |
| `aux006` | competition | 1669 | 1591 | 0.294139374 | 0.303733701 |
| `aux007` | competition | 2362 | 2360 | 0.351965650 | 0.385906165 |
| `aux008` | competition | 4834 | 4774 | 0.304565580 | 0.306224339 |
| `aux009` | competition | 4484 | 4482 | 0.374226283 | 0.411361655 |
| `aux010` | competition | 3806 | 3804 | 0.451200643 | 0.462311302 |
| `aux011` | competition | 71 | 71 | 0.642261489 | 0.641368934 |
| `aux012` | competition | 312 | 312 | 0.365528295 | 0.425587968 |
| `diag000` | competition | 11499 | 11490 | 0.395768484 | 0.360510155 |
| `diag001` | competition | 11358 | 11355 | 0.390712872 | 0.365079566 |
| `diag003` | competition | 23242 | 23207 | 0.211208638 | 0.200225646 |

Aggregate: samples=429326, corrected=416438, current RMSE=0.302028502 rad/s, corrected RMSE=0.263993314 rad/s.

## Data Sufficiency Judgment

Supported enough for calibration exploration, but still not production-final:
- Vf=-0.70 m/s, yaw=-0.50 rad/s: n=258, runs=6, medianOpp=0.0003 Nm, runSpread=0.0426
- Vf=-0.60 m/s, yaw=-1.00 rad/s: n=300, runs=10, medianOpp=0.0129 Nm, runSpread=0.0806
- Vf=-0.60 m/s, yaw=-0.50 rad/s: n=565, runs=12, medianOpp=0.0098 Nm, runSpread=0.0481
- Vf=-0.60 m/s, yaw=0.50 rad/s: n=1071, runs=15, medianOpp=0.0053 Nm, runSpread=0.0352
- Vf=-0.50 m/s, yaw=-1.00 rad/s: n=761, runs=16, medianOpp=0.0058 Nm, runSpread=0.0465
- Vf=-0.50 m/s, yaw=-0.50 rad/s: n=1229, runs=20, medianOpp=0.0061 Nm, runSpread=0.0462
- Vf=-0.50 m/s, yaw=0.50 rad/s: n=1735, runs=17, medianOpp=-0.0010 Nm, runSpread=0.0507
- Vf=-0.50 m/s, yaw=1.00 rad/s: n=353, runs=5, medianOpp=-0.0007 Nm, runSpread=0.0370
- Vf=-0.40 m/s, yaw=-0.50 rad/s: n=1086, runs=16, medianOpp=0.0172 Nm, runSpread=0.1079
- Vf=-0.40 m/s, yaw=0.50 rad/s: n=614, runs=9, medianOpp=-0.0099 Nm, runSpread=0.0330
- Vf=-0.30 m/s, yaw=-1.50 rad/s: n=327, runs=7, medianOpp=0.0304 Nm, runSpread=0.0746
- Vf=-0.30 m/s, yaw=-1.00 rad/s: n=820, runs=13, medianOpp=0.0318 Nm, runSpread=0.1093
- Vf=-0.30 m/s, yaw=-0.50 rad/s: n=1648, runs=18, medianOpp=0.0156 Nm, runSpread=0.1074
- Vf=-0.30 m/s, yaw=1.00 rad/s: n=536, runs=6, medianOpp=-0.0310 Nm, runSpread=0.0692
- Vf=-0.20 m/s, yaw=-3.00 rad/s: n=392, runs=4, medianOpp=0.0230 Nm, runSpread=0.0385
- Vf=-0.20 m/s, yaw=-2.50 rad/s: n=451, runs=4, medianOpp=0.0399 Nm, runSpread=0.0273
- Vf=-0.20 m/s, yaw=-0.50 rad/s: n=8865, runs=29, medianOpp=0.0097 Nm, runSpread=0.0936
- Vf=-0.20 m/s, yaw=0.50 rad/s: n=2862, runs=26, medianOpp=-0.0151 Nm, runSpread=0.0652
- Vf=-0.20 m/s, yaw=1.00 rad/s: n=1511, runs=16, medianOpp=-0.0416 Nm, runSpread=0.0734
- Vf=-0.20 m/s, yaw=1.50 rad/s: n=904, runs=12, medianOpp=-0.0490 Nm, runSpread=0.0767
- Vf=-0.20 m/s, yaw=2.00 rad/s: n=576, runs=9, medianOpp=-0.0528 Nm, runSpread=0.0637
- Vf=-0.20 m/s, yaw=2.50 rad/s: n=414, runs=7, medianOpp=-0.0630 Nm, runSpread=0.0336
- Vf=-0.10 m/s, yaw=-10.50 rad/s: n=399, runs=7, medianOpp=-0.1065 Nm, runSpread=0.0383
- Vf=-0.10 m/s, yaw=-10.00 rad/s: n=437, runs=8, medianOpp=-0.1036 Nm, runSpread=0.0633
- Vf=-0.10 m/s, yaw=-9.50 rad/s: n=443, runs=8, medianOpp=-0.1000 Nm, runSpread=0.0374
- Vf=-0.10 m/s, yaw=-9.00 rad/s: n=465, runs=9, medianOpp=-0.0911 Nm, runSpread=0.0741
- Vf=-0.10 m/s, yaw=-8.50 rad/s: n=454, runs=10, medianOpp=-0.0904 Nm, runSpread=0.0751
- Vf=-0.10 m/s, yaw=-8.00 rad/s: n=384, runs=10, medianOpp=-0.0849 Nm, runSpread=0.0730
- Vf=-0.10 m/s, yaw=-7.50 rad/s: n=335, runs=10, medianOpp=-0.0831 Nm, runSpread=0.0988
- Vf=-0.10 m/s, yaw=-7.00 rad/s: n=375, runs=10, medianOpp=-0.0763 Nm, runSpread=0.1161

Weak or targeted-sweep-needed bins that are populated but inconsistent or single-run dominated:
- Vf=-0.80 m/s, yaw=-1.00 rad/s: n=78, runs=1, medianOpp=0.0399 Nm, runSpread=0.0000
- Vf=-0.80 m/s, yaw=-0.50 rad/s: n=81, runs=1, medianOpp=0.0291 Nm, runSpread=0.0000
- Vf=-0.70 m/s, yaw=-1.00 rad/s: n=148, runs=2, medianOpp=0.0145 Nm, runSpread=0.0076
- Vf=-0.70 m/s, yaw=0.50 rad/s: n=97, runs=2, medianOpp=0.0018 Nm, runSpread=0.0212
- Vf=-0.60 m/s, yaw=-1.50 rad/s: n=90, runs=4, medianOpp=0.0052 Nm, runSpread=0.0415
- Vf=-0.60 m/s, yaw=1.00 rad/s: n=80, runs=2, medianOpp=0.0110 Nm, runSpread=0.0234
- Vf=-0.50 m/s, yaw=-2.00 rad/s: n=58, runs=3, medianOpp=-0.0174 Nm, runSpread=0.0594
- Vf=-0.50 m/s, yaw=-1.50 rad/s: n=244, runs=8, medianOpp=0.0019 Nm, runSpread=0.0450
- Vf=-0.40 m/s, yaw=-2.00 rad/s: n=112, runs=3, medianOpp=-0.0121 Nm, runSpread=0.0275
- Vf=-0.40 m/s, yaw=-1.50 rad/s: n=247, runs=6, medianOpp=0.0250 Nm, runSpread=0.0887
- Vf=-0.40 m/s, yaw=-1.00 rad/s: n=805, runs=15, medianOpp=0.0187 Nm, runSpread=0.1839
- Vf=-0.40 m/s, yaw=1.00 rad/s: n=193, runs=3, medianOpp=-0.0004 Nm, runSpread=0.0521
- Vf=-0.30 m/s, yaw=-3.00 rad/s: n=151, runs=3, medianOpp=0.0246 Nm, runSpread=0.0290
- Vf=-0.30 m/s, yaw=-2.50 rad/s: n=164, runs=3, medianOpp=0.0337 Nm, runSpread=0.0238
- Vf=-0.30 m/s, yaw=-2.00 rad/s: n=131, runs=3, medianOpp=0.0379 Nm, runSpread=0.0115
- Vf=-0.30 m/s, yaw=0.50 rad/s: n=2049, runs=23, medianOpp=-0.0025 Nm, runSpread=0.1445
- Vf=-0.30 m/s, yaw=1.50 rad/s: n=193, runs=5, medianOpp=-0.0490 Nm, runSpread=0.0636
- Vf=-0.30 m/s, yaw=2.00 rad/s: n=167, runs=4, medianOpp=-0.0506 Nm, runSpread=0.0150
- Vf=-0.30 m/s, yaw=2.50 rad/s: n=109, runs=3, medianOpp=-0.0533 Nm, runSpread=0.0196
- Vf=-0.30 m/s, yaw=3.00 rad/s: n=68, runs=2, medianOpp=-0.0468 Nm, runSpread=0.0026
- Vf=-0.20 m/s, yaw=-3.50 rad/s: n=142, runs=3, medianOpp=0.0259 Nm, runSpread=0.0140
- Vf=-0.20 m/s, yaw=-2.00 rad/s: n=424, runs=7, medianOpp=0.0365 Nm, runSpread=0.1773
- Vf=-0.20 m/s, yaw=-1.50 rad/s: n=479, runs=9, medianOpp=0.0444 Nm, runSpread=0.1793
- Vf=-0.20 m/s, yaw=-1.00 rad/s: n=1005, runs=16, medianOpp=0.0231 Nm, runSpread=0.1656
- Vf=-0.20 m/s, yaw=3.00 rad/s: n=199, runs=4, medianOpp=-0.0590 Nm, runSpread=0.0382
- Vf=-0.20 m/s, yaw=3.50 rad/s: n=82, runs=2, medianOpp=-0.0544 Nm, runSpread=0.0082
- Vf=-0.20 m/s, yaw=11.50 rad/s: n=68, runs=2, medianOpp=-0.1258 Nm, runSpread=0.0123
- Vf=-0.20 m/s, yaw=12.00 rad/s: n=65, runs=3, medianOpp=-0.1193 Nm, runSpread=0.0246
- Vf=-0.10 m/s, yaw=-15.00 rad/s: n=53, runs=2, medianOpp=-0.1089 Nm, runSpread=0.0329
- Vf=-0.10 m/s, yaw=-14.50 rad/s: n=92, runs=4, medianOpp=-0.1098 Nm, runSpread=0.0314
- Vf=-0.10 m/s, yaw=-14.00 rad/s: n=95, runs=3, medianOpp=-0.1078 Nm, runSpread=0.0204
- Vf=-0.10 m/s, yaw=-13.50 rad/s: n=78, runs=2, medianOpp=-0.1103 Nm, runSpread=0.0072
- Vf=-0.10 m/s, yaw=-13.00 rad/s: n=82, runs=4, medianOpp=-0.1114 Nm, runSpread=0.0208
- Vf=-0.10 m/s, yaw=-12.50 rad/s: n=90, runs=4, medianOpp=-0.1133 Nm, runSpread=0.0194
- Vf=-0.10 m/s, yaw=-12.00 rad/s: n=112, runs=4, medianOpp=-0.1046 Nm, runSpread=0.0178
- Vf=-0.10 m/s, yaw=-11.50 rad/s: n=186, runs=8, medianOpp=-0.1055 Nm, runSpread=0.0175
- Vf=-0.10 m/s, yaw=-11.00 rad/s: n=242, runs=5, medianOpp=-0.1069 Nm, runSpread=0.0330
- Vf=-0.10 m/s, yaw=-6.50 rad/s: n=403, runs=11, medianOpp=-0.0579 Nm, runSpread=0.1259
- Vf=-0.10 m/s, yaw=-6.00 rad/s: n=475, runs=10, medianOpp=-0.0161 Nm, runSpread=0.1297
- Vf=-0.10 m/s, yaw=-5.50 rad/s: n=382, runs=10, medianOpp=-0.0468 Nm, runSpread=0.1629

Regions absent or not reliable require targeted sweeps: high forward speed with high yaw rate, negative forward velocity, and any bins represented by one run only. Competition logs add useful nonzero-forward coverage, but many bins are maneuver/path dependent and not repeated cleanly across runs.

## Discrepancies With Previous Scratch Work

- This validation confirms the prior rejection of the approximately 0.9 Nm motor-only bins; using the fuller current PlantModel mirror keeps typical residuals in the hundredths to low-tenths of Nm range.
- Compared with the previous Worker E run, this extraction is deliberately coverage-first and includes competition diagnostic/audit CSVs plus all discoverable decoded open-floor candidates. It therefore exposes more nonzero-forward bins, but also more single-run and path-dependent bins.
- The sign pattern is not a monotonic counter-yaw table. Several populated bins still have negative opposing resistance, meaning the model sometimes needs yaw-aiding residual torque.

## Reproduce

```powershell
python codex_analysis\yaw_torque_expanded_validation\expanded_yaw_torque_validation.py --out-dir codex_analysis\yaw_torque_expanded_validation
python codex_analysis\yaw_torque_expanded_validation\summarize_competition_and_g.py
```
