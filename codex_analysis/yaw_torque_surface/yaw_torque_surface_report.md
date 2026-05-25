# Yaw Torque Resistance Surface

## Method

This scratch analysis uses decoded `open_floor_main.csv` logs and sensor-derived targets only: raw gyro yaw rate minus a per-run independently recomputed stationary bias, encoder-derived forward velocity, encoder wheel speeds, and logged drive commands. It does not use `gyro_bias_radps`, `gyro_bias_anchor_radps`, `ukf_state_*`, or estimator diagnostics as targets.

Sign convention: `+Yaw` is clockwise. The scratch PlantModel mirror computes positive yaw moment as `0.5 * track_width * (left_forward_force - right_forward_force)` plus the front/rear right-force moment. Per sample, `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`; this is the torque that must be added to the current model to match observed yaw acceleration. The reported opposing-yaw resistance is `-sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`, so positive values oppose the current yaw motion.

Current-model yaw moment mirrors the yaw-relevant production equations from `PlantModel`: MotorEncoderDrive command-to-torque, static launch and rolling loss, longitudinal tire stiffness, front/rear right-contact force gains, and the single sustained-lateral-acceleration-derived contact `mu` projection. Lateral velocity is not independently observable in these logs, so the sensor-only replay assumes rightward body velocity is zero.

Bins: forward velocity is rounded to 0.10 m/s; yaw rate is rounded to 0.50 rad/s. The correction evaluation uses the signed-bin median of `residual_additive_yaw_torque_nm` from all other runs, because that is the directly measured torque correction needed by the current model. The opposing-resistance columns are reported separately to show whether the residual acts against or with the current yaw motion.

## Authoritative Constants Read

| Constant | Value |
| --- | ---: |
| mass kg | 0.140000000 |
| track width m | 0.084635000 |
| yaw inertia kg m^2 | 0.000220000 |
| yaw denominator incl wheel spin-up kg m^2 | 0.000246510 |
| wheel radius m | 0.012610000 |
| sustained lateral accel used for current mu m/s^2 | 18.730701 |

## Logs And Sensor Tail Cut

| Run | Rows | Kept | Bias rows | Bias rad/s | Samples | Cutoff | First dropped tick/time |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `2026-04-21_05-32-06` | 234369 | 234369 | 15074 | -0.006108652 | 116087 | sensor tail check kept all rows |  |
| `2026-04-20_08-38-39` | 217867 | 217867 | 28297 | -0.001221730 | 62516 | sensor tail check kept all rows |  |
| `2026-05-04_20-35-47` | 145644 | 145644 | 57526 | -0.000000000 | 35213 | sensor tail check kept all rows |  |
| `2026-05-04_16-57-53` | 208375 | 208375 | 166205 | -0.000000000 | 26572 | sensor tail check kept all rows |  |
| `2026-04-21_05-59-46` | 100419 | 100419 | 15075 | -0.004886922 | 34766 | sensor tail check kept all rows |  |
| `2026-04-21_00-16-10` | 109329 | 109329 | 15074 | -0.008552114 | 50310 | sensor tail check kept all rows |  |
| `2026-04-21_01-09-34` | 101013 | 101013 | 15074 | -0.006108652 | 36807 | sensor tail check kept all rows |  |
| `2026-04-20_02-33-07` | 27971 | 27971 | 254 | -0.001221730 | 23147 | sensor tail check kept all rows |  |
| `2026-04-20_12-10-58` | 86556 | 86556 | 14990 | -0.001221730 | 29336 | sensor tail check kept all rows |  |
| `2026-04-14_04-43-48` | 116754 | 116754 | 30277 | -0.003665191 | 15209 | sensor tail check kept all rows |  |

Stationary bias rows are detected independently with near-zero drive command, near-zero encoder velocity, valid control timing, and robust median/MAD trimming of raw gyro. Bad-tail cutting is sensor-only: a final contiguous tail is dropped only when it is quiescent or invalid for at least 0.25 s. Many interrupted logs end while still moving; those are kept because the sensors do not prove a bad tail.

## Compact Opposing Torque Table

Full signed-bin table is in `yaw_torque_surface_bins.csv`. Values below combine positive and negative yaw-rate bins at the same absolute yaw-rate bin and show weighted median opposing resistance.

| Forward bin m/s | Abs yaw-rate bin rad/s | Count | Median opposing Nm | IQR Nm |
| ---: | ---: | ---: | ---: | ---: |
| 0.00 | 0.50 | 28826 | 0.085758753 | 0.165482710 |
| 0.00 | 1.00 | 13936 | 0.080160531 | 0.172986000 |
| 0.00 | 1.50 | 9913 | 0.078300269 | 0.166355492 |
| 0.00 | 2.00 | 10203 | 0.019093792 | 0.187954597 |
| 0.00 | 2.50 | 12089 | -0.039354409 | 0.195714262 |
| 0.00 | 3.00 | 10278 | -0.086537855 | 0.123620863 |
| 0.00 | 3.50 | 6211 | -0.090692942 | 0.092688367 |
| 0.00 | 4.00 | 4039 | -0.084048602 | 0.139775925 |
| 0.00 | 4.50 | 2872 | -0.094776158 | 0.104127838 |
| 0.00 | 5.00 | 2130 | -0.086258464 | 0.188924096 |
| 0.00 | 5.50 | 2033 | -0.098142070 | 0.135691607 |
| 0.00 | 6.00 | 1542 | -0.095267134 | 0.174890034 |
| 0.00 | 6.50 | 1157 | -0.083384712 | 0.158270217 |
| 0.00 | 7.00 | 973 | -0.092387373 | 0.129782033 |
| 0.00 | 7.50 | 786 | -0.101026208 | 0.044077128 |
| 0.00 | 8.00 | 715 | -0.102084459 | 0.037511191 |
| 0.00 | 8.50 | 865 | -0.106170136 | 0.031976113 |
| 0.00 | 9.00 | 719 | -0.109095431 | 0.030191465 |
| 0.00 | 9.50 | 765 | -0.109906746 | 0.032692927 |
| 0.00 | 10.00 | 622 | -0.110499990 | 0.034138486 |
| 0.00 | 10.50 | 457 | -0.113607865 | 0.035539393 |
| 0.00 | 11.00 | 453 | -0.111910622 | 0.029304884 |
| 0.00 | 11.50 | 575 | -0.114216675 | 0.027669339 |
| 0.00 | 12.00 | 591 | -0.113736469 | 0.029066654 |
| 0.00 | 12.50 | 395 | -0.112387310 | 0.030691063 |
| 0.00 | 13.00 | 364 | -0.113419291 | 0.033892756 |
| 0.00 | 13.50 | 331 | -0.114133047 | 0.026909969 |
| 0.00 | 14.50 | 263 | -0.114927259 | 0.032853851 |
| -0.10 | 0.50 | 3399 | -0.009286740 | 0.071031450 |
| -0.10 | 1.00 | 3140 | -0.010502818 | 0.063988617 |
| -0.10 | 1.50 | 3365 | -0.013805944 | 0.066693415 |
| -0.10 | 2.00 | 3590 | -0.022186093 | 0.065185499 |
| -0.10 | 2.50 | 3542 | -0.025081221 | 0.057222611 |
| -0.10 | 3.00 | 2311 | -0.027528756 | 0.066936437 |
| -0.10 | 3.50 | 1515 | -0.025039772 | 0.068342030 |
| -0.10 | 4.00 | 1236 | -0.041064745 | 0.057016861 |
| -0.10 | 4.50 | 638 | -0.040239606 | 0.077775135 |
| -0.10 | 5.00 | 524 | -0.055906608 | 0.095750786 |
| -0.10 | 5.50 | 389 | -0.055989066 | 0.120270936 |
| -0.10 | 6.00 | 477 | -0.017685774 | 0.098788835 |
| -0.10 | 6.50 | 376 | -0.041480566 | 0.093631657 |
| -0.10 | 7.00 | 383 | -0.081707507 | 0.071750150 |
| -0.10 | 7.50 | 422 | -0.096496005 | 0.064263174 |
| -0.10 | 8.00 | 405 | -0.095134874 | 0.065374334 |
| -0.10 | 8.50 | 414 | -0.102166180 | 0.039621443 |
| -0.10 | 9.00 | 396 | -0.099241474 | 0.040986328 |
| -0.10 | 9.50 | 308 | -0.104837750 | 0.032606949 |
| 0.10 | 0.50 | 2602 | -0.009572035 | 0.086159060 |
| 0.10 | 1.00 | 2011 | -0.016266026 | 0.057098377 |
| 0.10 | 1.50 | 2126 | -0.012977790 | 0.059845165 |
| 0.10 | 2.00 | 2213 | -0.017179434 | 0.062808036 |
| 0.10 | 2.50 | 2441 | -0.026683894 | 0.077028455 |
| 0.10 | 3.00 | 2683 | -0.030343023 | 0.068027681 |
| 0.10 | 3.50 | 2286 | -0.043959115 | 0.071328913 |
| 0.10 | 4.00 | 1355 | -0.068627413 | 0.069671444 |
| 0.10 | 4.50 | 776 | -0.090960780 | 0.036253120 |
| 0.10 | 5.00 | 519 | -0.088223535 | 0.039833849 |
| 0.10 | 5.50 | 450 | -0.091816184 | 0.039633246 |
| 0.10 | 6.00 | 415 | -0.096794570 | 0.035672625 |
| 0.10 | 6.50 | 431 | -0.097039902 | 0.039292311 |
| 0.10 | 7.00 | 355 | -0.094827148 | 0.032285935 |
| 0.10 | 11.00 | 251 | -0.110171441 | 0.030073231 |
| -0.20 | 0.50 | 608 | -0.004832445 | 0.025589221 |
| -0.20 | 1.00 | 496 | -0.000579225 | 0.026692233 |
| -0.20 | 1.50 | 473 | -0.004480080 | 0.031470453 |
| -0.20 | 2.00 | 461 | -0.002520426 | 0.064611373 |
| -0.20 | 2.50 | 457 | 0.002185083 | 0.084334875 |
| -0.20 | 3.00 | 278 | -0.001045519 | 0.082035878 |
| 0.20 | 0.50 | 452 | -0.016086753 | 0.060767465 |
| 0.20 | 1.00 | 450 | -0.008733837 | 0.047671895 |
| 0.20 | 1.50 | 370 | 0.001046498 | 0.060577389 |
| 0.20 | 2.00 | 361 | -0.005028309 | 0.086282080 |
| 0.20 | 2.50 | 437 | -0.008028549 | 0.085558248 |
| 0.20 | 3.00 | 292 | 0.002012651 | 0.083076438 |
| 0.30 | 0.50 | 1552 | -0.036910161 | 0.073090793 |
| 0.30 | 1.00 | 1352 | -0.005924517 | 0.076666723 |
| 0.30 | 1.50 | 1060 | -0.012186355 | 0.088707220 |
| 0.30 | 2.00 | 952 | -0.025283675 | 0.088157117 |
| 0.30 | 2.50 | 914 | -0.026774438 | 0.070107025 |
| 0.30 | 3.00 | 832 | -0.028529455 | 0.063039158 |
| 0.30 | 3.50 | 644 | -0.041276337 | 0.057931387 |
| 0.30 | 4.00 | 561 | -0.048281043 | 0.058692299 |
| 0.30 | 4.50 | 564 | -0.042464905 | 0.060037143 |
| 0.30 | 5.00 | 554 | -0.048686320 | 0.069523905 |
| 0.30 | 5.50 | 573 | -0.058799334 | 0.075212125 |
| 0.30 | 6.00 | 692 | -0.085954022 | 0.039211546 |
| 0.30 | 6.50 | 580 | -0.084233600 | 0.035347425 |
| 0.30 | 7.00 | 441 | -0.071878126 | 0.046562428 |
| 0.30 | 7.50 | 384 | -0.058885089 | 0.059307551 |
| 0.30 | 8.00 | 969 | -0.050329208 | 0.054340719 |
| 0.30 | 8.50 | 1145 | -0.059485575 | 0.047427898 |
| 0.30 | 9.00 | 993 | -0.059732222 | 0.050868802 |
| 0.30 | 9.50 | 780 | -0.084395683 | 0.048326419 |
| 0.30 | 10.00 | 653 | -0.088185939 | 0.040809242 |
| 0.30 | 10.50 | 872 | -0.091060448 | 0.044009319 |
| 0.30 | 11.00 | 1154 | -0.086749534 | 0.042984808 |
| 0.30 | 11.50 | 940 | -0.097270647 | 0.033018679 |
| 0.30 | 12.00 | 714 | -0.098076850 | 0.034106938 |
| 0.30 | 12.50 | 1183 | -0.108780769 | 0.024982983 |
| 0.30 | 13.00 | 1998 | -0.112328862 | 0.023501823 |
| 0.30 | 13.50 | 1593 | -0.110886027 | 0.024989213 |
| 0.30 | 14.00 | 1469 | -0.112233919 | 0.025383895 |
| 0.30 | 14.50 | 1757 | -0.114211304 | 0.024208780 |
| 0.30 | 15.00 | 1426 | -0.114138683 | 0.022895064 |
| 0.30 | 15.50 | 569 | -0.112304611 | 0.027633740 |
| 0.30 | 16.00 | 418 | -0.116315342 | 0.027887977 |
| 0.30 | 16.50 | 300 | -0.114809279 | 0.028464308 |
| 0.30 | 17.00 | 293 | -0.115996952 | 0.028767026 |
| 0.30 | 18.00 | 295 | -0.117269981 | 0.027759212 |
| 0.30 | 18.50 | 618 | -0.114308224 | 0.024177588 |
| 0.30 | 19.00 | 348 | -0.112746081 | 0.021774850 |
| 0.30 | 20.50 | 255 | -0.113956362 | 0.021325366 |
| 0.40 | 0.50 | 9887 | 0.001447571 | 0.072592370 |
| 0.40 | 1.00 | 6636 | -0.001473812 | 0.028817571 |
| 0.40 | 1.50 | 3850 | -0.001660836 | 0.027409578 |
| 0.40 | 2.00 | 3257 | -0.003524689 | 0.028325992 |
| 0.40 | 2.50 | 2800 | -0.003871529 | 0.029420688 |
| 0.40 | 3.00 | 2048 | -0.009095925 | 0.032559695 |
| 0.40 | 3.50 | 1711 | -0.008767191 | 0.028888737 |
| 0.40 | 4.00 | 2137 | -0.009460971 | 0.027075553 |
| 0.40 | 4.50 | 3650 | -0.009118667 | 0.024647279 |
| 0.40 | 5.00 | 3655 | -0.011052113 | 0.026085628 |
| 0.40 | 5.50 | 3343 | -0.014338544 | 0.028646160 |
| 0.40 | 6.00 | 3420 | -0.017708727 | 0.030947481 |
| 0.40 | 6.50 | 2983 | -0.019010944 | 0.034137981 |
| 0.40 | 7.00 | 2354 | -0.022750986 | 0.034808773 |
| 0.40 | 7.50 | 1662 | -0.027857894 | 0.042299933 |
| 0.40 | 8.00 | 1289 | -0.036694322 | 0.047316043 |
| 0.40 | 8.50 | 1328 | -0.047038977 | 0.045732275 |
| 0.40 | 9.00 | 1268 | -0.046427165 | 0.038309188 |
| 0.40 | 9.50 | 1135 | -0.051434052 | 0.051174147 |
| 0.40 | 10.00 | 1102 | -0.058041381 | 0.048779166 |
| 0.40 | 10.50 | 791 | -0.054289444 | 0.042384630 |
| 0.40 | 11.00 | 428 | -0.059128579 | 0.047396703 |
| 0.40 | 11.50 | 306 | -0.057155984 | 0.045575932 |
| 0.40 | 13.50 | 335 | -0.092623059 | 0.043132280 |
| 0.40 | 14.00 | 274 | -0.098944721 | 0.032993568 |
| 0.50 | 0.50 | 1354 | -0.002449946 | 0.020415932 |
| 0.50 | 1.00 | 807 | -0.005239938 | 0.025654724 |
| 0.50 | 1.50 | 1260 | -0.002627402 | 0.024801382 |
| 0.50 | 2.00 | 323 | -0.013040733 | 0.033645904 |
| 0.50 | 2.50 | 351 | -0.012121028 | 0.027330422 |
| 0.50 | 3.00 | 336 | -0.015250972 | 0.029114981 |
| 0.50 | 3.50 | 258 | -0.012789002 | 0.024192335 |
| 0.50 | 4.50 | 278 | -0.010505933 | 0.024867022 |
| 0.50 | 5.00 | 326 | -0.011401336 | 0.025002799 |
| 0.50 | 5.50 | 279 | -0.012935377 | 0.028574042 |
| 0.50 | 6.00 | 261 | -0.010388096 | 0.031410529 |
| 0.50 | 6.50 | 319 | -0.010144555 | 0.030942800 |
| 0.50 | 7.00 | 366 | -0.014358405 | 0.028649115 |
| 0.50 | 7.50 | 401 | -0.010685047 | 0.030985007 |

## Leave-One-Run-Out RMSE

| Holdout run | Samples | Surface bins | Samples corrected | Current RMSE rad/s | Surface RMSE rad/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| `2026-04-14_04-43-48` | 15209 | 460 | 15143 | 0.303747185 | 0.285886271 |
| `2026-04-20_02-33-07` | 23147 | 411 | 16587 | 0.376359660 | 0.322730252 |
| `2026-04-20_08-38-39` | 62516 | 436 | 60614 | 0.416445072 | 0.547518405 |
| `2026-04-20_12-10-58` | 29336 | 431 | 28075 | 0.325357649 | 0.287712285 |
| `2026-04-21_00-16-10` | 50310 | 442 | 49328 | 0.305366620 | 0.198289426 |
| `2026-04-21_01-09-34` | 36807 | 437 | 35719 | 0.262442949 | 0.197941042 |
| `2026-04-21_05-32-06` | 116087 | 460 | 116024 | 0.420523351 | 0.457478552 |
| `2026-04-21_05-59-46` | 34766 | 460 | 34718 | 0.351723557 | 0.322805740 |
| `2026-05-04_16-57-53` | 26572 | 461 | 26572 | 0.328207668 | 0.388422182 |
| `2026-05-04_20-35-47` | 35213 | 461 | 35213 | 0.376800391 | 0.513423077 |

Aggregate holdout samples: 429963; corrected by non-empty surface bins: 417993.

- Current one-step yaw-rate RMSE: 0.369009749 rad/s
- Surface-corrected one-step yaw-rate RMSE: 0.401910472 rad/s

## Recommendation

The historical sensor data supports a velocity/yaw-rate-dependent residual yaw-torque structure, but not a clean production-ready counter-yaw resistance surface yet. The robust per-bin medians vary materially with yaw-rate and forward speed, but several high-count bins report negative opposing resistance, meaning the current model needs yaw-aiding torque rather than extra resistance in those bins. Treat this as evidence that the single-CoF model is incomplete, not as enough evidence to install a simple monotonic counter-yaw table.

If this is implemented later, keep ownership in the existing authorities: `Vehicle` should continue to own physical construction facts and fixed hardware capabilities; `PlantModel` should own the plant equation that maps velocity/yaw-rate/contact state to tire-limited yaw resistance. Do not add a new generic CoF/safety-limit owner. A production change should replace the single sustained-acceleration-derived `mu` use inside `PlantModel` with a compact, directly testable tire/contact resistance model derived from authoritative Vehicle/MotorEncoderDrive facts and calibrated data.
