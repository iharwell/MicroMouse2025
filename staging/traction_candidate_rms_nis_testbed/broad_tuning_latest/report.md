# Traction RMS/NIS Broad Tuning Report

- Trials evaluated: `28`
- Train subset segments: `56`
- Validation subset segments: `42`
- Final aggregate-only segments: `1690`
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.
- RMS NIS definition: `sqrt(sum(nis^2) / count)` over the rows in each aggregate bucket.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 627.339023289 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_006` | 562.055473225 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_008` | 592.426158291 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_001` | 556.860833674 |

## Validation And Held-Out RMS NIS

Full table: `validation_heldout_rms_nis.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `SEC_10_STATIC` | 3.65109250174 | 0.115968116525 | 0.0699844987753 |
| `baseline/current_holdover` | `held_out` | `SEC_20_LAUNCH` | 10.1137909224 | 94.7266035613 | 0.678808412964 |
| `baseline/current_holdover` | `held_out` | `SEC_30_STRAIGHT` | 139.379379314 | 155.335722383 | 1.01603437398 |
| `baseline/current_holdover` | `held_out` | `launch` | 7.62453742266 | 114.190963875 | 1.59014824809 |
| `baseline/current_holdover` | `held_out` | `mixed_launch` | 136.551643979 |  |  |
| `baseline/current_holdover` | `held_out` | `static` | 0.0128084289532 |  |  |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 22452.5350273 | 59.9170043132 | 52.0480247348 |
| `baseline/current_holdover` | `validation` | `SEC_10_STATIC` | 112.711405617 | 0.125358172889 | 2.54799762673 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 8.43881482217 | 93.9848792404 | 0.830671606262 |
| `baseline/current_holdover` | `validation` | `SEC_30_STRAIGHT` | 139.379379314 | 155.335722383 | 1.01603437398 |
| `baseline/current_holdover` | `validation` | `launch` | 6.91802403514 | 109.736499643 | 1.1130259964 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 218.98178218 |  |  |
| `baseline/current_holdover` | `validation` | `static` | 4.10007678564 |  |  |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 11488.7692062 | 58.8757676782 | 46.8589602337 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_10_STATIC` | 3.66953838445 | 0.113894043683 | 0.0663452357593 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_20_LAUNCH` | 11.512316465 | 88.4729030248 | 1.16828280762 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_30_STRAIGHT` | 161.008808479 | 153.787268745 | 1.13659363594 |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 8.45256065275 | 105.603914808 | 1.26668004782 |
| `candidate_1_algebraic_envelope` | `held_out` | `mixed_launch` | 136.569349864 |  |  |
| `candidate_1_algebraic_envelope` | `held_out` | `static` | 0.0128084289532 |  |  |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 20512.0421113 | 85.7988132438 | 76.771116338 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_10_STATIC` | 112.719844126 | 0.125165066289 | 2.54770316923 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 10.7920454377 | 87.278544444 | 1.1056361855 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 161.008808479 | 153.787268745 | 1.13659363594 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 7.67451260254 | 106.051081726 | 1.98115814935 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 219.01780313 |  |  |
| `candidate_1_algebraic_envelope` | `validation` | `static` | 4.12013085501 |  |  |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 10226.9208481 | 85.4289147981 | 79.8321481068 |
| `candidate_2_stribeck` | `held_out` | `SEC_10_STATIC` | 3.65737172487 | 0.112939671341 | 0.068566884306 |
| `candidate_2_stribeck` | `held_out` | `SEC_20_LAUNCH` | 10.4997100882 | 94.6442559314 | 0.855662106766 |
| `candidate_2_stribeck` | `held_out` | `SEC_30_STRAIGHT` | 139.369898931 | 155.309814938 | 1.01533640206 |
| `candidate_2_stribeck` | `held_out` | `launch` | 7.80644363146 | 116.417925243 | 2.00744194976 |
| `candidate_2_stribeck` | `held_out` | `mixed_launch` | 136.572004136 |  |  |
| `candidate_2_stribeck` | `held_out` | `static` | 0.0128084289532 |  |  |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 21459.6295305 | 78.2868317079 | 68.8541164154 |
| `candidate_2_stribeck` | `validation` | `SEC_10_STATIC` | 112.713372442 | 0.125291561946 | 2.54788423966 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 9.0889081213 | 94.573214832 | 1.27683636973 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 139.369898931 | 155.309814938 | 1.01533640206 |
| `candidate_2_stribeck` | `validation` | `launch` | 6.96998332881 | 109.637071828 | 1.84554610125 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 218.988590164 |  |  |
| `candidate_2_stribeck` | `validation` | `static` | 4.10627866009 |  |  |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 10840.6420527 | 74.9601489024 | 62.4627937601 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_10_STATIC` | 12.0367005809 | 0.443083493016 | 0.39461356613 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_20_LAUNCH` | 15.0338418837 | 99.483206049 | 1.69166727887 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_30_STRAIGHT` | 139.379379314 | 155.383624316 | 1.01603437404 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 15.7485385607 | 121.544510313 | 3.33570105934 |
| `candidate_3_load_sensitive` | `held_out` | `mixed_launch` | 136.554842616 |  |  |
| `candidate_3_load_sensitive` | `held_out` | `static` | 0.0128084289532 |  |  |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 20354.3754661 | 118.759916952 | 104.346768888 |
| `candidate_3_load_sensitive` | `validation` | `SEC_10_STATIC` | 112.992976834 | 0.487827050554 | 2.58071402904 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 14.0909340839 | 100.327692133 | 2.26619122729 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 139.379379314 | 155.383624316 | 1.01603437404 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 9.91873158773 | 115.056889377 | 2.84056983765 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 218.669471971 |  |  |
| `candidate_3_load_sensitive` | `validation` | `static` | 4.75231185384 |  |  |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 10112.9338314 | 110.730287331 | 93.825951618 |

## Limitations

- This is a standalone residual replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than three whole segments cannot populate every split.
- Full scoring is aggregate-only; it does not emit per-row diagnostics for the selected trials.
- Row-bounded runs sample evenly across each selected whole segment; split assignment still remains whole-segment.
