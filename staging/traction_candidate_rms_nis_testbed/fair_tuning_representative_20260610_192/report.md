# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `580`
- Train subset segments: `319`
- Validation subset segments: `115`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `562`
- Stress diagnostic segments: `570`
- Source logs: `8`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch and yaw calibration sections as primary data.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 6955.28916281 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_050` | 2421.77935212 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_013` | 2459.17391381 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_179` | 2420.7340837 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `launch` | 72.6620528948 | 15.326572537 | 1.13311536584 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 488841.067338 | 7.58695793107 | 7.64424626944 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 37.8806292907 | 8.99993581016 | 0.637847711393 |
| `baseline/current_holdover` | `validation` | `SEC_30_STRAIGHT` | 408.284338971 | 11.1930731817 | 1.53365995939 |
| `baseline/current_holdover` | `validation` | `SEC_40_YAW` | 785387795.426 | 12.9370002042 | 12.1703355207 |
| `baseline/current_holdover` | `validation` | `launch` | 66.6814844667 | 15.0342147645 | 1.28647665478 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 785.958427661 | 17.3105068362 | 1.97969498407 |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 34055769.8152 | 12.8335476943 | 12.7321440443 |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 19.1571465299 | 6.77136495728 | 5.19157232336 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 136.102599547 | 3.38785145595 | 2.65955199794 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 14.4083816939 | 5.25117533025 | 4.15681292901 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 671.336000453 | 6.76717117776 | 5.50488966855 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_40_YAW` | 403074.342061 | 11.226258058 | 10.767005457 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 19.0204062162 | 6.92952685377 | 5.51387048359 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 81.5830201414 | 8.42265146503 | 6.97183697727 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 8881.41869441 | 6.09967858361 | 6.10223248098 |
| `candidate_2_stribeck` | `held_out` | `launch` | 13.914289166 | 3.84650295033 | 1.63972925883 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 513.3734745 | 2.07503411864 | 2.06186734531 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 8.48086001393 | 3.21399652437 | 1.21365110069 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 684.950305704 | 4.00316805814 | 2.47137559093 |
| `candidate_2_stribeck` | `validation` | `SEC_40_YAW` | 434908.815655 | 11.4381733171 | 10.8819064347 |
| `candidate_2_stribeck` | `validation` | `launch` | 12.9472503137 | 4.50678426119 | 1.88340324435 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 63.2763305957 | 5.39056691847 | 3.24265712501 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 10694.4241237 | 5.19439828925 | 4.98879771151 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 14.1805268768 | 3.85543623489 | 1.68428730084 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 546.657078503 | 2.13454205235 | 2.09092367135 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 8.67968237585 | 3.2235428315 | 1.25079991155 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 735.281639777 | 4.00207028487 | 2.47694782497 |
| `candidate_3_load_sensitive` | `validation` | `SEC_40_YAW` | 486979.008992 | 11.4136060058 | 10.7212971846 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 13.3190703618 | 4.51293896225 | 1.93769971899 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 68.6854632936 | 5.3745668907 | 3.26704251922 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 19726.1520035 | 5.14444140437 | 4.69361739223 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | -46984.0742134 | -46984.0742134 .. -46984.0742134 | 1 | 1 |
| `candidate_2_stribeck` | -46935.1556669 | -46935.1556669 .. -46935.1556669 | 1 | 1 |
| `candidate_3_load_sensitive` | -46930.7973978 | -46930.7973978 .. -46930.7973978 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
