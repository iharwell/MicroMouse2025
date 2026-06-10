# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `7`
- Train subset segments: `5`
- Validation subset segments: `3`
- Boundary-ended active rows in train/validation: `1` / `1`
- Final active-metric segments: `8`
- Stress diagnostic segments: `1690`
- Source logs: `51`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch and yaw calibration sections as primary data.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 4.14151590352 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:nominal` | 3.94024176677 |
| `candidate_2_stribeck` | `candidate_2_stribeck:nominal` | 5.62064772321 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:nominal` | 5.87040375102 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `SEC_20_LAUNCH` | 5.92039408025 | 12.8176543772 | 0.312736159088 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` |  | 17.5145449616 | 16.8059577799 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 6.18701167456 | 20.0800958893 | 0.347824657806 |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 0.85202542528 |  |  |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_20_LAUNCH` | 7.91575830596 | 12.1005585722 | 0.44276300328 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` |  | 7.45213170947 | 6.41176533097 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 6.65126318646 | 19.495431836 | 0.999412351171 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 11.4078184757 |  |  |
| `candidate_2_stribeck` | `held_out` | `SEC_20_LAUNCH` | 9.50707204347 | 11.7709173428 | 0.469810185488 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` |  | 6.95137897853 | 5.85026212789 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 7.17480864479 | 19.2207550248 | 0.194596568507 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 11.2548898549 |  |  |
| `candidate_3_load_sensitive` | `held_out` | `SEC_20_LAUNCH` | 8.13298113523 | 11.4559233539 | 0.789836645088 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` |  | 6.58997075156 | 5.51083674242 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 5.26425776602 | 18.9545662523 | 0.131039768391 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 10.9878455061 |  |  |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
