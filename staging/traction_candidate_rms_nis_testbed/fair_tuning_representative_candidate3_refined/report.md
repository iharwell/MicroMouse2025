# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `196`
- Train subset segments: `240`
- Validation subset segments: `110`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `578`
- Stress diagnostic segments: `584`
- Source logs: `6`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch and yaw calibration sections as primary data.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 4.56799503378 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:nominal` | 3.38809653108 |
| `candidate_2_stribeck` | `candidate_2_stribeck:nominal` | 3.4645413105 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:nominal` | 3.37878498777 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `launch` | 8.42056582952 | 15.3266922568 | 1.13311181352 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 17.2938444208 | 7.58717619766 | 7.64451966473 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 6.50164530433 | 8.99997400964 | 0.637845556079 |
| `baseline/current_holdover` | `validation` | `SEC_30_STRAIGHT` | 8.09045837949 | 11.1928068404 | 1.53359179251 |
| `baseline/current_holdover` | `validation` | `launch` | 7.71342596784 | 15.4791832009 | 1.25828757715 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 11.2721386076 | 17.461222158 | 1.90132870742 |
| `baseline/current_holdover` | `validation` | `yaw_launch` |  | 13.0319578221 | 13.017431424 |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 7.40521573663 | 4.92056311618 | 2.57201590074 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 11.6059097478 | 2.30817702934 | 2.11693378341 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 5.27231971651 | 4.10150416388 | 1.88719788374 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 7.85578469881 | 4.83474664162 | 2.9853506941 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 6.57838200085 | 5.36310640606 | 2.49156546885 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 8.9466230466 | 6.8395976826 | 4.14412245344 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 10.8384764438 | 4.92601276898 | 4.43287324896 |
| `candidate_2_stribeck` | `held_out` | `launch` | 7.45695719599 | 5.3585026352 | 3.08783505881 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 11.793447162 | 2.53572856668 | 2.15695046614 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 5.44739613853 | 4.28906766771 | 2.29146307569 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 7.93333301616 | 5.19064103935 | 3.35263893886 |
| `candidate_2_stribeck` | `validation` | `launch` | 6.77995194291 | 5.87664545048 | 2.96436842268 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 9.0877764586 | 7.15496651935 | 4.74918225578 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 11.1734435227 | 6.06955350515 | 4.74057435745 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 7.18487266048 | 4.43754560773 | 2.04348725588 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 11.348657095 | 2.10444665589 | 2.11336811656 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 5.04846808826 | 3.74712832792 | 1.54852592626 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 7.70671758694 | 4.53007311252 | 2.75592277893 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 6.39759939387 | 4.82894189657 | 2.016449492 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 8.65731164352 | 6.23536496441 | 3.41772136354 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 10.2343550007 | 4.41776512109 | 4.26998261339 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | -3.25382718273 | -3.25382718273 .. -3.25382718273 | 1 | 1 |
| `candidate_2_stribeck` | -3.17571792869 | -3.17571792869 .. -3.17571792869 | 1 | 1 |
| `candidate_3_load_sensitive` | -3.3167062333 | -3.3167062333 .. -3.3167062333 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
