# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `388`
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
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_094` | 3.43160136786 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_005` | 3.50926208929 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_003` | 3.43046017468 |

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
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 7.24103573541 | 4.44097936169 | 2.16111714236 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 12.4560029678 | 2.17225582643 | 2.11783661189 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 5.07615794191 | 3.75805722093 | 1.59580264273 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 7.75103638804 | 4.44026038211 | 2.68840893688 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 6.39226198757 | 4.83131324649 | 2.08578442993 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 8.86569451788 | 6.12903537649 | 3.61231725053 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 11.4661382909 | 4.61615475588 | 4.4540502834 |
| `candidate_2_stribeck` | `held_out` | `launch` | 7.42378379594 | 5.0558718305 | 2.83074148794 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 12.1765104902 | 2.44602242682 | 2.198307287 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 5.30405948501 | 4.1608171969 | 2.0768495005 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 7.93424862745 | 4.87231860871 | 3.1740996826 |
| `candidate_2_stribeck` | `validation` | `launch` | 6.68873584952 | 5.37042355466 | 2.6261029249 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 9.12278714171 | 6.83445331895 | 4.30032872798 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 11.3204658359 | 6.06811395179 | 4.79027098119 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 7.1118905281 | 4.51643620344 | 1.87538651869 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 12.4900373561 | 1.99526135488 | 2.0845828636 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 4.98592289366 | 3.74680126804 | 1.39711446695 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 7.27373491758 | 4.58395902961 | 2.74095300142 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 6.2060227686 | 4.81878367675 | 1.79993042345 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 8.83220761626 | 6.36306529583 | 3.00185780312 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 12.0646952613 | 3.41565640631 | 4.06813777669 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | -3.18712246166 | -3.18712246166 .. -3.18712246166 | 1 | 1 |
| `candidate_2_stribeck` | -3.14554454962 | -3.14554454962 .. -3.14554454962 | 1 | 1 |
| `candidate_3_load_sensitive` | -3.27044493197 | -3.27044493197 .. -3.27044493197 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
