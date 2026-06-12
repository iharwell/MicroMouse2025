# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `193`
- Train subset segments: `240`
- Validation subset segments: `118`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `638`
- Stress diagnostic segments: `638`
- Source logs: `7`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, using only production measurement residual-tail streams.
- Main score policy: production-equivalent channels are yaw rate, forward accel, and right accel; encoder residuals are diagnostic only and excluded.
- Fixed noise schedule: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`; candidate-specific covariance/noise changes: `false`.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_042` | 253.472812372 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `candidate_2_stribeck` | `held_out` | `launch` | 19.770474447 | 9.54396410369 | 3.88069203058 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 159.148584553 | 5.3045053244 | 4.71085108648 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 12.7783985733 | 8.04215224869 | 2.82796463462 |
| `candidate_2_stribeck` | `validation` | `launch` | 22.3845105103 | 15.383290971 | 5.09116641603 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 68.287369166 | 19.1028417551 | 7.77659238212 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 8576.13553865 | 219.757928775 | 67.0009355845 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Encoder wheel-rate residuals may still appear in diagnostic artifacts, but they are not production-equivalent NIS evidence.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.
