# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `4`
- Train subset segments: `2`
- Validation subset segments: `2`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `2`
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
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 18.6829077019 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:nominal` | 23.8471405433 |
| `candidate_2_stribeck` | `candidate_2_stribeck:nominal` | 53.1079147861 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:nominal` | 103.528509633 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 17.1099673216 | 7.14958831552 | 9.39218875098 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 16.7170060735 | 6.30801888581 | 8.42388792531 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 16.5674533307 | 6.01155378737 | 8.0807793758 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 16.3296206087 | 5.57199967546 | 7.56989735973 |

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
