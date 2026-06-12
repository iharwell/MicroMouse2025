# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `257`
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
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:nominal` | 253.904980182 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 17.0852997734 | 6.43969482518 | 2.60447764251 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 256.562940444 | 3.9005676799 | 4.12519793283 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 10.4602920095 | 5.50676254878 | 1.88416229838 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 18.3970111231 | 10.5867059348 | 3.3288194923 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 58.2625315719 | 12.9363925232 | 5.52004293621 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 8324.98345287 | 205.612797294 | 61.4991209202 |

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
