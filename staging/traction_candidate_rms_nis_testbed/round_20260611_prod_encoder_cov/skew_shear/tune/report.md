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
| `skew_shear` | `skew_shear:trial_172` | 253.278222465 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `skew_shear` | `held_out` | `launch` | 15.4951950587 | 5.12722114322 | 2.1456196482 |
| `skew_shear` | `held_out` | `yaw_launch` | 337.751622038 | 3.27444952999 | 4.26621551355 |
| `skew_shear` | `validation` | `SEC_20_LAUNCH` | 9.74955002463 | 4.8571100532 | 1.68954579028 |
| `skew_shear` | `validation` | `launch` | 16.3405835846 | 9.21207990146 | 2.82005337598 |
| `skew_shear` | `validation` | `mixed_launch` | 59.2876022151 | 10.1078091206 | 5.64915804182 |
| `skew_shear` | `validation` | `yaw_launch` | 8068.65729343 | 221.408358939 | 71.6314391016 |

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
