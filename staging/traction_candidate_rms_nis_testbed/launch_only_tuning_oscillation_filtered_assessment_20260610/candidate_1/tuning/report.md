# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `193`
- Train subset segments: `240`
- Validation subset segments: `118`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `638`
- Stress diagnostic segments: `638`
- Source logs: `7`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch, yaw calibration, and encoder residual/NIS streams as primary data.
- Main score policy: all finite NIS is retained; accepted-only RMS is diagnostic and high encoder/gyro NIS means model failure.
- Fixed noise schedule: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`; candidate-specific covariance/noise changes: `false`.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_148` | 162.412718245 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | left encoder | right encoder | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 3.14214642708 | 3.25518233542 | 16.76517804 | 5.93400943886 | 2.42283592851 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 3.73026973094 | 3.90741460941 | 278.1756647 | 3.6525425683 | 4.35877365937 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 2.89751849129 | 2.93720353356 | 10.3225139516 | 5.23982608782 | 1.78076309 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 3.34178888378 | 3.40861989962 | 17.783630104 | 11.1736722186 | 3.14769849977 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 3.81073115099 | 4.50825008129 | 60.197835752 | 12.1617223076 | 5.35466788057 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 270.269772853 | 332.138602161 | 8507.45211673 | 222.146554345 | 69.6315827273 |

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
- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.
