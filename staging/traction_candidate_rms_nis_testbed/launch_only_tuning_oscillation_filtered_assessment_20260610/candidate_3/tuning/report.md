# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `194`
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
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 1080387.55861 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_133` | 172.536453495 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | left encoder | right encoder | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `launch` | 32512.9109467 | 32492.4355473 | 72.6620528948 | 156.433211311 | 1.26647037054 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 6739.33239773 | 6869.05405505 | 488841.067338 | 924.52946774 | 883.928175553 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 25127.3109835 | 25150.0121005 | 37.8806292907 | 119.558869789 | 0.637845556079 |
| `baseline/current_holdover` | `validation` | `launch` | 32670.7999559 | 32662.0787819 | 40.4378303726 | 154.505808325 | 1.57126753567 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 71362.4704355 | 70518.0957255 | 756.18036762 | 153.909323985 | 2.20337233853 |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 469378.860263 | 472262.918576 | 34530002.4688 | 3401055.03033 | 3375136.19166 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 1.30460404154 | 1.39434739416 | 20.0701432034 | 10.0051434961 | 4.08412420808 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 1.35350992353 | 1.50006816511 | 159.414200835 | 5.55115984062 | 6.02787842977 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 1.15933532641 | 1.19108650561 | 13.7450566247 | 9.24963411524 | 3.13256922825 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 1.45967402003 | 1.52543878625 | 22.7922046624 | 17.3719757198 | 5.70835389952 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 1.77766000108 | 2.05868290714 | 72.5302831866 | 21.3051847934 | 8.69962707908 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 234.028468293 | 294.996128092 | 8652.93416116 | 270.49827994 | 96.9463999775 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_3_load_sensitive` | -31550.0077542 | -31550.0077542 .. -31550.0077542 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.
