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
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_005` | 163.678851654 |

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
| `candidate_2_stribeck` | `held_out` | `launch` | 2.01125381088 | 2.10986127367 | 18.5855539565 | 7.51085483439 | 3.08871443613 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 2.27425447667 | 2.45343241348 | 194.667769672 | 4.5069587893 | 4.7491529281 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 1.83245275577 | 1.86353682826 | 11.8634936871 | 6.61476876624 | 2.29029020774 |
| `candidate_2_stribeck` | `validation` | `launch` | 2.19292816835 | 2.24834649105 | 20.4385669657 | 14.2639740233 | 4.10553431939 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 2.54659550612 | 3.07801407228 | 66.5431744535 | 15.5224654135 | 6.54056848796 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 262.100742017 | 324.77495457 | 8715.40531014 | 229.276664846 | 72.387119881 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_2_stribeck` | -31547.3166693 | -31547.3166693 .. -31547.3166693 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.
