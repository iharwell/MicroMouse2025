# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `194`
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
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 1751797.41407 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_188` | 253.675918428 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `launch` | 72.6620528948 | 156.433284085 | 1.26648533771 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 488841.067338 | 924.526656123 | 883.924856189 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 37.8806292907 | 119.558860615 | 0.637847711393 |
| `baseline/current_holdover` | `validation` | `launch` | 40.4378303726 | 154.541048657 | 1.5709709771 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 756.18036762 | 153.867174311 | 2.20368528898 |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 34530002.4688 | 3401089.66801 | 3375151.02533 |
| `candidate_2_stribeck` | `held_out` | `launch` | 20.138973258 | 10.9488188399 | 4.47790896138 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 158.691730702 | 5.95319206865 | 5.42780185008 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 13.3211776092 | 9.02054265302 | 3.20699147866 |
| `candidate_2_stribeck` | `validation` | `launch` | 22.918997936 | 18.9076616662 | 5.95713483846 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 73.6620997986 | 21.9392232017 | 8.88997224189 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 8576.12103527 | 219.64224818 | 67.3312749308 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_2_stribeck` | -47127.3762718 | -47127.3762718 .. -47127.3762718 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Encoder wheel-rate residuals may still appear in diagnostic artifacts, but they are not production-equivalent NIS evidence.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.
