# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `388`
- Train subset segments: `96`
- Validation subset segments: `64`
- Boundary-ended active rows in train/validation: `6` / `5`
- Final active-metric segments: `1662`
- Stress diagnostic segments: `1690`
- Source logs: `51`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch and yaw calibration sections as primary data.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 5.20587881573 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_074` | 4.4522182586 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_028` | 4.6006656579 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_081` | 4.62148213859 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `SEC_20_LAUNCH` | 6.74472067867 | 9.06746405257 | 0.799064391615 |
| `baseline/current_holdover` | `held_out` | `SEC_40_YAW` |  | 10.391841936 | 8.36214116066 |
| `baseline/current_holdover` | `held_out` | `SEC_50_SMOOTH` | 6.35083429462 | 12.9450145095 | 5.920263274 |
| `baseline/current_holdover` | `held_out` | `launch` | 7.57569563196 | 15.5304404845 | 0.875580687574 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 11.9766183387 | 9.3887828332 | 9.292487685 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 6.94699756313 | 12.9105337307 | 0.817490346754 |
| `baseline/current_holdover` | `validation` | `SEC_30_STRAIGHT` | 9.93819843387 | 7.6853910434 | 2.33580098993 |
| `baseline/current_holdover` | `validation` | `SEC_40_YAW` |  | 15.8616150425 | 9.78723694407 |
| `baseline/current_holdover` | `validation` | `SEC_50_SMOOTH` | 7.04060440144 | 8.49509752474 | 5.85921456111 |
| `baseline/current_holdover` | `validation` | `launch` | 7.48001131247 | 16.9155896896 | 1.08446598368 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 8.91815302248 | 17.9086888576 | 3.27701066838 |
| `baseline/current_holdover` | `validation` | `straight` | 7.88726564524 | 11.8830901527 | 9.85850120728 |
| `baseline/current_holdover` | `validation` | `yaw_launch` | 11.3182688289 | 13.1677562468 | 12.86594297 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_20_LAUNCH` | 5.50433032327 | 8.3224076706 | 1.20181465206 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_40_YAW` |  | 9.85908897073 | 10.0466083125 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_50_SMOOTH` | 4.77645256971 | 10.6331306215 | 8.03233097492 |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 7.19179297865 | 14.718294607 | 1.91988759869 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 13.0753439949 | 6.06548390064 | 5.93365316038 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 5.85137299921 | 12.48483246 | 1.59412854975 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 7.57490208646 | 8.48804234529 | 3.16992165472 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_40_YAW` |  | 0.120224570025 | 2.32066376558 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_50_SMOOTH` | 6.06712944209 | 11.084222363 | 5.66436062485 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 6.6963400837 | 13.9442848996 | 1.59331201316 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 8.09974129603 | 16.4741812685 | 5.20303178316 |
| `candidate_1_algebraic_envelope` | `validation` | `straight` | 8.22867641034 | 11.7444163641 | 9.98560679484 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 10.9164190846 | 7.19242931209 | 6.84339109772 |
| `candidate_2_stribeck` | `held_out` | `SEC_20_LAUNCH` | 5.43287720512 | 8.32727922035 | 1.17531462661 |
| `candidate_2_stribeck` | `held_out` | `SEC_40_YAW` |  | 2.40646751761 | 16.564758303 |
| `candidate_2_stribeck` | `held_out` | `SEC_50_SMOOTH` | 7.54868562105 | 11.8651584811 | 7.48863500354 |
| `candidate_2_stribeck` | `held_out` | `launch` | 7.41215930359 | 14.6265279091 | 1.89086580332 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 13.0632827704 | 5.87362597612 | 5.82637225633 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 5.8570450804 | 12.4725122035 | 1.52453157058 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 8.19518706455 | 8.52911115307 | 3.34847116805 |
| `candidate_2_stribeck` | `validation` | `SEC_40_YAW` |  | 13.1527100543 | 0.803444242783 |
| `candidate_2_stribeck` | `validation` | `SEC_50_SMOOTH` | 8.39363350337 | 8.62374442014 | 7.68501471362 |
| `candidate_2_stribeck` | `validation` | `launch` | 6.78490076149 | 14.0001218394 | 1.54903415126 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 9.70541956223 | 16.7202323978 | 3.83503217447 |
| `candidate_2_stribeck` | `validation` | `straight` | 8.24549295048 | 11.7096542856 | 9.73635102954 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 10.8914073631 | 7.0731443921 | 6.88697838901 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_20_LAUNCH` | 5.48190565228 | 8.31710720581 | 1.16745850988 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_40_YAW` |  | 3.74586849827 | 14.0227627675 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_50_SMOOTH` | 7.7127352227 | 12.268168824 | 7.44327654496 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 7.40413096521 | 14.7096289511 | 1.86592282457 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 12.9544359278 | 5.90239275795 | 5.83322339224 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 5.83150487165 | 12.4838369597 | 1.51039614199 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 7.34212684323 | 8.50894833342 | 3.2101753002 |
| `candidate_3_load_sensitive` | `validation` | `SEC_40_YAW` |  | 6.72663032511 | 0.655085143286 |
| `candidate_3_load_sensitive` | `validation` | `SEC_50_SMOOTH` | 7.1870705215 | 12.134834081 | 7.63493660752 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 6.77984563516 | 13.9735933805 | 1.52364794491 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 9.36109370206 | 16.6638643615 | 4.08742582296 |
| `candidate_3_load_sensitive` | `validation` | `straight` | 8.24709599002 | 11.6086297104 | 9.71012931372 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 11.576105929 | 6.79021569106 | 6.9775436222 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | -0.0549987129803 | -0.805655631491 .. 0.825787991223 | 0.65 | 7 |
| `candidate_2_stribeck` | 0.254813843149 | -0.908982790376 .. 1.79504706701 | 0.49 | 7 |
| `candidate_3_load_sensitive` | 0.546605168639 | -0.5270846317 .. 2.24360613552 | 0.22 | 7 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
