# Traction Residual-Tail Fair Tuning Report

- Trials evaluated: `580`
- Train subset segments: `320`
- Validation subset segments: `111`
- Boundary-ended active rows in train/validation: `0` / `0`
- Final active-metric segments: `580`
- Stress diagnostic segments: `588`
- Source logs: `8`
- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.
- Selection policy: metric-balanced active traction rows first, including yaw launch and yaw calibration sections as primary data.
- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.
- Stress policy: full row-weighted residual tails are diagnostic-only.
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 4.6473878556 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_039` | 3.39810744088 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_163` | 3.51257276145 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_156` | 3.46741295282 |

## Validation And Held-Out Residual Tail

Full table: `validation_heldout_residual_tail.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `launch` | 8.42056582952 | 15.3266922568 | 1.13311181352 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 17.2938444208 | 7.58717619766 | 7.64451966473 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 6.50164530433 | 8.99997400964 | 0.637845556079 |
| `baseline/current_holdover` | `validation` | `SEC_30_STRAIGHT` | 8.09045837949 | 11.1928068404 | 1.53359179251 |
| `baseline/current_holdover` | `validation` | `SEC_40_YAW` | 12.7692707741 | 12.9371575691 | 12.1693537129 |
| `baseline/current_holdover` | `validation` | `launch` | 7.71342596784 | 15.4791832009 | 1.25828757715 |
| `baseline/current_holdover` | `validation` | `mixed_launch` | 11.2721386076 | 17.461222158 | 1.90132870742 |
| `baseline/current_holdover` | `validation` | `yaw_launch` |  | 13.0319578221 | 13.017431424 |
| `candidate_1_algebraic_envelope` | `held_out` | `launch` | 7.18784095012 | 4.39938060498 | 1.9220074141 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 12.534548524 | 2.11285821486 | 1.97542573176 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 4.99455419456 | 3.63662126055 | 1.42011944501 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_30_STRAIGHT` | 7.65843826126 | 4.78690443695 | 2.9113522325 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_40_YAW` | 10.8841105048 | 11.501465341 | 10.6361964129 |
| `candidate_1_algebraic_envelope` | `validation` | `launch` | 6.2455012351 | 4.84578338936 | 1.80894289628 |
| `candidate_1_algebraic_envelope` | `validation` | `mixed_launch` | 8.80106921365 | 6.03450549313 | 2.99013876789 |
| `candidate_1_algebraic_envelope` | `validation` | `yaw_launch` | 11.7235827302 | 5.44813224611 | 4.45533636666 |
| `candidate_2_stribeck` | `held_out` | `launch` | 7.51704730352 | 5.42400913473 | 3.22274712077 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 11.6746529448 | 2.56417325658 | 2.17050986715 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 5.43838768506 | 4.3740036782 | 2.31198446905 |
| `candidate_2_stribeck` | `validation` | `SEC_30_STRAIGHT` | 7.93650212361 | 5.29440313623 | 3.25737430505 |
| `candidate_2_stribeck` | `validation` | `SEC_40_YAW` | 10.6020403311 | 11.2334219419 | 11.3806432004 |
| `candidate_2_stribeck` | `validation` | `launch` | 6.8896896179 | 6.00372416547 | 2.9225899477 |
| `candidate_2_stribeck` | `validation` | `mixed_launch` | 9.09947577998 | 7.26255767667 | 4.88040542746 |
| `candidate_2_stribeck` | `validation` | `yaw_launch` | 10.894776649 | 5.58862468051 | 4.63892304143 |
| `candidate_3_load_sensitive` | `held_out` | `launch` | 6.79731533356 | 3.78923400116 | 1.50216624602 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 13.2585945019 | 1.92130298305 | 2.10197977709 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 4.73542141356 | 3.18937888456 | 1.1045762856 |
| `candidate_3_load_sensitive` | `validation` | `SEC_30_STRAIGHT` | 7.53319093986 | 3.97541942808 | 2.48339005331 |
| `candidate_3_load_sensitive` | `validation` | `SEC_40_YAW` | 10.8671754317 | 11.5022177677 | 11.2097252094 |
| `candidate_3_load_sensitive` | `validation` | `launch` | 5.88928396253 | 4.24209858458 | 1.43419284103 |
| `candidate_3_load_sensitive` | `validation` | `mixed_launch` | 8.5035551596 | 5.34247099044 | 2.653911137 |
| `candidate_3_load_sensitive` | `validation` | `yaw_launch` | 11.20704516 | 3.51065736839 | 4.03354781414 |

## Source-Log Bootstrap

Full table: `source_log_bootstrap_confidence.csv`.

| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |
| --- | ---: | ---: | ---: | ---: |
| `candidate_1_algebraic_envelope` | -3.22592854925 | -3.22592854925 .. -3.22592854925 | 1 | 1 |
| `candidate_2_stribeck` | -3.14979290414 | -3.14979290414 .. -3.14979290414 | 1 | 1 |
| `candidate_3_load_sensitive` | -3.2070541559 | -3.2070541559 .. -3.2070541559 | 1 | 1 |

## Limitations

- This is a standalone residual-tail replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.
- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.
- Yaw-rate residuals are available in the current standalone objective; yaw-acceleration or turn-response residual rows are included only when the residual update contract emits them.
- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.
- Row-bounded active runs sample evenly across active rows only; stress runs sample evenly across full segments.
