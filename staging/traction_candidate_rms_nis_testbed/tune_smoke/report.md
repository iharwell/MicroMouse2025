# Traction RMS/NIS Broad Tuning Report

- Trials evaluated: `10`
- Train subset segments: `12`
- Validation subset segments: `12`
- Final aggregate-only segments: `24`
- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.
- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.

## Selected Trials

| Candidate | Trial | Validation score |
| --- | --- | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 22.4811356178 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_001` | 22.0603489499 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_002` | 22.5760514056 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_002` | 22.8457588408 |

## Validation And Held-Out RMS NIS

Full table: `validation_heldout_rms_nis.csv`.

| Candidate | Split | Stage | yaw | forward accel | right accel |
| --- | --- | --- | ---: | ---: | ---: |
| `baseline/current_holdover` | `held_out` | `SEC_10_STATIC` | 2.42844271241 | 0.309338343548 | 0.14832043436 |
| `baseline/current_holdover` | `held_out` | `SEC_20_LAUNCH` | 6.21375819552 | 98.9163434239 | 0.441842997299 |
| `baseline/current_holdover` | `held_out` | `yaw_launch` | 128.910093525 | 33.7132061 | 26.5342408218 |
| `baseline/current_holdover` | `validation` | `SEC_20_LAUNCH` | 7.55539987004 | 104.937020154 | 0.710217581628 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_10_STATIC` | 2.37018318158 | 0.255744174407 | 0.158494296769 |
| `candidate_1_algebraic_envelope` | `held_out` | `SEC_20_LAUNCH` | 14.6839041548 | 88.1022759743 | 0.627606363208 |
| `candidate_1_algebraic_envelope` | `held_out` | `yaw_launch` | 183.85319124 | 27.9992098482 | 24.1653419693 |
| `candidate_1_algebraic_envelope` | `validation` | `SEC_20_LAUNCH` | 4.78358597191 | 91.3921617737 | 0.585777108621 |
| `candidate_2_stribeck` | `held_out` | `SEC_10_STATIC` | 2.38347321871 | 0.26750919704 | 0.15588621288 |
| `candidate_2_stribeck` | `held_out` | `SEC_20_LAUNCH` | 13.8100897698 | 93.0614142255 | 0.727734139615 |
| `candidate_2_stribeck` | `held_out` | `yaw_launch` | 158.130531014 | 29.5047473482 | 24.6593182456 |
| `candidate_2_stribeck` | `validation` | `SEC_20_LAUNCH` | 7.8063242443 | 95.228946998 | 0.651829408199 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_10_STATIC` | 2.45642040656 | 0.337716969278 | 0.145439623211 |
| `candidate_3_load_sensitive` | `held_out` | `SEC_20_LAUNCH` | 13.135260745 | 103.45194587 | 1.02420508677 |
| `candidate_3_load_sensitive` | `held_out` | `yaw_launch` | 109.224308223 | 34.33739315 | 26.3830799468 |
| `candidate_3_load_sensitive` | `validation` | `SEC_20_LAUNCH` | 5.14418550515 | 114.218613648 | 0.795861751793 |

## Limitations

- This is a standalone residual replay, not the production UKF path.
- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.
- Small coverage buckets with fewer than three whole segments cannot populate every split.
- Full scoring is aggregate-only; it does not emit per-row diagnostics for the selected trials.
- Row-bounded runs sample evenly across each selected whole segment; split assignment still remains whole-segment.
