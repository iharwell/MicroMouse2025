# Expanded Representative Fair Tuning and Validation

## Scope

- Corpus: `staging/traction_candidate_rms_nis_testbed/representative_corpus/segment_manifest.json`
- Active validation manifest: `staging/traction_candidate_rms_nis_testbed/representative_corpus_active_split_expanded_manifest.json`
- Output summary: `staging/traction_candidate_rms_nis_testbed/expanded_192_retry_analysis_summary.json`
- Writes stayed inside `Tools/TractionRmsNisTestbed/` and `staging/traction_candidate_rms_nis_testbed/`.

## Corpus Coverage

- Source logs: 8
- Primary active segments: 580
- Latest `mmlog_decode_2026-06-08_23-37-25` included: yes
- Latest mixed-launch active segments: 40
- `SEC_40_YAW` active yaw-calibration segments: 3
- Boundary-ended segments were cropped to `active_start_row_index..active_end_row_index`; terminal tails were excluded.
- Logged UKF/replay state was not consumed.

## Tuning

Command:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --config staging\traction_candidate_rms_nis_testbed\fair_tuning_config.json --manifest staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json --output-dir staging\traction_candidate_rms_nis_testbed\fair_tuning_representative_expanded_192_retry --trial-count 192 --train-segments 320 --validation-segments 220 --validation-top-k 16 --tuning-max-rows-per-segment 64 --final-max-rows-per-segment 0 --stress-max-rows-per-segment 0 --bootstrap-iterations 500
```

Artifacts:

- `staging/traction_candidate_rms_nis_testbed/fair_tuning_representative_expanded_192_retry/tuned_parameters.json`
- `staging/traction_candidate_rms_nis_testbed/fair_tuning_representative_expanded_192_retry/selected_trials.csv`
- `staging/traction_candidate_rms_nis_testbed/fair_tuning_representative_expanded_192_retry/validation_heldout_residual_tail.csv`
- `staging/traction_candidate_rms_nis_testbed/fair_tuning_representative_expanded_192_retry/stress_full_row_weighted_residual_tail.csv`

Selected trials:

| Candidate | Trial | Train Score | Validation Score |
| --- | --- | ---: | ---: |
| `baseline/current_holdover` | `baseline/current_holdover:fixed` | 4.8731 | 4.6474 |
| `candidate_1_algebraic_envelope` | `candidate_1_algebraic_envelope:trial_039` | 3.5460 | 3.3981 |
| `candidate_2_stribeck` | `candidate_2_stribeck:trial_163` | 3.5632 | 3.5126 |
| `candidate_3_load_sensitive` | `candidate_3_load_sensitive:trial_156` | 3.5332 | 3.4674 |

Bootstrap held-out deltas vs holdover were negative for all candidates, but the held-out source-log count was 1, so the interval is degenerate and should be treated as a directional check, not a broad uncertainty estimate.

## Tuned Parameters

`candidate_1_algebraic_envelope:trial_039`

```json
{
  "combined_slip_envelope_exponent": 2.3237727456412545,
  "lateral_slip_gain_n_per_mps": 18.340643057676093,
  "longitudinal_slip_gain_n_per_mps": 10.398425292704584,
  "low_speed_blend_mps": 0.018100549431756083,
  "peak_friction_coefficient_at_80pct_fan": 2.6123533679566027
}
```

`candidate_2_stribeck:trial_163`

```json
{
  "dynamic_to_static_grip_ratio": 0.9962129841791834,
  "low_speed_blend_mps": 0.07910722714553715,
  "peak_friction_coefficient_at_80pct_fan": 1.2586739940417646,
  "stribeck_velocity_mps": 0.05661644873708689,
  "viscous_slip_damping_n_per_mps": 23.150568435615085
}
```

`candidate_3_load_sensitive:trial_156`

```json
{
  "lateral_load_sensitivity": -0.3048196806534562,
  "lateral_slip_gain_n_per_mps": 76.12769034441766,
  "longitudinal_load_sensitivity": 0.12641988549386207,
  "longitudinal_slip_gain_n_per_mps": 18.240754201111233,
  "low_speed_blend_mps": 0.11174895899412565,
  "peak_friction_coefficient_at_80pct_fan": 2.6348427497751974,
  "yaw_coupling_gain": -0.30382776518825744
}
```

Covariance policy: fixed `staging/traction_candidate_rms_nis_testbed/covariance_conservative.json`; no covariance tuning or inflation.

## EKF Active Validation

Command timeout: `timeout_ms=7200000` (2 hours). No row cap was used.

Command:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py replay --candidate-config staging\traction_candidate_rms_nis_testbed\fair_tuning_representative_expanded_192_retry\tuned_parameters.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --segment-manifest staging\traction_candidate_rms_nis_testbed\representative_corpus_active_split_expanded_manifest.json --output-dir staging\traction_candidate_rms_nis_testbed\ekf_validation_active_expanded_192_retry --replay-mode ekf --include-corrupted --jobs 8
```

Artifacts:

- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/summary.json`
- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/candidate_rms_nis.csv`
- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/stage_channel_summary.csv`
- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/sec40_yaw_summary.csv`
- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/launch_command_bin_summary.csv`
- `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/comparison_to_holdover.csv`

Summary:

- Processed segments: 580
- Processed active samples: 243177
- Source logs: 8
- Skipped corrupted segments: 0
- Logged UKF state used: false

All-split EKF comparison:

| Candidate | RMS NIS | Rejected Rate | Physical Residual RMS |
| --- | ---: | ---: | ---: |
| `baseline/current_holdover` | 3.5292 | 0.4550 | 3331.7612 |
| `candidate_1_algebraic_envelope` | 4.5236 | 0.0671 | 4.5891 |
| `candidate_2_stribeck` | 4.6722 | 0.0676 | 4.2239 |
| `candidate_3_load_sensitive` | 4.1498 | 0.1145 | 3.9967 |

Held-out EKF comparison:

| Candidate | RMS NIS | Rejected Rate | Physical Residual RMS |
| --- | ---: | ---: | ---: |
| `baseline/current_holdover` | 4.6387 | 0.3390 | 13.3406 |
| `candidate_1_algebraic_envelope` | 3.6310 | 0.0083 | 2.3713 |
| `candidate_2_stribeck` | 4.3016 | 0.0157 | 2.7642 |
| `candidate_3_load_sensitive` | 3.8671 | 0.0511 | 2.8167 |

Validation EKF comparison:

| Candidate | RMS NIS | Rejected Rate | Physical Residual RMS |
| --- | ---: | ---: | ---: |
| `baseline/current_holdover` | 3.4234 | 0.4912 | 7166.1515 |
| `candidate_1_algebraic_envelope` | 4.4972 | 0.1151 | 5.8345 |
| `candidate_2_stribeck` | 4.6182 | 0.1159 | 6.2864 |
| `candidate_3_load_sensitive` | 4.0600 | 0.1532 | 4.5473 |

## SEC_40_YAW EKF Metrics

Full details: `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/sec40_yaw_summary.csv`

All-split highlights:

| Candidate | Channel | Count | Accepted | Rejected Rate | RMS NIS | Physical RMS |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | forward | 18087 | 11681 | 0.3542 | 2.9299 | 13911.7508 |
| `baseline/current_holdover` | right | 18087 | 11492 | 0.3646 | 2.5874 | 13901.6416 |
| `baseline/current_holdover` | yaw_rate | 18087 | 69 | 0.9962 | 12.3059 | 375.9369 |
| `candidate_1_algebraic_envelope` | forward | 18087 | 12344 | 0.3175 | 3.9557 | 9.4588 |
| `candidate_1_algebraic_envelope` | right | 18087 | 12259 | 0.3222 | 4.7735 | 9.7149 |
| `candidate_1_algebraic_envelope` | yaw_rate | 18087 | 11051 | 0.3890 | 9.5493 | 11.4847 |
| `candidate_2_stribeck` | forward | 18087 | 12782 | 0.2933 | 4.7863 | 9.8586 |
| `candidate_2_stribeck` | right | 18087 | 12465 | 0.3108 | 5.3927 | 10.2761 |
| `candidate_2_stribeck` | yaw_rate | 18087 | 11344 | 0.3728 | 8.6955 | 12.2263 |
| `candidate_3_load_sensitive` | forward | 18087 | 12902 | 0.2867 | 4.0254 | 6.5359 |
| `candidate_3_load_sensitive` | right | 18087 | 13045 | 0.2788 | 4.0320 | 6.9250 |
| `candidate_3_load_sensitive` | yaw_rate | 18087 | 153 | 0.9915 | 9.2848 | 9.2315 |

## Launch Command Bins

Full launch-bin metrics: `staging/traction_candidate_rms_nis_testbed/ekf_validation_active_expanded_192_retry/launch_command_bin_summary.csv`

The command-bin file groups by candidate, split, stage, channel, and per-row command signature. For example, `candidate_1_algebraic_envelope` on `mixed_launch` produced 30 all-split rows: 10 command signatures x 3 channels. Rejected rates across those `mixed_launch` command bins were low, mostly 0.0 to 0.015.

## UKF-Style Sigma Validation

Command timeout: `timeout_ms=7200000` (2 hours).

Command:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py validate-ukf --candidate-config staging\traction_candidate_rms_nis_testbed\fair_tuning_representative_expanded_192_retry\tuned_parameters.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --segment-manifest staging\traction_candidate_rms_nis_testbed\representative_corpus_active_split_expanded_manifest.json --output-dir staging\traction_candidate_rms_nis_testbed\ukf_validation_active_expanded_192_retry --include-corrupted --max-rows-per-segment 16
```

Artifacts:

- `staging/traction_candidate_rms_nis_testbed/ukf_validation_active_expanded_192_retry/ukf_validation_summary.json`
- `staging/traction_candidate_rms_nis_testbed/ukf_validation_active_expanded_192_retry/ukf_validation_candidate_summary.csv`
- `staging/traction_candidate_rms_nis_testbed/ukf_validation_active_expanded_192_retry/ukf_validation_events.csv`
- `staging/traction_candidate_rms_nis_testbed/ukf_validation_active_expanded_192_retry/ukf_validation_report.md`

Summary:

- Segment set: full expanded active manifest, 580 segments.
- Sampling: 16 representative rows per segment, 9280 samples total.
- Sigma policy: `2N+1` diagonal sigma points from fixed testbed covariance.
- Sigma points per candidate: 176320.
- Status: pass for all four candidates.
- Events: none.

## Tests

```powershell
python -m unittest discover Tools\TractionRmsNisTestbed
python -m unittest discover -s Tools\TractionRmsNisTestbed\tests
```

Results:

- Top-level discovery: 12 tests passed.
- Direct test discovery: 28 tests passed.

## Limitations

- This is a standalone testbed replay, not production `PlantModel`, production `Estimator`, `OpenFloorUkfReplay`, or `MazeMapTest`.
- The UKF-style validation used the full active segment set but representative row sampling (`--max-rows-per-segment 16`), not a full-row sigma pass.
- EKF metrics are accepted-row RMS plus explicit rejection rates. The holdover's accepted-only RMS can look lower while its rejection rate and physical residuals are pathological.
- Stage/channel `physical_residual_rms_weighted` is derived from aggregate rows and weighted by aggregate count; candidate-level physical residual RMS in `candidate_rms_nis.csv` is the authoritative aggregate for overall physical residuals.
- The first expanded 192 tuning attempt failed during uncapped final evaluation because a standalone holdover replay state overflowed. The testbed now records overflow/non-finite predictions as rejected validation observations instead of terminating, with unit coverage for that behavior.
