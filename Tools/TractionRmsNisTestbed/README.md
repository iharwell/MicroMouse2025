# Traction ANIS/Residual Testbed

Standalone Python harness for traction-model ANIS and residual work without production or hardware hooks.

## What It Does

- Generates bounded trial configs per candidate from `parameters` plus `search` ranges.
- Splits by whole `segment_id`; missing splits use a stable hash.
- Scores all non-terminal-external-force segments from manifest-listed estimator artifacts.
- Replays a host-only 9-state estimator/model core from sensor inputs and candidate JSON when estimator artifacts need to be generated without production hooks.
- Writes aggregate replay scoring artifacts by default so full-manifest runs do not produce row-scale CSVs.
- Reports itemized all-finite ANIS by candidate, trial, split, stage, parameter field, log field, and launch per-row command bucket.
- Keeps rejected finite accelerometer rows in the main ANIS averages and reports accepted-only accel RMS as an explicit diagnostic.
- Forces yaw/gyro and left/right encoder NIS rows accepted for testbed tuning/evaluation when finite; yaw and encoder gate settings are effectively disabled.
- Treats high yaw/gyro or encoder NIS as model failure evidence, not sensor corruption.
- Estimates accelerometer bias independently from stationary/static/bias assessment rows and applies that correction before replay/tuning evaluation.
- Keeps yaw launch and yaw maneuver sections as primary active calibration data alongside launches, straight runs, and smooth turns.
- Reports physical residual RMS for yaw rate, yaw acceleration, forward acceleration, and right acceleration.
- Rejects NIS CSVs containing logged UKF state columns.
- For active-only replay or UKF validation, accepts `--bias-segment-manifest` so static/stationary bias assessment can come from the full representative corpus while metrics are computed only over active rows.

## Commands

From the repo root:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py plan
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py score --manifest staging\traction_candidate_rms_nis_segments\segment_manifest.json --nis-artifact staging\traction_candidate_rms_nis_testbed\full_default_replay\nis_samples.csv
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py replay --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --segment-manifest staging\traction_candidate_rms_nis_testbed\smoke_segments.json --output-dir staging\traction_candidate_rms_nis_testbed\smoke_output
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py replay --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --segment-manifest staging\traction_candidate_rms_nis_testbed\smoke_segments.json --output-dir staging\traction_candidate_rms_nis_testbed\smoke_output_rows --write-row-artifacts
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py validate-ukf
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --output-dir staging\traction_candidate_rms_nis_testbed\broad_tuning_latest
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --config staging\traction_candidate_rms_nis_testbed\fair_tuning_config.json --output-dir staging\traction_candidate_rms_nis_testbed\fair_tuning_latest
python -m unittest discover Tools\TractionRmsNisTestbed
```

Default inputs live in `staging/traction_candidate_rms_nis_testbed/`. `score` writes `last_run/`. For the real segment manifest, run `replay` first, then pass the generated `nis_samples.csv` through `--nis-artifact`.

`replay` writes `nis_aggregates.csv`, `itemized_rms_nis.csv`, `candidate_rms_nis.csv`, and `summary.json` by default. EKF replay computes scalar NIS from `H P H^T + R`; aggregate RMS NIS and sqrt(mean NIS) use all finite rows. Accelerometer rejected rows are counted and still remain in the main all-finite average; accepted-only accelerometer RMS is diagnostic. Yaw/gyro and left/right encoder rows are ungated and always accepted when finite. Launch stages are bucketed by per-row command bins. Use `--write-row-artifacts` for small debug runs that also need `nis_samples.csv` and `residual_diagnostics.csv`.

The replay state is the theory-spec 9-state body model: `px_m`, `py_m`, `heading_rad`, `vf_mps`, `vr_mps`, `yaw_rate_radps`, `delta_af_mps2`, `delta_ar_mps2`, and `delta_yaw_accel_radps2`. Source CSVs may contain logged `ukf_state_*` columns, but the replay does not read them and its artifacts do not include them.

`validate-ukf` runs a bounded UKF-relevance check over `staging/traction_candidate_rms_nis_testbed/ukf_validation_5log_manifest.json`. It propagates 2N+1 diagonal sigma points from the fixed covariance through each standalone candidate plant on representative rows, checks finite and continuity behavior, validates covariance/innovation sanity where measurements are available, and probes Vf/yaw-rate zero crossings. It writes `ukf_validation_summary.json`, `ukf_validation_candidate_summary.csv`, `ukf_validation_events.csv`, and `ukf_validation_report.md`.

`tune` runs bounded Latin-hypercube candidate-parameter search. The fair config reserves whole source logs for held-out first, assigns the remaining data by whole segment, selects on metric-balanced active traction, yaw calibration, and encoder residual rows, keeps covariance/noise fixed from the single covariance config for every candidate and holdover, and writes full-row residual-tail diagnostics separately. Yaw calibration rows are primary active data, not stress/outlier buckets. Explicit `accel_valid` or `imu_accel_valid` columns control accelerometer validity; `accel_bias_valid` is retained as metadata only and is not a rejection gate.

`replay` and `validate-ukf` write `bias_summary.csv` when run. The bias summary reports the per-log static/stationary accelerometer correction applied to active samples plus the gyro-bias value observed from `raw_gyro - used_gyro` or the logged gyro-bias column.

## NIS CSV Contract

Long-form CSVs must contain:

- `candidate_id`
- `segment_id`
- `stage`
- `log_field` or `log_parameter`
- `nis`

Optional columns:

- `trial_id`
- `split`
- `corrupted`
- `measurement_dimension`
- `parameter_field`
- `parameter_value_kind`
- `parameter_value`

Wide-form CSVs may use `*_nis` columns instead of `log_field,nis`. `last_update_nis` is ignored to avoid double-counting update summaries.

Columns beginning with `ukf_state` or `logged_ukf_state` are rejected. This path must consume measurement residual/NIS artifacts, not logged estimator state.
Accepted/rejected flags on yaw/gyro and encoder NIS rows are ignored by scoring; finite rows are retained. Accelerometer accepted-only metrics are diagnostic and the main score uses all finite accelerometer NIS.
