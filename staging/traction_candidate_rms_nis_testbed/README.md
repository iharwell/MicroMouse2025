# Traction ANIS/Residual Testbed

This staging area belongs to `Tools/TractionRmsNisTestbed`, a standalone host-only Python replay tool. It does not modify, import, link, or call production/hardware code.

The replay uses sensor and command inputs only. Logged `ukf_state_*` columns may be present in source CSVs, but the testbed never reads them and never writes them to estimator artifacts.

All replay, tuning, and validation runs use `covariance_conservative.json` as the fixed process-noise and measurement-noise schedule for the holdover and every candidate. Candidate configs must not tune, scale, inflate, or otherwise alter covariance/noise.

Artifacts produced by the tool:

- `nis_samples.csv`: long-form accepted/rejected estimator rows with true EKF NIS where replay mode is `ekf`, including left/right encoder, yaw/gyro, and accelerometer streams.
- `residual_diagnostics.csv`: deterministic residual/contact diagnostics, including measured/predicted/residual encoder, yaw-rate, and yaw-accel fields.
- `summary.json`: per-candidate all-finite ANIS, accepted-only diagnostics, rejected-rate, and physical residual RMS summary.
- `last_run/trial_plan.csv`: bounded candidate trial grid for the scoring scaffold.
- `last_run/itemized_rms_nis.csv`: all-finite ANIS itemized by split, stage, log field, parameter field, and launch per-row command bucket.
- `last_run/candidate_rankings.csv`: ranking output using the configured selection split.

Yaw launch and yaw maneuver sections are primary active calibration data. Stationary/bias rows are itemized separately, and only terminal pickup/runoff/external-force boundary corruption is excluded from scoring. Finite yaw/gyro and encoder NIS rows are always retained in the main score; high NIS is model failure evidence, not sensor corruption. Finite accelerometer NIS rows are also retained in the main all-finite score, with accepted-only accelerometer RMS reported only as a diagnostic.

Scoring scaffold commands from the repo root:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py plan
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py score
```

`score` reads the generated segment manifest by default. For real scoring, first run replay over `staging/traction_candidate_rms_nis_segments/segment_manifest.json`, then pass the generated `nis_samples.csv` to `score` with `--nis-artifact`.

Smoke command from the repo root:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py replay --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --segment-manifest staging\traction_candidate_rms_nis_testbed\smoke_segments.json --output-dir staging\traction_candidate_rms_nis_testbed\smoke_output
```

UKF-relevance validation command from the repo root:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py validate-ukf
```

`validate-ukf` uses `ukf_validation_5log_manifest.json` by default. It is a standalone diagonal sigma-point propagation check, not a production UKF replay. It writes `ukf_validation_5log/ukf_validation_summary.json`, `ukf_validation_5log/ukf_validation_candidate_summary.csv`, `ukf_validation_5log/ukf_validation_events.csv`, and `ukf_validation_5log/ukf_validation_report.md`.

For active-only manifests, pass `--bias-segment-manifest staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json` so static/stationary bias rows remain available while replay/validation metrics stay active-only. Replay and UKF validation write `bias_summary.csv` with the per-log accelerometer and gyro bias values used.

Fair corrected-testbed tuning runner:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --config staging\traction_candidate_rms_nis_testbed\fair_tuning_config.json --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --output-dir staging\traction_candidate_rms_nis_testbed\fair_tuning_latest
```

See `fair_tuning_plan.md` for the held-out source-log split policy, primary active traction and yaw calibration objective, bootstrap output, and expected files.
