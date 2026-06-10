# Corrected Testbed Fair Tuning Plan

This plan uses only the standalone `Tools/TractionRmsNisTestbed` runner and staging config. It does not touch production code, production covariance, OpenFloorUkfReplay, PlantModel, Estimator, or MazeMapTest.

## Policy

1. Split whole source logs into held-out first.
2. Split the remaining non-held-out data by whole segment into train and validation.
3. Tune only physical traction candidate parameters. The covariance file is fixed and no covariance/noise search fields are allowed.
4. Use `staging/traction_candidate_rms_nis_testbed/covariance_conservative.json` as the single fixed process-noise and measurement-noise schedule for holdover and every candidate.
5. Select candidates on active traction, yaw calibration, and encoder measurement metrics first:
   - `launch_active_pulse`
   - `yaw_launch`
   - `yaw_calibration`
   - `mixed_launch`
   - `straight_active`
   - `smooth_turn_active`
6. Retain left/right encoder and gyro/yaw-rate NIS in main scoring. Their gates are effectively disabled for finite rows; high NIS is model failure evidence, not sensor corruption.
7. Treat yaw launch and yaw maneuver/calibration sections as valid primary calibration moves, not optional stress or outlier-only data.
8. Keep active rows before explicit pickup/runoff/terminal external-force boundaries; exclude boundary-corrupted full-row tails from stress diagnostics.
9. Keep the full row-weighted residual tail as a stress diagnostic only.
10. Compare selected candidates against the fixed holdover baseline and write source-log bootstrap confidence on held-out logs.

## Runner

From the repo root:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --config staging\traction_candidate_rms_nis_testbed\fair_tuning_config.json --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --output-dir staging\traction_candidate_rms_nis_testbed\fair_tuning_latest
```

For a faster integration check:

```powershell
python Tools\TractionRmsNisTestbed\traction_rms_nis_testbed.py tune --config staging\traction_candidate_rms_nis_testbed\fair_tuning_config.json --candidate-config staging\traction_candidate_rms_nis_testbed\candidates.json --covariance-config staging\traction_candidate_rms_nis_testbed\covariance_conservative.json --output-dir staging\traction_candidate_rms_nis_testbed\fair_tuning_smoke --trial-count 8 --train-segments 32 --validation-segments 24 --validation-top-k 3 --bootstrap-iterations 50
```

## Expected Outputs

- `report.md`: human-readable selected trials, validation/held-out active RMS NIS, and bootstrap table.
- `tuning_summary.json`: machine-readable run policy, limits, selected trials, split counts, and bootstrap rows.
- `tuned_parameters.json`: selected standalone testbed candidate parameters. This is not production config.
- `selected_trials.csv`: train and validation scores for selected trials.
- `trial_scores.csv`: train-screen and validation-select scores for all evaluated trials.
- `validation_heldout_residual_tail.csv`: primary active/yaw-calibration validation and held-out RMS residual rows.
- `stress_full_row_weighted_residual_tail.csv`: full-row residual-tail stress diagnostic for selected trials.
- `source_log_bootstrap_confidence.csv`: source-log bootstrap deltas versus holdover on held-out logs.
- `source_log_splits.csv`: audit table proving held-out source logs stay whole.
- `split_counts.csv`, `train_subset_segments.csv`, `validation_subset_segments.csv`: split and subset audit files.
