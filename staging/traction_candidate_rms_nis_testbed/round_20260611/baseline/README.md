# HoldoverBaseline Carried-Forward Result

Source: `../../production_measurement_nis_launch_only_20260610/baseline`
Candidate id: `baseline/current_holdover`
Model: `current_holdover_approximation`

No tuning, replay, or model execution was run in this folder. Files here are copied or compactly summarized from the corrected 2026-06-10 production-measurement assessment.

Scoring confirmation: `yaw_rate_nis`, `forward_accel_nis`, and `right_accel_nis` only. No encoder NIS channels are present in the source `nis_aggregates.csv` `log_parameter` values.

## Overall RMS NIS

| Metric | Value |
| --- | ---: |
| RMS NIS | 94.868159484117072 |
| sqrt(mean NIS) | 6.0497325148690431 |
| Accepted-only RMS NIS | 65.911638253718564 |
| Rejected rate | 0.17511571885091826 |
| NIS count | 1126221 |
| Segments | 658 |

## Files

- `summary.json` - compact traceable summary with source paths, selected parameters, and scoring confirmation.
- `candidate_rms_nis.csv` - copied overall/source split RMS NIS table.
- `stage_channel_summary.csv` - compact stage/channel reduction from source `nis_aggregates.csv` using `split == all`.
- `assessment_summary.json` and `assessment_report.md` - copied source assessment summary/report.
