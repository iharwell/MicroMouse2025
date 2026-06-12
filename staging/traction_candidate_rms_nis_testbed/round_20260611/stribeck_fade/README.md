# StribeckFade Carried-Forward Result

Source: `../../production_measurement_nis_launch_only_20260610/candidate_2`
Candidate id: `candidate_2_stribeck`
Model: `stribeck_algebraic`

No tuning, replay, or model execution was run in this folder. Files here are copied or compactly summarized from the corrected 2026-06-10 production-measurement assessment.

Scoring confirmation: `yaw_rate_nis`, `forward_accel_nis`, and `right_accel_nis` only. No encoder NIS channels are present in the source `nis_aggregates.csv` `log_parameter` values.

## Overall RMS NIS

| Metric | Value |
| --- | ---: |
| RMS NIS | 68.587392894504845 |
| sqrt(mean NIS) | 3.6035164943820912 |
| Accepted-only RMS NIS | 52.265689816338636 |
| Rejected rate | 0.050867458518354745 |
| NIS count | 1126221 |
| Segments | 658 |

## Files

- `summary.json` - compact traceable summary with source paths, selected parameters, and scoring confirmation.
- `candidate_rms_nis.csv` - copied overall/source split RMS NIS table.
- `stage_channel_summary.csv` - compact stage/channel reduction from source `nis_aggregates.csv` using `split == all`.
- `assessment_summary.json` and `assessment_report.md` - copied source assessment summary/report.
- `ukf_one_log_summary.json`, `ukf_one_log_candidate_summary.csv`, and `ukf_one_log_report.md` - copied one-log UKF validation result.

## UKF One-Log Result

Status: `pass`; segments: `100`; samples: `125088`; issue_count: `0`; max_innovation_nis: `894.28720902412749`.
