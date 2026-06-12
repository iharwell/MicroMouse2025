# Production Measurement NIS Launch-Only Run Setup

This directory is the clean handoff point for the corrected production-equivalent measurement objective.

- Launch-only tuning manifest: `..\launch_only_tuning_oscillation_filtered_assessment_20260610\launch_only_tuning_manifest.json` (`638` segments).
- Oscillation-filtered assessment manifest: `..\launch_only_tuning_oscillation_filtered_assessment_20260610\oscillation_filtered_assessment_manifest.json` (`658` segments).
- Fixed covariance/noise schedule: `..\covariance_conservative.json`.
- Independent bias source: `..\representative_corpus\segment_manifest.json`.

Production-equivalent NIS scoring uses only:

- `yaw_rate_nis`
- `forward_accel_nis`
- `right_accel_nis`

Residual-tail launch tuning uses only:

- `yaw_rate_residual_tail`
- `forward_accel_residual_tail`
- `right_accel_residual_tail`

Encoder wheel-rate data remains a plant/prediction input and may appear in diagnostics, but encoder NIS or encoder residual tails are not production-equivalent score evidence.
