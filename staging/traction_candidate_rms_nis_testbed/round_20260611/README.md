# Round 2026-06-11 Traction Models

This folder prepares the next short-name standalone testbed round only. No long tuning or assessment battery has been run here.

Candidate-only launch tuning configs:

- `skew_shear.json` -> output directory `skew_shear`
- `shear_rate.json` -> output directory `shear_rate`
- `in_shear.json` -> output directory `in_shear`

Fixed inputs:

- Launch tuning manifest: `..\launch_only_tuning_oscillation_filtered_assessment_20260610\launch_only_tuning_manifest.json`
- Assessment manifest: `..\launch_only_tuning_oscillation_filtered_assessment_20260610\oscillation_filtered_assessment_manifest.json`
- Covariance/noise: `..\covariance_conservative.json`
- Bias source: `..\representative_corpus\segment_manifest.json`

Scoring remains production-measurement only: `yaw_rate_nis`, `forward_accel_nis`, and `right_accel_nis`. Residual-tail tuning uses only `yaw_rate_residual_tail`, `forward_accel_residual_tail`, and `right_accel_residual_tail`. Encoder wheel rates remain drivetrain prediction inputs and diagnostics only.

Carry-forward result names are `slip_envelope` for old `candidate_1_algebraic_envelope` and `stribeck_fade` for old `candidate_2_stribeck`; they are not retuned in this folder.

Carried-forward corrected production-measurement result folders:

- `slip_envelope` -> source `..\production_measurement_nis_launch_only_20260610\candidate_1`
- `stribeck_fade` -> source `..\production_measurement_nis_launch_only_20260610\candidate_2`
- `baseline` -> source `..\production_measurement_nis_launch_only_20260610\baseline`

Traceability map: `carried_forward_sources.json`.

These copied/summarized results use production-measurement scoring only. The source aggregate `log_parameter` values are exactly `yaw_rate_nis`, `forward_accel_nis`, and `right_accel_nis`; no encoder NIS channel is included.

| Result | Source candidate | RMS NIS | sqrt(mean NIS) | Accepted-only RMS NIS | Rejected rate | UKF one-log |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `slip_envelope` | `candidate_1_algebraic_envelope` | 70.89037127094245 | 3.6115737679205475 | 53.52521378109838 | 0.04675192524380206 | n/a |
| `stribeck_fade` | `candidate_2_stribeck` | 68.58739289450484 | 3.603516494382091 | 52.265689816338636 | 0.050867458518354745 | pass, 100 segments |
| `baseline` | `baseline/current_holdover` | 94.86815948411707 | 6.049732514869043 | 65.91163825371856 | 0.17511571885091826 | n/a |
