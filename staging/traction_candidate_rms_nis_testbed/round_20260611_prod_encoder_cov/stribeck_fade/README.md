# StribeckFade Production Encoder Covariance Retune

Date: 2026-06-11

Candidate: `candidate_2_stribeck`
Model: `stribeck_algebraic`

This run retuned and evaluated only `stribeck_fade` after the production-equivalent correlated encoder process covariance fix. Baseline, `slip_envelope`, `skew_shear`, `shear_rate`, and `in_shear` were not enabled in the tuning config or replay candidate config.

## Inputs

- Tuning manifest: `staging\traction_candidate_rms_nis_testbed\launch_only_tuning_oscillation_filtered_assessment_20260610\launch_only_tuning_manifest.json`
- Assessment manifest: `staging\traction_candidate_rms_nis_testbed\launch_only_tuning_oscillation_filtered_assessment_20260610\oscillation_filtered_assessment_manifest.json`
- Independent bias manifest: `staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Fixed covariance: `staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`
- EKF replay jobs: `2`

Scored production measurement streams were `yaw_rate_nis`, `forward_accel_nis`, and `right_accel_nis`. The replay summary reports `uses_logged_ukf_state=false`.

## Selected Parameters

```json
{
  "dynamic_to_static_grip_ratio": 0.5693935769304257,
  "low_speed_blend_mps": 0.04894609070864059,
  "peak_friction_coefficient_at_80pct_fan": 2.2465453903263963,
  "stribeck_velocity_mps": 0.02574338212170059,
  "viscous_slip_damping_n_per_mps": 16.25051035965962
}
```

## Overall EKF Assessment

| Metric | Value |
| --- | ---: |
| RMS NIS | 70.18613212241766 |
| sqrt(mean NIS) | 3.6522487486353232 |
| Accepted-only RMS NIS | 55.01662949134703 |
| Rejected rate | 0.04779790112242624 |
| NIS count | 1126221 |
| Segments | 658 |

The nominal carry-forward parameter set was also replayed as a stribeck-only check and scored worse under the same covariance: RMS NIS `71.25761974659842`, sqrt(mean) `3.694204952752459`, rejected rate `0.04904987564607657`.

## Outcome

The tuned StribeckFade trial is the better stribeck-only result found here, but the full EKF assessment remains unacceptable. Yaw launch dominates the failure: stage RMS NIS is `100.12492314712425` overall, with yaw-rate RMS NIS `134.2996196691742`, forward-accel RMS NIS `96.4989216499684`, and right-accel RMS NIS `52.216577662281054`. High yaw-command bins are especially incompatible, for example yaw-rate RMS NIS `393.143142449` at command `-0.75` and `250.493882159` at command `0.75`.

## Artifacts

- `stribeck_fade_launch_only_tuning_config.json`
- `tune\tuned_parameters.json`
- `tune\report.md`
- `assessment_ekf\summary.json`
- `assessment_ekf\candidate_rms_nis.csv`
- `assessment_ekf\itemized_rms_nis.csv`
- `assessment_ekf\nis_aggregates.csv`
- `assessment_ekf\bias_summary.csv`
- `assessment_ekf_nominal_carryforward\summary.json`
- `stage_channel_summary.csv`
- `yaw_launch_command_bin_summary.csv`
