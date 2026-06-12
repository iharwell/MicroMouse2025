# InShear Production Encoder Covariance Retune

Date: 2026-06-11

Candidate: `in_shear`
Model: `in_shear`

This run retuned and evaluated only `in_shear` after the production-equivalent correlated encoder process covariance fix. Baseline, `slip_envelope`, `stribeck_fade`, `skew_shear`, and `shear_rate` were not enabled in the tuning or replay candidate configs.

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
  "combined_slip_envelope_exponent": 4.621170901317498,
  "inward_lateral_grip_gain": -0.19085731821960336,
  "inward_lateral_stiffness_gain": 0.46083919054768296,
  "inward_shear_blend_speed_mps": 0.11927884329967024,
  "lateral_slip_gain_n_per_mps": 29.14860684584759,
  "longitudinal_slip_gain_n_per_mps": 26.979203748661323,
  "low_speed_blend_mps": 0.030364148614410075,
  "peak_friction_coefficient_at_80pct_fan": 1.339762366593536
}
```

Selected trial: `in_shear:trial_120`.

## Overall EKF Assessment

| Metric | Value |
| --- | ---: |
| RMS NIS | 74.53922845520938 |
| sqrt(mean NIS) | 3.670411754717049 |
| Accepted-only RMS NIS | 60.279918490622734 |
| Rejected rate | 0.046695098031381055 |
| NIS count | 1126221 |
| Segments | 658 |

## Outcome

The tuned `in_shear` result remains unacceptable under the production-equivalent encoder covariance. The failure is dominated by yaw-launch: stage RMS NIS is `105.70706393903583`, with stream RMS NIS `146.1766802768574` for `yaw_rate_nis`, `99.3253404974415` for `forward_accel_nis`, and `47.84145668914098` for `right_accel_nis`.

High-command yaw-launch bins are fundamentally incompatible with this model shape. At command `-0.8`, the active yaw bin has RMS NIS `245.69484053846597` for `yaw_rate_nis`, `261.73240164993797` for `forward_accel_nis`, and `78.75151209468933` for `right_accel_nis`. At command `0.8`, the active yaw bin has RMS NIS `164.84682987414575`, `236.61210817144024`, and `125.14710951027905` respectively.

## Artifacts

- `in_shear_launch_tuning.json`
- `in_shear_tuned_only_config.json`
- `tune\tuned_parameters.json`
- `tune\report.md`
- `assessment_ekf\summary.json`
- `assessment_ekf\candidate_rms_nis.csv`
- `assessment_ekf\itemized_rms_nis.csv`
- `assessment_ekf\nis_aggregates.csv`
- `assessment_ekf\bias_summary.csv`
- `stage_breakdown.csv`
- `stage_stream_breakdown.csv`
- `yaw_launch_stream_breakdown.csv`
- `yaw_launch_command_bins.csv`
- `yaw_launch_high_command_bins.csv`
