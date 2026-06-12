# SlipEnvelope Production Encoder Covariance Retune

Scope: `candidate_1_algebraic_envelope` / `slip_envelope` only.

Inputs:

- Covariance: `staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`
- Launch tuning manifest: `staging\traction_candidate_rms_nis_testbed\launch_only_tuning_oscillation_filtered_assessment_20260610\launch_only_tuning_manifest.json`
- Assessment manifest: `staging\traction_candidate_rms_nis_testbed\launch_only_tuning_oscillation_filtered_assessment_20260610\oscillation_filtered_assessment_manifest.json`
- Bias manifest: `staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`

The first 193-trial launch-only retune selected `candidate_1_algebraic_envelope:trial_031`. Because that selected point was near search bounds, an expanded 257-trial slip-only retune was run. The expanded run selected the same parameter set as its nominal trial and did not improve the EKF assessment.

Final tuned parameters:

```json
{
  "peak_friction_coefficient_at_80pct_fan": 1.1091281882474364,
  "longitudinal_slip_gain_n_per_mps": 87.423645020917,
  "lateral_slip_gain_n_per_mps": 95.30906226734128,
  "combined_slip_envelope_exponent": 5.410495443316217,
  "low_speed_blend_mps": 0.2753939095614226
}
```

Final EKF assessment:

| Metric | Value |
| --- | ---: |
| Overall RMS NIS | 72.44479815679935 |
| sqrt(mean NIS) | 3.617493845478291 |
| Accepted-only RMS NIS | 57.45856830663309 |
| Rejected rate | 0.044511689979142635 |
| NIS count | 1126221 |
| Segments | 658 |
| Jobs requested/used | 2 / 2 |
| Logged UKF state used | false |

Stage breakdown is in `stage_breakdown_expanded.csv`. The dominant failure remains `yaw_launch`, with RMS NIS `104.7800663408554`, rejected rate `0.08514244810307803`, and accepted-only RMS NIS `85.27983628926849`.

Yaw-launch stream breakdown:

| Stream | RMS NIS | sqrt(mean) | Rejected rate |
| --- | ---: | ---: | ---: |
| `forward_accel_nis` | 103.3711407039546 | 5.812025721632607 | 0.1500586972083035 |
| `right_accel_nis` | 48.58584761145012 | 3.7804554916214004 | 0.10536864710093057 |
| `yaw_rate_nis` | 141.03336338832153 | 5.253124211004359 | 0 |

High-command yaw-launch bins are in `yaw_launch_high_command_bins_expanded.csv`. The worst production streams remain the +/-0.8 in-place yaw launch bins, including `yaw_rate_nis` RMS NIS `222.6975829158358` for `pair=-0.8,0.8` and `167.27484678582798` for `pair=0.8,-0.8`.

Conclusion: after the production-equivalent correlated encoder process covariance fix, the slip-envelope model remains fundamentally incompatible with the yaw-launch production measurement streams under this methodology. Widening the slip-only search did not find a better feasible parameter set.

