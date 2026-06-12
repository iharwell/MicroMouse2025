# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\launch_only_tuning_oscillation_filtered_assessment_20260610\launch_only_tuning_manifest.json`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Processed segments: `3`
- Source logs: `1`
- Representative row samples: `9`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\codex_perf_ukf_validation_3seg\bias_summary.csv`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | pass | 9 | 171 | 0 | 18 | 32.9382115346 | 15.3282830359 | 179.583407421 |
| `candidate_1_algebraic_envelope` | pass | 9 | 171 | 0 | 18 | 31.3979455266 | 15.3215922181 | 0.16460395896 |
| `candidate_2_stribeck` | pass | 9 | 171 | 0 | 18 | 42.0714905533 | 15.3220231597 | 0.190678039309 |
| `candidate_3_load_sensitive` | pass | 9 | 171 | 0 | 18 | 50.5590540735 | 15.3222162016 | 0.147256710654 |
