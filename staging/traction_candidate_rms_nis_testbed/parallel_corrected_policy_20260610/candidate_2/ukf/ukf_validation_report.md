# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus_active_split_20260610_manifest.json`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Processed segments: `562`
- Source logs: `8`
- Representative row samples: `261616`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\parallel_corrected_policy_20260610\candidate_2\ukf\bias_summary.csv`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `candidate_2_stribeck` | pass | 261616 | 4970704 | 0 | 370229 | 1197.58914823 | 15.946837688 | 1841.02463803 |
