# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus_active_split_20260610_manifest.json`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Processed segments: `562`
- Source logs: `8`
- Representative row samples: `8992`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ukf_validation_active_20260610_192\bias_summary.csv`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | pass | 8992 | 170848 | 0 | 13667 | 790.200929569 | 16.0630975181 | 1209.22282684 |
| `candidate_1_algebraic_envelope` | pass | 8992 | 170848 | 0 | 13667 | 1358.19194415 | 31.7964218736 | 911.448819832 |
| `candidate_2_stribeck` | pass | 8992 | 170848 | 0 | 13667 | 1183.52410699 | 15.946837688 | 862.540556548 |
| `candidate_3_load_sensitive` | pass | 8992 | 170848 | 0 | 13667 | 1182.3570216 | 16.1754199024 | 1742.8799855 |
