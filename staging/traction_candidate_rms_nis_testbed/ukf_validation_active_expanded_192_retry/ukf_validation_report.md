# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus_active_split_expanded_manifest.json`
- Processed segments: `580`
- Source logs: `8`
- Representative row samples: `9280`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | pass | 9280 | 176320 | 0 | 14931 | 790.200929569 | 16.0630975181 | 618.630422837 |
| `candidate_1_algebraic_envelope` | pass | 9280 | 176320 | 0 | 14931 | 1059.36665835 | 15.864593274 | 656.864840026 |
| `candidate_2_stribeck` | pass | 9280 | 176320 | 0 | 14931 | 504.094261385 | 16.8192200483 | 196.040136605 |
| `candidate_3_load_sensitive` | pass | 9280 | 176320 | 0 | 14931 | 1038.2644657 | 15.9578625109 | 357.388234548 |
