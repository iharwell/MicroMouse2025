# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ukf_validation_representative_subset_manifest.json`
- Processed segments: `152`
- Source logs: `6`
- Representative row samples: `1216`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | pass | 1216 | 23104 | 0 | 2069 | 693.708913347 | 16.7948433714 | 500.935279508 |
| `candidate_1_algebraic_envelope` | pass | 1216 | 23104 | 0 | 2069 | 637.379068236 | 19.9648628075 | 261.419949067 |
| `candidate_2_stribeck` | pass | 1216 | 23104 | 0 | 2069 | 693.435096391 | 22.8959134771 | 345.877000088 |
| `candidate_3_load_sensitive` | pass | 1216 | 23104 | 0 | 2069 | 939.026764896 | 18.1481090297 | 253.747208198 |
