# UKF Sigma-Point Validation Report

- Validation kind: `diagonal_sigma_point_candidate_plant`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ukf_validation_5log_manifest.json`
- Processed segments: `5`
- Source logs: `5`
- Representative row samples: `80`
- Sigma policy: `2N+1 diagonal sigma points from fixed testbed covariance`
- Uses logged UKF state: `false`
- Overall status: `pass`

| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | pass | 80 | 1520 | 0 | 142 | 319.273115727 | 15.3324220167 | 197.646586299 |
| `candidate_1_algebraic_envelope` | pass | 80 | 1520 | 0 | 142 | 129.422070262 | 15.3232328243 | 1.81977561448 |
| `candidate_2_stribeck` | pass | 80 | 1520 | 0 | 142 | 127.715811803 | 15.3236086883 | 1.78038476544 |
| `candidate_3_load_sensitive` | pass | 80 | 1520 | 0 | 142 | 126.211525905 | 15.3242319299 | 1.92791240552 |
