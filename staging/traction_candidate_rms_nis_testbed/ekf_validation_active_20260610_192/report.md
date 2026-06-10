# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `562`
- Skipped corrupted segments: `0`
- Source logs: `8`
- Jobs used: `8`
- Segment-row samples processed: `261616`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ekf_validation_active_20260610_192\bias_summary.csv`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 140.790675522 | 4.62029093851 | 784848 | 638116 | 146732 | 0.18695594561 | 562 |
| `candidate_1_algebraic_envelope` | 256.293825308 | 5.13944002737 | 784848 | 714710 | 70138 | 0.0893650745112 | 562 |
| `candidate_2_stribeck` | 138.255220214 | 4.04896013325 | 784848 | 725902 | 58946 | 0.0751049884818 | 562 |
| `candidate_3_load_sensitive` | 180.314858322 | 4.41195908758 | 784848 | 724114 | 60734 | 0.0773831366073 | 562 |
