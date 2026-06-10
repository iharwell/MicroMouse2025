# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `80`
- Skipped corrupted segments: `0`
- Source logs: `2`
- Jobs used: `2`
- Segment-row samples processed: `28000`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ekf_validation_mixed_launch_20260610_192\bias_summary.csv`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 16.5212397729 | 1.66734541178 | 84000 | 57590 | 26410 | 0.314404761905 | 80 |
| `candidate_1_algebraic_envelope` | 35.1328909674 | 1.94938777572 | 84000 | 80378 | 3622 | 0.043119047619 | 80 |
| `candidate_2_stribeck` | 40.5373729722 | 1.52007042498 | 84000 | 83415 | 585 | 0.00696428571429 | 80 |
| `candidate_3_load_sensitive` | 43.8133716404 | 1.53104505718 | 84000 | 83405 | 595 | 0.00708333333333 | 80 |
