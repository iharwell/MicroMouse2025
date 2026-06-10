# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `101`
- Skipped corrupted segments: `0`
- Source logs: `1`
- Jobs used: `1`
- Segment-row samples processed: `61595`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\ekf_validation_new_run_20260610_192\bias_summary.csv`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 116.896781754 | 5.4927605473 | 184785 | 137901 | 46884 | 0.253721893011 | 101 |
| `candidate_1_algebraic_envelope` | 184.410153485 | 5.41705068821 | 184785 | 150340 | 34445 | 0.186405822983 | 101 |
| `candidate_2_stribeck` | 171.048245596 | 5.23526975026 | 184785 | 156547 | 28238 | 0.152815434153 | 101 |
| `candidate_3_load_sensitive` | 224.522624897 | 5.84206777025 | 184785 | 155440 | 29345 | 0.158806180155 | 101 |
