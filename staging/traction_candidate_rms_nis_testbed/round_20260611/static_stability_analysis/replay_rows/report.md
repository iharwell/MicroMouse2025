# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `1`
- Skipped corrupted segments: `0`
- Source logs: `1`
- Jobs used: `1`
- Segment-row samples processed: `30523`
- Row artifacts enabled: `true`
- Uses logged UKF state: `false`
- Bias source manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\representative_corpus\segment_manifest.json`
- Bias summary CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\replay_rows\bias_summary.csv`

| Candidate | All-finite RMS NIS | sqrt(mean finite NIS) | Finite | Accepted-only RMS | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline` | 0.479907106551 | 0.496096072716 | 91569 | 0.479907106551 | 91569 | 91569 | 0 | 0 | 1 |
| `in_shear` | 0.260640580929 | 0.361184661285 | 91569 | 0.260640580929 | 91569 | 91569 | 0 | 0 | 1 |
| `shear_rate` | 0.25557955853 | 0.357662103084 | 91569 | 0.25557955853 | 91569 | 91569 | 0 | 0 | 1 |
| `skew_shear` | 0.264216766648 | 0.363770773431 | 91569 | 0.264216766648 | 91569 | 91569 | 0 | 0 | 1 |
| `slip_envelope` | 0.366547949715 | 0.430539297476 | 91569 | 0.366547949715 | 91569 | 91569 | 0 | 0 | 1 |
| `stribeck_fade` | 0.265122084241 | 0.364430141927 | 91569 | 0.265122084241 | 91569 | 91569 | 0 | 0 | 1 |
