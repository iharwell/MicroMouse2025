# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `580`
- Skipped corrupted segments: `0`
- Source logs: `8`
- Jobs used: `8`
- Segment-row samples processed: `243177`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 3.52915014531 | 1.21805345308 | 643219 | 350542 | 292677 | 0.455019208077 | 580 |
| `candidate_1_algebraic_envelope` | 4.52361787496 | 1.63727325379 | 643219 | 600066 | 43153 | 0.0670891251658 | 580 |
| `candidate_2_stribeck` | 4.67223079959 | 1.67124306486 | 643219 | 599738 | 43481 | 0.0675990603511 | 580 |
| `candidate_3_load_sensitive` | 4.14981586573 | 1.47386334417 | 643219 | 569569 | 73650 | 0.114502214642 | 580 |
