# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `6`
- Skipped corrupted segments: `0`
- Source logs: `6`
- Jobs used: `6`
- Segment-row samples processed: `6144`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 0.265706948151 | 0.322148792372 | 16384 | 16384 | 0 | 0 | 6 |
| `candidate_1_algebraic_envelope` | 0.196537351995 | 0.28136866119 | 16384 | 16384 | 0 | 0 | 6 |
| `candidate_2_stribeck` | 0.187409181004 | 0.275533771664 | 16384 | 16384 | 0 | 0 | 6 |
| `candidate_3_load_sensitive` | 0.213999078107 | 0.292254597143 | 16384 | 16384 | 0 | 0 | 6 |
