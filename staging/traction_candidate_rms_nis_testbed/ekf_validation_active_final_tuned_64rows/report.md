# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `578`
- Skipped corrupted segments: `0`
- Source logs: `6`
- Jobs used: `6`
- Segment-row samples processed: `36992`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 5.00664773328 | 1.53783958974 | 95744 | 69019 | 26725 | 0.279129762701 | 578 |
| `candidate_1_algebraic_envelope` | 3.82104673275 | 1.29392342153 | 95744 | 91692 | 4052 | 0.0423211898396 | 578 |
| `candidate_2_stribeck` | 3.70877602649 | 1.2790776153 | 95744 | 92261 | 3483 | 0.0363782586898 | 578 |
| `candidate_3_load_sensitive` | 3.62371799335 | 1.25022608776 | 95744 | 91742 | 4002 | 0.0417989639037 | 578 |
