# Full Traction RMS NIS Replay Report

- Replay mode: `residual`
- Processed non-corrupted segments: `1690`
- Skipped corrupted segments: `29`
- Source logs: `51`
- Jobs used: `10`
- Segment-row samples processed: `2920443`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`

| Candidate | RMS NIS | sqrt(mean NIS) | NIS count | Accepted | Rejected | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 165.018925503 | 3.84214070906 | 8525363 | 8485839 | 39524 | 1690 |
| `candidate_1_algebraic_envelope` | 207.683067228 | 4.66359601983 | 8525363 | 8489870 | 35493 | 1690 |
| `candidate_2_stribeck` | 185.585321851 | 4.0182984751 | 8525363 | 8484269 | 41094 | 1690 |
| `candidate_3_load_sensitive` | 215.704337435 | 6.03070447748 | 8525363 | 8469885 | 55478 | 1690 |
