# Full Traction ANIS Replay Report

- Replay mode: `ekf`
- Processed non-corrupted segments: `683`
- Skipped corrupted segments: `0`
- Source logs: `28`
- Jobs used: `4`
- Segment-row samples processed: `5464`
- Row artifacts enabled: `false`
- Uses logged UKF state: `false`

| Candidate | Accepted RMS NIS | sqrt(mean accepted NIS) | NIS count | Accepted | Rejected | Rejected rate | Segments |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline/current_holdover` | 5.00394431115 | 1.53563236081 | 15912 | 13638 | 2274 | 0.142911010558 | 683 |
| `candidate_1_algebraic_envelope` | 3.0420120645 | 1.11784257265 | 15912 | 14613 | 1299 | 0.0816365007541 | 683 |
| `candidate_2_stribeck` | 3.16762143534 | 1.11839236359 | 15912 | 14379 | 1533 | 0.0963423831071 | 683 |
| `candidate_3_load_sensitive` | 3.09949255033 | 1.10680998869 | 15912 | 14404 | 1508 | 0.0947712418301 | 683 |
