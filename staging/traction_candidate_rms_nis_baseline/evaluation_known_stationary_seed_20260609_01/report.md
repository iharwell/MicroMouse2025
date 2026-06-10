# Traction Candidate RMS NIS Evaluation

- Generated: `2026-06-09_06-35-01`
- Ranking uses validation split by default; held-out rows are reported but should not be used for candidate selection.
- Raw RMS NIS is reported directly. Ranking uses guarded RMS NIS, floored at expected chi-square RMS, so covariance inflation cannot improve a score below calibrated expectation.
- Logged `ukf_state_*` columns are rejected at CSV load time.

## Candidate Ranking

| Rank | Candidate | Selection score | Train | Validation | Held-out | Samples | Inflation flags |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
|  | `baseline_holdover` | inf | inf | inf | inf | 0 | 0 |
|  | `candidate_1_algebraic_envelope` | inf | inf | inf | inf | 0 | 0 |
|  | `candidate_2_stribeck_algebraic` | inf | inf | inf | inf | 0 | 0 |
|  | `candidate_3_load_sensitive_anisotropic` | inf | inf | inf | inf | 0 | 0 |
