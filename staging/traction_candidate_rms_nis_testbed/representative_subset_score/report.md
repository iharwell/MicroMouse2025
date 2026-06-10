# Traction RMS NIS Testbed Report

- Generated samples scored: `412920`
- Split policy: whole-segment assignment only; missing splits are stable-hashed by `segment_id`.
- Logged UKF state policy: `ukf_state*` and `logged_ukf_state*` CSV columns are rejected.
- Scoring policy: high RMS NIS is penalized; too-low RMS NIS receives under-expected and covariance-inflation penalties.
- Production/hardware hooks: none.

## Candidate Ranking

| Rank | Candidate | Selection score | Samples |
| ---: | --- | ---: | ---: |
| 1 | `candidate_3_load_sensitive` | 12.9724791508 | 1746 |
| 2 | `candidate_2_stribeck` | 24.1280981936 | 1746 |
| 3 | `baseline/current_holdover` | 34.8140998516 | 1746 |
| 4 | `candidate_1_algebraic_envelope` | 34.8432665384 | 1746 |
