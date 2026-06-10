# Traction RMS NIS Testbed Report

- Generated samples scored: `0`
- Split policy: whole-segment assignment only; missing splits are stable-hashed by `segment_id`.
- Logged UKF state policy: `ukf_state*` and `logged_ukf_state*` CSV columns are rejected.
- Scoring policy: high RMS NIS is penalized; too-low RMS NIS receives under-expected and covariance-inflation penalties.
- Production/hardware hooks: none.

## Candidate Ranking

| Rank | Candidate | Selection score | Samples |
| ---: | --- | ---: | ---: |
| 1 | `baseline_holdover` | inf | 0 |
| 2 | `candidate_1_algebraic_envelope` | inf | 0 |
| 3 | `candidate_2_stribeck_algebraic` | inf | 0 |
| 4 | `candidate_3_load_sensitive_anisotropic` | inf | 0 |

## Integration Status

No NIS artifacts were found in the manifest. `integration_todos.md` lists the standalone data/model work still needed.
