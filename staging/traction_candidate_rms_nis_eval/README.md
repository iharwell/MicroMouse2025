# Traction Candidate RMS NIS Evaluation

This staging area defines the first-round candidate evaluation contract. The driver is `tooling/evaluate_traction_candidate_rms_nis.py`.

## Artifact Contract

The manifest lists replay-exported NIS CSV artifacts. Each NIS row must identify:

- `candidate_id`
- `segment_id`
- `stage`
- `log_parameter`
- `nis`

Optional row or manifest fields:

- `split`: `train`, `validation`, or `held_out`; if omitted, the driver assigns a stable hash split by whole `segment_id`.
- `corrupted`: any true value excludes the whole segment.
- `measurement_dimension`: used only when a log parameter is not listed in the config scoring table.

Wide CSVs are also accepted when they contain `*_nis` columns plus `candidate_id`, `segment_id`, and `stage`. `last_update_nis` is ignored by default to avoid double-counting specific measurement updates. CSVs containing `ukf_state*` columns are rejected.

## Scoring

The driver reports raw RMS NIS by candidate, split, stage, and specific log parameter. Ranking uses the validation split by default. Held-out scores are emitted for audit only.

Ranking uses guarded RMS NIS:

- Expected RMS NIS for a chi-square parameter with dimension `k` is `sqrt(k * (k + 2))`.
- Raw RMS NIS below expected RMS is floored at expected RMS for ranking.
- Buckets below the configured inflation floor are flagged.

This keeps RMS NIS as the reported metric without rewarding covariance inflation.

## Current Hook Gap

The current `tooling/run_open_floor_ukf_replay.ps1` rejects `-Tuning`, and `OpenFloorUkfReplay` does not yet export NIS CSV. Running the driver with no NIS artifacts writes `hook_gap.md` describing the smallest safe C++ hook needed.

Minimum hook:

1. `PlantModel` accepts one canonical candidate/profile input for replay, keeping the production default unchanged.
2. `OpenFloorUkfReplay` accepts a candidate config path and a NIS CSV output path.
3. `OpenFloorUkfReplay` writes rows from existing estimator NIS accessors after yaw and planar-accel updates, without reading logged UKF state columns.
