# Traction Candidate Replay Hook Gap

The Python driver is ready to score replay-exported NIS artifacts, but the current replay surface does not yet execute candidate PlantModel profiles or export NIS CSV directly.

Smallest safe code hook:

1. `PlantModel`: add one canonical candidate/profile input at construction or through a narrowly scoped replay-only profile setter. The default production path must keep using the current built-in profile. The profile should carry traction-model kind plus bounded physical parameters only; it must not carry estimator covariance, process-noise, measurement-noise, or NIS scaling values.
2. `OpenFloorUkfReplay`: add CLI options equivalent to `--plant-candidate-config <json>` and `--nis-csv <path>`. For each replayed update, write `candidate_id`, `run_id`, `segment_id`, `split` when known, `stage`, `log_parameter`, `measurement_dimension`, `nis`, `accepted`, and corruption metadata.
3. `OpenFloorUkfReplay`: use the existing `Estimator::LastYawRateNis()`, `LastForwardAccelNis()`, and `LastRightAccelNis()` accessors after `updateYawRate(...)` and `updatePlanarAccel(...)`. Do not export or consume logged `ukf_state_*` columns for this scoring path.

Until that hook lands, produce the NIS CSV artifacts externally and list them in the manifest consumed by this driver.
