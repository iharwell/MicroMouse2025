# Contact-Continuum Phase Split and Ablation

Analysis-only output. Production code was not modified.

## Reproduce

```powershell
python codex_analysis\contact_continuum_yaw_identification\ablation\analyze_phase_ablation.py
```

## Phase Classifier

Samples are split by physics-active contact/yaw segments from `yaw_rate_radps`, `vbar_rel_mps`, yaw-command difference, force-utilization evidence, and local slopes. Existing section/phase/primitive labels are carried only as metadata in the classified CSV.

## Primary Open-Floor Counts

| Phase | Count | Fraction | Runs |
| --- | ---: | ---: | ---: |
| entry | 9719 | 0.205 | 15 |
| plateau | 26778 | 0.566 | 15 |
| exit | 10820 | 0.229 | 15 |

## Primary Open-Floor Residual Summary

| Phase | Median abs residual Nm | RMSE Nm | Median opposing-yaw residual Nm | Mean vbar_rel m/s | Limiter frac | Saturation frac | Gyro spike frac |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| all | 0.016045 | 0.036866 | 0.002332 | 0.064347 | 0.226 | 0.027 | 0.0004 |
| entry | 0.024289 | 0.045837 | 0.004482 | 0.118074 | 0.428 | 0.077 | 0.0009 |
| plateau | 0.011056 | 0.028771 | 0.001551 | 0.027643 | 0.064 | 0.005 | 0.0001 |
| exit | 0.027397 | 0.044807 | 0.005346 | 0.106925 | 0.444 | 0.034 | 0.0006 |

## Plateau Drop

Primary plateau median absolute residual is 0.011056 Nm versus 0.025923 Nm for entry+exit, a 57.4% reduction.

## Candidate Fits

| Candidate | Plateau weighted R2 | Plateau weighted MAE Nm | Plateau weighted RMSE Nm |
| --- | ---: | ---: | ---: |
| patch_force_contact_features | 0.7887 | 0.008917 | 0.012367 |
| artifact_transition_schema_run_proxy | -0.1297 | 0.016441 | 0.028594 |
| yaw_loss_aggregate | -2.4536 | 0.021854 | 0.049994 |

## Plateau Validation Scores

| Split | Yaw-loss R2 | Patch-force R2 | Artifact R2 |
| --- | ---: | ---: | ---: |
| open_floor_fit_downweighted | -7.2052 | -0.3726 | 0.0255 |
| open_floor_validation_only | -0.0052 | 0.5337 | 0.0251 |
| diag_validation_only | -0.2184 | 0.9640 | 0.2975 |
| aux_downweighted_validation | -2.0139 | 0.9599 | 0.0268 |

## Decision

Decision: `patch_force_contact_feature_candidate_with_validation_gaps`.

This is not a production table or production fit. The outputs identify which explanation is most consistent with the sampled logs and where the data are still confounded.

## Files

- `phase_classified_feature_sample.csv`
- `phase_split_counts.csv`
- `phase_split_summary.csv`
- `candidate_model_scores.csv`
- `correlation_scores.csv`
- `artifact_group_dominance_scores.csv`
- `plateau_residual_drop.csv`
- `commands_run.txt`

