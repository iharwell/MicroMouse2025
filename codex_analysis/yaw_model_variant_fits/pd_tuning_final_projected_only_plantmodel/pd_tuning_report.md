# PD Tuning For Final Projected-Only PlantModel

Analysis-only output. Production code, build metadata, and tests were not modified. The proposed coefficients were not installed.

## Recommendation

Recommended balanced `DriveBase` PD coefficients:

| Gain | Current | Recommended |
| --- | ---: | ---: |
| `VelocityStatePD.kp` | 5.5 | 5.5 |
| `VelocityStatePD.kd` | 0.01 | 0.01 |
| `HeadingStatePD.kp` | 9718 | 600 |
| `HeadingStatePD.kd` | 0 | 80 |
| `YawRateStatePD.kp` | 126 | 60 |
| `YawRateStatePD.kd` | 5 | 5 |

Classification: balanced. This keeps launch response active enough for the final projected-only yaw residual, but avoids the current high heading proportional gain saturating the yaw acceleration objective.

Alternates for review:

| Class | Heading kp | Heading kd | Yaw-rate kp | Yaw-rate kd | Score | Delta vs current |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Launch-focused | 300 | 112 | 60 | 5 | 163.413 | -97.59% |
| Broad-envelope | 600 | 80 | 48 | 5 | 178.474 | -98.78% |
| Balanced | 600 | 80 | 60 | 5 | 173.228 | -98.56% |

`VelocityStatePD` is left unchanged because this pass targets yaw dynamics. `YawRateStatePD.kd` is also unchanged because the current `DriveBase` yaw-rate path passes zero error rate, so this derivative is reported but not sampled there.

## Objective And Data

Inputs used:

- Current local `MazeMap/MazeMap/PlantModel.cpp`, `PlantModel.h`, and `CoreConfig.h` for source/gain inspection.
- Existing projected-force/contact-state-only selected model evidence in `codex_analysis/yaw_model_variant_fits/transition_options/cubic_smoothstep_partition/`.
- Existing yaw-launch step evidence in `codex_analysis/yaw_launch_step_response/`.

Not used: UKF state-vector fields as tuning targets, command/request values as traction selectors, unit tests, or project builds.

The compact replay scenarios were low-speed 0 to 1 rad/s yaw launch, 0 to 9 rad/s yaw-rate step, 9 to -9 rad/s reversal, 3 m/s plus 5.5 rad/s combined turn, and 15 degree heading correction at speed.

## Projected-Only Model Evidence

| Evidence | Value |
| --- | ---: |
| Model artifact | `transition_options/cubic_smoothstep_partition` |
| In-place +1 rad/s max command estimate | 0.600000 |
| In-place extra opposing yaw torque | 0.057719 Nm |
| Primary RMSE baseline -> corrected | 0.036866 -> 0.018341 Nm |
| Validation RMSE baseline -> corrected | 0.050234 -> 0.028489 Nm |
| Low-speed yaw RMSE baseline -> corrected | 0.073503 -> 0.034615 Nm |
| Limiter-active RMSE baseline -> corrected | 0.074601 -> 0.040101 Nm |
| Median selected-log corrected RMSE | 0.019099 Nm |
| Median measured launch delay used | 5.000 ms |
| Median launch time constant used | 27.250 ms |
| Median sustained initial yaw acceleration evidence | 115.265 rad/s^2 |

## Source Inspection Caveat

- `PdTuning.exe` newer than `PlantModel.cpp`: `False`. I did not run it as an authoritative evaluator.
- `PlantModel.cpp` contains Variant-C symbols: `True`.
- `PlantModel.cpp` contains a request/preprojection inverse helper name: `True`.

The owner-stated final behavior is projected-force/contact-state-only. The scoring therefore uses the projected-only selected artifact above, and treats the source flags as review caveats rather than installed production behavior.

## Result Versus Current

| Objective | Current score | Selected score | Delta |
| --- | ---: | ---: | ---: |
| launch_focused | 6790.938 | 163.413 | -97.59% |
| broad_envelope | 14646.078 | 178.474 | -98.78% |
| balanced | 12001.782 | 173.228 | -98.56% |

Balanced scenario highlights:

| Scenario | Current late RMS | Selected late RMS | Current saturation | Selected saturation | Selected final heading error |
| --- | ---: | ---: | ---: | ---: | ---: |
| launch_0_to_1_radps | 0.003463 | 0.000191 | 0.009 | 0.000 | 0.000000 |
| yaw_rate_0_to_9_radps | 0.000155 | 0.000003 | 0.022 | 0.000 | 0.000000 |
| yaw_rate_9_to_minus9_radps | 0.000040 | 0.000001 | 0.051 | 0.026 | 0.000000 |
| combined_3mps_5p5radps | 0.000565 | 0.000015 | 0.011 | 0.000 | 0.000000 |
| heading_correction_15deg_at_3mps | 53.535570 | 0.049330 | 0.978 | 0.000 | 0.006473 |

## Caveats

- This is a tuning recommendation, not adoption. The coefficients were not installed.
- The replay is compact and analysis-only; it does not replace a release-mode `PdTuning` run once rebuilding/running that tool is in scope.
- The final projected-only yaw residual has just-threshold in-place launch authority in the existing synthetic estimate (`|cmd| = 0.6`), so launch-focused keeps more heading damping than the previous broad replay.
- Launch-focused is best when immediate breakaway and low-speed yaw response dominate review. Broad-envelope is best when reversals, combined turns, and heading capture matter more. Balanced is the recommended default.

## Artifacts

- `tune_pd_final_projected_only_plantmodel.py`
- `candidate_summary.csv`
- `selected_coefficients.csv`
- `scenario_metrics.csv`
- `evidence_summary.csv`
- `pd_tuning_report.md`
