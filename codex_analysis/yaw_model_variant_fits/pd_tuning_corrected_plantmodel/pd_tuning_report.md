# PD Tuning For Corrected PlantModel

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

Classification: balanced. It keeps the corrected PlantModel's projected-contact force launch authority as the primary low-speed yaw source, then uses materially lower feedback gains to damp residual yaw-rate and heading error instead of brute-forcing launch through feedback.

Alternates for review:

| Class | Heading kp | Heading kd | Yaw-rate kp | Yaw-rate kd | Score | Delta vs current |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Launch-focused | 300 | 96 | 60 | 5 | 145.746 | -97.74% |
| Broad-envelope | 600 | 80 | 48 | 5 | 163.373 | -98.79% |
| Balanced | 600 | 80 | 60 | 5 | 156.626 | -98.58% |

`VelocityStatePD` is left unchanged because this pass targets the yaw dynamics affected by PlantModel feedforward and yaw-residual changes. `YawRateStatePD.kd` is reported unchanged because the current `DriveBase` yaw-rate path calls `Compute(yawRateError, 0.0f)`, so this derivative value is not actively sampled there.

## Objective And Data

Inputs used:

- Current local `MazeMap/MazeMap/PlantModel.cpp`, `PlantModel.h`, and `CoreConfig.h`.
- Existing rational speed/force partition evidence in `codex_analysis/yaw_model_variant_fits/transition_options/rational_speed_force_blend/`.
- Existing projected-force-domain yaw residual evidence in `codex_analysis/yaw_model_variant_fits/round2_force_domain_stribeck/` as supporting context.
- Existing yaw-launch step evidence in `codex_analysis/yaw_launch_step_response/`.

Not used: UKF state-vector fields as tuning targets, command/request values as traction selectors, unit tests, or project builds.

The compact replay scenarios were: low-speed 0 to 1 rad/s yaw launch, 0 to 9 rad/s yaw-rate step, 9 to -9 rad/s reversal, 3 m/s plus 5.5 rad/s combined turn, and 15 degree heading correction at speed.

## PlantModel Evidence

| Evidence | Value |
| --- | ---: |
| Rational partition +1 rad/s in-place max command | 0.649764 |
| Rational partition +1 rad/s extra opposing moment Nm | 0.066210 |
| Projected-force-only reference +1 rad/s max command | 0.655064 |
| Primary RMSE baseline -> rational corrected Nm | 0.036866 -> 0.021083 |
| Validation RMSE baseline -> rational corrected Nm | 0.050234 -> 0.029680 |
| Median measured launch delay used | 5.000 ms |
| Median launch time constant used | 27.250 ms |
| Median sustained initial yaw acceleration evidence | 115.265 rad/s^2 |

Source caveat: the current local `PlantModel.cpp` still contains Variant-C symbols and a request-conditioned feedforward helper according to static source inspection. This report scores the owner-described corrected model behavior: force/projected-contact-only rational partition and no command/request traction selector.

## Result Versus Current

| Objective | Current score | Selected score | Delta |
| --- | ---: | ---: | ---: |
| launch_focused | 6445.567 | 145.746 | -97.74% |
| broad_envelope | 13524.880 | 163.373 | -98.79% |
| balanced | 11003.806 | 156.626 | -98.58% |

Balanced scenario highlights:

| Scenario | Current late RMS | Selected late RMS | Current saturation | Selected saturation | Selected final heading error |
| --- | ---: | ---: | ---: | ---: | ---: |
| launch_0_to_1_radps | 0.003462 | 0.000191 | 0.000 | 0.000 | 0.000000 |
| yaw_rate_0_to_9_radps | 0.000155 | 0.000003 | 0.022 | 0.000 | 0.000000 |
| yaw_rate_9_to_minus9_radps | 0.000040 | 0.000001 | 0.051 | 0.026 | 0.000000 |
| combined_3mps_5p5radps | 0.000565 | 0.000015 | 0.011 | 0.000 | 0.000000 |
| heading_correction_15deg_at_3mps | 53.535570 | 0.049330 | 0.978 | 0.000 | 0.006473 |

## Caveats

- This is a tuning recommendation, not adoption. Coefficients should be reviewed before installation.
- The replay is compact and analysis-only; it does not replace a release-mode `PdTuning` run once rebuilding that tool is in scope.
- The rational force/projected-contact residual has stronger launch authority than old Variant C. The balanced recommendation avoids high heading proportional gain because the corrected feedforward should carry launch, not feedback saturation.
- Launch-focused is best when immediate breakaway and low-speed yaw response are the review priority. Broad-envelope is best when reversals, combined turns, and heading correction matter more. Balanced is the recommended default between those pressures.

## Artifacts

- `tune_pd_corrected_plantmodel.py`
- `candidate_summary.csv`
- `scenario_metrics.csv`
- `evidence_summary.csv`
- `pd_tuning_report.md`
