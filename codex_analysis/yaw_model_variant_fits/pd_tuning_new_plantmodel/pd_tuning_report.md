# PD Tuning For Updated PlantModel Yaw Dynamics

Analysis-only output. Production code, build metadata, and tests were not modified. The proposed coefficients below were not installed.

## Recommendation

Recommended balanced `DriveBase` PD coefficients:

| Gain | Current | Recommended |
| --- | ---: | ---: |
| `VelocityStatePD.kp` | 5.5 | 5.5 |
| `VelocityStatePD.kd` | 0.01 | 0.01 |
| `HeadingStatePD.kp` | 9718.0 | 600.0 |
| `HeadingStatePD.kd` | 0.0 | 64.0 |
| `YawRateStatePD.kp` | 126.0 | 60.0 |
| `YawRateStatePD.kd` | 5.0 | 5.0 |

Classification: balanced, with a launch-safety bias. The recommendation backs yaw-rate proportional gain down materially because the updated PlantModel feedforward inverse now carries the low-speed rational speed/force yaw-residual branch. It also replaces the previous very high heading proportional correction with a smaller heading proportional term plus active heading derivative damping.

`VelocityStatePD` is left unchanged because this pass targeted the yaw dynamics affected by the PlantModel yaw residual and inverse. `YawRateStatePD.kd` is also left unchanged because the current `DriveBase` yaw-rate path passes a zero error-rate argument, so that derivative value is reported but effectively not sampled in this path.

## Data And Objective

Inputs used:

- Current local `MazeMap/MazeMap/PlantModel.cpp` and `PlantModel.h`, including the rational speed/force partition yaw residual and feedforward inverse.
- Current `Config::kDriveBasePDCluster` values from `MazeMap/MazeMap/CoreConfig.h`.
- Recent selected rational-blend yaw residual artifacts under `codex_analysis/yaw_model_variant_fits/transition_options/rational_speed_force_blend/`.
- Recent yaw-launch step evidence under `codex_analysis/yaw_launch_step_response/`.

Not used:

- UKF state-vector fields as tuning targets.
- Command/request values as traction selectors.
- Unit tests or project builds.

The active `Tools/PdTuning/x64/Release/PdTuning.exe` is older than the current PlantModel source, so I did not use it as an authoritative evaluator. Instead, `tune_pd_new_plantmodel.py` uses the existing PdTuning scenario intent and scores a compact yaw-axis replay with measured launch delay/lag evidence:

- low-speed 0 to 1 rad/s yaw launch,
- 0 to 9 rad/s yaw-rate step,
- 9 to -9 rad/s yaw-rate reversal,
- 3 m/s plus 5.5 rad/s combined turn,
- 15 degree heading correction at speed.

The score penalizes early yaw-rate RMS, late residual/ringing, target crossing count, saturation, heading error, and excessive acceleration demand. It is not a replacement for the full C++ `PdTuning` tool once that tool is rebuilt in scope.

## Evidence Summary

The selected PlantModel yaw residual transition improved residual yaw-moment RMSE:

| Split | Baseline RMSE Nm | Corrected RMSE Nm | Reduction |
| --- | ---: | ---: | ---: |
| Primary open-floor fit | 0.036866 | 0.021083 | 42.8% |
| Validation non-authoritative | 0.050234 | 0.029680 | 40.9% |

Synthetic in-place 1 rad/s command estimate for the selected residual branch is `0.649764`, versus the old Variant C launch estimate reported in the selected-blend report as `0.368641`. That places the inverse near the measured practical launch boundary instead of relying on high feedback gain to force launch.

Yaw-launch evidence used by the compact replay:

| Metric | Value |
| --- | ---: |
| Median measured launch delay | 5 ms |
| Median launch time constant used | 27.25 ms |
| Median sustained launch initial yaw acceleration evidence | 115.265 rad/s^2 |

## Result Versus Current

Compact replay aggregate score:

| Candidate | Score | Delta vs current |
| --- | ---: | ---: |
| Current | 10575.381 | baseline |
| Grid-best low-ripple candidate | 137.510 | -98.7% |
| Recommended balanced candidate | 150.802 | -98.6% |

I am recommending the balanced candidate rather than the raw grid-best. The grid-best (`HeadingStatePD.kp=450`, `HeadingStatePD.kd=80`, `YawRateStatePD.kp=48`) is more conservative but sits at the low-gain edge of the grid. The selected recommendation keeps extra broad-envelope yaw authority while still removing modeled saturation/ringing.

Recommended balanced scenario highlights:

| Scenario | Early RMS norm | Late RMS norm | Saturation fraction | Sign changes | Final heading error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Launch 0 to 1 rad/s | 0.2334 | 0.00019 | 0.000 | 3 | 0 |
| Yaw 0 to 9 rad/s | 0.2334 | ~0.00000 | 0.000 | 3 | 0 |
| Yaw 9 to -9 rad/s | 0.2501 | ~0.00000 | 0.026 | 2 | 0 |
| Combined 3 m/s, 5.5 rad/s | 0.2334 | 0.00001 | 0.000 | 3 | 0 |
| 15 deg heading correction | 0.6760 | 0.03574 | 0.000 | 0 | 0.0038 rad |

The current coefficients scored acceptably on the direct yaw-rate steps in this simplified model, but the heading correction case was dominated by saturation and large overshoot because `HeadingStatePD.kp=9718` produces a very large acceleration-domain correction with no heading derivative damping.

## Caveats

- This is an analysis-only recommendation, not a release validation. It should be reviewed and then checked with the real release-mode `PdTuning` tool once rebuilding that tool is in scope.
- The compact replay assumes the updated PlantModel inverse is close enough that feedback should damp residual error rather than brute-force launch authority.
- The recommendation is intentionally not launch-maximal. If the owner wants the most conservative no-ripple candidate, review the grid-best row in `candidate_summary.csv`; if the owner wants more aggressive heading capture, inspect the nearby `600/80/60` and `900/64/60` rows.
- Because `YawRateStatePD.kd` is currently not sampled by `DriveBase`, changing it would not materially affect this control path.

## Artifacts

- `tune_pd_new_plantmodel.py`
- `candidate_summary.csv`
- `scenario_metrics.csv`
- `evidence_summary.csv`
- `pd_tuning_report.md`
