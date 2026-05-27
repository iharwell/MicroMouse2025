# PD Tuning For Final Rational Projected-Only PlantModel

Analysis-only output. Production code, build metadata, and tests were not modified. The proposed coefficients were not installed.

## Recommendation

Recommended balanced review candidate for `DriveBase` PD coefficients:

| Gain | Current | Recommended |
| --- | ---: | ---: |
| `VelocityStatePD.kp` | 5.5 | 5.5 |
| `VelocityStatePD.kd` | 0.01 | 0.01 |
| `HeadingStatePD.kp` | 9718 | 1200 |
| `HeadingStatePD.kd` | 0 | 0 |
| `YawRateStatePD.kp` | 126 | 122.394 |
| `YawRateStatePD.kd` | 5 | 5 |

This is a launch-capable review candidate, not an adoption-ready install. It saturates the yaw launch scenarios and improves the C++ replay's maneuver-tracking and high-speed heading-error metrics versus the current coefficients, but it still makes `PdTuning` report `failed=true` in the combined high-speed scenarios.

Alternates by objective:

| Objective | Classification | Gains | Status | Objective score | Launch sat max | Broad sat max | Yaw-launch OS | Heading OS | Max OS |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `launch_with_saturation` | launch-focused, saturation-permissive | `Velocity=(5.5, 0.01)`, `Heading=(1200, 0)`, `YawRate=(122.394, 5)` | review candidate; intentionally saturates yaw launch | 171.075 | 1.000 | 1.000 | 0.00% | 1193.70% | 1193.70% |
| `launch_without_saturation` | launch-focused, saturation-averse | `Velocity=(5.5, 0.01)`, `Heading=(1200, 80)`, `YawRate=(36, 5)` | diagnostic low-launch-saturation candidate; not broad-envelope clean | 165.528 | 0.005 | 1.000 | 0.00% | 1193.91% | 1193.91% |
| `nonlaunch_with_saturation` | broad-envelope, saturation-permissive | `Velocity=(5.5, 0.01)`, `Heading=(300, 40)`, `YawRate=(2, 5)` | diagnostic only; broad scenarios still fail | 363877.208 | 0.000 | 0.990 | 0.00% | 1169.91% | 1169.91% |
| `nonlaunch_without_saturation` | broad-envelope, saturation-averse | `Velocity=(5.5, 0.01)`, `Heading=(300, 40)`, `YawRate=(2, 5)` | diagnostic only; no true broad no-saturation candidate found | 364862.157 | 0.000 | 0.990 | 0.00% | 1169.91% | 1169.91% |
| `balanced` | balanced default | `Velocity=(5.5, 0.01)`, `Heading=(1200, 0)`, `YawRate=(122.394, 5)` | primary review candidate; not adoption-ready | 364.515 | 1.000 | 1.000 | 0.00% | 1193.70% | 1193.70% |

Performance versus current coefficients:

| Metric | Current | Balanced review candidate | Delta |
| --- | ---: | ---: | ---: |
| Yaw launch final error, rad/s | 6.555 | 6.555 | 0.000 |
| Yaw launch overshoot | 0.00% | 0.00% | 0.00 pp |
| Launch saturation max | 1.000 | 1.000 | 0.000 |
| High-speed heading final abs error, rad | 1.643 | 0.719 | 56.3% lower |
| High-speed heading overshoot | 1191.51% | 1193.70% | 2.18 pp |
| Max scenario overshoot | 1191.51% | 1193.70% | 2.18 pp |
| Maneuver tracking mean | 11.938 | 10.539 | 11.7% lower |
| Failed PdTuning scenarios | 2 | 2 | 0 |

## Model Provenance

This supersedes the flawed `pd_tuning_final_projected_only_plantmodel` report because that report used `cubic_smoothstep_partition` as a proxy. This pass uses the installed rational speed/force partition model facts:

- `M_opp = M_C + blend * (M_force - M_C)`.
- `speedLow = k_v^2 / (k_v^2 + v2)`, with `k_v = 0.500 m/s`.
- `forceGate = u^2 / (u^2 + k_u^2)`, with `k_u = 0.10`.
- `blend = clamp(speedLow * forceGate, 0, 1)`.
- `rel_weight = 0.75`, `speed_fade = 0.64 m/s`, `force_sliding = 0.067416756 Nm`.
- C-like branch assumed projected-force/contact-state-only: no request/preprojection terms and no limiter/projection-scale terms.

Rational model evidence used for context:

| Evidence | Value |
| --- | ---: |
| +1 rad/s in-place max command | 0.649764 |
| +1 rad/s extra opposing yaw torque | 0.066210 Nm |
| +1 rad/s blend gate | 0.981496 |
| Primary RMSE baseline -> corrected | 0.036866 -> 0.021083 Nm |
| Validation RMSE baseline -> corrected | 0.050234 -> 0.029680 Nm |

## Evaluation Setup

- Existing JSON came from the earlier `MazeMap` Release and `Tools/PdTuning` Release rebuilds. This overshoot-only update did not rebuild `MazeMap` or rerun `PdTuning`. It reprocessed the existing JSON outputs.
- The existing `Tools/PdTuning/x64/Release/PdTuning.exe` outputs execute `DriveBase::ProposeBodyTick(...)` and `PlantModel::integrate(...)` through the C++ model.
- Ran baseline plus broad, launch-authority, low-saturation, and non-launch broad bounded searches.
- Re-evaluated 117 unique top/manual candidates with the C++ tool and post-scored them into launch-focused, non-launch-focused, saturation-permissive, saturation-averse, and balanced objectives.
- Overshoot percentages are `100 * overshoot / abs(target - initial)` for each C++ PdTuning scenario.
- Did not use UKF state-vector fields as tuning targets and did not use command/request values as traction selectors.

## Result Details

| Candidate | Tool failed | Tool score | Launch sat | Broad sat | Yaw launch final err | Yaw OS | Heading final err | Heading OS | Max OS | Maneuver tracking mean |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| manual_authority_h1200_d0_y122 | True | 450887.842 | 1.000 | 1.000 | 6.555 | 0.00% | -0.719 | 1193.70% | 1193.70% | 10.539 |
| manual_authority_h600_d80_y122 | True | 451050.046 | 1.000 | 1.000 | 6.555 | 0.00% | -0.972 | 1193.60% | 1193.60% | 11.465 |
| compact_prior_broad | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | 1.392 | 1196.81% | 1196.81% | 10.975 |
| manual_authority_h600_d80_y126 | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | -1.334 | 1193.50% | 1193.50% | 11.493 |
| manual_grid_h1200_d80_y48 | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | -1.582 | 1193.52% | 1193.52% | 10.532 |
| manual_authority_h1200_d0_y100 | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | 1.625 | 1194.49% | 1194.49% | 10.554 |
| broad_rank_5 | True | 451111.164 | 1.000 | 1.000 | 6.555 | 0.00% | 1.607 | 1191.62% | 1191.62% | 11.327 |
| manual_authority_h2500_d0_y122 | True | 451285.404 | 1.000 | 1.000 | 6.555 | 0.00% | 1.613 | 1191.63% | 1191.63% | 11.356 |
| manual_authority_h3500_d20_y160 | True | 451585.277 | 1.000 | 1.000 | 6.555 | 0.00% | 1.643 | 1191.51% | 1191.51% | 11.525 |
| manual_grid_h1200_d80_y60 | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | -1.843 | 1193.18% | 1193.18% | 10.525 |
| manual_authority_h3500_d20_y126 | True | nan | 1.000 | 1.000 | 6.555 | 0.00% | 1.643 | 1191.51% | 1191.51% | 11.540 |
| nonlaunch_broad_rank_8 | True | 451334.425 | 1.000 | 1.000 | 6.555 | 0.00% | 1.643 | 1191.51% | 1191.51% | 11.541 |

## Caveats

- All selected rows still make `PdTuning` report `failed=true`. The common blockers are the combined 3 m/s, 5.5 rad/s turn and high-speed heading correction scenarios reaching non-finite/failed state. Treat the balanced row as the least-bad review candidate from this pass, not a clean release candidate.
- No true broad-envelope, no-saturation candidate was found. With heading authority enabled, the best broad-envelope saturation maximum in this evaluated set is still about 0.990, and those rows still fail the combined high-speed scenarios.
- `YawRateStatePD.kd` remains score-insensitive in the current `DriveBase` path because yaw-rate feedback calls `Compute(yawRateError, 0.0f)`.
- Launch-focused saturation-permissive intentionally requires yaw-launch saturation because the objective is to preserve static yaw-launch authority. Broad-envelope and saturation-averse objectives penalize sustained saturation strongly.
- These are recommendations for review; no coefficients were installed.

## Artifacts

- `pdtuning_baseline.json`
- `pdtuning_search_broad.json`
- `pdtuning_search_launch_authority.json`
- `pdtuning_search_low_saturation.json`
- `pdtuning_search_nonlaunch_broad.json`
- `candidate_evals/*.json`
- `candidate_summary.csv`
- `objective_rankings.csv`
- `selected_coefficients.csv`
- `scenario_metrics.csv`
- `analyze_full_pdtuning.py`
