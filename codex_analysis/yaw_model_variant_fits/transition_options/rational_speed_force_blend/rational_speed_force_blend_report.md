# Rational Speed/Force Blend Transition

Analysis-only output. Production code, build metadata, and tests were not modified.

## Recommendation

Recommended transition is `speed_force_partition` with `k_v=0.500 m/s`, `k_u=0.10`, and `alpha=1.00`:

`v2 = Vf_abs^2 + (0.75*vbar_rel)^2`

`speed_low = k_v^2 / (k_v^2 + v2)`

`force_gate = u^2 / (u^2 + k_u^2)` where `u = smooth_positive(M_projected_yaw)/M_yield`

`blend = clamp(alpha * speed_low * force_gate, 0, 1)`

`M_opposes = M_C + blend * (M_force_stribeck - M_C)`

This is a partitioned blend: at moving speed it returns to Variant C, while low-speed, high-force launch rows move toward the force-domain Stribeck prediction without adding both models together.

## Candidate Summary

| Candidate | k_v | k_u | alpha | Primary RMSE | Validation RMSE | Validation RB RMSE | In-place cmd | In-place extra | Mean blend |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| force_domain_stribeck_reference |  |  |  | 0.027908 | 0.048721 | 0.046206 |  |  |  |
| variant_c_reference |  |  |  | 0.024120 | 0.030342 | 0.028909 |  |  |  |
| speed_force_partition | 0.500 | 0.10 | 1.00 | 0.021083 | 0.029680 | 0.028078 | 0.649764 | 0.066210 | 0.171118 |

Best candidate by family:

| Mode | k_v | k_u | alpha | Validation RB RMSE | In-place max command |
| --- | ---: | ---: | ---: | ---: | ---: |
| speed_only_partition | 0.025 | 0.00 | 1.00 | 0.029810 | 0.608164 |
| speed_force_partition | 0.500 | 0.10 | 1.00 | 0.028078 | 0.649764 |
| speed_force_partition_squared | 0.500 | 0.10 | 1.00 | 0.028108 | 0.644698 |
| positive_launch_overlay | 0.700 | 0.10 | 1.25 | 0.028072 | 0.655064 |

## Split RMSE

| Split | Count | Baseline RMSE | Corrected RMSE | Corrected MAE | Median abs after | Mean blend |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.021083 | 0.014277 | 0.009682 | 0.244489 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.032823 | 0.020573 | 0.009665 | 0.178225 |
| open_floor_validation_only | 14542 | 0.016931 | 0.011207 | 0.007235 | 0.004512 | 0.126455 |
| diag_validation_only | 11108 | 0.084621 | 0.038489 | 0.030669 | 0.027893 | 0.016166 |
| aux_downweighted_validation | 14448 | 0.051266 | 0.027490 | 0.016843 | 0.009120 | 0.079583 |
| validation_non_authoritative | 71263 | 0.050234 | 0.029680 | 0.018669 | 0.008943 | 0.122401 |

## Selected Logs

| Run | Split | Count | Baseline RMSE | Corrected RMSE | Mean blend |
| --- | --- | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.020128 | 0.470918 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.015839 | 0.266649 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.013203 | 0.131028 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.016861 | 0.474293 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.017657 | 0.435810 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.026878 | 0.156110 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.027977 | 0.133348 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.042487 | 0.198823 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.038712 | 0.014422 |

## In-Place Command

Synthetic command estimate for `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`:

| Candidate | Blend gate | Extra opposing Nm | Total opposing Nm | Left cmd | Right cmd | Max abs cmd |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| force_domain_stribeck_reference |  | 0.067115 | 0.081909 | 0.655064 | -0.655064 | 0.655064 |
| variant_c_reference |  | 0.018241 | 0.033035 | 0.368641 | -0.368641 | 0.368641 |
| speed_force_partition | 0.981496 | 0.066210 | 0.081005 | 0.649764 | -0.649764 | 0.649764 |

The selected blend gives the launch branch an in-place gate of 0.981496, producing max command 0.649764. It clears the practical `|cmd| >= 0.6` launch check while avoiding the pure force-domain model's broader validation penalty.

## Computational Cost

Transition-only cost, assuming `Vf_abs`, `vbar_rel`, projected yaw moment, and yield moment are already available:

- `speed_force_partition`: about 5 multiplies, 2 adds, 2 divides, 1 absolute/max or smooth-positive source clamp, and 1 clamp. No sqrt, trig, exp, or table.
- If `u = smooth_positive(M)` is implemented branchlessly, add one multiply, two adds, one sqrt, and one multiply by 0.5. A branch/clamp `max(M,0)` is cheaper and acceptable if the sign convention is explicit.
- The selected transition has one final clamp. The partition equation itself has no data-dependent branch.
- `positive_launch_overlay` adds one `max(F-C,0)` branch/clamp; it was kept as a comparison because it prevents low-speed double counting, but it is less clean as the canonical partition.

## Strengths And Failures

- Strength: no trig, no exp, no table, and the selected transition can be evaluated with squared speeds rather than `sqrt(v2)`.
- Strength: force utilization prevents a low-speed zero-force row from suppressing Variant C just because the robot is slow.
- Strength: the in-place command lands near the force-domain branch rather than Variant C's underpowered in-place estimate.
- Caveat: validation RMSE improves slightly versus pure Variant C, but the margin is small; treat it as transition-shape evidence, not as proof that the launch branch is globally better.
- Failure: the transition relies on projected/actual contact yaw-moment utilization, so it belongs after contact projection or inside the same plant solve. Using pre-projection command/request moment would reintroduce the rejected command-conditioned path.
- Failure: the force-domain source model still uses its existing Stribeck exponent internally; this work only removes expensive functions from the transition gate.

## Provenance

UKF fields were not used. Feature provenance follows `codex_analysis\yaw_model_variant_fits\round2_ukf_dependency_audit\ukf_dependency_audit_report.md`, which reports no fitted input, target, or residual path directly uses logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate columns.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\transition_options\rational_speed_force_blend\fit_rational_speed_force_blend.py
```

## Output Files

- `fit_rational_speed_force_blend.py`
- `rational_speed_force_blend_report.md`
- `candidate_scores.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `in_place_1radps_command.csv`
- `commands_run.txt`
