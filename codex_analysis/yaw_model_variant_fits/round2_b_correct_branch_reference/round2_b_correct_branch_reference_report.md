# Round2 Variant B Correct-Branch Reference

Analysis-only output. Production code, build metadata, and tests were not modified.

## Exact Branch

Reference branch: `Variant B / B_stribeck` using the selected `request_only` activation and fixed-point request coupling.

Excluded branch: the `request_or_yaw` mixed activation family from the original grid search. No persisted selected B artifact used that mixed activation; the saved selected coefficients set `activation_mode_request_only = 1`.

For positive yaw, the correction adds yaw-opposing torque:

`M_extra = A_req(M_base + M_extra) * R(v_transition) * (K_slide + K_static * S(v_transition))`

where:

- `A_req(x) = 1 - exp(-(smooth_positive(x) / req_activation_nm)^2)`
- `v_transition = sqrt((rel_weight * drive_wheel_longitudinal_offset_m * abs(yaw_rate))^2 + abs(Vf)^2)`
- `S(v) = exp(-(v / stribeck_speed_mps)^2)`
- `R(v) = 1 / (1 + (v / speed_fade_mps)^2)`
- `M_extra` is solved by fixed-point iteration with `requested = M_base + M_extra`.

This is a request-conditioned diagnostic reference, not a production-eligible traction law.

## Coefficients

| parameter | value | unit |
| --- | ---: | --- |
| `yaw_activation_mps` | 0.002 | m/s |
| `req_activation_nm` | 0.035 | Nm |
| `stribeck_speed_mps` | 0.1 | m/s |
| `speed_fade_mps` | 0.64 | m/s |
| `rel_weight` | 0.75 | dimensionless |
| `activation_mode_request_only` | 1.0 | boolean |
| `include_yaw_viscous_basis` | 0.0 | boolean |
| `static_extra_nm` | 0.00239125532944932 | Nm |
| `sliding_nm` | 0.06303158239278875 | Nm |
| `yaw_viscous_nm_per_mps` | 0.0 | Nm per (m/s) |
| `weighted_train_opposes_rmse_nm` | 0.02430644256047251 | Nm |

## +1 rad/s In-Place Command

At `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`: extra opposing yaw torque `0.065013359207` Nm, total opposing yaw torque `0.079807609207` Nm, left/right command `0.642749605677/-0.642749605677`, L-R delta `1.285499211353`.

This passes the reference gate (`|cmd| >= 0.6`) and is close to the measured/calculated `+0.646/-0.646` target.

## Split RMSE

| split | count | baseline RMSE Nm | corrected RMSE Nm | improvement % |
| --- | ---: | ---: | ---: | ---: |
| `aux_downweighted_validation` | 14448 | 0.051266 | 0.048945 | 4.528 |
| `diag_validation_only` | 11108 | 0.084621 | 0.084412 | 0.247 |
| `open_floor_fit_downweighted` | 31165 | 0.043194 | 0.041880 | 3.042 |
| `open_floor_validation_only` | 14542 | 0.016931 | 0.011372 | 32.833 |
| `primary_open_floor_fit_authoritative` | 47317 | 0.036866 | 0.028528 | 22.618 |

## Selected-Log RMSE

| run_id | split | count | baseline RMSE Nm | corrected RMSE Nm | improvement % |
| --- | --- | ---: | ---: | ---: | ---: |
| `2026-05-04_20-35-47` | `open_floor_fit_downweighted` | 3456 | 0.035846 | 0.021129 | 41.057 |
| `2026-05-04_16-57-53` | `open_floor_validation_only` | 1761 | 0.023278 | 0.015289 | 34.319 |
| `2026-04-22_12-10-34` | `open_floor_fit_downweighted` | 2187 | 0.016217 | 0.013499 | 16.763 |
| `2026-04-22_01-06-32` | `primary_open_floor_fit_authoritative` | 1031 | 0.044975 | 0.034045 | 24.304 |
| `2026-04-21_05-32-06` | `primary_open_floor_fit_authoritative` | 8880 | 0.042584 | 0.027835 | 34.635 |
| `2026-04-21_00-16-10` | `primary_open_floor_fit_authoritative` | 3757 | 0.039824 | 0.035146 | 11.747 |
| `2026-04-20_12-10-58` | `primary_open_floor_fit_authoritative` | 2925 | 0.040355 | 0.036504 | 9.543 |
| `2026-04-20_08-38-39` | `open_floor_fit_downweighted` | 7284 | 0.056225 | 0.056260 | -0.061 |
| `diag003` | `diag_validation_only` | 5580 | 0.085238 | 0.085034 | 0.240 |

## L-R Delta Grid

The full 6x10 machine-readable grid is `lr_delta_grid_6x10.csv`; the Markdown pivot is `lr_delta_pivot.md`.

## Mixed-Output Comparison

The saved round2 `B_stribeck` in-place row matches this reference branch. The `Hybrid_BC_adhesion_partition` row is a mixed B/C diagnostic and is intentionally not the requested B branch; it estimates left/right command `+0.625298/-0.625298` at +1 rad/s in place.

## Production Eligibility

Rejected for production under the command-conditioning rule. This branch directly conditions the resistance activation on `requested = M_base + M_extra`, so the same physical contact state can receive different traction/resistance depending on request/command magnitude. Keep it only as a reference/diagnostic baseline for the correct in-place command scale.

## Output Files

- `coefficients.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid_6x10.csv`
- `lr_delta_pivot.md`
- `split_rmse.csv`
- `selected_log_rmse.csv`
- `comparison_to_prior_mixed_outputs.csv`
- `contact_feature_sample_summary.csv`
- `reference_metadata.json`
