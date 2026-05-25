# Expanded Yaw Torque Extraction

Scratch analysis only. No production code was modified.

## Method

Targets are sensor-derived only: raw gyro yaw rate minus an independently estimated stationary raw-gyro bias where stationary rows exist, encoder-derived forward velocity, logged drive commands, logged or derived wheel speeds, and timestamps. `ukf_state_*` and estimator diagnostics are not fit targets.

Residual sign convention follows the reconciled defensible basis: `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`. This is the additive yaw torque that would be added to the current yaw-relevant PlantModel mirror for one sample. `+Yaw` is clockwise. `opposing_yaw_resistance_nm = -sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`, so positive opposing torque resists the current yaw motion.

The current-model mirror is the prior reconciled scratch mirror of the yaw-relevant `PlantModel` terms. For legacy competition CSVs, wheel omega is derived from encoder velocity and current wheel radius, saturation/watchdog fields are unavailable, phase identity and command consistency carry more of the gating, and fan duty comes from metadata/default 0.8. Those runs are useful for coverage but are less authoritative than current decoded `open_floor_main.csv` logs.

The D:\ raw open-floor capture is not included by default because it duplicates `TestResults\mmlog_decode_2026-05-04_20-35-47`; pass `--include-d-decode` only when intentionally checking that duplicate decode.

Signed surface bins require at least 80 samples. Forward velocity is binned at 0.10 m/s and yaw rate at 0.50 rad/s.

## Log Use

Included runs: 66; excluded/zero-sample candidates: 0; extracted samples: 2382049.

| Family | Samples |
| --- | ---: |
| competition_aux | 256154 |
| competition_diag | 254898 |
| open_floor | 1870997 |

| Family | Moving-yaw samples `|Vf|>0.02`, `|gyro|>0.2` | Nonzero-Vf samples `|Vf|>=0.15` |
| --- | ---: | ---: |
| competition_aux | 70477 | 123297 |
| competition_diag | 75107 | 60637 |
| open_floor | 531861 | 591710 |

Full per-run inventory, tail cuts, bias rows, and limitations are in `expanded_yaw_torque_run_summary.csv`.

## Competition Impact

| Dataset | Samples | Moving-yaw samples | Nonzero-Vf samples | Holdout current RMSE | Holdout surface RMSE | Covered current RMSE | Covered surface RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| all_included | 2382049 | 677445 | 775644 | 0.272693052 | 0.248282427 | 0.270613964 | 0.245637214 |
| open_floor_only | 1870997 | 531861 | 591710 | 0.258347167 | 0.237744117 | 0.255885951 | 0.234647700 |
| competition_only | 511052 | 145584 | 183934 | 0.308298322 | 0.258216285 | 0.307276026 | 0.255477430 |

Competition logs materially add maze-turn coverage and make the surface less dominated by open-floor characterization phases. Because the competition schema is older and lacks saturation/watchdog fields, the all-included surface should be interpreted as an exploratory calibration candidate; `open_floor_only` remains the cleaner validation subset.

## Coverage By Absolute Forward Velocity

| Abs Vf bin m/s | Count | Yaw bins | Max abs yaw bin rad/s | Families |
| ---: | ---: | ---: | ---: | --- |
| 0.00 | 703322 | 47 | 23.50 | competition_aux;competition_diag;open_floor |
| 0.10 | 903083 | 47 | 23.50 | competition_aux;competition_diag;open_floor |
| 0.20 | 233547 | 45 | 22.50 | competition_aux;competition_diag;open_floor |
| 0.30 | 275516 | 46 | 23.00 | competition_aux;competition_diag;open_floor |
| 0.40 | 167724 | 42 | 21.00 | competition_aux;competition_diag;open_floor |
| 0.50 | 44348 | 36 | 18.50 | competition_aux;competition_diag;open_floor |
| 0.60 | 48487 | 25 | 18.00 | competition_aux;competition_diag;open_floor |
| 0.70 | 4777 | 22 | 13.00 | competition_aux;competition_diag;open_floor |
| 0.80 | 938 | 10 | 9.50 | competition_aux;open_floor |
| 0.90 | 201 | 4 | 2.00 | competition_aux;open_floor |

## Useful Nonzero-Vf Counter-Yaw Surface Rows

This compact table combines signs by absolute forward velocity and absolute yaw rate for readability. Use `expanded_yaw_torque_surface_signed_bins.csv` for the signed additive residual table.

| Abs Vf bin m/s | Abs yaw bin rad/s | Count | Median opposing Nm | Trimmed mean opposing Nm |
| ---: | ---: | ---: | ---: | ---: |
| 0.20 | 0.50 | 32519 | -0.002917872 | -0.012612201 |
| 0.40 | 0.50 | 22166 | 0.000854724 | 0.002542418 |
| 0.30 | 0.50 | 21194 | 0.001144501 | 0.001617590 |
| 0.40 | 1.00 | 12791 | 0.000559215 | 0.001028257 |
| 0.50 | 0.50 | 8354 | 0.000654997 | 0.000765584 |
| 0.20 | 1.00 | 6827 | -0.015066453 | -0.019475419 |
| 0.40 | 1.50 | 6742 | 0.000642613 | -0.000227193 |
| 0.30 | 1.00 | 6572 | 0.000914100 | 0.002577603 |
| 0.40 | 5.50 | 5512 | -0.010758936 | -0.011455591 |
| 0.40 | 5.00 | 5222 | -0.009644927 | -0.010039220 |
| 0.40 | 2.00 | 4824 | -0.002131181 | -0.002787085 |
| 0.40 | 6.00 | 4762 | -0.014129277 | -0.015425469 |
| 0.60 | 0.50 | 4687 | 0.003432694 | 0.003379021 |
| 0.40 | 4.50 | 4570 | -0.008484353 | -0.008484218 |
| 0.40 | 6.50 | 4419 | -0.014479006 | -0.016649157 |
| 0.40 | 2.50 | 3746 | -0.002983594 | -0.004327462 |
| 0.20 | 1.50 | 3477 | -0.030024085 | -0.028141282 |
| 0.40 | 7.00 | 3322 | -0.018069541 | -0.020317325 |
| 0.50 | 1.00 | 3295 | 0.000969483 | 0.002488762 |
| 0.30 | 1.50 | 3106 | -0.003102423 | -0.005007909 |
| 0.40 | 3.00 | 2953 | -0.006703562 | -0.007280585 |
| 0.30 | 2.00 | 2947 | -0.009395330 | -0.016697490 |
| 0.20 | 3.50 | 2945 | -0.016896105 | -0.022971672 |
| 0.40 | 4.00 | 2893 | -0.008144666 | -0.008198830 |

## Holdout RMSE

Deterministic run-level holdout split: 1570233 train samples, 811816 holdout samples, 393 train surface bins.

| Metric | RMSE rad/s |
| --- | ---: |
| Current model, all holdout samples | 0.272693052 |
| Surface corrected, all holdout samples | 0.248282427 |
| Current model, holdout samples with trained surface bin | 0.270613964 |
| Surface corrected, holdout samples with trained surface bin | 0.245637214 |

Holdout samples with a trained signed bin: 800676. Per-run holdout rows are in `expanded_yaw_torque_holdout_rmse.csv`.

## Recommendation

The expanded logs are enough to show a real velocity/yaw-rate-dependent residual structure beyond `Vf=0`, especially through 0.2-0.5 m/s with yaw-rate bins spanning normal maze turns and loops. They are not yet strong enough to install a production PlantModel-owned tire/yaw-resistance surface directly: several high-count bins change sign, competition rows are legacy-schema approximations, and high-speed forward coverage above about 0.7 m/s is thin.

A future production change should keep ownership in `PlantModel` and derive constants from `Vehicle`/drive owners. The still-needed targeted data is: symmetric clockwise/counter-clockwise maneuver runs at 0.6, 0.8, and 1.0+ m/s; repeated yaw-rate plateaus around 2-8 rad/s while moving; and clean per-row fan duty/saturation/watchdog logging for competition-like maze runs.

## Outputs

- Signed bins: 446
- Absolute summary bins: 223
