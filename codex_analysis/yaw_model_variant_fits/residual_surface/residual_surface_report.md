# Variant D Residual Surface Fit

Analysis-only output. Production code, build metadata, and tests were not modified.

## Contract Alignment

- Primary input: `codex_analysis\contact_continuum_yaw_identification\ablation\phase_classified_feature_sample.csv`.
- Existing yaw-torque bins: `codex_analysis\yaw_torque_expanded_validation\nonzero_vf_torque_bins.csv`.
- Target: `residual_additive_yaw_torque_nm`; correction is subtracted from that residual after fitting.
- Training signal: `primary_open_floor_fit_authoritative`; downweighted, validation-only, `diag`, and `aux` rows are reported separately.
- Predictors are continuous log/feature variables only. Phase labels are used for reporting, not fitting.

## Data Used

| Split | Rows |
| --- | ---: |
| aux_downweighted_validation | 14448 |
| diag_validation_only | 11108 |
| open_floor_fit_downweighted | 31165 |
| open_floor_validation_only | 14542 |
| primary_open_floor_fit_authoritative | 47317 |

Selected runs present: 2026-04-20_08-38-39, 2026-04-20_12-10-58, 2026-04-21_00-16-10, 2026-04-21_05-32-06, 2026-04-22_01-06-32, 2026-04-22_12-10-34, 2026-05-04_16-57-53, 2026-05-04_20-35-47, diag003.

## Model Forms

- `expanded_signed_bin_lookup`: existing `0.10 m/s x 0.50 rad/s` signed-bin median residual table from expanded validation.
- `vf_yaw_ridge_surface`: continuous ridge fit over forward speed, yaw rate, absolute terms, interactions, `vbar_rel`, and force utilization.
- `contact_feature_ridge_surface`: continuous ridge fit over contact-continuum velocity, force, load, and request primitives.
- `vf_yaw_vbar_kernel_cell_surface`: median residual cells keyed by `Vf`, yaw rate, and `vbar_rel`, blended with a Gaussian kernel.

## Primary Fit-Authoritative Performance

| Model | Coverage | Baseline RMSE Nm | Corrected RMSE Nm | RMSE improvement | Corrected MAE Nm | Corrected median abs Nm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| contact_feature_ridge_surface | 1.000 | 0.036866 | 0.027495 | 25.4% | 0.017581 | 0.011039 |
| vf_yaw_vbar_kernel_cell_surface | 0.989 | 0.036866 | 0.034759 | 5.7% | 0.022984 | 0.013892 |
| vf_yaw_ridge_surface | 1.000 | 0.036866 | 0.035792 | 2.9% | 0.024609 | 0.015166 |
| expanded_signed_bin_lookup | 0.198 | 0.036866 | 0.041341 | -12.1% | 0.028144 | 0.016750 |

## Selected-Log Aggregate

| Model | Coverage | Baseline RMSE Nm | Corrected RMSE Nm | RMSE improvement | Corrected MAE Nm | Corrected median abs Nm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| contact_feature_ridge_surface | 1.000 | 0.051680 | 0.029117 | 43.7% | 0.020878 | 0.014965 |
| vf_yaw_ridge_surface | 1.000 | 0.051680 | 0.053675 | -3.9% | 0.039406 | 0.028177 |
| expanded_signed_bin_lookup | 0.230 | 0.051680 | 0.054823 | -6.1% | 0.041535 | 0.032241 |
| vf_yaw_vbar_kernel_cell_surface | 0.984 | 0.051680 | 0.058552 | -13.3% | 0.040977 | 0.026368 |

## Leave-Selected-Run-Out

Fit-authoritative selected runs are held out of training; validation/downweighted selected runs use the full fit-authoritative training set.

| Model | Mean corrected RMSE Nm | Mean improvement | Mean coverage | Runs |
| --- | ---: | ---: | ---: | ---: |
| contact_feature_ridge_surface | 0.027902 | 31.5% | 1.000 | 9 |
| expanded_signed_bin_lookup | 0.045645 | -7.4% | 0.206 | 9 |
| vf_yaw_ridge_surface | 0.043381 | -0.2% | 1.000 | 9 |
| vf_yaw_vbar_kernel_cell_surface | 0.044039 | -0.1% | 0.985 | 9 |

## Phase Performance Of Best Variant

Best selected leave-run-out mean RMSE: `contact_feature_ridge_surface`.

| Group | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Improvement | Corrected median abs Nm |
| --- | ---: | ---: | ---: | ---: | ---: |
| split:primary_open_floor_fit_authoritative:all | 47317 | 0.036866 | 0.027495 | 25.4% | 0.011039 |
| split:primary_open_floor_fit_authoritative:phase:entry | 9719 | 0.045837 | 0.033554 | 26.8% | 0.014788 |
| split:primary_open_floor_fit_authoritative:phase:plateau | 26778 | 0.028771 | 0.020801 | 27.7% | 0.008735 |
| split:primary_open_floor_fit_authoritative:phase:exit | 10820 | 0.044807 | 0.034983 | 21.9% | 0.015379 |
| split:open_floor_fit_downweighted:all | 31165 | 0.043194 | 0.033002 | 23.6% | 0.009447 |
| split:open_floor_fit_downweighted:phase:entry | 8341 | 0.056178 | 0.039889 | 29.0% | 0.018352 |
| split:open_floor_fit_downweighted:phase:plateau | 14651 | 0.024365 | 0.018515 | 24.0% | 0.005049 |
| split:open_floor_fit_downweighted:phase:exit | 8173 | 0.053191 | 0.043758 | 17.7% | 0.018985 |
| split:open_floor_validation_only:all | 14542 | 0.016931 | 0.008414 | 50.3% | 0.004101 |
| split:open_floor_validation_only:phase:entry | 2104 | 0.022225 | 0.011251 | 49.4% | 0.005722 |
| split:open_floor_validation_only:phase:plateau | 10533 | 0.013257 | 0.006661 | 49.8% | 0.003644 |
| split:open_floor_validation_only:phase:exit | 1905 | 0.025902 | 0.012463 | 51.9% | 0.006570 |
| split:diag_validation_only:all | 11108 | 0.084621 | 0.032490 | 61.6% | 0.022894 |
| split:diag_validation_only:phase:entry | 3215 | 0.105908 | 0.040087 | 62.1% | 0.032383 |
| split:diag_validation_only:phase:plateau | 4712 | 0.064485 | 0.023497 | 63.6% | 0.015295 |
| split:diag_validation_only:phase:exit | 3181 | 0.086655 | 0.035274 | 59.3% | 0.028655 |
| split:aux_downweighted_validation:all | 14448 | 0.051266 | 0.033375 | 34.9% | 0.008469 |
| split:aux_downweighted_validation:phase:entry | 2710 | 0.074083 | 0.046364 | 37.4% | 0.014441 |
| split:aux_downweighted_validation:phase:plateau | 8708 | 0.037221 | 0.024299 | 34.7% | 0.005998 |
| split:aux_downweighted_validation:phase:exit | 3030 | 0.060349 | 0.041134 | 31.8% | 0.014119 |

## Existing One-Step Yaw-Rate Reference

The common contract asks that one-step yaw-rate error use the existing validation summaries. This table is copied from the expanded signed-bin leave-run-out reference for selected runs.

| Run | Samples | Current RMSE rad/s | Signed-bin RMSE rad/s | Improvement |
| --- | ---: | ---: | ---: | ---: |
| 2026-04-20_08-38-39 | 41332 | 0.330182 | 0.286593 | 13.2% |
| 2026-04-20_12-10-58 | 24297 | 0.306755 | 0.235130 | 23.3% |
| 2026-04-21_00-16-10 | 44880 | 0.295942 | 0.175360 | 40.7% |
| 2026-04-21_05-32-06 | 20317 | 0.275808 | 0.206967 | 25.0% |
| 2026-04-22_01-06-32 | 5043 | 0.187818 | 0.204479 | -8.9% |
| 2026-04-22_12-10-34 | 3350 | 0.207746 | 0.223172 | -7.4% |
| 2026-05-04_16-57-53 | 95 | 0.099930 | 0.128505 | -28.6% |
| 2026-05-04_20-35-47 | 1815 | 0.282798 | 0.462690 | -63.6% |
| diag003 | 23242 | 0.211209 | 0.200226 | 5.2% |

## Kernel Tuning

| min cells | sigma Vf | sigma yaw | sigma vbar | LORO RMSE Nm | LORO MAE Nm | Coverage |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.120 | 0.600 | 0.035 | 0.035447 | 0.023336 | 0.988 |
| 8 | 0.160 | 0.800 | 0.050 | 0.035455 | 0.023467 | 0.988 |
| 12 | 0.200 | 1.100 | 0.070 | 0.035460 | 0.023779 | 0.990 |

## Coverage Holes

| Region | Rows | Runs | Existing-bin coverage | Top run fraction | Baseline RMSE Nm |
| --- | ---: | ---: | ---: | ---: | ---: |
| selected_all | 36861 | 9 | 0.230 | 0.241 | 0.051680 |
| high_forward_abs_ge_0p6 | 403 | 24 | 0.459 | 0.211 | 0.073742 |
| low_speed_abs_vf_lt_0p05_abs_yaw_ge_0p25 | 18275 | 49 | 0.000 | 0.143 | 0.075038 |
| straightish_abs_yaw_lt_0p25 | 78871 | 49 | 0.000 | 0.093 | 0.028985 |
| selected_run:2026-05-04_20-35-47 | 3456 | 1 | 0.021 | 1.000 | 0.035846 |
| selected_run:2026-05-04_16-57-53 | 1761 | 1 | 0.003 | 1.000 | 0.023278 |
| selected_run:2026-04-22_12-10-34 | 2187 | 1 | 0.062 | 1.000 | 0.016217 |
| selected_run:2026-04-22_01-06-32 | 1031 | 1 | 0.195 | 1.000 | 0.044975 |
| selected_run:2026-04-21_05-32-06 | 8880 | 1 | 0.104 | 1.000 | 0.042584 |
| selected_run:2026-04-21_00-16-10 | 3757 | 1 | 0.485 | 1.000 | 0.039824 |
| selected_run:2026-04-20_12-10-58 | 2925 | 1 | 0.364 | 1.000 | 0.040355 |
| selected_run:2026-04-20_08-38-39 | 7284 | 1 | 0.449 | 1.000 | 0.056225 |
| selected_run:diag003 | 5580 | 1 | 0.175 | 1.000 | 0.085238 |

## Productionization Judgment

Do not productionize Variant D as the PlantModel correction.

The best residual-surface baseline, `contact_feature_ridge_surface`, reduces sampled residual error from 0.036866 to 0.027495 Nm on the primary fit-authoritative sample and from 0.051680 to 0.029117 Nm on the selected-log aggregate. That is useful as a diagnostic baseline, but it is still a learned residual map layered on top of the physics model.

The strongest production-relevant signal is that contact-feature predictors outperform the raw `Vf/yaw` table shape while preserving continuous variables. A production version should move that behavior into `PlantModel` contact-force equations, not ship a runtime residual table keyed by fitted errors.

Main blockers: sparse high-forward/high-yaw coverage, selected low-speed launch logs have little nonzero-forward coverage in the common feature input, no independent lateral velocity measurement, no measured motor current, and residual targets are gyro-differentiated single-sample torques.

## Files

- `fit_residual_surface.py`
- `surface_metrics_by_group.csv`
- `selected_leave_run_out_metrics.csv`
- `selected_run_signed_medians.csv`
- `ridge_coefficients.csv`
- `kernel_surface_cells.csv`
- `kernel_tuning.csv`
- `coverage_holes.csv`
- `existing_yaw_rate_reference.csv`
- `residual_surface_report.md`
- `commands_run.txt`
