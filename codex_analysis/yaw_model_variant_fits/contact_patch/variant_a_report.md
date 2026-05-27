# Variant A Contact-Patch Fit

Analysis-only output. Production code, build metadata, and tests were not edited.

## Model Form

The fitted correction is a PlantModel-local force-level patch-yaw correction. In production terms, the signed contact patch yaw velocity basis is

`q = sum_i load_fraction_i * (f_i * v_rel_r_i - r_i * v_rel_f_i)`.

The analysis model predicts an added yaw moment:

`delta_Mz = beta dot [q, q*g_rel, q*g_vf, q*g_rel*g_vf, tanh(q/q0)*g_rel*g_vf, tanh(q/q0)*g_rel, q*u, q*u*g_rel, q*abs_q]`.

`g_rel = 1 / (1 + (vbar_rel / s_rel)^2)`, `g_vf = 1 / (1 + (Vf / s_vf)^2)`, and `u` is a squashed preprojection contact-utilization signal. This corresponds to distributing `delta_Mz` back into raw contact forces before friction projection with the existing load-weighted patch-yaw basis.

## Selected Hyperparameters

- `s_rel`: `0.025 m/s`
- `s_vf`: `0.45 m/s`
- `q0`: `0.0025 m^2/s`
- `ridge_lambda`: `1e-05`
- yaw denominator reference from constants: `0.000246510401 kg*m^2`

## Coefficients

- `q_patch_yaw_velocity_m2ps`: `-7.17226438`
- `q_low_contact_speed`: `-141.783291`
- `q_low_forward_speed`: `4.95126949`
- `q_low_contact_and_forward`: `-10.0674376`
- `tanh_q_low_contact_and_forward`: `0.0105915857`
- `tanh_q_low_contact`: `0.368090313`
- `q_force_utilization`: `1.67495857`
- `q_force_utilization_low_contact`: `-16.7732614`
- `q_abs_patch_velocity`: `-9.09942652`

## Performance Summary

Fit input is authoritative open-floor train rows plus `open_floor_fit_downweighted` rows at 30% weight. Primary selected runs remain held out from the authoritative training subset.

| Set | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta % | Baseline MAE Nm | Corrected MAE Nm | R2 vs zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Weighted fit input | 65646 | 0.038933 | 0.032822 | -15.70 | 0.024912 | 0.020799 | 0.2893 |
| Primary selected-run holdout | 12836 | 0.042286 | 0.029916 | -29.25 | 0.034139 | 0.023436 | 0.4995 |
| Open-floor downweighted | 31165 | 0.043194 | 0.036381 | -15.77 | 0.027850 | 0.022936 | 0.2906 |
| Validation/competition | 40098 | 0.055087 | 0.041942 | -23.86 | 0.036117 | 0.028140 | 0.4203 |

## Phase Summary

| Split | Phase | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta % | Median abs before Nm | Median abs after Nm |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| aux_downweighted_validation | entry | 2710 | 0.074083 | 0.055902 | -24.54 | 0.027800 | 0.021209 |
| aux_downweighted_validation | exit | 3030 | 0.060349 | 0.043430 | -28.03 | 0.030536 | 0.024718 |
| aux_downweighted_validation | plateau | 8708 | 0.037221 | 0.029076 | -21.88 | 0.011438 | 0.010185 |
| diag_validation_only | entry | 3215 | 0.105908 | 0.077865 | -26.48 | 0.110339 | 0.079738 |
| diag_validation_only | exit | 3181 | 0.086655 | 0.064543 | -25.52 | 0.087035 | 0.061734 |
| diag_validation_only | plateau | 4712 | 0.064485 | 0.050373 | -21.88 | 0.045651 | 0.037780 |
| open_floor_fit_downweighted | entry | 8341 | 0.056178 | 0.047710 | -15.07 | 0.030788 | 0.023750 |
| open_floor_fit_downweighted | exit | 8173 | 0.053191 | 0.044318 | -16.68 | 0.031939 | 0.023631 |
| open_floor_fit_downweighted | plateau | 14651 | 0.024365 | 0.020587 | -15.51 | 0.005602 | 0.005055 |
| open_floor_validation_only | entry | 2104 | 0.022225 | 0.022993 | 3.46 | 0.008434 | 0.007457 |
| open_floor_validation_only | exit | 1905 | 0.025902 | 0.026495 | 2.29 | 0.012154 | 0.011728 |
| open_floor_validation_only | plateau | 10533 | 0.013257 | 0.013230 | -0.20 | 0.004279 | 0.004082 |
| primary_open_floor_fit_authoritative | entry | 9719 | 0.045837 | 0.038824 | -15.30 | 0.024289 | 0.020069 |
| primary_open_floor_fit_authoritative | exit | 10820 | 0.044807 | 0.035054 | -21.77 | 0.027397 | 0.019517 |
| primary_open_floor_fit_authoritative | plateau | 26778 | 0.028771 | 0.022045 | -23.38 | 0.011056 | 0.009473 |

## High-Speed Sanity

`high_forward_yaw` rows (`|Vf| >= 0.5 m/s`, `|yaw| >= 0.5 rad/s`): 442 rows, RMSE 0.083872 -> 0.066597 Nm (-20.60%).

## Selected Logs

| Run | Present | Split | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Median signed before Nm | Median signed after Nm |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | yes | open_floor_fit_downweighted | 3456 | 0.035846 | 0.037645 | 0.009360 | 0.009409 |
| 2026-05-04_16-57-53 | yes | open_floor_validation_only | 1761 | 0.023278 | 0.023400 | -0.000686 | -0.000603 |
| 2026-04-22_12-10-34 | yes | open_floor_fit_downweighted | 2187 | 0.016217 | 0.015883 | -0.003116 | -0.003034 |
| 2026-04-22_01-06-32 | yes | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.028355 | 0.009991 | 0.005876 |
| 2026-04-21_05-32-06 | yes | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.030375 | 0.012155 | 0.004611 |
| 2026-04-21_00-16-10 | yes | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.030105 | 0.008941 | 0.001242 |
| 2026-04-20_12-10-58 | yes | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.029036 | 0.003675 | 0.001335 |
| 2026-04-20_08-38-39 | yes | open_floor_fit_downweighted | 7284 | 0.056225 | 0.048791 | 0.000167 | 0.000254 |
| diag003 | yes | diag_validation_only | 5580 | 0.085238 | 0.063942 | -0.012440 | -0.009427 |

## Failure Modes

- The model still cannot represent true static yaw breakaway at exactly zero contact-patch velocity; the `tanh(q/q0)` term is continuous and steep near zero, but it is still velocity-basis driven.
- Entry and exit phases remain contaminated by timing delay and gyro differentiation noise, so coefficients were fit with lower transient weight and phase metrics should be read separately.
- High-utilization rows use the preprojection utilization only as a smooth gain schedule. The analysis does not replay the full force projection after correction, so saturation-bound behavior is approximate.
- Lateral body velocity is unavailable in the source logs; right-relative patch velocity assumes the existing feature extractor's `Vr = 0` reconstruction.

## Files

- `fit_contact_patch_variant_a.py`
- `variant_a_coefficients.json`
- `variant_a_hyperparameter_grid.csv`
- `variant_a_metrics_by_split.csv`
- `variant_a_metrics_by_phase.csv`
- `variant_a_selected_log_metrics.csv`
- `variant_a_motion_bucket_metrics.csv`
- `variant_a_report.md`
- `commands_run.txt`
