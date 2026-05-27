# Yaw Model Variant Fit Evaluation Contract

Analysis-only workspace. Do not edit production code from this workstream.

## Common Input

Primary feature input:

`codex_analysis/contact_continuum_yaw_identification/ablation/phase_classified_feature_sample.csv`

This file is the common comparison basis because it contains contact-continuum features, run-quality recommendations, phase labels, and residual yaw torque targets.

Secondary references:

- `codex_analysis/contact_continuum_yaw_identification/features/contact_continuum_feature_sample.csv`
- `codex_analysis/yaw_torque_expanded_validation/run_summary.csv`
- `codex_analysis/yaw_torque_expanded_validation/rmse_leave_one_run_out.csv`
- `codex_analysis/yaw_torque_expanded_validation/nonzero_vf_torque_bins.csv`

## Selected Logs

Focus reporting on these logs when present in the feature input:

- `2026-05-04_20-35-47`
- `2026-05-04_16-57-53`
- `2026-04-22_12-10-34`
- `2026-04-22_01-06-32`
- `2026-04-21_05-32-06`
- `2026-04-21_00-16-10`
- `2026-04-20_12-10-58`
- `2026-04-20_08-38-39`
- `diag003`

If a selected log is absent from the primary feature input, report that absence rather than fabricating coverage.

## Split Policy

Use the existing `dataset_split` column where available:

- Fit-authoritative rows are the primary training signal.
- Validation-only and downweighted rows are reported separately.
- `diag` and `aux` competition rows are validation, not fit authority.

Also report phase performance for `entry`, `plateau`, and `exit` when possible.

## Metrics

Report residual yaw torque in Nm:

- baseline RMSE and MAE of `residual_additive_yaw_torque_nm`
- corrected RMSE and MAE after fitted correction
- median absolute residual before/after
- signed median residual by selected run

When converting to yaw-rate one-step error, use the existing validation summaries rather than inventing new inertial constants unless the script reads them from `plant_mirror_constants.csv`.

## Production Eligibility Notes

A model is more eligible when it:

- uses per-contact relative velocity or force primitives,
- is continuous through zero forward speed,
- does not branch on maneuver labels,
- does not require a runtime residual table,
- improves held-out validation without worsening high-speed or straight-line behavior.

