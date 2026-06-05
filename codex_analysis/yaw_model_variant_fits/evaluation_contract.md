# Yaw Model Variant Fit Evaluation Contract

Analysis-only workspace. Do not edit production code from this workstream.

## Superseding Acceptance Rule

For model tuning, Python model investigations, and data-driven traction/model acceptance, valid evidence must evaluate full-run adherence over complete eligible logs. Do not accept or reject a model from isolated samples, one-row checks, short windows, short-run log entries, sampled prediction files, or step-response snippets. Step responses require several samples before the sensors show the physical effect, so one-sample/short-window fits are timing diagnostics only. If a diagnostic window is used as tuning evidence, it must cover at least `500 ms`; smaller first-sample or first-N snippets are invalid even as tuning evidence except for smoke/debug visualization.

This rule is not a unit-test sizing rule. Ordinary unit tests, small deterministic unit tests, and code-level behavior tests may use short fixtures when they verify code behavior rather than data-driven model acceptance.

Acceptance scoring must use raw log data and current source/build constants through the shared raw-sensor harness. Gyro and accelerometer bias must be independently evaluated through the centralized bias evaluator used by `codex_analysis/raw_sensor_model_eval_shared`. UKF state, estimator diagnostics, logged corrected IMU fields, prebuilt feature tables, and generated derived datasets are not scoring targets.

Core logs must be preserved. Generated testing-specific derived datasets that imply short-run/sample-level acceptance are cleanup candidates, not evidence; report a delete list before deleting them unless cleanup is explicitly assigned.

## Common Input

Legacy feature-table input for historical reports:

`codex_analysis/contact_continuum_yaw_identification/ablation/phase_classified_feature_sample.csv`

This file was the common comparison basis for earlier analysis because it contains contact-continuum features, run-quality recommendations, phase labels, and residual yaw torque targets. It is not valid acceptance input under the superseding rule above. New acceptance work must stream raw sensor rows from complete logs and may use feature tables only as provenance or historical comparison aids.

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

For model acceptance, report per-run and aggregate full-run adherence from the shared raw-sensor harness. One-step or sampled-row metrics may remain in reports as diagnostics only when the diagnostic window spans at least `500 ms`; they must be labeled non-acceptance and must not be the final pass/fail criterion.

## Production Eligibility Notes

A model is more eligible when it:

- uses per-contact relative velocity or force primitives,
- is continuous through zero forward speed,
- does not branch on maneuver labels,
- does not require a runtime residual table,
- improves held-out validation without worsening high-speed or straight-line behavior.

