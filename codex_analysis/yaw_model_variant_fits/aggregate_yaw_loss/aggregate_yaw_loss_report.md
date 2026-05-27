# Variant E: Aggregate Yaw-Loss Baseline

Analysis-only output. Production code, build metadata, and tests were not modified.

## Input and Split Basis

- Primary input: `codex_analysis\contact_continuum_yaw_identification\ablation\phase_classified_feature_sample.csv`
- Samples used: 118580 across 49 runs.
- Training signal: `primary_open_floor_fit_authoritative` rows from the shared `dataset_split` column.
- Validation: downweighted open-floor, validation-only open-floor, `diag`, and `aux` rows reported separately and combined.
- Target convention: positive predicted loss means an added yaw torque opposing the measured yaw rate. Corrected residual is `residual_additive_yaw_torque_nm + sign(yaw_rate) * predicted_loss_nm`.
- Yaw denominator reference from constants: `0.000246510401 kg*m^2`; this report keeps the required metrics in Nm.

## Validation-Guarded Physical Selection

- Model: `E1_viscous_yaw_rate_nnls`
- Form: `loss = k_yaw * |yaw_rate|`
- Train target weighted R2 against `residual_opposes_yaw_nm`: -0.022839
- Primary fit-authoritative residual RMSE: 0.036866177 -> 0.036866177 Nm (0.000%)
- Combined non-authoritative validation RMSE: 0.050233832 -> 0.050233832 Nm (0.000%)
- Validation worsened-sample fraction: 0.000%

This selection is intentionally allowed to choose zero. It is the validation-safe physical aggregate-loss result: do not add an aggregate yaw-loss term from this family.

Coefficients:

| Feature | Coefficient |
| --- | ---: |
| `abs_yaw_rate_radps` | 0 |

## Best Nonzero Physical Aggregate Loss

- Model: `E5_contact_faded_loss_nnls_yaw0_0.5_vrel0_0.01`
- Form: `loss = gate_vrel(0.01) * [tau_c*tanh(|yaw|/0.5) + k_yaw*|yaw|]`
- Train target weighted R2: 0.018765
- Primary fit-authoritative residual RMSE: 0.036866177 -> 0.036561548 Nm (-0.826%)
- Combined non-authoritative validation RMSE: 0.050233832 -> 0.051093245 Nm (1.711%)
- Validation worsened-sample fraction: 69.402%

| Feature | Coefficient |
| --- | ---: |
| `vrel_gate_0.01_tanh_yaw_0.5` | 0.0368985607533 |
| `vrel_gate_0.01_abs_yaw` | 0.0246164642736 |

## Signed Relief Diagnostic

The best signed aggregate surface is reported as a diagnostic because it can predict negative loss, which means adding yaw torque in the direction of rotation to compensate current over-resistance. That is not a pure aggregate Coulomb/damping loss.

- Model: `E8_signed_contact_relief_yaw0_0.1_vrel0_0.03`
- Form: `loss_or_relief = tau_c*tanh(|yaw|/0.1) + k_yaw*|yaw| + tau_h*high_vrel(0.03)*tanh(|yaw|/0.1) + k_h*high_vrel(0.03)*|yaw|`
- Train target weighted R2: 0.039381
- Combined validation RMSE: 0.050233832 -> 0.053275506 Nm (6.055%)

## Top Candidate Overview

Top physical yaw-loss candidates by combined validation corrected RMSE:

| Model | Train target R2 | Primary delta | Validation delta | Validation RMSE Nm |
| --- | ---: | ---: | ---: | ---: |
| `E1_viscous_yaw_rate_nnls` | -0.022839 | 0.000% | 0.000% | 0.050233832 |
| `E5_contact_faded_loss_nnls_yaw0_0.5_vrel0_0.01` | 0.018765 | -0.826% | 1.711% | 0.051093245 |
| `E5_contact_faded_loss_nnls_yaw0_1_vrel0_0.01` | 0.020282 | -0.911% | 1.717% | 0.051096485 |
| `E5_contact_faded_loss_nnls_yaw0_0.25_vrel0_0.01` | 0.018028 | -0.783% | 1.734% | 0.051104847 |
| `E5_contact_faded_loss_nnls_yaw0_0.1_vrel0_0.01` | 0.017324 | -0.767% | 1.756% | 0.051116015 |

Top signed-relief diagnostics:

| Model | Train target R2 | Primary delta | Validation delta | Validation RMSE Nm |
| --- | ---: | ---: | ---: | ---: |
| `E8_signed_contact_relief_yaw0_0.1_vrel0_0.03` | 0.039381 | -2.136% | 6.055% | 0.053275506 |
| `E8_signed_contact_relief_yaw0_0.1_vrel0_0.06` | 0.038591 | -2.210% | 6.937% | 0.053718608 |
| `E7_signed_forward_relief_yaw0_0.1_vf0_0.8` | 0.046450 | -2.533% | 7.568% | 0.054035538 |
| `E7_signed_forward_relief_yaw0_0.1_vf0_0.4` | 0.046736 | -2.587% | 7.697% | 0.054100527 |
| `E8_signed_contact_relief_yaw0_0.1_vrel0_0.12` | 0.039936 | -2.397% | 7.840% | 0.054172372 |

## Selected Log Results

These rows use the best nonzero physical aggregate-loss fit, `E5_contact_faded_loss_nnls_yaw0_0.5_vrel0_0.01`. The validation-guarded physical selection is the zero-loss model above.

| Run | Present | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta | Median residual before Nm | Median residual after Nm | Negative target frac |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `2026-05-04_20-35-47` | yes | 3456 | 0.035845956 | 0.033484175 | -6.589% | 0.009360001 | 0.008040902 | 33.478% |
| `2026-05-04_16-57-53` | yes | 1761 | 0.023277853 | 0.024208491 | 3.998% | -0.000686103 | -0.000530188 | 41.908% |
| `2026-04-22_12-10-34` | yes | 2187 | 0.016217133 | 0.016213394 | -0.023% | -0.003115615 | -0.002446830 | 41.701% |
| `2026-04-22_01-06-32` | yes | 1031 | 0.044975279 | 0.041571239 | -7.569% | 0.009991152 | 0.005536898 | 20.660% |
| `2026-04-21_05-32-06` | yes | 8880 | 0.042584174 | 0.042198086 | -0.907% | 0.012155008 | 0.011813429 | 40.146% |
| `2026-04-21_00-16-10` | yes | 3757 | 0.039823976 | 0.040400126 | 1.447% | 0.008941154 | 0.009301137 | 54.112% |
| `2026-04-20_12-10-58` | yes | 2925 | 0.040354823 | 0.041000959 | 1.601% | 0.003674954 | 0.003883093 | 53.402% |
| `2026-04-20_08-38-39` | yes | 7284 | 0.056225358 | 0.057121014 | 1.593% | 0.000167370 | 0.000111909 | 66.612% |
| `diag003` | yes | 5580 | 0.085238129 | 0.086589801 | 1.586% | -0.012440089 | -0.017130225 | 93.620% |

## Failure Modes

Failure groups below use the best nonzero physical aggregate-loss fit, `E5_contact_faded_loss_nnls_yaw0_0.5_vrel0_0.01`.

| Group | Rows | Baseline RMSE Nm | Physical corrected RMSE Nm | Delta | Worsened frac | Negative target frac |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `low_motion_near_zero_yaw` | 28208 | 0.035125067 | 0.035681183 | 1.583% | 57.927% | 52.599% |
| `in_place_or_scrub_yaw` | 18275 | 0.075038202 | 0.075493714 | 0.607% | 68.175% | 66.161% |
| `slow_forward_turn` | 12359 | 0.056429870 | 0.057197616 | 1.361% | 63.233% | 61.186% |
| `moderate_forward_turn` | 8848 | 0.061941433 | 0.062343969 | 0.650% | 67.236% | 65.472% |
| `high_forward_turn` | 227 | 0.085776489 | 0.086664110 | 1.035% | 78.855% | 77.974% |
| `high_yaw_low_forward_scrub` | 1837 | 0.041501660 | 0.041730862 | 0.552% | 69.134% | 68.808% |
| `mostly_straight_forward` | 13657 | 0.035127008 | 0.035766847 | 1.822% | 60.826% | 54.338% |
| `target_negative_over_resisted` | 66171 | 0.052717853 | 0.054390389 | 3.173% | 100.000% | 100.000% |

Interpretation:

- The physical aggregate-loss family can only add resistance. Rows whose target is negative are already over-resisted by the current mirror; a pure loss cannot fix those rows and often worsens them.
- Forward/contact fading helps avoid some high-speed conflict, but it cannot express front/rear or left/right patch asymmetry, so it misses the contact-continuum behavior that the prior ablation found.
- The signed-relief diagnostic usually scores better when high-speed over-resistance dominates, but it is no longer an aggregate yaw-loss model and would be less production-eligible than a contact-patch force formulation.

## Output Files

- `fit_aggregate_yaw_loss.py`
- `candidate_overview.csv`
- `selected_model_coefficients.csv`
- `split_phase_metrics.csv`
- `selected_log_metrics.csv`
- `failure_mode_metrics.csv`
- `commands_run.txt`
