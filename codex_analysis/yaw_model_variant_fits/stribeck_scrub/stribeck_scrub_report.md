# Variant B: Stribeck Static-to-Sliding Yaw Scrub Fit

Analysis-only output. Production code, build metadata, and tests were not modified.

## Model Form

The fitted correction predicts an additional yaw torque residual that opposes the current yaw direction:

`M_opp = A(v_yaw, M_req) * R(v_transition) * (K_slide + K_static * exp(-(v_transition / v_s)^2)) + K_viscous * v_yaw * R(v_transition)`

where:

- `A(v_yaw, M_req)` is selected by grid search as either request-only breakaway activation or a smooth union of yaw-patch motion and positive requested yaw-moment activation.
- `v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)` drives the static-to-sliding Stribeck fade.
- `R(v_transition) = 1 / (1 + (v_transition / speed_fade)^2)` limits this variant to low/contact-speed scrub rather than adding high-speed yaw drag.
- The fitted torque is converted back to additive yaw torque as `-sign(yaw_rate) * M_opp`.

This intentionally captures breakaway/static scrub and sliding scrub; it is not a full combined-slip contact model.

## Data Basis

- Primary input: `codex_analysis\contact_continuum_yaw_identification\ablation\phase_classified_feature_sample.csv`
- Rows evaluated: 118580
- Runs evaluated: 49
- Fit-authoritative rows are the primary training signal.
- Open-floor `fit_downweighted` rows contribute at reduced weight so the May 4 low-speed yaw-launch data influences the static breakaway term.
- Validation-only and competition rows are reported only; they are not used for fitting.

## Coefficients

| parameter | value | unit |
| --- | --- | --- |
| yaw_activation_mps | 0.002 | m/s |
| req_activation_nm | 0.035 | Nm |
| stribeck_speed_mps | 0.1 | m/s |
| speed_fade_mps | 0.64 | m/s |
| rel_weight | 0.75 | dimensionless |
| activation_mode_request_only | 1 | boolean |
| include_yaw_viscous_basis | 0 | boolean |
| static_extra_nm | 0.00239126 | Nm |
| sliding_nm | 0.0630316 | Nm |
| yaw_viscous_nm_per_mps | 0 | Nm per (m/s) |
| weighted_train_opposes_rmse_nm | 0.0243064 | Nm |

## Split Metrics

| dataset_split | count | run_count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_mae_nm | corrected_mae_nm | mae_improvement_pct | baseline_median_abs_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | 14448 | 13 | 0.0512664 | 0.048945 | 4.52798 | 0.0330564 | 0.0306099 | 7.40087 | 0.0163595 | 0.0138442 |
| diag_validation_only | 11108 | 3 | 0.0846209 | 0.084412 | 0.246899 | 0.0738061 | 0.0733077 | 0.675302 | 0.08236 | 0.0820118 |
| open_floor_fit_downweighted | 31165 | 9 | 0.0431941 | 0.0418801 | 3.04203 | 0.0278497 | 0.0260676 | 6.3991 | 0.0133144 | 0.0109004 |
| open_floor_validation_only | 14542 | 9 | 0.0169307 | 0.0113719 | 32.8329 | 0.0103688 | 0.0071844 | 30.7115 | 0.00520123 | 0.00446137 |
| primary_open_floor_fit_authoritative | 47317 | 15 | 0.0368662 | 0.0285279 | 22.6177 | 0.0254802 | 0.0183715 | 27.8987 | 0.0160446 | 0.01113 |

## Selected Log Metrics

| run_id | present | dataset_split | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_signed_median_nm | corrected_signed_median_nm | median_residual_opposes_yaw_before_nm | median_residual_opposes_yaw_after_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | True | open_floor_fit_downweighted | 3456 | 0.035846 | 0.0211286 | 41.0573 | 0.00936 | 0.00039884 | 0.0142265 | -0.00568916 |
| 2026-05-04_16-57-53 | True | open_floor_validation_only | 1761 | 0.0232779 | 0.0152891 | 34.3193 | -0.000686103 | -0.000805684 | 0.00412326 | -0.00248841 |
| 2026-04-22_12-10-34 | True | open_floor_fit_downweighted | 2187 | 0.0162171 | 0.0134987 | 16.763 | -0.00311561 | -0.00157131 | 0.00356107 | 0.00131292 |
| 2026-04-22_01-06-32 | True | primary_open_floor_fit_authoritative | 1031 | 0.0449753 | 0.0340446 | 24.3038 | 0.00999115 | 0.00257069 | 0.0217022 | -0.000140862 |
| 2026-04-21_05-32-06 | True | primary_open_floor_fit_authoritative | 8880 | 0.0425842 | 0.0278351 | 34.6351 | 0.012155 | 0.00547674 | 0.015317 | -0.00981986 |
| 2026-04-21_00-16-10 | True | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.035146 | 11.7466 | 0.00894115 | 0.00457281 | -0.0029261 | -0.00975657 |
| 2026-04-20_12-10-58 | True | primary_open_floor_fit_authoritative | 2925 | 0.0403548 | 0.0365036 | 9.54344 | 0.00367495 | 0.00270925 | -0.00190567 | -0.00620763 |
| 2026-04-20_08-38-39 | True | open_floor_fit_downweighted | 7284 | 0.0562254 | 0.0562596 | -0.0608978 | 0.00016737 | 2.67121e-06 | -0.0137606 | -0.0343632 |
| diag003 | True | diag_validation_only | 5580 | 0.0852381 | 0.085034 | 0.239518 | -0.0124401 | -0.0105773 | -0.0847313 | -0.0847437 |

## Motion Slice Metrics

| dataset_split | motion_slice | count | baseline_rmse_nm | corrected_rmse_nm | rmse_improvement_pct | baseline_median_abs_nm | corrected_median_abs_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | force_limited_or_saturated_contact | 2685 | 0.095825 | 0.0921311 | 3.85482 | 0.0995428 | 0.0935067 |
| aux_downweighted_validation | general | 3487 | 0.0205014 | 0.0189457 | 7.58836 | 0.00702336 | 0.00658315 |
| aux_downweighted_validation | high_forward_turn | 246 | 0.065724 | 0.0658067 | -0.125765 | 0.0282835 | 0.0273544 |
| aux_downweighted_validation | low_speed_low_yaw_breakaway | 4965 | 0.0320813 | 0.029409 | 8.32975 | 0.017448 | 0.0123557 |
| aux_downweighted_validation | low_speed_turn_scrub | 778 | 0.0617186 | 0.0575858 | 6.69619 | 0.0365356 | 0.0314815 |
| aux_downweighted_validation | mid_speed_turn | 858 | 0.0519633 | 0.0513896 | 1.10399 | 0.0392991 | 0.0373054 |
| aux_downweighted_validation | straight_or_near_straight | 1429 | 0.0167188 | 0.0159662 | 4.50134 | 0.00566648 | 0.00530207 |
| diag_validation_only | force_limited_or_saturated_contact | 4498 | 0.11109 | 0.11107 | 0.0176815 | 0.110811 | 0.110811 |
| diag_validation_only | general | 1416 | 0.0280335 | 0.0275914 | 1.57709 | 0.0127856 | 0.0124453 |
| diag_validation_only | high_forward_turn | 39 | 0.0328062 | 0.0328062 | 0 | 0.027717 | 0.027717 |
| diag_validation_only | low_speed_low_yaw_breakaway | 2407 | 0.0561826 | 0.0550878 | 1.94856 | 0.0497589 | 0.0487837 |
| diag_validation_only | low_speed_turn_scrub | 1491 | 0.0891771 | 0.0890423 | 0.151196 | 0.0890855 | 0.0890231 |
| diag_validation_only | mid_speed_turn | 811 | 0.0643611 | 0.0643611 | 0 | 0.0606142 | 0.0606142 |
| diag_validation_only | straight_or_near_straight | 446 | 0.0118 | 0.0109317 | 7.3583 | 0.00624869 | 0.0057968 |
| open_floor_fit_downweighted | force_limited_or_saturated_contact | 11234 | 0.0640083 | 0.063674 | 0.52228 | 0.0432153 | 0.0444205 |
| open_floor_fit_downweighted | general | 8397 | 0.0155486 | 0.014087 | 9.39979 | 0.00520301 | 0.00492787 |
| open_floor_fit_downweighted | high_forward_turn | 353 | 0.0539608 | 0.0494844 | 8.29565 | 0.0334946 | 0.0257719 |
| open_floor_fit_downweighted | low_speed_low_yaw_breakaway | 3646 | 0.0288905 | 0.0194934 | 32.5267 | 0.0136859 | 0.00751708 |
| open_floor_fit_downweighted | low_speed_turn_scrub | 1514 | 0.047817 | 0.0422037 | 11.739 | 0.0396902 | 0.0261308 |
| open_floor_fit_downweighted | mid_speed_turn | 706 | 0.0515571 | 0.0519774 | -0.815207 | 0.025437 | 0.0249707 |
| open_floor_fit_downweighted | straight_or_near_straight | 5315 | 0.0113077 | 0.0105762 | 6.46878 | 0.00457215 | 0.00438006 |
| open_floor_validation_only | force_limited_or_saturated_contact | 42 | 0.0638762 | 0.0520145 | 18.5698 | 0.0494223 | 0.0379969 |
| open_floor_validation_only | general | 5270 | 0.0132603 | 0.00741744 | 44.0628 | 0.00385908 | 0.00363827 |
| open_floor_validation_only | high_forward_turn | 1 | 0.000810716 | 0.000810716 | 0 | 0.000810716 | 0.000810716 |
| open_floor_validation_only | low_speed_low_yaw_breakaway | 5964 | 0.018067 | 0.011395 | 36.9294 | 0.0070793 | 0.00499809 |
| open_floor_validation_only | low_speed_turn_scrub | 1011 | 0.0298097 | 0.0227443 | 23.7017 | 0.0242984 | 0.0136528 |
| open_floor_validation_only | mid_speed_turn | 97 | 0.0378762 | 0.0314297 | 17.0199 | 0.0142446 | 0.00942398 |
| open_floor_validation_only | straight_or_near_straight | 2157 | 0.00631909 | 0.00623205 | 1.3774 | 0.00393422 | 0.00386626 |
| primary_open_floor_fit_authoritative | force_limited_or_saturated_contact | 9508 | 0.0565453 | 0.0469654 | 16.942 | 0.0350841 | 0.0267797 |
| primary_open_floor_fit_authoritative | general | 19559 | 0.0285147 | 0.0198023 | 30.554 | 0.0126825 | 0.00883796 |
| primary_open_floor_fit_authoritative | high_forward_turn | 1294 | 0.0377362 | 0.0305099 | 19.1495 | 0.0230572 | 0.0154848 |
| primary_open_floor_fit_authoritative | low_speed_low_yaw_breakaway | 4489 | 0.0313188 | 0.0202077 | 35.4773 | 0.016579 | 0.00883122 |
| primary_open_floor_fit_authoritative | low_speed_turn_scrub | 2723 | 0.0488966 | 0.0319719 | 34.6133 | 0.0349342 | 0.0158918 |
| primary_open_floor_fit_authoritative | mid_speed_turn | 1909 | 0.0379542 | 0.0337211 | 11.153 | 0.0220511 | 0.0129865 |
| primary_open_floor_fit_authoritative | straight_or_near_straight | 7835 | 0.0178606 | 0.0154686 | 13.3926 | 0.00696167 | 0.00666384 |

## Over/Under Fit Summary

| abs_forward_bin_mps | abs_yaw_bin_radps | count | run_count | median_target_opposes_yaw_nm | median_predicted_opposing_scrub_nm | median_corrected_opposes_yaw_nm | fit_direction |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0-0.03 | 3-8 | 4033 | 34 | -0.103265 | 0 | -0.103265 | unaddressed_current_model_overresistance |
| 0.20-0.35 | 3-8 | 1016 | 30 | -0.0916104 | 0 | -0.0918084 | unaddressed_current_model_overresistance |
| 0-0.03 | 1-3 | 3769 | 41 | -0.0584665 | 0 | -0.0626398 | unaddressed_current_model_overresistance |
| 0.60+ | 0.30-1 | 139 | 18 | -0.0466231 | 0 | -0.0468267 | unaddressed_current_model_overresistance |
| 0.08-0.20 | 1-3 | 2062 | 37 | -0.0246941 | 0 | -0.043754 | unaddressed_current_model_overresistance |
| 0.20-0.35 | 1-3 | 1132 | 35 | -0.0317007 | 0 | -0.0430571 | unaddressed_current_model_overresistance |
| 0.08-0.20 | 3-8 | 808 | 30 | -0.015899 | 0 | -0.0395629 | unaddressed_current_model_overresistance |
| 0.08-0.20 | 0.30-1 | 2726 | 48 | -0.0346634 | 0 | -0.0388835 | unaddressed_current_model_overresistance |
| 0.20-0.35 | 0.30-1 | 1589 | 41 | -0.0330018 | 0 | -0.0358203 | unaddressed_current_model_overresistance |
| 0.35-0.60 | 8+ | 461 | 10 | -0.032477 | 0 | -0.0330605 | unaddressed_current_model_overresistance |
| 0.03-0.08 | 1-3 | 2806 | 39 | 0.000299342 | 0.0156724 | -0.0295358 | overfit_added_too_much_resistance |
| 0.03-0.08 | 3-8 | 2001 | 34 | 0.00469478 | 0.0195875 | -0.0290717 | overfit_added_too_much_resistance |

## Interpretation

- The model is strongest in low-speed and low/medium-speed yaw-scrub rows where an attempted yaw moment exists but contact-relative speed is still low.
- It sacrifices high-speed coverage deliberately: the transition-speed relief term prevents the May 4 static/breakaway fit from becoming a large high-speed or high-yaw-rate damper.
- Remaining positive `median_corrected_opposes_yaw_nm` means this variant still underfits opposing resistance. Negative values with near-zero predicted scrub are existing current-model over-resistance that this one-sided scrub term cannot fix; negative values with nontrivial predicted scrub indicate over-added resistance.
- Because the model is always opposing-yaw, it cannot represent rows where the current plant over-resists and the correction should assist yaw.
- The static term relies on logged/contact-reconstructed requested yaw moment for near-zero-yaw launch behavior; a pure yaw-rate-only Stribeck model cannot explain the May 4 stalled-demand rows.

## Output Files

- `stribeck_coefficients.csv`
- `hyperparameter_grid_top.csv`
- `metrics_by_split.csv`
- `metrics_by_split_phase.csv`
- `metrics_by_motion_slice.csv`
- `metrics_by_selected_run.csv`
- `over_under_fit_bins.csv`
- `prediction_sample.csv`

## Constants Read

- yaw denominator including wheel spin-up: 0.000246510400642 kg m^2
- track width: 0.084635 m
- drive wheel longitudinal offset: 0.01475 m
