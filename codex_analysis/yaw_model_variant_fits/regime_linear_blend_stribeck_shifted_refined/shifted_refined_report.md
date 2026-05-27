# Shifted/Peak-Maintained Linear-Blend-to-Stribeck Refined Search

Analysis-only output. Production code, build metadata, tests, and prior analysis artifacts were not edited.

## Model

`y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v*k1`, `x2 = v*k2 + k3`, with `k1 < k2` and `x2 > x1`.

The selected form uses a shifted Stribeck section whose peak is maintained inside the blend interval. The linear branch is anchored at zero and its slope is derived from the clean handoff or peak-maintenance condition; no independent linear slope is fitted.

The primary interpretation is `x1_shifted_stribeck_peak_slope`: Stribeck parameters stay fixed, the Stribeck input is shifted by `x1`, and the peak lies at `xp=x1+peak_frac*(x2-x1)`. That means the rising Stribeck curve is already present between `x1` and `xp`, blended against the derived line continuation.

The search also evaluated `line_reaches_x2`, `line_meets_x1`, and `line_maintains_peak` as comparison interpretations. The blend section always uses `w=(y-x1)/(x2-x1)` and `(1-w)*L(y,v)+w*S_shifted(y,v)`.

The Stribeck peak is not forced to a transition endpoint. It is searched inside the transition region and the curve shifts with `Vf` so the maintained peak magnitude remains meaningful while the handoff locations move.

## Search Status

No candidate is accepted as selected. The best scored candidate remained boundary-adjacent after two range expansions, so the search is marked unresolved per the boundary rule.

The metrics below are for the best unresolved boundary candidate, included only to show the tradeoff and failure mode.

## Best Unresolved Candidate

| parameter | value |
| --- | ---: |
| interpretation | line_maintains_peak |
| k1 | 0.287364 |
| k2 | 14.800000 |
| k3 | 1.061140 |
| peak_frac | 0.960000 |
| decay0 | 0.120000 |
| decay_v | 0.000000 |
| vf_fade | 0.016200 |
| slide_nm | 0.018342 |
| peak_delta_nm | 0.041284 |
| peak_extra_nm | 0.059626 |
| weighted_train_rmse_nm | 0.028540 |
| launch_extra_nm | 0.058630 |
| launch_total_nm | 0.073424 |
| launch_left_command | 0.605340 |
| launch_right_command | -0.605340 |
| launch_max_abs_command | 0.605340 |
| boundary_status | unresolved_boundary:k2:high;peak_frac:high;decay0:low;decay_v:low;vf_fade:low |

## Search Audit

Base branch evaluated `69958` candidates. Latest-weighted sensitivity evaluated `69960` candidates.

| branch | iteration | broad | local | cumulative | boundary hits | action |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| base | 0 | 18120 | 5200 | 23320 | k2:high;k3:high;peak_frac:high;vf_fade:low | expand_and_rerun |
| base | 1 | 18120 | 5198 | 46638 | k1:high;k3:high;vf_fade:low | expand_and_rerun |
| base | 2 | 18120 | 5200 | 69958 | k2:high;peak_frac:high;decay0:low;decay_v:low;vf_fade:low | unresolved_boundary |
| latest_weighted | 0 | 18120 | 5200 | 23320 | k2:high;k3:high;decay0:low;vf_fade:low | expand_and_rerun |
| latest_weighted | 1 | 18120 | 5200 | 46640 | k3:high;peak_frac:high;vf_fade:low | expand_and_rerun |
| latest_weighted | 2 | 18120 | 5200 | 69960 | decay0:low;vf_fade:low | unresolved_boundary |

## Boundary Audit

| parameter | range min | selected | range max | from low | from high | adjacent |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| k1 | 0.025000 | 0.287364 | 2.220000 | 0.119528 | 0.880472 | False |
| k2 | 0.250000 | 14.800000 | 14.800000 | 1.000000 | 0.000000 | True |
| k3 | 0.035000 | 1.061140 | 5.476000 | 0.188594 | 0.811406 | False |
| peak_frac | 0.120000 | 0.960000 | 0.960000 | 1.000000 | 0.000000 | True |
| decay0 | 0.120000 | 0.120000 | 9.000000 | 0.000000 | 1.000000 | True |
| decay_v | 0.000000 | 0.000000 | 6.000000 | 0.000000 | 1.000000 | True |
| vf_fade | 0.016200 | 0.016200 | 4.000000 | 0.000000 | 1.000000 | True |

No selected base parameter is accepted if it remains boundary-adjacent after expansion. This run's selected base candidate is not interior: `unresolved_boundary:k2:high;peak_frac:high;decay0:low;decay_v:low;vf_fade:low`.

## Split Metrics

| split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| aux_downweighted_validation | 14448 | 0.051266 | 0.055463 | 0.019198 | 0.048820 | 0.027490 |
| diag_validation_only | 11108 | 0.084621 | 0.095534 | 0.020030 | 0.084410 | 0.038489 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.045264 | 0.027952 | 0.041507 | 0.032823 |
| open_floor_validation_only | 14542 | 0.016931 | 0.017879 | 0.007665 | 0.011394 | 0.011207 |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.036426 | 0.016688 | 0.027908 | 0.021083 |
| validation_non_authoritative | 71263 | 0.050234 | 0.054841 | 0.022157 |  | 0.029680 |

## Selected Logs

| run | split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.033667 | 0.011393 | 0.020956 | 0.020128 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.023850 | 0.009662 | 0.015366 | 0.015839 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.015639 | 0.012232 | 0.013487 | 0.013203 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.044231 | 0.011282 | 0.033938 | 0.016861 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.041693 | 0.015434 | 0.027284 | 0.017657 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.040308 | 0.020474 | 0.034302 | 0.026878 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.040883 | 0.022853 | 0.035648 | 0.027977 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.060091 | 0.036549 | 0.056141 | 0.042487 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.096450 | 0.019757 | 0.085032 | 0.038712 |

## Risk Metrics

| slice | count | baseline RMSE | shifted RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| latest_may4_all | 5217 | 0.032158 | 0.030706 | 0.045133 |
| launch_neighborhood_abs_vf_lt_0p08_yaw_0p5_to_1p5 | 5369 | 0.069236 | 0.078519 | -0.134071 |
| low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5 | 20694 | 0.072832 | 0.080057 | -0.099200 |
| high_speed_abs_vf_ge_0p7 | 144 | 0.088793 | 0.089799 | -0.011330 |
| limiter_active | 29069 | 0.074862 | 0.079943 | -0.067881 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.066332 | -0.041762 |
| line_branch_rows | 22159 | 0.026983 | 0.026989 | -0.000206 |
| blend_branch_rows | 81183 | 0.042423 | 0.044982 | -0.060307 |
| stribeck_branch_rows | 15238 | 0.073310 | 0.079664 | -0.086662 |

## Latest-Weighted Sensitivity

The sensitivity branch reserves 30% effective fit weight for the two May 4 logs while retaining quality penalties and run balancing. It is reported as sensitivity, not silently substituted for the base selection.

| parameter | base selected | latest-weighted selected |
| --- | ---: | ---: |
| k1 | 0.287364 | 0.146985 |
| k2 | 14.800000 | 7.314250 |
| k3 | 1.061140 | 1.084537 |
| peak_frac | 0.960000 | 0.912868 |
| decay0 | 0.120000 | 0.059162 |
| decay_v | 0.000000 | 1.231928 |
| vf_fade | 0.016200 | 0.016200 |
| peak_extra_nm | 0.059626 | 0.058102 |
| launch_max_abs_command | 0.605340 | 0.597547 |
| latest_may4_rmse_nm | 0.035412 | 0.029956 |

## Launch Tradeoff

The best unresolved boundary candidate's launch estimate at `Vf=0`, `Vr=0`, `yawRate=+1` is `0.605340/-0.605340` with `|cmd|=0.605340`. It passes the `|cmd| >= 0.6` gate but remains well below the `0.646` target and is not accepted because the candidate is boundary-bound.

## Output Files

- `candidate_scores.csv`
- `latest_weighted_sensitivity_candidates.csv`
- `selected_parameters.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `in_place_command_estimate.csv`
- `boundary_audit.csv`
- `search_audit.csv`
- `prediction_sample.csv`
- `metadata.json`
- `commands_run.txt`
