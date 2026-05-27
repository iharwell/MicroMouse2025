# Shifted/Peak-Maintained Linear-Blend-to-Stribeck Continuous Optimization

Analysis-only output. Production code, build metadata, tests, and prior analysis artifacts were not edited.

## Model

`y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v*k1`, `x2 = v*k2 + k3`, with `k1 < k2` and `x2 > x1`.

The selected form uses a shifted Stribeck section whose peak is maintained inside the blend interval. The linear branch is anchored at zero and its slope is derived from the clean handoff or peak-maintenance condition; no independent linear slope is fitted.

The optimized variants include `x1_shifted_stribeck_peak_slope`, `line_reaches_x2`, `line_meets_x1`, and `line_maintains_peak`. In the x1-shifted variant, Stribeck parameters stay fixed, the Stribeck input is shifted by `x1`, and the peak lies at `xp=x1+peak_frac*(x2-x1)`, so the rising Stribeck curve is already present between `x1` and `xp`.

The blend section always uses `w=(y-x1)/(x2-x1)` and `(1-w)*L(y,v)+w*S_shifted(y,v)`.

The Stribeck peak is not forced to a transition endpoint. It is optimized inside the transition region and the curve shifts with `x1(Vf)` so the maintained peak magnitude remains meaningful while the handoff locations move.

## Optimizer

Spaced/random points were used only to choose initial seeds. Final fitting used a differential-evolution-style continuous global pass, then Nelder-Mead on transformed continuous parameters, followed by coordinate descent on the full weighted objective.

Objective: weighted RMSE of `residual_opposes_yaw_nm - predicted_opposing_nm`, with coefficients solved analytically under the launch equality constraint and non-negativity. The launch equality target is derived from the measured `+/-0.646` in-place command threshold using the motor inverse model, not added as a giant pseudo-row.

Transforms: `k1=exp(a)`, `k2=k1+exp(b)`, `k3=exp(c)`, `peak_frac=0.005+0.99*sigmoid(d)`, `decay0=exp(e)`. The optimized comparison fixes `decay_v=0` and `vf_fade=1` because the Stribeck shape is shifted by input, not retuned with `Vf`.

No soft physical preference penalty is applied to the primary optimized objective. Boundary and shape diagnostics are reported separately so the best-fit behavior of the shifted family is visible even if it is not acceptable as a production recommendation.

## Search Status

No candidate is accepted as selected. The best scored candidate remained boundary-adjacent after range expansion, so the search is marked unresolved per the boundary rule.

## Best Unresolved Candidate

| parameter | value |
| --- | ---: |
| interpretation | line_reaches_x2 |
| k1 | 0.055611 |
| k2 | 6.196412 |
| k3 | 1.015480 |
| peak_frac | 0.990406 |
| decay0 | 0.019659 |
| decay_v | 0.000000 |
| vf_fade | 1.000000 |
| slide_nm | 0.003661 |
| peak_delta_nm | 0.067433 |
| peak_extra_nm | 0.071094 |
| weighted_train_rmse_nm | 0.031228 |
| launch_extra_nm | 0.065568 |
| launch_total_nm | 0.080362 |
| launch_left_command | 0.646000 |
| launch_right_command | -0.646000 |
| launch_max_abs_command | 0.646000 |
| boundary_status | k1:low;peak_frac:high;decay0:low |

## Variant Comparison

| variant | objective | train RMSE | validation RMSE | launch | peak | boundary/status |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| x1_shifted_stribeck_peak_slope | 0.031281 | 0.031281 | 0.052530 | 0.646000 | 0.065587 | k1:low;peak_frac:high;decay0:low |
| line_reaches_x2 | 0.031228 | 0.031228 | 0.052642 | 0.646000 | 0.071094 | k1:low;peak_frac:high;decay0:low |
| line_meets_x1 | 0.031368 | 0.031368 | 0.052307 | 0.646000 | 0.069351 | k1:low;k2:high;peak_frac:high;decay0:low |
| line_maintains_peak | 0.031281 | 0.031281 | 0.052712 | 0.646000 | 0.065569 | k1:low;peak_frac:high;decay0:low |

## Optimizer Audit

Base branch evaluated `56` continuous optimizer endpoints. Latest-weighted sensitivity evaluated `14` endpoints.

| branch | optimizer | seed | NM evals | NM converged | full evals | objective |
| --- | --- | ---: | ---: | --- | ---: | ---: |
| base:x1_shifted_stribeck_peak_slope | differential_evolution(sample objective) | global | 2210 |  |  | 0.028145 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 0 | 521 | False | 81 | 0.031322 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 1 | 521 | False | 81 | 0.031296 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 2 | 521 | False | 81 | 0.031303 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 3 | 520 | False | 81 | 0.031297 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 4 | 520 | False | 81 | 0.031302 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 5 | 520 | False | 81 | 0.031297 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 6 | 521 | False | 81 | 0.031335 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 7 | 521 | False | 81 | 0.031328 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 8 | 520 | False | 81 | 0.031336 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 9 | 521 | False | 81 | 0.031333 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 10 | 520 | False | 81 | 0.031282 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 11 | 520 | False | 81 | 0.031303 |
| base:x1_shifted_stribeck_peak_slope | Nelder-Mead(sample objective) + coordinate descent(full objective) | 12 | 520 | False | 81 | 0.031290 |
| base:x1_shifted_stribeck_peak_slope | final coordinate polish(full objective) | best |  |  | 121 | 0.031281 |
| base:line_reaches_x2 | differential_evolution(sample objective) | global | 2210 |  |  | 0.027945 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 0 | 520 | False | 81 | 0.031242 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 1 | 520 | False | 81 | 0.031356 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 2 | 520 | False | 81 | 0.031358 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 3 | 521 | False | 81 | 0.031269 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 4 | 521 | False | 81 | 0.031235 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 5 | 520 | False | 81 | 0.031401 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 6 | 521 | False | 81 | 0.031378 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 7 | 521 | False | 81 | 0.031233 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 8 | 520 | False | 81 | 0.031257 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 9 | 520 | False | 81 | 0.031261 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 10 | 520 | False | 81 | 0.031237 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 11 | 520 | False | 81 | 0.031230 |
| base:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 12 | 521 | False | 81 | 0.031234 |
| base:line_reaches_x2 | final coordinate polish(full objective) | best |  |  | 121 | 0.031228 |
| base:line_meets_x1 | differential_evolution(sample objective) | global | 2210 |  |  | 0.028446 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 0 | 493 | True | 81 | 0.031368 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 1 | 520 | False | 81 | 0.031533 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 2 | 520 | False | 81 | 0.031537 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 3 | 521 | False | 81 | 0.033546 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 4 | 521 | False | 81 | 0.032862 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 5 | 520 | False | 81 | 0.031537 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 6 | 520 | False | 81 | 0.031537 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 7 | 520 | False | 81 | 0.032801 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 8 | 520 | False | 81 | 0.033008 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 9 | 520 | False | 81 | 0.033594 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 10 | 520 | False | 81 | 0.033262 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 11 | 521 | False | 81 | 0.033547 |
| base:line_meets_x1 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 12 | 521 | False | 81 | 0.031535 |
| base:line_meets_x1 | final coordinate polish(full objective) | best |  |  | 121 | 0.031368 |
| base:line_maintains_peak | differential_evolution(sample objective) | global | 2210 |  |  | 0.028252 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 0 | 520 | False | 81 | 0.031365 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 1 | 520 | False | 81 | 0.031296 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 2 | 521 | False | 81 | 0.031310 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 3 | 520 | False | 81 | 0.031291 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 4 | 520 | False | 81 | 0.031298 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 5 | 520 | False | 81 | 0.031295 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 6 | 521 | False | 81 | 0.031373 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 7 | 520 | False | 81 | 0.031325 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 8 | 520 | False | 81 | 0.031342 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 9 | 520 | False | 81 | 0.031285 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 10 | 521 | False | 81 | 0.031284 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 11 | 521 | False | 81 | 0.031306 |
| base:line_maintains_peak | Nelder-Mead(sample objective) + coordinate descent(full objective) | 12 | 520 | False | 81 | 0.031286 |
| base:line_maintains_peak | final coordinate polish(full objective) | best |  |  | 121 | 0.031281 |
| latest_weighted:line_reaches_x2 | differential_evolution(sample objective) | global | 2210 |  |  | 0.028907 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 0 | 521 | False | 81 | 0.030911 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 1 | 520 | False | 81 | 0.030914 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 2 | 520 | False | 81 | 0.030954 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 3 | 521 | False | 81 | 0.030911 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 4 | 520 | False | 81 | 0.030927 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 5 | 520 | False | 81 | 0.030914 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 6 | 520 | False | 81 | 0.030958 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 7 | 520 | False | 81 | 0.030954 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 8 | 520 | False | 81 | 0.030920 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 9 | 520 | False | 81 | 0.030914 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 10 | 521 | False | 81 | 0.030916 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 11 | 521 | False | 81 | 0.030942 |
| latest_weighted:line_reaches_x2 | Nelder-Mead(sample objective) + coordinate descent(full objective) | 12 | 520 | False | 81 | 0.030917 |
| latest_weighted:line_reaches_x2 | final coordinate polish(full objective) | best |  |  | 121 | 0.030911 |

## Boundary Audit

| parameter | range min | selected | range max | from low | from high | adjacent |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| k1 | 0.000500 | 0.055611 | 20.000000 | 0.002756 | 0.997244 | True |
| k2 | 0.020000 | 6.196412 | 60.000000 | 0.102975 | 0.897025 | False |
| k3 | 0.005000 | 1.015480 | 8.000000 | 0.126389 | 0.873611 | False |
| peak_frac | 0.005000 | 0.990406 | 0.995000 | 0.995360 | 0.004640 | True |
| decay0 | 0.000500 | 0.019659 | 8.000000 | 0.002395 | 0.997605 | True |
| decay_v | 0.000000 | 0.000000 | 0.000000 |  |  | False |
| vf_fade | 1.000000 | 1.000000 | 1.000000 |  |  | False |

No selected base parameter is accepted if it remains boundary-adjacent after expansion. This run's selected base candidate is not interior: `k1:low;peak_frac:high;decay0:low`.

## Split Metrics

| split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| aux_downweighted_validation | 14448 | 0.051266 | 0.053631 | 0.019198 | 0.048820 | 0.027490 |
| diag_validation_only | 11108 | 0.084621 | 0.089793 | 0.020030 | 0.084410 | 0.038489 |
| open_floor_fit_downweighted | 31165 | 0.043194 | 0.044576 | 0.027952 | 0.041507 | 0.032823 |
| open_floor_validation_only | 14542 | 0.016931 | 0.017474 | 0.007665 | 0.011394 | 0.011207 |
| primary_open_floor_fit_authoritative | 47317 | 0.036866 | 0.036479 | 0.016688 | 0.027908 | 0.021083 |
| validation_non_authoritative | 71263 | 0.050234 | 0.052642 | 0.022157 |  | 0.029680 |

## Selected Logs

| run | split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035846 | 0.034326 | 0.011393 | 0.020956 | 0.020128 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023278 | 0.023430 | 0.009662 | 0.015366 | 0.015839 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217 | 0.015664 | 0.012232 | 0.013487 | 0.013203 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.044975 | 0.044010 | 0.011282 | 0.033938 | 0.016861 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.042584 | 0.041578 | 0.015434 | 0.027284 | 0.017657 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039824 | 0.040591 | 0.020474 | 0.034302 | 0.026878 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.040355 | 0.040956 | 0.022853 | 0.035648 | 0.027977 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.056225 | 0.058852 | 0.036549 | 0.056141 | 0.042487 |
| diag003 | diag_validation_only | 5580 | 0.085238 | 0.090490 | 0.019757 | 0.085032 | 0.038712 |

## Risk Metrics

| slice | count | baseline RMSE | shifted RMSE | improvement |
| --- | ---: | ---: | ---: | ---: |
| latest_may4_all | 5217 | 0.032158 | 0.031078 | 0.033574 |
| launch_neighborhood_abs_vf_lt_0p08_yaw_0p5_to_1p5 | 5369 | 0.069236 | 0.073944 | -0.067990 |
| low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5 | 20694 | 0.072832 | 0.075767 | -0.040297 |
| high_speed_abs_vf_ge_0p7 | 144 | 0.088793 | 0.089022 | -0.002583 |
| limiter_active | 29069 | 0.074862 | 0.076933 | -0.027677 |
| hardware_saturation_evidence | 5017 | 0.063673 | 0.065518 | -0.028977 |
| line_branch_rows | 5163 | 0.033433 | 0.033435 | -0.000045 |
| blend_branch_rows | 93733 | 0.038238 | 0.039682 | -0.037786 |
| stribeck_branch_rows | 19684 | 0.071744 | 0.073754 | -0.028019 |

## Latest-Weighted Sensitivity

The sensitivity branch reserves 30% effective fit weight for the two May 4 logs while retaining quality penalties and run balancing. It is reported as sensitivity, not silently substituted for the base selection.

| parameter | base selected | latest-weighted selected |
| --- | ---: | ---: |
| k1 | 0.055611 | 1.418913 |
| k2 | 6.196412 | 1.474135 |
| k3 | 1.015480 | 1.010898 |
| peak_frac | 0.990406 | 0.991040 |
| decay0 | 0.019659 | 0.008547 |
| decay_v | 0.000000 | 0.000000 |
| vf_fade | 1.000000 | 1.000000 |
| peak_extra_nm | 0.071094 | 0.068238 |
| launch_max_abs_command | 0.646000 | 0.646000 |
| latest_may4_rmse_nm | 0.031078 | 0.030704 |

## Launch Tradeoff

The best unresolved boundary candidate's launch estimate at `Vf=0`, `Vr=0`, `yawRate=+1` is `0.646000/-0.646000` with `|cmd|=0.646000`. It passes the `|cmd| >= 0.6` gate but is not accepted because the candidate is boundary-bound.

## Output Files

- `candidate_scores.csv`
- `variant_comparison.csv`
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

## Suggested Improved Formulations

The optimized shifted-only family can hit the launch equality analytically, but its best fits tend to collapse toward narrow peaks or boundary-like handoff geometry. That is useful evidence: one scalar shifted Stribeck curve is not enough to own both launch breakaway and moving tire behavior.

1. Use a launch/static component: an x1-shifted Stribeck or bristle-like term whose peak/yield is solved from measured in-place breakaway command, with width constrained by measured launch transient duration rather than broad residual RMSE alone.

2. Use a moving tire component based on contact-relative velocity or slip-angle style primitives: per-patch lateral and longitudinal forces from `v_rel_r`, `v_rel_f`, normal load, and a smooth saturation law such as rational/Pacejka-lite `mu*N*s/sqrt(s^2+k^2)`. This aligns better with the standalone contact traction testbed and Variant C behavior.

3. Partition the components smoothly instead of adding them blindly: static launch owns `Vf≈0` and very low contact-relative speed; conventional slip owns moving arcs. The partition should depend on speed/contact-relative velocity/utilization only, not command/request/preprojection/UKF state.

4. Fit the combined surface with launch as an equality or inequality constraint, then optimize validation RMSE and risk slices. Reject solutions whose peak width, peak location, or low-speed slope collapses to guard bounds, because that indicates missing moving-slip structure rather than a good Stribeck-only fit.
