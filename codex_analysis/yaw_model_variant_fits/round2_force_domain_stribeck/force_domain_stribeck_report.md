# Round2 Force-Domain Stribeck Rewrite

Analysis-only output. Production code, build metadata, and tests were not modified.

## Decision

The physically acceptable rewrite is the projected-force utilization form: it keeps Variant B's static-to-sliding Stribeck torque law, but replaces raw request/command activation with yaw-moment utilization computed from the projected/actual tire contact force state and normal-load-derived yaw-moment yield.

At `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`, it predicts left/right command `0.655064465453/-0.655064465453` with extra opposing yaw torque `0.067114720317` Nm. That passes the hard `|cmd| >= 0.6` gate and stays near the prior B target of about `+0.646/-0.646`.

## Equations

For each contact `i`, with lateral/right coordinate `r_i`, longitudinal/forward coordinate `f_i`, normal load `N_i`, and projected/actual contact force `(F_f,i, F_r,i)`:

`M_contact = sum_i(f_i * F_r,i - r_i * F_f,i)`

`M_drive = smooth_positive(sign(yawRate) * M_contact)`

`M_yield = mu_ref * sum_i(|r_i| * N_i)` for the selected longitudinal/differential-drive yaw-moment support.

`u = M_drive / M_yield`

`A_u = 1 - exp(-(u / u_activation)^2)`

`v_transition = sqrt((rel_weight * vbar_rel)^2 + |Vf|^2)`

`S(v) = exp(-(v_transition / stribeck_speed)^2)`

`R(v) = 1 / (1 + (v_transition / speed_fade)^2)`

`M_extra = A_u * R(v) * (K_slide + K_static * S(v))`

For command estimates, `M_extra` is solved as a fixed point because the projected contact force needed to hold the same contact state includes the extra scrub. The synthetic in-place/grid evaluator caps the projected source moment at the nominal contact yaw-moment yield.

## Fitted Parameters

| parameter | value | unit |
| --- | --- | --- |
| activation_source | projected_force_moment_utilization | enum |
| yield_geometry | longitudinal_moment_support | enum |
| nominal_longitudinal_yield_nm | 0.11096910450167499 | Nm |
| nominal_full_yaw_yield_nm | 0.1175168045456451 | Nm |
| equivalent_activation_nm | 0.035 | Nm at nominal load |
| utilization_activation | 0.3154031039285507 | dimensionless |
| stribeck_speed_mps | 0.025 | m/s |
| speed_fade_mps | 0.64 | m/s |
| rel_weight | 0.75 | dimensionless |
| static_extra_nm | 0.0 | Nm |
| sliding_nm | 0.06741675635026155 | Nm |
| weighted_train_opposes_rmse_nm | 0.024088350221216342 | Nm |

The utilization knee is equivalent to `0.035 Nm` at nominal load, expressed as `u_activation = equivalent_activation_nm / M_yield_nominal`.

## +1 rad/s In-Place

| variant | extra_opposing_yaw_torque_nm | total_opposing_yaw_torque_nm | left_command | right_command | lr_delta_command | max_abs_command |
| --- | --- | --- | --- | --- | --- | --- |
| ForceDomainStribeck_projected_force | 0.06711472031698633 | 0.08190897031698632 | 0.6550644654532793 | -0.6550644654532793 | 1.3101289309065587 | 0.6550644654532793 |
| RequestMomentDiagnostic_rejected_command_gate | 0.06498246371468605 | 0.07977671371468605 | 0.6425685451006024 | -0.6425685451006024 | 1.2851370902012047 | 0.6425685451006024 |
| Variant B Stribeck scrub | 0.06501335920678508 | 0.07980760920678508 | 0.6427496056765714 | -0.6427496056765714 | 1.2854992113531427 | 0.6427496056765714 |
| Variant C combined slip | 0.018240534311908203 | 0.033034784311908205 | 0.368641182738375 | -0.368641182738375 | 0.73728236547675 | 0.368641182738375 |

The request-moment diagnostic is not the selected rewrite because command/request gates are prohibited. It is retained only to show how close the rejected algebraic B branch remains.

## 6x10 L-R Delta Grid

Full machine-readable grid: `lr_delta_grid_6x10.csv`. Pivot:

| Vf m/s | 0.200 | 0.844 | 1.489 | 2.133 | 2.778 | 3.422 | 4.067 | 4.711 | 5.356 | 6.000 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.000 | 0.416837 | 1.280385 | 1.401801 | 1.520569 | 1.638453 | 1.755948 | 1.873231 | 1.990324 | 2.107228 | 2.223946 |
| 0.030 | 0.045562 | 1.352753 | 1.400047 | 1.518835 | 1.636727 | 1.754228 | 1.871516 | 1.988614 | 2.105525 | 2.222250 |
| 0.060 | 0.045466 | 0.980528 | 1.587422 | 1.513679 | 1.631595 | 1.749112 | 1.866414 | 1.983530 | 2.100460 | 2.217207 |
| 0.090 | 0.045311 | 0.971655 | 1.093635 | 1.780830 | 1.623187 | 1.740731 | 1.858058 | 1.975201 | 2.092162 | 2.208945 |
| 0.120 | 0.045103 | 0.959491 | 1.081946 | 1.201056 | 1.536454 | 1.729297 | 1.846655 | 1.963835 | 2.080839 | 2.197669 |
| 0.150 | 0.044851 | 0.944270 | 1.067387 | 1.186712 | 1.304809 | 1.516949 | 1.832480 | 1.949706 | 2.066762 | 2.183651 |

## Split RMSE Versus B/C

| dataset_split | count | baseline_rmse_nm | force_domain_corrected_rmse_nm | variant_b_corrected_rmse_nm | variant_c_corrected_rmse_nm | force_domain_minus_b_rmse_nm | force_domain_minus_c_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| aux_downweighted_validation | 14448 | 0.05126637392308135 | 0.048819706958314515 | 0.048945040795565886 | 0.028529439179585198 | -0.0001253338372513707 | 0.020290267778729318 |
| diag_validation_only | 11108 | 0.0846209251204878 | 0.08441017836709906 | 0.08441199698509007 | 0.03876675590868194 | -1.8186179910062439e-06 | 0.04564342245841712 |
| open_floor_fit_downweighted | 31165 | 0.043194073119807955 | 0.04150718685740355 | 0.041880095058734225 | 0.033092937468878494 | -0.00037290820133067254 | 0.008414249388525058 |
| open_floor_validation_only | 14542 | 0.01693070910006944 | 0.01139376781450519 | 0.011371864567424441 | 0.014417143137851248 | 2.1903247080748423e-05 | -0.0030233753233460583 |
| primary_open_floor_fit_authoritative | 47317 | 0.036866176895009706 | 0.027907821180421354 | 0.028527902105821207 | 0.024120243843366647 | -0.0006200809253998525 | 0.0037875773370547074 |

## Selected-Log RMSE Versus B/C

| run_id | dataset_split | count | baseline_rmse_nm | force_domain_corrected_rmse_nm | variant_b_corrected_rmse_nm | variant_c_corrected_rmse_nm | force_domain_minus_b_rmse_nm | force_domain_minus_c_rmse_nm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-05-04_20-35-47 | open_floor_fit_downweighted | 3456 | 0.035845955637305665 | 0.02095614161732105 | 0.021128580068950017 | 0.028076443199331575 | -0.00017243845162896732 | -0.007120301582010526 |
| 2026-05-04_16-57-53 | open_floor_validation_only | 1761 | 0.023277853453219116 | 0.015366476488899375 | 0.015289065666438393 | 0.01934116142597289 | 7.741082246098124e-05 | -0.003974684937073517 |
| 2026-04-22_12-10-34 | open_floor_fit_downweighted | 2187 | 0.016217132966177106 | 0.01348701867968651 | 0.013498658585659703 | 0.014402919863369421 | -1.1639905973193088e-05 | -0.0009159011836829111 |
| 2026-04-22_01-06-32 | primary_open_floor_fit_authoritative | 1031 | 0.04497527944035794 | 0.033937937440801695 | 0.03404458889504844 | 0.018608488843215733 | -0.00010665145424674366 | 0.015329448597585962 |
| 2026-04-21_05-32-06 | primary_open_floor_fit_authoritative | 8880 | 0.04258417382836224 | 0.027283610538090263 | 0.02783508710487629 | 0.023754464601601764 | -0.0005514765667860265 | 0.0035291459364884988 |
| 2026-04-21_00-16-10 | primary_open_floor_fit_authoritative | 3757 | 0.039823975586254654 | 0.03430247806398148 | 0.03514600677652352 | 0.02774307957781035 | -0.0008435287125420424 | 0.006559398486171129 |
| 2026-04-20_12-10-58 | primary_open_floor_fit_authoritative | 2925 | 0.04035482301988178 | 0.0356483649477145 | 0.036503583837767924 | 0.02933811998534102 | -0.0008552188900534247 | 0.006310244962373479 |
| 2026-04-20_08-38-39 | open_floor_fit_downweighted | 7284 | 0.05622535775152191 | 0.056141320109295866 | 0.05625959776146966 | 0.04131229724660652 | -0.0001182776521737941 | 0.014829022862689348 |
| diag003 | diag_validation_only | 5580 | 0.08523812922205401 | 0.08503230164347471 | 0.08503396826948129 | 0.03897045903766876 | -1.6666260065772986e-06 | 0.046061842605805946 |

## No-Command-Conditioning Argument

The selected law contains no left/right command, unit command, requested command scalar, pre-projection force request, or mode-local command branch. Its activation inputs are contact-relative speed, normal load, and the projected/actual contact force vector reduced to a yaw-moment utilization.

Therefore, if two cases have the same contact state `{v_rel_i, N_i, yawRate}` and the same projected tire/contact force state `{F_f,i, F_r,i}`, they produce the same `M_contact`, `M_yield`, `u`, Stribeck schedule, and `M_extra`, regardless of which upstream command representation happened to produce that force state. Command can still affect motor torque and thus the physical contact force state; it is not an independent traction selector.

The pre-projection request branch is rejected here even though the initial task allowed it as a possible analysis input. With the clarification that command gates are prohibited, projected/actual contact force is the only acceptable selector among the two fitted branches.

## Failure Modes

- The projected-force form is solve-order sensitive: it must run after, or be coupled with, contact projection. Evaluating it from pre-projection demand would reintroduce the prohibited command/request gate.
- The law inherits Variant B's one-sided positive contact-force branch. A production form should make the sign convention explicit and symmetric rather than depending on a logged feature name.
- The 6x10 grid is an algebraic command estimate, not a full force replay. High-yaw cells can require commands outside `[-1, 1]`, so those cells are feasibility warnings, not validated reachable behavior.
- The model fits low-speed yaw scrub and intentionally fades with forward/contact speed. It is not a replacement for the broader Variant C combined-slip surface, which still has better validation and diagnostic split RMSE.
- The selected fit is target-aware after the hard physical gate: among projected-force candidates that pass `|cmd| >= 0.6`, it prefers candidates within `0.020` command of the `0.646` reference before minimizing RMSE. Production should still pick the yield owner deliberately rather than baking this analysis helper into runtime.

## Reproduce

```powershell
& 'C:\Users\thene\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' codex_analysis\yaw_model_variant_fits\round2_force_domain_stribeck\fit_force_domain_stribeck.py
```

## Output Files

- `fit_force_domain_stribeck.py`
- `force_domain_stribeck_report.md`
- `force_domain_coefficients.csv`
- `candidate_tuning_scores.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid_6x10.csv`
- `lr_delta_pivot.md`
- `split_rmse.csv`
- `selected_log_rmse.csv`
- `split_rmse_comparison.csv`
- `selected_log_rmse_comparison.csv`
- `reference_metadata.json`
- `commands_run.txt`
