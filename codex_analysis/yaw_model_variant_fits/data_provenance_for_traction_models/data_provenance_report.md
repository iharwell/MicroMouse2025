# Data Provenance For Traction/Yaw Model Test Sets

Analysis-only report. Production code, build metadata, and tests were not edited.

## Scope And Sources

This report describes the data behind the `yaw_model_variant_fits` traction/yaw model comparisons, with emphasis on:

- `standalone_contact_traction_testbed`
- `transition_options/rational_speed_force_blend`
- `round2_force_domain_stribeck`
- `combined_slip_surface` Variant C
- the shared `evaluation_contract.md`

Primary data source for the model comparison contract:

- `codex_analysis/contact_continuum_yaw_identification/ablation/phase_classified_feature_sample.csv`

Secondary source merged by the model scripts for contact/load details:

- `codex_analysis/contact_continuum_yaw_identification/features/contact_continuum_feature_sample.csv`

Supporting provenance and quality sources:

- `codex_analysis/contact_continuum_yaw_identification/features/contact_continuum_feature_report.md`
- `codex_analysis/contact_continuum_yaw_identification/features/contact_continuum_run_summary.csv`
- `codex_analysis/contact_continuum_yaw_identification/data_quality/data_quality_report.md`
- `codex_analysis/contact_continuum_yaw_identification/data_quality/data_quality_recommendations_by_run.csv`
- `codex_analysis/yaw_model_variant_fits/evaluation_contract.md`
- `codex_analysis/yaw_model_variant_fits/round2_ukf_dependency_audit/ukf_dependency_audit_report.md`
- `codex_analysis/vehicle_physical_parameter_characterization/vehicle_physical_parameter_characterization_report.md`
- `codex_analysis/yaw_launch_step_response/yaw_launch_step_response_report.md`
- `codex_analysis/full_impulse_response_characterization/step_kernels/yaw_launch_compact_identification_report.md`

Machine-readable outputs written with this report:

- `split_summary.csv`
- `split_run_provenance.csv`
- `model_split_influence.csv`
- `coefficient_training_run_influence.csv`
- `risk_slice_counts.csv`

## Shared Feature Provenance

The common feature set is sensor/encoder/command derived. The feature report states that targets use raw gyro yaw rate minus an independently estimated stationary bias, encoder-derived forward velocity and wheel-bank speeds, logged drive commands, and timestamps. It explicitly says logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.

Key fields used by these models:

| Field group | Provenance | Caveat |
| --- | --- | --- |
| `forward_velocity_mps` | Encoder-bank average from left/right encoder velocity fields. | Depends on encoder/wheel calibration and wheel radius assumptions. |
| `yaw_rate_radps` | Raw gyro minus per-run stationary bias estimated from sensor rows. | Bias quality varies by log. |
| `observed_yaw_moment_nm` | Gyro-derived yaw acceleration times mirrored yaw denominator. | Single-sample gyro differentiation is noisy. |
| `residual_additive_yaw_torque_nm` | `observed_yaw_moment_nm - model_yaw_moment_nm`. | Includes PlantModel mirror assumptions. |
| `residual_opposes_yaw_nm` | Additive residual projected against yaw sign. | Depends on yaw sign/deadband handling. |
| contact relative velocities | Reconstructed from encoder wheel speeds, forward speed, yaw rate, track/contact geometry. | No lateral body velocity sensor exists; right/lateral velocity assumes `Vr=0`. |
| normal loads | Static mass plus fan load, split by static load assumptions. | Load transfer is not reconstructed; legacy competition logs use metadata/default fan duty. |
| force/request bases | Mirrored PlantModel drive/contact request and projected force terms. | Useful for analysis, but command/request-conditioned paths are not production-clean. |

The prior UKF audit found no fitted input, target, or residual path in the requested variants that directly uses logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate columns. That makes the fits UKF-free for the specific suspected state-vector dependency, but not automatically production-ready.

## Split Inventory

Counts below are rows in `phase_classified_feature_sample.csv` after excluding `excluded_or_unclassified`. Full per-run provenance is in `split_run_provenance.csv`.

| Split | Rows | Runs/date span | Source runs | Role | Hardware/coverage notes | Weighting/authority |
| --- | ---: | --- | --- | --- | --- | --- |
| `primary_open_floor_fit_authoritative` | 47,317 | 15 runs, 2026-04-10 through 2026-04-22 | `2026-04-22_01-06-32`; `2026-04-21_23-39-12`; `2026-04-21_05-59-46`; `2026-04-21_05-32-06`; `2026-04-21_04-14-05`; `2026-04-21_02-46-49`; `2026-04-21_01-09-34`; `2026-04-21_00-16-10`; `2026-04-20_12-10-58`; `2026-04-14_05-26-35`; `2026-04-14_04-43-48`; `2026-04-13_16-42-46`; `2026-04-12_22-27-00`; `2026-04-12_20-14-49`; `2026-04-10_18-08-20` | Main fit-authoritative open-floor set. | Current/full open-floor schema according to data-quality recommendations. Broad exploratory open-floor yaw/forward coverage, but mostly older than May 4. | Authoritative for standalone and Variant C coefficient fits. Primary base weight for force-domain Stribeck. |
| `open_floor_fit_downweighted` | 31,165 | 9 runs, 2026-04-14 through 2026-05-04 | `2026-05-04_20-35-47`; `2026-04-22_12-10-34`; `2026-04-20_10-22-09`; `2026-04-20_08-38-39`; `2026-04-20_04-54-09`; `2026-04-20_02-33-07`; `2026-04-15_02-09-58`; `2026-04-14_16-34-17`; `2026-04-14_02-00-02` | Non-authoritative open-floor evidence. | Includes the newest yaw-launch/open-floor log `2026-05-04_20-35-47`, but that run is legacy/partial, lacks watchdog trust fields, and has angular command mostly zero during moving-yaw rows. Several older runs are downweighted for saturation or missing trust evidence. | Force-domain Stribeck uses these rows at 0.25 base weight if `fit_downweighted`; standalone and Variant C report them but do not fit coefficients from them. |
| `open_floor_validation_only` | 14,542 | 9 runs, 2026-04-10 through 2026-05-04 | `2026-05-04_16-57-53`; `2026-04-20_23-05-07`; `2026-04-16_02-43-36`; `2026-04-12_06-44-12`; `2026-04-12_06-36-32`; `2026-04-12_05-13-55`; `2026-04-11_21-03-20`; `2026-04-11_06-58-25`; `2026-04-10_18-33-52` | Held-out open-floor validation. | Includes latest `2026-05-04_16-57-53`, but it is legacy/partial with watchdog unavailable, angular command mostly zero during moving-yaw rows, final quiescent/invalid tail evidence, and low extracted-sample count. | Zero coefficient-fit authority. Used for validation/objective selection in Variant C and rational blend. |
| `diag_validation_only` | 11,108 | 3 legacy diag runs; dates not encoded in run IDs | `diag003`; `diag001`; `diag000` | Competition diagnostic validation. | Maze/competition-like yaw stress; legacy schema derives wheel omega from encoder velocity/current radius and lacks saturation, watchdog, and per-row fan duty. `diag003` is one of the selected logs. | Zero coefficient-fit authority. Useful validation for low-speed/high-yaw and maze-like regimes. |
| `aux_downweighted_validation` | 14,448 | 13 legacy aux runs; dates not encoded in run IDs | `aux012`; `aux011`; `aux010`; `aux009`; `aux008`; `aux007`; `aux006`; `aux005`; `aux004`; `aux003`; `aux002`; `aux001`; `aux000` | Auxiliary competition/path validation. | Path/procedure dependent stress coverage. Legacy schema lacks saturation/watchdog/per-row fan duty; fan comes from metadata/default. | Zero coefficient-fit authority in the requested variants. Reported separately as non-authoritative validation. |
| `validation_non_authoritative` | 71,263 | Aggregate, all non-primary splits | All `open_floor_fit_downweighted`, `open_floor_validation_only`, `diag_validation_only`, and `aux_downweighted_validation` rows. | Combined non-authoritative validation rollup. | Mixes newer incomplete open-floor logs with legacy diag/aux competition logs. | Reporting/objective rollup only; not a separate source file split. |

## Model-Specific Use Of Splits

| Model/testbed | Coefficient-fitting data | Validation/selection data | Consequence |
| --- | --- | --- | --- |
| Standalone contact traction testbed | Only `primary_open_floor_fit_authoritative`. Fit weights are run-balanced, with quality penalties for gyro spikes, hardware saturation, and near-zero yaw. | All other splits are reported separately; synthetic +1 rad/s launch gate affects candidate selection. | Selected constants are controlled by older April authoritative open-floor logs plus the synthetic launch gate. Latest May logs have zero coefficient influence. |
| Variant C combined-slip surface | Only `primary_open_floor_fit_authoritative`. Fit weights are run-balanced, with the same quality penalties as above. | Non-primary splits choose among the narrow candidate family through run-balanced validation objective and risk penalties. | Latest May logs can influence family selection slightly, but not fitted coefficients. |
| Force-domain Stribeck | `primary_open_floor_fit_authoritative` base weight 1.0 plus `open_floor_fit_downweighted` base weight 0.25 for open-floor `fit_downweighted` rows. Then quality penalties and `1/sqrt(run_count)` run balancing are applied. | Validation splits reported; selected projected-force candidate must pass in-place command gate near 0.646. | Latest `2026-05-04_20-35-47` has some direct fit influence, but only through the downweighted branch. |
| Rational speed/force blend | No new raw coefficient fit. It blends existing force-domain Stribeck and Variant C predictions. | Selects `speed_force_partition` by non-primary run-balanced validation RMSE after requiring in-place command >= 0.6. | Latest logs influence validation metrics and blend behavior, not a raw traction fit. |
| Shared evaluation contract | Defines selected logs and split policy. | Requires selected-log reporting, split reporting, and phase reporting. | Keeps latest/selected logs visible even when not authoritative. |

Measured effective coefficient-weight fractions from `coefficient_training_run_influence.csv`:

| Model | Primary fraction | Downweighted fraction | Latest May 4 fraction |
| --- | ---: | ---: | ---: |
| Standalone contact traction testbed | 100.0% | 0.0% | 0.0% |
| Variant C combined-slip surface | 100.0% | 0.0% | 0.0% |
| Force-domain Stribeck | 87.68% | 12.32% | 1.74% |

## Selected Logs

These are the selected logs named in `evaluation_contract.md` and used by the requested model reports.

| Run | Date/era | Split | Rows in shared sample | Quality/coverage note | Fit influence |
| --- | --- | --- | ---: | --- | --- |
| `2026-05-04_20-35-47` | Latest open-floor/yaw-launch era | `open_floor_fit_downweighted` | 3,456 | Legacy/partial open-floor schema; watchdog unavailable; angular command mostly zero during moving-yaw rows. Also contains the yaw-launch phase used by yaw-launch reports. | Zero for standalone and Variant C coefficients; small downweighted influence for force-domain Stribeck. |
| `2026-05-04_16-57-53` | Latest open-floor era | `open_floor_validation_only` | 1,761 | Legacy/partial open-floor schema; watchdog unavailable; angular command mostly zero; final quiescent/invalid tail detected; low extracted-sample count. | Validation only. |
| `2026-04-22_12-10-34` | Recent April, partial schema | `open_floor_fit_downweighted` | 2,187 | Watchdog unavailable. | Downweighted only in force-domain; validation/reporting for standalone and Variant C. |
| `2026-04-22_01-06-32` | Latest authoritative April | `primary_open_floor_fit_authoritative` | 1,031 | Full current open-floor quality recommendation. | Direct coefficient fit for standalone, Variant C, and force-domain. |
| `2026-04-21_05-32-06` | Older authoritative April | `primary_open_floor_fit_authoritative` | 8,880 | Full current open-floor quality recommendation. | Direct coefficient fit. |
| `2026-04-21_00-16-10` | Older authoritative April | `primary_open_floor_fit_authoritative` | 3,757 | Full current open-floor quality recommendation. | Direct coefficient fit. |
| `2026-04-20_12-10-58` | Older authoritative April | `primary_open_floor_fit_authoritative` | 2,925 | Full current open-floor quality recommendation. | Direct coefficient fit. |
| `2026-04-20_08-38-39` | Older April, saturated | `open_floor_fit_downweighted` | 7,284 | Downweighted for high moving-yaw saturation fraction. | Downweighted only in force-domain; validation/reporting for standalone and Variant C. |
| `diag003` | Legacy competition diagnostic | `diag_validation_only` | 5,580 | Legacy competition schema; no saturation/watchdog/per-row fan duty. | Validation only. |

The selected latest logs are therefore visible in the reports, but they are not authoritative for the standalone constants. `2026-05-04_20-35-47` is the only latest log with any coefficient influence in the requested set, and only in the force-domain Stribeck fit.

## Risk Slices

The standalone report defines these risk slices over the full shared sampled data:

| Risk slice | Rows | What it tests |
| --- | ---: | --- |
| `high_speed_abs_vf_ge_0p7` | 144 | Sparse high-forward-speed coverage. |
| `low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5` | 20,694 | Low-speed yaw/turning behavior, closest to scrub and launch-like dynamics in the broad dataset. |
| `limiter_active` | 31,216 | Rows where contact force projection/limiting was active. |
| `hardware_saturation_evidence` | 5,017 | Open-floor rows with hardware saturation evidence. |
| `open_floor_all` | 93,024 | All open-floor rows in the shared sample. |
| `diag_all` | 11,108 | All competition diagnostic rows. |
| `aux_all` | 14,448 | All competition auxiliary rows. |

The high-speed slice is extremely small. The low-speed yaw slice is well represented, but it is not the same as the dedicated May 4 yaw-launch step protocol.

## Launch Reference And Yaw-Launch Data

The launch-specific reference is not part of the broad coefficient training split. It is used in these ways:

- The standalone contact traction testbed uses a synthetic `Vf=0`, `yawRate=+1 rad/s` launch gate. The selected 84.635 mm effective-track constants produce +/-0.618742 command; the measured-width rerun at 73.16 mm produces +/-0.647586 command.
- Force-domain Stribeck selects only projected-force candidates that pass a hard in-place command gate and then prefers candidates near the prior `0.646` command reference.
- The rational speed/force blend selects a transition that keeps the in-place command above 0.6 while minimizing non-primary validation RMSE.

Dedicated yaw-launch evidence comes from `2026-05-04_20-35-47`:

- `yaw_launch_step_response_report.md` reads `TestResults/mmlog_decode_2026-05-04_20-35-47/open_floor_main.csv`, phase 20 `Yaw Launch`.
- It reports 79,541 yaw-launch rows over 79.540 s and 100 command steps.
- Commands +/-0.50 and +/-0.55 twitch but do not sustain yaw; +/-0.60 is transitional; +/-0.65 is the practical launch boundary; +/-0.70 sustains.
- The compact launch report records fan duty metadata 0.800, raw gyro minus independent static bias, no UKF targets, and a train/validation split over repeats 3, 7, and 10.

That launch data is recent and important, but it is narrower than the broad contact-continuum dataset. It covers in-place/near-zero-forward yaw-launch behavior, not the full forward-speed, arc, fan/load, or maze coverage needed for a general traction surface.

## Hardware And Era Notes Found

Known or inferred from analysis artifacts:

| Topic | Evidence found | Impact on provenance |
| --- | --- | --- |
| Effective track width | Current mirrored constant is 84.635 mm. Vehicle audit says this is a log-derived effective kinematic value, not pure physical tire span. | Broad feature extraction and most fits use 84.635 mm unless rerun explicitly. |
| Physical/wear-pattern track | Vehicle audit lists physical contact span bounds 70.04..78.68 mm. Standalone rerun explicitly tests 73.16 mm measured wear-pattern track. | The 73.16 mm rerun changes drive scale and dynamic mu but broad validation RMSE is essentially neutral; launch command aligns better with the 0.646 target. |
| Fan/load | Feature report uses per-row `fan_duty_cycle` when present, competition metadata when present, otherwise 0.8. Data-quality report says competition diag/aux lack per-row fan duty. | Normal-load evidence is weaker in legacy competition rows and any fan/load extrapolation is risky. |
| Saturation/watchdog | Current open-floor logs usually expose these fields; legacy competition and some partial open-floor logs do not. | Missing saturation/watchdog is why diag/aux are validation-only and why the May 4 latest logs are not authoritative. |
| Sensor setup | Targets are raw gyro minus independent bias plus encoders and drive commands. | No UKF state-vector dependency, but gyro differentiation noise remains. |
| Lateral velocity | No independent lateral sensor exists; `Vr=0` is a feature-extractor assumption. | Right-relative contact features are reconstructed, not measured lateral truth. |
| Yaw-launch measurement | May 4 yaw-launch data identifies +4 ms first gyro response, +5 ms derivative/onset sanity, static/bristle twitch at low commands, sign-asymmetric breakaway near 0.610/0.650. | Useful for launch/contact dynamics, but not broad enough to dominate a general traction fit by itself. |
| Command calibration | The hard practical launch target is about +/-0.646 command. | Used as a synthetic/selection gate, not a measured row target in the broad fit. |
| Physical constants | Vehicle audit rejects dynamic yaw fits as replacements for Vehicle mass/track/yaw inertia. | Do not interpret these model fits as measuring construction facts. |

## Does This Favor The Latest Logs Enough?

Short answer: no, not for the standalone selected constants; partially, but weakly, for force-domain Stribeck and rational blend.

Evidence:

- The latest logs are `2026-05-04_20-35-47` and `2026-05-04_16-57-53`.
- They account for 3,456 rows in `open_floor_fit_downweighted` and 1,761 rows in `open_floor_validation_only`.
- They account for 0 rows in `primary_open_floor_fit_authoritative`.
- Standalone contact traction coefficients are fitted only on `primary_open_floor_fit_authoritative`, so latest logs have 0.0% coefficient influence.
- Variant C coefficients are also fitted only on `primary_open_floor_fit_authoritative`, so latest logs have 0.0% coefficient influence there too.
- Force-domain Stribeck includes `open_floor_fit_downweighted` at 0.25 base weight. After quality and run balancing, May 4 rows contribute about 1.74% of effective coefficient weight.
- Rational speed/force blend does not refit raw coefficients. It can favor latest behavior only through validation metrics and the synthetic launch gate, not through direct coefficient estimation.

Why the latest logs were not authoritative:

- `2026-05-04_20-35-47`: watchdog unavailable; angular command mostly zero during moving-yaw rows; legacy/partial schema. It also carries the important yaw-launch phase, but the broad contact-continuum sample has only 3,456 selected rows from it.
- `2026-05-04_16-57-53`: watchdog unavailable; angular command mostly zero; final quiescent/invalid tail detected; low extracted-sample count.

What the latest logs cover:

- Recent open-floor/yaw-launch behavior.
- Dedicated in-place yaw-launch command-step behavior in `2026-05-04_20-35-47`, including twitch, transition, and sustained launch thresholds.
- Some broad sampled contact-continuum rows in downweighted/validation splits.

What they miss or do not support cleanly:

- Full authoritative open-floor trust fields.
- Broad high-speed forward coverage.
- Complete saturation/watchdog/per-row trust evidence for fitting authority.
- Independent lateral velocity.
- A full fan/load matrix.
- Enough complete rows to override the older authoritative open-floor fit without a deliberate recency policy.

Recommended weighting changes if the goal is to favor latest hardware:

1. Do not simply promote the May 4 rows to `primary_open_floor_fit_authoritative`; their missing trust fields and low/partial coverage are real.
2. Add a recency-aware objective tier: keep coefficients fitted on trustworthy primary rows, but add a secondary selection term requiring no regression on `2026-05-04_20-35-47`, `2026-05-04_16-57-53`, and yaw-launch step metrics.
3. For the standalone testbed, rerun candidates with an explicit `latest_open_floor_downweighted` coefficient branch, capped by run-balanced weight. A practical starting point is 20-35% total effective fit weight reserved for latest rows, with per-row quality penalties retained.
4. Keep yaw-launch as a separate launch/reference objective, not as generic yaw-surface rows. It should constrain static/breakaway/low-speed behavior without dominating moving open-floor behavior.
5. Prefer collecting a current complete-schema rerun over weight hacking. The needed rerun should include watchdog/saturation/fan duty, raw gyro, encoders/wheel speeds, drive commands, battery voltage, phase labels, and explicit floor/tire/fan/skirt notes.
6. If the robot hardware has changed materially, treat older April authoritative rows as previous-era evidence. Use them for regularization and broad shape, but require the latest complete-schema logs to own final coefficients once available.

## Is The Standalone Model Dominated By Older Logs, Latest Logs, Diag Logs, Or Balanced?

The standalone selected constants are dominated by older April open-floor authoritative logs. They are not influenced by the latest May logs, diag logs, or aux logs during coefficient fitting.

Details:

- Coefficient fit split: only `primary_open_floor_fit_authoritative`.
- Date span: 2026-04-10 through 2026-04-22.
- Effective latest May 4 coefficient weight: 0.0%.
- Effective diag/aux coefficient weight: 0.0%.
- Within the primary split, the fit uses per-run balancing plus quality penalties, so it is not raw-row-count dominated by the largest April run. But it is still an older-authoritative-era fit because the only eligible runs are April 10-22.
- The launch gate pulls the selected family toward the recent observed launch command target, but the broad fitted coefficients are still from older primary open-floor rows.

The measured-width rerun at 73.16 mm is important for hardware-era interpretation: it shows the selected family survives the narrower measured wear-pattern track and moves the launch command estimate from +/-0.618742 to +/-0.647586 with nearly unchanged broad validation RMSE. That is a geometry sensitivity check, not a change in which logs own the fit.

## Bottom Line

The current evaluation contract is good at keeping latest and legacy stress logs visible, but the selected standalone constants are not latest-log-favoring. They are an older April open-floor fit constrained by synthetic launch behavior and then validated against newer/legacy splits. If the current hardware differs from April, the right next step is a complete current-schema data pass, not silent promotion of incomplete May logs.
