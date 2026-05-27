# UKF Dependency Audit: Yaw Model Variant Fits

Scope: analysis-only audit of `codex_analysis/yaw_model_variant_fits`, the requested `round2_*` variants, prior B/C baseline artifacts reused by those variants, and the shared feature-generation path under `codex_analysis/contact_continuum_yaw_identification`.

Conclusion: I found no fitted input, target, or residual path that directly uses logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate columns. The requested variants all trace their fitted inputs and targets to the shared contact-continuum feature samples, whose generator derives `forward_velocity_mps`, `yaw_rate_radps`, contact features, and residual targets from encoder velocities, raw gyro, drive commands, timestamps, and PlantModel mirror constants. The result is not UKF-contaminated by the specific suspect state-vector path, but it still has non-UKF caveats: residual targets are gyro-differentiated single-sample yaw torque residuals, lateral velocity is assumed zero because no lateral sensor exists, and several variants reuse request/command-derived diagnostics or B/C baselines for comparison/calibration.

## Direct Search Evidence

Commands run:

```text
rg -n -i "ukf_state|\bukf\b|state_vector|estimator|xhat|kalman" codex_analysis\yaw_model_variant_fits
```

Result: only one hit, a report/script note in `round2_state_minimal_lugre` saying a future estimator would need matching state if estimator prediction used the proposed internal model state. No logged UKF/estimator columns were read there.

```text
codex_analysis\yaw_model_variant_fits\round2_state_minimal_lugre\round2_state_minimal_lugre_report.md:156:- A true production internal state still belongs in `PlantModel`, but it must be driven by contact slip, normal load, and actual tangential force history. If estimator prediction uses it, the estimator needs the same state or a deterministic mirror advanced with the same physical inputs.
codex_analysis\yaw_model_variant_fits\round2_state_minimal_lugre\fit_state_minimal_lugre.py:1344:            "- A true production internal state still belongs in `PlantModel`, but it must be driven by contact slip, normal load, and actual tangential force history. If estimator prediction uses it, the estimator needs the same state or a deterministic mirror advanced with the same physical inputs.",
```

```text
rg -n -i "ukf_state|\bukf\b|state_vector|xhat|kalman" codex_analysis\yaw_model_variant_fits codex_analysis\contact_continuum_yaw_identification
```

Result: no fitting-use hits. The hits are provenance notes stating UKF fields are not used, plus a design draft note:

```text
codex_analysis\contact_continuum_yaw_identification\features\extract_contact_continuum_features.py:1316:        "Targets use sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where stationary rows exist, encoder-derived forward velocity and wheel-bank speeds, logged drive commands, and timestamps. Logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.",
codex_analysis\contact_continuum_yaw_identification\features\contact_continuum_feature_report.md:15:Targets use sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where stationary rows exist, encoder-derived forward velocity and wheel-bank speeds, logged drive commands, and timestamps. Logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.
codex_analysis\contact_continuum_yaw_identification\design_draft\plantmodel_contact_continuum_yaw_design_draft.md:14:- Feature extraction used contact-continuum variables only. It reconstructed per-contact `v_rel_f`, `v_rel_r`, `vbar_rel`, `vbar_lat`, `vbar_yaw`, force-request, and projected-force features without using `ukf_state_*` targets.
codex_analysis\contact_continuum_yaw_identification\data_quality\data_quality_report.md:3:Analysis-only output. Production code was not modified. Trust predicates use sensor, command, schema, and metadata fields only; `ukf_state_*` fields are not used.
codex_analysis\contact_continuum_yaw_identification\data_quality\analyze_data_quality.py:650:    lines.append("Analysis-only output. Production code was not modified. Trust predicates use sensor, command, schema, and metadata fields only; `ukf_state_*` fields are not used.")
```

Generated CSV header check for the two shared feature samples and requested model coefficient files found no `ukf`, `estimator`, `state_vector`, `xhat`, or `kalman` header matches.

## Shared Feature Provenance

The shared feature extraction path is the critical provenance source for the round2 fits.

- Raw source normalization reads timestamps, drive commands, encoder velocities/wheel speeds, and raw gyro:
  - `extract_contact_continuum_features.py:595-609`: open-floor rows use `master_time_us`, `left_drive_command`, `right_drive_command`, `left_encoder_velocity_mps`, `right_encoder_velocity_mps`, `left_encoder_omega_radps`, `right_encoder_omega_radps`, and `gyro_raw_radps`.
  - `extract_contact_continuum_features.py:627-641`: competition rows use `t_us`, `left_drive_cmd`, `right_drive_cmd`, `left_velocity_mps`, `right_velocity_mps`, and `gyro_raw_radps`.
- Gyro bias is estimated from stationary/near-stationary sensor rows:
  - `extract_contact_continuum_features.py:684-691`.
- `yaw_rate_radps` is raw gyro minus that bias; `forward_velocity_mps` is encoder-bank average:
  - `extract_contact_continuum_features.py:1184-1186`.
  - `extract_contact_continuum_features.py:909-913`.
- The residual target is derived from gyro differentiation and the PlantModel mirror:
  - `extract_contact_continuum_features.py:1189-1195`: `measured_yaw_accel = (next_yaw_rate - yaw_rate) / dt`, `observed_moment = denom * measured_yaw_accel`, `residual = observed_moment - features.model_yaw_moment_nm`.
  - `extract_contact_continuum_features.py:964-966`: writes `observed_yaw_moment_nm`, `model_yaw_moment_nm`, and `residual_additive_yaw_torque_nm`.
- `residual_opposes_yaw_nm` is generated in the ablation pass from the additive residual and yaw sign:
  - `analyze_phase_ablation.py:135-144`: `TARGET = -raw_residual * yaw_sign`.

Contact primitives are reconstructed from the same sensor/command inputs:

- `compute_features(...)` takes `forward_velocity_mps` and `yaw_rate_radps` as arguments, computes bank torques from commands and wheel speeds, reconstructs contact-relative velocities from encoder wheel velocities plus yaw geometry, and projects requested forces:
  - `extract_contact_continuum_features.py:402-520`.
- Lateral/right body velocity is not measured and is explicitly set to zero:
  - `extract_contact_continuum_features.py:432` and `extract_contact_continuum_features.py:910-911`.

## Model Classification

| Model/artifact | Uses UKF as fitted input/target? | Evidence | Consequence |
|---|---:|---|---|
| `round2_force_domain_stribeck` | No | Reads `phase_classified_feature_sample.csv`, `contact_continuum_feature_sample.csv`, and constants at `fit_force_domain_stribeck.py:23-42`; fitted columns include `forward_velocity_mps`, `yaw_rate_radps`, contact features, and residuals at `fit_force_domain_stribeck.py:76-100`; `load_frame` reads those CSVs and numeric columns at `fit_force_domain_stribeck.py:155-192`; target is `residual_opposes_yaw_nm` at `fit_force_domain_stribeck.py:392`. Shared feature provenance is sensor/encoder/command-only as cited above. | Not invalid for UKF contamination. Caveat: one rejected diagnostic branch uses request-moment activation; selected projected-force form is still fitted to gyro-differentiated residuals. |
| `round2_request_contact_surface` | No | Imports prior C loader and grid helper at `fit_request_contact_surface.py:24-38` and `97-98`; main rows are `cc.load_rows()` at `fit_request_contact_surface.py:1140`; target is recomputed as `-sign_yaw * residual_additive_yaw_torque_nm` at `fit_request_contact_surface.py:165-166`; inputs use abs forward/yaw and contact primitive fields at `fit_request_contact_surface.py:189-198` and contact creep at `169-186`. The imported C loader reads the same shared feature samples. | Not invalid for UKF contamination. Caveat: it is a residual fit over reconstructed contact features and uses B/C artifacts for calibration/comparison. |
| `round2_hybrid_b_c` | No | Reads `phase_classified_feature_sample.csv`, `contact_continuum_feature_sample.csv`, and constants at `fit_round2_hybrid_b_c.py:24-44`; primary columns are listed at `70-93`; data load and target construction are at `243-280`, where `target_opposes_yaw_nm = -yaw_direction * residual_additive_yaw_torque_nm`. | Not invalid for UKF contamination. Caveat: hybrid uses B/C comparison references and reconstructed contact features. |
| `round2_hybrid_b_c/dynamic_bristle_lugre` | No | Script uses the same base module row loader and B/C comparison rows; it states the candidate does not use command/request as traction input in output metadata at `fit_dynamic_bristle_lugre.py:647`. The internal bristle state is evolved from contact/yaw history, not logged UKF state. | Not invalid for UKF contamination. Caveat: internal dynamic state is an analysis-model state and would need a production `PlantModel`/estimator mirror only if adopted. |
| `round2_static_yield_contact` | No | Reads shared primary/secondary feature CSVs and B/C coefficient/metric references at `fit_static_yield_contact.py:20-28`; primary columns include `forward_velocity_mps`, `yaw_rate_radps`, and residual columns at `74-96`; `load_rows` merges only those shared rows at `270-301`; target is `residual_opposes_yaw_nm` at `978`. | Not invalid for UKF contamination. Caveat: synthetic command grid is approximate contact replay; residual surface remains gyro-derived. |
| `round2_state_minimal_lugre` | No | Reads shared `phase_classified_feature_sample.csv` and constants at `fit_state_minimal_lugre.py:24-37`; input columns are `55-82`; loading/dropna uses `forward_velocity_mps`, `yaw_rate_radps`, `residual_additive_yaw_torque_nm`, and `residual_opposes_yaw_nm` at `179-216`; target is `residual_opposes_yaw_nm` at `436`. Only `estimator` hit is a caveat sentence about future production estimator consistency, not a fitted data dependency. | Not invalid for UKF contamination. Caveat: model name says "state", but that is a fitted contact/friction state derived from contact slip history, not logged UKF state. |
| `round2_b_correct_branch_reference` | No | Packet maker reads prior Variant B artifacts (`stribeck_scrub`, `lr_delta_grid`, `in_place_1radps_command`) and a contact sample summary at `make_reference_packet.py:8-15` and `97-116`; it does not fit from UKF columns. | Not invalid for UKF contamination. Caveat: this B reference is already rejected for production command-conditioning reasons, but not because of UKF data. |
| Prior Variant B: `stribeck_scrub` | No | Reads `phase_classified_feature_sample.csv` at `fit_stribeck_scrub.py:22-28`; input columns include `forward_velocity_mps`, `yaw_rate_radps`, `residual_additive_yaw_torque_nm`, and `residual_opposes_yaw_nm` at `49-67`; fitting reads and converts those columns at `639-681`, with target `y_opp = residual_opposes_yaw_nm`. | Safe as a UKF-free baseline, subject to its known command/request and residual-noise caveats. |
| Prior Variant C: `combined_slip_surface` | No | Reads shared primary/secondary feature samples and constants at `fit_combined_slip_surface.py:20-22`; target constants are `residual_opposes_yaw_nm` and `residual_additive_yaw_torque_nm` at `40-41`; loader merges shared feature rows and derives `sign_yaw`, `abs_forward_velocity_mps`, and `abs_yaw_rate_radps` at `252-282`. | Safe as a UKF-free baseline, subject to reconstructed-contact and gyro-derived residual caveats. |
| `lr_delta_grid` and `in_place_1radps_command` baseline helpers | No | Helpers read PlantModel mirror constants and prior fit coefficient artifacts, not log state vectors: `estimate_lr_delta_grid.py:16-39`; `estimate_in_place_1radps_command.py:16-33`. A direct UKF grep over those helper directories returned no matches. | Safe as UKF-free algebraic helpers. They are not new fits and should not be treated as measured validation. |

## Provenance of Specific Columns

| Column | Provenance | UKF dependency? | Caveats |
|---|---|---:|---|
| `forward_velocity_mps` | Encoder-bank average: `0.5 * (current.left_velocity_mps + current.right_velocity_mps)` in `extract_contact_continuum_features.py:1186` and written at `909`. Open-floor source is `left_encoder_velocity_mps`/`right_encoder_velocity_mps` at `605-606`; competition source is `left_velocity_mps`/`right_velocity_mps` at `620-638`. | No | Forward speed inherits encoder calibration/noise. |
| `yaw_rate_radps` | Raw gyro minus stationary sensor-derived bias: stationary bias rows are selected at `extract_contact_continuum_features.py:684-691`; yaw rate is `current.gyro_raw_radps - bias` at `1184` and written at `912`. | No | Bias estimate quality varies by log; not UKF. |
| `residual_additive_yaw_torque_nm` | `observed_moment - features.model_yaw_moment_nm`; observed moment comes from gyro yaw acceleration using timestamps at `extract_contact_continuum_features.py:1189-1195`; model moment is computed from commands, wheel speeds, normal loads, contact force requests/projection, and yaw damping in `compute_features` at `402-520`; written at `964-966`. | No | Differentiated gyro makes single-row residuals noisy; PlantModel mirror assumptions affect target. |
| `residual_opposes_yaw_nm` / residual target variants | `analyze_phase_ablation.py:135-144` computes `-raw_residual * sign(yaw_rate_radps)`. Some round2 scripts recompute equivalent target from `residual_additive_yaw_torque_nm` and yaw/contact sign, e.g. `fit_round2_hybrid_b_c.py:273-278` and `fit_request_contact_surface.py:165-166`. | No | Depends on yaw sign deadband/fallback and the additive residual above. |

## Invalidity Assessment

No model/result in the requested set is invalidated by direct or indirect dependence on old suspect UKF state-vector data.

If a future rerun discovers a hidden source CSV generated from `ukf_state_*`, estimator velocity, estimator yaw-rate, pose-derived velocity, or logged gyro-bias columns, any model fitted from that CSV should be marked invalid. A sensor/encoder-only rerun would need to regenerate the shared feature samples from raw gyro, encoder velocities/wheel speeds, drive commands, and timestamps; regenerate `phase_classified_feature_sample.csv`; then rerun B/C baseline artifacts before the round2 variants that consume them.

## Caveats

- The file name `micromouse_ukf_plant_measurement_noise_theory_only_spec.md` appears in provenance text, but the feature extraction note explicitly says logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.
- Lack of UKF contamination does not make the fits production-ready. The residuals are gyro-differentiated and PlantModel-mirror-derived, contact right/lateral velocity assumes `Vr=0`, and command/request-conditioned baselines remain architecturally suspect for reasons unrelated to UKF.
