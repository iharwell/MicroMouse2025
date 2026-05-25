# Contact Correction Impact Investigation

Scratch investigation only. Production code and tests were inspected but not modified.

## Scope

Primary theory reference: `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`.

Primary implementation inspected:

- `MazeMap/MazeMap/PlantModel.h`
- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMapTest/PlantModelDynamicsTest.cpp`

Primary analysis artifacts inspected:

- `codex_analysis/contact_correction_tuning/alignment_tuning/alignment_tuning_report.md`
- `codex_analysis/contact_correction_tuning/alignment_tuning_exact_neg_6p496619/alignment_tuning_report.md`
- `codex_analysis/contact_correction_tuning/alignment_tuning_exact_neg_6p496619/*.csv`
- `codex_analysis/contact_correction_tuning/production_replay/evaluation_report.md`
- prior contact-continuum feature, ablation, data-quality, design-draft, and log-eval reports.

## Executive Finding

The tuned correction was underwhelming for open-floor in-place yaw because it is a single symmetric patch-force scalar that mostly reduces correlated yaw-rate prediction error in moving-yaw and mostly-forward samples. It is aligned with the theory spec at the implementation level, but it is not rich enough to model the dominant in-place yaw resistance terms after timing/filter delay is accounted for.

The +2 sample alignment materially changes the interpretation. With the LSM6DSV16X gyro LPF around 214 Hz and a 1 kHz control tick, the first-order phase-delay scale is about:

```text
1 / (2*pi*214 Hz) ~= 0.000744 s
```

That makes a measured +1 to +2 sample effective lag plausible once sensor filtering, command/effective-time convention, and sample pairing are combined. The alignment alone reduced open-floor in-place old RMSE from `0.205513948` at lag 0 to `0.184035414` at lag +2, about a 10.5% baseline improvement before the tuned contact coefficient is applied. That means prior residuals were partly sensor/effective-time residuals, not purely missing contact physics.

## Theory Alignment

The current production mechanism is mostly consistent with the theory spec:

- It uses per-contact relative velocities and signed contact locations.
- It applies a continuous patch-force correction before force projection.
- It avoids slip ratio, slip angle, curvature/radius, maneuver branches, and command rejection.
- The coefficient is owned by `PlantModel`, the correct owner for shared plant equations.

Relevant inspected implementation:

- `PlantModel.h` declares `kContactYawPatchForceGainNsPerM = -6.496619190f`.
- `PlantModel.cpp` computes contact-relative velocities in `wheelKinematics`.
- `PlantModel.cpp` computes a load-weighted patch yaw velocity, scales it by the coefficient, and distributes a force couple before force projection.
- `PlantModelDynamicsTest.cpp` has finite/continuous low-speed tests and a direct patch-force-couple test.

The weak part is not spec compliance. The weak part is model expressiveness: one load-weighted scalar cannot distinguish multiple physical causes that can have similar sign in the fit data.

## Tuning Impact

The exact +2 aligned replay with the selected gain shows real aggregate improvement:

| Dataset / split | Old RMSE | Tuned RMSE | Relative delta |
| --- | ---: | ---: | ---: |
| fit_authoritative_open_floor | 0.144705675 | 0.118536295 | -18.085% |
| open_floor_only | 0.139536353 | 0.120279694 | -13.800% |
| competition_stress | 0.360934074 | 0.313252733 | -13.211% |
| validation_only_open_floor | 0.068998992 | 0.070181036 | +1.713% |

But the open-floor motion split shows the fit is not evenly explaining all motion families:

| Open-floor +2 motion class | Samples | Old RMSE | Tuned RMSE | Relative delta |
| --- | ---: | ---: | ---: | ---: |
| in_place_yaw | 300727 | 0.184035414 | 0.176610275 | -4.035% |
| moving_yaw | 494414 | 0.187571543 | 0.158401369 | -15.551% |
| mostly_forward | 1042144 | 0.100065878 | 0.073314861 | -26.733% |
| low_motion_commanded | 515237 | 0.122952591 | 0.113585798 | -7.618% |

The production replay without the +2 target shift is more modest still:

| Production replay split | Old RMSE | New RMSE | Relative delta |
| --- | ---: | ---: | ---: |
| fit_authoritative_open_floor | 0.150058239 | 0.142405161 | -5.100% |
| open_floor_only | 0.148593814 | 0.142974849 | -3.781% |
| open_floor in_place_yaw | 0.205513948 | 0.201559037 | -1.924% |
| open_floor moving_yaw | 0.206063378 | 0.197974598 | -3.925% |
| open_floor mostly_forward | 0.099300462 | 0.092748045 | -6.599% |

This split is the core reason the result felt underwhelming: the aligned/tuned scalar looks much better in aggregate than it does in the specific open-floor in-place yaw regime.

## Why In-Place Yaw Benefits Less

1. The selected scalar is dominated by sample population and broad SSE reduction.

   Open-floor mostly-forward has about 3.5x the in-place sample count at +2. Mostly-forward also sees the largest relative improvement. The global fit therefore rewards a coefficient that fixes residuals correlated with the patch scalar across moving and mostly-forward samples.

2. In-place yaw residuals become less plant-like after alignment.

   The +2 alignment reduced open-floor in-place old RMSE by about 10.5% before tuning. That is larger than the coefficient's additional in-place benefit at +2. The LPF/effective-time effect consumed a large part of what previously looked like contact residual.

3. The installed feature has weak in-place specificity.

   In a reconstructed in-place rolling condition with `Vr=0`, the scalar is driven mostly by the forward contact offset term. That offset is only about 14.75 mm, while the effective half-track is about 42.3 mm. The scalar correction therefore has limited independent leverage on pure in-place scrub unless there is forward slip, lateral velocity, load shift, or force-envelope nonlinearity for it to use.

4. Existing lateral contact behavior already handles much of high-rate in-place resistance.

   The focused tests show high-rate in-place slip can already hit the sustained lateral force window. Since the new correction is applied before projection, extra pre-projection force can be clipped away when the force envelope is active.

5. The current logs do not independently measure lateral velocity.

   Prior feature extraction explicitly assumed `Vr=0`. That is acceptable for scratch reconstruction, but in-place contact scrub and chassis compliance can create lateral body motion or stored tire deflection that the current feature basis cannot see.

## Off-Distribution Interpretation

The off-distribution sets should remain excluded from coefficient fitting.

Evidence:

- `validation_only_open_floor` old RMSE is about 52% lower than the fit-authoritative baseline.
- `competition_stress` old RMSE is about 149% higher than the fit-authoritative baseline.
- The selected exact +2 gain slightly worsens validation-only open-floor while improving competition stress.
- Legacy competition data lacks saturation/watchdog/per-row fan fields, so it is physically useful but not fit-authoritative.

These data sets are not useless. They say the physical regime changes substantially across procedures, schemas, saturation availability, fan/load assumptions, and path dependence. They should be used as stress checks and feature-discovery evidence, not as direct coefficient authority.

## Likely Missing Basis Terms

The next in-place model should stay within the theory spec and avoid maneuver branches/tables. The most plausible missing terms are:

- Effective-time/timing covariance: make gyro LPF phase and command/effective-time semantics explicit before fitting acceleration-like residuals.
- Lateral velocity or a lateral-motion proxy: current fitting assumes `Vr=0`.
- Contact bristle/static deflection memory: in-place scrub is likely history-dependent, not instantaneously determined by contact-relative velocity alone.
- Fan/normal-load dependence and load transfer: current reconstruction uses static plus fan load and no transfer.
- Force-envelope/saturation shape: pre-projection correction may be projected away in high-contact rows.
- Low-speed regularization shape: in-place behavior may need a continuous low-speed contact-memory term, not a branch.
- Symmetric CW/CCW and left/right imbalance terms: only if held-out data shows sign/asymmetry structure after timing is corrected.
- Encoder/wheel spin-up denominator validation: current yaw denominator includes wheel spin-up; keep validating it because denominator error can masquerade as yaw torque error.

## Recommendation

Do not retune the production scalar again as the next step. The next useful step is targeted research data and replay instrumentation:

1. Characterize effective command-to-gyro timing with the actual 214 Hz LPF path. Use a targeted yaw step/chirp or repeated opposed-command sweep and fit the effective delay separately from contact coefficients.
2. Add scratch replay diagnostics that report correction before projection, correction after projection, utilization/saturation bins, and per-motion-class optimum gains.
3. Collect targeted open-floor in-place yaw sweeps with both yaw signs, multiple fan duties, clean saturation/watchdog fields, trimmed tails, and, if possible, an independent lateral-velocity reference.
4. In scratch, test a spec-compatible contact basis with one memory term or lateral-velocity proxy before any production change.

Production policy recommendation: keep the current coefficient only if aggregate/moving-yaw benefit is desired and the small in-place improvement is acceptable. Do not claim it solves in-place yaw resistance. The next production implementation, if any, should remain inside `PlantModel` and replace/refine the current contact-force basis rather than adding a maneuver branch, lookup table, wrapper, or second owner.

## Commands Run

Representative commands used for this read-only investigation:

```powershell
Get-Content -LiteralPath AGENTS.md
Get-Content -LiteralPath micromouse_ukf_plant_measurement_noise_theory_only_spec.md
Get-Content -LiteralPath codex_analysis\contact_correction_tuning\alignment_tuning\alignment_tuning_report.md
Get-Content -LiteralPath codex_analysis\contact_correction_tuning\alignment_tuning_exact_neg_6p496619\alignment_tuning_report.md
Get-Content -LiteralPath codex_analysis\contact_continuum_yaw_identification\ablation\phase_ablation_decision_report.md
Get-Content -LiteralPath codex_analysis\contact_continuum_yaw_identification\features\contact_continuum_feature_report.md
Get-Content -LiteralPath codex_analysis\contact_continuum_yaw_identification\data_quality\data_quality_report.md
Get-Content -LiteralPath codex_analysis\contact_continuum_yaw_identification\design_draft\plantmodel_contact_continuum_yaw_design_draft.md
Get-Content -LiteralPath codex_analysis\contact_correction_log_eval\evaluation_report.md
Get-Content -LiteralPath codex_analysis\contact_correction_tuning\production_replay\evaluation_report.md
Import-Csv -LiteralPath codex_analysis\contact_correction_tuning\alignment_tuning_exact_neg_6p496619\selected_motion_by_alignment.csv
Import-Csv -LiteralPath codex_analysis\contact_correction_tuning\alignment_tuning_exact_neg_6p496619\gain_sweep_by_alignment.csv
Import-Csv -LiteralPath codex_analysis\contact_correction_tuning\production_replay\family_motion_rmse.csv
rg -n "ContactYaw|Patch|kContact|wheelKinematics|evaluateAppliedBankTorqueStep" MazeMap\MazeMap\PlantModel.h MazeMap\MazeMap\PlantModel.cpp
rg -n "Contact|Yaw|LowForward|finite|continuous|Patch" MazeMap\MazeMapTest\PlantModelDynamicsTest.cpp
rg -n "GetBackLeftImuRuntimeGyroLpfCutoffHz|CUT_213|GYRO_LPF1" MazeMap\MazeMap
git status --short
```

No unit tests were run because this task was read-only production inspection plus scratch reporting; no production code or test behavior was changed.
