# Yaw Physics Research Report

Scratch research only. No production code was modified.

## Scope and Sources

This report interprets the expanded yaw-torque residual data as a physics problem, not as a production tuning change. The local data sources used were:

- `codex_analysis/yaw_torque_reconciliation/reconciled_yaw_torque_findings.md`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_report.md`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_surface_signed_bins.csv`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_surface_abs_summary.csv`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_coverage_by_forward.csv`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_dataset_comparison.csv`
- `codex_analysis/yaw_torque_expanded/expanded_yaw_torque_holdout_rmse.csv`
- `codex_analysis/yaw_torque_expanded_validation/expanded_yaw_torque_validation_report.md`
- `codex_analysis/yaw_torque_expanded_validation/nonzero_vf_torque_bins.csv`
- `codex_analysis/yaw_torque_expanded_validation/bin_run_consistency.csv`
- `codex_analysis/yaw_torque_expanded_validation/competition_bin_contribution.csv`
- `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`
- `MazeMap/MazeMap/PlantModel.h`
- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMap/Vehicle.h`
- `MazeMap/MazeMap/MotorEncoderDrive.h`

The root theory spec is the authority for model posture. In particular, it makes per-contact relative velocity the primary contact primitive and rejects speed-normalized slip ratio, slip angle, turn radius, curvature, maneuver-class branches, and zero-speed-singular contact math as plant semantics.

Outside references were used only as background sanity checks, not as authority over the repo spec:

- Tire lateral dynamics, slip angle, relaxation length, and cornering stiffness: [Sensors 2022, "Modeling of the Influence of Operational Parameters on Tire Lateral Dynamics"](https://www.mdpi.com/1424-8220/22/17/6380/html).
- Combined-slip tire force limits and why a simple friction ellipse is incomplete: [SAE 2011-01-0094 abstract](https://saemobilus.sae.org/papers/tire-force-ellipse-friction-ellipse-tire-characteristics-2011-01-0094).
- Contact patch force components and aligning torque context: [x-engineer tire force overview](https://x-engineer.org/automotive-engineering/chassis/vehicle-dynamics/tire-model-for-longitudinal-forces/).
- Load transfer as a tire normal-load input: [PMC tire-road friction estimator review](https://pmc.ncbi.nlm.nih.gov/articles/PMC5298280/).

## Sign Convention and Residual Meaning

Coordinates follow the project convention: `+X = right`, `+Y = forward/up`, and `+Yaw = clockwise`.

The reconciled extraction convention is:

```text
I_eff = I_z + m_wheel_spinup * (track_width / 2)^2
M_obs = I_eff * d(yaw_rate_sensor) / dt
M_res = M_obs - M_model
M_opp = -sign(yaw_rate_sensor) * M_res
```

`M_res` is the additive yaw torque that would be added to the current yaw-relevant PlantModel mirror for one sample. If `yaw_rate > 0` and `M_res > 0`, the model needs extra clockwise torque, so the residual is yaw-aiding and `M_opp < 0`. If `yaw_rate > 0` and `M_res < 0`, the model needs extra counter-clockwise torque, so the residual resists yaw and `M_opp > 0`.

Therefore negative opposing torque is not "negative friction" by itself. It means the current model, in that sample population, predicted too much counter-yaw effect or too little yaw-aiding effect after the existing motor, rolling/static loss, longitudinal tire stiffness, lateral contact force gains, normal/fan load, and contact saturation projection were mirrored.

The current PlantModel already models contact-relative-velocity force requests, drive force, combined force projection, contact-placement yaw moment, and wheel-spinup contribution to yaw inertia. That matters: the residual is what remains after the current mean model, not the whole tire torque. The recent history also matters: commit `b3031ff` removed the `PlantParams` middleman and explicitly marked old parameter-bag/proxy concepts as unsanctioned; later cleanup continued moving toward the root spec's contact-continuum posture.

## What the Data Says

The expanded run reported 2,382,049 extracted samples across open-floor and competition families. Nonzero-forward coverage is real but uneven: `|Vf|` coverage is strong through about `0.6 m/s`, thin at `0.7 m/s`, and sparse above `0.8 m/s`. The all-included deterministic holdout improved one-step yaw-rate RMSE from `0.272693` to `0.248282 rad/s`; open-floor-only improved from `0.258347` to `0.237744 rad/s`; competition-only improved from `0.308298` to `0.258216 rad/s`. The independent validation also improved aggregate leave-one-run-out RMSE from `0.302029` to `0.263993 rad/s`, but individual runs and competition subsets were mixed.

The nonzero-forward validation bins are dominated by high spread:

```text
407 signed nonzero-forward bins
23 bins pass a strict rough trust screen:
  count >= 500, run_count_ge10 >= 10, IQR <= 0.08 Nm, run spread <= 0.08 Nm
70 high-population bins still have IQR > 0.12 Nm or run spread > 0.12 Nm
```

Weighted by bin counts in `nonzero_vf_torque_bins.csv`, the median opposing torque is not monotonic with `|Vf|`:

| Abs Vf bin m/s | Bins | Samples | Count-weighted median opposing Nm | Count-weighted abs median opposing Nm |
| ---: | ---: | ---: | ---: | ---: |
| 0.1 | 123 | 164357 | -0.0289 | 0.0355 |
| 0.2 | 61 | 58862 | -0.0125 | 0.0190 |
| 0.3 | 82 | 73149 | -0.0339 | 0.0380 |
| 0.4 | 66 | 98337 | -0.0102 | 0.0120 |
| 0.5 | 47 | 21300 | -0.0048 | 0.0063 |
| 0.6 | 16 | 6388 | +0.0038 | 0.0049 |
| 0.7 | 7 | 1963 | +0.0022 | 0.0022 |
| 0.8 | 5 | 505 | +0.0155 | 0.0174 |

Interpretation: the robust nonzero-forward residuals are usually hundredths of a Nm, often near zero, and often yaw-aiding. The largest apparent structure lives in low-speed transition bins and high-yaw/low-forward bins, where bin leakage and transient effects are hardest to separate from real tire behavior.

## Likely Physical Mechanisms

### 1. Near-zero `Vf`: pivot scrub and stick-slip dominate

At `Vf ~= 0`, the robot is not a simple rolling tire problem. The contact patches see lateral/rightward relative velocity from yaw about the front/rear contact offsets and longitudinal differential wheel motion from the left/right banks. With small forward rolling speed, tire tread elements have little convective renewal through the contact patch, so static friction, bristle deflection, patch twist, and stick-slip become important.

The existing point-contact PlantModel handles some of this through contact right-relative velocity gains and force projection. What remains can plausibly be finite contact-patch scrub moment, low-speed static/dynamic friction transition, and front/rear contact-pressure asymmetry. The near-zero-forward residual being small-to-low-tenths of a Nm, not the rejected 0.9 Nm motor-only result, is physically plausible for "missing scrub details after an already substantial point-contact model."

The sign change at higher yaw rate is also plausible without invoking negative friction. At high in-place yaw, the present mean force/yaw-loss decomposition may over-resist for that mixed sample population, or derivative/bias/tail artifacts may dominate the median. The residual then becomes yaw-aiding because the correction is compensating model over-resistance. This should be read as a failure signature of the current empirical decomposition, not as a reason to preserve old effective-geometry or parameter-bag assumptions.

### 2. Nonzero `Vf`: rolling contact reduces pure pivot scrub

Once the robot is moving forward, the same yaw rate no longer means the same contact-patch physics. The repo spec's correct primitive is per-contact relative velocity, not slip angle:

```text
v_body_f,i = Vf - omega * r_i
v_body_r,i = Vr + omega * f_i
v_rel_f,i = wheel_surface_f,i - v_body_f,i
v_rel_r,i = -v_body_r,i
```

As `|Vf|` grows, the rolling wheel surface cancels more of the body forward contact velocity when the wheel is tracking the floor. That changes `v_rel_f,i`, the relative-speed magnitude, the force envelope, and the coupling between requested forward force and rightward force. It does not cancel `v_rel_r,i` from yaw and lateral body motion. This explains why the nonzero-forward bins are smaller and nonmonotonic without requiring a speed-normalized slip-angle plant.

The expanded data supports that view. Many `|Vf|=0.4..0.6` bins with good repeatability are around `-0.01..+0.006 Nm`, while lower-speed transition bins carry much larger IQR and cross-run spread. That is not the signature of a scalar "more yaw rate means more counter-yaw friction" law.

### 3. Sign changes point to model structure, not a simple CoF surface

Several high-count bins need yaw-aiding additive torque. Real tire forces can create apparent yaw-aiding residuals when the model is missing history:

- Contact deformation/history: force can lag contact-relative-velocity changes. During yaw acceleration, force may be underdeveloped; during yaw deceleration or turn exit, stored deformation can continue to act in the previous direction.
- Combined slip: longitudinal drive/brake force consumes friction budget and changes lateral force capacity. A bin with the same `Vf` and yaw rate can have different residuals depending on whether the robot is accelerating, coasting, or braking.
- Front/rear force balance: PlantModel yaw moment includes `longitudinal_offset * (front_right_force - rear_right_force)`. Small front/rear lateral-force errors can flip the yaw moment sign.
- Historical parameter contamination: some residual data and prior fits were produced while older effective-parameter assumptions were still being phased out. Treat that as a confound, not as the target model posture.

A static two-axis residual table can absorb these effects statistically, which explains the generally improved RMSE, but it aliases multiple mechanisms into `M_res(Vf_bin, yaw_bin)`.

### 4. Competition/open-floor differences are expected

Competition logs add real maze-turn coverage, especially around `Vf=0.10..0.30 m/s` and `|yaw|=0.50..4.50 rad/s`, but the validation found mixed prediction: diagnostic competition holdouts improved, auxiliary competition holdouts worsened. That points to procedure and environment dependence, not just more samples.

Likely contributors:

- old competition schema lacks saturation/watchdog fields and uses derived wheel omega/fan defaults,
- maze maneuvers mix turn entry, plateau, exit, wall interactions, and path-dependent controller behavior,
- floor and tire state are probably different from open-floor characterization,
- competition data populates useful bins but often not with controlled repeats.

Use competition data as coverage and plausibility evidence, not as the primary authority for coefficient fitting.

## Real Physics vs Likely Artifacts

| Candidate | Judgment | Reason |
| --- | --- | --- |
| Contact patch scrub at low rolling speed | Real physics | Strongly consistent with in-place and low-`Vf` behavior; current model is point-contact and cannot represent finite patch twist directly. |
| Contact-relative-velocity force surface | Real physics | Required by the root spec; nonzero-forward residuals shrink and change sign as `v_rel_f`, `v_rel_r`, normal load, and drive request change. |
| Contact deformation/history | Very likely | High spread and sign flips inside the same `Vf,yaw` bins are natural if entry/exit history is mixed, even when the mean plant remains algebraic. |
| Combined longitudinal/lateral saturation | Very likely | Current PlantModel projects combined contact force, but command-dependent drive/brake state can still leak into bins that ignore longitudinal force and saturation. |
| Dynamic normal load/load transfer | Likely at high acceleration | Fan/downforce and acceleration change contact loads. Current normal load is static front fraction plus fan; load transfer can alter front/rear yaw moment. |
| Gyro derivative noise | Important artifact | `M_obs` differentiates gyro rate, so adjacent-sample slope noise directly becomes torque noise. High IQR bins should not be treated as precise friction facts. |
| Command/motor torque model error | Important artifact | The residual is against a mirrored model. Any battery, motor constant, PWM, wheel speed, friction-loss, or saturation mismatch appears as tire torque. |
| Saturation | Important artifact/physics mix | Saturation is real, but old competition logs do not expose all flags. Saturated samples can bias a residual surface. |
| Battery/fan variation | Important artifact/physics mix | Battery affects motor torque; fan changes normal load and effective contact behavior. Old logs use defaults or metadata. |
| Old competition schema | Important artifact | Missing fields make competition bins lower authority than current decoded open-floor logs. |
| Floor/tire state | Real physics and uncontrolled nuisance | Rubber contamination, dust, temperature, fan skirt state, and surface finish can change friction and relaxation. |
| Unmodeled lateral slip `Vr` | Likely | Binning by `Vf` and yaw rate alone ignores side-slip state. PlantModel has `Vr`; the residual extraction table does not condition on it. |
| Wheel spin-up | Partly modeled, still a risk | The reconciled extraction uses the effective yaw denominator including wheel spin-up, but wheel inertia/bank asymmetry errors still enter. |
| Bin leakage | Major artifact | Same rounded `Vf,yaw` bin contains acceleration, deceleration, command sign, saturation, surface, and phase mixtures. |

## Is a Velocity-Dependent CoF Surface the Right Model?

Not as the primary conceptual model.

A velocity-dependent coefficient surface, for example `mu = f(|Vf|, |yaw_rate|)`, is tempting because the residual table improves RMSE. But the sign changes, cross-run spread, and competition/open-floor split are not what a clean CoF surface should produce. CoF changes should mostly scale force capacity; it should not by itself create yaw-aiding residuals in some signed bins and counter-yaw residuals in neighboring bins after the existing model already applies a combined-force projection.

The root spec points to a velocity-space and force-space contact model:

```text
F_req_f,i = F_driveReq_i + K_f,i * v_rel_f,i
F_req_r,i = K_r,i * v_rel_r,i
v_rel_mag,i = sqrt(v_rel_f,i^2 + v_rel_r,i^2 + v_E^2)
mu_i = mu_slide,i + (mu_peak,i - mu_slide,i) * exp(-(v_rel_mag,i / v_Stribeck,i)^2)
F_lim_i = mu_i * N_i
force-space limiter maps F_req_i -> F_i continuously
M_contact = sum_i(f_i * F_r,i - r_i * F_f,i)
M_nom = M_contact - M_loss
```

The first-pass yaw-loss family in the spec is also continuous and velocity-space:

```text
M_loss = (M_yawC * s_yaw + B_yaw * |omega| + K_yawRel * vbar_rel + K_yawLim * lambda_F_max)
         * smooth_sign(omega)
s_yaw = smooth_step(vbar_yaw)
```

This is the right conceptual frame: fit aggregate effective torque, contact-force, and yaw-loss behavior using per-contact relative velocities first, then decompose only when data supports it.

## Recommended Next Model Shape

Do not install the expanded residual table directly as production yaw torque.

The next PlantModel-owned candidate should stay aligned with the root spec's contact-continuum model:

1. Use per-contact `v_rel_f,i` and `v_rel_r,i` as the independent variables, not `Vf/yaw`, turn radius, curvature, or slip angle.
2. Treat the current yaw residual surface as evidence about the empirical contact-force and yaw-loss families, not as a production lookup table.
3. Fit aggregate effects in the spec order: timing/signs, torque, launch/straight friction, contact force, yaw loss, then ground envelope.
4. Keep force-envelope ratio and limiter activity as diagnostics/noise inputs, not command admissibility gates.
5. Consider moving yaw-loss behavior into patch-level forces only if held-out arcs show the separate yaw-loss term is confounded.
6. Record fitted active parameters with source, fit logs, validation logs, residual policy, confounds, robot revision, tire/floor condition, firmware, and estimator build.

A direct residual lookup surface can remain a scratch diagnostic and offline validation tool. It should not become a second plant equation path.

## Minimal Targeted Experiment Matrix

The goal is to break the aliasing between yaw rate, forward speed, longitudinal force, lateral slip, fan load, and transient phase.

Required logging:

- current decoded schema with raw gyro, stationary gyro bias evidence, encoder velocities, commands, battery voltage, fan duty, saturation/utilization, watchdog/fault flags, phase label, and timestamps,
- no old-schema-only fitting for final constants,
- explicit floor/tire condition notes: tire cleaned/not cleaned, floor region, fan skirt state, battery range.

Core matrix:

| Test | Values | Purpose |
| --- | --- | --- |
| In-place yaw plateaus | `Vf=0`, yaw `+-0.5,+-1,+-2,+-4,+-6,+-8 rad/s` where controllable | Identify low-speed scrub and derivative noise separately from rolling slip. |
| Constant-speed arcs | `Vf=0.2,0.4,0.6,0.8,1.0 m/s`, yaw `+-0.5,+-1,+-2,+-4,+-6 rad/s` within floor and lateral-accel limits | Identify steady slip-angle force and front/rear yaw moment. |
| Entry/exit steps | same core speeds, step yaw rate up/down with plateaus | Identify relaxation length/hysteresis. |
| Longitudinal-force split | selected arcs at coast, mild accel, mild brake | Separate combined-slip saturation from pure lateral slip. |
| Fan/load split | repeat a small subset at fan `0.5` and `0.8` or another safe project-approved pair | Check normal-load scaling without inventing a generic safety limit. |
| Direction symmetry | every core point in both yaw directions, forward direction first; reverse only as a validation subset | Detect sign/geometry bias and bin leakage. |

Minimum repeat count: 3 clean repeats per signed point, with direction order randomized enough that tire warmup and floor dust do not always correlate with yaw sign.

## Practical Flags for Existing Data

Use the existing data this way:

- Trust most: decoded open-floor current-schema bins, non-saturated, cross-run support, `count >= 500`, `run_count_ge10 >= 10`, `IQR <= 0.08 Nm`, `run spread <= 0.08 Nm`, and plausible symmetry across yaw signs.
- Trust as shape, not magnitude: high-count bins with small medians but high run spread, especially `|Vf|=0.1..0.3` transition bins.
- Downweight: old competition auxiliary bins, bins where competition contribution creates the bin, bins with missing saturation/fan fields, bins with `run spread > 0.12 Nm`, bins with `IQR > 0.12 Nm`, and bins near zero yaw.
- Treat as missing for production: `|Vf| > 0.8 m/s` except low yaw, high forward speed with high yaw rate, clean high-speed symmetric arcs, reverse high-speed arcs, and any one-run-only bin.

The companion `yaw_bin_trust_flags.csv` summarizes representative bins/regions.

## Commands Run

```powershell
Get-Content -LiteralPath AGENTS.md
Get-Content -LiteralPath codex_analysis\yaw_torque_reconciliation\reconciled_yaw_torque_findings.md
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_report.md
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\expanded_yaw_torque_validation_report.md
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_surface_signed_bins.csv -TotalCount 8
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_surface_abs_summary.csv -TotalCount 8
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_coverage_by_forward.csv -TotalCount 20
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\nonzero_vf_torque_bins.csv -TotalCount 8
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\bin_run_consistency.csv -TotalCount 8
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\competition_bin_contribution.csv -TotalCount 8
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_dataset_comparison.csv
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_holdout_rmse.csv -TotalCount 20
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\expanded_yaw_torque_validation_report.md | Select-Object -Last 80
python - <<inline CSV summary script through PowerShell here-string>>
rg --files | rg "(PlantModel|Vehicle|MotorEncoderDrive|DriveBase|MotionLimits)"
Get-Content -LiteralPath MazeMap\MazeMap\PlantModel.h
Get-Content -LiteralPath MazeMap\MazeMap\PlantModel.cpp -TotalCount 260
Get-Content -LiteralPath MazeMap\MazeMap\Vehicle.h -TotalCount 240
Get-Content -LiteralPath MazeMap\MazeMap\MotorEncoderDrive.h -TotalCount 220
rg -n "forwardStepFromAppliedBankTorques|contactRight|rightForce|yawAccel|saturation|project" MazeMap\MazeMap\PlantModel.cpp
Get-Content selected PlantModel.cpp line windows for contact force and kinematics
New-Item -ItemType Directory -Force -Path codex_analysis\yaw_physics_research
```

No build or release unit test run was performed because this was a scratch research task and no production code was modified.

## Bottom Line

The expanded residual surface is real enough to reject a pure `Vf=0` story, but not clean enough to install as a velocity-dependent CoF table. The most likely physics is a low-speed scrub/bristle transition plus rolling slip-angle lateral force with relaxation and combined-slip saturation. The existing bins are best used to design a PlantModel-owned contact dynamics experiment and to downweight misleading regions, not as direct production calibration data.
