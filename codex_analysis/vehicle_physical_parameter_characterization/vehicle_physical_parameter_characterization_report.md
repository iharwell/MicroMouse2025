# Vehicle Physical Parameter Characterization Audit

Scratch analysis only. No production code or tests were modified.

## Direct Answer

No. The existing work has not characterized the Vehicle-like empirical physical values well enough to safely update them as production construction facts.

The current source values are mostly a mix of hardware specification, bench/geometry estimates, log-derived kinematic trims, and dynamic residual fits. Several are useful and should remain as the current best known values, but the prior yaw fits do not justify replacing `Vehicle` mass, yaw inertia, or physical/effective track as if those were newly measured robot facts. The scalar four-run fit to `track = 0.104595474 m` and `yaw_inertia = 0.000603133 kg m^2` is explicitly rejected for `Vehicle`: it improved one-step in-place yaw RMSE by absorbing missing yaw resistance/contact dynamics, not by measuring chassis geometry or true yaw inertia.

The safe conclusion is: keep ownership where it is (`Vehicle` for construction facts, `MotorEncoderDrive` for wheel-bank facts, `PlantModel` for equations), do not install new `Vehicle` constants from the dynamic yaw fits, and run targeted physical/measurement characterization before production updates.

## Source Constants Audited

Current source values confirmed from `MazeMap/MazeMap/Vehicle.h`, `Vehicle.cpp`, `MotorEncoderDrive.h`, and `PlantModel.h`:

| Owner | Value | Current source value | Source line basis | Characterization judgment |
| --- | --- | ---: | --- | --- |
| `Vehicle` | mass | `0.14 kg` | `kPhysicalMassKg` | plausible bench value, but not tied to an auditable current weigh-in artifact |
| `Vehicle` | width | `0.0842 m` | `kPhysicalWidthM` | construction geometry, source measurement not documented in current analysis artifacts |
| `Vehicle` | length | `0.1085 m` | `kPhysicalLengthM` | construction geometry, source measurement not documented in current analysis artifacts |
| `Vehicle` | yaw inertia | `0.000220 kg m^2` | `kPhysicalYawInertiaKgM2` | derived from 140 g and body envelope; matches rectangular-body estimate `0.000220055 kg m^2`, not dynamically validated |
| `Vehicle` | front wall contact offset | `0.056 m` | `kFrontWallContactOffsetM` | geometry value; not reconciled here against recent wall-touch logs |
| `Vehicle` | effective track width | `0.084635 m` | `kPhysicalTrackWidthM` | log-derived IP180 effective kinematic fit, not pure physical tire span |
| `Vehicle` | physical contact span bounds | `0.07004..0.07868 m` | `kTrackWidthPhysicalMinM`, `kTrackWidthPhysicalMaxM` | measured chassis/tire span; conflicts with the larger effective kinematic track |
| `Vehicle` | drive wheel longitudinal offset | `0.01475 m` | `kDriveWheelLongitudinalOffsetM` | currently used as contact placement; note says PlantModel still uses effective track placement |
| `Vehicle` | arc effective track width | `0.13235 m` at `0.063` and `0.153 m` radii | arc track constants | one post-fan-swap smooth-card fit; low confidence |
| `Vehicle` | drive wheel diameter | `0.025220 m` | `kDriveWheelDiameterM` | March 22 low-speed straight-audit rolling trim; supplied tire OD was `25.000 mm` |
| `Vehicle` | encoder pulses per motor rev | `4096` | `kDriveEncoderPulsesPerRev` | hardware specification |
| `Vehicle` | drive gear ratio | `56/17 = 3.294117647` | `kDriveGearRatio` | hardware specification |
| `Vehicle` | fan downforce at full duty | `0.7 N` | `kFanDownforceAtFullDutyN` | force-envelope input; empirical basis not proven in these artifacts |
| `Vehicle` | sustained lateral acceleration reference | `1.91 g = 18.73 m/s^2` | ramp-slip comment | measured inclined-ramp slip claim; consistent with high-grip planning reference |
| `Vehicle.cpp` | peak forward accel/yaw rate/yaw accel/max speed | `15 m/s^2`, `27 rad/s`, `645 rad/s^2`, `4 m/s` | local static constants | explicitly provisional in source comment |
| `MotorEncoderDrive` | motor resistance | `4.31 ohm` | constructor input from `Vehicle` | likely datasheet/spec, not current bench fitted |
| `MotorEncoderDrive` | torque constant | `0.00396 Nm/A` | constructor input from `Vehicle` | likely datasheet/spec, not current bench fitted |
| `MotorEncoderDrive` | no-load current | `0.0459 A` | constructor input from `Vehicle` | likely datasheet/spec, not current bench fitted |
| `MotorEncoderDrive` | speed constant | `254.482 rad/s/V` computed | derived from no-load rpm/voltage/current/resistance | derived from motor specs |
| `MotorEncoderDrive` | wheel-bank equivalent inertia | `1.177e-6 kg m^2` | default wheel-bank inertia | first-pass calculation, missing hub/gear inertia |
| `MotorEncoderDrive` | longitudinal tire stiffness | `4.12 N` | default tire stiffness | derived from tire/contact notes; comment arithmetic is not self-consistent |
| `MotorEncoderDrive` | right-contact force gains | front `18`, rear `16 N/(m/s)` | default contact gains | fit from April open-floor right-slip logs; not enough shape validation |
| `PlantModel` | residual decay time constants | `0.075 s` forward/right/yaw accel residual | existing PlantModel constants | estimator/model tuning, not Vehicle physical characterization |
| `PlantModel` | rolling friction torque | `0.00372 Nm` | `kRollingFrictionTorqueNm` | appears in logs/model mirror; empirical basis not independently characterized here |
| `PlantModel` | reliable launch command | `0.30` | `kReliableLaunchDriveCommand` | procedural threshold proxy, not a measured physical constant |
| `PlantModel` | static friction max speed | `0.005 m/s` | `kStaticFrictionMaxSpeedMps` | low-speed model threshold, not bench characterized |
| `PlantModel` | contact friction coefficients | `mu_front = mu_rear = 1.65` | existing PlantModel constants | force-envelope assumption; not validated across fan/load/surface here |

Derived current values:

| Quantity | Current derived value |
| --- | ---: |
| wheel radius | `0.012610 m` |
| encoder distance per count | `5.872133e-6 m/count` |
| motor current limit proxy at 8.4 V / 4.31 ohm | `1.948956 A` |
| bank stall torque proxy after no-load current | `0.024825 Nm` |
| bank stall force proxy | `1.96866 N` |
| static launch torque proxy from `0.30` command at zero wheel speed | `0.007028 Nm/bank` |
| differential yaw moment proxy at `0.30/-0.30` command | `0.047172 Nm` |
| yaw accel proxy using current yaw denominator `0.0002465104` | `191.36 rad/s^2` |

## Reconciliation With Historical Analyses

### Rejected scalar geometry/inertia fit

The four-run scalar yaw fit produced:

| Parameter | Current | Fitted |
| --- | ---: | ---: |
| track width | `0.084635000 m` | `0.104595474 m` |
| yaw inertia | `0.000220000 kg m^2` | `0.000603133 kg m^2` |
| one-step yaw-rate RMSE | `0.226296125 rad/s` | `0.161693351 rad/s` |

This is not a valid physical update. The reconciliation report already found that the scalar fit was useful only as a diagnostic. It was fitting a narrow in-place/stop dataset and absorbing missing yaw resistance/contact behavior. It did not measure chassis track width or true yaw inertia.

### Yaw-launch impulse/compact characterization

The yaw-launch compact work did characterize dynamic behavior that matters to `PlantModel`, not replacement `Vehicle` construction facts:

- command-to-first-gyro response is `+4 ms`, with derivative/onset sanity at `+5 ms`;
- initial yaw acceleration is roughly `143..186 rad/s^2`, with filtered fan-filter summary showing all-step initial slope mean around `183.5 rad/s^2`;
- twitch/static-bristle behavior is clear at `0.50/0.55` commands: the robot moves initially but arrests while command remains on;
- breakaway is sign-asymmetric: CW near `0.610`, CCW near `0.650`;
- sustained launch/recovery time constants are about `16..17 ms`, but the `0.70` sustained cases show larger rise/settle variation;
- current PlantModel overpredicts the yaw-launch windows enough that a no-response baseline can beat it in the compact derivative comparison.

Those facts argue for better `PlantModel` contact/launch dynamics and possibly motor asymmetry testing. They do not justify changing mass, track, yaw inertia, or wheel geometry.

### Fan/vibration filter

The fan-filter work found a dominant stationary fan/vibration line near `149.2 Hz` at `0.8` fan duty. Per-step sinusoid subtraction improves onset detectability and preserves command-onset timing. It supports using filtered gyro for yaw-launch timing and initial-accel estimates, but it does not create final physical constants.

### Expanded yaw/contact residuals

The expanded yaw residual work is strong enough to reject a one-term model:

- residuals are not monotonic counter-yaw resistance;
- several high-count bins need yaw-aiding additive torque, not more resistance;
- nonzero-forward coverage exists through about `0.6 m/s`, is thin at `0.7 m/s`, and is sparse above `0.8 m/s`;
- competition data adds real maze coverage but is path/procedure/schema dependent;
- a direct residual surface improves some aggregate holdouts, but individual runs/regimes remain mixed.

This fails the shape-across-regimes gate for any single scalar update. A change that improves in-place yaw by distorting moving/open-floor or mostly-forward behavior should be rejected.

## Parameter Classification

| Parameter group | Classification | Reason |
| --- | --- | --- |
| gear ratio and encoder PPR | characterized and usable now | hardware specification, not controversial in the artifacts |
| mass `0.14 kg` | characterized but needs current bench confirmation before update | plausible current value, but source artifacts also reference `0.138 kg`; no attached weigh-in record |
| body width/length | characterized but needs bench confirmation before update | geometry values exist, but no current measurement artifact was found here |
| yaw inertia `0.000220 kg m^2` | provisional construction estimate | matches simple rectangular-envelope derivation; dynamic fits are rejected as physical evidence |
| physical contact span `70.04..78.68 mm` | characterized but not integrated cleanly | measured span exists, but effective track/contact placement uses larger kinematic values |
| effective in-place track `84.635 mm` | provisional log-derived kinematic value | derived from one audit class; not a pure construction fact |
| scalar-fit track `104.595 mm` and yaw inertia `0.000603 kg m^2` | not usable | rejected as absorbing missing yaw resistance |
| arc effective track `132.35 mm` | provisional/insufficient | one post-fan-swap smooth card only |
| wheel diameter `25.220 mm` | characterized but provisional | rolling-distance trim from straight audit; physical OD note says `25.000 mm`; needs post-estimator rerun |
| motor constants | usable as spec defaults, not empirically characterized | no bench current/torque/speed validation in the reviewed artifacts |
| wheel-bank equivalent inertia | insufficient | first-pass calculation excludes hub/gear and lacks spin-down/spin-up validation |
| tire/contact stiffness `4.12 N` | insufficient/provisional | derived note has arithmetic inconsistency and lacks direct force-deflection validation |
| front/rear right-contact gains `18/16 N/(m/s)` | provisional | fit from right-slip logs but not validated across fan/load/speed/yaw sign regimes |
| fan downforce `0.7 N` and sustained lateral `1.91 g` | provisional but useful envelope inputs | ramp/downforce claims exist; not tied to current fan/surface/tire matrix |
| rolling/static friction thresholds | provisional PlantModel terms | dynamic behavior clearly exists, but current scalar terms do not pass shape-across-regimes validation |

## Regime Coverage Gate

| Regime | Coverage status | Production-constant consequence |
| --- | --- | --- |
| twitch/static-bristle | covered by yaw-launch pulses at `0.50/0.55`; filtered gyro supports timing/onset | enough to show current model shape is missing bristle/arrest behavior; not enough for `Vehicle` constants |
| transitional breakaway | partially covered around `0.60/0.65`, sign-asymmetric | requires smooth PlantModel breakaway/contact term; do not replace track/inertia |
| sustained in-place yaw launch | covered for one primary fan duty/log and older open-floor runs | useful for PlantModel dynamics, but scalar geometry/inertia fit is rejected |
| moving/open-floor yaw | broad exploratory coverage to `~0.6 m/s`, thinner above | insufficient for final production coefficients; sign flips reject one-term resistance |
| mostly-forward | some straight audit evidence and residual improvements | wheel diameter remains provisional until post-estimator distance closure is rerun |
| high-speed forward/maze-like | insufficient, especially `>0.8 m/s` and high yaw | hard blocker for production shape changes that claim competition coverage |

Hard gate result: no one-term model or scalar physical-constant update passes. Sacrificing moving/open-floor or mostly-forward behavior to improve in-place launch is unacceptable.

## Missing Characterization Needed

Bench measurements needed before `Vehicle`/`MotorEncoderDrive` physical updates:

1. Current competition-ready mass with battery, fan/skirt, tires, and logging of measurement date/configuration.
2. Width, length, front wall contact offset, drive wheel longitudinal contact offsets, and physical tire contact span under the same tire/fan/skirt configuration.
3. Wheel OD unloaded and loaded rolling circumference at fan off and nominal fan duty; repeat left/right.
4. Encoder distance closure over measured straight distances using current estimator/log schema, before and after any wheel-diameter change.
5. Independent yaw inertia measurement, preferably bifilar/trifilar pendulum or equivalent, with wheel-bank contribution either fixed or separately accounted.
6. Wheel-bank inertia measurement or spin-up/spin-down bench test including tire, hub, gear, and reflected motor rotor.
7. Motor electrical/torque characterization: resistance hot/cold, PWM-to-voltage under battery load, no-load speed/current, and torque/current validation.
8. Tire/contact stiffness and force envelope: normal-load deflection, longitudinal force, lateral force, and fan/load split.
9. Fan downforce measurement at relevant duty values, including post-swap fan state.

Log analyses needed before production PlantModel changes:

1. Current-schema yaw-launch rerun using the fan-filter preprocessing and held-out validation, not zero-phase provisional filtering.
2. In-place yaw plateaus at `Vf=0`, yaw `+-0.5,+-1,+-2,+-4,+-6,+-8 rad/s` where controllable.
3. Constant-speed arcs at `Vf=0.2,0.4,0.6,0.8,1.0 m/s`, yaw `+-0.5,+-1,+-2,+-4,+-6 rad/s` within the robot/floor envelope.
4. Entry/exit yaw steps at the same speeds to identify relaxation/hysteresis.
5. Coast, mild-accel, and mild-brake splits to separate combined slip from pure lateral/yaw behavior.
6. Fan/load split at project-approved fan duty points.
7. Direction-symmetric repeats, at least three clean repeats per signed point, with randomized order enough to avoid tire warmup/floor dust confounds.

Required row fields: raw gyro, independently recoverable stationary bias evidence, encoder velocities and wheel speeds, drive commands, battery voltage, fan duty, saturation/utilization, watchdog/fault flags, phase labels, timestamps, and floor/tire/fan-skirt condition notes.

## Production Readiness Conclusion

The current values are reasonable as current working defaults, but not newly characterized production updates. The empirical work characterized important dynamic yaw/contact behavior, especially launch timing, breakaway, residual shape, and fan vibration contamination. It did not complete physical characterization of `Vehicle`-owned construction facts or `MotorEncoderDrive` wheel-bank constants.

Recommended action: do not update `Vehicle`, `MotorEncoderDrive`, or existing `PlantModel` physical constants from the reviewed fits. Use this audit to plan the bench measurements and targeted log matrix above, then update only the authoritative owner for each proven value.

## Commands Run

```powershell
Get-Content -LiteralPath AGENTS.md
Get-ChildItem -LiteralPath codex_analysis\full_impulse_response_characterization -Recurse -File
git status --short
rg -n "class Vehicle|struct Vehicle|Vehicle::|wheel|track|mass|inertia|gear|encoder|radius|diameter|MotorEncoderDrive|PlantModel|contact|stiff|force|torque|friction|yaw" MazeMap\MazeMap -g "*.h" -g "*.cpp"
Get-Content selected line windows from MazeMap\MazeMap\Vehicle.h
Get-Content selected line windows from MazeMap\MazeMap\Vehicle.cpp
Get-Content selected line windows from MazeMap\MazeMap\MotorEncoderDrive.h
Get-Content selected line windows from MazeMap\MazeMap\PlantModel.h
Get-Content selected line windows from MazeMap\MazeMap\PlantModel.cpp
rg -n "0\.104595474|0\.000603133|0\.084635|0\.000220|0\.025220|56\.0f / 17\.0f|132\.35|0\.13235|1\.177e-6|4\.12|18\.0f|16\.0f|63\.2|1\.91|0\.7f|15\.0f|27\.0f|645\.0f" codex_analysis MazeMap\MazeMap micromouse_ukf_plant_measurement_noise_theory_only_spec.md -g "*.md" -g "*.csv" -g "*.h" -g "*.cpp"
Get-Content -LiteralPath codex_analysis\yaw_torque_reconciliation\reconciled_yaw_torque_findings.md
Get-Content -LiteralPath codex_analysis\yaw_fit\yaw_fit_report.md
Get-Content -LiteralPath codex_analysis\yaw_physics_research\yaw_physics_research_report.md
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded\expanded_yaw_torque_report.md
Get-Content -LiteralPath codex_analysis\yaw_torque_expanded_validation\expanded_yaw_torque_validation_report.md
Get-Content -LiteralPath codex_analysis\full_impulse_response_characterization\step_kernels\yaw_launch_compact_identification_report.md
Get-Content -LiteralPath codex_analysis\full_impulse_response_characterization\fan_filter\fan_vibration_filter_report.md
Get-Content -LiteralPath codex_analysis\full_impulse_response_characterization\step_kernels\compact_model_parameter_ranges.csv
Get-Content -LiteralPath codex_analysis\full_impulse_response_characterization\fan_filter\yaw_launch_filter_identifiability_summary.csv
Get-Content -LiteralPath codex_analysis\full_impulse_response_characterization\step_kernels\condition_summary_metrics.csv
Get-Content -LiteralPath codex_analysis\contact_correction_tuning\tuning_report.md
Get-Content -LiteralPath codex_analysis\contact_correction_tuning\production_replay\evaluation_report.md
PowerShell derived-value calculations for wheel radius, encoder distance/count, speed constant, current limit, torque proxies, and rectangular yaw inertia.
New-Item -ItemType Directory -Force -Path codex_analysis\vehicle_physical_parameter_characterization
```

No build or release unit test run was performed because this was a scratch-only analysis task and production code/tests were not modified.
