# Compact System Identification Plan For Full Impulse/Step Response Characterization

Date: 2026-05-24

Scope: scratch report only. No production code or tests were modified.

## Decision

Do not promote a runtime impulse-response table, response kernel table, residual `Vf/yaw` map, maneuver branch, or wrapper. The Teensy-consumable output of this work must be a compact physical/dynamic model owned by `PlantModel`.

Impulse and step responses are diagnostic identification evidence, not deployment artifacts or production acceptance evidence. They are useful because these logs are structured measurement protocols: amplitude sweeps, repeated CW/CCW signs, stationary fan/noise baselines, delay-calibrated command edges, sustained-versus-twitch launch cases, and open-floor motion sections. The identification plan should exploit that protocol structure to infer a small set of equations and, if justified, one or two internal PlantModel states. For model tuning, Python model investigations, and data-driven traction/model/control acceptance, valid evidence still requires full-run adherence over complete eligible runs using raw sensors, centralized independent gyro/accelerometer bias evaluation, and current source/build constants. Any diagnostic window used as tuning evidence must span at least `500 ms`; smaller onset snippets are smoke/debug timing visualization only.

The `500 ms` minimum diagnostic-window rule is scoped to that model-tuning and data-driven model-acceptance work. It does not constrain ordinary unit tests, small deterministic unit tests, or code-level behavior tests.

The current single-term contact-yaw scalar should be treated as a failed low-dimensional proxy. It can improve broad aggregate replay in some alignments, but it does not explain the low-speed launch/breakaway behavior where the response changes from arrested twitch to sustained yaw. The next model must explain the measured response shape and threshold structure, not merely reduce a global residual by a few percent.

## Authoritative Boundary

Production owner: `PlantModel`.

Reason: the target behavior is shared plant dynamics: motor-to-contact delay, low-speed contact/bristle behavior, contact force envelope response, and yaw moment generation. `Vehicle` remains the owner of fixed construction facts such as mass, yaw inertia, track width, contact patch locations, wheel radius, and fan downforce capacity. `MotorEncoderDrive` remains the owner of motor/encoder conversion facts. Measurement modes and analysis scripts may produce calibration artifacts, but they must not become runtime owners.

Forbidden production shapes:

- deployable impulse-response kernels or residual tables;
- runtime `Vf/yaw`, maneuver, phase, or section lookup maps;
- wrappers/facades around `PlantModel`;
- per-mode plant branches;
- duplicated plant equations in controllers or measurement modes;
- generic safety-limit owners or command rejection based on fitted residuals.

Allowed production shape, if evidence supports it:

- private PlantModel constants with units and fit provenance;
- compact PlantModel-owned internal dynamic state if the state is a real physical memory term such as bristle/static deflection or actuator lag;
- direct changes to the existing contact-force/yaw-moment equations, with tests through PlantModel.

## What The Offline Responses Represent

For each protocol window, define the measured response as a delay-aligned, direction-normalized relationship from commanded wheel-bank input to observed yaw rate or yaw acceleration:

```text
u(t) = signed differential wheel-bank command or resolved bank torque request
y(t) = signed gyro yaw rate after independent stationary bias removal
a_yaw(t) = dy/dt after robust filtering or aggregate differentiation
```

The offline impulse/step response is not a runtime convolution target. It is a diagnostic view of the hidden low-order dynamics that PlantModel should approximate in closed form:

```text
u(t) -> motor/torque buildup -> wheel/contact relative velocity
     -> bristle/static deflection and breakaway
     -> contact force envelope/projection
     -> yaw moment / yaw inertia
     -> gyro measurement delay/noise
```

This stays aligned with `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`: contact-relative velocity remains the primary contact primitive; yaw behavior is not keyed by maneuver labels, turn radius, curvature, slip ratio, or slip angle. Offline response summaries should be expressed in terms of quantities PlantModel already owns or can derive: per-contact relative velocities, normal loads, force envelope utilization, yaw rate, yaw acceleration, and wheel-bank torque.

## Protocol Structure To Exploit

Use the measurement runs as designed experiments:

- Stationary fan/noise baseline: estimate gyro bias, gyro noise spectrum, fan vibration bands, and measurement filtering. This should define filtering and weighting for fitting, not PlantModel runtime behavior.
- Yaw-launch amplitude sweep: amplitudes 0.50, 0.55, 0.60, 0.65, 0.70 with repeated CW/CCW signs expose the transition from twitch-only arrested motion to sustained yaw. The 0.50 and 0.55 cases are not failures; they are static/bristle threshold evidence.
- Delay-calibrated response: existing yaw-launch analysis recommends +4 samples for first nonbaseline response, +5 for derivative/motion onset, and about +7 for sustained threshold. Fit physical dynamics after applying a fixed effective-time convention; do not absorb timing error into contact coefficients.
- CW/CCW repeats: use paired amplitudes and repeat indices for symmetry, repeatability, threshold variance, and sign-dependent bias tests.
- Open-floor motion sections: use launch, straight, yaw, smooth, loop_clockwise, loop_counter_clockwise, and moving-yaw sections to test whether the low-speed model generalizes once forward velocity, contact-relative velocity, and force envelope utilization are nonzero.
- Final bad tails and off-distribution legacy competition sections: use only as stress evidence unless their schema has per-row fan, saturation/watchdog, clean command evidence, and trimmed tails.

## Fan Filtering And Noise Handling

Fan vibration should be handled as an offline identification step:

1. Estimate stationary gyro/accelerometer spectra at each fan duty where possible.
2. Identify narrow-band fan tones, harmonics, and broad vibration floor.
3. Produce a fitting signal using zero-phase offline filters or robust band rejection. This filtered target is for system ID only.
4. Keep a parallel unfiltered scoring view to ensure the model is not fitting filter artifacts.
5. Record fan duty, fan source, and normal-load assumptions per window.

Do not implement fan notch filters or response filters inside `PlantModel` as a result of this report. Runtime measurement filtering belongs to the estimator/sensor path, while PlantModel should receive the compact physical consequences: fan-dependent normal load and any identified contact parameter dependence on load.

## Candidate Compact Models

The candidates below are ordered from most likely to explain yaw-launch and open-floor behavior with a Teensy-feasible state count.

### 1. Low-Speed Bristle/Static Deflection State

Purpose: explain twitch-only responses and the transition to sustained yaw.

Candidate state per side or aggregate yaw-contact mode:

```text
z_dot = v_rel_yaw - z / tau_z              while below breakaway
M_bristle = -K_z * sat_smooth(z, z_max(N, fan))
breakaway when |M_drive - M_bristle| exceeds M_static(N, fan)
```

Physical meaning: contact patches store elastic shear deflection before sliding. Below threshold, the robot twitches then the stored contact deflection arrests yaw. Above threshold, the stored deflection saturates and sliding yaw begins.

Teensy shape: one aggregate yaw bristle state may be enough for first implementation; four per-contact states are only justified if aggregate state fails CW/CCW or moving-yaw validation.

Fit evidence:

- 0.50 and 0.55 impulse-only responses constrain `K_z`, `tau_z`, and `z_max`.
- 0.60 transition cases constrain breakaway probability/threshold margin.
- 0.65 and 0.70 sustained cases constrain post-breakaway behavior.

### 2. Breakaway Plus Sustained Contact Resistance

Purpose: separate static launch threshold from sliding yaw resistance.

Compact form:

```text
M_contact_loss = sign_smooth(v_rel_yaw_or_omega) *
                 smooth_blend(M_static_hold, M_sliding + B_slide * |omega|, breakaway_state)
```

This must be implemented through PlantModel contact forces or a PlantModel-owned yaw-contact term derived from contact-relative velocity and normal load. It must not be a maneuver branch or a constant yaw damping table.

Fit evidence:

- Sustained/twitch labels define the transition region.
- CW/CCW amplitudes reveal asymmetry only after stationary bias and timing are corrected.
- Fan duty variation should scale threshold through normal load if the effect is physical contact, not motor timing.

### 3. Motor/Torque Lag

Purpose: explain response delay and early ramp shape before attributing it to contact.

Candidate state per wheel bank:

```text
tau_applied_dot = (tau_commanded - tau_applied) / tau_motor
```

or a small first-order current/torque lag folded into existing command-to-bank torque resolution.

Fit evidence:

- Delay-calibrated response has first nonbaseline around +4 samples, derivative/motion onset near +5.
- A fixed measurement delay alone should not explain amplitude-dependent twitch/sustain outcomes.
- If early ramp time constants are similar across amplitudes and signs, motor lag is plausible.

Implementation caution: this belongs in `PlantModel` only if it is part of the plant equation. If it is just sensor effective-time or gyro filtering, it belongs to offline fitting/estimator timing, not PlantModel contact dynamics.

### 4. Contact Patch Force-Envelope Dynamics

Purpose: explain why scalar pre-projection corrections may be clipped away or fail in high-contact rows.

Compact form:

```text
F_raw_i = existing longitudinal/right contact force request + compact correction
F_i = project_to_envelope(F_raw_i, mu(N_i), optional smooth dynamic utilization state)
```

Potential correction variables:

- per-contact relative velocity `v_rel_f_i`, `v_rel_r_i`;
- load `N_i`;
- smooth force utilization;
- bristle/sliding state from candidate 1.

The correction should modify raw contact force requests before projection, not add an after-the-fact residual yaw moment.

### 5. Fan/Load-Condition Parameter Dependence

Purpose: map response across conditions without a runtime table.

Compact form:

```text
M_static(N) = c_static0 + c_staticN * N_total
K_z(N) = c_z0 + c_zN * N_total
M_slide(N) = c_slide0 + c_slideN * N_total
```

Fan duty should enter through normal load already owned by Vehicle/PlantModel. Do not create a separate fan-response table. If fan duty only changes measurement noise, use it as a fitting weight/noise schedule, not a plant force term.

### 6. Delay/Noise Model For Fitting Only

Purpose: prevent timing and fan vibration from contaminating physical coefficients.

Offline-only model:

```text
y_meas(t) = LPF_gyro(y_true(t - d_eff)) + fan_noise(t) + white_noise
```

This is used to align, filter, and weight samples. It is not a deployable PlantModel impulse response or runtime measurement table.

## Identification Workflow

1. Build a protocol inventory.
   - Enumerate yaw-launch steps by amplitude, sign, repeat, quality flag, delay, peak response, steady response, and pre-baseline noise.
   - Enumerate open-floor motion windows by section/phase, fan source, saturation/watchdog availability, tail quality, and command evidence.

2. Establish fitting targets.
   - Use raw gyro yaw rate minus independent stationary bias.
   - Use delay-aligned windows with +4 samples as the primary first-response convention and +5 as derivative-onset sensitivity.
   - Use filtered yaw rate or aggregate differentiated yaw acceleration for fitting; retain raw scoring for audit.
   - Do not use UKF state as a target.

3. Fit timing before contact.
   - Fit a global effective delay and optional motor torque lag on sustained high-amplitude launch cases.
   - Reject amplitude-dependent delay unless a compact physical lag state explains it.
   - Freeze timing before fitting contact coefficients.

4. Fit low-speed contact memory.
   - Fit bristle/static-deflection and breakaway parameters on yaw-launch amplitude sweeps.
   - Include twitch-only cases as threshold evidence, not as failed sustained-yaw samples.
   - Use CW/CCW repeats to separate symmetric physics from motor or sensor sign bias.

5. Fit sustained/open-floor contact behavior.
   - Use reliable decoded open-floor runs with per-row fan, saturation/watchdog fields, clean command evidence, and trimmed tails.
   - Downweight or exclude saturated rows depending on candidate; saturation is evidence for force-envelope behavior but dangerous for fitting unconstrained torque residuals.
   - Fit moving-yaw and in-place-yaw jointly only after the launch threshold model is frozen.

6. Map condition dependence compactly.
   - Evaluate parameter variation versus fan duty/normal load, command amplitude, yaw rate, forward velocity, and force-envelope utilization.
   - Promote only smooth scalar dependencies with physical meaning.
   - Reject any dependency that only survives as a table by run, section, maneuver, or final failure segment.

7. Validate held-out behavior.
   - Hold out repeats by amplitude/sign, not random samples from the same step.
   - Hold out entire open-floor runs, including at least one moving-yaw-heavy and one low-speed/in-place-heavy run.
   - Score raw and filtered signals, but select using filtered protocol-consistent targets.

## Scoring And Diagnostic Promotion Criteria

Primary diagnostic promotion target: the compact model must explain more than half of the reliable prediction error that remains after timing/filter handling. This section screens candidates for possible PlantModel work; it is not standalone production acceptance.

Use normalized error reduction:

```text
reduction = (RMSE_baseline - RMSE_candidate) / RMSE_baseline
```

Baseline: current PlantModel with scalar contact-yaw correction disabled or zero, evaluated with the same delay/filter convention. Do not compare against a misaligned baseline that lets timing error inflate the apparent gain.

Required gates:

- Reliable yaw-launch subset: at least 50% RMSE reduction on delay-aligned, fan-filtered yaw rate or yaw acceleration windows for 0.50-0.70 amplitudes, with separate reporting for twitch-only, transition, and sustained cases.
- Reliable open-floor subset: at least 50% RMSE reduction on held-out in-place-yaw and moving-yaw subsets after excluding bad tails and handling saturation explicitly.
- Held-out repeats: improvement must hold when entire repeats, not random samples, are held out.
- CW/CCW symmetry: symmetric parameters should predict both signs. Any sign-specific term requires persistent sign evidence after gyro bias, motor command asymmetry, and timing are corrected.
- Threshold classification: the model should classify twitch-only versus sustained-launch cases around 0.60-0.65 materially better than the current scalar model.
- No overfit to failure tails: final sensor-quiescent or invalid tails must not improve the score; they should be excluded or scored separately as rejection evidence.
- No straight-line regression: low-demand mostly-forward/straight sections must not acquire yaw bias or increased RMSE.
- Parameter parsimony: prefer the smallest state/equation set that clears the gates. A model with many coefficients but no held-out repeat gain is rejected.

Secondary acceptance:

- residual mean near zero by amplitude/sign bin;
- reduced residual autocorrelation over launch windows;
- stable parameter estimates across fit-authoritative runs;
- plausible fan/load scaling;
- finite and continuous behavior at zero forward speed and zero yaw rate.

## What To Trust From Current Artifacts

Trust:

- `yaw_launch_step_response`: structured amplitude/sign/repeat windows, twitch/sustained labels, independent stationary gyro bias, and the finding that +4/+5 samples are the meaningful delay region.
- `yaw_launch_delay_calibration`: fixed effective-time guidance; +4 first-response and +5 derivative onset are strong enough for fitting sensitivity checks.
- `contact_continuum_yaw_identification/data_quality`: fit-authoritative versus downweighted/validation/excluded run categorization.
- `contact_continuum_yaw_identification/features`: contact-continuum feature definitions and PlantModel mirror constants, with the explicit limitation that lateral velocity is unavailable.
- `contact_continuum_yaw_identification/ablation`: evidence that contact-patch features are more plausible than aggregate yaw-loss for plateau residuals, but not sufficient for production promotion.
- `contact_correction_impact_investigation`: conclusion that the scalar correction is under-expressive, especially for in-place yaw and launch behavior.

Reject or downgrade:

- global scalar gain improvements that are dominated by sample population or mostly-forward sections;
- response fits that require runtime tables/kernels;
- fits using UKF state as target truth;
- legacy competition rows as fit authority when saturation/watchdog/per-row fan fields are absent;
- final bad tails and derivative spikes as model evidence;
- amplitude-dependent delay if it is not explained by a compact motor/contact state;
- sign asymmetry before stationary bias, motor asymmetry, and command timing are accounted for.

## Expected IR-A And IR-B Outputs

IR-A should provide fan-filtered yaw-launch protocol evidence:

- stationary fan/noise spectra and recommended offline filters;
- per-step delay-aligned yaw-rate and yaw-acceleration summaries;
- amplitude/sign/repeat aggregate response metrics;
- twitch/transition/sustained labels with confidence;
- CW/CCW symmetry report;
- candidate initial estimates for motor lag, bristle stiffness/time constant, static threshold, and sliding resistance.

Trust IR-A outputs when they are derived from raw gyro minus stationary bias, use the fixed delay convention, and report held-out repeat behavior. Reject IR-A outputs that package deployable kernels, fit per-amplitude tables, or merge twitch-only and sustained cases into one least-squares target without labels.

IR-B should provide open-floor compact-model validation evidence:

- reliable run inventory using the data-quality recommendations;
- open-floor windows grouped by in-place yaw, moving yaw, mostly-forward, and low-motion commanded behavior;
- fan/source/saturation/watchdog/tail annotations;
- contact-continuum feature summaries using PlantModel-compatible variables;
- held-out run and held-out section scores for each compact candidate;
- evidence for or against fan/load scaling and force-envelope dependence.

Trust IR-B outputs when they hold out entire runs/sections and preserve the PlantModel contact-continuum vocabulary. Reject IR-B outputs that rely on residual `Vf/yaw` maps, maneuver labels, final failure segments, or unmarked legacy schema gaps.

## Next Worker Checklist

1. Create a unified scratch evaluator that can simulate candidate compact models offline against yaw-launch and open-floor windows without changing production code.
2. Start with three candidates: motor lag only, bristle/breakaway only, and motor lag plus bristle/breakaway.
3. Fit timing on sustained 0.65/0.70 yaw-launch cases, then freeze it.
4. Fit bristle/static threshold on 0.50/0.55 twitch-only and 0.60 transition cases.
5. Validate sustained response on held-out 0.65/0.70 repeats.
6. Validate moving-yaw/open-floor sections using fit-authoritative runs and held-out run splits.
7. Promote only compact equations/states that clear the >50% reliable-subset reduction gate.

## Production Implementation Shape If Accepted Later

The future production change should be a copy-delete-stitch replacement of the scalar contact correction inside `PlantModel`, not an addition beside it.

Expected PlantModel touch point:

- resolve command-to-bank torque and any accepted torque lag;
- compute per-contact relative velocity using the existing PlantModel kinematics;
- update a private PlantModel contact memory state if accepted;
- compute raw contact force requests from compact contact equations;
- project forces through the existing envelope;
- sum yaw moment from projected contact forces.

Tests should be PlantModel-focused: finite behavior at zero forward speed, continuity across zero, no straight-line yaw bias, CW/CCW sign preservation, fan/load scaling sanity, and release replay/unit verification after confirming binaries reflect any future source changes.

