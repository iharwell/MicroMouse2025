# UKF Plant/Measurement Model Audit: Flexible Model-Family Foundation

**Reviewed document:** `ukf_plant_measurement_model_complete_fixed.md`  
**Audit standard:** The spec is useful only if it allows a model that can reasonably approximate the physical robot. That requires a disciplined middle ground: strict contracts for semantics, units, timing, covariance, and measurement meaning; flexible model families for empirical physics that may not match a first-pass equation.

## Firm specification stance

The spec should not treat “flexible” as “unspecified.” For every flexible physical submodel, the document should include all of the following:

| Required element | Purpose |
|---|---|
| **Model contract** | Defines inputs, outputs, units, signs, required diagnostics, and covariance hooks. |
| **First-pass implementation** | States the project’s best current model, not merely an example. |
| **Expected failure signatures** | Identifies the logs or innovation patterns that show the first pass is not representing the robot. |
| **Follow-on option[s]** | Gives at least one targeted upgrade path that addresses the known weaknesses of the first pass. |
| **Promotion rule** | Defines what evidence justifies replacing the first-pass model. |

This is the central correction to the previous audit. The spec should not simply lock equations down harder. It should lock down the **interface and validation obligations**, then give the implementation enough room to move from the best first-pass approximation to a better empirical representation when logs show the first pass is inadequate.

## Priority key

| Priority | Meaning under this audit standard |
|---:|---|
| **P0** | The spec could block or obscure a physically representative implementation. Resolve before serious physical calibration. |
| **P1** | High risk to physical fit, estimator consistency, or validation credibility. Resolve before relying on tuned logs. |
| **P2** | Important robustness, replay comparability, or diagnostic issue. |
| **P3** | Low-risk cleanup or implementation guardrail. |

## Audit table

| Section | Priority | Concern level |
|---|---:|---|
| **11 — Tire-force model / combined-slip scaling** | **P0** | The contact-force law is a flexible empirical model and must be presented as such. The current formula should not be treated as physical truth. The spec needs a first-pass force law plus targeted follow-on options for saturation-shape, anisotropy, and stick-slip failure modes. |
| **10 — Normal load model** | **P0** | Normal load is core to friction capacity, but the exact load-transfer algorithm should not be over-specified. The spec needs a first-pass algebraic load model plus explicit alternatives if acceleration-lag, wheel unloading, fan/downforce uncertainty, or ground-strike behavior invalidates it. |
| **8 / 15.2 — Motor-driver command-to-torque model** | **P0** | With no live `Vbat`, the actuator model must be an empirical model family. The spec should define a torque-map contract, a best first-pass map, and follow-on options for bus sag, DRV8871 current regulation, thermal drift, PWM decay mode, and phase/ripple mismatch. |
| **12 — Ground-strike / acceleration envelope** | **P0** | The spec must not force a single deterministic ground-strike mechanism if the real robot exhibits pitch impact, chassis rocking, and transient load redistribution. It needs a clear first-pass acceleration-envelope approximation and at least one force-consistent or event-based upgrade path. |
| **5.2 / 15.3 — Residual acceleration states** | **P0** | Residuals are necessary model slack, not merely errors to suppress. The spec should require residual diagnostics and low-residual validation in benign regimes, but it must not hard-fail valid aggressive maneuvers solely because residuals are large. |
| **22–23 — Calibration and validation** | **P0** | The validation section should require metrics and held-out tests, but threshold values should be versioned from data. Hard-coded thresholds before characterization risk rejecting the best physical model because the initial decomposition was wrong. |
| **1 / 6 / 8 / 10 / 11 / 12 / 19 — Flexible-model declaration pattern** | **P0** | The document needs an explicit pattern: every flexible submodel gets a contract, first-pass model, expected failure signatures, and follow-on option. Without that, future implementers may either over-lock equations or leave model-critical behavior vague. |
| **24 — Parameter table and identifiability** | **P1** | The glossary is not enough. The spec needs priors, units, soft/hard bounds, fitted-from logs, validation maneuvers, and whether residuals are enabled while fitting. Bounds should be soft unless they encode physical impossibility or hardware safety. |
| **12 / 14.1 — In-place yaw scrub and `yawKineticMoment` scope** | **P1** | The physical notes identify roughly constant in-place turning resistance. The best first pass is scrub-gated, but the spec should allow a separate global yaw-loss term if rolling arcs require it. Ambiguity between these interpretations is the risk. |
| **17.2 — Accelerometer measurement semantics** | **P1** | The measurement contract should be hard, but the calibration/filtering implementation should remain flexible. The spec must define exactly what `z_Af/z_Ar` mean, while allowing hardware-layer calibration or estimator-side small misalignment terms. |
| **19 — Wall-sensor measurement model** | **P1** | Wall sensing should be response-space and hypothesis-gated. The ray bundle and soft-min are good first-pass mechanisms, but the spec should allow empirical response models if the IR/log-amp behavior is not captured by geometry alone. |
| **15.4 / 15.5 — Covariance propagation and scheduling** | **P1** | This is one of the places where hard contracts are justified. Residual and encoder-input uncertainty must propagate through the full nonlinear plant. The implementation method can remain flexible: augmented sigma points, sensitivity mapping, or another validated equivalent method. |
| **4 / 21 — Timing and command-validity convention** | **P1** | Timing semantics must be hard enough for replay comparability. The spec should require effective sample times and command-validity convention, but should not prescribe ADC/SPI/scheduler implementation details. |
| **14.3 — High-speed stick-slip arcs** | **P1** | A smooth mean model plus increased process noise is a reasonable first pass. The spec should state what log evidence would trigger a richer empirical lateral/yaw model rather than letting residuals absorb all arc behavior indefinitely. |
| **7 / 18 — Encoder input semantics and optional rolling pseudo-measurement** | **P2** | Encoder-as-drivetrain-input is a hard semantic contract. The optional rolling pseudo-measurement should remain weak, gated, and explicitly replaceable by calibrated rolling-contact geometry if the two-wheel `b/2` form is too crude. |
| **20 — External stationary gyro-bias estimator** | **P2** | The external bias structure is appropriate. It needs operational stationary thresholds and lockouts, but these thresholds should be calibrated from logs rather than guessed as permanent constants. |
| **6 — Smooth helper functions** | **P3** | Helper functions should be defaults, not physical law. The spec should require continuity and numerical safety, while allowing replacement of smooth approximants if they preserve the model contract. |

---

# Detailed audit and proposed spec posture

## 1. Tire-force model / combined-slip scaling — P0

### Concern

The tire/contact law is the most important empirical physics block in the plant. It affects straight launch, rolling arcs, scrubbed in-place turns, stick-slip, yaw moment, and regime scheduling.

The current combined-slip limiter:

\[
forceScale_i = \frac{\tanh(util_i)}{util_i}
\]

is smooth, but it changes the meaning of the friction envelope. At `util = 1`, it scales the requested force by about 0.762. That is not “near the envelope”; it is already a substantial reduction. This is not necessarily unusable, but the spec should not let `util = 1` simultaneously mean “at the physical envelope” and “already softened to 76% of requested force.”

### Required spec posture

Treat the tire-force model as a **replaceable empirical contact-force family**, not as immutable physics.

The contract should require:

| Contract item | Requirement |
|---|---|
| Inputs | Patch slip velocities, normal loads, drive-force request, patch geometry, calibrated tire parameters. |
| Outputs | `F_f,i`, `F_r,i`, `util_i`, force-limit diagnostics, and any saturation/stick-slip scheduling diagnostics. |
| Smoothness | Continuous force output and continuous utilization diagnostics over zero speed, in-place turns, and saturation transitions. |
| Units | Forces in newtons, slips in m/s, loads in newtons, friction coefficients dimensionless. |
| Diagnostics | `utilMax`, per-patch utilization, per-patch force request, force scale, and regime scalars. |
| Covariance hooks | Must expose variables used by process-noise scheduling and rolling pseudo-measurement gating. |

### First-pass implementation

Use the current architecture as the first-pass family:

1. Four contact patches.
2. Algebraic slip velocities from body state and encoder wheel-bank rates.
3. Drive-force feedforward plus passive longitudinal/lateral slip stiffness.
4. Stribeck-like friction envelope.
5. Smooth radial combined-slip limiter.

However, the first-pass limiter should be changed or explicitly redefined. The strongest first pass is a smooth radial clamp whose semantics are clear:

\[
forceScale_i \approx 1 \quad \text{for } util_i < 1
\]

\[
forceScale_i \approx \frac{1}{util_i} \quad \text{for } util_i > 1
\]

implemented through a smooth maximum, for example:

\[
forceScale_i = \frac{1}{smoothMaxE(1, util_i, e_{util})}
\]

or another smooth limiter with documented behavior at `util = 1`.

If the project intentionally keeps `tanh(util)/util`, the spec must rename the semantics: `util = 1` then means “onset of soft empirical saturation,” not “physical friction-envelope boundary.”

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Fitted `mu_peak` or `mu_slide` becomes implausibly high | The limiter is underusing the envelope or stiffness is confounded with friction. |
| High-speed arcs require large persistent `DeltaAr` and `DeltaYawAccel` | Mean lateral/yaw force model is not representing stick-slip average behavior. |
| In-place turns fit only by extreme yaw residuals | Scrub resistance or lateral patch force model is wrong. |
| Mid-speed rolling arcs show biased innovations despite low utilization | Low-slip stiffness or track/contact geometry is wrong. |
| Force utilization jumps near regime boundaries | Smooth model is numerically continuous but physically mis-scaled. |

### Follow-on options if the first pass fails

1. **Anisotropic friction ellipse fit**  
   Fit separate longitudinal and lateral effective limits and allow the ellipse axes to depend on slip speed or normal load. This directly addresses the likely weakness that four small solid rubber contact patches will not behave like an isotropic tire.

2. **Empirical force-surface map**  
   Replace the analytic Stribeck envelope with a smooth fitted map:

   \[
   (F_f,F_r)=F_{contact}(slip_f, slip_r, N, F_{driveReq}, side, patch)
   \]

   constrained to be continuous and monotone enough for UKF propagation. This addresses cases where springback and solid-rubber scrub do not match the analytic form.

3. **Regime-scheduled mean model for stick-slip arcs**  
   Keep the low-slip model for rolling, but allow a separate smooth mean-force surface for high-utilization arcs. This addresses the case where stick-slip has a repeatable average effect but not a deterministic event timing.

4. **Correlated residual scheduling**  
   If the contact mean model is acceptable but unmodeled slip events couple lateral and yaw acceleration, add cross-covariance between `DeltaAr` and `DeltaYawAccel` driving noise rather than adding new states.

### What not to lock down

Do not permanently lock the exact Stribeck equation, exact smooth limiter, or isotropic combined-slip geometry unless held-out logs prove they are representative. Lock down the interface, units, smoothness, diagnostics, and validation obligations.

---

## 2. Normal load model — P0

### Concern

Normal load directly sets force capacity. If patch loads are wrong, the friction envelope is wrong even if the tire-force law is well tuned. But the exact load-transfer algorithm should not be over-specified before logs show how much pitch, chassis compliance, wheel unloading, fan load, and ground strikes matter.

### Required spec posture

Treat normal load as a **flexible algebraic load-estimation family** with a mandatory contract.

The contract should require:

| Contract item | Requirement |
|---|---|
| Inputs | Static mass distribution, contact geometry, fan command, nominal body acceleration estimate, optional drive/saturation diagnostics. |
| Outputs | Per-patch nonnegative normal loads `N_i`, total-load diagnostic, load-transfer diagnostic, clamp/unload diagnostic. |
| Smoothness | Continuous loads across zero acceleration and regime boundaries. |
| Safety | No negative patch loads in the tire-force calculation. |
| Traceability | Log pre-clamp and post-clamp loads. |

### First-pass implementation

Use an algebraic model:

\[
N_i = N_{static,i}+N_{fan,i}+N_{longTransfer,i}+N_{latTransfer,i}
\]

First pass should use the **previous accepted nominal acceleration** or a **lagged nominal acceleration estimate**, not raw accelerometer measurements and not residual-corrected accelerations, to avoid feeding measurement contamination back into the deterministic tire model.

Recommended first pass:

1. Compute load transfer from previous-step nominal `Af_nom`, `Ar_nom`, and calibrated geometry.
2. Add fan/downforce as a calibrated algebraic term from `fanCommand`.
3. Smooth-clamp loads to a small positive `N_min`.
4. Log:
   - pre-clamp `N_i`,
   - post-clamp `N_i`,
   - total load before and after clamp,
   - whether `N_min` was active.

Total-load conservation should be treated as a diagnostic first, not an unconditional law after clamping. If `N_min` is active, forced renormalization can hide wheel-unloading behavior.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| High-speed arcs fit only with extreme friction coefficients | Load transfer is under/overestimated. |
| Full-bore straights show wrong acceleration limit before ground-strike clip | Longitudinal load-transfer model is wrong. |
| In-place turns show side-dependent yaw residual not explained by command/rate | Left/right or front/rear load split is wrong. |
| `N_min` active often in ordinary maneuvers | Geometry, load-transfer scale, or clamp floor is wrong. |
| Fan command changes behavior but load model cannot explain it | Fan/downforce map is wrong or too slow. |

### Follow-on options if the first pass fails

1. **One-step fixed-point load transfer**  
   Use nominal forces to recompute acceleration and update load once. This addresses first-pass lag if load transfer strongly affects force capacity inside the same tick.

2. **Fitted load-transfer scale factors**  
   Keep the algebraic form but fit longitudinal and lateral transfer gains separately. This addresses uncertain effective CoG height, chassis compliance, and wheel/tire contact compliance without adding pitch/roll states.

3. **Regime-specific load approximation**  
   Use one smooth load-transfer family for rolling arcs and another for full-bore straight launch/braking. This addresses the case where ground-strike-limited straight behavior is not well represented by the same transfer law used for arcs.

4. **Fan/downforce response model**  
   Replace static `F_fan(fanCommand)` with a calibrated command-to-load curve or low-order response if logs show fan load is not instantaneous or not repeatable.

### What not to lock down

Do not lock previous-step acceleration, one-step fixed point, or strict post-clamp load conservation as the only valid implementation. Lock down nonnegative loads, input/output semantics, smoothness, logged diagnostics, and validation against held-out maneuvers.

---

## 3. Motor-driver command-to-torque model — P0

### Concern

The command-to-torque path anchors launch, braking, in-place scrub torque, current limiting, and drive-authority scheduling. The latest spec correctly excludes live `Vbat` from the sample contract and moves bus-voltage dependence into calibrated command-to-torque modeling. That is the right direction. The remaining risk is leaving `T_cmdMap` under-specified.

### Required spec posture

Treat the actuator model as a **flexible empirical torque model** with a hard interface.

The model should expose:

| Output | Meaning |
|---|---|
| `T_bankRaw,j` | Mean wheel-bank torque for side `j`. |
| `sigma_T,j` or equivalent | Torque uncertainty used by process-noise scheduling. |
| `driveSaturationIndex` | Dimensionless authority/saturation proxy. |
| `driveAuthorityUncertainty` | Dimensionless uncertainty proxy for bus sag, current regulation, thermal drift, and unmodeled driver behavior. |
| Optional phase/ripple diagnostics | Gear/ripple repeatability by `motorPhase`. |

### First-pass implementation

The first pass should be a fitted command/rate/phase torque map:

\[
T_{bankRaw,j}=T_{cmdMap,j}(u_j, motorRate_j, motorPhase_j)
\]

with `motorPhase` optional until there is enough data to fit ripple.

Minimum first-pass map:

\[
T_{bankRaw,j}=T_{cmdMap,j}(u_j, motorRate_j)
\]

where the map is fitted separately for left and right banks and includes:

1. command normalization,
2. back-EMF-like rate dependence,
3. launch/breakaway behavior,
4. current-limit/saturation behavior,
5. bank asymmetry,
6. uncertainty output.

The voltage-equivalent motor equation should remain as a prior or fallback, not the authoritative model once logs are available.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Straight launch requires persistent `DeltaAf` | Torque map or launch threshold is wrong. |
| In-place turns require persistent `DeltaYawAccel` even after scrub tuning | Differential torque or current limiting is wrong. |
| Similar commands behave differently over run duration | Thermal drift, battery sag, or driver heating is not represented. |
| Direction changes create biased innovations | Brake/coast/decay mode or command-validity phase is wrong. |
| Motor phase correlates with residuals | Gear/ripple term needs to be enabled. |

### Follow-on options if the first pass fails

1. **Hybrid electrical/empirical map**  
   Use the voltage-equivalent model as a structured prior and fit correction terms:

   \[
   T_{bank}=T_{electricalPrior}+T_{correction}(u,motorRate,motorPhase)
   \]

   This addresses cases where back-EMF behavior is broadly correct but driver/battery effects are not.

2. **Saturation-aware torque surface**  
   Add a map output for saturation/authority rather than inferring it from the raw torque alone. This directly addresses DRV8871 current regulation and unmeasured bus sag.

3. **Thermal/run-time compensation**  
   Add run-time, driver-temperature proxy, or recent-current proxy as a map input if logs show drift over the run.

4. **PWM decay-mode branch**  
   If brake/coast behavior changes torque response materially, split the map by PWM mode or add mode as an input.

### What not to lock down

Do not force the final actuator model to be `u * VbusEff` plus algebraic current limiting. That decomposition may be useful as a prior, but the physical robot may be better represented by a directly fitted torque map. Lock down the model contract and validation behavior.

---

## 4. Ground-strike / acceleration envelope — P0

### Concern

Full-bore straight acceleration is explicitly limited by ground strikes. But ground strike is not merely “longitudinal force saturation.” It may include pitch load transfer, nose/tail contact, chassis rocking, vertical impulses, transient load redistribution, and accelerometer contamination.

The current spec risks mixing two interpretations: it clips `Af`, scales longitudinal contact forces, says yaw moment should be recomputed if scaling is material, but then still assigns final yaw acceleration from the raw yaw term. That inconsistency matters more than the exact first-pass model choice.

### Required spec posture

Treat ground strike as a **flexible acceleration/impact approximation family** with an internal-consistency requirement.

The contract should require:

| Contract item | Requirement |
|---|---|
| Primary effect | Represents forward/reverse acceleration limiting and impact uncertainty. |
| Scope | Applies primarily to straight longitudinal acceleration/braking, not automatically to clean in-place turns. |
| Diagnostics | `Af_raw`, `Af_clip`, `groundUse`, impact/springback flags, and covariance inflation triggers. |
| Consistency | If forces are mutated, all dependent moments and accelerations must be recomputed. If forces are not mutated, the model must state it is an acceleration-envelope approximation. |

### First-pass implementation

Use a **pure acceleration-envelope approximation** first:

\[
Af_{nom}=clipAsymE(Af_{raw},-Af_{reverseLimit},Af_{forwardLimit})
\]

Do not mutate contact forces in the first pass. Keep:

\[
Ar_{nom}=Ar_{raw}
\]

\[
yawAccel_{nom}=yawAccel_{raw}
\]

and use:

\[
groundUse=\frac{|Af_{raw}-Af_{clip}|}{|Af_{raw}|+\epsilon}
\]

for residual-process-noise and accelerometer-covariance scheduling.

This is not claiming that physical ground strike leaves yaw unchanged. It is a deliberate first-pass approximation: forward acceleration is clipped as an envelope; lateral/yaw terms are left to the contact model and residual/noise handling.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Full-bore straights have repeatable yaw bias during clipping | Ground strike couples into yaw or left/right load distribution. |
| Accelerometer innovations spike but pose remains acceptable | Event/noise treatment is adequate, deterministic strike model may not be needed. |
| Acceleration envelope clips too early/late depending on battery or speed | Limit needs command/rate/load dependence. |
| In-place turns show strike-like transients | Treat as impact/springback unless logs show deterministic coupling. |

### Follow-on options if the first pass fails

1. **Force-consistent clipping**  
   Scale or redistribute contact forces, then recompute:

   \[
yawMoment=\sum_i(f_iF_{r,i}-r_iF_{f,i})
   \]

   and recompute yaw acceleration. This addresses repeatable yaw coupling during clipped full-bore acceleration.

2. **Event/noise model**  
   Keep the acceleration envelope but strengthen impact detection and covariance inflation. This addresses non-repeatable floor-strike impulses without pretending to model exact contact timing.

3. **Command/rate-dependent envelope**  
   Make `Af_forwardLimit` and `Af_reverseLimit` smooth functions of speed, command, or inferred load. This addresses cases where the limit is not constant.

4. **Algebraic pitch-risk proxy**  
   Add a non-state pitch-risk scalar used only for clipping/noise scheduling. This addresses pitch load transfer without adding pitch states to the 9-state UKF.

### What not to lock down

Do not force deterministic force scaling as the only ground-strike implementation. Also do not allow a mixed implementation that mutates forces but keeps stale yaw acceleration. The spec should allow either an acceleration-envelope model or a force-level model, but it must require internal consistency.

---

## 5. Residual acceleration states — P0

### Concern

The residual states are essential because the robot intentionally operates in regimes where an exact deterministic model is unrealistic: scrubbed in-place turns, high-speed stick-slip arcs, gear variation, springback, and ground-strike events. The risk is two-sided:

1. Too little residual flexibility makes the spec reject a physically representative implementation.
2. Too much unexamined residual flexibility lets the residuals hide a bad nominal plant.

### Required spec posture

Residuals should be treated as **structured model slack with diagnostics**, not as arbitrary correction states to minimize at all costs.

### First-pass implementation

Keep the current exact discrete OU residual model and scheduled steady-state residual sigmas. This is a strong first-pass stochastic contract:

\[
\Delta q_{k+1}=\phi_q\Delta q_k+\eta_q
\]

\[
Q_{\Delta q}=\sigma_{\Delta q,ss}^2(1-\phi_q^2)
\]

Use regime scheduling as already specified for utilization, stick-slip, in-place blend, ground use, impact suspicion, drive authority, and encoder validity.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Residuals are persistently biased in constant-speed straight motion | Nominal motor/friction/geometry fit is wrong. |
| Residuals are large only during stick-slip arcs but innovations are consistent | The model may be valid; aggressive regimes need stochastic slack. |
| Residuals remain active after a maneuver ends | OU time constant or regime schedule is too slow. |
| Residuals hide wall/IMU disagreement | Measurement covariance/gating may be wrong. |
| Residuals dominate all fitted maneuvers | Nominal actuator/contact model is too weak. |

### Follow-on options if the first pass fails

1. **Residual schedule reallocation**  
   If residuals are too active in benign regimes but needed in arcs, adjust regime scheduling rather than imposing hard global residual caps.

2. **Correlated residual driving noise**  
   If `DeltaAr` and `DeltaYawAccel` move together in stick-slip arcs, add cross-correlation in residual driving noise. This targets a real physical coupling without adding states.

3. **Nominal-model repair trigger**  
   If residuals are persistently biased in benign regimes, the spec should direct the implementer to repair motor, tire, normal-load, or geometry parameters before increasing residual noise.

4. **Regime-specific residual diagnostics**  
   Version expected residual RMS/mean by maneuver class. Do not apply the same residual expectations to a constant-speed straight and a high-speed stick-slip arc.

### What not to lock down

Do not impose hard residual magnitude caps as correctness criteria for aggressive maneuvers. Use diagnostics, held-out validation, and regime-specific expectations.

---

## 6. Calibration and validation — P0

### Concern

Validation needs to prevent both failure modes:

1. A rigid but wrong first-pass model being treated as authoritative.
2. A vague residual-heavy model fitting logs for the wrong reasons.

The current validation section is directionally right but still mostly qualitative.

### Required spec posture

The spec should require **metrics and held-out test classes**, while making thresholds **project-versioned and evidence-based**.

### First-pass implementation

The first-pass validation suite should record at least:

| Metric | Purpose |
|---|---|
| NIS by measurement type and regime | Detect overconfidence/underconfidence. |
| Innovation autocorrelation | Detect unmodeled dynamics and phase errors. |
| Residual mean/RMS by regime | Detect nominal-plant weakness. |
| Held-out replay pose/yaw error | Prevent overfit to tuning logs. |
| Contact/tire diagnostics | Identify force-law failure. |
| Load/ground-use diagnostics | Identify load-transfer and strike-model failure. |
| Drive-authority diagnostics | Identify torque-map failure. |
| Wall-edge innovation behavior | Validate wall response/gating. |

Initial thresholds may be labeled as warning thresholds, not final pass/fail values, until baseline characterization exists.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Fit logs pass but held-out logs fail | Overfit or confounded parameters. |
| NIS too low in aggressive regimes | Covariance overinflated, residuals hiding model error. |
| NIS too high in benign regimes | Measurement model or nominal plant wrong. |
| Innovations phase-shifted relative to commands | Timing convention or command-validity error. |
| Residuals carry systematic maneuver-dependent corrections | Nominal submodel needs repair or replacement. |

### Follow-on options if the first pass fails

1. **Versioned acceptance bands**  
   Establish thresholds after a baseline log corpus. Store them with robot hardware revision, tire condition, firmware revision, and estimator version.

2. **Ablation replays**  
   Replay with residual noise reduced, rolling pseudo-measurement disabled, wall updates disabled, and accelerometer updates weakened. This identifies which model block is carrying the fit.

3. **Fit-order constraints**  
   Fit actuator and encoder semantics before tire/contact parameters; fit low-slip behavior before stick-slip behavior; fit wall sensors separately from drivetrain dynamics.

4. **Holdout-by-regime**  
   Require held-out data for in-place turns, straight launch, mid-speed arcs, high-speed arcs, wall-edge passes, and ground-strike events.

### What not to lock down

Do not hard-code final numeric thresholds before the robot is characterized. Lock down the metrics, test classes, reporting format, and requirement that thresholds become versioned once baseline data exists.

---

## 7. Flexible-model declaration pattern — P0

### Concern

The current document mixes hard contracts and empirical first-pass equations without always labeling which is which. That invites both bad outcomes: over-specification of unproven physics and under-specification of model-critical behavior.

### Required spec posture

Add a standard subsection to every empirical submodel:

```text
Model status: flexible empirical model family
Hard contract:
First-pass implementation:
Expected failure signatures:
Permitted follow-on options:
Required diagnostics:
Promotion / replacement rule:
```

### First-pass implementation

Apply this pattern immediately to:

| Section | Why |
|---|---|
| 8 — Motor/driver algebra | Hardware behavior and unmeasured bus effects are empirical. |
| 10 — Normal load model | Load transfer and fan/downforce are approximate. |
| 11 — Tire-force model | Solid rubber contact and stick-slip are empirical. |
| 12 — Ground-strike envelope | Strike behavior is event-like and not purely planar. |
| 12 / 14.1 — Yaw scrub | Scrub resistance may be in-place-only or global. |
| 19 — Wall sensors | IR/log-amp response may not be purely geometric. |

### Follow-on option if the first pass fails

Promote the best-performing first-pass block to a versioned model family only after held-out logs show it works. Do not delete the follow-on path merely because the first pass fits one log set.

---

## 8. Parameter table and identifiability — P1

### Concern

The document has a glossary, but not a parameter-identification contract. Many parameters are confounded: friction coefficients, stiffnesses, effective mass, yaw inertia, yaw scrub terms, torque-map terms, normal-load gains, and residual schedules can all explain overlapping behavior.

### Required spec posture

Use a parameter table with soft priors and fitting rules.

### First-pass implementation

Each fitted parameter should have:

| Field | Requirement |
|---|---|
| Symbol/code name | Exact identifier. |
| Unit | SI unit or dimensionless. |
| Prior | Initial value and source. |
| Bound type | Hard physical bound or soft calibration prior. |
| Fit maneuver | Which logs are allowed to fit it. |
| Validation maneuver | Which held-out logs validate it. |
| Residual policy | Whether residuals are enabled, weakened, or disabled during fitting. |
| Confounds | Parameters that can mimic its effect. |

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Multiple parameter sets fit equally well | Identifiability collapse. |
| Parameters hit hard bounds often | Bounds are wrong or model family is wrong. |
| Residuals shrink but held-out behavior worsens | Overfit. |
| Fitted values violate obvious physical scale | Wrong model decomposition. |

### Follow-on options if the first pass fails

1. **Reparameterize confounded groups**  
   Fit aggregate quantities first, such as effective bank torque or effective lateral stiffness, then decompose only if data supports it.

2. **Lock sequential fit order**  
   Fit encoder/timing, then actuator, then low-slip contact, then yaw scrub, then stick-slip and ground-strike schedules.

3. **Use soft bounds by default**  
   Convert non-safety bounds into priors with penalties. Hard bounds should be reserved for impossibility or hardware safety.

---

## 9. In-place yaw scrub and `yawKineticMoment` scope — P1

### Concern

The physical notes identify roughly constant turning resistance in in-place turns. The current equation applies `yawKineticMoment` globally unless its fitted value is effectively zero. That can double-count yaw resistance in rolling arcs where combined-slip tire forces already model lateral/yaw behavior.

### Required spec posture

The spec must define the physical interpretation of each yaw-loss term.

### First-pass implementation

Best first pass:

1. Treat `yawBreakawayMoment` as static in-place scrub threshold.
2. Treat `yawKineticMoment` as moving in-place scrub resistance.
3. Gate both primarily by `inPlaceBlend` or a related scrub blend.

For example:

\[
yawMoment = yawMomentRaw
+ inPlaceBlend\,(yawMomentBreakaway - yawMomentRaw)
- inPlaceBlend\,yawKineticMoment\,sgnE(yawRate,yawRate_E)
\]

This matches the stated importance of calibrated in-place turns and avoids imposing a global yaw drag on rolling arcs.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Rolling arcs under-damp yaw | Some global yaw loss may be needed. |
| In-place turn onset wrong | Breakaway threshold wrong. |
| In-place steady yaw rate wrong | Kinetic scrub term wrong. |
| Same term improves in-place turns but hurts arcs | Term interpretation is confounded. |

### Follow-on options if the first pass fails

1. **Separate global yaw drag from scrub loss**  
   Keep `yawKineticScrubMoment` gated by in-place blend and add a separate small `yawDragMoment` or viscous yaw damping if rolling arcs require it.

2. **Utilization-scheduled yaw loss**  
   Schedule yaw resistance by `utilMax` and `inPlaceBlend` if data shows scrub-like loss also appears in high-utilization arcs.

3. **Patch-level scrub correction**  
   Move yaw loss back into the tire/contact force model if a separate yaw moment term causes confounding.

### What not to lock down

Do not force `yawKineticMoment` to be global or scrub-only without declaring the interpretation. The first pass should be scrub-gated, but the spec should allow evidence-driven introduction of a separate global yaw-loss term.

---

## 10. Accelerometer measurement semantics — P1

### Concern

The accelerometer is the main measurement that checks the predicted acceleration path. Ambiguity here can make residual states learn pitch/roll contamination, bias conventions, or filter phase error as if it were planar force.

### Required spec posture

The measurement contract should be hard. The calibration implementation can be flexible.

### First-pass implementation

Define `z_Af` and `z_Ar` as canonical body-frame planar accelerometer samples after hardware-layer sign correction and scale calibration, with the bias convention explicitly stated.

The spec should choose one of these contracts:

1. **Bias-included measurement contract**  
   `z_Af/z_Ar` include calibrated residual bias, and the measurement function adds `accelBiasFCal/RCal`.

2. **Bias-removed measurement contract**  
   `z_Af/z_Ar` are already bias removed, and the measurement function does not add bias.

Do not leave both interpretations possible in implementation.

First pass should keep the existing IMU-location acceleration model:

\[
a_{imu,f}=Af-yawAccel\,r_{imu}-yawRate^2f_{imu}
\]

\[
a_{imu,r}=Ar+yawAccel\,f_{imu}-yawRate^2r_{imu}
\]

Use HAODR low-pass accelerometer outputs for the planar measurement. Use high-pass/slope outputs only for event detection and covariance inflation.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Persistent acceleration innovation bias while stationary | Bias convention or calibration wrong. |
| Innovation phase lag during aggressive motion | IMU filter/timestamp convention wrong. |
| Large accelerometer innovation during ground strike but gyro/wall remain plausible | Planar model should not absorb vertical/pitch impulse. |
| Residuals learn static gravity projection | Pitch/roll or mounting calibration is contaminating planar channels. |

### Follow-on options if the first pass fails

1. **Estimator-side in-plane misalignment projection**  
   Use calibrated planar unit vectors for reported accelerometer axes if hardware-layer alignment is insufficient.

2. **Filter-phase compensation**  
   Add timing covariance or phase correction tied to the configured LPF/HAODR path if innovations show lag.

3. **Regime-specific accelerometer covariance**  
   Strengthen covariance inflation during ground strike, wall touch, springback, and high jerk rather than adding pitch/roll states prematurely.

### What not to lock down

Do not prescribe one hardware calibration procedure unless it is part of the measurement contract. Lock down what the estimator receives and how covariance handles known planar-model violations.

---

## 11. Wall-sensor measurement model — P1

### Concern

The wall sensors are custom IR sensors, including log-amp front sensors. A purely geometric range model may be too rigid. But leaving wall response vague is also unsafe because wall updates can dominate position correction.

### Required spec posture

Treat wall sensing as a **response-space measurement family** with a hard geometric/extrinsic contract.

### First-pass implementation

Use the current first-pass structure:

1. Calibrated sensor origin and look unit vector.
2. Ray bundle through calibrated angular support.
3. Soft-min distance to a valid wall hypothesis.
4. Response-space prediction:
   - distance output if calibrated distance is used,
   - fitted log/raw response curve if raw/log-amplified values are used.
5. Skip or assign extremely large covariance when no valid wall hypothesis exists.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Innovations grow near wall edges/corners | Ray bundle, soft-min, or edge covariance insufficient. |
| Front sensors saturate or flatten but updates remain strong | Response-space covariance is wrong. |
| Multiple sensors viewing same wall produce overconfident correction | Shared wall-geometry correlation is unmodeled. |
| Open-floor samples create false corrections | No-hit handling or wall-hypothesis gating is wrong. |

### Follow-on options if the first pass fails

1. **Empirical response maps**  
   Replace analytic log/inverse-power curves with fitted response tables over distance and incidence.

2. **No-hit likelihood model**  
   Explicitly model no-wall/no-hit response rather than only skipping updates.

3. **Shared-geometry covariance inflation**  
   Inflate or correlate measurements when multiple sensors view the same wall edge or corner.

4. **Edge-aware gating**  
   Add covariance terms based on distance to wall segment endpoints and posts.

### What not to lock down

Do not make ray-bundle soft-min the only permitted wall model. It is a good first pass, but the spec should permit empirically fitted response-space models if calibration data shows geometry alone is insufficient.

---

## 12. Covariance propagation and schedule evaluation — P1

### Concern

This is a place where strictness is not over-specification. If residual and encoder-input uncertainty do not propagate through the plant, the UKF is structurally inconsistent regardless of the physical submodel details.

### Required spec posture

The requirement should remain hard:

1. Residual OU noise must propagate into full state covariance.
2. Encoder wheel-rate covariance must enter before motor, slip, tire, utilization, scheduling, and rolling pseudo-measurement calculations.
3. Diagonal-only residual-state covariance injection is not compliant.

### First-pass implementation

Use one of the two current methods:

1. augmented prediction noise, or
2. sensitivity-mapped additive covariance.

The best first implementation for bring-up is sensitivity mapping because it is easier to instrument and compare. The best final implementation may be augmented sigma-point prediction if computational budget and UKF implementation allow it.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| `DeltaAf` noise does not increase `Vf/Px/Py` covariance | Residual noise is being injected only into residual states. |
| Encoder covariance does not affect utilization scheduling near thresholds | Input uncertainty is injected too late. |
| Changing timestep changes residual RMS | OU discretization or scheduling is wrong. |
| Covariance remains diagonal-like after prediction | Cross-covariances are being lost. |

### Follow-on options if the first pass fails

1. **Move from sensitivity mapping to augmented sigma points**  
   Use when nonlinear uncertainty through tire/contact scheduling is too strong for local finite differences.

2. **Integrated OU noise treatment**  
   If endpoint residual innovation timing distorts velocity/yaw covariance, use a propagation method that better approximates integrated residual-driving noise over the interval.

3. **Conservative schedule inflation**  
   If sigma-point-local scheduling is not implemented, use predicted-mean scheduling plus conservative inflation.

### What not to lock down

Do not force exactly one numerical method. Lock down the covariance effects that must appear in the predicted state distribution.

---

## 13. Timing and command-validity convention — P1

### Concern

At 1 kHz, a one-tick phase error can corrupt torque-map fitting, accelerometer comparisons, and wheel-rate input interpretation. This is not a low-level acquisition detail once it affects the measurement and plant model.

### Required spec posture

Timing semantics should be hard at the estimator interface, but the scheduler/readout implementation should remain out of scope.

### First-pass implementation

Require the tick packet to carry or imply:

1. effective timestamp for prediction,
2. IMU sample effective time/phase,
3. encoder interval time and phase,
4. wall-sensor sample phase,
5. command-validity convention:
   - command active over interval ending at timestamp, or
   - command issued at timestamp for next interval.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Innovations correlate with command derivative | Command/measurement phase error. |
| Torque map differs between replay and live logs | Timestamp convention mismatch. |
| Encoder-derived rates align poorly with IMU acceleration | Encoder sample interval or phase wrong. |

### Follow-on options if the first pass fails

1. **Timing covariance term**  
   Add measurement timing covariance:

   \[
R_{timing}\approx \sigma_t^2 \dot{h}\dot{h}^T
   \]

2. **Midpoint command integration**  
   Use stored previous command and next command to improve interval-level torque approximation.

3. **Offline phase fit**  
   Fit command, encoder, and IMU phase offsets from logged excitation before locking torque/contact parameters.

### What not to lock down

Do not prescribe SPI timing, ADC order, or interrupt details. Lock down effective sample timing and command-validity semantics.

---

## 14. High-speed stick-slip arcs — P1

### Concern

The project expects high-speed arcs to exceed conventional rolling traction and require stick-slip. The model should not pretend to deterministically predict every stick-slip event, but it also should not let residual noise become the entire arc model.

### Required spec posture

Use a smooth mean contact model plus scheduled process noise as first pass, with explicit criteria for when to improve the mean model.

### First-pass implementation

Keep:

1. Stribeck/combined-slip mean force,
2. `stickSlipIndex`,
3. elevated `DeltaAr` and `DeltaYawAccel` process noise,
4. weaker rolling pseudo-measurement,
5. more conservative wall gating.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Arcs remain stable only because residual noise is huge | Mean contact model is not capturing average arc behavior. |
| Same arc command produces repeatable lateral/yaw bias | Upgrade the mean model, not just Q. |
| Wall updates are repeatedly rejected in arcs | Wall covariance/gating or motion prediction is wrong. |
| Residuals do not decay after arc exit | Schedule transition or OU time constant wrong. |

### Follow-on options if the first pass fails

1. **Stick-slip mean-force correction**  
   Add a smooth correction surface active at high `stickSlipIndex`.

2. **Arc-regime parameter set**  
   Allow separate effective lateral stiffness/friction terms for high-speed arcs, smoothly blended with rolling terms.

3. **Coupled residual covariance**  
   Add correlation between lateral and yaw residual driving noise if logs show coupled slip impulses.

### What not to lock down

Do not require the first-pass tire law to explain all stick-slip behavior deterministically. Also do not allow unlimited residual noise to substitute for a mean model when repeatable arc behavior exists.

---

## 15. Encoder input semantics and optional rolling pseudo-measurement — P2

### Concern

The core semantic choice is correct: encoders measure drivetrain motion, not body motion. The optional rolling pseudo-measurement is useful only when it remains a weak, gated consistency check.

### Required spec posture

Encoder-as-input semantics should be hard. Rolling pseudo-measurement geometry should be flexible.

### First-pass implementation

Use the current optional pseudo-measurement only in benign rolling regimes:

\[
h_{roll,L}=r_w\hat{wheelRate}_L-\left(Vf+\frac{b}{2}yawRate\right)
\]

\[
h_{roll,R}=r_w\hat{wheelRate}_R-\left(Vf-\frac{b}{2}yawRate\right)
\]

Include `Romega` in `R_roll,total`.

### Likely first-pass failure signatures

| Failure signature | Interpretation |
|---|---|
| Pseudo-measurement fights gyro in turns | Rolling geometry too strong or enabled in slip. |
| Hard launch becomes overconstrained | Covariance lacks launch/drive-saturation terms. |
| Four-patch plant disagrees with `b/2` geometry | Effective rolling track width differs from nominal. |

### Follow-on options if the first pass fails

1. **Effective calibrated rolling track width**  
   Replace `b/2` with fitted `b_eff/2` for low-slip rolling only.

2. **Contact-weighted rolling geometry**  
   Derive the pseudo-measurement from the four contact patches and normal-load weights.

3. **Stronger regime covariance terms**  
   Add launch, ground-use, drive saturation, current limiting, and high wheel-rate slew terms to `R_roll`.

### What not to lock down

Do not allow the pseudo-measurement to become a hard differential-drive truth. It should remain optional, weak, and disabled during scrub/stick-slip.

---

## 16. External stationary gyro-bias estimator — P2

### Concern

The external bias estimator is appropriate given the claimed within-run stability, but operational thresholds should be calibrated from logs.

### Required spec posture

Keep `gyroBiasYawExt` outside the UKF, but define the stationary-update contract and lockouts.

### First-pass implementation

Use stationary updates only when:

1. gyro yaw rate is below threshold,
2. encoder rates are below threshold,
3. planar accelerometer channels are stable,
4. vertical acceleration/jerk is not impact-like,
5. no recent launch, wall touch, springback, or in-place scrub event is active.

### Follow-on options if the first pass fails

1. **Temperature/run-time bias schedule**  
   If bias changes over the run, fit a temperature or run-time bias correction outside the UKF.

2. **Heading covariance injection**  
   If residual bias uncertainty is material, inject coherent heading covariance over time without adding a UKF state.

3. **Reconsider gyro-bias state only with evidence**  
   Add a state only if within-run drift becomes comparable to the heading error budget and cannot be handled externally.

---

## 17. Smooth helper functions — P3

### Concern

Smooth helper functions are implementation tools. They should not become physical laws.

### Required spec posture

Require continuity, differentiability where needed, numerical stability, and correct limiting behavior. Permit replacement of specific functions if the replacement preserves the contract.

### First-pass implementation

Keep the existing smooth sign, smooth saturation, softplus, smooth deadzone, smooth step, smooth max/min, and asymmetric clipping functions.

### Follow-on options if the first pass fails

1. Replace `tanh`-based smooth steps with polynomial smoothsteps if transition tails are too long.
2. Replace softplus with a numerically safer branch implementation.
3. Replace smooth max/min sharpness per model block if one global epsilon causes scaling problems.

---

# Recommended structural edit to the spec

Add the following subsection template to the beginning of every flexible empirical submodel:

```markdown
### Model-family contract

**Status:** Flexible empirical model family. The first-pass implementation below is the project’s current best guess. It is not a permanent physical law.

**Hard contract:**
- Inputs:
- Outputs:
- Units/signs:
- Required smoothness:
- Required diagnostics:
- Required covariance hooks:

**First-pass implementation:**
- Formula or fitted-map form:
- Initial priors:
- Required logged fields:

**Expected failure signatures:**
- Innovation patterns:
- Residual patterns:
- Parameter-fit pathologies:
- Regime-specific symptoms:

**Follow-on options:**
1. Targeted alternative model:
2. What failure it addresses:
3. Required additional calibration data:

**Promotion rule:**
- Evidence required to replace the first-pass implementation:
- Held-out tests that must improve:
```

This template should be mandatory for Sections 8, 10, 11, 12, 14.1, and 19.

# Bottom line

The strongest foundation for this project is not maximal formula lock-down. It is a model-family specification with hard contracts where ambiguity would break the estimator and explicit flexibility where the robot’s physical behavior is empirical.

The spec should be strict about:

- state vector, units, signs, and coordinate convention;
- encoder-as-drivetrain-input semantics;
- measurement semantics;
- timing/effective-sample semantics;
- covariance propagation requirements;
- validation metrics and logging obligations.

The spec should be flexible but not vague about:

- tire/contact force law;
- normal-load/load-transfer approximation;
- command-to-torque model;
- ground-strike model;
- yaw scrub/global yaw-loss decomposition;
- wall-sensor response model;
- residual schedule thresholds.

For every flexible element, the spec should name the best first pass and at least one targeted path forward. That is the key discipline: **flexibility must be engineered, not left implicit.**
