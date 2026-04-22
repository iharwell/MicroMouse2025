# SR-UKF / Plant Model Rework Specification for Micromouse — Final Cost-Constrained Single-Filter 9-State Revision (V6.2)

## Purpose

This document defines the recommended **final cost-constrained** UKF / plant-model swap for the Micromouse project.

It supersedes V6.1 by preserving the strongest single-filter robustness changes while explicitly constraining the online implementation to remain in roughly the same runtime envelope as the original stack:

- about **30% of the loop tick for predict**,
- about **20% of the loop tick for updates**,
- and no hidden worst-case edge-mode path that materially exceeds that budget.

The design objective is:

1. materially improve robustness and precision in the regimes that are currently breaking the stack,
2. remain computationally realistic for the present 1 kHz Teensy deployment,
3. preserve a physically meaningful control / inverse / audit plant,
4. avoid another architectural reset after this rewrite,
5. improve contamination resistance during slip onset and re-grip,
6. improve post-peak and recovery behavior without moving to a multi-filter supervisor, and
7. hold online cost close to the original implementation envelope.

This revision keeps the strongest parts of the V4 / V6.1 direction:

- the single-filter 9-state backbone,
- estimator-correct wheel/body authority separation,
- the exact stationary branch,
- raw gated planar-accelerometer reintegration,
- the applied-torque process input,
- precursor saturation metrics,
- per-bank transient-contact memory,
- and explicit re-grip dwell.

It changes three cost-critical points:

- the **square-root spherical-simplex sigma set** becomes a base feature,
- **online predictor substepping is removed from the base runtime path**,
- and the **edge tire behavior is implemented as a lightweight shape modifier of the grip law**, not as a full second hot-path tire-law evaluation.

---

## Executive verdict

The recommended V6.2 architecture is:

- a **single 9-state square-root UKF**,
- with a **bank-split physical process model**,
- a **separate smooth `EstimatorPredictModel`** for sigma-point propagation,
- **strict wheel/body authority separation**,
- **exact stationary branching**,
- **raw synchronized planar-accelerometer support with staged online rollout**,
- **closure and adaptive lateral pseudo-measurements** in place of wheel-owned body overwrite,
- a **frozen once-per-cycle adaptive schedule**,
- a **runtime robustification layer** based on innovation consistency, onset holdoff, dwell, and re-grip recovery,
- a **bank-level friction-utilization precursor metric**,
- a **bounded per-bank transient-contact memory term**,
- a **single embedded grip-law family with bounded edge-shape modification**,
- and a **base square-root spherical-simplex sigma set**.

The online tire/contact law for both the estimator and the forward plant shall be standardized on:

- a **low-speed-regularized modified Fiala / Dugoff-class grip law**,
- smooth combined-slip competition,
- at least quasistatic load transfer,
- and a **lightweight bounded edge-shape modifier** driven by held bank memory and edge evidence.

A full IMM is **not** adopted in V6.2.
A learned residual tire model is **not** adopted in V6.2.
Added propagated tire-state dimensions are **not** adopted in V6.2.
Online predictor substepping is **not** part of base runtime acceptance in V6.2.

That combination is the strongest final single-filter rewrite I would recommend under the stated robustness goals and runtime budget.

---

# Part I — Core architecture

## 1. State definition

Retain the 9-state vector:

```text
x = [p_x, p_y, ψ, u, v, r, ω_L, ω_R, b_gz]^T
```

with heading normalized to `(-π, π]`.

### Why this state remains final

This remains the correct 9-state budget because it preserves the two quantities that matter most under real failure:

- left-bank wheel speed,
- right-bank wheel speed.

Those states are what allow the estimator to remain coherent when one side or both sides depart from body motion.

## 2. Authority partition

Partition the state into:

```text
x_nav = [p_x, p_y, ψ, u, v, r, b_gz]^T
x_wh  = [ω_L, ω_R]^T
```

Outside exact stationary lock:

- direct wheel-speed measurements own **only** `ω_L` and `ω_R`,
- body states shall **not** be directly overwritten from wheel information,
- wheel information may affect body states only through explicit, auditable estimator paths.

Those paths are:

1. closure pseudo-measurements,
2. adaptive lateral pseudo-measurement,
3. process / measurement scheduling,
4. slip-onset holdoff,
5. re-grip recovery logic,
6. innovation-consistency robustification,
7. optional benign-motion soft odometry when timing and operating conditions allow.

### Mandatory implementation rule

The direct wheel-speed update shall be implemented with a constrained or Schmidt-style gain structure enforcing:

```text
K_nav,ω = 0
```

for the direct wheel-speed update.

This is mandatory.

---

# Part II — Plant and estimator predict model

## 3. Split the control plant from estimator propagation

The project shall use **two closely related models**, not one overloaded implementation.

### 3.1 `PlantModel` (control / inverse / audit model)

The retained `PlantModel` remains authoritative for:

- feedforward / inverse use,
- technical limits,
- offline replay and audit,
- identification telemetry,
- and physically meaningful forward prediction for the controller side.

### 3.2 `EstimatorPredictModel` (UKF propagation model)

Add a dedicated `EstimatorPredictModel` for sigma-point propagation.

It uses:

- the same 9 states,
- the same geometry,
- the same bank-torque input contract,
- the same wheel-bank dynamics,
- the same lever-arm-aware IMU prediction contract,
- the same grip-law family,
- and the same held schedule / memory inputs,

but it removes estimator-hostile discontinuities and controller-oriented convenience logic.

### 3.3 What the estimator predict model shall not do

The moving estimator branch shall not depend on:

- motion-weight stop blending,
- snap-to-zero cleanup,
- hard Coulomb sign flips inside propagation,
- piecewise-sharp slip saturation corners,
- hidden state overwrite,
- post-step body cleanup.

The estimator moving branch shall be continuous and as smooth as practical around:

- zero wheel speed,
- zero slip,
- static-to-rolling crossover,
- tire-force saturation onset,
- post-peak force softening,
- slip recovery,
- re-grip.

## 4. Final online tire and contact law

The V6.2 online tire/contact law shall be a **single embedded grip-law family with bounded edge-shape modification**.

### 4.1 Grip branch

The base grip law shall be:

- pure-slip modified Fiala-style or Dugoff-class longitudinal and lateral force laws,
- with low-speed regularization,
- followed by smooth combined-slip competition,
- with at least quasistatic longitudinal and lateral load transfer.

### 4.2 Edge-shape modifier

The edge behavior shall **not** require a full second hot-path tire-law evaluation in the base online runtime.

Instead, the estimator and forward plant shall reuse the same core grip-law intermediates and then apply a bounded, smooth modifier that can produce:

- softened build-up near saturation,
- broader peak / plateau behavior,
- gentler post-peak decay,
- delayed force restoration after re-grip,
- and reduced immediate wheel-derived trust after a recent slip event.

The modifier shall be lightweight, bounded, and auditable.

### 4.3 Combined-slip and smoothness requirement

The final online law shall remain smooth in all of the following variables:

- slip ratio,
- slip angle,
- wheel speed,
- body speed,
- load,
- edge-shape modifier,
- and transient-contact memory.

### 4.4 Cost-control rule

The base online runtime shall not evaluate two separate full tire-law branches per wheel per sigma-point propagation.

Any implementation that effectively duplicates the full force-law cost in the hot path is non-compliant with V6.2.

## 5. Force and load model

For each wheel contact:

1. compute local longitudinal and lateral contact velocities,
2. apply low-speed regularization,
3. compute slip ratio and slip angle,
4. compute grip-law pure-slip forces,
5. apply the bounded edge-shape modifier,
6. apply smooth combined-slip competition,
7. compute body-force and yaw-moment contributions.

Do not hold wheel loads constant unless logs show that to be harmless.

Use at minimum:

```text
Fz = Fz_static + ΔFz_long + ΔFz_lat
```

If measured compliance effects materially improve fit, fold them into effective coefficients rather than expanding the propagated state.

## 6. Discretization

Use a semi-implicit step with the following order:

1. evaluate slips, loads, precursor terms, memory-held modifiers, and final tire forces from the current sigma-point state,
2. update `ω_L`, `ω_R`, `u`, `v`, and `r`,
3. update `ψ`,
4. update `p_x`, `p_y` from the updated body velocities.

---

# Part III — Exact stationary and moving branches

## 7. Exact stationary branch

This is the only hard branch in the estimator predict model.

### 7.1 Entry conditions

Enter exact stationary lock only with confidence, hysteresis, and dwell, using:

- near-zero wheel motion,
- near-zero corrected gyro,
- near-zero body-plane accel,
- effectively zero command,
- no active inconsistency evidence,
- no active onset holdoff,
- no active re-grip recovery dwell.

### 7.2 Process map in stationary lock

Use:

```math
p_{x,k+1} = p_{x,k}
p_{y,k+1} = p_{y,k}
ψ_{k+1}   = ψ_k
```

with contracting but non-frozen dynamic states:

```math
u_{k+1}      = a_u u_k + w_u
v_{k+1}      = a_v v_k + w_v
r_{k+1}      = a_r r_k + w_r
ω_{L,k+1}    = a_ω ω_{L,k} + w_{ωL}
ω_{R,k+1}    = a_ω ω_{R,k} + w_{ωR}
b_{gz,k+1}   = b_{gz,k} + w_b
```

with:

- `Q_px = 0`,
- `Q_py = 0`,
- `Q_ψ = 0`,
- small but nonzero dynamic-state noise.

### 7.3 Stationary pseudo-measurement

In exact stationary lock, apply:

```math
z_stat = [0,0,0,0,0]^T
h_stat(x) = [u, v, r, ω_L, ω_R]^T
```

with small but nonzero covariance.

### 7.4 Release behavior

On release from stationary lock, explicitly inflate covariance on at minimum:

- `u`,
- `v`,
- `r`,
- `ω_L`,
- `ω_R`.

A modest `ψ` inflation is permitted if release transients justify it.

## 8. Moving branch

Outside exact stationary lock, use the smooth `EstimatorPredictModel`.

---

# Part IV — Inputs and process-input contract

## 9. Use actual applied bank torque, not raw command alone

The estimator input shall be based on:

```text
u_c = [τ_app,L, τ_app,R]^T
```

where `τ_app,s` is the best available estimate of **actual applied bank torque**.

### 9.1 Required interpretation

The estimator shall not treat raw PWM or raw requested duty as the primary process input if a better torque-equivalent estimate is available.

The upstream torque estimate may use any combination of:

- commanded duty,
- measured or estimated bus voltage,
- current limit state,
- measured or estimated applied current,
- motor constants,
- back-EMF estimate from wheel speed,
- driver saturation logic,
- brownout / droop information.

### 9.2 Minimum contract

If direct current measurement is unavailable, the system shall still provide the best algebraic approximation of actual torque rather than the raw command alone.

### 9.3 Why this is mandatory

Near the traction limit, wrong wheel-torque prediction contaminates wheel-speed prediction before the slip logic has time to intervene. Feeding the estimator a closer approximation to the actual applied bank torque reduces model error where it matters most.

---

# Part V — Measurement model and update order

## 10. Direct measurements

### 10.1 Wheel speed

Use:

```math
z_ω = [ω_L, ω_R]^T + n_ω
```

with constrained gain so the update affects only:

- `ω_L`,
- `ω_R`.

### 10.2 Gyro

Use the correct bias-in-state measurement model:

```math
z_g = r + b_gz + n_g
```

The external bias anchor may remain only for:

- startup seeding,
- stationary support,
- bounded regularization,
- sanity telemetry.

It shall not replace `b_gz` inside the measurement equation.

### 10.3 Planar accelerometer

Use raw synchronized body-plane accelerometer samples in the estimator path.

Do not insert a default estimator-path software low-pass.

Use scalar rollout:

1. forward-axis update,
2. lateral-axis update,
3. both active once telemetry is clean and timing margin remains acceptable.

With IMU location `r_s = [x_s, y_s]^T`, predict:

```math
a_{x,s}^{pred} = \dot v + u r + \dot r\, y_s - r^2 x_s
a_{y,s}^{pred} = \dot u - v r - \dot r\, x_s - r^2 y_s
```

with project-consistent axis/sign handling.

Each accel axis shall have its own:

- innovation check,
- NIS telemetry,
- covariance adaptation,
- reject threshold.

### 10.4 Base runtime update policy

The mandatory base online update set shall be:

1. direct wheel-speed update,
2. direct gyro update,
3. closure pseudo-measurement update,
4. adaptive lateral pseudo-measurement update,
5. stationary pseudo-measurement when in exact stationary lock,
6. trusted wall / map updates.

The forward-axis scalar accelerometer update shall be implemented and is recommended for base runtime use if the timing audit remains inside budget.

The lateral-axis accelerometer update and benign-motion soft odometry shall be implemented behind feature gates and enabled online only after timing audit confirms that the base update budget remains acceptable.

## 11. Update ordering per cycle

Use the following order:

1. predict to the next fixed control boundary,
2. compute schedule variables from the predicted state plus held measurements,
3. freeze schedule variables for the rest of the cycle,
4. direct wheel-speed update with constrained gain,
5. direct gyro update,
6. scalar accel updates that are enabled for the current build,
7. closure pseudo-measurement update,
8. adaptive lateral pseudo-measurement update,
9. stationary pseudo-measurement if in exact stationary lock,
10. benign-motion soft odometry if enabled and allowed,
11. wall / map updates.

No same-cycle recursive schedule recomputation is allowed.

Trusted wall / map updates remain admissible during edge suppression, subject to their own innovation checks and gating.

---

# Part VI — Pseudo-measurements and adaptive authority

## 12. Shared geometry contract

Define one estimator geometry bundle:

```text
G_e = {R_e, W_e, a_e, r_s}
```

where:

- `R_e` = effective estimator wheel radius,
- `W_e` = effective estimator track width,
- `a_e` = effective force lever-arm quantity used consistently by the estimator,
- `r_s = [x_s, y_s]^T` = IMU location in body frame.

This bundle is computed once per cycle and reused consistently.

## 13. Wheel/body closure pseudo-measurement

Define held wheel references:

```math
u_k = \frac{R_e}{2}(ω_{L,h} + ω_{R,h})
r_k = \frac{R_e}{W_e}(ω_{L,h} - ω_{R,h})
```

These are references, not truths.

Define closure residuals:

```math
σ_L = R_e ω_{L,m} - \left(u + \frac{W_e}{2} r\right)
σ_R = R_e ω_{R,m} - \left(u - \frac{W_e}{2} r\right)
```

Use the pseudo-measurement:

```math
z_σ = [0,0]^T
h_σ(x) = [σ_L, σ_R]^T
```

### 13.1 Side-specific rule

Closure authority shall be side-specific.

If evidence points to left-bank failure, left closure authority weakens first.
If evidence points to right-bank failure, right closure authority weakens first.
Only common-mode evidence shall drive symmetric weakening.

### 13.2 Edge and re-grip rule

During confirmed edge operation on side `s`:

- do **not** permit that side’s closure to recover immediately after disagreement starts collapsing,
- require re-grip dwell and consistency hysteresis,
- and restore closure authority on that side with a slow recovery profile.

## 14. Adaptive lateral pseudo-measurement

Use:

```math
z_v = 0
h_v(x) = v
```

with covariance scheduled from:

- lateral utilization,
- closure disagreement,
- yaw-response deficit,
- launch / release windows,
- holdoff state,
- re-grip recovery state,
- edge-state confirmation,
- bank memory magnitude.

### 14.1 Launch rule

Immediately after stationary release and while speed is still low:

- keep `R_v` small,
- keep the lateral constraint strong,
- keep closure active,
- relax only after persistent contradictory evidence.

This is mandatory.

### 14.2 Confirmed edge rule

During confirmed two-bank saturation, sustained post-peak behavior, or severe re-grip recovery:

- `R_v` shall be inflated until the lateral pseudo-measurement becomes weak enough that it cannot dominate the estimator,
- and in the strongest confirmed cases it may be effectively disabled for a bounded dwell interval.

## 15. Benign-motion soft odometry aid

A weak pose / yaw soft odometry update is allowed only when all of the following hold:

- both closure residuals are small,
- lateral utilization is low,
- yaw residual is small,
- no launch or holdoff window is active,
- no re-grip recovery dwell is active,
- no confirmed edge state is active,
- accel consistency is acceptable,
- and the build / timing profile has explicitly enabled soft odometry.

It shall remain weak enough that it cannot dominate the estimator.

During confirmed edge operation, benign-motion soft odometry shall be disabled.

---

# Part VII — Schedules, precursor metrics, and adaptive noise

## 16. Baseline schedule variables

Compute once per cycle, then freeze:

```math
η_x = clip((|σ_L + σ_R| / 2) / σ_{x0}, 0, 1)
η_d = clip((|σ_L - σ_R| / 2) / σ_{d0}, 0, 1)
η_y = clip((|u r| - a_{y,soft}) / (a_{y,hard} - a_{y,soft}), 0, 1)
δ_r = |r_k - r| / (r_0 + |r_k|)
```

Additionally define per-bank anomaly scores:

```math
η_L = f(σ_L, \dot ω_L, a_{pred}-a_m, r_k-r)
η_R = f(σ_R, \dot ω_R, a_{pred}-a_m, r_k-r)
```

## 17. Mandatory precursor metric: bank friction utilization

Define the **grip-law pre-projection** utilization for each wheel:

```math
ξ_i = \sqrt{\left(\frac{F_{x0,i}^{grip}}{\mu_x F_{z,i}}\right)^2 + \left(\frac{F_{y0,i}^{grip}}{\mu_y F_{z,i}}\right)^2}
```

Then define per-bank precursor metrics:

```math
ξ_L = \max_{i \in L} ξ_i
ξ_R = \max_{i \in R} ξ_i
```

### 17.1 Interpretation

- `ξ_s << 1`: bank comfortably inside the nominal grip-law friction budget,
- `ξ_s ≈ 1`: bank approaching saturation,
- `ξ_s > 1`: the grip-law pure-slip request exceeds the available nominal envelope and projection is now essential.

### 17.2 Required use

`ξ_L` and `ξ_R` shall be used as **pre-onset indicators**.

When a bank’s `ξ_s` remains high or rises sharply, the estimator shall begin weakening suspect wheel-derived aids and inflating side-specific process / measurement uncertainty before gross wheel/body disagreement fully develops.

## 18. Process-noise schedules

At minimum, schedule:

```math
Q_u = Q_{u0}(1 + q_{ux}η_x^2)
Q_v = Q_{v0}(1 + q_{vy}η_y^2 + q_{vd}δ_r^2 + q_{vΔ}η_d^2)
Q_r = Q_{r0}(1 + q_{ry}η_y^2 + q_{rd}δ_r^2 + q_{rΔ}η_d^2)
```

Also allow side-triggered inflation:

- increase `Q_ωL` and left-side closure `R_L` when `η_L` rises sharply,
- increase `Q_ωR` and right-side closure `R_R` when `η_R` rises sharply,
- allow additional early inflation when `ξ_L` or `ξ_R` persist near saturation,
- allow temporary `R_v` inflation when both precursor utilization and measurement disagreement indicate that `v ≈ 0` is becoming unreliable,
- support further inflation when edge-memory magnitude or re-grip dwell is active.

## 19. Regime scheduling

Use **one filter** with three smooth operating regions for scheduling only:

- Grip,
- Transition,
- Edge.

No full multi-filter supervisor is part of V6.2.

Use logistic or similarly smooth weights, not hard switches, to schedule:

- closure covariance,
- `R_v`,
- enabled accel-axis covariance,
- wheel-speed process noise,
- optional soft-odometry covariance,
- and edge-shape modifier strength priors.

---

# Part VIII — Runtime robustification layer

## 20. Innovation consistency handling

Every major enabled update group shall compute runtime innovation consistency telemetry.

At minimum:

- gyro,
- any enabled accel axes,
- closure-L,
- closure-R,
- lateral pseudo-measurement,
- soft odometry if enabled,
- wall updates.

For each enabled update group, implement three response levels:

### Green
Use nominal covariance.

### Amber
Inflate covariance smoothly.

### Red
Reject the update or enter holdoff.

## 21. Slip-onset holdoff

When onset evidence appears, weaken suspect wheel-derived aids immediately.

### 21.1 Example onset triggers

- rapid growth in `|σ_L|` or `|σ_R|`,
- rapid growth in `|r_k - r|`,
- abrupt increase in `η_y`,
- wheel / body / IMU sign inconsistency,
- known launch or reversal windows,
- persistent `ξ_L` or `ξ_R` near or beyond saturation,
- known saturation / inconsistency windows.

### 21.2 Required responses

On onset:

- immediately inflate the suspect side’s closure covariance,
- weaken optional soft odometry if it is enabled,
- relax the lateral pseudo-measurement according to a short holdoff profile,
- increase edge-shape modifier influence on the suspect bank,
- hold weakened authority through a minimum dwell,
- restore only after hysteresis and consistency conditions are met.

## 22. Mandatory protection: re-grip detector and recovery dwell

V6.2 shall retain explicit protection against **re-grip shock**.

### 22.1 Motivation

A single-filter design is most vulnerable not only at slip onset, but also at the instant when a bank re-attaches and wheel/body disagreement collapses rapidly. If wheel-derived authority is restored too early, the estimator can be snapped by the same evidence it had correctly learned to distrust only moments before.

### 22.2 Re-grip event logic

Define a re-grip candidate on side `s` only if that side was recently in a slip / holdoff condition. Then monitor a re-grip score such as:

```math
ρ_s = w_σ \max\!\left(0, -\frac{d|σ_s|}{dt}\right)
    + w_ω \frac{|\dot ω_s|}{\dot ω_0}
    + w_a \frac{\|a_m - a_{pred}\|}{a_0}
```

or an equivalent normalized logic.

A side shall be considered in re-grip recovery when all of the following are jointly indicated:

- the bank had recent slip evidence,
- `|σ_s|` collapses rapidly,
- wheel deceleration or wheel-state jerk is high,
- a concurrent body / IMU transient is present.

### 22.3 Required responses

During re-grip recovery dwell:

- do **not** immediately restore full closure authority,
- keep suspect-side closure covariance elevated,
- keep optional soft odometry disabled,
- restore `v ≈ 0` authority gradually,
- keep the edge-shape modifier active on the suspect bank until hysteresis clears,
- require hysteresis and a dwell timer before returning to nominal trust.

### 22.4 Rise / decay asymmetry

Evidence shall accumulate quickly and decay slowly.

That asymmetry is mandatory.

---

# Part IX — Base transient-contact memory and edge-shape logic

## 23. Mandatory bounded transient-contact memory

V6.2 shall include a **bounded transient-contact memory term** in the base force-model architecture.

This term exists to capture short-horizon path dependence that a purely quasistatic combined-slip model misses during:

- abrupt breakaway,
- transient saturation,
- sustained edge running,
- abrupt re-grip,
- and the first few milliseconds after reattachment when restoring nominal wheel-derived authority is estimator-dangerous.

### 23.1 Architectural form

The transient-contact memory shall **not** expand the propagated UKF state beyond 9 states.

Instead, define deterministic per-bank internal memory variables:

```math
m_L, m_R \in [0,1]
```

These variables are:

- updated once per cycle from held measurements, predicted state, and robustification evidence,
- frozen for the remainder of the cycle exactly like the other schedule variables,
- not part of the SR-UKF covariance state,
- and not allowed to create hidden wheel-to-body direct authority.

Per-bank memory is mandatory because the project’s dominant failure modes are explicitly bank-asymmetric.

### 23.2 Update law

A representative update law is:

```math
m_{s,k+1}^{raw} = a_m m_{s,k} + b_{slip} g_{slip,s} + b_{rg} g_{rg,s}
```

```math
m_{s,k+1} = clip\left(m_{s,k+1}^{raw} - b_{clr} g_{clear,s}, 0, 1\right)
```

for `s ∈ {L,R}` with:

- `0 < a_m ≤ 1` providing short-horizon memory,
- `g_{slip,s}` rising with persistent precursor saturation, anomaly growth, and onset evidence,
- `g_{rg,s}` rising during re-grip candidates,
- `g_{clear,s}` active only after hysteresis and dwell show that the bank has genuinely returned to benign rolling.

The rise path shall be faster than the decay path.

That asymmetry is mandatory.

### 23.3 Required evidence inputs

At minimum, `g_{slip,s}` and `g_{rg,s}` shall be functions of:

- `ξ_s`,
- `η_s`,
- `|σ_s|` and `d|σ_s|/dt`,
- wheel acceleration / deceleration evidence,
- yaw disagreement,
- accel-consistency transients.

### 23.4 Required coupling to force shape and authority logic

The transient-contact memory shall feed both:

1. the bounded edge-shape modifier in the force model,
2. the robustification schedules.

At minimum:

- larger `m_s` shall support elevated suspect-side closure covariance,
- larger `m_s` shall support slower restoration of `v ≈ 0` authority,
- larger `m_s` shall suppress optional soft odometry,
- and larger `m_s` shall be logged and auditable.

### 23.5 Acceptance intent

This term is part of the **base V6.2 acceptance**, not an optional later addition.

Its implementation shall be lightweight, deterministic, bounded, and auditable.

---

# Part X — Sigma-point strategy and cost control

## 24. Base V6.2 requirement: spherical-simplex sigma set

Base V6.2 shall ship on a **square-root spherical-simplex sigma-point set**.

This is not a later optimization. It is part of the base architecture because the present loop budget requires reduced sigma-point propagation cost.

The implementation shall preserve:

- square-root covariance handling,
- QR-based square-root propagation,
- Cholesky update / downdate style correction,
- and numerical robustness comparable to the present SR-UKF implementation.

## 25. Predictor substepping policy

### 25.1 Base runtime rule

**Online predictor substepping is not part of the base V6.2 runtime path.**

The base online estimator shall perform one predictor integration step per control cycle.

### 25.2 Allowed optional use

A two-substep and four-substep predictor mode may exist only for:

- offline replay,
- audit,
- debug,
- or tightly controlled timing experiments behind a compile-time gate.

### 25.3 Non-adoption reason

Under the stated runtime budget, the recovered propagation cost from the simplex sigma set shall be spent on:

- preserving timing margin,
- enabling the simpler edge-memory and re-grip protections,
- and supporting staged sensor-update rollout,

not on a permanently more expensive worst-case predict path.

### 25.4 Acceptance rule

Base runtime acceptance shall be demonstrated without relying on online predictor substepping.

---

# Part XI — Explicit non-adoptions

## 26. Not part of base V6.2

The following are explicitly not part of the base swap:

1. full IMM / multi-filter supervisor,
2. learned residual tire model,
3. added propagated tire-state dimensions,
4. estimator-path software low-pass states,
5. reduced-state body-dominant estimator,
6. controller-oriented stop blending inside sigma-point propagation,
7. full dual-branch tire-law hot-path evaluation,
8. online predictor substepping as a required runtime feature.

## 27. Sanctioned future upgrade path

If V6.2 later proves insufficient specifically in sustained post-peak slide recovery, the sanctioned extension path is:

1. keep the same 9-state structure,
2. keep the same measurement authority architecture,
3. keep the same simplex SR-UKF core,
4. refine the edge-shape modifier first,
5. refine the memory and trigger thresholds second,
6. enable extra accel-axis authority only if timing margin allows,
7. explore substepped replay / audit behavior fourth,
8. and only then reconsider a more complex supervisor if the simpler path fails.

---

# Part XII — File-level implementation guidance

## 28. New or revised modules

Add or revise the following:

- `EstimatorPredictModel.h/.cpp`
- `WheelAuthorityPolicy.h/.cpp`
- `UkfRobustUpdatePolicy.h/.cpp`
- `EstimatorGeometry.h`
- `TorqueEstimateAdapter.h/.cpp`
- `GripUtilizationMetrics.h/.cpp`
- `RegripRecoveryMonitor.h/.cpp`
- `SigmaPointSetSimplex.h/.cpp` as a required base module
- `TransientContactMemory.h/.cpp` as a required base module
- `EdgeForceShapeModifier.h/.cpp` as a required base module

Optional / gated modules:

- `PlanarAccelLateralUpdate.h/.cpp`
- `SoftOdometryAid.h/.cpp`
- `PredictorSubstepPolicy.h/.cpp` for replay / audit only

## 29. `SrUkfCore`

### Keep

- square-root UKF core,
- state normalization,
- startup bias-seed logic concept,
- wall update flow,
- useful launch / inconsistency timers,
- debug dump infrastructure.

### Remove or materially revise

- routine pose anchoring to encoder delta,
- routine overwrite of `u` or `r` from encoder kinematics,
- use of the external bias anchor in place of `b_gz` during gyro update,
- disabled planar-accel stub,
- any wheel update that can move body states through hidden covariance leakage,
- standard `2n+1` sigma-point generation as the default path.

### Add

- constrained wheel update,
- closure residual helper,
- frozen schedule helper,
- side-specific closure adaptation,
- adaptive lateral pseudo-measurement logic,
- innovation-consistency helper,
- holdoff / dwell logic,
- forward-axis scalar accel update support,
- feature-gated lateral accel update support,
- feature-gated benign-motion soft odometry,
- stationary release inflation,
- precursor utilization metrics `ξ_L`, `ξ_R`,
- re-grip detector and recovery dwell logic,
- simplex sigma-point generation,
- telemetry for direct-wheel-update body invariance.

## 30. `PlantModel`

### Keep

- physical wheel-bank and drivetrain structure,
- inverse / feedforward path,
- lever-arm-aware IMU prediction support,
- physically meaningful technical limits,
- geometry consistency with the estimator.

### Standardize

- online tire/contact family to grip-law plus bounded edge-shape modification,
- smooth saturation projection or equivalent smooth combined-slip competition,
- quasistatic load transfer treatment,
- explicit access to grip-law pre-projection utilization terms for `ξ_L`, `ξ_R`,
- transient-contact memory coupling hooks,
- edge-shape modifier hooks.

### Runtime inverse / feedforward rule

The runtime inverse / feedforward solve shall **not** be required to invert the full hysteretic memory-dependent force law online.

For runtime command inversion, use either:

- the grip branch alone with runtime authority limiting,
- or a frozen-`m` / frozen-local-modifier approximation.

The full memory-dependent law remains available for forward prediction and offline audit.

### Remove from estimator propagation responsibility

- motion-weight stop blending,
- snap-to-zero cleanup,
- any discontinuous controller convenience logic.

## 31. `MouseUkfFacade`

The public facade can remain nearly unchanged.

Internally it shall now front:

- the 9-state SR-UKF,
- the simplex sigma-point set,
- constrained wheel authority,
- the dedicated estimator predict model,
- the robustification layer,
- the applied-torque input contract,
- the new precursor / re-grip protection helpers,
- the transient-contact memory term,
- the bounded edge-shape modifier,
- and the staged sensor-update feature gates.

---

# Part XIII — Calibration, thresholding, and offline mapping

## 32. Identification order

Fit in the following order:

1. geometry constants `R_e`, `W_e`, lever arms,
2. drivetrain loss and equivalent bank inertia terms,
3. applied-torque estimator contract,
4. baseline grip-law modified Fiala / Dugoff-class tire model,
5. quasistatic load-transfer terms,
6. edge-shape modifier parameters,
7. closure and lateral pseudo-measurement schedule parameters,
8. precursor thresholds for `ξ_L`, `ξ_R`,
9. onset holdoff thresholds,
10. re-grip detector thresholds and dwell timers,
11. transient-contact memory gains and decay logic,
12. optional enabled accel-axis thresholds,
13. optional soft-odometry enabling thresholds,
14. final compute / fidelity trade audit.

## 33. Offline equilibrium and stability mapping

Use offline near-limit mapping to set the scheduling thresholds and audit the model’s structural behavior.

In particular, use offline analysis to choose:

- `a_{y,soft}` and `a_{y,hard}`,
- precursor thresholds on `ξ_s`,
- holdoff and recovery dwell values,
- the edge-modifier onset range,
- the recovery profile.

The purpose is not to copy full-scale automotive observers directly. The purpose is to use the correct structural lens for:

- combined-slip saturation,
- transient friction-state effects,
- high-sideslip equilibrium existence,
- and loss-of-stability topology.

---

# Part XIV — Validation and acceptance

## 34. Required staged validation

### Stage 1 — exact stationary branch

Show:

- zero pose / heading diffusion at certified rest,
- bounded dynamic-state decay,
- clean release inflation.

### Stage 2 — constrained wheel update

Show:

- direct wheel measurements update `ω_L`, `ω_R` cleanly,
- body states do not move through the direct wheel update,
- no hidden wheel-to-body leak remains.

### Stage 3 — simplex sigma-set parity and timing

Show:

- state quality is at least on par with the prior standard sigma-set implementation in benign operation,
- worst-case estimator wall time is materially improved or at minimum not regressed,
- no numerical robustness regression appears.

### Stage 4 — closure and side-specific authority

Show:

- small closure residuals in adherent motion,
- side-first weakening during one-bank events,
- symmetric weakening only under common-mode evidence.

### Stage 5 — adaptive lateral pseudo-measurement

Show:

- bounded launch-side `v`,
- no premature release of lateral constraint,
- sufficiently weak authority in confirmed edge states,
- smooth recovery after re-grip.

### Stage 6 — gyro bias correction

Show:

- stable `b_gz`,
- no yaw-regression from moving the bias back into the true measurement equation.

### Stage 7 — staged planar accel rollout

Show:

- correct signs,
- correct lever-arm behavior,
- useful innovations,
- clean scalar gating,
- no destabilization,
- and acceptable timing impact for each enabled axis.

### Stage 8 — precursor metrics

Show:

- `ξ_L`, `ξ_R` rise before the worst wheel/body disagreement in saturation approach,
- precursor-triggered authority weakening reduces onset contamination,
- no material degradation in adherent operation.

### Stage 9 — transient-contact memory and edge-shape audit

Show:

- `m_L`, `m_R` rise in the intended breakaway / re-grip windows,
- adherent operation is not materially softened,
- force restoration is delayed enough to reduce re-grip contamination without becoming sluggish,
- post-peak behavior is better matched than coefficient-scaling-only behavior.

### Stage 10 — aggressive edge-regime audit

Show separately:

- one-bank launch slip,
- two-bank launch slip,
- aggressive turn saturation,
- yaw under-response,
- pivot / scrub events,
- adherent straight precision,
- adherent gentle-turn precision,
- saturation recovery / re-grip.

## 35. Required telemetry

At minimum log:

- `σ_L`, `σ_R`,
- `η_x`, `η_d`, `η_y`, `δ_r`, `η_L`, `η_R`,
- `ξ_L`, `ξ_R`,
- re-grip score / state per bank,
- transient-contact memory `m_L`, `m_R`,
- `Q_u`, `Q_v`, `Q_r`, `Q_ωL`, `Q_ωR`,
- `R_v`, closure `R_L`, closure `R_R`,
- accel innovations and NIS for each enabled axis,
- gyro innovation and NIS,
- soft-odometry innovation and NIS if enabled,
- stationary-lock state,
- holdoff state and dwell timers,
- recovery dwell state and timers,
- release-inflation events,
- covariance diagonals for `u`, `v`, `r`, `ω_L`, `ω_R`, `b_gz`,
- explicit flags proving the direct wheel update did not move body states,
- applied torque estimate per bank,
- per-cycle predict wall time,
- per-cycle update wall time,
- total estimator wall time.

## 36. Acceptance criteria

V6.2 is accepted only if all of the following are true:

1. no hidden wheel-to-body direct authority remains outside exact rest,
2. pose and heading do not diffuse during certified rest,
3. launch-side `v` remains bounded,
4. slip-onset contamination is materially reduced relative to the current implementation,
5. precursor metrics provide useful early warning before the worst mismatch develops,
6. re-grip contamination is materially reduced relative to onset-only behavior,
7. the edge-shape modifier reduces breakaway / re-grip brittleness without materially degrading adherent response,
8. aggressive-turn prediction / control behavior is at least as good as the best prior revision and materially better than the current stack,
9. adherent straight and gentle-turn precision do not materially regress,
10. raw planar-accel reintegration does not introduce instability when enabled,
11. trusted wall / map updates remain available during edge suppression,
12. the runtime inverse / feedforward path remains stable without inverting the full hysteretic law online,
13. **predict cost remains in roughly the original envelope (about 30% of the loop tick),**
14. **update cost remains in roughly the original envelope (about 20% of the loop tick),**
15. **total estimator runtime remains compatible with the current loop budget without relying on online predictor substepping.**

---

# Part XV — Bottom line

## 37. Final recommendation

The V6.2 swap should be built around the following statement:

> Keep the 9-state SR-UKF and the physically meaningful bank-split plant, make the simplex sigma set part of the base implementation, keep wheel/body authority explicit and constrained, make rest exact, restore raw planar accel conservatively and in stages, feed the estimator actual applied bank torque, use grip-law pre-projection precursor metrics and bounded per-bank transient-contact memory to harden onset and re-grip behavior, implement edge behavior as a lightweight force-shape modifier rather than a second full tire-law hot path, preserve trusted wall/map updates during edge suppression, and hold the online runtime close to the original predict/update cost envelope.

That is the strongest final single-filter rewrite I would recommend for this project under the stated requirements and compute constraints while still avoiding the cost and complexity of a multi-filter supervisor.

---

## Basis for this proposal

This proposal is synthesized from:

- the reviewed V4 / V6 / V6.1 single-filter architecture,
- the option study distinguishing single-filter, learned-residual, and IMM approaches,
- the traction-limit modelling notes identifying combined-slip structure, transient friction effects, and near-limit equilibrium topology as the most relevant external ideas,
- and the explicit practical constraint that runtime cost must remain close to the original implementation envelope.
