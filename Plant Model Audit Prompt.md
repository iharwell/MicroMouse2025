# Audit Prompt: Micromouse UKF / Plant / Measurement Model Specs

You are auditing one or more micromouse project specifications. Treat this as a serious engineering review, not a generic completeness checklist.

The project context is a high-performance micromouse using a planar UKF, encoder-derived drivetrain inputs, IMU and wall-sensor measurements, algebraic contact/slip modeling, and residual body-acceleration states. The operating envelope includes scrubbed in-place turns, high-speed arcing turns with expected stick-slip, full-bore straight acceleration limited by ground strikes and motor power, constant-velocity exploration, precision turns near walls, and per-run calibration maneuvers.

The audit standard is:

> A spec is weak if it either under-specifies hard contracts needed for coherent implementation, or over-specifies empirical physical models in a way that could lock the project out of a physically representative implementation.

Do **not** maximize prescriptiveness. Move toward the strongest foundation for this project.

---

## Core audit philosophy

Assess whether the spec is strict in the right places and flexible in the right places.

### Hard contracts should be locked down

Treat these as areas where ambiguity is dangerous:

- State vector, state order, units, signs, and naming.
- Coordinate frame and heading convention.
- Measurement semantics.
- Input semantics.
- Encoder-as-drivetrain-input semantics.
- IMU sign and calibration contract.
- Timestamp/effective-time semantics at the model boundary.
- Noise parameter meaning.
- Required covariance propagation.
- Sample validity/fault semantics.
- Interfaces between plant submodels.
- Diagnostics that must be emitted for validation.

If these are ambiguous, mark them as under-specified hard contracts.

### Empirical physical models should be model-family contracts

Treat these as flexible unless the spec has strong logged evidence:

- Tire/contact force law.
- Combined-slip saturation shape.
- Normal-load/load-transfer model.
- Motor/driver command-to-torque model.
- Ground-strike / acceleration-envelope approximation.
- Yaw scrub / breakaway / kinetic yaw resistance model.
- Fan/downforce approximation.
- Wall-sensor response model.
- Residual acceleration noise schedules.
- Calibration thresholds.
- Regime classification thresholds.

For these, the spec should not merely say “leave it flexible.” It must provide:

1. **A best first-pass implementation.**
2. **The specific failure signatures expected if that first pass is inadequate.**
3. **At least one targeted follow-on option that addresses those likely failures.**
4. **The hard interface/diagnostic contract that remains fixed while the model changes.**

A flexible element with no first pass is too vague.
A first-pass formula with no escape path is over-specified.
A follow-on option that does not specifically address the first pass’s likely failure mode is insufficient.

---

## Scope discipline

Stay inside the scope of the documents being audited.

Do not casually propose new sensors, new states, new subsystems, or new estimator architecture. Only recommend a scope expansion if the spec’s own stated abstraction cannot plausibly represent the robot within the project’s required operating envelope.

For example:

- Do not say “add wheel-rate states” unless the document’s encoder-as-input argument fails on its own terms.
- Do not say “add pitch/roll states” unless the planar treatment of floor strike cannot be made adequate through the spec’s existing residual/noise/measurement model.
- Do not say “add current sensors” unless the command-to-torque abstraction cannot be made physically representative with the available inputs/logs.
- Do not say “add map states” if the document intentionally keeps map/wall geometry outside the UKF.

The task is to assess the spec’s foundation, not to redesign the project from scratch.

---

## Audit dimensions

For each material concern, evaluate all applicable dimensions.

### 1. Physical representativeness

Does the spec allow a model that can reasonably approximate the actual robot?

Consider:

- Scrubbed in-place turns.
- Stick-slip arcing turns.
- Launch thresholds.
- Gear/ripple variation.
- Ground-strike-limited full-bore acceleration.
- Tire springback and wall-touch behavior.
- Motor/driver current limiting and command nonlinearity.
- IMU acceleration contamination from pitch/roll/impact.
- IR wall-sensor response behavior, saturation, incidence, and no-hit cases.

A physically plausible first-pass model is required, but the spec must not lock in a first-pass formula as if it were guaranteed physical truth.

### 2. Over-specification risk

Does the spec force a particular empirical decomposition where multiple decompositions could fit the robot better?

Flag this when:

- A formula is presented as mandatory physical truth despite being an empirical approximation.
- A parameter is given a physical interpretation that logs may not support.
- A clamp/saturation function changes the meaning of a stated physical envelope.
- Numeric validation thresholds are fixed before baseline characterization.
- Residual limits are so strict that real unmodeled physics would be rejected.
- A calibration sequence forces parameters to absorb the wrong effects.

For over-specified items, recommend turning the formula into a model-family contract with a first pass and targeted follow-on paths.

### 3. Under-specification risk

Does the spec leave implementation-critical semantics open?

Flag this when:

- Two implementations could satisfy the prose but produce materially different behavior.
- Noise quantities lack exact stochastic meaning.
- Timing/effective-time semantics are vague.
- Measurement values are not physically defined.
- Covariance propagation requirements are incomplete.
- Input validity/fault behavior is not operational.
- Required diagnostics are absent.

For under-specified hard contracts, recommend more precise requirements.

### 4. Flexible-model quality

For each flexible empirical element, answer:

- What is the best first-pass implementation?
- Why is that first pass the best current guess?
- Where is it likely to fail?
- What log signatures would reveal that failure?
- What follow-on model option should be tried next?
- What interface, units, signs, inputs, outputs, and diagnostics must remain unchanged?

If the spec does not contain these, mark the section as incomplete even if it correctly says the model is empirical.

### 5. Validation and identifiability

Assess whether the spec makes the model testable without over-locking it.

Look for:

- Held-out replay tests.
- Regime-labeled validation.
- Innovation/NIS analysis.
- Residual-state behavior diagnostics.
- Residual-disabled or low-residual replay tests.
- Parameter confounding analysis.
- Explicit fitting maneuvers.
- Soft vs hard parameter bounds.
- Evidence-based threshold versioning.
- Separation of calibration data from validation data.

Validation criteria should be required, but numeric pass/fail thresholds should be versioned from characterization unless they are based on physical impossibility or safety.

---

## Concern classification

Use these concern classes:

| Class | Meaning |
|---|---|
| **HC** | Hard contract missing, ambiguous, or internally inconsistent. |
| **OS** | Over-specified empirical model risks excluding physically representative implementations. |
| **UF** | Flexible model family is under-defined. |
| **FP** | First-pass implementation is missing, weak, or not justified. |
| **FO** | Follow-on path is missing or does not target likely first-pass failures. |
| **VAL** | Validation, diagnostics, or calibration criteria are insufficient. |
| **CONF** | Parameter confounding or identifiability risk. |
| **SCOPE** | Recommendation risks drifting outside the document’s intended scope. |
| **CONS** | Internal consistency problem between equations, prose, or interfaces. |

A concern may have multiple classes.

---

## Priority scale

Use this priority scale.

| Priority | Meaning |
|---:|---|
| **P0** | The spec may fail as a foundation: it either blocks a physically representative implementation, permits incoherent implementations, or lacks a required first-pass/follow-on structure for a central model family. Resolve before serious calibration. |
| **P1** | High risk to physical fit, estimator consistency, model-family evolution, or validation credibility. Resolve before relying on tuned logs or performance claims. |
| **P2** | Important robustness, diagnostics, comparability, or maintainability issue. Not usually a first-order blocker. |
| **P3** | Low-risk cleanup, wording improvement, or implementation guardrail. |

Do not assign P0 merely because a formula is imperfect. Assign P0 when the spec’s current wording would either force the wrong kind of implementation, fail to define a required contract, or provide no viable path from first pass to physically representative model.

---

## Required output format

Produce the audit in Markdown with the following sections.

---

# Executive summary

Give a concise assessment of whether the spec is moving toward the strongest foundation for the project.

Explicitly address:

- Whether the spec is too vague, too rigid, or appropriately balanced.
- Whether the physical-model portions are expressed as usable model-family contracts.
- Whether hard implementation contracts are sufficiently locked down.
- Whether the spec contains first-pass implementations and targeted follow-on paths for flexible empirical elements.

---

# Priority table

Provide a table:

| Section | Priority | Concern class | Concern level |
|---|---:|---|---|

The “Concern level” cell should be specific. Do not write generic statements such as “needs more detail.”

---

# Detailed audit findings

For each concern, use this format:

## [Priority] [Section] — [Concern title]

**Concern class:** HC / OS / UF / FP / FO / VAL / CONF / SCOPE / CONS

**What the spec currently does:**  
Summarize the relevant spec behavior.

**Why this matters for this robot:**  
Tie the concern to the micromouse operating envelope, hardware, measurement model, or calibration process.

**Risk if left unchanged:**  
Explain the concrete failure mode.

**Recommended spec direction:**  
State whether the spec should become stricter, more flexible, or both.

**If this is a flexible model element, require:**

- **Best first-pass implementation:** The best current guess the spec should present.
- **Likely first-pass failure signatures:** What logs/innovations/residuals would show it is inadequate.
- **Targeted follow-on option(s):** At least one specific next model option addressing those likely failures.
- **Hard contract that must remain fixed:** Interface, units, signs, diagnostics, timing, or covariance semantics that should not change.

**Suggested spec language:**  
Provide concise replacement or addition text when useful. Avoid rewriting the whole document unless necessary.

---

# Hard contracts vs flexible model families

Provide a table:

| Area | Should be hard or flexible? | Required treatment |
|---|---|---|

For example:

- State vector: hard.
- Coordinate signs: hard.
- Encoder-as-input semantics: hard.
- Tire force law: flexible model family with first pass and follow-on.
- Normal-load model: flexible model family with first pass and follow-on.
- Motor command-to-torque model: flexible model family with first pass and follow-on.
- Wall sensor response: flexible model family with first pass and follow-on.
- Validation metrics: hard requirement; numeric thresholds should be evidence-versioned.

---

# First-pass / follow-on coverage table

For each flexible empirical model, provide:

| Flexible element | First pass present? | Likely failure signatures present? | Follow-on path present? | Audit result |
|---|---:|---:|---:|---|

Mark missing pieces clearly.

---

# Top recommended edits

List the smallest set of edits that would most improve the spec foundation.

Each edit should be phrased as a spec change, not as general advice.

---

## Additional audit rules

- Do not reward the spec for appearing mathematically detailed if the formula is an empirical placeholder with no failure path.
- Do not punish the spec for leaving an empirical model flexible if it defines the interface, first pass, diagnostics, and follow-on options.
- Do not demand new states or sensors unless the current scope cannot plausibly satisfy its own goals.
- Do not treat residual acceleration states as proof the nominal plant is adequate. Residuals are model slack and diagnostics; they must not be allowed to hide a nonrepresentative nominal model without validation evidence.
- Do not require tiny residuals in aggressive maneuvers where the spec intentionally models only the mean behavior. Instead, require regime-specific residual diagnostics and held-out validation.
- Treat logs and characterization as central to the project. The spec should enable learning the robot, not trap the project in a plausible but wrong initial equation.
- Cite specific document sections or equations for every material concern.