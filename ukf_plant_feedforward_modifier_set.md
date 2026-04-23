# Modifier Set for Competitive Proposal Generation

Use the **common problem statement** as the base prompt for every run.

Apply **one primary modifier** per run. Occasionally apply **one secondary modifier** as well when you want to force a more specific corner of the design space.

When tradeoffs arise, the modifier guidance should take precedence over conventional defaults.

---

## Modifier A — Contact-centric estimator

Focus on the idea that the sensors are already extremely good, so the 9 UKF states should be spent primarily on **hidden contact / force-generation / regime variables**, not on re-estimating measured wheel quantities unless that can be strongly justified.

### Priority questions

- Which hidden variables most improve situational awareness?
- Should wheel speed remain outside the UKF as a measured input?
- What contact-memory or force-lag states matter most?
- How should yaw-only scrub behavior be represented?

Use this modifier to force proposals away from wheel-centric conventional observer structure.

---

## Modifier B — Wheel-centric estimator under strict justification

Assume the design keeps wheel-related states in the UKF. Require the generator to justify that choice against the critique that encoder data is already good enough and nearly centered in time.

### Priority questions

- What exactly do wheel states buy that the direct encoder path does not?
- Is the value structural, diagnostic, predictive, or fault-tolerance-related?
- How is the control path protected from estimator lag?
- Why is this still better than using those dimensions for hidden contact states?

Use this to test the strongest defensible wheel-state case.

---

## Modifier C — No in-motion gyro-bias adaptation

Assume gyro bias is effectively constant during motion and should not be a moving UKF state.

### Priority questions

- How is gyro bias handled outside the UKF?
- What state replaces it, if any?
- What estimator simplifications and robustness gains result?
- How are startup calibration and stationary recalibration handled?

Use this to force proposals to stop spending state budget on weakly useful bias dynamics.

---

## Modifier D — Pivot / scrub first

Emphasize yaw-dominant and near-in-place turning behavior.

### Priority questions

- What model structure handles `|R|` high and `|U|` near zero without collapsing?
- Is a distinct pivot-scrub branch required?
- How is the transition between scrub-dominant and rolling-dominant operation handled?
- How should feedforward command inversion work in yaw-only motion?

Use this to generate proposals that are strong where ordinary rolling models tend to fail.

---

## Modifier E — Traction-limit and re-grip first

Emphasize slip onset, post-peak behavior, and reattachment.

### Priority questions

- What hidden states or schedules best predict contamination at slip onset?
- How is re-grip prevented from snapping the estimate?
- How should the plant model represent post-peak behavior and delayed force restoration?
- What makes the design fault-tolerant near the traction limit?

Use this to surface designs optimized for the hardest dynamic regime.

---

## Modifier F — Feedforward-first architecture

Treat feedforward / inverse quality as the primary design center.

### Priority questions

- What plant formulation gives the best inverse map from state + requested acceleration to actuator demand?
- Which parts of the model must be invertible online?
- Which hysteretic or memory-dependent effects should be approximated, frozen, or excluded from runtime inversion?
- How are infeasible requests projected to the nearest feasible demand?

Use this to produce proposals that prioritize command quality and controllability.

---

## Modifier G — Estimator-first architecture

Treat estimator robustness and state quality as the primary design center, with feedforward adapted around it.

### Priority questions

- What model structure makes the UKF most coherent and observable?
- Which dynamics should be simplified in the runtime inverse to support better estimation?
- What state arrangement best prevents cross-contamination?
- What is the strongest single-filter architecture under the compute budget?

Use this to surface designs where the UKF is clearly the anchor of the system.

---

## Modifier H — Compute-minimalist

Assume every added state, branch, and nonlinear law must justify itself against the 1 kHz runtime budget.

### Priority questions

- What is the strongest design that remains close to the current runtime envelope?
- Which model elements buy the most robustness per unit compute?
- What should be moved offline, approximated, or scheduled instead of propagated?
- Where is the smallest acceptable complexity that still materially improves the hard regimes?

Use this to find designs that are disciplined rather than ambitious.

---

## Modifier I — Maximum capability within 1 kHz

Assume the goal is to push as much performance and robustness as possible while still fitting inside the real-time envelope.

### Priority questions

- If compute is tight but not zero, where should the budget be spent?
- Which added hidden states are worth it?
- Which nonlinearities are worth keeping online?
- What is the most capable design that still plausibly ships?

Use this when you want proposals that are aggressive but still real.

---

## Modifier J — Identifiability and calibration first

Force the generator to optimize for parameters and states that can actually be identified, calibrated, and maintained.

### Priority questions

- Which model terms can be fit reliably from logs?
- Which hidden states are actually observable with this sensor suite?
- Which elegant states are tempting but weakly identifiable?
- How should the identification process shape the architecture?

Use this to suppress beautiful but unmaintainable designs.

---

## Modifier K — Fault-containment first

Focus on graceful degradation, not just nominal performance.

### Priority questions

- How does the design behave under encoder inconsistency, IMU inconsistency, wall-update disagreement, or model mismatch?
- What prevents one bad channel from contaminating the whole state?
- Which state partition and update logic most cleanly contain faults?
- How does the system continue operating when assumptions are temporarily wrong?

Use this to find architectures with strong operational resilience.

---

## Modifier L — Minimal-regime / smooth-unified model

Bias the generator away from lots of discrete modes and toward a smooth unified model with as little hard branching as possible.

### Priority questions

- Can one coherent law cover stationary excepted, rolling, scrub, mixed yaw/rolling, and near-limit behavior with smooth scheduling?
- Which branches are truly necessary?
- What is the cost of too many modes in tuning and robustness?

Use this to generate elegant continuous designs.

---

## Modifier M — Explicit multi-regime specialist

Do the opposite: encourage explicit specialized handling where physics genuinely changes.

### Priority questions

- Which operating regions deserve separate treatment?
- Should pivot-scrub, rolling grip, and traction-limit slide recovery be represented by distinct submodels?
- Where does specialization beat smooth unification?

Use this to challenge the unified-model instinct.

---

## Modifier N — Measurement-authority-first

Focus on how each measurement source should and should not influence state.

### Priority questions

- Which measurements should directly own which states?
- Which quantities should never directly overwrite body states?
- How should wheel, gyro, accel, and wall/map information be partitioned?
- What is the cleanest authority architecture?

Use this to generate proposals with strong update discipline.

---

## Modifier O — Racing utility first

Ignore elegance unless it helps run fast and repeatably.

### Priority questions

- Which design would most likely improve actual run performance?
- Which complexities are justified only if they pay back in measurable speed or precision?
- Which parts of the design are luxuries rather than requirements?

Use this to keep proposals grounded in competition payoff.

---

# Suggested Competitive First Round

For a diverse first round, run these eight combinations:

1. **A + E** — contact-centric, traction-limit first
2. **B + N** — strongest possible wheel-centric case with explicit authority logic
3. **C + A** — no moving gyro bias, contact-centric state allocation
4. **D + F** — pivot/scrub-first, feedforward-first
5. **G + K** — estimator-first, fault-containment first
6. **H + J** — compute-minimalist, identifiability first
7. **I + E** — maximum capability within budget, traction-limit first
8. **O + H** — racing utility first, compute disciplined

That set should produce real architectural diversity rather than minor variations of one default design.

---

# Suggested Run Format

For each generator run:

1. Paste the **common problem statement**.
2. Append:

   > Apply the following modifier(s) strongly in this proposal: **[modifier names]**. When tradeoffs arise, prioritize the modifier guidance over conventional defaults.

3. Append:

   > Also include a short section titled **"Why this proposal is meaningfully different from a conventional wheel-speed-and-bias 9-state UKF."**

---

# Optional Scoring Rubric

Score each returned proposal from **1 to 5** on:

- hidden-state quality
- estimator robustness
- feedforward quality
- yaw-only handling
- traction-limit handling
- re-grip handling
- command-feasibility handling
- compute realism
- identifiability
- implementation clarity
- likely competition payoff

This makes the refinement round much easier and more defensible.
