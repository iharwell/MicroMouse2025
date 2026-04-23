# Common Problem Statement

Design a **plant model, feedforward system, and UKF-based state estimation architecture** for a high-performance micromouse robot operating near the limits of traction and precision.

The goal is to produce a proposal that is:

- **robust**
- **fault-tolerant**
- **computationally realistic at 1 kHz on the target hardware**
- **highly accurate across as much of the operating envelope as possible**
- **cleanly implementable as a production system**, not merely as a research concept

The proposal must be written as a **complete technical design**, not a loose discussion. It must include:

1. a plant model
2. a feedforward / inverse path
3. a UKF architecture
4. state definitions
5. measurement models
6. process and measurement noise assumptions
7. operating-regime handling
8. command-feasibility handling
9. failure-mode handling
10. implementation guidance
11. validation and acceptance criteria

## System Context

The robot is a differential-drive micromouse-class vehicle with:

- very high quality wheel encoder data
- gyro and planar accelerometer data of similarly high quality
- strong control authority
- millimeter-scale accuracy requirements
- operation at high speed and high lateral loading
- occasional operation in yaw-dominant turning regimes

The estimator is **not** being introduced to rescue poor sensors. It is intended to fuse already-strong measurements into a more physically consistent and situationally aware estimate that improves robustness and performance in difficult regimes.

## Design Goals

The proposal should optimize for the following:

- accurate body-state estimation
- strong predictive value for feedforward and control
- explicit robustness under wheel/body disagreement
- graceful handling of slip onset and re-grip
- clean treatment of stationary and near-stationary conditions
- good behavior in yaw-dominant motion
- high-quality command handling for all valid requests

## Required Operating Regimes

The design must explicitly and convincingly address, at minimum:

1. **Certified stationary**
2. **Yaw-dominant / in-place turning**, including at least `|R| > 18 rad/s` with `|U| < 0.01 m/s`
3. **High-yaw rolling motion**, including at least `|R| > 18 rad/s` with `0.2 < |U| < 0.6 m/s`
4. **Rolling traction-limited turning**
5. **Ordinary adherent rolling**
6. **Slip onset**
7. **Re-grip / recovery**

## Envelope and Command Constraints

The operating envelope is bounded by:

- maximum forward velocity
- maximum yaw rate
- and a traction envelope whose longitudinal and lateral limits are not the same

All **valid command requests** must be handled cleanly. The proposal must define how commands are projected, limited, or modified when the requested motion is outside the feasible envelope.

## Estimator Constraints

The proposal must use a **9-state UKF**.

However, the proposal is **not** required to use any particular 9 states. The state choice must be justified against:

- observability
- practical value
- fault tolerance
- compute cost
- usefulness to the overall system

Do not assume that wheel omega states, gyro-bias states, or contact-memory states are automatically correct or incorrect. Justify the chosen 9-state allocation.

## Sensor and Modeling Expectations

The proposal must:

- align noise modeling with the actual project sensor quality
- treat the sensors as strong inputs rather than poor ones needing rescue
- explicitly separate what is **measured well already** from what is **truly hidden and worth estimating**
- explain which quantities should be used directly in control/feedforward versus which should remain estimator-internal

## Feedforward Expectations

The feedforward path should be treated as a serious part of the architecture, not an afterthought.

It must define how the system maps from a desired motion target into physically feasible actuator demand while accounting for:

- present state
- requested acceleration
- drivetrain limits
- traction limits
- yaw-dominant motion
- low-speed / static / scrub effects where relevant

## Performance Expectations

The proposal must remain computationally realistic for a 1 kHz embedded implementation. It should not depend on a heavyweight multi-filter supervisor or similar architecture unless the added complexity is convincingly justified.

## Deliverable Requirements

The proposal must include:

- an executive summary
- explicit state definition
- explicit process model
- explicit measurement model
- explicit feedforward / inverse description
- regime logic or smooth scheduling logic
- handling of valid command projection
- justification for each state
- discussion of major alternatives rejected
- validation plan
- acceptance criteria

## Output Expectations

The proposal must be:

- specific
- algebraic where practical
- technically opinionated
- written as something that could actually be handed to an implementer

Do not merely restate standard UKF or vehicle-model theory. Tailor the design to the operating regime and priorities described above.

# Common Evaluation Instructions

Apply these to every generator run.

## Required Comparison Criteria

Every proposal must explicitly discuss:

1. **Why these 9 states were chosen**
2. **Which hidden dynamics are considered important**
3. **Which measured quantities should not be estimated unnecessarily**
4. **How the design handles stationary, yaw-only, mixed yaw/rolling, and traction-limit regimes**
5. **How it avoids contaminating body state from wheel disagreement**
6. **How it handles re-grip**
7. **How feedforward handles all valid command requests**
8. **How the design would fail if wrong**
9. **Why it is a good fit for the compute budget**

## Forbidden Shortcuts

The generator must not:

- hand-wave the feedforward path
- assume wheel motion always equals body motion
- assume a generic low-speed tire model is sufficient for yaw-only operation
- keep weakly useful states merely because they are common
- hide key logic in vague adaptive tuning language

# Standard Run Footer

Append the following instruction to every run:

> Also include a short section titled **"Why this proposal is meaningfully different from a conventional wheel-speed-and-bias 9-state UKF."**
