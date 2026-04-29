# 02 — Residual-Noise Covariance Propagation

## Scope

This note concerns how process noise injected into the existing residual acceleration states affects the rest of the existing state vector:

\[
Px, Py, heading, Vf, Vr, yawRate, \Delta Af, \Delta Ar, \Delta yawAccel
\]

No new states are proposed.

## Current issue

The residual states are accelerations or yaw acceleration. Noise injected into them affects more than the final three state entries. It integrates into:

- `Vf`, `Vr`, and `yawRate` immediately;
- `Px`, `Py`, and `heading` through continued propagation;
- cross-covariances between residuals and motion states.

If the implementation simply adds process covariance to the last three residual-state diagonal entries after propagation, the filter under-represents motion uncertainty.

## Why it matters

This error is small per 1 ms tick, but persistent. It is most visible when repeated high-rate propagation occurs between strong exteroceptive corrections or when measurements are gated:

- open-floor straights without useful walls;
- high-speed arcs where wall updates are weak or rejected;
- launch and ground-strike events where accelerometer rows are inflated;
- in-place turns where encoder differential is explicitly not trusted as yaw truth.

The filter can become overconfident in `Vf`, `Vr`, `yawRate`, `heading`, and position even though the residual states correctly show high uncertainty.

## Recommended spec requirement

The spec should require that residual-driving noise be propagated through the nonlinear plant, not merely added to the residual-state block.

There are two acceptable approaches.

## Option A — Augmented sigma points

Augment the UKF sigma vector with process-noise variables for:

\[
\eta_{Af},\quad \eta_{Ar},\quad \eta_{yaw}
\]

and inject them through the exact discrete residual update:

\[
\Delta Af_{k+1}=\phi_f\Delta Af_k+\eta_{Af}
\]

\[
\Delta Ar_{k+1}=\phi_r\Delta Ar_k+\eta_{Ar}
\]

\[
\Delta yawAccel_{k+1}=\phi_y\Delta yawAccel_k+\eta_{yaw}
\]

Then propagate the full state normally. This is the cleanest UKF formulation if computational budget permits it.

## Option B — Sensitivity-mapped additive process noise

If the implementation uses additive process noise, define a local process-noise injection matrix:

\[
Q_x \leftarrow Q_x + G Q_\eta G^T
\]

where:

\[
Q_\eta = \operatorname{diag}(Q_{\eta Af}, Q_{\eta Ar}, Q_{\eta yaw})
\]

The columns of \(G\) should represent the effect of each residual-driving-noise innovation on the full propagated state over one time step.

A robust implementation method is finite differencing:

1. propagate the nominal state with \(\eta=0\);
2. propagate three perturbed versions with small residual innovations;
3. compute the output-state differences after wrapping heading;
4. form \(G\) from those differences.

This keeps the requirement independent of the exact integration scheme used by the plant.

## Minimum practical approximation

At minimum, the process-noise contribution should affect:

- the residual state itself;
- the corresponding velocity or yaw-rate state;
- the corresponding residual-to-motion cross-covariance.

A diagonal-only residual-state injection is not sufficient for consistency.

## Proposed spec wording

> Process noise driving `DeltaAf`, `DeltaAr`, and `DeltaYawAccel` must be propagated through the plant into the full state covariance. The implementation may use augmented sigma points or a local sensitivity mapping `Qx += G Qeta G^T`. Adding residual process noise only to the residual-state diagonal entries is not compliant, because residual acceleration uncertainty integrates into velocity, yaw rate, pose, heading, and cross-covariances.

## Acceptance checks

The implementation should pass these checks:

1. With IMU and wall updates disabled, increasing `sigmaDeltaAf` increases uncertainty in `Vf` and position, not only `DeltaAf`.
2. Increasing `sigmaDeltaYawAccel` increases uncertainty in `yawRate` and `heading`.
3. Cross-covariances between residual states and motion states are nonzero after propagation.
4. A replay with high residual noise remains statistically conservative under NIS/NEES checks where ground truth or proxy truth exists.

## Non-goal

This does not alter the state vector. It only specifies how the existing stochastic residual model must affect the existing covariance.
