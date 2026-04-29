# 01 — Stochastic Model Semantics

## Scope

This note concerns only the existing process-noise scheduling and the three existing colored residual acceleration states:

\[
\Delta Af,\quad \Delta Ar,\quad \Delta yawAccel
\]

It does not propose adding states or sensors.

## Current issue

The spec defines residual dynamics in continuous time:

\[
\dot{\Delta A}=-\frac{\Delta A}{\tau}+w
\]

and later schedules quantities such as:

\[
\sigma_{\Delta Af},\quad \sigma_{\Delta Ar},\quad \sigma_{\Delta yawAccel}
\]

The ambiguity is what those sigmas mean. They could be interpreted as any of the following:

- continuous-time white-noise spectral-density parameters;
- discrete per-tick innovation standard deviations;
- steady-state residual-state standard deviations;
- heuristic covariance inflation values.

Those interpretations produce materially different filter behavior at a 1 kHz propagation rate.

## Why it matters

At 1 ms, a colored residual process can either become nearly frozen or nearly white depending on how `sigma` is discretized. Two implementations can use the same equations and schedules but get different estimator consistency, especially in these regimes:

- launch threshold crossing;
- in-place scrub breakaway;
- high-speed stick-slip arcs;
- ground-strike clipping events.

This is not a tuning detail. It is part of the mathematical plant/noise model.

## Recommended spec decision

Define `sigma` as one of the following and make every scheduled term use that definition.

### Preferred interpretation: scheduled steady-state residual standard deviation

For each residual state, define:

\[
\phi = e^{-\Delta t/\tau}
\]

\[
\Delta A_{k+1}=\phi \Delta A_k + \eta_k
\]

\[
\eta_k \sim \mathcal N\left(0,\sigma_{\Delta A,ss}^2(1-\phi^2)\right)
\]

where \(\sigma_{\Delta A,ss}\) is the scheduled steady-state standard deviation of that residual state in the current regime.

This is usually the most intuitive convention for tuning because a schedule such as “increase residual acceleration uncertainty during stick-slip” directly controls the expected residual-state magnitude, not a time-step-dependent noise density.

### Alternative interpretation: continuous-time spectral density

If the scheduled value is a continuous-time driving-noise spectral-density parameter \(q_c\), then the discrete residual-state noise is:

\[
Q_{\Delta A}=q_c\frac{\tau}{2}\left(1-e^{-2\Delta t/\tau}\right)
\]

This is valid, but less convenient for tuning unless the spec consistently uses spectral-density units.

## Proposed spec wording

Add a short subsection under process-noise scheduling:

> The scheduled values `sigmaDeltaAf`, `sigmaDeltaAr`, and `sigmaDeltaYawAccel` are steady-state standard deviations of the corresponding colored residual states. The discrete residual update is an exact OU update with `phi = exp(-dt/tau)` and process variance `sigma^2 * (1 - phi^2)`. These sigmas are not per-tick standard deviations and are not continuous-time spectral-density values.

If the project prefers spectral density, replace that sentence with the spectral-density equation above.

## Additional requirement

Whenever process noise is scheduled continuously, freeze the schedule over the propagation substep or evaluate it consistently at the sigma-point state. The spec should state which convention is used.

Recommended convention:

- compute regime scalars for each sigma point during propagation;
- compute the corresponding residual OU variance for that sigma point;
- if the UKF implementation cannot support sigma-point-specific process noise, use the predicted-mean regime scalars and inflate conservatively.

## Acceptance checks

The implementation should pass these checks:

1. With constant `sigma` and `tau`, a stationary residual-only simulation converges to RMS \(\sigma\), independent of `dt`.
2. Changing `dt` from 1.0 ms to 0.5 ms with the same continuous trajectory does not materially change residual variance.
3. With `sigma = 0`, residual states decay exactly by \(e^{-dt/\tau}\).
4. Scheduled noise does not jump discontinuously at regime boundaries.

## Non-goal

This note does not recommend adding slip states, bias states, or current states. It only makes the existing residual-state stochastic model exact.
