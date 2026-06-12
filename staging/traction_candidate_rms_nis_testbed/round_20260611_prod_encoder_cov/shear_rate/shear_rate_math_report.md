# Shear-Rate Traction Model Math Report

Date: 2026-06-12

This report describes the mathematical formulation and design intent of the `shear_rate` traction model for a downstream production implementation effort. It intentionally avoids prescribing production ownership or code structure. The useful contract is the physical interface: compute per-contact traction forces from wheel/contact kinematics, normal load, fan load, recent contact velocity change, and a compact parameter set.

This model is a backup option, not the primary production recommendation, because it is computationally more expensive than `slip_envelope` for similar production purpose. In the corrected production-equivalent encoder covariance round, it improved corrected EKF screening relative to weaker alternatives, but its added derivative gates, smooth-sign terms, and recent-history requirements make it less attractive than the simpler slip-envelope law unless yaw-launch transient behavior proves worth the extra cost.

## Design Intent

The model starts with the same base idea as the slip-envelope family: each tire/contact patch generates an unconstrained force tendency from driven wheel surface motion, body contact motion, commanded drive force, and slip stiffness. That requested force is then clipped by a combined longitudinal/lateral friction envelope.

The additional `shear_rate` idea is a short-lived transient force term driven by rapid change in contact-patch relative velocity. The intent is to represent tire/contact shear buildup during launch and yaw-launch events without introducing a persistent tire state. It is still algebraic over the current sample and recent velocity history; it does not create a relaxation state, hysteresis state, or mode-specific launch state.

The transient term is gated in two ways:

- it activates only when the contact velocity is changing quickly enough;
- it fades as relative slip speed grows, so it behaves like a breakaway/startup effect rather than a sustained sliding-force source.

## Coordinate and Sign Conventions

Use the project body frame:

- `+X` / right is positive lateral body velocity.
- `+Y` / forward is positive longitudinal body velocity.
- `+Yaw` is clockwise.

For each contact patch `i`, define:

- `r_i = (x_i, y_i)` as contact location in body coordinates, where `x_i` is right and `y_i` is forward.
- `v_x` as body right velocity.
- `v_y` as body forward velocity.
- `omega` as yaw rate, positive clockwise.
- `R_w` as wheel radius.
- `Omega_i` as wheel angular rate for the wheel bank associated with the contact.

The contact-patch ground velocity in the body frame is:

```text
v_contact_forward_i = v_y - omega * x_i
v_contact_right_i   = v_x + omega * y_i
```

The wheel surface velocity in the forward direction is:

```text
v_surface_i = R_w * Omega_i
```

The relative slip velocity used by the base envelope is:

```text
s_forward_i = v_surface_i - v_contact_forward_i
s_right_i   = -v_contact_right_i
s_i         = sqrt(s_forward_i^2 + s_right_i^2)
```

Positive `s_forward_i` means the wheel surface is moving forward faster than the contact patch. Positive `s_right_i` means the contact kinematics call for rightward traction under this sign convention. The model is intended to be odd-symmetric: reversing a signed relative velocity or signed velocity-rate input reverses the corresponding force tendency.

## Mathematical Inputs

For each plant/control evaluation, the model needs:

- Vehicle facts: mass, yaw inertia, gravity, wheel radius, contact geometry, drive force capability per bank, and fan downforce relation.
- State or sample values: body forward velocity, body right velocity, yaw rate, left/right wheel rates, left/right drive commands, fan duty, and time step.
- Recent-history values sufficient to estimate contact velocity rate: previous wheel rate and previous yaw rate, or an equivalent authoritative derivative source.
- Per-contact normal load `N_i` and per-bank normal load `N_bank`.
- The model parameters listed below.

## Mathematical Outputs

The traction law produces per-contact body-frame forces:

```text
F_forward_i
F_right_i
```

The plant can sum them into net force and yaw moment:

```text
F_forward = sum_i F_forward_i
F_right   = sum_i F_right_i
M_yaw     = sum_i (y_i * F_right_i - x_i * F_forward_i)
```

Then:

```text
a_forward = F_forward / m
a_right   = F_right / m
alpha_yaw = M_yaw / I_yaw
```

IMU-location acceleration projection, estimator covariance handling, and measurement update logic are outside this traction-law contract.

## Fan and Normal-Load Scaling

Available friction scales through normal load:

```text
N_total = m * g + D_fan(d)
```

where `d` is fan duty cycle and `D_fan(d)` is the authoritative fan-downforce relation. The candidate screening basis used a first-order linear relation:

```text
D_fan(d) = max(d, 0) * D_fan_full
```

The simplest contact allocation is:

```text
N_i = N_total / 4
```

If production has an authoritative per-contact load-transfer model, this traction law should consume those `N_i` values directly. The parameter `peak_friction_coefficient_at_80pct_fan` names the calibration condition, but force availability still enters as:

```text
F_long_limit_i = mu_peak * N_i
F_lat_limit_i  = mu_peak * N_i
```

## Base Slip-Envelope Term

The base model uses a smooth low-speed blend:

```text
b_i = s_i / sqrt(s_i^2 + v_blend^2)
```

where:

```text
v_blend = low_speed_blend_mps
```

This makes the slip-stiffness contribution fade smoothly toward zero at very small relative slip speed while preserving signed force direction.

Drive force is apportioned by contact load share on the corresponding left/right bank:

```text
F_drive_bank = command_bank * F_drive_bank_max
w_i          = N_i / max(N_bank, epsilon)
F_drive_i    = w_i * F_drive_bank
```

The base unconstrained requested force is:

```text
F_forward_base_i = F_drive_i + b_i * K_long * s_forward_i
F_right_base_i   =             b_i * K_lat  * s_right_i
```

where:

```text
K_long = longitudinal_slip_gain_n_per_mps
K_lat  = lateral_slip_gain_n_per_mps
```

## Contact Velocity Rate Term

The shear-rate extension estimates the recent rate of change of the contact velocity drivers. A production implementation may obtain these from authoritative derivatives rather than finite differences, but the mathematical quantities are:

```text
q_forward_i = d(v_surface_i) / dt
q_right_i   = -d(omega) / dt * y_i
q_i         = sqrt(q_forward_i^2 + q_right_i^2)
```

`q_forward_i` captures rapid wheel-surface acceleration. `q_right_i` captures rapid yaw-rate change producing lateral contact shear at fore/aft contact offsets. Both terms have acceleration units, `m/s^2`.

The activation gate is:

```text
g_rate_i = q_i / sqrt(q_i^2 + A_rate^2)
```

where:

```text
A_rate = shear_rate_activation_mps2
```

Properties:

- `g_rate_i -> 0` when contact velocity is changing slowly.
- `g_rate_i -> 1` when contact velocity rate is large compared with `A_rate`.
- Larger `A_rate` makes the transient harder to activate.

The breakaway gate is:

```text
g_breakaway_i = v_breakaway / sqrt(s_i^2 + v_breakaway^2)
```

where:

```text
v_breakaway = shear_rate_breakaway_speed_mps
```

Properties:

- `g_breakaway_i -> 1` near zero relative slip speed.
- `g_breakaway_i -> 0` as relative slip speed becomes large compared with `v_breakaway`.
- Smaller `v_breakaway` makes the transient fade sooner once the contact is sliding.

The transient force magnitude scale is:

```text
F_shear_mag_i = F_shear_peak * g_rate_i * g_breakaway_i
```

where:

```text
F_shear_peak = shear_rate_peak_force_n
```

The signed component injection uses smooth signs of the velocity-rate components:

```text
sigma_forward_i = q_forward_i / sqrt(q_forward_i^2 + A_rate^2)
sigma_right_i   = q_right_i   / sqrt(q_right_i^2   + A_rate^2)

F_forward_req_i = F_forward_base_i + F_shear_mag_i * sigma_forward_i
F_right_req_i   = F_right_base_i   + F_shear_mag_i * sigma_right_i
```

This keeps each transient component bounded by `F_shear_peak` before friction-envelope clipping:

```text
abs(F_shear_mag_i * sigma_forward_i) <= F_shear_peak
abs(F_shear_mag_i * sigma_right_i)   <= F_shear_peak
```

The final force is still bounded by the combined friction envelope below, so the transient term cannot bypass normal-load/friction availability.

## Combined Friction Envelope

The requested force is normalized by the per-contact longitudinal and lateral limits:

```text
u_i = ((abs(F_forward_req_i) / F_long_limit_i)^p
     + (abs(F_right_req_i)   / F_lat_limit_i)^p)^(1 / p)
```

where:

```text
p = combined_slip_envelope_exponent
```

The final contact force is:

```text
scale_i = min(1, 1 / u_i)

F_forward_i = scale_i * F_forward_req_i
F_right_i   = scale_i * F_right_req_i
```

A production implementation may use a smooth equivalent around `u_i = 1` if estimator differentiability requires it, but the intended behavior is this p-norm envelope:

- `p = 2` is ellipse-like.
- Larger `p` is more rectangular.
- The current `shear_rate` fit uses a high exponent, so it allows relatively independent longitudinal and lateral force until both components are near their limits.

## Current Parameter Set

The corrected production-equivalent encoder covariance round is the current parameter basis for this backup model:

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `peak_friction_coefficient_at_80pct_fan` | 0.7707542806410936 | Friction coefficient used with normal load for the per-contact force envelope. |
| `longitudinal_slip_gain_n_per_mps` | 9.045432890526632 | Longitudinal slip stiffness before transient addition and envelope clipping. |
| `lateral_slip_gain_n_per_mps` | 9.568490021721773 | Lateral slip stiffness before transient addition and envelope clipping. |
| `combined_slip_envelope_exponent` | 6.731595132063319 | p-norm exponent for combined longitudinal/lateral utilization. |
| `low_speed_blend_mps` | 0.023243063531958544 | Slip-speed scale for smoothly enabling base slip stiffness. |
| `shear_rate_peak_force_n` | 0.14422627344746675 | Maximum pre-envelope transient component scale from shear-rate activation. |
| `shear_rate_activation_mps2` | 106.5948251318894 | Contact velocity-rate scale required to activate the transient term. |
| `shear_rate_breakaway_speed_mps` | 0.1281232604558458 | Relative-slip speed scale over which the transient fades after breakaway. |

Interpretation of this set:

- The base slip gains are much lower than the current `slip_envelope` candidate, so more of the launch correction is delegated to the transient term.
- The high activation threshold means only abrupt wheel/yaw-rate changes receive much transient force.
- The breakaway speed keeps the transient active around startup but fades it once contact slip becomes clear.
- The low peak friction coefficient limits how much the transient can affect final force after envelope clipping.

## Superseded Scalar-Covariance Parameter History

The following values came from an earlier scalar-covariance basis and are superseded by the corrected production-equivalent covariance round:

| Parameter | Superseded Value |
| --- | ---: |
| `peak_friction_coefficient_at_80pct_fan` | 1.4466505804648273 |
| `longitudinal_slip_gain_n_per_mps` | 52.97592732737616 |
| `lateral_slip_gain_n_per_mps` | 59.98880432712187 |
| `combined_slip_envelope_exponent` | 2.799791426373489 |
| `low_speed_blend_mps` | 0.06270098670554379 |
| `shear_rate_peak_force_n` | 0.022373710970350933 |
| `shear_rate_activation_mps2` | 67.31730936246947 |
| `shear_rate_breakaway_speed_mps` | 0.22007916033344904 |

This history is useful because it shows how sensitive the model is to covariance assumptions. The older fit used higher base grip and slip stiffness, a lower envelope exponent, a smaller transient force, a lower activation threshold, and a slower breakaway fade. It should not be treated as the current candidate parameter set unless the covariance basis is intentionally reverted.

## Expected Strengths

- Captures a plausible launch transient without adding persistent tire state.
- Keeps the final force bounded by normal load and a combined friction envelope.
- Preserves sign symmetry for forward/reverse and left/right behavior when geometry and commands are mirrored.
- Gives yaw launch a direct path through contact geometry, yaw-rate change, lateral contact velocity rate, and yaw moment arm.
- Remains compact enough for direct unit tests of force signs, gates, limits, and saturation.
- Adds a plausible transient-shear mechanism while staying far cheaper than a stateful tire model.

In the standalone propagation timing audit, this candidate was about `1.31x` the cheapest measured plant propagation path and about `1.27x` the base slip-envelope timing in that narrow host timing context. Treat those numbers as relative screening evidence only; embedded C++ cost will depend on production math choices. The important design point is directional: `shear_rate` has a real per-contact cost premium over `slip_envelope`.

## Expected Weaknesses

- The transient is derivative-sensitive. Noisy or mis-timed wheel/yaw-rate derivatives can inject false launch force.
- Because the transient is algebraic rather than stateful, it cannot represent true relaxation length, tire memory, floor pickup, thermal effects, or hysteresis.
- Static-position behavior is risky: tiny derivative or measurement inconsistencies can integrate into drift if the estimator accepts them as physical acceleration.
- The force law has more parameters than `slip_envelope`, making it easier to fit a screening corpus while losing robustness outside it.
- The current fit uses low base slip gains and relies on the transient term for launch correction, which may make non-launch behavior less stable.
- It adds cost and validation burden relative to `slip_envelope`, which is the primary reason it remains a backup model.

## Yaw-Launch Behavior

Yaw launch is the main reason this model is worth retaining as a backup despite its cost. During rapid opposing-side drive changes, yaw acceleration and wheel acceleration can create contact shear before the ordinary slip-envelope term fully explains measured planar acceleration.

The model addresses that through:

```text
q_forward_i = d(R_w * Omega_i) / dt
q_right_i   = -d(omega) / dt * y_i
F_shear_mag_i = F_shear_peak * g_rate_i * g_breakaway_i
M_yaw_i = y_i * F_right_i - x_i * F_forward_i
```

That is structurally preferable to a yaw-specific correction because yaw behavior emerges from contact geometry and signed contact forces. However, the extra math should be justified by production-relevant yaw-launch behavior before this model displaces the cheaper `slip_envelope` candidate.

## Static-Position Validation Caveat

Static-position behavior remains a validation tangent for any traction law with derivative-sensitive transient terms. With zero commands and zero wheel rates on the selected static segment, the corrected `shear_rate` configuration showed the following drift results:

| Estimator / Case | Final Radial Drift | Max Radial Drift | Pass 5 mm |
| --- | ---: | ---: | --- |
| simplex UKF, full static replay | 0.085889180254 m | 0.085889180254 m | false |
| simplex UKF, prediction-only encoder-only | 0.0257110334186 m | 0.0257110334186 m | false |

These results should be treated as estimator-validation context, not as the model-selection rationale for backup status. The backup rationale for `shear_rate` is computational cost relative to `slip_envelope`.

## Computational Cost Relative to Slip Envelope

Relative to the base slip-envelope formulation, the `shear_rate` model adds:

- recent-history access or derivative inputs for wheel rate and yaw rate;
- two velocity-rate components per contact;
- one velocity-rate magnitude;
- one activation gate;
- one breakaway gate;
- two smooth-sign component multipliers;
- the same final combined-envelope clipping.

The model should therefore be considered a modest but real cost increase over `slip_envelope`. It does not require iterative solves or persistent tire state, but it does require more math and more validation around derivative quality. This cost premium is the primary reason `shear_rate` is a backup option rather than the current production recommendation.

## Validation Expectations

Minimum validation before any production consideration:

- Unit checks for base slip-envelope behavior: zero slip, pure longitudinal slip, pure lateral slip, combined-slip clipping, and saturation scaling.
- Unit checks for sign symmetry: reversing slip or velocity-rate inputs reverses the corresponding force tendency.
- Unit checks for transient activation: low `q_i` produces near-zero transient force; high `q_i` approaches the configured peak scale.
- Unit checks for breakaway fade: the transient is strong near zero slip and fades as `s_i` grows.
- Unit checks proving the final force cannot exceed the combined friction envelope after the transient is added.
- Fan/downforce checks showing force limits scale through normal load.
- Contact geometry checks proving yaw moment sign follows `+Yaw = clockwise`.
- Static-position replay with zero commands and zero wheel rates.
- Simplex UKF replay on launch, straight, mixed-launch, and yaw-launch segments.
- Racing behavior validation before production authority is claimed.

Validation caveats:

- EKF RMS/NIS results are screening only. They can expose bad candidates and guide investigation, but they are not final production authority.
- Production authority is simplex UKF behavior and real racing behavior.
- Encoder pseudo-measurements must not be scored as encoder NIS. If pseudo-measurements are injected for stabilization or diagnostics, report them separately from real encoder measurement consistency.
- The corrected production-equivalent encoder covariance round is the current basis for parameter comparison.
- Static-position checks remain important estimator-validation coverage, especially because derivative-sensitive transient terms can expose stationary drift risks.

## Source Note

The current parameter values and validation status are from the staging artifacts under:

```text
staging/traction_candidate_rms_nis_testbed/round_20260611_prod_encoder_cov/shear_rate
```

The superseded scalar-covariance parameter history is from the earlier covariance basis and is retained only to document tuning history. This report uses the candidate name `shear_rate` only to identify the mathematical model and its staging artifacts.
