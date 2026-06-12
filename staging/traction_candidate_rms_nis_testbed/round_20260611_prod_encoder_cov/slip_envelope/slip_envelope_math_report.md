# Slip Envelope Traction Model Math Report

Date: 2026-06-11

This report describes the mathematical formulation and design intent of the `slip_envelope` traction model for a downstream production implementation effort. It intentionally avoids prescribing production ownership, class layout, or file structure. The useful contract is the physical interface: compute per-contact traction forces from wheel surface motion, body motion, normal load, fan load, and a small parameter set.

## Design Intent

The model treats each tire/contact patch as a local velocity source that tends to generate force in the direction that reduces relative motion between the driven wheel surface and the ground contact frame. That unconstrained force tendency is then limited by a combined longitudinal/lateral friction envelope.

The intended production value is not that this candidate was the latest EKF-screening winner. The value is that it is compact, symmetric, explainable, and directly tied to contact-patch kinematics. It gives the production plant model a clean way to represent finite traction, combined slip, fan/downforce scaling, and smooth behavior near zero speed without adding transient state or mode-specific heuristics.

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

The contact patch body-frame ground velocity is:

```text
v_contact_forward_i = v_y - omega * x_i
v_contact_right_i   = v_x + omega * y_i
```

The wheel surface velocity in the forward direction is:

```text
v_surface_i = R_w * Omega_i
```

The relative slip velocity used by the model is:

```text
s_forward_i = v_surface_i - v_contact_forward_i
s_right_i   = -v_contact_right_i
s_i         = sqrt(s_forward_i^2 + s_right_i^2)
```

Positive `s_forward_i` means the wheel surface is moving forward faster than the contact patch. Positive `s_right_i` means the contact patch has relative tendency requiring rightward traction in this sign convention. The formulation is odd-symmetric: reversing the relevant relative velocity reverses the corresponding force tendency.

## Inputs

For each control/plant evaluation, the mathematical interface needs:

- Vehicle facts: mass, yaw inertia, gravity, wheel radius, track/contact geometry, drive force capability per wheel bank, and fan downforce at full duty.
- State: body forward velocity, body right velocity, yaw rate, and any plant bias terms that are already authoritative in the production plant.
- Command/sample values: left and right wheel commands, left and right wheel rates, and fan duty cycle.
- Per-contact normal load. For the base slip-envelope form, use the static load plus fan load distributed across contacts unless an authoritative production load-transfer model supplies per-contact loads.
- Model parameters listed below.

## Outputs

The model produces per-contact forces:

```text
F_forward_i
F_right_i
```

The plant can sum those into body-frame force and yaw moment:

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

Any IMU-location acceleration projection should remain part of the authoritative plant/sensor math, not part of the traction model itself.

## Fan and Normal Load Scaling

The available friction force scales with normal load:

```text
N_total = m * g + D_fan(d)
```

where `d` is fan duty cycle and `D_fan(d)` is the authoritative production fan-downforce relation. The current screening model used a linear fan relation:

```text
D_fan(d) = max(d, 0) * D_fan_full
```

For the base formulation, distribute the total load equally over four contacts:

```text
N_i = N_total / 4
```

If production already has a validated per-contact normal-load model, the slip-envelope force law can consume those `N_i` values directly. The friction coefficient parameter is named for its calibration at 80% fan duty, but the actual force limit is still `mu * N_i`; fan scaling enters through normal load.

## Low-Speed Blend

Raw slip-velocity force laws can become too abrupt around zero relative speed. The model therefore applies a smooth blend based on slip speed:

```text
b_i = s_i / sqrt(s_i^2 + v_blend^2)
```

where `v_blend = low_speed_blend_mps`.

Properties:

- `b_i -> 0` as contact relative speed approaches zero.
- `b_i -> 1` when relative speed is large compared with `v_blend`.
- The transition is smooth and sign-preserving because the blend multiplies signed force tendencies.

This prevents the slip stiffness terms from injecting discontinuous small-speed forces while retaining the commanded drive force path.

## Force Tendency Before Friction Limiting

For each contact, split commanded drive force by contact load share on that side of the drivetrain:

```text
F_drive_bank = command_bank * F_drive_bank_max
w_i          = N_i / max(N_bank, epsilon)
F_drive_i    = w_i * F_drive_bank
```

where `N_bank` is the total normal load on the same left/right bank as contact `i`.

The unconstrained requested contact force is:

```text
F_forward_req_i = F_drive_i + b_i * K_long * s_forward_i
F_right_req_i   =             b_i * K_lat  * s_right_i
```

Parameter semantics:

- `K_long = longitudinal_slip_gain_n_per_mps` is the longitudinal slip stiffness in N per m/s.
- `K_lat = lateral_slip_gain_n_per_mps` is the lateral slip stiffness in N per m/s.

This makes the model algebraic and memoryless. It does not introduce hidden tire state, relaxation length, breakaway state, or phase-specific launch state.

## Combined Friction Envelope

Each contact has available longitudinal and lateral limits:

```text
F_forward_lim_i = mu_peak * N_i
F_right_lim_i   = mu_peak * N_i
```

The normalized combined utilization is a p-norm:

```text
u_i = ((abs(F_forward_req_i) / F_forward_lim_i)^p
     + (abs(F_right_req_i)   / F_right_lim_i)^p)^(1 / p)
```

where:

```text
p = combined_slip_envelope_exponent
```

The contact force is the requested force if `u_i <= 1`. If `u_i > 1`, scale both components by the same factor so the final vector lands on the envelope:

```text
scale_i = min(1, 1 / u_i)

F_forward_i = scale_i * F_forward_req_i
F_right_i   = scale_i * F_right_req_i
```

A production implementation may use a differentiable approximation around `u_i = 1` if the plant/filter benefits from smooth derivatives, but the intended mathematical behavior is the p-norm envelope above.

The exponent controls the envelope shape:

- `p = 2` gives an ellipse-like friction circle.
- Higher `p` values make the envelope more rectangular, allowing stronger independent longitudinal and lateral force until both are near their limits.
- The current tuned values around `p = 5.4` indicate that the screening data preferred a squarer combined-slip envelope than a friction circle.

## Current Primary Parameter Set

The corrected production-equivalent encoder covariance round is the current primary parameter set:

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `peak_friction_coefficient_at_80pct_fan` | 1.1091281882474364 | Friction coefficient used with normal load to set per-contact force limits. |
| `longitudinal_slip_gain_n_per_mps` | 87.423645020917 | Longitudinal slip stiffness before envelope limiting. |
| `lateral_slip_gain_n_per_mps` | 95.30906226734128 | Lateral slip stiffness before envelope limiting. |
| `combined_slip_envelope_exponent` | 5.410495443316217 | p-norm exponent for combined longitudinal/lateral utilization. |
| `low_speed_blend_mps` | 0.2753939095614226 | Slip-speed scale for smooth low-speed blending. |

Interpretation of this fit:

- It uses relatively high slip stiffness values, so the unconstrained force tends to reach the envelope quickly once slip appears.
- It uses a lower peak friction coefficient than the earlier scalar-covariance fit, consistent with the corrected covariance round reducing the need to explain residuals through extra grip.
- It uses a wider low-speed blend than earlier fits, making the slip stiffness phase in more gradually at very low relative speeds.

## Parameter History

The following earlier fit came from the scalar-covariance round and is superseded by the corrected encoder covariance round above:

| Parameter | Superseded Value |
| --- | ---: |
| `peak_friction_coefficient_at_80pct_fan` | 1.2936470429574267 |
| `longitudinal_slip_gain_n_per_mps` | 33.15800734323761 |
| `lateral_slip_gain_n_per_mps` | 35.16069547272106 |
| `combined_slip_envelope_exponent` | 5.645826960231981 |
| `low_speed_blend_mps` | 0.10996085046880083 |

This earlier set remains useful as history because it shows the same qualitative envelope preference: a high p-norm exponent and similar longitudinal/lateral symmetry. It should not be treated as the production candidate parameter set unless the covariance basis is intentionally reverted, which is not the current assumption.

## Strengths

- The model is physically traceable: every contact force follows from wheel surface speed, body motion, normal load, command, and a combined friction limit.
- It is compact enough for production review and unit testing.
- It is symmetric between left/right and forward/reverse except for commanded drive input and contact geometry.
- It separates force tendency from force availability, making it easier to inspect whether behavior is stiffness-driven or envelope-limited.
- Fan/downforce effects enter through normal load, which is the correct place for first-order traction scaling.
- The algebraic form avoids hidden state that could interact badly with estimator initialization, boot-selected modes, or pause/resume boundaries.

## Weaknesses and Known Limits

- The model is memoryless. It cannot represent tire relaxation, transient carcass shear, thermal effects, floor dust pickup, or hysteresis.
- Equal base normal-load distribution is only a first-order assumption unless replaced by an authoritative load-transfer model.
- The friction coefficient is constant with respect to speed, load, floor region, and tire condition.
- The combined envelope is symmetric in lateral direction. It does not model different inward/outward sidewall behavior.
- The drive command enters as an immediate force request. Any motor, belt, or wheel-speed controller dynamics must be modeled elsewhere if needed.
- Yaw launch remains the dominant screening weakness. The model can express yaw contact slip through contact kinematics, but it does not add a special transient yaw-launch mechanism.

## Relation to Yaw-Launch Issues

Yaw launch stresses the model because contacts on opposite sides receive opposing drive commands while yaw rate, wheel speed, and lateral contact motion change rapidly. In the slip-envelope formulation, yaw launch affects force through:

```text
v_contact_forward_i = v_y - omega * x_i
v_contact_right_i   = v_x + omega * y_i
M_yaw_i             = y_i * F_right_i - x_i * F_forward_i
```

That is the right structural path: yaw behavior emerges from per-contact geometry and force, not from a yaw-specific correction term. However, screening showed yaw-launch residuals remain large under this formulation. This suggests that the missing behavior may be in one or more of:

- motor or wheel-speed dynamics during command reversal,
- transient tire/shear dynamics,
- per-contact load transfer,
- floor/tire directional effects,
- estimator/process covariance assumptions,
- sensor timing or measurement interpretation.

The production implementation agent should preserve the clean contact formulation and validate yaw launch against the authoritative production estimator and racing behavior before adding any yaw-specific model extension.

## Why This Is Still a Clean Production Candidate

The model is a clean candidate because it defines one small, inspectable traction law with parameters that have direct physical meaning. It is suitable as a production candidate even though it was not the latest EKF-screening winner because screening rank is not the same as production authority.

The production decision should prefer models that:

- preserve clear ownership of vehicle facts and plant equations,
- behave predictably outside the tuning corpus,
- remain testable by direct force/envelope checks,
- avoid per-mode heuristics,
- can be validated with the simplex UKF and racing behavior.

By those criteria, slip-envelope is a strong mathematical baseline. It may also serve as the canonical base law if a later production extension adds validated load transfer or transient shear in a controlled way.

## Validation Expectations

Minimum validation for production consideration:

- Unit checks for sign symmetry: reversing slip reverses force.
- Unit checks for zero slip: slip-stiffness force contribution approaches zero smoothly.
- Unit checks for pure longitudinal and pure lateral limits.
- Unit checks for combined-slip scaling at `u = 1` and `u > 1`.
- Fan/downforce checks showing force limits scale with normal load.
- Contact geometry checks showing yaw moment sign follows `+Yaw = clockwise`.
- Replay checks on launch, straight, mixed launch, static, and yaw-launch segments.
- Explicit comparison between commanded force tendency, envelope utilization, and final saturated force.

Validation caveats:

- EKF RMS/NIS results are screening results only. They can reject obviously bad candidates or expose failure modes, but they are not final production authority.
- Production authority is the simplex UKF and observed racing behavior.
- Encoder pseudo-measurements must not be scored as encoder NIS. If a validation path injects pseudo-measurements, it must report them separately from real encoder measurement consistency.
- The corrected production-equivalent encoder covariance round is the current basis for parameter comparison.
- Tuning should not be done against yaw launch alone unless the production goal explicitly accepts the resulting tradeoffs on static, straight, and mixed-launch behavior.

## Source Note

This report summarizes the candidate historically named `candidate_1_algebraic_envelope` and currently referred to as `slip_envelope` in the staging artifacts. The primary parameter values are from:

```text
staging/traction_candidate_rms_nis_testbed/round_20260611_prod_encoder_cov/slip_envelope
```

The superseded parameter history is from the earlier scalar-covariance `round_20260611` slip-envelope artifacts.
