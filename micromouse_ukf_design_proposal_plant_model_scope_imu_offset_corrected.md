# Proposed Runtime UKF Plant and Measurement Model for Micromouse Vehicle State Estimation

Status: proposal  
Target loop: 1 kHz control/update tick  
Primary objective: runtime prediction and situational awareness for aggressive motion, not offline parameter identification

---

## 1. Overarching design philosophy

This estimator is not intended to make the robot conservative. It is intended to let the control system operate closer to the physical boundary with better awareness of where the vehicle actually is, how strongly the model is being violated, and whether the next control command is likely to remain controllable.

The design philosophy is:

1. **Estimate body motion, not wheel odometry.** Encoders are high-quality drivetrain measurements, not direct measurements of ground-relative robot motion. This matters because the intended operating envelope includes in-place tire scrub, stick-slip arcing turns, and traction excursions.
2. **Do not clamp the plant model to the traction limit.** A traction clamp would remove the information the controller needs near the boundary. Instead, the model predicts nominal behavior, residual states absorb mismatch, and margin outputs quantify when the prediction is becoming physically dubious.
3. **Use the UKF for short-horizon dynamic state correction and margin monitoring.** The UKF is not a magic global-localization system. In open-floor modes, pose drift remains expected. The UKF is mainly valuable for combining IMU, drive model, encoder-derived drivetrain state, and opportunistic wall observations into a coherent tick-boundary vehicle belief.
4. **Keep sensor timing explicit.** The estimator must support sequential measurement updates because the sensors are not sampled simultaneously. IMU data, encoder deltas, and wall sensor groups arrive with different timing. Wall sensors require LED-on analog settling before ADC readout; front sensors are sampled as a pair, while side sensors are sampled separately.
5. **Prefer small, mode-dependent updates over one monolithic measurement vector.** A partitioned update lets the scheduler interleave UKF work with wall sensor settling, skip unavailable wall measurements, and gate bad wall readings without discarding IMU updates.
6. **Use residual states instead of explicit slip states for the first implementation.** Directly estimating slip ratios or tire friction states would require observability and parameterization that are not yet established. Residual acceleration states give much of the runtime benefit with lower dimension and lower risk.


### 1.1 Scope boundary

This document specifies the vehicle plant model, measurement models, sensor timing assumptions, and the information that the estimator must expose to the rest of the runtime architecture. It does not specify the generic mathematical machinery of the UKF core.

The reusable UKF core owns details such as sigma-point generation, unscented-transform parameters, covariance-update form, square-root versus covariance representation, and numerical stabilization strategy. Those implementation choices must remain outside this plant-model specification.

---

## 2. Design goals

### 2.1 Functional goals

The UKF should provide, at the next control activation boundary:

- `Px`, `Py`, `heading`
- `Vf`, `Vr`
- `yawRate`
- model residuals or equivalent mismatch indicators
- covariance-derived validity indicators
- wall-relative correction when wall sensors are valid
- slip / model-mismatch indicators for control diagnostics
- control-margin indicators without forcing command saturation

### 2.2 Runtime goals

The design is constrained by a 1 ms control tick. A 9-state UKF is already a significant fraction of the tick, so this proposal avoids:

- multiple concurrent UKFs
- large augmented parameter-state filters
- explicit tire-state estimation
- large batch measurement updates
- wall-update models that require heavy geometry searches inside the hot path

### 2.3 Non-goals

This proposal does **not** attempt to:

- estimate detailed tire friction parameters online
- perfectly model stick-slip microdynamics
- replace dedicated calibration procedures
- infer battery voltage, since it is not presently sensed
- turn encoders into ground-truth pose measurements during slip
- enforce traction-limited commands

---

## 3. Research and source basis

The UKF is selected because it propagates sigma points through nonlinear process and measurement models rather than relying on first-order linearization. Julier and Uhlmann describe the UKF as a way to address nonlinear filtering cases where EKF implementation and tuning become difficult; Wan and van der Merwe similarly present the UKF as an alternative to EKF linearization for nonlinear estimation.

Robotics practice also supports probabilistic state estimation as an explicit way to represent uncertainty rather than treating perception as deterministic. That matters here because the robot intentionally operates in regimes where encoders, IMU, and wall sensors can disagree.

Project-specific constraints dominate the design:

- In-place turns require tire scrub and exceed ordinary rolling assumptions.
- Straight motion has launch thresholds and roughly constant friction rather than clean viscous behavior.
- Arcing turns at target speeds/radii require stick-slip and traction excursions.
- Drivetrain force transfer stabilizes quickly relative to the 1 ms loop.
- The operating envelope includes full-bore acceleration, precision turns with small wall clearance, in-place calibration turns, and constant-velocity exploration/wall mapping.

The LSM6DSV16X supports high-rate accelerometer/gyro operation, with available accelerometer and gyroscope ODRs up to 7.68 kHz, and supports data-ready/FIFO mechanisms. Its gyro and accelerometer characteristics are strong enough to justify direct dynamic updates, but not enough to remove the need for calibration, gating, and covariance management.

---

## 4. Coordinate and naming conventions

This proposal uses the project coordinate conventions:

- Global frame:
  - `+Y` = forward
  - `+X` = right
- Body frame:
  - `+f` = robot forward
  - `+r` = robot right
- `heading = 0` means robot forward is aligned with global `+Y`
- positive `heading`, `yawRate`, and `yawAccel` are clockwise

Code naming:

| Math | Code |
|---|---|
| \(P_x, P_y\) | `Px`, `Py` |
| \(P_f, P_r\) | `Pf`, `Pr` |
| \(V_f, V_r\) | `Vf`, `Vr` |
| \(A_f, A_r\) | `Af`, `Ar` |
| \(\Theta\) | `heading` |
| \(\omega\) | `yawRate` |
| \(\alpha\) | `yawAccel` |
| \(C_L, C_R\) | `CL`, `CR` |

Mathematical vectors use the U+034D under-arrow notation where practical, e.g. \(x͍\), \(z͍\). Code should use the `_vec` suffix where that notation is unsupported.

---

## 5. Proposed state vector

The proposed 9-state UKF state is:

\[
x͍ =
\begin{bmatrix}
P_x \\
P_y \\
\Theta \\
V_f \\
V_r \\
\omega \\
\Delta A_f \\
\Delta A_r \\
\Delta \alpha
\end{bmatrix}
\]

| State | Code name | Units | Meaning |
|---|---|---:|---|
| \(P_x\) | `Px` | m | global rightward position |
| \(P_y\) | `Py` | m | global forward position |
| \(\Theta\) | `heading` | rad | clockwise-positive heading from +Y |
| \(V_f\) | `Vf` | m/s | body-frame forward velocity |
| \(V_r\) | `Vr` | m/s | body-frame rightward velocity |
| \(\omega\) | `yawRate` | rad/s | clockwise yaw rate |
| \(\Delta A_f\) | `deltaAf` | m/s² | forward acceleration residual |
| \(\Delta A_r\) | `deltaAr` | m/s² | lateral acceleration residual |
| \(\Delta \alpha\) | `deltaYawAccel` | rad/s² | yaw acceleration residual |

### 5.1 Why this state vector

This state vector is deliberately not a differential-drive odometry state. It estimates body velocity directly and carries model mismatch as residual acceleration states.

The residual states are included because the plant will intentionally operate where a clean no-slip model is false:

\[
A_{f,actual} = A_{f,nom} + \Delta A_f
\]

\[
A_{r,actual} = A_{r,nom} + \Delta A_r
\]

\[
\alpha_{actual} = \alpha_{nom} + \Delta \alpha
\]

This lets the estimator say "the model is wrong by this much" without pretending the wheels directly determine ground motion.

### 5.2 Why not include wheel speeds as states

Wheel-bank speeds are available from encoders at high resolution. The useful distinction is not whether wheel speed is known, but whether wheel speed equals ground motion. In this design, wheel speeds are treated as measured inputs/features:

\[
\dot{\phi}_{w,L},\quad \dot{\phi}_{w,R},\quad \ddot{\phi}_{w,L},\quad \ddot{\phi}_{w,R}
\]

They feed the plant model, but are not measurement updates on \(V_f\) or \(\omega\) during high-slip operation.

Adding wheel speeds to the state would cost two states and add a measurement update that mostly filters an already strong encoder-derived signal. That is not justified until logs show encoder quantization, timing jitter, or derivative filtering as a dominant estimator error source.

### 5.3 Why not include slip ratios or traction state

Slip ratio and friction state are attractive conceptually but poor first-pass UKF states here:

- they are not directly measured
- they are weakly observable during many short maneuvers
- they require a tire model that is not yet identified
- they add state dimension and runtime cost
- their estimates can become numerically confident but physically wrong

The residual states are a lower-risk proxy. They do not claim to know why the model is wrong; they estimate the effect of that mismatch on the vehicle.

---

## 6. Inputs and measured quantities

### 6.1 Control inputs

\[
C_L,\quad C_R
\]

These are the active motor commands during the propagation interval. Units are `[PWM units]` or normalized command units, depending on the firmware representation.

### 6.2 Encoder-derived drivetrain inputs

Raw encoder counts:

\[
N_L,\quad N_R
\]

Motor shaft angle:

\[
\phi_{m,i} = N_i \cdot \frac{2\pi}{N_{rev}}
\]

Wheel-bank angle:

\[
\phi_{w,i} = \frac{\phi_{m,i}}{G}
\]

Wheel-bank angular velocity:

\[
\dot{\phi}_{w,i,k} =
\frac{\phi_{w,i,k} - \phi_{w,i,k-1}}{\Delta t}
\]

Wheel-bank angular acceleration:

\[
\ddot{\phi}_{w,i,k} =
\frac{\dot{\phi}_{w,i,k} - \dot{\phi}_{w,i,k-1}}{\Delta t}
\]

where \(i \in \{L,R\}\).

Wheel surface speed:

\[
S_i = r_w \dot{\phi}_{w,i}
\]

Wheel surface acceleration:

\[
\dot{S}_i = r_w \ddot{\phi}_{w,i}
\]

No-slip pseudo-velocity references are computed only as model features or low-slip diagnostics:

\[
V_{f,wheel} =
\frac{S_L + S_R}{2}
\]

\[
\omega_{wheel} =
\frac{S_R - S_L}{b}
\]

They are not direct UKF measurements in high-slip modes.

---

## 7. Process model overview

The process model predicts the next state from:

\[
x͍_k,\quad C_L,\quad C_R,\quad
\dot{\phi}_{w,L},\quad \dot{\phi}_{w,R},\quad
\ddot{\phi}_{w,L},\quad \ddot{\phi}_{w,R},\quad \Delta t
\]

The propagation model is:

\[
x͍_{k+1} = f(x͍_k, u_k, \Delta t) + w͍_k
\]

where \(w͍_k\) is process noise.

---

## 8. Transform equations

### 8.1 Body to global velocity

With heading \( \Theta \), clockwise positive from global +Y:

\[
V_x = V_f \sin(\Theta) + V_r \cos(\Theta)
\]

\[
V_y = V_f \cos(\Theta) - V_r \sin(\Theta)
\]

### 8.2 Global position update

\[
\dot{P}_x = V_x
\]

\[
\dot{P}_y = V_y
\]

so:

\[
\dot{P}_x =
V_f \sin(\Theta) + V_r \cos(\Theta)
\]

\[
\dot{P}_y =
V_f \cos(\Theta) - V_r \sin(\Theta)
\]

### 8.3 Angular kinematics

\[
\dot{\Theta} = \omega
\]

### 8.4 Body-frame velocity dynamics

The body-frame velocity derivatives are related to body-frame inertial accelerations by:

\[
\dot{V}_f = A_f + \omega V_r
\]

\[
\dot{V}_r = A_r - \omega V_f
\]

Check case: steady right turn with \(V_f > 0\), \(V_r = 0\), and \(\omega > 0\). To keep \(V_r = 0\), the lateral acceleration must satisfy \(A_r = \omega V_f\), which is the expected rightward centripetal acceleration.

---

## 9. Nominal acceleration model

The model separates nominal dynamics from residuals:

\[
A_f = A_{f,nom} + \Delta A_f
\]

\[
A_r = A_{r,nom} + \Delta A_r
\]

\[
\alpha = \alpha_{nom} + \Delta \alpha
\]

The nominal model is not expected to be correct under all slip conditions. It must be good enough to provide a useful prediction and a meaningful residual.

---

## 10. Longitudinal force model

For each side \(i \in \{L,R\}\), define a softened sign function:

\[
\operatorname{softsign}(x; x_\epsilon) =
\frac{x}{\sqrt{x^2 + x_\epsilon^2}}
\]

Define effective command after launch deadband:

\[
C_{eff,i} =
\operatorname{sgn}(C_i)
\cdot
\max\left(0,\frac{|C_i| - C_{launch}}{1 - C_{launch}}\right)
\]

For implementation without a hard max, a smooth approximation may be used:

\[
\operatorname{softmax0}(x; \epsilon)
=
\frac{x + \sqrt{x^2+\epsilon^2}}{2}
\]

\[
C_{eff,i} =
\operatorname{sgn}(C_i)
\cdot
\operatorname{softmax0}
\left(
\frac{|C_i| - C_{launch}}{1 - C_{launch}};
\epsilon_C
\right)
\]

The bank drive force proposal is:

\[
F_i =
K_C C_{eff,i}
-
K_\omega \dot{\phi}_{w,i}
-
F_{c,drive}\operatorname{softsign}(S_i;S_\epsilon)
\]

where:

| Parameter | Units | Meaning |
|---|---:|---|
| \(K_C\) | N/[command] | command-to-force gain |
| \(K_\omega\) | N/(rad/s) | back-EMF / speed-load coefficient |
| \(F_{c,drive}\) | N | constant drivetrain/rolling loss per side |
| \(S_\epsilon\) | m/s | smoothing speed |

This is intentionally semi-empirical. Because battery voltage is not presently measured, a purely voltage-based motor model would create false precision. The normalized command model can be calibrated from launch and acceleration tests, and the residual states cover drift and mismatch.

The total forward drive acceleration is:

\[
A_{drive} =
\frac{F_L + F_R}{m}
\]

A simple forward damping term is:

\[
A_{f,damp} =
- K_{Vf} V_f
- A_{c,f}\operatorname{softsign}(V_f;V_\epsilon)
\]

Given project observations that straight motion has roughly zero viscous resistance, \(K_{Vf}\) should start near zero. The constant term \(A_{c,f}\) captures rolling/friction losses.

Thus:

\[
A_{f,nom} =
A_{drive} + A_{f,damp}
\]

or expanded:

\[
A_{f,nom} =
\frac{F_L + F_R}{m}
-
K_{Vf} V_f
-
A_{c,f}\operatorname{softsign}(V_f;V_\epsilon)
\]

---

## 11. Yaw acceleration model

The nominal drive yaw torque is:

\[
\tau_{drive} =
\frac{b_{eff}}{2}(F_R - F_L)
\]

where \(b_{eff}\) is the effective track width. Start with the measured wheel-bank center spacing and later identify any speed/radius-dependent effective-track correction.

The yaw resistance model is:

\[
\tau_{resist} =
- \tau_{c,yaw}\operatorname{softsign}(\omega;\omega_\epsilon)
- K_\omega^{yaw}\omega
\]

The constant yaw resistance term matters because in-place turns require tire scrub and show a roughly constant turning resistance.

The nominal yaw acceleration is:

\[
\alpha_{nom} =
\frac{\tau_{drive} + \tau_{resist}}{I_z}
\]

Expanded:

\[
\alpha_{nom} =
\frac{
\frac{b_{eff}}{2}(F_R-F_L)
-
\tau_{c,yaw}\operatorname{softsign}(\omega;\omega_\epsilon)
-
K_\omega^{yaw}\omega
}{I_z}
\]

Do not clamp \(\alpha_{nom}\) to a traction-derived maximum. Instead, use residual growth, innovation behavior, and margin estimates to indicate that the nominal model is operating beyond its identified regime.

---

## 12. Lateral acceleration model

The lateral acceleration model should not pretend to know detailed tire slip. Use a minimal physically interpretable nominal model:

\[
A_{r,nom} =
\omega V_f
-
K_{Vr} V_r
\]

where:

| Term | Meaning |
|---|---|
| \(\omega V_f\) | centripetal acceleration required for steady no-lateral-slip turning |
| \(-K_{Vr}V_r\) | weak lateral velocity relaxation |

This gives the correct steady-turn behavior:

If \(V_r = 0\) and \(\omega, V_f\) are constant:

\[
A_{r,nom} = \omega V_f
\]

and the body-frame lateral velocity derivative becomes:

\[
\dot{V}_r =
A_{r,nom} - \omega V_f = 0
\]

If the vehicle has lateral velocity \(V_r\), the model tends to decay it with time constant:

\[
\tau_{Vr} = \frac{1}{K_{Vr}}
\]

This is a nominal stabilizing assumption, not a tire-state estimate. During stick-slip, \(\Delta A_r\) absorbs mismatch.

---

## 13. Residual state dynamics

Use first-order decay plus process noise:

\[
\dot{\Delta A_f} =
-\frac{1}{\tau_{\Delta Af}}\Delta A_f + w_{\Delta Af}
\]

\[
\dot{\Delta A_r} =
-\frac{1}{\tau_{\Delta Ar}}\Delta A_r + w_{\Delta Ar}
\]

\[
\dot{\Delta \alpha} =
-\frac{1}{\tau_{\Delta \alpha}}\Delta \alpha + w_{\Delta \alpha}
\]

These residuals should not be constant biases unless testing proves that behavior is superior. A decay model prevents old slip/mismatch events from contaminating later segments indefinitely.

Recommended starting point:

| Residual | Initial time constant |
|---|---:|
| \(\tau_{\Delta Af}\) | 20-100 ms |
| \(\tau_{\Delta Ar}\) | 10-50 ms |
| \(\tau_{\Delta\alpha}\) | 10-50 ms |

Shorter time constants make residuals event-like; longer time constants make them bias-like. For aggressive turn recovery with only about 20 mm of straight travel between arcs in limiting cases, lateral and yaw residuals should decay quickly enough that a bad turn does not bias the next turn for too long.

---

## 14. Full continuous-time process model

\[
\dot{P}_x =
V_f \sin(\Theta) + V_r \cos(\Theta)
\]

\[
\dot{P}_y =
V_f \cos(\Theta) - V_r \sin(\Theta)
\]

\[
\dot{\Theta} = \omega
\]

\[
\dot{V}_f =
A_{f,nom} + \Delta A_f + \omega V_r
\]

\[
\dot{V}_r =
A_{r,nom} + \Delta A_r - \omega V_f
\]

\[
\dot{\omega} =
\alpha_{nom} + \Delta \alpha
\]

\[
\dot{\Delta A_f} =
-\frac{1}{\tau_{\Delta Af}}\Delta A_f + w_{\Delta Af}
\]

\[
\dot{\Delta A_r} =
-\frac{1}{\tau_{\Delta Ar}}\Delta A_r + w_{\Delta Ar}
\]

\[
\dot{\Delta \alpha} =
-\frac{1}{\tau_{\Delta \alpha}}\Delta \alpha + w_{\Delta \alpha}
\]

with:

\[
A_{f,nom} =
\frac{F_L + F_R}{m}
-
K_{Vf} V_f
-
A_{c,f}\operatorname{softsign}(V_f;V_\epsilon)
\]

\[
A_{r,nom} =
\omega V_f - K_{Vr}V_r
\]

\[
\alpha_{nom} =
\frac{
\frac{b_{eff}}{2}(F_R-F_L)
-
\tau_{c,yaw}\operatorname{softsign}(\omega;\omega_\epsilon)
-
K_\omega^{yaw}\omega
}{I_z}
\]

\[
F_i =
K_C C_{eff,i}
-
K_\omega \dot{\phi}_{w,i}
-
F_{c,drive}\operatorname{softsign}(S_i;S_\epsilon)
\]

\[
S_i = r_w\dot{\phi}_{w,i}
\]

---

## 15. Discrete propagation

Use one of two implementations.

### 15.1 Recommended first implementation: semi-implicit Euler

Evaluate accelerations at the beginning of the substep.

\[
P_{x,k+1} =
P_{x,k}
+
\Delta t
\left(
V_{f,k}\sin\Theta_k + V_{r,k}\cos\Theta_k
\right)
\]

\[
P_{y,k+1} =
P_{y,k}
+
\Delta t
\left(
V_{f,k}\cos\Theta_k - V_{r,k}\sin\Theta_k
\right)
\]

\[
\Theta_{k+1} =
\Theta_k + \Delta t \omega_k
\]

\[
V_{f,k+1} =
V_{f,k} + \Delta t
\left(
A_{f,nom,k} + \Delta A_{f,k} + \omega_k V_{r,k}
\right)
\]

\[
V_{r,k+1} =
V_{r,k} + \Delta t
\left(
A_{r,nom,k} + \Delta A_{r,k} - \omega_k V_{f,k}
\right)
\]

\[
\omega_{k+1} =
\omega_k + \Delta t
\left(
\alpha_{nom,k} + \Delta \alpha_k
\right)
\]

\[
\Delta A_{f,k+1} =
\Delta A_{f,k}
\exp\left(-\frac{\Delta t}{\tau_{\Delta Af}}\right)
\]

\[
\Delta A_{r,k+1} =
\Delta A_{r,k}
\exp\left(-\frac{\Delta t}{\tau_{\Delta Ar}}\right)
\]

\[
\Delta \alpha_{k+1} =
\Delta \alpha_k
\exp\left(-\frac{\Delta t}{\tau_{\Delta \alpha}}\right)
\]

Process noise is added through \(Q\), not directly in deterministic sigma-point propagation.

### 15.2 More accurate option: midpoint integration

For better prediction during high yaw-rate arcs, evaluate a midpoint state:

\[
x͍_{mid} =
x͍_k + \frac{\Delta t}{2} f(x͍_k,u_k)
\]

then:

\[
x͍_{k+1} =
x͍_k + \Delta t f(x͍_{mid},u_k)
\]

This is more accurate but roughly doubles plant-model evaluation cost. Because runtime is tight, semi-implicit Euler is acceptable for the first implementation, with midpoint reserved for targeted testing.

---

## 16. Measurement models

The UKF measurement model is partitioned.

Recommended update order within a tick:

\[
predict
\rightarrow
gyro
\rightarrow
accelerometer
\rightarrow
front\ wall\ pair
\rightarrow
left\ wall
\rightarrow
right\ wall
\]

The actual scheduler may interleave these with LED settling and sensor reads.

---

## 17. IMU measurement model

The IMU API is assumed to output canonical body-frame values. Axis/sign correction is handled in the hardware abstraction layer, not in the estimator.

### 17.1 Gyro

\[
z_{\omega} =
\omega + b_{\omega,external} + v_\omega
\]

If gyro bias is calibrated outside the UKF before the sample enters the estimator:

\[
z_{\omega} =
\omega + v_\omega
\]

Recommended first implementation: keep gyro bias outside this 9-state UKF unless testing proves the 9-state design cannot hold yaw over required windows.

### 17.2 Accelerometer

The accelerometer does not measure acceleration at the vehicle CoG. It measures body-frame inertial acceleration at the IMU package location. Therefore, the measurement model must include the rigid-body acceleration terms from yaw acceleration and yaw rate.

Define the IMU offset from the CoG in body-frame coordinates:

\[
r_{imu,f} = -0.011\ \text{m}
\]

\[
r_{imu,r} = -0.023\ \text{m}
\]

where:

- \(r_{imu,f}\) is positive forward; the IMU is 11 mm behind the CoG, so it is negative.
- \(r_{imu,r}\) is positive rightward; the IMU is 23 mm left of the CoG, so it is negative.

For each sigma point, compute the CoG acceleration and yaw acceleration:

\[
A_{f,cg} = A_{f,nom} + \Delta A_f
\]

\[
A_{r,cg} = A_{r,nom} + \Delta A_r
\]

\[
\alpha_{actual} = \alpha_{nom} + \Delta \alpha
\]

With the project convention that clockwise yaw is positive, the planar rigid-body acceleration at the IMU is:

\[
A_{f,imu} = A_{f,cg} - \alpha_{actual} r_{imu,r} - \omega^2 r_{imu,f}
\]

\[
A_{r,imu} = A_{r,cg} + \alpha_{actual} r_{imu,f} - \omega^2 r_{imu,r}
\]

Substituting the actual offset values:

\[
A_{f,imu} = A_{f,cg} + 0.023\alpha_{actual} + 0.011\omega^2
\]

\[
A_{r,imu} = A_{r,cg} - 0.011\alpha_{actual} + 0.023\omega^2
\]

The accelerometer measurement equations are then:

\[
z_{Af} = A_{f,imu} + v_{Af}
\]

\[
z_{Ar} = A_{r,imu} + v_{Ar}
\]

The offset terms are small but not negligible during aggressive turns. At high yaw rate, the centripetal term \(\omega^2 r\) can be comparable to real lateral/forward residuals, and during fast yaw acceleration the tangential terms \(\alpha r\) directly perturb the IMU acceleration relative to the CoG.

This is not redundant with the process model. The process model predicts how CoG velocity changes; the accelerometer update observes acceleration at the IMU location and must therefore map the CoG acceleration hypothesis to the actual sensor point before comparing against measurement.

### 17.3 Vertical and fault channels

Vertical acceleration and out-of-plane gyro channels should not enter the nominal state update. They should be used for validity/fault logic:

- ground strike
- chassis bounce
- wall impact
- sensor saturation
- loss of normal contact

These conditions should gate measurement trust and/or log flags.

---

## 18. Encoder handling

Encoders are not measurement updates on \(V_f\), \(V_r\), \(P_x\), \(P_y\), or \(\omega\) in high-slip modes.

They are used as:

1. drivetrain inputs:
   \[
   \dot{\phi}_{w,L},\quad \dot{\phi}_{w,R}
   \]
2. model features:
   \[
   S_L,\quad S_R,\quad V_{f,wheel},\quad \omega_{wheel}
   \]
3. diagnostics:
   \[
   V_f - V_{f,wheel}
   \]
   \[
   \omega - \omega_{wheel}
   \]
4. optional low-slip pseudo-measurements in explicitly gated modes.

Low-slip pseudo-measurements may be enabled during slow exploration or controlled straight motion:

\[
z_{Vf,wheel} =
V_f + v_{Vf,wheel}
\]

\[
z_{\omega,wheel} =
\omega + v_{\omega,wheel}
\]

with:

\[
z_{Vf,wheel} =
\frac{r_w}{2}(\dot{\phi}_{w,L}+\dot{\phi}_{w,R})
\]

\[
z_{\omega,wheel} =
\frac{r_w}{b}(\dot{\phi}_{w,R}-\dot{\phi}_{w,L})
\]

These updates must be disabled or heavily deweighted during:

- in-place turns
- high-slip arcs
- launch transients
- hard braking
- wall touch actions
- any mode where stick-slip is expected

---

## 19. Wall sensor measurement model

Wall sensor updates are optional and mode-gated.

Each wall sensor has a body-frame sensor pose:

\[
s͍_j =
\begin{bmatrix}
s_{f,j} \\
s_{r,j}
\end{bmatrix}
\]

and a body-frame beam unit vector:

\[
q͍_j =
\begin{bmatrix}
q_{f,j} \\
q_{r,j}
\end{bmatrix}
\]

with:

\[
q_{f,j}^2 + q_{r,j}^2 = 1
\]

### 19.1 Sensor position in global frame

\[
P_{x,s,j}
=
P_x + s_{f,j}\sin\Theta + s_{r,j}\cos\Theta
\]

\[
P_{y,s,j}
=
P_y + s_{f,j}\cos\Theta - s_{r,j}\sin\Theta
\]

### 19.2 Beam direction in global frame

\[
q_{x,j}
=
q_{f,j}\sin\Theta + q_{r,j}\cos\Theta
\]

\[
q_{y,j}
=
q_{f,j}\cos\Theta - q_{r,j}\sin\Theta
\]

### 19.3 Distance to a wall line

Represent a candidate wall segment as a line:

\[
n_x P_x + n_y P_y = c
\]

where \(n͍=[n_x,n_y]^T\) is the wall normal.

The predicted ray distance is:

\[
\hat{d}_j =
\frac{
c - n_x P_{x,s,j} - n_y P_{y,s,j}
}{
n_x q_{x,j} + n_y q_{y,j}
}
\]

The measurement is valid only if:

\[
\hat{d}_j > 0
\]

and the ray intersection lies on the finite wall segment.

### 19.4 Measurement equation

If the wall sensor API reports calibrated distance:

\[
z_{d,j} =
\hat{d}_j + v_{d,j}
\]

If the wall sensor API reports ADC counts, use the calibrated response curve:

\[
z_{adc,j} =
g_j(\hat{d}_j) + v_{adc,j}
\]

The distance-domain update is preferred if the calibration is stable enough, because covariance tuning is more intuitive in meters.

### 19.5 Front pair update

The front sensors are sampled together and should generally be updated as a 2D measurement:

\[
z͍_{front} =
\begin{bmatrix}
d_{FL} \\
d_{FR}
\end{bmatrix}
\]

\[
h͍_{front}(x͍) =
\begin{bmatrix}
\hat{d}_{FL} \\
\hat{d}_{FR}
\end{bmatrix}
\]

Use a 2x2 \(R_{front}\) so common-mode and differential uncertainty can be represented. The front pair constrains both distance to a front wall and angular misalignment.

### 19.6 Side updates

Side sensors are sampled separately:

\[
z_{left} = \hat{d}_{left} + v_{left}
\]

\[
z_{right} = \hat{d}_{right} + v_{right}
\]

They should be independent scalar updates unless hardware or calibration data shows correlation.

### 19.7 Wall update gating

Before applying a wall update, require:

1. wall existence hypothesis is valid
2. ray intersection is physically valid
3. distance is within calibrated sensor range
4. LED-settle interval has completed
5. innovation passes a gate:

\[
\nu^T S^{-1}\nu < \gamma
\]

where:

\[
\nu = z͍ - h͍(\hat{x͍})
\]

\[
S = P_{zz} + R
\]

For scalar updates:

\[
\frac{\nu^2}{S} < \gamma
\]

Recommended initial gates:

| Measurement | Gate |
|---|---:|
| IMU gyro | loose; saturation/fault gated separately |
| accelerometer | moderate; reject impact/saturation |
| side wall scalar | \(\gamma = 6.63\) for ~99% scalar gate |
| front wall pair | \(\gamma = 9.21\) for ~99% 2D gate |

---


## 20. Sequential measurement architecture

The estimator should support measurement blocks:

| Block | Size | Availability | Notes |
|---|---:|---|---|
| gyro | 1 | every IMU sample | direct yawRate update |
| accel | 2 | every IMU sample | constrains \(A_f,A_r\) residuals |
| front wall pair | 2 | LED-settled, valid wall geometry | update together |
| left wall | 1 | LED-settled, valid geometry | separate scalar |
| right wall | 1 | LED-settled, valid geometry | separate scalar |
| wheel pseudo | 1-2 | low-slip gated only | normally disabled |

This is mathematically equivalent to a batch update only under independence assumptions and consistent linearization. In this robot, the engineering benefits are more important:

- bad wall readings can be rejected without losing IMU updates
- open-floor modes skip wall updates naturally
- front pair can be treated as correlated
- side sensors can be read and updated independently
- UKF work can be interleaved with LED settle time

---

## 21. Tick scheduling and sensor interlace

The estimator should not own the LED timing directly, and `VehicleState` should not own the UKF. Use a tick estimation pipeline/scheduler that owns the execution choreography.

Conceptual 1 ms tick:

| Phase | Action |
|---|---|
| tick start | latch control currently active; latch encoder counts |
| early | consume latest canonical IMU sample |
| early | UKF predict using active controls and encoder-derived drivetrain inputs |
| wall phase A | enable front LEDs |
| settle interval | perform UKF work that does not require front ADC |
| after settle | read front pair ADC; disable front LEDs; apply front pair update |
| wall phase B | enable left side LED |
| settle interval | perform pending estimator work |
| after settle | read left ADC; disable left LED; apply left update |
| wall phase C | enable right side LED |
| settle interval | perform pending estimator work |
| after settle | read right ADC; disable right LED; apply right update |
| end | propagate estimate to control activation boundary |
| publish | update `VehicleState` |
| control | compute and stage next `CL`, `CR` |

Settling intervals should be treated as explicit time intervals. If the wall sensor analog side requires 50 us settling, the ADC timestamp used for the wall measurement should be the read time, not the LED-enable time.

---


## 22. Measurement noise proposal

### 26.1 IMU

Use datasheet noise as the floor, then inflate from logs.

For the LSM6DSV16X:

- gyro full-scale options include up to ±4000 dps
- accelerometer full-scale options include up to ±16 g
- high-performance gyro noise density is listed as 2.8 mdps/√Hz
- high-performance accelerometer noise density is listed as 60 µg/√Hz
- ODR options extend to 7.68 kHz

The effective measurement variance should account for configured bandwidth, mounting vibration, and filtering. The datasheet noise floor is not the final \(R\) for a robot with gear noise, ground impacts, and PWM/motor vibration.

### 26.2 Wall sensors

Wall \(R\) should be empirical and distance-dependent:

\[
R_{d,j}(d) =
\sigma_{d,j}^2(d)
\]

A usable starting form:

\[
\sigma_{d,j}(d) =
\sigma_{0,j} + k_{d,j} d^2
\]

or, if ADC-domain response is used:

\[
R_{adc,j}(d) =
\sigma_{adc,j}^2(d)
\]

Front pair covariance:

\[
R_{front} =
\begin{bmatrix}
\sigma_{FL}^2 & \rho_f\sigma_{FL}\sigma_{FR} \\
\rho_f\sigma_{FL}\sigma_{FR} & \sigma_{FR}^2
\end{bmatrix}
\]

Side scalar covariance:

\[
R_{side,j} = \sigma_{side,j}^2
\]

---

## 23. Margin and diagnostic outputs

The UKF should expose margin diagnostics to control/logging through `VehicleState`, not by exposing UKF internals.

### 27.1 Model residual magnitude

\[
M_A =
\sqrt{
(A_{f,nom}+\Delta A_f)^2+
(A_{r,nom}+\Delta A_r)^2
}
\]

Compare to:

\[
\mu g
\]

but do not clamp to it.

Use:

\[
m_{\mu} =
\frac{M_A}{\mu g}
\]

where \(\mu = 1.91\) for the measured tire/surface condition unless updated by calibration.

Interpretation:

| \(m_\mu\) | Meaning |
|---:|---|
| < 1 | conventional adhesive demand |
| ≈ 1 | near conventional limit |
| > 1 | slip/stick-slip likely or downforce/load transfer effects matter |

This is a diagnostic, not a command limiter.

### 27.2 Encoder/body consistency

\[
e_{Vf,wheel} =
V_f - \frac{r_w}{2}(\dot{\phi}_{w,L}+\dot{\phi}_{w,R})
\]

\[
e_{\omega,wheel} =
\omega -
\frac{r_w}{b}(\dot{\phi}_{w,R}-\dot{\phi}_{w,L})
\]

These are slip/mismatch indicators.

### 27.3 Turn recovery indicator

For a remaining straight recovery distance \(D_{recover}\), estimate lateral cleanup feasibility:

\[
t_{recover} \approx \frac{D_{recover}}{\max(V_f,V_{min})}
\]

\[
V_{r,pred} \approx V_r \exp(-K_{Vr}t_{recover})
\]

This is crude but useful. It helps distinguish "we are slipping but recoverable" from "we will enter the next turn with nontrivial lateral velocity."

---

## 24. Mode handling

### 28.1 Race / performance turns

- IMU updates active
- encoders as process inputs
- wall updates gated hard
- wheel pseudo-measurements disabled
- residual \(Q\) increased for lateral/yaw terms
- margin diagnostics logged

### 28.2 Open-floor identification

- IMU active
- encoders as process inputs
- wall updates disabled
- pose covariance allowed to grow
- focus on residuals and model fit

### 28.3 Exploration / mapping

- IMU active
- encoders as process inputs
- wall updates active when geometry is valid
- optional low-slip wheel pseudo-measurements
- lower residual \(Q\) than race mode

### 28.4 Wall touch / squaring

- IMU active
- wall events used as special-mode observations
- normal wall-distance model may be invalid during contact
- tire springback and impact transients should gate accelerometer updates

---

## 25. Initialization

Recommended startup initialization:

\[
P_x = P_{x,start}
\]

\[
P_y = P_{y,start}
\]

\[
\Theta = \Theta_{start}
\]

\[
V_f = 0
\]

\[
V_r = 0
\]

\[
\omega = 0
\]

\[
\Delta A_f = 0
\]

\[
\Delta A_r = 0
\]

\[
\Delta \alpha = 0
\]

Initial covariance should reflect confidence:

- small position/heading covariance if maze start pose is known
- larger heading if start alignment is not mechanically guaranteed
- larger residual covariance so the filter can adapt early
- nonzero velocity covariance even at rest due to launch disturbance

---

## 26. Architecture recommendation

Keep the estimator separate from `VehicleState`.

Recommended ownership:

| Object | Owns |
|---|---|
| Sensors | hardware timing, ADC/SPI reads, normalized samples |
| Estimation scheduler | interleaving and measurement timing |
| UKF estimator | sigma points, covariance, predict/update math |
| Plant model | \(f(x͍,u)\), nominal acceleration equations |
| VehicleState | canonical tick-boundary vehicle belief exposed to control |
| Controller | command selection |

`VehicleState` should not expose Eigen vectors, sigma points, covariance matrices, or Kalman gain. It should expose domain quantities such as `heading()`, `yawRate()`, `Vf()`, `Vr()`, margin summaries, and validity flags.

---

## 27. Parameter list

The plant model requires the following parameters.

### 31.1 Geometry and mass

| Parameter | Units | Meaning |
|---|---:|---|
| \(m\) | kg | vehicle mass |
| \(I_z\) | kg·m² | yaw inertia |
| \(r_w\) | m | effective wheel radius |
| \(b\) | m | measured track width |
| \(b_{eff}\) | m | effective dynamic track width |
| \(G\) | unitless | motor rotations per wheel rotation |
| \(N_{rev}\) | counts/rev | encoder counts per motor revolution |
| \(r_{imu,f}\) | m | IMU forward offset from CoG; current value \(-0.011\) m |
| \(r_{imu,r}\) | m | IMU rightward offset from CoG; current value \(-0.023\) m |

### 31.2 Drive model

| Parameter | Units | Meaning |
|---|---:|---|
| \(K_C\) | N/[command] | command-to-force gain |
| \(K_\omega\) | N/(rad/s) | wheel-speed load coefficient |
| \(F_{c,drive}\) | N | per-bank constant drive loss |
| \(C_{launch}\) | [command] | launch deadband |
| \(A_{c,f}\) | m/s² | forward constant loss |
| \(K_{Vf}\) | 1/s | forward viscous-like damping |
| \(S_\epsilon\) | m/s | wheel-speed smoothing |
| \(V_\epsilon\) | m/s | body-speed smoothing |

### 31.3 Yaw model

| Parameter | Units | Meaning |
|---|---:|---|
| \(\tau_{c,yaw}\) | N·m | constant yaw scrub resistance |
| \(K_\omega^{yaw}\) | N·m/(rad/s) | yaw viscous-like damping |
| \(\omega_\epsilon\) | rad/s | yaw softsign smoothing |

### 31.4 Lateral model

| Parameter | Units | Meaning |
|---|---:|---|
| \(K_{Vr}\) | 1/s | lateral velocity relaxation |

### 31.5 Residual dynamics

| Parameter | Units | Meaning |
|---|---:|---|
| \(\tau_{\Delta Af}\) | s | forward residual decay |
| \(\tau_{\Delta Ar}\) | s | lateral residual decay |
| \(\tau_{\Delta\alpha}\) | s | yaw residual decay |

### 31.6 Tire/margin

| Parameter | Units | Meaning |
|---|---:|---|
| \(\mu\) | unitless | measured tire/surface friction coefficient |
| \(g\) | m/s² | gravitational acceleration |

---

## 28. Calibration sequence needed before trusting the model

The plant model is implementable immediately, but its parameters are not inherently justified. They need identification.

Recommended minimum calibration:

1. **Encoder scale check**
   - verify counts/rev and gear ratio
   - verify sign of left/right wheel bank speed
2. **Straight launch tests**
   - identify \(C_{launch}\)
   - identify \(K_C\) and \(A_{c,f}\)
   - determine whether \(K_{Vf}\) can remain near zero
3. **Open-floor acceleration/braking**
   - fit command-to-acceleration model
   - compare \(A_{f,meas}\) to \(A_{f,nom}\)
4. **In-place yaw tests**
   - identify \(\tau_{c,yaw}\)
   - identify yaw acceleration response
   - verify no wheel pseudo-measurements are used
5. **Smooth arcing turns**
   - fit \(b_{eff}\) versus speed/radius if needed
   - tune \(\Delta A_r\), \(\Delta\alpha\) process noise
6. **Wall sensor static calibration**
   - distance response
   - noise versus distance
   - front pair common-mode/differential covariance
7. **Wall sensor dynamic validation**
   - verify LED settle timing
   - verify wall update gating
   - verify no bad wall update corrupts IMU-based dynamics

---

## 29. Primary risks

### 33.1 Residual states become hidden biases

If residual time constants are too long and process noise too low, residuals can become unobservable biases. This can produce stable-looking but wrong estimates.

Mitigation:

- use finite decay constants
- tune process noise from residual autocorrelation
- log residuals through all motion phases

### 33.2 Wall updates over-correct pose

Wall measurements are powerful but geometry-dependent. A false wall hypothesis or bad ADC sample can corrupt pose and heading.

Mitigation:

- strict geometry validation
- innovation gating
- front pair update as 2D block
- side updates scalar and independent
- mode gating

### 33.3 Encoder pseudo-measurements sneak back in

The most likely architectural regression is treating wheel speed as body speed during slip.

Mitigation:

- wheel pseudo-measurements disabled by default
- explicit mode gate required
- logs must distinguish encoder-derived diagnostics from UKF measurements

### 33.4 Estimator/core numerical behavior

A 9-state nonlinear estimator at 1 kHz with aggressive dynamics can become numerically overconfident if the plant and measurement models are mis-specified or if model uncertainty is understated.

Mitigation belongs primarily to the reusable estimator core and the tuning/configuration layer, not this plant-model specification. This document requires only that plant-model residuals, measurement innovations, rejected updates, and validity flags be observable in logs so numerical or modeling failures can be diagnosed.

---

## 30. Summary recommendation

Implement the first runtime UKF as:

\[
x͍ =
[P_x,\ P_y,\ \Theta,\ V_f,\ V_r,\ \omega,\ \Delta A_f,\ \Delta A_r,\ \Delta\alpha]^T
\]

Use:

- encoders as drivetrain inputs, not ground-truth motion measurements
- IMU gyro and accelerometer as primary dynamic updates
- wall sensors as gated, sequential, geometry-dependent updates
- front wall sensors as a 2D paired measurement
- side wall sensors as independent scalar updates
- residual acceleration states to represent model mismatch
- no traction clamp in the plant model
- margin indicators rather than command limiters

This plant/measurement-model design is the best tradeoff for the current platform because it preserves runtime feasibility, avoids unjustified tire-state estimation, supports the required sensor interleaving, and exposes the quantities the controller needs to push the controllability boundary rather than retreat from it. The generic UKF core remains outside this specification.

---

## 31. References

1. Simon J. Julier and Jeffrey K. Uhlmann, "Unscented Filtering and Nonlinear Estimation," *Proceedings of the IEEE*, 2004.
2. Eric A. Wan and Rudolph van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation," 2000.
3. Sebastian Thrun, Wolfram Burgard, and Dieter Fox, *Probabilistic Robotics*, MIT Press, 2005.
4. STMicroelectronics, *LSM6DSV16X Datasheet*, DS13510 Rev. 4, May 2023.
5. Project reference: *Noteworthy Physical Effects / Operating Envelope*.
6. Project reference: *Micromouse State Estimation Variable Reference*.
