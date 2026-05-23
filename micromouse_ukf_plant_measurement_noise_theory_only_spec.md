# Micromouse UKF Plant, Measurement, and Noise Specification

**Edition:** implementation-neutral mathematical specification derived from the contact-continuum reference.  
**Purpose:** define the estimator state semantics, plant physics, measurement models, stochastic/noise semantics, calibration posture, and validation criteria without prescribing software object boundaries, API names, filenames, runtime containers, or parallel implementation structures.

---

## 0. Scope and stance

This specification defines a planar body-state UKF target model for the micromouse estimator. Encoders are treated as drivetrain inputs. The IMU and wall sensors are treated as measurements. The plant is formulated through algebraic per-contact relative velocities in SI units.

The model is strict about:

| Area | Strict requirement |
|---|---|
| State | State dimension, ordering, physical meaning, and units |
| Coordinates | Global/body axes, heading convention, yaw sign, and IMU sign convention |
| Timing | Estimator-boundary timestamps and measurement effective-time semantics |
| Measurements | Meaning of gyro, accelerometer, encoder-derived quantities, and wall observations |
| Validity | Invalid samples must be skipped or routed through a defined degraded stochastic path |
| Covariance | Process-noise and measurement-noise meanings must be physically and statistically consistent |
| Contact physics | Per-contact relative velocity is the primary contact primitive |

The model is intentionally flexible about empirical physical surfaces that must be learned from logs:

| Area | Flexible model family |
|---|---|
| Drive | Motor/driver torque, current limiting, voltage and thermal corrections |
| Load | Normal load, fan load, and load transfer |
| Contact | Longitudinal/lateral contact force surfaces and force envelopes |
| Yaw | Continuous yaw resistance or yaw-loss terms |
| Ground strike | Forward/reverse acceleration envelope and impact uncertainty |
| Walls | Wall sensor response curves, ray aggregation, and covariance scheduling |
| Noise | Continuous process-noise and measurement-noise schedules |

The plant must not rely on speed-normalized slip ratio, slip angle, turn radius, curvature, instantaneous-center-of-rotation denominators, or maneuver-class thresholds. Human labels such as scrub, stick-slip, in-place, pivot, and rolling may describe test segments, but they must not define estimator states, plant branches, measurement modes, covariance modes, or command admissibility.

---

## 1. Source assumptions

The assumed robot and operating envelope are:

| Category | Assumption |
|---|---|
| Controller | Teensy 4.1-class controller, 1000 Hz control loop |
| Encoders | Two IE2-1024 encoders |
| IMU | LSM6DSV16X, pre-aligned to the estimator coordinate convention |
| Wall sensing | Custom IR wall sensors, including log-amplified front sensors |
| Motor drivers | DRV8871 H-bridge drivers |
| Drive motors | Two Faulhaber 1717T006SR 006 motors |
| Gear ratio | 17:56 |
| Wheels | 25 mm diameter wheels with solid-rubber tires |
| Layout | Four wheels, two left and two right, shared pinion per bank |
| Contact patch positions | Approximately \(\pm40\ \mathrm{mm}\) right and \(\pm14.75\ \mathrm{mm}\) forward |
| Mass | Approximately \(0.140\ \mathrm{kg}\) |
| Yaw inertia | Approximately \(220\times10^{-6}\ \mathrm{kg\ m^2}\) |

The operating envelope includes:

1. tire-scrubbing calibration turns;
2. high-speed arcing turns with expected stick-slip;
3. full-bore straight acceleration and braking, limited by available motor power and ground strike;
4. wall-proximate precision turns;
5. constant-velocity exploration and mapping.

---

## 2. Rejected modeling patterns

The following are rejected as estimator or plant semantics:

1. a yaw-gyro-bias state in the UKF when that state is effectively frozen by near-zero process noise;
2. wheel-bank-rate states;
3. motor-current states;
4. slip-ratio, slip-angle, left/right slip, or patch-slip states;
5. pitch, roll, or vertical states in the initial planar estimator;
6. contact or slip math that becomes singular or ill-conditioned at zero forward speed;
7. traction-feasibility gates that reject performance or calibration commands;
8. runtime maneuver taxonomy used to select turn/contact physics;
9. rolling or nonholonomic assumptions in operating regions where contact slip is expected.

Yaw gyro bias is treated as an external calibrated quantity. Wheel-bank rates are inputs. Motor current, driver behavior, and current limiting are represented algebraically. Pitch/roll/vertical effects are represented through acceleration envelopes, covariance inflation, event indicators, and replay diagnostics unless held-out evidence proves a planar model cannot preserve pose accuracy.

---

## 3. State and coordinate contract

### 3.1 State vector

The estimator state is:

\[
x=\begin{bmatrix}
P_x & P_y & \theta & V_f & V_r & \omega & \Delta a_f & \Delta a_r & \Delta\alpha
\end{bmatrix}^T
\]

| Symbol | Unit | Meaning |
|---|---:|---|
| \(P_x, P_y\) | m | Global maze position |
| \(\theta\) | rad | Global heading; \(\theta=0\) faces global \(+Y\); clockwise positive |
| \(V_f\) | m/s | Body-frame forward velocity |
| \(V_r\) | m/s | Body-frame rightward velocity |
| \(\omega\) | rad/s | Clockwise-positive body yaw rate |
| \(\Delta a_f\) | m/s² | Colored forward-acceleration residual |
| \(\Delta a_r\) | m/s² | Colored rightward/lateral-acceleration residual |
| \(\Delta\alpha\) | rad/s² | Colored yaw-acceleration residual |

### 3.2 Coordinate convention

| Frame/quantity | Convention |
|---|---|
| Global \(+Y\) | Forward |
| Global \(+X\) | Right |
| Body \(+f\) | Forward |
| Body \(+r\) | Right |
| Heading \(\theta=0\) | Facing global \(+Y\) |
| Positive yaw | Clockwise |

Body-to-global velocity:

\[
\dot P_x=V_f\sin\theta+V_r\cos\theta
\]

\[
\dot P_y=V_f\cos\theta-V_r\sin\theta
\]

Body dynamics:

\[
\dot\theta=\omega
\]

\[
\dot V_f=a_f+\omega V_r
\]

\[
\dot V_r=a_r-\omega V_f
\]

\[
\dot\omega=\alpha
\]

All IMU sign corrections occur before estimator entry. The estimator receives canonical body-frame quantities.

---

## 4. Timing, inputs, and validity semantics

Each prediction/update cycle is defined by an estimator-boundary timestamp. The prediction interval is:

\[
\Delta t_k=t_k-t_{k-1}
\]

The command stream must specify whether \(C_L,C_R\) were active during the interval ending at \(t_k\) or issued at \(t_k\) for the next interval.

No live bus-voltage measurement is assumed. Effective bus voltage and supply variation enter through calibration, torque uncertainty, and process-noise scheduling.

Encoder rates are drivetrain inputs. They feed motor-rate estimation, back-EMF/torque prediction, contact-relative velocity, force request, force limiting, and continuous noise scheduling. Invalid encoder samples are acquisition faults, not ordinary Gaussian missed-pulse noise.

IMU samples contain canonical yaw rate and canonical body-frame acceleration. Invalid IMU samples skip the gyro and accelerometer measurement rows and do not update the external yaw-bias estimate.

Wall samples contain a valid/invalid status, saturation status, noise estimate if available, and a wall hypothesis if one is available. Invalid wall samples skip the corresponding measurement row. Invalid raw values must not be converted into large innovations.

Each measurement type must use one replayable effective-time policy:

1. evaluate \(h(x)\) at the measurement effective time by propagation or interpolation; or
2. evaluate \(h(x)\) at the nearest propagated state and add timing covariance:

\[
R_{\mathrm{timing}}\approx \sigma_t^2\dot h\dot h^T
\]

---

## 5. Command limits and feedforward authority

Motion limits are external command-shaping and safety constraints. They may bound velocity, forward/lateral acceleration, yaw rate, yaw acceleration, command magnitude, current, temperature, or other hardware safety quantities. These limits are not plant-model semantics.

The plant may compute force-envelope ratios, contact-relative velocities, force-limiter activity, and drive-authority uncertainty for feedforward diagnostics, stochastic scheduling, measurement covariance scheduling, and replay analysis. These diagnostics must not be used as default hard gates for performance or calibration commands.

Accepted behavior:

1. commands remain governed by motion, hardware, current, thermal, and explicit safety limits;
2. plant diagnostics may indicate high force-envelope ratio or high uncertainty;
3. the estimator may weaken inappropriate measurements and increase process noise during high-uncertainty contact conditions.

Rejected behavior:

\[
\text{if nominal force envelope is exceeded, reject or clamp the command}
\]

The feedforward objective is high-performance contact behavior: tire-scrubbing calibration turns, high-contact-velocity arcs, full-bore acceleration/braking, and precision wall-proximate turns. Low-demand rolling-style motion remains useful for sign, scale, timing, geometry, and low-energy consistency checks, but it is not the primary optimization target.

---

## 6. Contact-continuum requirement

### 6.1 Red-flag variables

The plant and feedforward authority must not use the following as primary contact variables:

| Rejected variable pattern | Reason |
|---|---|
| \((\cdot)/V_f\) slip ratio | Singular or ill-conditioned near zero forward speed |
| \((\cdot)/\max(|V_f|,\epsilon)\) slip ratio | Disguised speed-normalized singularity |
| \(\arctan2(V_r,|V_f|)\) slip angle | Speed-normalized tire model not suited to zero-speed/contact-scrub operation |
| \(\omega/V_f\) or \(V_f/\omega\) | Curvature/radius singularities |
| Turn radius or curvature | Not primary contact physics |
| Instantaneous center of rotation | Not primary contact formulation |
| Stationary/opposed-command thresholds | Maneuver taxonomy rather than continuous physics |

Small epsilons are allowed for stable norms and smooth signs. They are not allowed as disguised denominators for speed-normalized tire models.

### 6.2 Required per-contact relative velocity

Each contact patch \(i\) has body-frame location:

\[
(r_i,f_i)
\]

where \(r_i>0\) means right and \(f_i>0\) means forward.

Body velocity at the patch:

\[
v^{body}_{f,i}=V_f-\omega r_i
\]

\[
v^{body}_{r,i}=V_r+\omega f_i
\]

Wheel surface forward velocity:

\[
v^{surf}_{f,i}=r_w\hat\omega_{w,s(i)}
\]

where \(s(i)\in\{L,R\}\) is the wheel bank associated with patch \(i\).

Primary contact-relative velocities:

\[
v^{rel}_{f,i}=v^{surf}_{f,i}-v^{body}_{f,i}=r_w\hat\omega_{w,s(i)}-V_f+\omega r_i
\]

\[
v^{rel}_{r,i}=-v^{body}_{r,i}=-V_r-\omega f_i
\]

These variables are affine in state and wheel-bank rate. They remain finite and continuous for zero forward speed, zero lateral speed, nonzero yaw rate, stopped wheel banks, moving wheel banks, and UKF sigma points that cross zero forward speed.

### 6.3 Continuous aggregate quantities

Load-weighted RMS contact-relative speed:

\[
\bar v_{rel}=\sqrt{\frac{\sum_i N_i\left((v^{rel}_{f,i})^2+(v^{rel}_{r,i})^2\right)}{\sum_i N_i+\epsilon}}
\]

Load-weighted RMS lateral relative speed:

\[
\bar v_{lat}=\sqrt{\frac{\sum_i N_i(v^{rel}_{r,i})^2}{\sum_i N_i+\epsilon}}
\]

Load-weighted yaw-induced contact speed:

\[
\bar v_{yaw}=|\omega|\sqrt{\frac{\sum_iN_i(r_i^2+f_i^2)}{\sum_iN_i+\epsilon}}
\]

These are continuous physical scalars, not maneuver labels.

---

## 7. Smooth numerical helpers

Smooth helpers are numerical defaults, not physical law. Replacements are acceptable if they preserve continuity, units, signs, and replay consistency.

Smooth sign:

\[
s_\epsilon(x;x_E)=\tanh(x/x_E)
\]

Stable softplus:

\[
\mathrm{softplus}_\epsilon(x;e)=e\log(1+\exp(x/e))
\]

Smooth deadzone:

\[
\mathrm{deadzone}_\epsilon(x,a,e)=\mathrm{softplus}_\epsilon(x-a,e)-\mathrm{softplus}_\epsilon(-x-a,e)
\]

Smooth step:

\[
S_\epsilon(x;x_0,x_1)=\frac{1}{2}\left[1+\tanh\left(\frac{x-(x_0+x_1)/2}{(x_1-x_0)/6}\right)\right]
\]

If \(x_0=x_1\), a defined fallback is required.

Smooth maximum and minimum:

\[
\max_\epsilon(a,b;e)=\frac{a+b+\sqrt{(a-b)^2+e^2}}{2}
\]

\[
\min_\epsilon(a,b;e)=\frac{a+b-\sqrt{(a-b)^2+e^2}}{2}
\]

Smooth asymmetric clipping:

\[
\mathrm{clip}_\epsilon(x;x_{min},x_{max},e)=\max_\epsilon\left(x_{min},\min_\epsilon(x,x_{max};e);e\right)
\]

---

## 8. Empirical model-family contract

Each empirical physical model family must define:

1. model status;
2. hard physical/statistical contract;
3. first-pass mathematical form;
4. expected failure signatures;
5. permitted follow-on refinements;
6. promotion or replacement criteria.

Hard contracts are physical meanings, units, signs, timing, validity behavior, covariance semantics, inputs, and outputs. First-pass empirical formulas are not permanent physical truth; they are initial mean models subject to validation.

---

## 9. Motor and driver torque model

### 9.1 Status

The motor/driver model is a flexible empirical family with a physics-first initial mean model.

### 9.2 Required physical quantities

For each drive side \(j\in\{L,R\}\), the model uses:

| Quantity | Meaning |
|---|---|
| \(C_j\) | Left/right command value |
| \(u_j\in[-1,1]\) | Normalized command |
| \(\hat\omega_{w,j}\) | Validated wheel-bank angular rate |
| \(G\) | Gear ratio |
| \(r_w\) | Wheel radius |
| \(R_m,R_{drv},R_{wire}\) | Motor, driver, and wiring resistance terms |
| \(K_t,K_e\) | Torque and back-EMF constants |
| \(M_R\) | Motor friction torque prior |
| \(V_{bus,eff}\) | Effective bus voltage prior |
| \(I_{trip}\) | Current limiting threshold, if relevant |
| \(\eta_{drive,j}\) | Drive efficiency for side \(j\) |

For the Faulhaber 1717T006SR 006 prior:

\[
K_t=3.96\ \mathrm{mNm/A}=0.00396\ \mathrm{N\ m/A}
\]

\[
K_e=0.00396\ \mathrm{V\ s/rad}
\]

### 9.3 First-pass mean model

Normalize command:

\[
u_j=\mathrm{normalize}(C_j),\qquad u_j\in[-1,1]
\]

Motor shaft rate:

\[
\Omega_{m,j}=G\hat\omega_{w,j}
\]

Voltage-equivalent drive:

\[
V_{drive,j}=D(u_j,\rho_{PWM})V_{bus,eff}
\]

Raw current:

\[
I^{raw}_j=\frac{V_{drive,j}-K_e\Omega_{m,j}}{R_m+R_{drv}+R_{wire}}
\]

If current limiting is nonbinding:

\[
I_j=I^{raw}_j
\]

If current limiting can bind:

\[
I_j=I_{trip}\tanh\left(\frac{I^{raw}_j}{I_{trip}}\right)
\]

Motor torque:

\[
T_{m,j}=K_t I_j-M_R s_\epsilon(\Omega_{m,j};\Omega_E)
\]

Raw wheel-bank torque:

\[
T^{raw}_{b,j}=\eta_{drive,j}G T_{m,j}
\]

Low-speed launch factor:

\[
\ell^{slow}_j=\exp\left[-\left(\frac{r_w|\hat\omega_{w,j}|}{v_{static}}\right)^2\right]
\]

Usable wheel-bank torque:

\[
T_{b,j}=\mathrm{deadzone}_\epsilon(T^{raw}_{b,j},\ell^{slow}_jT_{launch,j},T_E)
-T_{roll,j}s_\epsilon(\hat\omega_{w,j};\omega_E)
-B_{visc,j}\hat\omega_{w,j}
\]

Set \(B_{visc,j}=0\) initially unless replay evidence proves a viscous term is required.

Drive force:

\[
F_{drive,j}=\frac{T_{b,j}}{r_w}
\]

Current-saturation diagnostic:

\[
\chi_I=\max_j\left|\frac{I^{raw}_j}{I_{trip}+\epsilon}\right|
\]

Torque-authority uncertainty:

\[
\chi_T=\max_j\left(\frac{\sigma_{T,j}}{|T_{b,j}|+T_E}\right)
\]

Both \(\chi_I\) and \(\chi_T\) are covariance-scheduling diagnostics, not command gates.

### 9.4 Correction path

The production torque model may be represented as:

\[
T_{b,j}=T^{direct}_{b,j}+\delta T_j
\]

with \(\delta T_j=0\) initially.

Promote corrections in this order:

1. left/right scale and offset;
2. effective bus voltage, wiring resistance, or driver resistance calibration;
3. launch and rolling/friction terms;
4. compact command/rate correction surface;
5. runtime, thermal, or recent-current proxy;
6. periodic ripple correction only when its measured or replay-reconstructable source is explicitly defined with units and wrapping semantics.

Dense lookup tables are an escape path, not the initial mean model.

### 9.5 Failure signatures

| Signature | Likely interpretation |
|---|---|
| Straight launch requires persistent positive \(\Delta a_f\) | Torque, launch threshold, or voltage prior too weak |
| Braking requires persistent negative \(\Delta a_f\) | Decay/brake behavior or reverse torque wrong |
| Differential commands produce biased yaw residuals | Left/right torque asymmetry or contact/yaw model error |
| Similar commands drift over a run | Thermal or supply effect missing |
| Residuals show repeatable periodic structure versus a measured drivetrain coordinate | Define and validate a measured periodic source before adding ripple correction |

Promotion requires held-out replay improvement on launch, braking, high-contact-slip turns, and arcs without degrading low-demand straight consistency.

---

## 10. Normal-load model

### 10.1 Status

The normal-load model is a flexible algebraic load-estimation family.

### 10.2 First-pass model

For contact patch \(i\):

\[
N_i=N^{static}_i+N^{fan}_i+N^{long}_i+N^{lat}_i
\]

Fan/downforce term:

\[
N^{fan}_i=w_{fan,i}F_{fan}(C_{fan})
\]

Use previous-step or low-pass-lagged nominal acceleration, not raw accelerometer samples and not residual-corrected acceleration:

\[
a^{lag}_f,\qquad a^{lag}_r
\]

Longitudinal load transfer:

\[
N^{long}_i=-k_{long}a^{lag}_f\frac{f_i}{\sum_k f_k^2+\epsilon}
\]

Lateral load transfer:

\[
N^{lat}_i=-k_{lat}a^{lag}_r\frac{r_i}{\sum_k r_k^2+\epsilon}
\]

with:

\[
k_{long}\ge0,\qquad k_{lat}\ge0
\]

Sign convention:

| Acceleration | Load-transfer sign |
|---|---|
| Positive forward acceleration | unloads forward patches and loads rear patches |
| Positive rightward lateral acceleration | unloads right patches and loads left patches |

Nonnegative load clamp:

\[
N_i\leftarrow \max_\epsilon(N_i,N_{min};N_E)
\]

If load-transfer characterization is unavailable, \(k_{long}=0\) and \(k_{lat}=0\) are acceptable initial approximations, but must be treated as explicit approximations.

### 10.3 Failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Implausible force-limit parameters in high lateral acceleration arcs | Fit lateral load-transfer gain or use a one-step fixed point |
| Full-bore straights clip too early or too late | Fit longitudinal transfer and ground envelope jointly |
| Fan command changes behavior but load model cannot explain it | Fit fan load curve and distribution |
| Minimum-load clamp active in ordinary motion | Check gain, sign, static distribution, and clamp floor |

---

## 11. Contact-force model

### 11.1 Status

The contact-force model is a flexible empirical family. The first pass is velocity-space and force-space, not speed-normalized slip-space.

### 11.2 First-pass model

Distribute bank drive force by normal load:

\[
w_{i|j}=\frac{N_i}{\sum_{k\in j}N_k+\epsilon}
\]

\[
F^{driveReq}_i=w_{i|s(i)}F_{drive,s(i)}
\]

Raw force request:

\[
F^{req}_{f,i}=F^{driveReq}_i+K_{f,i}v^{rel}_{f,i}
\]

\[
F^{req}_{r,i}=K_{r,i}v^{rel}_{r,i}
\]

Relative speed magnitude:

\[
v^{rel}_{mag,i}=\sqrt{(v^{rel}_{f,i})^2+(v^{rel}_{r,i})^2+v_E^2}
\]

Velocity-dependent friction envelope:

\[
\mu_i=\mu_{slide,i}+(\mu_{peak,i}-\mu_{slide,i})\exp\left[-\left(\frac{v^{rel}_{mag,i}}{v_{Stribeck,i}}\right)^2\right]
\]

\[
F^{lim}_i=\mu_iN_i
\]

Force-envelope ratio:

\[
\rho_{F,i}=\sqrt{
\left(\frac{F^{req}_{f,i}}{\lambda_fF^{lim}_i+\epsilon}\right)^2+
\left(\frac{F^{req}_{r,i}}{\lambda_rF^{lim}_i+\epsilon}\right)^2+
\epsilon^2
}
\]

Smooth force-space limiter:

\[
s_{F,i}=\frac{1}{\max_\epsilon(1,\rho_{F,i};e_F)}
\]

\[
F_{f,i}=s_{F,i}F^{req}_{f,i}
\]

\[
F_{r,i}=s_{F,i}F^{req}_{r,i}
\]

Force-limiter activity:

\[
\lambda^{act}_{F,i}=1-s_{F,i}
\]

Aggregate diagnostics:

\[
\rho_F^{max}=\max_i \rho_{F,i}
\]

\[
\lambda_F^{max}=\max_i \lambda^{act}_{F,i}
\]

The force-envelope ratio is a diagnostic and noise-scheduling input, not a command admissibility gate. Force outputs must remain continuous through zero forward speed, finite yaw rate, and force-limiting transitions.

### 11.3 Failure signatures and follow-ons

| Signature | Likely interpretation |
|---|---|
| High lateral/yaw residuals in high-contact-slip arcs | Mean force surface too simple |
| Calibration turns require large persistent yaw residuals | Contact force law or yaw-loss model inadequate |
| Force-envelope ratio extreme in ordinary motion | Normal load, friction, or limiter scale wrong |
| Low-demand straights show biased residuals | Torque, straight friction, or timing wrong before contact model |
| Prediction unstable near \(V_f=0\) | Velocity-space contract violated |

Allowed follow-ons include anisotropic force envelopes, compact empirical force surfaces over \((v^{rel}_f,v^{rel}_r,N,F^{driveReq})\), patch/side-specific parameters, correlated lateral/yaw residual driving noise, and moving yaw-loss behavior into patch-level forces if a separate yaw-loss term becomes confounded.

---

## 12. Yaw moment and yaw-loss model

### 12.1 Status

The yaw model is a flexible empirical correction family. It is not a maneuver branch.

### 12.2 Nominal yaw order

Contact moment:

\[
M_{contact}=\sum_i(f_iF_{r,i}-r_iF_{f,i})
\]

Yaw-loss moment:

\[
M_{nom}=M_{contact}-M_{loss}
\]

Yaw acceleration:

\[
\alpha^{raw}=\frac{M_{nom}}{I_z}
\]

Yaw acceleration must not be computed from a pre-correction yaw moment when a yaw-loss term is active.

### 12.3 First-pass yaw-loss model

Use a smooth continuous yaw-loss moment:

\[
M_{loss}=\left(M_{yawC}s_{yaw}+B_{yaw}|\omega|+K_{yawRel}\bar v_{rel}+K_{yawLim}\lambda_F^{max}\right)s_\epsilon(\omega;\omega_E)
\]

with:

\[
s_{yaw}=S_\epsilon(\bar v_{yaw};v_{yawLow},v_{yawHigh})
\]

This represents approximately constant yaw resistance continuously, without detecting a turn category. All parameters are empirical and versioned.

### 12.4 Failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Calibration turns fit but high-speed arcs degrade | Move yaw loss into contact model or reduce global component |
| High-speed arcs fit but calibration turns fail | Increase or reshape constant yaw-loss component |
| Yaw residual sign depends on side command | Repair torque asymmetry before yaw-loss retuning |
| Same term improves one test family and hurts another | Split empirical decomposition or move to patch-level forces |

---

## 13. Ground-strike and acceleration envelope

### 13.1 Status

The ground-strike model is a flexible acceleration/impact approximation family.

### 13.2 First-pass model

Use pure acceleration-envelope clipping first:

\[
a^{clip}_f=\mathrm{clip}_\epsilon(a^{raw}_f;-a^{rev}_{max},a^{fwd}_{max},e_a)
\]

\[
a^{nom}_f=a^{clip}_f
\]

\[
a^{nom}_r=a^{raw}_r
\]

\[
\alpha^{nom}=\alpha^{raw}
\]

Ground-use scalar:

\[
g_u=\frac{|a^{raw}_f-a^{clip}_f|}{|a^{raw}_f|+\epsilon}
\]

The first pass does not scale contact forces. Yaw and lateral consequences of ground strike are treated as residual/noise unless held-out logs show repeatable deterministic coupling.

### 13.3 Failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Repeatable yaw bias during clipped straight acceleration | Force-consistent strike/load redistribution |
| Acceleration innovation spikes but pose/yaw acceptable | Event/noise treatment sufficient |
| Clip limit depends on speed, command, or run state | Scheduled acceleration envelope |
| Wall-proximate impact-like events | Wall/tire springback covariance inflation |

---

## 14. Nominal body dynamics

After drive torque, normal load, contact force, yaw loss, and ground envelope:

\[
a^{raw}_f=\frac{\sum_iF_{f,i}}{m_{eff,f}}
\]

\[
a^{raw}_r=\frac{\sum_iF_{r,i}}{m_{eff,r}}
\]

\[
M_{contact}=\sum_i(f_iF_{r,i}-r_iF_{f,i})
\]

\[
M_{nom}=M_{contact}-M_{loss}
\]

\[
\alpha^{raw}=\frac{M_{nom}}{I_z}
\]

The ground envelope produces:

\[
a^{nom}_f,\qquad a^{nom}_r,\qquad \alpha^{nom}
\]

Propagation accelerations include residuals:

\[
a_f=a^{nom}_f+\Delta a_f
\]

\[
a_r=a^{nom}_r+\Delta a_r
\]

\[
\alpha=\alpha^{nom}+\Delta\alpha
\]

Continuous-time state derivative:

\[
\dot x=\begin{bmatrix}
V_f\sin\theta+V_r\cos\theta \\
V_f\cos\theta-V_r\sin\theta \\
\omega \\
a_f+\omega V_r \\
a_r-\omega V_f \\
\alpha \\
-\Delta a_f/\tau_f \\
-\Delta a_r/\tau_r \\
-\Delta\alpha/\tau_\alpha
\end{bmatrix}
\]

RK2 is acceptable for bring-up; RK4 is preferred for aggressive replay comparison. Euler integration is limited to isolated low-demand tests.

---

## 15. Residual acceleration process model

### 15.1 OU residual semantics

For \(q\in\{a_f,a_r,\alpha\}\):

\[
\phi_q=\exp\left(-\frac{\Delta t}{\tau_q}\right)
\]

\[
\Delta q_{k+1}=\phi_q\Delta q_k+\eta_q
\]

\[
\eta_q\sim\mathcal N(0,Q_{\Delta q})
\]

\[
Q_{\Delta q}=\sigma^2_{\Delta q,ss}(1-\phi_q^2)
\]

The scheduled \(\sigma_{\Delta q,ss}\) values are steady-state residual-state standard deviations. They are not per-tick increments and not continuous-time spectral densities.

### 15.2 First-pass noise schedule

Continuous schedule inputs:

| Symbol | Meaning |
|---|---|
| \(\bar v_{rel}\) | Load-weighted RMS contact-relative speed |
| \(\bar v_{lat}\) | Load-weighted RMS lateral contact-relative speed |
| \(\bar v_{yaw}\) | Load-weighted yaw-induced contact speed |
| \(\rho_F^{max}\) | Maximum force-envelope ratio |
| \(\lambda_F^{max}\) | Maximum force-limiter activity |
| \(g_u\) | Ground-use scalar |
| \(\chi_I\) | Current saturation index |
| \(\chi_T\) | Drive-authority uncertainty |
| \(\iota\) | Continuous impact/event indicator |
| \(e_{enc}\) | Encoder fault/degraded-input indicator |

First-pass steady-state standard deviations:

\[
\sigma_{\Delta a_f,ss}=\sigma_{f,0}+k_{f,rel}\bar v_{rel}+k_{f,lim}\lambda_F^{max}+k_{f,g}g_u+k_{f,I}\chi_I^2+k_{f,T}\chi_T+k_{f,imp}\iota+k_{f,enc}e_{enc}
\]

\[
\sigma_{\Delta a_r,ss}=\sigma_{r,0}+k_{r,lat}\bar v_{lat}+k_{r,yaw}\bar v_{yaw}+k_{r,lim}\lambda_F^{max}+k_{r,g}g_u+k_{r,I}\chi_I^2+k_{r,T}\chi_T+k_{r,imp}\iota+k_{r,enc}e_{enc}
\]

\[
\sigma_{\Delta\alpha,ss}=\sigma_{\alpha,0}+k_{\alpha,rel}\bar v_{rel}+k_{\alpha,yaw}\bar v_{yaw}+k_{\alpha,lim}\lambda_F^{max}+k_{\alpha,I}\chi_I^2+k_{\alpha,T}\chi_T+k_{\alpha,imp}\iota+k_{\alpha,enc}e_{enc}
\]

### 15.3 Continuous event indicator

Planar jerk:

\[
j_{planar}=\sqrt{j_f^2+j_r^2}
\]

Jerk index:

\[
j_{idx}=S_\epsilon(j_{planar};j_{low},j_{high})
\]

IMU saturation index:

\[
s_{imu}=\max_c S_\epsilon(|z_c|;z_{margin,low},z_{margin,high})
\]

Impact/event indicator:

\[
\iota=\max_\epsilon\left(g_u,\max_\epsilon(j_{idx},s_{imu};e);e\right)
\]

If vertical acceleration is available:

\[
\iota\leftarrow \max_\epsilon\left(\iota,S_\epsilon(|a_z-g|;z_{low},z_{high});e\right)
\]

Additional wall-touch or tire-springback evidence may be added only as versioned continuous inputs.

### 15.4 Residual injection and full covariance

The OU innovation injection point must be documented and replay-identical. Midpoint injection is preferred for RK2/RK4; endpoint injection after deterministic OU decay is the fallback.

Residual process noise must affect the full state covariance, including velocity, yaw rate, pose, heading, and residual-to-motion cross-covariances. Encoder input covariance must propagate through the nonlinear plant before force limiting and noise scheduling.

Sensitivity-mapped additive covariance form:

\[
Q_x\leftarrow Q_x+G_\eta Q_\eta G_\eta^T+J_\omega R_\omega J_\omega^T
\]

Injecting noise only into the final three residual-state variances is not sufficient.

---

## 16. Measurement update structure

Measurement updates may be sequential or grouped into small correlated blocks. Correlated measurements must be grouped or have covariance inflation that accounts for the correlation.

Recommended mathematical block dimensions:

| Measurement | Dimension |
|---|---:|
| Gyro yaw rate | 1 |
| Planar accelerometer | 2 |
| Wall sensor | 1, or 2 if explicitly correlated |
| Optional encoder/body pseudo-measurement | 2 |

---

## 17. IMU measurement model

### 17.1 Gyro

External yaw-bias estimate:

\[
b^{ext}_{g,z}
\]

Gyro measurement entering the UKF:

\[
z_g=\omega^{canon}_{imu,z}-b^{ext}_{g,z}
\]

Measurement function:

\[
h_g(x)=\omega
\]

Gyro covariance includes white noise, scale uncertainty, external-bias uncertainty, and timing/filter phase uncertainty. Coherent bias uncertainty must not be treated solely as independent white sample noise if it is material.

### 17.2 Accelerometer

For each sigma point, use propagated:

\[
a_f,\qquad a_r,\qquad \alpha
\]

For an IMU located at body-frame position \((r_{imu},f_{imu})\):

\[
a^{imu}_f=a_f-\alpha r_{imu}-\omega^2 f_{imu}
\]

\[
a^{imu}_r=a_r+\alpha f_{imu}-\omega^2 r_{imu}
\]

Choose exactly one acceleration-bias convention.

Bias-included measurement convention:

\[
h_{a_f}=a^{imu}_f+b^{cal}_{a,f}
\]

\[
h_{a_r}=a^{imu}_r+b^{cal}_{a,r}
\]

If acceleration is already bias-removed before estimator entry, remove \(b^{cal}_{a,f}\) and \(b^{cal}_{a,r}\) from the measurement function. Both conventions must not be active simultaneously.

Use low-pass/UI-path accelerometer output for the planar measurement update. High-pass, slope, vertical, and jerk indicators are reserved for covariance scheduling and event detection.

Accelerometer covariance:

\[
R_a=R_{a,base}+R_{timing}+R_{filter}+R_{impact}+R_{ground}+R_{forceLimit}+R_{saturation}
\]

Inflation terms are functions of continuous physical/event scalars, not maneuver labels.

Invalid IMU samples skip the gyro and accelerometer rows, do not update the external yaw-bias estimate, and may trigger an outage/degraded process policy if persistent.

---

## 18. Encoder treatment and optional body-motion pseudo-measurement

Encoders are drivetrain inputs. They are not ordinary pose, velocity, or yaw measurements.

Validated encoder rates feed:

1. motor rate;
2. back-EMF and torque prior;
3. optional periodic torque correction;
4. contact-relative velocity;
5. force request and force limiting;
6. continuous noise scheduling.

An encoder/body pseudo-measurement is not part of the default high-performance estimator path. It may be enabled only for conservative, exploration, or debug configurations after replay proves benefit. It must not classify motion as rolling or nonrolling. Its covariance may depend only on continuous contact-relative velocities, force-envelope ratio, ground use, launch/low-speed torque state, and encoder validity.

If enabled, use calibrated effective rolling geometry:

\[
z_{enc}=\begin{bmatrix}0\\0\end{bmatrix}
\]

\[
h_L=r_w\hat\omega_{w,L}-\left(V_f+\frac{b_{eff}}{2}\omega\right)
\]

\[
h_R=r_w\hat\omega_{w,R}-\left(V_f-\frac{b_{eff}}{2}\omega\right)
\]

Here \(b_{eff}\) is a calibrated effective rolling track width, not necessarily the physical track width.

Covariance must include encoder covariance:

\[
R_{enc,total}=R_{enc,model}+r_w^2R_\omega
\]

Inflate or disable this pseudo-measurement when contact-relative velocity, force-limiter activity, ground use, launch torque, or encoder invalidity makes the constraint untrustworthy.

---

## 19. Wall-sensor measurement model

### 19.1 Status

The wall-sensor model is a flexible response-space measurement family with a hard geometry/extrinsic contract.

### 19.2 Geometry

Each wall sensor \(s\) has body-frame origin and unit look vector:

\[
(r_s,f_s),\qquad \hat u_s=(u_{f,s},u_{r,s})
\]

Global origin:

\[
P_{x,s}=P_x+f_s\sin\theta+r_s\cos\theta
\]

\[
P_{y,s}=P_y+f_s\cos\theta-r_s\sin\theta
\]

Global look vector:

\[
u_{x,s}=u_{f,s}\sin\theta+u_{r,s}\cos\theta
\]

\[
u_{y,s}=u_{f,s}\cos\theta-u_{r,s}\sin\theta
\]

### 19.3 First-pass response prediction

Use response-space prediction. Distance-space prediction is acceptable only if the upstream wall pipeline produces calibrated distance with known covariance.

Ray bundle requirements:

1. nonnegative normalized ray weights;
2. calibrated angular support;
3. stable log-sum-exp soft minimum;
4. explicit no-hit handling.

Effective distance:

\[
d_{eff,s}=-\frac{1}{\beta_s}\log\left(\sum_q w_{s,q}\exp(-\beta_sd_{s,q})\right)
\]

No-hit rays are excluded or assigned a documented max-range surrogate. If all rays are no-hit, skip the update unless a skipped-equivalent row is required by the active estimator math.

Raw/log response examples:

\[
h_{front,s}=a_s-b_s\log(d_{eff,s}+d_{0,s})+c_s\psi_s
\]

\[
h_{side,s}=a_s+\frac{b_s}{(d_{eff,s}+d_{0,s})^{p_s}}+c_s\psi_s
\]

where \(\psi_s\) is the incidence-dependent term.

Invalid wall samples skip the measurement row. Saturated samples either skip the row or inflate covariance through the saturation term. Locally flat response regions caused by saturation or far-range insensitivity must be skipped or assigned inflated covariance.

Follow-ons include empirical response maps over distance/incidence, explicit no-hit likelihood, shared-geometry covariance for sensors observing the same wall/edge/post, and edge-aware covariance based on proximity to segment endpoints or posts.

---

## 20. External stationary yaw-bias estimator

The external yaw-gyro bias \(b^{ext}_{g,z}\) is outside the UKF state. It may be updated only during confirmed stationary intervals for startup or maintenance.

Stationary detection is reserved for:

1. yaw-gyro bias calibration;
2. static sensor sanity checks;
3. startup calibration.

Stationary thresholds are calibration parameters. They are not part of the plant model and must not select contact physics, yaw-loss physics, force scheduling, command admissibility, or turn behavior.

---

## 21. Validation, replay, and logging boundary

The deployed estimator-boundary log must be sufficient to replay the input stream:

1. timestamps and phases;
2. command values and command-validity convention;
3. encoder samples and validity;
4. IMU samples, validity, and status;
5. wall samples, validity, and wall hypotheses;
6. calibration, configuration, and build identity.

The log does not need to carry per-tick innovation, NIS, residual, covariance, or diagnostic telemetry at 1 kHz. Those quantities may be reconstructed during replay from the logged input stream and active calibration.

Validation must replay the same estimator behavior as the deployed estimator, not a shadow estimator. If a diagnostic cannot be reconstructed from logged inputs and calibration identity, that is a replay-contract gap before it is a reason to expand embedded telemetry.

Replay products may include:

1. innovations and normalized innovation squared;
2. measurement accept/skip/gating decisions;
3. residual-state traces;
4. covariance summaries;
5. contact-relative velocities;
6. force request ratios and force-limiter activity;
7. normal-load diagnostics;
8. torque/drive-authority diagnostics;
9. wall ray-bundle diagnostics;
10. ablation reports.

---

## 22. Calibration and tuning procedure

### 22.1 Bring-up order

1. Coordinate and sign audit.
2. Timestamp and command-validity audit.
3. Encoder count convention, polarity, and covariance.
4. IMU scale, bias, alignment, and phase.
5. Wall sensor extrinsics and response behavior.
6. Direct motor/driver parameter priors.
7. Straight launch, braking, and friction parameters.
8. Contact-force and yaw-loss parameters using high-contact-slip calibration turns and arcs.
9. Ground-strike acceleration envelope.
10. Process and measurement covariance schedules.
11. Held-out replay and ablation.

### 22.2 Parameter traceability

Every fitted active parameter must record:

| Field | Requirement |
|---|---|
| Symbol/name | The identifier used by the active implementation or calibration file |
| Unit | SI or dimensionless |
| Prior | Value and source |
| Bound type | Hard physical/safety bound or soft prior |
| Fit data | Logs allowed to tune it |
| Validation data | Held-out logs used to validate it |
| Residual policy | Whether residuals are enabled, weakened, disabled, or penalized while fitting |
| Confounds | Other parameters that can mimic it |
| Version metadata | Robot revision, tire/floor condition, firmware, estimator build |

Physical tuning and performance claims are blocked until active fitted parameters have this metadata.

### 22.3 Identifiability posture

Do not tune all empirical submodels simultaneously from a mixed run. Fit aggregate effects first:

1. effective timing and signs;
2. effective torque;
3. effective launch and straight friction;
4. effective contact force;
5. effective yaw loss;
6. effective ground envelope.

Only decompose aggregate effects into physical sub-parameters when data supports the decomposition.

---

## 23. Validation tests and acceptance checks

### 23.1 Zero-forward-speed stability

Required sweeps:

1. \(V_f\) through negative, zero, and positive small values;
2. \(V_r\) through negative, zero, and positive small values;
3. \(\omega\) through realistic calibration-turn values;
4. wheel-bank rates through stopped, same-direction, and opposed cases.

Required assertions:

1. all plant outputs are finite;
2. contact-force outputs are continuous;
3. finite-difference sensitivities are bounded;
4. no command is rejected solely because of force-envelope ratio.

### 23.2 Sigma-point zero crossing

Set the UKF mean near \(V_f=0\) and covariance such that sigma points cross \(V_f=0\). Prediction must produce finite state mean and covariance, and no plant branch may change semantic meaning across the zero crossing.

### 23.3 Command admissibility

Commands inside motion and safety limits must not be rejected by force-envelope diagnostics. Motion limits may still clip velocity, acceleration, yaw rate, yaw acceleration, hardware command magnitude, current, thermal, and explicit safety quantities.

### 23.4 Feedforward performance

Primary feedforward validation uses high-performance contact conditions:

1. calibration angle turns;
2. high-lateral-acceleration arcs;
3. full-bore straight acceleration;
4. full-bore braking;
5. precision wall-proximate turns.

Low-demand rolling-style logs remain sanity and characterization data, not the primary pass/fail target.

### 23.5 Replay reports

Replay should report by commanded segment:

1. pose/yaw error where reference is available;
2. gyro and accelerometer innovation statistics;
3. wall innovation statistics;
4. residual mean, RMS, and autocorrelation;
5. force-envelope and contact-relative-velocity diagnostics;
6. normal-load diagnostics;
7. torque/drive-authority diagnostics;
8. covariance consistency checks;
9. ablation results.

Numeric thresholds are versioned from characterization data unless they are hardware safety limits or physical impossibility checks.

### 23.6 Required ablations

Replay with:

1. residual driving noise reduced;
2. accelerometer updates weakened;
3. wall updates disabled;
4. optional encoder/body pseudo-measurement disabled;
5. torque correction disabled;
6. ground-envelope covariance disabled.

A model is not accepted if it only works because residuals or covariance inflation hide systematic nominal-plant errors in low-demand conditions.

---

## 24. Diagnostic glossary

| Mathematical diagnostic | Meaning |
|---|---|
| \(v^{rel}_{f,i},v^{rel}_{r,i}\) | Per-contact relative velocity in forward/right directions; primary contact variables |
| \(\bar v_{rel}\) | Load-weighted RMS contact-relative speed |
| \(\bar v_{lat}\) | Load-weighted RMS lateral contact-relative speed |
| \(\bar v_{yaw}\) | Load-weighted contact speed induced by yaw rate and patch geometry |
| \(\rho_{F,i}\) | Requested force relative to current empirical force envelope |
| \(\lambda^{act}_{F,i}\) | Degree of active force limiting; diagnostic/noise input only |
| \(g_u\) | Degree of forward acceleration-envelope clipping |
| \(\chi_T\) | Torque authority uncertainty from unmeasured supply, thermal, and driver effects |
| \(\iota\) | Continuous covariance/event scalar from vertical acceleration, jerk, saturation, ground use, or wall-touch evidence |

Forbidden normative estimator terms include in-place-turn mode, rolling mode, stick-slip mode, pivot-scrub mode, traction-feasible command, and stationary-selected turn model.

---

## 25. Summary

The target estimator is a 9-state planar body UKF with encoder-derived drivetrain inputs and contact-relative-velocity plant physics. It rejects yaw gyro bias as a UKF state, traction-gated control authority, speed-normalized slip math, zero-speed-singular contact models, and runtime turn taxonomy.

The model is strict where ambiguity would break estimator meaning: state, coordinates, timing, measurement semantics, input validity, covariance semantics, and contact-relative velocity. It remains flexible where the physical robot must be learned from logs: motor/driver torque corrections, normal load, contact force, yaw loss, ground strike, wall response, and noise schedules.

The test of the design is replayed estimator performance across the robot’s actual high-performance operating envelope.
