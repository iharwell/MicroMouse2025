# Micromouse UKF Plant and Measurement Model Specification

**Purpose:** define a complete, standalone specification for the vehicle UKF plant model, measurement functions, process-noise handling, encoder-input treatment, covariance scheduling, and state-inclusion decisions. This document assumes the UKF implementation itself is numerically sound; it specifies the model and the estimator contracts that the implementation must satisfy.

---

## 1. Design position

Use one planar body-state UKF with encoder-derived wheel rates as measured drivetrain inputs, not as UKF states, and with slip represented algebraically through contact-patch slip velocities. Use three colored residual acceleration states to cover the dominant unmodeled body-force effects.

Recommended state vector:

\[
state\_vec =
\begin{bmatrix}
Px & Py & heading & Vf & Vr & yawRate & \Delta Af & \Delta Ar & \Delta yawAccel
\end{bmatrix}^T
\]

| State | Unit | Meaning |
|---:|---|---|
| `Px`, `Py` | m | global maze position |
| `heading` | rad | global heading, 0 facing +Y, clockwise positive |
| `Vf` | m/s | body-frame forward velocity |
| `Vr` | m/s | body-frame rightward velocity |
| `yawRate` | rad/s | body yaw rate, clockwise positive |
| `DeltaAf` | m/s² | colored forward-acceleration residual |
| `DeltaAr` | m/s² | colored lateral-acceleration residual |
| `DeltaYawAccel` | rad/s² | colored yaw-acceleration residual |

The model is explicitly **not** a no-slip differential-drive model. It uses validated measured wheel-bank rates to compute motor algebra, contact-patch slip velocities, contact forces, utilization, and regime-dependent covariance scheduling. Encoder acquisition faults are handled as faults, not as nominal Gaussian missed-pulse noise. The residual states then correct remaining body-force mismatch, and residual-driving noise must propagate into the full state covariance rather than only into the residual-state block.

---

## 2. Source assumptions

The design uses the following project and component assumptions.

* The vehicle uses a Teensy 4.1 with a 1000 Hz control loop, custom low-latency per-tick logging, two IE2-1024 encoders, an LSM6DSV16X IMU, custom IR wall sensors, DRV8871 H-bridges, two Faulhaber 1717T006SR drive motors, 17:56 gearing, 25 mm wheels, and four solid-rubber wheels with two wheels per bank sharing a pinion.[^hardware]
* The operating envelope includes scrubbed in-place turns, full-bore straight acceleration limited by ground strikes and motor power, precision turns near walls, constant-velocity exploration, and high-speed arcing turns where stick-slip is expected and required.[^physical]
* The coordinate contract is: global +Y forward, global +X right, body +f forward, body +r right, `heading = 0` facing +Y, and clockwise yaw positive. IMU sign correction is done only in the hardware layer; UKF code must treat IMU measurements as canonical.[^symbols]
* The Faulhaber 1717T006SR motor data used for first-pass priors include \(R_m=4.31\ \Omega\), \(L_m=65.6\ \mu H\), \(K_t=3.96\ \mathrm{mNm/A}\), no-load current \(I_0=0.0459\ \mathrm{A}\), friction torque \(M_R=0.18\ \mathrm{mNm}\), and rotor inertia \(J=0.58\ \mathrm{gcm^2}\).[^motor]
* The IE2-1024 encoder is a two-channel incremental magnetic encoder with 1024 lines/rev, intended for shaft velocity, direction, and positioning.[^encoder]
* The DRV8871 provides PWM H-bridge drive, typical high-side plus low-side \(R_{DS(on)}\approx 565\ \mathrm{m\Omega}\), current regulation, and a 3.6 A peak-current operating region. The datasheet gives \(I_{TRIP}\approx V_{ILIM}/R_{ILIM}\), with typical \(V_{ILIM}=64\ \mathrm{kV}\) in the datasheet’s unit convention, and a typical current-regulation off-time of 25 µs.[^driver]
* The LSM6DSV16X IMU is treated as a raw HAODR accelerometer/gyro source. The relevant software-setting scope is limited to IMU LPF/HPF choices; embedded SFLP/game-vector/gravity/bias outputs are not used in this estimator path. The datasheet supports accelerometer full scales through ±16 g, gyroscope full scales through ±4000 dps on the UI path, high-rate ODRs, timestamping, and configurable UI-path filtering, but robot-level calibration and logged validation remain authoritative.[^imu]

---

## 3. Coordinate and sign conventions

Use the project’s sign convention without reinterpretation.

Body to global velocity transform:

\[
\dot{Px} = Vf\sin(heading) + Vr\cos(heading)
\]

\[
\dot{Py} = Vf\cos(heading) - Vr\sin(heading)
\]

Angular propagation:

\[
\dot{heading} = yawRate
\]

Body dynamics:

\[
\dot{Vf} = Af + yawRate\,Vr
\]

\[
\dot{Vr} = Ar - yawRate\,Vf
\]

\[
\dot{yawRate} = yawAccel
\]

`heading` must be angle-wrapped after propagation. The UKF angle mean should use sine/cosine averaging; that is an implementation detail, but the plant assumes it.

---

## 4. Inputs to the plant

The estimator should receive a time-stamped tick/sample envelope. The timestamp is the single timebase source. The estimator owns the previous prediction timestamp and computes the propagation interval internally:

\[
\Delta t_k = timestamp_k - timestamp_{lastPredict}
\]

or, for fixed-tick replay/configurations, the estimator may use its internally configured tick period and verify consistency against timestamps.

Do **not** carry `dt` as an independent field in the externally supplied plant-input sample when `timestamp` is present. Passing both `timestamp` and `dt` creates two sources of truth; if they disagree, the estimator has to choose which clock represents the actual sample time. The timestamp should win. A `dt` value may exist as an internal derived variable in the prediction routine or as an argument to a pure integration helper, but it should not be part of the external sample contract.

A timing envelope may be represented as:

```text
estimator_tick = {
    timestamp,      // absolute sample/tick time used by the stateful estimator
    drive_sample,
    imu_sample,
    encoder_sample,
    wall_samples
}
```

The plant-specific drive input should contain only drive-relevant values sampled or derived for that tick:

```text
drive_sample = {
    CL, CR,                       // signed left/right motor commands
    wheelRateL, wheelRateR,       // validated encoder-derived wheel-bank rates, rad/s
    wheelRateCov,                 // 2x2 covariance R_omega for [wheelRateL, wheelRateR]^T
    encoderSampleValid,           // acquisition validation result for this tick
    encoderFaultFlags,            // empty/zero in valid operation; logged on fault
    motorPhaseL, motorPhaseR,     // optional encoder phase for ripple fits
    fanCommand                    // external fan/downforce command
}
```

The estimator may receive raw encoder counts in a separate `encoder_sample`, but the plant consumes validated wheel-rate inputs plus their covariance. A sample marked invalid shall not be silently treated as a normal noisy input. Invalid encoder samples activate the degraded prediction and logging path defined in Section 7.5.

This document does not specify the acquisition schedule beyond requiring that the measurement functions receive calibrated samples, timestamps/phase metadata, and the correct sensor extrinsics. Non-IMU sensors are treated as fixed-phase once-per-tick samples; the IMU may have an asynchronous phase and should carry its own measurement timestamp or equivalent phase metadata. The estimator converts those timestamps/phases into local propagation or interpolation intervals.

The implementation must define the command-validity convention. Either `CL/CR` are the commands that were active over the interval ending at `timestamp`, or they are the commands issued at `timestamp` for the next interval. The estimator should not infer this from `dt`; it should use the documented firmware/logging convention and its own stored previous sample to perform zero-order hold or midpoint integration consistently.

`wheelRateL` and `wheelRateR` are measured drivetrain inputs. Their uncertainty is folded into process noise; they are not estimated as latent UKF states.

The vehicle has no `Vbat` measurement, so `Vbat` is also not part of the sample contract. Any motor-bus or battery-voltage dependence must enter through a calibrated command-to-torque map, a nominal run-level prior, or process-noise scheduling, not through a nonexistent measurement.

`wheelAccelL` and `wheelAccelR` are likewise excluded from `drive_sample`. The drivetrain and electrical response are much faster than the 1 ms estimator tick, while tick-to-tick encoder acceleration is quantization-sensitive. Use wheel rates and command history for the mean model, and handle remaining drivetrain/contact mismatch with the torque map, residual acceleration states, and process-noise scheduling.

---

## 5. State inclusions and exclusions

### 5.1 Included: body pose, body velocity, and yaw rate

These six states are mandatory:

\[
Px,\ Py,\ heading,\ Vf,\ Vr,\ yawRate
\]

They are the quantities needed by the controller and mapper. They are also the quantities directly constrained by the IMU, wall sensors, and the kinematic part of the model.

### 5.2 Included: three colored acceleration residuals

The final three states are:

\[
\Delta Af,\quad \Delta Ar,\quad \Delta yawAccel
\]

Their deterministic mean dynamics are first-order decay:

\[
\dot{\Delta Af} = -\frac{\Delta Af}{\tau_f}
\]

\[
\dot{\Delta Ar} = -\frac{\Delta Ar}{\tau_r}
\]

\[
\dot{\Delta yawAccel} = -\frac{\Delta yawAccel}{\tau_y}
\]

Their stochastic dynamics are defined as exact discrete Ornstein-Uhlenbeck residual updates in Section 15.1. The scheduled residual `sigma` values are steady-state residual-state standard deviations, not per-tick perturbations and not continuous-time spectral densities.[^residual_covariance]

The nominal plant computes:

\[
Af_{nom},\quad Ar_{nom},\quad yawAccel_{nom}
\]

and propagation uses:

\[
Af = Af_{nom} + \Delta Af
\]

\[
Ar = Ar_{nom} + \Delta Ar
\]

\[
yawAccel = yawAccel_{nom} + \Delta yawAccel
\]

These residual states are deliberately body-acceleration residuals rather than slip residuals. They cover the actual estimator problem: not just slip, but the body-force mismatch caused by launch threshold, tire scrub, stick-slip, gear variation, downforce/load uncertainty, ground-strike clipping, springback, and current limiting.

### 5.3 Excluded: wheel-rate states

Do not include:

\[
wheelRateL,\quad wheelRateR
\]

as UKF states.

Reasoning:

* The encoders directly observe drivetrain shaft motion at high resolution.
* With 1024 lines/rev and 4× quadrature decoding, the encoder provides \(4096\) counts/motor rev.
* With \(G = 56/17\) motor revs per wheel rev:

\[
countsPerWheelRev = 4096\frac{56}{17}\approx 13493
\]

* For a 25 mm wheel:

\[
wheelCircumference = \pi(0.025) \approx 0.07854\ \mathrm{m}
\]

\[
distancePerCount \approx \frac{0.07854}{13493}\approx 5.82\ \mu\mathrm{m}
\]

* At a 1 ms tick, 1 count/tick is about 5.82 mm/s, and at 0.1 m/s the encoder sees about 17 counts/tick. The operating notes explicitly do not prioritize movement below 0.1 m/s except insofar as in-place turns must work.

The encoders should be used as a direct proxy for wheel-bank rate:

\[
\hat{wheelRate}_j =
\frac{2\pi}{G\,countsPerMotorRev}\frac{\Delta N_j}{\Delta t}
\]

but **not** as a direct proxy for body velocity or yaw rate during slip-heavy operation.

Do not use this as a hard measurement in the main filter:

\[
Vf \approx \frac{r_w}{2}(\hat{wheelRate}_L+\hat{wheelRate}_R)
\]

\[
yawRate \approx \frac{r_w}{b}(\hat{wheelRate}_L-\hat{wheelRate}_R)
\]

except as a gated, low-slip pseudo-measurement.

Wheel-rate states would mainly smooth quantization and handle dropout. They do not solve the real problem: estimating body motion under scrub and stick-slip. The two state slots are more valuable as body-acceleration residuals.

### 5.4 Excluded: gyro-bias state

Do not include:

\[
gyroBiasYaw
\]

as a UKF state for this vehicle, given the project observation that yaw-gyro bias is stable for at least about 15 minutes and the battery lasts about 20 minutes.

Maintain instead:

\[
gyroBiasYawExt
\]

outside the UKF as a run-level calibration parameter. Update it only during confirmed stationary intervals.

The gyro measurement entering the UKF is:

\[
z_{gyro}=gyroYawMeas - gyroBiasYawExt
\]

and the measurement model is:

\[
h_{gyro}(state\_vec)=yawRate
\]

If an external covariance is maintained for the bias estimate, propagate its contribution to heading covariance or fold it into a conservative yaw process-noise term. Do not represent residual bias uncertainty purely as independent per-sample white gyro noise if it is material; constant residual bias integrates coherently into heading. In this application, observed stability makes the state slot better spent on `DeltaYawAccel`.

A gyro-bias state should be reconsidered only if logs show within-run drift large enough that:

\[
|\delta b|\,T_{uncorrected}
\]

is comparable to the heading error budget.

### 5.5 Excluded: slip states

Do not include a pair of slip states such as:

\[
slipL,\quad slipR
\]

in the main 9-state UKF.

Reasons:

* A left/right slip pair is too low-dimensional for four contact patches, lateral scrub, front/rear load transfer, tire springback, downforce changes, and floor-condition variation.
* Slip is already computable algebraically from encoder-derived wheel rate and each sigma point’s body state.
* Slip-ratio states are poorly conditioned near zero speed, exactly where in-place turns matter.
* Slip-velocity states avoid the singularity, but then mostly duplicate an algebraic quantity that the model already computes.
* The primary estimator target is robust pose and body motion, not explicit tire-state estimation.

Use slip velocities algebraically:

\[
slipForward_i = r_w\hat{wheelRate}_{side(i)} - (Vf - yawRate\,r_i)
\]

\[
slipRight_i = -(Vr + yawRate\,f_i)
\]

If explicit slip estimates are useful for traction control or diagnostics, compute them from the UKF sigma points or run a separate side observer. Do not spend two of the nine main UKF states unless held-out logs show a clear pose-estimation improvement.

### 5.6 Excluded: motor-current states

Do not include motor current as a state.

For the 1717T006SR motor:

\[
\tau_e = \frac{L_m}{R_m}=\frac{65.6\ \mu H}{4.31\ \Omega}\approx 15.2\ \mu s
\]

This is far below the 1 ms control tick. The project notes also state drivetrain force transfer stabilizes in about 22 µs. The DRV8871 current regulation is likewise a fast electrical effect relative to the UKF tick. Model motor current algebraically with smooth current limiting, and put driver/motor mismatch into process-noise scheduling.

### 5.7 Excluded: accelerometer-bias states

Do not include forward/lateral accelerometer-bias states in the 9-state UKF initially.

Reasons:

* The three residual states already let the filter correct body-acceleration mismatch.
* Accelerometer biases can be calibrated externally at startup and monitored during confirmed stationary periods.
* High-dynamic accelerometer errors in this vehicle are more likely to be pitch/roll/impact contamination and tire/contact impulses than slow DC bias.
* Spending two states on accelerometer bias would displace residual dynamics that directly handle scrub, arcs, and ground-strike effects.

Use calibrated constants:

\[
accelBiasFCal,\quad accelBiasRCal
\]

and inflate accelerometer measurement noise during detected impact, pitch contamination, or severe stick-slip.

### 5.8 Excluded: pitch, roll, vertical position, and vertical velocity

Do not make this a 3D inertial navigation filter.

Floor strikes and body rocking matter, but the estimator’s job is planar pose. Model floor-strike effects as:

* acceleration-envelope clipping,
* process-noise inflation,
* accelerometer-measurement covariance inflation,
* optional impact flags from vertical acceleration or jerk.

Do not introduce pitch/roll/vertical states unless logs prove that a planar UKF cannot maintain pose accuracy through the relevant events.

### 5.9 Excluded: fan/downforce state

Do not estimate fan/downforce as a UKF state. Use an algebraic calibrated function:

\[
N_{fan}=F_{fan}(fanCommand; fanCalParams)
\]

and include unmeasured bus sag, fan variation, and floor/load uncertainty in process noise. A fan state is not justified unless fan dynamics are slow, repeatable, and observable from existing measurements. First-pass treatment should be a calibrated modifier to normal load.

### 5.10 Excluded: map or wall-geometry states

Do not put wall positions in this UKF. The maze map/wall hypothesis should be supplied by the mapper/controller. The UKF measurement function raycasts against known or hypothesized wall geometry and skips updates when no valid wall hypothesis exists.

---

## 6. Smooth helper functions

Regime changes must be continuous. Use smooth saturations and soft deadzones rather than discrete mode switches.

Smooth sign:

\[
sgnE(x,x_E)=\tanh\left(\frac{x}{x_E}\right)
\]

Smooth saturation:

\[
satE(x,x_{max}) = x_{max}\tanh\left(\frac{x}{x_{max}}\right)
\]

Softplus:

\[
softplusE(x,e)=e\log(1+\exp(x/e))
\]

Smooth deadzone:

\[
deadzoneE(x,a,e)=softplusE(x-a,e)-softplusE(-x-a,e)
\]

Smooth step from 0 to 1:

\[
smoothStepE(x,x_0,x_1)=\frac{1}{2}\left[1+\tanh\left(\frac{x-(x_0+x_1)/2}{(x_1-x_0)/6}\right)\right]
\]

Use a numerically stable implementation of `softplusE` for large positive or negative arguments.

Smooth max and min:

\[
maxE(a,b,e)=\frac{a+b+\sqrt{(a-b)^2+e^2}}{2}
\]

\[
minE(a,b,e)=\frac{a+b-\sqrt{(a-b)^2+e^2}}{2}
\]

Smooth asymmetric clipping:

\[
clipAsymE(x,x_{min},x_{max},e)=maxE(x_{min},minE(x,x_{max},e),e)
\]

---

## 7. Encoder-derived drivetrain inputs

The encoder subsystem provides drivetrain measurements, not body-motion measurements. The plant consumes validated wheel-bank rates:

\[
\hat{wheelRate}_L,\quad \hat{wheelRate}_R
\]

and their covariance. The estimator shall not create UKF wheel-rate states merely to smooth encoder data.

### 7.1 Wheel-rate conversion

For each side \(j\in\{L,R\}\):

\[
\hat{wheelRate}_j =
\frac{2\pi}{G\,countsPerMotorRev}\frac{\Delta N_j}{\Delta t}
\]

where:

\[
G = \frac{56}{17}
\]

if the convention is motor revolutions per wheel revolution, consistent with:

\[
\phi_w = \frac{\phi_m}{G}
\]

If firmware uses 4× quadrature:

\[
countsPerMotorRev=4096
\]

If firmware counts lines or uses a different edge convention, use the firmware's actual convention. With the 4× convention:

\[
countsPerWheelRev = 4096\frac{56}{17}\approx13493
\]

For a 25 mm wheel:

\[
distancePerCount=\frac{\pi(0.025)}{13493}\approx5.82\ \mu m
\]

This supports treating validated encoder samples as high-quality drivetrain inputs. It does **not** support treating drivetrain motion as body motion during scrub or slip.

### 7.2 Encoder sample validation

Each encoder-derived sample shall carry `encoderSampleValid` or an equivalent validation result. A sample is valid only if all configured acquisition checks pass, including at minimum:

```text
timestamp monotonic
count delta finite
count delta physically plausible for the sample interval
left/right polarity and direction convention valid
quadrature transition legal, if decoded in software or exposed by hardware
counter continuity preserved
sample interval inside accepted timing bounds
```

Additional hardware-specific checks may be added, such as counter-wrap validation, DMA/ISR overrun detection, and timestamp-source continuity.

### 7.3 Nominal wheel-rate covariance

For a valid encoder sample, the nominal wheel-rate variance is:

\[
\sigma^2_{wheelRate,j,normal}
=
\left(\frac{2\pi}{G\,countsPerMotorRev\,\Delta t}\right)^2
\sigma^2_{\Delta N}
+
\sigma^2_{wheelRate,timing,j}
\]

with first-pass count-quantization variance:

\[
\sigma^2_{\Delta N}\approx \frac{1}{12}
\]

A practical first timing term is:

\[
\sigma_{wheelRate,timing,j}
\approx
|\hat{wheelRate}_j|\frac{\sigma_{\Delta t,j}}{\Delta t}
\]

or an equivalent phase/timestamp uncertainty term derived from logged firmware timing characterization.

The nominal covariance for validated samples explicitly excludes routine missed-edge and filter-delay terms:

\[
\sigma^2_{wheelRate,missedEdge,j}=0
\]

\[
\sigma^2_{wheelRate,filterDelay,j}=0
\]

unless firmware actually applies a wheel-rate filter. If such a filter exists, its delay/phase uncertainty must be modeled from the actual filter implementation and logs, not inserted as a generic placeholder.

Therefore the valid-sample covariance is:

\[
\boxed{
\sigma^2_{wheelRate,j,normal}
=
\sigma^2_{quantization,j}+\sigma^2_{timing/phase,j}
}
\]

not:

\[
\sigma^2_{quantization}
+\sigma^2_{timing}
+\sigma^2_{missedEdge}
+\sigma^2_{filterDelay}
\]

### 7.4 Wheel-rate covariance matrix

Define:

\[
u_\omega =
\begin{bmatrix}
\hat{wheelRate}_L \\
\hat{wheelRate}_R
\end{bmatrix}
\]

\[
R_\omega =
\begin{bmatrix}
\sigma^2_{wheelRate,L} & \sigma_{LR} \\
\sigma_{LR} & \sigma^2_{wheelRate,R}
\end{bmatrix}
\]

Usually:

\[
\sigma_{LR}=0
\]

unless a shared timing or sample-phase error creates correlated wheel-rate uncertainty. For invalid samples, the estimator shall either not form a normal \(R_\omega\) or shall mark it as degraded and use one of the explicit degraded prediction choices in Section 7.5.

### 7.5 Invalid-sample fault handling

When `encoderSampleValid == true`:

```text
use normal encoder wheel-rate covariance
propagate wheel-rate input uncertainty through the nonlinear plant
allow the rolling pseudo-measurement only if regime gates also pass
```

When `encoderSampleValid == false`:

```text
do not consume the sample as a normal drivetrain input
disable the rolling pseudo-measurement for that tick
activate encoder-fault process-noise scheduling
mark estimator/input degraded for logging
log count delta, timestamp, validation result, and fault reason
```

Permitted degraded prediction choices are:

1. propagate from the command-to-torque prior with high process noise;
2. hold the previous valid wheel rates for one tick with high input covariance;
3. skip encoder-dependent contact-force refinement for that tick and rely on IMU/wall updates plus residual process noise.

The implementation shall not silently absorb an invalid encoder sample by increasing nominal encoder covariance. Missed edges, illegal transitions, counter discontinuities, implausible deltas, timestamp discontinuities, or acquisition overruns are fault events.

### 7.6 Required propagation of wheel-rate input uncertainty

Wheel-rate uncertainty affects motor back-EMF, command/rate torque mapping, contact-patch slip velocity, tire force, combined-slip utilization, regime scheduling, and the optional rolling pseudo-measurement. Therefore \(R_\omega\) must enter before these nonlinear calculations, not only as a final additive acceleration covariance.

The compliant propagation methods are specified in Section 15.4.

---

## 8. Motor and driver algebra

Normalize commands:

\[
u_L=normalize(CL),\quad u_R=normalize(CR)
\]

with \(u_j\in[-1,1]\).

Motor rate:

\[
motorRate_j = G\,\hat{wheelRate}_j
\]

Because the vehicle does not measure battery or motor-bus voltage, the preferred nominal drive model is a fitted command/rate/phase torque map:

\[
T_{bankRaw,j}=T_{cmdMap,j}(u_j,motorRate_j,motorPhase_j)
\]

The map should be identified from logs and bench tests. It should include the mean effects of motor back-EMF, driver loss, gearing, current limiting, command normalization, and gear/ripple structure that is repeatable with motor phase.

A voltage-equivalent model may be used only as a prior or fallback:

\[
V_{driveNom,j}=u_j V_{busEff}
\]

\[
I^{raw}_j=
\frac{V_{driveNom,j}-K_e motorRate_j}{R_m+R_{drv}+R_{wire}}
\]

where \(V_{busEff}\) is a calibrated/run-level parameter, not a measured vehicle input and not a UKF state. Since actual bus sag is unmeasured, any voltage-equivalent model must carry enough process noise to avoid over-trusting torque during high-current launch, braking, and scrub.

In SI units:

\[
K_e=K_t
\]

Use:

\[
R_{drv}\approx 0.565\ \Omega
\]

as a first-pass DRV8871 high-side plus low-side typical on-resistance, with temperature scaling if available.

Current regulation for the optional voltage-equivalent prior:

\[
I_j = I_{trip}\tanh\left(\frac{I^{raw}_j}{I_{trip}}\right)
\]

where:

\[
I_{trip}\approx \frac{64}{R_{ILIM,k\Omega}}\quad \mathrm{A}
\]

using the DRV8871 datasheet equation with \(R_{ILIM}\) in kΩ.

Optional motor torque prior:

\[
T_{motor,j}=K_t I_j - M_R\,sgnE(motorRate_j,motorRate_E)
\]

Do not double-count both friction torque \(M_R\) and no-load-current torque \(K_t I_0\) unless logs show an additional loss term.

Optional wheel-bank torque prior:

\[
T_{bankRaw,j}=\eta_{drive}G\,T_{motor,j}
\]

The fitted \(T_{cmdMap,j}\) form should supersede the voltage-equivalent expression once logs are available.

### Launch and low-speed breakaway

Straight movement has a distinct launch threshold independent of rolling resistance. Apply a smooth low-speed deadzone to usable bank torque:

\[
slow_j=\exp\left[-\left(\frac{r_w|\hat{wheelRate}_j|}{v_{static}}\right)^2\right]
\]

\[
T_{bank,j}=deadzoneE(T_{bankRaw,j},\ slow_j T_{launch,j},\ T_E)
\]

Then subtract rolling/gear losses:

\[
T_{bank,j}\leftarrow
T_{bank,j}
- T_{roll,j}\,sgnE(\hat{wheelRate}_j,wheelRate_E)
- T_{visc,j}\hat{wheelRate}_j
- T_{ripple,j}(motorPhase_j)
\]

Set \(T_{visc,j}=0\) initially. The physical notes say straight viscous resistance is roughly zero and constant friction dominates pure forward/reverse motion.

### Drive force feedforward

Because wheel rates are not states, there is no internal wheel angular dynamics equation. Do **not** pass encoder-derived wheel acceleration into the nominal plant.

The hardware does not justify `wheelAccelL` / `wheelAccelR` as stable plant inputs:

* the physical drivetrain force-transfer note gives about 22 µs settling, far below the 1 ms tick,
* the Faulhaber 1717T006SR electrical time constant is \(L_m/R_m \approx 65.6\ \mu H / 4.31\ \Omega \approx 15.2\ \mu s\),
* the DRV8871 current-regulation off-time is about 25 µs,
* a finite difference of encoder-derived wheel rate has poor tick-to-tick quantization and timing-noise behavior relative to the sub-tick drivetrain dynamics.

Therefore use the quasi-static mean drive-force input:

\[
F_{drive,j}\approx \frac{T_{bank,j}}{r_w}
\]

and handle remaining drivetrain/contact mismatch through the fitted torque map, launch/breakaway model, residual acceleration states, and process-noise scheduling. Wheel acceleration may still be useful for diagnostics or offline identification, but it should not be part of `drive_sample` for the UKF plant.

---

## 9. Contact-patch kinematics

Use four contact patches even though wheel rate is measured per side. Each contact patch \(i\) has calibrated body-frame location:

\[
(r_i,f_i)
\]

where \(r_i>0\) is right of center and \(f_i>0\) is forward of center.

Velocity of the body at the contact patch:

\[
contactForwardVelocity_i=Vf-yawRate\,r_i
\]

\[
contactRightVelocity_i=Vr+yawRate\,f_i
\]

Wheel surface forward velocity:

\[
surfaceForwardVelocity_i = r_w\hat{wheelRate}_{side(i)}
\]

Slip velocities:

\[
slipForwardVelocity_i = surfaceForwardVelocity_i-contactForwardVelocity_i
\]

\[
slipRightVelocity_i = -contactRightVelocity_i
\]

These slip velocities are continuous at zero body speed, in launch, in-place turns, mid-speed arcs, and high-speed stick-slip arcs.

---

## 10. Normal load model

Use an algebraic normal-load model:

\[
N_i=N_{static,i}+N_{fan,i}+N_{longTransfer,i}+N_{latTransfer,i}
\]

Then smooth-clamp:

\[
N_i\leftarrow maxE(N_i,N_{min})
\]

where `maxE` is the smooth maximum from the helper-function section.

Use fan/downforce as:

\[
N_{fan,i}=w_{fan,i}F_{fan}(fanCommand; fanCalParams)
\]

Use load transfer from the most recent raw acceleration estimate or a one-step fixed-point iteration. Do not make load transfer discontinuous.

---

## 11. Tire-force model

Use a smooth combined-slip model. The nominal force request can include both drive feedforward and passive slip stiffness.

Distribute side drive force by normal load weighting:

\[
w_{i|j}=\frac{N_i}{\sum_{k\in j}N_k+\epsilon}
\]

\[
F_{driveReq,i}=w_{i|side(i)}F_{drive,side(i)}
\]

Raw force request:

\[
F_{f,req,i}=F_{driveReq,i}+K_{f,i}\,slipForwardVelocity_i
\]

\[
F_{r,req,i}=K_{r,i}\,slipRightVelocity_i
\]

The feedforward term gives the model a physically reasonable torque-to-force path; the slip-stiffness term captures passive tire deformation, scrub, and mismatch between wheel and body motion. The split is empirical and should be fitted from logs.

Slip speed:

\[
slipSpeed_i =
\sqrt{slipForwardVelocity_i^2+slipRightVelocity_i^2+v_E^2}
\]

Stribeck-like friction envelope:

\[
\mu_i(slipSpeed_i)=
\mu_{slide,i}
+
(\mu_{peak,i}-\mu_{slide,i})
\exp\left[-\left(\frac{slipSpeed_i}{v_{Stribeck,i}}\right)^2\right]
\]

Contact force limit:

\[
F_{limit,i}=\mu_i N_i
\]

Combined-slip utilization:

\[
util_i=
\sqrt{
\left(\frac{F_{f,req,i}}{\lambda_f F_{limit,i}+\epsilon}\right)^2+
\left(\frac{F_{r,req,i}}{\lambda_r F_{limit,i}+\epsilon}\right)^2+
\epsilon^2
}
\]

Smooth scale:

\[
forceScale_i=\frac{\tanh(util_i)}{util_i}
\]

Final contact forces:

\[
F_{f,i}=forceScale_iF_{f,req,i}
\]

\[
F_{r,i}=forceScale_iF_{r,req,i}
\]

This gives three continuous regimes:

| Region | Condition | Behavior |
|---|---:|---|
| Rolling-dominant | \(util_i\ll 1\) | force approximately follows drive/slip request |
| Saturating | \(util_i\approx 1\) | force bends smoothly toward contact envelope |
| Sliding/stick-slip-prone | \(util_i>1\) | mean force saturates and process noise increases |

---

## 12. Nominal body dynamics

Raw nominal accelerations:

\[
Af_{raw}=\frac{\sum_iF_{f,i}}{m_{eff,f}}
\]

\[
Ar_{raw}=\frac{\sum_iF_{r,i}}{m_{eff,r}}
\]

Clockwise-positive yaw moment:

\[
yawMomentRaw=\sum_i(f_iF_{r,i}-r_iF_{f,i})
\]

### In-place yaw breakaway

Define a smooth in-place blend. A practical form is:

\[
vBody=\sqrt{Vf^2+Vr^2}
\]

\[
opposedDrive=\frac{1}{2}\left[1-\tanh\left(\frac{CL\,CR}{C_E^2}\right)\right]
\]

\[
turnDemand=\tanh\left(\frac{|CL-CR|}{C_{turnE}}\right)
\]

\[
inPlaceBlend=
\exp\left[-\left(\frac{vBody}{v_{inPlace}}\right)^2\right]
opposedDrive\,turnDemand
\]

Apply a yaw breakaway deadzone:

\[
yawMomentBreakaway=deadzoneE(yawMomentRaw,yawBreakawayMoment,yawMoment_E)
\]

Then:

\[
yawMoment=
(1-inPlaceBlend)yawMomentRaw
+inPlaceBlend\,yawMomentBreakaway
-yawKineticMoment\,sgnE(yawRate,yawRate_E)
\]

\[
yawAccel_{raw}=\frac{yawMoment}{I_z}
\]

### Ground-strike / acceleration envelope

The straight-line floor-strike mechanism is primarily longitudinal acceleration causing pitch load transfer and nose/tail strike. Apply this to forward/reverse acceleration, not automatically to every in-place turn.

Smooth asymmetric clip:

\[
Af_{clip}=clipAsymE(Af_{raw},-Af_{reverseLimit},Af_{forwardLimit})
\]

Use:

\[
scale_f=\frac{Af_{clip}}{Af_{raw}+\epsilon}
\]

and apply it only when \(|Af_{raw}|\) is sufficiently above \(\epsilon\). Then:

\[
Af_{nom}=Af_{clip}
\]

\[
F_{f,i}\leftarrow scale_fF_{f,i}
\]

Recompute yaw moment after scaling if the scaling is material.

Important: a clean in-place turn should not inherently floor-strike. If a strike occurs during an in-place turn, it is likely from asymmetric scrub breakaway, tire springback, a shifted instantaneous center of rotation, or chassis rocking. Treat that as an impact/noise event, not as a deterministic yaw-force effect.

Final nominal terms:

\[
Af_{nom}=Af_{clip}
\]

\[
Ar_{nom}=Ar_{raw}
\]

\[
yawAccel_{nom}=yawAccel_{raw}
\]

Actual propagation terms:

\[
Af=Af_{nom}+\Delta Af
\]

\[
Ar=Ar_{nom}+\Delta Ar
\]

\[
yawAccel=yawAccel_{nom}+\Delta yawAccel
\]

---

## 13. State propagation

The deterministic continuous-time derivative is:

\[
\dot{state\_vec}=\begin{bmatrix}
Vf\sin(heading)+Vr\cos(heading) \\
Vf\cos(heading)-Vr\sin(heading) \\
yawRate \\
Af+yawRate\,Vr \\
Ar-yawRate\,Vf \\
yawAccel \\
-\Delta Af/\tau_f \\
-\Delta Ar/\tau_r \\
-\Delta yawAccel/\tau_y
\end{bmatrix}
\]

The residual-state stochastic terms are not injected as informal per-tick additions to this derivative. They are discretized with the exact residual OU update in Section 15.1 and then represented in the UKF process covariance.

At 1 kHz, RK2 is usually acceptable. RK4 is safer for aggressive arcs, in-place breakaway, and stick-slip logs. Euler is acceptable only for bring-up or low-speed testing.

---

## 14. Regime handling

The model does not switch between separate plant equations. It computes continuous scalars and schedules covariance.

### 14.1 In-place turns with scrubbing

In-place turns have:

\[
Vf\approx0,\quad Vr\approx0,\quad yawRate\ne0
\]

At each contact:

\[
contactForwardVelocity_i=-yawRate\,r_i
\]

\[
contactRightVelocity_i=yawRate\,f_i
\]

So the lateral slip term:

\[
slipRightVelocity_i=-yawRate\,f_i
\]

is nonzero except at the centerline. This is the scrub. It is expected, not a fault.

Handling:

* `inPlaceBlend` approaches 1 when translation is low and the commanded drive is opposed.
* `yawMomentBreakaway` captures the observed static yaw threshold.
* `yawKineticMoment` captures roughly constant moving scrub resistance.
* The optional rolling pseudo-measurement is disabled.
* Process noise increases in `DeltaYawAccel` and usually `DeltaAr`.
* Accelerometer covariance is inflated if jerk or vertical acceleration indicates impact/springback.

Do not infer yaw rate from encoder differential during in-place turns. That assumes rolling contact, which is exactly false here.

### 14.2 Mid-speed turns with mostly rolling behavior

A rolling-dominant turn has moderate forward speed and low contact utilization:

\[
util_{max}=\max_i util_i < util_{roll}
\]

Define:

\[
rollingBlend=(1-inPlaceBlend)
\exp\left[-\left(\frac{util_{max}}{util_{roll}}\right)^4\right]
\,smoothStepE(vBody,v_{rollLow},v_{rollHigh})
\]

When `rollingBlend` is high:

* The tire model is near linear.
* Residuals should decay toward zero.
* The optional rolling pseudo-measurement can be enabled with low covariance.
* Wall, gyro, accelerometer, and encoder-derived inputs should be mutually consistent.

Do not make rolling exact. Use it as a soft constraint.

### 14.3 High-speed turns with stick-slip

High-speed arcs have high body speed and high contact utilization. The project notes identify 1–2.5 m/s arcs with 63–210 mm radius as requiring excursions beyond the conventional traction limit, with stick-slip expected.

Define:

\[
stickSlipIndex=
smoothStepE(util_{max},util_{ssLow},util_{ssHigh})
\,smoothStepE(vBody,v_{ssLow},v_{ssHigh})
\]

Optionally include lateral demand:

\[
latDemand=\frac{|Ar_{nom}|}{g}
\]

\[
stickSlipIndex\leftarrow stickSlipIndex\,smoothStepE(latDemand,latLow,latHigh)
\]

Handling:

* The Stribeck/combined-slip model gives a smooth mean force.
* The filter does not try to deterministically predict every stick-slip event.
* Process noise rises strongly in `DeltaAr` and `DeltaYawAccel`, and moderately in `DeltaAf`.
* The rolling pseudo-measurement covariance becomes very large.
* Wall-sensor updates are gated more aggressively because vibration, corner geometry, and transient attitude can increase bad innovations.

---

## 15. Process-noise scheduling and covariance propagation

Use continuous scheduling; do not switch plant models. This section defines the regime scalars used for scheduling, the stochastic semantics of the three colored residual states, and the required covariance propagation for both residual innovations and encoder wheel-rate input uncertainty.[^residual_covariance][^encoder_uncertainty]

### 15.1 Residual-state stochastic semantics

The scheduled residual-noise values are **steady-state standard deviations of the residual states**:

\[
\sigma_{\Delta Af,ss},\quad \sigma_{\Delta Ar,ss},\quad \sigma_{\Delta yawAccel,ss}
\]

They are not per-tick standard deviations, not continuous-time spectral densities, and not arbitrary covariance increments.

For each residual channel:

\[
q\in\{Af,Ar,yawAccel\}
\]

with residual state \(\Delta q\), time constant \(\tau_q\), and scheduled steady-state standard deviation \(\sigma_{\Delta q,ss}\), use the exact discrete Ornstein-Uhlenbeck update over the propagation interval \(\Delta t\):

\[
\phi_q=\exp\left(-\frac{\Delta t}{\tau_q}\right)
\]

\[
\Delta q_{k+1}=\phi_q\Delta q_k+\eta_q
\]

\[
\eta_q\sim\mathcal{N}\left(0,Q_{\Delta q}\right)
\]

\[
Q_{\Delta q}=\sigma_{\Delta q,ss}^2\left(1-\phi_q^2\right)
\]

The three concrete residual updates are:

\[
\phi_f=\exp\left(-\frac{\Delta t}{\tau_f}\right),\quad
Q_{\Delta Af}=\sigma_{\Delta Af,ss}^2\left(1-\phi_f^2\right)
\]

\[
\phi_r=\exp\left(-\frac{\Delta t}{\tau_r}\right),\quad
Q_{\Delta Ar}=\sigma_{\Delta Ar,ss}^2\left(1-\phi_r^2\right)
\]

\[
\phi_y=\exp\left(-\frac{\Delta t}{\tau_y}\right),\quad
Q_{\Delta yawAccel}=\sigma_{\Delta yawAccel,ss}^2\left(1-\phi_y^2\right)
\]

This convention makes the scheduled value directly tune the expected residual-state magnitude in a given regime. With constant \(\sigma_{\Delta q,ss}\) and \(\tau_q\), a residual-only simulation should converge to RMS \(\sigma_{\Delta q,ss}\) independent of timestep. With \(\sigma_{\Delta q,ss}=0\), the residual decays exactly by \(\exp(-\Delta t/\tau_q)\).

The equivalent continuous-time driving-noise spectral density would be:

\[
q_{c,q}=\frac{2\sigma_{\Delta q,ss}^2}{\tau_q}
\]

but this document does not use \(q_c\) as the scheduled tuning parameter.

### 15.2 Regime scalars

Define:

\[
util_{max}=\max_i util_i
\]

Use a dimensionless drive-saturation / drive-authority scalar rather than a measured-current scalar:

\[
driveSaturationIndex=
\begin{cases}
\max_j\left|\dfrac{I^{raw}_j}{I_{trip}}\right|, & \text{if using the voltage-equivalent prior}\\[6pt]
\text{map saturation proxy}, & \text{if using }T_{cmdMap,j}
\end{cases}
\]

Because there is no live bus-voltage measurement, also use a dimensionless drive-authority uncertainty term:

\[
driveAuthorityUncertainty=
\frac{\sigma_{VbusEff}}{\max(|V_{busEff}|,V_E)}
\]

or the equivalent uncertainty output from the fitted command-to-torque map.

\[
groundUse=\frac{|Af_{raw}-Af_{clip}|}{|Af_{raw}|+\epsilon}
\]

\[
impactSuspect=
smoothStepE(|Az-g|,zLow,zHigh)
+k_j smoothStepE(|jerk_f|+|jerk_r|,jLow,jHigh)
+k_s inPlaceBlend\,util_{max}
\]

Define a binary encoder-fault indicator for the current propagation tick:

\[
I_{encoderFault}=\begin{cases}
0, & encoderSampleValid=true\\
1, & encoderSampleValid=false
\end{cases}
\]

### 15.3 Scheduled steady-state residual sigmas

Schedule the steady-state residual standard deviations. These equations produce \(\sigma_{\Delta q,ss}\), not the discrete covariance entries directly:

\[
\sigma_{\Delta Af,ss}=\sigma_{Af,base,ss}
+k_{Af,util}util_{max}^2
+k_{Af,ground}groundUse
+k_{Af,drive}driveSaturationIndex^2
+k_{Af,supply}driveAuthorityUncertainty
+k_{Af,impact}impactSuspect
+k_{Af,encoderFault}I_{encoderFault}
\]

\[
\sigma_{\Delta Ar,ss}=\sigma_{Ar,base,ss}
+k_{Ar,util}util_{max}^2
+k_{Ar,ss}stickSlipIndex
+k_{Ar,scrub}inPlaceBlend
+k_{Ar,drive}driveSaturationIndex^2
+k_{Ar,supply}driveAuthorityUncertainty
+k_{Ar,impact}impactSuspect
+k_{Ar,encoderFault}I_{encoderFault}
\]

\[
\sigma_{\Delta yawAccel,ss}=\sigma_{yaw,base,ss}
+k_{yaw,util}util_{max}^2
+k_{yaw,ss}stickSlipIndex
+k_{yaw,inPlace}inPlaceBlend
+k_{yaw,drive}driveSaturationIndex^2
+k_{yaw,supply}driveAuthorityUncertainty
+k_{yaw,impact}impactSuspect
+k_{yaw,encoderFault}I_{encoderFault}
\]

Convert those scheduled steady-state standard deviations into discrete residual-state variances using the exact OU equations in Section 15.1 before applying them to the process covariance.

The encoder-fault terms are a degraded-data path. They are not a substitute for a nominal missed-pulse covariance term.

### 15.4 Required process-noise and input-noise propagation

Residual acceleration noise and encoder wheel-rate uncertainty shall propagate through the plant into the full state covariance. It is not compliant to add \(Q_{\Delta Af}\), \(Q_{\Delta Ar}\), and \(Q_{\Delta yawAccel}\) only to the last three diagonal entries after state propagation.

At minimum, residual process noise must affect:

```text
DeltaAf, DeltaAr, DeltaYawAccel
Vf, Vr, yawRate
Px, Py, heading over propagation
residual-to-motion cross-covariances
```

Wheel-rate input uncertainty must enter before the nonlinear calculations that depend on wheel rate:

```text
motor back-EMF
command/rate/phase torque map
drive-force feedforward
contact-patch slip velocity
tire-force model
combined-slip utilization
regime/noise scheduling
rolling pseudo-measurement covariance
```

Use one of the following two compliant methods.

#### Method A — augmented prediction noise

Augment the UKF prediction with process/input-noise variables at least for:

\[
\nu =
\begin{bmatrix}
\eta_f & \eta_r & \eta_y & \delta wheelRate_L & \delta wheelRate_R
\end{bmatrix}^T
\]

where \(\eta_f,\eta_r,\eta_y\) are the exact discrete OU residual innovations, and \(\delta wheelRate_L,\delta wheelRate_R\) are encoder wheel-rate input perturbations with covariance \(R_\omega\).

For each augmented sigma point:

\[
wheelRate_j^{sample}=\hat{wheelRate}_j+\delta wheelRate_j
\]

apply:

\[
\Delta Af_{k+1}=\phi_f\Delta Af_k+\eta_f
\]

\[
\Delta Ar_{k+1}=\phi_r\Delta Ar_k+\eta_r
\]

\[
\Delta yawAccel_{k+1}=\phi_y\Delta yawAccel_k+\eta_y
\]

Then run the complete plant propagation with the sampled wheel rates and sampled residual innovations.

Optional stochastic terms for \(\delta VbusEff\), \(\delta u_L\), or \(\delta u_R\) may be added only if their uncertainty is characterized and the added terms are not double-counted in the residual schedule.

#### Method B — sensitivity-mapped additive covariance

If the implementation uses additive process covariance, compute local sensitivity mappings and apply:

\[
Q_x \leftarrow Q_x + G_\eta Q_\eta G_\eta^T + J_\omega R_\omega J_\omega^T
\]

where:

\[
Q_\eta=\operatorname{diag}
\left(
Q_{\Delta Af},Q_{\Delta Ar},Q_{\Delta yawAccel}
\right)
\]

Residual-noise sensitivity:

\[
G_\eta[:,i]\approx
\frac{
wrapStateDelta\left(f(x,u,\eta=\epsilon e_i)-f(x,u,\eta=0)\right)
}{\epsilon}
\]

Encoder-input sensitivity:

\[
J_\omega[:,j]\approx
\frac{
wrapStateDelta\left(f(x,u_\omega+\epsilon e_j)-f(x,u_\omega-\epsilon e_j)\right)
}{2\epsilon}
\]

The finite difference shall be applied to the complete propagation function, including motor algebra, slip calculation, tire-force calculation, utilization, regime scheduling, residual application, and integration. Heading differences shall be angle-wrapped before forming sensitivity columns.

The sensitivity method may use the predicted mean state for computational economy, but the resulting covariance must be inflated conservatively if the operating point is near a saturation, breakaway threshold, stick-slip boundary, or encoder-fault state.

### 15.5 Schedule evaluation convention

The preferred convention is sigma-point-local scheduling:

1. For each sigma point, compute the algebraic plant terms and regime scalars: `utilMax`, `inPlaceBlend`, `rollingBlend`, `stickSlipIndex`, `groundUse`, `impactSuspect`, and drive-authority terms.
2. Compute \(\sigma_{\Delta Af,ss}\), \(\sigma_{\Delta Ar,ss}\), and \(\sigma_{\Delta yawAccel,ss}\) for that sigma point.
3. Convert those values to discrete OU variances over the propagation substep.
4. Propagate residual and encoder-input uncertainty using Method A or Method B from Section 15.4.
5. Freeze scheduled values over that propagation substep.

If the UKF implementation does not support sigma-point-specific process noise, use predicted-mean regime scalars to compute one shared process-noise matrix, then inflate conservatively. Use prior-mean scalars only as a documented fallback when predicted-mean auxiliary terms are unavailable. Do not mix per-sigma-point regime scalars with a process-noise matrix computed from a different state without documenting the approximation.

If the deterministic integrator uses RK2/RK4 substeps, either recompute the regime scalars at each substep or freeze them at the start of the substep. Do not let the schedule change discontinuously inside a single integration substep.

### 15.6 Acceptance checks

The implementation should pass these process-noise and covariance checks:

1. With constant \(\sigma_{\Delta q,ss}\) and \(\tau_q\), a stationary residual-only simulation converges to RMS \(\sigma_{\Delta q,ss}\), independent of \(\Delta t\).
2. Changing \(\Delta t\) from 1.0 ms to 0.5 ms while preserving the same continuous trajectory does not materially change residual variance.
3. With \(\sigma_{\Delta q,ss}=0\), residual states decay exactly by \(e^{-\Delta t/\tau_q}\).
4. Scheduled noise does not jump discontinuously at regime boundaries, because the schedule inputs use smooth scalars.
5. With IMU and wall updates disabled, increasing `sigmaDeltaAfSs` increases covariance in `Vf`, `Px`, and `Py`, not only `DeltaAf`.
6. Increasing `sigmaDeltaYawAccelSs` increases covariance in `yawRate` and `heading`, not only `DeltaYawAccel`.
7. Residual-to-motion cross-covariances become nonzero after prediction.
8. Increasing valid-sample encoder quantization/timing covariance increases predicted state covariance through the nonlinear plant.
9. Encoder wheel-rate uncertainty affects utilization-derived noise scheduling when the nominal operating point is near a utilization threshold.
10. Setting `encoderSampleValid=false` disables the rolling pseudo-measurement and activates the encoder-fault degraded-data process-noise path.

---

## 16. Measurement model overview and scope boundary

The measurement-acquisition setup is outside the scope of this plant/measurement specification. The model consumes calibrated samples and calibrated sensor extrinsics; it does not prescribe ADC timing, muxing, sensor power sequencing, or wall-sensor sampling policy.

System-level timing assumptions used by this spec:

* Non-IMU sensors are sampled once per tick at fixed phases.
* The IMU is sampled once per tick but has asynchronous phase relative to the rest of the tick schedule.
* Some sampled sensors may be irrelevant in some operating contexts, for example wall sensors on open floor. Relevance is handled by gating, skipping the corresponding update, or assigning very large covariance; the model should not require changing the sampling loop.
* The only IMU software settings this specification comments on are LPF/HPF choices.

Common measurement functions:

```text
h_imu(state_vec, imu_sample, imu_extrinsics, params)
h_wall(state_vec, wall_sample, wall_extrinsics, wall_hypothesis, params)
h_roll(state_vec, drive_sample, params)     // optional pseudo-measurement
h_stationary_bias(...)                      // external, not UKF state
```

A complete measurement container for one tick may hold:

\[
measurement\_vec=\begin{bmatrix}
z_{gyro} & z_{Af} & z_{Ar} & z_{wall,L} & z_{wall,R} & z_{wall,FL} & z_{wall,FR}
\end{bmatrix}^T
\]

but the UKF update should consume only the entries that are valid for the current wall hypothesis and gating decision.

## 17. IMU measurement model

### 17.1 Gyro

External bias correction:

\[
z_{gyro}=gyroYawMeas-gyroBiasYawExt
\]

Measurement function:

\[
h_{gyro}=yawRate
\]

Measurement residual:

\[
r_{gyro}=z_{gyro}-h_{gyro}
\]

All gyro sign corrections must already be done in the hardware layer.

### 17.2 Accelerometer

For each sigma point, recompute or reuse the plant auxiliary values:

\[
Af,\quad Ar,\quad yawAccel
\]

Let the IMU location in body coordinates be:

\[
(r_{imu},f_{imu})
\]

The planar acceleration at the IMU origin, expressed in body forward/right components, is:

\[
a_{imu,f}=Af-yawAccel\,r_{imu}-yawRate^2f_{imu}
\]

\[
a_{imu,r}=Ar+yawAccel\,f_{imu}-yawRate^2r_{imu}
\]

If the hardware layer outputs canonical body-frame accelerometer channels, use:

\[
h_{Af}=a_{imu,f}+accelBiasFCal
\]

\[
h_{Ar}=a_{imu,r}+accelBiasRCal
\]

If small in-plane IMU axis misalignment is intentionally represented in the estimator rather than absorbed by hardware-layer calibration, define unit vectors for the two reported planar accelerometer channels:

\[
\hat{a}_{F}=(a_{F,f},a_{F,r}),\quad \hat{a}_{R}=(a_{R,f},a_{R,r})
\]

with both vectors expressed in body-frame forward/right coordinates. Then use:

\[
h_{Af}=a_{F,f}a_{imu,f}+a_{F,r}a_{imu,r}+accelBiasFCal
\]

\[
h_{Ar}=a_{R,f}a_{imu,f}+a_{R,r}a_{imu,r}+accelBiasRCal
\]

No analogous planar heading unit vector is normally required for the yaw gyro: under the planar model, the yaw-rate measurement is the component about the vertical axis after hardware-layer sign correction. If gyro-axis misalignment is material, calibrate its scale/projection outside this 9-state UKF rather than adding orientation states.

So:

\[
\begin{bmatrix}
z_{Af}\\z_{Ar}
\end{bmatrix}
=
\begin{bmatrix}
h_{Af}\\h_{Ar}
\end{bmatrix}
+noise
\]

For the UKF acceleration measurement, use the low-pass path appropriate to the HAODR configuration. Use high-pass or slope-filtered IMU outputs only for event detection, such as impact or jerk gating, not as the primary acceleration measurement for the planar dynamics.

Inflate accelerometer measurement covariance when any of these are high:

* `impactSuspect`,
* `groundUse`,
* `stickSlipIndex`,
* severe vertical acceleration deviation,
* high jerk,
* known wall touch or springback event.

Do not force a planar acceleration model to explain pitch/roll/vertical impulses.

## 18. Encoder treatment and rolling pseudo-measurement

The encoders are not ordinary pose measurements in this design. They are converted into drivetrain inputs before UKF prediction.

Use:

\[
\hat{wheelRate}_L,\quad \hat{wheelRate}_R
\]

inside the plant for:

* motor back-EMF;
* command/rate/phase torque mapping;
* slip velocity;
* gear-ripple phase;
* drive-force feedforward;
* contact-utilization scheduling.

This avoids the incorrect assumption that drivetrain motion equals body motion. The encoders directly observe wheel-bank motion; they do not directly observe `Vf`, `Vr`, or `yawRate` when tires are scrubbing or slipping.

### 18.1 Optional rolling pseudo-measurement

In benign rolling operation only, use:

\[
z_{roll}=\begin{bmatrix}0\\0\end{bmatrix}
\]

\[
h_{roll,L}=r_w\hat{wheelRate}_L-\left(Vf+\frac{b}{2}yawRate\right)
\]

\[
h_{roll,R}=r_w\hat{wheelRate}_R-\left(Vf-\frac{b}{2}yawRate\right)
\]

Define the regime part of the covariance:

\[
R_{roll,regime}=R_{rollBase}
+k_{roll,util}util_{max}^4
+k_{roll,inPlace}inPlaceBlend
+k_{roll,ss}stickSlipIndex
+k_{roll,lowSpeed}\left(\frac{v_{low}}{\sqrt{Vf^2+Vr^2+v_E^2}}\right)^2
\]

Add the encoder wheel-rate contribution using the same \(R_\omega\) convention as the prediction model:

\[
R_{roll,total}=R_{roll,regime}+H_{\omega,roll}R_\omega H_{\omega,roll}^T
\]

with:

\[
H_{\omega,roll}=r_w
\begin{bmatrix}
1 & 0\\
0 & 1
\end{bmatrix}
\]

For independent encoder uncertainty:

\[
R_{roll,total}=R_{roll,regime}+
\begin{bmatrix}
r_w^2\sigma^2_{wheelRate,L} & 0\\
0 & r_w^2\sigma^2_{wheelRate,R}
\end{bmatrix}
\]

The rolling pseudo-measurement is:

```text
strong in mid-speed, low-utilization rolling motion
weak at low speed
weak during launch
disabled or extremely weak during in-place turns
disabled or extremely weak during high-speed stick-slip arcs
disabled when encoderSampleValid == false
```

Do not infer yaw rate from encoder differential during in-place turns. That assumes rolling contact, which is exactly false in scrubbed in-place turns.

---

## 19. Wall-sensor measurement model

Each wall sensor \(s\) has calibrated body-frame extrinsics:

\[
(r_s,f_s),\quad \hat{u}_s=(u_{f,s},u_{r,s})
\]

where \((r_s,f_s)\) is the sensor origin and \(\hat{u}_s\) is the sensor look direction expressed as a body-frame unit vector in forward/right components. This replaces scalar `sensorYaw_s` in the model spec; storing the look direction directly avoids angle-wrap ambiguity and matches the project extrinsics representation.

Sensor origin in global coordinates:

\[
sensorX_s=Px+f_s\sin(heading)+r_s\cos(heading)
\]

\[
sensorY_s=Py+f_s\cos(heading)-r_s\sin(heading)
\]

Sensor look vector in global coordinates:

\[
u_{x,s}=u_{f,s}\sin(heading)+u_{r,s}\cos(heading)
\]

\[
u_{y,s}=u_{f,s}\cos(heading)-u_{r,s}\sin(heading)
\]

Cast a small bundle of rays through the calibrated emitter/receiver angular support around \((u_{x,s},u_{y,s})\). For each ray candidate \(q\):

\[
d_{s,q}=rayDistanceToKnownWall(sensorPose_s,q)
\]

Use a soft-min so wall-edge and corner transitions are continuous:

\[
d_{eff,s}=-\frac{1}{\beta_s}\log\left(\sum_q w_{s,q}\exp(-\beta_sd_{s,q})\right)
\]

If the sensor pipeline outputs distance:

\[
h_{wall,s}=d_{eff,s}
\]

If using raw or log-amplified values directly, use fitted response curves. For log-amp front sensors:

\[
h_{frontRaw,s}=a_s-b_s\log(d_{eff,s}+d0_s)+c_s\,incidenceTerm_s
\]

For side IR sensors:

\[
h_{sideRaw,s}=a_s+\frac{b_s}{(d_{eff,s}+d0_s)^{p_s}}+c_s\,incidenceTerm_s
\]

Wall sensors may still be sampled when not relevant to the current mode or environment. Relevance is handled here: use wall measurements only when there is a valid wall hypothesis. If no wall should be visible or the raycast geometry is ambiguous, skip the update or set \(R\) extremely large.

Wall measurement covariance should increase when:

* the sensor is near a wall edge or post,
* the expected incidence angle is poor,
* the robot is in high-speed stick-slip,
* impact or springback is suspected,
* the innovation fails a chi-square gate.

## 20. External stationary gyro-bias estimator

Maintain `gyroBiasYawExt` outside the UKF.

During confirmed stationary intervals:

\[
yawRate=0
\]

so:

\[
gyroBiasYawExt\approx mean(gyroYawMeas)
\]

A covariance-aware scalar update is:

\[
\bar{g}=\frac{1}{N_{samples}}\sum_{k=1}^{N_{samples}}gyroYawMeas_k
\]

\[
K_b=\frac{P_b^-}{P_b^-+R_{\bar{g}}}
\]

\[
gyroBiasYawExt^+=gyroBiasYawExt^-+K_b(\bar{g}-gyroBiasYawExt^-)
\]

\[
P_b^+=(1-K_b)P_b^-
\]

Stationary detection should require:

\[
|gyroYawMeas-gyroBiasYawExt|<gyroStationaryThreshold
\]

\[
|\dot{N}_L|,|\dot{N}_R|<encoderStationaryThreshold
\]

\[
Af,Ar,Az\ \text{stable}
\]

Do not update external bias during tire springback, in-place scrub, wall touch, launch threshold events, or any impact-suspect interval.

If the external bias uncertainty is non-negligible, add a heading covariance term over intervals without reliable heading correction:

\[
P_{heading,heading}\leftarrow P_{heading,heading}+
\sigma_b^2\left[(t+\Delta t)^2-t^2\right]
\]

where \(t\) is time since the last reliable heading or bias correction. If logs show \(\sigma_b\) is negligible over the run length, this term will be negligible.

---

## 21. Practical implementation sketch

Prediction API sketch:

```cpp
void StateEstimator::predictFromTick(const EstimatorTick& tick)
{
    const double deltaTime = seconds(tick.timestamp - lastPredictionTimestamp_);
    lastPredictionTimestamp_ = tick.timestamp;

    state_vec_ = integratePlant(
        state_vec_,
        tick.drive_sample,
        params_,
        deltaTime);
}
```

Pure plant-integration helper:

```cpp
StateVec integratePlant(
    const StateVec& state_vec,
    const DriveSample& drive_sample,
    const Params& params,
    double deltaTime)
{
    // 1. Use validated wheel rates and wheelRateCov from DriveSample.
    // 2. If encoderSampleValid is false, select an explicit degraded prediction path.
    // 3. Compute nominal bank torque algebraically from command, wheel rate,
    //    and fitted torque-map parameters.
    // 4. Compute contact-patch slip velocities for each sigma point.
    // 5. Compute normal loads, tire forces, utilization, and regime scalars.
    // 6. Compute AfNom, ArNom, yawAccelNom.
    // 7. Add DeltaAf, DeltaAr, DeltaYawAccel.
    // 8. Propagate residual and encoder-input covariance by Section 15.4.
    // 9. Integrate planar body dynamics over the locally derived deltaTime.
    // 10. Wrap heading.
}
```

`deltaTime` is therefore an internal integration scalar derived from the tick timestamp. It is not carried as an external field in `drive_sample`.

Auxiliary outputs from the plant should include:

```text
AfNom, ArNom, yawAccelNom,
Af, Ar, yawAccel,
utilMax,
inPlaceBlend,
rollingBlend,
stickSlipIndex,
groundUse,
impactSuspect,
driveSaturationIndex,
driveAuthorityUncertainty
```

These are not UKF states. They are deterministic outputs used by measurement functions and Q/R scheduling.

---

## 22. Calibration sequence

1. **Coordinate/sign validation**  
   Verify that hardware-layer IMU signs match the project contract. Do not fix signs in the UKF.

2. **Encoder scale, timing, and validation**  
   Confirm actual count convention, gear ratio sign, left/right polarity, timestamping, wheel radius, sample-valid logic, and fault logging. Validate that missed edges or illegal transitions are reported as acquisition faults rather than absorbed into nominal covariance.

3. **Gyro external bias stability**  
   Fit `gyroBiasYawExt` and covariance from stationary logs. Confirm the observed 15-minute stability claim over temperature, battery state, and run duration.

4. **Motor/driver algebra**  
   Fit command normalization, the command-to-bank-torque map, driver/current-limit behavior, and wiring/thermal effects. Do not require measured `Vbat`.

5. **Launch and straight rolling**  
   Fit `Tlaunch`, constant rolling/friction losses, and gear ripple. Keep viscous straight drag at zero until logs prove otherwise.

6. **In-place turn scrub**  
   Fit `yawBreakawayMoment`, `yawKineticMoment`, and the in-place process-noise schedule from repeated angle turns.

7. **Mid-speed rolling arcs**  
   Fit low-utilization longitudinal/lateral stiffness and track geometry. Tune optional rolling pseudo-measurement covariance.

8. **High-speed stick-slip arcs**  
   Fit \(\mu_{peak}\), \(\mu_{slide}\), \(v_{Stribeck}\), and process-noise scheduling from held-out arcs at 1–2.5 m/s and practical turn radii.

9. **Ground-strike envelope**  
   Fit forward/reverse acceleration limits from full-bore acceleration and braking logs. Treat in-place strike-like events as impacts unless logs prove deterministic coupling.

10. **Wall sensors**  
    Fit extrinsics, raw response curves, incidence terms, and wall-edge covariance gates separately for side and front sensors.

---

## 23. Validation tests

Use held-out logs, not only fit logs. The residual and encoder-input covariance checks are listed in Section 15.6. The table below focuses on whole-vehicle behavior.

| Test | Expected behavior |
|---|---|
| Stationary 20 min | external gyro bias remains stable; UKF heading covariance does not become overconfident |
| Straight launch | launch threshold and acceleration envelope prevent unrealistic body acceleration |
| Constant-speed straight | residuals decay; no artificial viscous drag required |
| In-place turn | yaw-rate follows gyro; rolling pseudo-measurement disabled; `DeltaYawAccel` absorbs scrub mismatch |
| Mid-speed rolling arc | low residuals; rolling pseudo-measurement helpful but not dominant; rolling covariance includes encoder-rate covariance |
| High-speed arc | `stickSlipIndex` high; residuals active; filter does not diverge on slip events |
| Wall-edge pass | wall prediction transitions continuously via soft-min ray model |
| Floor strike / impact | accelerometer R inflates; planar state not forced to absorb vertical impulse |
| Residual covariance replay | increasing residual steady-state sigmas increases motion-state covariance and residual-to-motion cross-covariance |
| Encoder timing/quantization replay | increasing valid-sample wheel-rate covariance increases prediction covariance through the plant |
| Encoder fault injection | illegal transition, implausible count delta, or timestamp discontinuity disables rolling pseudo-measurement, activates degraded prediction, and logs the fault reason |

---

## 24. Glossary of symbols and units

Unless stated otherwise, all quantities use SI units: meters, kilograms, seconds, amperes, volts, ohms, newtons, and newton-meters. Radians are dimensionless for dimensional analysis, but this document keeps `rad`, `rad/s`, and `rad/s²` in the units column for clarity. Encoder `counts`, command units, and sample indexes are discrete/dimensionless units.

Subscript conventions:

| Subscript | Meaning |
|---|---|
| `L`, `R` | left and right wheel bank / motor side |
| `f`, `r` | body-frame forward and right components |
| `i` | contact patch index |
| `j` | wheel-bank or motor-side index, usually `L` or `R` |
| `s` | wall-sensor index |
| `q` | ray candidate within a wall-sensor ray bundle |
| `imu` | IMU location or quantity at the IMU origin |
| `nom`, `raw`, `clip` | nominal, unclipped/raw, and clipped values |
| `E` | smoothing scale or small regularization value |

### 24.1 State, kinematics, and dynamics

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `state_vec` | mixed | UKF state vector. |
| `measurement_vec` | mixed | Measurement container/vector. Entries may be gated or skipped. |
| `Px`, `Py` | m | Global position in maze coordinates. |
| `heading` | rad | Global heading; zero faces +Y; clockwise positive. |
| `Vf`, `Vr` | m/s | Body-frame forward and rightward velocity. |
| `yawRate` | rad/s | Body yaw rate; clockwise positive. |
| `Af`, `Ar` | m/s² | Actual forward and rightward acceleration used in propagation. |
| `yawAccel` | rad/s² | Actual yaw acceleration used in propagation. |
| `AfNom`, `ArNom`, `yawAccelNom` | m/s², m/s², rad/s² | Nominal plant accelerations before residual-state correction. |
| `AfRaw`, `ArRaw`, `yawAccelRaw` | m/s², m/s², rad/s² | Raw nominal accelerations before clipping or final limiting. |
| `AfClip` | m/s² | Forward acceleration after ground-strike / acceleration-envelope clipping. |
| `DeltaAf`, `DeltaAr` | m/s² | Colored forward and lateral acceleration residual states. |
| `DeltaYawAccel` | rad/s² | Colored yaw-acceleration residual state. |
| `tau_f`, `tau_r` | s | Time constants for `DeltaAf` and `DeltaAr` decay. |
| `tau_y` | s | Time constant for `DeltaYawAccel` decay. |
| `phi_f`, `phi_r`, `phi_y` | dimensionless | Exact OU residual decay factors, `exp(-Delta t / tau_*)`. |
| `eta_f`, `eta_r` | m/s² | Discrete OU process samples added to `DeltaAf` and `DeltaAr`. |
| `eta_y` | rad/s² | Discrete OU process sample added to `DeltaYawAccel`. |
| `Q_DeltaAf`, `Q_DeltaAr` | (m/s²)² | Exact discrete OU process variances for translational residual states. |
| `Q_DeltaYawAccel` | (rad/s²)² | Exact discrete OU process variance for the yaw-acceleration residual state. |
| `q_c,q` | residual²/s | Equivalent continuous-time driving-noise spectral density for residual `q`; documented for reference, not the scheduled tuning parameter. |
| `w_f`, `w_r` | m/s³ | Continuous-time disturbance notation used only in the formal OU interpretation; implementation uses the exact discrete OU samples `eta_f` and `eta_r`. |
| `w_y` | rad/s³ | Continuous-time disturbance notation used only in the formal OU interpretation; implementation uses the exact discrete OU sample `eta_y`. |
| `m_eff_f`, `m_eff_r` | kg | Effective forward and lateral mass used by the planar force model. |
| `I_z` | kg·m² | Effective yaw inertia about the vertical axis. |
| `yawMomentRaw` | N·m | Raw clockwise-positive yaw moment from contact forces. |
| `yawMomentBreakaway` | N·m | Yaw moment after smooth static breakaway deadzone. |
| `yawMoment` | N·m | Final yaw moment used to compute `yawAccelRaw`. |
| `yawBreakawayMoment` | N·m | Static in-place yaw breakaway threshold. |
| `yawMoment_E` | N·m | Smoothing scale for the yaw breakaway deadzone. |
| `yawKineticMoment` | N·m | Kinetic scrub-resistance yaw moment. |
| `yawRate_E` | rad/s | Smoothing scale for yaw-rate sign approximation. |
| `AfForwardLimit`, `AfReverseLimit` | m/s² | Smooth acceleration-envelope limits for forward and reverse acceleration. |
| `scale_f` | dimensionless | Scaling applied to forward contact forces after forward-acceleration clipping. |

### 24.2 Control, timing, encoder, and drivetrain symbols

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `CL`, `CR` | command units / PWM units | Signed left and right motor commands before normalization. |
| `u_L`, `u_R`, `u_j` | dimensionless | Normalized motor commands in `[-1, 1]`. |
| `C_E` | command units | Smoothing scale used in `opposedDrive`. |
| `C_turnE` | command units | Smoothing scale used in `turnDemand`. |
| `dt`, `Delta t` | s | Derived estimator/model timestep computed from `timestamp` and estimator state; not a `drive_sample` field. |
| `timestamp` | s | Effective sample timestamp in the estimator time base; the source of truth for deriving `Delta t`. |
| `VbusEff` | V | Optional calibrated effective motor-bus voltage prior used only inside the nominal drive model; not a measured vehicle input. |
| `sigmaVbusEff` | V | Standard deviation of the calibrated effective motor-bus voltage prior, used only to form `driveAuthorityUncertainty`. |
| `V_E` | V | Small positive voltage regularizer used in `driveAuthorityUncertainty`. |
| `G` | dimensionless | Gear ratio, motor revolutions per wheel revolution. |
| `r_w` | m | Wheel radius. For 25 mm wheel diameter, first-pass radius is 0.0125 m. |
| `b` | m | Track width. |
| `N_L`, `N_R` | counts | Left/right encoder counts when used as absolute or accumulated counts. |
| `Delta N_j` | counts | Encoder count increment on side `j` over `Delta t`. |
| `countsPerMotorRev` | counts/rev | Encoder counts per motor revolution under the firmware count convention. |
| `countsPerWheelRev` | counts/rev | Encoder counts per wheel-bank revolution. |
| `wheelCircumference` | m | Wheel rolling circumference. |
| `distancePerCount` | m/count | Wheel-surface distance represented by one encoder count under rolling geometry. |
| `wheelRateL`, `wheelRateR`, `hat{wheelRate}_j` | rad/s | Validated encoder-derived wheel-bank angular rate. |
| `encoderSampleValid` | Boolean | Encoder acquisition validation result for the tick. False activates degraded prediction and disables rolling pseudo-measurement. |
| `encoderFaultFlags` | bitset / enum | Logged reason for invalid encoder sample, such as illegal transition, implausible delta, timestamp discontinuity, or counter discontinuity. |
| `I_encoderFault` | dimensionless | Binary process-noise scheduling indicator derived from `encoderSampleValid`. |
| `wheelAccelL`, `wheelAccelR`, `hat{wheelAccel}_j` | rad/s² | Deprecated/excluded encoder-derived wheel-bank angular acceleration; useful for diagnostics/offline identification only, not part of `drive_sample`. |
| `wheelRate_E` | rad/s | Smoothing scale for wheel-rate sign approximation. |
| `phi_m`, `phi_w` | rad | Motor shaft angle and wheel-bank angle. |
| `motorPhase_j` | rad | Motor/gear phase used by ripple fits. |
| `sigma^2_DeltaN` | counts² | Encoder count-increment quantization variance. |
| `sigma^2_wheelRate,j` | (rad/s)² | Valid-sample wheel-rate input variance for side `j`, from quantization plus timing/phase uncertainty. |
| `R_omega` | (rad/s)² | 2x2 covariance matrix for `[wheelRateL, wheelRateR]^T`. |
| `sigma_LR` | (rad/s)² | Optional covariance between left/right wheel-rate errors from shared timing or sample-phase uncertainty. |
| `sigma^2_timestamp,j`, `sigma^2_wheelRate,timing,j` | (rad/s)² | Wheel-rate-equivalent variance due to timestamp/sample-phase uncertainty. |
| `sigma^2_missedEdge,j` | (rad/s)² | Must be zero for valid samples. Missed edges are acquisition faults, not nominal covariance. |
| `sigma^2_filterDelay,j` | (rad/s)² | Must be zero unless firmware actually applies an encoder-rate filter. |

### 24.3 Motor, driver, and torque algebra

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `motorRate_j` | rad/s | Motor shaft angular rate on side `j`. |
| `motorRate_E` | rad/s | Smoothing scale for motor-rate sign approximation. |
| `VdriveNom_j` | V | Optional voltage-equivalent signed motor drive voltage in a prior model, based on calibrated `VbusEff` rather than a measurement. |
| `T_cmdMap,j(u_j,motorRate_j,motorPhase_j)` | N·m | Fitted command/rate/phase map producing raw wheel-bank torque for side `j`. |
| `R_m` | ohm | Motor terminal resistance. |
| `L_m` | H | Motor winding inductance used for electrical-time-constant estimates. |
| `tau_e` | s | Motor electrical time constant, `L_m/R_m`. |
| `R_drv` | ohm | Effective H-bridge on-resistance in the active current path. |
| `R_wire` | ohm | Effective wiring/contact resistance. |
| `R_ILIM` | kΩ in formula, otherwise ohm | DRV8871 current-limit resistor. |
| `V_ILIM` | kV in DRV8871 equation convention | DRV8871 current-limit constant; `V_ILIM / R_ILIM,kΩ` yields amperes. |
| `Iraw_j` | A | Unclipped algebraic motor current estimate. |
| `I_j` | A | Smoothly current-limited motor current. |
| `Itrip` | A | DRV8871 current-regulation threshold. |
| `I_0` | A | Motor no-load current used as a datasheet prior. |
| `K_t` | N·m/A | Motor torque constant. |
| `K_e` | V·s/rad | Motor back-EMF constant. Numerically equal to `K_t` in SI when radians are dimensionless. |
| `M_R` | N·m | Motor friction torque from motor datasheet prior or equivalent fitted term. |
| `J` | kg·m², or converted from g·cm² | Rotor inertia from datasheet priors; convert to SI before use in equations. |
| `Tmotor_j` | N·m | Motor shaft torque on side `j`. |
| `eta_drive` | dimensionless | Geartrain/drivetrain efficiency factor. |
| `TbankRaw_j` | N·m | Raw wheel-bank torque before low-speed breakaway and losses. |
| `Tbank_j` | N·m | Final usable wheel-bank torque after breakaway and loss terms. |
| `Tlaunch_j` | N·m | Smooth low-speed launch/breakaway torque threshold. |
| `T_E` | N·m | Torque smoothing scale for launch deadzone. |
| `Troll_j` | N·m | Rolling/friction torque loss for side `j`. |
| `Tvisc_j` | N·m·s/rad | Viscous wheel-bank torque coefficient. Initially zero unless logs prove otherwise. |
| `T_ripple_j(motorPhase_j)` | N·m | Gear/motor ripple torque term as a function of motor phase. |
| `slow_j` | dimensionless | Low-speed weighting for launch threshold. |
| `v_static` | m/s | Wheel-surface speed scale for low-speed launch weighting. |
| `J_eq` | kg·m² | Effective wheel-bank reflected inertia. |
| `Fdrive_j` | N | Side-level quasi-static drive-force feedforward from bank torque, `Tbank_j/r_w`. |

### 24.4 Contact geometry, slip, normal load, and tire forces

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `r_i`, `f_i` | m | Contact-patch body-frame location, right and forward from vehicle center. |
| `r_s`, `f_s` | m | Wall-sensor body-frame origin, right and forward from vehicle center. |
| `r_imu`, `f_imu` | m | IMU body-frame origin, right and forward from vehicle center. |
| `hat{u}_s = (u_f,s, u_r,s)` | dimensionless | Wall-sensor body-frame unit look vector. |
| `u_x,s`, `u_y,s` | dimensionless | Wall-sensor look vector transformed to global coordinates. |
| `hat{a}_F`, `hat{a}_R` | dimensionless | Optional IMU accelerometer-channel unit vectors in body forward/right coordinates. |
| `contactForwardVelocity_i` | m/s | Body forward velocity at contact patch `i`. |
| `contactRightVelocity_i` | m/s | Body rightward velocity at contact patch `i`. |
| `surfaceForwardVelocity_i` | m/s | Wheel surface velocity at contact patch `i`. |
| `slipForwardVelocity_i` | m/s | Longitudinal slip velocity at contact patch `i`. |
| `slipRightVelocity_i` | m/s | Lateral/rightward slip velocity at contact patch `i`, sign chosen in force-producing direction. |
| `slipForward_i`, `slipRight_i` | m/s | Compact aliases used in exclusion rationale for the corresponding slip velocities. |
| `N_i` | N | Normal load at contact patch `i`; not encoder count. |
| `N_static,i` | N | Static normal-load contribution. |
| `N_fan,i` | N | Fan/downforce normal-load contribution. |
| `N_longTransfer,i` | N | Longitudinal load-transfer contribution. |
| `N_latTransfer,i` | N | Lateral load-transfer contribution. |
| `N_min` | N | Smooth lower bound for normal load. |
| `w_fan,i` | dimensionless | Fan/downforce distribution weight for contact patch `i`. |
| `F_fan(fanCommand; fanCalParams)` | N | Calibrated fan/downforce function using fan command and calibration parameters; no `Vbat` measurement is available. |
| `fanCommand` | command units | Fan command supplied to the downforce model. |
| `w_{i\mid j}` | dimensionless | Normal-load weighting from wheel bank `j` to contact patch `i`. |
| `FdriveReq_i` | N | Contact-patch drive-force request from side drive feedforward. |
| `F_f,req,i`, `F_r,req,i` | N | Requested forward and rightward contact forces before combined-slip saturation. |
| `F_f,i`, `F_r,i` | N | Final forward and rightward contact forces after saturation. |
| `K_f,i`, `K_r,i` | N·s/m | Longitudinal and lateral slip-velocity stiffness coefficients. |
| `slipSpeed_i` | m/s | Combined slip-speed magnitude. |
| `v_E` | m/s | Slip-speed regularization value. |
| `mu_i`, `muPeak_i`, `muSlide_i` | dimensionless | Effective, peak, and sliding friction coefficients. |
| `vStribeck_i` | m/s | Stribeck transition speed for contact patch `i`. |
| `F_limit_i` | N | Friction-envelope force limit at contact patch `i`. |
| `lambda_f`, `lambda_r` | dimensionless | Forward/right force-share factors in combined-slip utilization. |
| `util_i`, `utilMax` | dimensionless | Contact utilization for patch `i` and maximum utilization over all patches. |
| `forceScale_i` | dimensionless | Smooth saturation scale applied to requested contact force. |
| `epsilon` | context-dependent | Small positive regularizer. It must have the same units as the denominator or expression it regularizes. |

### 24.5 Regime scalars, covariance scheduling, and noise terms

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `vBody` | m/s | Magnitude of planar body velocity. |
| `opposedDrive` | dimensionless | Smooth indicator for opposite-side drive command. |
| `turnDemand` | dimensionless | Smooth indicator for turn command magnitude. |
| `inPlaceBlend` | dimensionless | Continuous weighting for in-place scrub/breakaway behavior. |
| `v_inPlace` | m/s | Speed scale for in-place blend decay. |
| `rollingBlend` | dimensionless | Continuous weighting for rolling-dominant operation. |
| `util_roll` | dimensionless | Contact-utilization scale below which rolling pseudo-measurement can become strong. |
| `v_rollLow`, `v_rollHigh` | m/s | Body-speed transition bounds for rolling confidence. |
| `stickSlipIndex` | dimensionless | Smooth indicator for high-speed, high-utilization stick-slip-prone operation. |
| `util_ssLow`, `util_ssHigh` | dimensionless | Utilization transition bounds for stick-slip scheduling. |
| `v_ssLow`, `v_ssHigh` | m/s | Speed transition bounds for stick-slip scheduling. |
| `latDemand` | dimensionless | Lateral acceleration demand normalized by `g`. |
| `g` | m/s² | Gravitational acceleration magnitude used for normalization and vertical-acceleration checks. |
| `latLow`, `latHigh` | dimensionless | Transition bounds for lateral-demand scheduling. |
| `driveSaturationIndex` | dimensionless | Drive-saturation/current-limit proxy, either from the voltage-equivalent prior or the fitted torque map. |
| `driveAuthorityUncertainty` | dimensionless | Uncertainty in drive authority from unmeasured bus voltage, torque-map residuals, thermal effects, and run-condition uncertainty. |
| `groundUse` | dimensionless | Fractional use of acceleration-envelope clipping. |
| `impactSuspect` | dimensionless | Impact/springback suspicion scalar used for covariance inflation. |
| `Az` | m/s² | Vertical acceleration measurement or derived vertical acceleration magnitude. |
| `zLow`, `zHigh` | m/s² | Vertical-acceleration transition bounds for impact detection. |
| `jerk_f`, `jerk_r` | m/s³ | Forward and rightward jerk estimates. |
| `jLow`, `jHigh` | m/s³ | Jerk transition bounds for impact detection. |
| `sigmaDeltaAfSs`, `sigmaDeltaArSs` | m/s² | Scheduled steady-state standard deviations of the translational residual states; not per-tick noise and not spectral density. |
| `sigmaDeltaYawAccelSs` | rad/s² | Scheduled steady-state standard deviation of the yaw-acceleration residual state; not per-tick noise and not spectral density. |
| `sigmaAfBaseSs`, `sigmaArBaseSs` | m/s² | Base steady-state standard deviations for translational residual states. |
| `sigmaYawBaseSs` | rad/s² | Base steady-state standard deviation for the yaw-acceleration residual state. |
| `sigma_b` | rad/s | External yaw-gyro bias uncertainty. |
| `k_*` | context-dependent | Scheduling gains; units must make each summed covariance/noise expression dimensionally consistent. |
| `P` | state-units product | UKF state covariance matrix. |
| `Q`, `Q_x` | state-units product | UKF process-noise covariance matrix. Residual and encoder-input terms must be mapped into the full state covariance, not only appended to residual diagonals. |
| `Q_eta` | residual-state-units product | Diagonal covariance of residual OU innovations `[eta_f, eta_r, eta_y]^T`. |
| `G_eta` | state/residual units | Local sensitivity mapping residual OU innovations into the full propagated state. |
| `J_omega` | state/(rad/s) | Local sensitivity mapping encoder wheel-rate input perturbations into the full propagated state. |
| `wrapStateDelta(...)` | mixed | Difference operation that wraps heading before forming covariance sensitivities. |
| `R` | measurement-units product | Measurement covariance matrix. Do not confuse with resistance symbols such as `R_m`. |
| `R_roll`, `R_rollBase` | (m/s)² | Rolling pseudo-measurement covariance and its base value because `h_roll` has velocity units. |
| `R_roll,regime` | (m/s)² | Rolling pseudo-measurement covariance from regime scheduling before encoder-rate contribution. |
| `R_roll,total` | (m/s)² | Final rolling pseudo-measurement covariance including `H_omega,roll R_omega H_omega,roll^T`. |
| `H_omega,roll` | m | Sensitivity from wheel angular rate to rolling pseudo-measurement velocity residual. |

### 24.6 IMU, wall-sensor, and external-bias measurement symbols

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `gyroYawMeas` | rad/s | Hardware-layer sign-corrected yaw gyro sample. |
| `gyroBiasYawExt` | rad/s | External stationary-calibrated yaw-gyro bias estimate. |
| `z_gyro`, `h_gyro`, `r_gyro` | rad/s | Bias-corrected gyro measurement, predicted gyro measurement, and gyro residual. |
| `gyroStationaryThreshold` | rad/s | Threshold for stationary detection using yaw gyro. |
| `encoderStationaryThreshold` | counts/s or rad/s | Threshold for stationary detection using encoder rate, depending on implementation point. |
| `z_Af`, `z_Ar` | m/s² | Forward and rightward accelerometer measurements. |
| `h_Af`, `h_Ar` | m/s² | Predicted forward and rightward accelerometer measurements. |
| `a_imu,f`, `a_imu,r` | m/s² | Predicted acceleration at the IMU origin in body forward/right components. |
| `accelBiasFCal`, `accelBiasRCal` | m/s² | Externally calibrated accelerometer bias constants. |
| `bar{g}` | rad/s | Mean stationary yaw gyro sample used by the external bias estimator. |
| `N_samples` | dimensionless | Number of samples in the stationary gyro-bias average. |
| `P_b` | (rad/s)² | External yaw-gyro bias-estimate variance. |
| `R_bar{g}` | (rad/s)² | Variance of the averaged stationary gyro-bias measurement. |
| `K_b` | dimensionless | Scalar Kalman gain for the external bias estimator. |
| `delta b` | rad/s | Residual yaw-gyro bias error. |
| `T_uncorrected` | s | Time interval without reliable heading or bias correction. |
| `t` | s | Elapsed time variable in covariance-growth equations. |
| `P_heading,heading` | rad² | Heading variance element of the UKF covariance matrix. |
| `sensorX_s`, `sensorY_s` | m | Global coordinates of wall sensor `s`. |
| `sensorPose_s` | mixed | Wall-sensor global raycast pose derived from `(sensorX_s, sensorY_s)` and the global look vector. |
| `rayDistanceToKnownWall(...)` | m | Raycast function returning distance to the active wall hypothesis. |
| `d_s,q` | m | Raycast distance for wall sensor `s` and ray candidate `q`. |
| `d_eff,s` | m | Soft-min effective wall distance for sensor `s`. |
| `beta_s` | 1/m | Soft-min sharpness for wall sensor `s`. |
| `w_s,q` | dimensionless | Weight for wall-sensor ray candidate `q`. |
| `h_wall,s` | m | Predicted wall distance when the sensor pipeline reports distance. |
| `h_frontRaw,s`, `h_sideRaw,s` | raw sensor units | Predicted raw/log-amplified wall-sensor reading. |
| `a_s`, `b_s`, `c_s`, `d0_s`, `p_s` | fitted units | Wall-sensor response-curve parameters. `d0_s` has units m; `p_s` is dimensionless; the others depend on raw output units. |
| `incidenceTerm_s` | dimensionless | Incidence-angle correction term for wall sensor `s`. |
| `z_roll` | m/s | Target vector for rolling pseudo-measurement, normally zero velocity residual. |
| `h_roll,L`, `h_roll,R` | m/s | Left/right rolling velocity residual predictions. |
| `v_low` | m/s | Low-speed covariance-inflation scale for rolling pseudo-measurement. |

### 24.7 Smooth helper functions and generic variables

| Symbol / function | Units | Meaning |
|---|---:|---|
| `normalize(CL/CR)` | dimensionless | Command normalization function producing `u_L` and `u_R`. |
| `sgnE(x, x_E)` | dimensionless | Smooth sign approximation. `x` and `x_E` must have the same units. |
| `satE(x, x_max)` | same as `x` | Smooth saturation. `x` and `x_max` must have the same units. |
| `softplusE(x, e)` | same as `x` | Smooth positive-part function. `x` and `e` must have the same units. |
| `deadzoneE(x, a, e)` | same as `x` | Smooth deadzone. `x`, `a`, and `e` must have the same units. |
| `smoothStepE(x, x0, x1)` | dimensionless | Smooth transition from 0 to 1. `x`, `x0`, and `x1` must have the same units. |
| `maxE(a, b, e)`, `minE(a, b, e)` | same as `a` and `b` | Smooth maximum/minimum. `a`, `b`, and `e` must have the same units. |
| `clipAsymE(x, x_min, x_max, e)` | same as `x` | Smooth asymmetric clipping. `x`, bounds, and `e` must have the same units. |
| `x`, `a`, `b`, `e`, `x0`, `x1`, `x_min`, `x_max` | context-dependent | Generic helper-function arguments. Units are constrained by each helper function. |

### 24.8 Excluded or deprecated symbols

These symbols appear only in exclusion rationale or migration notes; they are not active UKF states in the recommended design.

| Symbol / code name | Units | Meaning |
|---|---:|---|
| `Vbat` | V | Excluded from `drive_sample` because the vehicle does not measure battery / motor-bus voltage. |
| `wheelAccelL/R` | rad/s² | Excluded from `drive_sample` and nominal plant because encoder-derived acceleration is quantized/noisy at 1 kHz and drivetrain response is sub-tick. |
| `wheelRateL`, `wheelRateR` as states | rad/s | Excluded as latent UKF states; retained as encoder-derived drivetrain inputs. |
| `gyroBiasYaw` | rad/s | Excluded residual yaw-gyro bias UKF state; replaced by external `gyroBiasYawExt`. |
| `slipL`, `slipR` | m/s if slip velocity, dimensionless if slip ratio | Excluded main-UKF slip states. Algebraic slip velocities are used instead; slip-ratio states are rejected near zero speed. |
| motor-current states, such as `I_L`, `I_R` | A | Excluded because motor electrical dynamics are much faster than the 1 ms estimator tick. Algebraic current limiting is used. |
| accelerometer-bias states | m/s² | Excluded initially; calibrated externally and handled with measurement covariance inflation under impact/high-dynamic conditions. |
| pitch, roll, vertical position, vertical velocity | rad, rad, m, m/s | Excluded to keep a planar estimator; floor strikes and rocking are handled by clipping and covariance scheduling. |
| fan/downforce state | N or model-specific | Excluded; fan/downforce is an algebraic normal-load modifier with uncertainty in process noise. |
| wall/map states | m / geometry-dependent | Excluded; wall hypotheses are supplied by the mapper/controller. |
| `sensorYaw_s` | rad | Deprecated wall-sensor scalar heading-offset representation; replaced by body-frame look unit vector `hat{u}_s`. |

---

## 25. Consistency requirements

The following consistency requirements are part of this specification.

1. **Position and velocity transforms are dimensionally valid.** `sin(heading)` and `cos(heading)` are dimensionless, so `dPx/dt` and `dPy/dt` have units m/s.

2. **Body dynamics are dimensionally valid.** `Af` and `Ar` are m/s². The coupling terms `yawRate*Vr` and `yawRate*Vf` are also m/s² because radians are dimensionless.

3. **Yaw dynamics are dimensionally valid.** `yawMoment / I_z` gives rad/s² under the planar rigid-body convention.

4. **Encoder conversion is dimensionally valid.** `DeltaN / Delta t` gives counts/s; multiplying by `2*pi/(G*countsPerMotorRev)` gives rad/s.

5. **Encoder fault treatment is explicit.** Missed edges, illegal quadrature transitions, counter discontinuities, implausible count deltas, and timestamp discontinuities are acquisition faults. They are not nominal Gaussian covariance terms for validated samples.

6. **Encoder input covariance enters before nonlinear plant calculations.** `R_omega` must be propagated through motor algebra, slip velocity, tire force, utilization, regime scheduling, and rolling pseudo-measurement covariance.

7. **Residual process noise maps into the full state covariance.** Adding residual process variance only to `DeltaAf`, `DeltaAr`, and `DeltaYawAccel` diagonal entries after propagation is not compliant.

8. **Motor algebra is dimensionally valid.** `K_e*motorRate` gives volts, the current equation gives amperes, and `K_t*I_j` gives N·m.

9. **Slip-stiffness force requests are dimensionally valid.** `K_f,i` and `K_r,i` are N·s/m, so multiplying by slip velocity gives newtons.

10. **Combined-slip utilization is dimensionless.** Each force request is divided by a force limit; `lambda_f`, `lambda_r`, `mu`, `util_i`, and `forceScale_i` are dimensionless.

11. **Soft-min wall distance is dimensionally valid.** `beta_s` has units 1/m so `beta_s*d_s,q` is dimensionless and `d_eff,s` has units m.

12. **Wall-sensor extrinsics use body-frame unit vectors.** The active model uses `(r_s, f_s)` plus `hat{u}_s=(u_f,s,u_r,s)`, which matches the project extrinsics representation and avoids angle-wrap issues.

13. **IMU axis unit vectors are optional and dimensionless.** The default measurement model assumes hardware-layer canonical body-frame acceleration channels. Optional accelerometer-channel unit vectors only project the predicted body-frame acceleration into slightly misaligned reported axes. No planar heading unit vector is needed for the yaw gyro unless a separate calibration layer models gyro-axis projection.

14. **HAODR/raw-IMU scope is reflected.** Embedded SFLP/game-vector/gravity/bias outputs are excluded from the estimator path. The only IMU software settings discussed are LPF/HPF choices, and HPF/slope output is reserved for event gating rather than the primary acceleration measurement.

15. **Overloaded symbols are disambiguated.** `R` is measurement covariance, while `R_m`, `R_drv`, `R_wire`, and `R_ILIM` are resistances. `N_i` is normal load in newtons, while `DeltaN_j` is encoder count increment and `N_samples` is a sample count.

16. **Residual process-noise semantics are exact and timestep-independent.** The scheduled `sigmaDelta*Ss` terms are steady-state residual-state standard deviations. Discrete residual process variance is computed as `sigma^2 * (1 - phi^2)` with `phi = exp(-Delta t / tau)`. The scheduled sigmas are not per-tick standard deviations and not continuous-time spectral-density values; the equivalent `q_c = 2*sigma^2/tau` relationship is stated only for comparison.

17. **Timing contract is disambiguated.** `timestamp` is externally supplied and is the source of truth for propagation timing. `Delta t` / `dt` remains a mathematical/internal estimator variable derived from consecutive timestamps; it is intentionally not duplicated in `drive_sample`.

18. **Drive-sample contract is disambiguated.** `Vbat` and `wheelAccelL/R` are excluded from `drive_sample`. `VbusEff` is a calibrated parameter, and wheel acceleration is diagnostic/offline-identification data only.

19. **Process-noise schedule evaluation is specified.** The preferred convention is sigma-point-specific regime-scalar evaluation with scheduled values frozen over each propagation substep. If the implementation cannot support sigma-point-specific process noise, predicted-mean scalars with conservative inflation are required.

20. **Rolling pseudo-measurement covariance is not allowed to treat encoders as exact.** `R_roll,total` must include the `R_omega` contribution when the pseudo-measurement is enabled.

---

## 26. Summary

The recommended 9-state UKF is:

\[
\boxed{
state\_vec =
\begin{bmatrix}
Px & Py & heading & Vf & Vr & yawRate & \Delta Af & \Delta Ar & \Delta yawAccel
\end{bmatrix}^T
}
\]

The core architecture is:

\[
\text{encoders}\rightarrow\hat{wheelRate}_L,\hat{wheelRate}_R
\rightarrow\text{algebraic motor/contact plant}
\rightarrow\text{body-state UKF}
\]

not:

\[
\text{encoders}\rightarrow\text{wheel-rate UKF states}\rightarrow\text{pose}
\]

and not:

\[
\text{encoders}\rightarrow\text{hard rolling differential-drive constraint}
\]

This design is better matched to the application because the difficult regimes are not merely wheel-slip estimation problems. They are body-force prediction problems across launch, scrubbed in-place turns, rolling turns, high-speed stick-slip arcs, ground-strike limits, and wall-sensor correction. The residual acceleration states act directly on the body dynamics while the algebraic slip model preserves the essential contact physics without consuming state budget.

Estimator covariance consistency depends on two additional requirements: residual-driving noise must propagate into motion-state covariance and cross-covariances, and encoder wheel-rate covariance must enter the nonlinear plant before motor, slip, tire-force, utilization, and rolling pseudo-measurement calculations. Encoder acquisition faults are handled by validation, logging, degraded prediction, and process-noise inflation, not by a nominal missed-pulse covariance term.

---

## References

[^hardware]: `Hardware.md`, project hardware summary: Teensy 4.1, 1000 Hz control loop, IE2-1024 encoders, LSM6DSV16X IMU, IR wall sensors, DRV8871 drivers, Faulhaber 1717T006SR motors, 17:56 gearing, 25 mm wheels, and four solid-rubber wheels with shared pinion per bank.
[^physical]: `Physical effects and operating envelope.md`, project physical effects: in-place scrub and breakaway, straight launch threshold, near-zero straight viscous resistance, constant friction, arcing-turn stick-slip, ground strikes, drivetrain force-transfer time, tire springback, and operating envelope.
[^symbols]: `Symbology Glossary.md`, coordinate, naming, transform, and IMU sign-convention contract.
[^motor]: `EN_1717_SR_DFF.pdf`, Faulhaber 1717 SR motor datasheet, page 1 table for the 006 SR motor variant.
[^encoder]: `EN_IE2-1024_DFF.pdf`, Faulhaber IE2-1024 encoder datasheet, page 1 table and characteristics text.
[^driver]: `drv8871.pdf`, Texas Instruments DRV8871 datasheet: features, recommended operating conditions, electrical characteristics, current regulation, and H-bridge modes.
[^imu]: `lsm6dsv16x.pdf`, STMicroelectronics LSM6DSV16X datasheet: features, mechanical characteristics, HAODR and ODR/full-scale tables, timestamp registers, and UI-path LPF/HPF filtering registers. Embedded SFLP/game outputs are not used in the HAODR estimator path described here.
[^residual_covariance]: `02_residual_noise_covariance_propagation.md`, covariance requirement that residual-state process noise be propagated into the full state covariance rather than only the residual-state diagonal block.
[^encoder_uncertainty]: `03_encoder_input_uncertainty.md`, covariance requirement that encoder wheel-rate uncertainty enter the nonlinear plant before motor, slip, tire-force, utilization, regime-scheduling, and rolling pseudo-measurement calculations.
