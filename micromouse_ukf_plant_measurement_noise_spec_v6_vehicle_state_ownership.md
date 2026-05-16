# Micromouse UKF Plant, Measurement, and Noise Specification

**Document status:** rewritten target specification, contact-continuum revision.  
**Scope:** plant model, measurement models, stochastic/noise semantics, calibration, validation, host replay, and migration boundaries for the micromouse UKF.  
**Out of scope:** UKF sigma-point math core, square-root factorization, numerical linear algebra internals, low-level sensor acquisition, raw log-packet schema, and drive-layer motion-limit implementation. This document does define minimal plant/measurement/noise submodel ownership boundaries where needed to prevent parameter-bag implementations.

---

## 0. Executive stance

This specification defines the target estimator/plant model intended to replace deeply rooted problems in the current estimator/plant implementation. The current implementation is treated as **mature only in the sense that it has exposed and demonstrated failure modes**. Its infrastructure may be useful; its plant, state-vector, and control-authority semantics are not a behavioral baseline.

The target estimator is a single planar 9-state body UKF. Encoders provide drivetrain inputs. IMU and wall sensors provide measurements. The plant uses algebraic, per-contact **relative velocities** in SI units. It does not use speed-normalized slip ratio, slip angle, curvature, turn radius, instantaneous-center-of-rotation denominators, or maneuver-class thresholds.

Core stance:

```text
Strict: state order, units, signs, timing, measurement meaning, validity/fault behavior, covariance semantics.
Flexible: motor/driver torque, normal load, contact force, yaw loss, ground strike, wall response, noise schedules.
Forbidden: Bgz UKF state, traction-gated performance commands, speed-normalized slip math, runtime turn taxonomy.
```

Terms such as **scrub**, **stick-slip**, **in-place**, and **rolling** may appear in comments or human-facing test descriptions. They do not define estimator states, required telemetry, plant branches, command admissibility, or measurement-update modes.

---

## 1. Source assumptions

The project hardware assumptions are:

- Teensy 4.1, 1000 Hz control loop, and existing robust UKF-boundary input logging.
- Two IE2-1024 encoders.
- LSM6DSV16X IMU, with values aligned to project conventions before estimator software.
- Custom IR wall sensors, including log-amp front sensors.
- DRV8871 H-bridge motor drivers.
- Two Faulhaber 1717T006SR drive motors.
- 17:56 gearing.
- 25 mm wheels with solid-rubber tires.
- Four wheels, two left and two right, shared pinion per bank.
- Contact patches approximately centered at \(\pm40\ \mathrm{mm}\) right and \(\pm14.75\ \mathrm{mm}\) forward.
- Vehicle mass approximately \(0.140\ \mathrm{kg}\).
- Yaw inertia approximately \(220\times10^{-6}\ \mathrm{kg\ m^2}\).

The operating envelope includes tire-scrubbing calibration turns, high-speed arcing turns with expected stick-slip, full-bore straight acceleration limited by ground strike and motor power, wall-proximate precision turns, and constant-velocity exploration/mapping.

---

## 2. Relationship to the current implementation

### 2.1 Legacy implementation classification

The current codebase contains useful infrastructure:

```text
target/host build parity
same-code host replay
startup calibration scaffolding
wall preprocessing and geometry owners
existing test/replay tooling
drive/feedforward integration surfaces
```

Those assets may be reused after audit. However, the current plant/estimator design is known to contain failure-producing assumptions for this robot, including:

```text
Bgz / yaw-gyro-bias state in the UKF with effectively frozen process noise
contact/slip math unstable or ill-conditioned at zero forward speed
traction-gated control/feedforward authority
runtime maneuver labels and thresholds for turn behavior
rolling/nonholonomic assumptions used where contact slip is expected
```

`Ahead of the spec` means only implemented, integrated, or operationally mature. It does not mean correct or representative.

### 2.2 Migration rule

| Current implementation feature | Target treatment |
|---|---|
| Same-code host replay | Preserve; this is the validation mechanism. |
| Robust UKF-input logging | Preserve; do not expand embedded logging for validation metrics. |
| Startup calibration service | Preserve structure, but keep yaw gyro bias outside the UKF. |
| Wall preprocessing and geometry | Preserve after measurement-semantics audit. |
| Hardware/replay/API tests | Preserve or port. |
| Tests asserting traction-gated command rejection | Retire or rewrite. |
| Runtime turn labels | Do not promote into plant/noise/measurement semantics. |
| Slip-ratio, slip-angle, curvature, ICR, turn-radius math | Reject as primary plant math. |
| `Bgz` UKF state | Reject. |

### 2.3 Exceptional-scrutiny rule

Any feature imported from the current implementation must pass:

```text
Does it preserve the target state semantics?
Does it preserve coordinate/sign semantics?
Does it remain finite at zero forward speed?
Does it avoid runtime maneuver classification?
Does it avoid traction-based command admissibility?
Is it replayable from logged estimator-boundary inputs and configuration?
Does it improve held-out host replay under the target validation tests?
```

A negative answer means the feature remains legacy behavior.

---

## 3. Hard state and coordinate contract

### 3.1 State vector

\[
state\_vec=
\begin{bmatrix}
Px & Py & heading & Vf & Vr & yawRate & \Delta Af & \Delta Ar & \Delta yawAccel
\end{bmatrix}^T
\]

| State | Unit | Meaning |
|---|---:|---|
| `Px`, `Py` | m | Global maze position |
| `heading` | rad | Global heading; `0` faces +Y; clockwise positive |
| `Vf` | m/s | Body-frame forward velocity |
| `Vr` | m/s | Body-frame rightward velocity |
| `yawRate` | rad/s | Clockwise-positive body yaw rate |
| `DeltaAf` | m/s² | Colored forward acceleration residual |
| `DeltaAr` | m/s² | Colored rightward/lateral acceleration residual |
| `DeltaYawAccel` | rad/s² | Colored yaw acceleration residual |

### 3.2 Coordinate convention

```text
global +Y: forward
global +X: right
body +f: forward
body +r: right
heading = 0: facing global +Y
positive yaw: clockwise
```

Body-to-global velocity:

\[
\dot{Px}=Vf\sin(heading)+Vr\cos(heading)
\]

\[
\dot{Py}=Vf\cos(heading)-Vr\sin(heading)
\]

Body dynamics:

\[
\dot{heading}=yawRate
\]

\[
\dot{Vf}=Af+yawRate\,Vr
\]

\[
\dot{Vr}=Ar-yawRate\,Vf
\]

\[
\dot{yawRate}=yawAccel
\]

IMU sign correction occurs only before the estimator. The estimator receives canonical body-frame values.

---

## 4. Explicit state exclusions

### 4.1 No `Bgz` / gyro-bias state

Do not include `Bgz`, `gyroBiasYaw`, or any equivalent yaw-gyro-bias state in the UKF.

Target treatment:

```text
gyroBiasYawExt: external startup/run calibration value
```

Gyro measurement entering the UKF:

\[
z_{gyro}=gyroYawCanonical-gyroBiasYawExt
\]

Measurement function:

\[
h_{gyro}(state\_vec)=yawRate
\]

The current `Bgz` state with effectively zero process noise is not a useful state. It consumes state budget while being prevented from estimating meaningful variation. A gyro-bias UKF state may be reconsidered only if held-out logs show within-run drift large enough to compete with the heading error budget and external calibration cannot represent it.

### 4.2 No wheel-rate states

Do not include wheel-bank rates as UKF states. Encoders measure drivetrain motion directly. The estimator consumes validated wheel-bank rates as plant inputs.

### 4.3 No motor-current states

Do not include motor current as a UKF state. Electrical and driver current-regulation effects are much faster than the 1 ms estimator tick and are represented algebraically in the motor/driver model plus process-noise scheduling.

### 4.4 No slip states

Do not include slip-ratio, slip-angle, left/right slip, or patch-slip states. The target plant computes contact-relative velocities algebraically for each sigma point.

### 4.5 No pitch/roll/vertical states initially

The UKF remains planar. Ground strike, chassis rocking, and pitch/roll contamination are handled through acceleration-envelope effects, measurement covariance inflation, process-noise scheduling, and host-replay diagnostics unless held-out replay proves planar modeling cannot maintain pose accuracy.

---

## 5. Estimator input, timing, and implementation context

### 5.1 Canonical implementation context

The production codebase already has two canonical runtime types:

```text
CommandVector: left/right PWM command values
VehicleState: per-tick estimator context visible to model owners by constructor reference
```

This specification shall not introduce competing tick-specific carriers such as `MotorTorqueInput`, `MotorTorqueOutput`, `ContactForceInput`, `WheelAngularRate`, or `MotorPhase`. Submodel owners may read tick-specific state, derived values, timing, validated wheel-bank rates, sample-validity flags, and other estimator-context values from `VehicleState` through the existing project API.

`VehicleState` is a tick context, not a model-parameter bag. Durable model coefficients remain private to behavior-owning model classes and their validated construction-time calibration objects. If the current implementation still stores legacy calibration values such as `Bgz` inside `VehicleState`, the target estimator must not treat that value as a UKF state and plant submodels must not use it as an arbitrary shared model coefficient.

In equations below, `C_L` and `C_R` mean the left and right PWM values extracted from `CommandVector`. The spec does not require a new command wrapper.

### 5.2 Tick envelope semantics

The estimator consumes a time-stamped tick envelope:

```text
estimator_tick = {
    timestamp,
    commandVector,
    encoder_sample,
    imu_sample,
    wall_samples
}
```

The block above is a semantic boundary schema, not a production C++ object declaration. Production APIs should use the existing `CommandVector` and `VehicleState` architecture.

`timestamp` is the estimator-boundary timebase. The estimator derives:

\[
\Delta t_k=timestamp_k-timestamp_{lastPredict}
\]

If `VehicleState` exposes `dT`, that value must be derived from the same estimator-boundary timestamp/effective-time policy. It must not be an independent external timing source.

### 5.3 Command sample

`CommandVector` is the canonical command object and carries left/right PWM values. The firmware/logging contract must state whether the command values were active over the interval ending at `timestamp` or were issued at `timestamp` for the next interval.

No live `Vbat` measurement is assumed. Bus-voltage effects are calibrated parameters, torque uncertainty, and process-noise schedule inputs.

### 5.4 Encoder sample

Semantic encoder information required by the model:

```text
deltaCountL, deltaCountR
wheelRateL, wheelRateR
Romega
encoderSampleValid
encoderFaultReason
```

`wheelRateL/R` are drivetrain inputs, not body-motion measurements. They may already be stored as derived values in `VehicleState`; the spec does not require a `WheelAngularRate` wrapper. `Romega` must enter prediction before motor torque, contact-relative velocity, force limiting, noise scheduling, and any optional encoder-body pseudo-measurement.

Invalid encoder samples are acquisition faults. They are skipped or routed through an explicit degraded prediction path; they are not ordinary Gaussian missed-pulse noise.

There is no normative `MotorPhase` input. If a future ripple correction needs phase-like information, the source, units, wrapping convention, logging/replay reconstruction, and ownership must be specified before the correction is promoted. Until then, ripple is not part of the first-pass torque model.

### 5.5 IMU sample

```text
imu_sample = {
    timestamp_or_phase,
    gyroYawCanonical,
    accelFCanonical,
    accelRCanonical,
    accelZOptional,
    imuStatus,
    imuFilterConfig,
    imuSampleValid
}
```

When `imuSampleValid == false`:

```text
skip gyro and accelerometer measurement rows
do not update gyroBiasYawExt
use outage/degraded covariance policy if persistent
make skip reason reconstructable in host replay/debug diagnostics
```

### 5.6 Wall sample

```text
wall_sample_s = {
    timestamp_or_phase,
    raw_or_distance_value,
    sensorValid,
    sensorSaturation,
    sensorNoiseEstimate,
    wallHypothesisAvailable
}
```

When `sensorValid == false`, skip the wall row. Do not convert invalid raw values into large innovations. Very-large-covariance rows are allowed only as skipped-equivalent adapters for fixed-size APIs.

### 5.7 Effective-time handling

Each measurement type must use one replayable policy:

1. evaluate \(h(x)\) at the measurement effective time using local propagation/interpolation; or
2. evaluate at the nearest propagated state and add timing covariance:

\[
R_{timing}\approx\sigma_t^2\dot{h}\dot{h}^T
\]

The spec does not prescribe ADC/SPI/readout timing. It requires explicit estimator-boundary effective-time semantics.

---

## 6. Drive-layer motion limits and command admissibility

### 6.1 Motion limits are drive-layer policy

The drive layer may impose configurable bounds on:

```text
velocity
forward/lateral acceleration
yawRate
yawAccel / alpha
hardware command magnitude
thermal/current safety limits
```

These are external command-shaping or safety constraints. They are independent of the plant model.

### 6.2 Plant diagnostics do not reject performance commands

The plant may compute force-envelope ratios, contact-relative velocities, force limiter activity, and drive-authority uncertainty. These may be used for:

```text
feedforward diagnostics
process-noise scheduling
measurement covariance scheduling
optional conservative/debug reporting
host replay model comparison
```

They must not be used as default hard admissibility gates for performance or calibration commands.

Reject:

```text
if nominal traction / force envelope is exceeded:
    reject or clamp command
```

Accept:

```text
commands remain governed by drive-layer motion/hardware/safety limits
plant reports high force-envelope ratio or high model uncertainty
estimator weakens inappropriate measurements and increases process noise
```

A traction-limited inverse solver may exist only as an optional conservative/debug/analysis mode. It is not default performance feedforward authority.

### 6.3 Feedforward objective

The feedforward path shall be tuned primarily for high-performance contact conditions:

```text
calibration turns with substantial tire scrub
high relative-contact-velocity arcing turns
full-bore acceleration and braking
precision wall-proximate turns
```

Low-demand rolling-style motion is a calibration and sanity regime. It is useful for signs, timing, encoder scale, wheel radius, wall geometry, and low-energy consistency. It is not the primary optimization target.

---

## 7. Contact-continuum requirement

### 7.1 No runtime maneuver taxonomy

The plant shall not require runtime classification into:

```text
in-place
pivot
rolling
scrub
stick-slip
```

Those labels may appear in human prose or test-segment names. They do not define estimator states, telemetry fields, plant branches, command admissibility, measurement modes, or noise-schedule mode names.

If an implementation needs those categories to behave correctly, that is a design defect.

### 7.2 Red-flag patterns

The plant and feedforward authority must not use these as primary variables:

```text
slipRatio = (...) / Vf
slipRatio = (...) / max(abs(Vf), eps)
slipAngle = atan2(lateralVelocity, abs(forwardVelocity))
yawRate / Vf
Vf / yawRate
turn radius
curvature
instantaneous center of rotation as primary contact formulation
stationary threshold to select turn/contact physics
opposed-command threshold to select turn/contact physics
```

Small epsilons are permitted for stable norms and smooth signs. They are not permitted as disguised denominators for speed-normalized tire models.

### 7.3 Required contact-relative-velocity primitive

Each contact patch \(i\) has body-frame location:

\[
(r_i,f_i)
\]

where \(r_i>0\) means right and \(f_i>0\) means forward.

Body velocity at the patch:

\[
v_{body,f,i}=Vf-yawRate\,r_i
\]

\[
v_{body,r,i}=Vr+yawRate\,f_i
\]

Wheel surface forward velocity:

\[
v_{surf,f,i}=r_w\hat{wheelRate}_{side(i)}
\]

Primary contact-relative velocities:

\[
v_{rel,f,i}=v_{surf,f,i}-v_{body,f,i}=r_w\hat{wheelRate}_{side(i)}-Vf+yawRate\,r_i
\]

\[
v_{rel,r,i}=-v_{body,r,i}=-Vr-yawRate\,f_i
\]

These variables are affine in state and wheel rate. They are finite and continuous for:

```text
Vf = 0
Vr = 0
yawRate != 0
one or both wheel banks stopped
one or both wheel banks moving
UKF sigma points crossing zero forward speed
```

### 7.4 Continuous aggregate quantities

The plant may compute continuous physical scalars:

\[
contactRelRms=\sqrt{\frac{\sum_i N_i(v_{rel,f,i}^2+v_{rel,r,i}^2)}{\sum_i N_i+\epsilon}}
\]

\[
lateralRelRms=\sqrt{\frac{\sum_i N_i v_{rel,r,i}^2}{\sum_i N_i+\epsilon}}
\]

\[
yawContactSpeedRms=|yawRate|\sqrt{\frac{\sum_i N_i(r_i^2+f_i^2)}{\sum_i N_i+\epsilon}}
\]

These are physical scalars, not maneuver labels.

---

## 8. Smooth helper functions

Smooth helpers are numerical defaults, not physical law. Replacements are allowed if they preserve continuity, signs, units, and replay consistency.

Smooth sign:

\[
sgnE(x,x_E)=\tanh(x/x_E)
\]

Softplus, stable implementation required:

\[
softplusE(x,e)=e\log(1+\exp(x/e))
\]

Smooth deadzone:

\[
deadzoneE(x,a,e)=softplusE(x-a,e)-softplusE(-x-a,e)
\]

Smooth step:

\[
smoothStepE(x,x_0,x_1)=\frac{1}{2}\left[1+\tanh\left(\frac{x-(x_0+x_1)/2}{(x_1-x_0)/6}\right)\right]
\]

If \(x_0=x_1\), the implementation must use a documented fallback rather than divide by zero.

Smooth max/min:

\[
smoothMaxE(a,b,e)=\frac{a+b+\sqrt{(a-b)^2+e^2}}{2}
\]

\[
smoothMinE(a,b,e)=\frac{a+b-\sqrt{(a-b)^2+e^2}}{2}
\]

Smooth asymmetric clip:

\[
clipAsymE(x,x_{min},x_{max},e)=smoothMaxE(x_{min},smoothMinE(x,x_{max},e),e)
\]

---

## 9. Plant submodel pattern and anti-parameter-bag rule

Every empirical physical submodel must declare:

```text
Model status
Hard contract
First-pass implementation
Expected failure signatures
Permitted follow-on options
Promotion / replacement rule
```

Hard contracts are state meanings, units, signs, timing, validity behavior, covariance semantics, and submodel inputs/outputs. First-pass empirical formulas are not permanent physical truth.

### 9.1 Runtime ownership rule

Flexible does not mean globally parameterized. Runtime estimator code shall not expose or depend on one passive `PlantParams`, `EstimatorParams`, `NoiseParams`, `WallParams`, `Config`, or equivalent scalar namespace containing unrelated model coefficients.

Each flexible submodel shall be represented by a behavior-owning class. Calibration values are owned by that class or by a typed calibration class used only to construct that class. Public fields and public data members are prohibited. C++ `struct` declarations are prohibited for model, calibration, configuration, input, output, diagnostic, and metadata API objects. Cross-submodel coupling must happen through explicit owner methods, `VehicleState`, named calibration products, or returned math artifacts, not arbitrary reads from a shared scalar namespace.

The parameter registry in Section 23 is a metadata and replay-traceability artifact. It is not a runtime parameter lookup service.

### 9.2 VehicleState and output ownership rule

`VehicleState` is the canonical tick context. It may be passed by reference to model owners at construction time. Model owners may read required tick values from it and may write/update their own derived outputs through named methods or existing project channels.

Do not introduce separate per-submodel input/output carrier classes merely to shuttle values already available in `VehicleState`. In particular, this spec rejects generated API types such as:

```text
MotorTorqueInput
MotorTorqueOutput
ContactForceInput
ContactForceOutput
NormalLoadInput
NormalLoadOutput
WheelAngularRate
MotorPhase
```

If an implementation uses a returned result object for math-core convenience, that object must be immutable, private-field, narrowly scoped to one evaluation, and must not carry durable calibration coefficients. The preferred runtime pattern is a behavior-owning class reading `VehicleState` plus explicit external arguments such as `CommandVector`, then updating or exposing named computed quantities.

### 9.3 Behavior-to-field review heuristic

A class with fewer behavioral functions than stored fields is presumed to be a parameter bag in another form.

For this review heuristic:

```text
Counts as behavior:
    update/evaluate/predict methods
    invariant checks
    finite/continuity checks
    interpolation, clipping, force, torque, load, wall, or covariance logic
    validation/export logic tied to model behavior

Does not count as behavior:
    constructors/destructors
    trivial getters/setters
    public field access
    plain serialization
    configHash() alone
    variantId() alone
```

The heuristic is intentionally conservative. A violation must be justified explicitly before code using it is accepted.

### 9.4 Required starting class structures for bag-prone areas above moderate risk

The sketches below define ownership and API shape, not filenames, heap allocation, or a math-core implementation. Concrete implementations may use static storage, fixed-size arrays, and `final` classes.

#### 9.4.1 Plant assembly owner — high risk

```cpp
class EstimatorPlantModel final {
public:
    EstimatorPlantModel(VehicleState& vehicleState,
                        MotorTorqueModel motorTorque,
                        NormalLoadModel normalLoad,
                        ContactForceModel contactForce,
                        YawMomentModel yawMoment,
                        GroundEnvelopeModel groundEnvelope,
                        ResidualNoiseSchedule residualNoise);

    void predictFromCommand(const CommandVector& command);
    PredictionNoise buildPredictionNoise() const;
    bool verifyFinitePrediction() const;
    bool verifyStateContract() const;
    bool verifyNoTractionCommandGate() const;
    bool verifyZeroSpeedContinuity() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    void updateTimingFromVehicleState();
    void runMotorTorque(const CommandVector& command);
    void runNormalLoad();
    void runContactForce();
    void runYawMoment();
    void runGroundEnvelope();
    void runResidualNoise();

    VehicleState& vehicleState_;
    MotorTorqueModel motorTorque_;
    NormalLoadModel normalLoad_;
    ContactForceModel contactForce_;
    YawMomentModel yawMoment_;
    GroundEnvelopeModel groundEnvelope_;
    ResidualNoiseSchedule residualNoise_;
};
```

#### 9.4.2 Motor torque owner — high risk

```cpp
class MotorTorqueModel final {
public:
    MotorTorqueModel(VehicleState& vehicleState,
                     MotorTorqueCalibration calibration,
                     TorqueCorrectionModel correction);

    void updateFromCommand(const CommandVector& command);
    bool validateCommandRange(const CommandVector& command) const;
    bool verifyFiniteTorque() const;
    bool currentLimitCanBind() const;
    float bankTorqueNewtonMeters(Side side) const;
    float driveForceNewtons(Side side) const;
    float driveSaturationIndex() const;
    float driveAuthorityUncertainty() const;
    ModelVariantId variantId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float pwmForSide(const CommandVector& command, Side side) const;
    float normalizedPwm(float pwm) const;
    float motorRateFromVehicleState(Side side) const;
    float driveVoltage(float normalizedPwm) const;
    float rawCurrent(float driveVoltage, float motorRate) const;
    float limitedCurrent(float rawCurrent) const;
    float motorTorque(float current, float motorRate) const;
    float rawBankTorque(float motorTorque) const;
    float usableBankTorque(Side side, float rawBankTorque) const;
    float driveForceFromTorque(float bankTorque) const;
    void writeVehicleStateOutputs(float leftTorque, float rightTorque);

    VehicleState& vehicleState_;
    MotorTorqueCalibration calibration_;
    TorqueCorrectionModel correction_;
};
```

`CommandVector` is the only command argument. Wheel-bank rates and `dT` are read from `VehicleState`. There is no `MotorTorqueInput`, `MotorTorqueOutput`, `WheelAngularRate`, or `MotorPhase` API.

#### 9.4.3 Normal-load owner — high physical-coupling risk

```cpp
class NormalLoadModel final {
public:
    NormalLoadModel(VehicleState& vehicleState,
                    NormalLoadCalibration calibration,
                    FanLoadModel fanLoad,
                    LoadTransferModel loadTransfer);

    void updateLoads();
    bool verifyNonnegativeLoads() const;
    bool verifyLoadConservation() const;
    float normalLoadNewtons(ContactPatchId patch) const;
    bool minimumLoadClampActive(ContactPatchId patch) const;
    ModelVariantId variantId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float staticLoad(ContactPatchId patch) const;
    float fanLoad(ContactPatchId patch) const;
    float longitudinalTransfer(ContactPatchId patch) const;
    float lateralTransfer(ContactPatchId patch) const;
    float clampLoad(float load) const;
    void writeVehicleStateOutputs();

    VehicleState& vehicleState_;
    NormalLoadCalibration calibration_;
    FanLoadModel fanLoad_;
    LoadTransferModel loadTransfer_;
};
```

#### 9.4.4 Contact force owner — very high risk

```cpp
class ContactForceModel final {
public:
    ContactForceModel(VehicleState& vehicleState,
                      ContactForceCalibration calibration,
                      ContactEnvelopeModel envelope);

    void updateContactForces();
    bool verifyZeroForwardSpeedContinuity() const;
    bool verifyFiniteForces() const;
    bool verifyNoCommandAdmissibilityGate() const;
    float contactRelativeForwardVelocity(ContactPatchId patch) const;
    float contactRelativeRightVelocity(ContactPatchId patch) const;
    float forceEnvelopeRatio(ContactPatchId patch) const;
    float forceLimiterActivity(ContactPatchId patch) const;
    ModelVariantId variantId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float relativeForwardVelocity(ContactPatchId patch) const;
    float relativeRightVelocity(ContactPatchId patch) const;
    float driveForceRequest(ContactPatchId patch) const;
    Force2D rawForceRequest(ContactPatchId patch) const;
    float forceLimit(ContactPatchId patch, float relativeSpeed) const;
    float envelopeRatio(const Force2D& request, float limit) const;
    Force2D limitedForce(const Force2D& request, float envelopeRatio) const;
    void writeVehicleStateOutputs();

    VehicleState& vehicleState_;
    ContactForceCalibration calibration_;
    ContactEnvelopeModel envelope_;
};
```

#### 9.4.5 Yaw moment owner — moderate-high risk

```cpp
class YawMomentModel final {
public:
    YawMomentModel(VehicleState& vehicleState,
                   YawMomentCalibration calibration);

    void updateYawMoment();
    bool verifyNoTurnCategoryDependency() const;
    bool verifyFiniteYawAcceleration() const;
    float yawMomentContact() const;
    float yawMomentLoss() const;
    float yawMomentNominal() const;
    float yawAccelerationRaw() const;
    ModelVariantId variantId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float contactMomentFromVehicleState() const;
    float continuousYawLoss() const;
    float nominalMoment(float contactMoment, float lossMoment) const;
    float rawYawAcceleration(float nominalMoment) const;
    void writeVehicleStateOutputs(float contactMoment, float lossMoment, float yawAccel);

    VehicleState& vehicleState_;
    YawMomentCalibration calibration_;
};
```

#### 9.4.6 Ground-envelope owner — high physical-coupling risk

```cpp
class GroundEnvelopeModel final {
public:
    GroundEnvelopeModel(VehicleState& vehicleState,
                        GroundEnvelopeCalibration calibration,
                        ImpactIndicatorModel impactIndicator,
                        ForceConsistencyPolicy forceConsistency);

    void updateEnvelope();
    bool verifyInternalConsistency() const;
    bool forceMutationEnabled() const;
    float clippedForwardAcceleration() const;
    float groundUse() const;
    float impactIndicator() const;
    ModelVariantId variantId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float rawForwardAcceleration() const;
    float clipForwardAcceleration(float rawAf) const;
    float computeGroundUse(float rawAf, float clippedAf) const;
    float computeImpactIndicator(float groundUse) const;
    void applyForceConsistencyPolicy();
    void writeVehicleStateOutputs(float clippedAf, float groundUse, float impact);

    VehicleState& vehicleState_;
    GroundEnvelopeCalibration calibration_;
    ImpactIndicatorModel impactIndicator_;
    ForceConsistencyPolicy forceConsistency_;
};
```

#### 9.4.7 Residual noise schedule owner — very high risk

```cpp
class ResidualNoiseSchedule final {
public:
    ResidualNoiseSchedule(VehicleState& vehicleState,
                          ResidualNoiseCalibration calibration,
                          OuInjectionConvention convention);

    void updateForCurrentPrediction();
    PredictionNoise buildPredictionNoise() const;
    bool verifyOuSemantics(TimeStep deltaTime) const;
    bool verifyFullCovarianceInjection() const;
    float steadyStateSigma(ResidualChannel channel) const;
    float ouPhi(ResidualChannel channel, TimeStep deltaTime) const;
    float residualVariance(ResidualChannel channel, TimeStep deltaTime) const;
    OuInjectionConvention injectionConvention() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    float contactRelativeContribution(ResidualChannel channel) const;
    float forceLimitContribution(ResidualChannel channel) const;
    float groundContribution(ResidualChannel channel) const;
    float driveAuthorityContribution(ResidualChannel channel) const;
    float impactContribution(ResidualChannel channel) const;
    float encoderFaultContribution(ResidualChannel channel) const;
    void writeVehicleStateOutputs();

    VehicleState& vehicleState_;
    ResidualNoiseCalibration calibration_;
    OuInjectionConvention convention_;
};
```

#### 9.4.8 Wall measurement owner — high risk

```cpp
class WallSensorModel final {
public:
    WallSensorModel(VehicleState& vehicleState,
                    WallSensorCalibration calibration,
                    WallResponseModel responseModel,
                    WallNoiseModel noiseModel);

    void predictForCurrentState();
    bool makeUpdateCandidate(WallUpdateAssembler& assembler) const;
    bool shouldSkipSample(const WallSampleView& sample) const;
    bool verifyRayBundleNumerics() const;
    bool verifyNoHitPolicy() const;
    bool verifySaturationPolicy() const;
    SensorId sensorId() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    SensorPose sensorPoseFromVehicleState() const;
    RayBundle buildRayBundle(const SensorPose& pose) const;
    RayHitSet castAgainstWallHypothesis(const RayBundle& rays) const;
    float effectiveDistance(const RayHitSet& hits) const;
    float predictedResponse(float effectiveDistance) const;
    float measurementVariance(const WallSampleView& sample) const;

    VehicleState& vehicleState_;
    WallSensorCalibration calibration_;
    WallResponseModel responseModel_;
    WallNoiseModel noiseModel_;
};

class WallMeasurementModel final {
public:
    WallMeasurementModel(VehicleState& vehicleState,
                         WallSensorSet sensors);

    MeasurementBlock buildUpdateBlock();
    bool verifyCorrelationPolicy() const;
    bool verifyNoInvalidSampleInnovation() const;
    ModelConfigHash configHash() const;
    void exportParameterMetadata(ParameterMetadataRegistry& registry) const;

private:
    void collectCandidates(WallUpdateAssembler& assembler) const;
    void groupCorrelatedCandidates(WallUpdateAssembler& assembler) const;
    void applySkipEquivalentPolicy(WallUpdateAssembler& assembler) const;

    VehicleState& vehicleState_;
    WallSensorSet sensors_;
};
```

#### 9.4.9 Parameter metadata registry — very high risk

```cpp
class ParameterMetadataRegistry final {
public:
    void recordModel(const ModelMetadataProvider& model);
    void validateCompleteness() const;
    void validateNoRuntimeLookupApi() const;
    ParameterRegistrySnapshot snapshot() const;
    ModelConfigHash registryHash() const;

private:
    ParameterMetadataStore metadata_;
};
```

The registry records names, units, priors, bounds, fit data, validation data, residual policy, confounds, and version metadata. It must not provide runtime scalar lookup methods such as `getFloat(name)` for plant evaluation.

#### 9.4.10 Configuration manifest owner — moderate-high risk

```cpp
class EstimatorModelConfiguration final {
public:
    static Expected<EstimatorModelConfiguration, ConfigurationError>
    load(const ConfigurationSource& source, VehicleState& vehicleState);

    EstimatorPlantModel makePlantModel();
    MeasurementModel makeMeasurementModel();
    void validateOwnerGraph() const;
    void validateBehaviorToFieldHeuristic() const;
    void validateNoParameterBagApis() const;
    ModelConfigHash configHash() const;
    ParameterRegistrySnapshot parameterMetadata() const;

private:
    EstimatorModelConfiguration(VehicleState& vehicleState,
                                EstimatorPlantModel plantModel,
                                MeasurementModel measurementModel,
                                ParameterMetadataRegistry metadataRegistry,
                                BuildIdentity buildIdentity);

    VehicleState& vehicleState_;
    EstimatorPlantModel plantModel_;
    MeasurementModel measurementModel_;
    ParameterMetadataRegistry metadataRegistry_;
    BuildIdentity buildIdentity_;
};
```

The configuration manifest constructs typed model owners and records their identity. It is not a mutable global configuration object and not a runtime coefficient dictionary.

---

## 10. Motor and driver torque model

### 10.1 Model status

Flexible empirical model family with a physics-first initial mean model.

### 10.2 Hard contract

Runtime inputs:

```text
CommandVector left/right PWM values
validated left/right wheel-bank rates from VehicleState
current timestamp/dT from VehicleState
optional fan/supply/runtime estimator outputs only if already owned and versioned
```

Owned calibration:

```text
R_m, K_t/K_e, M_R
R_drv, R_wire
V_busEff prior and uncertainty
I_trip or current-limit policy if relevant
eta_drive left/right
launch and rolling/friction terms
compact torque-correction variant, initially zero
```

Outputs, normally written to or exposed from `VehicleState` by `MotorTorqueModel`:

```text
T_bankRaw_L, T_bankRaw_R
T_bank_L, T_bank_R
F_drive_L, F_drive_R
driveSaturationIndex
driveAuthorityUncertainty
torque uncertainty hooks
```

Units:

```text
torque: N*m
force: N
wheel and motor rate: rad/s
```

There is no normative `MotorPhase`. Calibration is not passed as a per-tick input; it is private owner state validated when `MotorTorqueModel` is constructed.

### 10.3 First-pass implementation

Use a direct quasi-static motor/driver model as the initial mean model.

Let \(C_j\) be the side-specific PWM command value extracted from `CommandVector`:

\[
u_j=normalize(C_j),\quad u_j\in[-1,1]
\]

Wheel-bank rate is read from `VehicleState`:

\[
motorRate_j=G\hat{wheelRate}_j
\]

In SI units, use:

\[
K_e=K_t
\]

For the 1717T006SR 006 prior:

\[
K_t=3.96\ \mathrm{mNm/A}=0.00396\ \mathrm{N\ m/A}
\]

\[
K_e=0.00396\ \mathrm{V\ s/rad}
\]

Voltage-equivalent drive:

\[
V_{drive,j}=D(u_j,pwmMode)V_{busEff}
\]

where \(V_{busEff}\) is calibrated/run-level, not a UKF state and not a live measurement.

Current prior:

\[
I^{raw}_j=\frac{V_{drive,j}-K_e motorRate_j}{R_m+R_{drv}+R_{wire}}
\]

If configured current limiting is nonbinding over the relevant operating region:

\[
I_j=I^{raw}_j
\]

If predicted current can bind:

\[
I_j=I_{trip}\tanh\left(\frac{I^{raw}_j}{I_{trip}}\right)
\]

or use a versioned mean current-regulator model.

Motor torque prior:

\[
T_{motor,j}=K_t I_j-M_R\,sgnE(motorRate_j,motorRate_E)
\]

Wheel-bank torque:

\[
T_{bankRaw,j}=\eta_{drive,j}G\,T_{motor,j}
\]

Usable torque:

\[
slow_j=\exp\left[-\left(\frac{r_w|\hat{wheelRate}_j|}{v_{static}}\right)^2\right]
\]

\[
T_{bank,j}=deadzoneE(T_{bankRaw,j},slow_jT_{launch,j},T_E)
-T_{roll,j}sgnE(\hat{wheelRate}_j,wheelRate_E)
-T_{visc,j}\hat{wheelRate}_j
\]

Set \(T_{visc,j}=0\) initially unless logs prove it is needed.

\[
F_{drive,j}=\frac{T_{bank,j}}{r_w}
\]

First-pass drive authority diagnostics:

\[
driveSaturationIndex=\max_j\left|\frac{I^{raw}_j}{I_{trip}+\epsilon}\right|
\]

when a configured current limit can bind. If the active torque model does not use the current-limit prior, `driveSaturationIndex` is the versioned saturation proxy emitted by that torque model.

\[
driveAuthorityUncertainty=\max_j\left(\frac{\sigma_{T,j}}{|T_{bank,j}|+T_E}\right)
\]

or the equivalent uncertainty output from the active torque model. These quantities are covariance-scheduling diagnostics. They do not gate commands.

### 10.4 Compact correction path

Target production form:

\[
T_{bank,j}=T_{direct,j}+\delta T_j
\]

with \(\delta T_j=0\) initially.

Promote corrections in this order:

1. left/right scale and offset;
2. effective \(V_{busEff}\), \(R_{wire}\), or \(R_{drv}\) calibration;
3. launch and rolling/friction terms;
4. compact command/rate correction surface;
5. runtime/thermal/recent-current proxy;
6. only if explicitly defined and replayable, a periodic ripple correction based on a specified measured source.

A dense LUT is a follow-on escape path, not the initial mean model. A phase-like ripple source must not be invented by the spec; it must be introduced as a separate measured or reconstructable quantity with units, wrapping, and replay semantics.

### 10.5 Expected failure signatures

| Signature | Likely interpretation |
|---|---|
| Straight launch requires persistent positive `DeltaAf` | Torque, launch threshold, or voltage prior too weak |
| Braking requires persistent negative `DeltaAf` | Decay/brake behavior or reverse torque wrong |
| Differential commands produce biased yaw residuals | Left/right torque asymmetry or contact/yaw model wrong |
| Similar commands drift over a run | Thermal or supply effect missing |
| Residuals show repeatable periodic structure versus a logged drivetrain coordinate | Define and validate a measured ripple source before adding ripple correction |

### 10.6 Promotion rule

Promote a torque-model change only if same-code host replay improves held-out launch, braking, high-contact-slip turn, and arc logs without degrading low-demand straight consistency.

---

## 11. Normal-load model

### 11.1 Model status

Flexible algebraic load-estimation model family.

### 11.2 Hard contract

Inputs:

```text
vehicle mass
static load distribution
contact patch geometry
fanCommand
lagged nominal Af/Ar
optional drive/force diagnostics
```

Outputs:

```text
N_i_preClamp
N_i
N_longTransfer_i
N_latTransfer_i
N_fan_i
N_min_active_i
```

Loads are in newtons and must remain nonnegative in the contact force calculation.

### 11.3 First-pass implementation

\[
N_i=N_{static,i}+N_{fan,i}+N_{longTransfer,i}+N_{latTransfer,i}
\]

Fan/downforce:

\[
N_{fan,i}=w_{fan,i}F_{fan}(fanCommand)
\]

Use previous-step or low-pass-lagged **nominal** acceleration, not raw accelerometer samples and not residual-corrected acceleration:

\[
Af_{lag},\quad Ar_{lag}
\]

Load transfer:

\[
N_{longTransfer,i}=-k_{longLoad}Af_{lag}\frac{f_i}{\sum_k f_k^2+\epsilon}
\]

\[
N_{latTransfer,i}=-k_{latLoad}Ar_{lag}\frac{r_i}{\sum_k r_k^2+\epsilon}
\]

with:

\[
k_{longLoad}\ge0,\quad k_{latLoad}\ge0
\]

Sign convention:

```text
positive forward acceleration unloads forward patches and loads rear patches
positive rightward lateral acceleration unloads right patches and loads left patches
```

Clamp:

\[
N_i\leftarrow smoothMaxE(N_i,N_{min},N_E)
\]

If load-transfer characterization is unavailable, set:

```text
k_longLoad = 0
k_latLoad = 0
```

but treat zero-transfer as an explicit approximation.

### 11.4 Expected failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Force-limit parameters become implausible in high lateral acceleration arcs | Fit lateral load-transfer gain or use one-step fixed point |
| Full-bore straights clip too early or late | Fit longitudinal transfer and ground envelope jointly |
| Fan command changes behavior but load model cannot explain it | Fit fan load curve and distribution |
| `N_min` active in ordinary motion | Check gain/sign/static distribution/clamp floor |

---

## 12. Contact force model

### 12.1 Model status

Flexible empirical contact-force model family. The first pass is velocity-space and force-space, not speed-normalized slip-space.

### 12.2 Hard contract

Inputs per contact patch:

```text
v_rel_f_i
v_rel_r_i
N_i
F_drive_side(i)
contact geometry
ContactForceModel calibration
```

Outputs:

```text
F_f_i
F_r_i
F_f_req_i
F_r_req_i
F_limit_i
forceEnvelopeRatio_i
forceScale_i
forceLimiterActivity_i
```

Force outputs must be continuous through zero forward speed, finite yaw rate, and force-limiting transitions.

### 12.3 First-pass implementation

Distribute bank drive force by normal load:

\[
w_{i|j}=\frac{N_i}{\sum_{k\in j}N_k+\epsilon}
\]

\[
F_{driveReq,i}=w_{i|side(i)}F_{drive,side(i)}
\]

Raw force request:

\[
F_{f,req,i}=F_{driveReq,i}+K_{f,i}v_{rel,f,i}
\]

\[
F_{r,req,i}=K_{r,i}v_{rel,r,i}
\]

Relative speed:

\[
v_{relMag,i}=\sqrt{v_{rel,f,i}^2+v_{rel,r,i}^2+v_E^2}
\]

First-pass force envelope:

\[
\mu_i=\mu_{slide,i}+(\mu_{peak,i}-\mu_{slide,i})\exp\left[-\left(\frac{v_{relMag,i}}{v_{Stribeck,i}}\right)^2\right]
\]

\[
F_{limit,i}=\mu_iN_i
\]

Force-envelope ratio:

\[
forceEnvelopeRatio_i=\sqrt{
\left(\frac{F_{f,req,i}}{\lambda_fF_{limit,i}+\epsilon}\right)^2+
\left(\frac{F_{r,req,i}}{\lambda_rF_{limit,i}+\epsilon}\right)^2+
\epsilon^2}
\]

Smooth force-space limiter:

\[
forceScale_i=\frac{1}{smoothMaxE(1,forceEnvelopeRatio_i,e_{force})}
\]

\[
F_{f,i}=forceScale_iF_{f,req,i}
\]

\[
F_{r,i}=forceScale_iF_{r,req,i}
\]

\[
forceLimiterActivity_i=1-forceScale_i
\]

Aggregate diagnostics:

\[
forceEnvelopeRatioMax=\max_i forceEnvelopeRatio_i
\]

\[
forceLimiterActivityMax=\max_i forceLimiterActivity_i
\]

`forceEnvelopeRatio` is a diagnostic and noise-scheduling input. It is not a command admissibility gate.

### 12.4 Expected failure signatures

| Signature | Likely interpretation |
|---|---|
| High lateral/yaw residuals in high-contact-slip arcs | Mean force surface too simple |
| Calibration turns require large persistent yaw residuals | Contact force law or yaw-loss model inadequate |
| Force-envelope ratio extreme in ordinary motion | Normal load, friction, or limiter scale wrong |
| Low-demand straights show biased residuals | Torque, straight friction, or timing wrong before contact model |
| Prediction unstable near `Vf=0` | Implementation violated velocity-space contract |

### 12.5 Follow-on options

1. Anisotropic force envelope with fitted longitudinal/lateral axes.
2. Low-rank empirical force surface over \((v_{rel,f},v_{rel,r},N,F_{driveReq})\).
3. Patch/side-specific stiffness and envelope parameters.
4. Correlated lateral/yaw residual driving noise if event timing is not deterministic.
5. Move yaw-loss behavior into patch-level forces if a separate yaw moment becomes confounded.

---

## 13. Yaw-moment and yaw-loss model

### 13.1 Model status

Flexible empirical yaw-moment correction family. This is not an in-place-turn branch.

### 13.2 Hard contract

Inputs:

```text
contact forces
contact geometry
v_rel_f_i, v_rel_r_i
N_i
yawRate
forceEnvelopeRatio_i / forceLimiterActivity_i
```

Outputs:

```text
yawMomentContact
yawMomentLoss
yawMomentNom
yawAccelRaw
```

### 13.3 Nominal yaw order

Contact moment:

\[
yawMomentContact=\sum_i(f_iF_{r,i}-r_iF_{f,i})
\]

The yaw-loss model consumes `yawMomentContact` and produces:

\[
yawMomentNom=yawMomentContact-yawMomentLoss
\]

Then:

\[
yawAccelRaw=\frac{yawMomentNom}{I_z}
\]

Do not compute `yawAccelRaw` from a pre-correction yaw moment if a yaw-loss model is active.

### 13.4 First-pass implementation

Use a smooth continuous yaw-loss moment:

\[
yawMomentLoss=\left(M_{yawC}s_{yaw}+B_{yaw}|yawRate|+K_{yawRel}contactRelRms+K_{yawLimit}forceLimiterActivityMax\right)sgnE(yawRate,yawRate_E)
\]

with:

\[
s_{yaw}=smoothStepE(yawContactSpeedRms,v_{yawLow},v_{yawHigh})
\]

This gives the model a continuous way to represent approximately constant yaw resistance without detecting a turn category. All parameters are empirical and versioned.

### 13.5 Expected failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Calibration turns fit but high-speed arcs degrade | Move yaw loss into contact model or reduce global component |
| High-speed arcs fit but calibration turns fail | Increase/reshape constant yaw-loss component |
| Yaw residual sign depends on side command | Repair torque asymmetry before yaw-loss retune |
| Same term improves one test family and hurts another | Split empirical decomposition or move to patch-level forces |

---

## 14. Ground-strike and acceleration envelope

### 14.1 Model status

Flexible acceleration/impact approximation family.

### 14.2 Hard contract

The model represents forward/reverse acceleration limiting and impact uncertainty. It must not mutate forces unless it recomputes all dependent moments and accelerations.

Outputs:

```text
AfRaw
AfClip
groundUse
impactIndicator
```

### 14.3 First-pass implementation

Use pure acceleration-envelope clipping first:

\[
AfClip=clipAsymE(AfRaw,-AfReverseLimit,AfForwardLimit,e_A)
\]

\[
AfNom=AfClip
\]

\[
ArNom=ArRaw
\]

\[
yawAccelNom=yawAccelRaw
\]

\[
groundUse=\frac{|AfRaw-AfClip|}{|AfRaw|+\epsilon}
\]

Do not scale contact forces in this first pass. Treat yaw/lateral consequences as residual/noise unless held-out logs show repeatable deterministic coupling.

### 14.4 Expected failure signatures and follow-ons

| Signature | Follow-on |
|---|---|
| Repeatable yaw bias during clipped straight acceleration | Force-consistent strike/load redistribution |
| Accel innovation spikes but pose/yaw acceptable | Event/noise treatment sufficient |
| Clip limit depends on speed/command/run state | Scheduled acceleration envelope |
| Impact-like wall-proximate events | Wall/tire springback covariance inflation |

---

## 15. Nominal body dynamics

After motor, load, contact force, yaw loss, and ground envelope:

\[
AfRaw=\frac{\sum_iF_{f,i}}{m_{eff,f}}
\]

\[
ArRaw=\frac{\sum_iF_{r,i}}{m_{eff,r}}
\]

\[
yawMomentContact=\sum_i(f_iF_{r,i}-r_iF_{f,i})
\]

\[
yawMomentNom=yawMomentContact-yawMomentLoss
\]

\[
yawAccelRaw=\frac{yawMomentNom}{I_z}
\]

Ground envelope produces:

\[
AfNom,\quad ArNom,\quad yawAccelNom
\]

Propagation accelerations include residuals:

\[
Af=AfNom+\Delta Af
\]

\[
Ar=ArNom+\Delta Ar
\]

\[
yawAccel=yawAccelNom+\Delta yawAccel
\]

Continuous-time derivative:

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

RK2 is acceptable for bring-up; RK4 is preferred for aggressive replay comparison. Euler is only for isolated low-demand tests.

---

## 16. Residual acceleration process model

### 16.1 Model status

Hard stochastic contract with flexible continuous scheduling inputs.

### 16.2 OU residual semantics

For \(q\in\{Af,Ar,yawAccel\}\):

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

### 16.3 First-pass noise schedule

Continuous schedule inputs:

```text
contactRelRms
lateralRelRms
yawContactSpeedRms
forceEnvelopeRatioMax
forceLimiterActivityMax
groundUse
driveSaturationIndex
driveAuthorityUncertainty
impactIndicator
encoderFaultIndicator
```

First-pass schedules:

\[
\sigma_{\Delta Af,ss}=\sigma_{Af,base,ss}+k_{Af,rel}contactRelRms+k_{Af,limit}forceLimiterActivityMax+k_{Af,ground}groundUse+k_{Af,drive}driveSaturationIndex^2+k_{Af,auth}driveAuthorityUncertainty+k_{Af,impact}impactIndicator+k_{Af,enc}encoderFaultIndicator
\]

\[
\sigma_{\Delta Ar,ss}=\sigma_{Ar,base,ss}+k_{Ar,lat}lateralRelRms+k_{Ar,yaw}yawContactSpeedRms+k_{Ar,limit}forceLimiterActivityMax+k_{Ar,ground}groundUse+k_{Ar,drive}driveSaturationIndex^2+k_{Ar,auth}driveAuthorityUncertainty+k_{Ar,impact}impactIndicator+k_{Ar,enc}encoderFaultIndicator
\]

\[
\sigma_{\Delta yawAccel,ss}=\sigma_{yaw,base,ss}+k_{yaw,rel}contactRelRms+k_{yaw,contact}yawContactSpeedRms+k_{yaw,limit}forceLimiterActivityMax+k_{yaw,drive}driveSaturationIndex^2+k_{yaw,auth}driveAuthorityUncertainty+k_{yaw,impact}impactIndicator+k_{yaw,enc}encoderFaultIndicator
\]

All coefficients are owned by `ResidualNoiseSchedule`, exposed only through validated owner APIs, and exported through the parameter metadata registry.

### 16.4 Event scalar first pass

`impactIndicator` is a continuous covariance-scheduling scalar, not a maneuver label.

First pass:

\[
jerkPlanar=\sqrt{jerkF^2+jerkR^2}
\]

\[
jerkIndex=smoothStepE(jerkPlanar,jerkLow,jerkHigh)
\]

\[
imuSaturation=\max_{channels}smoothStepE(|z_{channel}|,fsMarginLow,fsMarginHigh)
\]

\[
impactIndicator=smoothMaxE(groundUse,smoothMaxE(jerkIndex,imuSaturation,e),e)
\]

If vertical acceleration is available:

\[
impactIndicator\leftarrow smoothMaxE(impactIndicator,smoothStepE(|accelZOptional-g|,zLow,zHigh),e)
\]

Additional wall-touch or springback evidence may be added as versioned continuous inputs.

### 16.5 Residual injection convention

For replay comparability, document where OU innovation enters the one-step propagation:

```text
preferred: midpoint injection for RK2/RK4 if available
fallback: endpoint injection after deterministic OU decay
```

Target and host replay must use the same convention.

### 16.6 Full covariance propagation

Residual process noise must affect full state covariance, including velocity, yaw rate, pose, heading, and residual-to-motion cross-covariances. Encoder input covariance must propagate through the full nonlinear plant before force limiting and noise scheduling.

Compliant approaches:

```text
augmented prediction noise
sensitivity-mapped additive covariance
```

Sensitivity-mapped form:

\[
Q_x\leftarrow Q_x+G_\eta Q_\eta G_\eta^T+J_\omega R_\omega J_\omega^T
\]

Diagonal-only injection into the final three residual-state covariance entries is not compliant.

---

## 17. Measurement update structure

The production estimator should use sequential or small-block measurement updates. A full per-tick candidate measurement container may exist, but the default runtime path should not require a monolithic batch update.

Recommended blocks:

| Measurement | Block dimension |
|---|---:|
| Gyro yaw rate | 1 |
| Planar accelerometer | 2 |
| Wall sensor | 1, or 2 if explicitly correlated |
| Optional encoder-body pseudo-measurement | 2 |

Sequential updates are acceptable if measurement-noise correlations are handled by grouping or covariance inflation.

---

## 18. IMU measurement model

### 18.1 Gyro

\[
z_{gyro}=gyroYawCanonical-gyroBiasYawExt
\]

\[
h_{gyro}=yawRate
\]

Gyro covariance includes white noise, scale uncertainty, external-bias uncertainty, and timing/filter phase uncertainty. Coherent external-bias uncertainty must not be treated only as independent white sample noise if it is material.

### 18.2 Accelerometer

For each sigma point, use propagated acceleration:

\[
Af,\quad Ar,\quad yawAccel
\]

At IMU body-frame location \((r_{imu},f_{imu})\):

\[
a_{imu,f}=Af-yawAccel\,r_{imu}-yawRate^2f_{imu}
\]

\[
a_{imu,r}=Ar+yawAccel\,f_{imu}-yawRate^2r_{imu}
\]

Choose one bias contract and enforce it. Recommended first pass:

```text
z_accel channels are canonical, scale-corrected, but bias-included.
measurement function adds accelBiasFCal / accelBiasRCal.
```

\[
h_{Af}=a_{imu,f}+accelBiasFCal
\]

\[
h_{Ar}=a_{imu,r}+accelBiasRCal
\]

If the hardware layer already bias-removes acceleration, remove these bias terms from the measurement function. Do not allow both conventions.

Use low-pass/UI-path accelerometer output for the planar update. Use high-pass/slope/vertical/jerk indicators only for covariance scheduling and event detection.

### 18.3 Accelerometer covariance

\[
R_{accel}=R_{accel,base}+R_{timing}+R_{filter}+R_{impact}+R_{ground}+R_{forceLimit}+R_{saturation}
\]

Inflation terms are functions of continuous physical/event scalars, not maneuver labels.

### 18.4 Invalid IMU behavior

When `imuSampleValid == false`:

```text
skip gyro update
skip accelerometer update
do not update external gyro bias
use outage/degraded process policy if persistent
```

---

## 19. Encoder treatment and optional encoder-body pseudo-measurement

Encoders are drivetrain inputs. They are not ordinary pose, velocity, or yaw measurements.

Validated encoder rates feed:

```text
motor rate
back-EMF / torque prior
optional periodic torque correction
contact-relative velocity
force request / force limiting
continuous noise scheduling
```

### 19.1 Optional pseudo-measurement status

The encoder-body pseudo-measurement is not part of the default high-performance estimator path. It may be enabled only for conservative, exploration, or debug configurations after host replay proves benefit.

It must not classify the maneuver as rolling or nonrolling. It may only use continuous covariance based on contact-relative velocities, force-envelope ratio, ground use, launch/low-speed torque state, and encoder validity.

### 19.2 Optional first-pass form

If enabled, use calibrated effective rolling geometry:

\[
z_{encBody}=\begin{bmatrix}0\\0\end{bmatrix}
\]

\[
h_L=r_w\hat{wheelRate}_L-\left(Vf+\frac{b_{eff}}{2}yawRate\right)
\]

\[
h_R=r_w\hat{wheelRate}_R-\left(Vf-\frac{b_{eff}}{2}yawRate\right)
\]

`b_eff` is a calibrated effective rolling track width, not necessarily physical track width.

Covariance must include encoder covariance:

\[
R_{encBody,total}=R_{encBody,model}+r_w^2R_\omega
\]

Inflate or disable when contact-relative velocities, force-limit activity, ground use, launch torque, or encoder invalidity make the constraint untrustworthy.

---

## 20. Wall-sensor measurement model

### 20.1 Model status

Flexible response-space measurement family with hard geometry/extrinsic contract.

### 20.2 Hard contract

Each wall sensor has body-frame origin and unit look vector:

\[
(r_s,f_s),\quad \hat{u}_s=(u_{f,s},u_{r,s})
\]

Global origin:

\[
sensorX_s=Px+f_s\sin(heading)+r_s\cos(heading)
\]

\[
sensorY_s=Py+f_s\cos(heading)-r_s\sin(heading)
\]

Global look vector:

\[
u_{x,s}=u_{f,s}\sin(heading)+u_{r,s}\cos(heading)
\]

\[
u_{y,s}=u_{f,s}\cos(heading)-u_{r,s}\sin(heading)
\]

### 20.3 First-pass implementation

Use response-space prediction. Distance-space prediction is allowed only if the upstream wall pipeline already produces calibrated distance with known covariance.

Ray bundle requirements:

```text
nonnegative normalized ray weights
calibrated angular support
stable log-sum-exp soft-min
explicit no-hit handling
```

Effective distance:

\[
d_{eff,s}=-\frac{1}{\beta_s}\log\left(\sum_qw_{s,q}\exp(-\beta_sd_{s,q})\right)
\]

Implement with stable log-sum-exp. Exclude no-hit rays or assign a documented max-range surrogate. If all rays are no-hit, skip the update unless a fixed-size skipped-equivalent row is required.

Raw/log response examples:

\[
h_{frontRaw,s}=a_s-b_s\log(d_{eff,s}+d0_s)+c_sincidenceTerm_s
\]

\[
h_{sideRaw,s}=a_s+\frac{b_s}{(d_{eff,s}+d0_s)^{p_s}}+c_sincidenceTerm_s
\]

### 20.4 Invalid and saturated wall behavior

When `sensorValid == false`, skip the measurement row.

When `sensorSaturation == true`, either skip the row or inflate covariance via the saturation term. The chosen first-pass behavior must be replay-visible.

If the response curve is locally flat because of saturation or far-range insensitivity, inflate covariance or skip.

### 20.5 Follow-on options

1. Empirical response maps over distance and incidence.
2. Explicit no-hit likelihood model.
3. Shared-geometry covariance when multiple sensors see the same wall/edge/post.
4. Edge-aware covariance based on distance to segment endpoints and posts.

---

## 21. External stationary gyro-bias estimator

`gyroBiasYawExt` is external to the UKF.

It may be updated only during confirmed stationary intervals for startup or maintenance. Stationary detection is reserved for:

```text
gyro bias calibration
static sensor sanity checks
startup calibration
```

It must not select contact physics, yaw-loss physics, force scheduling, command admissibility, or turn behavior.

Stationary thresholds are calibration parameters. They are not part of the plant model and must not be used to distinguish turn types.

---

## 22. Validation, host replay, and logging boundary

### 22.1 Embedded logging boundary

The project already has robust logging of UKF-boundary inputs and commands. This spec does not define a raw logging dataset and does not require embedded logging of innovation/NIS/residual/covariance diagnostics at 1 kHz.

The embedded log must be sufficient to replay the estimator-boundary input stream:

```text
timestamps / phases
commands issued and command-validity convention
encoder samples and validity
IMU samples and validity/status
wall samples and validity/hypotheses
configuration / parameter / build identity
```

It does not need to carry validation metrics as per-tick telemetry.

### 22.2 Canonical validation mechanism

Validation is performed by replaying logged UKF-boundary input streams through the same estimator code built for host.

```text
Teensy:
    run estimator
    log UKF-boundary inputs + command/config identity

Host:
    replay same input stream through same estimator code
    enable diagnostic sinks
    compute validation reports
```

Do not create a shadow estimator implementation for validation.

If a diagnostic cannot be reconstructed from logged inputs plus configuration, treat that as a replay-contract gap before adding embedded trace fields.

### 22.3 Host-replay products

Host replay may compute:

```text
innovations
NIS
measurement accept/skip/gate decisions
residual state traces
covariance summaries
contact-relative velocities
force request ratios
force limiter activity
normal-load diagnostics
torque diagnostics
wall ray-bundle diagnostics
ablation reports
```

These are replay/debug products, not normal embedded telemetry requirements.

---

## 23. Calibration and tuning procedure

### 23.1 Foundational bring-up order

1. Coordinate/sign audit.
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

### 23.2 Parameter metadata registry

The parameter metadata registry is a replay, traceability, and calibration-readiness artifact. It is not a runtime parameter bag and must not be used by plant, measurement, or noise code as a shared scalar lookup table.

Every fitted parameter exported by an active model owner must have:

| Field | Requirement |
|---|---|
| Name | Exact code identifier |
| Unit | SI or dimensionless |
| Prior | Value and source |
| Bound type | Hard physical/safety bound or soft prior |
| Fit data | Which test logs may tune it |
| Validation data | Which held-out logs validate it |
| Residual policy | Enabled, weakened, disabled, or penalized while fitting |
| Confounds | Parameters that can mimic it |
| Version metadata | robot rev, tire/floor condition, firmware, estimator build |

Physical tuning and performance claims are blocked until this metadata registry exists for active fitted parameters and every active model owner exports its own parameter metadata.

### 23.3 Identifiability posture

Do not tune all submodels simultaneously from a mixed run. Fit aggregate effects first:

```text
effective timing / signs
effective torque
effective launch / straight friction
effective contact force
effective yaw loss
effective ground envelope
```

Only decompose them into physical sub-parameters when data supports the decomposition.

---

## 24. Validation tests and acceptance checks

### 24.1 Zero-forward-speed stability tests

Required tests:

```text
Sweep Vf through negative, zero, and positive small values.
Sweep Vr through negative, zero, and positive small values.
Sweep yawRate through realistic calibration-turn values.
Sweep wheel rates through stopped, same-direction, and opposed cases.
Assert all plant outputs are finite.
Assert force outputs are continuous.
Assert finite-difference sensitivities are bounded.
Assert no command is rejected solely from force-envelope ratio.
```

### 24.2 Sigma-point zero-crossing tests

```text
Set UKF mean near Vf = 0.
Set covariance so sigma points cross Vf = 0.
Run prediction.
Assert finite state mean and covariance.
Assert no plant branch changes semantic meaning across the zero crossing.
```

### 24.3 Command-admissibility tests

```text
Commands within drive-layer motion limits are not rejected by force-envelope diagnostics.
Configurable motion limits still clip velocity/acceleration/yawRate/yawAccel as drive policy.
Hardware, current, thermal, and explicit safety limits still apply.
```

### 24.4 Feedforward performance tests

Test feedforward primarily in high-performance contact conditions:

```text
calibration angle turns
high-lateral-acceleration arcs
full-bore straight acceleration
full-bore braking
precision wall-proximate turns
```

Low-demand rolling-style logs remain sanity and characterization data, not the primary performance pass/fail target.

### 24.5 Estimator validation reports

Host replay should report, by commanded test segment:

```text
pose/yaw error where reference is available
gyro and accelerometer innovation statistics
wall innovation statistics
residual mean/RMS/autocorrelation
force-envelope and contact-relative-velocity diagnostics
normal-load diagnostics
torque/drive-authority diagnostics
covariance consistency checks
ablation results
```

Numeric thresholds are versioned from characterization data unless they are hardware safety limits or physical impossibility checks.

### 24.6 Required ablations

Replay with:

```text
residual driving noise reduced
accelerometer updates weakened
wall updates disabled
optional encoder-body pseudo-measurement disabled
torque correction disabled
ground-envelope covariance disabled
```

A model is not accepted if it only works because residuals or covariance inflation hide systematic nominal-plant errors in low-demand conditions.

---

## 25. Implementation checklist

The target implementation is ready for physical tuning only when:

```text
9-state vector is implemented without Bgz/wheel/current/slip states
coordinate signs match the project convention
prediction timing derives from timestamp/effective time
commands have a documented validity convention
encoder rates are drivetrain inputs, not body-motion measurements
contact-relative velocity variables are finite at Vf = 0
no speed-normalized slip/curvature/ICR variables drive plant physics
no runtime turn category is required for plant behavior
no traction/force-envelope diagnostic gates performance commands
direct motor/driver mean model is implemented with correction hooks
normal-load first pass is executable
yaw-loss model is continuous and category-free
ground-strike envelope is internally consistent
residual OU semantics are exact and replay-identical
residual and encoder uncertainty affect full prediction covariance
invalid IMU/wall/encoder samples skip or degrade correctly
wall ray-bundle numerics are stable
host replay runs the same estimator code as target
runtime model coefficients are owned by typed model classes, not public-field parameter bags
typed submodel owner classes exist for motor/load/contact/yaw/ground/residual/wall models
no global public-field parameter bag exists
no runtime lookup of arbitrary scalar coefficients from a shared registry exists
CommandVector is used as the canonical left/right PWM command type
VehicleState is used as the tick context rather than generated submodel input bags
no normative MotorPhase, WheelAngularRate, MotorTorqueInput, or MotorTorqueOutput API exists
classes above moderate parameter-bag risk satisfy or explicitly justify the behavior-to-field heuristic
parameter metadata registry is complete for active fitted parameters
```

---

## 26. Glossary of target diagnostic terms

| Term | Meaning |
|---|---|
| `v_rel_f_i`, `v_rel_r_i` | Per-contact relative velocity in forward/right directions; primary contact variables |
| `contactRelRms` | Load-weighted RMS contact-relative speed |
| `lateralRelRms` | Load-weighted RMS lateral contact-relative speed |
| `yawContactSpeedRms` | Load-weighted RMS contact speed induced by yaw rate and patch geometry |
| `forceEnvelopeRatio_i` | Requested-force magnitude relative to current empirical force envelope |
| `forceLimiterActivity_i` | Degree to which force limiter is active; diagnostic/noise input only |
| `groundUse` | Degree of forward acceleration-envelope clipping |
| `driveAuthorityUncertainty` | Torque authority uncertainty from unmeasured supply/thermal/driver effects |
| `impactIndicator` | Continuous event/covariance scalar from vertical acceleration, jerk, saturation, ground use, or wall-touch evidence |

Forbidden as normative estimator terms:

```text
isInPlaceTurn
isRolling
isStickSlip
pivotScrub mode
traction feasible command
stationary-selected turn model
```

---

## 27. Summary

The target estimator is a 9-state planar body UKF with encoder-derived drivetrain inputs and contact-relative-velocity plant physics. It explicitly rejects the legacy failure modes: gyro bias as a UKF state, traction-gated control authority, speed-normalized slip math, zero-speed-singular contact models, runtime turn taxonomy, invented submodel input/output carrier bags, and embedded validation-diagnostic logging requirements that exceed the existing input-log/replay architecture.

The model is strict where ambiguity would break implementation: state, signs, timing, measurement semantics, input validity, covariance semantics, and contact-relative-velocity definition. It remains flexible where the physical robot must be learned from logs: motor/driver torque corrections, contact force law, normal load, yaw loss, ground strike, wall response, and noise schedules.

The test of the design is not whether it resembles the current implementation. The test is whether same-code host replay shows stable, representative behavior across the robot’s actual high-performance operating envelope.

---

## References

- `Hardware.md`
- `Physical effects and operating envelope.md`
- `Symbology Glossary.md`
- `EN_1717_SR_DFF.pdf`
- `EN_IE2-1024_DFF.pdf`
- `drv8871.pdf`
- `lsm6dsv16x.pdf`
- `ukf_plant_measurement_model_complete_fixed.md`
- `ukf_flexible_model_family_audit.md`
- `ukf_spec_remaining_concerns_for_author.md`
- `ukf_current_implementation_ahead_of_spec.md`
