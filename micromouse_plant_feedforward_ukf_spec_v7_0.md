# Micromouse Plant Model, Feedforward, and 9-State SR-UKF Specification — Robust Single-Filter Proposal (V7.0)

## 1. Purpose

This document specifies a **single-filter, 9-state, square-root UKF**, a matching **runtime feedforward / inverse path**, and a **bank-split plant model** for the Micromouse vehicle. The design goal is to preserve the cost discipline and estimator architecture of the present direction while fixing the remaining regime gap at **very high yaw rate with very low forward speed** and improving robustness through slip onset, sustained saturation, and re-grip.

This proposal is intentionally compatible with the strong parts of the attached V6.2 direction:

- single 9-state SR-UKF,
- explicit wheel/body authority separation,
- exact stationary branch,
- applied bank torque as the estimator process input,
- staged raw planar-accelerometer reintegration,
- precursor metrics,
- transient-contact memory,
- re-grip dwell,
- and runtime cost discipline.

It extends that base in three important ways:

1. it adds an explicit **pivot-scrub yaw branch** for the `|R| > 18 rad/s` and `|U| < 0.01 m/s` regime,
2. it adds an explicit **mixed pivot-roll blending law** for the `|R| > 18 rad/s` and `0.2 < |U| < 0.6 m/s` regime,
3. it adds a concrete **sensor-aligned noise specification** for the project IMU and encoder path.

The result is still a **single-filter architecture**, but it is no longer dependent on rolling-slip-only assumptions in the low-translation / high-yaw corner of the operating envelope.

---

## 2. Project coordinate convention

This specification uses the project convention:

- `+X` = right,
- `+Y` = forward,
- `+yaw` = clockwise.

The state names are:

- `positionX`
- `positionY`
- `headingYaw`
- `forwardSpeed`
- `rightSpeed`
- `yawRate`
- `leftBankWheelSpeed`
- `rightBankWheelSpeed`
- `gyroZBias`

The 9-state estimator vector is:

```text
x = [positionX, positionY, headingYaw,
     forwardSpeed, rightSpeed, yawRate,
     leftBankWheelSpeed, rightBankWheelSpeed,
     gyroZBias]^T
```

Heading shall always be normalized to `(-π, π]`.

---

## 3. Design goals

The system shall:

1. remain within the present runtime envelope of the example specification,
2. use a single 9-state SR-UKF,
3. handle all valid command requests cleanly,
4. support the full bounded operating envelope in forward speed and yaw rate,
5. additionally obey the anisotropic traction limit,
6. remain accurate in the following mandatory regimes:
   - certified stationary,
   - `|yawRate| > 18 rad/s` with `|forwardSpeed| < 0.01 m/s`,
   - `|yawRate| > 18 rad/s` with `0.2 < |forwardSpeed| < 0.6 m/s`,
   - rolling traction-limited turn,
7. preserve estimator stability through slip onset and re-grip,
8. avoid hidden wheel-to-body direct overwrite outside exact stationary lock,
9. use noise values and schedules that match the actual project sensor chain,
10. preserve a physically meaningful forward and inverse model for offline audit and online feedforward.

---

## 4. Executive architecture

### 4.1 Top-level structure

The system shall contain these runtime subsystems:

1. **PlantModel**
   - authoritative forward model for control, offline replay, and audit,
   - supports rolling combined-slip operation,
   - supports pivot scrub yaw operation,
   - supports mixed pivot-roll blending,
   - supports anisotropic traction projection.

2. **EstimatorPredictModel**
   - UKF propagation model,
   - same state and same geometry as the PlantModel,
   - same input contract,
   - same operating-region logic,
   - smoother than the control plant around low-speed transitions.

3. **FeedforwardInverseModel**
   - maps desired body motion into bank torque commands,
   - handles pivot, mixed, and rolling regimes cleanly,
   - returns the closest feasible command when the request is outside the traction envelope.

4. **SquareRootUKF9**
   - 9-state SR-UKF,
   - spherical-simplex sigma-point set,
   - exact stationary branch,
   - constrained direct wheel update,
   - side-specific closure and slip handling,
   - staged IMU measurement rollout.

### 4.2 Critical architectural rule

The direct wheel-speed update shall affect only the wheel-speed states:

```text
K_nav,wheel = 0
```

The encoders are never permitted to directly overwrite body speed, body yaw rate, or pose during moving operation.

All wheel-to-body influence must flow through explicit and auditable mechanisms:

- process propagation,
- closure pseudo-measurements,
- adaptive lateral pseudo-measurement,
- side-specific covariance schedules,
- onset holdoff,
- re-grip recovery,
- and trusted map or wall updates.

---

## 5. Operating regions

This proposal uses **one filter** and **one composite plant family**, but the plant and feedforward logic shall explicitly distinguish the following operating regions.

### 5.1 Certified stationary region

This region is active only when all of the following are true with hysteresis and dwell:

- `|leftBankWheelSpeed| < stationaryWheelSpeedEnter`
- `|rightBankWheelSpeed| < stationaryWheelSpeedEnter`
- `|yawRateCorrected| < stationaryYawRateEnter`
- `|bodyPlaneAccel| < stationaryAccelEnter`
- `|leftCommand| < stationaryCommandEnter`
- `|rightCommand| < stationaryCommandEnter`
- no slip holdoff is active
- no re-grip dwell is active

### 5.2 Pivot-scrub yaw region

This is the regime that V6.2 does not fully solve and this proposal explicitly does.

It is active when all of the following are true with hysteresis:

- `|forwardSpeed| < pivotForwardSpeedEnter`
- `|yawRate| > pivotYawRateEnter`
- `pivotWeight > 0.8`

Nominal initial values:

- `pivotForwardSpeedEnter = 0.03 m/s`
- `pivotForwardSpeedExit  = 0.06 m/s`
- `pivotYawRateEnter      = 14 rad/s`
- `pivotYawRateExit       = 10 rad/s`

### 5.3 Mixed pivot-roll region

This region covers the mandatory regime:

- `|yawRate| > 18 rad/s`
- `0.2 < |forwardSpeed| < 0.6 m/s`

Here neither a pure rolling-slip model nor a pure pivot-scrub model is sufficient. The model shall use a **smooth composite force law** with both contributions present.

### 5.4 Rolling adherent region

This region is ordinary rolling operation below the traction limit, with small closure residuals and no persistent saturation evidence.

### 5.5 Rolling traction-limited region

This region is active when either:

- the requested combined longitudinal / lateral force exceeds the available envelope,
- the grip-law precursor metric approaches one,
- or persistent wheel/body disagreement indicates sustained edge operation.

### 5.6 Motion-angle blend variable

To unify pivot, mixed, and rolling operation, define the motion-angle quantity:

```math
motionAngle = atan2(|forwardSpeed|, |yawRate| * pivotRadiusEffective + speedRegularization)
```

where:

- `pivotRadiusEffective` is the effective radius that maps pure yaw motion into local scrub speed,
- `speedRegularization` is a small positive constant.

Then define:

```math
rollingWeight = smoothstep(motionAnglePivotToRollStart, motionAnglePivotToRollEnd, motionAngle)
pivotWeight   = 1 - rollingWeight
```

Nominal initial values:

- `motionAnglePivotToRollStart = 0.20 rad`
- `motionAnglePivotToRollEnd   = 0.55 rad`

This is the required mechanism that makes the low-speed high-yaw region robust. The model is no longer forced to pretend that all high-yaw motion is a rolling-slip problem.

---

## 6. Plant model

## 6.1 Geometry and body kinematics

For any point with body-frame coordinates `[pointX, pointY]`, the body-frame point velocity is:

```math
pointRightSpeed   = rightSpeed + yawRate * pointY
pointForwardSpeed = forwardSpeed - yawRate * pointX
```

For the left and right wheel-bank centers at `pointX = -trackWidth/2` and `pointX = +trackWidth/2`:

```math
leftBankForwardGroundSpeed  = forwardSpeed + yawRate * trackWidth / 2
rightBankForwardGroundSpeed = forwardSpeed - yawRate * trackWidth / 2
```

These are the same forward closure relationships used by the estimator and feedforward.

## 6.2 Drivetrain input contract

The plant and estimator input shall be **applied bank torque**, not raw command:

```text
u_process = [appliedLeftBankTorque, appliedRightBankTorque]^T
```

The applied bank torque estimator shall include, at minimum:

- requested motor command,
- battery voltage or estimated bus voltage,
- back-EMF from bank wheel speed,
- winding resistance,
- torque constant,
- current limit state,
- drivetrain efficiency,
- bank friction torque.

If current is not measured directly, the torque estimate shall still be algebraic and shall not reduce to raw PWM duty.

## 6.3 Wheel-bank rotational dynamics

The wheel-bank speed dynamics are:

```math
leftBankWheelAccel = (appliedLeftBankTorque - leftBankContactTorque - leftBankFrictionTorque) / wheelBankInertia
rightBankWheelAccel = (appliedRightBankTorque - rightBankContactTorque - rightBankFrictionTorque) / wheelBankInertia
```

The friction torque term shall include:

- rolling drag,
- viscous drag,
- static breakaway torque in the low-wheel-speed window.

## 6.4 Rolling contact branch

The rolling branch is the standard combined-slip contact model used when translation is materially present.

For each tire contact:

1. compute local contact speeds,
2. compute slip ratio using low-speed regularization,
3. compute slip angle using low-speed regularization,
4. compute pure-slip longitudinal and lateral forces,
5. combine them using a smooth anisotropic traction projector,
6. accumulate force and yaw moment.

### 6.4.1 Longitudinal slip ratio

For each contact:

```math
slipRatio = (wheelSurfaceSpeed - contactForwardSpeed) /
            max(|contactForwardSpeed|, rollingSpeedRegularization)
```

### 6.4.2 Slip angle

For each contact:

```math
slipAngle = atan2(contactRightSpeed, max(|contactForwardSpeed|, rollingSpeedRegularization))
```

### 6.4.3 Pure-slip rolling forces

A modified Fiala or Dugoff-class law is acceptable. The required property is:

- linear near zero slip,
- smooth saturation,
- continuous first derivative through the operating region,
- separate longitudinal and lateral stiffness,
- separate longitudinal and lateral traction limits.

A compliant generic form is:

```math
longitudinalForcePure = longitudinalForceLaw(slipRatio, normalLoad, wheelParameters)
lateralForcePure      = lateralForceLaw(slipAngle, normalLoad, wheelParameters)
```

### 6.4.4 Anisotropic traction projection

The vehicle does not have the same longitudinal and lateral limit. Therefore the projection shall be anisotropic:

```math
combinedUtilization = sqrt(
    (longitudinalForcePure / longitudinalForceLimit)^2 +
    (lateralForcePure      / lateralForceLimit)^2 )
```

If `combinedUtilization <= 1`, keep the pure-slip forces.

If `combinedUtilization > 1`, project smoothly:

```math
projectionScale = smoothProject(combinedUtilization)
longitudinalForce = projectionScale * longitudinalForcePure
lateralForce      = projectionScale * lateralForcePure
```

The projection function shall be smooth and monotone and shall not introduce hot-path discontinuities.

## 6.5 Pivot-scrub yaw branch

This branch is mandatory.

The yaw-only region is not well modeled by rolling slip angle alone because the dominant physics there is lateral scrub and breakaway of the tire contact patch under near-zero body translation. The plant shall therefore include a dedicated pivot-scrub yaw moment law.

### 6.5.1 Effective scrub speed

Define an effective scrub speed at the tire patch:

```math
effectiveScrubSpeed = |yawRate| * pivotRadiusEffective
```

### 6.5.2 Static and rolling pivot moments

Define:

```math
pivotBreakawayYawMoment = pivotBreakawayYawMomentLaw(normalLoad, fanDownforce)
pivotRollingYawMoment   = pivotRollingYawMomentLaw(effectiveScrubSpeed, normalLoad, fanDownforce)
```

The branch shall have distinct breakaway and post-breakaway levels. A compliant form is:

```math
pivotYawMomentMagnitude =
    pivotBreakawayYawMoment * exp(-effectiveScrubSpeed / pivotBreakawayDecaySpeed)
    + pivotRollingYawMoment * (1 - exp(-effectiveScrubSpeed / pivotBreakawayDecaySpeed))
```

The signed yaw scrub moment is:

```math
pivotYawMoment = sign(yawRateOrRequestedYawAccel) * pivotYawMomentMagnitude
```

### 6.5.3 Bank torque mapping for pure pivot

In pure pivot, the feedforward torque split is driven by the required yaw moment and wheel inertia, not by forward traction demand.

With effective bank moment arm `trackWidth / 2`:

```math
requiredBankForceDifference = requiredYawMoment / (trackWidth / 2)
requiredBankTorqueDifference = wheelRadius * requiredBankForceDifference
```

Then add wheel-inertia and drivetrain-friction compensation.

This is the mechanism that prevents the low-output yaw-only failure mode.

## 6.6 Mixed pivot-roll composite branch

The mixed high-yaw / moderate-forward-speed region shall use a weighted sum of rolling and pivot contributions.

### 6.6.1 Composite yaw moment

```math
compositeYawMoment =
    rollingWeight * rollingYawMoment
    + pivotWeight * pivotYawMoment
```

### 6.6.2 Composite longitudinal force

```math
compositeLongitudinalForce = rollingWeight * rollingLongitudinalForce
```

The pivot branch does not directly add longitudinal traction demand. It only adds yaw scrub resistance and the associated bank differential demand.

### 6.6.3 Composite lateral force

The lateral body force is still supplied by the rolling branch, but the confidence in lateral rolling closure must weaken as `pivotWeight` rises.

### 6.6.4 Required property

The composite law shall be smooth in:

- `forwardSpeed`,
- `yawRate`,
- `motionAngle`,
- slip ratio,
- slip angle,
- normal load,
- transient-contact memory.

## 6.7 Normal load model

At minimum, each contact normal load shall be:

```math
normalLoad = staticLoad + longitudinalLoadTransfer + lateralLoadTransfer + fanDownforceContribution
```

If compliance is important, it shall be represented by effective coefficients rather than extra propagated states.

## 6.8 Body dynamics

The body equations are:

```math
forwardAccel = totalForwardForce / effectiveLongitudinalMass + yawRate * rightSpeed
rightAccel   = totalRightForce   / effectiveLateralMass   - yawRate * forwardSpeed
yawAccel     = totalYawMoment / yawInertia
```

Note the signs are for the project convention `+X right`, `+Y forward`, `+yaw clockwise`.

## 6.9 Continuous-to-discrete propagation

Use a single semi-implicit predict step per control cycle:

1. evaluate loads and forces from the current state,
2. update wheel speeds,
3. update `forwardSpeed`, `rightSpeed`, `yawRate`,
4. update `headingYaw`,
5. update `positionX`, `positionY` from the updated body velocity.

Online predictor substepping is not required for runtime acceptance.

---

## 7. Feedforward and inverse model

## 7.1 Request handling policy

The external request is the desired body motion:

```text
requestedForwardSpeed
requestedYawRate
```

The system shall first clamp the request to the independent kinematic envelope:

```math
requestedForwardSpeed = clamp(requestedForwardSpeed, -maxForwardSpeed, +maxForwardSpeed)
requestedYawRate      = clamp(requestedYawRate,      -maxYawRate,      +maxYawRate)
```

Then compute desired accelerations over the configured response horizon:

```math
desiredForwardAccel = (requestedForwardSpeed - estimatedForwardSpeed) / responseTime

desiredYawAccel = (requestedYawRate - estimatedYawRate) / responseTime
```

Feedback correction shall be applied in **body-motion space before inverse mapping**, not in command space after feedforward.

## 7.2 Regime-specific inverse path

### 7.2.1 Certified stationary request

If the request is effectively zero and the system is in certified stationary lock, command zero bank torque except for the bounded breakaway-preload logic needed to hold the state estimator and controller coherent.

### 7.2.2 Pivot request

If `pivotWeight` is dominant, solve from the yaw branch first:

```math
requiredYawMoment = yawInertia * desiredYawAccel + pivotYawMoment + yawDampingMoment
```

Then compute the required differential bank torque and add wheel-bank inertial compensation.

### 7.2.3 Rolling request

If `rollingWeight` is dominant, solve the rolling inverse model from desired body longitudinal force and desired yaw moment using the rolling contact branch and the anisotropic traction projector.

### 7.2.4 Mixed request

If neither branch dominates, solve a convex composite inverse:

```math
requiredYawMoment =
    rollingWeight * requiredRollingYawMoment
    + pivotWeight * requiredPivotYawMoment
```

```math
requiredForwardForce = rollingWeight * requiredRollingForwardForce
```

## 7.3 Feasibility projection

The inverse solve shall never emit an undefined or discontinuous command. If the requested motion is outside the feasible region, the solver shall return the closest feasible command under the present plant model.

A compliant method is homothetic projection of the body-demand vector:

```math
demandVector = [requiredForwardForce, requiredYawMoment]^T
issuedVector = λ * demandVector
```

with the largest `λ ∈ [0, 1]` that satisfies all of the following:

- bank torque limits,
- current limits,
- battery-voltage limits,
- anisotropic traction limits,
- pivot scrub limits,
- wheel-speed dependent back-EMF limits.

The solver shall report:

- issued command,
- predicted achieved forward acceleration,
- predicted achieved yaw acceleration,
- traction-limited flag,
- electrical-limited flag,
- and convergence flag.

## 7.4 Runtime inversion of memory-dependent behavior

The runtime inverse shall **not** attempt to invert the full history-dependent transient-contact memory law. Runtime inversion shall use either:

1. the nominal grip branch only, or
2. a frozen-memory approximation using the current held memory state.

The full memory-dependent law remains available for forward audit and offline replay.

---

## 8. 9-state SR-UKF

## 8.1 Sigma-point strategy

The estimator shall use:

- square-root covariance,
- spherical-simplex sigma points,
- QR-based square-root predict,
- Cholesky update or downdate style correction,
- heading normalization after predict and after update.

## 8.2 Exact stationary branch

This is the only hard branch in the estimator.

### 8.2.1 State update at certified rest

At certified rest:

```math
positionX_next = positionX
positionY_next = positionY
headingYaw_next = headingYaw
```

with contracting non-frozen dynamic states:

```math
forwardSpeed_next      = stationaryForwardDecay * forwardSpeed      + noiseForward
rightSpeed_next        = stationaryRightDecay   * rightSpeed        + noiseRight
yawRate_next           = stationaryYawDecay     * yawRate           + noiseYaw
leftBankWheelSpeed_next  = stationaryWheelDecay * leftBankWheelSpeed  + noiseLeftWheel
rightBankWheelSpeed_next = stationaryWheelDecay * rightBankWheelSpeed + noiseRightWheel
gyroZBias_next         = gyroZBias + noiseGyroBias
```

### 8.2.2 Stationary pseudo-measurement

Use:

```math
stationaryMeasurement = [0, 0, 0, 0, 0]^T
stationaryPrediction  = [forwardSpeed,
                         rightSpeed,
                         yawRate,
                         leftBankWheelSpeed,
                         rightBankWheelSpeed]^T
```

### 8.2.3 Release inflation

On exit from stationary lock, inflate covariance on:

- `forwardSpeed`,
- `rightSpeed`,
- `yawRate`,
- `leftBankWheelSpeed`,
- `rightBankWheelSpeed`.

A small heading inflation is allowed if justified by logs.

## 8.3 Moving branch

Outside exact stationary lock, the estimator shall propagate with the smooth `EstimatorPredictModel`, not the raw control plant.

The estimator branch shall not contain:

- controller stop blending,
- snap-to-zero cleanup,
- hard sign flips,
- hidden wheel-to-body overwrite.

## 8.4 Direct measurements

### 8.4.1 Wheel speed measurement

Use:

```math
wheelMeasurement = [leftBankWheelSpeed, rightBankWheelSpeed]^T + noiseWheel
```

with constrained gain so the direct update only touches the wheel-speed states.

### 8.4.2 Gyroscope measurement

Use the bias-in-state model:

```math
gyroMeasurement = yawRate + gyroZBias + noiseGyro
```

The external bias anchor is permitted only for startup seeding, stationary support, sanity checks, and bounded regularization. It shall not replace the state bias inside the measurement equation.

### 8.4.3 Planar accelerometer measurement

Use raw synchronized planar accelerometer samples with the project convention:

```math
predictedRightAccelAtImu = rightAccel + forwardSpeed * yawRate + yawAccel * imuY - yawRate^2 * imuX
predictedForwardAccelAtImu = forwardAccel - rightSpeed * yawRate - yawAccel * imuX - yawRate^2 * imuY
```

The rollout order shall be:

1. forward axis first,
2. lateral axis second,
3. both axes only after telemetry and timing audit pass.

### 8.4.4 Trusted map and wall updates

Trusted wall and map updates remain admissible even during slip suppression, subject to their own innovation tests.

## 8.5 Closure pseudo-measurements

Define wheel-derived references:

```math
wheelReferenceForwardSpeed = wheelRadiusEffective * (leftWheelMeasured + rightWheelMeasured) / 2
wheelReferenceYawRate      = wheelRadiusEffective * (leftWheelMeasured - rightWheelMeasured) / trackWidthEffective
```

Define closure residuals:

```math
leftClosureResidual  = wheelRadiusEffective * leftWheelMeasured  - (forwardSpeed + yawRate * trackWidthEffective / 2)
rightClosureResidual = wheelRadiusEffective * rightWheelMeasured - (forwardSpeed - yawRate * trackWidthEffective / 2)
```

Use the pseudo-measurement:

```math
closureMeasurement = [0, 0]^T
closurePrediction  = [leftClosureResidual, rightClosureResidual]^T
```

Closure authority shall be **side-specific**.

## 8.6 Adaptive lateral pseudo-measurement

Use:

```math
lateralMeasurement = 0
lateralPrediction  = rightSpeed
```

with covariance scheduled from:

- rolling or pivot blend state,
- lateral utilization,
- closure disagreement,
- yaw-response deficit,
- launch window,
- holdoff state,
- re-grip recovery,
- transient-contact memory.

This measurement shall be strong near launch and in low-speed benign motion, and weak in confirmed edge operation.

## 8.7 Innovation consistency, holdoff, and re-grip

Each enabled update group shall have:

- green band: nominal covariance,
- amber band: inflated covariance,
- red band: reject or hold off.

Slip-onset triggers shall include:

- rapid growth of closure residual,
- wheel/body/IMU sign inconsistency,
- precursor utilization near one,
- yaw-response deficit,
- abrupt wheel acceleration mismatch.

Re-grip triggers shall include:

- recent slip history,
- rapid collapse of a closure residual,
- concurrent wheel deceleration or jerk,
- concurrent IMU transient.

During re-grip recovery dwell, the system shall:

- keep suspect-side closure weak,
- keep soft odometry off,
- relax the lateral pseudo-measurement only gradually,
- keep the edge-shape modifier active until hysteresis clears.

---

## 9. Transient-contact memory and edge-shape logic

The plant and estimator schedules shall include deterministic per-bank memory variables:

```math
leftBankMemory, rightBankMemory ∈ [0, 1]
```

These are not propagated UKF states.

A compliant update law is:

```math
leftBankMemoryNextRaw = memoryPole * leftBankMemory
                      + memorySlipGain * leftSlipEvidence
                      + memoryRegripGain * leftRegripEvidence
```

```math
leftBankMemoryNext = clip(leftBankMemoryNextRaw - memoryClearGain * leftClearEvidence, 0, 1)
```

and similarly for the right side.

Rise shall be faster than decay.

The memory terms shall affect:

- edge-shape modifier strength,
- closure covariance,
- lateral pseudo-measurement covariance,
- soft-odometry availability,
- re-grip recovery duration.

---

## 10. Noise model aligned to project sensors

## 10.1 IMU measurement noise

The project IMU is the **LSM6DSV16X**. In high-performance mode, the datasheet gives:

- gyroscope noise density = `2.8 mdps/√Hz`,
- accelerometer noise density = `60 µg/√Hz`,
- gyroscope zero-rate temperature coefficient = `±0.006 dps/°C` typical. 

For a 1 kHz estimator using raw synchronized samples and a practical effective bandwidth near `ODR / 2 = 500 Hz`, initialize the measurement noise as follows.

### 10.1.1 Gyroscope measurement noise

Convert the gyroscope density:

```math
gyroNoiseDensity = 2.8e-3 deg/s/√Hz × π / 180
                 = 4.8869e-5 rad/s/√Hz
```

Then:

```math
gyroMeasurementSigma = gyroNoiseDensity × sqrt(500)
                     = 1.09e-3 rad/s
```

Use the initial covariance:

```math
R_gyro = (1.09e-3)^2 = 1.19e-6 (rad/s)^2
```

### 10.1.2 Accelerometer measurement noise

Convert the accelerometer density:

```math
accelNoiseDensity = 60e-6 × 9.80665
                  = 5.8840e-4 m/s^2/√Hz
```

Then:

```math
accelMeasurementSigma = accelNoiseDensity × sqrt(500)
                      = 1.32e-2 m/s^2
```

Use the initial covariance on each planar axis:

```math
R_accelForward = R_accelRight = (1.32e-2)^2 = 1.74e-4 (m/s^2)^2
```

These are the correct **raw-sample starting values**. Do not silently inflate them by an arbitrary order of magnitude; only inflate from telemetry, gating, and model mismatch evidence.

## 10.2 Gyro bias process noise

The project behavior reported for this IMU is that the gyro bias is effectively constant over tens of minutes. That matches the device’s small typical temperature coefficient. Therefore the gyro bias process noise shall be **near-zero and stationary-only**.

Use:

```math
Q_gyroBiasMoving = 0
```

and in certified stationary lock use a very small random walk for numerical flexibility:

```math
gyroBiasSigmaPerSqrtSecond = 2.0e-6 rad/s/√s
Q_gyroBiasStationary(dt) = (gyroBiasSigmaPerSqrtSecond^2) × dt
```

At `dt = 0.001 s`:

```math
Q_gyroBiasStationary = 4.0e-15 (rad/s)^2 per tick
```

If stationary Allan analysis on project logs indicates an even smaller value, reduce it. The important rule is structural:

- no moving random walk on gyro bias,
- only tiny stationary drift allowance.

## 10.3 Encoder measurement noise

The encoder path is:

- `4096` counts per motor revolution,
- gear ratio = `56 / 17` motor revolutions per wheel revolution.

Therefore:

```math
countsPerWheelRevolution = 4096 × 56 / 17 = 13492.706
```

With wheel radius `0.0125 m`:

```math
wheelDistancePerCount = 2π × 0.0125 / 13492.706
                      = 5.821e-6 m
```

For a raw one-tick finite-difference wheel-speed estimate at `dt = 0.001 s`, one count corresponds to:

```math
wheelSpeedStep = 2π / (13492.706 × 0.001)
               = 0.46567 rad/s
```

Uniform quantization gives:

```math
wheelMeasurementSigmaQuant = wheelSpeedStep / sqrt(12)
                           = 0.13443 rad/s
```

Use the raw-count initial covariance:

```math
R_wheelRaw = (0.13443)^2 = 1.81e-2 (rad/s)^2
```

If the implementation uses a longer estimation window or reciprocal-period timing capture, replace this with the actual estimator-path variance. Do not pretend the direct 1 kHz count-difference signal is quieter than it is.

## 10.4 Baseline process noise schedule

Use acceleration-equivalent disturbance terms instead of ad hoc direct-state noise whenever practical.

Nominal initial standard deviations for the moving branch:

```text
longitudinalDisturbanceSigma = 0.35 m/s^2
lateralDisturbanceSigma      = 0.45 m/s^2
yawDisturbanceSigma          = 18.0 rad/s^2
leftWheelAccelSigma          = 45.0 rad/s^2
rightWheelAccelSigma         = 45.0 rad/s^2
```

These are base values only. They are then scheduled upward by precursor utilization, closure disagreement, pivot weight, and re-grip evidence.

## 10.5 Mandatory adaptive schedules

Define the grip precursor per bank from the unprojected rolling branch:

```math
leftBankUtilization  = max(leftFrontUtilization,  leftRearUtilization)
rightBankUtilization = max(rightFrontUtilization, rightRearUtilization)
```

Define closure-based schedule variables:

```math
commonClosureMismatch = clip((|leftClosureResidual + rightClosureResidual| / 2) / commonClosureScale, 0, 1)

differentialClosureMismatch = clip((|leftClosureResidual - rightClosureResidual| / 2) / differentialClosureScale, 0, 1)

yawMismatch = |wheelReferenceYawRate - yawRate| / (yawMismatchScale + |wheelReferenceYawRate|)
```

Then schedule the disturbance terms:

```math
Q_forward = Q_forward_base ×
            (1 + k_forward_common × commonClosureMismatch^2)
```

```math
Q_right = Q_right_base ×
          (1 + k_right_util × max(leftBankUtilization, rightBankUtilization)^2
             + k_right_yaw  × yawMismatch^2
             + k_right_pivot × pivotWeight^2)
```

```math
Q_yaw = Q_yaw_base ×
        (1 + k_yaw_util × max(leftBankUtilization, rightBankUtilization)^2
           + k_yaw_diff × differentialClosureMismatch^2
           + k_yaw_pivot × pivotWeight^2)
```

```math
Q_leftWheel = Q_leftWheel_base ×
              (1 + k_left_pre  × leftBankUtilization^2
                 + k_left_edge × leftBankMemory^2)
```

```math
Q_rightWheel = Q_rightWheel_base ×
               (1 + k_right_pre  × rightBankUtilization^2
                  + k_right_edge × rightBankMemory^2)
```

The measurement covariances shall be adapted similarly:

- weaken left closure first on left-bank evidence,
- weaken right closure first on right-bank evidence,
- weaken the lateral pseudo-measurement as `pivotWeight` rises or edge confirmation strengthens,
- do not restore trust immediately on re-grip.

---

## 11. Runtime update order

Per control cycle, do the following:

1. sanitize the command request,
2. estimate applied bank torque,
3. predict the UKF to the fixed control boundary,
4. compute the operating-region variables and freeze them for the cycle,
5. direct wheel-speed update with constrained gain,
6. direct gyro update,
7. forward-axis accelerometer update if enabled,
8. lateral-axis accelerometer update if enabled,
9. closure pseudo-measurement update,
10. adaptive lateral pseudo-measurement update,
11. stationary pseudo-measurement if in exact lock,
12. benign-motion soft odometry if enabled and allowed,
13. trusted wall or map updates,
14. run the feedforward inverse using the updated state estimate,
15. issue the closest feasible bank command.

No same-cycle recursive schedule recomputation is allowed.

---

## 12. Required telemetry

At minimum log:

- `forwardSpeed`, `rightSpeed`, `yawRate`,
- `leftBankWheelSpeed`, `rightBankWheelSpeed`,
- `gyroZBias`,
- `leftClosureResidual`, `rightClosureResidual`,
- `leftBankUtilization`, `rightBankUtilization`,
- `pivotWeight`, `rollingWeight`, `motionAngle`,
- `leftBankMemory`, `rightBankMemory`,
- `Q_forward`, `Q_right`, `Q_yaw`, `Q_leftWheel`, `Q_rightWheel`,
- `R_gyro`, `R_accelForward`, `R_accelRight`, `R_wheel`,
- closure covariance per side,
- lateral pseudo-measurement covariance,
- holdoff flags,
- re-grip dwell flags,
- traction-limited flag,
- electrical-limited flag,
- commanded and estimated applied bank torque,
- predicted achieved forward acceleration,
- predicted achieved yaw acceleration,
- per-cycle predict wall time,
- per-cycle update wall time,
- total estimator wall time.

---

## 13. Acceptance tests

The system is accepted only if all of the following pass.

### 13.1 Certified stationary

- no pose diffusion,
- no heading diffusion,
- bounded dynamic-state decay,
- clean release.

### 13.2 Yaw-only pivot

At `|yawRate| > 18 rad/s` and `|forwardSpeed| < 0.01 m/s`:

- feedforward must break away reliably,
- predicted yaw response must be materially better than the rolling-only model,
- direct wheel update must not corrupt body states,
- the estimator must stay stable through breakaway and through re-grip.

### 13.3 Mixed pivot-roll

At `|yawRate| > 18 rad/s` and `0.2 < |forwardSpeed| < 0.6 m/s`:

- the composite law must outperform either pure branch alone,
- yaw under-response must be reduced,
- closure and lateral pseudo-measurement schedules must remain stable,
- the command solver must remain continuous.

### 13.4 Rolling traction-limited turn

- onset contamination must be materially reduced,
- precursor metrics must rise before the worst mismatch,
- re-grip recovery must not snap the estimate,
- straight-line and gentle-turn precision must not materially regress.

### 13.5 Runtime cost

- predict cost remains in the present envelope,
- update cost remains in the present envelope,
- total runtime remains compatible with 1 kHz operation on the current target.

---

## 14. Final recommendation

Build the next revision as a **9-state SR-UKF with a bank-split composite plant**:

- rolling combined-slip branch for ordinary motion,
- explicit pivot-scrub yaw branch for yaw-only and near-yaw-only operation,
- smooth motion-angle blending for the mixed high-yaw regime,
- applied bank torque input,
- constrained wheel update,
- side-specific closure and re-grip protection,
- anisotropic traction projection,
- raw IMU measurement models with sensor-derived covariances,
- near-zero stationary-only gyro-bias process noise,
- and a feedforward inverse that always returns the closest feasible command.

That is the strongest single-filter architecture I would recommend here without moving to a more expensive multi-filter supervisor.
