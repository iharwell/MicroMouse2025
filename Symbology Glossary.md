# Micromouse State Estimation Variable Reference

---

## Coordinate Conventions

### Global Frame

* (P_x, P_y): global position in maze coordinates
* **Axis orientation:**

  * +Y = forward
  * +X = right

---

### Body Frame

* +f → forward
* +r → right

---

### Heading Convention

* (\Theta = 0): robot facing **+Y direction**
* Positive rotation:

  * **Clockwise yaw is positive**

---

## Frames

| Frame      | Description                           |
| ---------- | ------------------------------------- |
| global     | Maze/world coordinates                |
| body       | Robot-relative (+f forward, +r right) |
| local/task | Frame fixed at maneuver start         |
| drivetrain | Motor/encoder domain                  |

---

## Control Variables

| Symbol | Units       | Meaning             |
| ------ | ----------- | ------------------- |
| (C_L)  | [PWM units] | Left motor command  |
| (C_R)  | [PWM units] | Right motor command |

---

## Encoder / Drivetrain Variables

| Symbol                                 | Units    | Meaning             |
| -------------------------------------- | -------- | ------------------- |
| (N_L, N_R)                             | counts   | Encoder counts      |
| (\dot{N}_L, \dot{N}_R)                 | counts/s | Encoder rates       |
| (\phi_{m,L}, \phi_{m,R})               | rad      | Motor shaft angles  |
| (\phi_{w,L}, \phi_{w,R})               | rad      | Wheel-bank angles   |
| (\dot{\phi}*{w,L}, \dot{\phi}*{w,R})   | rad/s    | Wheel speeds        |
| (\ddot{\phi}*{w,L}, \ddot{\phi}*{w,R}) | rad/s²   | Wheel accelerations |

---

## Conversion Relationships

$$
\phi_m = N \cdot [2\pi / \text{counts per rev}]
$$

$$
\phi_w = \frac{\phi_m}{G}
$$

---

## Geometry

| Symbol | Units    | Meaning      |
| ------ | -------- | ------------ |
| (r_w)  | m        | Wheel radius |
| (b)    | m        | Track width  |
| (G)    | unitless | Gear ratio   |

---

## Kinematics / Dynamics

| Symbol     | Units  | Frame  | Meaning                        |
| ---------- | ------ | ------ | ------------------------------ |
| (P_f, P_r) | m      | local  | Forward / lateral displacement |
| (\Theta)   | rad    | global | Heading                        |
| (V_f, V_r) | m/s    | body   | Forward / lateral velocity     |
| (\omega)   | rad/s  | body   | Yaw rate                       |
| (\alpha)   | rad/s² | body   | Yaw acceleration               |

---

## IMU Measurements

| Symbol          | Units | Meaning              |
| --------------- | ----- | -------------------- |
| (\omega_{meas}) | rad/s | Gyro yaw rate        |
| (A_{f,meas})    | m/s²  | Forward acceleration |
| (A_{r,meas})    | m/s²  | Lateral acceleration |

---

## Nominal Model Terms

| Symbol         | Units  | Meaning                      |
| -------------- | ------ | ---------------------------- |
| (A_{f,nom})    | m/s²   | Nominal forward acceleration |
| (A_{r,nom})    | m/s²   | Nominal lateral acceleration |
| (\alpha_{nom}) | rad/s² | Nominal yaw acceleration     |

---

## Residual / Error Terms

| Symbol          | Units  | Meaning                |
| --------------- | ------ | ---------------------- |
| (\Delta A_f)    | m/s²   | Forward accel mismatch |
| (\Delta A_r)    | m/s²   | Lateral accel mismatch |
| (\Delta \alpha) | rad/s² | Yaw accel mismatch     |

---

## Wall Sensors (Optional)

| Symbol           | Units | Meaning              |
| ---------------- | ----- | -------------------- |
| (d_L, d_R)       | m     | Side wall distances  |
| (d_{FL}, d_{FR}) | m     | Front wall distances |

---

## Filter Variables

| Symbol     | Meaning            |
| ---------- | ------------------ |
| (x͍)       | State vector       |
| (z͍)       | Measurement vector |
| (P)        | State covariance   |
| (Q)        | Process noise      |
| (R)        | Measurement noise  |
| (\Delta t) | Timestep           |

---

## Vector Notation Convention

* Mathematical vectors:

  * (x͍), (z͍)

* Code:

  * `_vec` suffix required
  * Example:

    * `state_vec`
    * `measurement_vec`

---

## Code Naming Conventions

### Required Name Mappings

| Symbol     | Code       |
| ---------- | ---------- |
| (P_x, P_y) | `Px`, `Py` |
| (P_f, P_r) | `Pf`, `Pr` |
| (V_f, V_r) | `Vf`, `Vr` |
| (A_f, A_r) | `Af`, `Ar` |
| (\Theta)   | `heading`  |
| (\omega)   | `yawRate`  |
| (\alpha)   | `yawAccel` |
| (C_L, C_R) | `CL`, `CR` |

---

### Naming Rules

* `heading`:

  * global frame
  * 0 = +Y
  * clockwise positive

* Angular quantities:

  * must be `yawRate`, `yawAccel`
  * never `theta`, `omega`

---

### Disallowed Naming

* `theta`, `omega`
* `vel`, `acc`, `pos` without axis
* single-letter variables

---

## Transform Equations

### Body → Global

$$
\begin{aligned}
V_x &= V_f \sin(\Theta) + V_r \cos(\Theta) \
V_y &= V_f \cos(\Theta) - V_r \sin(\Theta)
\end{aligned}
$$

---

### Global → Body

$$
\begin{aligned}
V_f &= V_x \sin(\Theta) + V_y \cos(\Theta) \
V_r &= V_x \cos(\Theta) - V_y \sin(\Theta)
\end{aligned}
$$

---

### Position Update

$$
\begin{aligned}
\dot{P}_x &= V_f \sin(\Theta) + V_r \cos(\Theta) \
\dot{P}_y &= V_f \cos(\Theta) - V_r \sin(\Theta)
\end{aligned}
$$

---

### Local Frame Transform

$$
\begin{aligned}
P_f &= (P_x - P_{x0}) \sin(\Theta_0) + (P_y - P_{y0}) \cos(\Theta_0) \
P_r &= (P_x - P_{x0}) \cos(\Theta_0) - (P_y - P_{y0}) \sin(\Theta_0)
\end{aligned}
$$

---

### Angular

$$
\dot{\Theta} = \omega
$$

---

### Body Dynamics

$$
\begin{aligned}
\dot{V}_f &= A_f + \omega V_r \
\dot{V}_r &= A_r - \omega V_f
\end{aligned}
$$

---

## IMU Sign Convention Contract

* All IMU sign corrections occur **only in hardware layer**

### Guarantee

$$
\omega_{meas} \rightarrow yawRate \quad (\text{clockwise positive})
$$

$$
A_{f,meas}, A_{r,meas} \rightarrow \text{correct body-frame signs}
$$

---

### Forbidden

* Any sign correction outside hardware layer
* Axis reinterpretation
* Conditional transforms

---

### Rule

IMU outputs are:

> **final and canonical**

---

## Code Architecture Constraints

### Object Model

* Classes represent **objects with behavior**
* Data-only containers are forbidden

---

### Forbidden

* Structs
* Public fields
* Passive containers

---

### API Rule

Prefer:

```cpp
state.applyImuSample(sample, dt);
state.predictFromDrive(drive_sample, dt);
```

Avoid:

```cpp
state.setVf(...);
state.setTheta(...);
```

---

### Internal Exception

* Eigen types allowed internally for performance
* Must not escape API boundaries

---

## Key Notes

* Encoders measure drivetrain motion only
* IMU measures true body motion
* Slip is expected and required in performance operation 

---

## Sign Summary

| Quantity | Positive  |
| -------- | --------- |
| +Y       | Forward   |
| +X       | Right     |
| (V_f)    | Forward   |
| (V_r)    | Right     |
| heading  | Clockwise |
| yawRate  | Clockwise |
| yawAccel | Clockwise |
