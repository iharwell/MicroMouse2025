# Sensor Uncertainty Estimation and Value Assessment

Project: **Micromouse 2**  
Purpose: characterize the available sensor packages before choosing UKF state variables.  
Status: baseline estimates before empirical log-based calibration.

---

## 1. Hardware context

The project hardware relevant to sensing consists of:

- **Control and logging**
  - Teensy 4.1 MCU
  - 1000 Hz control loop
  - custom per-tick datalogging system with 5–11 µs latency on 512-byte packets
- **Sensors**
  - two IE2-1024 left/right motor encoders
  - LSM6DSV16X 6-axis IMU
  - custom IR wall sensors with ±3° half-angle emitters and 20° phototransistor receivers
  - one left wall sensor, one right wall sensor, and two forward-facing wall sensors
  - log-amp based front wall sensor circuitry
- **Motion hardware affecting sensor noise**
  - two Faulhaber 1717T006SR drive motors
  - DRV8871 H-bridge motor drivers
  - 17:56 gear ratio
  - 25 mm wheels with 17.2 mm hubs and 20-durometer solid rubber tires
  - four wheels total, two per side, shared pinion per bank
  - Maxon RE 8 fan motor

The key implication is that the **digital sensors are not the only error source**. The practical uncertainty is dominated by drivetrain compliance, wheel slip, vibration, wall reflectance, wall geometry, and timing quality.

---

## 2. Summary value assessment

| Package | Raw resolution | Practical uncertainty estimate | Estimator value | Needs UKF cleanup? | Main reason |
|---|---:|---:|---|---|---|
| IE2-1024 L/R encoders | ~5.82 µm / wheel-travel count after gearing | Sensor-only σ ≈ 1.7 µm quantization; practical odometry much larger due to traction and geometry | **Very high** | **No for denoising; yes for fusion** | Encoder counts are precise. The UKF is valuable for correcting model inconsistency, not for smoothing counts. |
| LSM6DSV16X gyro z | 0.035 dps/LSB at ±1000 dps; 0.070 dps/LSB at ±2000 dps | ~0.05–0.08 dps RMS white noise; ~0.1–0.3 dps effective moving-chassis rate uncertainty after static calibration | **Very high** | **Yes, for bias/drift** | Short-term yaw-rate signal is good, but bias and thermal/vibration effects integrate into yaw error. |
| LSM6DSV16X accelerometer x/y | 0.061–0.488 mg/LSB depending full-scale | ~1.3 mg RMS datasheet noise at 500 Hz BW; ±12 mg zero-g offset baseline; moving chassis likely 0.1–0.5 m/s² | **Low to medium** | **Yes if used quantitatively** | Direct velocity/position integration is poor; useful for event detection and possibly dynamic consistency. |
| LSM6DSV16X temperature | 1/256 °C/LSB; 16-bit output | Absolute offset ±15 °C; relative trend useful | **Medium for bias compensation** | **No** | Useful as a regressor for gyro bias drift, not as a state measurement to clean. |
| Side IR wall sensors | ADC and calibration-curve limited; mm/LSB is distance-dependent | Valid-wall repeatability ~0.5–2 mm σ; absolute ~2–5 mm; outliers >10 mm near posts/gaps/angles | **High** | **Yes, conditionally** | Strong wall-relative pose information, but nonlinear and non-Gaussian. Needs map-aware gating. |
| Front log-amp IR sensors | ADC/log-amp/calibration-curve limited; roughly fractional distance resolution | Close-range repeatability ~1–3 mm σ; absolute ~3–8 mm; yaw from dual front sensors often ~1–4° depending spacing | **Medium to high** | **Yes, conditionally** | Valuable for front-wall approach and alignment; invalid near oblique, missing, or partial walls. |
| Timing / sample interval | 1000 Hz nominal loop; IMU timestamp LSB 21.75 µs typical | 1% timing error creates 1% integrated yaw/velocity error | **Very high** | **No — calibrate/log it** | A timebase error looks like scale error. The UKF should not be expected to repair bad timestamps. |

---

## 3. Encoders: IE2-1024 left/right wheel odometry

### 3.1 Datasheet and geometry inputs

IE2-1024 encoder:

- 1024 lines per motor revolution
- two square-wave digital channels
- channel A/B nominal phase shift 90° electrical, specified as 90 ± 45°e
- maximum frequency for the 1024-line variant: 300 kHz
- supply: 4.5–5.5 V

Robot geometry assumptions from the hardware file:

- 17:56 gear ratio
- 25 mm wheel diameter
- encoder mounted on the motor side
- four wheels total, two left/two right, shared pinion per bank

### 3.2 Resolution calculation

Assuming 4x quadrature decoding:

```math
N_\text{motor} = 1024 \times 4 = 4096 \text{ counts/motor rev}
```

With a 17:56 reduction:

```math
N_\text{wheel} = 4096 \cdot \frac{56}{17} = 13492.7 \text{ counts/wheel rev}
```

Wheel circumference:

```math
C = \pi \cdot 25\ \text{mm} = 78.54\ \text{mm}
```

Linear distance per wheel count:

```math
\Delta s = \frac{78.54}{13492.7} = 0.00582\ \text{mm/count}
```

So:

```math
\boxed{\Delta s \approx 5.82\ \mu\text{m/count}}
```

Quantization standard deviation, assuming uniform ±0.5 count:

```math
\sigma_q = \frac{5.82\ \mu\text{m}}{\sqrt{12}} = 1.68\ \mu\text{m}
```

At a 1 ms control tick, a one-count delta corresponds to:

```math
5.82\ \mu\text{m} / 1\ \text{ms} = 5.82\ \text{mm/s}
```

### 3.3 Estimated uncertainty

| Error term | Estimate | Notes |
|---|---:|---|
| Position quantization, per wheel | σ ≈ **1.68 µm** | Sensor-only, per position sample. |
| Velocity quantum at 1 kHz | **5.82 mm/s/count** | Relevant if estimating instantaneous velocity by count differencing. |
| Sensor-only per-tick wheel delta | σ ≈ **0.002–0.01 mm** | Includes quantization and practical edge-timing/digital effects. |
| Straight-line odometry, calibrated floor | **0.05–0.2% of distance** | Practical motion-model error, not encoder electrical error. |
| Aggressive acceleration/turning | **0.5–2% of distance** | Slip, tire deformation, scrub, and shared-bank effects. |
| Left/right scale mismatch before calibration | potentially **>0.5–1%** | Wheel diameter and tire loading dominate. |

### 3.4 Value assessment

**Value: very high.** Encoders should be the primary local displacement source.

**UKF cleanup requirement:**

- **No**, not for cleaning raw counts.
- **Yes**, for fusing encoder-derived motion against gyro and wall observations.

The encoder package is precise enough that smoothing counts is not the main issue. The actual problem is that encoder odometry assumes no slip, stable wheel radius, known track width, and no lateral motion. Those assumptions are false during fast turns and acceleration.

### 3.5 Calibration priority

Calibrate these before trusting encoder odometry:

```math
s_L = k_L \Delta c_L
```

```math
s_R = k_R \Delta c_R
```

```math
\Delta \psi = \frac{s_R - s_L}{b_\text{eff}}
```

Priority:

1. left/right wheel scale, `k_L`, `k_R`
2. effective track width, `b_eff`
3. velocity-dependent or acceleration-dependent slip model
4. straight-line yaw drift from unequal tire loading

---

## 4. IMU gyro: LSM6DSV16X angular rate

### 4.1 Datasheet inputs

The LSM6DSV16X gyro supports these full-scale ranges and sensitivities:

| Full scale | Sensitivity |
|---:|---:|
| ±125 dps | 4.375 mdps/LSB |
| ±250 dps | 8.75 mdps/LSB |
| ±500 dps | 17.5 mdps/LSB |
| ±1000 dps | 35 mdps/LSB |
| ±2000 dps | 70 mdps/LSB |
| ±4000 dps | 140 mdps/LSB |

Relevant datasheet terms:

- gyro sensitivity tolerance: **±0.3%** for full scales up to ±2000 dps
- sensitivity drift: **±0.007%/°C**
- zero-rate level: **±1 dps**
- zero-rate drift vs temperature: **±0.006 dps/°C**
- rate noise density in high-performance mode: **2.8 mdps/√Hz**
- timestamp resolution: **21.75 µs typical**
- `INTERNAL_FREQ_FINE` provides an ODR/timestamp correction in 0.13% steps

### 4.2 Resolution and quantization

| Full scale | LSB size | Quantization σ |
|---:|---:|---:|
| ±500 dps | 0.0175 dps/LSB | 0.0051 dps |
| ±1000 dps | 0.0350 dps/LSB | 0.0101 dps |
| ±2000 dps | 0.0700 dps/LSB | 0.0202 dps |
| ±4000 dps | 0.1400 dps/LSB | 0.0404 dps |

Recommended starting point:

- Use **±1000 dps** if the fastest commanded yaw rate stays below ~800 dps with margin.
- Use **±2000 dps** if fast turns can exceed ±1000 dps.
- Avoid **±4000 dps** unless required, because it doubles the LSB size relative to ±2000 dps and the datasheet sensitivity tolerance is specified up to ±2000 dps.

### 4.3 White-noise estimate

For high-performance mode, using the datasheet noise density:

```math
\sigma_\omega \approx 2.8\ \text{mdps}/\sqrt{\text{Hz}} \cdot \sqrt{BW}
```

Approximate cases:

| Bandwidth assumption | Noise RMS |
|---:|---:|
| 342 Hz, roughly LPF2 cutoff near 960 Hz ODR | **0.052 dps** |
| 500 Hz, simple 1 kHz/2 assumption | **0.063 dps** |

Practical gyro z-rate uncertainty to use before logs:

| Condition | Suggested σ |
|---|---:|
| Static, averaged bias measurement | mean uncertainty can be **<0.01 dps** with sufficient averaging |
| Static instantaneous sample | **0.05–0.08 dps RMS** |
| Moving chassis, motors/fan off or mild | **0.1 dps** |
| Moving chassis, motors/fan active/aggressive | **0.2–0.3 dps** |

### 4.4 Bias and drift significance

A constant gyro bias integrates directly into heading error:

| Residual bias | Heading drift |
|---:|---:|
| 0.05 dps | 0.05°/s |
| 0.10 dps | 0.10°/s |
| 0.30 dps | 0.30°/s |
| 1.00 dps | 1.00°/s |

Thermal drift estimate:

```math
0.006\ \text{dps}/^\circ\text{C} \cdot 10^\circ\text{C} = 0.06\ \text{dps}
```

A 10–20 °C local board/die change can therefore move the zero-rate bias enough to matter, even after a good static calibration.

### 4.5 Scale error significance

With ±0.3% sensitivity tolerance:

| Maneuver | Error from ±0.3% scale |
|---:|---:|
| 90° | ±0.27° |
| 180° | ±0.54° |
| 360° | ±1.08° |
| 1080° | ±3.24° |

This is worth removing with an offline scale calibration, but it is usually less damaging than residual bias and drivetrain slip.

### 4.6 Value assessment

**Value: very high.** The z gyro is the primary high-bandwidth yaw-rate sensor.

**UKF cleanup requirement:** **yes, for bias and drift.**  
The gyro is not “bad” in the sense of white noise. It is bad enough in bias behavior that integrated yaw should not be trusted without correction.

Recommended handling:

- static zero-rate calibration at startup
- one-time z-scale verification/calibration
- online bias estimation or bias absorption through wall/encoder consistency
- use measured timestamps, not nominal 1 ms intervals

---

## 5. IMU accelerometer: LSM6DSV16X linear acceleration

### 5.1 Datasheet inputs

| Full scale | Sensitivity |
|---:|---:|
| ±2 g | 0.061 mg/LSB |
| ±4 g | 0.122 mg/LSB |
| ±8 g | 0.244 mg/LSB |
| ±16 g | 0.488 mg/LSB |

Relevant datasheet terms:

- high-performance noise density: **60 µg/√Hz**
- normal-mode noise density: **100 µg/√Hz**
- zero-g offset accuracy: **±12 mg**
- zero-g offset drift vs temperature: **±0.07 mg/°C**

### 5.2 Noise and quantization

At ±4 g:

```math
\sigma_q = \frac{0.122\ \text{mg}}{\sqrt{12}} = 0.035\ \text{mg}
```

The quantization is much smaller than the analog/MEMS noise.

High-performance mode, 500 Hz bandwidth:

```math
\sigma_a = 60\ \mu g/\sqrt{\text{Hz}} \cdot \sqrt{500} = 1.34\ \text{mg}
```

In SI units:

```math
1.34\ \text{mg} \approx 0.013\ \text{m/s}^2
```

But the zero-g offset is much larger:

```math
12\ \text{mg} \approx 0.118\ \text{m/s}^2
```

### 5.3 Bias integration problem

A constant acceleration bias creates position error:

```math
e_x = \frac{1}{2} a_\text{bias} t^2
```

Using 12 mg = 0.118 m/s²:

| Integration time | Position error |
|---:|---:|
| 0.1 s | 0.6 mm |
| 0.5 s | 14.7 mm |
| 1.0 s | 58.9 mm |
| 2.0 s | 235 mm |

This is before accounting for motor vibration, tire impacts, floor texture, fan vibration, pitch/roll coupling, and chassis flex.

### 5.4 Estimated uncertainty

| Condition | Suggested σ |
|---|---:|
| Static, quiet chassis | **0.013–0.03 m/s²** |
| Powered chassis, motors/fan idle | **0.03–0.10 m/s²** |
| Normal motion | **0.10–0.30 m/s²** |
| Aggressive acceleration/turning | **0.30–0.50+ m/s²** |
| Bias prior, uncalibrated | **±0.12 m/s²** baseline |

### 5.5 Value assessment

**Value: low to medium for navigation; medium for event detection.**

**UKF cleanup requirement:** **yes if used quantitatively**, but do not expect it to become a reliable standalone position/velocity source.

Recommended uses:

- detect collisions or wall contact
- detect launch/braking transients
- detect severe vibration or wheel slip events
- possibly constrain short-term forward acceleration only after careful filtering and calibration

Avoid:

- direct position integration
- treating x/y acceleration as a high-trust odometry input
- estimating too many accelerometer bias states before proving observability in logs

---

## 6. IMU temperature sensor

### 6.1 Datasheet inputs

- temperature refresh rate: **60 Hz**
- temperature sensitivity: **256 LSB/°C**
- temperature ADC resolution: **16 bit**
- offset: **±15 °C**

### 6.2 Resolution and uncertainty

```math
\Delta T = \frac{1}{256}\ ^\circ\text{C/LSB} = 0.0039\ ^\circ\text{C/LSB}
```

The absolute accuracy is poor because the offset can be ±15 °C. The relative temperature trend is still valuable for modeling gyro bias drift.

### 6.3 Value assessment

**Value: medium as a bias regressor, low as a standalone sensor.**

**UKF cleanup requirement:** **no.**

Use it to correlate gyro bias with warm-up state. Do not rely on it for absolute board temperature unless calibrated.

---

## 7. Side IR wall sensors

### 7.1 Hardware inputs

The side wall sensor package is custom analog IR, using:

- ±3° half-angle emitters
- 20° phototransistor receivers
- one left-facing sensor
- one right-facing sensor
- analog circuitry described as highly EMI-resilient

The ADC path, ADC resolution, emitter current, receiver gain, modulation scheme, and exact calibration curve are not specified in the hardware file. Therefore, the meaningful resolution is not “ADC counts” alone; it is:

```math
\Delta d \approx \frac{\Delta V}{|dV/dd|}
```

where `dV/dd` must be obtained from calibration logs.

### 7.2 Optical geometry

Emitter illuminated spot width:

```math
w_\text{emit} \approx 2d\tan(3^\circ) \approx 0.105d
```

Receiver field width:

```math
w_\text{recv} \approx 2d\tan(20^\circ) \approx 0.728d
```

| Distance to wall | Emitter spot width | Receiver field width |
|---:|---:|---:|
| 20 mm | 2.1 mm | 14.6 mm |
| 40 mm | 4.2 mm | 29.1 mm |
| 60 mm | 6.3 mm | 43.7 mm |
| 80 mm | 8.4 mm | 58.2 mm |

The narrow emitter helps spatial selectivity; the receiver is broad enough that the signal still depends heavily on wall angle, wall finish, wall height, and whether the sensor is seeing a post, gap, or edge.

### 7.3 Estimated uncertainty

Assuming calibrated ADC-to-distance curves and a valid side wall:

| Condition | Suggested σ / error |
|---|---:|
| Valid flat wall, near nominal distance | **0.5–1.0 mm σ** optimistic |
| Valid flat wall, general use | **1–2 mm σ** conservative |
| Absolute distance before per-wall/per-sensor correction | **2–5 mm** |
| Near posts, gaps, wall edges, high incidence angle | **>10 mm outliers possible** |
| Wrong wall association | unbounded / non-Gaussian |

### 7.4 Value assessment

**Value: high when map-gated.**

**UKF cleanup requirement:** **yes, but conditionally.**

The side IR sensors are valuable because they provide maze-relative lateral position and yaw information that encoders and gyro cannot provide absolutely. However, they are nonlinear optical sensors, not clean Gaussian rangefinders.

Use only when:

- the map predicts a wall on that side
- the robot is away from posts and openings
- the incidence angle is within the calibrated region
- the reading is inside the calibrated voltage/distance range
- the innovation is gated

Do not use as a continuously trusted wall-distance measurement through cell transitions.

---

## 8. Forward log-amp IR wall sensors

### 8.1 Hardware inputs

The front wall package consists of two forward-facing analog IR sensors, mounted left and right on the vehicle, with log-amp based front wall circuitry.

The log amplifier is useful because IR return strength changes strongly with distance and wall angle. It compresses dynamic range and reduces near-wall saturation risk. The tradeoff is that the distance resolution is not constant in millimeters.

A generic model is:

```math
y = A + B\log(I_\text{return})
```

so the distance resolution depends on the slope of the fitted calibration curve:

```math
\sigma_d \approx \frac{\sigma_y}{|dy/dd|}
```

### 8.2 Estimated uncertainty

Assuming good calibration and valid front-wall geometry:

| Condition | Suggested σ / error |
|---|---:|
| Close-range repeatability | **1–3 mm σ** |
| Absolute front-wall distance | **3–8 mm** |
| Front-wall detection | likely strong when within calibrated range |
| Oblique front wall or partial wall | outlier-prone |
| Missing wall but residual reflection | false-positive risk |

### 8.3 Dual-front yaw estimate

If the two front sensors produce range estimates `r_L` and `r_R`, and their lateral spacing is `b_f`, then the front-wall yaw estimate uncertainty is approximately:

```math
\sigma_\psi \approx \frac{\sqrt{2}\sigma_r}{b_f}
```

Examples:

| Range σ | Front sensor spacing `b_f` | Yaw σ |
|---:|---:|---:|
| 1 mm | 70 mm | 1.16° |
| 2 mm | 70 mm | 2.31° |
| 2 mm | 60 mm | 2.70° |
| 3 mm | 60 mm | 4.05° |

This is useful for final alignment, but it is not a substitute for the gyro during high-speed turns.

### 8.4 Value assessment

**Value: medium to high.**  
The front sensors are especially useful for:

- front-wall approach distance
- stop distance correction
- final alignment before turn or after a run segment
- detecting unexpected front wall presence

**UKF cleanup requirement:** **yes, conditionally.**

They should be treated as valid only when a front wall is expected or when a wall-detection hypothesis has high confidence. A UKF cannot repair a wrong wall association; it will simply pull the pose estimate in the wrong direction.

---

## 9. Timing as a measurement-quality issue

### 9.1 Why timing matters

The robot runs a 1000 Hz control loop, but gyro integration and encoder velocity estimation should use actual acquisition timestamps, not a hardcoded 1 ms interval.

A timebase error is equivalent to a scale error:

```math
\theta = \sum \omega_i \Delta t_i
```

If `Δt` is 1% wrong, the integrated angle is 1% wrong.

| Maneuver | Error from 1% timing scale error |
|---:|---:|
| 90° | 0.9° |
| 180° | 1.8° |
| 360° | 3.6° |

The LSM6DSV16X provides timestamp support with a typical 21.75 µs LSB and an `INTERNAL_FREQ_FINE` correction register. The project also has low-latency per-tick logging, so the system has the instrumentation needed to measure and correct timing.

### 9.2 Value assessment

**Value: very high.**

**UKF cleanup requirement:** **no.**

Bad timing should be fixed in acquisition and calibration. Treating timing error as estimator noise is inferior to logging actual acquisition time and using it directly.

---

## 10. Motor and driver effects on sensor uncertainty

The drive system is not itself a pose sensor, but it affects sensor quality.

Relevant points:

- The DRV8871 supports current regulation, with typical off-time of 25 µs and blanking time of 2 µs.
- The driver has internal protection including undervoltage lockout, overcurrent protection, and thermal shutdown.
- The Faulhaber 1717T006SR drive motor has low winding inductance and high speed capability.
- The robot has solid rubber tires and shared two-wheel banks per side.
- The fan motor adds a separate vibration source.

Expected effects:

| Source | Affected sensor | Effect |
|---|---|---|
| PWM current ripple / current regulation | IMU accel, gyro | vibration and apparent rate/acceleration noise |
| Motor torque ripple | encoders, IMU | wheel speed ripple and chassis vibration |
| Solid rubber tire compliance | encoders | effective wheel radius changes under load |
| Two wheels per side, shared pinion | encoders | scrub and unequal contact loading |
| Fan vibration | IMU, IR | broadband vibration and possible optical modulation |
| Voltage ripple / ground bounce | analog IR, IMU interface | analog noise, timing jitter, or rare digital errors |

This is why the practical moving-chassis uncertainties are intentionally larger than the raw datasheet white-noise values.

---

## 11. Initial uncertainty values to use before log fitting

These are starting values, not final tuned covariances.

| Measurement / quantity | Suggested initial uncertainty | Use note |
|---|---:|---|
| Encoder left/right wheel position count | **1.7 µm σ** equivalent | Pure quantization only. Usually too optimistic for motion modeling. |
| Encoder wheel delta, sensor-only | **0.002–0.01 mm σ** | Digital sensor contribution. |
| Encoder-derived motion, straight calibrated run | **0.05–0.2% of traveled distance** | Practical odometry uncertainty. |
| Encoder-derived motion, aggressive turn/accel | **0.5–2% of traveled distance** | Slip/scrub regime. |
| Gyro z-rate, quiet | **0.05–0.08 dps σ** | Datasheet noise dominated. |
| Gyro z-rate, moving robot | **0.1–0.3 dps σ** | Includes vibration and residual bias effects. |
| Gyro z bias prior after startup cal | **±0.1–0.5 dps** | Depends on thermal state and calibration method. |
| Gyro z bias prior without good cal | **±1 dps** | Datasheet-level zero-rate baseline. |
| Accelerometer x/y quiet | **0.02–0.05 m/s² σ** | After static offset removal. |
| Accelerometer x/y moving | **0.1–0.5 m/s² σ** | Vibration dominated. |
| Accelerometer bias prior | **±0.12 m/s²** | From ±12 mg zero-g offset. |
| Side IR distance, valid wall | **1–2 mm σ** | Use only after calibration and gating. |
| Front IR distance, valid wall | **2–4 mm σ** | Use only after calibration and gating. |
| Front-wall yaw from dual sensors | `sqrt(2) σ_r / b_f` | Usually ~1–4° depending spacing and range noise. |
| IMU temperature | relative trend useful; absolute ±15 °C | Use as bias regressor. |
| Sample interval | use measured `Δt` | Do not hardcode 1 ms unless verified. |

---

## 12. Calibration and characterization priority

| Priority | Item | Why |
|---:|---|---|
| 1 | Timestamp/acquisition timing | Prevents fake gyro/encoder scale error. |
| 2 | Encoder left/right scale and effective track width | Dominates dead-reckoning quality. |
| 3 | Gyro static bias and z-scale | Bias dominates short-term yaw drift; scale matters for repeatable turns. |
| 4 | IR sensor ADC-to-distance curves | Required before treating wall readings as measurements. |
| 5 | IR validity/gating model | Prevents posts, gaps, and wall-angle errors from corrupting pose. |
| 6 | Accelerometer vibration/bias characterization | Determines whether it should be used for navigation or only event detection. |
| 7 | Temperature-to-gyro-bias correlation | Useful after warm-up behavior is logged. |

---

## 13. Bottom-line assessment

1. **Encoders are the most precise raw sensors.** They do not need UKF smoothing, but encoder-only odometry needs correction for wheel and floor physics.
2. **The gyro is high-value and should be used.** Its white noise is acceptable; the UKF-relevant problem is bias drift and integration error.
3. **The accelerometer is not a primary navigation sensor.** It is useful for dynamic-event detection and possibly consistency checks, but direct integration is too bias-sensitive.
4. **The IR wall sensors are high-value but dangerous if trusted blindly.** They need calibration curves, map-aware validity checks, and innovation gating.
5. **Timing quality is a first-class calibration item.** A small timebase error produces apparent scale error in both gyro and encoder-derived velocity.
6. **The UKF should not be used to hide poor calibration.** Calibrate scale, timing, and wall curves first; use the UKF for fusion, bias estimation, and uncertainty propagation.

---

## 14. Source references

- `Hardware.md`: project hardware inventory, control-loop rate, datalogging, drivetrain, IR sensor geometry, and sensor package list.
- `lsm6dsv16x.pdf`: ST LSM6DSV16X datasheet, especially mechanical characteristics, output data rates, gyro/accelerometer sensitivity/noise/offset, timestamp, filter, and register sections.
- `EN_IE2-1024_DFF.pdf`: Faulhaber IE2-1024 encoder datasheet.
- `EN_1717_SR_DFF.pdf`: Faulhaber 1717 SR motor datasheet.
- `drv8871.pdf`: TI DRV8871 motor driver datasheet.
