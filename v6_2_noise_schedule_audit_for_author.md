# Micromouse V6.2 Noise Schedule Audit

**Subject:** Audit of noise schedules in `ukf_plant_model_rework_spec_v6_2.md` against the project sensor stack  
**Purpose:** Provide a concise, implementation-focused review for the author of V6.2.

---

## Overall verdict

The noise architecture in V6.2 is directionally strong, but it is **not yet implementation-ready as a complete noise specification**.

The major issue is not the scheduling strategy itself. The major issue is that several critical noise terms are still qualitative where the project needs explicit numeric floors, caps, and sensor-driven baseline models.

The two largest gaps are:

1. **Wheel-derived covariances are under-specified relative to encoder quantization.**
2. **Gyro-bias process noise is effectively unspecified despite the IMU being stable enough that this matters.**

---

## What is already correct in the document

The following choices are technically sound and should remain:

- Correct gyro measurement form: `z_g = r + b_gz + n_g`
- Raw synchronized planar-accelerometer reintegration with staged rollout
- Side-specific weakening of closure authority
- Strong `v ≈ 0` authority at launch, with relaxation in confirmed edge regimes
- Innovation-based covariance inflation / rejection logic
- Re-grip dwell and delayed authority restoration

These are good structural decisions. The remaining work is mostly in turning the schedules into hard, sensor-grounded quantitative definitions.

---

## 1. Wheel-speed measurement noise `R_ω`

### Finding

The wheel-speed measurement model is defined, but its baseline covariance is not tied to encoder resolution.

That is a problem. If `R_ω` is tuned below the actual quantization floor, the filter will misinterpret encoder quantization as real wheel dynamics or early slip evidence.

### Why this matters

For the project encoder setup previously described:

- 1024 PPR quadrature encoder
- effectively 4096 counts per motor revolution
- 56/17 motor:wheel ratio
- 25 mm wheel OD

The wheel-bank angular increment is small, but at a 1 ms update cadence the implied one-count speed step is still large enough that the low-speed wheel measurement variance cannot be treated as negligible.

### Required correction

Define `R_ω` explicitly from the actual wheel-speed estimation window:

```text
Δω(Tw) = 2π / (Nwheel * Tw)
σ²_ω,quant(Tw) = Δω(Tw)² / 12
R_ω = max(R_ω,min, σ²_ω,quant(Tw) + R_ω,extra(speed))
```

where:

- `Tw` is the wheel-speed estimation window,
- `Nwheel` is wheel-side counts per revolution,
- `R_ω,min` is a fixed floor for interface / timestamp noise,
- `R_ω,extra(speed)` handles very-low-speed estimator behavior if needed.

### Recommendation

Do **not** try to cover encoder quantization by inflating process noise. It belongs in wheel measurement covariance.

---

## 2. Closure pseudo-measurement covariance `R_L`, `R_R`

### Finding

The closure residual formulation is correct, and the side-specific weakening / recovery logic is correct.

However, the baseline closure covariance is under-specified. It should inherit the encoder quantization floor explicitly.

### Why this matters

Closure is driven partly by wheel measurements. If closure covariance does not reflect encoder resolution, low-speed disagreement will be over-interpreted.

### Required correction

Define baseline closure covariance from wheel measurement variance plus geometry / model uncertainty:

```text
R_σ,L = Re² R_ωL + R_geom + R_model,L
R_σ,R = Re² R_ωR + R_geom + R_model,R
```

where:

- `Re` is the effective estimator wheel radius,
- `R_geom` covers wheel-radius / track-width uncertainty,
- `R_model,*` covers non-rolling mismatch and kinematic simplification error.

### Recommendation

The adaptive side-specific inflation already described in V6.2 should remain, but it should act on top of a correct encoder-aware baseline.

---

## 3. Gyro measurement noise `R_g`

### Finding

The gyro measurement equation is correct and fixes an important architectural issue.

What is missing is an explicit measurement-noise floor tied to the IMU.

### Why this matters

The gyro is one of the cleanest direct dynamic measurements in the stack. If `R_g` is set too loose, the estimator will underuse it. If it is set unrealistically tight without a justified floor, the estimator can become brittle.

### Required correction

Define a minimum gyro measurement covariance from the IMU white-noise level and effective bandwidth:

```text
R_g,min = (Nd_g * sqrt(BW) * π/180)²
```

where:

- `Nd_g` is gyro noise density in `deg/s/√Hz`,
- `BW` is the effective measurement bandwidth in Hz.

Then apply only a modest implementation margin for interface noise, synchronization error, and other real acquisition effects.

### Recommendation

Do not let `R_g` float as a loose tuning parameter unrelated to the sensor.

---

## 4. Gyro-bias process noise `Q_bgz`

### Finding

This is the single most important missing schedule.

The document includes `b_gz` in the state and gives it a random-walk form, but it never defines a proper operating schedule for `Q_bgz`.

### Why this matters

For this project, gyro bias is not expected to wander rapidly under ordinary operation. If `Q_bgz` is too large, the filter will use bias drift to absorb unrelated model errors.

That is not acceptable.

`Q_bgz` must not be used to hide:

- yaw-model error,
- encoder disagreement,
- launch transients,
- traction-limit behavior,
- wheel/body mismatch.

### Required correction

Specify `Q_bgz` by operating condition:

- **Certified rest:** small nonzero `Q_bgz`
- **Normal motion:** zero or near-zero `Q_bgz`
- **Exceptional thermal or re-identification cases:** very small temporary inflation only

### Recommendation

Bias should be nearly frozen in motion unless there is strong justification to do otherwise.

---

## 5. Planar accelerometer covariances `R_ax`, `R_ay`

### Finding

The accel-update architecture is good:

- raw synchronized samples,
- no default estimator-path low-pass,
- per-axis innovation checks,
- staged rollout,
- forward axis first,
- lateral axis gated later.

The missing piece is that the covariance floors need to account for **model error**, not just sensor white noise.

### Why this matters

For the planar accelerometer path, the dominant residual source is unlikely to be the MEMS noise floor alone. In many operating regions, the larger effects will be:

- lever-arm sensitivity to `ṙ`,
- one-step process-model mismatch,
- load-transfer approximation error,
- tire/contact-law mismatch near saturation.

### Required correction

Define each accel covariance as the maximum of a sensor floor and a model-error floor:

```text
R_ax = max(R_ax,sensor, R_ax,model)
R_ay = max(R_ay,sensor, R_ay,model)
```

### Recommendation

At initial rollout:

- `R_ax` may be relatively tight,
- `R_ay` should start materially larger than `R_ax`,
- both should have explicit floors and caps.

Do not set either axis purely from the datasheet noise floor.

---

## 6. Stationary pseudo-measurement covariance

### Finding

The exact stationary branch is structurally correct, but the pseudo-measurement covariance is still described only as “small but nonzero.”

That is too vague.

### Why this matters

The project has unusually strong rest evidence:

- encoder inactivity criterion,
- near-zero corrected gyro,
- near-zero body-plane accel,
- effectively zero command,
- hysteresis and dwell.

That means the stationary pseudo-measurement can be sharp without being reckless.

### Required correction

Define explicit stationary covariance values for:

```text
[u, v, r, ω_L, ω_R]
```

based on the actual certified-rest sensor limits and detection thresholds.

### Recommendation

Do not leave stationary pseudo-measurement strength as a qualitative tuning phrase.

---

## 7. Adaptive lateral pseudo-measurement `R_v`

### Finding

The overall philosophy is correct:

- strong at launch,
- relaxed in confirmed edge operation,
- weakened during re-grip recovery.

### Why this matters

There is no direct lateral-velocity sensor. Therefore, `v ≈ 0` must remain subordinate to contradictory evidence from real sensors and trusted map/wall information.

### Required refinement

Release of the lateral pseudo-measurement should be tied more explicitly to **sensor contradiction**, not just regime scheduling. In practice:

- keep `R_v` small at low-speed launch,
- inflate aggressively when accel / closure inconsistency rises,
- do not keep it tight once trustworthy contradictory evidence is present.

### Recommendation

The current structure is acceptable. It just needs tighter wording around the contradiction triggers.

---

## 8. Process-noise schedules `Q_u`, `Q_v`, `Q_r`

### Finding

The adaptive forms for `Q_u`, `Q_v`, and `Q_r` are reasonable.

### Required caution

These process-noise terms should represent actual unmodeled dynamics. They should **not** be used to absorb measurement-side problems.

Specifically:

- encoder quantization belongs in `R_ω`, `R_L`, `R_R`,
- gyro white noise belongs in `R_g`,
- accelerometer electronics and model mismatch belong in accel `R`,
- only true unmodeled motion or force uncertainty belongs in `Q_u`, `Q_v`, `Q_r`.

### Recommendation

Preserve the current process-noise structure, but explicitly forbid using `Q_u` or `Q_r` as a substitute for proper wheel-side covariance near zero speed.

---

## 9. Innovation-consistency thresholds

### Finding

The Green / Amber / Red robustification design is correct.

### Missing detail

The thresholds cannot be shared uniformly across sensor classes.

### Required correction

Define different innovation / NIS criteria for:

- gyro,
- encoder / closure,
- accelerometer,
- wall or map updates.

### Why this matters

These measurement families do not have the same residual statistics:

- gyro is closest to Gaussian,
- encoder-derived closure is quantized, especially near zero speed,
- accel residuals are more model-error dominated,
- wall residuals depend on the wall-sensor transfer function.

### Recommendation

Do not reuse one generic threshold family for every enabled update type.

---

## 10. Wall / map update covariance

### Finding

Wall/map updates remain part of the accepted measurement set, but their covariance model is not defined in a sensor-specific way.

### Audit limitation

This part cannot be fully audited from V6.2 alone. A proper audit requires the actual wall-sensor characteristics, including:

- transfer behavior,
- range sensitivity,
- incidence-angle dependence,
- reflectivity sensitivity,
- update geometry.

### Recommendation

This should be called out explicitly as still under-specified rather than treated as already settled.

---

## Bottom line for the author

V6.2 has the right **noise architecture**, but several of the most important schedules still need to be converted from qualitative guidance into hard numeric definitions.

### High-priority fixes

1. Define encoder-resolution-aware `R_ω`
2. Propagate that baseline into closure `R_L`, `R_R`
3. Define explicit `R_g` floor from the IMU
4. Add a proper operating schedule for `Q_bgz`
5. Add model-error floors for `R_ax`, `R_ay`
6. Define explicit stationary pseudo-measurement covariance
7. Separate innovation thresholds by sensor class
8. Finish the wall/map covariance model

### Most important single correction

**Do not let encoder quantization leak into process noise.**

That uncertainty belongs in the wheel and closure measurement covariances, not in `Q_u`, `Q_r`, or other process terms.

---

## Suggested author action

The next revision should add a dedicated subsection titled something like:

> **Noise Floors, Caps, and Sensor-Derived Baselines**

That subsection should define, at minimum:

- `R_ω`
- `R_g`
- `R_ax`, `R_ay`
- stationary pseudo-measurement covariance
- closure baseline covariance
- `Q_bgz`
- per-sensor-class innovation thresholds
- wall/map covariance structure

Without that, the document is a strong architecture note but not yet a complete noise specification.
