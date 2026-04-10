# Tooling

## `analyze_open_floor.py`

Purpose: summarize `open_floor_main.csv` and `open_floor_timing.csv` captures so the team can ground UKF and plant-tuning discussions in the same numbers.

What it does:

- Computes stationary-hold noise statistics from `SEC_10_STATIC` / `STATIC_HOLD`.
- Computes launch-repeatability statistics from `SEC_20_LAUNCH` / `OPEN_LOOP_LAUNCH`.
- Computes current-run launch-based tire-plant estimates when `logging.txt` is available:
  run ID, apparent equivalent wheel inertia, apparent rolling friction, apparent viscous drag, launch motion threshold, and whether the current card can identify lateral tire parameters at all.
- Extracts recovery-turn geometry from raw sensors only:
  bias-corrected gyro, encoder differential motion, and the IMU-offset rotational acceleration signature.
- Reports per-command launch behavior:
  peak wheel speed, peak linear speed, peak gyro, peak accel, encoder activity rate, and maximum pose drift.
- Suggests candidate UKF measurement sigmas from the data:
  `imu_yaw_sigma_radps`, `imu_accel_sigma_mps2_conservative`, `encoder_linear_sigma_mps`, and `encoder_yaw_sigma_radps`.
- Reports recovery-turn parameters when the data contains them:
  actual turn angle, whole-turn odometric-equivalent track width, per-sample effective-track-width percentiles (`L5`, `L10`, `L25`, `L50`, `L75`, `L90`, `L95`, mean, sigma), encoder-vs-gyro coherence, rotation-signature alignment, and a torque-only yaw-inertia upper bound when `logging.txt` is available.
- Optionally summarizes `open_floor_timing.csv` so control-loop timing regressions are visible beside the tuning run.

Method notes:

- Launch repeatability is computed from the nonzero-command window only.
- Positive and negative launch passes are sign-normalized before comparison.
- Repeats are aligned by sample index within each active launch window, then residual sigmas are measured against the per-command mean waveform.
- Recovery-turn angle estimation explicitly excludes UKF pose and integrates an independently debiased raw gyro over a sensor-only turn window.
- Recovery-turn angle estimation uses `gyro_raw_radps` minus an independently estimated stationary bias from `SEC_10_STATIC`; it does not trust the run's internal corrected-gyro stream for this.
- Recovery-turn windows are gated by encoder differential speed and the rotational acceleration expected at the back-left IMU location `(-23 mm, -11 mm)`.
- Per-sample effective track width is evaluated inside the detected turn window as `abs((v_r - v_l) / gyro)` on samples with `abs(gyro) >= 0.25 rad/s`.
- If encoder-implied yaw at the logged nominal track width diverges strongly from gyro yaw, treat the reported track width as slip-inflated rather than geometric.
- The yaw-inertia output is intentionally conservative: it is a torque-only upper bound from the recovery spin-up, not a full plant-identification solve.
- Launch-derived tire-plant outputs are labeled `apparent` on purpose: they use the current card only and absorb unmodeled drive efficiency, launch deadband, and the lack of an external body-speed reference.
- If the current card lacks completed `SEC_40_YAW`, `SEC_50_SMOOTH`, `SEC_60_LOOP_CW`, or `SEC_70_LOOP_CCW` sections, the tool reports lateral tire parameters as not identifiable from that run rather than fabricating fits.
- The script uses only the Python standard library.

Example usage:

```powershell
python tooling\analyze_open_floor.py --main D:\open_floor_main.csv --timing D:\open_floor_timing.csv
```

If the run directory has `logging.txt`, the script auto-detects it. You can also pass it explicitly:

```powershell
python tooling\analyze_open_floor.py --main D:\open_floor_main.csv --control-log D:\logging.txt
```

Typical use:

1. Run the script on a fresh capture.
2. Compare the suggested sigmas to the canonical owners in `SrUkfCore.h`.
3. Use the recovery-turn summary to check the actual raw-sensor turn angle before trusting any nominal `180 deg` assumption.
4. Use the per-command launch summary to decide whether a plant or launch-assist change is actually supported, or whether the run only justifies estimator-noise changes.
5. Use the tire-plant section only to update parameters that the current card actually excites; treat any output marked unstable or not identifiable as diagnostic-only.

Caveats:

- The script reports what the data says; it does not edit firmware.
- A single open-floor run is good evidence for measurement-noise tuning, but weak evidence for geometry changes.
- If recovery phases enter the next launch with residual motion, treat plant fits cautiously even if the repeatability numbers are still useful.
- If a recovery section times out, treat its track-width and inertia numbers as diagnostic-only and prefer watchdog-clean repeats for parameter updates.
