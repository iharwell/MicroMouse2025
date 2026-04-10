# Tooling

## `analyze_open_floor.py`

Purpose: summarize `open_floor_main.csv` and `open_floor_timing.csv` captures so the team can ground UKF and plant-tuning discussions in the same numbers.

What it does:

- Computes stationary-hold noise statistics from `SEC_10_STATIC` / `STATIC_HOLD`.
- Computes launch-repeatability statistics from `SEC_20_LAUNCH` / `OPEN_LOOP_LAUNCH`.
- Reports per-command launch behavior:
  peak wheel speed, peak linear speed, peak gyro, peak accel, encoder activity rate, and maximum pose drift.
- Suggests candidate UKF measurement sigmas from the data:
  `imu_yaw_sigma_radps`, `imu_accel_sigma_mps2_conservative`, `encoder_linear_sigma_mps`, and `encoder_yaw_sigma_radps`.
- Optionally summarizes `open_floor_timing.csv` so control-loop timing regressions are visible beside the tuning run.

Method notes:

- Launch repeatability is computed from the nonzero-command window only.
- Positive and negative launch passes are sign-normalized before comparison.
- Repeats are aligned by sample index within each active launch window, then residual sigmas are measured against the per-command mean waveform.
- The script uses only the Python standard library.

Example usage:

```powershell
python tooling\analyze_open_floor.py --main D:\open_floor_main.csv --timing D:\open_floor_timing.csv
```

Typical use:

1. Run the script on a fresh capture.
2. Compare the suggested sigmas to the canonical owners in `SrUkfCore.h`.
3. Use the per-command launch summary to decide whether a plant or launch-assist change is actually supported, or whether the run only justifies estimator-noise changes.

Caveats:

- The script reports what the data says; it does not edit firmware.
- A single open-floor run is good evidence for measurement-noise tuning, but weak evidence for geometry changes.
- If recovery phases enter the next launch with residual motion, treat plant fits cautiously even if the repeatability numbers are still useful.
