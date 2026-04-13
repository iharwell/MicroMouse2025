# Tooling

## `analyze_open_floor.py`

Purpose: summarize `open_floor_main.csv` and `open_floor_timing.csv` captures so the team can ground UKF and plant-tuning discussions in the same numbers.

What it does:

- Computes stationary-hold noise statistics from `SEC_10_STATIC` / `STATIC_HOLD`.
- Computes launch-repeatability statistics from `SEC_20_LAUNCH` / `OPEN_LOOP_LAUNCH`.
- Separates backlash-like launch twitches from sustained chassis motion using `LaunchPulse` only and reports:
  the speed quantization floor, backlash envelopes, first clear breakaway command, effective launch floor, and per-command clear-launch hit rate.
- Computes current-run launch-based tire-plant estimates when `logging.txt` is available:
  run ID, apparent equivalent wheel inertia, apparent rolling friction, apparent viscous drag, launch motion threshold, and whether the current card can identify lateral tire parameters at all.
- Inverts the configured launch-region feedforward model against the logged launch motion:
  per-command required normalized drive percentiles, steady-region required command, and overall command error/RMSE versus the logged command bins.
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
- Launch-floor classification derives a backlash envelope from repeats that never sustain encoder-derived body speed above the quantized speed floor for at least `10 ms`.
- A launch only counts as clear chassis motion when that sustained-speed test agrees with both inertial evidence and encoder/pose drift beyond the backlash envelope.
- Positive and negative launch passes are sign-normalized before comparison.
- Repeats are aligned by sample index within each active launch window, then residual sigmas are measured against the per-command mean waveform.
- Recovery-turn angle estimation explicitly excludes UKF pose and integrates an independently debiased raw gyro over a sensor-only turn window.
- Recovery-turn angle estimation uses `gyro_raw_radps` minus an independently estimated stationary bias from `SEC_10_STATIC`; it does not trust the run's internal corrected-gyro stream for this.
- Recovery-turn windows are gated by encoder differential speed and the rotational acceleration expected at the back-left IMU location `(-23 mm, -11 mm)`.
- Per-sample effective track width is evaluated inside the detected turn window as `abs((v_r - v_l) / gyro)` on samples with `abs(gyro) >= 0.25 rad/s`.
- If encoder-implied yaw at the logged nominal track width diverges strongly from gyro yaw, treat the reported track width as slip-inflated rather than geometric.
- The yaw-inertia output is intentionally conservative: it is a torque-only upper bound from the recovery spin-up, not a full plant-identification solve.
- Launch-derived tire-plant outputs are labeled `apparent` on purpose: they use the current card only and absorb unmodeled drive efficiency, launch deadband, and the lack of an external body-speed reference.
- Feedforward-alignment output uses the logged plant constants directly, including wheel-bank inertia and any logged static/rolling friction terms, then inverts the same normalized motor-command model shape used by the runtime plant.
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
5. Use the launch-floor section when you need a backlash-safe breakaway estimate; if the tool flags nonmonotonic clear motion, treat the reported floor as provisional and prefer another clean card before retuning launch thresholds.
6. Use the tire-plant section only to update parameters that the current card actually excites; treat any output marked unstable or not identifiable as diagnostic-only.

Caveats:

- The script reports what the data says; it does not edit firmware.
- A single open-floor run is good evidence for measurement-noise tuning, but weak evidence for geometry changes.
- If recovery phases enter the next launch with residual motion, treat plant fits cautiously even if the repeatability numbers are still useful.
- If a recovery section times out, treat its track-width and inertia numbers as diagnostic-only and prefer watchdog-clean repeats for parameter updates.

## `analyze_competition_feedforward.py`

Purpose: compare the legacy competition `diag*.csv` sweeps against the current repo feedforward and plant setup, instead of the stale constants embedded in the old log format.

What it does:

- Reads the current authoritative robot/setup values from the repo:
  `Vehicle.h`, `MotorEncoderDrive.h`, `PlantModel.h`, `PlantModel.cpp`, and `MazeMapRuntimeCore.h`.
- Reuses the same inverse-command plant math as the open-floor analyzer.
- Parses legacy `diag*.csv` files by reading `# phase`, `# event`, and `sample` rows.
- Separates the old diagnostic probes into:
  `kickoff_*_probe` launch segments and the hold-command portion of `forward_*_probe`.
- Reports per-command inverse required-drive percentiles and overall command error/RMSE against the measured command bins in those legacy probes.
- Reports old launch/carry success rates from `kickoff_result` and `forward_result`.
- Estimates current plant parameters from those same legacy measurements:
  apparent wheel-bank inertia, rolling drag, viscous drag, breakaway torque bounds, and one compromise parameter set chosen against the competition-command medians.

Example usage:

```powershell
python tooling\analyze_competition_feedforward.py --root "TestResults\Competition Testing Data"
```

Method notes:

- The competition analyzer intentionally uses the current repo setup, not the old `# meta` feedforward constants in the legacy logs.
- `forward_*_probe` phases are split by the actual logged drive command so the steady hold segment is analyzed separately from the kickoff pulse and zero-command settle.
- The current static-friction torque used by the inverse model is derived from the current plant breakaway command in `PlantModel.cpp`, while the runtime launch-assist settings from `MazeMapRuntimeCore.h` are reported beside it so mismatches are visible.
