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

## `analyze_open_floor_turn_bias.py`

Purpose: evaluate `SEC_40_YAW` turn-center bias from raw sensors only, with explicit separation between simple one-side bias and a true wheel-bank pivot.

What it does:

- Auto-discovers the latest decoded `open_floor_main.csv` under `TestResults` unless you pass `--main`.
- Reads the authoritative vehicle/tire geometry from `MazeMap/MazeMap/Vehicle.h` and the back-left IMU position from `MazeMap/MazeMap/Vehicle.cpp`.
- Estimates independent stationary gyro and accel biases from `SEC_10_STATIC` / `STATIC_HOLD`.
- Computes center-of-rotation `x` several ways on each yaw turn:
  encoder-average over raw gyro, encoder-average over encoder-derived yaw rate, and accelerometer moment-arm inversion over raw gyro.
- Splits each turn into a startup/steady core window and a stop/trail window so translational runoff is visible instead of being folded into the steady-turn estimate.
- Uses the tire-bank span from `trackWidthPhysicalMinM .. trackWidthPhysicalMaxM` to distinguish:
  `center`, `left/right_bias`, `left/right_pivot`, and `left/right_outboard`.
- Reports both a looser encoder bank-zone detection and a stricter consensus pivot detection that requires encoder geometry and accel-inverted geometry to agree on the same wheel bank.

Example usage:

```powershell
python tooling\analyze_open_floor_turn_bias.py
```

Explicit input example:

```powershell
python tooling\analyze_open_floor_turn_bias.py --main TestResults\mmlog_decode_2026-04-21_02-46-49\open_floor_main.csv
```

Method notes:

- This tool is intentionally sensor-only. It does not use UKF state.
- The stop window keeps only rows that are still turning in the commanded direction and above a configurable minimum `|gyro|`.
- The accelerometer inversion is part of the pivot-side test specifically so late low-yaw encoder tails do not get mistaken for a physical wheel-bank pivot.
- A true pivot classification is stricter than a one-bank bias classification:
  the tool requires a consecutive stop-window run where encoder-derived CoR and accel-inverted CoR both land inside the tire-bank span on the same side.

## `run_open_floor_ukf_replay.ps1`

Purpose: build and run the standalone open-floor UKF replay tool that replays decoded `open_floor_main.csv` captures through the current C++ UKF implementation, writes a batch report, and also runs the archival competition feedforward check, without rebuilding `MazeMap.dll`.

Default usage:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1
```

Optional filters:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1 `
  -Root TestResults `
  -Output TestResults\open_floor_ukf_replay_manual `
  -RunId ofm_10728325
```

Optional repeat-run shortcut when the standalone exe is already current:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1 `
  -SkipToolBuild `
  -RunId ofm_10728325
```

Optional competition-archive controls:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1 `
  -SkipToolBuild `
  -RunId ofm_10728325 `
  -CompetitionArchiveRoot "TestResults\Competition Testing Data"
```

Skip the archival competition check when you only want the decoded open-floor replay:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1 `
  -SkipToolBuild `
  -RunId ofm_10728325 `
  -SkipCompetitionArchive
```

Single-run sample export example:

```powershell
powershell -ExecutionPolicy Bypass -File tooling\run_open_floor_ukf_replay.ps1 `
  -SkipToolBuild `
  -RunId ofm_10728325 `
  -KnownStationarySeed `
  -Output TestResults\open_floor_ukf_replay_metrics_ofm_10728325 `
  -SampleCsv TestResults\open_floor_ukf_replay_metrics_ofm_10728325\ofm_10728325_accel_compare.csv `
  -Metrics context,accel_compare
```

Notes:

- The runner builds only `Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.vcxproj` in `Release|x64`; it links against the existing `MazeMap.lib` and runs against the existing `MazeMap.dll`.
- Before replay, the runner checks that `MazeMap.dll` and `MazeMap.lib` exist and are not older than the authoritative UKF sources.
- If git is available, the MazeMap freshness gate ignores timestamp-only touched files and blocks only on newer files with actual staged or unstaged content changes.
- The tool binds each decoded CSV to its sibling `open_floor_main.sidecar` and uses the bound `logging.txt` when present.
- It ignores the highest `section_id` in each run by default so the known failed final section does not contaminate the batch report.
- The generated report now includes section-phase error association tables and writes `section_phase_summary.csv` so agents can see which canonical `section_id` + `phase_id` buckets concentrate estimator error.
- `-KnownStationarySeed` seeds replay from the canonical stationary open-floor marker `C` state instead of the first logged UKF state.
- `-Tuning` loads a simple `key=value` override file and the report writes `aggregate_metrics.json` for machine scoring.
- `-SampleCsv` exports one replay-aligned per-sample CSV for the selected `-RunId`.
- `-FeedforwardSampleCsv` exports the per-sample feedforward-path audit matrix for the selected `-RunId`.
- `-Metrics` accepts comma-separated metric names or aliases. Current aliases are `context`, `accel_compare`, and `speed_compare`.
- The default sample-export metric set is the accel comparison layout shown above if you pass `-SampleCsv` without `-Metrics`.
- Unless you pass `-SkipCompetitionArchive`, the runner also invokes `tooling\analyze_competition_feedforward.py` against `TestResults\Competition Testing Data` by default and writes `competition_feedforward_report.txt` under the replay output directory.
- `-CompetitionArchiveRoot` overrides the default archival competition log root used by that additional check.

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

## `build_feedforward_tensor.py`

Purpose: scan every feedforward-relevant capture under `TestResults`, quantize the observed transition data into a 4D tensor over `(present velocity, present yaw rate, desired acceleration, desired alpha)`, and provide a small evaluator for recovering left/right raw command estimates from that tensor.

What it reads:

- Every decoded `open_floor_main.csv` under `TestResults`.
- Every archival competition `diag*.csv` under `TestResults`.
- Any `sensor_feedforward.csv` replay export already present.
- Any `feedforward_paths.csv` replay export already present, using one canonical `path_id` (default `state_closed_velocity`) so path-matrix duplicates do not get counted multiple times.

What it writes:

- `feedforward_tensor.json`: tensor axes, populated-cell means/stddevs, counts, and source totals.
- `feedforward_tensor_cells.csv`: one row per populated tensor cell for inspection in spreadsheets.
- `feedforward_tensor_summary.txt`: human-readable build summary.

Build example:

```powershell
python tooling\build_feedforward_tensor.py build
```

Evaluation example:

```powershell
python tooling\build_feedforward_tensor.py evaluate `
  --tensor-json TestResults\feedforward_tensor_dataset\feedforward_tensor.json `
  --present-velocity-mps 0.45 `
  --present-yaw-rate-radps 1.2 `
  --desired-accel-mps2 6.0 `
  --desired-alpha-radps2 30.0
```

Method notes:

- For paired logs, the script uses the current row's logged raw commands and the next row's sensor state to derive the desired body acceleration and yaw acceleration over that tick horizon.
- Forward velocity is treated as the mean of left/right encoder velocity when wheel-side sensors are available.
- Yaw-rate input is taken from `gyro_raw_radps - gyro_bias_radps` when available, otherwise from the schema's corrected yaw-rate field.
- Tensor axes are fit from a deterministic reservoir sample using quantile centers so the grid stays compact even though the raw corpus is large.
- Evaluation first tries multilinear interpolation across populated neighboring cells and falls back to the nearest populated cell when the local neighborhood is empty.
