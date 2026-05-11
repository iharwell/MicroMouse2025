# SrUkfCore Header Cleanup Context Handoff

Date: 2026-05-10

This file records the current state of the `SrUkfCore` header/API cleanup after resuming from the prior handoff and continuing the architectural convergence work.

## User Intent

The user challenged `SrUkfCore.h` as architecturally unacceptable:

- launch/reversal trigger logic should not be public UKF API,
- `gyroBiasAnchor` appears to duplicate `VehicleState::kBgz`,
- public parameter/config bags in the header violate the repo's acceptable-type rules,
- command/feedforward solving through `SrUkfCore` is forbidden.

The user also explicitly asked to run the architecture smell detection script on this class as work proceeds.

Use this scanner command while continuing:

```powershell
python tooling/architecture_smell_scan.py --changed --base HEAD --production-only --format text --max-findings 0 --fail-on none | Select-String -Pattern 'SrUkfCore' -Context 0,2
```

The scanner does not accept direct file arguments. It supports `--changed`, `--full`, and `--staged`.

## Important Worktree State

The worktree is broadly dirty. Do not revert unrelated edits. Treat dirty files outside this cleanup as user or prior-agent work unless proven otherwise.

Files intentionally touched for this `SrUkfCore` cleanup:

- `MazeMap/MazeMap/SrUkfCore.h`
- `MazeMap/MazeMap/SrUkfCore.cpp`
- `MazeMap/MazeMap/Estimator.cpp`
- `MazeMap/MazeMap/VehicleState.cpp`
- `MazeMap/MazeMap/PlantModel.h`
- `MazeMap/MazeMap/PlantModel.cpp`

No staging or commit has been performed.

## Completed Architecture Work

### Removed SrUkfCore command/feedforward facade API

Deleted from `SrUkfCore.h` / `SrUkfCore.cpp`:

- `resolveYawRateForFeedforward(...)`
- `solveAlignedDriveCommands(...)` overloads
- `solveAlignedDriveCommandsForVelocityTarget(...)` overloads
- `alignedVelocityTargetTechnicalLimits(...)`
- `evaluateAlignedFeedforwardOffline(...)`

PlantModel remains the owner for plant equations and command/feedforward solving. Do not restore these as SrUkfCore wrappers.

### Demoted public SrUkfCore policy/test surface

Moved or retained as private implementation details rather than public UKF API:

- `OperatingMode`
- `setRuntimeContext(...)`
- `ComputeNonholonomicSigmaMps(...)`
- `IsStationaryCandidate(...)`
- `HasLaunchOrReversalTrigger(...)`
- `HasInconsistentOrSaturatedTrigger(...)`
- `ClassifyOperatingMode(...)`
- `IsYawValidForFeedforward(...)`

Removed public diagnostic getters exposing internals, including:

- `operatingMode()`
- `gyroBiasAnchorRadps()`
- `yawConsistencyLowPassRadps()`
- NHC residual/sigma getters
- pivot scrub telemetry getters
- innovation/NIS getters
- grip/regrip/feedforward policy getters
- applied torque getters

The resulting test breakage is expected fallout. Do not restore this public API just to satisfy tests.

### Removed public SrUkfCore tuning/config bags

Removed public `SrUkfCore::RuntimeTuning` and `SrUkfCore::ModeProcessNoiseTuning` from `SrUkfCore.h`.

Removed public functions:

- `BuildDefaultRuntimeTuning()`
- `GetRuntimeTuning()`
- `SetRuntimeTuning(...)`
- `ResetRuntimeTuning()`

Then removed the temporary cpp-local `SrUkfRuntimeTuning` struct and `Tuning()` function as well. `SrUkfCore.cpp` now uses direct cpp-local constants for the former public tuning constants.

Examples of constants now local to `SrUkfCore.cpp`:

- `kGeneralEncoderLinearSpeedSigmaMps = 0.021187f`
- `kGeneralEncoderYawRateSigmaRadps = 0.111268f`
- `kStationaryEncoderVelocitySigmaMps = 0.002936f`
- `kEncoderPairNisThreshold = 13.81551f`
- `kImuYawRateVarianceRadps2 = 1.2e-6f`
- `kImuYawRateSigmaRadps = 0.0010954451f`
- `kGyroBiasProcessVarianceMovingRadps2PerSample = 0.0f`
- `kGyroBiasProcessVarianceStationaryRadps2PerSample = 3.0e-16f`
- `kGyroBiasInitialVarianceUnseededRadps2 = 3.05e-4f`
- `kImuAccelSigmaMps2 = 0.569900f`
- pivot scrub, NHC, and initial stationary gyro-bias seed constants.

`ModeProcessNoiseConfig` was replaced with an encapsulated file-local `ModeProcessNoise` class storing `std::array<float, 6>` and exposing named getters. This eliminated the scanner's local config-bag finding for that code.

### Removed SrUkfCore dependency on copied/prepared PlantModel params

`SrUkfCore.cpp` no longer uses `_params` or `_preparedParams`.

Important migrations:

- `WriteDebugTextDump(...)` delegates plant parameter output to `_plantModel.WriteUkfPlantDebugTextDump(context, sink)`.
- `IsPivotScrubCandidate(...)` uses `_plantModel.measuredLinearSpeedMps(observation)`.
- grip/regrip and frozen-policy state use PlantModel behavior methods instead of copied prepared fields.
- closure pseudo-measurements use PlantModel wheel velocity methods.
- `predictImpl(...)` calls `_plantModel.integrateAppliedBankTorquesWithEnvelopeScales(...)`.
- encoder covariance/noise/variance paths call PlantModel encoder methods.
- IMU planar acceleration update calls the public PlantModel overload using battery voltage.
- wall prediction uses `_plantModel.wallObservationNoHitRangeM()` and `Vehicle` sensor mounts.

Deleted duplicate SrUkfCore encoder helper definitions:

- `ComputeGeneralEncoderPairCovarianceRadps`
- `ComputeGeneralEncoderPairSqrtNoise`
- `ComputeStationaryEncoderOmegaSigmaRadps`
- `ComputeEncoderPairSqrtNoise`
- `ComputeMeasuredLinearSpeedMps`
- `ComputeMeasuredLinearSpeedVarianceMps2`
- `ComputeMeasuredYawRateRadps`
- `ComputeMeasuredYawRateVarianceRadps2`
- `ComputeMeasuredWheelVarianceRadps2`

`ComputeMeasuredLinearSpeedVarianceMps2` was also removed from `SrUkfCore.h`.

### Added PlantModel behavior methods

Added to `PlantModel.h` / `PlantModel.cpp` so SrUkfCore can ask PlantModel for plant-owned facts instead of owning copied params:

- `Eigen::Vector2f wheelLinearVelocityFromBodyState(const StateVector& state) const noexcept;`
- `float sustainedCombinedAccelerationUsage(float accelerationMps2) const noexcept;`
- `float nominalCombinedAccelerationUsage(float accelerationMps2) const noexcept;`
- `float peakCombinedAccelerationUsage(float accelerationMps2) const noexcept;`
- `float stopExitYawRateUsage(float yawRateRadps) const noexcept;`

Implementations use PlantModel's `_preparedParams` internally.

### VehicleState and Estimator cleanup

`VehicleState.cpp` no longer includes `SrUkfCore.h` or calls `SrUkfCore::GetRuntimeTuning()`.

It uses local fixed stationary-check constants:

- linear speed threshold: `0.002936f`
- yaw-rate threshold: `3.0f * 0.0010954451f`
- wheel radius from `Vehicle::GetDriveWheelRadiusM()`

`Estimator::ResetForSessionTransition(...)` no longer falls back to `_core.gyroBiasAnchorRadps()`. It uses:

1. `VehicleState::kBgz` from `_core.state()`,
2. `_runtimeState->GetGyroBiasZ()`,
3. zero fallback.

## Current Scanner Status

Run:

```powershell
python tooling/architecture_smell_scan.py --changed --base HEAD --production-only --format text --max-findings 0 --fail-on none | Select-String -Pattern 'SrUkfCore' -Context 0,2
```

Current expected `SrUkfCore` findings:

- P0 `new-simple-classifier`: `SrUkfCore.h`, `enum OperatingMode`.
- P0 `file-scope-mutable-state`: likely parser false positive around a multiline declaration near `IsPivotScrubCandidate`.
- P1 `_regripRecovery` has 8 parameters in `SrUkfCore.cpp`; the scanner appears to be mis-parsing the constructor initializer list as a wide interface.
- P1 wide/flag helper signatures in `SrUkfCore.h`: `setRuntimeContext`, `HasLaunchOrReversalTrigger`, `HasInconsistentOrSaturatedTrigger`, `ClassifyOperatingMode`, `IsYawValidForFeedforward`, `ComputeVelocityTargetBodyAction`, `IsPivotScrubCandidate`.

The scanner no longer reports the cpp-local `SrUkfRuntimeTuning` or `ModeProcessNoiseConfig` bags because they were removed/reworked.

Do not broaden into unrelated scanner findings in the dirty tree unless explicitly tasked.

## Searches That Should Currently Pass

These production searches should return no hits:

```powershell
rg -n "SrUkfCore::(RuntimeTuning|ModeProcessNoiseTuning|BuildDefaultRuntimeTuning|GetRuntimeTuning|SetRuntimeTuning|ResetRuntimeTuning)|SrUkfCore::k|gyroBiasAnchorRadps\(" MazeMap/MazeMap
rg -n "\b_params\b|\b_preparedParams\b" MazeMap/MazeMap/SrUkfCore.cpp MazeMap/MazeMap/SrUkfCore.h
rg -n "ComputeGeneralEncoderPair|ComputeStationaryEncoder|ComputeEncoderPairSqrtNoise|ComputeMeasuredWheelVariance|ComputeMeasuredYawRate|ComputeMeasuredLinearSpeed" MazeMap/MazeMap/SrUkfCore.cpp MazeMap/MazeMap/SrUkfCore.h
```

Tests still contain many references to removed SrUkfCore constants, getters, and helper APIs. Treat those as test fallout, not as authority to restore the old API.

## Build and Verification Status

First `codex_verify/build_and_verify_latest.cmd --no-pause` run failed in Teensy compile with one production error:

- `SrUkfCore.cpp:1130: error: 'operatingModeId' was not declared`

That was fixed by replacing the leftover helper call in `WriteDebugTextDump(...)` with a direct cast from `_operatingMode`. An unused file-local `HeadingUnitFromYaw` helper was also removed.

Second `codex_verify/build_and_verify_latest.cmd --no-pause` run was interrupted by the user, but the log continued far enough to show:

- Teensy compile completed successfully in about 190.6 seconds.
- Host Release build started and compiled production `MazeMap`.
- Host Release build then failed while compiling `MazeMapTest`.

Latest relevant log:

```text
C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260510_175335_761.txt
```

Representative test compile fallout:

- tests construct `SrUkfCore` from `PlantParams`, but the current constructor is `SrUkfCore(const PlantModel&) noexcept`,
- tests access private/removed `SrUkfCore::OperatingMode`,
- tests access removed getters such as `operatingMode()`, `appliedLeftBankTorqueNm()`, `appliedRightBankTorqueNm()`, `exactStationaryLock()`, `closureResidualLeftMps()`, and `closureResidualRightMps()`.

Production/Teensy compile appears clean after the latest fix. Host release verification is blocked by tests that still target removed noncanonical API.

There may be lingering shell/cmd processes from the interrupted build. Inspect before launching another long verification if needed:

```powershell
Get-Process | Where-Object { $_.ProcessName -match 'arduino|powershell|cmd|arm-none-eabi|gcc|g\+\+' } | Select-Object Id,ProcessName,StartTime,Path
```

Do not kill processes unless necessary or explicitly requested.

## Immediate Next Steps

1. Do not restore removed public SrUkfCore API, constants, or getters for tests.
2. Update SrUkfCore tests to construct through the canonical `PlantModel` path or move assertions to public SrUkfCore behavior / PlantModel behavior rather than internals.
3. Run the architecture smell scanner as you go, focused on `SrUkfCore` findings.
4. Rerun `codex_verify/build_and_verify_latest.cmd --no-pause` after test compile fallout is addressed.
5. If host Release test binaries are current and build succeeds, run the Release tests according to the repo verification path.
6. Consider private helper signature cleanup only if staying inside the `SrUkfCore` task boundary. Do not broaden into unrelated dirty-tree findings.

## Caution

Do not restore removed public SrUkfCore helpers/constants just to make tests compile. The user explicitly treats tests as non-authoritative for ownership.

Do not preserve old and new implementations in parallel. If a behavior needs a new owner, move it fully to the authoritative owner and delete the obsolete access path.

Do not chase every scanner finding in the repository. Continue the focused SrUkfCore cleanup unless the user expands scope.
