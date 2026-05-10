# VehicleState, Estimator, and SrUkfCore Refactor Spec

Date: 2026-05-10

## Purpose

Converge estimator state ownership around `VehicleState` and remove public APIs that allow arbitrary code to mutate or inspect raw UKF internals. The current design duplicates the live estimate between `VehicleState` and `SrUkfCore`/`UKF`, exposes raw `StateVector`/covariance details publicly, and provides scalar setters that make state and covariance invariants easy to violate.

This refactor should leave one authoritative live estimate, tightly controlled covariance mutation, and a private UKF core that performs algorithmic work for `Estimator`.

This is also an engine-swap hardening pass. The current `SrUkfCore` and `PlantModel` are likely to be replaced soon, and that replacement may bring a different internal state vector entirely. The point of this cleanup is to keep the rest of the robot from depending on the present UKF vector layout, matrix types, and plant-engine details so the estimator/plant replacement can happen behind a stable `VehicleState`/`Estimator` boundary.

## Core Ownership Rules

1. `VehicleState` owns the live estimate.
   - This includes the state vector and square-root covariance.
   - `VehicleState` is the authoritative runtime object of global importance.
   - Other systems may read domain-level state through `VehicleState` getters.

2. `Estimator` owns the estimator lifecycle.
   - `Estimator` coordinates prediction and measurement updates.
   - `Estimator` owns `SrUkfCore` as a private implementation member.
   - Production callers interact with `Estimator`, not with `SrUkfCore`.

3. `SrUkfCore` owns UKF algorithm machinery only.
   - It may own reusable bounded math workspace, tuning, diagnostics, policy state, and helper state needed by the estimator algorithm.
   - It must not own a second live estimate.
   - It must operate on the `VehicleState&` supplied by `Estimator`.

4. Tests may exercise `SrUkfCore`, but not by making it a convenient public subsystem.
   - Direct `SrUkfCore` tests are white-box tests.
   - They must wire `SrUkfCore` the same way `Estimator` does: construct a real `VehicleState`, then construct/bind the core to that state.
   - Do not add fallback/local estimate ownership to make tests easier.

## Current Problems To Remove

### Public Raw Estimate Types

`VehicleState::StateVector` and the corresponding covariance matrix type are currently public. That gives callers a raw index-based view of the estimate and encourages state/covariance mutation outside the owner.

Target:

- No public `VehicleState::StateVector`.
- No public raw covariance type exposed as general API.
- If internal math needs these aliases, move them to an internal estimator namespace or private implementation scope.
- Tests must not rely on public raw vectors as the normal setup/assertion path.

### Public Scalar Setters

The following public `VehicleState` setters should be removed from the public API:

- `SetPosition`
- `SetVelocity`
- `SetLateralVelocity`
- `SetOrientation`
- `SetRotationalVelocity`
- `SetWheelSpeedLeft`
- `SetWheelSpeedRight`
- `SetGyroBiasZ`
- scalar/time-like setters where they are only supporting estimator internals
- all `Set*Var(...)` functions
- `SetVarianceValues(...)`
- public `SetCovariance(...)`
- public `SetSqrtCovariance(...)`

The `Set*Var(...)` functions are especially bad because they look like cheap setters but internally reconstruct covariance and run LLT factoring.

Target:

- Public `VehicleState` should expose domain reads and tightly controlled high-level operations only.
- Bulk mutation of the estimate is allowed only through a private/friend commit path used by `Estimator`/`SrUkfCore`.
- Covariance setting must be a full-estimate operation, not piecemeal.

### Public `NormalizeStateVector`

`VehicleState::NormalizeStateVector(...)` should not be public.

Target:

- Angle normalization is an internal invariant of committing or deriving estimate state.
- Callers should not normalize raw state vectors because callers should not own raw state vectors.

### `SrUkfCore` as Public Escape Hatch

`Estimator::ukf()` currently exposes the core, which exposes state/covariance via `_filter`.

Target:

- Remove public `Estimator::ukf()` access.
- Replace legitimate public needs with focused `Estimator` methods or `VehicleState` getters.
- If tests need direct core access, use a white-box test harness in test code, not a production public getter.

### Duplicate Live Estimate

Current production flow copies `_core.state()` and `_core.covariance()` into runtime `VehicleState` through `Estimator::SyncRuntimeState()`.

Target:

- Eliminate periodic sync as an ownership bridge.
- There should be only one live estimate: the `VehicleState` instance bound to `Estimator`.
- `SrUkfCore` reads and commits against that `VehicleState`.

## Required Runtime Shape

### SharedRobotRuntime

`SharedRobotRuntime` already owns:

- `runtimeState`
- `estimator`

The construction order is already compatible with binding `Estimator` to `runtimeState`.

Target construction shape:

```cpp
SharedRobotRuntime::SharedRobotRuntime()
    : runtimeState()
    , estimator(runtimeState, MazeMap::PlantParams::Default(), plantModel)
    , drive(plantModel, estimator, MazeMap::Config::kDriveBasePDCluster)
{
}
```

Exact constructor parameters can differ, but `Estimator` must receive a `VehicleState&` and must not use a nullable pointer or local fallback state.

### Estimator

Target responsibilities:

- Own private `SrUkfCore _core`.
- Hold `VehicleState& _state`.
- Provide prediction/update entry points.
- Provide fault state and map evidence coordination.
- Provide public access to `VehicleState` only through domain-level state references or getters.

Proposed shape:

```cpp
class Estimator
{
public:
    Estimator(
        VehicleState& state,
        const PlantParams& params = PlantParams::Default(),
        const PlantModel& plantModel = PlantModel()) noexcept;

    VehicleState& State() noexcept;
    const VehicleState& State() const noexcept;

    bool Predict(float dtSeconds, const App::Internal::CommandVector& control) noexcept;

    MeasurementUpdateResult UpdateEncoderPair(float dtSeconds, bool updateYaw = true) noexcept;
    MeasurementUpdateResult UpdateYawRate() noexcept;
    MeasurementUpdateResult UpdatePlanarAccel() noexcept;
    FrontPairUpdateResult UpdateFrontPair(const Maze& maze, bool freezeMapMutation = false) noexcept;
    WallUpdateResult UpdateSideSensor(RelativeDirection which, const Maze& maze, bool freezeMapMutation = false) noexcept;

private:
    VehicleState& _state;
    SrUkfCore _core;
};
```

Notes:

- Names should follow existing project style, but do not keep `predictImpl`.
- Use either overloads or default parameters. Do not use `predictImpl` as a public/private naming crutch.
- Remove `SyncRuntimeState()`.
- Remove `_localRuntimeState`.
- Remove nullable `_runtimeState`.
- Remove `AttachRuntimeState(...)` unless a real production owner requires rebinding. Rebinding a live estimator state should be treated as suspicious.

### SrUkfCore

Target responsibilities:

- Private implementation detail of `Estimator`.
- May be directly constructed only by white-box tests with explicit `VehicleState&`.
- Owns UKF algorithm state, reusable math workspace, tuning, diagnostics, policy state.
- Does not own a live estimate copy.

Proposed shape:

```cpp
class SrUkfCore final
{
public:
    SrUkfCore(
        VehicleState& state,
        const PlantParams& params = PlantParams::Default(),
        const PlantModel& plantModel = PlantModel()) noexcept;

    bool Predict(float dtSeconds, const App::Internal::CommandVector& control) noexcept;

private:
    VehicleState& _state;
};
```

The actual class will keep many existing diagnostics and helpers, but the estimate must come from `_state`, not `_filter.state()`.

## Predict API Cleanup

### Remove `predictImpl`

`predictImpl` is bad naming here. It hides the real operation behind an implementation suffix.

Target:

- If only one public behavior exists, name the private worker after the exact behavior, or use overloads/default parameters.
- If the only difference is the loop hook, use an overload:

```cpp
bool Predict(float dtSeconds, const CommandVector& control) noexcept;

template <typename LoopHook>
bool Predict(float dtSeconds, const CommandVector& control, LoopHook&& loopHook) noexcept;
```

### Remove Battery Voltage Argument

Battery voltage is not directly measured. The estimator should use a nominal configured value instead of accepting battery voltage as an argument.

Target:

- Remove `batteryVoltageV` from `Estimator::Predict(...)` and `SrUkfCore::Predict(...)`.
- Resolve nominal voltage internally from the authoritative vehicle/plant parameter owner.
- If the plant model currently needs a voltage input, feed it the nominal value in one internal place.

### Move Fan Duty Source To VehicleState

`fanDutyCycle` should not be passed separately into predict. It should come from `VehicleState` or a tightly owned runtime state/context object attached to it.

Target:

- Add a controlled way for runtime code to stage the current fan duty cycle into `VehicleState`.
- `SrUkfCore` reads the current fan duty cycle from `VehicleState`.
- Do not add broad public scalar setters just to do this. Prefer a bulk per-tick runtime context/snapshot update operation.

Possible shape:

```cpp
struct RuntimeTickContext
{
    SensorSnapshot sensors;
    float fanDutyCycle = 0.80f;
    std::uint32_t timestampUs = 0U;
    float dtSeconds = 0.0f;
};

class VehicleState
{
public:
    const SensorSnapshot& GetSensorSnapshot() const noexcept;
    float GetFanDutyCycle() const noexcept;

private:
    friend class Estimator;
    friend class LoopController;
    void CommitRuntimeTickContext(const RuntimeTickContext& context) noexcept;
};
```

This shape is illustrative. The important rule is that fan duty is read from `VehicleState`, not threaded through estimator calls as an unrelated parameter.

## SensorSnapshot And Measurement Source

`SensorSnapshot` is already staged onto `VehicleState`, but the estimator still receives decomposed `EncoderObs`, `ImuAccelObs`, `WallObs`, and raw gyro arguments.

Target:

- Use the `SensorSnapshot` stored on `VehicleState` as the tick measurement source.
- IMU and wall update paths should derive observations from `VehicleState::GetSensorSnapshot()`.
- Encoder data is currently produced separately by `DriveBase::ConsumeEncoderObservation(...)`. Fold the encoder measurement into the same tick-owned measurement bundle before UKF update, or add a similarly controlled field in the staged runtime tick context.
- Avoid preserving parallel observation paths unless there is a clear domain reason.

Do not put measurements into the state vector. `VehicleState` owns both:

- the live estimate, internally represented by the UKF state and square-root covariance,
- current tick measurement/context data, represented by `SensorSnapshot` and adjacent tick context.

These are separate concepts and should remain separate internally.

## Loop Hook Requirement

The 1000 Hz cadence requires the existing interleaved callback pattern.

Important correction:

- The current hook does not suspend/resume UKF work.
- It is an inline callback invoked at convenient points.
- `SrUkfCore` does not need phase/order state for this.

Target:

- Preserve loop hook call points or equivalent call frequency.
- Do not introduce resumable phase machinery.
- Do not make `SrUkfCore` know the order of external runtime work.
- Keep the hook as a callback that UKF math calls between expensive chunks.

## VehicleState API Target

Public API should expose domain-level reads and operations, not raw storage.

Keep or add public reads such as:

- position getters
- orientation getter
- velocity getters
- wheel-speed getters
- gyro-bias getter
- variance/covariance reads only if they are truly needed by runtime consumers, and preferably as narrow scalar/domain reads
- sensor snapshot read
- fan duty/current tick context read, if needed

Remove public writes such as:

- scalar state setters
- scalar variance setters
- raw state vector setters
- covariance setters
- raw normalization helpers

Internal/friend commit path:

```cpp
class VehicleState
{
private:
    friend class Estimator;
    friend class SrUkfCore;

    using EstimateVector = ...;
    using EstimateSquareRootCovariance = ...;

    const EstimateVector& EstimateVectorForEstimator() const noexcept;
    const EstimateSquareRootCovariance& SqrtCovarianceForEstimator() const noexcept;

    bool CommitEstimate(
        const EstimateVector& state,
        const EstimateSquareRootCovariance& sqrtCovariance) noexcept;

    bool CommitEstimateFromCovariance(
        const EstimateVector& state,
        const EstimateMatrix& covariance) noexcept;
};
```

The names are illustrative. The important rules are:

- The raw types are private/internal.
- The commit path is full-estimate and invariant-preserving.
- Covariance factoring happens only on full-estimate commits or carefully justified estimator internals.
- No public caller can set one coordinate or one variance.

## UKF Internal Refactor Direction

The generic `UKF` object currently owns:

- `_state`
- `_sqrtCovariance`
- `_sqrtProcessNoise`
- `_sigmaPoints`
- `_sigmaMean`
- weights and strategy fields

Target:

- Move live `_state` and `_sqrtCovariance` ownership out to `VehicleState`.
- Keep reusable workspace in the estimator/core where justified.
- Avoid generic hard-sized buffers for modes that production does not use.
- Production currently forces simplex sigma points, so workspace should be sized for simplex unless standard sigma points remain a real supported mode.

RAM discipline:

- Do not reduce matrix count by increasing hard matrix footprint.
- Prefer smaller, purpose-built reusable buffers over broad catch-all matrices.
- Avoid dynamic `MatrixXf` in runtime estimator paths.
- Avoid reconstructing full covariance just to modify a small block unless no square-root update path exists.
- Treat full covariance factoring as an expensive controlled operation.

Specific current hotspots:

- `UKF::Predict()` creates `priorSigma`, `predictedSigma`, and a large stacked QR matrix.
- `UKF::Update()` creates measurement sigma, stacked QR, cross covariance, solved lower/upper, gain, update columns, and updated sqrt locals.
- `ProjectMaskedStateAndSquareRootCovariance()` uses dynamic `MatrixXf` submatrices and LLT factoring.
- `applyWheelSpeedConstraint()` reconstructs full covariance and calls `setState(...)`.
- stationary constraint code constructs a temporary `VehicleState` through public scalar setters, then extracts it back into a raw vector.

These are not all required in the first patch, but they define the cleanup direction and review standard.

## Stationary Constraint Ownership

Current `SrUkfCore::applyStationaryZeroMotionConstraint(...)` builds a temporary `VehicleState`, calls scalar setters and `SetCovariance(...)`, applies a constraint, then extracts fields.

Target:

- Move the stationary zero-motion constraint into a private/full-estimate operation on `VehicleState` or an internal estimator helper that operates on private estimate types.
- Do not assemble `VehicleState` through public setters.
- Do not expose raw vectors to do the same thing externally.
- Store only the pose reference data actually needed.

`_prePredictCovariance` and `_stationaryCandidatePoseReferenceCovariance` are full `9x9` matrices today. If only the pose block is needed, replace them with a compact pose covariance block.

## Testing Strategy

### Production-Level Tests

Prefer tests through `Estimator` and `VehicleState`:

```cpp
VehicleState state = MakeInitialVehicleState(...);
Estimator estimator(state, params, plant);
estimator.Predict(dt, control);
```

Assertions should use `VehicleState` getters or narrow test-only inspection helpers.

### SrUkfCore White-Box Tests

Direct `SrUkfCore` tests are allowed, but must mirror estimator internals:

```cpp
struct SrUkfCoreHarness
{
    VehicleState state;
    PlantModel plant;
    SrUkfCore core;

    explicit SrUkfCoreHarness(const PlantParams& params = PlantParams::Default())
        : state(...)
        , plant()
        , core(state, params, plant)
    {
    }
};
```

Rules:

- No default `SrUkfCore core;` unless the constructor requires a `VehicleState&`.
- No local fallback estimate inside `SrUkfCore`.
- No public `StateVector` just for test setup.
- If white-box tests require raw inspection, add a test-only/internal accessor with a narrow scope, not production public API.

### Test Migration

Expected impacted tests/tools:

- `SrUkfCore*Test.cpp`
- `SrUkfCoreTestSupport.h`
- `EstimatorTestSupport.h`
- `VehicleStateTest.cpp`
- `DriveBaseTest.cpp`
- `SharedRuntimeTest.cpp`
- `Tools/ukf_predict_diag.cpp`
- `Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp`
- `MazeSimulation/MazeSimulation.cpp`

Do not preserve old API for these callers. Migrate them.

## Migration Order

Use copy-delete-stitch discipline for ownership moves. Do not leave old and new compiled ownership paths side by side.

Recommended order:

1. Add private/internal estimate commit/read methods to `VehicleState`.
2. Add a controlled initial-estimate construction/reset path on `VehicleState`.
3. Bind `Estimator` to `VehicleState&`; remove nullable runtime state pointer and local fallback state.
4. Bind `SrUkfCore` to `VehicleState&` through `Estimator`.
5. Remove `Estimator::SyncRuntimeState()`.
6. Remove public `Estimator::ukf()` and replace call sites with focused methods.
7. Remove battery voltage and fan duty parameters from predict APIs.
8. Stage fan duty and measurement context through `VehicleState`.
9. Convert update paths to read from `VehicleState`/`SensorSnapshot` where feasible.
10. Remove public scalar state setters and variance setters.
11. Remove public `NormalizeStateVector`.
12. Make raw estimate vector/covariance aliases private/internal.
13. Migrate tests/tools to the new harness/API.
14. Remove dead declarations, includes, and compatibility shims.

If a step cannot be completed without preserving parallel ownership, stop and report the blocker rather than leaving a staged half-migration.

## Review Checklist

Reject the refactor if any of these remain:

- `VehicleState::StateVector` is publicly accessible.
- Public `Set*Var(...)` functions remain.
- Public scalar `VehicleState` state setters remain for estimator coordinates.
- Public `NormalizeStateVector(...)` remains.
- Public `SetCovariance(...)` or `SetSqrtCovariance(...)` remains as a general caller API.
- `Estimator::ukf()` exposes `SrUkfCore`.
- `Estimator::SyncRuntimeState()` still copies a core-owned estimate into runtime state.
- `SrUkfCore` owns a live estimate copy.
- `SrUkfCore` has a default constructor that creates hidden estimate ownership.
- Tests rely on convenient standalone `SrUkfCore` construction instead of wiring a real `VehicleState`.
- Predict takes `batteryVoltageV`.
- Predict takes `fanDutyCycle` instead of reading staged runtime state.
- `predictImpl` remains as the operation name.
- Runtime estimator paths allocate dynamic `MatrixXf` for ordinary update work.
- Full covariance setting/factoring is used for scalar variance tweaks or other piecemeal changes.

## Verification Requirements

Before testing, check that active binaries correspond to latest changes by timestamp/build metadata. Do not build from scratch unless required by the current task.

Run release-mode unit tests if supported by the project. At minimum, verify:

- `VehicleStateTest`
- `SrUkfCore*` tests
- `Estimator`/drive integration tests
- shared runtime tests that exercise the 1000 Hz update path

Also verify host build and Teensy build paths if the touched files affect cross-build behavior.

## Non-Goals For First Refactor

These are important but should not be mixed into the ownership migration unless needed to keep the build working:

- Retuning UKF noise values.
- Rewriting the UKF algorithm from scratch.
- Replacing `SrUkfCore` or `PlantModel` during this pass.
- Preserving the current UKF state-vector layout as a public or long-term contract.
- Changing the 1000 Hz callback cadence.
- Introducing resumable UKF phase execution.
- Changing top-level mode behavior.
- Reworking unrelated drive/pathfinder ownership.

## Summary

The desired end state is:

- `VehicleState` is the only owner of the live estimate.
- `Estimator` is the public estimator coordinator.
- `SrUkfCore` is a private/white-box UKF implementation detail wired to a real `VehicleState`.
- Measurements and fan/runtime context are staged on `VehicleState`, not passed as scattered parameters.
- Raw state vector/covariance APIs are not public.
- Covariance mutation is full-estimate, rare, and tightly controlled.
- The loop hook cadence remains intact.
