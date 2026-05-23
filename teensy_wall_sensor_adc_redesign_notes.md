# Teensy 4.1 Wall-Sensor ADC Timing Redesign Notes

Date: 2026-04-14

Scope: observations and implementation guidance only. This note does not change code. It documents the current timing path, why the pre-UKF sensing block is larger than expected, and a concrete redesign aimed at a practical budget of about `3-4 us` per sensor sample while removing explicit wait loops from the control path.

## Executive Summary

The current control loop does apply motor actuation at tick start, but the timing logs do not expose that accurately. The large pre-UKF timing block is not an encoder transaction. It is mostly wall-sensor and instrumentation work.

The largest avoidable cost is that the wall-sensor dark samples are currently taken with generic `analogRead()` calls before UKF predict begins, and each wall-sensor sample path does two `analogRead()` calls. On Teensy 4.x, the stock PJRC `analogRead()` path is configured conservatively for noise and generality, not for minimum latency. It is not configured for a `3-4 us` per-sensor sample budget. With the current code shape, that means:

- four dark samples are read before UKF predict starts,
- each dark sample performs two blocking ADC calls,
- all four wall sensors are wired to `ADC1`, so the current board cannot parallelize those reads across both ADC peripherals,
- the existing asynchronous wall-sensor sweep still contains blocking completion and explicit settle waits after the estimator callback.

The good news is that the codebase already has the right seam for the intended fix:

- `DriveBase::UpdateOdometry(...)` already accepts a UKF loop hook,
- the UKF predict and update paths already call that hook many times,
- the wall-sensor sweep can be reworked into a no-wait state machine that advances only when a single time check says the current LED phase has settled.

The recommended implementation is:

1. take immediate dark samples for all four sensors at tick start with a fast direct `ADC1` path,
2. turn on the front LEDs immediately afterward,
3. enter UKF predict immediately,
4. let the existing UKF loop hook opportunistically advance the front, left, and right lit phases with no waiting,
5. finish the sweep before predict end if predict remains longer than the total required optical settle window,
6. remove the blocking `CompleteAsyncWallSensorSweepRead(...)` and explicit ambient-settle wait from the control path.

## Current Repo Observations

### 1. Actuation is applied at tick start

`LoopController::Run()` applies the queued control immediately after `tickStartUs`:

- `MazeMap/MazeMap/LoopController.cpp`
- `ApplyControlAtTickStart(_appliedControl, dtSeconds);`

For raw open-loop commands that path is:

- `LoopController::ApplyControlAtTickStart(...)`
- `DriveBase::CommandOpenLoopRaw(...)`
- `MotorEncoderDrive::setDriveCommand(...)`
- `Platform::WriteMotorPwmCode(...)`

This is a direct pin/PWM write path, not a deferred "latch later" path.

Relevant files:

- `MazeMap/MazeMap/LoopController.cpp`
- `MazeMap/MazeMap/DriveBase.h`
- `MazeMap/MazeMap/MotorEncoderDrive.h`
- `MazeMap/MazeMap/Defines.h`

### 2. The timing field names are misleading

The current timing log names do not reflect the actual operations:

- `encoderLatchUs` is stamped before diagnostic sensor capture begins.
- `encoderReadDoneUs` is stamped before `Drive().UpdateOdometry(...)` is called.
- the actual encoder count consumption happens later inside `DriveBase::ConsumeEncoderCycleSample(...)`.
- `pwmLatchUs` is not the actuation write time. It is currently written during loop finalization.

This means the block between `encoderLatchUs` and `encoderReadDoneUs` is not "encoder hardware time". It is an overloaded pre-estimator sensing bucket.

### 3. The selected control path is currently the diagnostic sensor path

`LoopController::CaptureSelectedTickState(...)` is hard-wired to:

- `CaptureDiagnosticTickState(...)`

That means the loop uses `RuntimeSensorSuite::Capture(...)` in the current path.

### 4. The actual encoder sample point is later than the current log suggests

Encoder counts are consumed inside:

- `DriveBase::ConsumeEncoderCycleSample(...)`

and that happens immediately before:

- `timing->ukfPredictStartUs = micros();`

So the real encoder sample point is much closer to `ukfPredictStartUs` than to the current `encoderReadDoneUs` timestamp.

### 5. The current wall-sensor sweep is only partially asynchronous

`RuntimeSensorSuite::Capture(...)` currently does this:

1. start the async sweep,
2. prime the dark samples immediately,
3. enter the estimator callback,
4. service the sweep opportunistically during the estimator callback,
5. after the estimator callback returns, explicitly force completion if still active,
6. explicitly wait for ambient settle before continuing.

The last two steps are the architectural problem. They reintroduce blocking waits into a path that should be purely opportunistic.

## What Is Actually In the "Questionable Time Block"

The timing block between the current `encoderLatchUs` and `encoderReadDoneUs` is mostly this work:

1. compute the stationary hint,
2. call `RuntimeSensorSuite::Capture(...)`,
3. allocate and initialize the sweep state,
4. prime dark samples for all four wall sensors,
5. turn on the front LED pair,
6. create the service lambdas used by the estimator callback,
7. enter the estimator callback,
8. stamp `encoderReadDoneUs`,
9. call `Drive().UpdateOdometry(...)`.

The heavy part of that block is the dark-sample priming.

### Why the dark-sample priming is expensive

`StartAsyncWallSensorSweepRead(...)` calls `PrimeAsyncWallSensorDarkSample(...)` for each of:

- front-left,
- front-right,
- side-left,
- side-right.

`PrimeAsyncWallSensorDarkSample(...)` does:

- `sample.ambientLight = sensor.ReadLightLevel();`

`WallSensor::ReadLightLevel()` does:

- one throwaway `analogRead(pin)`,
- one real `analogRead(pin)`.

So the current dark-sample startup cost is:

- 4 sensors
- x 2 blocking `analogRead()` calls each
- = 8 blocking ADC conversions before UKF predict starts.

This is the main reason the pre-UKF block is larger than expected even though it is not waiting for optical settle yet.

## Hardware / Platform Findings

### 1. Teensy 4.1 platform facts relevant to this redesign

From PJRC and NXP documentation:

- Teensy 4.1 uses the NXP i.MX RT1062 at up to `600 MHz`.
- Teensy 4.1 has `18` analog inputs.
- the i.MX RT1062 ADC block has `2` 12-bit ADC peripherals.
- NXP documents the ADC family at up to `1 MS/s`.

Sources:

- PJRC Teensy 4.1 product page
- NXP i.MX RT1060 product page
- NXP MCUXpresso MIMXRT1062 ADC driver docs

### 2. The current wall-sensor pins all land on ADC1

Current wall-sensor wiring from `Pins.h`:

- `WS_Side_Left = 20`
- `WS_Side_Right = 21`
- `WS_Forward_Left = 22`
- `WS_Forward_Right = 23`

PJRC's Teensy 4.x `analog.c` channel map shows:

- pin `20/A6` -> ADC channel `15` on `ADC1`
- pin `21/A7` -> ADC channel `0` on `ADC1`
- pin `22/A8` -> ADC channel `13` on `ADC1`
- pin `23/A9` -> ADC channel `14` on `ADC1`

That has an important consequence:

- the current board cannot split the four wall-sensor reads across `ADC1` and `ADC2`,
- any "parallel four-sensor read" design is impossible on the current pinout,
- the correct low-risk design is a fast sequential `ADC1` reader plus a no-wait sweep state machine.

If a future board revision wants to halve active ADC time further, at least one sensor from each concurrently sampled phase needs to move to an `ADC2`-capable pin.

### 3. The stock PJRC `analogRead()` path is conservative, not minimal-latency

The official Teensy 4.x core source shows:

- `analog_num_average = 4` by default,
- `analogReadResolution(12)` selects the 12-bit mode,
- the 12-bit mode comment says `25 clocks` conversion plus `24 clocks` input settling,
- the core uses the asynchronous ADC clock path and high-speed mode,
- `analogRead()` polls for completion and contains a `yield()` path,
- it also handles pad-keeper disable logic generically.

Important consequence:

- the stock path is optimized for general Arduino use, not for control-loop latency,
- the default `4`-sample hardware average makes each `analogRead()` materially slower,
- the current repo never overrides the average count for the wall-sensor path,
- the current wall-sensor code also performs a throwaway conversion before the real one.

### 4. Verified lower bound: the current setup is not aimed at `3-4 us`

This conclusion can be established without guessing the exact ADC clock actually chosen by the PJRC core.

The NXP RT1060 datasheet gives 12-bit conversion time at `Fadc = 40 MHz`:

- short sample, longest `ADSTS`: about `0.85 us`
- long sample, longest `ADSTS`: about `1.25 us`

The PJRC Teensy 4.x core currently configures the stock `analogRead()` path for:

- 12-bit resolution,
- long sample enabled,
- `ADSTS = 3`,
- hardware averaging enabled with `analog_num_average = 4`.

That means even in the best possible case, if the ADC were running at the datasheet's top 12-bit high-speed rate of `40 MHz`, one `analogRead()` still has a hard lower bound of approximately:

- `4 averaged conversions x 1.25 us = 5.0 us`

before software overhead.

`WallSensor::ReadLightLevel()` performs two `analogRead()` calls, so one wall-sensor sample has a hard lower bound of approximately:

- `2 x 5.0 us = 10.0 us`

before software overhead.

That is already more than `2x` the requested `3-4 us` budget, even under a best-case assumption for ADC clocking and with zero extra software cost.

So the current setup is not merely "not optimized" for `3-4 us`. It is structurally incompatible with that target.

### 5. Practical timing estimate for the current path

This is an estimate, not a measured claim, but it is directionally important.

The PJRC core comments imply:

- 12-bit mode: `25 + 24 = 49` ADC clocks per conversion,
- with default hardware averaging of `4`, one `analogRead()` costs roughly `4 x 49 = 196` ADC clocks.

If the effective ADC clock is near the `20 MHz` design target commonly used by the core comments, then:

- one `analogRead()` is about `196 / 20e6 = 9.8 us`,
- one `WallSensor::ReadLightLevel()` is about `2 x 9.8 us = 19.6 us`,
- four dark samples are about `4 x 19.6 us = 78.4 us`,
- and that is before software overhead, GPIO writes, lambda setup, and estimator entry.

That matches the observed fact that the pre-UKF bucket is much larger than a "single-cycle ADC read" mental model would predict.

Even if the exact ADC clock differs, the architectural conclusion does not change:

- the current path is far slower than the `3-4 us` per sensor budget,
- the current setup is not aimed at that budget,
- and it is slow because of the chosen software path, not because the RT1062 ADC hardware is incapable.

### 6. Optical settle limits are real and currently explicit

The repo currently defines:

- front wall sensor switch settle: `60 us`
- side wall sensor switch settle: `30 us`

Those values live in:

- `MazeMap/MazeMap/HardwareConfig.h`

These settle windows dominate the lit-phase schedule. They do not justify blocking wait loops. They justify a state machine with deadlines.

## Existing UKF Hook Capacity

This part of the codebase is already aligned with the intended redesign.

### 1. The hook seam already exists

`DriveBase::UpdateOdometry(...)` already has templated overloads that accept:

- a `loopHook`
- a `beforeYawUpdate` callback

`LoopController` is already passing callbacks into that interface from the diagnostic path.

### 2. The UKF core already invokes the hook many times

The state dimension is:

- `VehicleState::kDimension = 9`

so the sigma-point count is:

- `2 * 9 + 1 = 19`

The generic UKF implementation calls `loopHook()` repeatedly during predict and update. For the current dimensions, the hook call count is already large:

- predict:
  - 19 calls during sigma propagation,
  - 18 during mean accumulation,
  - 18 during covariance stacking,
  - total `55` hook opportunities
- encoder update:
  - 19 during measurement sigma generation,
  - 18 during predicted measurement accumulation,
  - 18 during innovation stacking,
  - 18 during cross-covariance accumulation,
  - 2 during the final covariance update columns,
  - total `75` hook opportunities
- yaw update:
  - same structure with `M = 1`,
  - total `74` hook opportunities

This is exactly the kind of hook density needed for a nonblocking wall-sensor sweep state machine.

## Timing Target and Feasibility

### Requested sample budget

Target budget from the request:

- about `3-4 us` per sensor input,
- dark phase: `4 sensors x 3-4 us = 12-16 us`,
- lit phase total active ADC time:
  - front pair: `2 sensors x 3-4 us = 6-8 us`
  - left side: `1 x 3-4 us`
  - right side: `1 x 3-4 us`
  - total lit sampling time: `12-16 us`

Total active ADC time target:

- about `24-32 us` per full sweep

This is realistic on the current board only with a different ADC path than the current stock `analogRead()` configuration. Specifically:

- the code stops using stock `analogRead()` for this hot path,
- runtime hardware averaging is disabled,
- the throwaway conversion is removed,
- the ADC is driven through a direct `ADC1` register path with calibrated fixed settings,
- and the front-end is validated to show that one direct conversion after the mux switch is accurate enough.

### Hard limit imposed by current optical settle constants

Even with a perfect ADC path, the current optical timing still imposes:

- front settle: `60 us`
- left settle: `30 us`
- right settle: `30 us`

Minimum lit-settle chain:

- `60 + 30 + 30 = 120 us`

Add the dark-phase ADC work and the lit sampling work:

- dark samples: `12-16 us`
- front pair read: `6-8 us`
- left read: `3-4 us`
- right read: `3-4 us`

That makes the full sweep completion time approximately:

- about `144-152 us` from tick start,
- plus a small amount of GPIO and state-machine overhead.

So if the hard requirement is:

- "all wall readings must be complete before predict ends"

then the predict phase must remain longer than about `150 us` on the target build, or the optical settle constants must be reduced by evidence, or the board must be repinned / redesigned for more parallelism.

That is not a software bug. It is the arithmetic consequence of the current settle constants and current one-ADC pinout.

## Recommended Redesign

### Design goals

1. No explicit busy-wait loops in the control path for wall-sensor settle.
2. One fast dark sampling pass at tick start.
3. Enter UKF predict immediately after dark sampling and LED arm.
4. Use the existing UKF loop hook to opportunistically progress the sweep.
5. One time read per hook invocation, not repeated polls inside one callback.
6. No forced blocking completion after the estimator callback.
7. No explicit ambient-settle wait at the end of the loop.
8. Preserve current external wall-sensor outputs and data products unless intentionally changed.

### Recommended owner

The authoritative owner for this behavior should remain with the runtime sensor pipeline, not with `LoopController`.

Concretely:

- keep wall-sensor sweep state and logic inside `RuntimeSensorSuite` or a private helper owned by `RuntimeSensorSuite`,
- do not introduce a new public timing helper family,
- let `LoopController` only orchestrate the existing callback boundaries.

That matches the repository ownership rules better than moving sensor sequencing into the loop controller itself.

### New control-flow shape

Recommended high-level sequence:

1. `LoopController` applies control at tick start.
2. `RuntimeSensorSuite::Capture(...)` immediately takes dark samples for all four sensors with a fast direct `ADC1` read path.
3. `RuntimeSensorSuite::Capture(...)` turns on the front LED pair immediately and records the front deadline.
4. `LoopController` enters `Drive().UpdateOdometry(...)`.
5. `Drive().UpdateOdometry(...)` passes a nonblocking wall-sweep progress callback into UKF predict and update.
6. Each hook invocation:
   - reads time once,
   - checks whether the current stage deadline has passed,
   - if not, returns immediately,
   - if yes, performs the current stage sample(s), toggles the next LED(s), records the next deadline, and returns.
7. After predict and updates complete, `Capture(...)` builds the final wall snapshot from whatever stages completed.
8. There is no wait loop. If a stage somehow did not complete in time, that should be surfaced as a timing/health signal, not hidden behind a busy wait.

### State-machine stages

Recommended sweep state:

- `Idle`
- `FrontLitPending`
- `LeftLitPending`
- `RightLitPending`
- `Complete`

Recommended stage ownership:

- one active stage at a time,
- one deadline at a time,
- one hook function to advance at most one stage per invocation.

Recommended stage behavior:

- Tick start:
  - LEDs all off
  - read dark FL, FR, SL, SR
  - turn on front LEDs
  - set deadline = `now + front_settle`
  - stage = `FrontLitPending`
- Hook while `FrontLitPending`:
  - if not ready, return
  - read front-left and front-right lit values
  - turn front LEDs off
  - turn left LED on
  - set deadline = `now + side_settle`
  - stage = `LeftLitPending`
- Hook while `LeftLitPending`:
  - if not ready, return
  - read left lit value
  - turn left LED off
  - turn right LED on
  - set deadline = `now + side_settle`
  - stage = `RightLitPending`
- Hook while `RightLitPending`:
  - if not ready, return
  - read right lit value
  - turn right LED off
  - stage = `Complete`

### Time-base recommendation

The request said the hook should check time once. That is correct.

Recommendation:

- use a single monotonic time value per hook call,
- compare against a precomputed stage deadline,
- do not call `micros()` multiple times in the same hook body.

For tighter control and lower overhead, prefer `ARM_DWT_CYCCNT` on Teensy 4.1 rather than `micros()`:

- higher resolution,
- cheaper comparison,
- easy wrap-safe signed subtraction style,
- already used elsewhere in the loop timing code.

Suggested pattern:

```cpp
const uint32_t nowCycles = ARM_DWT_CYCCNT;
if (static_cast<int32_t>(nowCycles - deadlineCycles) < 0) {
    return;
}
```

Convert the configured `30 us` / `60 us` settle windows to cycle counts once when arming the stage.

`micros()` is still acceptable if simplicity is preferred over absolute overhead minimization, but `ARM_DWT_CYCCNT` is the better fit for this hot path.

## ADC Implementation Recommendation

### Recommended runtime ADC path

Do not use `analogRead()` for the control-loop wall-sensor sweep.

Use:

- one-time boot calibration,
- fixed `ADC1` configuration for the wall sensors,
- direct channel writes to `ADC1_HC0`,
- poll `ADC1_HS.COCO0`,
- read `ADC1_R0`,
- no `yield()`,
- no generic pad reconfiguration,
- no hardware averaging in the runtime sweep.

This is the simplest path that respects the current board wiring and the requested timing budget.

### Initial ADC settings to try

The recommended starting point is:

- `ADC1` only
- calibrated at boot
- 12-bit resolution
- high-speed enabled
- asynchronous ADC clock path or another validated fast clock source
- hardware averaging disabled
- start with long sample only if required by measured source impedance behavior
- otherwise prefer the shortest validated sample period that still preserves measurement accuracy
- sample period chosen so the total per-channel time stays inside the `3-4 us` budget

Why not blindly keep the stock long-sample setup:

- it gives the best chance of preserving accuracy with a single conversion after a mux switch,
- but it is not automatically required,
- and it should be kept only if measured front-end settling actually needs it.

The target here is not to copy the stock `analogRead()` configuration minus averaging. The target is to find the shortest validated ADC setup that still preserves wall-sensor fidelity.

Recommended order:

1. start with a direct single-conversion path and no averaging,
2. measure wall-sensor repeatability and inter-channel memory effects,
3. shorten sample time until error becomes unacceptable,
4. back off to the fastest stable setting.

### Why hardware averaging should be disabled in the runtime sweep

For this control loop, hardware averaging is the wrong place to buy noise reduction because it:

- linearly increases conversion time,
- blocks the control path,
- obscures the true single-sample timing,
- duplicates work that downstream filtering and the UKF measurement model can already absorb.

If more noise suppression is needed, prefer:

- per-sensor rolling filters outside the blocking ADC path,
- measurement covariance tuning,
- confidence gating,
- offline calibration improvements.

### Why the current dummy-read pattern should be removed

The current `ReadLightLevel()` does:

- one throwaway conversion,
- then the actual conversion.

That is expensive and only justified if a single conversion after the mux switch is provably inaccurate for this front-end.

Recommended approach:

1. start with one calibrated direct conversion per sample using a conservative long-sample setting,
2. measure per-channel error and inter-channel memory effects,
3. only reintroduce a dummy conversion if real evidence proves it is required.

The requested timing budget strongly argues against carrying the dummy conversion forward by default.

### Accuracy expectation on Teensy 4.1

PJRC's Teensy 4.1 documentation explicitly notes:

- hardware allows up to 12 bits,
- in practice only about 10 bits are normally usable due to noise.

That means the right design target is not "perfect 12-bit ENOB". The right target is:

- stable and repeatable readings,
- low latency,
- known calibration,
- fixed timing,
- and enough real information content for the wall-observation thresholds and UKF updates.

For this application, a single well-timed 12-bit conversion with no averaging is likely better than multiple slow averaged conversions that force the control loop to wait.

## Direct Board-Level Constraint: No True Simultaneous Front Pair Read on Current Pins

Because all four wall sensors are on `ADC1`, the current board cannot produce truly simultaneous front-left and front-right ADC samples.

The best achievable behavior on the current board is:

- sequential front-left then front-right reads,
- with about `3-4 us` gap between them if the direct ADC path is optimized.

That is likely acceptable for this application.

If truly simultaneous front-pair capture becomes a hard requirement, one front sensor must move to an `ADC2` pin on a future hardware revision.

## Recommended Instrumentation Fixes

The current logs obscure the real behavior. That should be corrected before or alongside implementation.

### Keep or expose the actual actuation timestamp

The loop already records:

- `tActuationAppliedUs`

That value should be published in the timing log. It is the real "actuation written" marker.

### Rename or replace the current encoder markers

Current names are misleading. Suggested replacements:

- `pre_sensing_start_us`
- `wall_dark_done_us`
- `encoder_sample_us`
- `estimator_predict_start_us`

If preserving the old fields for compatibility is required, then at least add new correctly named fields and stop interpreting the old pair as encoder timing.

### Add explicit wall-sweep stage timestamps

Suggested new diagnostic fields:

- `wall_dark_start_us`
- `wall_dark_done_us`
- `front_led_on_us`
- `front_ready_us`
- `front_sample_done_us`
- `left_led_on_us`
- `left_ready_us`
- `left_sample_done_us`
- `right_led_on_us`
- `right_ready_us`
- `right_sample_done_us`
- `wall_sweep_complete_us`
- `wall_hook_calls_predict`
- `wall_hook_calls_update`

These fields will make it obvious whether the sweep is actually completing during predict as intended.

## Suggested Implementation Steps

### Phase 1: make the timing observable

1. publish `tActuationAppliedUs`
2. add real wall-sweep stage timestamps
3. add a real encoder sample timestamp around `ConsumeEncoderCycleSample(...)`
4. stop treating the current `encoderLatchUs` / `encoderReadDoneUs` pair as encoder timing

### Phase 2: remove `analogRead()` from the hot wall-sensor path

1. add a private direct `ADC1` runtime reader for the wall-sensor channels
2. calibrate/configure ADC1 once at setup
3. disable runtime hardware averaging for this path
4. remove the dummy conversion from hot-path reads

### Phase 3: convert the wall sweep into a true no-wait state machine

1. take dark samples at tick start
2. arm the front phase immediately
3. advance lit phases from the UKF loop hook
4. remove `CompleteAsyncWallSensorSweepRead(...)`
5. remove explicit ambient-settle waiting at loop end

### Phase 4: validate and tune

1. measure actual per-sample ADC duration on target hardware
2. confirm the dark phase stays inside `12-16 us`
3. confirm the lit phase reads stay inside `12-16 us` total active ADC time
4. confirm the full sweep reliably completes before predict end if that remains a hard requirement
5. tune sample-period settings only after correctness is proven

## Validation Plan

The redesign should be validated on hardware, not inferred only from host-side reasoning.

### Minimum validation set

1. Scope or timing-log measurement of one direct ADC sample on each wall sensor.
2. Tick-level measurement of:
   - actuation write,
   - dark phase complete,
   - front phase complete,
   - left phase complete,
   - right phase complete,
   - predict start,
   - predict end.
3. Confirmation that no explicit `delayMicroseconds(...)` remains in the control-path sweep.
4. Comparison of wall distance repeatability before and after removing averaging and dummy reads.
5. Verification that side/front wall classification thresholds remain stable enough for control and UKF updates.

### Recommended acceptance checks

- dark phase active ADC time <= `16 us`
- total lit active ADC time <= `16 us`
- no blocking completion loop in the capture path
- no explicit ambient-settle wait in the control path
- all sweep stages complete before predict end on representative runs, if that is required
- no measurable regression in wall-detection correctness

## Open Questions / Risks

### 1. Will a single conversion after a mux switch be accurate enough?

Probably yes with a calibrated direct ADC path and conservative sample time, but this must be measured on the actual wall-sensor front-end.

### 2. Is predict always long enough to finish the full optical schedule?

Not proven here. The code has enough hook density, but the total required settle chain is still about `120 us` plus ADC time. This must be checked against real `ukfPredictDurationUs` on the target build.

### 3. Are the current `60 us` / `30 us` settle constants still justified?

They may be, but they should be periodically revalidated. If scope data says they are conservative, reducing them is the most direct path to finishing earlier without changing estimator structure.

### 4. Is ADC_ETC worth using here?

It is available on the platform and can be combined with DMA, but it is probably not the first implementation to ship:

- current pins are all on `ADC1`, so ADC_ETC cannot create true parallelism on the current board,
- the simpler direct polled `ADC1` state machine should already meet the requested budget,
- ADC_ETC becomes attractive if jitter reduction or lower CPU overhead is later needed.

## Recommended Final Shape

The intended end state should look like this:

- `LoopController` still owns loop sequencing and mode callbacks.
- `RuntimeSensorSuite` owns wall-sensor capture state.
- `DriveBase` still owns the single UKF execution path.
- the wall sweep uses the existing UKF hook rather than introducing a second scheduler.
- the wall-sensor control path has zero explicit settle waits.
- the timing logs show the true actuation time, the true encoder sample time, and the real wall-sweep stage boundaries.

That shape is consistent with the architecture rules in this repo and with the practical timing constraints of the current Teensy 4.1 hardware.

## Source Links

Official sources used for the hardware/platform recommendations:

- PJRC Teensy 4.1 product page: <https://www.pjrc.com/store/teensy41.html>
- PJRC Teensy 4.x core `analog.c`: <https://raw.githubusercontent.com/PaulStoffregen/cores/master/teensy4/analog.c>
- NXP i.MX RT1060 product page: <https://www.nxp.com/products/i.MX-RT1060>
- NXP MCUXpresso MIMXRT1062 ADC driver docs: <https://mcuxpresso.nxp.com/mcuxsdk/latest/html/drivers/RT/RT1060/MIMXRT1062/index.html>
- NXP MCUXpresso ADC_ETC example index: <https://mcuxpresso.nxp.com/mcuxsdk/latest/html/examples/driver_examples/adc_etc/index.html>
- NXP MCUXpresso ADC_ETC + eDMA example: <https://mcuxpresso.nxp.com/mcuxsdk/latest/html/examples/driver_examples/adc_etc/adc_etc_edma/readme.html>

Local code references used for the current-behavior observations:

- `MazeMap/MazeMap/LoopController.cpp`
- `MazeMap/MazeMap/LoopController.h`
- `MazeMap/MazeMap/DriveBase.h`
- `MazeMap/MazeMap/RuntimeSensorSuite.h`
- `MazeMap/MazeMap/RuntimeSensorSuite.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MazeMap/MazeMap/WallSensor.h`
- `MazeMap/MazeMap/MotorEncoderDrive.h`
- `MazeMap/MazeMap/HardwareConfig.h`
- `MazeMap/MazeMap/Pins.h`
- `MazeMap/MazeMap/VehicleState.h`
