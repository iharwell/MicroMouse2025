#include "pch.h"
#include "LoopController.h"

#include "Defines.h"
#include "DriveBase.h"
#include "HardwareConfig.h"
#include "SharedRobotRuntime.h"
#include "RuntimeSensorSuite.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr float kZeroMotorPwmThreshold = 1.0e-4f;
        constexpr std::uint16_t kTickTimingSaturatedUs = 0xFFFFU;
        constexpr float kDefaultPauseLinearThresholdMps = 0.01f;
        constexpr float kDefaultPauseAngularThresholdRadps = 0.05f;
        constexpr std::uint8_t kDefaultPauseSettledTicks = 2U;

        inline bool IsFinitePositive(const float value) noexcept
        {
            return std::isfinite(value) && (value > 0.0f);
        }

        bool SetMotorPwmThunk(void* const context, const float leftMotorPwm, const float rightMotorPwm) noexcept
        {
            if (context == nullptr)
            {
                return false;
            }

            return static_cast<SharedRobotRuntime*>(context)->SetMotorPWM(leftMotorPwm, rightMotorPwm);
        }

        // LoopController intentionally owns the one normal cadence wait. If callers ever regain a
        // public per-tick boundary, schedule ownership fragments immediately and this class stops
        // being the strict timing authority it exists to be.
        inline void WaitUntilUs(const std::uint32_t absoluteDeadlineUs) noexcept
        {
            while (static_cast<std::int32_t>(absoluteDeadlineUs - static_cast<std::uint32_t>(micros())) > 0)
            {
                const std::uint32_t remainingUs =
                    static_cast<std::uint32_t>(absoluteDeadlineUs - static_cast<std::uint32_t>(micros()));
                if (remainingUs > 10U)
                {
                    delayMicroseconds(5U);
                }
            }
        }
    }

    const LoopController::ControlVector LoopController::ControlVector::Brake =
        LoopController::ControlVector::RawMotorPwm(
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN());

    LoopController::ControlVector LoopController::ControlVector::RawMotorPwm(
        const float leftMotorPwm,
        const float rightMotorPwm) noexcept
    {
        ControlVector control{};
        control.leftMotorPwm = leftMotorPwm;
        control.rightMotorPwm = rightMotorPwm;
        return control;
    }

    LoopController::PauseDisposition LoopController::PauseDisposition::Resume() noexcept
    {
        return PauseDisposition{};
    }

    LoopController::PauseDisposition LoopController::PauseDisposition::Complete() noexcept
    {
        PauseDisposition result{};
        result.action = Action::Complete;
        return result;
    }

    LoopController::PauseDisposition LoopController::PauseDisposition::StopByRuntime(const char* reason) noexcept
    {
        PauseDisposition result{};
        result.action = Action::StopByRuntime;
        result.stopReason = reason;
        return result;
    }

    LoopController::TickServices::TickServices(LoopController& owner) noexcept
        : _owner(&owner)
    {
    }

    void LoopController::TickServices::Fault(const char* reason) noexcept
    {
        if ((_owner != nullptr) && (_owner->_requests.runtimeStopReason == nullptr))
        {
            _owner->_requests.runtimeStopReason = reason;
        }
    }

    void LoopController::TickServices::RequestPause(const PauseRequest& request) noexcept
    {
        if (_owner == nullptr)
        {
            return;
        }

        if (request.onPauseGranted == nullptr)
        {
            if (_owner->_requests.runtimeStopReason == nullptr)
            {
                _owner->_requests.runtimeStopReason = "LoopController pause request missing callback";
            }
            return;
        }

        _owner->_requests.pauseRequested = true;
        _owner->_requests.pauseRequest = request;
    }

    void LoopController::TickServices::RequestEndLoop() noexcept
    {
        if (_owner != nullptr)
        {
            _owner->_requests.endRequested = true;
        }
    }

    void LoopController::TickServices::SetNextModeWorkCallbacks(const ModeCallbacks& callbacks) noexcept
    {
        if ((_owner != nullptr) && (callbacks.onModeWork != nullptr))
        {
            _owner->_requests.nextModeWorkRequested = true;
            _owner->_requests.nextModeWork = callbacks;
        }
    }

    void LoopController::TickServices::SetNextModeWorkCallback(const ModeWorkCallback callback) noexcept
    {
        ModeCallbacks callbacks{};
        callbacks.onModeWork = callback;
        callbacks.context = (_owner != nullptr) ? _owner->_activeModeWorkContext : nullptr;
        SetNextModeWorkCallbacks(callbacks);
    }

    bool LoopController::BeginSession(const SessionOptions& options, const ModeCallbacks& callbacks)
    {
        // Session startup arms one continuous cadence owner. There is intentionally no "prime one
        // tick" or "manually advance" companion API because yielding the schedule back to callers
        // between ticks would fundamentally undercut LoopController's reason for existing.
        if ((_runtime == nullptr) ||
            !_motorPwmSink ||
            _sessionActive ||
            _sessionBegun ||
            (callbacks.onModeWork == nullptr) ||
            !ValidateSessionOptions(options))
        {
            return false;
        }

        _sessionStartWallSensorAdcProbePending = true;
        RunSessionStartWallSensorAdcProbe();

        _options = options;
        _callbacks = callbacks;
        _activeModeWorkCallback = callbacks.onModeWork;
        _activeModeWorkContext = callbacks.context;
        _sessionBegun = true;
        _sessionActive = true;
        _resumePending = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
        _lastTickStartUs = nowUs - _options.controlPeriodUs;
        _nextSyncTargetUs = nowUs + _options.controlPeriodUs;
        _queuedControl = ControlVector::Brake;
        _appliedControl = ControlVector::Brake;
        _publishedTimingIndex = 0U;
        _workingTimingIndex = 1U;
        _timingBuffers[0] = TimingDiagnostics{};
        _timingBuffers[1] = TimingDiagnostics{};
        _pauseContextScratch = PauseContext{};
        _deferredTerminalOutcome = DeferredTerminalOutcome::None;
        _deferredTerminalReason = nullptr;
        ResetLatchedRequests();
        return true;
    }

    LoopController::SessionResult LoopController::Run()
    {
        // Run() owns the full active cadence. If this class ever exposes a public single-tick
        // function, callers would be forced to reassemble cadence discipline around it and the
        // result would only be a worse version of what this loop already does correctly.
        SessionResult result{};
        result.tickCount = _tickCount;

        if (!_sessionActive || (_runtime == nullptr) || !_motorPwmSink || (_activeModeWorkCallback == nullptr))
        {
            result.status = SessionResult::Status::StoppedByRuntime;
            return result;
        }

        while (_sessionActive)
        {
            // Keep the entire tick private to this loop. Command application, sensing, mode work,
            // post-work service, and the sync wait all stay under one owner specifically so no
            // caller can wedge its own "just one tick" orchestration into the cadence path.
            if (_deferredTerminalOutcome != DeferredTerminalOutcome::None)
            {
                const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
                const std::uint32_t dtUs = nowUs - _lastTickStartUs;
                _lastTickStartUs = nowUs;
                (void)dtUs;
                _appliedControl = ControlVector::Brake;
                _queuedControl = _appliedControl;
                (void)ApplyControlAtTickStart(_appliedControl);

                result.tickCount = _tickCount;
                if (_deferredTerminalOutcome == DeferredTerminalOutcome::Complete)
                {
                    result.status = SessionResult::Status::Completed;
                }
                else
                {
                    if (_runtime != nullptr)
                    {
                        (void)_runtime->FailActiveMode(_deferredTerminalReason);
                    }
                    result.status = SessionResult::Status::StoppedByRuntime;
                }

                ResetSessionState();
                return result;
            }

            const std::uint32_t tickStartUs = static_cast<std::uint32_t>(micros());
            const std::uint32_t dtUs = tickStartUs - _lastTickStartUs;
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _lastTickStartUs = tickStartUs;
            ++_tickCount;

            ResetWorkingTiming(_tickCount, tickStartUs, dtUs);
            TimingDiagnostics& timing = WorkingTiming();
            timing.controlTiming.controlStartUs = tickStartUs;
            timing.controlTiming.cycleCounterStart = ReadCycleCounter();
            if (_resumePending)
            {
                timing.flags |= kTimingFlagResumedFromPause;
                _resumePending = false;
            }

            const std::uint32_t loopEndTimeUs = _nextSyncTargetUs;

            _appliedControl = _queuedControl;
            if (!ApplyControlAtTickStart(_appliedControl))
            {
                _queuedControl = ControlVector::Brake;
                timing.flags |= kTimingFlagRuntimeStopPending;
                _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                _deferredTerminalReason = "LoopController motor PWM hook failed";
                ServiceRuntimeLogsForFaultPath();
                RecordPostServiceTiming();
                FinalizeTiming();
                PublishWorkingTiming();
                WaitUntilUs(loopEndTimeUs);
                _nextSyncTargetUs += _options.controlPeriodUs;
                continue;
            }
            timing.tActuationAppliedUs =
                RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));

            if (_sessionStartWallSensorAdcProbePending)
            {
                RunSessionStartWallSensorAdcProbe();
            }

            ResetLatchedRequests();

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                _queuedControl = ControlVector::Brake;
                timing.flags |= kTimingFlagRuntimeStopPending;
                _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                _deferredTerminalReason =
                    (_deferredTerminalReason != nullptr) ?
                    _deferredTerminalReason :
                    "LoopController sensing update failed";

                ServiceRuntimeLogsForFaultPath();
                RecordPostServiceTiming();
                FinalizeTiming();
                PublishWorkingTiming();
                WaitUntilUs(loopEndTimeUs);
                _nextSyncTargetUs += _options.controlPeriodUs;
                continue;
            }

            TickServices services(*this);
            ControlVector candidateControl =
                _activeModeWorkCallback(_activeModeWorkContext, loopEndTimeUs, _runtime->RuntimeState(), services);
            RecordModeReturnTiming();

            if (_requests.nextModeWorkRequested && (_requests.nextModeWork.onModeWork != nullptr))
            {
                _activeModeWorkCallback = _requests.nextModeWork.onModeWork;
                _activeModeWorkContext = _requests.nextModeWork.context;
            }

            bool faultPathFlushRequired = false;
            if (_requests.runtimeStopReason != nullptr)
            {
                _queuedControl = ControlVector::Brake;
                timing.flags |= kTimingFlagRuntimeStopPending;
                _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                _deferredTerminalReason = _requests.runtimeStopReason;
                faultPathFlushRequired = true;
            }
            else if (_requests.endRequested)
            {
                _queuedControl = ControlVector::Brake;
                _deferredTerminalOutcome = DeferredTerminalOutcome::Complete;
            }
            else if (_requests.pauseRequested)
            {
                _queuedControl = ControlVector::Brake;
                timing.flags |= kTimingFlagPausePending;
            }
            else
            {
                _queuedControl = candidateControl;
            }

            if (faultPathFlushRequired)
            {
                ServiceRuntimeLogsForFaultPath();
            }
            else if (ComputeRemainingSlackUs(loopEndTimeUs) > 0U)
            {
                if (!ServiceRuntimeLogsNormal())
                {
                    const char* const runtimeReason =
                        (_runtime != nullptr) ? _runtime->LastRuntimeLogError() : nullptr;
                    _queuedControl = ControlVector::Brake;
                    timing.flags |= kTimingFlagRuntimeStopPending;
                    _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                    _deferredTerminalReason =
                        ((runtimeReason != nullptr) && (runtimeReason[0] != '\0')) ?
                        runtimeReason :
                        "LoopController runtime log service failed";
                    ServiceRuntimeLogsForFaultPath();
                }
            }

            RecordPostServiceTiming();
            FinalizeTiming();
            PublishWorkingTiming();

            // There is exactly one normal wait block per tick and it stays here at end-of-tick.
            // Moving this behind a caller-visible per-tick return path would split cadence
            // ownership and turn LoopController into a bad cooperative timer instead of the
            // authoritative loop scheduler.
            WaitUntilUs(loopEndTimeUs);
            _nextSyncTargetUs += _options.controlPeriodUs;

            // Pause is the explicit escape hatch for work that cannot remain inside strict cadence.
            // Replacing this with caller-driven "tick once and come back" flow would break the
            // class's sole responsibility while also doing a worse job of maintaining sync.
            if (_requests.pauseRequested && (_deferredTerminalOutcome == DeferredTerminalOutcome::None))
            {
                result.tickCount = _tickCount;
                if (!ResolvePauseRequest(result))
                {
                    ResetSessionState();
                    return result;
                }
            }
        }

        result.status = SessionResult::Status::Completed;
        result.tickCount = _tickCount;
        return result;
    }

    void LoopController::EndSession()
    {
        if (!_sessionBegun)
        {
            return;
        }

        if (_runtime != nullptr)
        {
            (void)_motorPwmSink.Apply(ControlVector::Brake);
        }

        ResetSessionState();
    }

    bool LoopController::SessionActive() const noexcept
    {
        return _sessionActive;
    }

    const LoopController::TimingDiagnostics& LoopController::LastDiagnostics() const noexcept
    {
        return PublishedTiming();
    }

    const LoopController::ControlVector& LoopController::LastAppliedCommand() const noexcept
    {
        return _appliedControl;
    }

    std::uint32_t LoopController::CurrentTickSequence() const noexcept
    {
        const TimingDiagnostics* const timing = CurrentTimingForReaders();
        return (timing != nullptr) ? timing->sequence : 0U;
    }

    std::uint32_t LoopController::CurrentTickStartUs() const noexcept
    {
        const TimingDiagnostics* const timing = CurrentTimingForReaders();
        return (timing != nullptr) ? timing->tickStartUs : 0U;
    }

    std::uint32_t LoopController::CurrentTickDtUs() const noexcept
    {
        const TimingDiagnostics* const timing = CurrentTimingForReaders();
        return (timing != nullptr) ? timing->dtUs : 0U;
    }

    float LoopController::CurrentTickDtSeconds() const noexcept
    {
        return static_cast<float>(CurrentTickDtUs()) * 1.0e-6f;
    }

    std::uint16_t LoopController::RelativeTickUs(
        const std::uint32_t tickStartUs,
        const std::uint32_t timestampUs) noexcept
    {
        if (timestampUs <= tickStartUs)
        {
            return 0U;
        }

        const std::uint32_t elapsedUs = timestampUs - tickStartUs;
        return static_cast<std::uint16_t>((std::min)(elapsedUs, static_cast<std::uint32_t>(kTickTimingSaturatedUs)));
    }

    bool LoopController::IsBrakeMotorPwmCommand(const ControlVector& command) noexcept
    {
        return !std::isfinite(command.leftMotorPwm) || !std::isfinite(command.rightMotorPwm);
    }

    bool LoopController::IsZeroMotorPwmCommand(const ControlVector& command) noexcept
    {
        return
            std::isfinite(command.leftMotorPwm) &&
            std::isfinite(command.rightMotorPwm) &&
            (std::fabs(command.leftMotorPwm) <= kZeroMotorPwmThreshold) &&
            (std::fabs(command.rightMotorPwm) <= kZeroMotorPwmThreshold);
    }

    bool LoopController::IsFullSensorWorkPlan(const SensorWorkPlan& workPlan) noexcept
    {
        return (workPlan.wallMask == WallMask::All) &&
            workPlan.readEncoders &&
            workPlan.readImuBundle &&
            workPlan.useEncoderUpdate &&
            workPlan.useGyroUpdate &&
            workPlan.useAccelUpdate &&
            workPlan.useWallUpdates;
    }

    std::uint32_t LoopController::ReadCycleCounter() noexcept
    {
#if defined(ARDUINO_TEENSY41)
        return ARM_DWT_CYCCNT;
#else
        return 0UL;
#endif
    }

    void LoopController::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _motorPwmSink.context = &runtime;
        _motorPwmSink.setMotorPwm = &SetMotorPwmThunk;
    }

    void LoopController::RunSessionStartWallSensorAdcProbe() noexcept
    {
        if (_runtime == nullptr)
        {
            return;
        }

        if (!_runtime->TextLogIsOpen())
        {
            _sessionStartWallSensorAdcProbePending = true;
            return;
        }

        const uint32_t targetCfg = MazeMap::Platform::GetWallSensorAdcRuntimeMode();
        MazeMap::App::Internal::LogWallSensorAdcRegisterWrite(
            "session_pre_write",
            targetCfg,
            MazeMap::Platform::GetWallSensorAdcCurrentCfg(),
            MazeMap::Platform::GetWallSensorAdcCurrentGc());
        MazeMap::Platform::PrepareWallSensorAdcForRead();
        MazeMap::App::Internal::LogWallSensorAdcRegisterWrite(
            "session_post_write",
            targetCfg,
            MazeMap::Platform::GetWallSensorAdcCurrentCfg(),
            MazeMap::Platform::GetWallSensorAdcCurrentGc());

        uint8_t probeChannel = 0U;
        const uint8_t probePin = _runtime->SpeedVehicle().FrontLeft.GetWallSensorInPin();
        if (MazeMap::Platform::ResolveWallSensorAdc1Channel(probePin, probeChannel))
        {
            (void)MazeMap::Platform::ReadSingleWallSensorAdcCodeFromConfiguredChannel(probeChannel);
            MazeMap::App::Internal::LogWallSensorAdcRegisterWrite(
                "session_post_sample",
                targetCfg,
                MazeMap::Platform::GetWallSensorAdcCurrentCfg(),
                MazeMap::Platform::GetWallSensorAdcCurrentGc());
        }
        else
        {
            MazeMap::App::Internal::LogWallSensorAdcRegisterWrite(
                "session_probe_unresolved",
                targetCfg,
                MazeMap::Platform::GetWallSensorAdcCurrentCfg(),
                MazeMap::Platform::GetWallSensorAdcCurrentGc());
        }

        _sessionStartWallSensorAdcProbePending = false;
    }

    bool LoopController::ValidateSessionOptions(const SessionOptions& options) const noexcept
    {
        return (options.controlPeriodUs > 0U) && SupportsSensorWorkPlan(options.workPlan);
    }

    bool LoopController::SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept
    {
        return (workPlan.wallMask == WallMask::All) &&
            workPlan.readEncoders &&
            workPlan.readImuBundle &&
            workPlan.useEncoderUpdate &&
            workPlan.useGyroUpdate &&
            workPlan.useAccelUpdate;
    }

    const LoopController::TimingDiagnostics* LoopController::CurrentTimingForReaders() const noexcept
    {
        if (_sessionActive && (_tickCount > 0U))
        {
            return &_timingBuffers[_workingTimingIndex];
        }

        return _publishedTimingValid ? &_timingBuffers[_publishedTimingIndex] : nullptr;
    }

    void LoopController::ResetLatchedRequests() noexcept
    {
        _requests = LatchedRequests{};
    }

    bool LoopController::ApplyControlAtTickStart(const ControlVector& control) noexcept
    {
        if (!_motorPwmSink)
        {
            return false;
        }

        return _motorPwmSink.Apply(control);
    }

    bool LoopController::CaptureTickState(const float dtSeconds, const std::uint32_t tickStartUs)
    {
        if (_runtime == nullptr)
        {
            _deferredTerminalReason = "LoopController runtime unavailable";
            return false;
        }

        TimingDiagnostics& timing = WorkingTiming();
        const bool stationaryHint = ShouldTreatAppliedControlAsStationary();
        MazeMap::Maze* const map = _options.workPlan.useWallUpdates ? &_runtime->Maze() : nullptr;
        timing.controlTiming.encoderLatchUs = static_cast<std::uint32_t>(micros());
        class CaptureContext final
        {
        public:
            CaptureContext(
                SharedRobotRuntime* runtime,
                TimingDiagnostics* timing,
                MazeMap::Maze* map,
                float dtSeconds) noexcept
                : _runtime(runtime)
                , _timing(timing)
                , _map(map)
                , _dtSeconds(dtSeconds)
            {
            }

            SharedRobotRuntime& Runtime() const noexcept
            {
                return *_runtime;
            }

            TimingDiagnostics& Timing() const noexcept
            {
                return *_timing;
            }

            MazeMap::Maze* Map() const noexcept
            {
                return _map;
            }

            float DtSeconds() const noexcept
            {
                return _dtSeconds;
            }

        private:
            SharedRobotRuntime* _runtime{};
            TimingDiagnostics* _timing{};
            MazeMap::Maze* _map{};
            float _dtSeconds{};
        } captureContext{ _runtime, &timing, map, dtSeconds };

        const RuntimeSensorSuite::CaptureHandler callback =
            [](void* rawContext, SensorSnapshot& captureSnapshot, RuntimeSensorSuite::CaptureServices& services) noexcept
            {
                auto& context = *static_cast<CaptureContext*>(rawContext);
                context.Timing().controlTiming.encoderReadDoneUs = static_cast<std::uint32_t>(micros());
                DriveBase& drive = context.Runtime().Drive();
                drive.RecordMeasurementInputs(captureSnapshot);
                const MazeMap::ControlInput control = drive.CurrentControlInput();
                const MazeMap::VehicleState::DriveCommandState driveCommandState =
                    drive.BuildDriveCommandState(control);
                const MazeMap::EncoderObs encoderObservation = drive.ConsumeEncoderObservation(context.DtSeconds());
                (void)context.Runtime().Estimator().UpdateRuntimeState(
                    context.DtSeconds(),
                    control,
                    drive.GetLastLinearCommandMps(),
                    drive.GetLastAngularCommandRadps(),
                    drive.GetLastSaturationFlags(),
                    drive.GetLastLeftLaunchAssistFloor(),
                    drive.GetLastRightLaunchAssistFloor(),
                    encoderObservation,
                    captureSnapshot,
                    context.Map(),
                    &context.Timing().controlTiming,
                    [&context, &services]() noexcept
                    {
                        (void)context.Runtime().ServiceUtilityDataLog();
                        (void)services.ServiceWallRead();
                    },
                    [&context, &services]() noexcept
                     {
                         (void)context.Runtime().ServiceUtilityDataLog();
                         services.CaptureImu();
                     });
                context.Runtime().RuntimeState().SetDriveCommandState(driveCommandState);
            };

        SensorSnapshot snapshot{};
        _runtime->Sensors().Capture(
            stationaryHint,
            _runtime->RuntimeState(),
            snapshot,
            callback,
            &captureContext);

        _runtime->RuntimeState().SetTimestampUs(tickStartUs);
        timing.frontTiming = snapshot.frontTiming;
        timing.leftTiming = snapshot.leftTiming;
        timing.rightTiming = snapshot.rightTiming;
        timing.imuTiming = snapshot.imuTiming;
        return true;
    }

    void LoopController::ResetWorkingTiming(
        const std::uint32_t sequence,
        const std::uint32_t tickStartUs,
        const std::uint32_t dtUs) noexcept
    {
        TimingDiagnostics& timing = WorkingTiming();
        timing = TimingDiagnostics{};
        timing.sequence = sequence;
        timing.tickStartUs = tickStartUs;
        timing.dtUs = dtUs;
    }

    LoopController::TimingDiagnostics& LoopController::WorkingTiming() noexcept
    {
        return _timingBuffers[_workingTimingIndex];
    }

    const LoopController::TimingDiagnostics& LoopController::PublishedTiming() const noexcept
    {
        return _timingBuffers[_publishedTimingIndex];
    }

    void LoopController::PublishWorkingTiming() noexcept
    {
        _publishedTimingIndex = _workingTimingIndex;
        _workingTimingIndex = static_cast<std::uint8_t>(1U - _workingTimingIndex);
        _publishedTimingValid = true;
    }

    void LoopController::RecordModeReturnTiming() noexcept
    {
        const std::uint32_t tickStartUs = WorkingTiming().tickStartUs;
        WorkingTiming().tModeReturnUs =
            RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));
    }

    void LoopController::RecordPostServiceTiming() noexcept
    {
        const std::uint32_t tickStartUs = WorkingTiming().tickStartUs;
        WorkingTiming().tPostServiceDoneUs =
            RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));
    }

    void LoopController::FinalizeTiming() noexcept
    {
        TimingDiagnostics& timing = WorkingTiming();
        const std::uint32_t finalizeUs = static_cast<std::uint32_t>(micros());
        timing.controlTiming.pwmLatchUs = finalizeUs;
        timing.controlTiming.controlEndUs = finalizeUs;
        timing.controlTiming.cycleCounterEnd = ReadCycleCounter();
        if (finalizeUs <= _nextSyncTargetUs)
        {
            timing.overrunUs = 0U;
        }
        else
        {
            timing.overrunUs = static_cast<std::uint16_t>((std::min)(
                finalizeUs - _nextSyncTargetUs,
                static_cast<std::uint32_t>(kTickTimingSaturatedUs)));
        }
    }

    bool LoopController::ServiceRuntimeLogsNormal() noexcept
    {
        return (_runtime != nullptr) ? _runtime->ServiceUtilityDataLog() : false;
    }

    void LoopController::ServiceRuntimeLogsForFaultPath() noexcept
    {
        if (_runtime == nullptr)
        {
            return;
        }

        if (!_runtime->ServiceUtilityDataLog() && (_deferredTerminalReason == nullptr))
        {
            const char* const runtimeReason = _runtime->LastRuntimeLogError();
            if ((runtimeReason != nullptr) && (runtimeReason[0] != '\0'))
            {
                _deferredTerminalReason = runtimeReason;
            }
        }
    }

    std::uint32_t LoopController::ComputeRemainingSlackUs(const std::uint32_t absoluteDeadlineUs) const noexcept
    {
        const std::int32_t remainingUs =
            static_cast<std::int32_t>(absoluteDeadlineUs - static_cast<std::uint32_t>(micros()));
        return (remainingUs > 0) ? static_cast<std::uint32_t>(remainingUs) : 0U;
    }

    bool LoopController::ShouldTreatAppliedControlAsStationary() const noexcept
    {
        return IsBrakeMotorPwmCommand(_appliedControl) || IsZeroMotorPwmCommand(_appliedControl);
    }

    bool LoopController::ResolvePauseRequest(SessionResult& result)
    {
        if (!WaitForPauseSettlement(_requests.pauseRequest))
        {
            if (_runtime != nullptr)
            {
                (void)_runtime->FailActiveMode(_deferredTerminalReason);
            }
            result.status = SessionResult::Status::StoppedByRuntime;
            result.tickCount = _tickCount;
            return false;
        }

        _pauseContextScratch.reason = _requests.pauseRequest.reason;
        const PauseDisposition disposition =
            _requests.pauseRequest.onPauseGranted(_activeModeWorkContext, _pauseContextScratch);
        switch (disposition.action)
        {
        case PauseDisposition::Action::Complete:
            result.status = SessionResult::Status::Completed;
            result.tickCount = _tickCount;
            return false;

        case PauseDisposition::Action::StopByRuntime:
            if (_runtime != nullptr)
            {
                (void)_runtime->FailActiveMode(disposition.stopReason);
            }
            result.status = SessionResult::Status::StoppedByRuntime;
            result.tickCount = _tickCount;
            return false;

        case PauseDisposition::Action::Resume:
        default:
        {
            if (disposition.resetClockOnResume || _requests.pauseRequest.resetClockOnResume)
            {
                const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
                _lastTickStartUs = nowUs - _options.controlPeriodUs;
                _nextSyncTargetUs = nowUs + _options.controlPeriodUs;
            }
            _queuedControl = ControlVector::Brake;
            _appliedControl = ControlVector::Brake;
            _resumePending = true;
            result.tickCount = _tickCount;
            return true;
        }
        }
    }

    bool LoopController::WaitForPauseSettlement(const PauseRequest& request)
    {
        if (_runtime == nullptr)
        {
            _deferredTerminalReason = "LoopController pause settlement missing runtime";
            return false;
        }

        const float linearThreshold =
            IsFinitePositive(request.maxAbsLinearSpeed) ?
            request.maxAbsLinearSpeed :
            kDefaultPauseLinearThresholdMps;
        const float angularThreshold =
            IsFinitePositive(request.maxAbsAngularSpeed) ?
            request.maxAbsAngularSpeed :
            kDefaultPauseAngularThresholdRadps;
        const std::uint8_t settledTicks =
            (request.consecutiveSettledTicks > 0U) ?
            request.consecutiveSettledTicks :
            kDefaultPauseSettledTicks;

        std::uint8_t settledCount = 0U;
        while (settledCount < settledTicks)
        {
            const std::uint32_t tickStartUs = static_cast<std::uint32_t>(micros());
            const std::uint32_t dtUs = tickStartUs - _lastTickStartUs;
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            const std::uint32_t deadlineUs = tickStartUs + _options.controlPeriodUs;
            _lastTickStartUs = tickStartUs;
            _nextSyncTargetUs = deadlineUs;
            ++_tickCount;

            ResetWorkingTiming(_tickCount, tickStartUs, dtUs);
            TimingDiagnostics& timing = WorkingTiming();
            timing.flags = kTimingFlagPausePending;
            timing.controlTiming.controlStartUs = tickStartUs;
            timing.controlTiming.cycleCounterStart = ReadCycleCounter();

            _appliedControl = ControlVector::Brake;
            _queuedControl = _appliedControl;
            (void)dtSeconds;
            if (!ApplyControlAtTickStart(_appliedControl))
            {
                _deferredTerminalReason = "LoopController motor PWM hook failed during pause settlement";
                ServiceRuntimeLogsForFaultPath();
                return false;
            }
            timing.tActuationAppliedUs =
                RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                _deferredTerminalReason =
                    (_deferredTerminalReason != nullptr) ?
                    _deferredTerminalReason :
                    "LoopController pause settlement capture failed";
                ServiceRuntimeLogsForFaultPath();
                return false;
            }

            if (request.flushLogsBeforeGrant)
            {
                ServiceRuntimeLogsForFaultPath();
            }

            const MazeMap::VehicleState& runtimeState = _runtime->RuntimeState();
            const float linearSpeedMps = runtimeState.GetVelocity();
            const float angularSpeedRadps = runtimeState.GetRotationalVelocity();
            const bool settled =
                std::isfinite(linearSpeedMps) &&
                std::isfinite(angularSpeedRadps) &&
                (std::fabs(linearSpeedMps) <= linearThreshold) &&
                (std::fabs(angularSpeedRadps) <= angularThreshold) &&
                !_runtime->Estimator().HasFault();
            settledCount = settled ? static_cast<std::uint8_t>(settledCount + 1U) : 0U;

            RecordPostServiceTiming();
            FinalizeTiming();
            PublishWorkingTiming();
            WaitUntilUs(deadlineUs);
        }

        return true;
    }

    void LoopController::ResetSessionState() noexcept
    {
        _options = SessionOptions{};
        _callbacks = ModeCallbacks{};
        _activeModeWorkCallback = nullptr;
        _activeModeWorkContext = nullptr;
        _sessionBegun = false;
        _sessionActive = false;
        _resumePending = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        _lastTickStartUs = 0U;
        _nextSyncTargetUs = 0U;
        _queuedControl = ControlVector::Brake;
        _appliedControl = ControlVector::Brake;
        _sessionStartWallSensorAdcProbePending = false;
        _requests = LatchedRequests{};
        _deferredTerminalOutcome = DeferredTerminalOutcome::None;
        _deferredTerminalReason = nullptr;
        _pauseContextScratch = PauseContext{};
    }
}
