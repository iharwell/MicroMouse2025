#include "pch.h"
#include "LoopController.h"

#include "Defines.h"
#include "DriveBase.h"
#include "HardwareConfig.h"
#include "IApplicationMode.h"
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
        constexpr float kPauseLinearThresholdMps = 0.01f;
        constexpr float kPauseAngularThresholdRadps = 0.05f;
        constexpr std::uint8_t kPauseSettledTicks = 2U;

        bool SetMotorPwmThunk(void* const context, const float leftMotorPwm, const float rightMotorPwm) noexcept
        {
            if (context == nullptr)
            {
                return false;
            }

            return static_cast<SharedRobotRuntime*>(context)->SetMotorPWM(leftMotorPwm, rightMotorPwm);
        }

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

    void LoopController::StageNextSessionState(const SessionOptions& options) noexcept
    {
        if (!ValidateSessionOptions(options))
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode("LoopController staged session state is invalid");
            }
            while (true)
            {
            }
        }

        _stagedNextSessionOptions = options;
        _stagedNextSessionValid = true;
    }

    void LoopController::RequestPause(const PauseCallback callback, void* const context) noexcept
    {
        if (!_sessionActive)
        {
            return;
        }

        if (callback == nullptr)
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode("LoopController pause request missing callback");
            }
            while (true)
            {
            }
            return;
        }

        _pendingPauseCallback = callback;
        _pendingPauseContext = context;
    }

    void LoopController::RequestEndSession(
        const EndSessionCallback callback,
        void* const context) noexcept
    {
        if (!_sessionActive)
        {
            return;
        }

        if (callback == nullptr)
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode("LoopController end-session request missing callback");
            }
            while (true)
            {
            }
            return;
        }

        _pendingEndSessionCallback = callback;
        _pendingEndSessionContext = context;
    }

    void LoopController::HaltExecutionEndProgram() noexcept
    {
        if (_sessionActive)
        {
            _programHaltRequested = true;
        }
    }

    void LoopController::SetNextModeWorkCallback(
        const ModeWorkCallback callback,
        void* const context) noexcept
    {
        if (!_sessionActive)
        {
            return;
        }

        if (callback == nullptr)
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode("LoopController next mode-work callback missing callback");
            }
            while (true)
            {
            }
            return;
        }

        _stagedModeWorkCallback = callback;
        _stagedModeWorkContext = context;
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

    LoopController::ControlVector LoopController::RunApplicationModeTick(
        void* const context,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        return static_cast<IApplicationMode*>(context)->RunTick(loopEndTimeUs, state, loopController);
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

    void LoopController::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _motorPwmSink.context = &runtime;
        _motorPwmSink.setMotorPwm = &SetMotorPwmThunk;
    }

    void LoopController::BindApplicationMode(IApplicationMode& mode) noexcept
    {
        _boundMode = &mode;
    }

    void LoopController::StartSessionFromStagedState() noexcept
    {
        if (!_stagedNextSessionValid)
        {
            _runtime->FailActiveMode("LoopController session state was not staged before session start");
        }

        if (!ValidateSessionOptions(_stagedNextSessionOptions))
        {
            _runtime->FailActiveMode("LoopController staged session state is invalid");
        }

        _sessionStartWallSensorAdcProbePending = true;
        RunSessionStartWallSensorAdcProbe();
        _options = _stagedNextSessionOptions;
        _stagedNextSessionOptions = SessionOptions{};
        _stagedNextSessionValid = false;
        _activeModeWorkCallback = &RunApplicationModeTick;
        _activeModeWorkContext = _boundMode;
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
        ClearPendingRequests();
    }

    void LoopController::Run()
    {
        if ((_runtime == nullptr) || !_motorPwmSink || (_boundMode == nullptr))
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode(
                    "LoopController run missing bound infrastructure mode or runtime");
            }
            while (true)
            {
            }
        }

        StartSessionFromStagedState();

        while (true)
        {
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
            const ControlVector brakeControl = ControlVector::Brake;
            const char* terminalFaultReason = nullptr;

            _appliedControl = _queuedControl;
            if (!ApplyControlAtTickStart(_appliedControl))
            {
                _queuedControl = ControlVector::Brake;
                terminalFaultReason = "LoopController motor PWM hook failed";
                ServiceRuntimeLogsForFaultPath();
                RecordPostServiceTiming();
                FinalizeTiming();
                PublishWorkingTiming();
                WaitUntilUs(loopEndTimeUs);
                _nextSyncTargetUs += _options.controlPeriodUs;
                _runtime->FailActiveMode(terminalFaultReason);
            }
            timing.tActuationAppliedUs =
                RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));

            if (_sessionStartWallSensorAdcProbePending)
            {
                RunSessionStartWallSensorAdcProbe();
            }

            ClearPendingRequests();

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                terminalFaultReason = "LoopController sensing update failed";
            }

            ControlVector candidateControl = ControlVector::Brake;
            if ((terminalFaultReason == nullptr) && (_activeModeWorkCallback == nullptr))
            {
                _runtime->FailActiveMode("LoopController active callback missing");
            }

            if (terminalFaultReason == nullptr)
            {
                candidateControl =
                    _activeModeWorkCallback(
                        _activeModeWorkContext,
                        loopEndTimeUs,
                        _runtime->RuntimeState(),
                        *this);
                RecordModeReturnTiming();

                if ((_pendingPauseCallback == nullptr) &&
                    (_pendingEndSessionCallback == nullptr) &&
                    !_programHaltRequested &&
                    (_stagedModeWorkCallback != nullptr))
                {
                    _activeModeWorkCallback = _stagedModeWorkCallback;
                    _activeModeWorkContext = _stagedModeWorkContext;
                }
            }

            if (terminalFaultReason != nullptr)
            {
                _queuedControl = ControlVector::Brake;
            }
            else if (_programHaltRequested)
            {
                _queuedControl = ControlVector::Brake;
            }
            else if (_pendingEndSessionCallback != nullptr)
            {
                _queuedControl = ControlVector::Brake;
            }
            else if (_pendingPauseCallback != nullptr)
            {
                _queuedControl = ControlVector::Brake;
            }
            else
            {
                _queuedControl = candidateControl;
            }

            if (terminalFaultReason != nullptr)
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
                    terminalFaultReason =
                        ((runtimeReason != nullptr) && (runtimeReason[0] != '\0')) ?
                        runtimeReason :
                        "LoopController runtime log service failed";
                    ServiceRuntimeLogsForFaultPath();
                }
            }

            RecordPostServiceTiming();
            FinalizeTiming();
            PublishWorkingTiming();

            WaitUntilUs(loopEndTimeUs);
            _nextSyncTargetUs += _options.controlPeriodUs;

            if (terminalFaultReason != nullptr)
            {
                _appliedControl = brakeControl;
                _queuedControl = brakeControl;
                if (!ApplyControlAtTickStart(brakeControl))
                {
                    _runtime->FailActiveMode(
                        "LoopController motor PWM hook failed during terminal fault handling");
                }
                _runtime->FailActiveMode(terminalFaultReason);
            }

            if (_programHaltRequested)
            {
                _appliedControl = brakeControl;
                _queuedControl = brakeControl;
                if (!ApplyControlAtTickStart(brakeControl))
                {
                    _runtime->FailActiveMode(
                        "LoopController motor PWM hook failed during program halt");
                }
                ResetExecutionState();
                return;
            }

            if (_pendingEndSessionCallback != nullptr)
            {
                _appliedControl = brakeControl;
                _queuedControl = brakeControl;
                if (!ApplyControlAtTickStart(brakeControl))
                {
                    _runtime->FailActiveMode(
                        "LoopController motor PWM hook failed during session handoff");
                }
                ResolveEndSessionRequest();

                if (_programHaltRequested)
                {
                    _appliedControl = brakeControl;
                    _queuedControl = brakeControl;
                    if (!ApplyControlAtTickStart(brakeControl))
                    {
                        _runtime->FailActiveMode(
                            "LoopController motor PWM hook failed during end-session-driven program halt");
                    }
                    ResetExecutionState();
                    return;
                }

                StartSessionFromStagedState();
                continue;
            }

            if (_pendingPauseCallback != nullptr)
            {
                ResolvePauseRequest();

                if (_programHaltRequested)
                {
                    _appliedControl = brakeControl;
                    _queuedControl = brakeControl;
                    if (!ApplyControlAtTickStart(brakeControl))
                    {
                        _runtime->FailActiveMode(
                            "LoopController motor PWM hook failed during pause-driven program halt");
                    }
                    ResetExecutionState();
                    return;
                }

                if (_pendingEndSessionCallback != nullptr)
                {
                    _appliedControl = brakeControl;
                    _queuedControl = brakeControl;
                    if (!ApplyControlAtTickStart(brakeControl))
                    {
                        _runtime->FailActiveMode(
                            "LoopController motor PWM hook failed during pause-driven session handoff");
                    }
                    ResolveEndSessionRequest();

                    if (_programHaltRequested)
                    {
                        _appliedControl = brakeControl;
                        _queuedControl = brakeControl;
                        if (!ApplyControlAtTickStart(brakeControl))
                        {
                            _runtime->FailActiveMode(
                                "LoopController motor PWM hook failed during pause-and-end-session-driven program halt");
                        }
                        ResetExecutionState();
                        return;
                    }

                    StartSessionFromStagedState();
                    continue;
                }
            }
        }
    }

    bool LoopController::ValidateSessionOptions(const SessionOptions& options) const noexcept
    {
        return (options.controlPeriodUs > 0U) && SupportsSensorWorkPlan(options.workPlan);
    }

    bool LoopController::SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept
    {
        return IsFullSensorWorkPlan(workPlan);
    }

    const LoopController::TimingDiagnostics* LoopController::CurrentTimingForReaders() const noexcept
    {
        if (_publishedTimingValid)
        {
            return &_timingBuffers[_publishedTimingIndex];
        }

        if (_sessionActive && (_tickCount > 0U))
        {
            return &_timingBuffers[_workingTimingIndex];
        }

        return nullptr;
    }

    void LoopController::ClearPendingRequests() noexcept
    {
        _pendingPauseCallback = nullptr;
        _pendingPauseContext = nullptr;
        _pendingEndSessionCallback = nullptr;
        _pendingEndSessionContext = nullptr;
        _programHaltRequested = false;
        _stagedModeWorkCallback = nullptr;
        _stagedModeWorkContext = nullptr;
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
        if (_runtime != nullptr)
        {
            (void)_runtime->ServiceUtilityDataLog();
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

    void LoopController::ResolvePauseRequest()
    {
        WaitForBrakeSettlement();

        const PauseCallback pauseCallback = _pendingPauseCallback;
        void* const pauseContext = _pendingPauseContext;
        ClearPendingRequests();

        if (pauseCallback == nullptr)
        {
            _runtime->FailActiveMode("LoopController pause request missing callback");
        }

        pauseCallback(pauseContext, *this);

        if (_pendingPauseCallback != nullptr)
        {
            _runtime->FailActiveMode("LoopController pause callback requested a nested pause");
        }

        if (_pendingEndSessionCallback != nullptr)
        {
            return;
        }

        if (_programHaltRequested)
        {
            return;
        }

        if (_stagedModeWorkCallback != nullptr)
        {
            _activeModeWorkCallback = _stagedModeWorkCallback;
            _activeModeWorkContext = _stagedModeWorkContext;
        }

        _queuedControl = ControlVector::Brake;
        _appliedControl = ControlVector::Brake;
        _resumePending = true;
        const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
        _lastTickStartUs = nowUs - _options.controlPeriodUs;
        _nextSyncTargetUs = nowUs + _options.controlPeriodUs;
        ClearPendingRequests();
    }

    void LoopController::ResolveEndSessionRequest()
    {
        WaitForBrakeSettlement();

        const EndSessionCallback endSessionCallback = _pendingEndSessionCallback;
        void* const endSessionContext = _pendingEndSessionContext;
        ClearPendingRequests();
        _stagedNextSessionOptions = SessionOptions{};
        _stagedNextSessionValid = false;

        if (endSessionCallback == nullptr)
        {
            _runtime->FailActiveMode("LoopController end-session request missing callback");
        }

        endSessionCallback(endSessionContext, *this);

        if (_pendingPauseCallback != nullptr)
        {
            _runtime->FailActiveMode(
                "LoopController end-session callback requested a pause boundary");
        }

        if (_pendingEndSessionCallback != nullptr)
        {
            _runtime->FailActiveMode(
                "LoopController end-session callback requested a nested end-session");
        }

        if (_stagedModeWorkCallback != nullptr)
        {
            _runtime->FailActiveMode(
                "LoopController end-session callback requested in-session callback transfer");
        }

        if (_programHaltRequested)
        {
            return;
        }

        if (!_stagedNextSessionValid)
        {
            _runtime->FailActiveMode(
                "LoopController end-session callback did not stage the next session state");
        }

        _queuedControl = ControlVector::Brake;
        _appliedControl = ControlVector::Brake;
    }

    void LoopController::WaitForBrakeSettlement()
    {
        if (_runtime == nullptr)
        {
            while (true)
            {
            }
        }

        std::uint8_t settledCount = 0U;
        while (settledCount < kPauseSettledTicks)
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
            timing.controlTiming.controlStartUs = tickStartUs;
            timing.controlTiming.cycleCounterStart = ReadCycleCounter();

            _appliedControl = ControlVector::Brake;
            _queuedControl = _appliedControl;
            if (!ApplyControlAtTickStart(_appliedControl))
            {
                ServiceRuntimeLogsForFaultPath();
                _runtime->FailActiveMode(
                    "LoopController motor PWM hook failed during brake settlement");
            }
            timing.tActuationAppliedUs =
                RelativeTickUs(tickStartUs, static_cast<std::uint32_t>(micros()));

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                ServiceRuntimeLogsForFaultPath();
                _runtime->FailActiveMode("LoopController brake settlement capture failed");
            }

            if (!ServiceRuntimeLogsNormal())
            {
                const char* const runtimeReason = _runtime->LastRuntimeLogError();
                ServiceRuntimeLogsForFaultPath();
                _runtime->FailActiveMode(
                    ((runtimeReason != nullptr) && (runtimeReason[0] != '\0')) ?
                    runtimeReason :
                    "LoopController runtime log service failed during brake settlement");
            }

            const MazeMap::VehicleState& runtimeState = _runtime->RuntimeState();
            const float linearSpeedMps = runtimeState.GetVelocity();
            const float angularSpeedRadps = runtimeState.GetRotationalVelocity();
            const bool settled =
                std::isfinite(linearSpeedMps) &&
                std::isfinite(angularSpeedRadps) &&
                (std::fabs(linearSpeedMps) <= kPauseLinearThresholdMps) &&
                (std::fabs(angularSpeedRadps) <= kPauseAngularThresholdRadps) &&
                !_runtime->Estimator().HasFault();
            settledCount = settled ? static_cast<std::uint8_t>(settledCount + 1U) : 0U;

            RecordPostServiceTiming();
            FinalizeTiming();
            PublishWorkingTiming();
            WaitUntilUs(deadlineUs);
        }
    }

    void LoopController::ResetExecutionState() noexcept
    {
        _options = SessionOptions{};
        _activeModeWorkCallback = nullptr;
        _activeModeWorkContext = nullptr;
        _sessionActive = false;
        _resumePending = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        _lastTickStartUs = 0U;
        _nextSyncTargetUs = 0U;
        _queuedControl = ControlVector::Brake;
        _appliedControl = ControlVector::Brake;
        _sessionStartWallSensorAdcProbePending = false;
        _timingBuffers[0] = TimingDiagnostics{};
        _timingBuffers[1] = TimingDiagnostics{};
        _publishedTimingIndex = 0U;
        _workingTimingIndex = 1U;
        ClearPendingRequests();
        _boundMode = nullptr;
        _stagedNextSessionOptions = SessionOptions{};
        _stagedNextSessionValid = false;
    }
}
