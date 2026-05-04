#include "pch.h"
#include "LoopController.h"

#include "Defines.h"
#include "DriveBase.h"
#include "HardwareConfig.h"
#include "IApplicationMode.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "SharedRobotRuntime.h"
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
        constexpr float kNoImuObservation = std::numeric_limits<float>::quiet_NaN();

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

        bool WallMaskIncludes(
            const LoopController::WallMask workMask,
            const LoopController::WallMask includedMask) noexcept
        {
            return
                (static_cast<std::uint8_t>(workMask) & static_cast<std::uint8_t>(includedMask)) ==
                static_cast<std::uint8_t>(includedMask);
        }

        bool WorkPlanRequestsWallSensors(const LoopController::SensorWorkPlan& workPlan) noexcept
        {
            return workPlan.wallMask != LoopController::WallMask::None;
        }

        void ClearFrontWallSnapshot(SensorSnapshot& snapshot) noexcept
        {
            snapshot.frontLeftDistanceM = 0.0f;
            snapshot.frontRightDistanceM = 0.0f;
            snapshot.frontLeftDifferentialLight = 0.0f;
            snapshot.frontRightDifferentialLight = 0.0f;
            snapshot.frontSkewM = 0.0f;
            snapshot.frontWall = false;
            snapshot.frontLeftWall = false;
            snapshot.frontRightWall = false;
            snapshot.frontWallObservationValid = false;
            snapshot.frontWallUsesFallbackDetection = false;
            snapshot.frontWallUsesCharacterizationDetection = false;
            snapshot.frontLeft = WallSensorTelemetry{};
            snapshot.frontRight = WallSensorTelemetry{};
            snapshot.frontTiming = OpticalObservationTiming{};
        }

        void ClearLeftWallSnapshot(SensorSnapshot& snapshot) noexcept
        {
            snapshot.sideLeftDistanceM = 0.0f;
            snapshot.sideLeftDifferentialLight = 0.0f;
            snapshot.leftWall = false;
            snapshot.leftDistanceValidForControl = false;
            snapshot.leftWallObservation = false;
            snapshot.leftWallObservationWindowValid = false;
            snapshot.leftTransitionDetected = false;
            snapshot.sideLeft = WallSensorTelemetry{};
            snapshot.leftTiming = OpticalObservationTiming{};
        }

        void ClearRightWallSnapshot(SensorSnapshot& snapshot) noexcept
        {
            snapshot.sideRightDistanceM = 0.0f;
            snapshot.sideRightDifferentialLight = 0.0f;
            snapshot.rightWall = false;
            snapshot.rightDistanceValidForControl = false;
            snapshot.rightWallObservation = false;
            snapshot.rightWallObservationWindowValid = false;
            snapshot.rightTransitionDetected = false;
            snapshot.sideRight = WallSensorTelemetry{};
            snapshot.rightTiming = OpticalObservationTiming{};
        }

        void ClearImuSnapshot(SensorSnapshot& snapshot) noexcept
        {
            snapshot.accelBodyXMps2 = 0.0f;
            snapshot.accelBodyYMps2 = 0.0f;
            snapshot.planarAccelMps2 = 0.0f;
            snapshot.gyroRawRadps = kNoImuObservation;
            snapshot.gyroBiasRadps = kNoImuObservation;
            snapshot.gyroRadps = kNoImuObservation;
            snapshot.accelBiasValid = false;
            snapshot.imuFrontRight = ImuTelemetry{};
            snapshot.imuBackLeft = ImuTelemetry{};
            snapshot.imuTiming = ImuObservationTiming{};
        }

        void ApplySensorWorkPlanToSnapshot(
            const LoopController::SensorWorkPlan& workPlan,
            SensorSnapshot& snapshot,
            const float expectedSideWallDistanceM) noexcept
        {
            if (!workPlan.readImuBundle)
            {
                ClearImuSnapshot(snapshot);
            }

            if (!WallMaskIncludes(workPlan.wallMask, LoopController::WallMask::Front))
            {
                ClearFrontWallSnapshot(snapshot);
            }

            if (!WallMaskIncludes(workPlan.wallMask, LoopController::WallMask::Left))
            {
                ClearLeftWallSnapshot(snapshot);
            }

            if (!WallMaskIncludes(workPlan.wallMask, LoopController::WallMask::Right))
            {
                ClearRightWallSnapshot(snapshot);
            }

            snapshot.corridorErrorM = MazeMap::App::Internal::Runtime::ComputeCorridorError(
                snapshot.sideLeftDistanceM,
                snapshot.sideRightDistanceM,
                snapshot.leftDistanceValidForControl,
                snapshot.rightDistanceValidForControl,
                expectedSideWallDistanceM);
        }

        MazeMap::WallObs BuildFrontWallObservation(
            const float distanceM,
            const bool valid,
            const bool useFallbackDetection,
            const bool useCharacterizationDetection,
            const float maxRangeM) noexcept
        {
            MazeMap::WallObs observation{};
            if (!valid || !std::isfinite(distanceM) || (distanceM <= 0.0f))
            {
                return observation;
            }

            observation.valid = true;
            observation.rho = (std::clamp)(distanceM, 0.01f, maxRangeM);
            observation.confidence =
                useCharacterizationDetection ? 0.90f :
                (useFallbackDetection ? 0.60f : 0.80f);
            observation.cls = MazeMap::ObsClass::WallLike;
            return observation;
        }

        MazeMap::WallObs BuildSideWallObservation(
            const bool distanceValidForControl,
            const bool transitionDetected,
            const bool wallObservation,
            const float sideDistanceM,
            const float maxRangeM) noexcept
        {
            MazeMap::WallObs observation{};
            if (!distanceValidForControl ||
                transitionDetected ||
                !wallObservation ||
                !std::isfinite(sideDistanceM) ||
                (sideDistanceM <= 0.0f))
            {
                return observation;
            }

            observation.valid = true;
            observation.rho = (std::clamp)(sideDistanceM, 0.01f, maxRangeM);
            observation.confidence = 0.80f;
            observation.cls = MazeMap::ObsClass::WallLike;
            return observation;
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
        if ((_runtime == nullptr) || !_sessionStartWallSensorAdcProbePending)
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

        _options = _stagedNextSessionOptions;
        _stagedNextSessionOptions = SessionOptions{};
        _stagedNextSessionValid = false;
        if (!std::isfinite(_modeStartYawRad))
        {
            _modeStartYawRad = _runtime->RuntimeState().GetOrientation();
        }
        RestoreSessionStartPhysicalState();
        _sessionStartWallSensorAdcProbePending = WorkPlanRequestsWallSensors(_options.workPlan);
        RunSessionStartWallSensorAdcProbe();
        _activeModeWorkCallback = &RunApplicationModeTick;
        _activeModeWorkContext = _boundMode;
        _sessionActive = true;
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

    void LoopController::RestoreSessionStartPhysicalState() noexcept
    {
        if (_runtime == nullptr)
        {
            while (true)
            {
            }
        }

        if (!std::isfinite(_modeStartYawRad))
        {
            _runtime->FailActiveMode("LoopController session-start yaw was unavailable");
        }

        if (!_runtime->Drive().RestoreSessionStartPhysicalState(
                _options.SessionStartPointX,
                _options.SessionStartPointY,
                _modeStartYawRad))
        {
            _runtime->FailActiveMode("LoopController failed to restore the staged session-start physical state");
        }

        _runtime->DriveService().StartHold(0U, false);
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
        return
            (options.controlPeriodUs > 0U) &&
            std::isfinite(options.SessionStartPointX) &&
            std::isfinite(options.SessionStartPointY) &&
            SupportsSensorWorkPlan(options.workPlan);
    }

    bool LoopController::SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept
    {
        const std::uint8_t maskBits = static_cast<std::uint8_t>(workPlan.wallMask);
        const std::uint8_t allowedMaskBits = static_cast<std::uint8_t>(WallMask::All);
        if ((maskBits & static_cast<std::uint8_t>(~allowedMaskBits)) != 0U)
        {
            return false;
        }

        if (workPlan.useEncoderUpdate && !workPlan.readEncoders)
        {
            return false;
        }

        if ((workPlan.useGyroUpdate || workPlan.useAccelUpdate) && !workPlan.readImuBundle)
        {
            return false;
        }

        if (workPlan.useWallUpdates && !WorkPlanRequestsWallSensors(workPlan))
        {
            return false;
        }

        return true;
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
        timing.controlTiming.encoderLatchUs = static_cast<std::uint32_t>(micros());
        SensorSnapshot snapshot{};
        if (WorkPlanRequestsWallSensors(_options.workPlan) || _options.workPlan.readImuBundle)
        {
            _runtime->Sensors().Capture(
                stationaryHint,
                _runtime->RuntimeState(),
                snapshot);
        }

        ApplySensorWorkPlanToSnapshot(
            _options.workPlan,
            snapshot,
            MazeMap::Config::kExpectedSideWallDistanceM);

        DriveBase& drive = _runtime->Drive();
        drive.RecordMeasurementInputs(snapshot);
        const MazeMap::ControlInput control = drive.CurrentControlInput();
        const MazeMap::VehicleState::DriveCommandState driveCommandState =
            drive.BuildDriveCommandState(control);
        const MazeMap::EncoderObs encoderObservation = drive.ConsumeEncoderObservation(dtSeconds);
        timing.controlTiming.encoderReadDoneUs = static_cast<std::uint32_t>(micros());

        MazeMap::Estimator& estimator = _runtime->Estimator();
        MazeMap::VehicleState& runtimeState = _runtime->RuntimeState();
        runtimeState.SetControlInput(control);
        runtimeState.SetSensorSnapshot(snapshot);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        }

        if (estimator.HasFault())
        {
            return false;
        }

        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            timing.controlTiming.ukfPredictStartUs = static_cast<std::uint32_t>(micros());
            if (!estimator.predict(dtSeconds, control))
            {
                timing.controlTiming.ukfPredictEndUs = static_cast<std::uint32_t>(micros());
                timing.controlTiming.ukfPredictDurationUs =
                    timing.controlTiming.ukfPredictEndUs - timing.controlTiming.ukfPredictStartUs;
                timing.controlTiming.ukfUpdateStartUs = timing.controlTiming.ukfPredictEndUs;
                timing.controlTiming.ukfUpdateEndUs = timing.controlTiming.ukfPredictEndUs;
                timing.controlTiming.ukfUpdateDurationUs = 0U;
                timing.controlTiming.ukfTotalDurationUs = timing.controlTiming.ukfPredictDurationUs;
                return false;
            }

            timing.controlTiming.ukfPredictEndUs = static_cast<std::uint32_t>(micros());
            timing.controlTiming.ukfPredictDurationUs =
                timing.controlTiming.ukfPredictEndUs - timing.controlTiming.ukfPredictStartUs;
        }
        else
        {
            timing.controlTiming.ukfPredictStartUs = static_cast<std::uint32_t>(micros());
            timing.controlTiming.ukfPredictEndUs = timing.controlTiming.ukfPredictStartUs;
            timing.controlTiming.ukfPredictDurationUs = 0U;
        }

        timing.controlTiming.ukfUpdateStartUs = static_cast<std::uint32_t>(micros());

        if (_options.workPlan.readEncoders && _options.workPlan.useEncoderUpdate)
        {
            const bool updateYawFromEncoder =
                !_options.workPlan.readImuBundle ||
                !_options.workPlan.useGyroUpdate ||
                !std::isfinite(snapshot.gyroRawRadps);
            (void)estimator.updateEncoderPair(encoderObservation, dtSeconds, updateYawFromEncoder);
        }

        if (_options.workPlan.readImuBundle &&
            _options.workPlan.useGyroUpdate &&
            std::isfinite(snapshot.gyroRawRadps))
        {
            const MazeMap::MeasurementUpdateResult yawUpdate = estimator.updateYawRate(snapshot.gyroRawRadps);
            if (!yawUpdate.accepted)
            {
                timing.controlTiming.ukfUpdateEndUs = static_cast<std::uint32_t>(micros());
                timing.controlTiming.ukfUpdateDurationUs =
                    timing.controlTiming.ukfUpdateEndUs - timing.controlTiming.ukfUpdateStartUs;
                timing.controlTiming.ukfTotalDurationUs =
                    timing.controlTiming.ukfPredictDurationUs + timing.controlTiming.ukfUpdateDurationUs;
                return false;
            }
        }

        if (_options.workPlan.readImuBundle && _options.workPlan.useAccelUpdate)
        {
            MazeMap::ImuAccelObs accelObservation{};
            accelObservation.valid =
                snapshot.accelBiasValid &&
                std::isfinite(snapshot.accelBodyXMps2) &&
                std::isfinite(snapshot.accelBodyYMps2);
            accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
            accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
            (void)estimator.updatePlanarAccel(accelObservation);
        }

        if (_options.workPlan.useWallUpdates)
        {
            MazeMap::Maze& maze = _runtime->Maze();
            if (WallMaskIncludes(_options.workPlan.wallMask, WallMask::Front))
            {
                const MazeMap::WallObs frontLeftObservation = BuildFrontWallObservation(
                    snapshot.frontLeftDistanceM,
                    snapshot.frontWallObservationValid && snapshot.frontWall,
                    snapshot.frontWallUsesFallbackDetection,
                    snapshot.frontWallUsesCharacterizationDetection,
                    estimator.ukf().params().noHitRangeM);
                const MazeMap::WallObs frontRightObservation = BuildFrontWallObservation(
                    snapshot.frontRightDistanceM,
                    snapshot.frontWallObservationValid && snapshot.frontWall,
                    snapshot.frontWallUsesFallbackDetection,
                    snapshot.frontWallUsesCharacterizationDetection,
                    estimator.ukf().params().noHitRangeM);
                (void)estimator.updateFrontPair(frontLeftObservation, frontRightObservation, maze, true);
            }

            if (WallMaskIncludes(_options.workPlan.wallMask, WallMask::Left))
            {
                const MazeMap::WallObs leftObservation = BuildSideWallObservation(
                    snapshot.leftDistanceValidForControl,
                    snapshot.leftTransitionDetected,
                    snapshot.leftWallObservation,
                    snapshot.sideLeftDistanceM,
                    estimator.ukf().params().noHitRangeM);
                (void)estimator.updateSideSensor(MazeMap::Side::Left, leftObservation, maze, true);
            }

            if (WallMaskIncludes(_options.workPlan.wallMask, WallMask::Right))
            {
                const MazeMap::WallObs rightObservation = BuildSideWallObservation(
                    snapshot.rightDistanceValidForControl,
                    snapshot.rightTransitionDetected,
                    snapshot.rightWallObservation,
                    snapshot.sideRightDistanceM,
                    estimator.ukf().params().noHitRangeM);
                (void)estimator.updateSideSensor(MazeMap::Side::Right, rightObservation, maze, true);
            }
        }

        timing.controlTiming.ukfUpdateEndUs = static_cast<std::uint32_t>(micros());
        timing.controlTiming.ukfUpdateDurationUs =
            timing.controlTiming.ukfUpdateEndUs - timing.controlTiming.ukfUpdateStartUs;
        timing.controlTiming.ukfTotalDurationUs =
            timing.controlTiming.ukfPredictDurationUs + timing.controlTiming.ukfUpdateDurationUs;

        runtimeState.SetStateVector(estimator.ukf().state());
        runtimeState.SetCovariance(estimator.ukf().covariance());
        runtimeState.SetDriveCommandState(driveCommandState);
        runtimeState.SetTimestampUs(tickStartUs);
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
        _modeStartYawRad = std::numeric_limits<float>::quiet_NaN();
        _activeModeWorkCallback = nullptr;
        _activeModeWorkContext = nullptr;
        _sessionActive = false;
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
