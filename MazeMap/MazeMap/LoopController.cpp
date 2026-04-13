#include "pch.h"
#include "LoopController.h"

#include "DriveBase.h"
#include "DiagnosticSensorSuite.h"
#include "MazeMapSharedRuntime.h"
#include "SensorSuite.h"

#include <algorithm>
#include <cmath>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr float kZeroVelocityThresholdMps = 1.0e-4f;
        constexpr float kZeroYawThresholdRadps = 1.0e-4f;
        constexpr std::uint16_t kTickTimingSaturatedUs = 0xFFFFU;

        bool IsFinitePositive(float value) noexcept
        {
            return std::isfinite(value) && (value > 0.0f);
        }
    }

    LoopController::ControlVector LoopController::ControlVector::BrakeCommand() noexcept
    {
        return ControlVector{};
    }

    LoopController::ControlVector LoopController::ControlVector::HoldZeroVelocityCommand() noexcept
    {
        return VelocityCommand(0.0f, 0.0f);
    }

    LoopController::ControlVector LoopController::ControlVector::VelocityCommand(
        float linearTarget,
        float angularTarget) noexcept
    {
        ControlVector control{};
        control.kind = Kind::Velocity;
        control.linearTarget = linearTarget;
        control.angularTarget = angularTarget;
        return control;
    }

    LoopController::ControlVector LoopController::ControlVector::OpenLoopCommand(
        float leftCommand,
        float rightCommand) noexcept
    {
        ControlVector control{};
        control.kind = Kind::OpenLoopRaw;
        control.leftOpenLoop = leftCommand;
        control.rightOpenLoop = rightCommand;
        return control;
    }

    LoopController::ControlVector LoopController::ControlVector::NoChangeCommand() noexcept
    {
        ControlVector control{};
        control.kind = Kind::NoChange;
        return control;
    }

    LoopController::HeavyWorkResult LoopController::HeavyWorkResult::Resume() noexcept
    {
        return HeavyWorkResult{};
    }

    LoopController::HeavyWorkResult LoopController::HeavyWorkResult::Complete() noexcept
    {
        HeavyWorkResult result{};
        result.action = Action::Complete;
        return result;
    }

    LoopController::HeavyWorkResult LoopController::HeavyWorkResult::Fault(const char* reason) noexcept
    {
        HeavyWorkResult result{};
        result.action = Action::Fault;
        result.faultReason = reason;
        return result;
    }

    LoopController::TickServices::TickServices(LoopController& owner) noexcept
        : _owner(&owner)
    {
    }

    void LoopController::TickServices::Fault(const char* reason)
    {
        if (_owner != nullptr)
        {
            _owner->_requests.faultReason = reason;
        }
    }

    void LoopController::TickServices::RequestPauseForHeavyWork() noexcept
    {
        RequestPauseForHeavyWork(PauseRequest{});
    }

    void LoopController::TickServices::RequestPauseForHeavyWork(const PauseRequest& request) noexcept
    {
        if (_owner != nullptr)
        {
            _owner->_requests.pauseRequested = true;
            _owner->_requests.pauseRequest = request;
        }
    }

    void LoopController::TickServices::RequestEndLoop() noexcept
    {
        if (_owner != nullptr)
        {
            _owner->_requests.endRequested = true;
        }
    }

    void LoopController::TickServices::SetNextTickCaptureOptions(const CaptureOptions& options) noexcept
    {
        if (_owner != nullptr)
        {
            _owner->_requests.captureOverrideRequested = true;
            _owner->_requests.nextCapture = options;
        }
    }

    bool LoopController::BeginSession(
        const SessionConfig& config,
        RuntimeBundle& runtime,
        IMode& mode)
    {
        if (_sessionActive || _sessionBegun || !ValidateSessionConfig(config))
        {
            return false;
        }

        _config = config;
        _runtime = &runtime;
        _mode = &mode;
        _sessionBegun = true;
        _sessionActive = true;
        _sessionEndNotified = false;
        _captureOverrideActive = false;
        _resumePending = false;
        _tickCount = 0U;
        _lastTickStartUs = micros();
        _queuedControl = ResolveStartupCommand();
        _appliedControl = _queuedControl;
        _captureForNextTick = _config.defaultCapture;
        _faultReason = nullptr;
        _tickStepContext = nullptr;
        _tickStepCallback = nullptr;
        ResetLatchedRequests();

        const VehicleState initial = BuildInitialState();
        if (!_mode->OnSessionBegin(initial))
        {
            _sessionActive = false;
            _sessionBegun = false;
            _mode = nullptr;
            _runtime = nullptr;
            return false;
        }

        return true;
    }

    LoopController::SessionResult LoopController::Run()
    {
        if (!_sessionActive)
        {
            return SessionResult{};
        }

        while (_sessionActive)
        {
            const SessionResult result = RunOneTick();
            if (result.status != SessionResult::Status::Running)
            {
                return result;
            }
        }

        return SessionResult{};
    }

    LoopController::SessionResult LoopController::RunOneTick()
    {
        if (!_sessionActive || _runtime == nullptr || _mode == nullptr)
        {
            SessionResult result{};
            result.status = SessionResult::Status::Faulted;
            result.faultReason = "LoopController session is not active";
            return result;
        }

        if (!WaitForTickBoundaryAndService())
        {
            SessionResult result{};
            result.status = SessionResult::Status::Faulted;
            result.tickCount = _tickCount;
            result.faultReason = (_faultReason != nullptr) ? _faultReason : "LoopController boundary wait failed";
            return FinishSession(result);
        }

        VehicleState state{};
        state.tickStartUs = micros();
        state.dtUs = static_cast<std::uint32_t>(state.tickStartUs - _lastTickStartUs);
        state.dtSeconds = static_cast<float>(state.dtUs) * 1.0e-6f;
        _lastTickStartUs = state.tickStartUs;

        ++_tickCount;
        state.sequence = _config.maintainTickSequence ? _tickCount : 0U;
        state.captureUsed = _captureForNextTick;
        state.timing.tickStartUs = state.tickStartUs;
        state.timing.dtUs = state.dtUs;
        state.timing.flags = _captureOverrideActive ? kTimingFlagCaptureOverride : 0U;
        if (_resumePending)
        {
            state.resumedFromPause = true;
            state.timing.flags |= kTimingFlagResumedFromPause;
            _resumePending = false;
        }

        _appliedControl = NormalizeQueuedControl(_queuedControl);
        ApplyControlAtTickStart(_appliedControl, state.dtSeconds);
        state.appliedControl = _appliedControl;
        state.timing.tActuationAppliedUs = RelativeTickUs(state.tickStartUs, micros());

        if (_config.snapshotDriveTelemetry)
        {
            state.driveTelemetry = _runtime->driveBase.GetTelemetry();
        }

        if (!CaptureTickState(state))
        {
            _faultReason = (state.faultReason != nullptr) ? state.faultReason : "LoopController capture failed";
            SessionResult result{};
            result.status = SessionResult::Status::Faulted;
            result.tickCount = _tickCount;
            result.faultReason = _faultReason;
            ApplyFaultActuation();
            return FinishSession(result);
        }

        const std::uint32_t availableComputeUs = ComputeRemainingBudgetUs(state.tickStartUs);
        ResetLatchedRequests();
        TickServices services(*this);
        const ControlVector candidateControl = InvokeTickStep(availableComputeUs, state, services);
        RecordModeReturnTiming(state);

        SessionResult result{};
        result.status = SessionResult::Status::Running;
        result.tickCount = _tickCount;
        result.resumedFromPause = state.resumedFromPause;

        if (_requests.faultReason != nullptr)
        {
            _faultReason = _requests.faultReason;
            ApplyFaultActuation();
            result.status = SessionResult::Status::Faulted;
            result.faultReason = _faultReason;
        }
        else if (_requests.pauseRequested)
        {
            result.pauseGranted = true;
        }
        else if (_requests.endRequested)
        {
            ApplyTerminalActuation();
            result.status = SessionResult::Status::Completed;
        }
        else
        {
            _queuedControl = NormalizeQueuedControl(candidateControl);
            if (_requests.captureOverrideRequested)
            {
                if (!SupportsCaptureOptions(_requests.nextCapture))
                {
                    _faultReason = "Unsupported capture override requested";
                    ApplyFaultActuation();
                    result.status = SessionResult::Status::Faulted;
                    result.faultReason = _faultReason;
                }
                else
                {
                    _captureForNextTick = _requests.nextCapture;
                    _captureOverrideActive = true;
                }
            }
            else
            {
                _captureForNextTick = _config.defaultCapture;
                _captureOverrideActive = false;
            }
        }

        ServiceSlackState();
        RecordPostServiceTiming(state);

        if (result.status == SessionResult::Status::Running && !result.pauseGranted)
        {
            const unsigned long deadlineUs = state.tickStartUs + _config.controlPeriodUs;
            while (static_cast<unsigned long>(micros() - state.tickStartUs) < _config.controlPeriodUs)
            {
                ServiceSlackState();
                const unsigned long nowUs = micros();
                if (static_cast<unsigned long>(nowUs - state.tickStartUs) >= _config.controlPeriodUs)
                {
                    break;
                }

                const unsigned long remainingUs = deadlineUs - nowUs;
                if (_config.idleSleepUs > 0U && remainingUs > _config.idleSleepUs)
                {
                    delayMicroseconds(static_cast<unsigned int>(_config.idleSleepUs));
                }
            }
        }

        RecordOverrun(state);

        if (result.pauseGranted)
        {
            if (!ResolvePauseRequest(result))
            {
                return FinishSession(result);
            }

            if (result.status == SessionResult::Status::Completed ||
                result.status == SessionResult::Status::Faulted)
            {
                return FinishSession(result);
            }
        }

        if (result.status == SessionResult::Status::Running)
        {
            return result;
        }

        return FinishSession(result);
    }

    void LoopController::EndSession()
    {
        if (!_sessionBegun)
        {
            return;
        }

        if (_sessionActive)
        {
            SessionResult result{};
            result.status = SessionResult::Status::Completed;
            result.tickCount = _tickCount;
            (void)FinishSession(result);
            return;
        }

        _sessionBegun = false;
        _runtime = nullptr;
        _mode = nullptr;
        _tickStepContext = nullptr;
        _tickStepCallback = nullptr;
    }

    bool LoopController::SessionActive() const noexcept
    {
        return _sessionActive;
    }

    std::uint16_t LoopController::RelativeTickUs(
        std::uint32_t tickStartUs,
        std::uint32_t timestampUs) noexcept
    {
        if (timestampUs <= tickStartUs)
        {
            return 0U;
        }

        const std::uint32_t elapsedUs = timestampUs - tickStartUs;
        return static_cast<std::uint16_t>((std::min)(elapsedUs, static_cast<std::uint32_t>(kTickTimingSaturatedUs)));
    }

    bool LoopController::IsZeroVelocityCommand(const ControlVector& command) noexcept
    {
        return (command.kind == ControlVector::Kind::Velocity) &&
            (std::fabs(command.linearTarget) <= kZeroVelocityThresholdMps) &&
            (std::fabs(command.angularTarget) <= kZeroYawThresholdRadps);
    }

    bool LoopController::IsFullCapture(const CaptureOptions& options) noexcept
    {
        return (options.walls == CaptureOptions::WallMask::All) &&
            options.readGyro &&
            options.readAccel;
    }

    bool LoopController::ValidateSessionConfig(const SessionConfig& config) const noexcept
    {
        return (config.controlPeriodUs > 0U) && SupportsCaptureOptions(config.defaultCapture);
    }

    LoopController::VehicleState LoopController::BuildInitialState() const noexcept
    {
        VehicleState initial{};
        if (_runtime != nullptr)
        {
            initial.captureUsed = _captureForNextTick;
            initial.appliedControl = _appliedControl;
            initial.estimate = _runtime->driveBase.GetPose();
            const DriveBase::MeasuredKinematics measured = _runtime->driveBase.GetMeasuredKinematics();
            initial.measured.leftVelocityMps = measured.leftVelocityMps;
            initial.measured.rightVelocityMps = measured.rightVelocityMps;
            initial.measured.linearSpeedMps = measured.linearSpeedMps;
            initial.measured.angularSpeedRadps = measured.angularSpeedRadps;
            initial.driveTelemetry = _runtime->driveBase.GetTelemetry();
            initial.hasDiagnosticSensors = (_runtime->diagnosticSensors != nullptr);
        }
        return initial;
    }

    LoopController::ControlVector LoopController::ResolveStartupCommand() const noexcept
    {
        switch (_config.startupCommandPolicy)
        {
        case SessionConfig::StartupCommandPolicy::UseProvidedInitialCommand:
            return NormalizeQueuedControl(_config.initialCommand);
        case SessionConfig::StartupCommandPolicy::HoldZeroVelocity:
            return ControlVector::HoldZeroVelocityCommand();
        case SessionConfig::StartupCommandPolicy::Brake:
        default:
            return ControlVector::BrakeCommand();
        }
    }

    LoopController::ControlVector LoopController::NormalizeQueuedControl(const ControlVector& candidate) const noexcept
    {
        switch (candidate.kind)
        {
        case ControlVector::Kind::NoChange:
            return _appliedControl;
        case ControlVector::Kind::OpenLoopRaw:
            if (_config.actuationPolicy == SessionConfig::ActuationPolicy::VelocityBrakeOpenLoop)
            {
                return candidate;
            }
            return ControlVector::BrakeCommand();
        case ControlVector::Kind::Velocity:
        case ControlVector::Kind::Brake:
        default:
            return candidate;
        }
    }

    LoopController::ControlVector LoopController::InvokeTickStep(
        std::uint32_t availableComputeUs,
        const VehicleState& state,
        TickServices& services)
    {
        if (_tickStepCallback != nullptr)
        {
            return _tickStepCallback(_tickStepContext, availableComputeUs, state, services);
        }

        return _mode->Step(availableComputeUs, state, services);
    }

    void LoopController::ApplyControlAtTickStart(const ControlVector& control, float dtSeconds)
    {
        if (_runtime == nullptr)
        {
            return;
        }

        const ControlVector resolvedControl = NormalizeQueuedControl(control);
        switch (resolvedControl.kind)
        {
        case ControlVector::Kind::OpenLoopRaw:
            _runtime->driveBase.CommandOpenLoopRaw(resolvedControl.leftOpenLoop, resolvedControl.rightOpenLoop);
            break;
        case ControlVector::Kind::Velocity:
            _runtime->driveBase.CommandVelocity(resolvedControl.linearTarget, resolvedControl.angularTarget, dtSeconds);
            break;
        case ControlVector::Kind::Brake:
        case ControlVector::Kind::NoChange:
        default:
            _runtime->driveBase.Brake();
            break;
        }
    }

    void LoopController::ApplyTerminalActuation() noexcept
    {
        if (_runtime == nullptr)
        {
            return;
        }

        if (_config.actuationPolicy == SessionConfig::ActuationPolicy::VelocityBrakeOpenLoop)
        {
            _runtime->driveBase.Brake();
            return;
        }

        _runtime->driveBase.CommandVelocity(0.0f, 0.0f, 0.0f);
    }

    void LoopController::ApplyFaultActuation() noexcept
    {
        if (_runtime != nullptr)
        {
            _runtime->driveBase.Brake();
        }
    }

    bool LoopController::WaitForTickBoundaryAndService()
    {
        if (_runtime == nullptr)
        {
            return false;
        }

        while (static_cast<unsigned long>(micros() - _lastTickStartUs) < _config.controlPeriodUs)
        {
            ServiceBackgroundWork(_config.serviceWaitState);

            const unsigned long nowUs = micros();
            if (static_cast<unsigned long>(nowUs - _lastTickStartUs) >= _config.controlPeriodUs)
            {
                break;
            }

            if (_config.idleSleepUs > 0U)
            {
                const unsigned long remainingUs = (_lastTickStartUs + _config.controlPeriodUs) - nowUs;
                if (remainingUs > _config.idleSleepUs)
                {
                    delayMicroseconds(static_cast<unsigned int>(_config.idleSleepUs));
                }
            }
        }

        return true;
    }

    void LoopController::ServiceBackgroundWork(bool waitState) noexcept
    {
        if (_runtime == nullptr)
        {
            return;
        }

        (void)_runtime->shared.ServiceUtilityDataLog();
        if (waitState && _mode != nullptr)
        {
            _mode->ServiceWaitState();
        }
    }

    void LoopController::ServiceSlackState() noexcept
    {
        if (_runtime == nullptr)
        {
            return;
        }

        (void)_runtime->shared.ServiceUtilityDataLog();
        if (_config.serviceSlackState && _mode != nullptr)
        {
            _mode->ServiceSlackState();
        }
    }

    bool LoopController::CaptureTickState(VehicleState& state)
    {
        state.captureUsed = _captureForNextTick;
        const bool captured = CaptureSelectedTickState(state);
        if (!captured)
        {
            return false;
        }

        state.estimate = _runtime->driveBase.GetPose();
        if (!_config.snapshotDriveTelemetry)
        {
            state.driveTelemetry = _runtime->driveBase.GetTelemetry();
        }

        if (state.hasDiagnosticSensors)
        {
            const DriveBase::MeasuredKinematics measured =
                _runtime->driveBase.GetMeasuredKinematics(state.diagnosticSensors.gyroRadps);
            state.measured.leftVelocityMps = measured.leftVelocityMps;
            state.measured.rightVelocityMps = measured.rightVelocityMps;
            state.measured.linearSpeedMps = measured.linearSpeedMps;
            state.measured.angularSpeedRadps = measured.angularSpeedRadps;
            state.timing.tFrontReadyUs = RelativeTickUs(state.tickStartUs, state.diagnosticSensors.frontTiming.observationReadyUs);
            state.timing.tLeftReadyUs = RelativeTickUs(state.tickStartUs, state.diagnosticSensors.leftTiming.observationReadyUs);
            state.timing.tRightReadyUs = RelativeTickUs(state.tickStartUs, state.diagnosticSensors.rightTiming.observationReadyUs);
            state.timing.tImuDoneUs = RelativeTickUs(state.tickStartUs, state.diagnosticSensors.imuTiming.readDoneUs);
        }
        else
        {
            const DriveBase::MeasuredKinematics measured =
                _runtime->driveBase.GetMeasuredKinematics(state.sensors.gyroRadps);
            state.measured.leftVelocityMps = measured.leftVelocityMps;
            state.measured.rightVelocityMps = measured.rightVelocityMps;
            state.measured.linearSpeedMps = measured.linearSpeedMps;
            state.measured.angularSpeedRadps = measured.angularSpeedRadps;
        }

        state.timing.tEncoderDoneUs = RelativeTickUs(state.tickStartUs, state.controlCycleTiming.encoderReadDoneUs);
        state.timing.tEstimatorDoneUs = RelativeTickUs(state.tickStartUs, state.controlCycleTiming.ukfUpdateEndUs);

        if (_runtime->driveBase.HasEstimatorFault())
        {
            state.estimatorHealthy = false;
            state.faultReason = _runtime->driveBase.GetEstimatorFaultReason();
        }

        return true;
    }

    bool LoopController::CaptureMissionTickState(VehicleState& state)
    {
        if (_runtime == nullptr || _runtime->missionSensors == nullptr)
        {
            state.faultReason = "LoopController mission sensor pipeline unavailable";
            return false;
        }

        const bool stationaryHint = ShouldTreatAppliedControlAsStationary();
        state.hasDiagnosticSensors = false;
        state.controlCycleTiming.controlStartUs = state.tickStartUs;
        state.controlCycleTiming.encoderLatchUs = micros();
        state.sensors = _runtime->missionSensors->Capture(
            stationaryHint,
            _runtime->driveBase.GetPose(),
            [this, &state](SensorSnapshot& captureSnapshot, auto&& serviceWallRead, auto&& captureImu) noexcept
            {
                state.controlCycleTiming.encoderReadDoneUs = micros();
                _runtime->driveBase.UpdateOdometry(
                    state.dtSeconds,
                    captureSnapshot,
                    _runtime->maze,
                    &state.controlCycleTiming,
                    [this, &serviceWallRead]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->shared.ServiceUtilityDataLog();
                        }
                        serviceWallRead();
                    },
                    [this, &captureImu]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->shared.ServiceUtilityDataLog();
                        }
                        captureImu();
                    });
            });
        return true;
    }

    bool LoopController::CaptureDiagnosticTickState(VehicleState& state)
    {
        if (_runtime == nullptr || _runtime->diagnosticSensors == nullptr)
        {
            state.faultReason = "LoopController diagnostic sensor pipeline unavailable";
            return false;
        }

        const bool stationaryHint = ShouldTreatAppliedControlAsStationary();
        state.hasDiagnosticSensors = true;
        state.controlCycleTiming.controlStartUs = state.tickStartUs;
        state.controlCycleTiming.encoderLatchUs = micros();
        state.diagnosticSensors = _runtime->diagnosticSensors->Capture(
            stationaryHint,
            _runtime->driveBase.GetPose(),
            [this, &state](DiagnosticSensorSnapshot& captureSnapshot, auto&& serviceWallRead, auto&& captureImu) noexcept
            {
                state.controlCycleTiming.encoderReadDoneUs = micros();
                _runtime->driveBase.UpdateOdometry(
                    state.dtSeconds,
                    captureSnapshot,
                    _runtime->maze,
                    &state.controlCycleTiming,
                    [this, &serviceWallRead]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->shared.ServiceUtilityDataLog();
                        }
                        serviceWallRead();
                    },
                    [this, &captureImu]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->shared.ServiceUtilityDataLog();
                        }
                        captureImu();
                    });
            });

        state.sensors.frontLeftDistanceM = state.diagnosticSensors.frontLeft.distanceM;
        state.sensors.frontRightDistanceM = state.diagnosticSensors.frontRight.distanceM;
        state.sensors.frontLeftDifferentialLight = state.diagnosticSensors.frontLeft.differentialLight;
        state.sensors.frontRightDifferentialLight = state.diagnosticSensors.frontRight.differentialLight;
        state.sensors.sideLeftDistanceM = state.diagnosticSensors.sideLeft.distanceM;
        state.sensors.sideRightDistanceM = state.diagnosticSensors.sideRight.distanceM;
        state.sensors.sideLeftDifferentialLight = state.diagnosticSensors.sideLeft.differentialLight;
        state.sensors.sideRightDifferentialLight = state.diagnosticSensors.sideRight.differentialLight;
        state.sensors.corridorErrorM = state.diagnosticSensors.corridorErrorM;
        state.sensors.frontSkewM = state.diagnosticSensors.frontSkewM;
        state.sensors.accelBodyXMps2 = state.diagnosticSensors.accelBodyXMps2;
        state.sensors.accelBodyYMps2 = state.diagnosticSensors.accelBodyYMps2;
        state.sensors.planarAccelMps2 = 0.0f;
        state.sensors.gyroRawRadps = state.diagnosticSensors.gyroRawRadps;
        state.sensors.gyroBiasRadps = state.diagnosticSensors.gyroBiasRadps;
        state.sensors.gyroRadps = state.diagnosticSensors.gyroRadps;
        state.sensors.accelBiasValid = state.diagnosticSensors.accelBiasValid;
        state.sensors.frontWall = state.diagnosticSensors.frontWall;
        state.sensors.leftWall = state.diagnosticSensors.leftWall;
        state.sensors.rightWall = state.diagnosticSensors.rightWall;
        state.sensors.leftDistanceValidForControl = state.diagnosticSensors.leftDistanceValidForControl;
        state.sensors.rightDistanceValidForControl = state.diagnosticSensors.rightDistanceValidForControl;
        return true;
    }

    bool LoopController::CaptureSelectedTickState(VehicleState& state)
    {
        if (_runtime == nullptr)
        {
            state.faultReason = "LoopController runtime bundle unavailable";
            return false;
        }

        if (_runtime->diagnosticSensors != nullptr)
        {
            return CaptureDiagnosticTickState(state);
        }

        if (_runtime->missionSensors != nullptr)
        {
            return CaptureMissionTickState(state);
        }

        state.faultReason = "LoopController has no configured sensor pipeline";
        return false;
    }

    bool LoopController::CaptureTickStateWithResolvedSensors(VehicleState& state, bool stationaryHint)
    {
        (void)state;
        (void)stationaryHint;
        return false;
    }

    bool LoopController::SupportsCaptureOptions(const CaptureOptions& options) const noexcept
    {
        // The current shared sensor pipelines still execute the full capture contract.
        // Land the controller-owned loop first, then widen selective capture support on top.
        return IsFullCapture(options);
    }

    bool LoopController::ResolvePauseRequest(SessionResult& result)
    {
        VehicleState settledState{};
        if (!WaitForPauseSettlement(_requests.pauseRequest, settledState))
        {
            result.status = SessionResult::Status::Faulted;
            result.faultReason = (_faultReason != nullptr) ? _faultReason : "LoopController pause settlement failed";
            return false;
        }

        PauseContext pause{};
        pause.stateEstimate = settledState;
        pause.reason = _requests.pauseRequest.reason;
        const HeavyWorkResult heavyWork = _mode->OnPauseGranted(pause);
        switch (heavyWork.action)
        {
        case HeavyWorkResult::Action::Fault:
            _faultReason = (heavyWork.faultReason != nullptr) ? heavyWork.faultReason : "LoopController heavy work faulted";
            ApplyFaultActuation();
            result.status = SessionResult::Status::Faulted;
            result.faultReason = _faultReason;
            return false;

        case HeavyWorkResult::Action::Complete:
            ApplyTerminalActuation();
            result.status = SessionResult::Status::Completed;
            return false;

        case HeavyWorkResult::Action::Resume:
        default:
            if (heavyWork.resetClockOnResume || _requests.pauseRequest.resetClockOnResume)
            {
                _lastTickStartUs = micros();
            }
            _resumePending = true;
            result.resumedFromPause = true;
            result.pauseGranted = false;
            result.status = SessionResult::Status::Running;
            _captureForNextTick = _config.defaultCapture;
            _captureOverrideActive = false;
            return true;
        }
    }

    bool LoopController::WaitForPauseSettlement(
        const PauseRequest& request,
        VehicleState& settledState)
    {
        if (_runtime == nullptr)
        {
            _faultReason = "LoopController pause settlement missing runtime";
            return false;
        }

        const float linearThreshold =
            IsFinitePositive(request.maxAbsLinearSpeed) ?
            request.maxAbsLinearSpeed :
            _config.pauseDefaults.maxAbsLinearSpeed;
        const float angularThreshold =
            IsFinitePositive(request.maxAbsAngularSpeed) ?
            request.maxAbsAngularSpeed :
            _config.pauseDefaults.maxAbsAngularSpeed;
        const std::uint8_t settledTicks =
            (request.consecutiveSettledTicks > 0U) ?
            request.consecutiveSettledTicks :
            _config.pauseDefaults.consecutiveSettledTicks;

        std::uint8_t settledCount = 0U;
        const ControlVector settleControl =
            (_config.pauseDefaults.settleActuation == SessionConfig::PauseDefaults::SettleActuation::HoldZeroVelocity) ?
            ControlVector::HoldZeroVelocityCommand() :
            ControlVector::BrakeCommand();

        while (settledCount < settledTicks)
        {
            if (!WaitForTickBoundaryAndService())
            {
                _faultReason = "LoopController pause boundary wait failed";
                return false;
            }

            settledState = {};
            settledState.tickStartUs = micros();
            settledState.dtUs = static_cast<std::uint32_t>(settledState.tickStartUs - _lastTickStartUs);
            settledState.dtSeconds = static_cast<float>(settledState.dtUs) * 1.0e-6f;
            _lastTickStartUs = settledState.tickStartUs;
            ++_tickCount;
            settledState.sequence = _config.maintainTickSequence ? _tickCount : 0U;
            settledState.captureUsed = _config.defaultCapture;
            settledState.timing.tickStartUs = settledState.tickStartUs;
            settledState.timing.dtUs = settledState.dtUs;
            settledState.timing.flags = kTimingFlagPausePending;
            ApplyControlAtTickStart(settleControl, settledState.dtSeconds);
            settledState.appliedControl = settleControl;
            settledState.timing.tActuationAppliedUs = RelativeTickUs(settledState.tickStartUs, micros());

            if (!CaptureTickState(settledState))
            {
                _faultReason = (settledState.faultReason != nullptr) ? settledState.faultReason : "LoopController pause capture failed";
                return false;
            }

            if (request.flushServicesBeforeGrant || _config.pauseDefaults.flushServicesBeforeGrant)
            {
                (void)_runtime->shared.ServiceUtilityDataLog();
            }

            const bool settled =
                std::fabs(settledState.measured.linearSpeedMps) <= linearThreshold &&
                std::fabs(settledState.measured.angularSpeedRadps) <= angularThreshold &&
                settledState.estimatorHealthy;
            settledCount = settled ? static_cast<std::uint8_t>(settledCount + 1U) : 0U;

            ServiceSlackState();
        }

        return true;
    }

    LoopController::SessionResult LoopController::FinishSession(SessionResult result)
    {
        if (!_sessionEndNotified && _mode != nullptr)
        {
            if (result.status == SessionResult::Status::Faulted)
            {
                ApplyFaultActuation();
            }
            else
            {
                ApplyTerminalActuation();
            }

            _mode->OnSessionEnd(result);
            _sessionEndNotified = true;
        }

        _sessionActive = false;
        _sessionBegun = false;
        _runtime = nullptr;
        _mode = nullptr;
        _captureOverrideActive = false;
        _resumePending = false;
        ResetLatchedRequests();
        _tickStepContext = nullptr;
        _tickStepCallback = nullptr;
        return result;
    }

    std::uint32_t LoopController::ComputeRemainingBudgetUs(std::uint32_t tickStartUs) const noexcept
    {
        const std::uint32_t elapsedUs = static_cast<std::uint32_t>(micros() - tickStartUs);
        return (elapsedUs >= _config.controlPeriodUs) ? 0U : (_config.controlPeriodUs - elapsedUs);
    }

    bool LoopController::ShouldTreatAppliedControlAsStationary() const noexcept
    {
        return (_appliedControl.kind == ControlVector::Kind::Brake) || IsZeroVelocityCommand(_appliedControl);
    }

    void LoopController::ResetLatchedRequests() noexcept
    {
        _requests = LatchedRequests{};
    }

    void LoopController::RecordModeReturnTiming(VehicleState& state) const noexcept
    {
        state.timing.tModeReturnUs = RelativeTickUs(state.tickStartUs, micros());
    }

    void LoopController::RecordPostServiceTiming(VehicleState& state) const noexcept
    {
        state.timing.tPostServiceDoneUs = RelativeTickUs(state.tickStartUs, micros());
    }

    void LoopController::RecordOverrun(VehicleState& state) const noexcept
    {
        const std::uint32_t elapsedUs = static_cast<std::uint32_t>(micros() - state.tickStartUs);
        if (elapsedUs > _config.controlPeriodUs)
        {
            state.overrun = true;
            state.timing.overrunUs = static_cast<std::uint16_t>((std::min)(
                elapsedUs - _config.controlPeriodUs,
                static_cast<std::uint32_t>(kTickTimingSaturatedUs)));
        }
    }
}
