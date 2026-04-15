#include "pch.h"
#include "LoopController.h"

#include "Defines.h"
#include "DriveBase.h"
#include "DiagnosticSensorSuite.h"
#include "HardwareConfig.h"
#include "MazeMapSharedRuntime.h"
#include "SensorSuite.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr float kZeroVelocityThresholdMps = 1.0e-4f;
        constexpr float kZeroYawThresholdRadps = 1.0e-4f;
        constexpr std::uint16_t kTickTimingSaturatedUs = 0xFFFFU;
        constexpr float kDefaultPauseLinearThresholdMps = 0.01f;
        constexpr float kDefaultPauseAngularThresholdRadps = 0.05f;
        constexpr std::uint8_t kDefaultPauseSettledTicks = 2U;

        inline std::uint32_t NowUs() noexcept
        {
            return static_cast<std::uint32_t>(micros());
        }

        inline bool IsFinitePositive(const float value) noexcept
        {
            return std::isfinite(value) && (value > 0.0f);
        }

        inline void WaitUntilUs(const std::uint32_t absoluteDeadlineUs) noexcept
        {
            while (static_cast<std::int32_t>(absoluteDeadlineUs - NowUs()) > 0)
            {
                const std::uint32_t remainingUs = static_cast<std::uint32_t>(absoluteDeadlineUs - NowUs());
                if (remainingUs > 10U)
                {
                    delayMicroseconds(5U);
                }
            }
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
        const float linearTarget,
        const float angularTarget) noexcept
    {
        ControlVector control{};
        control.kind = Kind::Velocity;
        control.linearTarget = linearTarget;
        control.angularTarget = angularTarget;
        return control;
    }

    LoopController::ControlVector LoopController::ControlVector::OpenLoopCommand(
        const float leftCommand,
        const float rightCommand) noexcept
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

    void LoopController::TickServices::SetNextModeWorkCallback(const ModeWorkCallback callback) noexcept
    {
        if ((_owner != nullptr) && (callback != nullptr))
        {
            _owner->_requests.nextModeWorkRequested = true;
            _owner->_requests.nextModeWorkCallback = callback;
        }
    }

    bool LoopController::BeginSession(const SessionOptions& options, const ModeCallbacks& callbacks)
    {
        if ((_runtime == nullptr) ||
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
        _sessionBegun = true;
        _sessionActive = true;
        _resumePending = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        const std::uint32_t nowUs = NowUs();
        _lastTickStartUs = nowUs - _options.controlPeriodUs;
        _nextSyncTargetUs = nowUs + _options.controlPeriodUs;
        _queuedControl = ControlVector::BrakeCommand();
        _appliedControl = ControlVector::BrakeCommand();
        _publishedTimingIndex = 0U;
        _workingTimingIndex = 1U;
        _timingBuffers[0] = TimingDiagnostics{};
        _timingBuffers[1] = TimingDiagnostics{};
        _observedScratch = ObservedTickState{};
        _pauseContextScratch = PauseContext{};
        _deferredTerminalOutcome = DeferredTerminalOutcome::None;
        _deferredTerminalReason = nullptr;
        ResetLatchedRequests();
        return true;
    }

    LoopController::SessionResult LoopController::Run()
    {
        SessionResult result{};
        result.tickCount = _tickCount;

        if (!_sessionActive || (_runtime == nullptr) || (_activeModeWorkCallback == nullptr))
        {
            result.status = SessionResult::Status::StoppedByRuntime;
            return result;
        }

        while (_sessionActive)
        {
            if (_deferredTerminalOutcome != DeferredTerminalOutcome::None)
            {
                const std::uint32_t nowUs = NowUs();
                const std::uint32_t dtUs = nowUs - _lastTickStartUs;
                const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
                _lastTickStartUs = nowUs;
                _appliedControl = ControlVector::BrakeCommand();
                _queuedControl = _appliedControl;
                ApplyControlAtTickStart(_appliedControl, dtSeconds);

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

            const std::uint32_t tickStartUs = NowUs();
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

            _appliedControl = NormalizeQueuedControl(_queuedControl);
            ApplyControlAtTickStart(_appliedControl, dtSeconds);
            timing.tActuationAppliedUs = RelativeTickUs(tickStartUs, NowUs());

            if (_sessionStartWallSensorAdcProbePending)
            {
                RunSessionStartWallSensorAdcProbe();
            }

            _observedScratch = ObservedTickState{};
            _observedScratch.sequence = _tickCount;
            _observedScratch.tickStartUs = tickStartUs;
            _observedScratch.dtUs = dtUs;
            _observedScratch.dtSeconds = dtSeconds;

            ResetLatchedRequests();

            if (!ExecuteSensingUpdate(_observedScratch, timing))
            {
                _queuedControl = ControlVector::BrakeCommand();
                timing.flags |= kTimingFlagRuntimeStopPending;
                _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                _deferredTerminalReason =
                    (_observedScratch.faultReason != nullptr) ?
                    _observedScratch.faultReason :
                    "LoopController sensing update failed";

                ServiceRuntimeLogsForFaultPath();
                RecordPostServiceTiming(tickStartUs);
                FinalizeTiming(tickStartUs);
                PublishWorkingTiming();
                WaitUntilUs(loopEndTimeUs);
                _nextSyncTargetUs += _options.controlPeriodUs;
                continue;
            }

            const std::uint32_t projectionAnchorUs =
                (timing.controlTiming.ukfUpdateEndUs != 0U) ?
                timing.controlTiming.ukfUpdateEndUs :
                tickStartUs;
            const bool overrunBeforeModeWork = (ComputeRemainingSlackUs(loopEndTimeUs) == 0U);
            const ModeState modeState =
                BuildModeState(_observedScratch, projectionAnchorUs, loopEndTimeUs, overrunBeforeModeWork);

            TickServices services(*this);
            ControlVector candidateControl = ControlVector::BrakeCommand();
            if (_tickCount >= kInitialModeCallbackTick)
            {
                candidateControl =
                    _activeModeWorkCallback(_callbacks.context, loopEndTimeUs, modeState, services);
                RecordModeReturnTiming(tickStartUs);

                if (_requests.nextModeWorkRequested && (_requests.nextModeWorkCallback != nullptr))
                {
                    _activeModeWorkCallback = _requests.nextModeWorkCallback;
                }
            }

            bool faultPathFlushRequired = false;
            if (_requests.runtimeStopReason != nullptr)
            {
                _queuedControl = ControlVector::BrakeCommand();
                timing.flags |= kTimingFlagRuntimeStopPending;
                _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                _deferredTerminalReason = _requests.runtimeStopReason;
                faultPathFlushRequired = true;
            }
            else if (_requests.endRequested)
            {
                _queuedControl = ControlVector::BrakeCommand();
                _deferredTerminalOutcome = DeferredTerminalOutcome::Complete;
            }
            else if (_requests.pauseRequested)
            {
                _queuedControl = ControlVector::BrakeCommand();
                timing.flags |= kTimingFlagPausePending;
            }
            else
            {
                _queuedControl = NormalizeQueuedControl(candidateControl);
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
                    _queuedControl = ControlVector::BrakeCommand();
                    timing.flags |= kTimingFlagRuntimeStopPending;
                    _deferredTerminalOutcome = DeferredTerminalOutcome::RuntimeStop;
                    _deferredTerminalReason =
                        ((runtimeReason != nullptr) && (runtimeReason[0] != '\0')) ?
                        runtimeReason :
                        "LoopController runtime log service failed";
                    ServiceRuntimeLogsForFaultPath();
                }
            }

            RecordPostServiceTiming(tickStartUs);
            FinalizeTiming(tickStartUs);
            PublishWorkingTiming();

            WaitUntilUs(loopEndTimeUs);
            _nextSyncTargetUs += _options.controlPeriodUs;

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
            _runtime->Drive().Brake();
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

    bool LoopController::IsZeroVelocityCommand(const ControlVector& command) noexcept
    {
        return (command.kind == ControlVector::Kind::Velocity) &&
            (std::fabs(command.linearTarget) <= kZeroVelocityThresholdMps) &&
            (std::fabs(command.angularTarget) <= kZeroYawThresholdRadps);
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

    PoseEstimate LoopController::ProjectEstimate(
        const PoseEstimate& estimate,
        const std::uint32_t projectionAnchorUs,
        const std::uint32_t commandApplyTimeUs) noexcept
    {
        PoseEstimate projected = estimate;
        const std::int32_t deltaUsSigned = static_cast<std::int32_t>(commandApplyTimeUs - projectionAnchorUs);
        if (deltaUsSigned <= 0)
        {
            projected.headingUnit = HeadingUnitFromYawRad(projected.yawRad);
            return projected;
        }

        const float dtSeconds = static_cast<float>(deltaUsSigned) * 1.0e-6f;
        if (!std::isfinite(dtSeconds) || (dtSeconds <= 0.0f))
        {
            projected.headingUnit = HeadingUnitFromYawRad(projected.yawRad);
            return projected;
        }

        const float linearSpeedMps = std::isfinite(projected.linearSpeedMps) ? projected.linearSpeedMps : 0.0f;
        const float angularSpeedRadps = std::isfinite(projected.angularSpeedRadps) ? projected.angularSpeedRadps : 0.0f;
        const float midYawRad = WrapAngleRad(projected.yawRad + (0.5f * angularSpeedRadps * dtSeconds));
        const Eigen::Vector2f midHeading = HeadingUnitFromYawRad(midYawRad);
        projected.xMeters += linearSpeedMps * midHeading.x() * dtSeconds;
        projected.yMeters += linearSpeedMps * midHeading.y() * dtSeconds;
        projected.yawRad = WrapAngleRad(projected.yawRad + (angularSpeedRadps * dtSeconds));
        projected.headingUnit = HeadingUnitFromYawRad(projected.yawRad);
        return projected;
    }

    void LoopController::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
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
        // Preserve the current interlaced diagnostic capture/update ordering exactly.
        // Widen selective sensor/update support only when the lower-level runtime tie-in supports it.
        return IsFullSensorWorkPlan(workPlan);
    }

    void LoopController::ResetLatchedRequests() noexcept
    {
        _requests = LatchedRequests{};
    }

    LoopController::ControlVector LoopController::NormalizeQueuedControl(const ControlVector& candidate) const noexcept
    {
        if (candidate.kind == ControlVector::Kind::NoChange)
        {
            return _appliedControl;
        }

        return candidate;
    }

    void LoopController::ApplyControlAtTickStart(const ControlVector& control, const float dtSeconds)
    {
        if (_runtime == nullptr)
        {
            return;
        }

        const ControlVector resolvedControl = NormalizeQueuedControl(control);
        switch (resolvedControl.kind)
        {
        case ControlVector::Kind::OpenLoopRaw:
            _runtime->Drive().CommandOpenLoopRaw(resolvedControl.leftOpenLoop, resolvedControl.rightOpenLoop);
            break;

        case ControlVector::Kind::Velocity:
            _runtime->Drive().CommandVelocity(resolvedControl.linearTarget, resolvedControl.angularTarget, dtSeconds);
            break;

        case ControlVector::Kind::Brake:
        case ControlVector::Kind::NoChange:
        default:
            _runtime->Drive().Brake();
            break;
        }
    }

    bool LoopController::ExecuteSensingUpdate(ObservedTickState& observed, TimingDiagnostics& timing)
    {
        if (!CaptureSelectedTickState(observed, timing))
        {
            return false;
        }

        observed.estimate = _runtime->Drive().GetPose();
        observed.driveTelemetry = _runtime->Drive().GetTelemetry();

        const float measuredYawRateRadps =
            observed.hasDiagnosticSensors ? observed.diagnosticSensors.gyroRadps : observed.sensors.gyroRadps;
        const DriveBase::MeasuredKinematics measured = _runtime->Drive().GetMeasuredKinematics(measuredYawRateRadps);
        observed.measured.linearSpeedMps = measured.linearSpeedMps;
        observed.measured.angularSpeedRadps = measured.angularSpeedRadps;

        if (_runtime->Drive().HasEstimatorFault())
        {
            observed.estimatorHealthy = false;
            observed.faultReason = _runtime->Drive().GetEstimatorFaultReason();
        }

        return true;
    }

    bool LoopController::CaptureMissionTickState(ObservedTickState& observed, TimingDiagnostics& timing)
    {
        if (_runtime == nullptr)
        {
            observed.faultReason = "LoopController runtime unavailable";
            return false;
        }

        const bool stationaryHint = ShouldTreatAppliedControlAsStationary();
        timing.controlTiming.encoderLatchUs = NowUs();
        observed.hasDiagnosticSensors = false;
        observed.sensors = _runtime->MissionSensors().Capture(
            stationaryHint,
            _runtime->Drive().GetPose(),
            [this, &observed, &timing](SensorSnapshot& captureSnapshot, auto&& serviceWallRead, auto&& captureImu) noexcept
            {
                timing.controlTiming.encoderReadDoneUs = NowUs();
                _runtime->Drive().UpdateOdometry(
                    observed.dtSeconds,
                    captureSnapshot,
                    &_runtime->Maze(),
                    &timing.controlTiming,
                    [this, &serviceWallRead]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->ServiceUtilityDataLog();
                        }
                        serviceWallRead();
                    },
                    [this, &captureImu]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->ServiceUtilityDataLog();
                        }
                        captureImu();
                    });
            });
        return true;
    }

    bool LoopController::CaptureDiagnosticTickState(ObservedTickState& observed, TimingDiagnostics& timing)
    {
        if (_runtime == nullptr)
        {
            observed.faultReason = "LoopController runtime unavailable";
            return false;
        }

        const bool stationaryHint = ShouldTreatAppliedControlAsStationary();
        timing.controlTiming.encoderLatchUs = NowUs();
        observed.hasDiagnosticSensors = true;
        observed.diagnosticSensors = _runtime->DiagnosticSensors().Capture(
            stationaryHint,
            _runtime->Drive().GetPose(),
            [this, &observed, &timing](
                DiagnosticSensorSnapshot& captureSnapshot,
                auto&& serviceWallRead,
                auto&& captureImu) noexcept
            {
                timing.controlTiming.encoderReadDoneUs = NowUs();
                _runtime->Drive().UpdateOdometry(
                    observed.dtSeconds,
                    captureSnapshot,
                    &_runtime->Maze(),
                    &timing.controlTiming,
                    [this, &serviceWallRead]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->ServiceUtilityDataLog();
                        }
                        serviceWallRead();
                    },
                    [this, &captureImu]() noexcept
                    {
                        if (_runtime != nullptr)
                        {
                            (void)_runtime->ServiceUtilityDataLog();
                        }
                        captureImu();
                    });
            });

        observed.sensors = SensorSnapshot{};
        observed.sensors.frontLeftDistanceM = observed.diagnosticSensors.frontLeft.distanceM;
        observed.sensors.frontRightDistanceM = observed.diagnosticSensors.frontRight.distanceM;
        observed.sensors.frontLeftDifferentialLight = observed.diagnosticSensors.frontLeft.differentialLight;
        observed.sensors.frontRightDifferentialLight = observed.diagnosticSensors.frontRight.differentialLight;
        observed.sensors.sideLeftDistanceM = observed.diagnosticSensors.sideLeft.distanceM;
        observed.sensors.sideRightDistanceM = observed.diagnosticSensors.sideRight.distanceM;
        observed.sensors.sideLeftDifferentialLight = observed.diagnosticSensors.sideLeft.differentialLight;
        observed.sensors.sideRightDifferentialLight = observed.diagnosticSensors.sideRight.differentialLight;
        observed.sensors.corridorErrorM = observed.diagnosticSensors.corridorErrorM;
        observed.sensors.frontSkewM = observed.diagnosticSensors.frontSkewM;
        observed.sensors.accelBodyXMps2 = observed.diagnosticSensors.accelBodyXMps2;
        observed.sensors.accelBodyYMps2 = observed.diagnosticSensors.accelBodyYMps2;
        observed.sensors.planarAccelMps2 = _runtime->DiagnosticSensors().GetPlanarAccelMps2(observed.diagnosticSensors);
        observed.sensors.gyroRawRadps = observed.diagnosticSensors.gyroRawRadps;
        observed.sensors.gyroBiasRadps = observed.diagnosticSensors.gyroBiasRadps;
        observed.sensors.gyroRadps = observed.diagnosticSensors.gyroRadps;
        observed.sensors.accelBiasValid = observed.diagnosticSensors.accelBiasValid;
        observed.sensors.frontWall = observed.diagnosticSensors.frontWall;
        observed.sensors.frontLeftWall = observed.diagnosticSensors.frontLeft.wall;
        observed.sensors.frontRightWall = observed.diagnosticSensors.frontRight.wall;
        observed.sensors.leftWall = observed.diagnosticSensors.leftWall;
        observed.sensors.rightWall = observed.diagnosticSensors.rightWall;
        observed.sensors.leftDistanceValidForControl = observed.diagnosticSensors.leftDistanceValidForControl;
        observed.sensors.rightDistanceValidForControl = observed.diagnosticSensors.rightDistanceValidForControl;

        timing.frontTiming = observed.diagnosticSensors.frontTiming;
        timing.leftTiming = observed.diagnosticSensors.leftTiming;
        timing.rightTiming = observed.diagnosticSensors.rightTiming;
        timing.imuTiming = observed.diagnosticSensors.imuTiming;
        return true;
    }

    bool LoopController::CaptureSelectedTickState(ObservedTickState& observed, TimingDiagnostics& timing)
    {
        return CaptureDiagnosticTickState(observed, timing);
    }

    LoopController::ModeState LoopController::BuildModeState(
        const ObservedTickState& observed,
        const std::uint32_t projectionAnchorUs,
        const std::uint32_t commandApplyTimeUs,
        const bool overrunBeforeModeWork) const noexcept
    {
        ModeState state{};
        state.sequence = observed.sequence;
        state.tickStartUs = observed.tickStartUs;
        state.commandApplyTimeUs = commandApplyTimeUs;
        state.dtUs = observed.dtUs;
        state.dtSeconds = observed.dtSeconds;
        state.estimate = ProjectEstimate(observed.estimate, projectionAnchorUs, commandApplyTimeUs);
        state.measured = observed.measured;
        state.driveTelemetry = observed.driveTelemetry;
        state.sensors = observed.sensors;
        state.diagnosticSensors = observed.diagnosticSensors;
        state.hasDiagnosticSensors = observed.hasDiagnosticSensors;
        state.estimatorHealthy = observed.estimatorHealthy;
        state.overrun = overrunBeforeModeWork || observed.overrun;
        state.faultReason = observed.faultReason;
        return state;
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

    void LoopController::RecordModeReturnTiming(const std::uint32_t tickStartUs) noexcept
    {
        WorkingTiming().tModeReturnUs = RelativeTickUs(tickStartUs, NowUs());
    }

    void LoopController::RecordPostServiceTiming(const std::uint32_t tickStartUs) noexcept
    {
        WorkingTiming().tPostServiceDoneUs = RelativeTickUs(tickStartUs, NowUs());
    }

    void LoopController::FinalizeTiming(const std::uint32_t tickStartUs) noexcept
    {
        TimingDiagnostics& timing = WorkingTiming();
        const std::uint32_t finalizeUs = NowUs();
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
        (void)tickStartUs;
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
        const std::int32_t remainingUs = static_cast<std::int32_t>(absoluteDeadlineUs - NowUs());
        return (remainingUs > 0) ? static_cast<std::uint32_t>(remainingUs) : 0U;
    }

    bool LoopController::ShouldTreatAppliedControlAsStationary() const noexcept
    {
        return (_appliedControl.kind == ControlVector::Kind::Brake) || IsZeroVelocityCommand(_appliedControl);
    }

    bool LoopController::ResolvePauseRequest(SessionResult& result)
    {
        ModeState settledState{};
        if (!WaitForPauseSettlement(_requests.pauseRequest, settledState))
        {
            if (_runtime != nullptr)
            {
                (void)_runtime->FailActiveMode(_deferredTerminalReason);
            }
            result.status = SessionResult::Status::StoppedByRuntime;
            result.tickCount = _tickCount;
            return false;
        }

        _pauseContextScratch.stateEstimate = settledState;
        _pauseContextScratch.reason = _requests.pauseRequest.reason;
        const PauseDisposition disposition =
            _requests.pauseRequest.onPauseGranted(_callbacks.context, _pauseContextScratch);
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
                const std::uint32_t nowUs = NowUs();
                _lastTickStartUs = nowUs - _options.controlPeriodUs;
                _nextSyncTargetUs = nowUs + _options.controlPeriodUs;
            }
            _queuedControl = ControlVector::BrakeCommand();
            _appliedControl = ControlVector::BrakeCommand();
            _resumePending = true;
            result.tickCount = _tickCount;
            return true;
        }
        }
    }

    bool LoopController::WaitForPauseSettlement(const PauseRequest& request, ModeState& settledState)
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
            const std::uint32_t tickStartUs = NowUs();
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

            _appliedControl = ControlVector::BrakeCommand();
            _queuedControl = _appliedControl;
            ApplyControlAtTickStart(_appliedControl, dtSeconds);
            timing.tActuationAppliedUs = RelativeTickUs(tickStartUs, NowUs());

            _observedScratch = ObservedTickState{};
            _observedScratch.sequence = _tickCount;
            _observedScratch.tickStartUs = tickStartUs;
            _observedScratch.dtUs = dtUs;
            _observedScratch.dtSeconds = dtSeconds;

            if (!ExecuteSensingUpdate(_observedScratch, timing))
            {
                _deferredTerminalReason =
                    (_observedScratch.faultReason != nullptr) ?
                    _observedScratch.faultReason :
                    "LoopController pause settlement capture failed";
                ServiceRuntimeLogsForFaultPath();
                return false;
            }

            const std::uint32_t projectionAnchorUs =
                (timing.controlTiming.ukfUpdateEndUs != 0U) ?
                timing.controlTiming.ukfUpdateEndUs :
                tickStartUs;
            settledState = BuildModeState(_observedScratch, projectionAnchorUs, projectionAnchorUs, false);

            if (request.flushLogsBeforeGrant)
            {
                ServiceRuntimeLogsForFaultPath();
            }

            const bool settled =
                std::fabs(settledState.measured.linearSpeedMps) <= linearThreshold &&
                std::fabs(settledState.measured.angularSpeedRadps) <= angularThreshold &&
                settledState.estimatorHealthy;
            settledCount = settled ? static_cast<std::uint8_t>(settledCount + 1U) : 0U;

            RecordPostServiceTiming(tickStartUs);
            FinalizeTiming(tickStartUs);
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
        _sessionBegun = false;
        _sessionActive = false;
        _resumePending = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        _lastTickStartUs = 0U;
        _nextSyncTargetUs = 0U;
        _queuedControl = ControlVector::BrakeCommand();
        _appliedControl = ControlVector::BrakeCommand();
        _sessionStartWallSensorAdcProbePending = false;
        _requests = LatchedRequests{};
        _deferredTerminalOutcome = DeferredTerminalOutcome::None;
        _deferredTerminalReason = nullptr;
        _observedScratch = ObservedTickState{};
        _pauseContextScratch = PauseContext{};
    }
}
