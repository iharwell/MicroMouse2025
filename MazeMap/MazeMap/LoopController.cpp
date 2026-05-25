#include "pch.h"
#include "LoopController.h"

#include "Defines.h"
#include "Direction.h"
#include "Estimator.h"
#include "IApplicationMode.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "EncoderObs.h"
#include "ImuAccelObs.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SharedRobotRuntime.h"
#include "Vehicle.h"
#include "WallObservationPipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MazeMap::App::Internal
{
    void LoopController::WaitUntilUs(const std::uint32_t absoluteDeadlineUs) noexcept
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

    void LoopController::ServiceInterlacedSensorCapture(void* const context) noexcept
    {
        LoopController* const loopController = static_cast<LoopController*>(context);
        if ((loopController == nullptr) || (loopController->_runtime == nullptr))
        {
            return;
        }

        RuntimeSensorSuite& sensors = loopController->_runtime->Sensors();
        sensors.ServiceFrontWallCollection();
        sensors.CaptureInterlacedInertialSnapshot();
        sensors.ServiceLeftWallCollection();
        sensors.ServiceRightWallCollection();
    }

    void LoopController::StageNextSessionState(
        const std::uint32_t controlPeriodUs,
        const float sessionStartPointX,
        const float sessionStartPointY,
        const WallMask wallMask,
        const bool useEncoderUpdate,
        const bool useGyroUpdate,
        const bool useAccelUpdate,
        const bool useWallUpdates) noexcept
    {
        std::uint8_t sensorWorkBits =
            static_cast<std::uint8_t>(static_cast<std::uint8_t>(wallMask) & RuntimeSensorSuite::kWallSensorBits);
        if (useEncoderUpdate)
        {
            sensorWorkBits = static_cast<std::uint8_t>(sensorWorkBits | RuntimeSensorSuite::kEncoderSensorBit);
        }
        if (useGyroUpdate)
        {
            sensorWorkBits = static_cast<std::uint8_t>(sensorWorkBits | RuntimeSensorSuite::kGyroSensorBit);
        }
        if (useAccelUpdate)
        {
            sensorWorkBits = static_cast<std::uint8_t>(sensorWorkBits | RuntimeSensorSuite::kAccelSensorBit);
        }
        if (useWallUpdates)
        {
            sensorWorkBits = static_cast<std::uint8_t>(sensorWorkBits | RuntimeSensorSuite::kWallUpdateSensorBit);
        }

        if (!ValidateSessionState(controlPeriodUs, sessionStartPointX, sessionStartPointY, sensorWorkBits))
        {
            if (_runtime != nullptr)
            {
                _runtime->FailActiveMode("LoopController staged session state is invalid");
            }
            while (true)
            {
            }
        }

        _stagedControlPeriodUs = controlPeriodUs;
        _stagedSessionStartPointX = sessionStartPointX;
        _stagedSessionStartPointY = sessionStartPointY;
        _stagedSensorWorkBits = sensorWorkBits;
        _stagedNextSessionValid = true;
    }

    void LoopController::RequestPause(
        void (* const callback)(void*, LoopController&),
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
        void (* const callback)(void*, LoopController&),
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
        CommandVector (* const callback)(void*, std::uint32_t, const MazeMap::VehicleState&, LoopController&),
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

    std::uint32_t LoopController::LastTimingSequence() const noexcept { return PublishedTiming().Sequence(); }
    std::uint32_t LoopController::LastTimingTickStartUs() const noexcept { return PublishedTiming().TickStartUs(); }
    std::uint32_t LoopController::LastTimingDtUs() const noexcept { return PublishedTiming().DtUs(); }
    std::uint32_t LoopController::LastTimingCommandAppliedUs() const noexcept { return PublishedTiming().CommandAppliedUs(); }
    std::uint32_t LoopController::LastTimingEncoderLatchUs() const noexcept { return PublishedTiming().EncoderLatchUs(); }
    std::uint32_t LoopController::LastTimingEncoderReadDoneUs() const noexcept { return PublishedTiming().EncoderReadDoneUs(); }
    std::uint32_t LoopController::LastTimingEstimatorPredictStartUs() const noexcept { return PublishedTiming().EstimatorPredictStartUs(); }
    std::uint32_t LoopController::LastTimingEstimatorPredictEndUs() const noexcept { return PublishedTiming().EstimatorPredictEndUs(); }
    std::uint32_t LoopController::LastTimingEstimatorPredictDurationUs() const noexcept { return PublishedTiming().EstimatorPredictDurationUs(); }
    std::uint32_t LoopController::LastTimingEstimatorUpdateStartUs() const noexcept { return PublishedTiming().EstimatorUpdateStartUs(); }
    std::uint32_t LoopController::LastTimingEstimatorUpdateEndUs() const noexcept { return PublishedTiming().EstimatorUpdateEndUs(); }
    std::uint32_t LoopController::LastTimingEstimatorUpdateDurationUs() const noexcept { return PublishedTiming().EstimatorUpdateDurationUs(); }
    std::uint32_t LoopController::LastTimingEstimatorTotalDurationUs() const noexcept { return PublishedTiming().EstimatorTotalDurationUs(); }
    std::uint32_t LoopController::LastTimingCallbackReturnUs() const noexcept { return PublishedTiming().CallbackReturnUs(); }
    std::uint32_t LoopController::LastTimingPostServiceDoneUs() const noexcept { return PublishedTiming().PostServiceDoneUs(); }
    std::uint32_t LoopController::LastTimingTickFinalizeUs() const noexcept { return PublishedTiming().TickFinalizeUs(); }
    std::uint32_t LoopController::LastTimingCycleCounterStart() const noexcept { return PublishedTiming().CycleCounterStart(); }
    std::uint32_t LoopController::LastTimingCycleCounterEnd() const noexcept { return PublishedTiming().CycleCounterEnd(); }
    std::uint16_t LoopController::LastTimingOverrunUs() const noexcept { return PublishedTiming().OverrunUs(); }

    const CommandVector& LoopController::LastAppliedCommand() const noexcept
    {
        return _currentControl;
    }

    CommandVector LoopController::RunApplicationModeTick(
        void* const context,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        return static_cast<IApplicationMode*>(context)->RunTick(loopEndTimeUs, state, loopController);
    }

    bool LoopController::IsBrakeCommand(const CommandVector& command) noexcept
    {
        return !std::isfinite(command.LeftCommand()) || !std::isfinite(command.RightCommand());
    }

    bool LoopController::IsZeroCommand(const CommandVector& command) noexcept
    {
        constexpr float kZeroMotorCommandThreshold = 1.0e-4f;

        return
            std::isfinite(command.LeftCommand()) &&
            std::isfinite(command.RightCommand()) &&
            (std::fabs(command.LeftCommand()) <= kZeroMotorCommandThreshold) &&
            (std::fabs(command.RightCommand()) <= kZeroMotorCommandThreshold);
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
        const uint8_t probePin = _runtime->Vehicle().FrontLeftWallSensor().GetWallSensorInPin();
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

        if (!ValidateSessionState(
                _stagedControlPeriodUs,
                _stagedSessionStartPointX,
                _stagedSessionStartPointY,
                _stagedSensorWorkBits))
        {
            _runtime->FailActiveMode("LoopController staged session state is invalid");
        }

        _controlPeriodUs = _stagedControlPeriodUs;
        _sessionStartPointX = _stagedSessionStartPointX;
        _sessionStartPointY = _stagedSessionStartPointY;
        _sensorWorkBits = _stagedSensorWorkBits;
        _stagedControlPeriodUs = 0U;
        _stagedSessionStartPointX = std::numeric_limits<float>::quiet_NaN();
        _stagedSessionStartPointY = std::numeric_limits<float>::quiet_NaN();
        _stagedSensorWorkBits = kDefaultSensorWorkBits;
        _stagedNextSessionValid = false;
        if (!std::isfinite(_modeStartHeadingRad))
        {
            _modeStartHeadingRad = _runtime->RuntimeState().GetHeading();
        }
        RestoreSessionStartPhysicalState();
        _sessionStartWallSensorAdcProbePending = RuntimeSensorSuite::SensorWorkBitsRequestWallSensors(_sensorWorkBits);
        RunSessionStartWallSensorAdcProbe();
        _activeModeWorkCallback = &RunApplicationModeTick;
        _activeModeWorkContext = _boundMode;
        _sessionActive = true;
        _publishedTimingValid = false;
        _tickCount = 0U;
        const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
        _lastTickStartUs = nowUs - _controlPeriodUs;
        _nextSyncTargetUs = nowUs + _controlPeriodUs;
        _nextControl = CommandVector::Brake();
        _currentControl = CommandVector::Brake();
        _publishedTimingIndex = 0U;
        _workingTimingIndex = 1U;
        _timingBuffers[0] = TimingBuffer{};
        _timingBuffers[1] = TimingBuffer{};
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

        if (!std::isfinite(_modeStartHeadingRad))
        {
            _runtime->FailActiveMode("LoopController session-start heading was unavailable");
        }

        _runtime->Vehicle().ResetDriveEncoders();
        if (!_runtime->Estimator().RestoreSessionStartPhysicalState(
                _sessionStartPointX,
                _sessionStartPointY,
                _modeStartHeadingRad))
        {
            _runtime->FailActiveMode("LoopController failed to restore the staged session-start physical state");
        }

        _runtime->DriveService().StartHold(0U, false);
    }

    void LoopController::Run()
    {
        if ((_runtime == nullptr) || (_boundMode == nullptr))
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
            TimingBuffer& timing = WorkingTiming();
            timing._cycleCounterStart = ReadCycleCounter();

            const std::uint32_t loopEndTimeUs = _nextSyncTargetUs;
            const CommandVector brakeControl = CommandVector::Brake();
            const char* terminalFaultReason = nullptr;

            if (_sessionStartWallSensorAdcProbePending)
            {
                RunSessionStartWallSensorAdcProbe();
            }

            ClearPendingRequests();

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                terminalFaultReason = "LoopController sensing update failed";
            }

            if (terminalFaultReason != nullptr)
            {
                _nextControl = brakeControl;
                if (!ApplyControlAtApplicationPoint(brakeControl))
                {
                    terminalFaultReason =
                        "LoopController motor command application failed during sensing fault handling";
                }
                timing._commandAppliedUs = static_cast<std::uint32_t>(micros());
            }
            else if (!ApplyControlAtApplicationPoint(_nextControl))
            {
                _nextControl = brakeControl;
                terminalFaultReason = "LoopController motor command application failed";
                timing._commandAppliedUs = static_cast<std::uint32_t>(micros());
            }
            else
            {
                timing._commandAppliedUs = static_cast<std::uint32_t>(micros());
            }

            CommandVector candidateControl = CommandVector::Brake();
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
                _nextControl = CommandVector::Brake();
            }
            else if (_programHaltRequested)
            {
                _nextControl = CommandVector::Brake();
            }
            else if (_pendingEndSessionCallback != nullptr)
            {
                _nextControl = CommandVector::Brake();
            }
            else if (_pendingPauseCallback != nullptr)
            {
                _nextControl = CommandVector::Brake();
            }
            else
            {
                _nextControl = candidateControl;
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
                    _nextControl = CommandVector::Brake();
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
            _nextSyncTargetUs += _controlPeriodUs;

            if (terminalFaultReason != nullptr)
            {
                _nextControl = brakeControl;
                if (!ApplyControlAtApplicationPoint(brakeControl))
                {
                    _runtime->FailActiveMode(
                        "LoopController motor command application failed during terminal fault handling");
                }
                _runtime->FailActiveMode(terminalFaultReason);
            }

            if (_programHaltRequested)
            {
                _nextControl = brakeControl;
                if (!ApplyControlAtApplicationPoint(brakeControl))
                {
                    _runtime->FailActiveMode(
                        "LoopController motor command application failed during program halt");
                }
                ResetExecutionState();
                return;
            }

            if (_pendingEndSessionCallback != nullptr)
            {
                ResolveEndSessionRequest();

                if (_programHaltRequested)
                {
                    _nextControl = brakeControl;
                    if (!ApplyControlAtApplicationPoint(brakeControl))
                    {
                        _runtime->FailActiveMode(
                            "LoopController motor command application failed during end-session-driven program halt");
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
                    _nextControl = brakeControl;
                    if (!ApplyControlAtApplicationPoint(brakeControl))
                    {
                        _runtime->FailActiveMode(
                            "LoopController motor command application failed during pause-driven program halt");
                    }
                    ResetExecutionState();
                    return;
                }

                if (_pendingEndSessionCallback != nullptr)
                {
                    ResolveEndSessionRequest();

                    if (_programHaltRequested)
                    {
                        _nextControl = brakeControl;
                        if (!ApplyControlAtApplicationPoint(brakeControl))
                        {
                            _runtime->FailActiveMode(
                                "LoopController motor command application failed during pause-and-end-session-driven program halt");
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

    bool LoopController::ValidateSessionState(
        const std::uint32_t controlPeriodUs,
        const float sessionStartPointX,
        const float sessionStartPointY,
        const std::uint8_t sensorWorkBits) const noexcept
    {
        return
            (controlPeriodUs > 0U) &&
            std::isfinite(sessionStartPointX) &&
            std::isfinite(sessionStartPointY) &&
            SupportsSensorWorkBits(sensorWorkBits);
    }

    bool LoopController::SupportsSensorWorkBits(const std::uint8_t sensorWorkBits) const noexcept
    {
        if (!RuntimeSensorSuite::SensorWorkBitsSupportWallUpdates(sensorWorkBits))
        {
            return false;
        }

        return true;
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

    bool LoopController::ApplyControlAtApplicationPoint(const CommandVector& control) noexcept
    {
        if (_runtime == nullptr)
        {
            return false;
        }

        _runtime->Vehicle().ApplyMotorCommand(control);
        _currentControl = control;
        _runtime->RuntimeState().SetCurrentCommand(_currentControl);
        return true;
    }

    bool LoopController::CaptureTickState(const float dtSeconds, const std::uint32_t tickStartUs)
    {
        if (_runtime == nullptr)
        {
            return false;
        }

        TimingBuffer& timing = WorkingTiming();
        const bool stationaryHint = ShouldTreatCurrentControlAsStationary();
        timing._encoderLatchUs = static_cast<std::uint32_t>(micros());
        SensorSnapshot snapshot{};
        const bool captureSensors = RuntimeSensorSuite::SensorWorkBitsRequestCapture(_sensorWorkBits);
        _runtime->Sensors().BeginInterlacedCapture(
            stationaryHint,
            _runtime->RuntimeState(),
            snapshot,
            _sensorWorkBits,
            dtSeconds);

        const CommandVector control = _currentControl;
        timing._encoderReadDoneUs = static_cast<std::uint32_t>(micros());

        MazeMap::Estimator& estimator = _runtime->Estimator();
        MazeMap::VehicleState& runtimeState = _runtime->RuntimeState();
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        }

        if (estimator.HasFault())
        {
            _runtime->Sensors().FinishInterlacedCapture();
            runtimeState.SetSensorSnapshot(snapshot);
            return false;
        }

        bool predictOk = true;
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            timing._estimatorPredictStartUs = static_cast<std::uint32_t>(micros());
            predictOk = estimator.predictWithInterleavedSensorService(
                dtSeconds,
                control,
                &snapshot.EncoderObservation(),
                snapshot.EncoderObservationValid(),
                captureSensors ? this : nullptr,
                captureSensors ? &LoopController::ServiceInterlacedSensorCapture : nullptr);
            if (!predictOk)
            {
                _runtime->Sensors().FinishInterlacedCapture();
                runtimeState.SetSensorSnapshot(snapshot);
                timing._estimatorPredictEndUs = static_cast<std::uint32_t>(micros());
                timing._estimatorPredictDurationUs =
                    timing._estimatorPredictEndUs - timing._estimatorPredictStartUs;
                timing._estimatorUpdateStartUs = timing._estimatorPredictEndUs;
                timing._estimatorUpdateEndUs = timing._estimatorPredictEndUs;
                timing._estimatorUpdateDurationUs = 0U;
                timing._estimatorTotalDurationUs = timing._estimatorPredictDurationUs;
                return false;
            }
        }
        else
        {
            timing._estimatorPredictStartUs = static_cast<std::uint32_t>(micros());
            timing._estimatorPredictEndUs = timing._estimatorPredictStartUs;
            timing._estimatorPredictDurationUs = 0U;
        }

        _runtime->Sensors().FinishInterlacedCapture();
        runtimeState.SetSensorSnapshot(snapshot);

        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            timing._estimatorPredictEndUs = static_cast<std::uint32_t>(micros());
            timing._estimatorPredictDurationUs =
                timing._estimatorPredictEndUs - timing._estimatorPredictStartUs;
        }
        if (!predictOk)
        {
            timing._estimatorUpdateStartUs = timing._estimatorPredictEndUs;
            timing._estimatorUpdateEndUs = timing._estimatorPredictEndUs;
            timing._estimatorUpdateDurationUs = 0U;
            timing._estimatorTotalDurationUs = timing._estimatorPredictDurationUs;
            return false;
        }

        timing._estimatorUpdateStartUs = static_cast<std::uint32_t>(micros());

        if (((_sensorWorkBits & (RuntimeSensorSuite::kGyroSensorBit | RuntimeSensorSuite::kAccelSensorBit)) != 0U) &&
            ((_sensorWorkBits & RuntimeSensorSuite::kGyroSensorBit) != 0U) &&
            std::isfinite(snapshot.RawYawRateRadps()))
        {
            if (!estimator.updateYawRate(snapshot.RawYawRateRadps()))
            {
                timing._estimatorUpdateEndUs = static_cast<std::uint32_t>(micros());
                timing._estimatorUpdateDurationUs =
                    timing._estimatorUpdateEndUs - timing._estimatorUpdateStartUs;
                timing._estimatorTotalDurationUs =
                    timing._estimatorPredictDurationUs + timing._estimatorUpdateDurationUs;
                return false;
            }
        }

        if (((_sensorWorkBits & (RuntimeSensorSuite::kGyroSensorBit | RuntimeSensorSuite::kAccelSensorBit)) != 0U) &&
            ((_sensorWorkBits & RuntimeSensorSuite::kAccelSensorBit) != 0U))
        {
            const bool accelObservationValid =
                snapshot.AccelerationBiasValid() &&
                std::isfinite(snapshot.BodyRightAccelerationMps2()) &&
                std::isfinite(snapshot.BodyForwardAccelerationMps2());
            const MazeMap::ImuAccelObs accelObservation(
                accelObservationValid,
                snapshot.BodyForwardAccelerationMps2(),
                snapshot.BodyRightAccelerationMps2());
            (void)estimator.updatePlanarAccel(accelObservation);
        }

        if ((_sensorWorkBits & RuntimeSensorSuite::kWallUpdateSensorBit) != 0U)
        {
            MazeMap::Maze& maze = _runtime->Maze();
            if ((_sensorWorkBits & static_cast<std::uint8_t>(WallMask::Front)) != 0U)
            {
                const MazeMap::WallObs& frontLeftPreprocessedObservation =
                    snapshot.FrontLeftWallSensorObservation();
                const MazeMap::WallObs& frontRightPreprocessedObservation =
                    snapshot.FrontRightWallSensorObservation();
                MazeMap::WallObs frontLeftObservation{};
                MazeMap::WallObs frontRightObservation{};
                MazeMap::WallObs::BuildFrontWallObservations(
                    snapshot.FrontWallObservationValid(),
                    snapshot.HasFrontWall(),
                    snapshot.FrontWallUsesFallbackDetection(),
                    snapshot.FrontWallUsesCharacterizationDetection(),
                    frontLeftPreprocessedObservation.IsValid() ?
                        frontLeftPreprocessedObservation.Rho() :
                        snapshot.FrontLeftDistanceM(),
                    frontRightPreprocessedObservation.IsValid() ?
                        frontRightPreprocessedObservation.Rho() :
                        snapshot.FrontRightDistanceM(),
                    MazeMap::kDefaultWallObservationMaxRangeM,
                    frontLeftObservation,
                    frontRightObservation,
                    frontLeftPreprocessedObservation.MeasurementNoiseSigmaM(),
                    frontRightPreprocessedObservation.MeasurementNoiseSigmaM(),
                    frontLeftPreprocessedObservation.IsValid() ?
                        frontLeftPreprocessedObservation.Confidence() :
                        -1.0f,
                    frontRightPreprocessedObservation.IsValid() ?
                        frontRightPreprocessedObservation.Confidence() :
                        -1.0f,
                    frontLeftPreprocessedObservation.IsValid() ?
                        frontLeftPreprocessedObservation.Class() :
                        MazeMap::ObsClass::WallLike,
                    frontRightPreprocessedObservation.IsValid() ?
                        frontRightPreprocessedObservation.Class() :
                        MazeMap::ObsClass::WallLike);
                (void)estimator.updateFrontPair(frontLeftObservation, frontRightObservation, maze, true);
            }

            if ((_sensorWorkBits & static_cast<std::uint8_t>(WallMask::Left)) != 0U)
            {
                const MazeMap::WallObs& sideLeftPreprocessedObservation =
                    snapshot.SideLeftWallSensorObservation();
                const MazeMap::WallObs leftObservation = MazeMap::WallObs::BuildSideWallObservation(
                    snapshot.LeftWallObservationWindowValid(),
                    snapshot.LeftTransitionDetected(),
                    snapshot.HasLeftWallObservation(),
                    sideLeftPreprocessedObservation.IsValid() ?
                        sideLeftPreprocessedObservation.Rho() :
                        snapshot.SideLeftDistanceM(),
                    MazeMap::kDefaultWallObservationMaxRangeM,
                    sideLeftPreprocessedObservation.MeasurementNoiseSigmaM(),
                    sideLeftPreprocessedObservation.IsValid() ?
                        sideLeftPreprocessedObservation.Confidence() :
                        0.80f,
                    sideLeftPreprocessedObservation.IsValid() ?
                        sideLeftPreprocessedObservation.Class() :
                        MazeMap::ObsClass::WallLike);
                (void)estimator.updateSideSensor(MazeMap::RelativeDirection::Left90, leftObservation, maze, true);
            }

            if ((_sensorWorkBits & static_cast<std::uint8_t>(WallMask::Right)) != 0U)
            {
                const MazeMap::WallObs& sideRightPreprocessedObservation =
                    snapshot.SideRightWallSensorObservation();
                const MazeMap::WallObs rightObservation = MazeMap::WallObs::BuildSideWallObservation(
                    snapshot.RightWallObservationWindowValid(),
                    snapshot.RightTransitionDetected(),
                    snapshot.HasRightWallObservation(),
                    sideRightPreprocessedObservation.IsValid() ?
                        sideRightPreprocessedObservation.Rho() :
                        snapshot.SideRightDistanceM(),
                    MazeMap::kDefaultWallObservationMaxRangeM,
                    sideRightPreprocessedObservation.MeasurementNoiseSigmaM(),
                    sideRightPreprocessedObservation.IsValid() ?
                        sideRightPreprocessedObservation.Confidence() :
                        0.80f,
                    sideRightPreprocessedObservation.IsValid() ?
                        sideRightPreprocessedObservation.Class() :
                        MazeMap::ObsClass::WallLike);
                (void)estimator.updateSideSensor(MazeMap::RelativeDirection::Right90, rightObservation, maze, true);
            }
        }

        timing._estimatorUpdateEndUs = static_cast<std::uint32_t>(micros());
        timing._estimatorUpdateDurationUs =
            timing._estimatorUpdateEndUs - timing._estimatorUpdateStartUs;
        timing._estimatorTotalDurationUs =
            timing._estimatorPredictDurationUs + timing._estimatorUpdateDurationUs;

        runtimeState.SetTimestampUs(tickStartUs);
        return true;
    }

    void LoopController::ResetWorkingTiming(
        const std::uint32_t sequence,
        const std::uint32_t tickStartUs,
        const std::uint32_t dtUs) noexcept
    {
        TimingBuffer& timing = WorkingTiming();
        timing = TimingBuffer{};
        timing._sequence = sequence;
        timing._tickStartUs = tickStartUs;
        timing._dtUs = dtUs;
    }

    LoopController::TimingBuffer& LoopController::WorkingTiming() noexcept
    {
        return _timingBuffers[_workingTimingIndex];
    }

    const LoopController::TimingBuffer& LoopController::PublishedTiming() const noexcept
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
        WorkingTiming()._callbackReturnUs = static_cast<std::uint32_t>(micros());
    }

    void LoopController::RecordPostServiceTiming() noexcept
    {
        WorkingTiming()._postServiceDoneUs = static_cast<std::uint32_t>(micros());
    }

    void LoopController::FinalizeTiming() noexcept
    {
        TimingBuffer& timing = WorkingTiming();
        const std::uint32_t finalizeUs = static_cast<std::uint32_t>(micros());
        constexpr std::uint16_t kTickTimingSaturatedUs = 0xFFFFU;

        timing._tickFinalizeUs = finalizeUs;
        timing._cycleCounterEnd = ReadCycleCounter();
        if (finalizeUs <= _nextSyncTargetUs)
        {
            timing._overrunUs = 0U;
        }
        else
        {
            timing._overrunUs = static_cast<std::uint16_t>((std::min)(
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

    bool LoopController::ShouldTreatCurrentControlAsStationary() const noexcept
    {
        return IsBrakeCommand(_currentControl) || IsZeroCommand(_currentControl);
    }

    void LoopController::ResolvePauseRequest()
    {
        WaitForBrakeSettlement();

        void (* const pauseCallback)(void*, LoopController&) = _pendingPauseCallback;
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

        _nextControl = CommandVector::Brake();
        _currentControl = CommandVector::Brake();
        const std::uint32_t nowUs = static_cast<std::uint32_t>(micros());
        _lastTickStartUs = nowUs - _controlPeriodUs;
        _nextSyncTargetUs = nowUs + _controlPeriodUs;
        ClearPendingRequests();
    }

    void LoopController::ResolveEndSessionRequest()
    {
        WaitForBrakeSettlement();

        void (* const endSessionCallback)(void*, LoopController&) = _pendingEndSessionCallback;
        void* const endSessionContext = _pendingEndSessionContext;
        ClearPendingRequests();
        _stagedControlPeriodUs = 0U;
        _stagedSessionStartPointX = std::numeric_limits<float>::quiet_NaN();
        _stagedSessionStartPointY = std::numeric_limits<float>::quiet_NaN();
        _stagedSensorWorkBits = kDefaultSensorWorkBits;
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

        _nextControl = CommandVector::Brake();
        _currentControl = CommandVector::Brake();
    }

    void LoopController::WaitForBrakeSettlement()
    {
        constexpr float kPauseLinearThresholdMps = 0.01f;
        constexpr float kPauseAngularThresholdRadps = 0.05f;
        constexpr std::uint8_t kPauseSettledTicks = 2U;

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
            const std::uint32_t deadlineUs = tickStartUs + _controlPeriodUs;
            _lastTickStartUs = tickStartUs;
            _nextSyncTargetUs = deadlineUs;
            ++_tickCount;

            ResetWorkingTiming(_tickCount, tickStartUs, dtUs);
            TimingBuffer& timing = WorkingTiming();
            timing._cycleCounterStart = ReadCycleCounter();

            if (!CaptureTickState(dtSeconds, tickStartUs))
            {
                ServiceRuntimeLogsForFaultPath();
                _runtime->FailActiveMode("LoopController brake settlement capture failed");
            }

            _nextControl = CommandVector::Brake();
            if (!ApplyControlAtApplicationPoint(_nextControl))
            {
                ServiceRuntimeLogsForFaultPath();
                _runtime->FailActiveMode(
                    "LoopController motor command application failed during brake settlement");
            }
            timing._commandAppliedUs = static_cast<std::uint32_t>(micros());

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
            const float linearSpeedMps = runtimeState.GetForwardVelocity();
            const float angularSpeedRadps = runtimeState.GetYawRate();
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
        _controlPeriodUs = 0U;
        _sessionStartPointX = std::numeric_limits<float>::quiet_NaN();
        _sessionStartPointY = std::numeric_limits<float>::quiet_NaN();
        _sensorWorkBits = kDefaultSensorWorkBits;
        _modeStartHeadingRad = std::numeric_limits<float>::quiet_NaN();
        _activeModeWorkCallback = nullptr;
        _activeModeWorkContext = nullptr;
        _sessionActive = false;
        _publishedTimingValid = false;
        _tickCount = 0U;
        _lastTickStartUs = 0U;
        _nextSyncTargetUs = 0U;
        _nextControl = CommandVector::Brake();
        _currentControl = CommandVector::Brake();
        _sessionStartWallSensorAdcProbePending = false;
        _timingBuffers[0] = TimingBuffer{};
        _timingBuffers[1] = TimingBuffer{};
        _publishedTimingIndex = 0U;
        _workingTimingIndex = 1U;
        ClearPendingRequests();
        _boundMode = nullptr;
        _stagedControlPeriodUs = 0U;
        _stagedSessionStartPointX = std::numeric_limits<float>::quiet_NaN();
        _stagedSessionStartPointY = std::numeric_limits<float>::quiet_NaN();
        _stagedSensorWorkBits = kDefaultSensorWorkBits;
        _stagedNextSessionValid = false;
    }
}

