#include "pch.h"
#include "ManeuverExecutor.h"

#include "DriveBase.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "MazeMapSharedRuntime.h"
#include "MotionTargetProjection.h"
#include "TurnWallEdgeTracker.h"

#include <algorithm>
#include <cmath>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr MazeMap::CommandPD kManeuverTrackingCommandPd =
            MazeMap::CommandPD::StateWheelOmegaPD |
            MazeMap::CommandPD::IMUYaw;

        float ManeuverDistanceMeters(const MazeMap::ManeuverCode code)
        {
            return MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, Config::kCellSizeM);
        }
    }

    LoopController::ControlVector ManeuverExecutor::ActiveRoutineThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<ManeuverExecutor*>(context);
        if ((self == nullptr) || (self->_activeState == nullptr) || (self->_activePhaseTick == nullptr))
        {
            services.Fault("ManeuverExecutor callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_activePhaseTick)(self->_activeState, loopEndTimeUs, state, services);
    }

    void ManeuverExecutor::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _drive = &runtime.Drive();
        _speedVehicle = &runtime.SpeedVehicle();
        _maze = &runtime.Maze();
    }

    bool ManeuverExecutor::Active() const noexcept
    {
        return (_activeState != nullptr) && (_activePhaseTick != nullptr);
    }

    void ManeuverExecutor::CancelActivePhase() noexcept
    {
        ResetActiveRoutine();
    }

    bool ManeuverExecutor::CanBeginPhase() const noexcept
    {
        return
            (_runtime != nullptr) &&
            (_drive != nullptr) &&
            (_speedVehicle != nullptr) &&
            (_maze != nullptr) &&
            !Active();
    }

    bool ManeuverExecutor::BeginPhase(
        void* const activeState,
        const ActivePhaseTickFn activePhaseTick) noexcept
    {
        if (!CanBeginPhase() || (activeState == nullptr) || (activePhaseTick == nullptr))
        {
            return false;
        }

        _returnCallbacks = LoopController::ModeCallbacks{};
        ActivatePhase(activeState, activePhaseTick);
        return true;
    }

    bool ManeuverExecutor::BuildRoutineCallbacks(LoopController::ModeCallbacks& callbacks) const noexcept
    {
        if ((_activeState == nullptr) || (_activePhaseTick == nullptr))
        {
            callbacks = {};
            return false;
        }

        callbacks = {};
        callbacks.onModeWork = &ManeuverExecutor::ActiveRoutineThunk;
        callbacks.context = const_cast<ManeuverExecutor*>(this);
        return true;
    }

    bool ManeuverExecutor::InstallRoutineCallbacks(
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services)
    {
        if ((returnCallbacks.onModeWork == nullptr) ||
            (_activeState == nullptr) ||
            (_activePhaseTick == nullptr))
        {
            ResetActiveRoutine();
            return false;
        }

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &ManeuverExecutor::ActiveRoutineThunk;
        callbacks.context = this;
        services.SetNextModeWorkCallbacks(callbacks);
        _returnCallbacks = returnCallbacks;
        return true;
    }

    void ManeuverExecutor::ActivatePhase(
        void* const activeState,
        const ActivePhaseTickFn activePhaseTick) noexcept
    {
        _activeState = activeState;
        _activePhaseTick = activePhaseTick;
    }

    void ManeuverExecutor::ResetActiveRoutine() noexcept
    {
        _activeState = nullptr;
        _activePhaseTick = nullptr;
        _returnCallbacks = LoopController::ModeCallbacks{};
        _holdState = HoldRoutineState{};
        _settleState = SettleRoutineState{};
        _straightState = StraightRoutineState{};
        _turnState = TurnRoutineState{};
        _arcState = ArcRoutineState{};
        _smoothTurnState = SmoothTurnRoutineState{};
        _queueState = QueueRoutineState{};
    }

    LoopController::ControlVector ManeuverExecutor::ReturnToContinuation(
        LoopController::TickServices& services) noexcept
    {
        const LoopController::ModeCallbacks callbacks = _returnCallbacks;
        ResetActiveRoutine();
        if (callbacks.onModeWork != nullptr)
        {
            services.SetNextModeWorkCallbacks(callbacks);
        }
        else
        {
            services.RequestEndLoop();
        }
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector ManeuverExecutor::CompleteCurrentPhase(
        void* const nextState,
        const ActivePhaseTickFn nextPhaseTick,
        LoopController::TickServices& services) noexcept
    {
        if ((nextState != nullptr) && (nextPhaseTick != nullptr))
        {
            ActivatePhase(nextState, nextPhaseTick);
            return LoopController::ControlVector::Brake;
        }

        return ReturnToContinuation(services);
    }

    LoopController::ControlVector ManeuverExecutor::FaultPhase(
        LoopController::TickServices& services,
        const char* const reason) noexcept
    {
        ResetActiveRoutine();
        services.Fault(reason);
        return LoopController::ControlVector::Brake;
    }

    bool ManeuverExecutor::SEND_IT(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        const bool snapToExpectedLocation,
        MazeMap::DirectionalLocation& currentLocation,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::TickServices& services)
    {
        if (returnCallbacks.onModeWork == nullptr)
        {
            return false;
        }

        if (queue.empty())
        {
            services.SetNextModeWorkCallbacks(returnCallbacks);
            return true;
        }

        currentLocation = queue.front().getStart();
        _queueState = QueueRoutineState{};
        _queueState.queue = &queue;
        _queueState.currentLocation = &currentLocation;
        _queueState.limits = limits;
        _queueState.snapToExpectedLocation = snapToExpectedLocation;
        return
            BeginPhase(&_queueState, &ManeuverExecutor::QueueDispatchRoutineTick) &&
            InstallRoutineCallbacks(returnCallbacks, services);
    }

    bool ManeuverExecutor::SEND_IT(
        MazeMap::ManeuverQueue& queue,
        const MotionLimits& limits,
        const bool snapToExpectedLocation,
        MazeMap::DirectionalLocation& currentLocation,
        const LoopController::ModeCallbacks& returnCallbacks,
        LoopController::ModeCallbacks& initialCallbacks)
    {
        initialCallbacks = {};
        if (returnCallbacks.onModeWork == nullptr)
        {
            return false;
        }

        if (queue.empty())
        {
            initialCallbacks = returnCallbacks;
            return true;
        }

        currentLocation = queue.front().getStart();
        _queueState = QueueRoutineState{};
        _queueState.queue = &queue;
        _queueState.currentLocation = &currentLocation;
        _queueState.limits = limits;
        _queueState.snapToExpectedLocation = snapToExpectedLocation;
        if (!BeginPhase(&_queueState, &ManeuverExecutor::QueueDispatchRoutineTick) ||
            !BuildRoutineCallbacks(initialCallbacks))
        {
            ResetActiveRoutine();
            initialCallbacks = {};
            return false;
        }

        _returnCallbacks = returnCallbacks;
        return true;
    }

    bool ManeuverExecutor::IsDriveMotionSettled(
        const DriveTelemetry& stationaryReferenceTelemetry,
        const unsigned long stationaryReferenceMs,
        const DriveTelemetry& telemetry,
        const SensorSnapshot& snapshot,
        const unsigned long nowMs) const
    {
        const unsigned long elapsedMs = nowMs - stationaryReferenceMs;
        return MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            telemetry.leftDistanceM - stationaryReferenceTelemetry.leftDistanceM,
            telemetry.rightDistanceM - stationaryReferenceTelemetry.rightDistanceM,
            static_cast<float>(elapsedMs) * 1.0e-3f,
            snapshot.gyroRadps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    float ManeuverExecutor::ManeuverSpeedLimit(
        const MazeMap::ManeuverCode code,
        const MotionLimits& limits,
        const MazeMap::Vehicle& vehicle)
    {
        const MazeMap::ManeuverInstance maneuver(code, MazeMap::DirectionalLocation());
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (IsStraightCode(code))
        {
            return limits.maxSpeedMps;
        }
        return (std::min)(limits.maxSpeedMps, maneuver.GetSpeedLimit(vehicle));
    }

    LoopController::ControlVector ManeuverExecutor::HoldRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& hold = *static_cast<HoldRoutineState*>(rawState);
        if (!hold.started)
        {
            hold.started = true;
            hold.startMs = millis();
        }

        if (static_cast<unsigned long>(millis() - hold.startMs) < hold.durationMs)
        {
            return LoopController::ControlVector::Brake;
        }

        return ReturnToContinuation(services);
    }

    LoopController::ControlVector ManeuverExecutor::SettleRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& settle = *static_cast<SettleRoutineState*>(rawState);
        if (!settle.started)
        {
            settle.started = true;
            settle.startMs = millis();
        }

        const unsigned long nowMs = millis();
        if (!settle.stationaryWindowActive)
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
            settle.stationaryWindowActive = true;
        }
        else if (!IsDriveMotionSettled(
                     settle.stationaryStartTelemetry,
                     settle.stationaryStartMs,
                     state.driveTelemetry,
                     state.sensors,
                     nowMs))
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
        }
        else if ((nowMs - settle.stationaryStartMs) >= settle.stationaryHoldMs)
        {
            return CompleteCurrentPhase(settle.nextState, settle.nextPhaseTick, services);
        }

        if ((settle.timeoutMs > 0U) && ((nowMs - settle.startMs) >= settle.timeoutMs))
        {
            return FaultPhase(
                services,
                (settle.timeoutMessage != nullptr) ? settle.timeoutMessage : "Drive settle timed out");
        }

        return settle.brakeCommand ?
            LoopController::ControlVector::Brake :
            _drive->PointControlVector(0.0f, 0.0f, kManeuverTrackingCommandPd);
    }

    LoopController::ControlVector ManeuverExecutor::StraightRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& straight = *static_cast<StraightRoutineState*>(rawState);

        const float traveledM = std::fabs(_drive->GetAverageDistanceMeters() - straight.startDistanceM);
        float remainingM = (std::max)(0.0f, straight.distanceM - traveledM);
        if (straight.targetPositionOverride != nullptr)
        {
            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    state.estimate.xMeters,
                    state.estimate.yMeters,
                    straight.targetPositionOverride->x(),
                    straight.targetPositionOverride->y(),
                    straight.targetHeading.x(),
                    straight.targetHeading.y(),
                    projectedRemainingM))
            {
                return FaultPhase(services, "Straight target projection is invalid");
            }
            remainingM = (std::max)(0.0f, projectedRemainingM);
        }

        const bool stoppingAtEndpoint = straight.exitSpeed <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            straight.completionSettle = SettleRoutineState{};
            straight.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            straight.completionSettle.timeoutMs = 0U;
            straight.completionSettle.brakeCommand = true;
            straight.completionSettle.nextState = straight.nextState;
            straight.completionSettle.nextPhaseTick = straight.nextPhaseTick;
            return CompleteCurrentPhase(
                &straight.completionSettle,
                &ManeuverExecutor::SettleRoutineTick,
                services);
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - straight.exitSpeed) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return CompleteCurrentPhase(straight.nextState, straight.nextPhaseTick, services);
        }

        const float accelLimitedSpeedMps = (std::min)(
            straight.cruiseSpeed,
            straight.commandedSpeedMps + (straight.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(straight.exitSpeed, remainingM, straight.limits.decelMps2);
        straight.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        float wallOmegaRadps = 0.0f;
        if (straight.useWallCentering)
        {
            if (straight.diagonalHeading)
            {
                wallOmegaRadps += ComputeDiagonalWallCenterOmegaRadps(
                    gWallDistanceCalibration,
                    state.sensors.sideLeftDifferentialLight,
                    state.sensors.sideRightDifferentialLight);
            }
            else if ((straight.currentLocation != nullptr))
            {
                float signalCorridorErrorM = 0.0f;
                if (Runtime::TryComputeWallGroundedCorridorErrorM(
                        *_maze,
                        *_speedVehicle,
                        *straight.currentLocation,
                        state.estimate,
                        state.sensors,
                        signalCorridorErrorM))
                {
                    wallOmegaRadps += Runtime::ComputeWallCenterPdOmegaRadps(
                        signalCorridorErrorM,
                        straight.commandedSpeedMps,
                        state.dtSeconds,
                        straight.previousCorridorErrorM,
                        straight.filteredCorridorErrorRateMps,
                        straight.previousCorridorErrorValid);
                }
                else
                {
                    straight.filteredCorridorErrorRateMps = 0.0f;
                    straight.previousCorridorErrorValid = false;
                }

                if (stoppingAtEndpoint &&
                    std::isfinite(state.sensors.frontLeftDistanceM) &&
                    std::isfinite(state.sensors.frontRightDistanceM) &&
                    (state.sensors.frontLeftDistanceM < Config::kFrontWallOnThresholdM) &&
                    (state.sensors.frontRightDistanceM < Config::kFrontWallOnThresholdM) &&
                    (remainingM < 0.07f))
                {
                    wallOmegaRadps += Config::kFrontSkewGain * state.sensors.frontSkewM;
                }
            }
        }
        else
        {
            straight.filteredCorridorErrorRateMps = 0.0f;
            straight.previousCorridorErrorValid = false;
        }

        const float headingErrorRad = HeadingErrorRad(straight.targetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) +
            wallOmegaRadps;
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -straight.limits.maxAngularSpeedRadps,
            straight.limits.maxAngularSpeedRadps);
        return _drive->PointControlVector(
            straight.commandedSpeedMps,
            angularCommandRadps,
            kManeuverTrackingCommandPd);
    }

    LoopController::ControlVector ManeuverExecutor::TurnRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& turn = *static_cast<TurnRoutineState*>(rawState);
        if (turn.wallEdgeTracker != nullptr)
        {
            MazeMap::ObserveTurnWallStates(*turn.wallEdgeTracker, state.sensors.leftWall, state.sensors.rightWall);
        }

        const float errorRad = AngleErrorRad(turn.targetYawRad, state.estimate.yawRad);
        if (MazeMap::IsInPlaceTurnComplete(errorRad, state.estimate.angularSpeedRadps, turn.turnProfile))
        {
            turn.completionSettle = SettleRoutineState{};
            turn.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            turn.completionSettle.timeoutMs = 0U;
            turn.completionSettle.brakeCommand = false;
            turn.completionSettle.nextState = turn.nextState;
            turn.completionSettle.nextPhaseTick = turn.nextPhaseTick;
            return CompleteCurrentPhase(
                &turn.completionSettle,
                &ManeuverExecutor::SettleRoutineTick,
                services);
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                errorRad,
                state.estimate.angularSpeedRadps,
                turn.turnProfile,
                angularCommandRadps))
        {
            return FaultPhase(services, "Turn profile became invalid");
        }

        return _drive->PointControlVector(
            0.0f,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    LoopController::ControlVector ManeuverExecutor::ArcRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& arc = *static_cast<ArcRoutineState*>(rawState);

        const float traveledM = std::fabs(_drive->GetAverageDistanceMeters() - arc.startDistanceM);
        const float remainingM = (std::max)(0.0f, arc.distanceM - traveledM);
        const bool stoppingAtEndpoint = arc.exitSpeed <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            arc.completionSettle = SettleRoutineState{};
            arc.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            arc.completionSettle.timeoutMs = 0U;
            arc.completionSettle.brakeCommand = true;
            arc.completionSettle.nextState = arc.nextState;
            arc.completionSettle.nextPhaseTick = arc.nextPhaseTick;
            return CompleteCurrentPhase(
                &arc.completionSettle,
                &ManeuverExecutor::SettleRoutineTick,
                services);
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - arc.exitSpeed) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return CompleteCurrentPhase(arc.nextState, arc.nextPhaseTick, services);
        }

        const float accelLimitedSpeedMps = (std::min)(
            arc.cruiseSpeed,
            arc.commandedSpeedMps + (arc.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(arc.exitSpeed, remainingM, arc.limits.decelMps2);
        arc.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float progress = (std::clamp)(traveledM / arc.distanceM, 0.0f, 1.0f);
        const float targetYawRad = WrapAngleRad(arc.startYawRad + (arc.angleRad * progress));
        const float headingErrorRad = AngleErrorRad(targetYawRad, state.estimate.yawRad);
        float angularCommandRadps =
            (arc.curvature * arc.commandedSpeedMps) +
            (Config::kArcHeadingKp * headingErrorRad);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -arc.limits.maxAngularSpeedRadps,
            arc.limits.maxAngularSpeedRadps);
        return _drive->PointControlVector(
            arc.commandedSpeedMps,
            angularCommandRadps,
            kManeuverTrackingCommandPd);
    }

    LoopController::ControlVector ManeuverExecutor::SmoothTurnRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& smoothTurn = *static_cast<SmoothTurnRoutineState*>(rawState);

        const float traveledM = std::fabs(_drive->GetAverageDistanceMeters() - smoothTurn.startDistanceM);
        const float remainingM = (std::max)(0.0f, smoothTurn.totalDistanceM - traveledM);
        if (remainingM <= Config::kDistanceToleranceM)
        {
            return CompleteCurrentPhase(smoothTurn.nextState, smoothTurn.nextPhaseTick, services);
        }

        MazeMap::ManeuverPoint point{};
        if (!smoothTurn.maneuver.TryGetManeuverPoint(
                traveledM,
                smoothTurn.maneuverSpeedMps,
                point,
                Config::kCellSizeM))
        {
            return FaultPhase(services, "Maneuver point became invalid");
        }

        point.Omega = (std::clamp)(
            point.Omega,
            -smoothTurn.limits.maxAngularSpeedRadps,
            smoothTurn.limits.maxAngularSpeedRadps);
        return _drive->PointControlVector(point, kManeuverTrackingCommandPd);
    }

    LoopController::ControlVector ManeuverExecutor::QueueDispatchRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) || (queueState.currentLocation == nullptr))
        {
            return FaultPhase(services, "Queued maneuver routine state was invalid");
        }
        if (queueState.nextIndex >= queueState.queue->size())
        {
            return ReturnToContinuation(services);
        }

        queueState.activeIndex = queueState.nextIndex;
        const MazeMap::ManeuverInstance& entry = (*queueState.queue)[queueState.activeIndex];
        *queueState.currentLocation = entry.getStart();

        const MazeMap::ManeuverCode code = entry.getCode();
        if (IsStraightCode(code))
        {
            _straightState = StraightRoutineState{};
            _straightState.distanceM =
                0.5f * Config::kCellSizeM * static_cast<float>(static_cast<std::uint8_t>(code));
            _straightState.entrySpeed = entry.getEntrySpeed();
            _straightState.cruiseSpeed = queueState.limits.maxSpeedMps;
            _straightState.exitSpeed = entry.getExitSpeed();
            _straightState.limits = queueState.limits;
            _straightState.useWallCentering = true;
            _straightState.currentLocation = queueState.currentLocation;
            _straightState.targetHeading = _drive->GetPose().headingUnit;
            _straightState.diagonalHeading = IsApproximatelyDiagonalHeadingUnit(_straightState.targetHeading);
            _straightState.commandedSpeedMps = (std::max)(_straightState.entrySpeed, 0.0f);
            _straightState.startDistanceM = _drive->GetAverageDistanceMeters();
            _straightState.nextState = &queueState;
            _straightState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
            return CompleteCurrentPhase(&_straightState, &ManeuverExecutor::StraightRoutineTick, services);
        }

        const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        if (entry.SupportsPointTracking())
        {
            _smoothTurnState = SmoothTurnRoutineState{};
            _smoothTurnState.maneuver = entry;
            _smoothTurnState.limits = queueState.limits;
            _smoothTurnState.maneuverSpeedMps = ManeuverSpeedLimit(code, queueState.limits, *_speedVehicle);
            if (!(_smoothTurnState.maneuverSpeedMps > 0.0f))
            {
                _smoothTurnState.maneuverSpeedMps =
                    (std::max)(entry.getEntrySpeed(), entry.getExitSpeed());
            }
            _smoothTurnState.totalDistanceM =
                _smoothTurnState.maneuver.GetTravelDistanceMeters(Config::kCellSizeM);
            _smoothTurnState.startDistanceM = _drive->GetAverageDistanceMeters();
            _smoothTurnState.nextState = &queueState;
            _smoothTurnState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
            _smoothTurnState.translationWatchdog.Reset(0.0f, millis());
            return CompleteCurrentPhase(&_smoothTurnState, &ManeuverExecutor::SmoothTurnRoutineTick, services);
        }

        const float distanceM = entry.GetTravelDistanceMeters(Config::kCellSizeM);
        if (distanceM <= 0.0f)
        {
            _turnState = TurnRoutineState{};
            _turnState.targetYawRad = WrapAngleRad(_drive->GetPose().yawRad + angleRad);
            _turnState.turnProfile = BuildSharedInPlaceTurnProfile(queueState.limits);
            _turnState.nextState = &queueState;
            _turnState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
            return CompleteCurrentPhase(&_turnState, &ManeuverExecutor::TurnRoutineTick, services);
        }

        _arcState = ArcRoutineState{};
        _arcState.distanceM = distanceM;
        _arcState.angleRad = angleRad;
        _arcState.exitSpeed = entry.getExitSpeed();
        _arcState.cruiseSpeed = ManeuverSpeedLimit(code, queueState.limits, *_speedVehicle);
        _arcState.limits = queueState.limits;
        _arcState.startDistanceM = _drive->GetAverageDistanceMeters();
        _arcState.startYawRad = _drive->GetPose().yawRad;
        _arcState.curvature = angleRad / distanceM;
        _arcState.commandedSpeedMps = (std::max)(entry.getEntrySpeed(), 0.0f);
        _arcState.translationWatchdog.Reset(0.0f, millis());
        _arcState.nextState = &queueState;
        _arcState.nextPhaseTick = &ManeuverExecutor::QueueAdvanceRoutineTick;
        return CompleteCurrentPhase(&_arcState, &ManeuverExecutor::ArcRoutineTick, services);
    }

    LoopController::ControlVector ManeuverExecutor::QueueAdvanceRoutineTick(
        void* const rawState,
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        auto& queueState = *static_cast<QueueRoutineState*>(rawState);
        if ((queueState.queue == nullptr) ||
            (queueState.currentLocation == nullptr) ||
            (queueState.activeIndex >= queueState.queue->size()))
        {
            return FaultPhase(services, "Queued maneuver advance state was invalid");
        }

        const MazeMap::ManeuverInstance& entry = (*queueState.queue)[queueState.activeIndex];
        *queueState.currentLocation = entry.getEnd();

        if (queueState.snapToExpectedLocation)
        {
            _drive->SetStartPoint(*queueState.currentLocation);
        }

        queueState.nextIndex = static_cast<std::uint16_t>(queueState.activeIndex + 1U);
        if (queueState.nextIndex >= queueState.queue->size())
        {
            return ReturnToContinuation(services);
        }

        return CompleteCurrentPhase(&queueState, &ManeuverExecutor::QueueDispatchRoutineTick, services);
    }
}


