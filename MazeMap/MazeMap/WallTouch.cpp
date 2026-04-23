#include "pch.h"
#include "WallTouch.h"

#include "Drive.h"
#include "DriveBase.h"
#include "SharedRobotRuntime.h"
#include "MissionStartPolicy.h"
#include "MotionTargetProjection.h"

#include <algorithm>
#include <cmath>

namespace MazeMap::App::Internal
{
    WallTouch::WallTouch()
        : _trackingCommandPd(Config::kWallTouchTrackingCommandPd)
    {
    }

    void WallTouch::SetLimits(const MotionLimits& limits) noexcept
    {
        _limits = limits;
    }

    void WallTouch::SetAllowPassThroughNoWall(const bool allowPassThroughNoWall) noexcept
    {
        _allowPassThroughNoWall = allowPassThroughNoWall;
    }

    bool WallTouch::Active() const noexcept
    {
        return _activePhase != ActivePhase::None;
    }

    void WallTouch::Cancel() noexcept
    {
        if (_driveService != nullptr)
        {
            _driveService->Cancel();
        }

        ResetActivePhase();
        _faulted = false;
    }

    void WallTouch::Start(
        const MazeMap::CellCoordinates& wallCell,
        const MazeMap::Direction wallDirection)
    {
        if (!CanStart())
        {
            return;
        }

        float contactCoordinateM = 0.0f;
        CalibrationWall ignoredWall = CalibrationWall::West;
        if (!TryComputeWallTouchTargetCoordinateForCellWall(
                wallCell,
                wallDirection,
                contactCoordinateM,
                ignoredWall))
        {
            return;
        }

        const PoseEstimate& pose = _drive->GetPose();
        const float preferredXMeters = (static_cast<float>(wallCell.GetX()) + 0.5f) * Config::kCellSizeM;
        const float preferredYMeters = (static_cast<float>(wallCell.GetY()) + 0.5f) * Config::kCellSizeM;
        float contactXMeters = preferredXMeters;
        float contactYMeters = preferredYMeters;
        float expectedTravelM = 0.0f;
        switch (wallDirection)
        {
        case MazeMap::Left:
            contactXMeters = contactCoordinateM;
            expectedTravelM = pose.xMeters - contactCoordinateM;
            break;
        case MazeMap::Right:
            contactXMeters = contactCoordinateM;
            expectedTravelM = contactCoordinateM - pose.xMeters;
            break;
        case MazeMap::Down:
            contactYMeters = contactCoordinateM;
            expectedTravelM = pose.yMeters - contactCoordinateM;
            break;
        case MazeMap::Up:
            contactYMeters = contactCoordinateM;
            expectedTravelM = contactCoordinateM - pose.yMeters;
            break;
        default:
            return;
        }

        expectedTravelM = (std::max)(0.0f, expectedTravelM);
        float minLatchTravelM = Config::kWallTouchMinApproachDistanceM;
        if (expectedTravelM > Config::kWallTouchExpectedTravelSlackM)
        {
            minLatchTravelM = (std::max)(
                minLatchTravelM,
                expectedTravelM - Config::kWallTouchExpectedTravelSlackM);
        }
        if (expectedTravelM > 0.0f)
        {
            minLatchTravelM = (std::min)(minLatchTravelM, expectedTravelM);
        }

        float maxApproachTravelM = Config::kWallTouchBaseMaxApproachDistanceM;
        if (expectedTravelM > 0.0f)
        {
            maxApproachTravelM = (std::max)(
                maxApproachTravelM,
                expectedTravelM + Config::kWallTouchExpectedTravelSlackM);
        }

        const float targetYawRad = DirectionToYawRad(wallDirection);
        if (!std::isfinite(targetYawRad) ||
            !std::isfinite(minLatchTravelM) ||
            !std::isfinite(maxApproachTravelM) ||
            !(maxApproachTravelM > 0.0f))
        {
            return;
        }

        if (_driveService != nullptr)
        {
            _driveService->Cancel();
        }

        ResetActivePhase();
        _faulted = false;

        _state.wallDirection = wallDirection;
        _state.preferredXMeters = preferredXMeters;
        _state.preferredYMeters = preferredYMeters;
        _state.contactXMeters = contactXMeters;
        _state.contactYMeters = contactYMeters;
        _state.targetYawRad = targetYawRad;
        _state.startDistanceM = _drive->GetAverageDistanceMeters();
        _state.lastProgressDistanceM = _state.startDistanceM;
        _state.minLatchTravelM = minLatchTravelM;
        _state.maxApproachTravelM = maxApproachTravelM;
        _state.touchStartMs = millis();
        _state.phaseStartMs = _state.touchStartMs;
        _state.lastMotionMs = _state.touchStartMs;
        _activePhase = ActivePhase::Seek;
    }

    LoopController::ControlVector WallTouch::GetNextControls(bool& done)
    {
        done = false;
        if (_faulted)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (_activePhase == ActivePhase::None)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        const LoopController::ModeState* const state = TryGetLoopState();
        if (state == nullptr)
        {
            SetFault("WallTouch requires an active LoopController mode state");
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (!state->estimatorHealthy)
        {
            SetFault((state->faultReason != nullptr) ? state->faultReason : "WallTouch estimator unhealthy");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        LoopController::ControlVector control = LoopController::ControlVector::Brake;
        switch (_activePhase)
        {
        case ActivePhase::Seek:
            control = SeekControls(*state, done);
            break;
        case ActivePhase::Seat:
            control = SeatControls(*state, done);
            break;
        case ActivePhase::Square:
            control = SquareControls(*state, done);
            break;
        case ActivePhase::HoldBeforeReturn:
            control = HoldBeforeReturnControls(*state, done);
            break;
        case ActivePhase::ReturnToPreferred:
            control = ReturnToPreferredControls(done);
            break;
        default:
            done = true;
            break;
        }

        if (done)
        {
            ResetActivePhase();
        }
        return control;
    }

    void WallTouch::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _loopController = &runtime.ControlLoop();
        _drive = &runtime.Drive();
        _driveService = &runtime.DriveService();
        _limits.maxSpeedMps = runtime.SpeedVehicle().GetMaxSpeed();
        _limits.accelMps2 = runtime.SpeedVehicle().GetMaxForwardAcceleration();
        _limits.decelMps2 = runtime.SpeedVehicle().GetMaxForwardAcceleration();
        _limits.maxAngularSpeedRadps = runtime.SpeedVehicle().GetMaxRotationalVelocity();
        _limits.angularAccelRadps2 = runtime.SpeedVehicle().GetMaxAngularAcceleration();
    }

    bool WallTouch::CanStart() const noexcept
    {
        return (_runtime != nullptr) &&
            (_loopController != nullptr) &&
            (_drive != nullptr) &&
            (_driveService != nullptr);
    }

    void WallTouch::ResetActivePhase() noexcept
    {
        _activePhase = ActivePhase::None;
        _state = {};
    }

    void WallTouch::SetFault(const char* reason) noexcept
    {
        _faulted = true;
        if (_driveService != nullptr)
        {
            _driveService->Cancel();
        }

        ResetActivePhase();
        if (_runtime != nullptr)
        {
            (void)_runtime->FailActiveMode((reason != nullptr) ? reason : "WallTouch fault");
        }
    }

    const LoopController::ModeState* WallTouch::TryGetLoopState() const noexcept
    {
        return (_loopController != nullptr) ? _loopController->CurrentModeState() : nullptr;
    }

    bool WallTouch::BeginHoldBeforeReturn(const bool resetPoseDuringHold) noexcept
    {
        _state.phaseStartMs = millis();
        _state.frontSignalMissingStartMs = 0UL;
        _state.squareStableStartMs = 0UL;
        _state.resetPoseDuringHold = resetPoseDuringHold;
        _state.poseResetApplied = false;
        _activePhase = ActivePhase::HoldBeforeReturn;
        return true;
    }

    bool WallTouch::BeginReturnToPreferred() noexcept
    {
        if ((_drive == nullptr) || (_driveService == nullptr))
        {
            return false;
        }

        const PoseEstimate& pose = _drive->GetPose();
        const Eigen::Vector2f heading = DirectionToUnitVector(_state.wallDirection);
        float projectedDistanceM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                pose.xMeters,
                pose.yMeters,
                _state.preferredXMeters,
                _state.preferredYMeters,
                heading.x(),
                heading.y(),
                projectedDistanceM))
        {
            return false;
        }

        const float returnDistanceM = std::fabs(projectedDistanceM);
        if (returnDistanceM <= Config::kDistanceToleranceM)
        {
            _activePhase = ActivePhase::None;
            return true;
        }

        float returnCruiseSpeedMps = Config::kWallTouchReverseSpeedMps;
        if (std::isfinite(_limits.maxSpeedMps) && (_limits.maxSpeedMps > 0.0f))
        {
            returnCruiseSpeedMps = (std::min)(returnCruiseSpeedMps, _limits.maxSpeedMps);
        }
        if (!(returnCruiseSpeedMps > 0.0f))
        {
            return false;
        }

        const Eigen::Vector2f targetPosition(_state.preferredXMeters, _state.preferredYMeters);
        _driveService->Cancel();
        _driveService->SetLimits(_limits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartStraight(
            returnDistanceM,
            -returnCruiseSpeedMps,
            0.0f,
            &heading,
            &targetPosition);
        if (!_driveService->Active())
        {
            return false;
        }

        _activePhase = ActivePhase::ReturnToPreferred;
        return true;
    }

    LoopController::ControlVector WallTouch::ForwardControl(
        const LoopController::ModeState& state,
        float desiredSpeedMps,
        float yawRateBiasRadps) const
    {
        if (_drive == nullptr)
        {
            return LoopController::ControlVector::Brake;
        }

        float clampedSpeedMps = desiredSpeedMps;
        if (std::isfinite(_limits.maxSpeedMps) && (_limits.maxSpeedMps > 0.0f))
        {
            clampedSpeedMps = (std::clamp)(clampedSpeedMps, -_limits.maxSpeedMps, _limits.maxSpeedMps);
        }

        float angularCommandRadps =
            (Config::kStraightHeadingKp * AngleErrorRad(_state.targetYawRad, state.estimate.yawRad)) -
            (Config::kStraightYawD * state.estimate.angularSpeedRadps) +
            yawRateBiasRadps;
        if (std::isfinite(_limits.maxAngularSpeedRadps) && (_limits.maxAngularSpeedRadps > 0.0f))
        {
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -_limits.maxAngularSpeedRadps,
                _limits.maxAngularSpeedRadps);
        }

        return _drive->PointControlVector(clampedSpeedMps, angularCommandRadps, _trackingCommandPd);
    }

    LoopController::ControlVector WallTouch::SeekControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        const unsigned long nowMs = millis();
        const float currentDistanceM =
            0.5f * (state.driveTelemetry.leftDistanceM + state.driveTelemetry.rightDistanceM);
        const float traveledDistanceM = std::fabs(currentDistanceM - _state.startDistanceM);
        const unsigned long elapsedMs = nowMs - _state.touchStartMs;
        const bool frontSignalActive =
            state.sensors.frontWall ||
            state.sensors.frontLeftWall ||
            state.sensors.frontRightWall;

        if (traveledDistanceM >= _state.maxApproachTravelM)
        {
            if (_allowPassThroughNoWall)
            {
                (void)BeginHoldBeforeReturn(false);
                return LoopController::ControlVector::Brake;
            }

            SetFault("WallTouch exceeded max approach travel");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        if (std::isfinite(currentDistanceM) &&
            std::isfinite(_state.lastProgressDistanceM) &&
            (std::fabs(currentDistanceM - _state.lastProgressDistanceM) >= Config::kWallTouchProgressStallDistanceM))
        {
            _state.lastProgressDistanceM = currentDistanceM;
            _state.lastMotionMs = nowMs;
        }

        float approachSpeedMps = Config::kWallTouchMaxApproachEncoderSpeedMps;
        const float finalApproachWindowM =
            (std::clamp)(Config::kWallTouchFinalApproachWindowM, 0.0f, 0.5f * _state.minLatchTravelM);
        if ((traveledDistanceM + finalApproachWindowM) >= _state.minLatchTravelM)
        {
            const float driveRatio =
                (Config::kWallTouchDriveCommand > 0.0f) ?
                (Config::kWallTouchFinalApproachDriveCommand / Config::kWallTouchDriveCommand) :
                1.0f;
            approachSpeedMps *= (std::clamp)(driveRatio, 0.1f, 1.0f);
        }

        LoopController::ControlVector control = LoopController::ControlVector::Brake;
        const float encoderSpeedMps =
            0.5f * (std::fabs(state.driveTelemetry.leftVelocityMps) + std::fabs(state.driveTelemetry.rightVelocityMps));
        if (!std::isfinite(encoderSpeedMps) || (encoderSpeedMps < Config::kWallTouchMaxApproachEncoderSpeedMps))
        {
            control = ForwardControl(state, approachSpeedMps);
        }

        std::uint8_t contactIndicators = 0U;
        if (frontSignalActive)
        {
            ++contactIndicators;
        }
        if ((std::fabs(state.estimate.linearSpeedMps) <= Config::kMotionSettleSpeedThresholdMps) &&
            ((traveledDistanceM >= Config::kWallTouchMinApproachDistanceM) ||
                ((elapsedMs >= Config::kWallTouchMinCommandTimeMs) && (traveledDistanceM >= _state.minLatchTravelM))))
        {
            ++contactIndicators;
        }
        if ((elapsedMs >= Config::kWallTouchMinCommandTimeMs) &&
            ((nowMs - _state.lastMotionMs) >= Config::kWallTouchProgressStallWindowMs))
        {
            ++contactIndicators;
        }

        if (contactIndicators >= 2U)
        {
            if (_state.contactCandidateStartMs == 0UL)
            {
                _state.contactCandidateStartMs = nowMs;
            }
            else if ((nowMs - _state.contactCandidateStartMs) >= Config::kWallTouchContactConfirmationMs)
            {
                _state.phaseStartMs = nowMs;
                _state.contactCandidateStartMs = 0UL;
                _activePhase = ActivePhase::Seat;
            }
        }
        else
        {
            _state.contactCandidateStartMs = 0UL;
        }

        return control;
    }

    LoopController::ControlVector WallTouch::SeatControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        (void)done;

        const unsigned long nowMs = millis();
        const unsigned long phaseElapsedMs = nowMs - _state.phaseStartMs;
        const unsigned long rampDurationMs =
            (std::max)(static_cast<unsigned long>(1U), static_cast<unsigned long>(Config::kWallTouchSeatRampMs));
        const unsigned long totalSeatMs =
            static_cast<unsigned long>(Config::kWallTouchSeatRampMs) +
            static_cast<unsigned long>(Config::kWallTouchInitialSeatDwellMs);

        float seatSpeedMps = Config::kWallTouchMaxSeatEncoderSpeedMps;
        if (phaseElapsedMs < Config::kWallTouchSeatRampMs)
        {
            const float rampAlpha =
                static_cast<float>((std::min)(phaseElapsedMs, rampDurationMs)) /
                static_cast<float>(rampDurationMs);
            const float driveRatio =
                (Config::kWallTouchDriveCommand > 0.0f) ?
                (Config::kWallTouchFinalApproachDriveCommand / Config::kWallTouchDriveCommand) :
                1.0f;
            const float entrySpeedMps =
                Config::kWallTouchMaxApproachEncoderSpeedMps * (std::clamp)(driveRatio, 0.1f, 1.0f);
            seatSpeedMps = entrySpeedMps + ((seatSpeedMps - entrySpeedMps) * rampAlpha);
        }

        if (phaseElapsedMs >= totalSeatMs)
        {
            _state.phaseStartMs = nowMs;
            _state.frontSignalMissingStartMs = 0UL;
            _state.squareStableStartMs = 0UL;
            _activePhase = ActivePhase::Square;
        }

        return ForwardControl(state, seatSpeedMps);
    }

    LoopController::ControlVector WallTouch::SquareControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        const unsigned long nowMs = millis();
        const unsigned long phaseElapsedMs = nowMs - _state.phaseStartMs;
        const bool frontSignalActive =
            state.sensors.frontWall ||
            state.sensors.frontLeftWall ||
            state.sensors.frontRightWall;

        if (!frontSignalActive)
        {
            if (_state.frontSignalMissingStartMs == 0UL)
            {
                _state.frontSignalMissingStartMs = nowMs;
            }
            else if ((nowMs - _state.frontSignalMissingStartMs) >= Config::kWallTouchContactConfirmationMs)
            {
                SetFault("WallTouch lost front-wall signal during square-up");
                done = true;
                return LoopController::ControlVector::Brake;
            }
        }
        else
        {
            _state.frontSignalMissingStartMs = 0UL;
        }

        float seatSpeedMps =
            Config::kWallTouchMaxSeatEncoderSpeedMps * Config::kWallTouchSeatWiggleRetainedForwardFraction;
        if (std::isfinite(_limits.maxSpeedMps) && (_limits.maxSpeedMps > 0.0f))
        {
            seatSpeedMps = (std::min)(seatSpeedMps, _limits.maxSpeedMps);
        }

        const float headingErrorRad = AngleErrorRad(_state.targetYawRad, state.estimate.yawRad);
        const bool squareStable =
            frontSignalActive &&
            (std::fabs(state.sensors.frontSkewM) <= Config::kWallTouchSquareFrontSkewThresholdM) &&
            (std::fabs(state.estimate.angularSpeedRadps) <= Config::kWallTouchSquareResidualYawRateThresholdRadps) &&
            (std::fabs(headingErrorRad) <= Config::kWallTouchSquareNetYawChangeThresholdRad);
        if (squareStable)
        {
            if (_state.squareStableStartMs == 0UL)
            {
                _state.squareStableStartMs = nowMs;
            }
            else if ((nowMs - _state.squareStableStartMs) >= Config::kWallTouchContactConfirmationMs)
            {
                (void)BeginHoldBeforeReturn(true);
                const float holdSeatSpeedMps =
                    Config::kWallTouchMaxSeatEncoderSpeedMps *
                    Config::kWallTouchSeatWiggleRetainedForwardFraction;
                return ForwardControl(state, holdSeatSpeedMps);
            }
        }
        else
        {
            _state.squareStableStartMs = 0UL;
        }

        if (phaseElapsedMs >= Config::kWallTouchSquareUpTimeoutMs)
        {
            SetFault("WallTouch square-up timed out");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        float yawBiasRadps = Config::kFrontSkewGain * state.sensors.frontSkewM;
        yawBiasRadps = (std::clamp)(
            yawBiasRadps,
            -Config::kWallTouchReverseMaxAngularCommandRadps,
            Config::kWallTouchReverseMaxAngularCommandRadps);
        return ForwardControl(state, seatSpeedMps, yawBiasRadps);
    }

    LoopController::ControlVector WallTouch::HoldBeforeReturnControls(
        const LoopController::ModeState& state,
        bool& done)
    {
        const unsigned long nowMs = millis();
        const unsigned long phaseElapsedMs = nowMs - _state.phaseStartMs;
        const unsigned long holdDurationMs =
            _state.resetPoseDuringHold ?
            static_cast<unsigned long>(Config::kWallTouchPostSquareHoldMs) :
            static_cast<unsigned long>(Config::kStartupWallCalibrationSettleMs);

        if (_state.resetPoseDuringHold &&
            !_state.poseResetApplied &&
            (phaseElapsedMs >= (holdDurationMs / 2UL)))
        {
            _state.poseResetApplied = true;
            _drive->SetPose(_state.contactXMeters, _state.contactYMeters, _state.targetYawRad);
        }

        if (phaseElapsedMs >= holdDurationMs)
        {
            if (!BeginReturnToPreferred())
            {
                SetFault("WallTouch return-to-preferred could not start");
                done = true;
            }
            else if (_activePhase == ActivePhase::None)
            {
                done = true;
            }

            return LoopController::ControlVector::Brake;
        }

        if (_state.resetPoseDuringHold)
        {
            const float holdSeatSpeedMps =
                Config::kWallTouchMaxSeatEncoderSpeedMps *
                Config::kWallTouchSeatWiggleRetainedForwardFraction;
            return ForwardControl(state, holdSeatSpeedMps);
        }

        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector WallTouch::ReturnToPreferredControls(bool& done)
    {
        if (_driveService == nullptr)
        {
            SetFault("WallTouch delegated Drive service is unavailable");
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (!_driveService->Active())
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        bool delegatedDone = false;
        const LoopController::ControlVector control = _driveService->GetNextControls(delegatedDone);
        if (!delegatedDone)
        {
            return control;
        }

        done = true;
        return LoopController::ControlVector::Brake;
    }
}
