#include "pch.h"
#include "StartupCalibration.h"

#include "Drive.h"
#include "DriveBase.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeSensorSuite.h"
#include "WallTouch.h"

#include <cmath>

namespace MazeMap::App::Internal
{
    namespace
    {
        float StartupCellCenterCoordinateM() noexcept
        {
            return 0.5f * Config::kCellSizeM;
        }

        MotionLimits BuildStartupTravelLimits() noexcept
        {
            MotionLimits limits{};
            limits.maxSpeedMps = Config::kStartupWallCalibrationSpeedMps;
            limits.accelMps2 = Config::kStartupWallCalibrationAccelMps2;
            limits.decelMps2 = Config::kStartupWallCalibrationDecelMps2;
            limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationTurnMaxOmegaRadps;
            limits.angularAccelRadps2 = Config::kStartupWallCalibrationTurnAccelRadps2;
            return limits;
        }

        bool IsValidPositiveBand(const RobustSignalBand& band) noexcept
        {
            return std::isfinite(band.low) &&
                std::isfinite(band.high) &&
                (band.low > 0.0f) &&
                (band.high >= band.low);
        }

        bool IsValidNonNegativeBand(const RobustSignalBand& band) noexcept
        {
            return std::isfinite(band.low) &&
                std::isfinite(band.high) &&
                (band.low >= 0.0f) &&
                (band.high >= band.low);
        }
    }

    StartupCalibration::StartupCalibration()
        : _travelLimits(BuildStartupTravelLimits())
    {
    }

    void StartupCalibration::SetIsInMaze(const bool isInMaze) noexcept
    {
        _isInMaze = isInMaze;
    }

    bool StartupCalibration::GetIsInMaze() const noexcept
    {
        return _isInMaze;
    }

    bool StartupCalibration::Active() const noexcept
    {
        return _phase != Phase::None;
    }

    bool StartupCalibration::BringUp()
    {
        if ((_sensors == nullptr) || (_loopController == nullptr) || Active())
        {
            return false;
        }

        const unsigned long controlPeriodUs =
            (_loopController->_options.controlPeriodUs > 0U) ?
                static_cast<unsigned long>(_loopController->_options.controlPeriodUs) :
                static_cast<unsigned long>(Config::kControlPeriodUs);
        const bool ok = _sensors->Begin(controlPeriodUs);
        _broughtUp = ok;
        if (ok)
        {
            if (_drive != nullptr)
            {
                const_cast<MazeMap::VehicleState::StateVector&>(_drive->GetEstimatorStateVector())(MazeMap::VehicleState::kBgz) = _sensors->GetGyroBiasRadps();
            }
        }
        return ok;
    }

    void StartupCalibration::Cancel() noexcept
    {
        if (_driveService != nullptr)
        {
            _driveService->Cancel();
        }
        if (_wallTouch != nullptr)
        {
            _wallTouch->Cancel();
        }

        ResetState();
    }

    void StartupCalibration::Start()
    {
        if (!CanStart())
        {
            return;
        }

        ResetState();
        if (!_isInMaze)
        {
            return;
        }

        if (_drive == nullptr)
        {
            return;
        }

        gWallDistanceCalibration.Clear();
        _drive->SetPose(
            StartupCellCenterCoordinateM(),
            Config::kMissionStartRearWallInsetM,
            DirectionToYawRad(MazeMap::Up));
        if (!BeginDriveHoldPhase(Phase::SouthStartHold, Config::kStartupWallCalibrationSettleMs))
        {
            return;
        }
    }

    LoopController::ControlVector StartupCalibration::GetNextControls(bool& done)
    {
        done = false;
        if (_faulted)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (_phase == Phase::None)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }

        if ((_phase == Phase::SampleWest) ||
            (_phase == Phase::SampleEast) ||
            (_phase == Phase::SampleFrontBaseline))
        {
            bool ok = false;
            switch (_phase)
            {
            case Phase::SampleWest:
                ok = SampleWestFacingSideCalibration();
                break;
            case Phase::SampleEast:
                ok = SampleEastFacingSideCalibration();
                break;
            case Phase::SampleFrontBaseline:
                ok = SampleFrontBaseline();
                break;
            default:
                break;
            }

            if (!ok)
            {
                done = true;
                return LoopController::ControlVector::Brake;
            }

            done = (_phase == Phase::None);
            return LoopController::ControlVector::Brake;
        }

        if ((_phase == Phase::SouthTouch) || (_phase == Phase::WestTouch))
        {
            if ((_wallTouch == nullptr) || !_wallTouch->Active())
            {
                Fail("StartupCalibration expected an active WallTouch phase");
                done = true;
                return LoopController::ControlVector::Brake;
            }

            bool childDone = false;
            const LoopController::ControlVector control = _wallTouch->GetNextControls(childDone);
            if (!childDone)
            {
                return control;
            }

            AdvanceAfterWallTouchPhase();
            done = (_phase == Phase::None) || _faulted;
            return LoopController::ControlVector::Brake;
        }

        if ((_driveService == nullptr) || !_driveService->Active())
        {
            Fail("StartupCalibration expected an active Drive phase");
            done = true;
            return LoopController::ControlVector::Brake;
        }

        bool childDone = false;
        const LoopController::ControlVector control = _driveService->GetNextControls(childDone);
        if (!childDone)
        {
            return control;
        }

        AdvanceAfterDrivePhase();
        done = (_phase == Phase::None) || _faulted;
        return LoopController::ControlVector::Brake;
    }

    void StartupCalibration::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _loopController = &runtime.ControlLoop();
        _sensors = &runtime.Sensors();
        _drive = &runtime.Drive();
        _driveService = &runtime.DriveService();
        _wallTouch = &runtime.WallTouchService();
        _speedVehicle = &runtime.SpeedVehicle();
        _travelLimits = BuildStartupTravelLimits();
    }

    bool StartupCalibration::CanStart() const noexcept
    {
        return (_runtime != nullptr) &&
            (_loopController != nullptr) &&
            (_sensors != nullptr) &&
            (_drive != nullptr) &&
            (_driveService != nullptr) &&
            (_wallTouch != nullptr) &&
            (_speedVehicle != nullptr) &&
            _broughtUp &&
            !Active() &&
            !_driveService->Active() &&
            !_wallTouch->Active();
    }

    void StartupCalibration::ResetState() noexcept
    {
        _faulted = false;
        _phase = Phase::None;
        _sideReferenceDistancesM = {};
        _sideReferenceValid = {};
    }

    void StartupCalibration::Fail(const char* const reason) noexcept
    {
        _faulted = true;
        if (_driveService != nullptr)
        {
            _driveService->Cancel();
        }
        if (_wallTouch != nullptr)
        {
            _wallTouch->Cancel();
        }
        _phase = Phase::None;
        if (_runtime != nullptr)
        {
            (void)_runtime->FailActiveMode((reason != nullptr) ? reason : "StartupCalibration fault");
        }
    }

    bool StartupCalibration::BeginDriveHoldPhase(const Phase phase, const std::uint16_t durationMs) noexcept
    {
        if (_driveService == nullptr)
        {
            return false;
        }

        _driveService->Cancel();
        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartHold(durationMs, true);
        if (!_driveService->Active())
        {
            return false;
        }

        _phase = phase;
        return true;
    }

    bool StartupCalibration::BeginDriveMovePhase(
        const Phase phase,
        const float targetXMeters,
        const float targetYMeters,
        const MazeMap::Direction headingDirection) noexcept
    {
        if ((_drive == nullptr) || (_driveService == nullptr))
        {
            return false;
        }

        const PoseEstimate& pose = _drive->GetPose();
        float distanceM = 0.0f;
        switch (headingDirection)
        {
        case MazeMap::Left:
        case MazeMap::Right:
            distanceM = std::fabs(targetXMeters - pose.xMeters);
            break;
        case MazeMap::Up:
        case MazeMap::Down:
            distanceM = std::fabs(targetYMeters - pose.yMeters);
            break;
        default:
            return false;
        }

        if (!(std::isfinite(distanceM) && (distanceM > 0.0f)))
        {
            return false;
        }

        const Eigen::Vector2f heading = DirectionToUnitVector(headingDirection);
        const Eigen::Vector2f targetPosition(targetXMeters, targetYMeters);
        _driveService->Cancel();
        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartStraight(
            distanceM,
            _travelLimits.maxSpeedMps,
            0.0f,
            &heading,
            &targetPosition);
        if (!_driveService->Active())
        {
            return false;
        }

        _phase = phase;
        return true;
    }

    bool StartupCalibration::BeginDriveTurnPhase(
        const Phase phase,
        const MazeMap::Direction targetDirection) noexcept
    {
        if ((_drive == nullptr) || (_driveService == nullptr))
        {
            return false;
        }

        const float targetYawRad = DirectionToYawRad(targetDirection);
        const float angleRad = AngleErrorRad(targetYawRad, _drive->GetPose().yawRad);
        if (!std::isfinite(angleRad))
        {
            return false;
        }

        _driveService->Cancel();
        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartTurn(angleRad);
        if (!_driveService->Active())
        {
            return false;
        }

        _phase = phase;
        return true;
    }

    bool StartupCalibration::BeginWallTouchPhase(
        const Phase phase,
        const MazeMap::Direction wallDirection) noexcept
    {
        if (_wallTouch == nullptr)
        {
            return false;
        }

        _wallTouch->Cancel();
        _wallTouch->SetLimits(_travelLimits);
        _wallTouch->SetAllowPassThroughNoWall(false);
        _wallTouch->Start(MazeMap::CellCoordinates(0U, 0U), wallDirection);
        if (!_wallTouch->Active())
        {
            return false;
        }

        _phase = phase;
        return true;
    }

    void StartupCalibration::AdvanceAfterDrivePhase() noexcept
    {
        switch (_phase)
        {
        case Phase::SouthStartHold:
            if (!BeginDriveMovePhase(
                    Phase::MoveToCenter,
                    StartupCellCenterCoordinateM(),
                    StartupCellCenterCoordinateM(),
                    MazeMap::Up))
            {
                Fail("StartupCalibration could not move to start-cell center");
            }
            return;
        case Phase::MoveToCenter:
            if (!BeginDriveHoldPhase(Phase::CenterHold, Config::kStartupWallCalibrationSettleMs))
            {
                Fail("StartupCalibration could not settle at start-cell center");
            }
            return;
        case Phase::CenterHold:
            if (!BeginDriveTurnPhase(Phase::RotateWest, MazeMap::Left))
            {
                Fail("StartupCalibration could not rotate west for left-side sampling");
            }
            return;
        case Phase::RotateWest:
            if (!BeginDriveHoldPhase(Phase::WestHold, Config::kStartupWallCalibrationSettleMs))
            {
                Fail("StartupCalibration could not settle before left-side sampling");
            }
            return;
        case Phase::WestHold:
            _phase = Phase::SampleWest;
            return;
        case Phase::RotateEast:
            if (!BeginDriveHoldPhase(Phase::EastHold, Config::kStartupWallCalibrationSettleMs))
            {
                Fail("StartupCalibration could not settle before right-side sampling");
            }
            return;
        case Phase::EastHold:
            _phase = Phase::SampleEast;
            return;
        case Phase::RotateSouth:
            if (!BeginWallTouchPhase(Phase::SouthTouch, MazeMap::Down))
            {
                Fail("StartupCalibration could not begin the south-wall reseat touch");
            }
            return;
        case Phase::RotateWestReseat:
            if (!BeginWallTouchPhase(Phase::WestTouch, MazeMap::Left))
            {
                Fail("StartupCalibration could not begin the west-wall reseat touch");
            }
            return;
        case Phase::RotateNorth:
            if (!BeginDriveHoldPhase(Phase::NorthHold, Config::kStartupWallCalibrationSettleMs))
            {
                Fail("StartupCalibration could not settle at the reseated start pose");
            }
            return;
        case Phase::NorthHold:
            _phase = Phase::SampleFrontBaseline;
            return;
        case Phase::FinalHold:
            _phase = Phase::None;
            return;
        default:
            Fail("StartupCalibration encountered an unexpected Drive phase completion");
            return;
        }
    }

    void StartupCalibration::AdvanceAfterWallTouchPhase() noexcept
    {
        switch (_phase)
        {
        case Phase::SouthTouch:
            if (!BeginDriveTurnPhase(Phase::RotateWestReseat, MazeMap::Left))
            {
                Fail("StartupCalibration could not rotate west after south-wall reseat");
            }
            return;
        case Phase::WestTouch:
            if (!BeginDriveTurnPhase(Phase::RotateNorth, MazeMap::Up))
            {
                Fail("StartupCalibration could not rotate north after west-wall reseat");
            }
            return;
        default:
            Fail("StartupCalibration encountered an unexpected WallTouch phase completion");
            return;
        }
    }

    bool StartupCalibration::SampleWestFacingSideCalibration() noexcept
    {
        if (_speedVehicle == nullptr)
        {
            return false;
        }

        WallSensorCalibrationCapture leftCapture{};
        WallSensorCalibrationCapture rightCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::SideLeft,
            _speedVehicle->SideLeft,
            WallSensorId::SideRight,
            _speedVehicle->SideRight,
            leftCapture,
            rightCapture);

        float actualDistanceM = 0.0f;
        if ((_drive == nullptr) ||
            !TryDistanceToSouthWall(_drive->GetPose(), _speedVehicle->SideLeft, actualDistanceM) ||
            !StoreSideReference(WallSensorId::SideLeft, leftCapture, actualDistanceM) ||
            !StoreSideBaseline(WallSensorId::SideRight, rightCapture))
        {
            Fail("StartupCalibration could not store the west-facing side-wall calibration");
            return false;
        }

        _sideReferenceDistancesM[0] = actualDistanceM;
        _sideReferenceValid[0] = true;
        if (!BeginDriveTurnPhase(Phase::RotateEast, MazeMap::Right))
        {
            Fail("StartupCalibration could not rotate east for right-side sampling");
            return false;
        }

        return true;
    }

    bool StartupCalibration::SampleEastFacingSideCalibration() noexcept
    {
        if (_speedVehicle == nullptr)
        {
            return false;
        }

        WallSensorCalibrationCapture leftCapture{};
        WallSensorCalibrationCapture rightCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::SideLeft,
            _speedVehicle->SideLeft,
            WallSensorId::SideRight,
            _speedVehicle->SideRight,
            leftCapture,
            rightCapture);

        float actualDistanceM = 0.0f;
        if ((_drive == nullptr) ||
            !TryDistanceToSouthWall(_drive->GetPose(), _speedVehicle->SideRight, actualDistanceM) ||
            !StoreSideReference(WallSensorId::SideRight, rightCapture, actualDistanceM) ||
            !StoreSideBaseline(WallSensorId::SideLeft, leftCapture))
        {
            Fail("StartupCalibration could not store the east-facing side-wall calibration");
            return false;
        }

        _sideReferenceDistancesM[1] = actualDistanceM;
        _sideReferenceValid[1] = true;
        if (!BeginDriveTurnPhase(Phase::RotateSouth, MazeMap::Down))
        {
            Fail("StartupCalibration could not rotate south for the reseat touch");
            return false;
        }

        return true;
    }

    bool StartupCalibration::SampleFrontBaseline() noexcept
    {
        if (_speedVehicle == nullptr)
        {
            return false;
        }

        WallSensorCalibrationCapture frontLeftCapture{};
        WallSensorCalibrationCapture frontRightCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::FrontLeft,
            _speedVehicle->FrontLeft,
            WallSensorId::FrontRight,
            _speedVehicle->FrontRight,
            frontLeftCapture,
            frontRightCapture);

        if (!StoreFrontBaseline(WallSensorId::FrontLeft, frontLeftCapture) ||
            !StoreFrontBaseline(WallSensorId::FrontRight, frontRightCapture) ||
            !_sideReferenceValid[0] ||
            !_sideReferenceValid[1])
        {
            Fail("StartupCalibration could not finalize the startup calibration references");
            return false;
        }

        const float expectedSideWallDistanceM =
            0.5f * (_sideReferenceDistancesM[0] + _sideReferenceDistancesM[1]);
        gWallDistanceCalibration.SetExpectedSideWallDistanceM(expectedSideWallDistanceM);
        if (_drive != nullptr)
        {
            _drive->SetPose(
                StartupCellCenterCoordinateM(),
                StartupCellCenterCoordinateM(),
                DirectionToYawRad(MazeMap::Up));
        }
        if (!BeginDriveHoldPhase(Phase::FinalHold, Config::kStartupWallCalibrationSettleMs))
        {
            Fail("StartupCalibration could not begin the final startup settle");
            return false;
        }

        return true;
    }

    bool StartupCalibration::StoreSideReference(
        const WallSensorId sensorId,
        const WallSensorCalibrationCapture& capture,
        const float actualDistanceM) noexcept
    {
        const WallSensorCalibrationInput& input = capture.input;
        if (!(std::isfinite(actualDistanceM) &&
            (actualDistanceM > 0.0f) &&
            std::isfinite(input.measuredValue) &&
            (input.measuredValue > 0.0f) &&
            std::isfinite(input.differentialLight) &&
            (input.differentialLight > 0.0f) &&
            gWallDistanceCalibration.AddPoint(sensorId, input.measuredValue, actualDistanceM, input.ambientLight)))
        {
            return false;
        }

        gWallDistanceCalibration.SetSideWallReferenceDifferentialLight(sensorId, input.differentialLight);
        gWallDistanceCalibration.SetSideWallReferenceDistanceM(sensorId, actualDistanceM);
        if (capture.haveDifferentialLightBand && IsValidPositiveBand(capture.differentialLightBand))
        {
            gWallDistanceCalibration.SetSideWallReferenceDifferentialLightBand(
                sensorId,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
        }

        return true;
    }

    bool StartupCalibration::StoreSideBaseline(
        const WallSensorId sensorId,
        const WallSensorCalibrationCapture& capture) noexcept
    {
        const WallSensorCalibrationInput& input = capture.input;
        if (!(std::isfinite(input.differentialLight) && (input.differentialLight >= 0.0f)))
        {
            return false;
        }

        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(sensorId, input.differentialLight);
        if (capture.haveDifferentialLightBand && IsValidNonNegativeBand(capture.differentialLightBand))
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                sensorId,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
        }

        return true;
    }

    bool StartupCalibration::StoreFrontBaseline(
        const WallSensorId sensorId,
        const WallSensorCalibrationCapture& capture) noexcept
    {
        const WallSensorCalibrationInput& input = capture.input;
        if (!(std::isfinite(input.differentialLight) &&
            (input.differentialLight >= 0.0f) &&
            capture.haveDifferentialLightBand &&
            IsValidNonNegativeBand(capture.differentialLightBand)))
        {
            return false;
        }

        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(sensorId, input.differentialLight);
        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(
            sensorId,
            capture.differentialLightBand.low,
            capture.differentialLightBand.high);
        return true;
    }
}
