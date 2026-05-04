#include "pch.h"
#include "StartupCalibration.h"

#include "Drive.h"
#include "DriveBase.h"
#include "SharedRobotRuntime.h"
#include "RuntimeSensorSuite.h"
#include "WallDistanceCalibration.h"
#include "WallTouch.h"

#include <cmath>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr const char* kStartupCalibrationLogSource = "startup_calibration";

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

        bool HasFrontBaselineCalibration(const WallSensorId sensorId) noexcept
        {
            float differentialLight = 0.0f;
            float lowDifferentialLight = 0.0f;
            float highDifferentialLight = 0.0f;
            return gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
                    sensorId,
                    differentialLight) &&
                gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
                    sensorId,
                    lowDifferentialLight,
                    highDifferentialLight) &&
                std::isfinite(differentialLight) &&
                (differentialLight >= 0.0f) &&
                IsValidNonNegativeBand(RobustSignalBand{ lowDifferentialLight, highDifferentialLight });
        }

        bool HasFullSideCalibration(const WallSensorId sensorId) noexcept
        {
            float baselineDifferentialLight = 0.0f;
            float referenceDifferentialLight = 0.0f;
            float referenceDistanceM = 0.0f;
            return gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
                    sensorId,
                    baselineDifferentialLight) &&
                gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
                    sensorId,
                    referenceDifferentialLight) &&
                gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                    sensorId,
                    referenceDistanceM) &&
                std::isfinite(baselineDifferentialLight) &&
                (baselineDifferentialLight >= 0.0f) &&
                std::isfinite(referenceDifferentialLight) &&
                (referenceDifferentialLight > 0.0f) &&
                std::isfinite(referenceDistanceM) &&
                (referenceDistanceM > 0.0f);
        }

        bool HasAnySideCalibrationData(const WallSensorId sensorId) noexcept
        {
            float differentialLight = 0.0f;
            float distanceM = 0.0f;
            return gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
                    sensorId,
                    differentialLight) ||
                gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
                    sensorId,
                    differentialLight) ||
                gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                    sensorId,
                    distanceM);
        }

        bool HasAnyWallCalibrationData() noexcept
        {
            return HasFrontBaselineCalibration(WallSensorId::FrontLeft) ||
                HasFrontBaselineCalibration(WallSensorId::FrontRight) ||
                HasAnySideCalibrationData(WallSensorId::SideLeft) ||
                HasAnySideCalibrationData(WallSensorId::SideRight);
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

    StartupCalibration::SensorCalibration StartupCalibration::GetSensorsCalibrated() const noexcept
    {
        return _sensorsCalibrated;
    }

    bool StartupCalibration::Active() const noexcept
    {
        return _phase != Phase::None;
    }

    bool StartupCalibration::BringUp()
    {
        if ((_sensors == nullptr) || Active())
        {
            return false;
        }

        const bool ok = _sensors->Begin(Config::kControlPeriodUs);
        _broughtUp = ok;
        _sensorsCalibrated = ok ? SensorCalibration::Imu : SensorCalibration::None;
        if (ok)
        {
            if (_drive != nullptr)
            {
                _drive->SetGyroBiasZ(_sensors->GetGyroBiasRadps());
            }
            RefreshSensorsCalibrated();
        }
        return ok;
    }

    void StartupCalibration::Cancel() noexcept
    {
        if (_wallTouch != nullptr)
        {
            _wallTouch->Cancel();
        }

        ResetState();
    }

    void StartupCalibration::Start()
    {
        if ((_wallTouch != nullptr) && _wallTouch->Active())
        {
            _wallTouch->Cancel();
        }

        ResetState();
        RefreshSensorsCalibrated();
        RestoreSideReferenceStateFromCalibration();
        if (!_broughtUp)
        {
            CompleteBestEffort("StartupCalibration was started without a successful BringUp");
            return;
        }
        if (!_isInMaze)
        {
            _phase = Phase::ReportCompletion;
            return;
        }
        _useFallbackWallCalibration = !HasAnyWallCalibrationData();

        if ((_runtime == nullptr) ||
            (_sensors == nullptr) ||
            (_drive == nullptr) ||
            (_driveService == nullptr) ||
            (_wallTouch == nullptr) ||
            (_speedVehicle == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not begin because shared runtime services were unavailable");
            return;
        }
        if (_useFallbackWallCalibration)
        {
            LogIssue("StartupCalibration is falling back to live wall calibration because no saved wall-calibration dataset was available");
        }

        _drive->SetPose(
            StartupCellCenterCoordinateM(),
            Config::kMissionStartRearWallInsetM,
            DirectionToYawRad(MazeMap::Up));
        if (!BeginDriveHoldPhase(Phase::SouthStartHold, Config::kStartupWallCalibrationSettleMs))
        {
            CompleteBestEffort("StartupCalibration could not begin the initial startup settle");
        }
    }

    LoopController::ControlVector StartupCalibration::GetNextControls(bool& done)
    {
        done = false;
        if (_phase == Phase::None)
        {
            done = true;
            return LoopController::ControlVector::Brake;
        }
        if (_phase == Phase::ReportCompletion)
        {
            _phase = Phase::None;
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
                CompleteBestEffort("StartupCalibration could not continue the active sampling phase");
            }

            UpdateDoneState(done);
            return LoopController::ControlVector::Brake;
        }

        if ((_phase == Phase::SouthTouch) || (_phase == Phase::WestTouch))
        {
            if ((_wallTouch == nullptr) || !_wallTouch->Active())
            {
                LogIssue("StartupCalibration expected an active WallTouch phase and will continue with best-effort fallback");
                AdvanceAfterWallTouchPhase();
                UpdateDoneState(done);
                return LoopController::ControlVector::Brake;
            }

            bool childDone = false;
            const LoopController::ControlVector control = _wallTouch->GetNextControls(childDone);
            if (!childDone)
            {
                return control;
            }

            AdvanceAfterWallTouchPhase();
            UpdateDoneState(done);
            return LoopController::ControlVector::Brake;
        }

        if (_driveService == nullptr)
        {
            CompleteBestEffort("StartupCalibration expected an active Drive phase and cannot continue");
            UpdateDoneState(done);
            return LoopController::ControlVector::Brake;
        }

        bool childDone = false;
        const LoopController::ControlVector control = _driveService->GetNextControls(childDone);
        if (!childDone)
        {
            return control;
        }

        AdvanceAfterDrivePhase();
        UpdateDoneState(done);
        return LoopController::ControlVector::Brake;
    }

    void StartupCalibration::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _sensors = &runtime.Sensors();
        _drive = &runtime.Drive();
        _driveService = &runtime.DriveService();
        _wallTouch = &runtime.WallTouchService();
        _speedVehicle = &runtime.SpeedVehicle();
        _travelLimits = BuildStartupTravelLimits();
    }

    void StartupCalibration::ResetState() noexcept
    {
        _phase = Phase::None;
        _useFallbackWallCalibration = false;
        _sideReferenceDistancesM = {};
        _sideReferenceValid = {};
    }

    void StartupCalibration::UpdateDoneState(bool& done) noexcept
    {
        if (_phase == Phase::ReportCompletion)
        {
            _phase = Phase::None;
            done = true;
            return;
        }

        done = (_phase == Phase::None);
    }

    void StartupCalibration::LogIssue(const char* const reason) noexcept
    {
        if ((_runtime != nullptr) && (reason != nullptr) && (reason[0] != '\0'))
        {
            (void)_runtime->WriteTextLogEntry(
                kStartupCalibrationLogSource,
                micros(),
                "issue",
                reason);
        }
    }

    void StartupCalibration::CompleteBestEffort(const char* const reason) noexcept
    {
        if (_wallTouch != nullptr)
        {
            _wallTouch->Cancel();
        }

        LogIssue(reason);
        _phase = Phase::ReportCompletion;
    }

    void StartupCalibration::RefreshSensorsCalibrated() noexcept
    {
        SensorCalibration calibrated = _broughtUp ? SensorCalibration::Imu : SensorCalibration::None;
        if (HasFrontBaselineCalibration(WallSensorId::FrontLeft))
        {
            calibrated |= SensorCalibration::FrontLeft;
        }
        if (HasFrontBaselineCalibration(WallSensorId::FrontRight))
        {
            calibrated |= SensorCalibration::FrontRight;
        }
        if (HasFullSideCalibration(WallSensorId::SideLeft))
        {
            calibrated |= SensorCalibration::SideLeft;
        }
        if (HasFullSideCalibration(WallSensorId::SideRight))
        {
            calibrated |= SensorCalibration::SideRight;
        }

        _sensorsCalibrated = calibrated;
    }

    void StartupCalibration::RestoreSideReferenceStateFromCalibration() noexcept
    {
        float leftReferenceDistanceM = 0.0f;
        if (gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                WallSensorId::SideLeft,
                leftReferenceDistanceM) &&
            std::isfinite(leftReferenceDistanceM) &&
            (leftReferenceDistanceM > 0.0f))
        {
            _sideReferenceDistancesM[0] = leftReferenceDistanceM;
            _sideReferenceValid[0] = true;
        }

        float rightReferenceDistanceM = 0.0f;
        if (gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                WallSensorId::SideRight,
                rightReferenceDistanceM) &&
            std::isfinite(rightReferenceDistanceM) &&
            (rightReferenceDistanceM > 0.0f))
        {
            _sideReferenceDistancesM[1] = rightReferenceDistanceM;
            _sideReferenceValid[1] = true;
        }
    }

    bool StartupCalibration::BeginDriveHoldPhase(const Phase phase, const std::uint16_t durationMs) noexcept
    {
        if (_driveService == nullptr)
        {
            return false;
        }

        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartHold(durationMs, true);
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

        const MazeMap::VehicleState& pose = _runtime->RuntimeState();
        float distanceM = 0.0f;
        switch (headingDirection)
        {
        case MazeMap::Left:
        case MazeMap::Right:
            distanceM = std::fabs(targetXMeters - pose.GetPositionX());
            break;
        case MazeMap::Up:
        case MazeMap::Down:
            distanceM = std::fabs(targetYMeters - pose.GetPositionY());
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
        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartStraight(
            distanceM,
            _travelLimits.maxSpeedMps,
            0.0f,
            &heading,
            &targetPosition);
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
        const float angleRad = AngleErrorRad(targetYawRad, _runtime->RuntimeState().GetOrientation());
        if (!std::isfinite(angleRad))
        {
            return false;
        }

        _driveService->SetLimits(_travelLimits);
        _driveService->SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService->StartTurn(angleRad);
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
        _wallTouch->SetAllowPassThroughNoWall(true);
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
                LogIssue("StartupCalibration could not move to the start-cell center and will continue from the current pose");
                if (!BeginDriveHoldPhase(Phase::CenterHold, Config::kStartupWallCalibrationSettleMs))
                {
                    CompleteBestEffort("StartupCalibration could not continue after the start-cell centering advisory");
                }
            }
            return;
        case Phase::MoveToCenter:
            if (!BeginDriveHoldPhase(Phase::CenterHold, Config::kStartupWallCalibrationSettleMs))
            {
                LogIssue("StartupCalibration could not settle at the start-cell center and will continue without that hold");
                if (!BeginDriveTurnPhase(Phase::RotateWest, MazeMap::Left))
                {
                    CompleteBestEffort("StartupCalibration could not rotate west for left-side sampling");
                }
            }
            return;
        case Phase::CenterHold:
            if (!BeginDriveTurnPhase(Phase::RotateWest, MazeMap::Left))
            {
                CompleteBestEffort("StartupCalibration could not rotate west for left-side sampling");
            }
            return;
        case Phase::RotateWest:
            if (!BeginDriveHoldPhase(Phase::WestHold, Config::kStartupWallCalibrationSettleMs))
            {
                LogIssue("StartupCalibration could not settle before left-side sampling and will sample immediately");
                _phase = Phase::SampleWest;
            }
            return;
        case Phase::WestHold:
            _phase = Phase::SampleWest;
            return;
        case Phase::RotateEast:
            if (!BeginDriveHoldPhase(Phase::EastHold, Config::kStartupWallCalibrationSettleMs))
            {
                LogIssue("StartupCalibration could not settle before right-side sampling and will sample immediately");
                _phase = Phase::SampleEast;
            }
            return;
        case Phase::EastHold:
            _phase = Phase::SampleEast;
            return;
        case Phase::RotateSouth:
            if (!BeginWallTouchPhase(Phase::SouthTouch, MazeMap::Down))
            {
                LogIssue("StartupCalibration could not begin the south-wall advisory touch and will try a west-wall fallback");
                if (!BeginDriveTurnPhase(Phase::RotateWestReseat, MazeMap::Left))
                {
                    CompleteBestEffort("StartupCalibration could not rotate west for fallback reseat");
                }
            }
            return;
        case Phase::RotateWestReseat:
            if (!BeginWallTouchPhase(Phase::WestTouch, MazeMap::Left))
            {
                LogIssue("StartupCalibration could not begin the west-wall advisory touch and will finish without pose reseat");
                if (!BeginDriveTurnPhase(Phase::RotateNorth, MazeMap::Up))
                {
                    CompleteBestEffort("StartupCalibration could not rotate north for the final front-baseline capture");
                }
            }
            return;
        case Phase::RotateNorth:
            if (!BeginDriveHoldPhase(Phase::NorthHold, Config::kStartupWallCalibrationSettleMs))
            {
                LogIssue("StartupCalibration could not settle at the reseated start pose and will sample the front baseline immediately");
                _phase = Phase::SampleFrontBaseline;
            }
            return;
        case Phase::NorthHold:
            _phase = Phase::SampleFrontBaseline;
            return;
        case Phase::FinalHold:
            _phase = Phase::None;
            return;
        default:
            CompleteBestEffort("StartupCalibration encountered an unexpected Drive phase completion");
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
                LogIssue("StartupCalibration could not rotate west after the south-wall advisory touch and will try to finish without that reseat");
                if (!BeginDriveTurnPhase(Phase::RotateNorth, MazeMap::Up))
                {
                    CompleteBestEffort("StartupCalibration could not rotate north after the south-wall advisory touch");
                }
            }
            return;
        case Phase::WestTouch:
            if (!BeginDriveTurnPhase(Phase::RotateNorth, MazeMap::Up))
            {
                CompleteBestEffort("StartupCalibration could not rotate north after the west-wall advisory touch");
            }
            return;
        default:
            CompleteBestEffort("StartupCalibration encountered an unexpected WallTouch phase completion");
            return;
        }
    }

    bool StartupCalibration::SampleWestFacingSideCalibration() noexcept
    {
        if ((_speedVehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample west-facing side calibration because runtime state was unavailable");
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

        if (_useFallbackWallCalibration)
        {
            float actualDistanceM = 0.0f;
            const bool storedReference =
                TryDistanceToSouthWall(_runtime->RuntimeState(), _speedVehicle->SideLeft, actualDistanceM) &&
                StoreSideReference(WallSensorId::SideLeft, leftCapture, actualDistanceM);
            if (storedReference)
            {
                _sideReferenceDistancesM[0] = actualDistanceM;
                _sideReferenceValid[0] = true;
            }
            else
            {
                LogIssue("StartupCalibration could not derive the west-facing left-side wall reference and will retain baseline-only coverage");
                if (!StoreSideBaseline(WallSensorId::SideLeft, leftCapture))
                {
                    LogIssue("StartupCalibration could not store the west-facing left-side baseline");
                }
            }
        }
        if (!StoreSideBaseline(WallSensorId::SideRight, rightCapture))
        {
            LogIssue("StartupCalibration could not store the west-facing right-side baseline");
        }

        if (!BeginDriveTurnPhase(Phase::RotateEast, MazeMap::Right))
        {
            CompleteBestEffort("StartupCalibration could not rotate east for right-side sampling");
            return false;
        }

        return true;
    }

    bool StartupCalibration::SampleEastFacingSideCalibration() noexcept
    {
        if ((_speedVehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample east-facing side calibration because runtime state was unavailable");
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

        if (_useFallbackWallCalibration)
        {
            float actualDistanceM = 0.0f;
            const bool storedReference =
                TryDistanceToSouthWall(_runtime->RuntimeState(), _speedVehicle->SideRight, actualDistanceM) &&
                StoreSideReference(WallSensorId::SideRight, rightCapture, actualDistanceM);
            if (storedReference)
            {
                _sideReferenceDistancesM[1] = actualDistanceM;
                _sideReferenceValid[1] = true;
            }
            else
            {
                LogIssue("StartupCalibration could not derive the east-facing right-side wall reference and will retain baseline-only coverage");
                if (!StoreSideBaseline(WallSensorId::SideRight, rightCapture))
                {
                    LogIssue("StartupCalibration could not store the east-facing right-side baseline");
                }
            }
        }
        if (!StoreSideBaseline(WallSensorId::SideLeft, leftCapture))
        {
            LogIssue("StartupCalibration could not store the east-facing left-side baseline");
        }
        const Phase nextPhase = _useFallbackWallCalibration ? Phase::RotateSouth : Phase::RotateNorth;
        const MazeMap::Direction nextDirection = _useFallbackWallCalibration ? MazeMap::Down : MazeMap::Up;
        if (!BeginDriveTurnPhase(nextPhase, nextDirection))
        {
            CompleteBestEffort(
                _useFallbackWallCalibration ?
                    "StartupCalibration could not rotate south for the reseat advisory touch" :
                    "StartupCalibration could not rotate north for final ambient-baseline capture");
            return false;
        }

        return true;
    }

    bool StartupCalibration::SampleFrontBaseline() noexcept
    {
        if ((_speedVehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample the front baseline because runtime state was unavailable");
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

        if (!StoreFrontBaseline(WallSensorId::FrontLeft, frontLeftCapture))
        {
            LogIssue("StartupCalibration could not store the front-left baseline");
        }
        if (!StoreFrontBaseline(WallSensorId::FrontRight, frontRightCapture))
        {
            LogIssue("StartupCalibration could not store the front-right baseline");
        }

        if (_sideReferenceValid[0] && _sideReferenceValid[1])
        {
            const float expectedSideWallDistanceM =
                0.5f * (_sideReferenceDistancesM[0] + _sideReferenceDistancesM[1]);
            gWallDistanceCalibration.SetExpectedSideWallDistanceM(expectedSideWallDistanceM);
        }
        else
        {
            LogIssue("StartupCalibration completed without full side-wall references and will keep the best available side-wall distance model");
        }
        if (_drive != nullptr)
        {
            _drive->SetPose(
                StartupCellCenterCoordinateM(),
                StartupCellCenterCoordinateM(),
                DirectionToYawRad(MazeMap::Up));
        }
        else
        {
            LogIssue("StartupCalibration could not reseat the final startup pose because DriveBase was unavailable");
        }
        if (!BeginDriveHoldPhase(Phase::FinalHold, Config::kStartupWallCalibrationSettleMs))
        {
            CompleteBestEffort("StartupCalibration could not begin the final startup settle");
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

        RefreshSensorsCalibrated();
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

        RefreshSensorsCalibrated();
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
        RefreshSensorsCalibrated();
        return true;
    }
}
