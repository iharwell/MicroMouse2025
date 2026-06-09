#include "pch.h"
#include "StartupCalibration.h"

#include "Drive.h"
#include "Imu.h"
#include "MazeMapRuntimeCore.h"
#include "RuntimeSensorSuite.h"
#include "SensorSnapshot.h"
#include "SharedRobotRuntime.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallDetectionThresholds.h"
#include "WallDistanceCalibration.h"
#include "WallSensor.h"
#include "WallTouch.h"

#include <cmath>
#include <limits>

namespace MazeMap::App::Internal
{
    float StartupCalibration::StartupCellCenterCoordinateM() noexcept
    {
        return 0.5f * Config::kCellSizeM;
    }

    MotionLimits StartupCalibration::BuildStartupTravelLimits() noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps(Config::kStartupWallCalibrationSpeedMps);
        limits.SetAccelMps2(Config::kStartupWallCalibrationAccelMps2);
        limits.SetDecelMps2(Config::kStartupWallCalibrationDecelMps2);
        limits.SetMaxAngularSpeedRadps(Config::kStartupWallCalibrationTurnMaxOmegaRadps);
        limits.SetAngularAccelRadps2(Config::kStartupWallCalibrationTurnAccelRadps2);
        return limits;
    }

    bool StartupCalibration::IsValidPositiveBand(const float low, const float high) noexcept
    {
        return std::isfinite(low) && std::isfinite(high) && (low > 0.0f) && (high >= low);
    }

    bool StartupCalibration::IsValidNonNegativeBand(const float low, const float high) noexcept
    {
        return std::isfinite(low) && std::isfinite(high) && (low >= 0.0f) && (high >= low);
    }

    bool StartupCalibration::HasFrontLeftBaselineCalibration() noexcept
    {
        float differentialLight = 0.0f;
        float lowDifferentialLight = 0.0f;
        float highDifferentialLight = 0.0f;
        return gWallDistanceCalibration.TryGetFrontLeftWallBaselineDifferentialLight(differentialLight) &&
            gWallDistanceCalibration.TryGetFrontLeftWallBaselineDifferentialLightBand(
                lowDifferentialLight,
                highDifferentialLight) &&
            std::isfinite(differentialLight) &&
            (differentialLight >= 0.0f) &&
            IsValidNonNegativeBand(lowDifferentialLight, highDifferentialLight);
    }

    bool StartupCalibration::HasFrontRightBaselineCalibration() noexcept
    {
        float differentialLight = 0.0f;
        float lowDifferentialLight = 0.0f;
        float highDifferentialLight = 0.0f;
        return gWallDistanceCalibration.TryGetFrontRightWallBaselineDifferentialLight(differentialLight) &&
            gWallDistanceCalibration.TryGetFrontRightWallBaselineDifferentialLightBand(
                lowDifferentialLight,
                highDifferentialLight) &&
            std::isfinite(differentialLight) &&
            (differentialLight >= 0.0f) &&
            IsValidNonNegativeBand(lowDifferentialLight, highDifferentialLight);
    }

    bool StartupCalibration::HasFullSideCalibration(const MazeMap::RelativeDirection side) noexcept
    {
        float baselineDifferentialLight = 0.0f;
        float referenceDifferentialLight = 0.0f;
        float referenceDistanceM = 0.0f;
        return gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(side, baselineDifferentialLight) &&
            gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(side, referenceDifferentialLight) &&
            gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(side, referenceDistanceM) &&
            std::isfinite(baselineDifferentialLight) &&
            (baselineDifferentialLight >= 0.0f) &&
            std::isfinite(referenceDifferentialLight) &&
            (referenceDifferentialLight > 0.0f) &&
            std::isfinite(referenceDistanceM) &&
            (referenceDistanceM > 0.0f);
    }

    bool StartupCalibration::HasAnySideCalibrationData(const MazeMap::RelativeDirection side) noexcept
    {
        float differentialLight = 0.0f;
        float distanceM = 0.0f;
        return gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(side, differentialLight) ||
            gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(side, differentialLight) ||
            gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(side, distanceM);
    }

    bool StartupCalibration::HasAnyWallCalibrationData() noexcept
    {
        return HasFrontLeftBaselineCalibration() ||
            HasFrontRightBaselineCalibration() ||
            HasAnySideCalibrationData(MazeMap::RelativeDirection::Left90) ||
            HasAnySideCalibrationData(MazeMap::RelativeDirection::Right90);
    }

    bool StartupCalibration::TryComputeDistanceToSouthStartWall(
        const MazeMap::VehicleState& state,
        const MazeMap::WallSensor& sensor,
        float& distanceM) noexcept
    {
        distanceM = 0.0f;
        const Eigen::Vector2f sensorPosition = sensor.WorldPosition(state);
        const Eigen::Vector2f sensorFacing = sensor.WorldFacing(state);
        const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(
            MazeMap::Config::kCellSizeM,
            MazeMap::Config::kMazeWallThicknessM);
        if (sensorFacing.y() >= -0.1f)
        {
            return false;
        }

        const float candidateDistanceM = (southWallYM - sensorPosition.y()) / sensorFacing.y();
        const float intersectionX = sensorPosition.x() + (candidateDistanceM * sensorFacing.x());
        if (candidateDistanceM <= 0.0f ||
            intersectionX < (westWallXM - 0.005f) ||
            intersectionX > (eastWallXM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && (distanceM > 0.0f);
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

        _controlPeriodUs = Config::kControlPeriodUs;
        const bool ok = _sensors->Begin(Config::kControlPeriodUs);
        _broughtUp = ok;
        _imuCalibrationComplete = false;
        RefreshSensorsCalibrated();
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

        if ((_runtime == nullptr) || (_sensors == nullptr) || (_vehicle == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not begin because shared runtime services were unavailable");
            return;
        }

        if (!BeginImuCalibration())
        {
            LogIssue("StartupCalibration could not begin IMU calibration; continuing without IMU calibration");
            if (!BeginMazeWallCalibration())
            {
                CompleteBestEffort("StartupCalibration could not begin wall calibration after IMU calibration setup failed");
            }
        }
    }

    CommandVector StartupCalibration::GetNextControls(bool& done)
    {
        done = false;
        if (_phase == Phase::None)
        {
            done = true;
            return CommandVector::Brake();
        }
        if (_phase == Phase::ReportCompletion)
        {
            _phase = Phase::None;
            done = true;
            return CommandVector::Brake();
        }

        if ((_phase == Phase::ImuBaselineSettle) ||
            (_phase == Phase::ImuBaselineSample) ||
            (_phase == Phase::ImuStimulatedSettle) ||
            (_phase == Phase::ImuStimulatedSample) ||
            (_phase == Phase::ImuDisabledSettle) ||
            (_phase == Phase::ImuBiasSample))
        {
            return RunImuCalibrationPhase(done);
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
            return CommandVector::Brake();
        }

        if ((_phase == Phase::SouthTouch) || (_phase == Phase::WestTouch))
        {
            if ((_wallTouch == nullptr) || !_wallTouch->Active())
            {
                LogIssue("StartupCalibration expected an active WallTouch phase and will continue with best-effort fallback");
                AdvanceAfterWallTouchPhase();
                UpdateDoneState(done);
                return CommandVector::Brake();
            }

            bool childDone = false;
            const CommandVector control = _wallTouch->GetNextControls(childDone);
            if (!childDone)
            {
                return control;
            }

            AdvanceAfterWallTouchPhase();
            UpdateDoneState(done);
            return CommandVector::Brake();
        }

        if (_driveService == nullptr)
        {
            CompleteBestEffort("StartupCalibration expected an active Drive phase and cannot continue");
            UpdateDoneState(done);
            return CommandVector::Brake();
        }

        bool childDone = false;
        const CommandVector control = _driveService->GetNextControls(childDone);
        if (!childDone)
        {
            return control;
        }

        AdvanceAfterDrivePhase();
        UpdateDoneState(done);
        return CommandVector::Brake();
    }

    void StartupCalibration::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _sensors = &runtime.Sensors();
        _driveService = &runtime.DriveService();
        _wallTouch = &runtime.WallTouchService();
        _vehicle = &runtime.Vehicle();
        _travelLimits = BuildStartupTravelLimits();
    }

    void StartupCalibration::ResetState() noexcept
    {
        if (_vehicle != nullptr)
        {
            _vehicle->BackLeftImu().DisableSelfTest();
        }

        _phase = Phase::None;
        _useFallbackWallCalibration = false;
        _leftSideReferenceDistanceM = 0.0f;
        _rightSideReferenceDistanceM = 0.0f;
        _leftSideReferenceValid = false;
        _rightSideReferenceValid = false;
        ResetWallSamplingState();
        ResetImuCalibrationState();
    }

    void StartupCalibration::ResetImuCalibrationState() noexcept
    {
        _imuCalibrationComplete = false;
        _imuPhaseTicksRemaining = 0U;
        _imuSampleCountdownTicks = 0U;
        _imuRequiredSamples = 0UL;
        _imuStartLeftEncoderCounts = 0;
        _imuStartRightEncoderCounts = 0;
        _imuEncoderBaselineCaptured = false;
        if (_vehicle != nullptr)
        {
            _vehicle->BackLeftImu().ResetCalibrationSampling();
        }
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
                kLogSource,
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
        ResetWallSamplingState();
        if (_vehicle != nullptr)
        {
            _vehicle->BackLeftImu().DisableSelfTest();
        }

        LogIssue(reason);
        _phase = Phase::ReportCompletion;
    }

    void StartupCalibration::RefreshSensorsCalibrated() noexcept
    {
        SensorCalibration calibrated = _imuCalibrationComplete ? SensorCalibration::Imu : SensorCalibration::None;
        if (HasFrontLeftBaselineCalibration())
        {
            calibrated |= SensorCalibration::FrontLeft;
        }
        if (HasFrontRightBaselineCalibration())
        {
            calibrated |= SensorCalibration::FrontRight;
        }
        if (HasFullSideCalibration(MazeMap::RelativeDirection::Left90))
        {
            calibrated |= SensorCalibration::SideLeft;
        }
        if (HasFullSideCalibration(MazeMap::RelativeDirection::Right90))
        {
            calibrated |= SensorCalibration::SideRight;
        }

        _sensorsCalibrated = calibrated;
    }

    void StartupCalibration::RestoreSideReferenceStateFromCalibration() noexcept
    {
        float leftReferenceDistanceM = 0.0f;
        if (gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                MazeMap::RelativeDirection::Left90,
                leftReferenceDistanceM) &&
            std::isfinite(leftReferenceDistanceM) &&
            (leftReferenceDistanceM > 0.0f))
        {
            _leftSideReferenceDistanceM = leftReferenceDistanceM;
            _leftSideReferenceValid = true;
        }

        float rightReferenceDistanceM = 0.0f;
        if (gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
                MazeMap::RelativeDirection::Right90,
                rightReferenceDistanceM) &&
            std::isfinite(rightReferenceDistanceM) &&
            (rightReferenceDistanceM > 0.0f))
        {
            _rightSideReferenceDistanceM = rightReferenceDistanceM;
            _rightSideReferenceValid = true;
        }
    }

    std::uint32_t StartupCalibration::TicksForDurationUs(const std::uint32_t durationUs) const noexcept
    {
        if (durationUs == 0U)
        {
            return 0U;
        }

        const unsigned long controlPeriodUs =
            (_controlPeriodUs == 0UL) ? Config::kControlPeriodUs : _controlPeriodUs;
        if (controlPeriodUs == 0UL)
        {
            return 1U;
        }

        const std::uint32_t ticks = static_cast<std::uint32_t>(
            (static_cast<unsigned long>(durationUs) + controlPeriodUs - 1UL) / controlPeriodUs);
        return (ticks == 0U) ? 1U : ticks;
    }

    std::uint32_t StartupCalibration::TicksForDurationMs(const std::uint32_t durationMs) const noexcept
    {
        if (durationMs > ((std::numeric_limits<std::uint32_t>::max)() / 1000U))
        {
            return TicksForDurationUs((std::numeric_limits<std::uint32_t>::max)());
        }
        return TicksForDurationUs(durationMs * 1000U);
    }

    std::uint32_t StartupCalibration::ImuCalibrationSampleIntervalTicks() const noexcept
    {
        const std::uint32_t ticks = TicksForDurationUs(kImuCalibrationSampleIntervalUs);
        return (ticks == 0U) ? 1U : ticks;
    }

    unsigned long StartupCalibration::RequiredGyroBiasSamples() const noexcept
    {
        const std::uint32_t sampleIntervalTicks = ImuCalibrationSampleIntervalTicks();
        const unsigned long controlPeriodUs =
            (_controlPeriodUs == 0UL) ? Config::kControlPeriodUs : _controlPeriodUs;
        return MazeMap::Imu::ComputeRequiredStationaryBiasSamples(
            static_cast<unsigned long>(Config::kGyroBiasSamples),
            controlPeriodUs,
            sampleIntervalTicks,
            static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs));
    }

    void StartupCalibration::CaptureCurrentEncoderTotalsForImuCalibration() noexcept
    {
        if (_runtime == nullptr)
        {
            _imuStartLeftEncoderCounts = 0;
            _imuStartRightEncoderCounts = 0;
            _imuEncoderBaselineCaptured = false;
            return;
        }

        const SensorSnapshot& snapshot = _runtime->RuntimeState().GetSensorSnapshot();
        _imuStartLeftEncoderCounts = snapshot.LeftEncoderTotalCounts();
        _imuStartRightEncoderCounts = snapshot.RightEncoderTotalCounts();
        _imuEncoderBaselineCaptured = true;
    }

    bool StartupCalibration::EncoderTotalsChangedDuringImuCalibration() const noexcept
    {
        if ((_runtime == nullptr) || !_imuEncoderBaselineCaptured)
        {
            return false;
        }

        const SensorSnapshot& snapshot = _runtime->RuntimeState().GetSensorSnapshot();
        return
            (snapshot.LeftEncoderTotalCounts() != _imuStartLeftEncoderCounts) ||
            (snapshot.RightEncoderTotalCounts() != _imuStartRightEncoderCounts);
    }

    void StartupCalibration::RestartImuCalibrationAfterMotion(const char* const reason) noexcept
    {
        LogIssue(reason);
        if (!BeginImuCalibration())
        {
            LogIssue("StartupCalibration could not restart IMU calibration after encoder motion; continuing without IMU calibration");
            if (!BeginMazeWallCalibration())
            {
                CompleteBestEffort("StartupCalibration could not begin wall calibration after IMU calibration restart failed");
            }
        }
    }

    bool StartupCalibration::BeginImuCalibration() noexcept
    {
        if ((_vehicle == nullptr) || (_sensors == nullptr))
        {
            return false;
        }

        ResetImuCalibrationState();
        _vehicle->BackLeftImu().ResetRuntimeCalibration();
        if (!_vehicle->BackLeftImu().ConfigureRuntimeForControlPeriod(
                _controlPeriodUs,
                true,
                Config::kMissionRuntimeAccelFilterFreq))
        {
            LogIssue("StartupCalibration could not configure IMU runtime sampling");
            return false;
        }

        _vehicle->BackLeftImu().DisableSelfTest();
        BeginImuSettlePhase(Phase::ImuBaselineSettle);
        return true;
    }

    CommandVector StartupCalibration::RunImuCalibrationPhase(bool& done)
    {
        done = false;
        if (!_imuEncoderBaselineCaptured)
        {
            CaptureCurrentEncoderTotalsForImuCalibration();
        }
        if (EncoderTotalsChangedDuringImuCalibration())
        {
            RestartImuCalibrationAfterMotion(
                "Encoder motion detected during stationary IMU calibration; restarting IMU self-test");
            return CommandVector::Brake();
        }

        if ((_phase == Phase::ImuBaselineSettle) ||
            (_phase == Phase::ImuStimulatedSettle) ||
            (_phase == Phase::ImuDisabledSettle))
        {
            if (_imuPhaseTicksRemaining > 0U)
            {
                --_imuPhaseTicksRemaining;
                return CommandVector::Brake();
            }

            if (_phase == Phase::ImuBaselineSettle)
            {
                BeginImuSamplePhase(Phase::ImuBaselineSample, kImuSelfTestAverageSamples);
                return CommandVector::Brake();
            }
            if (_phase == Phase::ImuStimulatedSettle)
            {
                BeginImuSamplePhase(Phase::ImuStimulatedSample, kImuSelfTestAverageSamples);
                return CommandVector::Brake();
            }

            CaptureCurrentEncoderTotalsForImuCalibration();
            BeginImuSamplePhase(Phase::ImuBiasSample, RequiredGyroBiasSamples());
            return CommandVector::Brake();
        }

        if ((_phase == Phase::ImuBaselineSample) ||
            (_phase == Phase::ImuStimulatedSample) ||
            (_phase == Phase::ImuBiasSample))
        {
            if (_imuSampleCountdownTicks > 0U)
            {
                --_imuSampleCountdownTicks;
                return CommandVector::Brake();
            }

            if (_phase == Phase::ImuBiasSample)
            {
                AccumulateCurrentImuBiasSample();
            }
            else
            {
                AccumulateCurrentImuSelfTestSample();
            }

            if ((_vehicle != nullptr) &&
                (_vehicle->BackLeftImu().CalibrationCollectedSamples() < _imuRequiredSamples))
            {
                const std::uint32_t intervalTicks = ImuCalibrationSampleIntervalTicks();
                _imuSampleCountdownTicks = (intervalTicks > 0U) ? (intervalTicks - 1U) : 0U;
                return CommandVector::Brake();
            }

            if (_phase == Phase::ImuBaselineSample)
            {
                StoreCurrentSelfTestAverageAsBaseline();
                _vehicle->BackLeftImu().EnablePositiveSelfTest();
                BeginImuSettlePhase(Phase::ImuStimulatedSettle);
                return CommandVector::Brake();
            }
            if (_phase == Phase::ImuStimulatedSample)
            {
                if (!ValidateAndStoreStimulatedSelfTestAverage())
                {
                    LogIssue("StartupCalibration IMU self-test failed; continuing without IMU calibration");
                    _vehicle->BackLeftImu().DisableSelfTest();
                    if (!BeginMazeWallCalibration())
                    {
                        CompleteBestEffort("StartupCalibration could not begin wall calibration after IMU self-test failed");
                    }
                    UpdateDoneState(done);
                    return CommandVector::Brake();
                }
                _vehicle->BackLeftImu().DisableSelfTest();
                BeginImuSettlePhase(Phase::ImuDisabledSettle);
                return CommandVector::Brake();
            }

            if (!CompleteImuBiasMeasurement())
            {
                LogIssue("StartupCalibration could not complete IMU bias measurement; continuing without IMU calibration");
                if (!BeginMazeWallCalibration())
                {
                    CompleteBestEffort("StartupCalibration could not begin wall calibration after IMU bias measurement failed");
                }
                UpdateDoneState(done);
                return CommandVector::Brake();
            }
            if (!BeginMazeWallCalibration())
            {
                CompleteBestEffort("StartupCalibration could not begin wall calibration after IMU calibration");
            }
            UpdateDoneState(done);
            return CommandVector::Brake();
        }

        CompleteBestEffort("StartupCalibration encountered an unexpected IMU calibration phase");
        UpdateDoneState(done);
        return CommandVector::Brake();
    }

    void StartupCalibration::BeginImuSettlePhase(const Phase phase) noexcept
    {
        _imuPhaseTicksRemaining = TicksForDurationUs(kImuSelfTestSettleUs);
        _imuSampleCountdownTicks = 0U;
        _phase = phase;
    }

    void StartupCalibration::BeginImuSamplePhase(
        const Phase phase,
        const unsigned long requiredSamples) noexcept
    {
        _imuRequiredSamples = requiredSamples;
        _imuSampleCountdownTicks = 0U;
        if (_vehicle != nullptr)
        {
            if ((phase == Phase::ImuBaselineSample) || (phase == Phase::ImuStimulatedSample))
            {
                _vehicle->BackLeftImu().BeginSelfTestSampling();
            }
            else if (phase == Phase::ImuBiasSample)
            {
                _vehicle->BackLeftImu().BeginStationaryBiasSampling();
            }
        }
        _phase = phase;
    }

    void StartupCalibration::AccumulateCurrentImuSelfTestSample() noexcept
    {
        if (_vehicle == nullptr)
        {
            return;
        }

        _vehicle->BackLeftImu().AccumulateSelfTestSample();
    }

    void StartupCalibration::AccumulateCurrentImuBiasSample() noexcept
    {
        if (_vehicle == nullptr)
        {
            return;
        }

        _vehicle->BackLeftImu().AccumulateStationaryBiasSample();
    }

    void StartupCalibration::StoreCurrentSelfTestAverageAsBaseline() noexcept
    {
        if (_vehicle == nullptr)
        {
            return;
        }

        _vehicle->BackLeftImu().StoreCurrentSelfTestAverageAsBaseline();
    }

    bool StartupCalibration::ValidateAndStoreStimulatedSelfTestAverage() noexcept
    {
        if (_vehicle == nullptr)
        {
            return false;
        }

        const bool ok = _vehicle->BackLeftImu().ValidateStimulatedSelfTestAverage();
        if (!ok && (_runtime != nullptr))
        {
            (void)_runtime->AppendTextLogFormatted(
                "IMU stationary self-test failed; accel_delta_mg=[%.1f,%.1f,%.1f], gyro_delta_dps=[%.1f,%.1f,%.1f]",
                _vehicle->BackLeftImu().LastSelfTestAccelDeltaMg(0U),
                _vehicle->BackLeftImu().LastSelfTestAccelDeltaMg(1U),
                _vehicle->BackLeftImu().LastSelfTestAccelDeltaMg(2U),
                _vehicle->BackLeftImu().LastSelfTestGyroDeltaDps(0U),
                _vehicle->BackLeftImu().LastSelfTestGyroDeltaDps(1U),
                _vehicle->BackLeftImu().LastSelfTestGyroDeltaDps(2U));
        }
        return ok;
    }

    bool StartupCalibration::CompleteImuBiasMeasurement() noexcept
    {
        if (_vehicle == nullptr)
        {
            return false;
        }

        if (!_vehicle->BackLeftImu().CompleteStationaryBiasSampling())
        {
            return false;
        }

        _imuCalibrationComplete = true;
        RefreshSensorsCalibrated();
        return true;
    }

    bool StartupCalibration::BeginMazeWallCalibration() noexcept
    {
        if (!_isInMaze)
        {
            _phase = Phase::ReportCompletion;
            return true;
        }

        _useFallbackWallCalibration = !HasAnyWallCalibrationData();
        if ((_runtime == nullptr) ||
            (_sensors == nullptr) ||
            (_driveService == nullptr) ||
            (_wallTouch == nullptr) ||
            (_vehicle == nullptr))
        {
            return false;
        }
        if (_useFallbackWallCalibration)
        {
            LogIssue("StartupCalibration is falling back to live wall calibration because no saved wall-calibration dataset was available");
        }

        if (!_runtime->Estimator().ResetPose(
                StartupCellCenterCoordinateM(),
                Config::kMissionStartRearWallInsetM,
                DirectionToYawRad(MazeMap::Up)))
        {
            CompleteBestEffort(_runtime->Estimator().FaultReason());
            return true;
        }
        if (!BeginDriveHoldPhase(Phase::SouthStartHold, Config::kStartupWallCalibrationSettleMs))
        {
            CompleteBestEffort("StartupCalibration could not begin the initial startup settle");
            return true;
        }
        return true;
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
        if ((_runtime == nullptr) || (_driveService == nullptr))
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
            _travelLimits.GetMaxSpeedMps(),
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
        if ((_runtime == nullptr) || (_driveService == nullptr))
        {
            return false;
        }

        const float targetYawRad = DirectionToYawRad(targetDirection);
        const float angleRad = AngleErrorRad(targetYawRad, _runtime->RuntimeState().GetHeading());
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
                LogIssue("StartupCalibration could not begin the south-wall advisory touch and will try to continue");
                if (!BeginDriveTurnPhase(Phase::RotateWestReseat, MazeMap::Left))
                {
                    CompleteBestEffort("StartupCalibration could not rotate west after the failed south-wall advisory touch");
                }
            }
            return;
        case Phase::RotateWestReseat:
            if (!BeginWallTouchPhase(Phase::WestTouch, MazeMap::Left))
            {
                LogIssue("StartupCalibration could not begin the west-wall advisory touch and will try to finish without that reseat");
                if (!BeginDriveTurnPhase(Phase::RotateNorth, MazeMap::Up))
                {
                    CompleteBestEffort("StartupCalibration could not rotate north after the failed west-wall advisory touch");
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
        if ((_vehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample west-facing side calibration because runtime state was unavailable");
            return false;
        }

        bool sampleComplete = false;
        if (!SampleSideWallPair(sampleComplete))
        {
            return false;
        }
        if (!sampleComplete)
        {
            return true;
        }

        const float leftMeasuredValue = _wallFirstMeasuredValue;
        const float leftAmbientLight = _wallFirstAmbientLight;
        const float leftDifferentialLight = _wallFirstDifferentialLight;
        const bool leftDifferentialLightBandValid = _wallFirstDifferentialLightBandValid;
        const float leftDifferentialLightBandLow = _wallFirstDifferentialLightBandLow;
        const float leftDifferentialLightBandHigh = _wallFirstDifferentialLightBandHigh;
        const float rightMeasuredValue = _wallSecondMeasuredValue;
        const float rightAmbientLight = _wallSecondAmbientLight;
        const float rightDifferentialLight = _wallSecondDifferentialLight;
        const bool rightDifferentialLightBandValid = _wallSecondDifferentialLightBandValid;
        const float rightDifferentialLightBandLow = _wallSecondDifferentialLightBandLow;
        const float rightDifferentialLightBandHigh = _wallSecondDifferentialLightBandHigh;

        if (_useFallbackWallCalibration)
        {
            float actualDistanceM = 0.0f;
            const bool storedReference =
                TryComputeDistanceToSouthStartWall(
                    _runtime->RuntimeState(),
                    _vehicle->SideLeftWallSensor(),
                    actualDistanceM) &&
                StoreSideReference(
                    MazeMap::RelativeDirection::Left90,
                    leftMeasuredValue,
                    leftAmbientLight,
                    leftDifferentialLight,
                    leftDifferentialLightBandValid,
                    leftDifferentialLightBandLow,
                    leftDifferentialLightBandHigh,
                    actualDistanceM);
            if (storedReference)
            {
                _leftSideReferenceDistanceM = actualDistanceM;
                _leftSideReferenceValid = true;
            }
            else
            {
                LogIssue("StartupCalibration could not derive the west-facing left-side wall reference and will retain baseline-only coverage");
                if (!StoreSideBaseline(
                        MazeMap::RelativeDirection::Left90,
                        leftDifferentialLight,
                        leftDifferentialLightBandValid,
                        leftDifferentialLightBandLow,
                        leftDifferentialLightBandHigh))
                {
                    LogIssue("StartupCalibration could not store the west-facing left-side baseline");
                }
            }
        }
        if (!StoreSideBaseline(
                MazeMap::RelativeDirection::Right90,
                rightDifferentialLight,
                rightDifferentialLightBandValid,
                rightDifferentialLightBandLow,
                rightDifferentialLightBandHigh))
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
        if ((_vehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample east-facing side calibration because runtime state was unavailable");
            return false;
        }

        bool sampleComplete = false;
        if (!SampleSideWallPair(sampleComplete))
        {
            return false;
        }
        if (!sampleComplete)
        {
            return true;
        }

        const float leftMeasuredValue = _wallFirstMeasuredValue;
        const float leftAmbientLight = _wallFirstAmbientLight;
        const float leftDifferentialLight = _wallFirstDifferentialLight;
        const bool leftDifferentialLightBandValid = _wallFirstDifferentialLightBandValid;
        const float leftDifferentialLightBandLow = _wallFirstDifferentialLightBandLow;
        const float leftDifferentialLightBandHigh = _wallFirstDifferentialLightBandHigh;
        const float rightMeasuredValue = _wallSecondMeasuredValue;
        const float rightAmbientLight = _wallSecondAmbientLight;
        const float rightDifferentialLight = _wallSecondDifferentialLight;
        const bool rightDifferentialLightBandValid = _wallSecondDifferentialLightBandValid;
        const float rightDifferentialLightBandLow = _wallSecondDifferentialLightBandLow;
        const float rightDifferentialLightBandHigh = _wallSecondDifferentialLightBandHigh;

        if (_useFallbackWallCalibration)
        {
            float actualDistanceM = 0.0f;
            const bool storedReference =
                TryComputeDistanceToSouthStartWall(
                    _runtime->RuntimeState(),
                    _vehicle->SideRightWallSensor(),
                    actualDistanceM) &&
                StoreSideReference(
                    MazeMap::RelativeDirection::Right90,
                    rightMeasuredValue,
                    rightAmbientLight,
                    rightDifferentialLight,
                    rightDifferentialLightBandValid,
                    rightDifferentialLightBandLow,
                    rightDifferentialLightBandHigh,
                    actualDistanceM);
            if (storedReference)
            {
                _rightSideReferenceDistanceM = actualDistanceM;
                _rightSideReferenceValid = true;
            }
            else
            {
                LogIssue("StartupCalibration could not derive the east-facing right-side wall reference and will retain baseline-only coverage");
                if (!StoreSideBaseline(
                        MazeMap::RelativeDirection::Right90,
                        rightDifferentialLight,
                        rightDifferentialLightBandValid,
                        rightDifferentialLightBandLow,
                        rightDifferentialLightBandHigh))
                {
                    LogIssue("StartupCalibration could not store the east-facing right-side baseline");
                }
            }
        }
        if (!StoreSideBaseline(
                MazeMap::RelativeDirection::Left90,
                leftDifferentialLight,
                leftDifferentialLightBandValid,
                leftDifferentialLightBandLow,
                leftDifferentialLightBandHigh))
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
        if ((_vehicle == nullptr) || (_runtime == nullptr))
        {
            CompleteBestEffort("StartupCalibration could not sample the front baseline because runtime state was unavailable");
            return false;
        }

        bool sampleComplete = false;
        if (!SampleFrontWallPair(sampleComplete))
        {
            return false;
        }
        if (!sampleComplete)
        {
            return true;
        }

        const float frontLeftDifferentialLight = _wallFirstDifferentialLight;
        const bool frontLeftDifferentialLightBandValid = _wallFirstDifferentialLightBandValid;
        const float frontLeftDifferentialLightBandLow = _wallFirstDifferentialLightBandLow;
        const float frontLeftDifferentialLightBandHigh = _wallFirstDifferentialLightBandHigh;
        const float frontRightDifferentialLight = _wallSecondDifferentialLight;
        const bool frontRightDifferentialLightBandValid = _wallSecondDifferentialLightBandValid;
        const float frontRightDifferentialLightBandLow = _wallSecondDifferentialLightBandLow;
        const float frontRightDifferentialLightBandHigh = _wallSecondDifferentialLightBandHigh;

        if (!StoreFrontLeftBaseline(
                frontLeftDifferentialLight,
                frontLeftDifferentialLightBandValid,
                frontLeftDifferentialLightBandLow,
                frontLeftDifferentialLightBandHigh))
        {
            LogIssue("StartupCalibration could not store the front-left baseline");
        }
        if (!StoreFrontRightBaseline(
                frontRightDifferentialLight,
                frontRightDifferentialLightBandValid,
                frontRightDifferentialLightBandLow,
                frontRightDifferentialLightBandHigh))
        {
            LogIssue("StartupCalibration could not store the front-right baseline");
        }

        if (_leftSideReferenceValid && _rightSideReferenceValid)
        {
            const float expectedSideWallDistanceM =
                0.5f * (_leftSideReferenceDistanceM + _rightSideReferenceDistanceM);
            gWallDistanceCalibration.SetExpectedSideWallDistanceM(expectedSideWallDistanceM);
        }
        else
        {
            LogIssue("StartupCalibration completed without full side-wall references and will keep the best available side-wall distance model");
        }

        if (!_runtime->Estimator().ResetPose(
                StartupCellCenterCoordinateM(),
                StartupCellCenterCoordinateM(),
                DirectionToYawRad(MazeMap::Up)))
        {
            CompleteBestEffort(_runtime->Estimator().FaultReason());
            return false;
        }
        if (!BeginDriveHoldPhase(Phase::FinalHold, Config::kStartupWallCalibrationSettleMs))
        {
            CompleteBestEffort("StartupCalibration could not begin the final startup settle");
            return false;
        }

        return true;
    }

    bool StartupCalibration::SampleSideWallPair(bool& sampleComplete) noexcept
    {
        sampleComplete = false;
        if (_vehicle == nullptr)
        {
            return false;
        }

        if (!_wallSampleActive)
        {
            if (!BeginWallSensorPairSampling(
                    _vehicle->SideLeftWallSensor(),
                    _vehicle->SideRightWallSensor(),
                    true))
            {
                return false;
            }
        }

        return ServiceWallSensorPairSampling(sampleComplete);
    }

    bool StartupCalibration::SampleFrontWallPair(bool& sampleComplete) noexcept
    {
        sampleComplete = false;
        if (_vehicle == nullptr)
        {
            return false;
        }

        if (!_wallSampleActive)
        {
            if (!BeginWallSensorPairSampling(
                    _vehicle->FrontLeftWallSensor(),
                    _vehicle->FrontRightWallSensor(),
                    false))
            {
                return false;
            }
        }

        return ServiceWallSensorPairSampling(sampleComplete);
    }

    bool StartupCalibration::BeginWallSensorPairSampling(
        MazeMap::WallSensor& first,
        MazeMap::WallSensor& second,
        const bool measuredValueFromRawDistance) noexcept
    {
        ResetWallSamplingState();
        _wallSampleFirstSensor = &first;
        _wallSampleSecondSensor = &second;
        _wallSampleMeasuredValueFromRawDistance = measuredValueFromRawDistance;
        _wallSampleTicksRemaining = TicksForDurationMs(kWallCalibrationPairSamplingTimeoutMs);
        if (_wallSampleTicksRemaining == 0U)
        {
            _wallSampleTicksRemaining = 1U;
        }
        _wallSampleActive = true;

        const std::uint32_t ledOffCommandUs = micros();
        first.CommandLedOff(ledOffCommandUs);
        second.CommandLedOff(ledOffCommandUs);
        return true;
    }

    bool StartupCalibration::ServiceWallSensorPairSampling(bool& sampleComplete) noexcept
    {
        sampleComplete = false;
        if (!_wallSampleActive ||
            (_wallSampleFirstSensor == nullptr) ||
            (_wallSampleSecondSensor == nullptr))
        {
            return false;
        }

        if (_wallSampleTicksRemaining == 0U)
        {
            _wallSampleTimedOut = true;
            FinishWallSensorPairSampling();
            sampleComplete = true;
            return true;
        }
        --_wallSampleTicksRemaining;

        MazeMap::WallSensor& first = *_wallSampleFirstSensor;
        MazeMap::WallSensor& second = *_wallSampleSecondSensor;
        if (!_wallSampleAmbientCaptured)
        {
            const std::uint32_t ambientNowUs = micros();
            if (!first.IsAmbientReadReady(ambientNowUs) || !second.IsAmbientReadReady(ambientNowUs))
            {
                return true;
            }

            first.CaptureAmbientRead();
            second.CaptureAmbientRead();
            const std::uint32_t ledOnCommandUs = micros();
            first.CommandLedOn(ledOnCommandUs);
            second.CommandLedOn(ledOnCommandUs);
            _wallSampleAmbientCaptured = true;
        }

        const std::uint32_t litNowUs = micros();
        if (!first.IsLitReadReady(litNowUs) || !second.IsLitReadReady(litNowUs))
        {
            return true;
        }

        first.CaptureLitRead();
        second.CaptureLitRead();
        const std::uint32_t completeUs = micros();
        first.CompleteCapture(completeUs);
        second.CompleteCapture(completeUs);
        AccumulateWallSensorPairSample();

        if (_wallSampleCount >= kWallCalibrationSampleCount)
        {
            FinishWallSensorPairSampling();
            sampleComplete = true;
            return true;
        }

        const std::uint32_t nextLedOffCommandUs = micros();
        first.CommandLedOff(nextLedOffCommandUs);
        second.CommandLedOff(nextLedOffCommandUs);
        _wallSampleAmbientCaptured = false;
        return true;
    }

    void StartupCalibration::AccumulateWallSensorPairSample() noexcept
    {
        if ((_wallSampleFirstSensor == nullptr) ||
            (_wallSampleSecondSensor == nullptr) ||
            (_wallSampleCount >= kWallCalibrationSampleCount))
        {
            return;
        }

        const MazeMap::WallSensor& first = *_wallSampleFirstSensor;
        const MazeMap::WallSensor& second = *_wallSampleSecondSensor;
        const std::uint16_t sampleIndex = _wallSampleCount;
        const float firstMeasuredValue =
            _wallSampleMeasuredValueFromRawDistance ? first.LatestRawDistanceM() : first.LatestDifferentialLight();
        const float secondMeasuredValue =
            _wallSampleMeasuredValueFromRawDistance ? second.LatestRawDistanceM() : second.LatestDifferentialLight();
        const float firstDifferentialLight = first.LatestDifferentialLight();
        const float secondDifferentialLight = second.LatestDifferentialLight();
        _wallFirstDifferentialSamples[sampleIndex] = firstDifferentialLight;
        _wallSecondDifferentialSamples[sampleIndex] = secondDifferentialLight;
        AccumulateFiniteWallValue(firstMeasuredValue, _wallFirstMeasuredSum, _wallFirstMeasuredCount);
        AccumulateFiniteWallValue(first.LatestAmbientLight(), _wallFirstAmbientSum, _wallFirstAmbientCount);
        AccumulateFiniteWallValue(firstDifferentialLight, _wallFirstDifferentialSum, _wallFirstDifferentialCount);
        AccumulateFiniteWallValue(secondMeasuredValue, _wallSecondMeasuredSum, _wallSecondMeasuredCount);
        AccumulateFiniteWallValue(second.LatestAmbientLight(), _wallSecondAmbientSum, _wallSecondAmbientCount);
        AccumulateFiniteWallValue(secondDifferentialLight, _wallSecondDifferentialSum, _wallSecondDifferentialCount);
        ++_wallSampleCount;
    }

    void StartupCalibration::FinishWallSensorPairSampling() noexcept
    {
        if (_wallSampleTimedOut)
        {
            if ((_wallSampleFirstSensor != nullptr) && (_wallSampleSecondSensor != nullptr))
            {
                const std::uint32_t ledOffCommandUs = micros();
                _wallSampleFirstSensor->CommandLedOff(ledOffCommandUs);
                _wallSampleSecondSensor->CommandLedOff(ledOffCommandUs);
            }
            LogIssue(
                (_wallSampleCount > 0U) ?
                    "StartupCalibration wall-sensor sampling timed out and will continue with partial samples" :
                    "StartupCalibration wall-sensor sampling timed out with no completed samples");
        }

        _wallFirstMeasuredValue = AverageWallCalibrationSum(_wallFirstMeasuredSum, _wallFirstMeasuredCount);
        _wallFirstAmbientLight = AverageWallCalibrationSum(_wallFirstAmbientSum, _wallFirstAmbientCount);
        _wallFirstDifferentialLight = AverageWallCalibrationSum(_wallFirstDifferentialSum, _wallFirstDifferentialCount);
        float median = 0.0f;
        _wallFirstDifferentialLightBandValid = MazeMap::TryComputeRobustSignalBandFromSamples(
            _wallFirstDifferentialSamples,
            _wallSampleCount,
            MazeMap::Config::kWallCalibrationScaledMadMultiplier,
            median,
            _wallFirstDifferentialLightBandLow,
            _wallFirstDifferentialLightBandHigh);

        _wallSecondMeasuredValue = AverageWallCalibrationSum(_wallSecondMeasuredSum, _wallSecondMeasuredCount);
        _wallSecondAmbientLight = AverageWallCalibrationSum(_wallSecondAmbientSum, _wallSecondAmbientCount);
        _wallSecondDifferentialLight = AverageWallCalibrationSum(
            _wallSecondDifferentialSum,
            _wallSecondDifferentialCount);
        median = 0.0f;
        _wallSecondDifferentialLightBandValid = MazeMap::TryComputeRobustSignalBandFromSamples(
            _wallSecondDifferentialSamples,
            _wallSampleCount,
            MazeMap::Config::kWallCalibrationScaledMadMultiplier,
            median,
            _wallSecondDifferentialLightBandLow,
            _wallSecondDifferentialLightBandHigh);

        _wallSampleFirstSensor = nullptr;
        _wallSampleSecondSensor = nullptr;
        _wallSampleActive = false;
        _wallSampleAmbientCaptured = false;
    }

    void StartupCalibration::ResetWallSamplingState() noexcept
    {
        _wallSampleFirstSensor = nullptr;
        _wallSampleSecondSensor = nullptr;
        _wallFirstDifferentialSamples.fill(0.0f);
        _wallSecondDifferentialSamples.fill(0.0f);
        _wallSampleCount = 0U;
        _wallFirstMeasuredCount = 0U;
        _wallFirstAmbientCount = 0U;
        _wallFirstDifferentialCount = 0U;
        _wallSecondMeasuredCount = 0U;
        _wallSecondAmbientCount = 0U;
        _wallSecondDifferentialCount = 0U;
        _wallSampleTicksRemaining = 0U;
        _wallSampleActive = false;
        _wallSampleAmbientCaptured = false;
        _wallSampleMeasuredValueFromRawDistance = false;
        _wallSampleTimedOut = false;
        _wallFirstMeasuredSum = 0.0;
        _wallFirstAmbientSum = 0.0;
        _wallFirstDifferentialSum = 0.0;
        _wallSecondMeasuredSum = 0.0;
        _wallSecondAmbientSum = 0.0;
        _wallSecondDifferentialSum = 0.0;
        _wallFirstMeasuredValue = 0.0f;
        _wallFirstAmbientLight = 0.0f;
        _wallFirstDifferentialLight = 0.0f;
        _wallFirstDifferentialLightBandValid = false;
        _wallFirstDifferentialLightBandLow = 0.0f;
        _wallFirstDifferentialLightBandHigh = 0.0f;
        _wallSecondMeasuredValue = 0.0f;
        _wallSecondAmbientLight = 0.0f;
        _wallSecondDifferentialLight = 0.0f;
        _wallSecondDifferentialLightBandValid = false;
        _wallSecondDifferentialLightBandLow = 0.0f;
        _wallSecondDifferentialLightBandHigh = 0.0f;
    }

    void StartupCalibration::AccumulateFiniteWallValue(
        const float sample,
        double& sum,
        std::uint16_t& count) noexcept
    {
        if (!std::isfinite(sample))
        {
            return;
        }

        sum += static_cast<double>(sample);
        ++count;
    }

    float StartupCalibration::AverageWallCalibrationSum(const double sum, const std::uint16_t count) noexcept
    {
        if (count == 0U)
        {
            return 0.0f;
        }

        return static_cast<float>(sum / static_cast<double>(count));
    }

    bool StartupCalibration::StoreSideReference(
        const MazeMap::RelativeDirection side,
        const float measuredValue,
        const float ambientLight,
        const float differentialLight,
        const bool differentialLightBandValid,
        const float differentialLightBandLow,
        const float differentialLightBandHigh,
        const float actualDistanceM) noexcept
    {
        if (!(std::isfinite(actualDistanceM) &&
            (actualDistanceM > 0.0f) &&
            std::isfinite(measuredValue) &&
            (measuredValue > 0.0f) &&
            std::isfinite(differentialLight) &&
            (differentialLight > 0.0f) &&
            gWallDistanceCalibration.AddSidePoint(side, measuredValue, actualDistanceM, ambientLight)))
        {
            return false;
        }

        gWallDistanceCalibration.SetSideWallReferenceDifferentialLight(side, differentialLight);
        gWallDistanceCalibration.SetSideWallReferenceDistanceM(side, actualDistanceM);
        if (differentialLightBandValid &&
            IsValidPositiveBand(differentialLightBandLow, differentialLightBandHigh))
        {
            gWallDistanceCalibration.SetSideWallReferenceDifferentialLightBand(
                side,
                differentialLightBandLow,
                differentialLightBandHigh);
        }

        RefreshSensorsCalibrated();
        return true;
    }

    bool StartupCalibration::StoreSideBaseline(
        const MazeMap::RelativeDirection side,
        const float differentialLight,
        const bool differentialLightBandValid,
        const float differentialLightBandLow,
        const float differentialLightBandHigh) noexcept
    {
        if (!(std::isfinite(differentialLight) && (differentialLight >= 0.0f)))
        {
            return false;
        }

        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(side, differentialLight);
        if (differentialLightBandValid &&
            IsValidNonNegativeBand(differentialLightBandLow, differentialLightBandHigh))
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                side,
                differentialLightBandLow,
                differentialLightBandHigh);
        }

        RefreshSensorsCalibrated();
        return true;
    }

    bool StartupCalibration::StoreFrontLeftBaseline(
        const float differentialLight,
        const bool differentialLightBandValid,
        const float differentialLightBandLow,
        const float differentialLightBandHigh) noexcept
    {
        if (!(std::isfinite(differentialLight) &&
            (differentialLight >= 0.0f) &&
            differentialLightBandValid &&
            IsValidNonNegativeBand(differentialLightBandLow, differentialLightBandHigh)))
        {
            return false;
        }

        gWallDistanceCalibration.SetFrontLeftWallBaselineDifferentialLight(differentialLight);
        gWallDistanceCalibration.SetFrontLeftWallBaselineDifferentialLightBand(
            differentialLightBandLow,
            differentialLightBandHigh);
        RefreshSensorsCalibrated();
        return true;
    }

    bool StartupCalibration::StoreFrontRightBaseline(
        const float differentialLight,
        const bool differentialLightBandValid,
        const float differentialLightBandLow,
        const float differentialLightBandHigh) noexcept
    {
        if (!(std::isfinite(differentialLight) &&
            (differentialLight >= 0.0f) &&
            differentialLightBandValid &&
            IsValidNonNegativeBand(differentialLightBandLow, differentialLightBandHigh)))
        {
            return false;
        }

        gWallDistanceCalibration.SetFrontRightWallBaselineDifferentialLight(differentialLight);
        gWallDistanceCalibration.SetFrontRightWallBaselineDifferentialLightBand(
            differentialLightBandLow,
            differentialLightBandHigh);
        RefreshSensorsCalibrated();
        return true;
    }
}
