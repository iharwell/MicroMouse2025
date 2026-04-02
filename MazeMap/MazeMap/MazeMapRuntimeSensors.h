#pragma once
#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeSignalHelpers.h"
// Private sensor pipeline implementations for the MazeMap application runtime.
class SensorSuite
{
public:
    SensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _gyroBiasRadps(0.0f)
        , _frontLeft{}
        , _frontRight{}
        , _sideLeft{}
        , _sideRight{}
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
        , _accelBiasXG(0.0f)
        , _accelBiasYG(0.0f)
        , _frontLeftWallState(false)
        , _frontRightWallState(false)
        , _frontWallUsesFallbackDetection(false)
        , _frontLeftWallSignalInitialized(false)
        , _frontRightWallSignalInitialized(false)
        , _sideLeftWallSignalInitialized(false)
        , _sideRightWallSignalInitialized(false)
        , _sideLeftPreviousSignalRise(0.0f)
        , _sideRightPreviousSignalRise(0.0f)
        , _sideLeftPreviousSignalRiseValid(false)
        , _sideRightPreviousSignalRiseValid(false)
        , _accelBiasInitialized(false)
    {
    }

    bool Begin()
    {
        _frontLeft = {};
        _frontRight = {};
        _sideLeft = {};
        _sideRight = {};
        _frontLeftWallSignalFiltered = 0.0f;
        _frontRightWallSignalFiltered = 0.0f;
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _frontLeftInputAverage.Clear();
        _frontRightInputAverage.Clear();
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _frontLeftWallState = false;
        _frontRightWallState = false;
        _frontWallUsesFallbackDetection = false;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _sideLeftPreviousSignalRise = 0.0f;
        _sideRightPreviousSignalRise = 0.0f;
        _sideLeftPreviousSignalRiseValid = false;
        _sideRightPreviousSignalRiseValid = false;
        _accelBiasXG = 0.0f;
        _accelBiasYG = 0.0f;
        _accelBiasInitialized = false;
        InitializeWallSensorLedOffState();
        bool ok = true;
#if defined(ARDUINO_TEENSY41)
        Serial.println("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.IMU_BL.Begin();
        if (!imuBackLeftOk)
        {
            Serial.print("IMU_BL init failed (");
            Serial.print(_vehicle.IMU_BL.GetLastBeginFailureReasonName());
            Serial.print("), WHO_AM_I=0x");
            const uint8_t whoAmI = _vehicle.IMU_BL.GetLastWhoAmI();
            PrintHexByte(whoAmI);
            Serial.print(", INT1_pullup=");
            Serial.print(ReadDrivenLowPinWithPullup(Pins::IMU_INT_1B) == LOW ? "low" : "high");
            Serial.print(", WHO_AM_I@mode3/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE3));
            Serial.print(", WHO_AM_I@mode0/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE0));
            Serial.println();
        }
        ok = imuBackLeftOk && ok;
#endif
        if (ok)
        {
            ok = CalibrateGyroBias(Config::kControlPeriodUs, true) && ok;
        }
        return ok;
    }

    bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime)
    {
        return CalibrateStationaryBackLeftGyroBias(
            _vehicle,
            controlPeriodUs,
            enableAccelRuntime,
            _gyroBiasRadps,
            &_accelBiasXG,
            &_accelBiasYG,
            &_accelBiasInitialized,
            Config::kMissionRuntimeAccelFilterFreq);
    }

    void ResetSideWallMemory() noexcept
    {
        _sideLeft = {};
        _sideRight = {};
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _sideLeftPreviousSignalRise = 0.0f;
        _sideRightPreviousSignalRise = 0.0f;
        _sideLeftPreviousSignalRiseValid = false;
        _sideRightPreviousSignalRiseValid = false;
    }

    SensorSnapshot Capture(bool stationary, const PoseEstimate& pose)
    {
        return Capture(
            stationary,
            pose,
            [](SensorSnapshot&, auto&&, auto&& captureImu) noexcept
            {
                captureImu();
            },
            []() noexcept {});
    }

    template <typename TEstimatorWork, typename TFlushLogs>
    SensorSnapshot Capture(
        bool stationary,
        const PoseEstimate& pose,
        TEstimatorWork&& estimatorWork,
        TFlushLogs&& flushLogs)
    {
        SensorSnapshot snapshot{};
        AsyncWallSensorSweepRead wallRead{};
        StartAsyncWallSensorSweepRead(
            _vehicle.FrontLeft,
            _frontLeftLedOffCommandUs,
            _vehicle.FrontRight,
            _frontRightLedOffCommandUs,
            _vehicle.SideLeft,
            _sideLeftLedOffCommandUs,
            _vehicle.SideRight,
            _sideRightLedOffCommandUs,
            wallRead);

        bool imuCaptured = false;
        auto serviceWallRead = [&wallRead]() noexcept
        {
            (void)ServiceAsyncWallSensorSweepRead(wallRead);
        };
        auto captureImu = [&]() noexcept
        {
            if (!imuCaptured)
            {
                CaptureInertialSnapshot(stationary, snapshot);
                imuCaptured = true;
            }
            serviceWallRead();
        };

        estimatorWork(snapshot, serviceWallRead, captureImu);
        if (!imuCaptured)
        {
            captureImu();
        }

        serviceWallRead();
        if (wallRead.active)
        {
            flushLogs();
            serviceWallRead();
            if (wallRead.active)
            {
                CompleteAsyncWallSensorSweepRead(wallRead);
            }
        }

        _frontLeftLedOffCommandUs = wallRead.nextFrontLeftLedOffCommandUs;
        _frontRightLedOffCommandUs = wallRead.nextFrontRightLedOffCommandUs;
        _sideLeftLedOffCommandUs = wallRead.nextSideLeftLedOffCommandUs;
        _sideRightLedOffCommandUs = wallRead.nextSideRightLedOffCommandUs;

        const uint32_t nextAmbientReadyUs = NextAsyncWallSensorSweepAmbientReadyUs(wallRead);
        while (static_cast<int32_t>(micros() - nextAmbientReadyUs) < 0)
        {
            delayMicroseconds(5);
        }

        const WallSensorCalibrationInput frontLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontLeft, wallRead.frontLeftSample);
        const WallSensorCalibrationInput frontRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontRight, wallRead.frontRightSample);
        const WallSensorCalibrationInput sideLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideLeft, wallRead.sideLeftSample);
        const WallSensorCalibrationInput sideRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideRight, wallRead.sideRightSample);
        const WallSensorCalibrationInput frontLeftInput = _frontLeftInputAverage.PushAndAverage(frontLeftRawInput);
        const WallSensorCalibrationInput frontRightInput = _frontRightInputAverage.PushAndAverage(frontRightRawInput);
        const WallSensorCalibrationInput sideLeftInput = _sideLeftInputAverage.PushAndAverage(sideLeftRawInput);
        const WallSensorCalibrationInput sideRightInput = _sideRightInputAverage.PushAndAverage(sideRightRawInput);
        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        _wallCalibration.TryComputeSideWallDistanceThresholds(
            Config::kSideWallDistanceLatchFractionOfCalibration,
            Config::kSideWallDistanceReleaseFractionOfCalibration,
            sideWallOnThresholdM,
            sideWallOffThresholdM);
        snapshot.frontLeftDistanceM = UpdateChannelFromMeasuredDistance(
            _frontLeft,
            _wallCalibration.Apply(WallSensorId::FrontLeft, frontLeftInput.measuredValue, frontLeftInput.fallbackDistanceM));
        snapshot.frontRightDistanceM = UpdateChannelFromMeasuredDistance(
            _frontRight,
            _wallCalibration.Apply(WallSensorId::FrontRight, frontRightInput.measuredValue, frontRightInput.fallbackDistanceM));
        snapshot.frontLeftDifferentialLight = frontLeftInput.differentialLight;
        snapshot.frontRightDifferentialLight = frontRightInput.differentialLight;
        float sideLeftDistanceM = _wallCalibration.Apply(
            WallSensorId::SideLeft,
            sideLeftInput.measuredValue,
            sideLeftInput.fallbackDistanceM);
        float sideRightDistanceM = _wallCalibration.Apply(
            WallSensorId::SideRight,
            sideRightInput.measuredValue,
            sideRightInput.fallbackDistanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftDistanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightDistanceM);
        snapshot.sideLeftDistanceM = UpdateChannelFromMeasuredDistance(
            _sideLeft,
            sideLeftDistanceM);
        snapshot.sideRightDistanceM = UpdateChannelFromMeasuredDistance(
            _sideRight,
            sideRightDistanceM);
        snapshot.sideLeftDifferentialLight = sideLeftInput.differentialLight;
        snapshot.sideRightDifferentialLight = sideRightInput.differentialLight;

        snapshot.frontWall = UpdateFrontWallState(
            frontLeftInput.ambientLight,
            frontLeftInput.measuredValue,
            frontRightInput.ambientLight,
            frontRightInput.measuredValue,
            (std::min)(snapshot.frontLeftDistanceM, snapshot.frontRightDistanceM));
        snapshot.frontLeftWall = _frontLeftWallState;
        snapshot.frontRightWall = _frontRightWallState;
        snapshot.frontWallObservationValid = false;
        snapshot.frontWallUsesFallbackDetection = _frontWallUsesFallbackDetection;
        snapshot.frontWallUsesCharacterizationDetection = false;
        const bool sideLeftWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideLeft);
        const bool sideRightWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideRight);
        float sideLeftSignalRise = 0.0f;
        float sideLeftLatchRiseThreshold = 0.0f;
        float sideLeftMissRiseThreshold = 0.0f;
        const bool sideLeftSignalMetricsValid = TryComputeSideSignalMetrics(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold,
            sideLeftMissRiseThreshold);
        float sideRightSignalRise = 0.0f;
        float sideRightLatchRiseThreshold = 0.0f;
        float sideRightMissRiseThreshold = 0.0f;
        const bool sideRightSignalMetricsValid = TryComputeSideSignalMetrics(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightSignalRise,
            sideRightLatchRiseThreshold,
            sideRightMissRiseThreshold);
        const bool sideLeftSignalClassifiable =
            sideLeftSignalMetricsValid &&
            ((sideLeftSignalRise >= sideLeftLatchRiseThreshold) ||
                (sideLeftSignalRise <= sideLeftMissRiseThreshold));
        const bool sideRightSignalClassifiable =
            sideRightSignalMetricsValid &&
            ((sideRightSignalRise >= sideRightLatchRiseThreshold) ||
                (sideRightSignalRise <= sideRightMissRiseThreshold));
        const bool sideLeftFallbackValid =
            std::isfinite(sideLeftInput.fallbackDistanceM) &&
            sideLeftInput.fallbackDistanceM > 0.0f;
        const bool sideRightFallbackValid =
            std::isfinite(sideRightInput.fallbackDistanceM) &&
            sideRightInput.fallbackDistanceM > 0.0f;

        const bool sideLeftObservationEligible =
            sideLeftWindowValid &&
            (sideLeftSignalClassifiable || sideLeftFallbackValid);
        const bool sideRightObservationEligible =
            sideRightWindowValid &&
            (sideRightSignalClassifiable || sideRightFallbackValid);
        const bool sideLeftControlRangeValid =
            sideLeftSignalMetricsValid ?
            (sideLeftSignalRise >= sideLeftLatchRiseThreshold) :
            (sideLeftFallbackValid && sideLeftInput.fallbackDistanceM < sideWallOffThresholdM);
        const bool sideRightControlRangeValid =
            sideRightSignalMetricsValid ?
            (sideRightSignalRise >= sideRightLatchRiseThreshold) :
            (sideRightFallbackValid && sideRightInput.fallbackDistanceM < sideWallOffThresholdM);
        snapshot.leftTransitionDetected = DetectTransitionFromSignalRise(
            sideLeftWindowValid,
            sideLeftSignalMetricsValid,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold * Config::kSideWallTransitionSignalFractionOfLatch,
            _sideLeftPreviousSignalRise,
            _sideLeftPreviousSignalRiseValid);
        snapshot.rightTransitionDetected = DetectTransitionFromSignalRise(
            sideRightWindowValid,
            sideRightSignalMetricsValid,
            sideRightSignalRise,
            sideRightLatchRiseThreshold * Config::kSideWallTransitionSignalFractionOfLatch,
            _sideRightPreviousSignalRise,
            _sideRightPreviousSignalRiseValid);
        snapshot.leftWallObservationWindowValid = sideLeftObservationEligible;
        snapshot.rightWallObservationWindowValid = sideRightObservationEligible;
        snapshot.leftDistanceValidForControl =
            sideLeftObservationEligible &&
            !snapshot.leftTransitionDetected &&
            sideLeftControlRangeValid;
        snapshot.rightDistanceValidForControl =
            sideRightObservationEligible &&
            !snapshot.rightTransitionDetected &&
            sideRightControlRangeValid;
        snapshot.leftWall = UpdateSideWallState(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _sideLeft.wall);
        snapshot.rightWall = UpdateSideWallState(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _sideRight.wall);
        snapshot.leftWallObservation = ComputeSideWallObservationHit(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideLeftObservationEligible);
        snapshot.rightWallObservation = ComputeSideWallObservationHit(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideRightObservationEligible);
        snapshot.frontSkewM = snapshot.frontLeftDistanceM - snapshot.frontRightDistanceM;
        snapshot.corridorErrorM = MazeMapApp::Internal::Runtime::ComputeCorridorError(
            snapshot.sideLeftDistanceM,
            snapshot.sideRightDistanceM,
            snapshot.leftDistanceValidForControl,
            snapshot.rightDistanceValidForControl,
            _wallCalibration.GetExpectedSideWallDistanceM());
        return snapshot;
    }

    float GetGyroSensitivityMdpsPerLsb() const
    {
        return _vehicle.IMU_BL.GyroSensitivityMdpsPerLsb();
    }

    float GetAccelSensitivityMgPerLsb() const
    {
        return _vehicle.IMU_BL.AccelSensitivityMgPerLsb();
    }

    float GetGyroBiasRadps() const
    {
        return _gyroBiasRadps;
    }

    bool HasAccelBias() const
    {
        return _accelBiasInitialized;
    }

    float GetAccelBiasXG() const
    {
        return _accelBiasXG;
    }

    float GetAccelBiasYG() const
    {
        return _accelBiasYG;
    }

private:
    void InitializeWallSensorLedOffState() noexcept
    {
        _vehicle.FrontLeft.SetLedEnabled(false);
        _vehicle.FrontRight.SetLedEnabled(false);
        _vehicle.SideLeft.SetLedEnabled(false);
        _vehicle.SideRight.SetLedEnabled(false);
        const uint32_t nowUs = micros();
        _frontLeftLedOffCommandUs = nowUs;
        _frontRightLedOffCommandUs = nowUs;
        _sideLeftLedOffCommandUs = nowUs;
        _sideRightLedOffCommandUs = nowUs;
    }

    void CaptureInertialSnapshot(bool stationary, SensorSnapshot& snapshot)
    {
        const MazeMap::ImuExtrinsics imuExtrinsics = MazeMap::Vehicle::GetBackLeftImuExtrinsics();
#if defined(ARDUINO_TEENSY41)
        const MazeMap::Vehicle::ImuBackLeft::Axes accel = _vehicle.IMU_BL.ReadAccel();
        const Eigen::Vector2f accelImuG(
            _vehicle.IMU_BL.AccelRawToG(accel.x),
            _vehicle.IMU_BL.AccelRawToG(accel.y));
        const Eigen::Vector2f accelBodyG = imuExtrinsics.accelBodyFromImu * accelImuG;
        snapshot.accelBiasValid = _accelBiasInitialized;
        if (_accelBiasInitialized && stationary)
        {
            _accelBiasXG = (0.998f * _accelBiasXG) + (0.002f * accelBodyG.x());
            _accelBiasYG = (0.998f * _accelBiasYG) + (0.002f * accelBodyG.y());
        }
        if (_accelBiasInitialized)
        {
            const float accelDeltaXG = accelBodyG.x() - _accelBiasXG;
            const float accelDeltaYG = accelBodyG.y() - _accelBiasYG;
            snapshot.accelBodyXMps2 = kStandardGravityMps2 * accelDeltaXG;
            snapshot.accelBodyYMps2 = kStandardGravityMps2 * accelDeltaYG;
            snapshot.planarAccelMps2 =
                kStandardGravityMps2 * MazeMap::Math::Sqrtf((accelDeltaXG * accelDeltaXG) + (accelDeltaYG * accelDeltaYG));
        }
        else
        {
            snapshot.accelBodyXMps2 = 0.0f;
            snapshot.accelBodyYMps2 = 0.0f;
            snapshot.planarAccelMps2 = 0.0f;
        }
#else
        snapshot.accelBodyXMps2 = 0.0f;
        snapshot.accelBodyYMps2 = 0.0f;
        snapshot.planarAccelMps2 = 0.0f;
        snapshot.accelBiasValid = false;
#endif

        const float rawGyroRadps = imuExtrinsics.gyroZSign * ReadGyroZRadpsRaw();
        if (stationary &&
            MazeMap::ShouldUpdateGyroBiasFromStationarySample(rawGyroRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
        {
            _gyroBiasRadps = (0.995f * _gyroBiasRadps) + (0.005f * rawGyroRadps);
        }
        snapshot.gyroRawRadps = rawGyroRadps;
        snapshot.gyroBiasRadps = _gyroBiasRadps;
        snapshot.gyroRadps = rawGyroRadps - _gyroBiasRadps;
    }

    struct FilteredIrChannel
    {
        float filteredDistanceM = 0.20f;
        bool wall = false;
        bool initialized = false;
    };

    MazeMap::Vehicle& _vehicle;
    WallDistanceCalibration& _wallCalibration;
    float _gyroBiasRadps;
    FilteredIrChannel _frontLeft;
    FilteredIrChannel _frontRight;
    FilteredIrChannel _sideLeft;
    FilteredIrChannel _sideRight;
    float _frontLeftWallSignalFiltered;
    float _frontRightWallSignalFiltered;
    float _sideLeftWallSignalFiltered;
    float _sideRightWallSignalFiltered;
    float _accelBiasXG;
    float _accelBiasYG;
    uint32_t _frontLeftLedOffCommandUs = 0UL;
    uint32_t _frontRightLedOffCommandUs = 0UL;
    uint32_t _sideLeftLedOffCommandUs = 0UL;
    uint32_t _sideRightLedOffCommandUs = 0UL;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
    bool _frontLeftWallState;
    bool _frontRightWallState;
    bool _frontWallUsesFallbackDetection;
    bool _frontLeftWallSignalInitialized;
    bool _frontRightWallSignalInitialized;
    bool _sideLeftWallSignalInitialized;
    bool _sideRightWallSignalInitialized;
    float _sideLeftPreviousSignalRise;
    float _sideRightPreviousSignalRise;
    bool _sideLeftPreviousSignalRiseValid;
    bool _sideRightPreviousSignalRiseValid;
    bool _accelBiasInitialized;

    bool TryComputeSideSignalMetrics(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float& signalRise,
        float& latchRiseThreshold,
        float& missRiseThreshold) const
    {
        signalRise = 0.0f;
        latchRiseThreshold = 0.0f;
        missRiseThreshold = 0.0f;

        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (!_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                latchRiseThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return false;
        }

        signalRise = MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
            measuredDifferentialLight,
            signalBaseline);
        missRiseThreshold = Config::kWallMapMissSignalFractionOfLatch * latchRiseThreshold;
        return
            std::isfinite(signalRise) &&
            std::isfinite(latchRiseThreshold) &&
            std::isfinite(missRiseThreshold) &&
            latchRiseThreshold > 0.0f &&
            missRiseThreshold >= 0.0f;
    }

    static bool DetectTransitionFromSignalRise(
        bool windowValid,
        bool signalMetricsValid,
        float signalRise,
        float transitionThreshold,
        float& previousSignalRise,
        bool& previousValid) noexcept
    {
        bool transitionDetected = false;
        const bool currentValid =
            windowValid &&
            signalMetricsValid &&
            std::isfinite(signalRise) &&
            std::isfinite(transitionThreshold) &&
            transitionThreshold > 0.0f;
        if (currentValid && previousValid)
        {
            transitionDetected = std::fabs(signalRise - previousSignalRise) >= transitionThreshold;
        }

        previousSignalRise = currentValid ? signalRise : 0.0f;
        previousValid = currentValid;
        return transitionDetected;
    }

    static float UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, float measuredDistanceM)
    {
        channel.filteredDistanceM = measuredDistanceM;
        channel.initialized = true;
        return measuredDistanceM;
    }

    static float UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, float measuredDistanceM, float onThresholdM, float offThresholdM)
    {
        (void)offThresholdM;
        const float currentDistanceM = UpdateChannelFromMeasuredDistance(channel, measuredDistanceM);
        channel.wall = currentDistanceM < onThresholdM;
        return currentDistanceM;
    }

    bool ComputeSideWallObservationHit(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        bool detectionWindowValid) const
    {
        if (!detectionWindowValid)
        {
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                       measuredDifferentialLight,
                       signalBaseline) >= onMeasuredThreshold;
        }

        return std::isfinite(fallbackDistanceM) && fallbackDistanceM < onThresholdM;
    }

    bool UpdateSideWallState(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        float offThresholdM,
        bool detectionWindowValid,
        float& filteredSignal,
        bool& signalInitialized,
        bool& currentState)
    {
        if (!detectionWindowValid)
        {
            filteredSignal = 0.0f;
            signalInitialized = false;
            currentState = false;
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                    measuredDifferentialLight,
                    signalBaseline),
                onMeasuredThreshold,
                offMeasuredThreshold,
                filteredSignal,
                currentState,
                signalInitialized);
        }

        signalInitialized = false;
        currentState = HysteresisWall(
            currentState,
            fallbackDistanceM,
            onThresholdM,
            offThresholdM);
        return currentState;
    }

    bool UpdateFrontWallState(
        float leftAmbientLight,
        float leftMeasuredDifferentialLight,
        float rightAmbientLight,
        float rightMeasuredDifferentialLight,
        float fallbackDistanceM)
    {
        float leftOnMeasuredThreshold = 0.0f;
        float leftOffMeasuredThreshold = 0.0f;
        float leftSignalBaseline = 0.0f;
        float rightOnMeasuredThreshold = 0.0f;
        float rightOffMeasuredThreshold = 0.0f;
        float rightSignalBaseline = 0.0f;
        const bool haveLeftThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontLeft,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                leftAmbientLight,
                leftOnMeasuredThreshold,
                leftOffMeasuredThreshold,
                leftSignalBaseline);
        const bool haveRightThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontRight,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                rightAmbientLight,
                rightOnMeasuredThreshold,
                rightOffMeasuredThreshold,
                rightSignalBaseline);

        if (haveLeftThreshold || haveRightThreshold)
        {
            _frontWallUsesFallbackDetection = false;
            if (haveLeftThreshold)
            {
                MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                        leftMeasuredDifferentialLight,
                        leftSignalBaseline),
                    leftOnMeasuredThreshold,
                    leftOffMeasuredThreshold,
                    _frontLeftWallSignalFiltered,
                    _frontLeftWallState,
                    _frontLeftWallSignalInitialized);
            }
            else
            {
                _frontLeftWallState = false;
                _frontLeftWallSignalInitialized = false;
            }

            if (haveRightThreshold)
            {
                MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                        rightMeasuredDifferentialLight,
                        rightSignalBaseline),
                    rightOnMeasuredThreshold,
                    rightOffMeasuredThreshold,
                    _frontRightWallSignalFiltered,
                    _frontRightWallState,
                    _frontRightWallSignalInitialized);
            }
            else
            {
                _frontRightWallState = false;
                _frontRightWallSignalInitialized = false;
            }

            return _frontLeftWallState || _frontRightWallState;
        }

        const bool fallbackState = HysteresisWall(
            _frontLeftWallState || _frontRightWallState,
            fallbackDistanceM,
            Config::kFrontWallOnThresholdM,
            Config::kFrontWallOffThresholdM);
        _frontWallUsesFallbackDetection = true;
        _frontLeftWallState = fallbackState;
        _frontRightWallState = fallbackState;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        return fallbackState;
    }

    float ReadGyroZRadpsRaw()
    {
        return ReadBackLeftGyroZRadpsRaw(_vehicle);
    }
};


class DiagnosticSensorSuite
{
public:
    DiagnosticSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _gyroBiasRadps(0.0f)
        , _accelBiasXG(0.0f)
        , _accelBiasYG(0.0f)
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
        , _frontLeftWallState(false)
        , _frontRightWallState(false)
        , _leftWallState(false)
        , _rightWallState(false)
        , _frontLeftWallSignalInitialized(false)
        , _frontRightWallSignalInitialized(false)
        , _sideLeftWallSignalInitialized(false)
        , _sideRightWallSignalInitialized(false)
        , _accelBiasInitialized(false)
    {
    }

    bool Begin(unsigned long controlPeriodUs = DiagnosticConfig::kControlPeriodUs)
    {
        bool ok = true;
        _frontLeftWallSignalFiltered = 0.0f;
        _frontRightWallSignalFiltered = 0.0f;
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _frontLeftInputAverage.Clear();
        _frontRightInputAverage.Clear();
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _frontLeftWallState = false;
        _frontRightWallState = false;
        _leftWallState = false;
        _rightWallState = false;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _accelBiasXG = 0.0f;
        _accelBiasYG = 0.0f;
        _accelBiasInitialized = false;
        InitializeWallSensorLedOffState();
#if defined(ARDUINO_TEENSY41)
        Serial.println("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.IMU_BL.Begin();
        if (!imuBackLeftOk)
        {
            Serial.print("IMU_BL init failed (");
            Serial.print(_vehicle.IMU_BL.GetLastBeginFailureReasonName());
            Serial.print("), WHO_AM_I=0x");
            const uint8_t whoAmI = _vehicle.IMU_BL.GetLastWhoAmI();
            PrintHexByte(whoAmI);
            Serial.print(", INT1_pullup=");
            Serial.print(ReadDrivenLowPinWithPullup(Pins::IMU_INT_1B) == LOW ? "low" : "high");
            Serial.print(", WHO_AM_I@mode3/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE3));
            Serial.print(", WHO_AM_I@mode0/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE0));
            Serial.println();
        }
        ok = imuBackLeftOk && ok;
#endif
        if (ok)
        {
            ok = CalibrateGyroBias(controlPeriodUs, true) && ok;
        }
        return ok;
    }

    bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime)
    {
        return CalibrateStationaryBackLeftGyroBias(
            _vehicle,
            controlPeriodUs,
            enableAccelRuntime,
            _gyroBiasRadps,
            &_accelBiasXG,
            &_accelBiasYG,
            &_accelBiasInitialized,
            Config::kMissionRuntimeAccelFilterFreq);
    }

    DiagnosticSensorSnapshot Capture(bool stationary, const PoseEstimate& pose)
    {
        return Capture(
            stationary,
            pose,
            [](DiagnosticSensorSnapshot&, auto&&, auto&& captureImu) noexcept
            {
                captureImu();
            },
            []() noexcept {});
    }

    template <typename TEstimatorWork, typename TFlushLogs>
    DiagnosticSensorSnapshot Capture(
        bool stationary,
        const PoseEstimate& pose,
        TEstimatorWork&& estimatorWork,
        TFlushLogs&& flushLogs)
    {
        DiagnosticSensorSnapshot snapshot{};
        AsyncWallSensorSweepRead wallRead{};
        StartAsyncWallSensorSweepRead(
            _vehicle.FrontLeft,
            _frontLeftLedOffCommandUs,
            _vehicle.FrontRight,
            _frontRightLedOffCommandUs,
            _vehicle.SideLeft,
            _sideLeftLedOffCommandUs,
            _vehicle.SideRight,
            _sideRightLedOffCommandUs,
            wallRead);

        bool imuCaptured = false;
        auto serviceWallRead = [&wallRead]() noexcept
        {
            (void)ServiceAsyncWallSensorSweepRead(wallRead);
        };
        auto captureImu = [&]() noexcept
        {
            if (!imuCaptured)
            {
                CaptureInertialSnapshot(stationary, snapshot);
                imuCaptured = true;
            }
            serviceWallRead();
        };

        estimatorWork(snapshot, serviceWallRead, captureImu);
        if (!imuCaptured)
        {
            captureImu();
        }

        serviceWallRead();
        if (wallRead.active)
        {
            flushLogs();
            serviceWallRead();
            if (wallRead.active)
            {
                CompleteAsyncWallSensorSweepRead(wallRead);
            }
        }

        _frontLeftLedOffCommandUs = wallRead.nextFrontLeftLedOffCommandUs;
        _frontRightLedOffCommandUs = wallRead.nextFrontRightLedOffCommandUs;
        _sideLeftLedOffCommandUs = wallRead.nextSideLeftLedOffCommandUs;
        _sideRightLedOffCommandUs = wallRead.nextSideRightLedOffCommandUs;

        const uint32_t nextAmbientReadyUs = NextAsyncWallSensorSweepAmbientReadyUs(wallRead);
        while (static_cast<int32_t>(micros() - nextAmbientReadyUs) < 0)
        {
            delayMicroseconds(5);
        }

        const WallSensorCalibrationInput frontLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontLeft, wallRead.frontLeftSample);
        const WallSensorCalibrationInput frontRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontRight, wallRead.frontRightSample);
        const WallSensorCalibrationInput sideLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideLeft, wallRead.sideLeftSample);
        const WallSensorCalibrationInput sideRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideRight, wallRead.sideRightSample);
        const WallSensorCalibrationInput frontLeftInput = _frontLeftInputAverage.PushAndAverage(frontLeftRawInput);
        const WallSensorCalibrationInput frontRightInput = _frontRightInputAverage.PushAndAverage(frontRightRawInput);
        const WallSensorCalibrationInput sideLeftInput = _sideLeftInputAverage.PushAndAverage(sideLeftRawInput);
        const WallSensorCalibrationInput sideRightInput = _sideRightInputAverage.PushAndAverage(sideRightRawInput);
        snapshot.frontLeft = BuildWallSensorTelemetry(WallSensorId::FrontLeft, frontLeftInput);
        snapshot.frontRight = BuildWallSensorTelemetry(WallSensorId::FrontRight, frontRightInput);
        snapshot.sideLeft = BuildWallSensorTelemetry(WallSensorId::SideLeft, sideLeftInput);
        snapshot.sideRight = BuildWallSensorTelemetry(WallSensorId::SideRight, sideRightInput);
        snapshot.frontTiming = frontLeftRawInput.timing;
        snapshot.leftTiming = sideLeftInput.timing;
        snapshot.rightTiming = sideRightInput.timing;
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            snapshot.sideLeft.distanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            snapshot.sideRight.distanceM);
        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        _wallCalibration.TryComputeSideWallDistanceThresholds(
            Config::kSideWallDistanceLatchFractionOfCalibration,
            Config::kSideWallDistanceReleaseFractionOfCalibration,
            sideWallOnThresholdM,
            sideWallOffThresholdM);

        snapshot.frontWall = UpdateFrontWallState(
            frontLeftInput.ambientLight,
            frontLeftInput.measuredValue,
            frontRightInput.ambientLight,
            frontRightInput.measuredValue,
            (std::min)(snapshot.frontLeft.distanceM, snapshot.frontRight.distanceM));
        const bool sideLeftWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideLeft);
        const bool sideRightWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideRight);
        snapshot.leftDistanceValidForControl =
            sideLeftWindowValid &&
            IsCalibratedSideDistanceValidForControl(
                _wallCalibration,
                WallSensorId::SideLeft,
                sideLeftInput.differentialLight);
        snapshot.rightDistanceValidForControl =
            sideRightWindowValid &&
            IsCalibratedSideDistanceValidForControl(
                _wallCalibration,
                WallSensorId::SideRight,
                sideRightInput.differentialLight);
        snapshot.leftWall = UpdateSideWallState(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _leftWallState);
        snapshot.rightWall = UpdateSideWallState(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _rightWallState);

        snapshot.frontLeft.wall = snapshot.frontWall && (snapshot.frontLeft.distanceM <= snapshot.frontRight.distanceM);
        snapshot.frontRight.wall = snapshot.frontWall && (snapshot.frontRight.distanceM <= snapshot.frontLeft.distanceM);
        snapshot.sideLeft.wall = snapshot.leftWall;
        snapshot.sideRight.wall = snapshot.rightWall;
        snapshot.frontSkewM = snapshot.frontLeft.distanceM - snapshot.frontRight.distanceM;
        snapshot.corridorErrorM = MazeMapApp::Internal::Runtime::ComputeCorridorError(
            snapshot.sideLeft.distanceM,
            snapshot.sideRight.distanceM,
            snapshot.leftDistanceValidForControl,
            snapshot.rightDistanceValidForControl,
            _wallCalibration.GetExpectedSideWallDistanceM());
        return snapshot;
    }

    float GetGyroSensitivityMdpsPerLsb() const
    {
        return _vehicle.IMU_BL.GyroSensitivityMdpsPerLsb();
    }

    float GetAccelSensitivityMgPerLsb() const
    {
        return _vehicle.IMU_BL.AccelSensitivityMgPerLsb();
    }

    float GetGyroBiasRadps() const
    {
        return _gyroBiasRadps;
    }

    bool HasAccelBias() const
    {
        return _accelBiasInitialized;
    }

    float GetAccelBiasXG() const
    {
        return _accelBiasXG;
    }

    float GetAccelBiasYG() const
    {
        return _accelBiasYG;
    }

    float GetPlanarAccelMps2(const DiagnosticSensorSnapshot& snapshot) const
    {
        if (!_accelBiasInitialized)
        {
            return 0.0f;
        }
        const MazeMap::ImuExtrinsics imuExtrinsics = MazeMap::Vehicle::GetBackLeftImuExtrinsics();
        const Eigen::Vector2f accelImuG(
            _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelX),
            _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelY));
        const Eigen::Vector2f accelBodyG = imuExtrinsics.accelBodyFromImu * accelImuG;
        const float accelXG = accelBodyG.x() - _accelBiasXG;
        const float accelYG = accelBodyG.y() - _accelBiasYG;
        return kStandardGravityMps2 * std::sqrt((accelXG * accelXG) + (accelYG * accelYG));
    }

private:
    void InitializeWallSensorLedOffState() noexcept
    {
        _vehicle.FrontLeft.SetLedEnabled(false);
        _vehicle.FrontRight.SetLedEnabled(false);
        _vehicle.SideLeft.SetLedEnabled(false);
        _vehicle.SideRight.SetLedEnabled(false);
        const uint32_t nowUs = micros();
        _frontLeftLedOffCommandUs = nowUs;
        _frontRightLedOffCommandUs = nowUs;
        _sideLeftLedOffCommandUs = nowUs;
        _sideRightLedOffCommandUs = nowUs;
    }

    void CaptureInertialSnapshot(bool stationary, DiagnosticSensorSnapshot& snapshot)
    {
        const MazeMap::ImuExtrinsics imuExtrinsics = MazeMap::Vehicle::GetBackLeftImuExtrinsics();
        snapshot.imuFrontRight = {};
        snapshot.imuBackLeft = CaptureImu(_vehicle.IMU_BL, Pins::IMU_INT_1B, &snapshot.imuTiming);
        const float blGyroZRadps =
            imuExtrinsics.gyroZSign *
            (_vehicle.IMU_BL.GyroRawToDps(snapshot.imuBackLeft.gyroZ) * DEG_TO_RAD_F);
        const Eigen::Vector2f accelImuG(
            _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelX),
            _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelY));
        const Eigen::Vector2f accelBodyG = imuExtrinsics.accelBodyFromImu * accelImuG;
        snapshot.accelBiasValid = _accelBiasInitialized;
        if (_accelBiasInitialized && stationary)
        {
            _accelBiasXG = (0.998f * _accelBiasXG) + (0.002f * accelBodyG.x());
            _accelBiasYG = (0.998f * _accelBiasYG) + (0.002f * accelBodyG.y());
        }
        if (_accelBiasInitialized)
        {
            snapshot.accelBodyXMps2 = kStandardGravityMps2 * (accelBodyG.x() - _accelBiasXG);
            snapshot.accelBodyYMps2 = kStandardGravityMps2 * (accelBodyG.y() - _accelBiasYG);
        }
        else
        {
            snapshot.accelBodyXMps2 = 0.0f;
            snapshot.accelBodyYMps2 = 0.0f;
        }
        snapshot.gyroRawRadps = blGyroZRadps;
        if (stationary &&
            MazeMap::ShouldUpdateGyroBiasFromStationarySample(snapshot.gyroRawRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
        {
            _gyroBiasRadps = (0.998f * _gyroBiasRadps) + (0.002f * snapshot.gyroRawRadps);
        }
        snapshot.gyroBiasRadps = _gyroBiasRadps;
        snapshot.gyroRadps = snapshot.gyroRawRadps - snapshot.gyroBiasRadps;
    }

    MazeMap::Vehicle& _vehicle;
    WallDistanceCalibration& _wallCalibration;
    float _gyroBiasRadps;
    float _accelBiasXG;
    float _accelBiasYG;
    uint32_t _frontLeftLedOffCommandUs = 0UL;
    uint32_t _frontRightLedOffCommandUs = 0UL;
    uint32_t _sideLeftLedOffCommandUs = 0UL;
    uint32_t _sideRightLedOffCommandUs = 0UL;
    float _frontLeftWallSignalFiltered;
    float _frontRightWallSignalFiltered;
    float _sideLeftWallSignalFiltered;
    float _sideRightWallSignalFiltered;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
    bool _frontLeftWallState;
    bool _frontRightWallState;
    bool _leftWallState;
    bool _rightWallState;
    bool _frontLeftWallSignalInitialized;
    bool _frontRightWallSignalInitialized;
    bool _sideLeftWallSignalInitialized;
    bool _sideRightWallSignalInitialized;
    bool _accelBiasInitialized;

    bool UpdateSideWallState(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        float offThresholdM,
        bool detectionWindowValid,
        float& filteredSignal,
        bool& signalInitialized,
        bool& currentState)
    {
        if (!detectionWindowValid)
        {
            filteredSignal = 0.0f;
            signalInitialized = false;
            currentState = false;
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                    measuredDifferentialLight,
                    signalBaseline),
                onMeasuredThreshold,
                offMeasuredThreshold,
                filteredSignal,
                currentState,
                signalInitialized);
        }

        signalInitialized = false;
        currentState = HysteresisWall(
            currentState,
            fallbackDistanceM,
            onThresholdM,
            offThresholdM);
        return currentState;
    }

    bool UpdateFrontWallState(
        float leftAmbientLight,
        float leftMeasuredDifferentialLight,
        float rightAmbientLight,
        float rightMeasuredDifferentialLight,
        float fallbackDistanceM)
    {
        float leftOnMeasuredThreshold = 0.0f;
        float leftOffMeasuredThreshold = 0.0f;
        float leftSignalBaseline = 0.0f;
        float rightOnMeasuredThreshold = 0.0f;
        float rightOffMeasuredThreshold = 0.0f;
        float rightSignalBaseline = 0.0f;
        const bool haveLeftThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontLeft,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                leftAmbientLight,
                leftOnMeasuredThreshold,
                leftOffMeasuredThreshold,
                leftSignalBaseline);
        const bool haveRightThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontRight,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                rightAmbientLight,
                rightOnMeasuredThreshold,
                rightOffMeasuredThreshold,
                rightSignalBaseline);

        if (haveLeftThreshold || haveRightThreshold)
        {
            if (haveLeftThreshold)
            {
                MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                        leftMeasuredDifferentialLight,
                        leftSignalBaseline),
                    leftOnMeasuredThreshold,
                    leftOffMeasuredThreshold,
                    _frontLeftWallSignalFiltered,
                    _frontLeftWallState,
                    _frontLeftWallSignalInitialized);
            }
            else
            {
                _frontLeftWallState = false;
                _frontLeftWallSignalInitialized = false;
            }

            if (haveRightThreshold)
            {
                MazeMapApp::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                        rightMeasuredDifferentialLight,
                        rightSignalBaseline),
                    rightOnMeasuredThreshold,
                    rightOffMeasuredThreshold,
                    _frontRightWallSignalFiltered,
                    _frontRightWallState,
                    _frontRightWallSignalInitialized);
            }
            else
            {
                _frontRightWallState = false;
                _frontRightWallSignalInitialized = false;
            }

            return _frontLeftWallState || _frontRightWallState;
        }

        const bool fallbackState = HysteresisWall(
            _frontLeftWallState || _frontRightWallState,
            fallbackDistanceM,
            Config::kFrontWallOnThresholdM,
            Config::kFrontWallOffThresholdM);
        _frontLeftWallState = fallbackState;
        _frontRightWallState = fallbackState;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        return fallbackState;
    }

    WallSensorTelemetry BuildWallSensorTelemetry(WallSensorId sensorId, const WallSensorCalibrationInput& input) const
    {
        WallSensorTelemetry telemetry{};
        telemetry.ambientLight = input.ambientLight;
        telemetry.litLight = input.litLight;
        telemetry.differentialLight = input.differentialLight;
        telemetry.rawDistanceM = input.fallbackDistanceM;
        telemetry.distanceM = _wallCalibration.Apply(sensorId, input.measuredValue, input.fallbackDistanceM);
        return telemetry;
    }

    WallSensorTelemetry SampleWallSensor(WallSensorId sensorId, const MazeMap::WallSensor& sensor) const
    {
        return BuildWallSensorTelemetry(sensorId, SampleWallCalibrationInputRaw(sensorId, sensor));
    }

    template <typename TImu>
    static ImuTelemetry CaptureImu(TImu& imu, uint8_t interruptPin, ImuObservationTiming* timing = nullptr)
    {
        ImuTelemetry telemetry{};
#if defined(ARDUINO_TEENSY41)
        const uint32_t readStartUs = micros();
        if (timing != nullptr)
        {
            timing->readStartUs = readStartUs;
            timing->drdyUs = (digitalRead(interruptPin) == HIGH) ? readStartUs : 0UL;
        }
        const typename TImu::StatusReg status = imu.ReadStatus();
        const typename TImu::Axes gyro = imu.ReadGyro();
        const typename TImu::Axes accel = imu.ReadAccel();

        telemetry.status = status.Raw();
        telemetry.gyroX = gyro.x;
        telemetry.gyroY = gyro.y;
        telemetry.gyroZ = gyro.z;
        telemetry.accelX = accel.x;
        telemetry.accelY = accel.y;
        telemetry.accelZ = accel.z;
        telemetry.temp = imu.ReadTemp();
        telemetry.interruptHigh = (digitalRead(interruptPin) == HIGH);
        if (timing != nullptr)
        {
            timing->readDoneUs = micros();
        }
#else
        (void)imu;
        (void)interruptPin;
        (void)timing;
#endif
        return telemetry;
    }

    float ReadAverageGyroZRadpsRaw()
    {
        return ReadBackLeftGyroZRadpsRaw(_vehicle);
    }
};



