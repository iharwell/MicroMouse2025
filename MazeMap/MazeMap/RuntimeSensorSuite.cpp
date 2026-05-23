#include "pch.h"
#include "RuntimeSensorSuite.h"

#include "SharedRobotRuntime.h"
#include "WallSensorPreprocessor.h"

#include <algorithm>
#include <cmath>
namespace MazeMap
{
    template <typename TImu>
    static ImuTelemetry CaptureImuTelemetry(TImu& imu, const uint8_t interruptPin, ImuObservationTiming* timing = nullptr)
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

    #if defined(ARDUINO_TEENSY41)
    static bool ConfigureLoopMatchedBackLeftImu(
        MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>& imu,
        unsigned long controlPeriodUs,
        bool enableAccel)
    {
        switch (MazeMap::SelectUiImuSamplingProfile(controlPeriodUs))
        {
        case MazeMap::UiImuSamplingProfile::Exact1000Hz:
            imu.ConfigureUiHighAccuracyOdr(
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                enableAccel ?
                    MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::ODR_0960HZ_HP_N_LP :
                    MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::DISABLE,
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::ODR_0960HZ_HP_N_LP);
            return true;
        case MazeMap::UiImuSamplingProfile::Exact2000Hz:
            imu.ConfigureUiHighAccuracyOdr(
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                enableAccel ?
                    MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::ODR_1920HZ_HP_N_LP :
                    MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::DISABLE,
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ODR_SETTING::ODR_1920HZ_HP_N_LP);
            return true;
        default:
            (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
                "Unsupported IMU control period us: %lu",
                controlPeriodUs);
            return false;
        }
    }

    static bool ConfigureBackLeftImuForRuntime(
        MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>& imu,
        unsigned long controlPeriodUs,
        bool enableAccel,
        MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ACCEL_FILTER_FREQ accelFilterFreq =
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ACCEL_FILTER_FREQ::FRAC_1_400)
    {
        const bool imuConfigured = ConfigureLoopMatchedBackLeftImu(imu, controlPeriodUs, enableAccel);
        if (!imuConfigured)
        {
            return false;
        }

        imu.SetSelfTest(
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED,
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED);
        if (enableAccel)
        {
            imu.SetAccelRange(
                accelFilterFreq,
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::ACCEL_FULLSCALE::G8);
        }

        imu.SetGyroRange(
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::GYRO_LPF1_MODE::CUT_213,
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::GYRO_FULLSCALE_RANGE::DPS2000);
        return true;
    }

    #endif

    RuntimeSensorSuite::RuntimeSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _gyroBiasRadps(0.0f)
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
        , _accelBiasRightG(0.0f)
        , _accelBiasForwardG(0.0f)
        , _frontLeftWallState(false)
        , _frontRightWallState(false)
        , _sideLeftWallState(false)
        , _sideRightWallState(false)
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
        , _encoderObservationDegraded(false)
    {
    }

    bool RuntimeSensorSuite::Begin(const unsigned long controlPeriodUs)
    {
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
        _sideLeftWallState = false;
        _sideRightWallState = false;
        _frontWallUsesFallbackDetection = false;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _sideLeftPreviousSignalRise = 0.0f;
        _sideRightPreviousSignalRise = 0.0f;
        _sideLeftPreviousSignalRiseValid = false;
        _sideRightPreviousSignalRiseValid = false;
        _accelBiasRightG = 0.0f;
        _accelBiasForwardG = 0.0f;
        _accelBiasInitialized = false;
        _leftEncoderTotalCounts = 0;
        _rightEncoderTotalCounts = 0;
        _encoderObservationDegraded = false;
        InitializeWallSensorLedOffState();

        bool ok = true;
    #if defined(ARDUINO_TEENSY41)
        (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.BackLeftImu().Begin();
        if (!imuBackLeftOk)
        {
            const uint8_t whoAmI = _vehicle.BackLeftImu().GetLastWhoAmI();
            const uint8_t whoAmIMode3 = _vehicle.BackLeftImu().ReadWhoAmIWithSettings(400000UL, SPI_MODE3);
            const uint8_t whoAmIMode0 = _vehicle.BackLeftImu().ReadWhoAmIWithSettings(400000UL, SPI_MODE0);
            char whoAmIBuffer[3] = {};
            char whoAmIMode3Buffer[3] = {};
            char whoAmIMode0Buffer[3] = {};
            FormatHexByte(whoAmI, whoAmIBuffer);
            FormatHexByte(whoAmIMode3, whoAmIMode3Buffer);
            FormatHexByte(whoAmIMode0, whoAmIMode0Buffer);
            (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
                "IMU_BL init failed (%s), WHO_AM_I=0x%s, INT1_pullup=%s, WHO_AM_I@mode3/400kHz=0x%s, WHO_AM_I@mode0/400kHz=0x%s",
                _vehicle.BackLeftImu().GetLastBeginFailureReasonName(),
                whoAmIBuffer,
                ReadDrivenLowPinWithPullup(Pins::IMU_INT_1B) == LOW ? "low" : "high",
                whoAmIMode3Buffer,
                whoAmIMode0Buffer);
        }
        ok = imuBackLeftOk && ok;
    #endif

        if (ok)
        {
            ok = CalibrateGyroBias(controlPeriodUs, true) && ok;
        }

        return ok;
    }

    bool RuntimeSensorSuite::CalibrateGyroBias(const unsigned long controlPeriodUs, const bool enableAccelRuntime)
    {
        return CalibrateStationaryBackLeftGyroBias(controlPeriodUs, enableAccelRuntime);
    }

    StationaryImuCalibrationResult RuntimeSensorSuite::WaitForBackLeftImuCalibrationSettle(
        const MazeMap::EncoderCountPair& startCounts,
        const unsigned long settleMs) noexcept
    {
        const unsigned long settleStartMs = millis();
        while ((millis() - settleStartMs) < settleMs)
        {
            if (MazeMap::HaveEncoderCountsChanged(startCounts, CaptureEncoderCountPairForCalibration()))
            {
                return StationaryImuCalibrationResult::RestartEncoderMotion;
            }

            delay(1);
        }

        return StationaryImuCalibrationResult::Success;
    }

    StationaryImuCalibrationResult RuntimeSensorSuite::AverageBackLeftImuSelfTestSample(
        const std::uint16_t sampleCount,
        const MazeMap::EncoderCountPair& startCounts,
        AveragedBackLeftImuSample& averagedSample) noexcept
    {
        if (sampleCount == 0U)
        {
            return StationaryImuCalibrationResult::Failure;
        }

    #if defined(ARDUINO_TEENSY41)
        const float accelMgPerLsb = _vehicle.BackLeftImu().AccelSensitivityMgPerLsb();
        const float gyroDpsPerLsb = _vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() / 1000.0f;
        double accelMgSumX = 0.0;
        double accelMgSumY = 0.0;
        double accelMgSumZ = 0.0;
        double gyroDpsSumX = 0.0;
        double gyroDpsSumY = 0.0;
        double gyroDpsSumZ = 0.0;

        for (std::uint16_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
        {
            if (MazeMap::HaveEncoderCountsChanged(startCounts, CaptureEncoderCountPairForCalibration()))
            {
                return StationaryImuCalibrationResult::RestartEncoderMotion;
            }

            const MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::Axes accel = _vehicle.BackLeftImu().ReadAccel();
            const MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::Axes gyro = _vehicle.BackLeftImu().ReadGyro();
            accelMgSumX += static_cast<double>(accel.x) * accelMgPerLsb;
            accelMgSumY += static_cast<double>(accel.y) * accelMgPerLsb;
            accelMgSumZ += static_cast<double>(accel.z) * accelMgPerLsb;
            gyroDpsSumX += static_cast<double>(gyro.x) * gyroDpsPerLsb;
            gyroDpsSumY += static_cast<double>(gyro.y) * gyroDpsPerLsb;
            gyroDpsSumZ += static_cast<double>(gyro.z) * gyroDpsPerLsb;
            delay(kImuCalibrationSampleIntervalMs);
        }

        if (MazeMap::HaveEncoderCountsChanged(startCounts, CaptureEncoderCountPairForCalibration()))
        {
            return StationaryImuCalibrationResult::RestartEncoderMotion;
        }

        const double normalization = 1.0 / static_cast<double>(sampleCount);
        averagedSample.accelMgX = static_cast<float>(accelMgSumX * normalization);
        averagedSample.accelMgY = static_cast<float>(accelMgSumY * normalization);
        averagedSample.accelMgZ = static_cast<float>(accelMgSumZ * normalization);
        averagedSample.gyroDpsX = static_cast<float>(gyroDpsSumX * normalization);
        averagedSample.gyroDpsY = static_cast<float>(gyroDpsSumY * normalization);
        averagedSample.gyroDpsZ = static_cast<float>(gyroDpsSumZ * normalization);
        return StationaryImuCalibrationResult::Success;
    #else
        (void)startCounts;
        averagedSample = AveragedBackLeftImuSample{};
        return StationaryImuCalibrationResult::Success;
    #endif
    }

    StationaryImuCalibrationResult RuntimeSensorSuite::RunStationaryBackLeftImuSelfTest(
        const unsigned long controlPeriodUs)
    {
    #if defined(ARDUINO_TEENSY41)
        if (!ConfigureBackLeftImuForRuntime(
                _vehicle.BackLeftImu(),
                controlPeriodUs,
                true,
                Config::kMissionRuntimeAccelFilterFreq))
        {
            return StationaryImuCalibrationResult::Failure;
        }

        _vehicle.BackLeftImu().SetSelfTest(
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED,
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED);
        (void)CaptureEncoderCountPairForCalibration();
        const MazeMap::EncoderCountPair startCounts{};
        StationaryImuCalibrationResult settleResult =
            WaitForBackLeftImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
        if (settleResult != StationaryImuCalibrationResult::Success)
        {
            return settleResult;
        }

        AveragedBackLeftImuSample baseline{};
        StationaryImuCalibrationResult sampleResult =
            AverageBackLeftImuSelfTestSample(kImuSelfTestAverageSamples, startCounts, baseline);
        if (sampleResult != StationaryImuCalibrationResult::Success)
        {
            return sampleResult;
        }

        _vehicle.BackLeftImu().SetSelfTest(
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::POSITIVE,
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::POSITIVE);
        settleResult = WaitForBackLeftImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
        if (settleResult != StationaryImuCalibrationResult::Success)
        {
            _vehicle.BackLeftImu().SetSelfTest(
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED,
                MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED);
            return settleResult;
        }

        AveragedBackLeftImuSample stimulated{};
        sampleResult =
            AverageBackLeftImuSelfTestSample(kImuSelfTestAverageSamples, startCounts, stimulated);
        _vehicle.BackLeftImu().SetSelfTest(
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED,
            MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::SELF_TEST_MODE::DISABLED);
        if (sampleResult != StationaryImuCalibrationResult::Success)
        {
            return sampleResult;
        }

        settleResult = WaitForBackLeftImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
        if (settleResult != StationaryImuCalibrationResult::Success)
        {
            return settleResult;
        }

        const float accelDeltaMgX = std::fabs(stimulated.accelMgX - baseline.accelMgX);
        const float accelDeltaMgY = std::fabs(stimulated.accelMgY - baseline.accelMgY);
        const float accelDeltaMgZ = std::fabs(stimulated.accelMgZ - baseline.accelMgZ);
        const float gyroDeltaDpsX = std::fabs(stimulated.gyroDpsX - baseline.gyroDpsX);
        const float gyroDeltaDpsY = std::fabs(stimulated.gyroDpsY - baseline.gyroDpsY);
        const float gyroDeltaDpsZ = std::fabs(stimulated.gyroDpsZ - baseline.gyroDpsZ);
        const bool accelOk =
            MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgX) &&
            MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgY) &&
            MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgZ);
        const bool gyroOk =
            MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsX, kImuSelfTestGyroFullScaleDps) &&
            MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsY, kImuSelfTestGyroFullScaleDps) &&
            MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsZ, kImuSelfTestGyroFullScaleDps);
        if (accelOk && gyroOk)
        {
            return StationaryImuCalibrationResult::Success;
        }

        (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
            "IMU stationary self-test failed; accel_delta_mg=[%.1f,%.1f,%.1f], gyro_delta_dps=[%.1f,%.1f,%.1f]",
            accelDeltaMgX,
            accelDeltaMgY,
            accelDeltaMgZ,
            gyroDeltaDpsX,
            gyroDeltaDpsY,
            gyroDeltaDpsZ);
        return StationaryImuCalibrationResult::Failure;
    #else
        (void)controlPeriodUs;
        return StationaryImuCalibrationResult::Success;
    #endif
    }

    bool RuntimeSensorSuite::CalibrateStationaryBackLeftGyroBias(
        const unsigned long controlPeriodUs,
        const bool enableAccelRuntime)
    {
        _accelBiasInitialized = false;

    #if defined(ARDUINO_TEENSY41)
        const bool captureAccelBias = enableAccelRuntime;
        while (true)
        {
            (void)CaptureEncoderCountPairForCalibration();
            const StationaryImuCalibrationResult selfTestResult =
                RunStationaryBackLeftImuSelfTest(controlPeriodUs);
            if (selfTestResult == StationaryImuCalibrationResult::RestartEncoderMotion)
            {
                (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                    "Encoder motion detected during stationary IMU self-test; restarting bias calibration");
                continue;
            }
            if (selfTestResult != StationaryImuCalibrationResult::Success)
            {
                return false;
            }

            if (!ConfigureBackLeftImuForRuntime(
                    _vehicle.BackLeftImu(),
                    controlPeriodUs,
                    enableAccelRuntime,
                    Config::kMissionRuntimeAccelFilterFreq))
            {
                return false;
            }

            (void)CaptureEncoderCountPairForCalibration();
            const MazeMap::EncoderCountPair startCounts{};
            const unsigned long requiredSamples = MazeMap::ComputeGyroBiasSampleCount(
                static_cast<unsigned long>(MazeMap::Config::kGyroBiasSamples),
                kImuCalibrationSampleIntervalMs,
                static_cast<unsigned long>(MazeMap::Config::kGyroBiasMinimumAveragingWindowMs));
            const unsigned long measurementStartMs = millis();
            double accumulatedRadps = 0.0;
            double accumulatedAccelRightG = 0.0;
            double accumulatedAccelForwardG = 0.0;
            unsigned long collectedSamples = 0UL;
            while ((collectedSamples < requiredSamples) ||
                ((millis() - measurementStartMs) < static_cast<unsigned long>(MazeMap::Config::kGyroBiasMinimumAveragingWindowMs)))
            {
                if (MazeMap::HaveEncoderCountsChanged(startCounts, CaptureEncoderCountPairForCalibration()))
                {
                    (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                        "Encoder motion detected during gyro bias measurement; restarting IMU self-test");
                    accumulatedRadps = 0.0;
                    collectedSamples = 0UL;
                    break;
                }

                if (captureAccelBias)
                {
                    const MazeMap::LSM6DSV16X_IMU<37, 33, 11, 12, 13>::Axes accel = _vehicle.BackLeftImu().ReadAccel();
                    accumulatedAccelRightG += static_cast<double>(_vehicle.BackLeftImu().AccelRawToG(accel.x));
                    accumulatedAccelForwardG += static_cast<double>(_vehicle.BackLeftImu().AccelRawToG(accel.y));
                }
                accumulatedRadps += static_cast<double>(ReadBackLeftGyroZRadpsRaw(_vehicle));
                ++collectedSamples;
                delay(kImuCalibrationSampleIntervalMs);
            }

            if (collectedSamples == 0UL)
            {
                continue;
            }

            if (MazeMap::HaveEncoderCountsChanged(startCounts, CaptureEncoderCountPairForCalibration()))
            {
                (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                    "Encoder motion detected after gyro bias capture; restarting IMU self-test");
                continue;
            }

            _gyroBiasRadps = static_cast<float>(accumulatedRadps / static_cast<double>(collectedSamples));
            if (captureAccelBias)
            {
                _accelBiasRightG = static_cast<float>(accumulatedAccelRightG / static_cast<double>(collectedSamples));
                _accelBiasForwardG = static_cast<float>(accumulatedAccelForwardG / static_cast<double>(collectedSamples));
            }
            _accelBiasInitialized = captureAccelBias;
            return true;
        }
    #else
        (void)controlPeriodUs;
        (void)enableAccelRuntime;
        _gyroBiasRadps = EstimateMissionGyroBiasRadps(_vehicle);
        return true;
    #endif
    }

    void RuntimeSensorSuite::ResetSideWallMemory() noexcept
    {
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _sideLeftWallState = false;
        _sideRightWallState = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _sideLeftPreviousSignalRise = 0.0f;
        _sideRightPreviousSignalRise = 0.0f;
        _sideLeftPreviousSignalRiseValid = false;
        _sideRightPreviousSignalRiseValid = false;
    }

    void RuntimeSensorSuite::BeginInterlacedCapture(
        const bool stationary,
        const MazeMap::VehicleState& state,
        SensorSnapshot& snapshot,
        const bool captureWalls,
        const bool captureEncoders,
        const float encoderDtSeconds)
    {
        if (_interlacedCaptureActive && _interlacedCaptureWalls)
        {
            AbortAsyncWallSensorSweepRead(_interlacedWallRead);
            _frontLeftLedOffCommandUs = _interlacedWallRead.nextFrontLeftLedOffCommandUs;
            _frontRightLedOffCommandUs = _interlacedWallRead.nextFrontRightLedOffCommandUs;
            _sideLeftLedOffCommandUs = _interlacedWallRead.nextSideLeftLedOffCommandUs;
            _sideRightLedOffCommandUs = _interlacedWallRead.nextSideRightLedOffCommandUs;
        }
        ClearInterlacedCaptureState();

        snapshot = SensorSnapshot{};
        _interlacedCaptureActive = true;
        _interlacedCaptureWalls = captureWalls;
        _interlacedCaptureStationary = stationary;
        _interlacedCaptureSnapshot = &snapshot;
        _interlacedCaptureState = &state;
        PublishEncoderTotals(snapshot);
        if (captureEncoders)
        {
            CaptureEncoderSnapshot(snapshot, encoderDtSeconds);
        }

        if (!captureWalls)
        {
            return;
        }

        StartAsyncWallSensorSweepRead(
            _vehicle.FrontLeftWallSensor(),
            _frontLeftLedOffCommandUs,
            _vehicle.FrontRightWallSensor(),
            _frontRightLedOffCommandUs,
            _vehicle.SideLeftWallSensor(),
            _sideLeftLedOffCommandUs,
            _vehicle.SideRightWallSensor(),
            _sideRightLedOffCommandUs,
            _interlacedWallRead);
    }

    void RuntimeSensorSuite::CaptureInterlacedInertialSnapshot() noexcept
    {
        if (!_interlacedCaptureActive ||
            (_interlacedCaptureSnapshot == nullptr) ||
            _interlacedCaptureImuCaptured)
        {
            return;
        }

        CaptureInertialSnapshot(_interlacedCaptureStationary, *_interlacedCaptureSnapshot);
        _interlacedCaptureImuCaptured = true;
    }

    void RuntimeSensorSuite::ServiceFrontWallCollection() noexcept
    {
        if (_interlacedWallRead.stage == AsyncWallSensorSweepStage::Front)
        {
            (void)ServiceAsyncWallSensorSweepRead(_interlacedWallRead);
        }
    }

    void RuntimeSensorSuite::ServiceLeftWallCollection() noexcept
    {
        if (_interlacedWallRead.stage == AsyncWallSensorSweepStage::Left)
        {
            (void)ServiceAsyncWallSensorSweepRead(_interlacedWallRead);
        }
    }

    void RuntimeSensorSuite::ServiceRightWallCollection() noexcept
    {
        if (_interlacedWallRead.stage == AsyncWallSensorSweepStage::Right)
        {
            (void)ServiceAsyncWallSensorSweepRead(_interlacedWallRead);
        }
    }

    void RuntimeSensorSuite::FinishInterlacedCapture()
    {
        if (!_interlacedCaptureActive || (_interlacedCaptureSnapshot == nullptr))
        {
            return;
        }

        SensorSnapshot& snapshot = *_interlacedCaptureSnapshot;
        const MazeMap::VehicleState* const state = _interlacedCaptureState;
        if (!_interlacedCaptureImuCaptured)
        {
            CaptureInterlacedInertialSnapshot();
        }

        if (!_interlacedCaptureWalls)
        {
            ClearInterlacedCaptureState();
            return;
        }

        ServiceFrontWallCollection();
        ServiceLeftWallCollection();
        ServiceRightWallCollection();
        if (_interlacedWallRead.active)
        {
            AwaitAsyncWallSensorSweepRead(_interlacedWallRead);
        }

        _frontLeftLedOffCommandUs = _interlacedWallRead.nextFrontLeftLedOffCommandUs;
        _frontRightLedOffCommandUs = _interlacedWallRead.nextFrontRightLedOffCommandUs;
        _sideLeftLedOffCommandUs = _interlacedWallRead.nextSideLeftLedOffCommandUs;
        _sideRightLedOffCommandUs = _interlacedWallRead.nextSideRightLedOffCommandUs;

        const WallSensorCalibrationInput frontLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontLeft, _interlacedWallRead.frontLeftSample);
        const WallSensorCalibrationInput frontRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::FrontRight, _interlacedWallRead.frontRightSample);
        const WallSensorCalibrationInput sideLeftRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideLeft, _interlacedWallRead.sideLeftSample);
        const WallSensorCalibrationInput sideRightRawInput =
            BuildWallSensorCalibrationInput(WallSensorId::SideRight, _interlacedWallRead.sideRightSample);
        const WallSensorCalibrationInput frontLeftInput = _frontLeftInputAverage.PushAndAverage(frontLeftRawInput);
        const WallSensorCalibrationInput frontRightInput = _frontRightInputAverage.PushAndAverage(frontRightRawInput);
        const WallSensorCalibrationInput sideLeftInput = _sideLeftInputAverage.PushAndAverage(sideLeftRawInput);
        const WallSensorCalibrationInput sideRightInput = _sideRightInputAverage.PushAndAverage(sideRightRawInput);

        WallSensorTelemetry frontLeftTelemetry = BuildWallSensorTelemetry(WallSensorId::FrontLeft, frontLeftInput);
        WallSensorTelemetry frontRightTelemetry = BuildWallSensorTelemetry(WallSensorId::FrontRight, frontRightInput);
        WallSensorTelemetry sideLeftTelemetry = BuildWallSensorTelemetry(WallSensorId::SideLeft, sideLeftInput);
        WallSensorTelemetry sideRightTelemetry = BuildWallSensorTelemetry(WallSensorId::SideRight, sideRightInput);
        snapshot.SetFrontTiming(frontLeftRawInput.timing);
        snapshot.SetLeftTiming(sideLeftInput.timing);
        snapshot.SetRightTiming(sideRightInput.timing);

        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        _wallCalibration.TryComputeSideWallDistanceThresholds(
            Config::kSideWallDistanceLatchFractionOfCalibration,
            Config::kSideWallDistanceReleaseFractionOfCalibration,
            sideWallOnThresholdM,
            sideWallOffThresholdM);

        snapshot.SetFrontLeftDistanceM(frontLeftTelemetry.distanceM);
        snapshot.SetFrontRightDistanceM(frontRightTelemetry.distanceM);
        snapshot.SetFrontLeftDifferentialLight(frontLeftTelemetry.differentialLight);
        snapshot.SetFrontRightDifferentialLight(frontRightTelemetry.differentialLight);

        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideLeft,
            sideLeftTelemetry.differentialLight,
            sideLeftTelemetry.distanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideRight,
            sideRightTelemetry.differentialLight,
            sideRightTelemetry.distanceM);
        WallSensorPreprocessor wallPreprocessor{};
        const WallObs frontLeftWallSensorObservation = wallPreprocessor.process(
            _vehicle.FrontLeftWallSensor(),
            frontLeftTelemetry.ambientLight,
            frontLeftTelemetry.litLight,
            frontLeftTelemetry.distanceM);
        const WallObs frontRightWallSensorObservation = wallPreprocessor.process(
            _vehicle.FrontRightWallSensor(),
            frontRightTelemetry.ambientLight,
            frontRightTelemetry.litLight,
            frontRightTelemetry.distanceM);
        const WallObs sideLeftWallSensorObservation = wallPreprocessor.process(
            _vehicle.SideLeftWallSensor(),
            sideLeftTelemetry.ambientLight,
            sideLeftTelemetry.litLight,
            sideLeftTelemetry.distanceM);
        const WallObs sideRightWallSensorObservation = wallPreprocessor.process(
            _vehicle.SideRightWallSensor(),
            sideRightTelemetry.ambientLight,
            sideRightTelemetry.litLight,
            sideRightTelemetry.distanceM);
        snapshot.SetSideLeftDistanceM(sideLeftTelemetry.distanceM);
        snapshot.SetSideRightDistanceM(sideRightTelemetry.distanceM);
        snapshot.SetSideLeftDifferentialLight(sideLeftTelemetry.differentialLight);
        snapshot.SetSideRightDifferentialLight(sideRightTelemetry.differentialLight);

        snapshot.SetFrontWall(UpdateFrontWallState(
            frontLeftInput.ambientLight,
            frontLeftInput.measuredValue,
            frontRightInput.ambientLight,
            frontRightInput.measuredValue,
            (std::min)(snapshot.FrontLeftDistanceM(), snapshot.FrontRightDistanceM())));
        snapshot.SetFrontLeftWall(_frontLeftWallState);
        snapshot.SetFrontRightWall(_frontRightWallState);
        snapshot.SetFrontWallObservationValid(
            std::isfinite(snapshot.FrontLeftDistanceM()) &&
            (snapshot.FrontLeftDistanceM() > 0.0f) &&
            std::isfinite(snapshot.FrontRightDistanceM()) &&
            (snapshot.FrontRightDistanceM() > 0.0f));
        snapshot.SetFrontWallUsesFallbackDetection(_frontWallUsesFallbackDetection);
        snapshot.SetFrontWallUsesCharacterizationDetection(false);

        const bool sideLeftWindowValid =
            (state != nullptr) &&
            IsSideWallDetectionWindowValid(*state, _vehicle.SideLeftWallSensor());
        const bool sideRightWindowValid =
            (state != nullptr) &&
            IsSideWallDetectionWindowValid(*state, _vehicle.SideRightWallSensor());

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
            (sideLeftInput.fallbackDistanceM > 0.0f);
        const bool sideRightFallbackValid =
            std::isfinite(sideRightInput.fallbackDistanceM) &&
            (sideRightInput.fallbackDistanceM > 0.0f);

        const bool sideLeftObservationEligible =
            sideLeftWindowValid &&
            (sideLeftSignalClassifiable || sideLeftFallbackValid);
        const bool sideRightObservationEligible =
            sideRightWindowValid &&
            (sideRightSignalClassifiable || sideRightFallbackValid);
        const bool sideLeftControlRangeValid =
            sideLeftSignalMetricsValid ?
            (sideLeftSignalRise >= sideLeftLatchRiseThreshold) :
            (sideLeftFallbackValid && (sideLeftInput.fallbackDistanceM < sideWallOffThresholdM));
        const bool sideRightControlRangeValid =
            sideRightSignalMetricsValid ?
            (sideRightSignalRise >= sideRightLatchRiseThreshold) :
            (sideRightFallbackValid && (sideRightInput.fallbackDistanceM < sideWallOffThresholdM));

        snapshot.SetLeftTransitionDetected(DetectTransitionFromSignalRise(
            sideLeftWindowValid,
            sideLeftSignalMetricsValid,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold * Config::kSideWallTransitionSignalFractionOfLatch,
            _sideLeftPreviousSignalRise,
            _sideLeftPreviousSignalRiseValid));
        snapshot.SetRightTransitionDetected(DetectTransitionFromSignalRise(
            sideRightWindowValid,
            sideRightSignalMetricsValid,
            sideRightSignalRise,
            sideRightLatchRiseThreshold * Config::kSideWallTransitionSignalFractionOfLatch,
            _sideRightPreviousSignalRise,
            _sideRightPreviousSignalRiseValid));
        snapshot.SetLeftWallObservationWindowValid(sideLeftObservationEligible);
        snapshot.SetRightWallObservationWindowValid(sideRightObservationEligible);
        snapshot.SetLeftDistanceValidForControl(
            sideLeftObservationEligible &&
            !snapshot.LeftTransitionDetected() &&
            sideLeftControlRangeValid);
        snapshot.SetRightDistanceValidForControl(
            sideRightObservationEligible &&
            !snapshot.RightTransitionDetected() &&
            sideRightControlRangeValid);
        snapshot.SetLeftWall(UpdateSideWallState(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _sideLeftWallState));
        snapshot.SetRightWall(UpdateSideWallState(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _sideRightWallState));
        snapshot.SetLeftWallObservation(ComputeSideWallObservationHit(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideLeftObservationEligible));
        snapshot.SetRightWallObservation(ComputeSideWallObservationHit(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideRightObservationEligible));

        frontLeftTelemetry.wall = snapshot.HasFrontLeftWall();
        frontRightTelemetry.wall = snapshot.HasFrontRightWall();
        sideLeftTelemetry.wall = snapshot.HasLeftWall();
        sideRightTelemetry.wall = snapshot.HasRightWall();
        snapshot.SetFrontLeftTelemetry(frontLeftTelemetry);
        snapshot.SetFrontRightTelemetry(frontRightTelemetry);
        snapshot.SetSideLeftTelemetry(sideLeftTelemetry);
        snapshot.SetSideRightTelemetry(sideRightTelemetry);
        snapshot.SetFrontLeftWallSensorObservation(frontLeftWallSensorObservation);
        snapshot.SetFrontRightWallSensorObservation(frontRightWallSensorObservation);
        snapshot.SetSideLeftWallSensorObservation(sideLeftWallSensorObservation);
        snapshot.SetSideRightWallSensorObservation(sideRightWallSensorObservation);
        snapshot.SetFrontSkewM(snapshot.FrontLeftDistanceM() - snapshot.FrontRightDistanceM());
        snapshot.SetCorridorErrorM(MazeMap::App::Internal::Runtime::ComputeCorridorError(
            snapshot.SideLeftDistanceM(),
            snapshot.SideRightDistanceM(),
            snapshot.LeftDistanceValidForControl(),
            snapshot.RightDistanceValidForControl(),
            _wallCalibration.GetExpectedSideWallDistanceM()));

        ClearInterlacedCaptureState();
    }

    void RuntimeSensorSuite::ClearInterlacedCaptureState() noexcept
    {
        _interlacedWallRead = AsyncWallSensorSweepRead{};
        _interlacedCaptureState = nullptr;
        _interlacedCaptureSnapshot = nullptr;
        _interlacedCaptureActive = false;
        _interlacedCaptureWalls = false;
        _interlacedCaptureStationary = false;
        _interlacedCaptureImuCaptured = false;
    }

    float RuntimeSensorSuite::GetGyroSensitivityMdpsPerLsb() const noexcept
    {
        return _vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb();
    }

    float RuntimeSensorSuite::GetAccelSensitivityMgPerLsb() const noexcept
    {
        return _vehicle.BackLeftImu().AccelSensitivityMgPerLsb();
    }

    float RuntimeSensorSuite::GetGyroBiasRadps() const noexcept
    {
        return _gyroBiasRadps;
    }

    bool RuntimeSensorSuite::HasAccelBias() const noexcept
    {
        return _accelBiasInitialized;
    }

    float RuntimeSensorSuite::GetAccelBiasRightG() const noexcept
    {
        return _accelBiasRightG;
    }

    float RuntimeSensorSuite::GetAccelBiasForwardG() const noexcept
    {
        return _accelBiasForwardG;
    }

    MazeMap::EncoderObs RuntimeSensorSuite::CaptureEncoderObservation(const float dtSeconds) noexcept
    {
        MazeMap::EncoderObs observation = _vehicle.CaptureEncoderObservation(dtSeconds);
        _leftEncoderTotalCounts += static_cast<std::int64_t>(observation.TotalLeftCounts());
        _rightEncoderTotalCounts += static_cast<std::int64_t>(observation.TotalRightCounts());
        return observation;
    }

    MazeMap::EncoderCountPair RuntimeSensorSuite::CaptureEncoderCountPairForCalibration() noexcept
    {
        const MazeMap::EncoderObs observation = CaptureEncoderObservation(0.0f);
        return MazeMap::EncoderCountPair{ observation.TotalLeftCounts(), observation.TotalRightCounts() };
    }

    bool RuntimeSensorSuite::IsEncoderObservationUsableForPrediction(
        const MazeMap::EncoderObs& observation,
        const float dtSeconds,
        const char*& degradedReason) const noexcept
    {
        degradedReason = "ok";
        if (!std::isfinite(dtSeconds) || !(dtSeconds > 0.0f))
        {
            degradedReason = "invalid_encoder_dt";
            return false;
        }

        const float wheelRadiusM = MazeMap::Vehicle::GetDriveWheelRadiusM();
        if (!std::isfinite(wheelRadiusM) || !(wheelRadiusM > 0.0f))
        {
            degradedReason = "invalid_vehicle_wheel_radius";
            return false;
        }

        const float leftDistanceFromCountsM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(observation.TotalLeftCounts());
        const float rightDistanceFromCountsM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(observation.TotalRightCounts());
        if (!std::isfinite(leftDistanceFromCountsM) || !std::isfinite(rightDistanceFromCountsM))
        {
            degradedReason = "nonfinite_encoder_count_distance";
            return false;
        }

        if (!std::isfinite(observation.LeftDistanceDeltaM()) ||
            !std::isfinite(observation.RightDistanceDeltaM()) ||
            !std::isfinite(observation.LeftVelocityMps()) ||
            !std::isfinite(observation.RightVelocityMps()) ||
            !std::isfinite(observation.LeftWheelSpeedRadps()) ||
            !std::isfinite(observation.RightWheelSpeedRadps()))
        {
            degradedReason = "nonfinite_encoder_observation";
            return false;
        }

        const float leftVelocityFromWheelSpeedMps =
            MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(observation.LeftWheelSpeedRadps());
        const float rightVelocityFromWheelSpeedMps =
            MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(observation.RightWheelSpeedRadps());
        if (!std::isfinite(leftVelocityFromWheelSpeedMps) ||
            !std::isfinite(rightVelocityFromWheelSpeedMps))
        {
            degradedReason = "nonfinite_vehicle_encoder_velocity";
            return false;
        }

        return true;
    }

    void RuntimeSensorSuite::ReportEncoderObservationState(
        const bool validForPrediction,
        const char* const degradedReason,
        const MazeMap::EncoderObs& observation,
        const float dtSeconds) noexcept
    {
        if (validForPrediction)
        {
            if (_encoderObservationDegraded)
            {
                (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                    "Encoder observation recovered; prediction input restored");
            }
            _encoderObservationDegraded = false;
            return;
        }

        if (!_encoderObservationDegraded)
        {
            (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
                "Encoder observation degraded; prediction input omitted; reason=%s; dt_s=%.9g; left_counts=%ld; right_counts=%ld; left_speed_radps=%.9g; right_speed_radps=%.9g",
                (degradedReason != nullptr) ? degradedReason : "unknown",
                static_cast<double>(dtSeconds),
                static_cast<long>(observation.TotalLeftCounts()),
                static_cast<long>(observation.TotalRightCounts()),
                static_cast<double>(observation.LeftWheelSpeedRadps()),
                static_cast<double>(observation.RightWheelSpeedRadps()));
        }
        _encoderObservationDegraded = true;
    }

    void RuntimeSensorSuite::PublishEncoderTotals(SensorSnapshot& snapshot) const noexcept
    {
        snapshot.SetEncoderTotals(_leftEncoderTotalCounts, _rightEncoderTotalCounts);
        snapshot.SetEncoderDistancesM(
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(_leftEncoderTotalCounts),
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(_rightEncoderTotalCounts));
    }

    void RuntimeSensorSuite::CaptureEncoderSnapshot(
        SensorSnapshot& snapshot,
        const float dtSeconds) noexcept
    {
        const EncoderObs encoderObservation = CaptureEncoderObservation(dtSeconds);
        const char* degradedReason = "ok";
        const bool encoderObservationValid =
            IsEncoderObservationUsableForPrediction(encoderObservation, dtSeconds, degradedReason);
        snapshot.SetEncoderObservation(encoderObservation, encoderObservationValid);
        ReportEncoderObservationState(
            encoderObservationValid,
            degradedReason,
            encoderObservation,
            dtSeconds);
        PublishEncoderTotals(snapshot);
    }

    void RuntimeSensorSuite::InitializeWallSensorLedOffState() noexcept
    {
        _vehicle.FrontLeftWallSensor().SetLedEnabled(false);
        _vehicle.FrontRightWallSensor().SetLedEnabled(false);
        _vehicle.SideLeftWallSensor().SetLedEnabled(false);
        _vehicle.SideRightWallSensor().SetLedEnabled(false);
        const uint32_t nowUs = micros();
        _frontLeftLedOffCommandUs = nowUs;
        _frontRightLedOffCommandUs = nowUs;
        _sideLeftLedOffCommandUs = nowUs;
        _sideRightLedOffCommandUs = nowUs;
    }

    void RuntimeSensorSuite::CaptureInertialSnapshot(const bool stationary, SensorSnapshot& snapshot)
    {
        const MazeMap::SensorMount imuMount = MazeMap::Vehicle::GetBackLeftImuMount();
        ImuObservationTiming imuTiming{};
        const ImuTelemetry backLeftImuTelemetry =
            CaptureImuTelemetry(_vehicle.BackLeftImu(), Pins::IMU_INT_1B, &imuTiming);
        snapshot.SetFrontRightImuTelemetry(ImuTelemetry{});
        snapshot.SetBackLeftImuTelemetry(backLeftImuTelemetry);
        snapshot.SetImuTiming(imuTiming);

        float rawYawRateRadps = imuMount.TransformClockwiseYawRateToBody(ReadGyroZRadpsRaw());
    #if defined(ARDUINO_TEENSY41)
        rawYawRateRadps =
            imuMount.TransformClockwiseYawRateToBody(
                _vehicle.BackLeftImu().GyroRawToClockwiseYawDps(backLeftImuTelemetry.gyroZ) * DEG_TO_RAD_F);

        const Eigen::Vector2f accelImuG(
            _vehicle.BackLeftImu().AccelRawToG(backLeftImuTelemetry.accelX),
            _vehicle.BackLeftImu().AccelRawToG(backLeftImuTelemetry.accelY));
        const Eigen::Vector2f accelBodyG = imuMount.TransformPlanarVectorToBody(accelImuG);
        snapshot.SetAccelerationBiasValid(_accelBiasInitialized);
        if (_accelBiasInitialized && stationary)
        {
            _accelBiasRightG = (0.998f * _accelBiasRightG) + (0.002f * accelBodyG.x());
            _accelBiasForwardG = (0.998f * _accelBiasForwardG) + (0.002f * accelBodyG.y());
        }

        if (_accelBiasInitialized)
        {
            const float accelDeltaRightG = accelBodyG.x() - _accelBiasRightG;
            const float accelDeltaForwardG = accelBodyG.y() - _accelBiasForwardG;
            snapshot.SetBodyRightAccelerationMps2(kStandardGravityMps2 * accelDeltaRightG);
            snapshot.SetBodyForwardAccelerationMps2(kStandardGravityMps2 * accelDeltaForwardG);
            snapshot.SetPlanarAccelerationMps2(
                kStandardGravityMps2 * MazeMap::Math::Sqrtf((accelDeltaRightG * accelDeltaRightG) + (accelDeltaForwardG * accelDeltaForwardG)));
        }
        else
        {
            snapshot.SetBodyRightAccelerationMps2(0.0f);
            snapshot.SetBodyForwardAccelerationMps2(0.0f);
            snapshot.SetPlanarAccelerationMps2(0.0f);
        }
    #else
        snapshot.SetBodyRightAccelerationMps2(0.0f);
        snapshot.SetBodyForwardAccelerationMps2(0.0f);
        snapshot.SetPlanarAccelerationMps2(0.0f);
        snapshot.SetAccelerationBiasValid(false);
    #endif

        if (stationary &&
            MazeMap::ShouldUpdateGyroBiasFromStationarySample(rawYawRateRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
        {
            _gyroBiasRadps = (0.995f * _gyroBiasRadps) + (0.005f * rawYawRateRadps);
        }
        snapshot.SetRawYawRateRadps(rawYawRateRadps);
        snapshot.SetYawRateBiasRadps(_gyroBiasRadps);
        snapshot.SetYawRateRadps(rawYawRateRadps - _gyroBiasRadps);
    }

    bool RuntimeSensorSuite::TryComputeSideSignalMetrics(
        const WallSensorId sensorId,
        const float measuredDifferentialLight,
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

        signalRise = MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline(
            measuredDifferentialLight,
            signalBaseline);
        missRiseThreshold = Config::kWallMapMissSignalFractionOfLatch * latchRiseThreshold;
        return
            std::isfinite(signalRise) &&
            std::isfinite(latchRiseThreshold) &&
            std::isfinite(missRiseThreshold) &&
            (latchRiseThreshold > 0.0f) &&
            (missRiseThreshold >= 0.0f);
    }

    bool RuntimeSensorSuite::DetectTransitionFromSignalRise(
        const bool windowValid,
        const bool signalMetricsValid,
        const float signalRise,
        const float transitionThreshold,
        float& previousSignalRise,
        bool& previousValid) noexcept
    {
        bool transitionDetected = false;
        const bool currentValid =
            windowValid &&
            signalMetricsValid &&
            std::isfinite(signalRise) &&
            std::isfinite(transitionThreshold) &&
            (transitionThreshold > 0.0f);
        if (currentValid && previousValid)
        {
            transitionDetected = std::fabs(signalRise - previousSignalRise) >= transitionThreshold;
        }

        previousSignalRise = currentValid ? signalRise : 0.0f;
        previousValid = currentValid;
        return transitionDetected;
    }

    bool RuntimeSensorSuite::ComputeSideWallObservationHit(
        const WallSensorId sensorId,
        const float measuredDifferentialLight,
        const float fallbackDistanceM,
        const float onThresholdM,
        const bool detectionWindowValid) const
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
            return MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline(
                       measuredDifferentialLight,
                       signalBaseline) >= onMeasuredThreshold;
        }

        return std::isfinite(fallbackDistanceM) && (fallbackDistanceM < onThresholdM);
    }

    bool RuntimeSensorSuite::UpdateSideWallState(
        const WallSensorId sensorId,
        const float measuredDifferentialLight,
        const float fallbackDistanceM,
        const float onThresholdM,
        const float offThresholdM,
        const bool detectionWindowValid,
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
            return MazeMap::App::Internal::Runtime::UpdateFilteredSignalState(
                MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline(
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

    bool RuntimeSensorSuite::UpdateFrontWallState(
        const float leftAmbientLight,
        const float leftMeasuredDifferentialLight,
        const float rightAmbientLight,
        const float rightMeasuredDifferentialLight,
        const float fallbackDistanceM)
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
                MazeMap::App::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline(
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
                MazeMap::App::Internal::Runtime::UpdateFilteredSignalState(
                    MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline(
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

    WallSensorTelemetry RuntimeSensorSuite::BuildWallSensorTelemetry(
        const WallSensorId sensorId,
        const WallSensorCalibrationInput& input) const
    {
        WallSensorTelemetry telemetry{};
        telemetry.ambientLight = input.ambientLight;
        telemetry.litLight = input.litLight;
        telemetry.differentialLight = input.differentialLight;
        telemetry.rawDistanceM = input.fallbackDistanceM;
        telemetry.distanceM = _wallCalibration.Apply(sensorId, input.measuredValue, input.fallbackDistanceM);
        return telemetry;
    }

    float RuntimeSensorSuite::ReadGyroZRadpsRaw()
    {
        return ReadBackLeftGyroZRadpsRaw(_vehicle);
    }
}
