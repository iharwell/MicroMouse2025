#include "pch.h"
#include "RuntimeSensorSuite.h"

#include "SharedRobotRuntime.h"

#include <algorithm>
#include <cmath>

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
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    bool enableAccel)
{
    using Imu = MazeMap::Vehicle::ImuBackLeft;

    switch (MazeMap::SelectUiImuSamplingProfile(controlPeriodUs))
    {
    case MazeMap::UiImuSamplingProfile::Exact1000Hz:
        imu.ConfigureUiHighAccuracyOdr(
            Imu::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
            enableAccel ? Imu::ODR_SETTING::ODR_0960HZ_HP_N_LP : Imu::ODR_SETTING::DISABLE,
            Imu::ODR_SETTING::ODR_0960HZ_HP_N_LP);
        return true;
    case MazeMap::UiImuSamplingProfile::Exact2000Hz:
        imu.ConfigureUiHighAccuracyOdr(
            Imu::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
            enableAccel ? Imu::ODR_SETTING::ODR_1920HZ_HP_N_LP : Imu::ODR_SETTING::DISABLE,
            Imu::ODR_SETTING::ODR_1920HZ_HP_N_LP);
        return true;
    default:
        (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
            "Unsupported IMU control period us: %lu",
            controlPeriodUs);
        return false;
    }
}

static bool ConfigureBackLeftImuForRuntime(
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    bool enableAccel,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
    const bool imuConfigured = ConfigureLoopMatchedBackLeftImu(imu, controlPeriodUs, enableAccel);
    if (!imuConfigured)
    {
        return false;
    }

    imu.SetSelfTest(
        MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE::DISABLED,
        MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE::DISABLED);
    if (enableAccel)
    {
        imu.SetAccelRange(
            accelFilterFreq,
            MazeMap::Vehicle::ImuBackLeft::ACCEL_FULLSCALE::G8);
    }

    imu.SetGyroRange(
        MazeMap::Vehicle::ImuBackLeft::GYRO_LPF1_MODE::CUT_213,
        MazeMap::Vehicle::ImuBackLeft::GYRO_FULLSCALE_RANGE::DPS2000);
    return true;
}

template <typename CaptureEncoderCounts>
static StationaryImuCalibrationResult RunStationaryBackLeftImuSelfTest(
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    const MazeMap::EncoderCountPair& startCounts,
    CaptureEncoderCounts& captureEncoderCounts,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
    if (!ConfigureBackLeftImuForRuntime(imu, controlPeriodUs, true, accelFilterFreq))
    {
        return StationaryImuCalibrationResult::Failure;
    }

    using SelfTestMode = MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE;

    imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
    StationaryImuCalibrationResult settleResult =
        WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs, captureEncoderCounts);
    if (settleResult != StationaryImuCalibrationResult::Success)
    {
        return settleResult;
    }

    AveragedBackLeftImuSample baseline{};
    StationaryImuCalibrationResult sampleResult =
        AverageBackLeftImuSelfTestSample(
            imu,
            kImuSelfTestAverageSamples,
            startCounts,
            baseline,
            captureEncoderCounts);
    if (sampleResult != StationaryImuCalibrationResult::Success)
    {
        return sampleResult;
    }

    imu.SetSelfTest(SelfTestMode::POSITIVE, SelfTestMode::POSITIVE);
    settleResult = WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs, captureEncoderCounts);
    if (settleResult != StationaryImuCalibrationResult::Success)
    {
        imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
        return settleResult;
    }

    AveragedBackLeftImuSample stimulated{};
    sampleResult = AverageBackLeftImuSelfTestSample(
        imu,
        kImuSelfTestAverageSamples,
        startCounts,
        stimulated,
        captureEncoderCounts);
    imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
    if (sampleResult != StationaryImuCalibrationResult::Success)
    {
        return sampleResult;
    }

    settleResult = WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs, captureEncoderCounts);
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
}
#endif

template <typename CaptureEncoderCounts>
static bool CalibrateStationaryBackLeftGyroBias(
    MazeMap::Vehicle& vehicle,
    unsigned long controlPeriodUs,
    bool enableAccelRuntime,
    float& gyroBiasRadps,
    float* accelBiasXG,
    float* accelBiasYG,
    bool* accelBiasInitialized,
    CaptureEncoderCounts& captureEncoderCounts,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
    if (accelBiasInitialized != nullptr)
    {
        *accelBiasInitialized = false;
    }

#if defined(ARDUINO_TEENSY41)
    const bool captureAccelBias =
        enableAccelRuntime &&
        (accelBiasXG != nullptr) &&
        (accelBiasYG != nullptr);
    while (true)
    {
        (void)captureEncoderCounts();
        const MazeMap::EncoderCountPair startCounts{};
        const StationaryImuCalibrationResult selfTestResult =
            RunStationaryBackLeftImuSelfTest(
                vehicle.IMU_BL,
                controlPeriodUs,
                startCounts,
                captureEncoderCounts,
                accelFilterFreq);
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

        if (!ConfigureBackLeftImuForRuntime(vehicle.IMU_BL, controlPeriodUs, enableAccelRuntime, accelFilterFreq))
        {
            return false;
        }

        const unsigned long requiredSamples = MazeMap::ComputeGyroBiasSampleCount(
            static_cast<unsigned long>(MazeMap::Config::kGyroBiasSamples),
            kImuCalibrationSampleIntervalMs,
            static_cast<unsigned long>(MazeMap::Config::kGyroBiasMinimumAveragingWindowMs));
        const unsigned long measurementStartMs = millis();
        double accumulatedRadps = 0.0;
        double accumulatedAccelXG = 0.0;
        double accumulatedAccelYG = 0.0;
        unsigned long collectedSamples = 0UL;
        while ((collectedSamples < requiredSamples) ||
            ((millis() - measurementStartMs) < static_cast<unsigned long>(MazeMap::Config::kGyroBiasMinimumAveragingWindowMs)))
        {
            if (MazeMap::HaveEncoderCountsChanged(startCounts, captureEncoderCounts()))
            {
                (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                    "Encoder motion detected during gyro bias measurement; restarting IMU self-test");
                accumulatedRadps = 0.0;
                collectedSamples = 0UL;
                break;
            }

            if (captureAccelBias)
            {
                const MazeMap::Vehicle::ImuBackLeft::Axes accel = vehicle.IMU_BL.ReadAccel();
                accumulatedAccelXG += static_cast<double>(vehicle.IMU_BL.AccelRawToG(accel.x));
                accumulatedAccelYG += static_cast<double>(vehicle.IMU_BL.AccelRawToG(accel.y));
            }
            accumulatedRadps += static_cast<double>(ReadBackLeftGyroZRadpsRaw(vehicle));
            ++collectedSamples;
            delay(kImuCalibrationSampleIntervalMs);
        }

        if (collectedSamples == 0UL)
        {
            continue;
        }

        if (MazeMap::HaveEncoderCountsChanged(startCounts, captureEncoderCounts()))
        {
            (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine(
                "Encoder motion detected after gyro bias capture; restarting IMU self-test");
            continue;
        }

        gyroBiasRadps = static_cast<float>(accumulatedRadps / static_cast<double>(collectedSamples));
        if (captureAccelBias)
        {
            *accelBiasXG = static_cast<float>(accumulatedAccelXG / static_cast<double>(collectedSamples));
            *accelBiasYG = static_cast<float>(accumulatedAccelYG / static_cast<double>(collectedSamples));
        }
        if (accelBiasInitialized != nullptr)
        {
            *accelBiasInitialized = captureAccelBias;
        }
        return true;
    }
#else
    (void)controlPeriodUs;
    (void)enableAccelRuntime;
    (void)accelBiasXG;
    (void)accelBiasYG;
    (void)accelBiasInitialized;
    (void)captureEncoderCounts;
    (void)accelFilterFreq;
    gyroBiasRadps = EstimateMissionGyroBiasRadps(vehicle);
    return true;
#endif
}

bool RuntimeSensorSuite::CaptureServices::ServiceWallRead() noexcept
{
    return (_serviceWallRead != nullptr) ? _serviceWallRead(_context) : false;
}

void RuntimeSensorSuite::CaptureServices::CaptureImu() noexcept
{
    if (_captureImu != nullptr)
    {
        _captureImu(_context);
    }
}

RuntimeSensorSuite::RuntimeSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
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

bool RuntimeSensorSuite::Begin(const unsigned long controlPeriodUs)
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
    (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogLine("IMU_FR disabled; using IMU_BL only");

    const bool imuBackLeftOk = _vehicle.IMU_BL.Begin();
    if (!imuBackLeftOk)
    {
        const uint8_t whoAmI = _vehicle.IMU_BL.GetLastWhoAmI();
        const uint8_t whoAmIMode3 = _vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE3);
        const uint8_t whoAmIMode0 = _vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE0);
        char whoAmIBuffer[3] = {};
        char whoAmIMode3Buffer[3] = {};
        char whoAmIMode0Buffer[3] = {};
        FormatHexByte(whoAmI, whoAmIBuffer);
        FormatHexByte(whoAmIMode3, whoAmIMode3Buffer);
        FormatHexByte(whoAmIMode0, whoAmIMode0Buffer);
        (void)MazeMap::App::Internal::GetSharedRobotRuntime().AppendTextLogFormatted(
            "IMU_BL init failed (%s), WHO_AM_I=0x%s, INT1_pullup=%s, WHO_AM_I@mode3/400kHz=0x%s, WHO_AM_I@mode0/400kHz=0x%s",
            _vehicle.IMU_BL.GetLastBeginFailureReasonName(),
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
    auto captureEncoderCounts = [this]() noexcept
    {
        const MazeMap::EncoderObs observation = _vehicle.CaptureEncoderObservation(0.0f);
        return MazeMap::EncoderCountPair{ observation.totalLeftCounts, observation.totalRightCounts };
    };

    return CalibrateStationaryBackLeftGyroBias(
        _vehicle,
        controlPeriodUs,
        enableAccelRuntime,
        _gyroBiasRadps,
        &_accelBiasXG,
        &_accelBiasYG,
        &_accelBiasInitialized,
        captureEncoderCounts,
        Config::kMissionRuntimeAccelFilterFreq);
}

void RuntimeSensorSuite::ResetSideWallMemory() noexcept
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

void RuntimeSensorSuite::Capture(
    const bool stationary,
    const MazeMap::VehicleState& state,
    SensorSnapshot& snapshot,
    const CaptureHandler callback,
    void* const callbackContext,
    const bool captureEncoders,
    const float encoderDtSeconds)
{
    snapshot = SensorSnapshot{};
    if (captureEncoders)
    {
        snapshot.encoderObservation = _vehicle.CaptureEncoderObservation(encoderDtSeconds);
        snapshot.encoderObservationValid = true;
    }

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
    class CaptureContext final
    {
    public:
        CaptureContext(
            AsyncWallSensorSweepRead* wallRead,
            bool* imuCaptured,
            RuntimeSensorSuite* owner,
            const bool stationary,
            SensorSnapshot* snapshot) noexcept
            : _wallRead(wallRead)
            , _imuCaptured(imuCaptured)
            , _owner(owner)
            , _stationary(stationary)
            , _snapshot(snapshot)
        {
        }

        AsyncWallSensorSweepRead& WallRead() const noexcept
        {
            return *_wallRead;
        }

        bool& ImuCaptured() const noexcept
        {
            return *_imuCaptured;
        }

        RuntimeSensorSuite& Owner() const noexcept
        {
            return *_owner;
        }

        bool Stationary() const noexcept
        {
            return _stationary;
        }

        SensorSnapshot& Snapshot() const noexcept
        {
            return *_snapshot;
        }

    private:
        AsyncWallSensorSweepRead* _wallRead{};
        bool* _imuCaptured{};
        RuntimeSensorSuite* _owner{};
        bool _stationary{};
        SensorSnapshot* _snapshot{};
    } captureContext{ &wallRead, &imuCaptured, this, stationary, &snapshot };

    CaptureServices services{};
    services._context = &captureContext;
    services._serviceWallRead = [](void* rawContext) noexcept -> bool
    {
        auto& context = *static_cast<CaptureContext*>(rawContext);
        return ServiceAsyncWallSensorSweepRead(context.WallRead());
    };
    services._captureImu = [](void* rawContext) noexcept
    {
        auto& context = *static_cast<CaptureContext*>(rawContext);
        (void)ServiceAsyncWallSensorSweepRead(context.WallRead());
        if (!context.ImuCaptured())
        {
            context.Owner().CaptureInertialSnapshot(context.Stationary(), context.Snapshot());
            context.ImuCaptured() = true;
        }
        (void)ServiceAsyncWallSensorSweepRead(context.WallRead());
    };

    if (callback != nullptr)
    {
        callback(callbackContext, snapshot, services);
    }
    else
    {
        services.CaptureImu();
    }

    if (!imuCaptured)
    {
        services.CaptureImu();
    }

    (void)services.ServiceWallRead();
    if (wallRead.active)
    {
        AwaitAsyncWallSensorSweepRead(wallRead);
    }

    _frontLeftLedOffCommandUs = wallRead.nextFrontLeftLedOffCommandUs;
    _frontRightLedOffCommandUs = wallRead.nextFrontRightLedOffCommandUs;
    _sideLeftLedOffCommandUs = wallRead.nextSideLeftLedOffCommandUs;
    _sideRightLedOffCommandUs = wallRead.nextSideRightLedOffCommandUs;

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

    float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
    float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
    _wallCalibration.TryComputeSideWallDistanceThresholds(
        Config::kSideWallDistanceLatchFractionOfCalibration,
        Config::kSideWallDistanceReleaseFractionOfCalibration,
        sideWallOnThresholdM,
        sideWallOffThresholdM);

    snapshot.frontLeftDistanceM = UpdateChannelFromMeasuredDistance(_frontLeft, snapshot.frontLeft.distanceM);
    snapshot.frontRightDistanceM = UpdateChannelFromMeasuredDistance(_frontRight, snapshot.frontRight.distanceM);
    snapshot.frontLeftDifferentialLight = snapshot.frontLeft.differentialLight;
    snapshot.frontRightDifferentialLight = snapshot.frontRight.differentialLight;

    (void)TryComputeSideWallSignalDistanceM(
        _wallCalibration,
        WallSensorId::SideLeft,
        snapshot.sideLeft.differentialLight,
        snapshot.sideLeft.distanceM);
    (void)TryComputeSideWallSignalDistanceM(
        _wallCalibration,
        WallSensorId::SideRight,
        snapshot.sideRight.differentialLight,
        snapshot.sideRight.distanceM);
    snapshot.sideLeftDistanceM = UpdateChannelFromMeasuredDistance(_sideLeft, snapshot.sideLeft.distanceM);
    snapshot.sideRightDistanceM = UpdateChannelFromMeasuredDistance(_sideRight, snapshot.sideRight.distanceM);
    snapshot.sideLeftDifferentialLight = snapshot.sideLeft.differentialLight;
    snapshot.sideRightDifferentialLight = snapshot.sideRight.differentialLight;

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

    const bool sideLeftWindowValid = IsSideWallDetectionWindowValid(state, _vehicle.SideLeft);
    const bool sideRightWindowValid = IsSideWallDetectionWindowValid(state, _vehicle.SideRight);

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

    snapshot.frontLeft.wall = snapshot.frontLeftWall;
    snapshot.frontRight.wall = snapshot.frontRightWall;
    snapshot.sideLeft.wall = snapshot.leftWall;
    snapshot.sideRight.wall = snapshot.rightWall;
    snapshot.frontSkewM = snapshot.frontLeftDistanceM - snapshot.frontRightDistanceM;
    snapshot.corridorErrorM = MazeMap::App::Internal::Runtime::ComputeCorridorError(
        snapshot.sideLeftDistanceM,
        snapshot.sideRightDistanceM,
        snapshot.leftDistanceValidForControl,
        snapshot.rightDistanceValidForControl,
        _wallCalibration.GetExpectedSideWallDistanceM());
}

float RuntimeSensorSuite::GetGyroSensitivityMdpsPerLsb() const noexcept
{
    return _vehicle.IMU_BL.GyroSensitivityMdpsPerLsb();
}

float RuntimeSensorSuite::GetAccelSensitivityMgPerLsb() const noexcept
{
    return _vehicle.IMU_BL.AccelSensitivityMgPerLsb();
}

float RuntimeSensorSuite::GetGyroBiasRadps() const noexcept
{
    return _gyroBiasRadps;
}

bool RuntimeSensorSuite::HasAccelBias() const noexcept
{
    return _accelBiasInitialized;
}

float RuntimeSensorSuite::GetAccelBiasXG() const noexcept
{
    return _accelBiasXG;
}

float RuntimeSensorSuite::GetAccelBiasYG() const noexcept
{
    return _accelBiasYG;
}

void RuntimeSensorSuite::InitializeWallSensorLedOffState() noexcept
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

void RuntimeSensorSuite::CaptureInertialSnapshot(const bool stationary, SensorSnapshot& snapshot)
{
    const MazeMap::SensorMount imuMount = MazeMap::Vehicle::GetBackLeftImuMount();
    snapshot.imuFrontRight = {};
    snapshot.imuBackLeft = CaptureImuTelemetry(_vehicle.IMU_BL, Pins::IMU_INT_1B, &snapshot.imuTiming);

    float rawGyroRadps = imuMount.TransformClockwiseYawRateToBody(ReadGyroZRadpsRaw());
#if defined(ARDUINO_TEENSY41)
    rawGyroRadps =
        imuMount.TransformClockwiseYawRateToBody(
            _vehicle.IMU_BL.GyroRawToClockwiseYawDps(snapshot.imuBackLeft.gyroZ) * DEG_TO_RAD_F);

    const Eigen::Vector2f accelImuG(
        _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelX),
        _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelY));
    const Eigen::Vector2f accelBodyG = imuMount.TransformPlanarVectorToBody(accelImuG);
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

    if (stationary &&
        MazeMap::ShouldUpdateGyroBiasFromStationarySample(rawGyroRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
    {
        _gyroBiasRadps = (0.995f * _gyroBiasRadps) + (0.005f * rawGyroRadps);
    }
    snapshot.gyroRawRadps = rawGyroRadps;
    snapshot.gyroBiasRadps = _gyroBiasRadps;
    snapshot.gyroRadps = rawGyroRadps - _gyroBiasRadps;
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

float RuntimeSensorSuite::UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, const float measuredDistanceM)
{
    channel.filteredDistanceM = measuredDistanceM;
    channel.initialized = true;
    return measuredDistanceM;
}

float RuntimeSensorSuite::UpdateChannelFromMeasuredDistance(
    FilteredIrChannel& channel,
    const float measuredDistanceM,
    const float onThresholdM,
    const float offThresholdM)
{
    (void)offThresholdM;
    const float currentDistanceM = UpdateChannelFromMeasuredDistance(channel, measuredDistanceM);
    channel.wall = currentDistanceM < onThresholdM;
    return currentDistanceM;
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
