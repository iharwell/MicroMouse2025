#include "pch.h"
#include "RuntimeSensorSuite.h"

#include "Pins.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "SharedRobotRuntime.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallDetectionThresholds.h"
#include "WallDistanceCalibration.h"
#include "WallSensor.h"
#include "WallSensorPreprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace MazeMap
{
    void RuntimeSensorSuite::WallTelemetryAverager::Clear() noexcept
    {
        _ambientLight.Clear();
        _litLight.Clear();
        _differentialLight.Clear();
        _rawDistanceM.Clear();
    }

    void RuntimeSensorSuite::WallTelemetryAverager::PushAndAverage(
        const MazeMap::WallSensor& sensor) noexcept
    {
        _ambientLight.Push(sensor.LatestAmbientLight());
        _litLight.Push(sensor.LatestLitLight());
        _differentialLight.Push(sensor.LatestDifferentialLight());
        _rawDistanceM.Push(sensor.LatestRawDistanceM());
    }

    float RuntimeSensorSuite::WallTelemetryAverager::AmbientLight() const noexcept
    {
        return _ambientLight.Average();
    }

    float RuntimeSensorSuite::WallTelemetryAverager::LitLight() const noexcept
    {
        return _litLight.Average();
    }

    float RuntimeSensorSuite::WallTelemetryAverager::DifferentialLight() const noexcept
    {
        return _differentialLight.Average();
    }

    float RuntimeSensorSuite::WallTelemetryAverager::RawDistanceM() const noexcept
    {
        return _rawDistanceM.Average();
    }

    RuntimeSensorSuite::RuntimeSensorSuite(MazeMap::Vehicle& vehicle, ::WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
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
        , _encoderObservationDegraded(false)
    {
    }

    void RuntimeSensorSuite::AttachRuntime(App::Internal::SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
    }

    bool RuntimeSensorSuite::AppendTextLogLine(const char* const line) noexcept
    {
        return (_runtime != nullptr) && _runtime->AppendTextLogLine(line);
    }

    bool RuntimeSensorSuite::AppendTextLogFormatted(const char* const format, ...) noexcept
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[384] = {};
        va_list args;
        va_start(args, format);
        const int length = std::vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (length <= 0)
        {
            return false;
        }

        line[sizeof(line) - 1U] = '\0';
        return AppendTextLogLine(line);
    }

    bool RuntimeSensorSuite::Begin(const unsigned long controlPeriodUs)
    {
        _frontLeftWallSignalFiltered = 0.0f;
        _frontRightWallSignalFiltered = 0.0f;
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _frontLeftTelemetryAverage.Clear();
        _frontRightTelemetryAverage.Clear();
        _sideLeftTelemetryAverage.Clear();
        _sideRightTelemetryAverage.Clear();
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
        _leftEncoderTotalCounts = 0;
        _rightEncoderTotalCounts = 0;
        _encoderObservationDegraded = false;
        _vehicle.BackLeftImu().ResetRuntimeCalibration();
        InitializeWallSensorLedOffState();

        bool ok = true;
    #if defined(ARDUINO_TEENSY41)
        (void)AppendTextLogLine("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.BackLeftImu().Begin();
        if (!imuBackLeftOk)
        {
            const std::uint8_t whoAmI = _vehicle.BackLeftImu().GetLastWhoAmI();
            const std::uint8_t whoAmIMode3 = _vehicle.BackLeftImu().ReadWhoAmIWithSettings(400000UL, SPI_MODE3);
            const std::uint8_t whoAmIMode0 = _vehicle.BackLeftImu().ReadWhoAmIWithSettings(400000UL, SPI_MODE0);
            char whoAmIBuffer[3] = {};
            char whoAmIMode3Buffer[3] = {};
            char whoAmIMode0Buffer[3] = {};
            std::snprintf(whoAmIBuffer, sizeof(whoAmIBuffer), "%02X", static_cast<unsigned>(whoAmI));
            std::snprintf(whoAmIMode3Buffer, sizeof(whoAmIMode3Buffer), "%02X", static_cast<unsigned>(whoAmIMode3));
            std::snprintf(whoAmIMode0Buffer, sizeof(whoAmIMode0Buffer), "%02X", static_cast<unsigned>(whoAmIMode0));
            pinMode(Pins::IMU_INT_1B, INPUT_PULLUP);
            const bool imuInterruptDrivenLow = digitalRead(Pins::IMU_INT_1B) == LOW;
            pinMode(Pins::IMU_INT_1B, INPUT);
            (void)AppendTextLogFormatted(
                "IMU_BL init failed (%s), WHO_AM_I=0x%s, INT1_pullup=%s, WHO_AM_I@mode3/400kHz=0x%s, WHO_AM_I@mode0/400kHz=0x%s",
                _vehicle.BackLeftImu().GetLastBeginFailureReasonName(),
                whoAmIBuffer,
                imuInterruptDrivenLow ? "low" : "high",
                whoAmIMode3Buffer,
                whoAmIMode0Buffer);
        }
        ok = imuBackLeftOk && ok;
    #endif

        if (ok && !_vehicle.BackLeftImu().ConfigureRuntimeForControlPeriod(
                controlPeriodUs,
                true,
                Config::kMissionRuntimeAccelFilterFreq))
        {
            (void)AppendTextLogFormatted("Unsupported IMU control period us: %lu", controlPeriodUs);
            ok = false;
        }

        return ok;
    }

    void RuntimeSensorSuite::ResetSideWallMemory() noexcept
    {
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _sideLeftTelemetryAverage.Clear();
        _sideRightTelemetryAverage.Clear();
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
        const MazeMap::VehicleState& state,
        SensorSnapshot& snapshot,
        const std::uint8_t sensorWorkBits,
        const float encoderDtSeconds)
    {
        const bool captureWalls = SensorWorkBitsRequestWallSensors(sensorWorkBits);
        const bool captureEncoders = (sensorWorkBits & kEncoderSensorBit) != 0U;
        if (_interlacedCaptureActive && _interlacedCaptureWalls)
        {
            AbortInterlacedWallCapture();
        }
        ClearInterlacedCaptureState();

        snapshot = SensorSnapshot{};
        _interlacedCaptureActive = true;
        _interlacedCaptureSensorWorkBits = sensorWorkBits;
        _interlacedCaptureWalls = captureWalls;
        _interlacedCaptureInertial = (sensorWorkBits & (kGyroSensorBit | kAccelSensorBit)) != 0U;
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

        _vehicle.FrontLeftWallSensor().CaptureAmbientRead();
        _vehicle.FrontRightWallSensor().CaptureAmbientRead();
        _vehicle.SideLeftWallSensor().CaptureAmbientRead();
        _vehicle.SideRightWallSensor().CaptureAmbientRead();

        const std::uint32_t frontLedOnCommandUs = micros();
        _vehicle.FrontLeftWallSensor().CommandLedOn(frontLedOnCommandUs);
        _vehicle.FrontRightWallSensor().CommandLedOn(frontLedOnCommandUs);
        _frontWallCollectionReadyUs = (std::max)(
            _vehicle.FrontLeftWallSensor().LitReadyUs(),
            _vehicle.FrontRightWallSensor().LitReadyUs());
        _frontWallCollectionPending = true;
        _interlacedWallCaptureActive = true;
    }

    void RuntimeSensorSuite::CaptureInterlacedInertialSnapshot() noexcept
    {
        if (!_interlacedCaptureActive ||
            (_interlacedCaptureSnapshot == nullptr) ||
            !_interlacedCaptureInertial ||
            _interlacedCaptureImuCaptured)
        {
            return;
        }

        CaptureInertialSnapshot(*_interlacedCaptureSnapshot);
        _interlacedCaptureImuCaptured = true;
    }

    void RuntimeSensorSuite::ServiceFrontWallCollection() noexcept
    {
        if (!_frontWallCollectionPending)
        {
            return;
        }

        const std::uint32_t nowUs = micros();
        if ((static_cast<std::int32_t>(nowUs - _frontWallCollectionReadyUs) < 0) ||
            !_vehicle.FrontLeftWallSensor().IsLitReadReady(nowUs) ||
            !_vehicle.FrontRightWallSensor().IsLitReadReady(nowUs))
        {
            return;
        }

        _vehicle.FrontLeftWallSensor().CaptureLitRead();
        _vehicle.FrontRightWallSensor().CaptureLitRead();
        const std::uint32_t ledOffCommandUs = micros();
        _vehicle.FrontLeftWallSensor().CompleteCapture(ledOffCommandUs);
        _vehicle.FrontRightWallSensor().CompleteCapture(ledOffCommandUs);
        _frontWallCollectionPending = false;

        const std::uint32_t leftLedOnCommandUs = micros();
        _vehicle.SideLeftWallSensor().CommandLedOn(leftLedOnCommandUs);
        _leftWallCollectionReadyUs = _vehicle.SideLeftWallSensor().LitReadyUs();
        _leftWallCollectionPending = true;
    }

    void RuntimeSensorSuite::ServiceLeftWallCollection() noexcept
    {
        if (!_leftWallCollectionPending)
        {
            return;
        }

        const std::uint32_t nowUs = micros();
        if ((static_cast<std::int32_t>(nowUs - _leftWallCollectionReadyUs) < 0) ||
            !_vehicle.SideLeftWallSensor().IsLitReadReady(nowUs))
        {
            return;
        }

        _vehicle.SideLeftWallSensor().CaptureLitRead();
        const std::uint32_t ledOffCommandUs = micros();
        _vehicle.SideLeftWallSensor().CompleteCapture(ledOffCommandUs);
        _leftWallCollectionPending = false;

        const std::uint32_t rightLedOnCommandUs = micros();
        _vehicle.SideRightWallSensor().CommandLedOn(rightLedOnCommandUs);
        _rightWallCollectionReadyUs = _vehicle.SideRightWallSensor().LitReadyUs();
        _rightWallCollectionPending = true;
    }

    void RuntimeSensorSuite::ServiceRightWallCollection() noexcept
    {
        if (!_rightWallCollectionPending)
        {
            return;
        }

        const std::uint32_t nowUs = micros();
        if ((static_cast<std::int32_t>(nowUs - _rightWallCollectionReadyUs) < 0) ||
            !_vehicle.SideRightWallSensor().IsLitReadReady(nowUs))
        {
            return;
        }

        _vehicle.SideRightWallSensor().CaptureLitRead();
        const std::uint32_t ledOffCommandUs = micros();
        _vehicle.SideRightWallSensor().CompleteCapture(ledOffCommandUs);
        _rightWallCollectionPending = false;
        _interlacedWallCaptureActive = false;
    }

    void RuntimeSensorSuite::AbortInterlacedWallCapture() noexcept
    {
        if (!_interlacedWallCaptureActive)
        {
            return;
        }

        const std::uint32_t ledOffCommandUs = micros();
        if (_frontWallCollectionPending)
        {
            _vehicle.FrontLeftWallSensor().CommandLedOff(ledOffCommandUs);
            _vehicle.FrontRightWallSensor().CommandLedOff(ledOffCommandUs);
        }
        if (_leftWallCollectionPending)
        {
            _vehicle.SideLeftWallSensor().CommandLedOff(ledOffCommandUs);
        }
        if (_rightWallCollectionPending)
        {
            _vehicle.SideRightWallSensor().CommandLedOff(ledOffCommandUs);
        }

        _interlacedWallCaptureActive = false;
        _frontWallCollectionPending = false;
        _leftWallCollectionPending = false;
        _rightWallCollectionPending = false;
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
            FinalizeInterlacedSnapshot(snapshot);
            ClearInterlacedCaptureState();
            return;
        }

        ServiceFrontWallCollection();
        ServiceLeftWallCollection();
        ServiceRightWallCollection();
        if (_interlacedWallCaptureActive)
        {
            if (!_interlacedCaptureIncompleteLogged)
            {
                (void)AppendTextLogLine(
                    "Interlaced wall capture did not complete before tick finish; omitting wall observation for this tick");
                _interlacedCaptureIncompleteLogged = true;
            }
            AbortInterlacedWallCapture();
            FinalizeInterlacedSnapshot(snapshot);
            ClearInterlacedCaptureState();
            return;
        }

        _frontLeftTelemetryAverage.PushAndAverage(_vehicle.FrontLeftWallSensor());
        _frontRightTelemetryAverage.PushAndAverage(_vehicle.FrontRightWallSensor());
        _sideLeftTelemetryAverage.PushAndAverage(_vehicle.SideLeftWallSensor());
        _sideRightTelemetryAverage.PushAndAverage(_vehicle.SideRightWallSensor());

        const float frontLeftAmbientLight = _frontLeftTelemetryAverage.AmbientLight();
        const float frontLeftLitLight = _frontLeftTelemetryAverage.LitLight();
        const float frontLeftDifferentialLight = _frontLeftTelemetryAverage.DifferentialLight();
        const float frontLeftRawDistanceM = _frontLeftTelemetryAverage.RawDistanceM();
        const float frontLeftDistanceM = _wallCalibration.ApplyFrontLeft(
            frontLeftDifferentialLight,
            frontLeftRawDistanceM);

        const float frontRightAmbientLight = _frontRightTelemetryAverage.AmbientLight();
        const float frontRightLitLight = _frontRightTelemetryAverage.LitLight();
        const float frontRightDifferentialLight = _frontRightTelemetryAverage.DifferentialLight();
        const float frontRightRawDistanceM = _frontRightTelemetryAverage.RawDistanceM();
        const float frontRightDistanceM = _wallCalibration.ApplyFrontRight(
            frontRightDifferentialLight,
            frontRightRawDistanceM);

        const float sideLeftAmbientLight = _sideLeftTelemetryAverage.AmbientLight();
        const float sideLeftLitLight = _sideLeftTelemetryAverage.LitLight();
        const float sideLeftDifferentialLight = _sideLeftTelemetryAverage.DifferentialLight();
        const float sideLeftRawDistanceM = _sideLeftTelemetryAverage.RawDistanceM();
        float sideLeftDistanceM = _wallCalibration.ApplySide(
            MazeMap::RelativeDirection::Left90,
            sideLeftRawDistanceM,
            sideLeftRawDistanceM);

        const float sideRightAmbientLight = _sideRightTelemetryAverage.AmbientLight();
        const float sideRightLitLight = _sideRightTelemetryAverage.LitLight();
        const float sideRightDifferentialLight = _sideRightTelemetryAverage.DifferentialLight();
        const float sideRightRawDistanceM = _sideRightTelemetryAverage.RawDistanceM();
        float sideRightDistanceM = _wallCalibration.ApplySide(
            MazeMap::RelativeDirection::Right90,
            sideRightRawDistanceM,
            sideRightRawDistanceM);

        snapshot.SetFrontTiming(_vehicle.FrontLeftWallSensor().LatestTiming());
        snapshot.SetLeftTiming(_vehicle.SideLeftWallSensor().LatestTiming());
        snapshot.SetRightTiming(_vehicle.SideRightWallSensor().LatestTiming());

        snapshot.SetFrontLeftDistanceM(frontLeftDistanceM);
        snapshot.SetFrontRightDistanceM(frontRightDistanceM);
        snapshot.SetFrontLeftDifferentialLight(frontLeftDifferentialLight);
        snapshot.SetFrontRightDifferentialLight(frontRightDifferentialLight);

        (void)_wallCalibration.TryComputeSideWallSignalDistanceM(
            MazeMap::RelativeDirection::Left90,
            sideLeftDifferentialLight,
            sideLeftDistanceM);
        (void)_wallCalibration.TryComputeSideWallSignalDistanceM(
            MazeMap::RelativeDirection::Right90,
            sideRightDifferentialLight,
            sideRightDistanceM);

        WallSensorPreprocessor wallPreprocessor{};
        const WallObs frontLeftWallSensorObservation = wallPreprocessor.process(
            _vehicle.FrontLeftWallSensor(),
            frontLeftAmbientLight,
            frontLeftLitLight,
            frontLeftDistanceM);
        const WallObs frontRightWallSensorObservation = wallPreprocessor.process(
            _vehicle.FrontRightWallSensor(),
            frontRightAmbientLight,
            frontRightLitLight,
            frontRightDistanceM);
        const WallObs sideLeftWallSensorObservation = wallPreprocessor.process(
            _vehicle.SideLeftWallSensor(),
            sideLeftAmbientLight,
            sideLeftLitLight,
            sideLeftDistanceM);
        const WallObs sideRightWallSensorObservation = wallPreprocessor.process(
            _vehicle.SideRightWallSensor(),
            sideRightAmbientLight,
            sideRightLitLight,
            sideRightDistanceM);

        snapshot.SetSideLeftDistanceM(sideLeftDistanceM);
        snapshot.SetSideRightDistanceM(sideRightDistanceM);
        snapshot.SetSideLeftDifferentialLight(sideLeftDifferentialLight);
        snapshot.SetSideRightDifferentialLight(sideRightDifferentialLight);

        snapshot.SetFrontWall(UpdateFrontWallState(
            frontLeftAmbientLight,
            frontLeftDifferentialLight,
            frontRightAmbientLight,
            frontRightDifferentialLight,
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
            _vehicle.SideLeftWallSensor().IsSideWallSegmentCenterAligned(*state);
        const bool sideRightWindowValid =
            (state != nullptr) &&
            _vehicle.SideRightWallSensor().IsSideWallSegmentCenterAligned(*state);

        float sideLeftSignalRise = 0.0f;
        float sideLeftLatchRiseThreshold = 0.0f;
        float sideLeftMissRiseThreshold = 0.0f;
        const bool sideLeftSignalMetricsValid = _wallCalibration.TryComputeSideWallSignalRiseMetrics(
            MazeMap::RelativeDirection::Left90,
            sideLeftDifferentialLight,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            Config::kWallMapMissSignalFractionOfLatch,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold,
            sideLeftMissRiseThreshold);

        float sideRightSignalRise = 0.0f;
        float sideRightLatchRiseThreshold = 0.0f;
        float sideRightMissRiseThreshold = 0.0f;
        const bool sideRightSignalMetricsValid = _wallCalibration.TryComputeSideWallSignalRiseMetrics(
            MazeMap::RelativeDirection::Right90,
            sideRightDifferentialLight,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            Config::kWallMapMissSignalFractionOfLatch,
            sideRightSignalRise,
            sideRightLatchRiseThreshold,
            sideRightMissRiseThreshold);

        const bool sideLeftSignalClassifiable = _wallCalibration.IsSideWallSignalClassifiable(
            sideLeftSignalMetricsValid,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold,
            sideLeftMissRiseThreshold);
        const bool sideRightSignalClassifiable = _wallCalibration.IsSideWallSignalClassifiable(
            sideRightSignalMetricsValid,
            sideRightSignalRise,
            sideRightLatchRiseThreshold,
            sideRightMissRiseThreshold);
        const bool sideLeftObservationEligible = _wallCalibration.IsSideWallObservationEligible(
            sideLeftWindowValid,
            sideLeftSignalClassifiable);
        const bool sideRightObservationEligible = _wallCalibration.IsSideWallObservationEligible(
            sideRightWindowValid,
            sideRightSignalClassifiable);

        snapshot.SetLeftTransitionDetected(_wallCalibration.DetectSideWallTransitionFromSignalRise(
            sideLeftWindowValid,
            sideLeftSignalMetricsValid,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold,
            Config::kSideWallTransitionSignalFractionOfLatch,
            _sideLeftPreviousSignalRise,
            _sideLeftPreviousSignalRiseValid));
        snapshot.SetRightTransitionDetected(_wallCalibration.DetectSideWallTransitionFromSignalRise(
            sideRightWindowValid,
            sideRightSignalMetricsValid,
            sideRightSignalRise,
            sideRightLatchRiseThreshold,
            Config::kSideWallTransitionSignalFractionOfLatch,
            _sideRightPreviousSignalRise,
            _sideRightPreviousSignalRiseValid));
        const bool sideLeftControlRangeValid = _wallCalibration.IsSideWallControlRangeValid(
            sideLeftObservationEligible,
            snapshot.LeftTransitionDetected(),
            sideLeftSignalMetricsValid,
            sideLeftSignalRise,
            sideLeftLatchRiseThreshold);
        const bool sideRightControlRangeValid = _wallCalibration.IsSideWallControlRangeValid(
            sideRightObservationEligible,
            snapshot.RightTransitionDetected(),
            sideRightSignalMetricsValid,
            sideRightSignalRise,
            sideRightLatchRiseThreshold);
        snapshot.SetLeftWallObservationWindowValid(sideLeftObservationEligible);
        snapshot.SetRightWallObservationWindowValid(sideRightObservationEligible);
        snapshot.SetLeftDistanceValidForControl(sideLeftControlRangeValid);
        snapshot.SetRightDistanceValidForControl(sideRightControlRangeValid);
        snapshot.SetLeftWall(_wallCalibration.UpdateSideWallState(
            MazeMap::RelativeDirection::Left90,
            sideLeftDifferentialLight,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _sideLeftWallState));
        snapshot.SetRightWall(_wallCalibration.UpdateSideWallState(
            MazeMap::RelativeDirection::Right90,
            sideRightDifferentialLight,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _sideRightWallState));
        snapshot.SetLeftWallObservation(_wallCalibration.ComputeSideWallObservationHit(
            MazeMap::RelativeDirection::Left90,
            sideLeftDifferentialLight,
            sideLeftObservationEligible));
        snapshot.SetRightWallObservation(_wallCalibration.ComputeSideWallObservationHit(
            MazeMap::RelativeDirection::Right90,
            sideRightDifferentialLight,
            sideRightObservationEligible));

        snapshot.SetFrontLeftTelemetryValues(
            frontLeftAmbientLight,
            frontLeftLitLight,
            frontLeftDifferentialLight,
            frontLeftRawDistanceM,
            frontLeftDistanceM,
            snapshot.HasFrontLeftWall());
        snapshot.SetFrontRightTelemetryValues(
            frontRightAmbientLight,
            frontRightLitLight,
            frontRightDifferentialLight,
            frontRightRawDistanceM,
            frontRightDistanceM,
            snapshot.HasFrontRightWall());
        snapshot.SetSideLeftTelemetryValues(
            sideLeftAmbientLight,
            sideLeftLitLight,
            sideLeftDifferentialLight,
            sideLeftRawDistanceM,
            sideLeftDistanceM,
            snapshot.HasLeftWall());
        snapshot.SetSideRightTelemetryValues(
            sideRightAmbientLight,
            sideRightLitLight,
            sideRightDifferentialLight,
            sideRightRawDistanceM,
            sideRightDistanceM,
            snapshot.HasRightWall());
        snapshot.SetFrontLeftWallSensorObservation(frontLeftWallSensorObservation);
        snapshot.SetFrontRightWallSensorObservation(frontRightWallSensorObservation);
        snapshot.SetSideLeftWallSensorObservation(sideLeftWallSensorObservation);
        snapshot.SetSideRightWallSensorObservation(sideRightWallSensorObservation);
        snapshot.SetFrontSkewM(snapshot.FrontLeftDistanceM() - snapshot.FrontRightDistanceM());

        FinalizeInterlacedSnapshot(snapshot);

        ClearInterlacedCaptureState();
    }

    void RuntimeSensorSuite::FinalizeInterlacedSnapshot(SensorSnapshot& snapshot) const noexcept
    {
        snapshot.ClearUnavailableObservations(
            _interlacedCaptureInertial,
            (_interlacedCaptureSensorWorkBits & kFrontWallSensorBit) != 0U,
            (_interlacedCaptureSensorWorkBits & kLeftWallSensorBit) != 0U,
            (_interlacedCaptureSensorWorkBits & kRightWallSensorBit) != 0U);
        snapshot.RecomputeCorridorErrorM(Config::kExpectedSideWallDistanceM);
    }

    void RuntimeSensorSuite::ClearInterlacedCaptureState() noexcept
    {
        _interlacedCaptureState = nullptr;
        _interlacedCaptureSnapshot = nullptr;
        _interlacedCaptureSensorWorkBits = 0U;
        _interlacedCaptureActive = false;
        _interlacedCaptureWalls = false;
        _interlacedCaptureInertial = false;
        _interlacedWallCaptureActive = false;
        _frontWallCollectionPending = false;
        _leftWallCollectionPending = false;
        _rightWallCollectionPending = false;
        _interlacedCaptureImuCaptured = false;
        _frontWallCollectionReadyUs = 0UL;
        _leftWallCollectionReadyUs = 0UL;
        _rightWallCollectionReadyUs = 0UL;
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
        return _vehicle.BackLeftImu().RuntimeGyroBiasRadps();
    }

    bool RuntimeSensorSuite::HasAccelBias() const noexcept
    {
        return _vehicle.BackLeftImu().HasRuntimeAccelBias();
    }

    float RuntimeSensorSuite::GetAccelBiasRightG() const noexcept
    {
        return _vehicle.BackLeftImu().RuntimeAccelBiasRightG();
    }

    float RuntimeSensorSuite::GetAccelBiasForwardG() const noexcept
    {
        return _vehicle.BackLeftImu().RuntimeAccelBiasForwardG();
    }

    bool RuntimeSensorSuite::SensorWorkBitsRequestWallSensors(const std::uint8_t sensorWorkBits) noexcept
    {
        return (sensorWorkBits & kWallSensorBits) != 0U;
    }

    bool RuntimeSensorSuite::SensorWorkBitsRequestCapture(const std::uint8_t sensorWorkBits) noexcept
    {
        return
            SensorWorkBitsRequestWallSensors(sensorWorkBits) ||
            ((sensorWorkBits & (kGyroSensorBit | kAccelSensorBit | kEncoderSensorBit)) != 0U);
    }

    bool RuntimeSensorSuite::SensorWorkBitsSupportWallUpdates(const std::uint8_t sensorWorkBits) noexcept
    {
        return
            ((sensorWorkBits & kWallUpdateSensorBit) == 0U) ||
            SensorWorkBitsRequestWallSensors(sensorWorkBits);
    }

    MazeMap::EncoderObs RuntimeSensorSuite::CaptureEncoderObservation(const float dtSeconds) noexcept
    {
        MazeMap::EncoderObs observation = _vehicle.CaptureEncoderObservation(dtSeconds);
        _leftEncoderTotalCounts += static_cast<std::int64_t>(observation.TotalLeftCounts());
        _rightEncoderTotalCounts += static_cast<std::int64_t>(observation.TotalRightCounts());
        return observation;
    }

    void RuntimeSensorSuite::CaptureEncoderCountsForCalibration(
        std::int32_t& leftCounts,
        std::int32_t& rightCounts) noexcept
    {
        const MazeMap::EncoderObs observation = CaptureEncoderObservation(0.0f);
        leftCounts = observation.TotalLeftCounts();
        rightCounts = observation.TotalRightCounts();
    }

    bool RuntimeSensorSuite::HaveEncoderCountsChangedForCalibration(
        const std::int32_t startLeftCounts,
        const std::int32_t startRightCounts) noexcept
    {
        std::int32_t leftCounts = 0;
        std::int32_t rightCounts = 0;
        CaptureEncoderCountsForCalibration(leftCounts, rightCounts);
        return (leftCounts != startLeftCounts) || (rightCounts != startRightCounts);
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
                (void)AppendTextLogLine("Encoder observation recovered; prediction input restored");
            }
            _encoderObservationDegraded = false;
            return;
        }

        if (!_encoderObservationDegraded)
        {
            (void)AppendTextLogFormatted(
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
        const std::uint32_t nowUs = micros();
        _vehicle.FrontLeftWallSensor().CommandLedOff(nowUs);
        _vehicle.FrontRightWallSensor().CommandLedOff(nowUs);
        _vehicle.SideLeftWallSensor().CommandLedOff(nowUs);
        _vehicle.SideRightWallSensor().CommandLedOff(nowUs);
    }

    void RuntimeSensorSuite::CaptureInertialSnapshot(SensorSnapshot& snapshot)
    {
        _vehicle.BackLeftImu().CaptureInertialSnapshot(snapshot);
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
            _wallCalibration.TryComputeFrontLeftSensorMeasuredThresholds(
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                leftAmbientLight,
                leftOnMeasuredThreshold,
                leftOffMeasuredThreshold,
                leftSignalBaseline);
        const bool haveRightThreshold =
            _wallCalibration.TryComputeFrontRightSensorMeasuredThresholds(
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
                const float leftSignalRise =
                    (std::isfinite(leftMeasuredDifferentialLight) &&
                        std::isfinite(leftSignalBaseline) &&
                        (leftMeasuredDifferentialLight > leftSignalBaseline)) ?
                    (leftMeasuredDifferentialLight - leftSignalBaseline) :
                    0.0f;
                _frontLeftWallSignalFiltered = leftSignalRise;
                _frontLeftWallSignalInitialized = true;
                _frontLeftWallState = _frontLeftWallState ?
                    (leftSignalRise >= leftOffMeasuredThreshold) :
                    (leftSignalRise >= leftOnMeasuredThreshold);
            }
            else
            {
                _frontLeftWallState = false;
                _frontLeftWallSignalInitialized = false;
            }

            if (haveRightThreshold)
            {
                const float rightSignalRise =
                    (std::isfinite(rightMeasuredDifferentialLight) &&
                        std::isfinite(rightSignalBaseline) &&
                        (rightMeasuredDifferentialLight > rightSignalBaseline)) ?
                    (rightMeasuredDifferentialLight - rightSignalBaseline) :
                    0.0f;
                _frontRightWallSignalFiltered = rightSignalRise;
                _frontRightWallSignalInitialized = true;
                _frontRightWallState = _frontRightWallState ?
                    (rightSignalRise >= rightOffMeasuredThreshold) :
                    (rightSignalRise >= rightOnMeasuredThreshold);
            }
            else
            {
                _frontRightWallState = false;
                _frontRightWallSignalInitialized = false;
            }

            return _frontLeftWallState || _frontRightWallState;
        }

        const bool currentFrontWallState = _frontLeftWallState || _frontRightWallState;
        const bool fallbackState = currentFrontWallState ?
            (fallbackDistanceM < Config::kFrontWallOffThresholdM) :
            (fallbackDistanceM < Config::kFrontWallOnThresholdM);
        _frontWallUsesFallbackDetection = true;
        _frontLeftWallState = fallbackState;
        _frontRightWallState = fallbackState;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        return fallbackState;
    }
}
