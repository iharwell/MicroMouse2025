#pragma once

#include "Defines.h"
#include "SensorTelemetryTypes.h"
#include "WallObservationPipeline.h"

#include <cstdint>
#include <limits>
#include <stdint.h>

namespace MazeMap
{
    class RuntimeSensorSuite;
}

class SensorSnapshot final
{
public:
    // Exact wheel-encoder measurement for the current committed sensor snapshot.
    class EncoderObs final
    {
    public:
        constexpr EncoderObs(const EncoderObs&) noexcept = default;
        constexpr EncoderObs(EncoderObs&&) noexcept = default;
        constexpr EncoderObs& operator=(const EncoderObs&) noexcept = default;
        constexpr EncoderObs& operator=(EncoderObs&&) noexcept = default;

        constexpr std::int32_t TotalLeftCounts() const noexcept { return _totalLeftCounts; }
        constexpr std::int32_t TotalRightCounts() const noexcept { return _totalRightCounts; }
        constexpr float LeftDistanceDeltaM() const noexcept { return _leftDistanceDeltaM; }
        constexpr float RightDistanceDeltaM() const noexcept { return _rightDistanceDeltaM; }
        constexpr float LeftVelocityMps() const noexcept { return _leftVelocityMps; }
        constexpr float RightVelocityMps() const noexcept { return _rightVelocityMps; }
        constexpr float LeftWheelSpeedRadps() const noexcept { return _leftWheelSpeedRadps; }
        constexpr float RightWheelSpeedRadps() const noexcept { return _rightWheelSpeedRadps; }

        constexpr void SetTotalLeftCounts(std::int32_t counts) noexcept { _totalLeftCounts = counts; }
        constexpr void SetTotalRightCounts(std::int32_t counts) noexcept { _totalRightCounts = counts; }
        constexpr void SetLeftDistanceDeltaM(float distanceM) noexcept { _leftDistanceDeltaM = distanceM; }
        constexpr void SetRightDistanceDeltaM(float distanceM) noexcept { _rightDistanceDeltaM = distanceM; }
        constexpr void SetLeftVelocityMps(float velocityMps) noexcept { _leftVelocityMps = velocityMps; }
        constexpr void SetRightVelocityMps(float velocityMps) noexcept { _rightVelocityMps = velocityMps; }
        constexpr void SetLeftWheelSpeedRadps(float wheelSpeedRadps) noexcept { _leftWheelSpeedRadps = wheelSpeedRadps; }
        constexpr void SetRightWheelSpeedRadps(float wheelSpeedRadps) noexcept { _rightWheelSpeedRadps = wheelSpeedRadps; }

        constexpr void SetCounts(std::int32_t leftCounts, std::int32_t rightCounts) noexcept
        {
            _totalLeftCounts = leftCounts;
            _totalRightCounts = rightCounts;
        }

        constexpr void SetDistanceDeltasM(float leftDistanceM, float rightDistanceM) noexcept
        {
            _leftDistanceDeltaM = leftDistanceM;
            _rightDistanceDeltaM = rightDistanceM;
        }

        constexpr void SetWheelLinearVelocityMps(float leftVelocityMps, float rightVelocityMps) noexcept
        {
            _leftVelocityMps = leftVelocityMps;
            _rightVelocityMps = rightVelocityMps;
        }

        constexpr void SetWheelSpeedRadps(float leftWheelSpeedRadps, float rightWheelSpeedRadps) noexcept
        {
            _leftWheelSpeedRadps = leftWheelSpeedRadps;
            _rightWheelSpeedRadps = rightWheelSpeedRadps;
        }

    private:
        friend class SensorSnapshot;

        constexpr EncoderObs() noexcept = default;

        constexpr EncoderObs(
            std::int32_t totalLeftCounts,
            std::int32_t totalRightCounts,
            float leftDistanceDeltaM,
            float rightDistanceDeltaM,
            float leftVelocityMps,
            float rightVelocityMps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps) noexcept
            : _totalLeftCounts(totalLeftCounts)
            , _totalRightCounts(totalRightCounts)
            , _leftDistanceDeltaM(leftDistanceDeltaM)
            , _rightDistanceDeltaM(rightDistanceDeltaM)
            , _leftVelocityMps(leftVelocityMps)
            , _rightVelocityMps(rightVelocityMps)
            , _leftWheelSpeedRadps(leftWheelSpeedRadps)
            , _rightWheelSpeedRadps(rightWheelSpeedRadps)
        {
        }

        std::int32_t _totalLeftCounts = 0;
        std::int32_t _totalRightCounts = 0;
        float _leftDistanceDeltaM = 0.0f;
        float _rightDistanceDeltaM = 0.0f;
        float _leftVelocityMps = 0.0f;
        float _rightVelocityMps = 0.0f;
        float _leftWheelSpeedRadps = 0.0f;
        float _rightWheelSpeedRadps = 0.0f;
    };

    constexpr SensorSnapshot() noexcept = default;

    EXPORT bool BuildEvidenceObservationSnapshot(
        const SensorSnapshot* samples,
        uint8_t sampleCount) noexcept;

    constexpr float FrontLeftDistanceM() const noexcept { return _frontLeftDistanceM; }
    constexpr float FrontRightDistanceM() const noexcept { return _frontRightDistanceM; }
    constexpr float FrontLeftDifferentialLight() const noexcept { return _frontLeftDifferentialLight; }
    constexpr float FrontRightDifferentialLight() const noexcept { return _frontRightDifferentialLight; }
    constexpr float SideLeftDistanceM() const noexcept { return _sideLeftDistanceM; }
    constexpr float SideRightDistanceM() const noexcept { return _sideRightDistanceM; }
    constexpr float SideLeftDifferentialLight() const noexcept { return _sideLeftDifferentialLight; }
    constexpr float SideRightDifferentialLight() const noexcept { return _sideRightDifferentialLight; }
    constexpr float CorridorErrorM() const noexcept { return _corridorErrorM; }
    constexpr float FrontSkewM() const noexcept { return _frontSkewM; }
    constexpr float BodyRightAccelerationMps2() const noexcept { return _bodyRightAccelerationMps2; }
    constexpr float BodyForwardAccelerationMps2() const noexcept { return _bodyForwardAccelerationMps2; }
    constexpr float PlanarAccelerationMps2() const noexcept { return _planarAccelerationMps2; }
    constexpr float RawYawRateRadps() const noexcept { return _rawYawRateRadps; }
    constexpr float YawRateBiasRadps() const noexcept { return _yawRateBiasRadps; }
    constexpr float YawRateRadps() const noexcept { return _yawRateRadps; }
    constexpr bool AccelerationBiasValid() const noexcept { return _accelerationBiasValid; }
    constexpr bool HasFrontWall() const noexcept { return _frontWall; }
    constexpr bool HasFrontLeftWall() const noexcept { return _frontLeftWall; }
    constexpr bool HasFrontRightWall() const noexcept { return _frontRightWall; }
    constexpr bool FrontWallObservationValid() const noexcept { return _frontWallObservationValid; }
    constexpr bool FrontWallUsesFallbackDetection() const noexcept { return _frontWallUsesFallbackDetection; }
    constexpr bool FrontWallUsesCharacterizationDetection() const noexcept { return _frontWallUsesCharacterizationDetection; }
    constexpr bool HasLeftWall() const noexcept { return _leftWall; }
    constexpr bool HasRightWall() const noexcept { return _rightWall; }
    constexpr bool LeftDistanceValidForControl() const noexcept { return _leftDistanceValidForControl; }
    constexpr bool RightDistanceValidForControl() const noexcept { return _rightDistanceValidForControl; }
    constexpr bool HasLeftWallObservation() const noexcept { return _leftWallObservation; }
    constexpr bool HasRightWallObservation() const noexcept { return _rightWallObservation; }
    constexpr bool LeftWallObservationWindowValid() const noexcept { return _leftWallObservationWindowValid; }
    constexpr bool RightWallObservationWindowValid() const noexcept { return _rightWallObservationWindowValid; }
    constexpr bool LeftTransitionDetected() const noexcept { return _leftTransitionDetected; }
    constexpr bool RightTransitionDetected() const noexcept { return _rightTransitionDetected; }
    constexpr bool EncoderObservationValid() const noexcept { return _encoderObservationValid; }
    constexpr std::int64_t LeftEncoderTotalCounts() const noexcept { return _leftEncoderTotalCounts; }
    constexpr std::int64_t RightEncoderTotalCounts() const noexcept { return _rightEncoderTotalCounts; }
    constexpr float LeftEncoderDistanceM() const noexcept { return _leftEncoderDistanceM; }
    constexpr float RightEncoderDistanceM() const noexcept { return _rightEncoderDistanceM; }
    constexpr float AverageEncoderDistanceM() const noexcept
    {
        return 0.5f * (_leftEncoderDistanceM + _rightEncoderDistanceM);
    }

    constexpr const SensorSnapshot::EncoderObs& EncoderObservation() const noexcept { return _encoderObservation; }
    constexpr void PublishEncoderObservation(
        const SensorSnapshot::EncoderObs& observation,
        bool valid,
        std::int64_t leftTotalCounts,
        std::int64_t rightTotalCounts,
        float leftDistanceM,
        float rightDistanceM) noexcept
    {
        _encoderObservation = observation;
        _encoderObservationValid = valid;
        _leftEncoderTotalCounts = leftTotalCounts;
        _rightEncoderTotalCounts = rightTotalCounts;
        _leftEncoderDistanceM = leftDistanceM;
        _rightEncoderDistanceM = rightDistanceM;
    }
    constexpr const WallSensorTelemetry& FrontLeftTelemetry() const noexcept { return _frontLeftTelemetry; }
    constexpr const WallSensorTelemetry& FrontRightTelemetry() const noexcept { return _frontRightTelemetry; }
    constexpr const WallSensorTelemetry& SideLeftTelemetry() const noexcept { return _sideLeftTelemetry; }
    constexpr const WallSensorTelemetry& SideRightTelemetry() const noexcept { return _sideRightTelemetry; }
    const MazeMap::WallObs& FrontLeftWallSensorObservation() const noexcept { return _frontLeftWallSensorObservation; }
    const MazeMap::WallObs& FrontRightWallSensorObservation() const noexcept { return _frontRightWallSensorObservation; }
    const MazeMap::WallObs& SideLeftWallSensorObservation() const noexcept { return _sideLeftWallSensorObservation; }
    const MazeMap::WallObs& SideRightWallSensorObservation() const noexcept { return _sideRightWallSensorObservation; }
    constexpr const OpticalObservationTiming& FrontTiming() const noexcept { return _frontTiming; }
    constexpr const OpticalObservationTiming& LeftTiming() const noexcept { return _leftTiming; }
    constexpr const OpticalObservationTiming& RightTiming() const noexcept { return _rightTiming; }
    constexpr const ImuTelemetry& FrontRightImuTelemetry() const noexcept { return _frontRightImuTelemetry; }
    constexpr const ImuTelemetry& BackLeftImuTelemetry() const noexcept { return _backLeftImuTelemetry; }
    constexpr const ImuObservationTiming& ImuTiming() const noexcept { return _imuTiming; }

    constexpr void SetFrontLeftDistanceM(float distanceM) noexcept { _frontLeftDistanceM = distanceM; }
    constexpr void SetFrontRightDistanceM(float distanceM) noexcept { _frontRightDistanceM = distanceM; }
    constexpr void SetFrontLeftDifferentialLight(float differentialLight) noexcept { _frontLeftDifferentialLight = differentialLight; }
    constexpr void SetFrontRightDifferentialLight(float differentialLight) noexcept { _frontRightDifferentialLight = differentialLight; }
    constexpr void SetSideLeftDistanceM(float distanceM) noexcept { _sideLeftDistanceM = distanceM; }
    constexpr void SetSideRightDistanceM(float distanceM) noexcept { _sideRightDistanceM = distanceM; }
    constexpr void SetSideLeftDifferentialLight(float differentialLight) noexcept { _sideLeftDifferentialLight = differentialLight; }
    constexpr void SetSideRightDifferentialLight(float differentialLight) noexcept { _sideRightDifferentialLight = differentialLight; }
    constexpr void SetCorridorErrorM(float corridorErrorM) noexcept { _corridorErrorM = corridorErrorM; }
    constexpr void SetFrontSkewM(float frontSkewM) noexcept { _frontSkewM = frontSkewM; }
    constexpr void SetBodyRightAccelerationMps2(float accelerationMps2) noexcept { _bodyRightAccelerationMps2 = accelerationMps2; }
    constexpr void SetBodyForwardAccelerationMps2(float accelerationMps2) noexcept { _bodyForwardAccelerationMps2 = accelerationMps2; }
    constexpr void SetPlanarAccelerationMps2(float accelerationMps2) noexcept { _planarAccelerationMps2 = accelerationMps2; }
    constexpr void SetRawYawRateRadps(float yawRateRadps) noexcept { _rawYawRateRadps = yawRateRadps; }
    constexpr void SetYawRateBiasRadps(float yawRateBiasRadps) noexcept { _yawRateBiasRadps = yawRateBiasRadps; }
    constexpr void SetYawRateRadps(float yawRateRadps) noexcept { _yawRateRadps = yawRateRadps; }
    constexpr void SetAccelerationBiasValid(bool valid) noexcept { _accelerationBiasValid = valid; }
    constexpr void SetFrontWall(bool present) noexcept { _frontWall = present; }
    constexpr void SetFrontLeftWall(bool present) noexcept { _frontLeftWall = present; }
    constexpr void SetFrontRightWall(bool present) noexcept { _frontRightWall = present; }
    constexpr void SetFrontWallObservationValid(bool valid) noexcept { _frontWallObservationValid = valid; }
    constexpr void SetFrontWallUsesFallbackDetection(bool usesFallback) noexcept { _frontWallUsesFallbackDetection = usesFallback; }
    constexpr void SetFrontWallUsesCharacterizationDetection(bool usesCharacterization) noexcept { _frontWallUsesCharacterizationDetection = usesCharacterization; }
    constexpr void SetLeftWall(bool present) noexcept { _leftWall = present; }
    constexpr void SetRightWall(bool present) noexcept { _rightWall = present; }
    constexpr void SetLeftDistanceValidForControl(bool valid) noexcept { _leftDistanceValidForControl = valid; }
    constexpr void SetRightDistanceValidForControl(bool valid) noexcept { _rightDistanceValidForControl = valid; }
    constexpr void SetLeftWallObservation(bool present) noexcept { _leftWallObservation = present; }
    constexpr void SetRightWallObservation(bool present) noexcept { _rightWallObservation = present; }
    constexpr void SetLeftWallObservationWindowValid(bool valid) noexcept { _leftWallObservationWindowValid = valid; }
    constexpr void SetRightWallObservationWindowValid(bool valid) noexcept { _rightWallObservationWindowValid = valid; }
    constexpr void SetLeftTransitionDetected(bool detected) noexcept { _leftTransitionDetected = detected; }
    constexpr void SetRightTransitionDetected(bool detected) noexcept { _rightTransitionDetected = detected; }
    constexpr void SetFrontLeftTelemetry(const WallSensorTelemetry& telemetry) noexcept { _frontLeftTelemetry = telemetry; }
    constexpr void SetFrontRightTelemetry(const WallSensorTelemetry& telemetry) noexcept { _frontRightTelemetry = telemetry; }
    constexpr void SetSideLeftTelemetry(const WallSensorTelemetry& telemetry) noexcept { _sideLeftTelemetry = telemetry; }
    constexpr void SetSideRightTelemetry(const WallSensorTelemetry& telemetry) noexcept { _sideRightTelemetry = telemetry; }
    constexpr void SetFrontLeftTelemetryValues(
        float ambientLight,
        float litLight,
        float differentialLight,
        float rawDistanceM,
        float distanceM,
        bool wall) noexcept
    {
        _frontLeftTelemetry.ambientLight = ambientLight;
        _frontLeftTelemetry.litLight = litLight;
        _frontLeftTelemetry.differentialLight = differentialLight;
        _frontLeftTelemetry.rawDistanceM = rawDistanceM;
        _frontLeftTelemetry.distanceM = distanceM;
        _frontLeftTelemetry.wall = wall;
    }
    constexpr void SetFrontRightTelemetryValues(
        float ambientLight,
        float litLight,
        float differentialLight,
        float rawDistanceM,
        float distanceM,
        bool wall) noexcept
    {
        _frontRightTelemetry.ambientLight = ambientLight;
        _frontRightTelemetry.litLight = litLight;
        _frontRightTelemetry.differentialLight = differentialLight;
        _frontRightTelemetry.rawDistanceM = rawDistanceM;
        _frontRightTelemetry.distanceM = distanceM;
        _frontRightTelemetry.wall = wall;
    }
    constexpr void SetSideLeftTelemetryValues(
        float ambientLight,
        float litLight,
        float differentialLight,
        float rawDistanceM,
        float distanceM,
        bool wall) noexcept
    {
        _sideLeftTelemetry.ambientLight = ambientLight;
        _sideLeftTelemetry.litLight = litLight;
        _sideLeftTelemetry.differentialLight = differentialLight;
        _sideLeftTelemetry.rawDistanceM = rawDistanceM;
        _sideLeftTelemetry.distanceM = distanceM;
        _sideLeftTelemetry.wall = wall;
    }
    constexpr void SetSideRightTelemetryValues(
        float ambientLight,
        float litLight,
        float differentialLight,
        float rawDistanceM,
        float distanceM,
        bool wall) noexcept
    {
        _sideRightTelemetry.ambientLight = ambientLight;
        _sideRightTelemetry.litLight = litLight;
        _sideRightTelemetry.differentialLight = differentialLight;
        _sideRightTelemetry.rawDistanceM = rawDistanceM;
        _sideRightTelemetry.distanceM = distanceM;
        _sideRightTelemetry.wall = wall;
    }
    void SetFrontLeftWallSensorObservation(const MazeMap::WallObs& observation) noexcept { _frontLeftWallSensorObservation = observation; }
    void SetFrontRightWallSensorObservation(const MazeMap::WallObs& observation) noexcept { _frontRightWallSensorObservation = observation; }
    void SetSideLeftWallSensorObservation(const MazeMap::WallObs& observation) noexcept { _sideLeftWallSensorObservation = observation; }
    void SetSideRightWallSensorObservation(const MazeMap::WallObs& observation) noexcept { _sideRightWallSensorObservation = observation; }
    constexpr void SetFrontTiming(const OpticalObservationTiming& timing) noexcept { _frontTiming = timing; }
    constexpr void SetLeftTiming(const OpticalObservationTiming& timing) noexcept { _leftTiming = timing; }
    constexpr void SetRightTiming(const OpticalObservationTiming& timing) noexcept { _rightTiming = timing; }
    constexpr void SetFrontRightImuTelemetry(const ImuTelemetry& telemetry) noexcept { _frontRightImuTelemetry = telemetry; }
    constexpr void SetBackLeftImuTelemetry(const ImuTelemetry& telemetry) noexcept { _backLeftImuTelemetry = telemetry; }
    constexpr void SetImuTiming(const ImuObservationTiming& timing) noexcept { _imuTiming = timing; }

    void ClearUnavailableObservations(
        bool inertialObserved,
        bool frontWallObserved,
        bool leftWallObserved,
        bool rightWallObserved) noexcept
    {
        if (!inertialObserved)
        {
            const float noInertialObservation = std::numeric_limits<float>::quiet_NaN();
            _bodyRightAccelerationMps2 = 0.0f;
            _bodyForwardAccelerationMps2 = 0.0f;
            _planarAccelerationMps2 = 0.0f;
            _rawYawRateRadps = noInertialObservation;
            _yawRateBiasRadps = noInertialObservation;
            _yawRateRadps = noInertialObservation;
            _accelerationBiasValid = false;
            _frontRightImuTelemetry = {};
            _backLeftImuTelemetry = {};
            _imuTiming = {};
        }

        if (!frontWallObserved)
        {
            _frontLeftDistanceM = 0.0f;
            _frontRightDistanceM = 0.0f;
            _frontLeftDifferentialLight = 0.0f;
            _frontRightDifferentialLight = 0.0f;
            _frontSkewM = 0.0f;
            _frontWall = false;
            _frontLeftWall = false;
            _frontRightWall = false;
            _frontWallObservationValid = false;
            _frontWallUsesFallbackDetection = false;
            _frontWallUsesCharacterizationDetection = false;
            _frontLeftTelemetry = {};
            _frontRightTelemetry = {};
            _frontLeftWallSensorObservation = {};
            _frontRightWallSensorObservation = {};
            _frontTiming = {};
        }

        if (!leftWallObserved)
        {
            _sideLeftDistanceM = 0.0f;
            _sideLeftDifferentialLight = 0.0f;
            _leftWall = false;
            _leftDistanceValidForControl = false;
            _leftWallObservation = false;
            _leftWallObservationWindowValid = false;
            _leftTransitionDetected = false;
            _sideLeftTelemetry = {};
            _sideLeftWallSensorObservation = {};
            _leftTiming = {};
        }

        if (!rightWallObserved)
        {
            _sideRightDistanceM = 0.0f;
            _sideRightDifferentialLight = 0.0f;
            _rightWall = false;
            _rightDistanceValidForControl = false;
            _rightWallObservation = false;
            _rightWallObservationWindowValid = false;
            _rightTransitionDetected = false;
            _sideRightTelemetry = {};
            _sideRightWallSensorObservation = {};
            _rightTiming = {};
        }
    }

    constexpr void RecomputeCorridorErrorM(float expectedSideWallDistanceM) noexcept
    {
        if (_leftDistanceValidForControl && _rightDistanceValidForControl)
        {
            _corridorErrorM = 0.5f * (_sideLeftDistanceM - _sideRightDistanceM);
            return;
        }
        if (_leftDistanceValidForControl)
        {
            _corridorErrorM = _sideLeftDistanceM - expectedSideWallDistanceM;
            return;
        }
        if (_rightDistanceValidForControl)
        {
            _corridorErrorM = expectedSideWallDistanceM - _sideRightDistanceM;
            return;
        }

        _corridorErrorM = 0.0f;
    }

private:
    friend class MazeMap::RuntimeSensorSuite;

    constexpr SensorSnapshot::EncoderObs& MutableEncoderObservationForCapture() noexcept { return _encoderObservation; }
    constexpr void SetEncoderObservationValid(bool valid) noexcept { _encoderObservationValid = valid; }
    constexpr void SetLeftEncoderTotalCounts(std::int64_t counts) noexcept { _leftEncoderTotalCounts = counts; }
    constexpr void SetRightEncoderTotalCounts(std::int64_t counts) noexcept { _rightEncoderTotalCounts = counts; }
    constexpr void SetLeftEncoderDistanceM(float distanceM) noexcept { _leftEncoderDistanceM = distanceM; }
    constexpr void SetRightEncoderDistanceM(float distanceM) noexcept { _rightEncoderDistanceM = distanceM; }
    constexpr void SetEncoderTotals(std::int64_t leftCounts, std::int64_t rightCounts) noexcept
    {
        _leftEncoderTotalCounts = leftCounts;
        _rightEncoderTotalCounts = rightCounts;
    }
    constexpr void SetEncoderDistancesM(float leftDistanceM, float rightDistanceM) noexcept
    {
        _leftEncoderDistanceM = leftDistanceM;
        _rightEncoderDistanceM = rightDistanceM;
    }
    constexpr void SetEncoderObservation(const SensorSnapshot::EncoderObs& observation) noexcept { _encoderObservation = observation; }
    constexpr void SetEncoderObservation(const SensorSnapshot::EncoderObs& observation, bool valid) noexcept
    {
        _encoderObservation = observation;
        _encoderObservationValid = valid;
    }

    float _frontLeftDistanceM = 0.0f;
    float _frontRightDistanceM = 0.0f;
    float _frontLeftDifferentialLight = 0.0f;
    float _frontRightDifferentialLight = 0.0f;
    float _sideLeftDistanceM = 0.0f;
    float _sideRightDistanceM = 0.0f;
    float _sideLeftDifferentialLight = 0.0f;
    float _sideRightDifferentialLight = 0.0f;
    float _corridorErrorM = 0.0f;
    float _frontSkewM = 0.0f;
    float _bodyRightAccelerationMps2 = 0.0f;
    float _bodyForwardAccelerationMps2 = 0.0f;
    float _planarAccelerationMps2 = 0.0f;
    float _rawYawRateRadps = 0.0f;
    float _yawRateBiasRadps = 0.0f;
    float _yawRateRadps = 0.0f;
    bool _accelerationBiasValid = false;
    bool _frontWall = false;
    bool _frontLeftWall = false;
    bool _frontRightWall = false;
    bool _frontWallObservationValid = false;
    bool _frontWallUsesFallbackDetection = false;
    bool _frontWallUsesCharacterizationDetection = false;
    bool _leftWall = false;
    bool _rightWall = false;
    bool _leftDistanceValidForControl = false;
    bool _rightDistanceValidForControl = false;
    bool _leftWallObservation = false;
    bool _rightWallObservation = false;
    bool _leftWallObservationWindowValid = false;
    bool _rightWallObservationWindowValid = false;
    bool _leftTransitionDetected = false;
    bool _rightTransitionDetected = false;
    bool _encoderObservationValid = false;
    std::int64_t _leftEncoderTotalCounts = 0;
    std::int64_t _rightEncoderTotalCounts = 0;
    float _leftEncoderDistanceM = 0.0f;
    float _rightEncoderDistanceM = 0.0f;
    SensorSnapshot::EncoderObs _encoderObservation{};
    WallSensorTelemetry _frontLeftTelemetry{};
    WallSensorTelemetry _frontRightTelemetry{};
    WallSensorTelemetry _sideLeftTelemetry{};
    WallSensorTelemetry _sideRightTelemetry{};
    MazeMap::WallObs _frontLeftWallSensorObservation{};
    MazeMap::WallObs _frontRightWallSensorObservation{};
    MazeMap::WallObs _sideLeftWallSensorObservation{};
    MazeMap::WallObs _sideRightWallSensorObservation{};
    OpticalObservationTiming _frontTiming{};
    OpticalObservationTiming _leftTiming{};
    OpticalObservationTiming _rightTiming{};
    ImuTelemetry _frontRightImuTelemetry{};
    ImuTelemetry _backLeftImuTelemetry{};
    ImuObservationTiming _imuTiming{};
};
