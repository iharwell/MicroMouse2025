#pragma once

#include "Defines.h"
#include "EncoderObs.h"
#include "SensorTelemetryTypes.h"
#include "WallObservationPipeline.h"

#include <cstdint>
#include <stdint.h>

class SensorSnapshot final
{
public:
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

    constexpr const MazeMap::EncoderObs& EncoderObservation() const noexcept { return _encoderObservation; }
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
    constexpr void SetEncoderObservation(const MazeMap::EncoderObs& observation) noexcept { _encoderObservation = observation; }
    constexpr void SetEncoderObservation(const MazeMap::EncoderObs& observation, bool valid) noexcept
    {
        _encoderObservation = observation;
        _encoderObservationValid = valid;
    }
    constexpr void SetFrontLeftTelemetry(const WallSensorTelemetry& telemetry) noexcept { _frontLeftTelemetry = telemetry; }
    constexpr void SetFrontRightTelemetry(const WallSensorTelemetry& telemetry) noexcept { _frontRightTelemetry = telemetry; }
    constexpr void SetSideLeftTelemetry(const WallSensorTelemetry& telemetry) noexcept { _sideLeftTelemetry = telemetry; }
    constexpr void SetSideRightTelemetry(const WallSensorTelemetry& telemetry) noexcept { _sideRightTelemetry = telemetry; }
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

private:
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
    MazeMap::EncoderObs _encoderObservation{};
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
