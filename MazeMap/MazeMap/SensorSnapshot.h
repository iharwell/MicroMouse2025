#pragma once

#include "SensorTelemetryTypes.h"

#include <cstdint>
#include <stdint.h>

struct SensorSnapshot
{
    float frontLeftDistanceM = 0.0f;
    float frontRightDistanceM = 0.0f;
    float frontLeftDifferentialLight = 0.0f;
    float frontRightDifferentialLight = 0.0f;
    float sideLeftDistanceM = 0.0f;
    float sideRightDistanceM = 0.0f;
    float sideLeftDifferentialLight = 0.0f;
    float sideRightDifferentialLight = 0.0f;
    float corridorErrorM = 0.0f;
    float frontSkewM = 0.0f;
    float accelBodyXMps2 = 0.0f;
    float accelBodyYMps2 = 0.0f;
    float planarAccelMps2 = 0.0f;
    float gyroRawRadps = 0.0f;
    float gyroBiasRadps = 0.0f;
    float gyroRadps = 0.0f;
    bool accelBiasValid = false;
    bool frontWall = false;
    bool frontLeftWall = false;
    bool frontRightWall = false;
    bool frontWallObservationValid = false;
    bool frontWallUsesFallbackDetection = false;
    bool frontWallUsesCharacterizationDetection = false;
    bool leftWall = false;
    bool rightWall = false;
    bool leftDistanceValidForControl = false;
    bool rightDistanceValidForControl = false;
    bool leftWallObservation = false;
    bool rightWallObservation = false;
    bool leftWallObservationWindowValid = false;
    bool rightWallObservationWindowValid = false;
    bool leftTransitionDetected = false;
    bool rightTransitionDetected = false;

    WallSensorTelemetry frontLeft{};
    WallSensorTelemetry frontRight{};
    WallSensorTelemetry sideLeft{};
    WallSensorTelemetry sideRight{};
    OpticalObservationTiming frontTiming{};
    OpticalObservationTiming leftTiming{};
    OpticalObservationTiming rightTiming{};
    ImuTelemetry imuFrontRight{};
    ImuTelemetry imuBackLeft{};
    ImuObservationTiming imuTiming{};
};

struct RollingObservationVoteSummary
{
    uint8_t sampleCount = 0U;
    uint8_t frontWallVotes = 0U;
    uint8_t frontLeftWallVotes = 0U;
    uint8_t frontRightWallVotes = 0U;
    uint8_t frontFallbackVotes = 0U;
    uint8_t leftWallVotes = 0U;
    uint8_t rightWallVotes = 0U;
    uint8_t leftWindowValidVotes = 0U;
    uint8_t rightWindowValidVotes = 0U;
};

void ClearFrontWallObservationDecision(SensorSnapshot& snapshot);

bool BuildEvidenceObservationSnapshot(
    const SensorSnapshot* samples,
    uint8_t sampleCount,
    SensorSnapshot& combinedSnapshot,
    RollingObservationVoteSummary& voteSummary);
