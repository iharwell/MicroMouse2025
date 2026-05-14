#pragma once

#include <cstdint>
#include <stdint.h>

// Authoritative per-cycle drive report emitted by DriveBase. Estimator/UKF diagnostics are
// intentionally not part of this drive-owned contract.
struct DriveTelemetry
{
    float leftDriveCommand = 0.0f;
    float rightDriveCommand = 0.0f;
    float commandedLinearSpeedMps = 0.0f;
    float commandedAngularSpeedRadps = 0.0f;
    float leftFeedforwardCommand = 0.0f;
    float rightFeedforwardCommand = 0.0f;
    float leftFeedbackCommand = 0.0f;
    float rightFeedbackCommand = 0.0f;
    float leftTargetVelocityMps = 0.0f;
    float rightTargetVelocityMps = 0.0f;
    std::int64_t leftEncoderCount = 0;
    std::int64_t rightEncoderCount = 0;
    float leftDistanceM = 0.0f;
    float rightDistanceM = 0.0f;
    float leftVelocityMps = 0.0f;
    float rightVelocityMps = 0.0f;
    float leftEncoderOmegaRadps = 0.0f;
    float rightEncoderOmegaRadps = 0.0f;
    uint16_t modeFlags = 0U;
    uint16_t saturationFlags = 0U;
    bool encoderObservationValid = false;
};
