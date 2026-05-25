#pragma once

#include <cstdint>
#include <limits>

// Solver-local evidence for the latest committed DriveBase command proposal.
// Raw sensor/estimator state stays with the runtime state and mode logs.
struct DriveTelemetry
{
    std::uint32_t proposalSequenceId = 0U;

    float requestedForwardMps = (std::numeric_limits<float>::quiet_NaN)();
    float requestedYawRateRadps = (std::numeric_limits<float>::quiet_NaN)();
    float requestedForwardAccelMps2 = (std::numeric_limits<float>::quiet_NaN)();
    float requestedYawAccelRadps2 = (std::numeric_limits<float>::quiet_NaN)();
    float requestedYawRad = (std::numeric_limits<float>::quiet_NaN)();

    float composedForwardAccelMps2 = (std::numeric_limits<float>::quiet_NaN)();
    float composedYawAccelRadps2 = (std::numeric_limits<float>::quiet_NaN)();

    float leftPlantCommand = 0.0f;
    float rightPlantCommand = 0.0f;
    float leftDriveCommand = 0.0f;
    float rightDriveCommand = 0.0f;

    std::uint16_t commandKindFlags = 0U;
    std::uint16_t telemetryValidFlags = 0U;

    static constexpr std::uint16_t kCommandKindStaleEvidence = 1U << 0;
    static constexpr std::uint16_t kCommandKindBodyProposal = 1U << 1;
    static constexpr std::uint16_t kCommandKindSolverFailureEvidence = 1U << 2;

    static constexpr std::uint16_t kTelemetryProposalSequenceValid = 1U << 0;
    static constexpr std::uint16_t kTelemetryCommandEvidenceValid = 1U << 4;
    static constexpr std::uint16_t kTelemetryPlantCommandValid = 1U << 5;
};
