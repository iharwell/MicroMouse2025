#pragma once

#include <cstdint>
#include <limits>

// Solver-local evidence for the latest committed DriveBase command proposal.
// Raw sensor/estimator state stays with the runtime state and mode logs.
struct DriveTelemetry
{
    std::uint32_t proposalSequenceId = 0U;
    std::uint32_t plantEvaluationId = 0U;
    std::uint32_t feedbackTuningRevisionId = 0U;
    std::uint32_t feedbackStrategyRevisionId = 0U;

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
    std::uint16_t scalarIntentFlags = 0U;
    std::uint16_t feedbackBranchFlags = 0U;
    std::uint16_t telemetryValidFlags = 0U;
    std::uint16_t solverFailureFlags = 0U;

    static constexpr std::uint16_t kCommandKindStaleEvidence = 1U << 0;
    static constexpr std::uint16_t kCommandKindBodyProposal = 1U << 1;
    static constexpr std::uint16_t kCommandKindSolverFailureEvidence = 1U << 2;

    static constexpr std::uint16_t kScalarForwardVelocityInactive = 1U << 0;
    static constexpr std::uint16_t kScalarForwardVelocityFinite = 1U << 1;
    static constexpr std::uint16_t kScalarForwardVelocityMaximize = 1U << 2;
    static constexpr std::uint16_t kScalarYawRateInactive = 1U << 3;
    static constexpr std::uint16_t kScalarYawRateFinite = 1U << 4;
    static constexpr std::uint16_t kScalarYawRateMaximize = 1U << 5;
    static constexpr std::uint16_t kScalarForwardAccelInactive = 1U << 6;
    static constexpr std::uint16_t kScalarForwardAccelFinite = 1U << 7;
    static constexpr std::uint16_t kScalarForwardAccelMaximize = 1U << 8;
    static constexpr std::uint16_t kScalarYawAccelInactive = 1U << 9;
    static constexpr std::uint16_t kScalarYawAccelFinite = 1U << 10;
    static constexpr std::uint16_t kScalarYawAccelMaximize = 1U << 11;
    static constexpr std::uint16_t kScalarYawInactive = 1U << 12;
    static constexpr std::uint16_t kScalarYawFinite = 1U << 13;
    static constexpr std::uint16_t kScalarYawMaximizeUnsupported = 1U << 14;

    static constexpr std::uint16_t kFeedbackForwardVelocityInactive = 1U << 0;
    static constexpr std::uint16_t kFeedbackYawRateInactive = 1U << 1;
    static constexpr std::uint16_t kFeedbackHeadingInactive = 1U << 2;
    static constexpr std::uint16_t kFeedbackForwardSuppressedForMaximize = 1U << 3;
    static constexpr std::uint16_t kFeedbackYawSuppressedForMaximize = 1U << 4;

    static constexpr std::uint16_t kTelemetryProposalSequenceValid = 1U << 0;
    static constexpr std::uint16_t kTelemetryPlantEvaluationValid = 1U << 1;
    static constexpr std::uint16_t kTelemetryFeedbackTuningRevisionValid = 1U << 2;
    static constexpr std::uint16_t kTelemetryFeedbackStrategyRevisionValid = 1U << 3;
    static constexpr std::uint16_t kTelemetryCommandEvidenceValid = 1U << 4;
    static constexpr std::uint16_t kTelemetryPlantCommandValid = 1U << 5;

    static constexpr std::uint16_t kSolverFailurePlantNonFinite = 1U << 0;
    static constexpr std::uint16_t kSolverFailureUnsupportedScalarIntent = 1U << 1;
};
