#include "pch.h"
#include "DriveBase.h"

#include <cmath>
#include <limits>

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;
}

namespace MazeMap
{
    DriveBase::DriveBase(
        const MazeMap::PlantModel& plant,
        const MazeMap::VehicleState& runtimeState,
        const MazeMap::PDCluster& feedbackTuning) noexcept
        : _plant(plant)
        , _runtimeState(runtimeState)
        , _feedbackTuning(feedbackTuning)
    {
        ClearCommandEvidence();
    }

    void DriveBase::ClearCommandEvidence() noexcept
    {
        _lastTelemetry = DriveTelemetry{};
        _lastTelemetry.commandKindFlags = DriveTelemetry::kCommandKindStaleEvidence;
    }

    CommandVector DriveBase::ProposeBodyTick(
        float targetForwardMps,
        float targetYawRateRadps,
        float targetForwardAccelMps2,
        float targetYawAccelRadps2,
        float targetYawRad) noexcept
    {
        DriveTelemetry telemetry =
            BuildBaseTelemetry(
                DriveTelemetry::kCommandKindBodyProposal,
                targetForwardMps,
                targetYawRateRadps,
                targetForwardAccelMps2,
                targetYawAccelRadps2,
                targetYawRad);

        const float observedForwardMps = _runtimeState.GetForwardVelocity();
        const float observedYawRateRadps = _runtimeState.GetYawRate();
        const float observedYawRad = _runtimeState.GetHeading();

        bool hasForwardFeedbackAccel = false;
        float forwardFeedbackAccelMps2 = 0.0f;
        if (std::isfinite(targetForwardMps) && std::isfinite(observedForwardMps))
        {
            forwardFeedbackAccelMps2 +=
                ComputeForwardVelocityFeedbackAccelMps2(
                    _feedbackTuning,
                    targetForwardMps - observedForwardMps);
            hasForwardFeedbackAccel = true;
        }

        bool hasYawFeedbackAccel = false;
        float yawFeedbackAccelRadps2 = 0.0f;
        if (std::isfinite(targetYawRateRadps) && std::isfinite(observedYawRateRadps))
        {
            yawFeedbackAccelRadps2 +=
                ComputeYawRateFeedbackAccelRadps2(
                    _feedbackTuning,
                    targetYawRateRadps - observedYawRateRadps);
            hasYawFeedbackAccel = true;
        }

        if (std::isfinite(targetYawRad) && std::isfinite(observedYawRad))
        {
            const float headingTargetYawRateRadps =
                std::isfinite(targetYawRateRadps) ? targetYawRateRadps : 0.0f;
            const float headingErrorRateRadps =
                std::isfinite(observedYawRateRadps) ?
                (headingTargetYawRateRadps - observedYawRateRadps) :
                0.0f;
            yawFeedbackAccelRadps2 +=
                ComputeHeadingFeedbackAccelRadps2(
                    _feedbackTuning,
                    AngleDifference(observedYawRad, targetYawRad),
                    headingErrorRateRadps);
            hasYawFeedbackAccel = true;
        }

        telemetry.composedForwardAccelMps2 =
            ComposeAccelerationObjective(
                targetForwardAccelMps2,
                forwardFeedbackAccelMps2,
                hasForwardFeedbackAccel);
        telemetry.composedYawAccelRadps2 =
            ComposeAccelerationObjective(
                targetYawAccelRadps2,
                yawFeedbackAccelRadps2,
                hasYawFeedbackAccel);

        const CommandVector plantCommand =
            _plant.ComputeFeedforward(
                telemetry.composedForwardAccelMps2,
                telemetry.composedYawAccelRadps2);

        telemetry.leftPlantCommand = plantCommand.LeftCommand();
        telemetry.rightPlantCommand = plantCommand.RightCommand();

        CommandVector finalCommand = plantCommand;
        finalCommand.ClampCommand();
        if (!plantCommand.IsFinite())
        {
            finalCommand = CommandVector(0.0f, 0.0f);
            telemetry.commandKindFlags |= DriveTelemetry::kCommandKindSolverFailureEvidence;
            telemetry.telemetryValidFlags &= ~DriveTelemetry::kTelemetryPlantCommandValid;
        }

        if (std::isinf(targetYawRad))
        {
            finalCommand = CommandVector(0.0f, 0.0f);
            telemetry.commandKindFlags |= DriveTelemetry::kCommandKindSolverFailureEvidence;
        }

        telemetry.leftDriveCommand = finalCommand.LeftCommand();
        telemetry.rightDriveCommand = finalCommand.RightCommand();
        telemetry.telemetryValidFlags |= DriveTelemetry::kTelemetryCommandEvidenceValid;
        _lastTelemetry = telemetry;
        return finalCommand;
    }

    const DriveTelemetry& DriveBase::LastTelemetry() const noexcept
    {
        return _lastTelemetry;
    }

    float DriveBase::ComposeAccelerationObjective(
        float requestedAccel,
        float feedbackAccel,
        bool hasFeedback) noexcept
    {
        if (std::isinf(requestedAccel))
        {
            return requestedAccel;
        }

        const bool hasRequestedAccel = std::isfinite(requestedAccel);
        if (hasRequestedAccel)
        {
            return requestedAccel + (hasFeedback ? feedbackAccel : 0.0f);
        }
        return hasFeedback ? feedbackAccel : (std::numeric_limits<float>::quiet_NaN)();
    }

    float DriveBase::ComputeForwardVelocityFeedbackAccelMps2(
        const MazeMap::PDCluster& feedbackTuning,
        float forwardVelocityErrorMps) noexcept
    {
        // PDCluster is the canonical tuning owner. In this DriveBase path the state-velocity
        // proportional gain is interpreted as (m/s^2)/(m/s); derivative history is not sampled.
        return feedbackTuning.VelocityStatePD.Compute(forwardVelocityErrorMps, 0.0f);
    }

    float DriveBase::ComputeYawRateFeedbackAccelRadps2(
        const MazeMap::PDCluster& feedbackTuning,
        float yawRateErrorRadps) noexcept
    {
        // The yaw-rate proportional gain is interpreted as (rad/s^2)/(rad/s) for same-tick
        // acceleration-domain correction before PlantModel inverse dynamics.
        return feedbackTuning.YawRateStatePD.Compute(yawRateErrorRadps, 0.0f);
    }

    float DriveBase::ComputeHeadingFeedbackAccelRadps2(
        const MazeMap::PDCluster& feedbackTuning,
        float headingErrorRad,
        float headingErrorRateRadps) noexcept
    {
        // Heading PD contributes acceleration-domain correction directly before PlantModel
        // inverse dynamics. The derivative term is the requested minus observed yaw rate.
        return feedbackTuning.HeadingStatePD.Compute(headingErrorRad, headingErrorRateRadps);
    }

    DriveTelemetry DriveBase::BuildBaseTelemetry(
        std::uint16_t commandKindFlags,
        float targetForwardMps,
        float targetYawRateRadps,
        float targetForwardAccelMps2,
        float targetYawAccelRadps2,
        float targetYawRad) noexcept
    {
        DriveTelemetry telemetry{};
        telemetry.proposalSequenceId = _nextProposalSequenceId++;
        telemetry.requestedForwardMps = targetForwardMps;
        telemetry.requestedYawRateRadps = targetYawRateRadps;
        telemetry.requestedForwardAccelMps2 = targetForwardAccelMps2;
        telemetry.requestedYawAccelRadps2 = targetYawAccelRadps2;
        telemetry.requestedYawRad = targetYawRad;
        telemetry.commandKindFlags = commandKindFlags;
        telemetry.telemetryValidFlags =
            DriveTelemetry::kTelemetryProposalSequenceValid |
            DriveTelemetry::kTelemetryPlantCommandValid;
        return telemetry;
    }
}
