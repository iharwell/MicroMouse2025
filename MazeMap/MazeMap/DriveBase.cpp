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
        const MazeMap::DriveBaseTrackingTuning& feedbackTuning) noexcept
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
        const float observedForwardAccelMps2 = _runtimeState.GetForwardAcceleration();
        const float observedYawAccelRadps2 = _runtimeState.GetYawAccel();
        const float omittedTrackingPoint = (std::numeric_limits<float>::quiet_NaN)();

        float forwardVelocityErrorMps = omittedTrackingPoint;
        if (std::isfinite(targetForwardMps) && std::isfinite(observedForwardMps))
        {
            forwardVelocityErrorMps = targetForwardMps - observedForwardMps;
        }

        float forwardAccelerationErrorMps2 = omittedTrackingPoint;
        if (std::isfinite(targetForwardAccelMps2) && std::isfinite(observedForwardAccelMps2))
        {
            forwardAccelerationErrorMps2 = targetForwardAccelMps2 - observedForwardAccelMps2;
        }

        float yawVelocityErrorRadps = omittedTrackingPoint;
        if (std::isfinite(targetYawRateRadps) && std::isfinite(observedYawRateRadps))
        {
            yawVelocityErrorRadps = targetYawRateRadps - observedYawRateRadps;
        }

        float yawPositionErrorRad = omittedTrackingPoint;
        if (std::isfinite(targetYawRad) && std::isfinite(observedYawRad))
        {
            yawPositionErrorRad = AngleDifference(observedYawRad, targetYawRad);
        }

        float yawAccelerationErrorRadps2 = omittedTrackingPoint;
        if (std::isfinite(targetYawAccelRadps2) && std::isfinite(observedYawAccelRadps2))
        {
            yawAccelerationErrorRadps2 = targetYawAccelRadps2 - observedYawAccelRadps2;
        }

        telemetry.composedForwardAccelMps2 =
            _feedbackTuning.ComposeForwardAccelerationMps2(
                targetForwardAccelMps2,
                omittedTrackingPoint,
                forwardVelocityErrorMps,
                forwardAccelerationErrorMps2);
        telemetry.composedYawAccelRadps2 =
            _feedbackTuning.ComposeYawAccelerationRadps2(
                targetYawAccelRadps2,
                yawPositionErrorRad,
                yawVelocityErrorRadps,
                yawAccelerationErrorRadps2);

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
