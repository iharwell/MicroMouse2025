#include "pch.h"
#include "DriveBase.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;

    float NormalizeClockwiseYawError(float errorRad) noexcept
    {
        if (!std::isfinite(errorRad))
        {
            return 0.0f;
        }

        while (errorRad > kPi)
        {
            errorRad -= kTwoPi;
        }
        while (errorRad <= -kPi)
        {
            errorRad += kTwoPi;
        }
        return errorRad;
    }

    bool IsMaximizeObjective(float value) noexcept
    {
        return std::isinf(value);
    }

    float AddFeedbackAccel(float accumulatedAccel, float feedbackAccel) noexcept
    {
        if (!std::isfinite(feedbackAccel))
        {
            return accumulatedAccel;
        }
        return std::isfinite(accumulatedAccel) ? (accumulatedAccel + feedbackAccel) : feedbackAccel;
    }
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

        const VehicleState::StateVector stateSnapshot = CaptureStateSnapshot(_runtimeState);
        const float observedForwardMps = stateSnapshot(VehicleState::kU);
        const float observedYawRateRadps = stateSnapshot(VehicleState::kR);
        const float observedYawRad = stateSnapshot(VehicleState::kPsi);

        float forwardFeedbackAccelMps2 = (std::numeric_limits<float>::quiet_NaN)();
        if (std::isfinite(targetForwardMps) && std::isfinite(observedForwardMps))
        {
            forwardFeedbackAccelMps2 =
                AddFeedbackAccel(
                    forwardFeedbackAccelMps2,
                    ComputeForwardVelocityFeedbackAccelMps2(
                        _feedbackTuning,
                        targetForwardMps - observedForwardMps));
        }
        else
        {
            telemetry.feedbackBranchFlags |= DriveTelemetry::kFeedbackForwardVelocityInactive;
        }

        float yawFeedbackAccelRadps2 = (std::numeric_limits<float>::quiet_NaN)();
        if (std::isfinite(targetYawRateRadps) && std::isfinite(observedYawRateRadps))
        {
            yawFeedbackAccelRadps2 =
                AddFeedbackAccel(
                    yawFeedbackAccelRadps2,
                    ComputeYawRateFeedbackAccelRadps2(
                        _feedbackTuning,
                        targetYawRateRadps - observedYawRateRadps));
        }
        else
        {
            telemetry.feedbackBranchFlags |= DriveTelemetry::kFeedbackYawRateInactive;
        }

        if (std::isfinite(targetYawRad) && std::isfinite(observedYawRad))
        {
            const float headingTargetYawRateRadps =
                std::isfinite(targetYawRateRadps) ? targetYawRateRadps : 0.0f;
            const float headingErrorRateRadps =
                std::isfinite(observedYawRateRadps) ?
                (headingTargetYawRateRadps - observedYawRateRadps) :
                0.0f;
            yawFeedbackAccelRadps2 =
                AddFeedbackAccel(
                    yawFeedbackAccelRadps2,
                    ComputeHeadingFeedbackAccelRadps2(
                        _feedbackTuning,
                        NormalizeClockwiseYawError(targetYawRad - observedYawRad),
                        headingErrorRateRadps));
        }
        else
        {
            telemetry.feedbackBranchFlags |= DriveTelemetry::kFeedbackHeadingInactive;
        }

        if (IsMaximizeObjective(targetForwardAccelMps2))
        {
            telemetry.composedForwardAccelMps2 = targetForwardAccelMps2;
            telemetry.feedbackBranchFlags |= DriveTelemetry::kFeedbackForwardSuppressedForMaximize;
        }
        else
        {
            telemetry.composedForwardAccelMps2 =
                ComposeAccelerationObjective(targetForwardAccelMps2, forwardFeedbackAccelMps2);
        }

        if (IsMaximizeObjective(targetYawAccelRadps2))
        {
            telemetry.composedYawAccelRadps2 = targetYawAccelRadps2;
            telemetry.feedbackBranchFlags |= DriveTelemetry::kFeedbackYawSuppressedForMaximize;
        }
        else
        {
            telemetry.composedYawAccelRadps2 =
                ComposeAccelerationObjective(targetYawAccelRadps2, yawFeedbackAccelRadps2);
        }

        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        _plant.velocityTargetTechnicalLimits(maxLongitudinalAccelMps2, maxYawAccelRadps2);

        const float feedforwardForwardAccelMps2 =
            ResolveComposedAccelerationObjective(telemetry.composedForwardAccelMps2, maxLongitudinalAccelMps2);
        const float feedforwardYawAccelRadps2 =
            ResolveComposedAccelerationObjective(telemetry.composedYawAccelRadps2, maxYawAccelRadps2);
        const CommandVector plantCommand =
            _plant.ComputeFeedforward(feedforwardForwardAccelMps2, feedforwardYawAccelRadps2);

        telemetry.leftPlantCommand = plantCommand.LeftCommand();
        telemetry.rightPlantCommand = plantCommand.RightCommand();

        CommandVector finalCommand(
            ClampCommandComponent(plantCommand.LeftCommand()),
            ClampCommandComponent(plantCommand.RightCommand()));
        if (!plantCommand.IsFinite())
        {
            finalCommand = CommandVector(0.0f, 0.0f);
            telemetry.commandKindFlags |= DriveTelemetry::kCommandKindSolverFailureEvidence;
            telemetry.solverFailureFlags |= DriveTelemetry::kSolverFailurePlantNonFinite;
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

    VehicleState::StateVector DriveBase::CaptureStateSnapshot(
        const MazeMap::VehicleState& runtimeState) noexcept
    {
        VehicleState::StateVector state = VehicleState::StateVector::Zero();
        state(VehicleState::kPx) = runtimeState.GetPositionX();
        state(VehicleState::kPy) = runtimeState.GetPositionY();
        state(VehicleState::kPsi) = runtimeState.GetOrientation();
        state(VehicleState::kU) = runtimeState.GetVelocity();
        state(VehicleState::kV) = runtimeState.GetLateralVelocity();
        state(VehicleState::kR) = runtimeState.GetRotationalVelocity();
        state(VehicleState::kOmegaL) = runtimeState.GetWheelSpeedLeft();
        state(VehicleState::kOmegaR) = runtimeState.GetWheelSpeedRight();
        state(VehicleState::kBgz) = runtimeState.GetGyroBiasZ();
        VehicleState::NormalizeStateVector(state);
        return state;
    }

    float DriveBase::ComposeAccelerationObjective(float requestedAccel, float feedbackAccel) noexcept
    {
        const bool hasRequestedAccel = std::isfinite(requestedAccel);
        const bool hasFeedbackAccel = std::isfinite(feedbackAccel);
        if (hasRequestedAccel)
        {
            return requestedAccel + (hasFeedbackAccel ? feedbackAccel : 0.0f);
        }
        return hasFeedbackAccel ? feedbackAccel : (std::numeric_limits<float>::quiet_NaN)();
    }

    float DriveBase::ClampCommandComponent(float command) noexcept
    {
        return std::isfinite(command) ? (std::clamp)(command, -1.0f, 1.0f) : 0.0f;
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

    float DriveBase::ResolveComposedAccelerationObjective(float composedAccel, float maximizeLimit) noexcept
    {
        if (std::isnan(composedAccel))
        {
            return 0.0f;
        }
        if (std::isinf(composedAccel))
        {
            const float resolvedLimit =
                (std::isfinite(maximizeLimit) && (maximizeLimit > 0.0f)) ? maximizeLimit : 0.0f;
            return std::signbit(composedAccel) ? -resolvedLimit : resolvedLimit;
        }
        return std::isfinite(composedAccel) ? composedAccel : 0.0f;
    }

    std::uint16_t DriveBase::DecodeScalarIntentFlags(
        float targetForwardMps,
        float targetYawRateRadps,
        float targetForwardAccelMps2,
        float targetYawAccelRadps2,
        float targetYawRad) noexcept
    {
        std::uint16_t flags = 0U;
        flags |=
            std::isnan(targetForwardMps) ? DriveTelemetry::kScalarForwardVelocityInactive :
            (std::isfinite(targetForwardMps) ? DriveTelemetry::kScalarForwardVelocityFinite :
                DriveTelemetry::kScalarForwardVelocityMaximize);
        flags |=
            std::isnan(targetYawRateRadps) ? DriveTelemetry::kScalarYawRateInactive :
            (std::isfinite(targetYawRateRadps) ? DriveTelemetry::kScalarYawRateFinite :
                DriveTelemetry::kScalarYawRateMaximize);
        flags |=
            std::isnan(targetForwardAccelMps2) ? DriveTelemetry::kScalarForwardAccelInactive :
            (std::isfinite(targetForwardAccelMps2) ? DriveTelemetry::kScalarForwardAccelFinite :
                DriveTelemetry::kScalarForwardAccelMaximize);
        flags |=
            std::isnan(targetYawAccelRadps2) ? DriveTelemetry::kScalarYawAccelInactive :
            (std::isfinite(targetYawAccelRadps2) ? DriveTelemetry::kScalarYawAccelFinite :
                DriveTelemetry::kScalarYawAccelMaximize);
        flags |=
            std::isnan(targetYawRad) ? DriveTelemetry::kScalarYawInactive :
            (std::isfinite(targetYawRad) ? DriveTelemetry::kScalarYawFinite :
                DriveTelemetry::kScalarYawMaximizeUnsupported);
        return flags;
    }

    std::uint16_t DriveBase::DecodeUnsupportedScalarIntentFlags(float targetYawRad) noexcept
    {
        return
            std::isinf(targetYawRad) ?
            DriveTelemetry::kSolverFailureUnsupportedScalarIntent :
            0U;
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
        telemetry.scalarIntentFlags =
            DecodeScalarIntentFlags(
                targetForwardMps,
                targetYawRateRadps,
                targetForwardAccelMps2,
                targetYawAccelRadps2,
                targetYawRad);
        telemetry.telemetryValidFlags =
            DriveTelemetry::kTelemetryProposalSequenceValid |
            DriveTelemetry::kTelemetryPlantCommandValid;
        telemetry.solverFailureFlags =
            DecodeUnsupportedScalarIntentFlags(targetYawRad);
        return telemetry;
    }
}
