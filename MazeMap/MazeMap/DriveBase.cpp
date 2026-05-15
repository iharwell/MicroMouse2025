#include "pch.h"
#include "DriveBase.h"

#include "PlantModel.h"
#include "Vehicle.h"

using CommandVector = MazeMap::App::Internal::CommandVector;
namespace MazeMap {
    CommandVector DriveBase::DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        MazeMap::FeedbackSource linearSources) const
    {
        const CommandVector baseCommand =
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                0.0f,
                desiredLongitudinalAccelMps2,
                0.0f);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);

        CommandVector command = baseCommand;
        if (linearSources != MazeMap::FeedbackSource::None)
        {
            command +=
                ResolveLongitudinalCorrectionCommand(
                    presentLinearSpeedMps,
                    0.0f,
                    maxLongitudinalAccelMps2,
                    _linearFeedback.GetFeedback(
                        2U,
                        desiredLongitudinalAccelMps2,
                        linearSources,
                        GetProportionalDerivativeCluster()));
        }

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = presentLinearSpeedMps;
        _lastAngularCommandRadps = 0.0f;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    CommandVector DriveBase::DeltaCommand(
        float presentLinearSpeedMps,
        float desiredLongitudinalAccelMps2,
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources) const
    {
        const CommandVector baseCommand =
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                presentYawRateRadps,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);

        CommandVector command = baseCommand;
        if (linearSources != MazeMap::FeedbackSource::None)
        {
            command +=
                ResolveLongitudinalCorrectionCommand(
                    presentLinearSpeedMps,
                    presentYawRateRadps,
                    maxLongitudinalAccelMps2,
                    _linearFeedback.GetFeedback(
                        2U,
                        desiredLongitudinalAccelMps2,
                        linearSources,
                        GetProportionalDerivativeCluster()));
        }
        if (rotationalSources != MazeMap::FeedbackSource::None)
        {
            command +=
                ResolveYawCorrectionCommand(
                    presentLinearSpeedMps,
                    presentYawRateRadps,
                    maxYawAccelRadps2,
                    _rotationalFeedback.GetFeedback(
                        2U,
                        desiredYawAccelRadps2,
                        rotationalSources,
                        GetProportionalDerivativeCluster()));
        }

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = presentLinearSpeedMps;
        _lastAngularCommandRadps = presentYawRateRadps;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::DeltaYawRateCommand(
        float presentYawRateRadps,
        float desiredYawAccelRadps2,
        MazeMap::FeedbackSource rotationalSources) const
    {
        const CommandVector baseCommand =
            ResolveRawAccelerationCommand(
                0.0f,
                presentYawRateRadps,
                0.0f,
                desiredYawAccelRadps2);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);

        CommandVector command = baseCommand;
        if (rotationalSources != MazeMap::FeedbackSource::None)
        {
            command +=
                ResolveYawCorrectionCommand(
                    0.0f,
                    presentYawRateRadps,
                    maxYawAccelRadps2,
                    _rotationalFeedback.GetFeedback(
                        2U,
                        desiredYawAccelRadps2,
                        rotationalSources,
                        GetProportionalDerivativeCluster()));
        }

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = presentYawRateRadps;
        _lastLeftTargetVelocityMps = 0.0f;
        _lastRightTargetVelocityMps = 0.0f;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
        float desiredLinearSpeedMps,
        MazeMap::FeedbackSource linearSources) const
    {
        float presentLinearSpeedMps = 0.0f;
        float presentYawRateRadps = 0.0f;
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingPoint(presentLinearSpeedMps, presentYawRateRadps, batteryVoltageV);

        const CommandVector baseCommand =
            ResolveRawVelocityTargetCommand(
                desiredLinearSpeedMps,
                presentYawRateRadps);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);

        float adjustedLinearSpeedMps = desiredLinearSpeedMps;
        if (linearSources != MazeMap::FeedbackSource::None)
        {
            adjustedLinearSpeedMps +=
                ClampMagnitude(
                    _linearFeedback.GetFeedback(
                        1U,
                        desiredLinearSpeedMps,
                        linearSources,
                        GetProportionalDerivativeCluster()),
                    ResolveCommandResponseTimeS() * maxLongitudinalAccelMps2);
        }

        CommandVector command =
            (adjustedLinearSpeedMps != desiredLinearSpeedMps) ?
            ResolveRawVelocityTargetCommand(adjustedLinearSpeedMps, presentYawRateRadps) :
            baseCommand;
        float leftWheelLinearTargetMps = 0.0f;
        float rightWheelLinearTargetMps = 0.0f;
        ResolveWheelTargets(
            adjustedLinearSpeedMps,
            presentYawRateRadps,
            leftWheelLinearTargetMps,
            rightWheelLinearTargetMps);

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = adjustedLinearSpeedMps;
        _lastAngularCommandRadps = presentYawRateRadps;
        _lastLeftTargetVelocityMps = leftWheelLinearTargetMps;
        _lastRightTargetVelocityMps = rightWheelLinearTargetMps;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources) const
    {
        float presentLinearSpeedMps = 0.0f;
        float presentYawRateRadps = 0.0f;
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingPoint(presentLinearSpeedMps, presentYawRateRadps, batteryVoltageV);

        const CommandVector baseCommand =
            ResolveRawVelocityTargetCommand(
                desiredLinearSpeedMps,
                desiredYawRateRadps);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);
        const float responseTimeS = ResolveCommandResponseTimeS();

        float adjustedLinearSpeedMps = desiredLinearSpeedMps;
        float adjustedYawRateRadps = desiredYawRateRadps;
        if (linearSources != MazeMap::FeedbackSource::None)
        {
            adjustedLinearSpeedMps +=
                ClampMagnitude(
                    _linearFeedback.GetFeedback(
                        1U,
                        desiredLinearSpeedMps,
                        linearSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxLongitudinalAccelMps2);
        }
        if (rotationalSources != MazeMap::FeedbackSource::None)
        {
            adjustedYawRateRadps +=
                ClampMagnitude(
                    responseTimeS *
                    _rotationalFeedback.GetFeedback(
                        1U,
                        desiredYawRateRadps,
                        rotationalSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxYawAccelRadps2);
        }

        CommandVector command =
            ((adjustedLinearSpeedMps != desiredLinearSpeedMps) ||
                (adjustedYawRateRadps != desiredYawRateRadps)) ?
            ResolveRawVelocityTargetCommand(adjustedLinearSpeedMps, adjustedYawRateRadps) :
            baseCommand;
        float leftWheelLinearTargetMps = 0.0f;
        float rightWheelLinearTargetMps = 0.0f;
        ResolveWheelTargets(
            adjustedLinearSpeedMps,
            adjustedYawRateRadps,
            leftWheelLinearTargetMps,
            rightWheelLinearTargetMps);

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = adjustedLinearSpeedMps;
        _lastAngularCommandRadps = adjustedYawRateRadps;
        _lastLeftTargetVelocityMps = leftWheelLinearTargetMps;
        _lastRightTargetVelocityMps = rightWheelLinearTargetMps;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointCommandWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources,
        MazeMap::FeedbackSource headingSources) const
    {
        float presentLinearSpeedMps = 0.0f;
        float presentYawRateRadps = 0.0f;
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingPoint(presentLinearSpeedMps, presentYawRateRadps, batteryVoltageV);

        const CommandVector baseCommand =
            ResolveRawVelocityTargetCommand(
                desiredLinearSpeedMps,
                desiredYawRateRadps);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);
        const float responseTimeS = ResolveCommandResponseTimeS();

        float adjustedLinearSpeedMps = desiredLinearSpeedMps;
        float adjustedYawRateRadps = desiredYawRateRadps;
        if (linearSources != MazeMap::FeedbackSource::None)
        {
            adjustedLinearSpeedMps +=
                ClampMagnitude(
                    _linearFeedback.GetFeedback(
                        1U,
                        desiredLinearSpeedMps,
                        linearSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxLongitudinalAccelMps2);
        }
        if (headingSources != MazeMap::FeedbackSource::None)
        {
            adjustedYawRateRadps +=
                ClampMagnitude(
                    _rotationalFeedback.GetFeedback(
                        0U,
                        targetYawRad,
                        headingSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxYawAccelRadps2);
        }
        else if (rotationalSources != MazeMap::FeedbackSource::None)
        {
            adjustedYawRateRadps +=
                ClampMagnitude(
                    responseTimeS *
                    _rotationalFeedback.GetFeedback(
                        1U,
                        desiredYawRateRadps,
                        rotationalSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxYawAccelRadps2);
        }

        CommandVector command =
            ((adjustedLinearSpeedMps != desiredLinearSpeedMps) ||
                (adjustedYawRateRadps != desiredYawRateRadps)) ?
            ResolveRawVelocityTargetCommand(adjustedLinearSpeedMps, adjustedYawRateRadps) :
            baseCommand;
        float leftWheelLinearTargetMps = 0.0f;
        float rightWheelLinearTargetMps = 0.0f;
        ResolveWheelTargets(
            adjustedLinearSpeedMps,
            adjustedYawRateRadps,
            leftWheelLinearTargetMps,
            rightWheelLinearTargetMps);

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = adjustedLinearSpeedMps;
        _lastAngularCommandRadps = adjustedYawRateRadps;
        _lastLeftTargetVelocityMps = leftWheelLinearTargetMps;
        _lastRightTargetVelocityMps = rightWheelLinearTargetMps;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointControlVector(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources) const
    {
        return PointCommand(
            desiredLinearSpeedMps,
            desiredYawRateRadps,
            linearSources,
            rotationalSources);
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointControlVectorWithHeadingTarget(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float targetYawRad,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources,
        MazeMap::FeedbackSource headingSources) const
    {
        return PointCommandWithHeadingTarget(
            desiredLinearSpeedMps,
            desiredYawRateRadps,
            targetYawRad,
            linearSources,
            rotationalSources,
            headingSources);
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
        const MazeMap::ManeuverPoint& point,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources) const
    {
        if (!point.IsFinite())
        {
            return {};
        }

        return PointCommand(point.Velocity, point.Omega, linearSources, rotationalSources);
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointControlVector(
        const MazeMap::ManeuverPoint& point,
        MazeMap::FeedbackSource linearSources,
        MazeMap::FeedbackSource rotationalSources) const
    {
        return PointCommand(point, linearSources, rotationalSources);
    }

    MazeMap::App::Internal::CommandVector DriveBase::PointYawRateCommand(
        float desiredYawRateRadps,
        MazeMap::FeedbackSource rotationalSources) const
    {
        float presentLinearSpeedMps = 0.0f;
        float presentYawRateRadps = 0.0f;
        float batteryVoltageV = 0.0f;
        GetVelocityCommandOperatingPoint(presentLinearSpeedMps, presentYawRateRadps, batteryVoltageV);

        const CommandVector baseCommand =
            ResolveRawVelocityTargetCommand(
                presentLinearSpeedMps,
                desiredYawRateRadps);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        ResolveDefaultVelocityTargetCommandEnvelope(maxLongitudinalAccelMps2, maxYawAccelRadps2);
        const float responseTimeS = ResolveCommandResponseTimeS();

        float adjustedYawRateRadps = desiredYawRateRadps;
        if (rotationalSources != MazeMap::FeedbackSource::None)
        {
            adjustedYawRateRadps +=
                ClampMagnitude(
                    responseTimeS *
                    _rotationalFeedback.GetFeedback(
                        1U,
                        desiredYawRateRadps,
                        rotationalSources,
                        GetProportionalDerivativeCluster()),
                    responseTimeS * maxYawAccelRadps2);
        }

        CommandVector command =
            (adjustedYawRateRadps != desiredYawRateRadps) ?
            ResolveRawVelocityTargetCommand(presentLinearSpeedMps, adjustedYawRateRadps) :
            baseCommand;
        float leftWheelLinearTargetMps = 0.0f;
        float rightWheelLinearTargetMps = 0.0f;
        ResolveWheelTargets(
            presentLinearSpeedMps,
            adjustedYawRateRadps,
            leftWheelLinearTargetMps,
            rightWheelLinearTargetMps);

        const CommandVector feedbackCommand = command - baseCommand;
        const CommandVector clampedCommand(
            (std::clamp)(command.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(command.RightMotorPwm(), -1.0f, 1.0f));
        CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
        _lastLinearCommandMps = presentLinearSpeedMps;
        _lastAngularCommandRadps = adjustedYawRateRadps;
        _lastLeftTargetVelocityMps = leftWheelLinearTargetMps;
        _lastRightTargetVelocityMps = rightWheelLinearTargetMps;
        _lastModeFlags = kModeClosedLoop;
        _lastSaturationFlags =
            ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
            ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
        _lastProposedCommand = clampedCommand;
        return clampedCommand;
    }

    MazeMap::App::Internal::CommandVector DriveBase::GetFeedbackCommand(
        const std::uint8_t linearDerivativeOrder,
        const float linearTarget,
        const MazeMap::FeedbackSource linearSources,
        const std::uint8_t rotationalDerivativeOrder,
        const float rotationalTarget,
        const MazeMap::FeedbackSource rotationalSources) const
    {
        const float linearFeedback =
            _linearFeedback.GetFeedback(
                linearDerivativeOrder,
                linearTarget,
                linearSources,
                GetProportionalDerivativeCluster());
        const float rotationalFeedback =
            _rotationalFeedback.GetFeedback(
                rotationalDerivativeOrder,
                rotationalTarget,
                rotationalSources,
                GetProportionalDerivativeCluster());
        return CommandVector::FromAverageAndDifferential(linearFeedback, rotationalFeedback);
    }

    void DriveBase::GetVelocityCommandOperatingPoint(
        float& presentLinearSpeedMps,
        float& presentYawRateRadps,
        float& batteryVoltageV) const
    {
        RefreshSensorSnapshotDerivedState();
        const float measuredLeftVelocityMps = _leftEncoderVelocityMps;
        const float measuredRightVelocityMps = _rightEncoderVelocityMps;
        float measuredLinearSpeedMps = 0.0f;
        float measuredAngularSpeedRadps = 0.0f;
        measuredLinearSpeedMps =
            MazeMap::Vehicle::BodyForwardVelocityFromWheelLinear(measuredLeftVelocityMps, measuredRightVelocityMps);
        measuredAngularSpeedRadps =
            MazeMap::Vehicle::BodyYawRateFromWheelLinear(measuredLeftVelocityMps, measuredRightVelocityMps);
        if (_lastImuYawRateValid)
        {
            measuredAngularSpeedRadps = _lastImuYawRateRadps;
        }
        presentLinearSpeedMps =
            std::isfinite(_runtimeState.GetVelocity()) ?
            _runtimeState.GetVelocity() :
            measuredLinearSpeedMps;
        presentYawRateRadps =
            std::isfinite(_runtimeState.GetRotationalVelocity()) ?
            _runtimeState.GetRotationalVelocity() :
            measuredAngularSpeedRadps;
        if (!std::isfinite(presentLinearSpeedMps))
        {
            presentLinearSpeedMps = 0.0f;
        }
        if (!std::isfinite(presentYawRateRadps))
        {
            presentYawRateRadps = 0.0f;
        }
    }

    float DriveBase::ResolveCommandResponseTimeS() noexcept
    {
        return
            (std::isfinite(MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS) &&
                (MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS > 0.0f)) ?
            MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS :
            1.0f;
    }

    float DriveBase::ResolvePositiveOrZero(float value) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : 0.0f;
    }

    float DriveBase::ClampMagnitude(float value, float limit) noexcept
    {
        return
            (ResolvePositiveOrZero(limit) > 0.0f) ?
            (std::clamp)(value, -limit, limit) :
            0.0f;
    }

    void DriveBase::RefreshSensorSnapshotDerivedState() const noexcept
    {
        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        _lastGyroRawRadps = snapshot.gyroRawRadps;
        _lastImuYawRateRadps = snapshot.gyroRadps;
        _lastImuYawRateValid = std::isfinite(snapshot.gyroRadps);
        _lastImuAccelBodyXMps2 = snapshot.accelBodyXMps2;
        _lastImuAccelBodyYMps2 = snapshot.accelBodyYMps2;
        _lastImuAccelValid =
            snapshot.accelBiasValid &&
            std::isfinite(snapshot.accelBodyXMps2) &&
            std::isfinite(snapshot.accelBodyYMps2);
        if (!_encoderReferenceInitialized)
        {
            _leftEncoderReferenceCounts = snapshot.leftEncoderTotalCounts;
            _rightEncoderReferenceCounts = snapshot.rightEncoderTotalCounts;
            _encoderReferenceInitialized = true;
        }

        _leftEncoderCountTotal = snapshot.leftEncoderTotalCounts - _leftEncoderReferenceCounts;
        _rightEncoderCountTotal = snapshot.rightEncoderTotalCounts - _rightEncoderReferenceCounts;
        _leftEncoderDistanceMeters =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(_leftEncoderCountTotal);
        _rightEncoderDistanceMeters =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(_rightEncoderCountTotal);

        if (!snapshot.encoderObservationValid)
        {
            // Invalid current encoder data clears instantaneous encoder state while preserving the
            // sensor-suite-owned accumulated totals copied above.
            _encoderObservationValid = false;
            _lastEncoderObservation = {};
            _leftEncoderVelocityMps = 0.0f;
            _rightEncoderVelocityMps = 0.0f;
            return;
        }

        const MazeMap::EncoderObs& observation = snapshot.encoderObservation;
        _lastEncoderObservation = observation;
        _encoderObservationValid = true;
        _leftEncoderVelocityMps =
            std::isfinite(observation.leftVelocityMps) ? observation.leftVelocityMps : 0.0f;
        _rightEncoderVelocityMps =
            std::isfinite(observation.rightVelocityMps) ? observation.rightVelocityMps : 0.0f;
    }

    void DriveBase::ResolveWheelTargets(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps,
        float& leftWheelLinearTargetMps,
        float& rightWheelLinearTargetMps) const
    {
        float unusedLeftTargetAccelMps2 = 0.0f;
        float unusedRightTargetAccelMps2 = 0.0f;
        float unusedLeftTargetOmegaRadps = 0.0f;
        float unusedRightTargetOmegaRadps = 0.0f;
        _plantModel.resolveWheelMotionTargets(
            desiredLinearSpeedMps,
            desiredYawRateRadps,
            0.0f,
            0.0f,
            leftWheelLinearTargetMps,
            rightWheelLinearTargetMps,
            unusedLeftTargetAccelMps2,
            unusedRightTargetAccelMps2,
            unusedLeftTargetOmegaRadps,
            unusedRightTargetOmegaRadps);
    }

    MazeMap::App::Internal::CommandVector DriveBase::ResolveRawAccelerationCommand(
        float presentLinearSpeedMps,
        float presentYawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2) const
    {
        const bool isSteadyHoldRequest =
            (std::fabs(desiredLongitudinalAccelMps2) <= 1.0e-5f) &&
            (std::fabs(desiredYawAccelRadps2) <= 1.0e-5f);
        const CommandVector rawCommand =
            isSteadyHoldRequest ?
            _plantModel.solveSteadyStateFeedforward(
                presentLinearSpeedMps,
                presentYawRateRadps) :
            _plantModel.solveAccelerationFeedforward(
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);
        // Precondition: PlantModel feedforward returns finite PWM. DriveBase only enforces the PWM envelope.
        return CommandVector(
            (std::clamp)(rawCommand.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(rawCommand.RightMotorPwm(), -1.0f, 1.0f));
    }

    MazeMap::App::Internal::CommandVector DriveBase::ResolveRawVelocityTargetCommand(
        float desiredLinearSpeedMps,
        float desiredYawRateRadps) const
    {
        const CommandVector rawCommand =
            _plantModel.solveSteadyStateFeedforward(
                desiredLinearSpeedMps,
                desiredYawRateRadps);
        // Precondition: PlantModel feedforward returns finite PWM. DriveBase only enforces the PWM envelope.
        return CommandVector(
            (std::clamp)(rawCommand.LeftMotorPwm(), -1.0f, 1.0f),
            (std::clamp)(rawCommand.RightMotorPwm(), -1.0f, 1.0f));
    }

    MazeMap::App::Internal::CommandVector DriveBase::ResolveLongitudinalCorrectionCommand(
        const float presentLinearSpeedMps,
        const float presentYawRateRadps,
        const float maxLongitudinalAccelMps2,
        float desiredLongitudinalAccelCorrectionMps2) const
    {
        const float resolvedCorrectionMps2 =
            ClampMagnitude(
                desiredLongitudinalAccelCorrectionMps2,
                maxLongitudinalAccelMps2);
        return
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                presentYawRateRadps,
                resolvedCorrectionMps2,
                0.0f) -
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                presentYawRateRadps,
                0.0f,
                0.0f);
    }

    MazeMap::App::Internal::CommandVector DriveBase::ResolveYawCorrectionCommand(
        const float presentLinearSpeedMps,
        const float presentYawRateRadps,
        const float maxYawAccelRadps2,
        float desiredYawAccelCorrectionRadps2) const
    {
        const float resolvedCorrectionRadps2 =
            ClampMagnitude(
                desiredYawAccelCorrectionRadps2,
                maxYawAccelRadps2);
        return
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                presentYawRateRadps,
                0.0f,
                resolvedCorrectionRadps2) -
            ResolveRawAccelerationCommand(
                presentLinearSpeedMps,
                presentYawRateRadps,
                0.0f,
                0.0f);
    }

    void DriveBase::CacheGeneratedCommandTelemetry(
        const MazeMap::App::Internal::CommandVector& feedforwardCommand,
        const MazeMap::App::Internal::CommandVector& feedbackCommand) const noexcept
    {
        _lastFeedforwardCommand = feedforwardCommand;
        _lastFeedbackCommand = feedbackCommand;
    }

}