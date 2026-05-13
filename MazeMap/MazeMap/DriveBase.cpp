#include "pch.h"
#include "DriveBase.h"

#include "PlantModel.h"
#include "Vehicle.h"

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;

    CommandVector MakeClampedDriveControlVector(float leftMotorPwm, float rightMotorPwm) noexcept
    {
        if (!std::isfinite(leftMotorPwm) || !std::isfinite(rightMotorPwm))
        {
            return {};
        }

        return CommandVector(
            (std::clamp)(leftMotorPwm, -1.0f, 1.0f),
            (std::clamp)(rightMotorPwm, -1.0f, 1.0f));
    }
}

MazeMap::App::Internal::CommandVector DriveBase::DeltaCommand(
    float presentLinearSpeedMps,
    float desiredLongitudinalAccelMps2,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasLongitudinalAccelTarget = true;
    targets.longitudinalAccelTargetMps2 = desiredLongitudinalAccelMps2;

    const CommandVector baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            0.0f,
            desiredLongitudinalAccelMps2,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::DeltaCommand(
    float presentLinearSpeedMps,
    float desiredLongitudinalAccelMps2,
    float presentYawRateRadps,
    float desiredYawAccelRadps2,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasLongitudinalAccelTarget = true;
    targets.longitudinalAccelTargetMps2 = desiredLongitudinalAccelMps2;
    targets.hasYawAccelTarget = true;
    targets.yawAccelTargetRadps2 = desiredYawAccelRadps2;

    const CommandVector baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            presentYawRateRadps,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::DeltaYawRateCommand(
    float presentYawRateRadps,
    float desiredYawAccelRadps2,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasYawAccelTarget = true;
    targets.yawAccelTargetRadps2 = desiredYawAccelRadps2;

    const CommandVector baseCommand =
        ResolveRawAccelerationCommand(
            0.0f,
            presentYawRateRadps,
            0.0f,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
    float desiredLinearSpeedMps,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasVelocityTarget = true;
    targets.velocityTargetMps = desiredLinearSpeedMps;
    ResolveWheelTargets(desiredLinearSpeedMps, 0.0f, targets);

    const CommandVector baseCommand =
        ResolveRawVelocityTargetCommand(
            desiredLinearSpeedMps,
            context.presentYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasVelocityTarget = true;
    targets.velocityTargetMps = desiredLinearSpeedMps;
    targets.hasYawRateTarget = true;
    targets.yawRateTargetRadps = desiredYawRateRadps;
    ResolveWheelTargets(desiredLinearSpeedMps, desiredYawRateRadps, targets);

    const CommandVector baseCommand =
        ResolveRawVelocityTargetCommand(
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointCommandWithHeadingTarget(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    float targetYawRad,
    MazeMap::CommandPD pointPd,
    MazeMap::CommandPD headingPd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasVelocityTarget = true;
    targets.velocityTargetMps = desiredLinearSpeedMps;
    targets.hasYawRateTarget = true;
    targets.yawRateTargetRadps = desiredYawRateRadps;
    if (MazeMap::HasCommandPD(headingPd, MazeMap::CommandPD::StateHeadingPD))
    {
        targets.hasHeadingTarget = true;
        targets.headingTargetYawRad = targetYawRad;
    }
    ResolveWheelTargets(desiredLinearSpeedMps, desiredYawRateRadps, targets);

    const CommandVector baseCommand =
        ResolveRawVelocityTargetCommand(
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pointPd | headingPd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointControlVector(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    return PointCommand(
        desiredLinearSpeedMps,
        desiredYawRateRadps,
        pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointControlVectorWithHeadingTarget(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    float targetYawRad,
    MazeMap::CommandPD pointPd,
    MazeMap::CommandPD headingPd) const
{
    return PointCommandWithHeadingTarget(
        desiredLinearSpeedMps,
        desiredYawRateRadps,
        targetYawRad,
        pointPd,
        headingPd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointCommand(
    const MazeMap::ManeuverPoint& point,
    MazeMap::CommandPD pd) const
{
    if (!point.IsFinite())
    {
        return {};
    }

    return PointCommand(point.Velocity, point.Omega, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointControlVector(
    const MazeMap::ManeuverPoint& point,
    MazeMap::CommandPD pd) const
{
    return PointCommand(point, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::PointYawRateCommand(
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasYawRateTarget = true;
    targets.yawRateTargetRadps = desiredYawRateRadps;
    ResolveWheelTargets(0.0f, desiredYawRateRadps, targets);

    const CommandVector baseCommand =
        ResolveRawVelocityTargetCommand(
            context.presentLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::CommandVector DriveBase::FeedbackCommand(
    float setpoint,
    MazeMap::CommandPD pd) const
{
    if (pd == MazeMap::CommandPD::RawCommand)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasHeadingTarget = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateHeadingPD);
    targets.headingTargetYawRad = setpoint;
    targets.hasVelocityTarget =
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateVelocityPD) ||
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    targets.velocityTargetMps = setpoint;
    targets.hasYawRateTarget =
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD) ||
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw);
    targets.yawRateTargetRadps = setpoint;
    targets.hasLongitudinalAccelTarget =
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateAccelerationPD) ||
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUForwardAccel);
    targets.longitudinalAccelTargetMps2 = setpoint;
    targets.hasLateralAccelTarget =
        MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMULateralAccel);
    targets.lateralAccelTargetMps2 = setpoint;
    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity))
    {
        targets.hasWheelLinearTargets = true;
        targets.leftWheelLinearTargetMps = setpoint;
        targets.rightWheelLinearTargetMps = setpoint;
    }

    const CommandVector baseCommand =
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

void DriveBase::CommandGenerated(
    const MazeMap::App::Internal::CommandVector& command,
    float linearSpeedMps,
    float angularSpeedRadps,
    bool applyLaunchAssist)
{
    RefreshSensorSnapshotDerivedState();
    const float leftMeasuredMps = _leftEncoderVelocityMps;
    const float rightMeasuredMps = _rightEncoderVelocityMps;
    float leftDriveCommand = command.LeftMotorPwm();
    float rightDriveCommand = command.RightMotorPwm();
    const unsigned long nowMs = millis();
    if (applyLaunchAssist &&
        UpdateWheelLaunchAssistState(
            _leftLaunchAssist,
            leftMeasuredMps,
            leftDriveCommand,
            _lastProposedCommand.LeftMotorPwm(),
            nowMs))
    {
        _lastLeftLaunchAssistFloor = GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs);
        leftDriveCommand = ApplyLaunchAssistFloor(
            leftDriveCommand,
            leftDriveCommand,
            _lastLeftLaunchAssistFloor);
    }
    else
    {
        ResetWheelLaunchAssistState(_leftLaunchAssist, nowMs);
        _lastLeftLaunchAssistFloor = 0.0f;
    }
    if (applyLaunchAssist &&
        UpdateWheelLaunchAssistState(
            _rightLaunchAssist,
            rightMeasuredMps,
            rightDriveCommand,
            _lastProposedCommand.RightMotorPwm(),
            nowMs))
    {
        _lastRightLaunchAssistFloor = GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs);
        rightDriveCommand = ApplyLaunchAssistFloor(
            rightDriveCommand,
            rightDriveCommand,
            _lastRightLaunchAssistFloor);
    }
    else
    {
        ResetWheelLaunchAssistState(_rightLaunchAssist, nowMs);
        _lastRightLaunchAssistFloor = 0.0f;
    }

    _lastLinearCommandMps = linearSpeedMps;
    _lastAngularCommandRadps = angularSpeedRadps;
    _lastFeedforwardCommand = {};
    _lastFeedbackCommand = CommandVector(leftDriveCommand, rightDriveCommand);
    _lastLeftTargetVelocityMps = 0.0f;
    _lastRightTargetVelocityMps = 0.0f;
    _lastModeFlags = kModeClosedLoop |
        ((_lastLeftLaunchAssistFloor > 0.0f) ? kModeLaunchAssistLeft : 0u) |
        ((_lastRightLaunchAssistFloor > 0.0f) ? kModeLaunchAssistRight : 0u);
    _lastSaturationFlags =
        ((std::fabs(leftDriveCommand) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(rightDriveCommand) >= 0.999f) ? 0x2u : 0u);
    _lastProposedCommand = MakeClampedDriveControlVector(leftDriveCommand, rightDriveCommand);
}

void DriveBase::CommandOpenLoopRaw(
    const MazeMap::App::Internal::CommandVector& command)
{
    _lastLinearCommandMps = 0.0f;
    _lastAngularCommandRadps = 0.0f;
    ResetLaunchAssist();
    _lastFeedforwardCommand = {};
    _lastFeedbackCommand = command;
    _lastLeftTargetVelocityMps = 0.0f;
    _lastRightTargetVelocityMps = 0.0f;
    _lastLeftLaunchAssistFloor = 0.0f;
    _lastRightLaunchAssistFloor = 0.0f;
    _lastModeFlags = kModeRawOpenLoop;
    _lastSaturationFlags =
        ((std::fabs(command.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(command.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
    _lastProposedCommand = MakeClampedDriveControlVector(command.LeftMotorPwm(), command.RightMotorPwm());
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
    batteryVoltageV = CurrentBatteryVoltageV();
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

    if (!snapshot.encoderObservationValid)
    {
        _encoderObservationValid = false;
        _lastEncoderObservation = {};
        _leftEncoderVelocityMps = 0.0f;
        _rightEncoderVelocityMps = 0.0f;
        return;
    }

    const MazeMap::EncoderObs& observation = snapshot.encoderObservation;
    const float snapshotTimeS = _runtimeState.GetTime();
    const bool duplicateObservation =
        _hasProcessedEncoderSnapshot &&
        (_lastEncoderSnapshotTimeS == snapshotTimeS) &&
        (_lastEncoderObservation.totalLeftCounts == observation.totalLeftCounts) &&
        (_lastEncoderObservation.totalRightCounts == observation.totalRightCounts) &&
        (_lastEncoderObservation.omegaLeftRadps == observation.omegaLeftRadps) &&
        (_lastEncoderObservation.omegaRightRadps == observation.omegaRightRadps);

    if (!duplicateObservation)
    {
        _leftEncoderCountTotal += observation.totalLeftCounts;
        _rightEncoderCountTotal += observation.totalRightCounts;
        if (std::isfinite(observation.leftDistanceDeltaM))
        {
            _leftEncoderDistanceMeters += observation.leftDistanceDeltaM;
        }
        if (std::isfinite(observation.rightDistanceDeltaM))
        {
            _rightEncoderDistanceMeters += observation.rightDistanceDeltaM;
        }
        _lastEncoderObservation = observation;
        _lastEncoderSnapshotTimeS = snapshotTimeS;
        _hasProcessedEncoderSnapshot = true;
    }

    _encoderObservationValid = true;
    _leftEncoderVelocityMps =
        std::isfinite(observation.leftVelocityMps) ? observation.leftVelocityMps : 0.0f;
    _rightEncoderVelocityMps =
        std::isfinite(observation.rightVelocityMps) ? observation.rightVelocityMps : 0.0f;
}

DriveBase::CommandTargets DriveBase::BuildHoldTargets(const CommandContext& context) const noexcept
{
    CommandTargets targets{};
    targets.headingTargetYawRad = context.presentYawRad;
    targets.velocityTargetMps = context.presentLinearSpeedMps;
    targets.yawRateTargetRadps = context.presentYawRateRadps;
    targets.longitudinalAccelTargetMps2 = context.stateLongitudinalAccelMps2;
    targets.lateralAccelTargetMps2 = context.imuLateralAccelMps2;
    targets.leftWheelLinearTargetMps = context.encoderLeftVelocityMps;
    targets.rightWheelLinearTargetMps = context.encoderRightVelocityMps;
    return targets;
}

DriveBase::CommandContext DriveBase::CaptureCommandContext() const
{
    CommandContext context{};
    GetVelocityCommandOperatingPoint(
        context.presentLinearSpeedMps,
        context.presentYawRateRadps,
        context.batteryVoltageV);
    context.presentYawRad =
        WrapAngleRad(std::isfinite(_runtimeState.GetOrientation()) ? _runtimeState.GetOrientation() : 0.0f);

    context.encoderLeftVelocityMps = _leftEncoderVelocityMps;
    context.encoderRightVelocityMps = _rightEncoderVelocityMps;

    context.presentDerivatives =
        _plantModel.forwardStep(
            _lastProposedCommand,
            GetMissionFanDutyCycle(),
            context.batteryVoltageV);
    context.stateLongitudinalAccelMps2 =
        context.presentDerivatives.longitudinalAccelMps2;
    context.stateImuLateralAccelMps2 =
        context.presentDerivatives.imuAccelBodyMps2.x();
    context.stateImuForwardAccelMps2 =
        context.presentDerivatives.imuAccelBodyMps2.y();
    context.imuYawRateRadps =
        _lastImuYawRateValid ?
        _lastImuYawRateRadps :
        context.presentYawRateRadps;
    context.imuForwardAccelMps2 =
        _lastImuAccelValid ?
        _lastImuAccelBodyYMps2 :
        context.stateImuForwardAccelMps2;
    context.imuLateralAccelMps2 =
        _lastImuAccelValid ?
        _lastImuAccelBodyXMps2 :
        context.stateImuLateralAccelMps2;

    ResolveDefaultVelocityTargetOperatingEnvelope(
        context.maxLongitudinalAccelMps2,
        context.maxYawAccelRadps2);
    return context;
}

void DriveBase::ResolveWheelTargets(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    CommandTargets& targets) const
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
            targets.leftWheelLinearTargetMps,
            targets.rightWheelLinearTargetMps,
            unusedLeftTargetAccelMps2,
            unusedRightTargetAccelMps2,
            unusedLeftTargetOmegaRadps,
            unusedRightTargetOmegaRadps);
    targets.hasWheelLinearTargets = true;
}

void DriveBase::ResolveVelocityPointAcceleration(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float& desiredLongitudinalAccelMps2) const noexcept
{
    _plantModel.ComputeBodyAction(
        context.presentLinearSpeedMps,
        desiredLinearSpeedMps,
        context.presentYawRateRadps,
        context.maxLongitudinalAccelMps2,
        ResolveCommandResponseTimeS(),
        desiredLongitudinalAccelMps2);
}

void DriveBase::ResolveYawPointAcceleration(
    const CommandContext& context,
    float desiredYawRateRadps,
    float& desiredYawAccelRadps2) const noexcept
{
    _plantModel.ComputeBodyActionFromYawRate(
        context.presentLinearSpeedMps,
        context.presentYawRateRadps,
        desiredYawRateRadps,
        context.maxYawAccelRadps2,
        ResolveCommandResponseTimeS(),
        desiredYawAccelRadps2);
}

void DriveBase::ResolveVelocityPointAccelerations(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    float& desiredLongitudinalAccelMps2,
    float& desiredYawAccelRadps2) const noexcept
{
    _plantModel.ComputeBodyAction(
        context.presentLinearSpeedMps,
        desiredLinearSpeedMps,
        context.presentYawRateRadps,
        desiredYawRateRadps,
        context.maxLongitudinalAccelMps2,
        context.maxYawAccelRadps2,
        ResolveCommandResponseTimeS(),
        desiredLongitudinalAccelMps2,
        desiredYawAccelRadps2);
}

MazeMap::App::Internal::CommandVector DriveBase::ResolveRawAccelerationCommand(
    float presentLinearSpeedMps,
    float presentYawRateRadps,
    float desiredLongitudinalAccelMps2,
    float desiredYawAccelRadps2) const
{
    const float resolvedPresentLinearSpeedMps =
        std::isfinite(presentLinearSpeedMps) ?
        presentLinearSpeedMps :
        0.0f;
    const float resolvedPresentYawRateRadps =
        std::isfinite(presentYawRateRadps) ?
        presentYawRateRadps :
        0.0f;
    const float resolvedLongitudinalAccelMps2 =
        std::isfinite(desiredLongitudinalAccelMps2) ?
        desiredLongitudinalAccelMps2 :
        0.0f;
    const float resolvedYawAccelRadps2 =
        std::isfinite(desiredYawAccelRadps2) ?
        desiredYawAccelRadps2 :
        0.0f;

    const bool isSteadyHoldRequest =
        (std::fabs(resolvedLongitudinalAccelMps2) <= 1.0e-5f) &&
        (std::fabs(resolvedYawAccelRadps2) <= 1.0e-5f);
    const CommandVector rawCommand =
        isSteadyHoldRequest ?
        _plantModel.solveSteadyStateFeedforward(
            resolvedPresentLinearSpeedMps,
            resolvedPresentYawRateRadps) :
        _plantModel.solveAccelerationFeedforward(
            resolvedLongitudinalAccelMps2,
            resolvedYawAccelRadps2);
    return MakeClampedDriveControlVector(
        rawCommand.LeftMotorPwm(),
        rawCommand.RightMotorPwm());
}

MazeMap::App::Internal::CommandVector DriveBase::ResolveRawVelocityTargetCommand(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps) const
{
    const CommandVector rawCommand =
        _plantModel.solveSteadyStateFeedforward(
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return MakeClampedDriveControlVector(
        rawCommand.LeftMotorPwm(),
        rawCommand.RightMotorPwm());
}

MazeMap::App::Internal::CommandVector DriveBase::ResolveLongitudinalCorrectionCommand(
    const CommandContext& context,
    float desiredLongitudinalAccelCorrectionMps2) const
{
    const float resolvedCorrectionMps2 =
        ClampMagnitude(
            desiredLongitudinalAccelCorrectionMps2,
            context.maxLongitudinalAccelMps2);
    return
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            resolvedCorrectionMps2,
            0.0f) -
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f);
}

MazeMap::App::Internal::CommandVector DriveBase::ResolveYawCorrectionCommand(
    const CommandContext& context,
    float desiredYawAccelCorrectionRadps2) const
{
    const float resolvedCorrectionRadps2 =
        ClampMagnitude(
            desiredYawAccelCorrectionRadps2,
            context.maxYawAccelRadps2);
    return
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            resolvedCorrectionRadps2) -
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f);
}

MazeMap::App::Internal::CommandVector DriveBase::ComposeGeneratedCommand(
    const MazeMap::App::Internal::CommandVector& baseCommand,
    const CommandContext& context,
    const CommandTargets& targets,
    MazeMap::CommandPD pd) const
{
    CommandVector command = baseCommand;
    CommandTargets adjustedTargets = targets;
    const bool useVelocityTargetComposition =
        targets.hasVelocityTarget ||
        targets.hasYawRateTarget ||
        targets.hasHeadingTarget ||
        targets.hasWheelLinearTargets;
    const float responseTimeS = ResolveCommandResponseTimeS();
    const float maxVelocityTargetDeltaMps = responseTimeS * context.maxLongitudinalAccelMps2;
    const float maxYawRateTargetDeltaRadps = responseTimeS * context.maxYawAccelRadps2;
    bool targetCommandAdjusted = false;
    if (useVelocityTargetComposition)
    {
        adjustedTargets.hasVelocityTarget = true;
        adjustedTargets.velocityTargetMps =
            targets.hasVelocityTarget ?
            targets.velocityTargetMps :
            context.presentLinearSpeedMps;
        adjustedTargets.hasYawRateTarget = true;
        adjustedTargets.yawRateTargetRadps =
            targets.hasYawRateTarget ?
            targets.yawRateTargetRadps :
            context.presentYawRateRadps;
    }
    if (pd != MazeMap::CommandPD::RawCommand)
    {
        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateVelocityPD))
        {
            const float targetVelocityMps =
                targets.hasVelocityTarget ?
                targets.velocityTargetMps :
                context.presentLinearSpeedMps;
            const MazeMap::ProportionalDerivative& velocityPD =
                GetProportionalDerivativeCluster().GetVelocityPD(MazeMap::CommandPD::StateVelocityPD);
            const float desiredAccelCorrectionMps2 =
                velocityPD.ComputeFromMeasurementRate(
                    targetVelocityMps - context.presentLinearSpeedMps,
                    context.stateLongitudinalAccelMps2);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.velocityTargetMps +=
                    ClampMagnitude(responseTimeS * desiredAccelCorrectionMps2, maxVelocityTargetDeltaMps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveLongitudinalCorrectionCommand(
                        context,
                        desiredAccelCorrectionMps2);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateAccelerationPD))
        {
            const float targetAccelMps2 =
                targets.hasLongitudinalAccelTarget ?
                targets.longitudinalAccelTargetMps2 :
                context.stateLongitudinalAccelMps2;
            const MazeMap::ProportionalDerivative& accelerationPD =
                GetProportionalDerivativeCluster().GetLongitudinalAccelerationPD(
                    MazeMap::CommandPD::StateAccelerationPD);
            const float desiredAccelCorrectionMps2 =
                accelerationPD.Compute(
                    targetAccelMps2 - context.stateLongitudinalAccelMps2,
                    0.0f);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.velocityTargetMps +=
                    ClampMagnitude(responseTimeS * desiredAccelCorrectionMps2, maxVelocityTargetDeltaMps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveLongitudinalCorrectionCommand(
                        context,
                        desiredAccelCorrectionMps2);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUForwardAccel))
        {
            const float targetAccelMps2 =
                targets.hasLongitudinalAccelTarget ?
                targets.longitudinalAccelTargetMps2 :
                context.imuForwardAccelMps2;
            const MazeMap::ProportionalDerivative& accelerationPD =
                GetProportionalDerivativeCluster().GetLongitudinalAccelerationPD(
                    MazeMap::CommandPD::IMUForwardAccel);
            const float desiredAccelCorrectionMps2 =
                accelerationPD.Compute(
                    targetAccelMps2 - context.imuForwardAccelMps2,
                    0.0f);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.velocityTargetMps +=
                    ClampMagnitude(responseTimeS * desiredAccelCorrectionMps2, maxVelocityTargetDeltaMps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveLongitudinalCorrectionCommand(
                        context,
                        desiredAccelCorrectionMps2);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateHeadingPD))
        {
            const float targetYawRad =
                targets.hasHeadingTarget ?
                targets.headingTargetYawRad :
                context.presentYawRad;
            const float desiredYawRateCorrectionRadps =
                ResolveStraightHeadingYawRateCommand(
                    targetYawRad,
                    context.presentYawRad,
                    context.presentYawRateRadps);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.yawRateTargetRadps +=
                    ClampMagnitude(desiredYawRateCorrectionRadps, maxYawRateTargetDeltaRadps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveYawCorrectionCommand(
                        context,
                        desiredYawRateCorrectionRadps / responseTimeS);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD))
        {
            const float targetYawRateRadps =
                targets.hasYawRateTarget ?
                targets.yawRateTargetRadps :
                context.presentYawRateRadps;
            const MazeMap::ProportionalDerivative& yawRatePD =
                GetProportionalDerivativeCluster().GetYawRatePD(MazeMap::CommandPD::StateYawPD);
            const float desiredYawAccelCorrectionRadps2 =
                yawRatePD.ComputeFromMeasurementRate(
                    targetYawRateRadps - context.presentYawRateRadps,
                    context.presentDerivatives.yawAccelRadps2);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.yawRateTargetRadps +=
                    ClampMagnitude(responseTimeS * desiredYawAccelCorrectionRadps2, maxYawRateTargetDeltaRadps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveYawCorrectionCommand(
                        context,
                        desiredYawAccelCorrectionRadps2);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw))
        {
            const float targetYawRateRadps =
                targets.hasYawRateTarget ?
                targets.yawRateTargetRadps :
                context.imuYawRateRadps;
            const MazeMap::ProportionalDerivative& yawRatePD =
                GetProportionalDerivativeCluster().GetYawRatePD(MazeMap::CommandPD::IMUYaw);
            const float desiredYawAccelCorrectionRadps2 =
                yawRatePD.Compute(
                    targetYawRateRadps - context.imuYawRateRadps,
                    0.0f);
            if (useVelocityTargetComposition)
            {
                adjustedTargets.yawRateTargetRadps +=
                    ClampMagnitude(responseTimeS * desiredYawAccelCorrectionRadps2, maxYawRateTargetDeltaRadps);
                targetCommandAdjusted = true;
            }
            else
            {
                command +=
                    ResolveYawCorrectionCommand(
                        context,
                        desiredYawAccelCorrectionRadps2);
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMULateralAccel))
        {
            const float targetLateralAccelMps2 =
                targets.hasLateralAccelTarget ?
                targets.lateralAccelTargetMps2 :
                context.imuLateralAccelMps2;
            const float speedReferenceMps =
                (targets.hasVelocityTarget && std::isfinite(targets.velocityTargetMps) &&
                 (std::fabs(targets.velocityTargetMps) > 0.05f)) ?
                targets.velocityTargetMps :
                context.presentLinearSpeedMps;
            if (std::fabs(speedReferenceMps) > 0.05f)
            {
                const MazeMap::ProportionalDerivative& lateralAccelPD =
                    GetProportionalDerivativeCluster().GetYawRatePD(
                        MazeMap::CommandPD::IMULateralAccel);
                const float desiredYawRateCorrectionRadps =
                    lateralAccelPD.Compute(
                        (targetLateralAccelMps2 - context.imuLateralAccelMps2) /
                            speedReferenceMps,
                        0.0f);
                if (useVelocityTargetComposition)
                {
                    adjustedTargets.yawRateTargetRadps +=
                        ClampMagnitude(desiredYawRateCorrectionRadps, maxYawRateTargetDeltaRadps);
                    targetCommandAdjusted = true;
                }
                else
                {
                    command +=
                        ResolveYawCorrectionCommand(
                            context,
                            desiredYawRateCorrectionRadps / responseTimeS);
                }
            }
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity))
        {
            const MazeMap::ProportionalDerivative& wheelVelocityEncoderPD =
                GetProportionalDerivativeCluster()
                    .GetWheelVelocityPD(MazeMap::CommandPD::EncoderVelocity);
            const float leftTargetVelocityMps =
                adjustedTargets.hasWheelLinearTargets ?
                adjustedTargets.leftWheelLinearTargetMps :
                context.encoderLeftVelocityMps;
            const float rightTargetVelocityMps =
                adjustedTargets.hasWheelLinearTargets ?
                adjustedTargets.rightWheelLinearTargetMps :
                context.encoderRightVelocityMps;
            const float leftVelocityErrorMps =
                leftTargetVelocityMps - context.encoderLeftVelocityMps;
            const float rightVelocityErrorMps =
                rightTargetVelocityMps - context.encoderRightVelocityMps;
            if (useVelocityTargetComposition)
            {
                float forwardVelocityErrorMps = 0.0f;
                float yawRateErrorRadps = 0.0f;
                forwardVelocityErrorMps =
                    MazeMap::Vehicle::BodyForwardVelocityFromWheelLinear(leftVelocityErrorMps, rightVelocityErrorMps);
                yawRateErrorRadps =
                    MazeMap::Vehicle::BodyYawRateFromWheelLinear(leftVelocityErrorMps, rightVelocityErrorMps);
                adjustedTargets.velocityTargetMps +=
                    wheelVelocityEncoderPD.Compute(forwardVelocityErrorMps, 0.0f);
                adjustedTargets.yawRateTargetRadps +=
                    wheelVelocityEncoderPD.Compute(yawRateErrorRadps, 0.0f);
                targetCommandAdjusted = true;
            }
            else
            {
                command += CommandVector(
                    wheelVelocityEncoderPD.Compute(leftVelocityErrorMps, 0.0f),
                    wheelVelocityEncoderPD.Compute(rightVelocityErrorMps, 0.0f));
            }
        }

        if (targetCommandAdjusted && useVelocityTargetComposition)
        {
            ResolveWheelTargets(
                adjustedTargets.velocityTargetMps,
                adjustedTargets.yawRateTargetRadps,
                adjustedTargets);
            command =
                ResolveRawVelocityTargetCommand(
                    adjustedTargets.velocityTargetMps,
                    adjustedTargets.yawRateTargetRadps);
        }
    }

    const CommandVector feedbackCommand = command - baseCommand;
    const CommandVector clampedCommand =
        MakeClampedDriveControlVector(command.LeftMotorPwm(), command.RightMotorPwm());
    CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
    const CommandTargets& loggedTargets =
        useVelocityTargetComposition ?
        adjustedTargets :
        targets;
    _lastLinearCommandMps =
        loggedTargets.hasVelocityTarget ?
        loggedTargets.velocityTargetMps :
        context.presentLinearSpeedMps;
    _lastAngularCommandRadps =
        loggedTargets.hasYawRateTarget ?
        loggedTargets.yawRateTargetRadps :
        context.presentYawRateRadps;
    _lastLeftTargetVelocityMps =
        loggedTargets.hasWheelLinearTargets ? loggedTargets.leftWheelLinearTargetMps : 0.0f;
    _lastRightTargetVelocityMps =
        loggedTargets.hasWheelLinearTargets ? loggedTargets.rightWheelLinearTargetMps : 0.0f;
    _lastLeftLaunchAssistFloor = 0.0f;
    _lastRightLaunchAssistFloor = 0.0f;
    _lastModeFlags = kModeClosedLoop;
    _lastSaturationFlags =
        ((std::fabs(clampedCommand.LeftMotorPwm()) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(clampedCommand.RightMotorPwm()) >= 0.999f) ? 0x2u : 0u);
    return clampedCommand;
}

void DriveBase::CacheGeneratedCommandTelemetry(
    const MazeMap::App::Internal::CommandVector& feedforwardCommand,
    const MazeMap::App::Internal::CommandVector& feedbackCommand) const noexcept
{
    _lastFeedforwardCommand = feedforwardCommand;
    _lastFeedbackCommand = feedbackCommand;
}

