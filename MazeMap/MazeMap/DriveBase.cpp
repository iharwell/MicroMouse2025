#include "pch.h"
#include "DriveBase.h"

MazeMap::OpenLoopDriveCommand DriveBase::DeltaCommand(
    float presentLinearSpeedMps,
    float desiredLongitudinalAccelMps2,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasLongitudinalAccelTarget = true;
    targets.longitudinalAccelTargetMps2 = desiredLongitudinalAccelMps2;

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            0.0f,
            desiredLongitudinalAccelMps2,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::DeltaCommand(
    float presentLinearSpeedMps,
    float desiredLongitudinalAccelMps2,
    float presentYawRateRadps,
    float desiredYawAccelRadps2,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasLongitudinalAccelTarget = true;
    targets.longitudinalAccelTargetMps2 = desiredLongitudinalAccelMps2;
    targets.hasYawAccelTarget = true;
    targets.yawAccelTargetRadps2 = desiredYawAccelRadps2;

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            presentYawRateRadps,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::DeltaYawRateCommand(
    float presentYawRateRadps,
    float desiredYawAccelRadps2,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasYawAccelTarget = true;
    targets.yawAccelTargetRadps2 = desiredYawAccelRadps2;

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawAccelerationCommand(
            0.0f,
            presentYawRateRadps,
            0.0f,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::PointCommand(
    float desiredLinearSpeedMps,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasVelocityTarget = true;
    targets.velocityTargetMps = desiredLinearSpeedMps;
    ResolveWheelTargets(desiredLinearSpeedMps, 0.0f, targets);

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            desiredLinearSpeedMps,
            context.presentYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::PointCommand(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasVelocityTarget = true;
    targets.velocityTargetMps = desiredLinearSpeedMps;
    targets.hasYawRateTarget = true;
    targets.yawRateTargetRadps = desiredYawRateRadps;
    ResolveWheelTargets(desiredLinearSpeedMps, desiredYawRateRadps, targets);

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::PointYawRateCommand(
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted)
    {
        return {};
    }

    const CommandContext context = CaptureCommandContext();
    CommandTargets targets = BuildHoldTargets(context);
    targets.hasYawRateTarget = true;
    targets.yawRateTargetRadps = desiredYawRateRadps;
    ResolveWheelTargets(0.0f, desiredYawRateRadps, targets);

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            context.presentLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::OpenLoopDriveCommand DriveBase::FeedbackCommand(
    float setpoint,
    MazeMap::CommandPD pd) const
{
    if (_estimatorFaulted || (pd == MazeMap::CommandPD::RawCommand))
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
    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateWheelOmegaPD))
    {
        targets.hasWheelOmegaTargets = true;
        targets.leftWheelOmegaTargetRadps = setpoint;
        targets.rightWheelOmegaTargetRadps = setpoint;
    }
    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity))
    {
        targets.hasWheelLinearTargets = true;
        targets.leftWheelLinearTargetMps = setpoint;
        targets.rightWheelLinearTargetMps = setpoint;
    }

    const MazeMap::OpenLoopDriveCommand baseCommand =
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

void DriveBase::CommandGenerated(
    const MazeMap::OpenLoopDriveCommand& command,
    float linearSpeedMps,
    float angularSpeedRadps,
    bool applyLaunchAssist)
{
    if (_estimatorFaulted)
    {
        Brake();
        return;
    }

    const float leftMeasuredMps = _leftEncoderVelocityMps;
    const float rightMeasuredMps = _rightEncoderVelocityMps;
    float leftDriveCommand = command.leftDriveCommand;
    float rightDriveCommand = command.rightDriveCommand;
    const unsigned long nowMs = millis();
    if (applyLaunchAssist &&
        UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftDriveCommand, _leftMotor.getDriveCommand(), nowMs))
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
        UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightDriveCommand, _rightMotor.getDriveCommand(), nowMs))
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
    _lastLeftFeedforwardCommand = 0.0f;
    _lastRightFeedforwardCommand = 0.0f;
    _lastLeftFeedbackCommand = leftDriveCommand;
    _lastRightFeedbackCommand = rightDriveCommand;
    _lastLeftTargetVelocityMps = 0.0f;
    _lastRightTargetVelocityMps = 0.0f;
    _lastModeFlags = kModeClosedLoop |
        ((_lastLeftLaunchAssistFloor > 0.0f) ? kModeLaunchAssistLeft : 0u) |
        ((_lastRightLaunchAssistFloor > 0.0f) ? kModeLaunchAssistRight : 0u);
    _lastSaturationFlags =
        ((std::fabs(leftDriveCommand) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(rightDriveCommand) >= 0.999f) ? 0x2u : 0u);
    SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
}

void DriveBase::GetVelocityCommandOperatingState(
    MazeMap::VehicleState::StateVector& presentState,
    float& batteryVoltageV) const
{
    presentState = _ukf.ukf().state();
    const MeasuredKinematics measured = GetMeasuredKinematics();
    const MazeMap::PlantParams& params = _ukf.ukf().params();
    const float wheelRadiusM =
        (std::isfinite(params.wheelRadiusM) && (params.wheelRadiusM > 0.0f)) ?
        params.wheelRadiusM :
        0.0f;
    if (!std::isfinite(presentState(MazeMap::VehicleState::kPx)))
    {
        presentState(MazeMap::VehicleState::kPx) = 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kPy)))
    {
        presentState(MazeMap::VehicleState::kPy) = 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kPsi)))
    {
        presentState(MazeMap::VehicleState::kPsi) = 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kBgz)))
    {
        presentState(MazeMap::VehicleState::kBgz) = 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kU)))
    {
        presentState(MazeMap::VehicleState::kU) = measured.linearSpeedMps;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kV)))
    {
        presentState(MazeMap::VehicleState::kV) = 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kR)))
    {
        presentState(MazeMap::VehicleState::kR) = measured.angularSpeedRadps;
    }
    const float feedforwardYawRateRadps = _ukf.ukf().resolveYawRateForFeedforward(_lastGyroRawRadps);
    if (std::isfinite(feedforwardYawRateRadps))
    {
        presentState(MazeMap::VehicleState::kR) = feedforwardYawRateRadps;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kOmegaL)))
    {
        presentState(MazeMap::VehicleState::kOmegaL) =
            (wheelRadiusM > 0.0f) ? (measured.leftVelocityMps / wheelRadiusM) : 0.0f;
    }
    if (!std::isfinite(presentState(MazeMap::VehicleState::kOmegaR)))
    {
        presentState(MazeMap::VehicleState::kOmegaR) =
            (wheelRadiusM > 0.0f) ? (measured.rightVelocityMps / wheelRadiusM) : 0.0f;
    }
    MazeMap::VehicleState::NormalizeStateVector(presentState);
    batteryVoltageV = 0.5f * (_leftMotor.getVoltage() + _rightMotor.getVoltage());
}

float DriveBase::ResolveCommandResponseTimeS() noexcept
{
    return
        (std::isfinite(MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS) &&
         (MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS > 0.0f)) ?
        MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS :
        1.0f;
}

MazeMap::OpenLoopDriveCommand DriveBase::AddDriveCommands(
    const MazeMap::OpenLoopDriveCommand& lhs,
    const MazeMap::OpenLoopDriveCommand& rhs) noexcept
{
    return MazeMap::MakeOpenLoopDriveCommand(
        lhs.leftDriveCommand + rhs.leftDriveCommand,
        lhs.rightDriveCommand + rhs.rightDriveCommand);
}

MazeMap::OpenLoopDriveCommand DriveBase::SubtractDriveCommands(
    const MazeMap::OpenLoopDriveCommand& lhs,
    const MazeMap::OpenLoopDriveCommand& rhs) noexcept
{
    return MazeMap::MakeOpenLoopDriveCommand(
        lhs.leftDriveCommand - rhs.leftDriveCommand,
        lhs.rightDriveCommand - rhs.rightDriveCommand);
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
    if (context.wheelRadiusM > 0.0f)
    {
        targets.leftWheelOmegaTargetRadps =
            context.presentState(MazeMap::VehicleState::kOmegaL);
        targets.rightWheelOmegaTargetRadps =
            context.presentState(MazeMap::VehicleState::kOmegaR);
    }
    return targets;
}

DriveBase::CommandContext DriveBase::CaptureCommandContext() const
{
    CommandContext context{};
    GetVelocityCommandOperatingState(context.presentState, context.batteryVoltageV);
    context.presentYawRad =
        WrapAngleRad(context.presentState(MazeMap::VehicleState::kPsi));
    context.presentLinearSpeedMps =
        context.presentState(MazeMap::VehicleState::kU);
    context.presentYawRateRadps =
        context.presentState(MazeMap::VehicleState::kR);

    const MazeMap::PlantPreparedParams& prepared = _ukf.ukf().preparedParams();
    context.wheelRadiusM = ResolvePositiveOrZero(prepared.wheelRadiusM);
    context.encoderLeftVelocityMps = _leftEncoderVelocityMps;
    context.encoderRightVelocityMps = _rightEncoderVelocityMps;

    MazeMap::ControlInput control{};
    control.leftMotorCommand = _leftMotor.getDriveCommand();
    control.rightMotorCommand = _rightMotor.getDriveCommand();
    control.fanDutyCycle = GetMissionFanDutyCycle();
    control.batteryVoltageV = context.batteryVoltageV;

    MazeMap::PlantModel plantModel;
    context.presentDerivatives =
        plantModel.forwardStep(
            context.presentState,
            control,
            prepared);
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
        context.presentState,
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
    float leftTargetOmegaRadps = 0.0f;
    float rightTargetOmegaRadps = 0.0f;
    MazeMap::PlantModel plantModel;
    plantModel.resolveWheelMotionTargets(
        desiredLinearSpeedMps,
        desiredYawRateRadps,
        0.0f,
        0.0f,
        _ukf.ukf().preparedParams(),
        targets.leftWheelLinearTargetMps,
        targets.rightWheelLinearTargetMps,
        unusedLeftTargetAccelMps2,
        unusedRightTargetAccelMps2,
        leftTargetOmegaRadps,
        rightTargetOmegaRadps);
    targets.hasWheelLinearTargets = true;
    if (_ukf.ukf().preparedParams().wheelRadiusM > 0.0f)
    {
        targets.hasWheelOmegaTargets = true;
        targets.leftWheelOmegaTargetRadps = leftTargetOmegaRadps;
        targets.rightWheelOmegaTargetRadps = rightTargetOmegaRadps;
    }
}

void DriveBase::ResolveVelocityPointAcceleration(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float& desiredLongitudinalAccelMps2) const noexcept
{
    float unusedDesiredYawAccelRadps2 = 0.0f;
    MazeMap::PlantModel plantModel;
    plantModel.resolveVelocityTargetAccelerations(
        context.presentLinearSpeedMps,
        desiredLinearSpeedMps,
        context.presentYawRateRadps,
        context.presentYawRateRadps,
        context.maxLongitudinalAccelMps2,
        (std::numeric_limits<float>::max)(),
        ResolveCommandResponseTimeS(),
        desiredLongitudinalAccelMps2,
        unusedDesiredYawAccelRadps2);
}

void DriveBase::ResolveYawPointAcceleration(
    const CommandContext& context,
    float desiredYawRateRadps,
    float& desiredYawAccelRadps2) const noexcept
{
    float unusedDesiredLongitudinalAccelMps2 = 0.0f;
    MazeMap::PlantModel plantModel;
    plantModel.resolveVelocityTargetAccelerations(
        context.presentLinearSpeedMps,
        context.presentLinearSpeedMps,
        context.presentYawRateRadps,
        desiredYawRateRadps,
        (std::numeric_limits<float>::max)(),
        context.maxYawAccelRadps2,
        ResolveCommandResponseTimeS(),
        unusedDesiredLongitudinalAccelMps2,
        desiredYawAccelRadps2);
}

void DriveBase::ResolveVelocityPointAccelerations(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    float& desiredLongitudinalAccelMps2,
    float& desiredYawAccelRadps2) const noexcept
{
    MazeMap::PlantModel plantModel;
    plantModel.resolveVelocityTargetAccelerations(
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

MazeMap::OpenLoopDriveCommand DriveBase::ResolveRawAccelerationCommand(
    float presentLinearSpeedMps,
    float presentYawRateRadps,
    float desiredLongitudinalAccelMps2,
    float desiredYawAccelRadps2) const
{
    float batteryVoltageV = 0.0f;
    MazeMap::VehicleState::StateVector unusedPresentState = MazeMap::VehicleState::StateVector::Zero();
    GetVelocityCommandOperatingState(unusedPresentState, batteryVoltageV);

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

    MazeMap::PlantModel plantModel;
    const bool isSteadyHoldRequest =
        (std::fabs(resolvedLongitudinalAccelMps2) <= 1.0e-5f) &&
        (std::fabs(resolvedYawAccelRadps2) <= 1.0e-5f);
    const MazeMap::DriveCommandSolution solution =
        isSteadyHoldRequest ?
        plantModel.solveDriveCommandsForVelocityTarget(
            resolvedPresentLinearSpeedMps,
            resolvedPresentLinearSpeedMps,
            resolvedPresentYawRateRadps,
            resolvedPresentYawRateRadps,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            batteryVoltageV,
            ResolveCommandResponseTimeS()) :
        plantModel.solveDriveCommands(
            resolvedPresentLinearSpeedMps,
            resolvedLongitudinalAccelMps2,
            resolvedPresentYawRateRadps,
            resolvedYawAccelRadps2,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            batteryVoltageV);
    return MazeMap::ClampOpenLoopDriveCommand(
        MazeMap::MakeOpenLoopDriveCommand(
            solution.control.leftMotorCommand,
            solution.control.rightMotorCommand));
}

MazeMap::OpenLoopDriveCommand DriveBase::ResolveRawVelocityTargetCommand(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float desiredYawRateRadps) const
{
    MazeMap::PlantModel plantModel;
    const MazeMap::DriveCommandSolution solution =
        plantModel.solveDriveCommandsForVelocityTarget(
            context.presentState,
            desiredLinearSpeedMps,
            desiredYawRateRadps,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            context.batteryVoltageV,
            ResolveCommandResponseTimeS());
    return MazeMap::ClampOpenLoopDriveCommand(
        MazeMap::MakeOpenLoopDriveCommand(
            solution.control.leftMotorCommand,
            solution.control.rightMotorCommand));
}

MazeMap::OpenLoopDriveCommand DriveBase::ResolveLongitudinalCorrectionCommand(
    const CommandContext& context,
    float desiredLongitudinalAccelCorrectionMps2) const
{
    const float resolvedCorrectionMps2 =
        ClampMagnitude(
            desiredLongitudinalAccelCorrectionMps2,
            context.maxLongitudinalAccelMps2);
    return SubtractDriveCommands(
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            0.0f,
            resolvedCorrectionMps2,
            0.0f),
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            0.0f,
            0.0f,
            0.0f));
}

MazeMap::OpenLoopDriveCommand DriveBase::ResolveYawCorrectionCommand(
    const CommandContext& context,
    float desiredYawAccelCorrectionRadps2) const
{
    const float resolvedCorrectionRadps2 =
        ClampMagnitude(
            desiredYawAccelCorrectionRadps2,
            context.maxYawAccelRadps2);
    return SubtractDriveCommands(
        ResolveRawAccelerationCommand(
            0.0f,
            context.presentYawRateRadps,
            0.0f,
            resolvedCorrectionRadps2),
        ResolveRawAccelerationCommand(
            0.0f,
            context.presentYawRateRadps,
            0.0f,
            0.0f));
}

MazeMap::OpenLoopDriveCommand DriveBase::ComposeGeneratedCommand(
    const MazeMap::OpenLoopDriveCommand& baseCommand,
    const CommandContext& context,
    const CommandTargets& targets,
    MazeMap::CommandPD pd) const
{
    if (pd == MazeMap::CommandPD::RawCommand)
    {
        return MazeMap::ClampOpenLoopDriveCommand(baseCommand);
    }

    MazeMap::OpenLoopDriveCommand command = baseCommand;

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateVelocityPD))
    {
        const float targetVelocityMps =
            targets.hasVelocityTarget ?
            targets.velocityTargetMps :
            context.presentLinearSpeedMps;
        const float desiredAccelCorrectionMps2 =
            (targetVelocityMps - context.presentLinearSpeedMps) /
            ResolveCommandResponseTimeS();
        command = AddDriveCommands(
            command,
            ResolveLongitudinalCorrectionCommand(
                context,
                desiredAccelCorrectionMps2));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateAccelerationPD))
    {
        const float targetAccelMps2 =
            targets.hasLongitudinalAccelTarget ?
            targets.longitudinalAccelTargetMps2 :
            context.stateLongitudinalAccelMps2;
        command = AddDriveCommands(
            command,
            ResolveLongitudinalCorrectionCommand(
                context,
                targetAccelMps2 - context.stateLongitudinalAccelMps2));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUForwardAccel))
    {
        const float targetAccelMps2 =
            targets.hasLongitudinalAccelTarget ?
            targets.longitudinalAccelTargetMps2 :
            context.imuForwardAccelMps2;
        command = AddDriveCommands(
            command,
            ResolveLongitudinalCorrectionCommand(
                context,
                targetAccelMps2 - context.imuForwardAccelMps2));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateHeadingPD))
    {
        const float targetYawRad =
            targets.hasHeadingTarget ?
            targets.headingTargetYawRad :
            context.presentYawRad;
        const float headingErrorRad =
            HeadingErrorRad(
                HeadingUnitFromYawRad(targetYawRad),
                HeadingUnitFromYawRad(context.presentYawRad));
        const float desiredYawRateCorrectionRadps =
            Config::kStraightHeadingKp * headingErrorRad;
        command = AddDriveCommands(
            command,
            ResolveYawCorrectionCommand(
                context,
                desiredYawRateCorrectionRadps / ResolveCommandResponseTimeS()));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD))
    {
        const float targetYawRateRadps =
            targets.hasYawRateTarget ?
            targets.yawRateTargetRadps :
            context.presentYawRateRadps;
        command = AddDriveCommands(
            command,
            ResolveYawCorrectionCommand(
                context,
                (targetYawRateRadps - context.presentYawRateRadps) /
                ResolveCommandResponseTimeS()));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw))
    {
        const float targetYawRateRadps =
            targets.hasYawRateTarget ?
            targets.yawRateTargetRadps :
            context.imuYawRateRadps;
        command = AddDriveCommands(
            command,
            ResolveYawCorrectionCommand(
                context,
                (targetYawRateRadps - context.imuYawRateRadps) /
                ResolveCommandResponseTimeS()));
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
            const float desiredYawRateCorrectionRadps =
                (targetLateralAccelMps2 - context.imuLateralAccelMps2) /
                speedReferenceMps;
            command = AddDriveCommands(
                command,
                ResolveYawCorrectionCommand(
                    context,
                    desiredYawRateCorrectionRadps / ResolveCommandResponseTimeS()));
        }
    }

    const float wheelRadiusM = context.wheelRadiusM;
    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateWheelOmegaPD) &&
        (wheelRadiusM > 0.0f))
    {
        const float leftTargetOmegaRadps =
            targets.hasWheelOmegaTargets ?
            targets.leftWheelOmegaTargetRadps :
            context.presentState(MazeMap::VehicleState::kOmegaL);
        const float rightTargetOmegaRadps =
            targets.hasWheelOmegaTargets ?
            targets.rightWheelOmegaTargetRadps :
            context.presentState(MazeMap::VehicleState::kOmegaR);
        command.leftDriveCommand +=
            GetWheelVelocityKp() *
            (wheelRadiusM * (leftTargetOmegaRadps - context.presentState(MazeMap::VehicleState::kOmegaL)));
        command.rightDriveCommand +=
            GetWheelVelocityKp() *
            (wheelRadiusM * (rightTargetOmegaRadps - context.presentState(MazeMap::VehicleState::kOmegaR)));
    }

    if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity))
    {
        const float leftTargetVelocityMps =
            targets.hasWheelLinearTargets ?
            targets.leftWheelLinearTargetMps :
            context.encoderLeftVelocityMps;
        const float rightTargetVelocityMps =
            targets.hasWheelLinearTargets ?
            targets.rightWheelLinearTargetMps :
            context.encoderRightVelocityMps;
        command.leftDriveCommand +=
            GetWheelVelocityKp() *
            (leftTargetVelocityMps - context.encoderLeftVelocityMps);
        command.rightDriveCommand +=
            GetWheelVelocityKp() *
            (rightTargetVelocityMps - context.encoderRightVelocityMps);
    }

    return MazeMap::ClampOpenLoopDriveCommand(command);
}

float DriveBase::GetWheelVelocityKp() const
{
    return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, _wheelControlProfile.velocityKpScale);
}

float DriveBase::GetWheelIntegralLimit() const
{
    return MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, _wheelControlProfile.integralLimitScale);
}
