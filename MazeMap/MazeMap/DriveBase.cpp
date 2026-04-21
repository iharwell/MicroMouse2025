#include "pch.h"
#include "DriveBase.h"

#include "PlantModel.h"

namespace
{
    using ControlVector = MazeMap::App::Internal::LoopController::ControlVector;

    ControlVector MakeClampedDriveControlVector(float leftMotorPwm, float rightMotorPwm) noexcept
    {
        if (!std::isfinite(leftMotorPwm) || !std::isfinite(rightMotorPwm))
        {
            return {};
        }

        return ControlVector::RawMotorPwm(
            MazeMap::ClampWheelDriveCommand(leftMotorPwm),
            MazeMap::ClampWheelDriveCommand(rightMotorPwm));
    }
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::DeltaCommand(
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

    const ControlVector baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            0.0f,
            desiredLongitudinalAccelMps2,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::DeltaCommand(
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

    const ControlVector baseCommand =
        ResolveRawAccelerationCommand(
            presentLinearSpeedMps,
            presentYawRateRadps,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::DeltaYawRateCommand(
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

    const ControlVector baseCommand =
        ResolveRawAccelerationCommand(
            0.0f,
            presentYawRateRadps,
            0.0f,
            desiredYawAccelRadps2);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointCommand(
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

    const ControlVector baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            desiredLinearSpeedMps,
            context.presentYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointCommand(
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

    const ControlVector baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointCommandWithHeadingTarget(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    float targetYawRad,
    MazeMap::CommandPD pointPd,
    MazeMap::CommandPD headingPd) const
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
    if (MazeMap::HasCommandPD(headingPd, MazeMap::CommandPD::StateHeadingPD))
    {
        targets.hasHeadingTarget = true;
        targets.headingTargetYawRad = targetYawRad;
    }
    ResolveWheelTargets(desiredLinearSpeedMps, desiredYawRateRadps, targets);

    const ControlVector baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            desiredLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pointPd | headingPd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointControlVector(
    float desiredLinearSpeedMps,
    float desiredYawRateRadps,
    MazeMap::CommandPD pd) const
{
    return PointCommand(
        desiredLinearSpeedMps,
        desiredYawRateRadps,
        pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointControlVectorWithHeadingTarget(
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

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointCommand(
    const MazeMap::ManeuverPoint& point,
    MazeMap::CommandPD pd) const
{
    if (!point.IsFinite())
    {
        return {};
    }

    return PointCommand(point.Velocity, point.Omega, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointControlVector(
    const MazeMap::ManeuverPoint& point,
    MazeMap::CommandPD pd) const
{
    return PointCommand(point, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::PointYawRateCommand(
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

    const ControlVector baseCommand =
        ResolveRawVelocityTargetCommand(
            context,
            context.presentLinearSpeedMps,
            desiredYawRateRadps);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::FeedbackCommand(
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

    const ControlVector baseCommand =
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f);
    return ComposeGeneratedCommand(baseCommand, context, targets, pd);
}

void DriveBase::CommandGenerated(
    const MazeMap::App::Internal::LoopController::ControlVector& command,
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
    float leftDriveCommand = command.leftMotorPwm;
    float rightDriveCommand = command.rightMotorPwm;
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

void DriveBase::CommandOpenLoopRaw(
    const MazeMap::App::Internal::LoopController::ControlVector& command)
{
    if (_estimatorFaulted)
    {
        Brake();
        return;
    }

    _lastLinearCommandMps = 0.0f;
    _lastAngularCommandRadps = 0.0f;
    ResetLaunchAssist();
    _lastLeftFeedforwardCommand = 0.0f;
    _lastRightFeedforwardCommand = 0.0f;
    _lastLeftFeedbackCommand = command.leftMotorPwm;
    _lastRightFeedbackCommand = command.rightMotorPwm;
    _lastLeftTargetVelocityMps = 0.0f;
    _lastRightTargetVelocityMps = 0.0f;
    _lastLeftLaunchAssistFloor = 0.0f;
    _lastRightLaunchAssistFloor = 0.0f;
    _lastModeFlags = kModeRawOpenLoop;
    _lastSaturationFlags =
        ((std::fabs(command.leftMotorPwm) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(command.rightMotorPwm) >= 0.999f) ? 0x2u : 0u);
    SetOpenLoopRaw(command.leftMotorPwm, command.rightMotorPwm);
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

MazeMap::App::Internal::LoopController::ControlVector DriveBase::AddDriveCommands(
    const MazeMap::App::Internal::LoopController::ControlVector& lhs,
    const MazeMap::App::Internal::LoopController::ControlVector& rhs) noexcept
{
    return ControlVector::RawMotorPwm(
        lhs.leftMotorPwm + rhs.leftMotorPwm,
        lhs.rightMotorPwm + rhs.rightMotorPwm);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::SubtractDriveCommands(
    const MazeMap::App::Internal::LoopController::ControlVector& lhs,
    const MazeMap::App::Internal::LoopController::ControlVector& rhs) noexcept
{
    return ControlVector::RawMotorPwm(
        lhs.leftMotorPwm - rhs.leftMotorPwm,
        lhs.rightMotorPwm - rhs.rightMotorPwm);
}

float DriveBase::AverageDriveCommand(
    const MazeMap::App::Internal::LoopController::ControlVector& command) noexcept
{
    return 0.5f * (command.leftMotorPwm + command.rightMotorPwm);
}

float DriveBase::DeltaDriveCommand(
    const MazeMap::App::Internal::LoopController::ControlVector& command) noexcept
{
    return 0.5f * (command.leftMotorPwm - command.rightMotorPwm);
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

    context.presentDerivatives =
        _plantModel.forwardStep(
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
    _plantModel.resolveWheelMotionTargets(
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

MazeMap::App::Internal::LoopController::ControlVector DriveBase::ResolveRawAccelerationCommand(
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

    const bool isSteadyHoldRequest =
        (std::fabs(resolvedLongitudinalAccelMps2) <= 1.0e-5f) &&
        (std::fabs(resolvedYawAccelRadps2) <= 1.0e-5f);
    const MazeMap::DriveCommandSolution solution =
        isSteadyHoldRequest ?
        _plantModel.solveDriveCommandsForVelocityTarget(
            resolvedPresentLinearSpeedMps,
            resolvedPresentLinearSpeedMps,
            resolvedPresentYawRateRadps,
            resolvedPresentYawRateRadps,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            batteryVoltageV,
            ResolveCommandResponseTimeS()) :
        _plantModel.solveDriveCommands(
            resolvedPresentLinearSpeedMps,
            resolvedLongitudinalAccelMps2,
            resolvedPresentYawRateRadps,
            resolvedYawAccelRadps2,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            batteryVoltageV);
    return MakeClampedDriveControlVector(
        solution.control.leftMotorCommand,
        solution.control.rightMotorCommand);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::ResolveRawVelocityTargetCommand(
    const CommandContext& context,
    float desiredLinearSpeedMps,
    float desiredYawRateRadps) const
{
    const MazeMap::DriveCommandSolution solution =
        _plantModel.solveDriveCommandsForVelocityTarget(
            context.presentState,
            desiredLinearSpeedMps,
            desiredYawRateRadps,
            _ukf.ukf().preparedParams(),
            GetMissionFanDutyCycle(),
            context.batteryVoltageV,
            ResolveCommandResponseTimeS());
    return MakeClampedDriveControlVector(
        solution.control.leftMotorCommand,
        solution.control.rightMotorCommand);
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::ResolveLongitudinalCorrectionCommand(
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
            context.presentYawRateRadps,
            resolvedCorrectionMps2,
            0.0f),
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f));
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::ResolveYawCorrectionCommand(
    const CommandContext& context,
    float desiredYawAccelCorrectionRadps2) const
{
    const float resolvedCorrectionRadps2 =
        ClampMagnitude(
            desiredYawAccelCorrectionRadps2,
            context.maxYawAccelRadps2);
    return SubtractDriveCommands(
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            resolvedCorrectionRadps2),
        ResolveRawAccelerationCommand(
            context.presentLinearSpeedMps,
            context.presentYawRateRadps,
            0.0f,
            0.0f));
}

MazeMap::App::Internal::LoopController::ControlVector DriveBase::ComposeGeneratedCommand(
    const MazeMap::App::Internal::LoopController::ControlVector& baseCommand,
    const CommandContext& context,
    const CommandTargets& targets,
    MazeMap::CommandPD pd) const
{
    ControlVector command = baseCommand;
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
            const MazeMap::ProportionalDerivative& accelerationPD =
                GetProportionalDerivativeCluster().GetLongitudinalAccelerationPD(
                    MazeMap::CommandPD::StateAccelerationPD);
            command = AddDriveCommands(
                command,
                ResolveLongitudinalCorrectionCommand(
                    context,
                    accelerationPD.Compute(
                        targetAccelMps2 - context.stateLongitudinalAccelMps2,
                        0.0f)));
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
            command = AddDriveCommands(
                command,
                ResolveYawCorrectionCommand(
                    context,
                    ResolveStraightHeadingYawRateCommand(
                        targetYawRad,
                        context.presentYawRad,
                        context.presentYawRateRadps) /
                    ResolveCommandResponseTimeS()));
        }

        if (MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD))
        {
            const float targetYawRateRadps =
                targets.hasYawRateTarget ?
                targets.yawRateTargetRadps :
                context.presentYawRateRadps;
            const MazeMap::ProportionalDerivative& yawRatePD =
                GetProportionalDerivativeCluster().GetYawRatePD(MazeMap::CommandPD::StateYawPD);
            command = AddDriveCommands(
                command,
                ResolveYawCorrectionCommand(
                    context,
                    yawRatePD.ComputeFromMeasurementRate(
                        targetYawRateRadps - context.presentYawRateRadps,
                        context.presentDerivatives.yawAccelRadps2)));
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
            command.leftMotorPwm +=
                GetWheelVelocityKp() *
                (wheelRadiusM * (leftTargetOmegaRadps - context.presentState(MazeMap::VehicleState::kOmegaL)));
            command.rightMotorPwm +=
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
            command.leftMotorPwm +=
                GetWheelVelocityKp() *
                (leftTargetVelocityMps - context.encoderLeftVelocityMps);
            command.rightMotorPwm +=
                GetWheelVelocityKp() *
                (rightTargetVelocityMps - context.encoderRightVelocityMps);
        }
    }

    const ControlVector feedbackCommand = SubtractDriveCommands(command, baseCommand);
    const ControlVector clampedCommand =
        MakeClampedDriveControlVector(command.leftMotorPwm, command.rightMotorPwm);
    CacheGeneratedCommandTelemetry(baseCommand, feedbackCommand);
    _lastLinearCommandMps =
        targets.hasVelocityTarget ?
        targets.velocityTargetMps :
        context.presentLinearSpeedMps;
    _lastAngularCommandRadps =
        targets.hasYawRateTarget ?
        targets.yawRateTargetRadps :
        context.presentYawRateRadps;
    _lastLeftTargetVelocityMps =
        targets.hasWheelLinearTargets ? targets.leftWheelLinearTargetMps : 0.0f;
    _lastRightTargetVelocityMps =
        targets.hasWheelLinearTargets ? targets.rightWheelLinearTargetMps : 0.0f;
    _lastLeftLaunchAssistFloor = 0.0f;
    _lastRightLaunchAssistFloor = 0.0f;
    _lastModeFlags = kModeClosedLoop;
    _lastSaturationFlags =
        ((std::fabs(clampedCommand.leftMotorPwm) >= 0.999f) ? 0x1u : 0u) |
        ((std::fabs(clampedCommand.rightMotorPwm) >= 0.999f) ? 0x2u : 0u);
    return clampedCommand;
}

void DriveBase::CacheGeneratedCommandTelemetry(
    const MazeMap::App::Internal::LoopController::ControlVector& feedforwardCommand,
    const MazeMap::App::Internal::LoopController::ControlVector& feedbackCommand) const noexcept
{
    _lastFeedforwardCommandAverage = AverageDriveCommand(feedforwardCommand);
    _lastFeedforwardCommandDelta = DeltaDriveCommand(feedforwardCommand);
    _lastFeedbackCommandAverage = AverageDriveCommand(feedbackCommand);
    _lastFeedbackCommandDelta = DeltaDriveCommand(feedbackCommand);
    _lastLeftFeedforwardCommand = feedforwardCommand.leftMotorPwm;
    _lastRightFeedforwardCommand = feedforwardCommand.rightMotorPwm;
    _lastLeftFeedbackCommand = feedbackCommand.leftMotorPwm;
    _lastRightFeedbackCommand = feedbackCommand.rightMotorPwm;
}

float DriveBase::GetWheelVelocityKp() const
{
    return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, _wheelControlProfile.velocityKpScale);
}

float DriveBase::GetWheelIntegralLimit() const
{
    return MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, _wheelControlProfile.integralLimitScale);
}
