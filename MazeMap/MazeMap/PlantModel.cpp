#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>

namespace
{
    Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yaw, s, c);
        return Eigen::Vector2f(s, c);
    }

    Eigen::Vector2f RightUnitFromHeading(const Eigen::Vector2f& heading) noexcept
    {
        return Eigen::Vector2f(heading.y(), -heading.x());
    }
}

namespace MazeMap
{
    PlantParams PlantParams::Default() noexcept
    {
        PlantParams params{};
        const VehiclePhysicalModel& physicalModel = Vehicle::GetPhysicalModel();
        const MotorEncoderDrivePhysicalModel& driveModel = MotorEncoderDrive::GetSharedPhysicalModel();
        params.massKg = physicalModel.massKg;
        params.effectiveLongitudinalMassKg = physicalModel.massKg;
        params.yawInertiaKgM2 = physicalModel.yawInertiaKgM2;
        params.trackWidthM = physicalModel.trackWidthM;
        params.wheelRadiusM = 0.5f * driveModel.wheelDiameterM;
        params.supplyVoltageV = driveModel.supplyVoltageV;
        params.driveResistanceOhms = driveModel.resistanceOhms;
        params.torqueConstantNmPerA = driveModel.torqueConstantNmPerA;
        params.speedConstantRadpsPerVolt = driveModel.speedConstantRadpsPerVolt;
        params.noLoadCurrentA = driveModel.noLoadCurrentA;
        params.gearRatio = driveModel.gearRatio;
        params.encoderCountsPerMotorRev = driveModel.pulsesPerRev;
        params.motorCurrentLimitA =
            (params.driveResistanceOhms > 0.0f) ?
            (params.supplyVoltageV / params.driveResistanceOhms) :
            0.0f;

        params.frontLeftSensor = Vehicle::GetFrontLeftSensorExtrinsics();
        params.frontRightSensor = Vehicle::GetFrontRightSensorExtrinsics();
        params.sideLeftSensor = Vehicle::GetSideLeftSensorExtrinsics();
        params.sideRightSensor = Vehicle::GetSideRightSensorExtrinsics();
        params.imu = Vehicle::GetBackLeftImuExtrinsics();

        const float frontContactY = std::fabs(params.contactPatchLongitudinalOffsetM);
        const float halfTrackWidthM = 0.5f * params.trackWidthM;
        params.contactPositionsBodyM[0] = Eigen::Vector2f(-halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[1] = Eigen::Vector2f(halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[2] = Eigen::Vector2f(-halfTrackWidthM, -frontContactY);
        params.contactPositionsBodyM[3] = Eigen::Vector2f(halfTrackWidthM, -frontContactY);
        return params;
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        PlantDerivatives derivatives{};
        const WheelKinematics kinematics = wheelKinematics(state, params);
        const SlipTargets targets = slipTargets(state, kinematics, params);
        const ContactForces forces = tireForces(kinematics, targets, control, params);

        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeft = state(VehicleState::kOmegaL);
        const float omegaRight = state(VehicleState::kOmegaR);
        const float batteryVoltageV =
            (std::isfinite(control.batteryVoltageV) && (control.batteryVoltageV > 0.0f)) ?
            control.batteryVoltageV :
            params.supplyVoltageV;

        float yawMomentNm = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < params.contactPositionsBodyM.size(); ++contactIndex)
        {
            const Eigen::Vector2f contactPosition = params.ContactPosition(contactIndex);
            yawMomentNm +=
                (contactPosition.y() * forces.contacts[contactIndex].rightForceN) -
                (contactPosition.x() * forces.contacts[contactIndex].forwardForceN);
        }

        const float tauMotorLeft = driveTorqueFromCommand(control.leftMotorCommand, omegaLeft, batteryVoltageV, params);
        const float tauMotorRight = driveTorqueFromCommand(control.rightMotorCommand, omegaRight, batteryVoltageV, params);
        const float tauFrictionLeft = driveFrictionTorque(omegaLeft, params);
        const float tauFrictionRight = driveFrictionTorque(omegaRight, params);
        const Eigen::Vector2f heading = HeadingUnitFromYaw(psi);
        const Eigen::Vector2f right = RightUnitFromHeading(heading);

        derivatives.stateDot(VehicleState::kPx) =
            (rightVelocityMps * right.x()) + (forwardVelocityMps * heading.x());
        derivatives.stateDot(VehicleState::kPy) =
            (rightVelocityMps * right.y()) + (forwardVelocityMps * heading.y());
        derivatives.stateDot(VehicleState::kPsi) = yawRateRadps;
        derivatives.stateDot(VehicleState::kU) =
            (yawRateRadps * rightVelocityMps) +
            (forces.SumForwardForceN() / params.effectiveLongitudinalMassKg);
        derivatives.stateDot(VehicleState::kV) =
            (-yawRateRadps * forwardVelocityMps) +
            (forces.SumRightForceN() / params.massKg);
        derivatives.stateDot(VehicleState::kR) = yawMomentNm / params.yawInertiaKgM2;
        derivatives.stateDot(VehicleState::kOmegaL) =
            (tauMotorLeft - (params.wheelRadiusM * forces.LeftBankForwardForceN()) - tauFrictionLeft) /
            params.equivalentWheelInertiaKgM2;
        derivatives.stateDot(VehicleState::kOmegaR) =
            (tauMotorRight - (params.wheelRadiusM * forces.RightBankForwardForceN()) - tauFrictionRight) /
            params.equivalentWheelInertiaKgM2;
        derivatives.stateDot(VehicleState::kBgz) = 0.0f;

        derivatives.contactForces = forces;
        derivatives.wheelKinematics = kinematics;
        derivatives.slipTargets = targets;
        derivatives.originAccelBodyMps2 = Eigen::Vector2f(
            derivatives.stateDot(VehicleState::kV) + (yawRateRadps * forwardVelocityMps),
            derivatives.stateDot(VehicleState::kU) - (yawRateRadps * rightVelocityMps));
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            derivatives.originAccelBodyMps2.x() -
                ((yawRateRadps * yawRateRadps) * params.imu.positionBodyM.x()) +
                (derivatives.stateDot(VehicleState::kR) * params.imu.positionBodyM.y()),
            derivatives.originAccelBodyMps2.y() +
                ((yawRateRadps * yawRateRadps) * -params.imu.positionBodyM.y()) -
                (derivatives.stateDot(VehicleState::kR) * params.imu.positionBodyM.x()));
        derivatives.longitudinalAccelMps2 = derivatives.originAccelBodyMps2.y();
        derivatives.lateralAccelMps2 = derivatives.originAccelBodyMps2.x();
        derivatives.yawAccelRadps2 = derivatives.stateDot(VehicleState::kR);
        return derivatives;
    }

    WheelKinematics PlantModel::wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept
    {
        WheelKinematics kinematics{};
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);

        for (uint8_t contactIndex = 0U; contactIndex < params.contactPositionsBodyM.size(); ++contactIndex)
        {
            const Eigen::Vector2f position = params.ContactPosition(contactIndex);
            ContactKinematics& contact = kinematics.contacts[contactIndex];
            contact.rightVelocityMps = rightVelocityMps + (yawRateRadps * position.y());
            contact.forwardVelocityMps = forwardVelocityMps - (yawRateRadps * position.x());
        }

        kinematics.leftBankForwardVelocityMps = kinematics.contacts[0].forwardVelocityMps;
        kinematics.rightBankForwardVelocityMps = kinematics.contacts[1].forwardVelocityMps;
        return kinematics;
    }

    SlipTargets PlantModel::slipTargets(const StateVector& state, const PlantParams& params) const noexcept
    {
        return slipTargets(state, wheelKinematics(state, params), params);
    }

    SlipTargets PlantModel::slipTargets(
        const StateVector& state,
        const WheelKinematics& kinematics,
        const PlantParams& params) const noexcept
    {
        SlipTargets targets{};
        const float velocityEpsilonMps = (std::max)(params.velocityEpsilonMps, 1.0e-3f);
        const float wheelCircumferentialLeft = params.wheelRadiusM * state(VehicleState::kOmegaL);
        const float wheelCircumferentialRight = params.wheelRadiusM * state(VehicleState::kOmegaR);

        targets.kappaLeft =
            (wheelCircumferentialLeft - kinematics.leftBankForwardVelocityMps) /
            (std::max)(std::fabs(kinematics.leftBankForwardVelocityMps), velocityEpsilonMps);
        targets.kappaRight =
            (wheelCircumferentialRight - kinematics.rightBankForwardVelocityMps) /
            (std::max)(std::fabs(kinematics.rightBankForwardVelocityMps), velocityEpsilonMps);

        for (uint8_t contactIndex = 0U; contactIndex < kinematics.contacts.size(); ++contactIndex)
        {
            const ContactKinematics& contact = kinematics.contacts[contactIndex];
            targets.lateralRatio[contactIndex] =
                contact.rightVelocityMps /
                (std::max)(std::fabs(contact.forwardVelocityMps), velocityEpsilonMps);
        }
        return targets;
    }

    ContactForces PlantModel::tireForces(const StateVector& state, const PlantParams& params) const noexcept
    {
        return tireForces(state, ControlInput{}, params);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        const WheelKinematics kinematics = wheelKinematics(state, params);
        const SlipTargets targets = slipTargets(state, kinematics, params);
        return tireForces(kinematics, targets, control, params);
    }

    ContactForces PlantModel::tireForces(
        const WheelKinematics& kinematics,
        const SlipTargets& targets,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        ContactForces forces{};
        const float fanDutyCycle = (std::clamp)(control.fanDutyCycle, 0.0f, 1.0f);
        const float frontWheelLoadN = params.FrontWheelLoadN(fanDutyCycle);
        const float rearWheelLoadN = params.RearWheelLoadN(fanDutyCycle);

        for (uint8_t contactIndex = 0U; contactIndex < forces.contacts.size(); ++contactIndex)
        {
            ContactForce& force = forces.contacts[contactIndex];
            const bool isFront = contactIndex < 2U;
            const bool isLeft = (contactIndex == 0U) || (contactIndex == 2U);
            const float kappa = isLeft ? targets.kappaLeft : targets.kappaRight;
            const float lateralRatio = targets.lateralRatio[contactIndex];
            const float mu = isFront ? params.muFront : params.muRear;
            const float corneringStiffness =
                isFront ? params.corneringStiffnessFrontNPerRad : params.corneringStiffnessRearNPerRad;
            const float normalLoadN = isFront ? frontWheelLoadN : rearWheelLoadN;
            const float forwardForceN = 0.5f * params.longitudinalStiffnessN * kappa;
            const float rightForceN = -corneringStiffness * lateralRatio;
            const float requestedForceMagnitudeN =
                MazeMap::Math::Sqrtf((forwardForceN * forwardForceN) + (rightForceN * rightForceN));
            const float lambda =
                (mu * normalLoadN) /
                ((2.0f * requestedForceMagnitudeN) + (std::max)(params.forceEpsilonN, 1.0e-5f));
            const float saturation =
                (lambda >= 1.0f) ? 1.0f :
                (lambda <= 0.0f) ? 0.0f :
                (lambda * (2.0f - lambda));
            force.forwardForceN = saturation * forwardForceN;
            force.rightForceN = saturation * rightForceN;
            force.normalForceN = normalLoadN;
            force.saturation = saturation;
        }
        return forces;
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        return forwardStep(state, control, params).imuAccelBodyMps2;
    }

    PlantModel::StateVector PlantModel::integrateMidpoint(
        const StateVector& state,
        const ControlInput& control,
        float dt,
        const PlantParams& params) const noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return state;
        }

        const PlantDerivatives first = forwardStep(state, control, params);
        StateVector integratedState = state + (dt * first.stateDot);
        integratedState(VehicleState::kPsi) = VehicleState::NormalizeAngle(integratedState(VehicleState::kPsi));
        return integratedState;
    }

    float PlantModel::driveTorqueFromCommand(
        float motorCommand,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const float command = (std::clamp)(motorCommand, -1.0f, 1.0f);
        const float motorSpeedRadps = wheelBankSpeedRadps * params.gearRatio;
        const float appliedVoltageV =
            command *
            ((std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ? batteryVoltageV : params.supplyVoltageV);
        const float backEmfVoltageV =
            (params.speedConstantRadpsPerVolt > 0.0f) ?
            (motorSpeedRadps / params.speedConstantRadpsPerVolt) :
            0.0f;
        float currentA =
            (params.driveResistanceOhms > 0.0f) ?
            ((appliedVoltageV - backEmfVoltageV) / params.driveResistanceOhms) :
            0.0f;
        if (params.motorCurrentLimitA > 0.0f)
        {
            currentA = (std::clamp)(currentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float motorTorqueNm = params.torqueConstantNmPerA * currentA;
        return motorTorqueNm * params.gearRatio * params.drivetrainEfficiency;
    }

    float PlantModel::driveFrictionTorque(float wheelBankSpeedRadps, const PlantParams& params) const noexcept
    {
        const float sign =
            (wheelBankSpeedRadps > 0.0f) ? 1.0f :
            (wheelBankSpeedRadps < 0.0f) ? -1.0f :
            0.0f;
        return (params.rollingFrictionTorqueNm * sign) +
            (params.viscousFrictionNmPerRadps * wheelBankSpeedRadps);
    }
}
