#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    struct MotionEvaluation
    {
        MazeMap::PlantModel::StateVector state = MazeMap::PlantModel::StateVector::Zero();
        MazeMap::PlantDerivatives derivatives{};
    };

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

    float EffectiveBatteryVoltage(float batteryVoltageV, const MazeMap::PlantParams& params) noexcept
    {
        return
            (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
            batteryVoltageV :
            params.supplyVoltageV;
    }

    float SignedDirection(float preferredValue, float fallbackValue) noexcept
    {
        if (preferredValue > 1.0e-6f)
        {
            return 1.0f;
        }
        if (preferredValue < -1.0e-6f)
        {
            return -1.0f;
        }
        if (fallbackValue > 1.0e-6f)
        {
            return 1.0f;
        }
        if (fallbackValue < -1.0e-6f)
        {
            return -1.0f;
        }
        return 0.0f;
    }

    float LeftBankForwardVelocityMps(
        float forwardVelocityMps,
        float yawRateRadps,
        const MazeMap::PlantParams& params) noexcept
    {
        return forwardVelocityMps + (0.5f * params.trackWidthM * yawRateRadps);
    }

    float RightBankForwardVelocityMps(
        float forwardVelocityMps,
        float yawRateRadps,
        const MazeMap::PlantParams& params) noexcept
    {
        return forwardVelocityMps - (0.5f * params.trackWidthM * yawRateRadps);
    }

    float WheelSpeedFromSlip(
        float bankForwardVelocityMps,
        float slipRatio,
        const MazeMap::PlantParams& params) noexcept
    {
        const float velocityEpsilonMps = (std::max)(params.velocityEpsilonMps, 1.0e-3f);
        const float speedScaleMps = (std::max)(std::fabs(bankForwardVelocityMps), velocityEpsilonMps);
        return
            (params.wheelRadiusM > 0.0f) ?
            ((bankForwardVelocityMps + (slipRatio * speedScaleMps)) / params.wheelRadiusM) :
            0.0f;
    }

    float YawMomentNm(
        const MazeMap::ContactForces& forces,
        const MazeMap::PlantParams& params) noexcept
    {
        float yawMomentNm = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < params.contactPositionsBodyM.size(); ++contactIndex)
        {
            const Eigen::Vector2f contactPosition = params.ContactPosition(contactIndex);
            yawMomentNm +=
                (contactPosition.y() * forces.contacts[contactIndex].rightForceN) -
                (contactPosition.x() * forces.contacts[contactIndex].forwardForceN);
        }
        return yawMomentNm;
    }

    MazeMap::PlantModel::StateVector BuildMotionState(
        float forwardVelocityMps,
        float yawRateRadps,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps) noexcept
    {
        MazeMap::PlantModel::StateVector state = MazeMap::PlantModel::StateVector::Zero();
        state(MazeMap::VehicleState::kU) = forwardVelocityMps;
        state(MazeMap::VehicleState::kR) = yawRateRadps;
        state(MazeMap::VehicleState::kOmegaL) = leftWheelSpeedRadps;
        state(MazeMap::VehicleState::kOmegaR) = rightWheelSpeedRadps;
        return state;
    }

    MotionEvaluation EvaluateMotionTarget(
        const MazeMap::PlantModel& plant,
        float forwardVelocityMps,
        float yawRateRadps,
        float leftSlipRatio,
        float rightSlipRatio,
        float fanDutyCycle,
        const MazeMap::PlantParams& params) noexcept
    {
        MotionEvaluation evaluation{};
        const float leftBankForwardVelocityMps = LeftBankForwardVelocityMps(forwardVelocityMps, yawRateRadps, params);
        const float rightBankForwardVelocityMps = RightBankForwardVelocityMps(forwardVelocityMps, yawRateRadps, params);
        const float leftWheelSpeedRadps = WheelSpeedFromSlip(leftBankForwardVelocityMps, leftSlipRatio, params);
        const float rightWheelSpeedRadps = WheelSpeedFromSlip(rightBankForwardVelocityMps, rightSlipRatio, params);
        evaluation.state =
            BuildMotionState(forwardVelocityMps, yawRateRadps, leftWheelSpeedRadps, rightWheelSpeedRadps);

        MazeMap::ControlInput control{};
        control.fanDutyCycle = fanDutyCycle;
        control.batteryVoltageV = params.supplyVoltageV;
        evaluation.derivatives = plant.forwardStep(evaluation.state, control, params);
        return evaluation;
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
        params.contactPatchLongitudinalOffsetM = driveModel.wheelYOffsetM;
        params.motorCurrentLimitA =
            (params.driveResistanceOhms > 0.0f) ?
            (params.supplyVoltageV / params.driveResistanceOhms) :
            2.4f;
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
            const float forwardForceN = 0.5f * params.longitudinalTireStiffnessN * kappa;
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

    PlantModel::StateVector PlantModel::integrate(
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

    DriveCommandSolution PlantModel::solveDriveCommands(
        float forwardVelocityMps,
        float desiredLongitudinalAccelMps2,
        float yawRateRadps,
        float desiredYawAccelRadps2,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        DriveCommandSolution solution{};
        solution.control.fanDutyCycle = (std::clamp)(fanDutyCycle, 0.0f, 1.0f);
        solution.control.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);

        if (!(std::isfinite(forwardVelocityMps) &&
            std::isfinite(desiredLongitudinalAccelMps2) &&
            std::isfinite(yawRateRadps) &&
            std::isfinite(desiredYawAccelRadps2) &&
            (params.massKg > 0.0f) &&
            (params.effectiveLongitudinalMassKg > 0.0f) &&
            (params.yawInertiaKgM2 > 0.0f) &&
            (params.wheelRadiusM > 0.0f) &&
            (params.longitudinalTireStiffnessN > 0.0f) &&
            (params.equivalentWheelInertiaKgM2 > 0.0f)))
        {
            return solution;
        }

        constexpr float kSlipStep = 1.0e-3f;
        constexpr float kSlipTolerance = 5.0e-4f;
        constexpr uint8_t kMaxIterations = 12U;
        constexpr float kDamping = 1.0e-3f;
        constexpr float kMaxSlipStep = 0.75f;
        const float clampedFanDutyCycle = solution.control.fanDutyCycle;

        const float targetForwardForceN = params.effectiveLongitudinalMassKg * desiredLongitudinalAccelMps2;
        const float targetYawMomentNm = params.yawInertiaKgM2 * desiredYawAccelRadps2;
        const MotionEvaluation zeroSlipEvaluation =
            EvaluateMotionTarget(*this, forwardVelocityMps, yawRateRadps, 0.0f, 0.0f, clampedFanDutyCycle, params);
        const float baselineYawMomentNm = YawMomentNm(zeroSlipEvaluation.derivatives.contactForces, params);
        const float requestedForwardYawMomentNm = targetYawMomentNm - baselineYawMomentNm;
        const float yawForceBiasN =
            (std::fabs(params.trackWidthM) > 1.0e-6f) ?
            (requestedForwardYawMomentNm / params.trackWidthM) :
            0.0f;
        float leftSlipRatio =
            ((0.5f * targetForwardForceN) + yawForceBiasN) /
            params.longitudinalTireStiffnessN;
        float rightSlipRatio =
            ((0.5f * targetForwardForceN) - yawForceBiasN) /
            params.longitudinalTireStiffnessN;

        float bestResidualNorm = (std::numeric_limits<float>::max)();
        MotionEvaluation bestEvaluation = zeroSlipEvaluation;
        float bestLeftSlipRatio = leftSlipRatio;
        float bestRightSlipRatio = rightSlipRatio;

        for (uint8_t iteration = 0U; iteration < kMaxIterations; ++iteration)
        {
            const MotionEvaluation evaluation =
                EvaluateMotionTarget(
                    *this,
                    forwardVelocityMps,
                    yawRateRadps,
                    leftSlipRatio,
                    rightSlipRatio,
                    clampedFanDutyCycle,
                    params);
            const float longitudinalResidualMps2 =
                evaluation.derivatives.longitudinalAccelMps2 - desiredLongitudinalAccelMps2;
            const float yawResidualRadps2 =
                evaluation.derivatives.yawAccelRadps2 - desiredYawAccelRadps2;
            const float residualNorm =
                std::fabs(longitudinalResidualMps2) +
                std::fabs(yawResidualRadps2);
            if (residualNorm < bestResidualNorm)
            {
                bestResidualNorm = residualNorm;
                bestEvaluation = evaluation;
                bestLeftSlipRatio = leftSlipRatio;
                bestRightSlipRatio = rightSlipRatio;
            }

            if ((std::fabs(longitudinalResidualMps2) <= kSlipTolerance) &&
                (std::fabs(yawResidualRadps2) <= kSlipTolerance))
            {
                break;
            }

            const float leftStep = kSlipStep * (std::max)(1.0f, std::fabs(leftSlipRatio));
            const float rightStep = kSlipStep * (std::max)(1.0f, std::fabs(rightSlipRatio));
            const MotionEvaluation leftPerturbedEvaluation =
                EvaluateMotionTarget(
                    *this,
                    forwardVelocityMps,
                    yawRateRadps,
                    leftSlipRatio + leftStep,
                    rightSlipRatio,
                    clampedFanDutyCycle,
                    params);
            const MotionEvaluation rightPerturbedEvaluation =
                EvaluateMotionTarget(
                    *this,
                    forwardVelocityMps,
                    yawRateRadps,
                    leftSlipRatio,
                    rightSlipRatio + rightStep,
                    clampedFanDutyCycle,
                    params);

            const float j00 =
                (leftPerturbedEvaluation.derivatives.longitudinalAccelMps2 - evaluation.derivatives.longitudinalAccelMps2) /
                leftStep;
            const float j10 =
                (leftPerturbedEvaluation.derivatives.yawAccelRadps2 - evaluation.derivatives.yawAccelRadps2) /
                leftStep;
            const float j01 =
                (rightPerturbedEvaluation.derivatives.longitudinalAccelMps2 - evaluation.derivatives.longitudinalAccelMps2) /
                rightStep;
            const float j11 =
                (rightPerturbedEvaluation.derivatives.yawAccelRadps2 - evaluation.derivatives.yawAccelRadps2) /
                rightStep;

            const float a00 = (j00 * j00) + (j10 * j10) + kDamping;
            const float a01 = (j00 * j01) + (j10 * j11);
            const float a11 = (j01 * j01) + (j11 * j11) + kDamping;
            const float b0 = (j00 * longitudinalResidualMps2) + (j10 * yawResidualRadps2);
            const float b1 = (j01 * longitudinalResidualMps2) + (j11 * yawResidualRadps2);
            const float determinant = (a00 * a11) - (a01 * a01);
            if (!(determinant > 1.0e-8f))
            {
                break;
            }

            float deltaLeftSlipRatio = -((a11 * b0) - (a01 * b1)) / determinant;
            float deltaRightSlipRatio = -((a00 * b1) - (a01 * b0)) / determinant;
            deltaLeftSlipRatio = (std::clamp)(deltaLeftSlipRatio, -kMaxSlipStep, kMaxSlipStep);
            deltaRightSlipRatio = (std::clamp)(deltaRightSlipRatio, -kMaxSlipStep, kMaxSlipStep);

            bool acceptedStep = false;
            for (float stepScale = 1.0f; stepScale >= 0.125f; stepScale *= 0.5f)
            {
                const float candidateLeftSlipRatio = leftSlipRatio + (stepScale * deltaLeftSlipRatio);
                const float candidateRightSlipRatio = rightSlipRatio + (stepScale * deltaRightSlipRatio);
                const MotionEvaluation candidateEvaluation =
                    EvaluateMotionTarget(
                        *this,
                        forwardVelocityMps,
                        yawRateRadps,
                        candidateLeftSlipRatio,
                        candidateRightSlipRatio,
                        clampedFanDutyCycle,
                        params);
                const float candidateResidualNorm =
                    std::fabs(candidateEvaluation.derivatives.longitudinalAccelMps2 - desiredLongitudinalAccelMps2) +
                    std::fabs(candidateEvaluation.derivatives.yawAccelRadps2 - desiredYawAccelRadps2);
                if (candidateResidualNorm < residualNorm)
                {
                    leftSlipRatio = candidateLeftSlipRatio;
                    rightSlipRatio = candidateRightSlipRatio;
                    acceptedStep = true;
                    break;
                }
            }

            if (!acceptedStep)
            {
                break;
            }
        }

        const MotionEvaluation finalEvaluation =
            EvaluateMotionTarget(
                *this,
                forwardVelocityMps,
                yawRateRadps,
                leftSlipRatio,
                rightSlipRatio,
                clampedFanDutyCycle,
                params);
        const float finalResidualNorm =
            std::fabs(finalEvaluation.derivatives.longitudinalAccelMps2 - desiredLongitudinalAccelMps2) +
            std::fabs(finalEvaluation.derivatives.yawAccelRadps2 - desiredYawAccelRadps2);
        if (finalResidualNorm < bestResidualNorm)
        {
            bestResidualNorm = finalResidualNorm;
            bestEvaluation = finalEvaluation;
            bestLeftSlipRatio = leftSlipRatio;
            bestRightSlipRatio = rightSlipRatio;
        }

        solution.leftSlipRatio = bestLeftSlipRatio;
        solution.rightSlipRatio = bestRightSlipRatio;
        solution.leftWheelSpeedRadps = bestEvaluation.state(VehicleState::kOmegaL);
        solution.rightWheelSpeedRadps = bestEvaluation.state(VehicleState::kOmegaR);
        solution.leftWheelTorqueNm =
            params.wheelRadiusM * bestEvaluation.derivatives.contactForces.LeftBankForwardForceN();
        solution.rightWheelTorqueNm =
            params.wheelRadiusM * bestEvaluation.derivatives.contactForces.RightBankForwardForceN();

        solution.control.leftMotorCommand =
            driveCommandFromTorque(
                solution.leftWheelTorqueNm,
                solution.leftWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);
        solution.control.rightMotorCommand =
            driveCommandFromTorque(
                solution.rightWheelTorqueNm,
                solution.rightWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);

        const PlantDerivatives achievedDerivatives = forwardStep(bestEvaluation.state, solution.control, params);
        solution.leftWheelAccelRadps2 = achievedDerivatives.stateDot(VehicleState::kOmegaL);
        solution.rightWheelAccelRadps2 = achievedDerivatives.stateDot(VehicleState::kOmegaR);
        solution.longitudinalAccelErrorMps2 =
            achievedDerivatives.longitudinalAccelMps2 - desiredLongitudinalAccelMps2;
        solution.yawAccelErrorRadps2 =
            achievedDerivatives.yawAccelRadps2 - desiredYawAccelRadps2;
        solution.converged =
            (std::fabs(solution.longitudinalAccelErrorMps2) <= kSlipTolerance) &&
            (std::fabs(solution.yawAccelErrorRadps2) <= kSlipTolerance);
        return solution;
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
        float armatureCurrentA =
            (params.driveResistanceOhms > 0.0f) ?
            ((appliedVoltageV - backEmfVoltageV) / params.driveResistanceOhms) :
            0.0f;
        if (params.motorCurrentLimitA > 0.0f)
        {
            armatureCurrentA = (std::clamp)(armatureCurrentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float noLoadDirection = SignedDirection(armatureCurrentA, motorSpeedRadps);
        float loadCurrentA = armatureCurrentA - (noLoadDirection * params.noLoadCurrentA);
        if ((noLoadDirection > 0.0f) && (loadCurrentA < 0.0f))
        {
            loadCurrentA = 0.0f;
        }
        else if ((noLoadDirection < 0.0f) && (loadCurrentA > 0.0f))
        {
            loadCurrentA = 0.0f;
        }

        const float motorTorqueNm = params.torqueConstantNmPerA * loadCurrentA;
        return motorTorqueNm * params.gearRatio * params.drivetrainEfficiency;
    }

    float PlantModel::driveCommandFromTorque(
        float wheelTorqueNm,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const float appliedBatteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);
        const float driveGain = params.gearRatio * params.drivetrainEfficiency;
        if (!(std::isfinite(wheelTorqueNm) &&
            std::isfinite(wheelBankSpeedRadps) &&
            (appliedBatteryVoltageV > 0.0f) &&
            (driveGain > 0.0f) &&
            (params.torqueConstantNmPerA > 0.0f)))
        {
            return 0.0f;
        }

        const float motorTorqueNm = wheelTorqueNm / driveGain;
        const float noLoadDirection = SignedDirection(motorTorqueNm, wheelBankSpeedRadps);
        float armatureCurrentA =
            (motorTorqueNm / params.torqueConstantNmPerA) +
            (noLoadDirection * params.noLoadCurrentA);
        if (params.motorCurrentLimitA > 0.0f)
        {
            armatureCurrentA = (std::clamp)(armatureCurrentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float motorSpeedRadps = wheelBankSpeedRadps * params.gearRatio;
        const float backEmfVoltageV =
            (params.speedConstantRadpsPerVolt > 0.0f) ?
            (motorSpeedRadps / params.speedConstantRadpsPerVolt) :
            0.0f;
        const float appliedVoltageV = (armatureCurrentA * params.driveResistanceOhms) + backEmfVoltageV;
        return (std::clamp)(appliedVoltageV / appliedBatteryVoltageV, -1.0f, 1.0f);
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
