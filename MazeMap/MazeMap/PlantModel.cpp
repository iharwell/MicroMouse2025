#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using MazeMap::ContactForce;
    using MazeMap::ContactForces;
    using MazeMap::ControlInput;
    using MazeMap::MotionRegime;
    using MazeMap::PlantDerivatives;
    using MazeMap::PlantModel;
    using MazeMap::PlantParams;
    using MazeMap::SlipTargets;
    using MazeMap::VehicleState;
    using MazeMap::WheelKinematics;

    constexpr uint8_t kFrontLeft = 0U;
    constexpr uint8_t kFrontRight = 1U;
    constexpr uint8_t kRearLeft = 2U;
    constexpr uint8_t kRearRight = 3U;

    struct MotionMetrics
    {
        float speedNormMps = 0.0f;
        float commandNorm = 0.0f;
        float rollWeight = 0.0f;
    };

    struct ContactLoads
    {
        float totalN = 0.0f;
        float frontTotalN = 0.0f;
        float rearTotalN = 0.0f;
        float flN = 0.0f;
        float frN = 0.0f;
        float rlN = 0.0f;
        float rrN = 0.0f;
    };

    struct PeakFrictionCoefficients
    {
        float front = 0.0f;
        float rear = 0.0f;
    };

    struct SplitForceRequest
    {
        float leftBankForwardForceRawN = 0.0f;
        float rightBankForwardForceRawN = 0.0f;
        float frontAxleRightForceRawN = 0.0f;
        float rearAxleRightForceRawN = 0.0f;
    };

    struct RollingContactEvaluation
    {
        ContactForces forces{};
        float maxUtilization = 0.0f;
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

    float EffectiveBatteryVoltage(float batteryVoltageV, const PlantParams& params) noexcept
    {
        return
            (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
            batteryVoltageV :
            params.supplyVoltageV;
    }

    float ResolveVelocityTargetResponseTimeS(float responseTimeS) noexcept
    {
        return
            (std::isfinite(responseTimeS) && (responseTimeS > 0.0f)) ?
            responseTimeS :
            MazeMap::PlantModel::kDefaultVelocityTargetResponseTimeS;
    }

    float ResolveClosedLoopTractionReserveScale(float tractionReserveScale) noexcept
    {
        return
            (std::isfinite(tractionReserveScale) &&
             (tractionReserveScale > 0.0f) &&
             (tractionReserveScale <= 1.0f)) ?
            tractionReserveScale :
            MazeMap::PlantModel::kClosedLoopTractionReserveScale;
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

    float SafeForceEpsilon(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.forceEpsilonN) && (params.forceEpsilonN > 0.0f)) ?
            params.forceEpsilonN :
            1.0e-4f;
    }

    float SafeTrackWidth(const PlantParams& params) noexcept
    {
        return std::fabs(params.trackWidthM);
    }

    float SafeWheelRadius(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.wheelRadiusM) && (params.wheelRadiusM > 0.0f)) ?
            params.wheelRadiusM :
            0.0f;
    }

    float StaticFrictionSpeedWindowMps(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.staticFrictionMaxSpeedMps) && (params.staticFrictionMaxSpeedMps > 0.0f)) ?
            params.staticFrictionMaxSpeedMps :
            0.0f;
    }

    float StaticFrictionSpeedThresholdRadps(const PlantParams& params) noexcept
    {
        const float wheelRadiusM = SafeWheelRadius(params);
        const float speedWindowMps = StaticFrictionSpeedWindowMps(params);
        return (wheelRadiusM > 0.0f) ? (speedWindowMps / wheelRadiusM) : 0.0f;
    }

    float ConfiguredStaticFrictionTorqueNm(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.staticFrictionTorqueNm) && (params.staticFrictionTorqueNm > 0.0f)) ?
            params.staticFrictionTorqueNm :
            0.0f;
    }

    bool IsWithinStaticFrictionWindow(float wheelBankSpeedRadps, const PlantParams& params) noexcept
    {
        return std::fabs(wheelBankSpeedRadps) <= StaticFrictionSpeedThresholdRadps(params);
    }

    float EffectiveLongitudinalMassKg(const PlantParams& params) noexcept
    {
        if (std::isfinite(params.effectiveLongitudinalMassKg) && (params.effectiveLongitudinalMassKg > 0.0f))
        {
            return params.effectiveLongitudinalMassKg;
        }
        if (std::isfinite(params.massKg) && (params.massKg > 0.0f))
        {
            return params.massKg;
        }
        return 1.0f;
    }

    float EffectiveLateralMassKg(const PlantParams& params) noexcept
    {
        if (std::isfinite(params.effectiveLateralMassKg) && (params.effectiveLateralMassKg > 0.0f))
        {
            return params.effectiveLateralMassKg;
        }
        if (std::isfinite(params.massKg) && (params.massKg > 0.0f))
        {
            return params.massKg;
        }
        return 1.0f;
    }

    float SafeYawInertiaKgM2(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.yawInertiaKgM2) && (params.yawInertiaKgM2 > 0.0f)) ?
            params.yawInertiaKgM2 :
            1.0f;
    }

    float SafeWheelInertiaKgM2(const PlantParams& params) noexcept
    {
        return
            (std::isfinite(params.equivalentWheelInertiaKgM2) && (params.equivalentWheelInertiaKgM2 > 0.0f)) ?
            params.equivalentWheelInertiaKgM2 :
            1.0f;
    }

    float RollingRegularizationMps(const PlantParams& params) noexcept
    {
        if (std::isfinite(params.rollingSpeedRegularizationMps) && (params.rollingSpeedRegularizationMps > 0.0f))
        {
            return params.rollingSpeedRegularizationMps;
        }
        if (std::isfinite(params.velocityEpsilonMps) && (params.velocityEpsilonMps > 0.0f))
        {
            return params.velocityEpsilonMps;
        }
        return 1.0e-3f;
    }

    float NominalCombinedAccelerationEnvelopeMps2(const PlantParams& params) noexcept
    {
        if (std::isfinite(params.combinedAccelNominalMps2) && (params.combinedAccelNominalMps2 > 0.0f))
        {
            return params.combinedAccelNominalMps2;
        }
        if (std::isfinite(params.combinedAccelPeakMps2) && (params.combinedAccelPeakMps2 > 0.0f))
        {
            return params.combinedAccelPeakMps2;
        }
        return (std::numeric_limits<float>::infinity)();
    }

    float SmoothStep(float edge0, float edge1, float value) noexcept
    {
        if (!(std::isfinite(edge0) && std::isfinite(edge1) && std::isfinite(value)))
        {
            return 0.0f;
        }
        if (edge1 <= edge0)
        {
            return (value >= edge1) ? 1.0f : 0.0f;
        }

        const float t = (std::clamp)((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - (2.0f * t));
    }

    float StopLeverArmM(const PlantParams& params) noexcept
    {
        return (std::max)(std::fabs(params.contactPatchLongitudinalOffsetM), 0.5f * SafeTrackWidth(params));
    }

    MotionMetrics EvaluateMotionMetrics(
        const PlantModel::StateVector& state,
        const ControlInput& control,
        const PlantParams& params) noexcept
    {
        MotionMetrics metrics{};
        const float wheelRadiusM = SafeWheelRadius(params);
        const float yawLeverArmM = StopLeverArmM(params);
        metrics.speedNormMps =
            (std::max)(
                (std::max)(
                    (std::max)(std::fabs(state(VehicleState::kU)), std::fabs(state(VehicleState::kV))),
                    std::fabs(state(VehicleState::kR) * yawLeverArmM)),
                (std::max)(
                    std::fabs(wheelRadiusM * state(VehicleState::kOmegaL)),
                    std::fabs(wheelRadiusM * state(VehicleState::kOmegaR))));
        metrics.commandNorm =
            (std::max)(std::fabs(control.leftMotorCommand), std::fabs(control.rightMotorCommand));

        const float speedWeight =
            SmoothStep(params.stopEnterSpeedMps, params.stopExitSpeedMps, metrics.speedNormMps);
        const float yawRateWeight =
            SmoothStep(params.stopEnterYawRateRadps, params.stopExitYawRateRadps, std::fabs(state(VehicleState::kR)));
        const float commandWeight =
            SmoothStep(params.stopEnterCommand, params.stopExitCommand, metrics.commandNorm);
        metrics.rollWeight = (std::max)((std::max)(speedWeight, yawRateWeight), commandWeight);
        return metrics;
    }

    ContactLoads BuildContactLoads(float fanDutyCycle, const PlantParams& params) noexcept
    {
        ContactLoads loads{};
        const float clampedFanDutyCycle = (std::clamp)(fanDutyCycle, 0.0f, 1.0f);
        const float frontLoadFraction =
            (std::isfinite(params.frontLoadFraction) ? (std::clamp)(params.frontLoadFraction, 0.0f, 1.0f) : 0.5f);

        loads.totalN = params.TotalNormalLoadN(clampedFanDutyCycle);
        loads.frontTotalN = frontLoadFraction * loads.totalN;
        loads.rearTotalN = (1.0f - frontLoadFraction) * loads.totalN;
        loads.flN = 0.5f * loads.frontTotalN;
        loads.frN = 0.5f * loads.frontTotalN;
        loads.rlN = 0.5f * loads.rearTotalN;
        loads.rrN = 0.5f * loads.rearTotalN;
        return loads;
    }

    PeakFrictionCoefficients BuildPeakFrictionCoefficients(
        const ContactLoads& loads,
        const PlantParams& params) noexcept
    {
        PeakFrictionCoefficients coefficients{};
        const float forceEpsilonN = SafeForceEpsilon(params);
        const float envelopeMu =
            (std::isfinite(params.combinedAccelPeakMps2) &&
             (params.combinedAccelPeakMps2 > 0.0f) &&
             std::isfinite(params.massKg) &&
             (params.massKg > 0.0f)) ?
            ((params.combinedAccelPeakMps2 * params.massKg) / (std::max)(loads.totalN, forceEpsilonN)) :
            0.0f;

        coefficients.front =
            (std::isfinite(params.muFrontPeak) && (params.muFrontPeak > 0.0f)) ?
            params.muFrontPeak :
            ((envelopeMu > 0.0f) ? envelopeMu : (std::max)(0.0f, params.muFront));
        coefficients.rear =
            (std::isfinite(params.muRearPeak) && (params.muRearPeak > 0.0f)) ?
            params.muRearPeak :
            ((envelopeMu > 0.0f) ? envelopeMu : (std::max)(0.0f, params.muRear));
        return coefficients;
    }

    bool ShouldSnapToZero(
        const PlantModel::StateVector& state,
        const ControlInput& control,
        const PlantParams& params) noexcept
    {
        const MotionMetrics metrics = EvaluateMotionMetrics(state, control, params);
        return
            (metrics.speedNormMps < params.stopEnterSpeedMps) &&
            (std::fabs(state(VehicleState::kR)) < params.stopEnterYawRateRadps) &&
            (std::fabs(state(VehicleState::kOmegaL)) < params.stopEnterWheelSpeedRadps) &&
            (std::fabs(state(VehicleState::kOmegaR)) < params.stopEnterWheelSpeedRadps) &&
            (metrics.commandNorm < params.stopEnterCommand);
    }

    bool ShouldReportStoppedDiagnostics(
        const PlantModel::StateVector& state,
        const PlantParams& params) noexcept
    {
        const MotionMetrics metrics = EvaluateMotionMetrics(state, ControlInput{}, params);
        return
            (metrics.speedNormMps < params.stopEnterSpeedMps) &&
            (std::fabs(state(VehicleState::kR)) < params.stopEnterYawRateRadps) &&
            (std::fabs(state(VehicleState::kOmegaL)) < params.stopEnterWheelSpeedRadps) &&
            (std::fabs(state(VehicleState::kOmegaR)) < params.stopEnterWheelSpeedRadps);
    }

    ContactForce BuildClampedContactForce(
        float rawForwardForceN,
        float rawRightForceN,
        float normalForceN,
        float peakMu,
        const PlantParams& params) noexcept
    {
        ContactForce contact{};
        const float forceEpsilonN = SafeForceEpsilon(params);
        const float rawMagnitudeN =
            MazeMap::Math::Sqrtf((rawForwardForceN * rawForwardForceN) + (rawRightForceN * rawRightForceN));
        const float maxForceN = (std::max)(0.0f, peakMu * normalForceN);
        const float forceScale = maxForceN / (std::max)(rawMagnitudeN, forceEpsilonN);
        const float scale = (forceScale < 1.0f) ? forceScale : 1.0f;
        const float saturationDenominatorN = (maxForceN > forceEpsilonN) ? maxForceN : forceEpsilonN;
        const float utilization = rawMagnitudeN / saturationDenominatorN;

        contact.forwardForceN = scale * rawForwardForceN;
        contact.rightForceN = scale * rawRightForceN;
        contact.normalForceN = normalForceN;
        contact.saturation = (utilization < 1.0f) ? utilization : 1.0f;
        return contact;
    }

    RollingContactEvaluation EvaluateSplitContactForces(
        const SplitForceRequest& request,
        float fanDutyCycle,
        const PlantParams& params) noexcept
    {
        RollingContactEvaluation evaluation{};
        const ContactLoads loads = BuildContactLoads(fanDutyCycle, params);
        const PeakFrictionCoefficients peak = BuildPeakFrictionCoefficients(loads, params);
        const float lambdaFront =
            (std::isfinite(params.frontLongitudinalForceSplit) ?
                 (std::clamp)(params.frontLongitudinalForceSplit, 0.0f, 1.0f) :
                 0.5f);
        const float lambdaRear = 1.0f - lambdaFront;

        const float fxFlRaw = lambdaFront * request.leftBankForwardForceRawN;
        const float fxRlRaw = lambdaRear * request.leftBankForwardForceRawN;
        const float fxFrRaw = lambdaFront * request.rightBankForwardForceRawN;
        const float fxRrRaw = lambdaRear * request.rightBankForwardForceRawN;

        const float fyFlRaw = 0.5f * request.frontAxleRightForceRawN;
        const float fyFrRaw = 0.5f * request.frontAxleRightForceRawN;
        const float fyRlRaw = 0.5f * request.rearAxleRightForceRawN;
        const float fyRrRaw = 0.5f * request.rearAxleRightForceRawN;

        evaluation.forces.contacts[kFrontLeft] = BuildClampedContactForce(fxFlRaw, fyFlRaw, loads.flN, peak.front, params);
        evaluation.forces.contacts[kFrontRight] = BuildClampedContactForce(fxFrRaw, fyFrRaw, loads.frN, peak.front, params);
        evaluation.forces.contacts[kRearLeft] = BuildClampedContactForce(fxRlRaw, fyRlRaw, loads.rlN, peak.rear, params);
        evaluation.forces.contacts[kRearRight] = BuildClampedContactForce(fxRrRaw, fyRrRaw, loads.rrN, peak.rear, params);

        for (const ContactForce& contact : evaluation.forces.contacts)
        {
            evaluation.maxUtilization = (std::max)(evaluation.maxUtilization, contact.saturation);
        }
        return evaluation;
    }

    SlipTargets ComputeRollingSlipTargets(
        const PlantModel::StateVector& state,
        const WheelKinematics& kinematics,
        const PlantParams& params) noexcept
    {
        SlipTargets targets{};
        const float forwardVelocityMps = state(VehicleState::kU);
        const float wheelRadiusM = SafeWheelRadius(params);
        const float regularizationMps = RollingRegularizationMps(params);
        const float leftCircumferentialVelocityMps = wheelRadiusM * state(VehicleState::kOmegaL);
        const float rightCircumferentialVelocityMps = wheelRadiusM * state(VehicleState::kOmegaR);
        const float uRefLeft = (std::max)(std::fabs(kinematics.leftBankForwardVelocityMps), regularizationMps);
        const float uRefRight = (std::max)(std::fabs(kinematics.rightBankForwardVelocityMps), regularizationMps);
        const float uRefBody = (std::max)(std::fabs(forwardVelocityMps), regularizationMps);
        const float frontLateralRatio = kinematics.contacts[kFrontLeft].rightVelocityMps / uRefBody;
        const float rearLateralRatio = kinematics.contacts[kRearLeft].rightVelocityMps / uRefBody;

        targets.kappaLeft = (leftCircumferentialVelocityMps - kinematics.leftBankForwardVelocityMps) / uRefLeft;
        targets.kappaRight = (rightCircumferentialVelocityMps - kinematics.rightBankForwardVelocityMps) / uRefRight;
        targets.lateralRatio[kFrontLeft] = frontLateralRatio;
        targets.lateralRatio[kFrontRight] = frontLateralRatio;
        targets.lateralRatio[kRearLeft] = rearLateralRatio;
        targets.lateralRatio[kRearRight] = rearLateralRatio;
        return targets;
    }

    RollingContactEvaluation EvaluateRollingContactForces(
        const SlipTargets& targets,
        const ControlInput& control,
        const PlantParams& params) noexcept
    {
        const float alphaFront = std::atan(targets.lateralRatio[kFrontLeft]);
        const float alphaRear = std::atan(targets.lateralRatio[kRearLeft]);
        const float longitudinalStiffnessN = params.longitudinalTireStiffnessN;
        const float frontCorneringStiffnessNPerRad = 2.0f * params.corneringStiffnessFrontNPerRad;
        const float rearCorneringStiffnessNPerRad = 2.0f * params.corneringStiffnessRearNPerRad;

        SplitForceRequest request{};
        request.leftBankForwardForceRawN = longitudinalStiffnessN * targets.kappaLeft;
        request.rightBankForwardForceRawN = longitudinalStiffnessN * targets.kappaRight;
        request.frontAxleRightForceRawN = -frontCorneringStiffnessNPerRad * alphaFront;
        request.rearAxleRightForceRawN = -rearCorneringStiffnessNPerRad * alphaRear;
        return EvaluateSplitContactForces(request, control.fanDutyCycle, params);
    }

    ContactForces BlendContactForces(const ContactForces& rollingForces, float rollWeight) noexcept
    {
        ContactForces blended = rollingForces;
        const float clampedWeight = (std::clamp)(rollWeight, 0.0f, 1.0f);
        for (ContactForce& contact : blended.contacts)
        {
            contact.forwardForceN *= clampedWeight;
            contact.rightForceN *= clampedWeight;
            contact.saturation *= clampedWeight;
        }
        return blended;
    }

    float LateralScrubMotionWeight(const PlantModel::StateVector& state) noexcept
    {
        const float lateralVelocityMps = state(VehicleState::kV);
        return (std::isfinite(lateralVelocityMps) && (lateralVelocityMps != 0.0f)) ? 1.0f : 0.0f;
    }

    PlantModel::StateVector BuildMotionState(
        float forwardVelocityMps,
        float yawRateRadps,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps) noexcept
    {
        PlantModel::StateVector state = PlantModel::StateVector::Zero();
        state(VehicleState::kU) = forwardVelocityMps;
        state(VehicleState::kR) = yawRateRadps;
        state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
        state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
        return state;
    }
}

namespace MazeMap
{
    PlantParams PlantParams::Default() noexcept
    {
        PlantParams params{};
        constexpr float kReliableLaunchDriveCommand = 0.30f;
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
        if ((params.driveResistanceOhms > 0.0f) &&
            (params.torqueConstantNmPerA > 0.0f) &&
            (params.gearRatio > 0.0f) &&
            (params.supplyVoltageV > 0.0f))
        {
            float breakawayCurrentA =
                (kReliableLaunchDriveCommand * params.supplyVoltageV) /
                params.driveResistanceOhms;
            if (params.motorCurrentLimitA > 0.0f)
            {
                breakawayCurrentA =
                    (std::clamp)(breakawayCurrentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
            }

            const float breakawayLoadCurrentA =
                (std::max)(0.0f, breakawayCurrentA - params.noLoadCurrentA);
            params.staticFrictionTorqueNm =
                params.torqueConstantNmPerA *
                breakawayLoadCurrentA *
                params.gearRatio *
                params.drivetrainEfficiency;
        }
        params.frontLeftSensor = Vehicle::GetFrontLeftSensorExtrinsics();
        params.frontRightSensor = Vehicle::GetFrontRightSensorExtrinsics();
        params.sideLeftSensor = Vehicle::GetSideLeftSensorExtrinsics();
        params.sideRightSensor = Vehicle::GetSideRightSensorExtrinsics();
        params.imu = Vehicle::GetBackLeftImuExtrinsics();

        const float frontContactY = std::fabs(params.contactPatchLongitudinalOffsetM);
        const float halfTrackWidthM = 0.5f * params.trackWidthM;
        params.contactPositionsBodyM[kFrontLeft] = Eigen::Vector2f(-halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[kFrontRight] = Eigen::Vector2f(halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[kRearLeft] = Eigen::Vector2f(-halfTrackWidthM, -frontContactY);
        params.contactPositionsBodyM[kRearRight] = Eigen::Vector2f(halfTrackWidthM, -frontContactY);
        return params;
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        PlantDerivatives derivatives{};
        const MotionMetrics metrics = EvaluateMotionMetrics(state, control, params);
        const float motionWeight = (std::max)(metrics.rollWeight, LateralScrubMotionWeight(state));
        const WheelKinematics kinematics = wheelKinematics(state, params);
        SlipTargets rollingTargets{};
        RollingContactEvaluation rollingForces{};
        if (motionWeight > 0.0f)
        {
            rollingTargets = ComputeRollingSlipTargets(state, kinematics, params);
            rollingForces = EvaluateRollingContactForces(rollingTargets, control, params);
        }
        else
        {
            rollingForces = EvaluateSplitContactForces(SplitForceRequest{}, control.fanDutyCycle, params);
        }

        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float batteryVoltageV = EffectiveBatteryVoltage(control.batteryVoltageV, params);
        const float trackWidthM = SafeTrackWidth(params);
        const float wheelRadiusM = SafeWheelRadius(params);
        const float wheelInertiaKgM2 = SafeWheelInertiaKgM2(params);
        const float longitudinalMassKg = EffectiveLongitudinalMassKg(params);
        const float lateralMassKg = EffectiveLateralMassKg(params);
        const float yawInertiaKgM2 = SafeYawInertiaKgM2(params);
        const float longitudinalOffsetM = std::fabs(params.contactPatchLongitudinalOffsetM);

        const float leftBankForwardForceN = rollingForces.forces.LeftBankForwardForceN();
        const float rightBankForwardForceN = rollingForces.forces.RightBankForwardForceN();
        const float frontRightForceN =
            rollingForces.forces.contacts[kFrontLeft].rightForceN +
            rollingForces.forces.contacts[kFrontRight].rightForceN;
        const float rearRightForceN =
            rollingForces.forces.contacts[kRearLeft].rightForceN +
            rollingForces.forces.contacts[kRearRight].rightForceN;
        const float sumForwardForceN = rollingForces.forces.SumForwardForceN();
        const float sumRightForceN = rollingForces.forces.SumRightForceN();
        const float yawMomentNm =
            (0.5f * trackWidthM * (leftBankForwardForceN - rightBankForwardForceN)) +
            (longitudinalOffsetM * (frontRightForceN - rearRightForceN));
        const float lateralDampingNPerM = (std::max)(0.0f, params.lateralVelocityDampingNsPerM);
        const float yawDampingNmPerRadps = (std::max)(0.0f, params.yawRateDampingNmsPerRad);
        const float leftDriveTorqueNm =
            driveTorqueFromCommand(control.leftMotorCommand, omegaLeftRadps, batteryVoltageV, params);
        const float rightDriveTorqueNm =
            driveTorqueFromCommand(control.rightMotorCommand, omegaRightRadps, batteryVoltageV, params);
        const float leftPreFrictionWheelTorqueNm = leftDriveTorqueNm - (wheelRadiusM * leftBankForwardForceN);
        const float rightPreFrictionWheelTorqueNm = rightDriveTorqueNm - (wheelRadiusM * rightBankForwardForceN);
        const float leftFrictionTorqueNm =
            driveFrictionTorque(omegaLeftRadps, leftPreFrictionWheelTorqueNm, params);
        const float rightFrictionTorqueNm =
            driveFrictionTorque(omegaRightRadps, rightPreFrictionWheelTorqueNm, params);
        const float staticFrictionTorqueNm = ConfiguredStaticFrictionTorqueNm(params);
        float leftNetWheelTorqueNm = leftPreFrictionWheelTorqueNm - leftFrictionTorqueNm;
        float rightNetWheelTorqueNm = rightPreFrictionWheelTorqueNm - rightFrictionTorqueNm;
        if (IsWithinStaticFrictionWindow(omegaLeftRadps, params) &&
            (std::fabs(leftPreFrictionWheelTorqueNm) <= staticFrictionTorqueNm) &&
            (SignedDirection(leftPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            leftNetWheelTorqueNm = 0.0f;
        }
        if (IsWithinStaticFrictionWindow(omegaRightRadps, params) &&
            (std::fabs(rightPreFrictionWheelTorqueNm) <= staticFrictionTorqueNm) &&
            (SignedDirection(rightPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            rightNetWheelTorqueNm = 0.0f;
        }

        StateVector rollingStateDot = StateVector::Zero();
        const Eigen::Vector2f heading = HeadingUnitFromYaw(psi);
        const Eigen::Vector2f right = RightUnitFromHeading(heading);
        rollingStateDot(VehicleState::kPx) =
            (rightVelocityMps * right.x()) + (forwardVelocityMps * heading.x());
        rollingStateDot(VehicleState::kPy) =
            (rightVelocityMps * right.y()) + (forwardVelocityMps * heading.y());
        rollingStateDot(VehicleState::kPsi) = yawRateRadps;
        rollingStateDot(VehicleState::kU) =
            (yawRateRadps * rightVelocityMps) +
            (sumForwardForceN / longitudinalMassKg);
        rollingStateDot(VehicleState::kV) =
            (-yawRateRadps * forwardVelocityMps) +
            (sumRightForceN / lateralMassKg) -
            ((lateralDampingNPerM / lateralMassKg) * rightVelocityMps);
        rollingStateDot(VehicleState::kR) =
            (yawMomentNm / yawInertiaKgM2) -
            ((yawDampingNmPerRadps / yawInertiaKgM2) * yawRateRadps);
        rollingStateDot(VehicleState::kOmegaL) = leftNetWheelTorqueNm / wheelInertiaKgM2;
        rollingStateDot(VehicleState::kOmegaR) = rightNetWheelTorqueNm / wheelInertiaKgM2;
        rollingStateDot(VehicleState::kBgz) = 0.0f;

        derivatives.stateDot = motionWeight * rollingStateDot;
        derivatives.contactForces = BlendContactForces(rollingForces.forces, motionWeight);
        derivatives.wheelKinematics = kinematics;
        derivatives.maxContactUtilization = motionWeight * rollingForces.maxUtilization;

        if (motionWeight < 0.5f)
        {
            derivatives.regime = MotionRegime::StoppedHold;
            derivatives.slipTargets = SlipTargets{};
        }
        else if (rollingForces.maxUtilization >= (1.0f - 1.0e-4f))
        {
            derivatives.regime = MotionRegime::RollingSaturated;
            derivatives.slipTargets = rollingTargets;
            derivatives.maxContactUtilization = rollingForces.maxUtilization;
        }
        else
        {
            derivatives.regime = MotionRegime::RollingAdherent;
            derivatives.slipTargets = rollingTargets;
            derivatives.maxContactUtilization = rollingForces.maxUtilization;
        }

        derivatives.originAccelBodyMps2 = Eigen::Vector2f(
            derivatives.stateDot(VehicleState::kV) + (yawRateRadps * forwardVelocityMps),
            derivatives.stateDot(VehicleState::kU) - (yawRateRadps * rightVelocityMps));
        const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            derivatives.originAccelBodyMps2.x() -
                (yawRateSquaredRadps2 * params.imu.positionBodyM.x()) +
                (derivatives.stateDot(VehicleState::kR) * params.imu.positionBodyM.y()),
            derivatives.originAccelBodyMps2.y() +
                (yawRateSquaredRadps2 * -params.imu.positionBodyM.y()) -
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
        const float halfTrackWidthM = 0.5f * SafeTrackWidth(params);
        const float longitudinalOffsetM = std::fabs(params.contactPatchLongitudinalOffsetM);
        const float leftBankVelocityMps = forwardVelocityMps + (halfTrackWidthM * yawRateRadps);
        const float rightBankVelocityMps = forwardVelocityMps - (halfTrackWidthM * yawRateRadps);
        const float frontLateralVelocityMps = rightVelocityMps + (longitudinalOffsetM * yawRateRadps);
        const float rearLateralVelocityMps = rightVelocityMps - (longitudinalOffsetM * yawRateRadps);

        kinematics.leftBankForwardVelocityMps = leftBankVelocityMps;
        kinematics.rightBankForwardVelocityMps = rightBankVelocityMps;

        kinematics.contacts[kFrontLeft].forwardVelocityMps = leftBankVelocityMps;
        kinematics.contacts[kFrontRight].forwardVelocityMps = rightBankVelocityMps;
        kinematics.contacts[kRearLeft].forwardVelocityMps = leftBankVelocityMps;
        kinematics.contacts[kRearRight].forwardVelocityMps = rightBankVelocityMps;

        kinematics.contacts[kFrontLeft].rightVelocityMps = frontLateralVelocityMps;
        kinematics.contacts[kFrontRight].rightVelocityMps = frontLateralVelocityMps;
        kinematics.contacts[kRearLeft].rightVelocityMps = rearLateralVelocityMps;
        kinematics.contacts[kRearRight].rightVelocityMps = rearLateralVelocityMps;
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
        if (ShouldReportStoppedDiagnostics(state, params))
        {
            return SlipTargets{};
        }
        return ComputeRollingSlipTargets(state, kinematics, params);
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
        const MotionMetrics metrics = EvaluateMotionMetrics(state, control, params);
        if (metrics.rollWeight <= 0.0f)
        {
            const RollingContactEvaluation zeroForces =
                EvaluateSplitContactForces(SplitForceRequest{}, control.fanDutyCycle, params);
            return zeroForces.forces;
        }

        const WheelKinematics kinematics = wheelKinematics(state, params);
        const SlipTargets targets = ComputeRollingSlipTargets(state, kinematics, params);
        const ContactForces rollingForces = EvaluateRollingContactForces(targets, control, params).forces;
        return BlendContactForces(rollingForces, metrics.rollWeight);
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

        // This function runs once per UKF sigma point on the Teensy control path. The runtime budget
        // cannot afford a second plant function evaluation, substeps, or a predictor/corrector pass here.
        const PlantDerivatives derivatives = forwardStep(state, control, params);
        StateVector implicitState = state + (dt * derivatives.stateDot);
        implicitState(VehicleState::kPsi) = VehicleState::NormalizeAngle(implicitState(VehicleState::kPsi));

        if (ShouldSnapToZero(implicitState, control, params))
        {
            implicitState(VehicleState::kU) = 0.0f;
            implicitState(VehicleState::kV) = 0.0f;
            implicitState(VehicleState::kR) = 0.0f;
            implicitState(VehicleState::kOmegaL) = 0.0f;
            implicitState(VehicleState::kOmegaR) = 0.0f;
        }

        return implicitState;
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

        const float longitudinalMassKg = EffectiveLongitudinalMassKg(params);
        const float yawInertiaKgM2 = SafeYawInertiaKgM2(params);
        const float trackWidthM = SafeTrackWidth(params);
        const float wheelRadiusM = SafeWheelRadius(params);
        const float wheelInertiaKgM2 = SafeWheelInertiaKgM2(params);
        const float forceEpsilonN = SafeForceEpsilon(params);
        const float rollingRegularizationMps = RollingRegularizationMps(params);
        const float longitudinalStiffnessN = params.longitudinalTireStiffnessN;
        const float frontCorneringStiffnessNPerRad = 2.0f * params.corneringStiffnessFrontNPerRad;
        const float rearCorneringStiffnessNPerRad = 2.0f * params.corneringStiffnessRearNPerRad;
        const float longitudinalOffsetM = std::fabs(params.contactPatchLongitudinalOffsetM);

        if (!(std::isfinite(forwardVelocityMps) &&
            std::isfinite(desiredLongitudinalAccelMps2) &&
            std::isfinite(yawRateRadps) &&
            std::isfinite(desiredYawAccelRadps2) &&
            (trackWidthM > 0.0f) &&
            (wheelRadiusM > 0.0f) &&
            (longitudinalStiffnessN > 0.0f) &&
            (wheelInertiaKgM2 > 0.0f)))
        {
            return solution;
        }

        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float longitudinalStiffnessSafeN = (std::max)(longitudinalStiffnessN, forceEpsilonN);
        const float uRefMps = (std::max)(std::fabs(forwardVelocityMps), rollingRegularizationMps);
        const float alphaFront = std::atan2(longitudinalOffsetM * yawRateRadps, uRefMps);
        const float alphaRear = std::atan2(-longitudinalOffsetM * yawRateRadps, uRefMps);
        SplitForceRequest baselineRequest{};
        baselineRequest.frontAxleRightForceRawN = -frontCorneringStiffnessNPerRad * alphaFront;
        baselineRequest.rearAxleRightForceRawN = -rearCorneringStiffnessNPerRad * alphaRear;
        const RollingContactEvaluation baselineForces =
            EvaluateSplitContactForces(baselineRequest, solution.control.fanDutyCycle, params);
        const float baselineFrontRightForceN =
            baselineForces.forces.contacts[kFrontLeft].rightForceN +
            baselineForces.forces.contacts[kFrontRight].rightForceN;
        const float baselineRearRightForceN =
            baselineForces.forces.contacts[kRearLeft].rightForceN +
            baselineForces.forces.contacts[kRearRight].rightForceN;
        const float baselineYawMomentNm = longitudinalOffsetM * (baselineFrontRightForceN - baselineRearRightForceN);
        const float estimatedLateralAccelMps2 = std::fabs(forwardVelocityMps * yawRateRadps);
        const float desiredLongitudinalAccelMagnitudeMps2 = std::fabs(desiredLongitudinalAccelMps2);
        const float combinedEnvelopeMps2 = NominalCombinedAccelerationEnvelopeMps2(params);
        const float availableLongitudinalAccelMps2 =
            std::isfinite(combinedEnvelopeMps2) ?
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                (combinedEnvelopeMps2 * combinedEnvelopeMps2) -
                    (estimatedLateralAccelMps2 * estimatedLateralAccelMps2))) :
            desiredLongitudinalAccelMagnitudeMps2;
        const float clippedLongitudinalAccelMps2 =
            (std::clamp)(
                desiredLongitudinalAccelMps2,
                -availableLongitudinalAccelMps2,
                availableLongitudinalAccelMps2);
        const float totalForwardForceCommandN = longitudinalMassKg * clippedLongitudinalAccelMps2;
        const float totalYawMomentCommandNm = yawInertiaKgM2 * desiredYawAccelRadps2;
        const float longitudinalYawMomentCommandNm = totalYawMomentCommandNm - baselineYawMomentNm;
        const float leftBankForceUnclippedN =
            (0.5f * totalForwardForceCommandN) + (longitudinalYawMomentCommandNm / trackWidthM);
        const float rightBankForceUnclippedN =
            (0.5f * totalForwardForceCommandN) - (longitudinalYawMomentCommandNm / trackWidthM);

        const ContactLoads loads = BuildContactLoads(solution.control.fanDutyCycle, params);
        const PeakFrictionCoefficients peak = BuildPeakFrictionCoefficients(loads, params);
        const float flPeakForceN = peak.front * loads.flN;
        const float frPeakForceN = peak.front * loads.frN;
        const float rlPeakForceN = peak.rear * loads.rlN;
        const float rrPeakForceN = peak.rear * loads.rrN;
        const float flRightForceN = baselineForces.forces.contacts[kFrontLeft].rightForceN;
        const float frRightForceN = baselineForces.forces.contacts[kFrontRight].rightForceN;
        const float rlRightForceN = baselineForces.forces.contacts[kRearLeft].rightForceN;
        const float rrRightForceN = baselineForces.forces.contacts[kRearRight].rightForceN;
        const float leftBankForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                (flPeakForceN * flPeakForceN) - (flRightForceN * flRightForceN))) +
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                (rlPeakForceN * rlPeakForceN) - (rlRightForceN * rlRightForceN)));
        const float rightBankForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                (frPeakForceN * frPeakForceN) - (frRightForceN * frRightForceN))) +
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                (rrPeakForceN * rrPeakForceN) - (rrRightForceN * rrRightForceN)));

        float tractionScale = 1.0f;
        tractionScale =
            (std::min)(
                tractionScale,
                leftBankForwardCapacityN / (std::max)(std::fabs(leftBankForceUnclippedN), forceEpsilonN));
        tractionScale =
            (std::min)(
                tractionScale,
                rightBankForwardCapacityN / (std::max)(std::fabs(rightBankForceUnclippedN), forceEpsilonN));

        const float envelopeScale =
            (desiredLongitudinalAccelMagnitudeMps2 > forceEpsilonN) ?
            (std::fabs(clippedLongitudinalAccelMps2) / desiredLongitudinalAccelMagnitudeMps2) :
            1.0f;
        solution.tractionScale = (std::min)(tractionScale, envelopeScale);
        solution.tractionLimited = (solution.tractionScale < (1.0f - 1.0e-4f));

        const float leftBankForceCommandN = tractionScale * leftBankForceUnclippedN;
        const float rightBankForceCommandN = tractionScale * rightBankForceUnclippedN;
        const float leftBankForwardVelocityMps = forwardVelocityMps + (halfTrackWidthM * yawRateRadps);
        const float rightBankForwardVelocityMps = forwardVelocityMps - (halfTrackWidthM * yawRateRadps);
        const float leftRollingWheelSpeedRadps = leftBankForwardVelocityMps / wheelRadiusM;
        const float rightRollingWheelSpeedRadps = rightBankForwardVelocityMps / wheelRadiusM;
        const float leftWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 + (halfTrackWidthM * desiredYawAccelRadps2)) / wheelRadiusM;
        const float rightWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 - (halfTrackWidthM * desiredYawAccelRadps2)) / wheelRadiusM;
        const float leftContactTorqueNm = wheelRadiusM * leftBankForceCommandN;
        const float rightContactTorqueNm = wheelRadiusM * rightBankForceCommandN;
        const float leftWheelSpeedRadps =
            (leftBankForwardVelocityMps +
             ((leftBankForceCommandN / longitudinalStiffnessSafeN) *
              (std::max)(std::fabs(leftBankForwardVelocityMps), rollingRegularizationMps))) /
            wheelRadiusM;
        const float rightWheelSpeedRadps =
            (rightBankForwardVelocityMps +
             ((rightBankForceCommandN / longitudinalStiffnessSafeN) *
              (std::max)(std::fabs(rightBankForwardVelocityMps), rollingRegularizationMps))) /
            wheelRadiusM;
        const float leftWheelTorqueRequestNm =
            leftContactTorqueNm +
            (wheelInertiaKgM2 * leftWheelAccelRadps2);
        const float rightWheelTorqueRequestNm =
            rightContactTorqueNm +
            (wheelInertiaKgM2 * rightWheelAccelRadps2);
        const float leftWheelTorqueNm =
            leftWheelTorqueRequestNm +
            driveFrictionTorque(leftWheelSpeedRadps, leftWheelTorqueRequestNm, params);
        const float rightWheelTorqueNm =
            rightWheelTorqueRequestNm +
            driveFrictionTorque(rightWheelSpeedRadps, rightWheelTorqueRequestNm, params);

        solution.leftSlipRatio = leftBankForceCommandN / longitudinalStiffnessSafeN;
        solution.rightSlipRatio = rightBankForceCommandN / longitudinalStiffnessSafeN;
        solution.leftWheelSpeedRadps = leftWheelSpeedRadps;
        solution.rightWheelSpeedRadps = rightWheelSpeedRadps;
        solution.leftRollingWheelSpeedRadps = leftRollingWheelSpeedRadps;
        solution.rightRollingWheelSpeedRadps = rightRollingWheelSpeedRadps;
        solution.leftWheelAccelRadps2 = leftWheelAccelRadps2;
        solution.rightWheelAccelRadps2 = rightWheelAccelRadps2;
        solution.leftContactForceN = leftBankForceCommandN;
        solution.rightContactForceN = rightBankForceCommandN;
        solution.leftContactTorqueNm = leftContactTorqueNm;
        solution.rightContactTorqueNm = rightContactTorqueNm;
        solution.leftWheelTorqueNm = leftWheelTorqueNm;
        solution.rightWheelTorqueNm = rightWheelTorqueNm;

        solution.control.leftMotorCommand =
            driveCommandFromTorque(
                leftWheelTorqueNm,
                leftWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);
        solution.control.rightMotorCommand =
            driveCommandFromTorque(
                rightWheelTorqueNm,
                rightWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);

        const StateVector validationState =
            BuildMotionState(forwardVelocityMps, yawRateRadps, leftWheelSpeedRadps, rightWheelSpeedRadps);
        const PlantDerivatives achievedDerivatives = forwardStep(validationState, solution.control, params);
        solution.commandedLongitudinalAccelMps2 = achievedDerivatives.longitudinalAccelMps2;
        solution.commandedYawAccelRadps2 = achievedDerivatives.yawAccelRadps2;
        solution.longitudinalAccelErrorMps2 =
            achievedDerivatives.longitudinalAccelMps2 - desiredLongitudinalAccelMps2;
        solution.yawAccelErrorRadps2 =
            achievedDerivatives.yawAccelRadps2 - desiredYawAccelRadps2;
        solution.converged =
            !solution.tractionLimited &&
            (std::fabs(solution.longitudinalAccelErrorMps2) <= 0.05f) &&
            (std::fabs(solution.yawAccelErrorRadps2) <= 0.2f);
        return solution;
    }

    DriveCommandSolution PlantModel::solveDriveCommandsForVelocityTarget(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        const float desiredLongitudinalAccelMps2 =
            (targetForwardVelocityMps - currentForwardVelocityMps) / resolvedResponseTimeS;
        const float desiredYawAccelRadps2 =
            (targetYawRateRadps - currentYawRateRadps) / resolvedResponseTimeS;
        return solveDriveCommands(
            currentForwardVelocityMps,
            desiredLongitudinalAccelMps2,
            currentYawRateRadps,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommands(
        float forwardVelocityMps,
        float desiredLongitudinalAccelMps2,
        float yawRateRadps,
        float desiredYawAccelRadps2,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float tractionReserveScale) const noexcept
    {
        const float resolvedReserveScale = ResolveClosedLoopTractionReserveScale(tractionReserveScale);
        const DriveCommandSolution tractionLimitedSolution =
            solveDriveCommands(
                forwardVelocityMps,
                desiredLongitudinalAccelMps2,
                yawRateRadps,
                desiredYawAccelRadps2,
                params,
                fanDutyCycle,
                batteryVoltageV);
        if (!tractionLimitedSolution.tractionLimited || (resolvedReserveScale >= 1.0f))
        {
            return tractionLimitedSolution;
        }

        return solveDriveCommands(
            forwardVelocityMps,
            resolvedReserveScale * tractionLimitedSolution.commandedLongitudinalAccelMps2,
            yawRateRadps,
            resolvedReserveScale * tractionLimitedSolution.commandedYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommandsForVelocityTarget(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS,
        float tractionReserveScale) const noexcept
    {
        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        const float desiredLongitudinalAccelMps2 =
            (targetForwardVelocityMps - currentForwardVelocityMps) / resolvedResponseTimeS;
        const float desiredYawAccelRadps2 =
            (targetYawRateRadps - currentYawRateRadps) / resolvedResponseTimeS;
        return solveClosedLoopDriveCommands(
            currentForwardVelocityMps,
            desiredLongitudinalAccelMps2,
            currentYawRateRadps,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV,
            tractionReserveScale);
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

    float PlantModel::driveFrictionTorque(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        const PlantParams& params) const noexcept
    {
        const float viscousFrictionTorqueNm = params.viscousFrictionNmPerRadps * wheelBankSpeedRadps;
        if (IsWithinStaticFrictionWindow(wheelBankSpeedRadps, params))
        {
            const float sign = SignedDirection(wheelTorqueRequestNm, wheelBankSpeedRadps);
            return (ConfiguredStaticFrictionTorqueNm(params) * sign) + viscousFrictionTorqueNm;
        }

        const float sign = SignedDirection(wheelBankSpeedRadps, wheelTorqueRequestNm);
        return (params.rollingFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
    }
}
