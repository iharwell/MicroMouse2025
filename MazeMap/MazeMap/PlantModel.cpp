#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef MAZEMAP_PLANTMODEL_VALIDATE_INVERSE
#define MAZEMAP_PLANTMODEL_VALIDATE_INVERSE 0
#endif

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

    float ReserveScaledDemandWithRequestedSign(
        float requestedDemand,
        float achievedDemand,
        float reserveScale) noexcept
    {
        if (!std::isfinite(requestedDemand) ||
            !std::isfinite(achievedDemand) ||
            !std::isfinite(reserveScale) ||
            !(reserveScale > 0.0f))
        {
            return 0.0f;
        }

        const float requestedSign = SignedDirection(requestedDemand, 0.0f);
        if (requestedSign == 0.0f)
        {
            return 0.0f;
        }

        const float reservedMagnitude =
            reserveScale *
            (std::min)(
                std::fabs(requestedDemand),
                std::fabs(achievedDemand));
        return requestedSign * reservedMagnitude;
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

    float FixedSplitBankForwardCapacityN(
        float frontCapacityN,
        float rearCapacityN,
        float lambdaFront,
        float lambdaRear,
        float forceEpsilonN) noexcept
    {
        float capacityN = (std::numeric_limits<float>::infinity)();

        if (lambdaFront > forceEpsilonN)
        {
            capacityN = (std::min)(capacityN, frontCapacityN / lambdaFront);
        }
        if (lambdaRear > forceEpsilonN)
        {
            capacityN = (std::min)(capacityN, rearCapacityN / lambdaRear);
        }

        return std::isfinite(capacityN) ? (std::max)(0.0f, capacityN) : 0.0f;
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
        // This is the actual traction-limit clamp for each contact patch. With the current default
        // plant (`combinedAccelPeakMps2 = 20.1`, `massKg = 0.14`, `frontLoadFraction = 0.5`),
        // pure-lateral saturation occurs at about 0.0391 rad / 2.24 deg front and
        // 0.0440 rad / 2.52 deg rear because the per-contact linear lateral force
        // (`corneringStiffness*alpha`) reaches `peakMu*normalForce`. Any simultaneous
        // longitudinal force demand reduces that slip-angle limit through this friction circle.
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

    PlantModel::StateVector BuildDriveCommandOperatingState(
        const PlantModel::StateVector& currentState,
        const PlantParams& params) noexcept
    {
        PlantModel::StateVector state = PlantModel::StateVector::Zero();
        state(VehicleState::kPx) =
            std::isfinite(currentState(VehicleState::kPx)) ? currentState(VehicleState::kPx) : 0.0f;
        state(VehicleState::kPy) =
            std::isfinite(currentState(VehicleState::kPy)) ? currentState(VehicleState::kPy) : 0.0f;
        state(VehicleState::kPsi) =
            std::isfinite(currentState(VehicleState::kPsi)) ? currentState(VehicleState::kPsi) : 0.0f;
        state(VehicleState::kU) =
            std::isfinite(currentState(VehicleState::kU)) ? currentState(VehicleState::kU) : 0.0f;
        state(VehicleState::kV) =
            std::isfinite(currentState(VehicleState::kV)) ? currentState(VehicleState::kV) : 0.0f;
        state(VehicleState::kR) =
            std::isfinite(currentState(VehicleState::kR)) ? currentState(VehicleState::kR) : 0.0f;

        const float wheelRadiusM = SafeWheelRadius(params);
        const float halfTrackWidthM = 0.5f * SafeTrackWidth(params);
        const float fallbackLeftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((state(VehicleState::kU) + (halfTrackWidthM * state(VehicleState::kR))) / wheelRadiusM) :
            0.0f;
        const float fallbackRightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((state(VehicleState::kU) - (halfTrackWidthM * state(VehicleState::kR))) / wheelRadiusM) :
            0.0f;
        state(VehicleState::kOmegaL) =
            std::isfinite(currentState(VehicleState::kOmegaL)) ?
            currentState(VehicleState::kOmegaL) :
            fallbackLeftWheelSpeedRadps;
        state(VehicleState::kOmegaR) =
            std::isfinite(currentState(VehicleState::kOmegaR)) ?
            currentState(VehicleState::kOmegaR) :
            fallbackRightWheelSpeedRadps;
        state(VehicleState::kBgz) =
            std::isfinite(currentState(VehicleState::kBgz)) ? currentState(VehicleState::kBgz) : 0.0f;
        VehicleState::NormalizeStateVector(state);
        return state;
    }

    PlantModel::StateVector BuildReducedDriveCommandOperatingState(
        float forwardVelocityMps,
        float yawRateRadps,
        const PlantParams& params) noexcept
    {
        PlantModel::StateVector state = PlantModel::StateVector::Zero();
        state(VehicleState::kU) = forwardVelocityMps;
        state(VehicleState::kR) = yawRateRadps;
        return BuildDriveCommandOperatingState(state, params);
    }

    PlantModel::StateVector BuildDriveCommandValidationState(
        const PlantModel::StateVector& currentState,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps,
        const PlantParams& params) noexcept
    {
        PlantModel::StateVector validationState = BuildDriveCommandOperatingState(currentState, params);
        validationState(VehicleState::kOmegaL) = leftWheelSpeedRadps;
        validationState(VehicleState::kOmegaR) = rightWheelSpeedRadps;
        VehicleState::NormalizeStateVector(validationState);
        return validationState;
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
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        DriveCommandSolution solution{};
        solution.control.fanDutyCycle = (std::clamp)(fanDutyCycle, 0.0f, 1.0f);
        solution.control.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);

        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float forwardVelocityMps = operatingState(VehicleState::kU);
        const float rightVelocityMps = operatingState(VehicleState::kV);
        const float yawRateRadps = operatingState(VehicleState::kR);
        const float longitudinalMassKg = EffectiveLongitudinalMassKg(params);
        const float lateralMassKg = EffectiveLateralMassKg(params);
        const float yawInertiaKgM2 = SafeYawInertiaKgM2(params);
        const float trackWidthM = SafeTrackWidth(params);
        const float wheelRadiusM = SafeWheelRadius(params);
        const float wheelInertiaKgM2 = SafeWheelInertiaKgM2(params);
        const float forceEpsilonN = SafeForceEpsilon(params);
        const float rollingRegularizationMps = RollingRegularizationMps(params);
        const float longitudinalStiffnessN = params.longitudinalTireStiffnessN;
        const float longitudinalOffsetM = std::fabs(params.contactPatchLongitudinalOffsetM);
        const float lateralDampingNPerM = (std::max)(0.0f, params.lateralVelocityDampingNsPerM);
        const float yawDampingNmPerRadps = (std::max)(0.0f, params.yawRateDampingNmsPerRad);

        if (!(std::isfinite(desiredLongitudinalAccelMps2) &&
            std::isfinite(desiredYawAccelRadps2) &&
            (trackWidthM > 0.0f) &&
            (wheelRadiusM > 0.0f) &&
            (lateralMassKg > 0.0f) &&
            (longitudinalStiffnessN > 0.0f) &&
            (wheelInertiaKgM2 > 0.0f)))
        {
            return solution;
        }

        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float longitudinalStiffnessSafeN = (std::max)(longitudinalStiffnessN, forceEpsilonN);
        const WheelKinematics operatingKinematics = wheelKinematics(operatingState, params);
        const SlipTargets baselineTargets = slipTargets(operatingState, operatingKinematics, params);
        const RollingContactEvaluation baselineForces =
            EvaluateRollingContactForces(baselineTargets, solution.control, params);
        const float baselineFrontRightForceN =
            baselineForces.forces.contacts[kFrontLeft].rightForceN +
            baselineForces.forces.contacts[kFrontRight].rightForceN;
        const float baselineRearRightForceN =
            baselineForces.forces.contacts[kRearLeft].rightForceN +
            baselineForces.forces.contacts[kRearRight].rightForceN;
        const float baselineYawMomentNm = longitudinalOffsetM * (baselineFrontRightForceN - baselineRearRightForceN);
        const float baselineSumRightForceN = baselineForces.forces.SumRightForceN();
        const float estimatedLateralAccelMps2 =
            std::fabs((baselineSumRightForceN / lateralMassKg) - ((lateralDampingNPerM / lateralMassKg) * rightVelocityMps));
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
        const float totalYawMomentCommandNm =
            (yawInertiaKgM2 * desiredYawAccelRadps2) +
            (yawDampingNmPerRadps * yawRateRadps);
        const float longitudinalYawMomentCommandNm = totalYawMomentCommandNm - baselineYawMomentNm;
        const float leftBankForceUnclippedN =
            (0.5f * totalForwardForceCommandN) + (longitudinalYawMomentCommandNm / trackWidthM);
        const float rightBankForceUnclippedN =
            (0.5f * totalForwardForceCommandN) - (longitudinalYawMomentCommandNm / trackWidthM);

        const ContactLoads loads = BuildContactLoads(solution.control.fanDutyCycle, params);
        const PeakFrictionCoefficients peak = BuildPeakFrictionCoefficients(loads, params);
        const float lambdaFront =
            (std::isfinite(params.frontLongitudinalForceSplit) ?
                 (std::clamp)(params.frontLongitudinalForceSplit, 0.0f, 1.0f) :
                 0.5f);
        const float lambdaRear = 1.0f - lambdaFront;
        const float flPeakForceN = peak.front * loads.flN;
        const float frPeakForceN = peak.front * loads.frN;
        const float rlPeakForceN = peak.rear * loads.rlN;
        const float rrPeakForceN = peak.rear * loads.rrN;
        const float flRightForceN = baselineForces.forces.contacts[kFrontLeft].rightForceN;
        const float frRightForceN = baselineForces.forces.contacts[kFrontRight].rightForceN;
        const float rlRightForceN = baselineForces.forces.contacts[kRearLeft].rightForceN;
        const float rrRightForceN = baselineForces.forces.contacts[kRearRight].rightForceN;
        const float flForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (flPeakForceN * flPeakForceN) - (flRightForceN * flRightForceN)));
        const float frForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (frPeakForceN * frPeakForceN) - (frRightForceN * frRightForceN)));
        const float rlForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (rlPeakForceN * rlPeakForceN) - (rlRightForceN * rlRightForceN)));
        const float rrForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (rrPeakForceN * rrPeakForceN) - (rrRightForceN * rrRightForceN)));
        const float leftBankForwardCapacityN =
            FixedSplitBankForwardCapacityN(
                flForwardCapacityN,
                rlForwardCapacityN,
                lambdaFront,
                lambdaRear,
                forceEpsilonN);
        const float rightBankForwardCapacityN =
            FixedSplitBankForwardCapacityN(
                frForwardCapacityN,
                rrForwardCapacityN,
                lambdaFront,
                lambdaRear,
                forceEpsilonN);

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

        const float totalForwardForceCommandScaledN = tractionScale * totalForwardForceCommandN;
        const float halfScaledTotalForwardForceN = 0.5f * totalForwardForceCommandScaledN;
        const float requestedLongitudinalYawMomentNm =
            tractionScale * longitudinalYawMomentCommandNm;
        const float minLongitudinalYawMomentNm =
            (std::max)(
                trackWidthM * (-leftBankForwardCapacityN - halfScaledTotalForwardForceN),
                trackWidthM * (halfScaledTotalForwardForceN - rightBankForwardCapacityN));
        const float maxLongitudinalYawMomentNm =
            (std::min)(
                trackWidthM * (leftBankForwardCapacityN - halfScaledTotalForwardForceN),
                trackWidthM * (halfScaledTotalForwardForceN + rightBankForwardCapacityN));
        const float refinedLongitudinalYawMomentNm =
            (minLongitudinalYawMomentNm <= maxLongitudinalYawMomentNm) ?
            (std::clamp)(
                requestedLongitudinalYawMomentNm,
                minLongitudinalYawMomentNm,
                maxLongitudinalYawMomentNm) :
            requestedLongitudinalYawMomentNm;
        const float leftBankForwardVelocityMps = forwardVelocityMps + (halfTrackWidthM * yawRateRadps);
        const float rightBankForwardVelocityMps = forwardVelocityMps - (halfTrackWidthM * yawRateRadps);
        const float leftRollingWheelSpeedRadps = leftBankForwardVelocityMps / wheelRadiusM;
        const float rightRollingWheelSpeedRadps = rightBankForwardVelocityMps / wheelRadiusM;
        const float achievedYawMomentNm = baselineYawMomentNm + refinedLongitudinalYawMomentNm;
        const float achievedYawAccelRadps2 =
            (achievedYawMomentNm - (yawDampingNmPerRadps * yawRateRadps)) / yawInertiaKgM2;
        const float leftWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 + (halfTrackWidthM * achievedYawAccelRadps2)) / wheelRadiusM;
        const float rightWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 - (halfTrackWidthM * achievedYawAccelRadps2)) / wheelRadiusM;

        const float leftBankForceCommandN =
            halfScaledTotalForwardForceN + (refinedLongitudinalYawMomentNm / trackWidthM);
        const float rightBankForceCommandN =
            halfScaledTotalForwardForceN - (refinedLongitudinalYawMomentNm / trackWidthM);
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

        solution.commandedLongitudinalAccelMps2 = clippedLongitudinalAccelMps2;
        solution.commandedYawAccelRadps2 = achievedYawAccelRadps2;
        solution.longitudinalAccelErrorMps2 =
            solution.commandedLongitudinalAccelMps2 - desiredLongitudinalAccelMps2;
        solution.yawAccelErrorRadps2 =
            solution.commandedYawAccelRadps2 - desiredYawAccelRadps2;
        solution.converged =
            !solution.tractionLimited &&
            (std::fabs(solution.longitudinalAccelErrorMps2) <= 0.05f) &&
            (std::fabs(solution.yawAccelErrorRadps2) <= 0.2f);

#if MAZEMAP_PLANTMODEL_VALIDATE_INVERSE
        const StateVector validationState =
            BuildDriveCommandValidationState(
                operatingState,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                params);
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
#endif
        return solution;
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
        return solveDriveCommands(
            BuildReducedDriveCommandOperatingState(forwardVelocityMps, yawRateRadps, params),
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV);
    }

    DriveCommandSolution PlantModel::solveDriveCommandsForVelocityTarget(
        const StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        const float desiredLongitudinalAccelMps2 =
            (targetForwardVelocityMps - operatingState(VehicleState::kU)) / resolvedResponseTimeS;
        const float desiredYawAccelRadps2 =
            (targetYawRateRadps - operatingState(VehicleState::kR)) / resolvedResponseTimeS;
        return solveDriveCommands(
            operatingState,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV);
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
        return solveDriveCommandsForVelocityTarget(
            BuildReducedDriveCommandOperatingState(currentForwardVelocityMps, currentYawRateRadps, params),
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        const StateVector& currentState,
        const PlantParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2,
        float fanDutyCycle) const noexcept
    {
        maxLongitudinalAccelMps2 = 0.0f;
        maxYawAccelRadps2 = 0.0f;
        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);

        constexpr float kLargeRequestedAccelMagnitude = 1.0e6f;

        const DriveCommandSolution positiveLongitudinal =
            solveDriveCommands(
                operatingState,
                kLargeRequestedAccelMagnitude,
                0.0f,
                params,
                fanDutyCycle);
        const DriveCommandSolution negativeLongitudinal =
            solveDriveCommands(
                operatingState,
                -kLargeRequestedAccelMagnitude,
                0.0f,
                params,
                fanDutyCycle);
        const DriveCommandSolution positiveYaw =
            solveDriveCommands(
                operatingState,
                0.0f,
                kLargeRequestedAccelMagnitude,
                params,
                fanDutyCycle);
        const DriveCommandSolution negativeYaw =
            solveDriveCommands(
                operatingState,
                0.0f,
                -kLargeRequestedAccelMagnitude,
                params,
                fanDutyCycle);

        const float positiveLongitudinalLimitMps2 =
            std::fabs(positiveLongitudinal.commandedLongitudinalAccelMps2);
        const float negativeLongitudinalLimitMps2 =
            std::fabs(negativeLongitudinal.commandedLongitudinalAccelMps2);
        if (std::isfinite(positiveLongitudinalLimitMps2) && std::isfinite(negativeLongitudinalLimitMps2))
        {
            maxLongitudinalAccelMps2 =
                (std::min)(positiveLongitudinalLimitMps2, negativeLongitudinalLimitMps2);
        }

        const float positiveYawLimitRadps2 =
            std::fabs(positiveYaw.commandedYawAccelRadps2);
        const float negativeYawLimitRadps2 =
            std::fabs(negativeYaw.commandedYawAccelRadps2);
        if (std::isfinite(positiveYawLimitRadps2) && std::isfinite(negativeYawLimitRadps2))
        {
            maxYawAccelRadps2 =
                (std::min)(positiveYawLimitRadps2, negativeYawLimitRadps2);
        }
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float forwardVelocityMps,
        float yawRateRadps,
        const PlantParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2,
        float fanDutyCycle) const noexcept
    {
        velocityTargetTechnicalLimits(
            BuildReducedDriveCommandOperatingState(forwardVelocityMps, yawRateRadps, params),
            params,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2,
            fanDutyCycle);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommands(
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float tractionReserveScale) const noexcept
    {
        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float resolvedReserveScale = ResolveClosedLoopTractionReserveScale(tractionReserveScale);
        const DriveCommandSolution tractionLimitedSolution =
            solveDriveCommands(
                operatingState,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                params,
                fanDutyCycle,
                batteryVoltageV);
        if (!tractionLimitedSolution.tractionLimited || (resolvedReserveScale >= 1.0f))
        {
            return tractionLimitedSolution;
        }

        const float reservedLongitudinalAccelMps2 =
            ReserveScaledDemandWithRequestedSign(
                desiredLongitudinalAccelMps2,
                tractionLimitedSolution.commandedLongitudinalAccelMps2,
                resolvedReserveScale);
        const float reservedYawAccelRadps2 =
            ReserveScaledDemandWithRequestedSign(
                desiredYawAccelRadps2,
                tractionLimitedSolution.commandedYawAccelRadps2,
                resolvedReserveScale);

        return solveDriveCommands(
            operatingState,
            reservedLongitudinalAccelMps2,
            reservedYawAccelRadps2,
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
        return solveClosedLoopDriveCommands(
            BuildReducedDriveCommandOperatingState(forwardVelocityMps, yawRateRadps, params),
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV,
            tractionReserveScale);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommandsForVelocityTarget(
        const StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PlantParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS,
        float tractionReserveScale) const noexcept
    {
        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        const float desiredLongitudinalAccelMps2 =
            (targetForwardVelocityMps - operatingState(VehicleState::kU)) / resolvedResponseTimeS;
        const float desiredYawAccelRadps2 =
            (targetYawRateRadps - operatingState(VehicleState::kR)) / resolvedResponseTimeS;
        return solveClosedLoopDriveCommands(
            operatingState,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            params,
            fanDutyCycle,
            batteryVoltageV,
            tractionReserveScale);
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
        return solveClosedLoopDriveCommandsForVelocityTarget(
            BuildReducedDriveCommandOperatingState(currentForwardVelocityMps, currentYawRateRadps, params),
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
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
