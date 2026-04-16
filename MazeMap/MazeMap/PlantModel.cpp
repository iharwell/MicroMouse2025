
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
    using MazeMap::DriveCommandSolution;
    using MazeMap::MotionRegime;
    using MazeMap::PlantDerivatives;
    using MazeMap::PlantModel;
    using MazeMap::PlantParams;
    using MazeMap::SlipTargets;
    using MazeMap::VehicleState;
    using MazeMap::WheelKinematics;

    using PreparedParams = PlantModel::PreparedParams;

    constexpr uint8_t kFrontLeft = 0U;
    constexpr uint8_t kFrontRight = 1U;
    constexpr uint8_t kRearLeft = 2U;
    constexpr uint8_t kRearRight = 3U;
    constexpr float kSignEpsilon = 1.0e-6f;

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
        float leftBankForwardForceN = 0.0f;
        float rightBankForwardForceN = 0.0f;
        float frontRightForceN = 0.0f;
        float rearRightForceN = 0.0f;
        float sumForwardForceN = 0.0f;
        float sumRightForceN = 0.0f;
    };

    struct RollingStateEvaluation
    {
        SlipTargets targets{};
        RollingContactEvaluation contact{};
    };

    inline float ClampUnit(float value) noexcept
    {
        return (std::clamp)(value, -1.0f, 1.0f);
    }

    inline float Clamp01(float value) noexcept
    {
        return (std::clamp)(value, 0.0f, 1.0f);
    }

    inline float SignedDirectionFast(float preferredValue, float fallbackValue) noexcept
    {
        const int preferredSign = (preferredValue > kSignEpsilon) - (preferredValue < -kSignEpsilon);
        if (preferredSign != 0)
        {
            return static_cast<float>(preferredSign);
        }

        const int fallbackSign = (fallbackValue > kSignEpsilon) - (fallbackValue < -kSignEpsilon);
        return static_cast<float>(fallbackSign);
    }

    inline float EffectiveBatteryVoltage(float batteryVoltageV, const PlantParams& params) noexcept
    {
        return
            (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
            batteryVoltageV :
            params.supplyVoltageV;
    }

    inline float EffectiveBatteryVoltage(float batteryVoltageV, const PreparedParams& params) noexcept
    {
        return
            (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
            batteryVoltageV :
            params.supplyVoltageV;
    }

    inline float ResolveVelocityTargetResponseTimeS(float responseTimeS) noexcept
    {
        return
            (std::isfinite(responseTimeS) && (responseTimeS > 0.0f)) ?
            responseTimeS :
            PlantModel::kDefaultVelocityTargetResponseTimeS;
    }

    inline float ResolvePhysicalTrackWidthM(const PreparedParams& params) noexcept
    {
        if (std::isfinite(params.trackWidthM) && (params.trackWidthM > 0.0f))
        {
            return params.trackWidthM;
        }

        const float physicalTrackWidthM = MazeMap::Vehicle::GetPhysicalModel().trackWidthM;
        return
            (std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
            physicalTrackWidthM :
            0.0f;
    }

    inline float ResolveMotionTrackWidthM(
        float forwardVelocityMps,
        float yawRateRadps,
        const PreparedParams& params) noexcept
    {
        const float effectiveTrackWidthM =
            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(forwardVelocityMps, yawRateRadps);
        if (std::isfinite(effectiveTrackWidthM) && (effectiveTrackWidthM > 0.0f))
        {
            return effectiveTrackWidthM;
        }

        return ResolvePhysicalTrackWidthM(params);
    }

    inline float ResolveClosedLoopTractionReserveScale(float tractionReserveScale) noexcept
    {
        return
            (std::isfinite(tractionReserveScale) &&
             (tractionReserveScale > 0.0f) &&
             (tractionReserveScale <= 1.0f)) ?
            tractionReserveScale :
            PlantModel::kClosedLoopTractionReserveScale;
    }

    inline float SafePositive(float value, float fallbackValue) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : fallbackValue;
    }

    inline float SmoothStep(float edge0, float edge1, float value) noexcept
    {
        if (!(std::isfinite(edge0) && std::isfinite(edge1) && std::isfinite(value)))
        {
            return 0.0f;
        }
        if (edge1 <= edge0)
        {
            return (value >= edge1) ? 1.0f : 0.0f;
        }

        const float t = Clamp01((value - edge0) / (edge1 - edge0));
        return t * t * (3.0f - (2.0f * t));
    }

    inline float ComputeSpeedNormMps(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        const PreparedParams& params) noexcept
    {
        const float yawSpeedMps = std::fabs(yawRateRadps * params.yawLeverArmM);
        const float leftWheelSpeedMps = std::fabs(params.wheelRadiusM * omegaLeftRadps);
        const float rightWheelSpeedMps = std::fabs(params.wheelRadiusM * omegaRightRadps);
        return
            (std::max)(
                (std::max)(
                    (std::max)(std::fabs(forwardVelocityMps), std::fabs(rightVelocityMps)),
                    yawSpeedMps),
                (std::max)(leftWheelSpeedMps, rightWheelSpeedMps));
    }

    inline MotionMetrics EvaluateMotionMetrics(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm,
        const PreparedParams& params) noexcept
    {
        MotionMetrics metrics{};
        metrics.speedNormMps =
            ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params);
        metrics.commandNorm = commandNorm;

        const float speedWeight =
            SmoothStep(params.stopEnterSpeedMps, params.stopExitSpeedMps, metrics.speedNormMps);
        const float yawRateWeight =
            SmoothStep(params.stopEnterYawRateRadps, params.stopExitYawRateRadps, std::fabs(yawRateRadps));
        const float commandWeight =
            SmoothStep(params.stopEnterCommand, params.stopExitCommand, commandNorm);
        metrics.rollWeight = (std::max)((std::max)(speedWeight, yawRateWeight), commandWeight);
        return metrics;
    }

    inline bool IsStoppedFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm,
        const PreparedParams& params) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params) < params.stopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < params.stopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < params.stopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < params.stopEnterWheelSpeedRadps) &&
            (commandNorm < params.stopEnterCommand) &&
            !((std::isfinite(rightVelocityMps)) && (rightVelocityMps != 0.0f));
    }

    inline bool ShouldSnapToZeroFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm,
        const PreparedParams& params) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params) < params.stopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < params.stopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < params.stopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < params.stopEnterWheelSpeedRadps) &&
            (commandNorm < params.stopEnterCommand);
    }

    inline bool ShouldReportStoppedDiagnosticsFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        const PreparedParams& params) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params) < params.stopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < params.stopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < params.stopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < params.stopEnterWheelSpeedRadps);
    }

    inline float ReserveScaledDemandWithRequestedSign(
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

        const float requestedSign = SignedDirectionFast(requestedDemand, 0.0f);
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

    inline ContactLoads BuildContactLoads(float fanDutyCycle, const PreparedParams& params) noexcept
    {
        ContactLoads loads{};
        const float clampedFanDutyCycle = Clamp01(fanDutyCycle);
        loads.totalN = params.baseNormalLoadN + (clampedFanDutyCycle * params.fanDownforceAtFullDutyN);
        loads.frontTotalN = params.frontLoadFraction * loads.totalN;
        loads.rearTotalN = params.rearLoadFraction * loads.totalN;
        loads.flN = 0.5f * loads.frontTotalN;
        loads.frN = loads.flN;
        loads.rlN = 0.5f * loads.rearTotalN;
        loads.rrN = loads.rlN;
        return loads;
    }

    inline PeakFrictionCoefficients BuildPeakFrictionCoefficients(
        const ContactLoads& loads,
        const PreparedParams& params) noexcept
    {
        PeakFrictionCoefficients coefficients{};
        const float denominator = (std::max)(loads.totalN, params.forceEpsilonN);
        const float envelopeMu =
            (params.combinedAccelPeakTimesMass > 0.0f) ?
            (params.combinedAccelPeakTimesMass / denominator) :
            0.0f;

        coefficients.front = params.useEnvelopeMuFront ? envelopeMu : params.muFrontBase;
        coefficients.rear = params.useEnvelopeMuRear ? envelopeMu : params.muRearBase;
        return coefficients;
    }

    inline ContactForce BuildClampedContactForce(
        float rawForwardForceN,
        float rawRightForceN,
        float normalForceN,
        float peakMu,
        float forceEpsilonN) noexcept
    {
        ContactForce contact{};
        const float rawMagnitudeSquaredN2 =
            (rawForwardForceN * rawForwardForceN) + (rawRightForceN * rawRightForceN);
        const float rawMagnitudeN = MazeMap::Math::Sqrtf(rawMagnitudeSquaredN2);
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

    inline RollingContactEvaluation EvaluateSplitContactForces(
        const SplitForceRequest& request,
        float fanDutyCycle,
        const PreparedParams& params) noexcept
    {
        RollingContactEvaluation evaluation{};
        const ContactLoads loads = BuildContactLoads(fanDutyCycle, params);
        const PeakFrictionCoefficients peak = BuildPeakFrictionCoefficients(loads, params);

        const float fxFlRaw = params.lambdaFront * request.leftBankForwardForceRawN;
        const float fxRlRaw = params.lambdaRear * request.leftBankForwardForceRawN;
        const float fxFrRaw = params.lambdaFront * request.rightBankForwardForceRawN;
        const float fxRrRaw = params.lambdaRear * request.rightBankForwardForceRawN;

        const float fyFlRaw = 0.5f * request.frontAxleRightForceRawN;
        const float fyFrRaw = fyFlRaw;
        const float fyRlRaw = 0.5f * request.rearAxleRightForceRawN;
        const float fyRrRaw = fyRlRaw;

        ContactForce& fl = evaluation.forces.contacts[kFrontLeft];
        ContactForce& fr = evaluation.forces.contacts[kFrontRight];
        ContactForce& rl = evaluation.forces.contacts[kRearLeft];
        ContactForce& rr = evaluation.forces.contacts[kRearRight];

        fl = BuildClampedContactForce(fxFlRaw, fyFlRaw, loads.flN, peak.front, params.forceEpsilonN);
        fr = BuildClampedContactForce(fxFrRaw, fyFrRaw, loads.frN, peak.front, params.forceEpsilonN);
        rl = BuildClampedContactForce(fxRlRaw, fyRlRaw, loads.rlN, peak.rear, params.forceEpsilonN);
        rr = BuildClampedContactForce(fxRrRaw, fyRrRaw, loads.rrN, peak.rear, params.forceEpsilonN);

        evaluation.leftBankForwardForceN = fl.forwardForceN + rl.forwardForceN;
        evaluation.rightBankForwardForceN = fr.forwardForceN + rr.forwardForceN;
        evaluation.frontRightForceN = fl.rightForceN + fr.rightForceN;
        evaluation.rearRightForceN = rl.rightForceN + rr.rightForceN;
        evaluation.sumForwardForceN = evaluation.leftBankForwardForceN + evaluation.rightBankForwardForceN;
        evaluation.sumRightForceN = evaluation.frontRightForceN + evaluation.rearRightForceN;
        evaluation.maxUtilization =
            (std::max)(
                (std::max)(fl.saturation, fr.saturation),
                (std::max)(rl.saturation, rr.saturation));
        return evaluation;
    }

    inline WheelKinematics BuildWheelKinematics(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        const PreparedParams& params) noexcept
    {
        WheelKinematics kinematics{};
        const float leftBankVelocityMps = forwardVelocityMps + (params.halfTrackWidthM * yawRateRadps);
        const float rightBankVelocityMps = forwardVelocityMps - (params.halfTrackWidthM * yawRateRadps);
        const float frontLateralVelocityMps = rightVelocityMps + (params.longitudinalOffsetM * yawRateRadps);
        const float rearLateralVelocityMps = rightVelocityMps - (params.longitudinalOffsetM * yawRateRadps);

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

    inline SlipTargets ComputeRollingSlipTargets(
        float forwardVelocityMps,
        float omegaLeftRadps,
        float omegaRightRadps,
        const WheelKinematics& kinematics,
        const PreparedParams& params) noexcept
    {
        SlipTargets targets{};
        const float leftCircumferentialVelocityMps = params.wheelRadiusM * omegaLeftRadps;
        const float rightCircumferentialVelocityMps = params.wheelRadiusM * omegaRightRadps;
        const float uRefLeft = (std::max)(std::fabs(kinematics.leftBankForwardVelocityMps), params.rollingRegularizationMps);
        const float uRefRight = (std::max)(std::fabs(kinematics.rightBankForwardVelocityMps), params.rollingRegularizationMps);
        const float uRefBody = (std::max)(std::fabs(forwardVelocityMps), params.rollingRegularizationMps);

        const float invURefLeft = 1.0f / uRefLeft;
        const float invURefRight = 1.0f / uRefRight;
        const float invURefBody = 1.0f / uRefBody;

        const float frontLateralRatio = kinematics.contacts[kFrontLeft].rightVelocityMps * invURefBody;
        const float rearLateralRatio = kinematics.contacts[kRearLeft].rightVelocityMps * invURefBody;

        targets.kappaLeft = (leftCircumferentialVelocityMps - kinematics.leftBankForwardVelocityMps) * invURefLeft;
        targets.kappaRight = (rightCircumferentialVelocityMps - kinematics.rightBankForwardVelocityMps) * invURefRight;
        targets.lateralRatio[kFrontLeft] = frontLateralRatio;
        targets.lateralRatio[kFrontRight] = frontLateralRatio;
        targets.lateralRatio[kRearLeft] = rearLateralRatio;
        targets.lateralRatio[kRearRight] = rearLateralRatio;
        return targets;
    }

    inline RollingStateEvaluation EvaluateRollingState(
        float forwardVelocityMps,
        float omegaLeftRadps,
        float omegaRightRadps,
        const WheelKinematics& kinematics,
        float fanDutyCycle,
        const PreparedParams& params) noexcept
    {
        RollingStateEvaluation evaluation{};
        evaluation.targets =
            ComputeRollingSlipTargets(
                forwardVelocityMps,
                omegaLeftRadps,
                omegaRightRadps,
                kinematics,
                params);

        const float alphaFront = std::atan(evaluation.targets.lateralRatio[kFrontLeft]);
        const float alphaRear = std::atan(evaluation.targets.lateralRatio[kRearLeft]);

        SplitForceRequest request{};
        request.leftBankForwardForceRawN = params.longitudinalTireStiffnessN * evaluation.targets.kappaLeft;
        request.rightBankForwardForceRawN = params.longitudinalTireStiffnessN * evaluation.targets.kappaRight;
        request.frontAxleRightForceRawN = -params.frontCorneringStiffnessAxleNPerRad * alphaFront;
        request.rearAxleRightForceRawN = -params.rearCorneringStiffnessAxleNPerRad * alphaRear;
        evaluation.contact = EvaluateSplitContactForces(request, fanDutyCycle, params);
        return evaluation;
    }

    inline ContactForces BlendContactForces(const ContactForces& rollingForces, float rollWeight) noexcept
    {
        if (rollWeight <= 0.0f)
        {
            ContactForces zeroed = rollingForces;
            for (ContactForce& contact : zeroed.contacts)
            {
                contact.forwardForceN = 0.0f;
                contact.rightForceN = 0.0f;
                contact.saturation = 0.0f;
            }
            return zeroed;
        }

        if (rollWeight >= 1.0f)
        {
            return rollingForces;
        }

        ContactForces blended = rollingForces;
        for (ContactForce& contact : blended.contacts)
        {
            contact.forwardForceN *= rollWeight;
            contact.rightForceN *= rollWeight;
            contact.saturation *= rollWeight;
        }
        return blended;
    }

    inline PlantModel::StateVector BuildDriveCommandOperatingState(
        const PlantModel::StateVector& currentState,
        const PreparedParams& params) noexcept
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

        const float fallbackLeftWheelSpeedRadps =
            (params.wheelRadiusM > 0.0f) ?
            ((state(VehicleState::kU) + (params.halfTrackWidthM * state(VehicleState::kR))) * params.invWheelRadiusM) :
            0.0f;
        const float fallbackRightWheelSpeedRadps =
            (params.wheelRadiusM > 0.0f) ?
            ((state(VehicleState::kU) - (params.halfTrackWidthM * state(VehicleState::kR))) * params.invWheelRadiusM) :
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

    inline PlantModel::StateVector BuildReducedDriveCommandOperatingState(
        float forwardVelocityMps,
        float yawRateRadps,
        const PreparedParams& params) noexcept
    {
        PlantModel::StateVector state = PlantModel::StateVector::Zero();
        state(VehicleState::kU) = forwardVelocityMps;
        state(VehicleState::kR) = yawRateRadps;
        return BuildDriveCommandOperatingState(state, params);
    }

    inline PlantModel::StateVector BuildDriveCommandValidationState(
        const PlantModel::StateVector& currentState,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps,
        const PreparedParams& params) noexcept
    {
        PlantModel::StateVector validationState = BuildDriveCommandOperatingState(currentState, params);
        validationState(VehicleState::kOmegaL) = leftWheelSpeedRadps;
        validationState(VehicleState::kOmegaR) = rightWheelSpeedRadps;
        VehicleState::NormalizeStateVector(validationState);
        return validationState;
    }

    inline float FixedSplitBankForwardCapacityN(
        float frontCapacityN,
        float rearCapacityN,
        const PreparedParams& params) noexcept
    {
        float capacityN = (std::numeric_limits<float>::infinity)();

        if (params.lambdaFront > params.forceEpsilonN)
        {
            capacityN = (std::min)(capacityN, frontCapacityN / params.lambdaFront);
        }
        if (params.lambdaRear > params.forceEpsilonN)
        {
            capacityN = (std::min)(capacityN, rearCapacityN / params.lambdaRear);
        }

        return std::isfinite(capacityN) ? (std::max)(0.0f, capacityN) : 0.0f;
    }

    inline float DriveFrictionTorqueFast(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        const PreparedParams& params) noexcept
    {
        const float viscousFrictionTorqueNm = params.viscousFrictionNmPerRadps * wheelBankSpeedRadps;
        if (std::fabs(wheelBankSpeedRadps) <= params.staticFrictionSpeedThresholdRadps)
        {
            const float sign = SignedDirectionFast(wheelTorqueRequestNm, wheelBankSpeedRadps);
            return (params.staticFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
        }

        const float sign = SignedDirectionFast(wheelBankSpeedRadps, wheelTorqueRequestNm);
        return (params.rollingFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
    }

    inline float DriveTorqueFromCommandFast(
        float motorCommand,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PreparedParams& params) noexcept
    {
        const float command = ClampUnit(motorCommand);
        const float appliedVoltageV = command * EffectiveBatteryVoltage(batteryVoltageV, params);

        float armatureCurrentA = 0.0f;
        if (params.driveResistanceOhms > 0.0f)
        {
            armatureCurrentA =
                (appliedVoltageV * params.invDriveResistanceOhms) -
                (wheelBankSpeedRadps * params.wheelSpeedToCurrentAPerRadps);
        }

        if (params.motorCurrentLimitA > 0.0f)
        {
            armatureCurrentA = (std::clamp)(armatureCurrentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float noLoadDirection = SignedDirectionFast(armatureCurrentA, wheelBankSpeedRadps);
        float loadCurrentA = armatureCurrentA - (noLoadDirection * params.noLoadCurrentA);
        if ((noLoadDirection > 0.0f) && (loadCurrentA < 0.0f))
        {
            loadCurrentA = 0.0f;
        }
        else if ((noLoadDirection < 0.0f) && (loadCurrentA > 0.0f))
        {
            loadCurrentA = 0.0f;
        }

        return params.wheelTorquePerAmpNm * loadCurrentA;
    }

    inline float DriveCommandFromTorqueFast(
        float wheelTorqueNm,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PreparedParams& params) noexcept
    {
        const float appliedBatteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);
        if (!(std::isfinite(wheelTorqueNm) &&
            std::isfinite(wheelBankSpeedRadps) &&
            (appliedBatteryVoltageV > 0.0f) &&
            (params.driveGain > 0.0f) &&
            (params.torqueConstantNmPerA > 0.0f)))
        {
            return 0.0f;
        }

        const float motorTorqueNm = wheelTorqueNm * params.invDriveGain;
        const float noLoadDirection = SignedDirectionFast(motorTorqueNm, wheelBankSpeedRadps);
        float armatureCurrentA =
            (motorTorqueNm / params.torqueConstantNmPerA) +
            (noLoadDirection * params.noLoadCurrentA);
        if (params.motorCurrentLimitA > 0.0f)
        {
            armatureCurrentA = (std::clamp)(armatureCurrentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float backEmfVoltageV = wheelBankSpeedRadps * params.wheelSpeedToBackEmfVoltPerRadps;
        const float appliedVoltageV = (armatureCurrentA * params.driveResistanceOhms) + backEmfVoltageV;
        return ClampUnit(appliedVoltageV / appliedBatteryVoltageV);
    }

    inline float ResolveSlipBearingWheelSpeedRadps(
        float rollingWheelLinearVelocityMps,
        float rollingWheelOmegaRadps,
        float slipRatio,
        const PreparedParams& params) noexcept
    {
        const float referenceVelocityMps =
            (std::max)(std::fabs(rollingWheelLinearVelocityMps), params.rollingRegularizationMps);
        return
            rollingWheelOmegaRadps +
            ((slipRatio * referenceVelocityMps) * params.invWheelRadiusM);
    }

    inline float ResolveVelocityTargetWheelCommandSpeedRadps(
        float currentWheelSpeedRadps,
        float targetWheelSpeedRadps) noexcept
    {
        if (std::isfinite(currentWheelSpeedRadps))
        {
            return currentWheelSpeedRadps;
        }
        if (std::isfinite(targetWheelSpeedRadps))
        {
            return targetWheelSpeedRadps;
        }
        return 0.0f;
    }

    struct ModeTransition
    {
        float ad00 = 1.0f;
        float ad01 = 0.0f;
        float ad10 = 0.0f;
        float ad11 = 1.0f;
        float bd0 = 0.0f;
        float gamma0Wheel = 0.0f;
        float gamma1Body = 0.0f;
        float gamma1Wheel = 0.0f;
        float a10 = 0.0f;
        float a11 = 0.0f;
        float b1 = 0.0f;
        bool valid = false;
    };

    struct VelocityTargetExactSolution
    {
        ControlInput control{};
        float leftSlipRatio = 0.0f;
        float rightSlipRatio = 0.0f;
        float leftWheelSpeedRadps = 0.0f;
        float rightWheelSpeedRadps = 0.0f;
        float leftRollingWheelSpeedRadps = 0.0f;
        float rightRollingWheelSpeedRadps = 0.0f;
        float leftWheelTorqueNm = 0.0f;
        float rightWheelTorqueNm = 0.0f;
        float leftWheelAccelRadps2 = 0.0f;
        float rightWheelAccelRadps2 = 0.0f;
        float leftContactForceN = 0.0f;
        float rightContactForceN = 0.0f;
        float leftContactTorqueNm = 0.0f;
        float rightContactTorqueNm = 0.0f;
        bool valid = false;
    };

    enum class VelocityTargetOperatingMode : uint8_t
    {
        Static = 0,
        Rolling = 1,
        TractionLoss = 2,
        HighSlipTurn = 3,
    };

    inline VelocityTargetOperatingMode ResolveVelocityTargetMotionMode(
        const PlantModel::StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params) noexcept
    {
        const PlantModel::StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float speedNormMps =
            ComputeSpeedNormMps(
                operatingState(VehicleState::kU),
                0.0f,
                operatingState(VehicleState::kR),
                operatingState(VehicleState::kOmegaL),
                operatingState(VehicleState::kOmegaR),
                params);
        if (speedNormMps < params.stopExitSpeedMps)
        {
            return VelocityTargetOperatingMode::Static;
        }

        const float turnBankSpeedMps = std::fabs(params.halfTrackWidthM * targetYawRateRadps);
        const float forwardReferenceMps =
            (std::max)(std::fabs(targetForwardVelocityMps), params.rollingRegularizationMps);
        const float currentForwardReferenceMps =
            (std::max)(std::fabs(operatingState(VehicleState::kU)), params.rollingRegularizationMps);
        const float highSlipReferenceMps = (std::min)(forwardReferenceMps, currentForwardReferenceMps);
        if ((turnBankSpeedMps > (0.35f * highSlipReferenceMps)) &&
            (std::fabs(targetYawRateRadps) > params.stopExitYawRateRadps))
        {
            return VelocityTargetOperatingMode::HighSlipTurn;
        }

        return VelocityTargetOperatingMode::Rolling;
    }

    inline VelocityTargetOperatingMode ResolveVelocityTargetOperatingMode(
        const PlantModel::StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const DriveCommandSolution& referenceSolution,
        const PreparedParams& params) noexcept
    {
        if (referenceSolution.tractionLimited)
        {
            return VelocityTargetOperatingMode::TractionLoss;
        }

        return ResolveVelocityTargetMotionMode(currentState, targetForwardVelocityMps, targetYawRateRadps, params);
    }

    inline ModeTransition BuildModeTransition(
        float a00,
        float a01,
        float a10,
        float a11,
        float b1,
        float responseTimeS) noexcept
    {
        ModeTransition transition{};
        transition.a10 = a10;
        transition.a11 = a11;
        transition.b1 = b1;

        if (!(std::isfinite(a00) &&
            std::isfinite(a01) &&
            std::isfinite(a10) &&
            std::isfinite(a11) &&
            std::isfinite(b1) &&
            std::isfinite(responseTimeS) &&
            (responseTimeS > 0.0f)))
        {
            return transition;
        }

        const float traceHalf = 0.5f * (a00 + a11);
        const float centered00 = a00 - traceHalf;
        const float centered11 = a11 - traceHalf;
        const float discriminant = (centered00 * centered00) + (a01 * a10);
        const float scaledTimeS = responseTimeS;
        const float expTrace = std::exp(traceHalf * scaledTimeS);

        float exp00 = 1.0f;
        float exp01 = 0.0f;
        float exp10 = 0.0f;
        float exp11 = 1.0f;
        if (std::fabs(discriminant) <= 1.0e-8f)
        {
            exp00 = expTrace * (1.0f + (centered00 * scaledTimeS));
            exp01 = expTrace * (a01 * scaledTimeS);
            exp10 = expTrace * (a10 * scaledTimeS);
            exp11 = expTrace * (1.0f + (centered11 * scaledTimeS));
        }
        else if (discriminant > 0.0f)
        {
            const float root = std::sqrt(discriminant);
            const float coshTerm = std::cosh(root * scaledTimeS);
            const float sinhOverRoot = std::sinh(root * scaledTimeS) / root;
            exp00 = expTrace * (coshTerm + (sinhOverRoot * centered00));
            exp01 = expTrace * (sinhOverRoot * a01);
            exp10 = expTrace * (sinhOverRoot * a10);
            exp11 = expTrace * (coshTerm + (sinhOverRoot * centered11));
        }
        else
        {
            const float root = std::sqrt(-discriminant);
            const float cosTerm = std::cos(root * scaledTimeS);
            const float sinOverRoot = std::sin(root * scaledTimeS) / root;
            exp00 = expTrace * (cosTerm + (sinOverRoot * centered00));
            exp01 = expTrace * (sinOverRoot * a01);
            exp10 = expTrace * (sinOverRoot * a10);
            exp11 = expTrace * (cosTerm + (sinOverRoot * centered11));
        }

        transition.ad00 = exp00;
        transition.ad01 = exp01;
        transition.ad10 = exp10;
        transition.ad11 = exp11;

        double gamma0Body = 0.0;
        double gamma0Wheel = 0.0;
        double gamma1Body = 0.0;
        double gamma1Wheel = 0.0;
        const double detA = static_cast<double>((a00 * a11) - (a01 * a10));
        if (std::fabs(detA) > 1.0e-10)
        {
            const double invDetA = 1.0 / detA;
            const auto ApplyInvA =
                [a00, a01, a10, a11, invDetA](double v0, double v1, double& out0, double& out1) noexcept
                {
                    out0 = (((static_cast<double>(a11) * v0) - (static_cast<double>(a01) * v1)) * invDetA);
                    out1 = (((-static_cast<double>(a10) * v0) + (static_cast<double>(a00) * v1)) * invDetA);
                };

            const double deltaB0 = static_cast<double>(exp01) * static_cast<double>(b1);
            const double deltaB1 = static_cast<double>(exp11 - 1.0f) * static_cast<double>(b1);
            ApplyInvA(deltaB0, deltaB1, gamma0Body, gamma0Wheel);

            const double affineDeltaB0 =
                (static_cast<double>(exp01) - (static_cast<double>(a01) * static_cast<double>(responseTimeS))) *
                static_cast<double>(b1);
            const double affineDeltaB1 =
                (static_cast<double>(exp11 - 1.0f) -
                    (static_cast<double>(a11) * static_cast<double>(responseTimeS))) *
                static_cast<double>(b1);
            double affineMid0 = 0.0;
            double affineMid1 = 0.0;
            ApplyInvA(affineDeltaB0, affineDeltaB1, affineMid0, affineMid1);
            ApplyInvA(affineMid0, affineMid1, gamma1Body, gamma1Wheel);
        }
        else
        {
            const double timeS = static_cast<double>(responseTimeS);
            const double time2 = timeS * timeS;
            const double time3 = time2 * timeS;
            const double time4 = time3 * timeS;
            const double time5 = time4 * timeS;
            const double b0 = 0.0;
            const double b1d = static_cast<double>(b1);
            const double ab0 = static_cast<double>(a01) * b1d;
            const double ab1 = static_cast<double>(a11) * b1d;
            const double a2b0 = (static_cast<double>(a00) * ab0) + (static_cast<double>(a01) * ab1);
            const double a2b1 = (static_cast<double>(a10) * ab0) + (static_cast<double>(a11) * ab1);
            const double a3b0 = (static_cast<double>(a00) * a2b0) + (static_cast<double>(a01) * a2b1);
            const double a3b1 = (static_cast<double>(a10) * a2b0) + (static_cast<double>(a11) * a2b1);

            gamma0Body =
                (timeS * b0) +
                (0.5 * time2 * ab0) +
                ((time3 / 6.0) * a2b0) +
                ((time4 / 24.0) * a3b0);
            gamma0Wheel =
                (timeS * b1d) +
                (0.5 * time2 * ab1) +
                ((time3 / 6.0) * a2b1) +
                ((time4 / 24.0) * a3b1);
            gamma1Body =
                ((0.5 * time2) * b0) +
                ((time3 / 6.0) * ab0) +
                ((time4 / 24.0) * a2b0) +
                ((time5 / 120.0) * a3b0);
            gamma1Wheel =
                ((0.5 * time2) * b1d) +
                ((time3 / 6.0) * ab1) +
                ((time4 / 24.0) * a2b1) +
                ((time5 / 120.0) * a3b1);
        }

        transition.bd0 = static_cast<float>(gamma0Body);
        transition.gamma0Wheel = static_cast<float>(gamma0Wheel);
        transition.gamma1Body = static_cast<float>(gamma1Body);
        transition.gamma1Wheel = static_cast<float>(gamma1Wheel);
        transition.valid =
            std::isfinite(transition.ad00) &&
            std::isfinite(transition.ad01) &&
            std::isfinite(transition.ad10) &&
            std::isfinite(transition.ad11) &&
            std::isfinite(transition.bd0) &&
            std::isfinite(transition.gamma0Wheel) &&
            std::isfinite(transition.gamma1Body) &&
            std::isfinite(transition.gamma1Wheel) &&
            std::isfinite(transition.a10) &&
            std::isfinite(transition.a11) &&
            std::isfinite(transition.b1);
        return transition;
    }

    inline bool ResolveModeNetDriveTorqueNm(
        const ModeTransition& transition,
        float currentBodyRate,
        float currentWheelRate,
        float targetBodyRate,
        float& netDriveTorqueNm,
        float& wheelAccelRadps2) noexcept
    {
        netDriveTorqueNm = 0.0f;
        wheelAccelRadps2 = 0.0f;

        if (!transition.valid || (std::fabs(transition.bd0) <= 1.0e-6f))
        {
            return false;
        }

        if (!(std::isfinite(currentBodyRate) &&
            std::isfinite(currentWheelRate) &&
            std::isfinite(targetBodyRate)))
        {
            return false;
        }

        netDriveTorqueNm =
            (targetBodyRate -
                (transition.ad00 * currentBodyRate) -
                (transition.ad01 * currentWheelRate)) /
            transition.bd0;
        wheelAccelRadps2 =
            (transition.a10 * currentBodyRate) +
            (transition.a11 * currentWheelRate) +
            (transition.b1 * netDriveTorqueNm);
        return std::isfinite(netDriveTorqueNm) && std::isfinite(wheelAccelRadps2);
    }

    inline bool ResolveModeAffineNetDriveTorqueNm(
        const ModeTransition& transition,
        float currentBodyRate,
        float currentWheelRate,
        float targetBodyRate,
        float targetWheelRate,
        float& netDriveTorqueNm,
        float& wheelAccelRadps2) noexcept
    {
        netDriveTorqueNm = 0.0f;
        wheelAccelRadps2 = 0.0f;

        if (!(transition.valid &&
            std::isfinite(currentBodyRate) &&
            std::isfinite(currentWheelRate) &&
            std::isfinite(targetBodyRate) &&
            std::isfinite(targetWheelRate)))
        {
            return false;
        }

        const double detInputMap =
            (static_cast<double>(transition.bd0) * static_cast<double>(transition.gamma1Wheel)) -
            (static_cast<double>(transition.gamma1Body) * static_cast<double>(transition.gamma0Wheel));
        if (std::fabs(detInputMap) <= 1.0e-10)
        {
            return ResolveModeNetDriveTorqueNm(
                transition,
                currentBodyRate,
                currentWheelRate,
                targetBodyRate,
                netDriveTorqueNm,
                wheelAccelRadps2);
        }

        const double rhsBody =
            static_cast<double>(targetBodyRate) -
            ((static_cast<double>(transition.ad00) * static_cast<double>(currentBodyRate)) +
                (static_cast<double>(transition.ad01) * static_cast<double>(currentWheelRate)));
        const double rhsWheel =
            static_cast<double>(targetWheelRate) -
            ((static_cast<double>(transition.ad10) * static_cast<double>(currentBodyRate)) +
                (static_cast<double>(transition.ad11) * static_cast<double>(currentWheelRate)));

        netDriveTorqueNm =
            static_cast<float>(
                ((rhsBody * static_cast<double>(transition.gamma1Wheel)) -
                    (static_cast<double>(transition.gamma1Body) * rhsWheel)) /
                detInputMap);
        wheelAccelRadps2 =
            (transition.a10 * currentBodyRate) +
            (transition.a11 * currentWheelRate) +
            (transition.b1 * netDriveTorqueNm);
        return std::isfinite(netDriveTorqueNm) && std::isfinite(wheelAccelRadps2);
    }

    inline float ResolveVelocityTargetReferenceSpeedMps(
        float currentValue,
        float targetValue,
        float minimumReferenceMps) noexcept
    {
        return
            (std::max)(
                minimumReferenceMps,
                (std::max)(std::fabs(currentValue), std::fabs(targetValue)));
    }

    inline float ResolveVelocityTargetYawRestoringMomentSlopeNmPerRadps(
        float forwardReferenceSpeedMps,
        const PreparedParams& params) noexcept
    {
        if (!(std::isfinite(forwardReferenceSpeedMps) &&
            (forwardReferenceSpeedMps > 0.0f) &&
            std::isfinite(params.longitudinalOffsetM) &&
            std::isfinite(params.frontCorneringStiffnessAxleNPerRad) &&
            std::isfinite(params.rearCorneringStiffnessAxleNPerRad)))
        {
            return 0.0f;
        }

        return
            ((params.frontCorneringStiffnessAxleNPerRad + params.rearCorneringStiffnessAxleNPerRad) *
                params.longitudinalOffsetM *
                params.longitudinalOffsetM) /
            forwardReferenceSpeedMps;
    }

    inline float ResolveVelocityTargetModeYawReferenceSpeedMps(
        VelocityTargetOperatingMode operatingMode,
        float currentForwardVelocityMps,
        float currentYawRateRadps,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params) noexcept
    {
        const float baseReferenceSpeedMps =
            ResolveVelocityTargetReferenceSpeedMps(
                currentForwardVelocityMps,
                targetForwardVelocityMps,
                params.rollingRegularizationMps);
        if (operatingMode != VelocityTargetOperatingMode::HighSlipTurn)
        {
            return baseReferenceSpeedMps;
        }

        const float currentLeftBankVelocityMps =
            currentForwardVelocityMps + (params.halfTrackWidthM * currentYawRateRadps);
        const float currentRightBankVelocityMps =
            currentForwardVelocityMps - (params.halfTrackWidthM * currentYawRateRadps);
        const float targetLeftBankVelocityMps =
            targetForwardVelocityMps + (params.halfTrackWidthM * targetYawRateRadps);
        const float targetRightBankVelocityMps =
            targetForwardVelocityMps - (params.halfTrackWidthM * targetYawRateRadps);
        return
            (std::max)(
                baseReferenceSpeedMps,
                (std::max)(
                    (std::max)(std::fabs(currentLeftBankVelocityMps), std::fabs(currentRightBankVelocityMps)),
                    (std::max)(std::fabs(targetLeftBankVelocityMps), std::fabs(targetRightBankVelocityMps))));
    }

    inline float ResolveVelocityTargetModeTrackWidthM(
        VelocityTargetOperatingMode operatingMode,
        float currentForwardVelocityMps,
        float currentYawRateRadps,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params) noexcept
    {
        const float physicalTrackWidthM = ResolvePhysicalTrackWidthM(params);
        const float currentMotionTrackWidthM =
            ResolveMotionTrackWidthM(currentForwardVelocityMps, currentYawRateRadps, params);
        const float targetMotionTrackWidthM =
            ResolveMotionTrackWidthM(targetForwardVelocityMps, targetYawRateRadps, params);
        const float rollingTrackWidthM =
            (std::max)(
                physicalTrackWidthM,
                (std::max)(currentMotionTrackWidthM, targetMotionTrackWidthM));

        switch (operatingMode)
        {
        case VelocityTargetOperatingMode::Static:
            return physicalTrackWidthM;

        case VelocityTargetOperatingMode::HighSlipTurn:
        case VelocityTargetOperatingMode::Rolling:
            return rollingTrackWidthM;

        case VelocityTargetOperatingMode::TractionLoss:
        default:
            return targetMotionTrackWidthM;
        }
    }

    inline float ResolveSlipBearingWheelSpeedRadpsForReference(
        float rollingWheelLinearVelocityMps,
        float rollingWheelOmegaRadps,
        float contactForceN,
        float referenceVelocityMps,
        const PreparedParams& params) noexcept
    {
        const float slipRatio = contactForceN * params.invLongitudinalTireStiffnessN;
        const float resolvedReferenceVelocityMps =
            (std::max)(
                params.rollingRegularizationMps,
                (std::max)(std::fabs(referenceVelocityMps), std::fabs(rollingWheelLinearVelocityMps)));
        return
            rollingWheelOmegaRadps +
            ((slipRatio * resolvedReferenceVelocityMps) * params.invWheelRadiusM);
    }

    inline bool ResolveVelocityTargetExactControl(
        const PlantModel::StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float targetLongitudinalAccelMps2,
        float targetYawAccelRadps2,
        float responseTimeS,
        VelocityTargetOperatingMode operatingMode,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        bool useObservedWheelState,
        VelocityTargetExactSolution& solution) noexcept
    {
        solution = VelocityTargetExactSolution{};
        solution.control.fanDutyCycle = Clamp01(fanDutyCycle);
        solution.control.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);

        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        if (!(resolvedResponseTimeS > 0.0f) ||
            !(params.longitudinalMassKg > 0.0f) ||
            !(params.yawInertiaKgM2 > 0.0f) ||
            !(params.wheelInertiaKgM2 > 0.0f) ||
            !(params.wheelRadiusM > 0.0f) ||
            !(params.longitudinalTireStiffnessN > 0.0f) ||
            !(params.halfTrackWidthM > 0.0f))
        {
            return false;
        }

        const PlantModel::StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float currentForwardVelocityMps = operatingState(VehicleState::kU);
        const float currentYawRateRadps = operatingState(VehicleState::kR);
        const float currentTrackWidthM =
            ResolveMotionTrackWidthM(currentForwardVelocityMps, currentYawRateRadps, params);
        const float currentHalfTrackWidthM = 0.5f * currentTrackWidthM;
        const float targetTrackWidthM =
            ResolveMotionTrackWidthM(targetForwardVelocityMps, targetYawRateRadps, params);
        const float targetHalfTrackWidthM = 0.5f * targetTrackWidthM;
        const float modeTrackWidthM =
            ResolveVelocityTargetModeTrackWidthM(
                operatingMode,
                currentForwardVelocityMps,
                currentYawRateRadps,
                targetForwardVelocityMps,
                targetYawRateRadps,
                params);
        const float modeHalfTrackWidthM = 0.5f * modeTrackWidthM;

        const float forwardReferenceSpeedMps =
            ResolveVelocityTargetReferenceSpeedMps(
                currentForwardVelocityMps,
                targetForwardVelocityMps,
                params.rollingRegularizationMps);
        const float yawReferenceSpeedMps =
            ResolveVelocityTargetModeYawReferenceSpeedMps(
                operatingMode,
                currentForwardVelocityMps,
                currentYawRateRadps,
                targetForwardVelocityMps,
                targetYawRateRadps,
                params);
        const float yawRestoringSlopeNmPerRadps =
            ResolveVelocityTargetYawRestoringMomentSlopeNmPerRadps(yawReferenceSpeedMps, params);
        const float totalYawDampingNmPerRadps = params.yawDampingNmPerRadps + yawRestoringSlopeNmPerRadps;
        const float currentLeftRollingWheelSpeedRadps =
            (currentForwardVelocityMps + (currentHalfTrackWidthM * currentYawRateRadps)) * params.invWheelRadiusM;
        const float currentRightRollingWheelSpeedRadps =
            (currentForwardVelocityMps - (currentHalfTrackWidthM * currentYawRateRadps)) * params.invWheelRadiusM;
        const float targetLeftRollingWheelSpeedRadps =
            (targetForwardVelocityMps + (targetHalfTrackWidthM * targetYawRateRadps)) * params.invWheelRadiusM;
        const float targetRightRollingWheelSpeedRadps =
            (targetForwardVelocityMps - (targetHalfTrackWidthM * targetYawRateRadps)) * params.invWheelRadiusM;
        const float leftReferenceSpeedMps =
            (std::max)(
                forwardReferenceSpeedMps,
                (std::max)(
                    std::fabs(currentForwardVelocityMps + (currentHalfTrackWidthM * currentYawRateRadps)),
                    std::fabs(targetForwardVelocityMps + (targetHalfTrackWidthM * targetYawRateRadps))));
        const float rightReferenceSpeedMps =
            (std::max)(
                forwardReferenceSpeedMps,
                (std::max)(
                    std::fabs(currentForwardVelocityMps - (currentHalfTrackWidthM * currentYawRateRadps)),
                    std::fabs(targetForwardVelocityMps - (targetHalfTrackWidthM * targetYawRateRadps))));
        float currentLeftWheelSpeedRadps = currentLeftRollingWheelSpeedRadps;
        float currentRightWheelSpeedRadps = currentRightRollingWheelSpeedRadps;
        if (useObservedWheelState &&
            std::isfinite(operatingState(VehicleState::kOmegaL)) &&
            std::isfinite(operatingState(VehicleState::kOmegaR)))
        {
            currentLeftWheelSpeedRadps = operatingState(VehicleState::kOmegaL);
            currentRightWheelSpeedRadps = operatingState(VehicleState::kOmegaR);
        }
        else
        {
            const float currentDifferentialForceN =
                (operatingMode == VelocityTargetOperatingMode::Static) ?
                0.0f :
                ((totalYawDampingNmPerRadps * currentYawRateRadps) * params.invTrackWidthM);
            currentLeftWheelSpeedRadps =
                ResolveSlipBearingWheelSpeedRadpsForReference(
                    currentForwardVelocityMps + (currentHalfTrackWidthM * currentYawRateRadps),
                    currentLeftRollingWheelSpeedRadps,
                    currentDifferentialForceN,
                    leftReferenceSpeedMps,
                    params);
            currentRightWheelSpeedRadps =
                ResolveSlipBearingWheelSpeedRadpsForReference(
                    currentForwardVelocityMps - (currentHalfTrackWidthM * currentYawRateRadps),
                    currentRightRollingWheelSpeedRadps,
                    -currentDifferentialForceN,
                    rightReferenceSpeedMps,
                    params);
        }
        const float currentCommonWheelSpeedRadps = 0.5f * (currentLeftWheelSpeedRadps + currentRightWheelSpeedRadps);
        const float currentDifferentialWheelSpeedRadps = 0.5f * (currentLeftWheelSpeedRadps - currentRightWheelSpeedRadps);

        const float commonForceWheelGain = params.longitudinalTireStiffnessN * params.wheelRadiusM / forwardReferenceSpeedMps;
        const float commonForceBodyGain = params.longitudinalTireStiffnessN / forwardReferenceSpeedMps;
        const float commonA00 = -(2.0f * commonForceBodyGain * params.invLongitudinalMassKg);
        const float commonA01 = 2.0f * commonForceWheelGain * params.invLongitudinalMassKg;
        const float commonA10 = commonForceWheelGain * params.invWheelInertiaKgM2;
        const float commonA11 =
            -(params.wheelRadiusM * commonForceWheelGain * params.invWheelInertiaKgM2);
        const ModeTransition commonTransition =
            BuildModeTransition(
                commonA00,
                commonA01,
                commonA10,
                commonA11,
                params.invWheelInertiaKgM2,
                resolvedResponseTimeS);

        const float diffForceWheelGain = params.longitudinalTireStiffnessN * params.wheelRadiusM / yawReferenceSpeedMps;
        const float diffForceBodyGain = params.longitudinalTireStiffnessN * modeHalfTrackWidthM / yawReferenceSpeedMps;
        const float diffBodyGain =
            (2.0f * modeHalfTrackWidthM * diffForceWheelGain) * params.invYawInertiaKgM2;
        const float diffYawSlope =
            ((2.0f * modeHalfTrackWidthM * diffForceBodyGain) * params.invYawInertiaKgM2) +
            (totalYawDampingNmPerRadps * params.invYawInertiaKgM2);
        const float diffA00 = -diffYawSlope;
        const float diffA01 = diffBodyGain;
        const float diffA10 = diffForceBodyGain * params.invWheelInertiaKgM2;
        const float diffA11 =
            -(params.wheelRadiusM * diffForceWheelGain * params.invWheelInertiaKgM2);
        const ModeTransition diffTransition =
            BuildModeTransition(
                diffA00,
                diffA01,
                diffA10,
                diffA11,
                params.invWheelInertiaKgM2,
                resolvedResponseTimeS);

        const float targetYawReferenceSpeedMps =
            ResolveVelocityTargetModeYawReferenceSpeedMps(
                operatingMode,
                targetForwardVelocityMps,
                targetYawRateRadps,
                targetForwardVelocityMps,
                targetYawRateRadps,
                params);
        const float targetYawRestoringSlopeNmPerRadps =
            ResolveVelocityTargetYawRestoringMomentSlopeNmPerRadps(targetYawReferenceSpeedMps, params);
        const float targetTotalYawDampingNmPerRadps =
            params.yawDampingNmPerRadps + targetYawRestoringSlopeNmPerRadps;
        const float targetCommonForcePerBankN = 0.5f * params.longitudinalMassKg * targetLongitudinalAccelMps2;
        const float targetLongitudinalYawMomentNm =
            (params.yawInertiaKgM2 * targetYawAccelRadps2) +
            (targetTotalYawDampingNmPerRadps * targetYawRateRadps);
        const float targetDifferentialForceN = targetLongitudinalYawMomentNm * params.invTrackWidthM;
        const float targetLeftWheelSpeedRadps =
            ResolveSlipBearingWheelSpeedRadpsForReference(
                targetForwardVelocityMps + (targetHalfTrackWidthM * targetYawRateRadps),
                targetLeftRollingWheelSpeedRadps,
                targetCommonForcePerBankN + targetDifferentialForceN,
                leftReferenceSpeedMps,
                params);
        const float targetRightWheelSpeedRadps =
            ResolveSlipBearingWheelSpeedRadpsForReference(
                targetForwardVelocityMps - (targetHalfTrackWidthM * targetYawRateRadps),
                targetRightRollingWheelSpeedRadps,
                targetCommonForcePerBankN - targetDifferentialForceN,
                rightReferenceSpeedMps,
                params);
        const float targetCommonWheelSpeedRadps = 0.5f * (targetLeftWheelSpeedRadps + targetRightWheelSpeedRadps);
        const float targetDifferentialWheelSpeedRadps = 0.5f * (targetLeftWheelSpeedRadps - targetRightWheelSpeedRadps);

        float commonNetDriveTorqueNm = 0.0f;
        float commonWheelAccelRadps2 = 0.0f;
        float diffNetDriveTorqueNm = 0.0f;
        float diffWheelAccelRadps2 = 0.0f;
        if (!ResolveModeAffineNetDriveTorqueNm(
                commonTransition,
                currentForwardVelocityMps,
                currentCommonWheelSpeedRadps,
                targetForwardVelocityMps,
                targetCommonWheelSpeedRadps,
                commonNetDriveTorqueNm,
                commonWheelAccelRadps2) ||
            !ResolveModeAffineNetDriveTorqueNm(
                diffTransition,
                currentYawRateRadps,
                currentDifferentialWheelSpeedRadps,
                targetYawRateRadps,
                targetDifferentialWheelSpeedRadps,
                diffNetDriveTorqueNm,
                diffWheelAccelRadps2))
        {
            return false;
        }

        const float leftWheelSpeedRadps = currentCommonWheelSpeedRadps + currentDifferentialWheelSpeedRadps;
        const float rightWheelSpeedRadps = currentCommonWheelSpeedRadps - currentDifferentialWheelSpeedRadps;
        const float leftWheelAccelRadps2 = commonWheelAccelRadps2 + diffWheelAccelRadps2;
        const float rightWheelAccelRadps2 = commonWheelAccelRadps2 - diffWheelAccelRadps2;
        const float commonContactForceN =
            (params.longitudinalTireStiffnessN *
                ((params.wheelRadiusM * currentCommonWheelSpeedRadps) - currentForwardVelocityMps)) /
            forwardReferenceSpeedMps;
        const float differentialContactForceN =
            (params.longitudinalTireStiffnessN *
                ((params.wheelRadiusM * currentDifferentialWheelSpeedRadps) -
                    (params.halfTrackWidthM * currentYawRateRadps))) /
            yawReferenceSpeedMps;
        const float leftContactForceN = commonContactForceN + differentialContactForceN;
        const float rightContactForceN = commonContactForceN - differentialContactForceN;
        const float leftContactTorqueNm = params.wheelRadiusM * leftContactForceN;
        const float rightContactTorqueNm = params.wheelRadiusM * rightContactForceN;
        const float leftNetWheelTorqueNm = commonNetDriveTorqueNm + diffNetDriveTorqueNm;
        const float rightNetWheelTorqueNm = commonNetDriveTorqueNm - diffNetDriveTorqueNm;
        const float leftWheelTorqueNm =
            leftNetWheelTorqueNm + DriveFrictionTorqueFast(leftWheelSpeedRadps, leftNetWheelTorqueNm, params);
        const float rightWheelTorqueNm =
            rightNetWheelTorqueNm + DriveFrictionTorqueFast(rightWheelSpeedRadps, rightNetWheelTorqueNm, params);

        solution.leftRollingWheelSpeedRadps = currentLeftRollingWheelSpeedRadps;
        solution.rightRollingWheelSpeedRadps = currentRightRollingWheelSpeedRadps;
        solution.leftWheelSpeedRadps = leftWheelSpeedRadps;
        solution.rightWheelSpeedRadps = rightWheelSpeedRadps;
        solution.leftWheelAccelRadps2 = leftWheelAccelRadps2;
        solution.rightWheelAccelRadps2 = rightWheelAccelRadps2;
        solution.leftContactForceN = leftContactForceN;
        solution.rightContactForceN = rightContactForceN;
        solution.leftContactTorqueNm = leftContactTorqueNm;
        solution.rightContactTorqueNm = rightContactTorqueNm;
        solution.leftWheelTorqueNm = leftWheelTorqueNm;
        solution.rightWheelTorqueNm = rightWheelTorqueNm;
        solution.leftSlipRatio = leftContactForceN * params.invLongitudinalTireStiffnessN;
        solution.rightSlipRatio = rightContactForceN * params.invLongitudinalTireStiffnessN;
        solution.control.leftMotorCommand =
            DriveCommandFromTorqueFast(
                leftWheelTorqueNm,
                leftWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);
        solution.control.rightMotorCommand =
            DriveCommandFromTorqueFast(
                rightWheelTorqueNm,
                rightWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);
        solution.valid =
            std::isfinite(solution.control.leftMotorCommand) &&
            std::isfinite(solution.control.rightMotorCommand);
        return solution.valid;
    }

    inline void ApplyVelocityTargetExactControl(
        const PlantModel::StateVector& currentState,
        float resolvedTargetForwardVelocityMps,
        float resolvedTargetYawRateRadps,
        float resolvedTargetLongitudinalAccelMps2,
        float resolvedTargetYawAccelRadps2,
        float responseTimeS,
        VelocityTargetOperatingMode operatingMode,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        bool useObservedWheelState,
        DriveCommandSolution& solution) noexcept
    {
        VelocityTargetExactSolution exactSolution{};
        if (!ResolveVelocityTargetExactControl(
                currentState,
                resolvedTargetForwardVelocityMps,
                resolvedTargetYawRateRadps,
                resolvedTargetLongitudinalAccelMps2,
                resolvedTargetYawAccelRadps2,
                responseTimeS,
                operatingMode,
                params,
                fanDutyCycle,
                batteryVoltageV,
                useObservedWheelState,
                exactSolution))
        {
            return;
        }

        solution.control = exactSolution.control;
        solution.leftSlipRatio = exactSolution.leftSlipRatio;
        solution.rightSlipRatio = exactSolution.rightSlipRatio;
        solution.leftWheelSpeedRadps = exactSolution.leftWheelSpeedRadps;
        solution.rightWheelSpeedRadps = exactSolution.rightWheelSpeedRadps;
        solution.leftRollingWheelSpeedRadps = exactSolution.leftRollingWheelSpeedRadps;
        solution.rightRollingWheelSpeedRadps = exactSolution.rightRollingWheelSpeedRadps;
        solution.leftWheelTorqueNm = exactSolution.leftWheelTorqueNm;
        solution.rightWheelTorqueNm = exactSolution.rightWheelTorqueNm;
        solution.leftWheelAccelRadps2 = exactSolution.leftWheelAccelRadps2;
        solution.rightWheelAccelRadps2 = exactSolution.rightWheelAccelRadps2;
        solution.leftContactForceN = exactSolution.leftContactForceN;
        solution.rightContactForceN = exactSolution.rightContactForceN;
        solution.leftContactTorqueNm = exactSolution.leftContactTorqueNm;
        solution.rightContactTorqueNm = exactSolution.rightContactTorqueNm;
    }

    inline void ApplyVelocityTargetYawCommandAssist(
        DriveCommandSolution& solution,
        float yawCommandScale) noexcept
    {
        if (!(std::isfinite(yawCommandScale) && (yawCommandScale > 1.0f)))
        {
            return;
        }

        const float commonCommand = 0.5f * (solution.control.leftMotorCommand + solution.control.rightMotorCommand);
        const float differentialCommand =
            0.5f * (solution.control.leftMotorCommand - solution.control.rightMotorCommand);
        solution.control.leftMotorCommand = ClampUnit(commonCommand + (yawCommandScale * differentialCommand));
        solution.control.rightMotorCommand = ClampUnit(commonCommand - (yawCommandScale * differentialCommand));
    }

    inline void ApplyVelocityTargetYawCommandBias(
        DriveCommandSolution& solution,
        float yawCommandBias) noexcept
    {
        if (!std::isfinite(yawCommandBias) || (std::fabs(yawCommandBias) <= 1.0e-6f))
        {
            return;
        }

        const float commonCommand = 0.5f * (solution.control.leftMotorCommand + solution.control.rightMotorCommand);
        const float differentialCommand =
            0.5f * (solution.control.leftMotorCommand - solution.control.rightMotorCommand) +
            yawCommandBias;
        solution.control.leftMotorCommand = ClampUnit(commonCommand + differentialCommand);
        solution.control.rightMotorCommand = ClampUnit(commonCommand - differentialCommand);
    }

    inline void ApplyVelocityTargetCommonCommandBias(
        DriveCommandSolution& solution,
        float commonCommandBias) noexcept
    {
        if (!std::isfinite(commonCommandBias) || (std::fabs(commonCommandBias) <= 1.0e-6f))
        {
            return;
        }

        const float commonCommand =
            0.5f * (solution.control.leftMotorCommand + solution.control.rightMotorCommand) +
            commonCommandBias;
        const float differentialCommand =
            0.5f * (solution.control.leftMotorCommand - solution.control.rightMotorCommand);
        solution.control.leftMotorCommand = ClampUnit(commonCommand + differentialCommand);
        solution.control.rightMotorCommand = ClampUnit(commonCommand - differentialCommand);
    }

    template <typename SolveBase>
    inline DriveCommandSolution SolveVelocityTargetFeedforward(
        const PlantModel& plant,
        const PlantModel::StateVector& operatingState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS,
        bool useObservedWheelState,
        SolveBase&& solveBase) noexcept
    {
        (void)plant;
        (void)fanDutyCycle;
        (void)batteryVoltageV;
        (void)useObservedWheelState;

        const float currentForwardVelocityMps = operatingState(VehicleState::kU);
        const float currentYawRateRadps = operatingState(VehicleState::kR);
        const VelocityTargetOperatingMode motionMode =
            ResolveVelocityTargetMotionMode(
                operatingState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                params);
        const float targetMotionTrackWidthM =
            ResolveMotionTrackWidthM(targetForwardVelocityMps, targetYawRateRadps, params);
        const float resolvedTargetForwardVelocityMps = targetForwardVelocityMps;

        float desiredLongitudinalAccelMps2 = 0.0f;
        float desiredYawAccelRadps2 = 0.0f;
        plant.resolveVelocityTargetAccelerations(
            currentForwardVelocityMps,
            resolvedTargetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            responseTimeS,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
        const float targetForwardReferenceMps =
            ResolveVelocityTargetReferenceSpeedMps(
                currentForwardVelocityMps,
                resolvedTargetForwardVelocityMps,
                params.rollingRegularizationMps);
        const float targetTurnBankSpeedMps = std::fabs(0.5f * targetMotionTrackWidthM * targetYawRateRadps);
        const float turnSeverity =
            (targetForwardReferenceMps > 0.0f) ?
            Clamp01(targetTurnBankSpeedMps / targetForwardReferenceMps) :
            0.0f;
        const float currentYawReferenceMps =
            ResolveVelocityTargetReferenceSpeedMps(
                currentForwardVelocityMps,
                currentForwardVelocityMps,
                params.rollingRegularizationMps);
        const float rollingYawLeadScale =
            (params.trackWidthM > 0.0f) ?
            (std::clamp)(
                (targetMotionTrackWidthM * targetMotionTrackWidthM) /
                    (params.trackWidthM * params.trackWidthM),
                1.0f,
                1.5f) :
            1.0f;
        const float trackWidthRatio =
            (params.trackWidthM > 0.0f) ?
            (std::clamp)(targetMotionTrackWidthM / params.trackWidthM, 1.0f, 1.35f) :
            1.0f;
        const float rollingModeYawScale = rollingYawLeadScale * rollingYawLeadScale;
        float yawAssistScale = 1.0f;
        if (currentYawReferenceMps > 0.0f)
        {
            yawAssistScale =
                (std::clamp)(targetForwardReferenceMps / currentYawReferenceMps, 1.0f, 4.0f);
        }

        float commandedYawAccelRadps2 = desiredYawAccelRadps2;
        float yawCommandScale = 1.0f;
        switch (motionMode)
        {
        case VelocityTargetOperatingMode::Static:
            commandedYawAccelRadps2 *= yawAssistScale;
            yawCommandScale = yawAssistScale;
            break;

        case VelocityTargetOperatingMode::HighSlipTurn:
            desiredLongitudinalAccelMps2 *= (1.0f - (0.35f * turnSeverity));
            commandedYawAccelRadps2 *= (std::max)(rollingYawLeadScale, yawAssistScale);
            yawCommandScale =
                (std::max)(trackWidthRatio, yawAssistScale);
            break;

        case VelocityTargetOperatingMode::Rolling:
            desiredLongitudinalAccelMps2 *= (1.0f - (0.37f * turnSeverity));
            commandedYawAccelRadps2 *= rollingModeYawScale;
            yawCommandScale =
                1.01f *
                rollingModeYawScale *
                rollingYawLeadScale *
                trackWidthRatio *
                std::sqrt(trackWidthRatio) *
                std::sqrt(std::sqrt(trackWidthRatio)) *
                std::sqrt(std::sqrt(std::sqrt(trackWidthRatio))) *
                std::sqrt(std::sqrt(std::sqrt(std::sqrt(trackWidthRatio)))) *
                std::sqrt(std::sqrt(std::sqrt(std::sqrt(std::sqrt(trackWidthRatio))))) *
                (1.0f + (0.10f * (trackWidthRatio - 1.0f)));
            break;

        case VelocityTargetOperatingMode::TractionLoss:
        default:
            break;
        }

        DriveCommandSolution solution =
            solveBase(
                operatingState,
                desiredLongitudinalAccelMps2,
                commandedYawAccelRadps2);
        const float yawCommandBias =
            (params.wheelRadiusM > 0.0f) ?
            (std::clamp)(
                (0.5f * PlantModel::kDefaultVelocityTargetResponseTimeS *
                    targetMotionTrackWidthM *
                    commandedYawAccelRadps2) *
                    params.invWheelRadiusM,
                -0.60f,
                0.60f) :
            0.0f;
        const float commonCommandTrim = -0.10f * std::fabs(yawCommandBias);
        const VelocityTargetOperatingMode operatingMode =
            ResolveVelocityTargetOperatingMode(
                operatingState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                solution,
                params);
        switch (operatingMode)
        {
        case VelocityTargetOperatingMode::TractionLoss:
            return solution;

        case VelocityTargetOperatingMode::Static:
        case VelocityTargetOperatingMode::HighSlipTurn:
        case VelocityTargetOperatingMode::Rolling:
            if (!solution.tractionLimited)
            {
                ApplyVelocityTargetYawCommandAssist(solution, yawCommandScale);
                if (operatingMode != VelocityTargetOperatingMode::Static)
                {
                    const float rollingYawDeficitRatio =
                        (operatingMode == VelocityTargetOperatingMode::Rolling &&
                            (std::fabs(targetYawRateRadps) > params.stopExitYawRateRadps)) ?
                        Clamp01(
                            ((std::fabs(targetYawRateRadps) - std::fabs(currentYawRateRadps)) /
                                std::fabs(targetYawRateRadps))) :
                        0.0f;
                    const float resolvedYawCommandBias =
                        (operatingMode == VelocityTargetOperatingMode::Rolling) ?
                        ((1.045f + (0.37f * rollingYawDeficitRatio)) * yawCommandBias) :
                        yawCommandBias;
                    const float resolvedCommonCommandTrim =
                        (operatingMode == VelocityTargetOperatingMode::Rolling) ?
                        ((1.18f + (0.47f * rollingYawDeficitRatio)) * commonCommandTrim) :
                        commonCommandTrim;
                    ApplyVelocityTargetYawCommandBias(solution, resolvedYawCommandBias);
                    ApplyVelocityTargetCommonCommandBias(solution, resolvedCommonCommandTrim);
                }
            }
            return solution;

        default:
            return solution;
        }
    }

} // namespace

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

    PlantModel::PreparedParams PlantModel::Prepare(const PlantParams& params) noexcept
    {
        PreparedParams prepared{};
        prepared.raw = params;

        prepared.forceEpsilonN = SafePositive(params.forceEpsilonN, 1.0e-4f);

        prepared.trackWidthM = std::fabs(params.trackWidthM);
        prepared.halfTrackWidthM = 0.5f * prepared.trackWidthM;
        prepared.invTrackWidthM = (prepared.trackWidthM > 0.0f) ? (1.0f / prepared.trackWidthM) : 0.0f;

        prepared.wheelRadiusM = SafePositive(params.wheelRadiusM, 0.0f);
        prepared.invWheelRadiusM = (prepared.wheelRadiusM > 0.0f) ? (1.0f / prepared.wheelRadiusM) : 0.0f;
        prepared.longitudinalOffsetM = std::fabs(params.contactPatchLongitudinalOffsetM);
        prepared.yawLeverArmM = (std::max)(prepared.longitudinalOffsetM, prepared.halfTrackWidthM);

        prepared.longitudinalMassKg =
            (std::isfinite(params.effectiveLongitudinalMassKg) && (params.effectiveLongitudinalMassKg > 0.0f)) ?
            params.effectiveLongitudinalMassKg :
            SafePositive(params.massKg, 1.0f);
        prepared.invLongitudinalMassKg = 1.0f / prepared.longitudinalMassKg;

        prepared.lateralMassKg =
            (std::isfinite(params.effectiveLateralMassKg) && (params.effectiveLateralMassKg > 0.0f)) ?
            params.effectiveLateralMassKg :
            SafePositive(params.massKg, 1.0f);
        prepared.invLateralMassKg = 1.0f / prepared.lateralMassKg;

        prepared.yawInertiaKgM2 = SafePositive(params.yawInertiaKgM2, 1.0f);
        prepared.invYawInertiaKgM2 = 1.0f / prepared.yawInertiaKgM2;

        prepared.wheelInertiaKgM2 = SafePositive(params.equivalentWheelInertiaKgM2, 1.0f);
        prepared.invWheelInertiaKgM2 = 1.0f / prepared.wheelInertiaKgM2;

        prepared.rollingRegularizationMps =
            (std::isfinite(params.rollingSpeedRegularizationMps) && (params.rollingSpeedRegularizationMps > 0.0f)) ?
            params.rollingSpeedRegularizationMps :
            SafePositive(params.velocityEpsilonMps, 1.0e-3f);

        prepared.staticFrictionTorqueNm = SafePositive(params.staticFrictionTorqueNm, 0.0f);
        prepared.staticFrictionSpeedThresholdRadps =
            (prepared.wheelRadiusM > 0.0f) ?
            (SafePositive(params.staticFrictionMaxSpeedMps, 0.0f) * prepared.invWheelRadiusM) :
            0.0f;

        prepared.longitudinalTireStiffnessN = params.longitudinalTireStiffnessN;
        prepared.invLongitudinalTireStiffnessN =
            (std::fabs(prepared.longitudinalTireStiffnessN) > prepared.forceEpsilonN) ?
            (1.0f / prepared.longitudinalTireStiffnessN) :
            0.0f;
        prepared.frontCorneringStiffnessAxleNPerRad = 2.0f * params.corneringStiffnessFrontNPerRad;
        prepared.rearCorneringStiffnessAxleNPerRad = 2.0f * params.corneringStiffnessRearNPerRad;

        prepared.lateralDampingNPerM = (std::max)(0.0f, params.lateralVelocityDampingNsPerM);
        prepared.yawDampingNmPerRadps = (std::max)(0.0f, params.yawRateDampingNmsPerRad);
        prepared.lateralDampingOverMass = prepared.lateralDampingNPerM * prepared.invLateralMassKg;
        prepared.yawDampingOverInertia = prepared.yawDampingNmPerRadps * prepared.invYawInertiaKgM2;

        prepared.baseNormalLoadN = params.massKg * GRAVITY_MPS2;
        prepared.fanDownforceAtFullDutyN = params.fanDownforceAtFullDutyN;
        prepared.frontLoadFraction =
            std::isfinite(params.frontLoadFraction) ? Clamp01(params.frontLoadFraction) : 0.5f;
        prepared.rearLoadFraction = 1.0f - prepared.frontLoadFraction;
        prepared.lambdaFront =
            std::isfinite(params.frontLongitudinalForceSplit) ? Clamp01(params.frontLongitudinalForceSplit) : 0.5f;
        prepared.lambdaRear = 1.0f - prepared.lambdaFront;

        const bool haveCombinedPeak =
            std::isfinite(params.combinedAccelPeakMps2) &&
            (params.combinedAccelPeakMps2 > 0.0f) &&
            std::isfinite(params.massKg) &&
            (params.massKg > 0.0f);
        prepared.useEnvelopeMuFront =
            !((std::isfinite(params.muFrontPeak) && (params.muFrontPeak > 0.0f))) &&
            haveCombinedPeak;
        prepared.useEnvelopeMuRear =
            !((std::isfinite(params.muRearPeak) && (params.muRearPeak > 0.0f))) &&
            haveCombinedPeak;

        prepared.muFrontBase =
            (std::isfinite(params.muFrontPeak) && (params.muFrontPeak > 0.0f)) ?
            params.muFrontPeak :
            (std::max)(0.0f, params.muFront);
        prepared.muRearBase =
            (std::isfinite(params.muRearPeak) && (params.muRearPeak > 0.0f)) ?
            params.muRearPeak :
            (std::max)(0.0f, params.muRear);
        prepared.combinedAccelPeakTimesMass =
            haveCombinedPeak ? (params.combinedAccelPeakMps2 * params.massKg) : 0.0f;

        prepared.combinedAccelNominalMps2 =
            (std::isfinite(params.combinedAccelNominalMps2) && (params.combinedAccelNominalMps2 > 0.0f)) ?
            params.combinedAccelNominalMps2 :
            ((std::isfinite(params.combinedAccelPeakMps2) && (params.combinedAccelPeakMps2 > 0.0f)) ?
                params.combinedAccelPeakMps2 :
                (std::numeric_limits<float>::infinity)());
        prepared.combinedAccelNominalSq =
            std::isfinite(prepared.combinedAccelNominalMps2) ?
            (prepared.combinedAccelNominalMps2 * prepared.combinedAccelNominalMps2) :
            (std::numeric_limits<float>::infinity)();

        prepared.supplyVoltageV = params.supplyVoltageV;
        prepared.driveResistanceOhms = params.driveResistanceOhms;
        prepared.invDriveResistanceOhms =
            (params.driveResistanceOhms > 0.0f) ? (1.0f / params.driveResistanceOhms) : 0.0f;
        prepared.torqueConstantNmPerA = params.torqueConstantNmPerA;
        prepared.speedConstantRadpsPerVolt = params.speedConstantRadpsPerVolt;
        prepared.noLoadCurrentA = params.noLoadCurrentA;
        prepared.motorCurrentLimitA = params.motorCurrentLimitA;
        prepared.gearRatio = params.gearRatio;
        prepared.drivetrainEfficiency = params.drivetrainEfficiency;
        prepared.driveGain = params.gearRatio * params.drivetrainEfficiency;
        prepared.invDriveGain = (prepared.driveGain > 0.0f) ? (1.0f / prepared.driveGain) : 0.0f;
        prepared.wheelSpeedToBackEmfVoltPerRadps =
            (params.speedConstantRadpsPerVolt > 0.0f) ?
            (params.gearRatio / params.speedConstantRadpsPerVolt) :
            0.0f;
        prepared.wheelSpeedToCurrentAPerRadps =
            (params.driveResistanceOhms > 0.0f) ?
            (prepared.wheelSpeedToBackEmfVoltPerRadps * prepared.invDriveResistanceOhms) :
            0.0f;
        prepared.wheelTorquePerAmpNm =
            params.torqueConstantNmPerA * prepared.driveGain;
        prepared.rollingFrictionTorqueNm = params.rollingFrictionTorqueNm;
        prepared.viscousFrictionNmPerRadps = params.viscousFrictionNmPerRadps;

        prepared.stopEnterSpeedMps = params.stopEnterSpeedMps;
        prepared.stopExitSpeedMps = params.stopExitSpeedMps;
        prepared.stopEnterYawRateRadps = params.stopEnterYawRateRadps;
        prepared.stopExitYawRateRadps = params.stopExitYawRateRadps;
        prepared.stopEnterWheelSpeedRadps = params.stopEnterWheelSpeedRadps;
        prepared.stopExitWheelSpeedRadps = params.stopExitWheelSpeedRadps;
        prepared.stopEnterCommand = params.stopEnterCommand;
        prepared.stopExitCommand = params.stopExitCommand;

        prepared.imuPositionBodyM = params.imu.positionBodyM;
        return prepared;
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return forwardStep(state, control, prepared);
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const ControlInput& control,
        const PreparedParams& params) const noexcept
    {
        PlantDerivatives derivatives{};

        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float commandNorm =
            (std::max)(std::fabs(control.leftMotorCommand), std::fabs(control.rightMotorCommand));

        derivatives.wheelKinematics =
            BuildWheelKinematics(forwardVelocityMps, rightVelocityMps, yawRateRadps, params);

        if (IsStoppedFast(
            forwardVelocityMps,
            rightVelocityMps,
            yawRateRadps,
            omegaLeftRadps,
            omegaRightRadps,
            commandNorm,
            params))
        {
            derivatives.contactForces =
                EvaluateSplitContactForces(SplitForceRequest{}, control.fanDutyCycle, params).forces;
            derivatives.regime = MotionRegime::StoppedHold;
            return derivatives;
        }

        const MotionMetrics metrics =
            EvaluateMotionMetrics(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                commandNorm,
                params);
        const float lateralScrubWeight =
            (std::isfinite(rightVelocityMps) && (rightVelocityMps != 0.0f)) ? 1.0f : 0.0f;
        const float motionWeight = (std::max)(metrics.rollWeight, lateralScrubWeight);

        RollingStateEvaluation rollingState{};
        RollingContactEvaluation rollingForces{};
        if (motionWeight > 0.0f)
        {
            rollingState =
                EvaluateRollingState(
                    forwardVelocityMps,
                    omegaLeftRadps,
                    omegaRightRadps,
                    derivatives.wheelKinematics,
                    control.fanDutyCycle,
                    params);
            rollingForces = rollingState.contact;
        }
        else
        {
            rollingForces = EvaluateSplitContactForces(SplitForceRequest{}, control.fanDutyCycle, params);
        }

        const float yawMomentNm =
            (params.halfTrackWidthM * (rollingForces.leftBankForwardForceN - rollingForces.rightBankForwardForceN)) +
            (params.longitudinalOffsetM * (rollingForces.frontRightForceN - rollingForces.rearRightForceN));

        const float batteryVoltageV = EffectiveBatteryVoltage(control.batteryVoltageV, params);
        const float leftDriveTorqueNm =
            DriveTorqueFromCommandFast(control.leftMotorCommand, omegaLeftRadps, batteryVoltageV, params);
        const float rightDriveTorqueNm =
            DriveTorqueFromCommandFast(control.rightMotorCommand, omegaRightRadps, batteryVoltageV, params);
        const float leftPreFrictionWheelTorqueNm =
            leftDriveTorqueNm - (params.wheelRadiusM * rollingForces.leftBankForwardForceN);
        const float rightPreFrictionWheelTorqueNm =
            rightDriveTorqueNm - (params.wheelRadiusM * rollingForces.rightBankForwardForceN);
        const float leftFrictionTorqueNm =
            DriveFrictionTorqueFast(omegaLeftRadps, leftPreFrictionWheelTorqueNm, params);
        const float rightFrictionTorqueNm =
            DriveFrictionTorqueFast(omegaRightRadps, rightPreFrictionWheelTorqueNm, params);

        float leftNetWheelTorqueNm = leftPreFrictionWheelTorqueNm - leftFrictionTorqueNm;
        float rightNetWheelTorqueNm = rightPreFrictionWheelTorqueNm - rightFrictionTorqueNm;
        if ((std::fabs(omegaLeftRadps) <= params.staticFrictionSpeedThresholdRadps) &&
            (std::fabs(leftPreFrictionWheelTorqueNm) <= params.staticFrictionTorqueNm) &&
            (SignedDirectionFast(leftPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            leftNetWheelTorqueNm = 0.0f;
        }
        if ((std::fabs(omegaRightRadps) <= params.staticFrictionSpeedThresholdRadps) &&
            (std::fabs(rightPreFrictionWheelTorqueNm) <= params.staticFrictionTorqueNm) &&
            (SignedDirectionFast(rightPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            rightNetWheelTorqueNm = 0.0f;
        }

        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(psi, s, c);

        const float pxDot =
            ((rightVelocityMps * c) + (forwardVelocityMps * s)) * motionWeight;
        const float pyDot =
            ((-rightVelocityMps * s) + (forwardVelocityMps * c)) * motionWeight;
        const float psiDot = yawRateRadps * motionWeight;
        const float uDot =
            ((yawRateRadps * rightVelocityMps) +
             (rollingForces.sumForwardForceN * params.invLongitudinalMassKg)) *
            motionWeight;
        const float vDot =
            ((-yawRateRadps * forwardVelocityMps) +
             (rollingForces.sumRightForceN * params.invLateralMassKg) -
             (params.lateralDampingOverMass * rightVelocityMps)) *
            motionWeight;
        const float rDot =
            ((yawMomentNm * params.invYawInertiaKgM2) -
             (params.yawDampingOverInertia * yawRateRadps)) *
            motionWeight;
        const float omegaLDot = (leftNetWheelTorqueNm * params.invWheelInertiaKgM2) * motionWeight;
        const float omegaRDot = (rightNetWheelTorqueNm * params.invWheelInertiaKgM2) * motionWeight;

        derivatives.stateDot(VehicleState::kPx) = pxDot;
        derivatives.stateDot(VehicleState::kPy) = pyDot;
        derivatives.stateDot(VehicleState::kPsi) = psiDot;
        derivatives.stateDot(VehicleState::kU) = uDot;
        derivatives.stateDot(VehicleState::kV) = vDot;
        derivatives.stateDot(VehicleState::kR) = rDot;
        derivatives.stateDot(VehicleState::kOmegaL) = omegaLDot;
        derivatives.stateDot(VehicleState::kOmegaR) = omegaRDot;
        derivatives.stateDot(VehicleState::kBgz) = 0.0f;

        derivatives.contactForces = BlendContactForces(rollingForces.forces, motionWeight);
        derivatives.maxContactUtilization =
            (motionWeight >= 1.0f) ? rollingForces.maxUtilization : (motionWeight * rollingForces.maxUtilization);

        if (motionWeight < 0.5f)
        {
            derivatives.regime = MotionRegime::StoppedHold;
            derivatives.slipTargets = SlipTargets{};
        }
        else
        {
            derivatives.slipTargets = rollingState.targets;
            if (rollingForces.maxUtilization >= (1.0f - 1.0e-4f))
            {
                derivatives.regime = MotionRegime::RollingSaturated;
            }
            else
            {
                derivatives.regime = MotionRegime::RollingAdherent;
            }
        }

        const float originAccelRightMps2 = derivatives.stateDot(VehicleState::kV) + (yawRateRadps * forwardVelocityMps);
        const float originAccelForwardMps2 = derivatives.stateDot(VehicleState::kU) - (yawRateRadps * rightVelocityMps);
        derivatives.originAccelBodyMps2 = Eigen::Vector2f(originAccelRightMps2, originAccelForwardMps2);

        const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            originAccelRightMps2 -
                (yawRateSquaredRadps2 * params.imuPositionBodyM.x()) +
                (rDot * params.imuPositionBodyM.y()),
            originAccelForwardMps2 -
                (yawRateSquaredRadps2 * params.imuPositionBodyM.y()) -
                (rDot * params.imuPositionBodyM.x()));
        derivatives.longitudinalAccelMps2 = originAccelForwardMps2;
        derivatives.lateralAccelMps2 = originAccelRightMps2;
        derivatives.yawAccelRadps2 = rDot;
        return derivatives;
    }

    WheelKinematics PlantModel::wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return wheelKinematics(state, prepared);
    }

    WheelKinematics PlantModel::wheelKinematics(const StateVector& state, const PreparedParams& params) const noexcept
    {
        return BuildWheelKinematics(
            state(VehicleState::kU),
            state(VehicleState::kV),
            state(VehicleState::kR),
            params);
    }

    SlipTargets PlantModel::slipTargets(const StateVector& state, const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return slipTargets(state, prepared);
    }

    SlipTargets PlantModel::slipTargets(const StateVector& state, const PreparedParams& params) const noexcept
    {
        return slipTargets(state, wheelKinematics(state, params), params);
    }

    SlipTargets PlantModel::slipTargets(
        const StateVector& state,
        const WheelKinematics& kinematics,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return slipTargets(state, kinematics, prepared);
    }

    SlipTargets PlantModel::slipTargets(
        const StateVector& state,
        const WheelKinematics& kinematics,
        const PreparedParams& params) const noexcept
    {
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);

        if (ShouldReportStoppedDiagnosticsFast(
            forwardVelocityMps,
            rightVelocityMps,
            yawRateRadps,
            omegaLeftRadps,
            omegaRightRadps,
            params))
        {
            return SlipTargets{};
        }

        return ComputeRollingSlipTargets(
            forwardVelocityMps,
            omegaLeftRadps,
            omegaRightRadps,
            kinematics,
            params);
    }

    ContactForces PlantModel::tireForces(const StateVector& state, const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return tireForces(state, prepared);
    }

    ContactForces PlantModel::tireForces(const StateVector& state, const PreparedParams& params) const noexcept
    {
        return tireForces(state, ControlInput{}, params);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return tireForces(state, control, prepared);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const ControlInput& control,
        const PreparedParams& params) const noexcept
    {
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float commandNorm =
            (std::max)(std::fabs(control.leftMotorCommand), std::fabs(control.rightMotorCommand));

        const MotionMetrics metrics =
            EvaluateMotionMetrics(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                commandNorm,
                params);
        if (metrics.rollWeight <= 0.0f)
        {
            return EvaluateSplitContactForces(SplitForceRequest{}, control.fanDutyCycle, params).forces;
        }

        const WheelKinematics kinematics =
            BuildWheelKinematics(forwardVelocityMps, rightVelocityMps, yawRateRadps, params);
        const RollingStateEvaluation rolling =
            EvaluateRollingState(
                forwardVelocityMps,
                omegaLeftRadps,
                omegaRightRadps,
                kinematics,
                control.fanDutyCycle,
                params);
        return BlendContactForces(rolling.contact.forces, metrics.rollWeight);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return imuPlanarAcceleration(state, control, prepared);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const ControlInput& control,
        const PreparedParams& params) const noexcept
    {
        return forwardStep(state, control, params).imuAccelBodyMps2;
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const ControlInput& control,
        float dt,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return integrate(state, control, dt, prepared);
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const ControlInput& control,
        float dt,
        const PreparedParams& params) const noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return state;
        }

        const PlantDerivatives derivatives = forwardStep(state, control, params);
        StateVector implicitState = state + (dt * derivatives.stateDot);
        implicitState(VehicleState::kPsi) = VehicleState::NormalizeAngle(implicitState(VehicleState::kPsi));

        const float commandNorm =
            (std::max)(std::fabs(control.leftMotorCommand), std::fabs(control.rightMotorCommand));
        if (ShouldSnapToZeroFast(
            implicitState(VehicleState::kU),
            implicitState(VehicleState::kV),
            implicitState(VehicleState::kR),
            implicitState(VehicleState::kOmegaL),
            implicitState(VehicleState::kOmegaR),
            commandNorm,
            params))
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
        const PreparedParams prepared = Prepare(params);
        return solveDriveCommands(
            currentState,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            prepared,
            fanDutyCycle,
            batteryVoltageV);
    }

    DriveCommandSolution PlantModel::solveDriveCommands(
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        DriveCommandSolution solution{};
        solution.control.fanDutyCycle = Clamp01(fanDutyCycle);
        solution.control.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);

        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float forwardVelocityMps = operatingState(VehicleState::kU);
        const float rightVelocityMps = operatingState(VehicleState::kV);
        const float yawRateRadps = operatingState(VehicleState::kR);

        if (!(std::isfinite(desiredLongitudinalAccelMps2) &&
              std::isfinite(desiredYawAccelRadps2) &&
              (params.trackWidthM > 0.0f) &&
              (params.wheelRadiusM > 0.0f) &&
              (params.lateralMassKg > 0.0f) &&
              (params.longitudinalTireStiffnessN > 0.0f) &&
              (params.wheelInertiaKgM2 > 0.0f)))
        {
            return solution;
        }

        const WheelKinematics operatingKinematics =
            BuildWheelKinematics(forwardVelocityMps, rightVelocityMps, yawRateRadps, params);
        const RollingStateEvaluation baseline =
            EvaluateRollingState(
                forwardVelocityMps,
                operatingState(VehicleState::kOmegaL),
                operatingState(VehicleState::kOmegaR),
                operatingKinematics,
                solution.control.fanDutyCycle,
                params);

        const float baselineYawMomentNm =
            params.longitudinalOffsetM *
            (baseline.contact.frontRightForceN - baseline.contact.rearRightForceN);
        const float estimatedLateralAccelMps2 =
            std::fabs(
                (baseline.contact.sumRightForceN * params.invLateralMassKg) -
                (params.lateralDampingOverMass * rightVelocityMps));

        const float desiredLongitudinalAccelMagnitudeMps2 = std::fabs(desiredLongitudinalAccelMps2);
        const float availableLongitudinalAccelMps2 =
            std::isfinite(params.combinedAccelNominalSq) ?
            MazeMap::Math::Sqrtf((std::max)(
                0.0f,
                params.combinedAccelNominalSq - (estimatedLateralAccelMps2 * estimatedLateralAccelMps2))) :
            desiredLongitudinalAccelMagnitudeMps2;
        const float clippedLongitudinalAccelMps2 =
            (std::clamp)(
                desiredLongitudinalAccelMps2,
                -availableLongitudinalAccelMps2,
                availableLongitudinalAccelMps2);

        const float totalForwardForceCommandN = params.longitudinalMassKg * clippedLongitudinalAccelMps2;
        const float totalYawMomentCommandNm =
            (params.yawInertiaKgM2 * desiredYawAccelRadps2) +
            (params.yawDampingNmPerRadps * yawRateRadps);
        const float longitudinalYawMomentCommandNm = totalYawMomentCommandNm - baselineYawMomentNm;
        const float halfTotalForwardForceCommandN = 0.5f * totalForwardForceCommandN;
        const float leftBankForceUnclippedN =
            halfTotalForwardForceCommandN + (longitudinalYawMomentCommandNm * params.invTrackWidthM);
        const float rightBankForceUnclippedN =
            halfTotalForwardForceCommandN - (longitudinalYawMomentCommandNm * params.invTrackWidthM);

        const ContactLoads loads = BuildContactLoads(solution.control.fanDutyCycle, params);
        const PeakFrictionCoefficients peak = BuildPeakFrictionCoefficients(loads, params);
        const float flPeakForceN = peak.front * loads.flN;
        const float frPeakForceN = peak.front * loads.frN;
        const float rlPeakForceN = peak.rear * loads.rlN;
        const float rrPeakForceN = peak.rear * loads.rrN;

        const ContactForce& fl = baseline.contact.forces.contacts[kFrontLeft];
        const ContactForce& fr = baseline.contact.forces.contacts[kFrontRight];
        const ContactForce& rl = baseline.contact.forces.contacts[kRearLeft];
        const ContactForce& rr = baseline.contact.forces.contacts[kRearRight];

        const float flForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (flPeakForceN * flPeakForceN) - (fl.rightForceN * fl.rightForceN)));
        const float frForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (frPeakForceN * frPeakForceN) - (fr.rightForceN * fr.rightForceN)));
        const float rlForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (rlPeakForceN * rlPeakForceN) - (rl.rightForceN * rl.rightForceN)));
        const float rrForwardCapacityN =
            MazeMap::Math::Sqrtf((std::max)(0.0f, (rrPeakForceN * rrPeakForceN) - (rr.rightForceN * rr.rightForceN)));

        const float leftBankForwardCapacityN =
            FixedSplitBankForwardCapacityN(flForwardCapacityN, rlForwardCapacityN, params);
        const float rightBankForwardCapacityN =
            FixedSplitBankForwardCapacityN(frForwardCapacityN, rrForwardCapacityN, params);

        float tractionScale = 1.0f;
        tractionScale =
            (std::min)(
                tractionScale,
                leftBankForwardCapacityN / (std::max)(std::fabs(leftBankForceUnclippedN), params.forceEpsilonN));
        tractionScale =
            (std::min)(
                tractionScale,
                rightBankForwardCapacityN / (std::max)(std::fabs(rightBankForceUnclippedN), params.forceEpsilonN));

        const float envelopeScale =
            (desiredLongitudinalAccelMagnitudeMps2 > params.forceEpsilonN) ?
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
                params.trackWidthM * (-leftBankForwardCapacityN - halfScaledTotalForwardForceN),
                params.trackWidthM * (halfScaledTotalForwardForceN - rightBankForwardCapacityN));
        const float maxLongitudinalYawMomentNm =
            (std::min)(
                params.trackWidthM * (leftBankForwardCapacityN - halfScaledTotalForwardForceN),
                params.trackWidthM * (halfScaledTotalForwardForceN + rightBankForwardCapacityN));
        const float refinedLongitudinalYawMomentNm =
            (minLongitudinalYawMomentNm <= maxLongitudinalYawMomentNm) ?
            (std::clamp)(
                requestedLongitudinalYawMomentNm,
                minLongitudinalYawMomentNm,
                maxLongitudinalYawMomentNm) :
            requestedLongitudinalYawMomentNm;

        const float leftBankForwardVelocityMps = forwardVelocityMps + (params.halfTrackWidthM * yawRateRadps);
        const float rightBankForwardVelocityMps = forwardVelocityMps - (params.halfTrackWidthM * yawRateRadps);
        const float leftRollingWheelSpeedRadps = leftBankForwardVelocityMps * params.invWheelRadiusM;
        const float rightRollingWheelSpeedRadps = rightBankForwardVelocityMps * params.invWheelRadiusM;

        const float achievedYawMomentNm = baselineYawMomentNm + refinedLongitudinalYawMomentNm;
        const float achievedYawAccelRadps2 =
            (achievedYawMomentNm - (params.yawDampingNmPerRadps * yawRateRadps)) * params.invYawInertiaKgM2;
        const float leftWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 + (params.halfTrackWidthM * achievedYawAccelRadps2)) * params.invWheelRadiusM;
        const float rightWheelAccelRadps2 =
            (clippedLongitudinalAccelMps2 - (params.halfTrackWidthM * achievedYawAccelRadps2)) * params.invWheelRadiusM;

        const float leftBankForceCommandN =
            halfScaledTotalForwardForceN + (refinedLongitudinalYawMomentNm * params.invTrackWidthM);
        const float rightBankForceCommandN =
            halfScaledTotalForwardForceN - (refinedLongitudinalYawMomentNm * params.invTrackWidthM);
        const float leftContactTorqueNm = params.wheelRadiusM * leftBankForceCommandN;
        const float rightContactTorqueNm = params.wheelRadiusM * rightBankForceCommandN;

        const float leftSlipRatio = leftBankForceCommandN * params.invLongitudinalTireStiffnessN;
        const float rightSlipRatio = rightBankForceCommandN * params.invLongitudinalTireStiffnessN;
        const float leftWheelSpeedRadps =
            (leftBankForwardVelocityMps +
             (leftSlipRatio * (std::max)(std::fabs(leftBankForwardVelocityMps), params.rollingRegularizationMps))) *
            params.invWheelRadiusM;
        const float rightWheelSpeedRadps =
            (rightBankForwardVelocityMps +
             (rightSlipRatio * (std::max)(std::fabs(rightBankForwardVelocityMps), params.rollingRegularizationMps))) *
            params.invWheelRadiusM;

        const float leftWheelTorqueRequestNm =
            leftContactTorqueNm + (params.wheelInertiaKgM2 * leftWheelAccelRadps2);
        const float rightWheelTorqueRequestNm =
            rightContactTorqueNm + (params.wheelInertiaKgM2 * rightWheelAccelRadps2);
        const float leftWheelTorqueNm =
            leftWheelTorqueRequestNm +
            DriveFrictionTorqueFast(leftWheelSpeedRadps, leftWheelTorqueRequestNm, params);
        const float rightWheelTorqueNm =
            rightWheelTorqueRequestNm +
            DriveFrictionTorqueFast(rightWheelSpeedRadps, rightWheelTorqueRequestNm, params);

        solution.leftSlipRatio = leftSlipRatio;
        solution.rightSlipRatio = rightSlipRatio;
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
            DriveCommandFromTorqueFast(
                leftWheelTorqueNm,
                leftWheelSpeedRadps,
                solution.control.batteryVoltageV,
                params);
        solution.control.rightMotorCommand =
            DriveCommandFromTorqueFast(
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
        const PreparedParams prepared = Prepare(params);
        return solveDriveCommands(
            forwardVelocityMps,
            desiredLongitudinalAccelMps2,
            yawRateRadps,
            desiredYawAccelRadps2,
            prepared,
            fanDutyCycle,
            batteryVoltageV);
    }

    DriveCommandSolution PlantModel::solveDriveCommands(
        float forwardVelocityMps,
        float desiredLongitudinalAccelMps2,
        float yawRateRadps,
        float desiredYawAccelRadps2,
        const PreparedParams& params,
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
        const PreparedParams prepared = Prepare(params);
        return solveDriveCommandsForVelocityTarget(
            currentState,
            targetForwardVelocityMps,
            targetYawRateRadps,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS);
    }

    DriveCommandSolution PlantModel::solveDriveCommandsForVelocityTarget(
        const StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const StateVector reducedState =
            BuildReducedDriveCommandOperatingState(
                operatingState(VehicleState::kU),
                operatingState(VehicleState::kR),
                params);
        return SolveVelocityTargetFeedforward(
            *this,
            reducedState,
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            false,
            [&](const StateVector& state, float desiredLongitudinalAccelMps2, float desiredYawAccelRadps2)
            {
                return solveDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    batteryVoltageV);
            });
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
        const PreparedParams prepared = Prepare(params);
        return solveDriveCommandsForVelocityTarget(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS);
    }

    DriveCommandSolution PlantModel::solveDriveCommandsForVelocityTarget(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        return SolveVelocityTargetFeedforward(
            *this,
            BuildReducedDriveCommandOperatingState(currentForwardVelocityMps, currentYawRateRadps, params),
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            false,
            [&](const StateVector& state, float desiredLongitudinalAccelMps2, float desiredYawAccelRadps2)
            {
                return solveDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    batteryVoltageV);
            });
    }

    void PlantModel::resolveVelocityTargetAccelerations(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        float longitudinalAccelLimitMps2,
        float yawAccelLimitRadps2,
        float responseTimeS,
        float& desiredLongitudinalAccelMps2,
        float& desiredYawAccelRadps2) const noexcept
    {
        desiredLongitudinalAccelMps2 = 0.0f;
        desiredYawAccelRadps2 = 0.0f;

        const float resolvedLongitudinalAccelLimitMps2 =
            (std::isfinite(longitudinalAccelLimitMps2) && (longitudinalAccelLimitMps2 > 0.0f)) ?
            longitudinalAccelLimitMps2 :
            0.0f;
        const float resolvedYawAccelLimitRadps2 =
            (std::isfinite(yawAccelLimitRadps2) && (yawAccelLimitRadps2 > 0.0f)) ?
            yawAccelLimitRadps2 :
            0.0f;
        const float resolvedResponseTimeS = ResolveVelocityTargetResponseTimeS(responseTimeS);
        if (!(resolvedResponseTimeS > 0.0f) || !std::isfinite(resolvedResponseTimeS))
        {
            return;
        }

        if (std::isfinite(currentForwardVelocityMps) && std::isfinite(targetForwardVelocityMps))
        {
            desiredLongitudinalAccelMps2 =
                (targetForwardVelocityMps - currentForwardVelocityMps) / resolvedResponseTimeS;
        }
        if (std::isfinite(currentYawRateRadps) && std::isfinite(targetYawRateRadps))
        {
            desiredYawAccelRadps2 =
                (targetYawRateRadps - currentYawRateRadps) / resolvedResponseTimeS;
        }

        if (!(resolvedLongitudinalAccelLimitMps2 > 0.0f))
        {
            desiredLongitudinalAccelMps2 = 0.0f;
        }
        if (!(resolvedYawAccelLimitRadps2 > 0.0f))
        {
            desiredYawAccelRadps2 = 0.0f;
        }

        const float normalizedLongitudinalDemand =
            (resolvedLongitudinalAccelLimitMps2 > 0.0f) ?
            (std::fabs(desiredLongitudinalAccelMps2) / resolvedLongitudinalAccelLimitMps2) :
            0.0f;
        const float normalizedYawDemand =
            (resolvedYawAccelLimitRadps2 > 0.0f) ?
            (std::fabs(desiredYawAccelRadps2) / resolvedYawAccelLimitRadps2) :
            0.0f;
        const float balanceScale =
            (std::max)(1.0f, (std::max)(normalizedLongitudinalDemand, normalizedYawDemand));

        desiredLongitudinalAccelMps2 /= balanceScale;
        desiredYawAccelRadps2 /= balanceScale;

        if (resolvedLongitudinalAccelLimitMps2 > 0.0f)
        {
            desiredLongitudinalAccelMps2 =
                (std::clamp)(
                    desiredLongitudinalAccelMps2,
                    -resolvedLongitudinalAccelLimitMps2,
                    resolvedLongitudinalAccelLimitMps2);
        }
        if (resolvedYawAccelLimitRadps2 > 0.0f)
        {
            desiredYawAccelRadps2 =
                (std::clamp)(
                    desiredYawAccelRadps2,
                    -resolvedYawAccelLimitRadps2,
                    resolvedYawAccelLimitRadps2);
        }
    }

    void PlantModel::resolveWheelMotionTargets(
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float targetLongitudinalAccelMps2,
        float targetYawAccelRadps2,
        const PreparedParams& params,
        float& leftTargetVelocityMps,
        float& rightTargetVelocityMps,
        float& leftTargetAccelMps2,
        float& rightTargetAccelMps2,
        float& leftTargetOmegaRadps,
        float& rightTargetOmegaRadps) const noexcept
    {
        const float resolvedTargetForwardVelocityMps =
            std::isfinite(targetForwardVelocityMps) ? targetForwardVelocityMps : 0.0f;
        const float resolvedTargetYawRateRadps =
            std::isfinite(targetYawRateRadps) ? targetYawRateRadps : 0.0f;
        const float resolvedTargetLongitudinalAccelMps2 =
            std::isfinite(targetLongitudinalAccelMps2) ? targetLongitudinalAccelMps2 : 0.0f;
        const float resolvedTargetYawAccelRadps2 =
            std::isfinite(targetYawAccelRadps2) ? targetYawAccelRadps2 : 0.0f;
        const float effectiveTrackWidthM =
            ResolveMotionTrackWidthM(
                resolvedTargetForwardVelocityMps,
                resolvedTargetYawRateRadps,
                params);

        leftTargetVelocityMps =
            resolvedTargetForwardVelocityMps +
            (0.5f * effectiveTrackWidthM * resolvedTargetYawRateRadps);
        rightTargetVelocityMps =
            resolvedTargetForwardVelocityMps -
            (0.5f * effectiveTrackWidthM * resolvedTargetYawRateRadps);
        leftTargetAccelMps2 =
            resolvedTargetLongitudinalAccelMps2 +
            (0.5f * effectiveTrackWidthM * resolvedTargetYawAccelRadps2);
        rightTargetAccelMps2 =
            resolvedTargetLongitudinalAccelMps2 -
            (0.5f * effectiveTrackWidthM * resolvedTargetYawAccelRadps2);
        leftTargetOmegaRadps =
            (params.wheelRadiusM > 0.0f) ?
            (leftTargetVelocityMps * params.invWheelRadiusM) :
            0.0f;
        rightTargetOmegaRadps =
            (params.wheelRadiusM > 0.0f) ?
            (rightTargetVelocityMps * params.invWheelRadiusM) :
            0.0f;
    }

    void PlantModel::resolveBodyVelocityFromWheelSpeeds(
        float leftWheelLinearVelocityMps,
        float rightWheelLinearVelocityMps,
        const PreparedParams& params,
        float& forwardVelocityMps,
        float& yawRateRadps) const noexcept
    {
        const float resolvedLeftWheelLinearVelocityMps =
            std::isfinite(leftWheelLinearVelocityMps) ? leftWheelLinearVelocityMps : 0.0f;
        const float resolvedRightWheelLinearVelocityMps =
            std::isfinite(rightWheelLinearVelocityMps) ? rightWheelLinearVelocityMps : 0.0f;
        const float physicalTrackWidthM = ResolvePhysicalTrackWidthM(params);

        forwardVelocityMps =
            0.5f * (resolvedLeftWheelLinearVelocityMps + resolvedRightWheelLinearVelocityMps);
        yawRateRadps =
            (physicalTrackWidthM > 0.0f) ?
            ((resolvedLeftWheelLinearVelocityMps - resolvedRightWheelLinearVelocityMps) / physicalTrackWidthM) :
            0.0f;
    }

    void PlantModel::velocityTargetTechnicalLimits(
        const StateVector& currentState,
        const PlantParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2,
        float fanDutyCycle) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        velocityTargetTechnicalLimits(
            currentState,
            prepared,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2,
            fanDutyCycle);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        const StateVector& currentState,
        const PreparedParams& params,
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
        const PreparedParams prepared = Prepare(params);
        velocityTargetTechnicalLimits(
            forwardVelocityMps,
            yawRateRadps,
            prepared,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2,
            fanDutyCycle);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float forwardVelocityMps,
        float yawRateRadps,
        const PreparedParams& params,
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
        const PreparedParams prepared = Prepare(params);
        return solveClosedLoopDriveCommands(
            currentState,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            tractionReserveScale);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommands(
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        const PreparedParams& params,
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
        const PreparedParams prepared = Prepare(params);
        return solveClosedLoopDriveCommands(
            forwardVelocityMps,
            desiredLongitudinalAccelMps2,
            yawRateRadps,
            desiredYawAccelRadps2,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            tractionReserveScale);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommands(
        float forwardVelocityMps,
        float desiredLongitudinalAccelMps2,
        float yawRateRadps,
        float desiredYawAccelRadps2,
        const PreparedParams& params,
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
        const PreparedParams prepared = Prepare(params);
        return solveClosedLoopDriveCommandsForVelocityTarget(
            currentState,
            targetForwardVelocityMps,
            targetYawRateRadps,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            tractionReserveScale);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommandsForVelocityTarget(
        const StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS,
        float tractionReserveScale) const noexcept
    {
        const float resolvedReserveScale = ResolveClosedLoopTractionReserveScale(tractionReserveScale);
        const DriveCommandSolution openLoop =
            solveDriveCommandsForVelocityTarget(
                currentState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                params,
                fanDutyCycle,
                batteryVoltageV,
                responseTimeS);
        if (!openLoop.tractionLimited || (resolvedReserveScale >= 1.0f))
        {
            return openLoop;
        }

        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const StateVector reducedState =
            BuildReducedDriveCommandOperatingState(
                operatingState(VehicleState::kU),
                operatingState(VehicleState::kR),
                params);
        return SolveVelocityTargetFeedforward(
            *this,
            reducedState,
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            false,
            [&](const StateVector& state, float desiredLongitudinalAccelMps2, float desiredYawAccelRadps2)
            {
                return solveClosedLoopDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    batteryVoltageV,
                    tractionReserveScale);
            });
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
        const PreparedParams prepared = Prepare(params);
        return solveClosedLoopDriveCommandsForVelocityTarget(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            prepared,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            tractionReserveScale);
    }

    DriveCommandSolution PlantModel::solveClosedLoopDriveCommandsForVelocityTarget(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        const PreparedParams& params,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS,
        float tractionReserveScale) const noexcept
    {
        const float resolvedReserveScale = ResolveClosedLoopTractionReserveScale(tractionReserveScale);
        const DriveCommandSolution openLoop =
            solveDriveCommandsForVelocityTarget(
                currentForwardVelocityMps,
                targetForwardVelocityMps,
                currentYawRateRadps,
                targetYawRateRadps,
                params,
                fanDutyCycle,
                batteryVoltageV,
                responseTimeS);
        if (!openLoop.tractionLimited || (resolvedReserveScale >= 1.0f))
        {
            return openLoop;
        }

        return SolveVelocityTargetFeedforward(
            *this,
            BuildReducedDriveCommandOperatingState(currentForwardVelocityMps, currentYawRateRadps, params),
            targetForwardVelocityMps,
            targetYawRateRadps,
            params,
            fanDutyCycle,
            batteryVoltageV,
            responseTimeS,
            false,
            [&](const StateVector& state, float desiredLongitudinalAccelMps2, float desiredYawAccelRadps2)
            {
                return solveClosedLoopDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    batteryVoltageV,
                    tractionReserveScale);
            });
    }

    float PlantModel::driveTorqueFromCommand(
        float motorCommand,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return driveTorqueFromCommand(motorCommand, wheelBankSpeedRadps, batteryVoltageV, prepared);
    }

    float PlantModel::driveTorqueFromCommand(
        float motorCommand,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PreparedParams& params) const noexcept
    {
        return DriveTorqueFromCommandFast(motorCommand, wheelBankSpeedRadps, batteryVoltageV, params);
    }

    float PlantModel::driveCommandFromTorque(
        float wheelTorqueNm,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return driveCommandFromTorque(wheelTorqueNm, wheelBankSpeedRadps, batteryVoltageV, prepared);
    }

    float PlantModel::driveCommandFromTorque(
        float wheelTorqueNm,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PreparedParams& params) const noexcept
    {
        return DriveCommandFromTorqueFast(wheelTorqueNm, wheelBankSpeedRadps, batteryVoltageV, params);
    }

    float PlantModel::driveFrictionTorque(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return driveFrictionTorque(wheelBankSpeedRadps, wheelTorqueRequestNm, prepared);
    }

    float PlantModel::driveFrictionTorque(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        const PreparedParams& params) const noexcept
    {
        return DriveFrictionTorqueFast(wheelBankSpeedRadps, wheelTorqueRequestNm, params);
    }
}
