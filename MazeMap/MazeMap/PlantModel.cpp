
#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

#ifndef MAZEMAP_PLANTMODEL_VALIDATE_INVERSE
#define MAZEMAP_PLANTMODEL_VALIDATE_INVERSE 0
#endif

namespace
{
    using MazeMap::ContactForce;
    using MazeMap::ContactForces;
    using MazeMap::MotorEncoderDrive;
    using MazeMap::MotionRegime;
    using MazeMap::PlantDerivatives;
    using MazeMap::PlantModel;
    using MazeMap::PlantParams;
    using MazeMap::SlipTargets;
    using MazeMap::VehicleState;
    using MazeMap::WheelKinematics;
    using CommandVector = MazeMap::App::Internal::CommandVector;

    using PreparedParams = PlantModel::PreparedParams;

    struct DriveCommandSolution
    {
        CommandVector control{};
        float fanDutyCycle = 0.80f;
        float batteryVoltageV = 8.4f;
        float requestedCommonForceN = 0.0f;
        float requestedDifferentialForceN = 0.0f;
        float commandedCommonForceN = 0.0f;
        float commandedDifferentialForceN = 0.0f;
        float leftForceLimitN = 0.0f;
        float rightForceLimitN = 0.0f;
        float leftTangentialCapacityN = 0.0f;
        float rightTangentialCapacityN = 0.0f;
        bool commonForceClamped = false;
        bool differentialForceClamped = false;
        bool closedLoopReserveMode = false;
        float reserveUsage = 1.0f;
        float slipSpeedFloorMps = 0.0f;
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
        float tractionScale = 1.0f;
        bool tractionLimited = false;
        float commandedLongitudinalAccelMps2 = 0.0f;
        float commandedYawAccelRadps2 = 0.0f;
        float longitudinalAccelErrorMps2 = 0.0f;
        float yawAccelErrorRadps2 = 0.0f;
        bool converged = false;
        bool valid = false;
    };

    struct FeedforwardSolveContext
    {
        float batteryVoltageV = 8.4f;
        float fanDutyCycle = 0.80f;
        float reserveUsage = 1.0f;
        float slipSpeedFloorMps = 0.0f;
    };

    struct FeedforwardForceRequest
    {
        float commonForceRequestN = 0.0f;
        float differentialForceRequestN = 0.0f;
        float baselineLateralYawMomentNm = 0.0f;
    };

    struct FeedforwardForceAllocation
    {
        float commonForceCommandN = 0.0f;
        float differentialForceCommandN = 0.0f;
        float leftForceCommandN = 0.0f;
        float rightForceCommandN = 0.0f;
        float leftForceLimitN = 0.0f;
        float rightForceLimitN = 0.0f;
        bool commonForceClamped = false;
        bool differentialForceClamped = false;
    };

    constexpr uint8_t kFrontLeft = 0U;
    constexpr uint8_t kFrontRight = 1U;
    constexpr uint8_t kRearLeft = 2U;
    constexpr uint8_t kRearRight = 3U;
    constexpr float kSignEpsilon = 1.0e-6f;

    bool EmitPlantDebugTextLine(
        void* context,
        const PlantModel::DebugTextSink sink,
        const char* type,
        const char* format,
        ...) noexcept
    {
        if (sink == nullptr || type == nullptr || type[0] == '\0' || format == nullptr)
        {
            return false;
        }

        std::va_list args;
        va_start(args, format);
        const bool ok = sink(context, type, format, args);
        va_end(args);
        return ok;
    }

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

    inline float ClampUnit(float value) noexcept
    {
        return (std::clamp)(value, -1.0f, 1.0f);
    }

    inline float Clamp01(float value) noexcept
    {
        return (std::clamp)(value, 0.0f, 1.0f);
    }

    inline CommandVector ZeroControlVector() noexcept
    {
        return CommandVector(0.0f, 0.0f);
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

    inline float ResolveTractionLimitedReserveScale(float tractionReserveScale) noexcept
    {
        return
            (std::isfinite(tractionReserveScale) &&
             (tractionReserveScale > 0.0f) &&
             (tractionReserveScale <= 1.0f)) ?
            tractionReserveScale :
            PlantModel::kTractionLimitedReserveScale;
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

    inline float ComputeRegularizedLongitudinalSpeedMps(
        float longitudinalSpeedMps,
        float slipSpeedFloorMps) noexcept
    {
        const float resolvedLongitudinalSpeedMps =
            std::isfinite(longitudinalSpeedMps) ? longitudinalSpeedMps : 0.0f;
        const float resolvedSlipSpeedFloorMps =
            (std::isfinite(slipSpeedFloorMps) && (slipSpeedFloorMps > 0.0f)) ?
            slipSpeedFloorMps :
            0.0f;
        return MazeMap::Math::Sqrtf(
            (resolvedLongitudinalSpeedMps * resolvedLongitudinalSpeedMps) +
            (resolvedSlipSpeedFloorMps * resolvedSlipSpeedFloorMps));
    }

    inline float ComputeLateralScrubWeight(
        float rightVelocityMps,
        float yawRateRadps,
        const PreparedParams& params) noexcept
    {
        const float frontLateralSpeedMps = rightVelocityMps + (params.longitudinalOffsetM * yawRateRadps);
        const float rearLateralSpeedMps = rightVelocityMps - (params.longitudinalOffsetM * yawRateRadps);
        const float lateralScrubSpeedMps =
            (std::max)(std::fabs(frontLateralSpeedMps), std::fabs(rearLateralSpeedMps));
        return
            std::isfinite(lateralScrubSpeedMps) ?
            SmoothStep(
                params.stopEnterSpeedMps,
                params.stopExitSpeedMps,
                lateralScrubSpeedMps) :
            0.0f;
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
            (ComputeLateralScrubWeight(rightVelocityMps, yawRateRadps, params) <= 0.0f);
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

    inline float ResolveRollingMotionWeight(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm,
        const PreparedParams& params) noexcept
    {
        const float speedNormMps =
            ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params);
        const float speedWeight =
            SmoothStep(params.stopEnterSpeedMps, params.stopExitSpeedMps, speedNormMps);
        const float yawRateWeight =
            SmoothStep(params.stopEnterYawRateRadps, params.stopExitYawRateRadps, std::fabs(yawRateRadps));
        const float commandWeight =
            SmoothStep(params.stopEnterCommand, params.stopExitCommand, commandNorm);
        const float rollWeight = (std::max)((std::max)(speedWeight, yawRateWeight), commandWeight);
        const float lateralScrubWeight = ComputeLateralScrubWeight(rightVelocityMps, yawRateRadps, params);
        return (std::max)(rollWeight, lateralScrubWeight);
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
        contact.preProjectionUtilization = utilization;
        return contact;
    }

    inline RollingContactEvaluation EvaluateSplitContactForces(
        float leftBankForwardForceRawN,
        float rightBankForwardForceRawN,
        float frontAxleRightForceRawN,
        float rearAxleRightForceRawN,
        float fanDutyCycle,
        const PreparedParams& params) noexcept
    {
        RollingContactEvaluation evaluation{};
        const float clampedFanDutyCycle = Clamp01(fanDutyCycle);
        const float totalN = params.baseNormalLoadN + (clampedFanDutyCycle * params.fanDownforceAtFullDutyN);
        const float frontTotalN = params.frontLoadFraction * totalN;
        const float rearTotalN = params.rearLoadFraction * totalN;
        const float flN = 0.5f * frontTotalN;
        const float frN = flN;
        const float rlN = 0.5f * rearTotalN;
        const float rrN = rlN;
        const float denominator = (std::max)(totalN, params.forceEpsilonN);
        const float envelopeMu =
            (params.combinedAccelPeakTimesMass > 0.0f) ?
            (params.combinedAccelPeakTimesMass / denominator) :
            0.0f;
        const float peakFrontMu = params.useEnvelopeMuFront ? envelopeMu : params.muFrontBase;
        const float peakRearMu = params.useEnvelopeMuRear ? envelopeMu : params.muRearBase;

        const float fxFlRaw = params.lambdaFront * leftBankForwardForceRawN;
        const float fxRlRaw = params.lambdaRear * leftBankForwardForceRawN;
        const float fxFrRaw = params.lambdaFront * rightBankForwardForceRawN;
        const float fxRrRaw = params.lambdaRear * rightBankForwardForceRawN;

        const float fyFlRaw = 0.5f * frontAxleRightForceRawN;
        const float fyFrRaw = fyFlRaw;
        const float fyRlRaw = 0.5f * rearAxleRightForceRawN;
        const float fyRrRaw = fyRlRaw;

        ContactForce& fl = evaluation.forces.contacts[kFrontLeft];
        ContactForce& fr = evaluation.forces.contacts[kFrontRight];
        ContactForce& rl = evaluation.forces.contacts[kRearLeft];
        ContactForce& rr = evaluation.forces.contacts[kRearRight];

        fl = BuildClampedContactForce(
            fxFlRaw,
            fyFlRaw,
            flN,
            peakFrontMu,
            params.forceEpsilonN);
        fr = BuildClampedContactForce(
            fxFrRaw,
            fyFrRaw,
            frN,
            peakFrontMu,
            params.forceEpsilonN);
        rl = BuildClampedContactForce(
            fxRlRaw,
            fyRlRaw,
            rlN,
            peakRearMu,
            params.forceEpsilonN);
        rr = BuildClampedContactForce(
            fxRrRaw,
            fyRrRaw,
            rrN,
            peakRearMu,
            params.forceEpsilonN);

        evaluation.leftBankForwardForceN = fl.forwardForceN + rl.forwardForceN;
        evaluation.rightBankForwardForceN = fr.forwardForceN + rr.forwardForceN;
        evaluation.frontRightForceN = fl.rightForceN + fr.rightForceN;
        evaluation.rearRightForceN = rl.rightForceN + rr.rightForceN;
        evaluation.sumForwardForceN = evaluation.leftBankForwardForceN + evaluation.rightBankForwardForceN;
        evaluation.sumRightForceN = evaluation.frontRightForceN + evaluation.rearRightForceN;
        evaluation.maxUtilization =
            (std::max)(
                (std::max)(fl.preProjectionUtilization, fr.preProjectionUtilization),
                (std::max)(rl.preProjectionUtilization, rr.preProjectionUtilization));
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
        const float uRefLeft =
            ComputeRegularizedLongitudinalSpeedMps(
                kinematics.leftBankForwardVelocityMps,
                params.rollingRegularizationMps);
        const float uRefRight =
            ComputeRegularizedLongitudinalSpeedMps(
                kinematics.rightBankForwardVelocityMps,
                params.rollingRegularizationMps);
        const float uRefBody =
            ComputeRegularizedLongitudinalSpeedMps(
                forwardVelocityMps,
                params.rollingRegularizationMps);

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

    inline RollingContactEvaluation EvaluateRollingState(
        float forwardVelocityMps,
        float omegaLeftRadps,
        float omegaRightRadps,
        const WheelKinematics& kinematics,
        float fanDutyCycle,
        const PreparedParams& params) noexcept
    {
        const SlipTargets targets =
            ComputeRollingSlipTargets(
                forwardVelocityMps,
                omegaLeftRadps,
                omegaRightRadps,
                kinematics,
                params);

        const float alphaFront = std::atan(targets.lateralRatio[kFrontLeft]);
        const float alphaRear = std::atan(targets.lateralRatio[kRearLeft]);
        return EvaluateSplitContactForces(
            params.longitudinalTireStiffnessN * targets.kappaLeft,
            params.longitudinalTireStiffnessN * targets.kappaRight,
            -params.frontCorneringStiffnessAxleNPerRad * alphaFront,
            -params.rearCorneringStiffnessAxleNPerRad * alphaRear,
            fanDutyCycle,
            params);
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
                contact.preProjectionUtilization = 0.0f;
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
            contact.preProjectionUtilization *= rollWeight;
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
        // Reduced overloads do not provide observed wheel state. Mark wheel speeds unknown so
        // BuildDriveCommandOperatingState(...) uses the rolling-kinematic fallback instead of
        // incorrectly treating literal zeros as a stopped-wheel operating point.
        state(VehicleState::kOmegaL) = (std::numeric_limits<float>::quiet_NaN)();
        state(VehicleState::kOmegaR) = (std::numeric_limits<float>::quiet_NaN)();
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

    inline bool UpdateVelocityTargetSolutionPrediction(
        const PlantModel& plant,
        const PlantModel::StateVector& currentState,
        float targetLongitudinalAccelMps2,
        float targetYawAccelRadps2,
        const PreparedParams& params,
        DriveCommandSolution& solution) noexcept
    {
        if (!(std::isfinite(solution.leftWheelSpeedRadps) &&
            std::isfinite(solution.rightWheelSpeedRadps)))
        {
            return false;
        }

        const PlantModel::StateVector validationState =
            BuildDriveCommandValidationState(
                currentState,
                solution.leftWheelSpeedRadps,
                solution.rightWheelSpeedRadps,
                params);
        const PlantDerivatives achievedDerivatives =
            plant.forwardStep(
                validationState,
                solution.control,
                solution.fanDutyCycle,
                solution.batteryVoltageV,
                params);
        solution.commandedLongitudinalAccelMps2 = achievedDerivatives.longitudinalAccelMps2;
        solution.commandedYawAccelRadps2 = achievedDerivatives.yawAccelRadps2;
        solution.longitudinalAccelErrorMps2 =
            solution.commandedLongitudinalAccelMps2 - targetLongitudinalAccelMps2;
        solution.yawAccelErrorRadps2 =
            solution.commandedYawAccelRadps2 - targetYawAccelRadps2;
        return
            std::isfinite(solution.commandedLongitudinalAccelMps2) &&
            std::isfinite(solution.commandedYawAccelRadps2) &&
            std::isfinite(solution.longitudinalAccelErrorMps2) &&
            std::isfinite(solution.yawAccelErrorRadps2);
    }

    inline float ResolveVelocityTargetResponseTolerance(
        float targetValue,
        float minimumTolerance) noexcept
    {
        return (std::max)(minimumTolerance, 0.10f * std::fabs(targetValue));
    }

    inline float ComputeVelocityTargetResponseErrorMetric(
        const PlantModel::StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float responseTimeS,
        const DriveCommandSolution& solution) noexcept
    {
        if (!(std::isfinite(responseTimeS) &&
            (responseTimeS > 0.0f) &&
            std::isfinite(currentState(VehicleState::kU)) &&
            std::isfinite(currentState(VehicleState::kR)) &&
            std::isfinite(solution.commandedLongitudinalAccelMps2) &&
            std::isfinite(solution.commandedYawAccelRadps2)))
        {
            return (std::numeric_limits<float>::infinity)();
        }

        const float predictedForwardVelocityMps =
            currentState(VehicleState::kU) + (responseTimeS * solution.commandedLongitudinalAccelMps2);
        const float predictedYawRateRadps =
            currentState(VehicleState::kR) + (responseTimeS * solution.commandedYawAccelRadps2);
        const float forwardError =
            std::fabs(predictedForwardVelocityMps - targetForwardVelocityMps) /
            ResolveVelocityTargetResponseTolerance(targetForwardVelocityMps, 0.01f);
        const float yawError =
            std::fabs(predictedYawRateRadps - targetYawRateRadps) /
            ResolveVelocityTargetResponseTolerance(targetYawRateRadps, 0.02f);
        return (std::max)(forwardError, yawError);
    }

    inline bool ShouldPreferExactVelocityTargetCandidate(
        float baseErrorMetric,
        float exactErrorMetric) noexcept
    {
        if (!std::isfinite(exactErrorMetric))
        {
            return false;
        }

        if (!std::isfinite(baseErrorMetric))
        {
            return true;
        }

        return (exactErrorMetric + 1.0e-4f) < baseErrorMetric;
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

    inline float SmoothDirection(float value, float epsilon) noexcept
    {
        const float resolvedEpsilon =
            (std::isfinite(epsilon) && (epsilon > 0.0f)) ?
            epsilon :
            1.0e-4f;
        const float denominator = MazeMap::Math::Sqrtf((value * value) + (resolvedEpsilon * resolvedEpsilon));
        return (denominator > 0.0f) ? (value / denominator) : 0.0f;
    }

    inline float DriveFrictionTorqueSmooth(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        const PreparedParams& params) noexcept
    {
        const float viscousFrictionTorqueNm = params.viscousFrictionNmPerRadps * wheelBankSpeedRadps;
        const float speedScaleRadps =
            (std::max)(
                1.0e-4f,
                (std::max)(
                    params.staticFrictionSpeedThresholdRadps,
                    params.rollingRegularizationMps * params.invWheelRadiusM));
        const float rollingBlend =
            SmoothStep(speedScaleRadps, 2.0f * speedScaleRadps, std::fabs(wheelBankSpeedRadps));
        const float staticBlend = 1.0f - rollingBlend;
        const float staticDirection = SmoothDirection(wheelTorqueRequestNm, speedScaleRadps);
        const float rollingDirection = SmoothDirection(wheelBankSpeedRadps, speedScaleRadps);
        const float coulombTorqueNm =
            (staticBlend * params.staticFrictionTorqueNm * staticDirection) +
            (rollingBlend * params.rollingFrictionTorqueNm * rollingDirection);
        return coulombTorqueNm + viscousFrictionTorqueNm;
    }

    inline void PopulateDriveCommandSolutionFromBankForces(
        const MotorEncoderDrive* leftDrive,
        const MotorEncoderDrive* rightDrive,
        float forwardVelocityMps,
        float yawRateRadps,
        float baselineYawMomentNm,
        float leftBankForceCommandN,
        float rightBankForceCommandN,
        const PreparedParams& params,
        DriveCommandSolution& solution) noexcept
    {
        const float leftBankForwardVelocityMps = forwardVelocityMps + (params.halfTrackWidthM * yawRateRadps);
        const float rightBankForwardVelocityMps = forwardVelocityMps - (params.halfTrackWidthM * yawRateRadps);
        const float leftRollingWheelSpeedRadps = leftBankForwardVelocityMps * params.invWheelRadiusM;
        const float rightRollingWheelSpeedRadps = rightBankForwardVelocityMps * params.invWheelRadiusM;

        const float achievedLongitudinalAccelMps2 =
            (leftBankForceCommandN + rightBankForceCommandN) * params.invLongitudinalMassKg;
        const float achievedYawMomentNm =
            baselineYawMomentNm +
            (params.halfTrackWidthM * (leftBankForceCommandN - rightBankForceCommandN));
        const float achievedYawAccelRadps2 =
            (achievedYawMomentNm - (params.yawDampingNmPerRadps * yawRateRadps)) * params.invYawInertiaKgM2;
        const float leftWheelAccelRadps2 =
            (achievedLongitudinalAccelMps2 + (params.halfTrackWidthM * achievedYawAccelRadps2)) *
            params.invWheelRadiusM;
        const float rightWheelAccelRadps2 =
            (achievedLongitudinalAccelMps2 - (params.halfTrackWidthM * achievedYawAccelRadps2)) *
            params.invWheelRadiusM;

        const float leftContactTorqueNm = params.wheelRadiusM * leftBankForceCommandN;
        const float rightContactTorqueNm = params.wheelRadiusM * rightBankForceCommandN;

        const float leftSlipRatio = leftBankForceCommandN * params.invLongitudinalTireStiffnessN;
        const float rightSlipRatio = rightBankForceCommandN * params.invLongitudinalTireStiffnessN;
        const float leftRegularizedForwardSpeedMps =
            ComputeRegularizedLongitudinalSpeedMps(
                leftBankForwardVelocityMps,
                params.rollingRegularizationMps);
        const float rightRegularizedForwardSpeedMps =
            ComputeRegularizedLongitudinalSpeedMps(
                rightBankForwardVelocityMps,
                params.rollingRegularizationMps);
        const float leftWheelSpeedRadps =
            (leftBankForwardVelocityMps +
             (leftSlipRatio * leftRegularizedForwardSpeedMps)) *
            params.invWheelRadiusM;
        const float rightWheelSpeedRadps =
            (rightBankForwardVelocityMps +
             (rightSlipRatio * rightRegularizedForwardSpeedMps)) *
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
        solution.control.SetLeftMotorPwm(
            (leftDrive != nullptr) ?
            leftDrive->getCommandFromTorque(
                leftWheelTorqueNm,
                leftWheelSpeedRadps,
                solution.batteryVoltageV) :
            0.0f);
        solution.control.SetRightMotorPwm(
            (rightDrive != nullptr) ?
            rightDrive->getCommandFromTorque(
                rightWheelTorqueNm,
                rightWheelSpeedRadps,
                solution.batteryVoltageV) :
            0.0f);
        solution.commandedLongitudinalAccelMps2 = achievedLongitudinalAccelMps2;
        solution.commandedYawAccelRadps2 = achievedYawAccelRadps2;
    }

    inline float ResolveSlipBearingWheelSpeedRadps(
        float rollingWheelLinearVelocityMps,
        float rollingWheelOmegaRadps,
        float slipRatio,
        const PreparedParams& params) noexcept
    {
        const float referenceVelocityMps =
            ComputeRegularizedLongitudinalSpeedMps(
                rollingWheelLinearVelocityMps,
                params.rollingRegularizationMps);
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
        CommandVector control = ZeroControlVector();
        float fanDutyCycle = 0.80f;
        float batteryVoltageV = 8.4f;
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
        float commandedLongitudinalAccelMps2 = 0.0f;
        float commandedYawAccelRadps2 = 0.0f;
        float longitudinalAccelErrorMps2 = 0.0f;
        float yawAccelErrorRadps2 = 0.0f;
        bool converged = false;
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
        const MotorEncoderDrive* leftDrive,
        const MotorEncoderDrive* rightDrive,
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
        solution.fanDutyCycle = Clamp01(fanDutyCycle);
        solution.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, params);

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
        solution.control.SetLeftMotorPwm(
            (leftDrive != nullptr) ?
            leftDrive->getCommandFromTorque(
                leftWheelTorqueNm,
                leftWheelSpeedRadps,
                solution.batteryVoltageV) :
            0.0f);
        solution.control.SetRightMotorPwm(
            (rightDrive != nullptr) ?
            rightDrive->getCommandFromTorque(
                rightWheelTorqueNm,
                rightWheelSpeedRadps,
                solution.batteryVoltageV) :
            0.0f);
        solution.commandedLongitudinalAccelMps2 =
            (leftContactForceN + rightContactForceN) * params.invLongitudinalMassKg;
        solution.commandedYawAccelRadps2 =
            ((modeTrackWidthM * differentialContactForceN) - (totalYawDampingNmPerRadps * currentYawRateRadps)) *
            params.invYawInertiaKgM2;
        solution.longitudinalAccelErrorMps2 =
            solution.commandedLongitudinalAccelMps2 - targetLongitudinalAccelMps2;
        solution.yawAccelErrorRadps2 =
            solution.commandedYawAccelRadps2 - targetYawAccelRadps2;
        solution.converged =
            (std::fabs(solution.longitudinalAccelErrorMps2) <= 0.05f) &&
            (std::fabs(solution.yawAccelErrorRadps2) <= 0.2f);
        solution.valid =
            std::isfinite(solution.control.LeftMotorPwm()) &&
            std::isfinite(solution.control.RightMotorPwm()) &&
            std::isfinite(solution.commandedLongitudinalAccelMps2) &&
            std::isfinite(solution.commandedYawAccelRadps2) &&
            std::isfinite(solution.longitudinalAccelErrorMps2) &&
            std::isfinite(solution.yawAccelErrorRadps2) &&
            (std::fabs(solution.control.LeftMotorPwm()) < 0.999f) &&
            (std::fabs(solution.control.RightMotorPwm()) < 0.999f);
        return solution.valid;
    }

    inline bool ApplyVelocityTargetExactControl(
        const PlantModel& plant,
        const MotorEncoderDrive* leftDrive,
        const MotorEncoderDrive* rightDrive,
        const PlantModel::StateVector& solveState,
        const PlantModel::StateVector& validationState,
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
                leftDrive,
                rightDrive,
                solveState,
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
            return false;
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
        return
            UpdateVelocityTargetSolutionPrediction(
                plant,
                validationState,
                resolvedTargetLongitudinalAccelMps2,
                resolvedTargetYawAccelRadps2,
                params,
                solution);
    }

} // namespace

namespace MazeMap
{
    static DriveCommandSolution SolveFeedforwardCanonical(
        const PlantModel& plant,
        const MotorEncoderDrive* leftDrive,
        const MotorEncoderDrive* rightDrive,
        float forwardVelocityMps,
        float yawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV,
        bool hasVelocityTargets,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& prepared) noexcept;

    PlantModel::PlantModel(const Vehicle& vehicle, const VehicleState& runtimeState) noexcept
        : _preparedParams(Prepare(BuildParamsFromVehicleFacts()))
        , _runtimeState(runtimeState)
        , _leftDrive(vehicle.GetLeftMotorEncoderDrive())
        , _rightDrive(vehicle.GetRightMotorEncoderDrive())
    {
    }

    float PlantModel::wallObservationNoHitRangeM() const noexcept
    {
        return
            (std::isfinite(_preparedParams.raw.noHitRangeM) && (_preparedParams.raw.noHitRangeM > 0.0f)) ?
            _preparedParams.raw.noHitRangeM :
            0.30f;
    }

    bool PlantModel::WriteUkfPlantDebugTextDump(void* context, DebugTextSink sink) const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        const PlantParams& params = _preparedParams.raw;

        if (!EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_mass_geometry",
                "mass_kg=%.9g;effective_longitudinal_mass_kg=%.9g;yaw_inertia_kg_m2=%.9g;track_width_m=%.9g;contact_patch_longitudinal_offset_m=%.9g;wheel_radius_m=%.9g;equivalent_wheel_inertia_kg_m2=%.9g",
                static_cast<double>(params.massKg),
                static_cast<double>(params.effectiveLongitudinalMassKg),
                static_cast<double>(params.yawInertiaKgM2),
                static_cast<double>(params.trackWidthM),
                static_cast<double>(params.contactPatchLongitudinalOffsetM),
                static_cast<double>(params.wheelRadiusM),
                static_cast<double>(params.equivalentWheelInertiaKgM2)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_drive_electrical",
                "supply_voltage_v=%.9g;drive_resistance_ohms=%.9g;torque_constant_nm_per_a=%.9g;speed_constant_radps_per_volt=%.9g;no_load_current_a=%.9g;motor_current_limit_a=%.9g;gear_ratio=%.9g;encoder_counts_per_motor_rev=%u",
                static_cast<double>(params.supplyVoltageV),
                static_cast<double>(params.driveResistanceOhms),
                static_cast<double>(params.torqueConstantNmPerA),
                static_cast<double>(params.speedConstantRadpsPerVolt),
                static_cast<double>(params.noLoadCurrentA),
                static_cast<double>(params.motorCurrentLimitA),
                static_cast<double>(params.gearRatio),
                static_cast<unsigned>(params.encoderCountsPerMotorRev)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_tire_friction",
                "drivetrain_efficiency=%.9g;rolling_friction_torque_nm=%.9g;viscous_friction_nm_per_radps=%.9g;longitudinal_tire_stiffness_n=%.9g;cornering_stiffness_front_n_per_rad=%.9g;cornering_stiffness_rear_n_per_rad=%.9g;mu_front=%.9g;mu_rear=%.9g;front_load_fraction=%.9g",
                static_cast<double>(params.drivetrainEfficiency),
                static_cast<double>(params.rollingFrictionTorqueNm),
                static_cast<double>(params.viscousFrictionNmPerRadps),
                static_cast<double>(params.longitudinalTireStiffnessN),
                static_cast<double>(params.corneringStiffnessFrontNPerRad),
                static_cast<double>(params.corneringStiffnessRearNPerRad),
                static_cast<double>(params.muFront),
                static_cast<double>(params.muRear),
                static_cast<double>(params.frontLoadFraction)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_static_friction",
                "static_friction_torque_nm=%.9g;static_friction_max_speed_mps=%.9g",
                static_cast<double>(params.staticFrictionTorqueNm),
                static_cast<double>(params.staticFrictionMaxSpeedMps)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_misc",
                "velocity_epsilon_mps=%.9g;force_epsilon_n=%.9g;fan_downforce_at_full_duty_n=%.9g;no_hit_range_m=%.9g",
                static_cast<double>(params.velocityEpsilonMps),
                static_cast<double>(params.forceEpsilonN),
                static_cast<double>(params.fanDownforceAtFullDutyN),
                static_cast<double>(params.noHitRangeM)))
        {
            return false;
        }

        for (std::size_t index = 0; index < params.contactPositionsBodyM.size(); ++index)
        {
            const Eigen::Vector2f& position = params.contactPositionsBodyM[index];
            if (!EmitPlantDebugTextLine(
                    context,
                    sink,
                    "ukf_dump_contact_position",
                    "index=%u;x_m=%.9g;y_m=%.9g",
                    static_cast<unsigned>(index),
                    static_cast<double>(position.x()),
                    static_cast<double>(position.y())))
            {
                return false;
            }
        }

        const auto emitSensorMount =
            [&](const char* type, const SensorMount& sensor) noexcept
        {
            const Eigen::Matrix2f& bodyFromSensor = sensor.bodyFromSensor();
            return EmitPlantDebugTextLine(
                context,
                sink,
                type,
                "position_x_m=%.9g;position_y_m=%.9g;body_from_sensor_00=%.9g;body_from_sensor_01=%.9g;body_from_sensor_10=%.9g;body_from_sensor_11=%.9g;clockwise_yaw_sign=%.9g",
                static_cast<double>(sensor.positionBodyM().x()),
                static_cast<double>(sensor.positionBodyM().y()),
                static_cast<double>(bodyFromSensor(0, 0)),
                static_cast<double>(bodyFromSensor(0, 1)),
                static_cast<double>(bodyFromSensor(1, 0)),
                static_cast<double>(bodyFromSensor(1, 1)),
                static_cast<double>(sensor.clockwiseYawSign()));
        };
        return
            emitSensorMount("ukf_dump_sensor_front_left", params.frontLeftSensor) &&
            emitSensorMount("ukf_dump_sensor_front_right", params.frontRightSensor) &&
            emitSensorMount("ukf_dump_sensor_side_left", params.sideLeftSensor) &&
            emitSensorMount("ukf_dump_sensor_side_right", params.sideRightSensor) &&
            emitSensorMount("ukf_dump_imu_mount", params.backLeftImuMount);
    }

    PlantModel::StateVector PlantModel::BuildBoundStateVector() const noexcept
    {
        StateVector state = StateVector::Zero();
        state(VehicleState::kPx) = _runtimeState.GetPositionX();
        state(VehicleState::kPy) = _runtimeState.GetPositionY();
        state(VehicleState::kPsi) = _runtimeState.GetOrientation();
        state(VehicleState::kU) = _runtimeState.GetVelocity();
        state(VehicleState::kV) = _runtimeState.GetLateralVelocity();
        state(VehicleState::kR) = _runtimeState.GetRotationalVelocity();
        state(VehicleState::kOmegaL) = _runtimeState.GetWheelSpeedLeft();
        state(VehicleState::kOmegaR) = _runtimeState.GetWheelSpeedRight();
        state(VehicleState::kBgz) = _runtimeState.GetGyroBiasZ();
        VehicleState::NormalizeStateVector(state);
        return state;
    }

    void PlantModel::resolveAppliedBankTorques(
        const StateVector& currentState,
        const App::Internal::CommandVector& control,
        float batteryVoltageV,
        float& leftAppliedBankTorqueNm,
        float& rightAppliedBankTorqueNm) const noexcept
    {
        const float leftWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaL)) ? currentState(VehicleState::kOmegaL) : 0.0f;
        const float rightWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaR)) ? currentState(VehicleState::kOmegaR) : 0.0f;
        const float leftMotorCommand = std::isfinite(control.LeftMotorPwm()) ? control.LeftMotorPwm() : 0.0f;
        const float rightMotorCommand = std::isfinite(control.RightMotorPwm()) ? control.RightMotorPwm() : 0.0f;

        leftAppliedBankTorqueNm =
            _leftDrive.getTorqueFromCommand(
                leftMotorCommand,
                leftWheelSpeedRadps,
                batteryVoltageV);
        rightAppliedBankTorqueNm =
            _rightDrive.getTorqueFromCommand(
                rightMotorCommand,
                rightWheelSpeedRadps,
                batteryVoltageV);
    }

    PlantParams PlantModel::BuildParamsFromVehicleFacts() noexcept
    {
        PlantParams params{};
        constexpr float kReliableLaunchDriveCommand = 0.30f;
        const VehiclePhysicalModel& physical = Vehicle::GetPhysicalModel();
        params.massKg = physical.massKg;
        params.effectiveLongitudinalMassKg = params.massKg;
        params.yawInertiaKgM2 = physical.yawInertiaKgM2;
        params.trackWidthM = physical.trackWidthM;
        params.wheelRadiusM = 0.5f * Vehicle::kDriveWheelDiameterM;
        params.supplyVoltageV = Vehicle::kDriveSupplyVoltageV;
        params.driveResistanceOhms = Vehicle::kDriveResistanceOhms;
        params.torqueConstantNmPerA = Vehicle::kDriveTorqueConstantNmPerA;
        params.speedConstantRadpsPerVolt = Vehicle::kDriveSpeedConstantRadpsPerVolt;
        params.noLoadCurrentA = Vehicle::kDriveNoLoadCurrentA;
        params.gearRatio = Vehicle::kDriveGearRatio;
        params.encoderCountsPerMotorRev = Vehicle::kDriveEncoderPulsesPerRev;
        params.contactPatchLongitudinalOffsetM = Vehicle::kDriveWheelYOffsetM;
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
        params.frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
        params.frontRightSensor = Vehicle::GetFrontRightSensorMount();
        params.sideLeftSensor = Vehicle::GetSideLeftSensorMount();
        params.sideRightSensor = Vehicle::GetSideRightSensorMount();
        params.backLeftImuMount = Vehicle::GetBackLeftImuMount();

        const float frontContactY = std::fabs(params.contactPatchLongitudinalOffsetM);
        const float halfTrackWidthM = 0.5f * params.trackWidthM;
        params.contactPositionsBodyM[kFrontLeft] = Eigen::Vector2f(-halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[kFrontRight] = Eigen::Vector2f(halfTrackWidthM, frontContactY);
        params.contactPositionsBodyM[kRearLeft] = Eigen::Vector2f(-halfTrackWidthM, -frontContactY);
        params.contactPositionsBodyM[kRearRight] = Eigen::Vector2f(halfTrackWidthM, -frontContactY);
        return params;
    }

    PlantParams PlantParams::Default() noexcept
    {
        return PlantModel::BuildParamsFromVehicleFacts();
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
        prepared.rollingFrictionTorqueNm = params.rollingFrictionTorqueNm;
        prepared.viscousFrictionNmPerRadps = params.viscousFrictionNmPerRadps;
        prepared.pivotScrubBreakawayYawMomentNm = SafePositive(params.pivotScrubBreakawayYawMomentNm, 0.0f);
        prepared.pivotScrubRollingYawMomentNm = SafePositive(params.pivotScrubRollingYawMomentNm, 0.0f);
        prepared.pivotScrubMaxForwardSpeedMps = SafePositive(params.pivotScrubMaxForwardSpeedMps, 0.0f);
        prepared.pivotScrubMinCommandYawRateRadps = SafePositive(params.pivotScrubMinCommandYawRateRadps, 0.0f);
        prepared.pivotScrubBreakawayYawRateRadps = SafePositive(params.pivotScrubBreakawayYawRateRadps, 0.0f);
        prepared.pivotScrubBreakawayYawRateBandRadps =
            SafePositive(params.pivotScrubBreakawayYawRateBandRadps, 0.0f);

        prepared.stopEnterSpeedMps = params.stopEnterSpeedMps;
        prepared.stopExitSpeedMps = params.stopExitSpeedMps;
        prepared.stopEnterYawRateRadps = params.stopEnterYawRateRadps;
        prepared.stopExitYawRateRadps = params.stopExitYawRateRadps;
        prepared.stopEnterWheelSpeedRadps = params.stopEnterWheelSpeedRadps;
        prepared.stopExitWheelSpeedRadps = params.stopExitWheelSpeedRadps;
        prepared.stopEnterCommand = params.stopEnterCommand;
        prepared.stopExitCommand = params.stopExitCommand;

        return prepared;
    }

    PlantDerivatives PlantModel::forwardStep(
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        return forwardStep(BuildBoundStateVector(), control, fanDutyCycle, batteryVoltageV, _preparedParams);
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return forwardStep(state, control, fanDutyCycle, batteryVoltageV, prepared);
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        const PreparedParams& params) const noexcept
    {
        float leftDriveTorqueNm = 0.0f;
        float rightDriveTorqueNm = 0.0f;
        resolveAppliedBankTorques(
            state,
            control,
            batteryVoltageV,
            leftDriveTorqueNm,
            rightDriveTorqueNm);
        const float activityNorm =
            (std::max)(std::fabs(control.LeftMotorPwm()), std::fabs(control.RightMotorPwm()));
        return evaluateAppliedBankTorqueStep(
            state,
            leftDriveTorqueNm,
            rightDriveTorqueNm,
            activityNorm,
            params,
            fanDutyCycle);
    }

    PlantModel::WheelOnlyMeasurementPrediction PlantModel::predictWheelOnlyMeasurement(
        const StateVector& state,
        const PreparedParams& params) const noexcept
    {
        WheelOnlyMeasurementPrediction prediction{};
        const float wheelRadiusM =
            (std::isfinite(params.wheelRadiusM) && (params.wheelRadiusM > 0.0f)) ?
            params.wheelRadiusM :
            0.0f;
        const float trackWidthM =
            (std::isfinite(params.trackWidthM) && (params.trackWidthM > 0.0f)) ?
            params.trackWidthM :
            0.0f;
        prediction.forwardSpeedMps = std::isfinite(state(VehicleState::kU)) ? state(VehicleState::kU) : 0.0f;
        prediction.yawRateRadps = std::isfinite(state(VehicleState::kR)) ? state(VehicleState::kR) : 0.0f;
        prediction.leftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((prediction.forwardSpeedMps + (0.5f * trackWidthM * prediction.yawRateRadps)) / wheelRadiusM) :
            0.0f;
        prediction.rightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((prediction.forwardSpeedMps - (0.5f * trackWidthM * prediction.yawRateRadps)) / wheelRadiusM) :
            0.0f;
        return prediction;
    }

    PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float fanDutyCycle) const noexcept
    {
        return forwardStepFromAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            _preparedParams,
            fanDutyCycle);
    }

    PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PlantParams& params,
        float fanDutyCycle) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return forwardStepFromAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            prepared,
            fanDutyCycle);
    }

    PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PreparedParams& params,
        float fanDutyCycle) const noexcept
    {
        return evaluateAppliedBankTorqueStep(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            (std::max)(std::fabs(leftAppliedBankTorqueNm), std::fabs(rightAppliedBankTorqueNm)),
            params,
            fanDutyCycle);
    }

    PlantDerivatives PlantModel::evaluateAppliedBankTorqueStep(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float activityNorm,
        const PreparedParams& params,
        float fanDutyCycle) const noexcept
    {
        PlantDerivatives derivatives{};

        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float commandNorm =
            std::isfinite(activityNorm) ? std::fabs(activityNorm) : 0.0f;

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
                EvaluateSplitContactForces(0.0f, 0.0f, 0.0f, 0.0f, fanDutyCycle, params).forces;
            derivatives.regime = MotionRegime::StoppedHold;
            return derivatives;
        }

        const float motionWeight =
            ResolveRollingMotionWeight(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                commandNorm,
                params);

        RollingContactEvaluation rollingForces{};
        if (motionWeight > 0.0f)
        {
            rollingForces =
                EvaluateRollingState(
                    forwardVelocityMps,
                    omegaLeftRadps,
                    omegaRightRadps,
                    derivatives.wheelKinematics,
                    fanDutyCycle,
                    params);
        }
        else
        {
            rollingForces = EvaluateSplitContactForces(0.0f, 0.0f, 0.0f, 0.0f, fanDutyCycle, params);
        }

        const float yawMomentNm =
            (params.halfTrackWidthM * (rollingForces.leftBankForwardForceN - rollingForces.rightBankForwardForceN)) +
            (params.longitudinalOffsetM * (rollingForces.frontRightForceN - rollingForces.rearRightForceN));

        const float leftPreFrictionWheelTorqueNm =
            leftAppliedBankTorqueNm - (params.wheelRadiusM * rollingForces.leftBankForwardForceN);
        const float rightPreFrictionWheelTorqueNm =
            rightAppliedBankTorqueNm - (params.wheelRadiusM * rollingForces.rightBankForwardForceN);
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
            derivatives.slipTargets = ComputeRollingSlipTargets(
                forwardVelocityMps,
                omegaLeftRadps,
                omegaRightRadps,
                derivatives.wheelKinematics,
                params);
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

        const Eigen::Vector2f imuLeverArmBodyM = params.raw.backLeftImuMount.positionBodyM();
        const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            originAccelRightMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (rDot * imuLeverArmBodyM.y()),
            originAccelForwardMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (rDot * imuLeverArmBodyM.x()));
        derivatives.longitudinalAccelMps2 = originAccelForwardMps2;
        derivatives.lateralAccelMps2 = originAccelRightMps2;
        derivatives.yawAccelRadps2 = rDot;
        return derivatives;
    }

    PlantModel::StateVector PlantModel::integrateAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float fanDutyCycle,
        float dtS) const noexcept
    {
        return integrateAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            _preparedParams,
            fanDutyCycle,
            dtS);
    }

    PlantModel::StateVector PlantModel::integrateAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PreparedParams& params,
        float fanDutyCycle,
        float dtS) const noexcept
    {
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return state;
        }

        const PlantDerivatives evaluatedStep =
            evaluateAppliedBankTorqueStep(
                state,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
                (std::max)(std::fabs(leftAppliedBankTorqueNm), std::fabs(rightAppliedBankTorqueNm)),
                params,
                fanDutyCycle);
        return advanceStateFromDerivatives(state, evaluatedStep, dtS);
    }

    PlantModel::StateVector PlantModel::advanceStateFromDerivatives(
        const StateVector& currentState,
        const PlantDerivatives& evaluatedStep,
        float dtS) noexcept
    {
        StateVector nextState = currentState;
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return nextState;
        }

        nextState(VehicleState::kOmegaL) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaL);
        nextState(VehicleState::kOmegaR) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaR);

        nextState(VehicleState::kU) += dtS * evaluatedStep.stateDot(VehicleState::kU);
        nextState(VehicleState::kV) += dtS * evaluatedStep.stateDot(VehicleState::kV);
        nextState(VehicleState::kR) += dtS * evaluatedStep.stateDot(VehicleState::kR);

        nextState(VehicleState::kPsi) =
            VehicleState::NormalizeAngle(
                currentState(VehicleState::kPsi) + (dtS * nextState(VehicleState::kR)));

        float sineHeading = 0.0f;
        float cosineHeading = 0.0f;
        sin_cosf(nextState(VehicleState::kPsi), sineHeading, cosineHeading);
        const float worldRightVelocityMps =
            (nextState(VehicleState::kV) * cosineHeading) +
            (nextState(VehicleState::kU) * sineHeading);
        const float worldForwardVelocityMps =
            (-nextState(VehicleState::kV) * sineHeading) +
            (nextState(VehicleState::kU) * cosineHeading);
        nextState(VehicleState::kPx) += dtS * worldRightVelocityMps;
        nextState(VehicleState::kPy) += dtS * worldForwardVelocityMps;

        nextState(VehicleState::kBgz) += dtS * evaluatedStep.stateDot(VehicleState::kBgz);
        nextState(VehicleState::kPsi) = VehicleState::NormalizeAngle(nextState(VehicleState::kPsi));
        return nextState;
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
        return tireForces(
            state,
            App::Internal::CommandVector(0.0f, 0.0f),
            0.80f,
            params);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return tireForces(state, control, fanDutyCycle, prepared);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        const PreparedParams& params) const noexcept
    {
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float commandNorm =
            (std::max)(std::fabs(control.LeftMotorPwm()), std::fabs(control.RightMotorPwm()));

        const float speedNormMps =
            ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                params);
        const float speedWeight =
            SmoothStep(params.stopEnterSpeedMps, params.stopExitSpeedMps, speedNormMps);
        const float yawRateWeight =
            SmoothStep(params.stopEnterYawRateRadps, params.stopExitYawRateRadps, std::fabs(yawRateRadps));
        const float commandWeight =
            SmoothStep(params.stopEnterCommand, params.stopExitCommand, commandNorm);
        const float rollWeight = (std::max)((std::max)(speedWeight, yawRateWeight), commandWeight);
        if (rollWeight <= 0.0f)
        {
            return EvaluateSplitContactForces(0.0f, 0.0f, 0.0f, 0.0f, fanDutyCycle, params).forces;
        }

        const WheelKinematics kinematics =
            BuildWheelKinematics(forwardVelocityMps, rightVelocityMps, yawRateRadps, params);
        const RollingContactEvaluation rolling =
            EvaluateRollingState(
                forwardVelocityMps,
                omegaLeftRadps,
                omegaRightRadps,
                kinematics,
                fanDutyCycle,
                params);
        return BlendContactForces(rolling.forces, rollWeight);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        return imuPlanarAcceleration(state, control, fanDutyCycle, batteryVoltageV, _preparedParams);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return imuPlanarAcceleration(state, control, fanDutyCycle, batteryVoltageV, prepared);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        const PreparedParams& params) const noexcept
    {
        return forwardStep(state, control, fanDutyCycle, batteryVoltageV, params).imuAccelBodyMps2;
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        float dt,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return integrate(state, control, fanDutyCycle, batteryVoltageV, dt, prepared);
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        float dt,
        const PreparedParams& params) const noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return state;
        }

        const float commandNorm =
            (std::max)(std::fabs(control.LeftMotorPwm()), std::fabs(control.RightMotorPwm()));
        const PlantDerivatives derivatives = forwardStep(state, control, fanDutyCycle, batteryVoltageV, params);
        StateVector implicitState = state + (dt * derivatives.stateDot);
        implicitState(VehicleState::kPsi) = VehicleState::NormalizeAngle(implicitState(VehicleState::kPsi));

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

    App::Internal::CommandVector PlantModel::solveSteadyStateFeedforward(
        float desiredForwardVelocityMps,
        float desiredYawRateRadps,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        const float resolvedForwardVelocityMps =
            std::isfinite(desiredForwardVelocityMps) ? desiredForwardVelocityMps : 0.0f;
        const float resolvedYawRateRadps =
            std::isfinite(desiredYawRateRadps) ? desiredYawRateRadps : 0.0f;

        return SolveFeedforwardCanonical(
            *this,
            &_leftDrive,
            &_rightDrive,
            resolvedForwardVelocityMps,
            resolvedYawRateRadps,
            0.0f,
            0.0f,
            fanDutyCycle,
            batteryVoltageV,
            true,
            resolvedForwardVelocityMps,
            resolvedYawRateRadps,
            _preparedParams).control;
    }

    App::Internal::CommandVector PlantModel::solveAccelerationFeedforward(
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        const StateVector currentState = BuildBoundStateVector();
        const float forwardVelocityMps =
            std::isfinite(currentState(VehicleState::kU)) ? currentState(VehicleState::kU) : 0.0f;
        const float yawRateRadps =
            std::isfinite(currentState(VehicleState::kR)) ? currentState(VehicleState::kR) : 0.0f;
        const float resolvedLongitudinalAccelMps2 =
            std::isfinite(desiredLongitudinalAccelMps2) ? desiredLongitudinalAccelMps2 : 0.0f;
        const float resolvedYawAccelRadps2 =
            std::isfinite(desiredYawAccelRadps2) ? desiredYawAccelRadps2 : 0.0f;
        return SolveFeedforwardCanonical(
            *this,
            &_leftDrive,
            &_rightDrive,
            forwardVelocityMps,
            yawRateRadps,
            resolvedLongitudinalAccelMps2,
            resolvedYawAccelRadps2,
            fanDutyCycle,
            batteryVoltageV,
            false,
            0.0f,
            0.0f,
            _preparedParams).control;
    }

    void PlantModel::ComputeBodyAction(
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

    void PlantModel::ComputeBodyAction(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float longitudinalAccelLimitMps2,
        float responseTimeS,
        float& desiredLongitudinalAccelMps2) const noexcept
    {
        float unusedDesiredYawAccelRadps2 = 0.0f;
        ComputeBodyAction(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            currentYawRateRadps,
            longitudinalAccelLimitMps2,
            0.0f,
            responseTimeS,
            desiredLongitudinalAccelMps2,
            unusedDesiredYawAccelRadps2);
    }

    void PlantModel::ComputeBodyActionFromYawRate(
        float currentForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        float yawAccelLimitRadps2,
        float responseTimeS,
        float& desiredYawAccelRadps2) const noexcept
    {
        float unusedDesiredLongitudinalAccelMps2 = 0.0f;
        ComputeBodyAction(
            currentForwardVelocityMps,
            currentForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            0.0f,
            yawAccelLimitRadps2,
            responseTimeS,
            unusedDesiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
    }

    void PlantModel::resolveWheelMotionTargets(
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float targetLongitudinalAccelMps2,
        float targetYawAccelRadps2,
        float& leftTargetVelocityMps,
        float& rightTargetVelocityMps,
        float& leftTargetAccelMps2,
        float& rightTargetAccelMps2,
        float& leftTargetOmegaRadps,
        float& rightTargetOmegaRadps) const noexcept
    {
        resolveWheelMotionTargets(
            targetForwardVelocityMps,
            targetYawRateRadps,
            targetLongitudinalAccelMps2,
            targetYawAccelRadps2,
            _preparedParams,
            leftTargetVelocityMps,
            rightTargetVelocityMps,
            leftTargetAccelMps2,
            rightTargetAccelMps2,
            leftTargetOmegaRadps,
            rightTargetOmegaRadps);
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

    Eigen::Matrix<float, 2, 2> PlantModel::encoderPairCovarianceRadps(
        float linearSpeedSigmaMps,
        float yawRateSigmaRadps) const noexcept
    {
        Eigen::Matrix<float, 2, 2> covariance = Eigen::Matrix<float, 2, 2>::Zero();
        if (!(_preparedParams.wheelRadiusM > 0.0f) || !std::isfinite(_preparedParams.wheelRadiusM))
        {
            covariance(0, 0) = 1.0f;
            covariance(1, 1) = 1.0f;
            return covariance;
        }

        const float resolvedLinearSigmaMps =
            (std::isfinite(linearSpeedSigmaMps) && (linearSpeedSigmaMps > 0.0f)) ?
            linearSpeedSigmaMps :
            1.0f;
        const float resolvedYawSigmaRadps =
            (std::isfinite(yawRateSigmaRadps) && (yawRateSigmaRadps > 0.0f)) ?
            yawRateSigmaRadps :
            1.0f;
        const float halfTrackWidthM = 0.5f * ResolvePhysicalTrackWidthM(_preparedParams);
        const float varianceUMps2 = resolvedLinearSigmaMps * resolvedLinearSigmaMps;
        const float varianceYawRateRadps2 = resolvedYawSigmaRadps * resolvedYawSigmaRadps;
        const float varianceWheelLinearMps2 =
            varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float covarianceWheelLinearMps2 =
            varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float invWheelRadius2 = _preparedParams.invWheelRadiusM * _preparedParams.invWheelRadiusM;
        covariance(0, 0) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 1) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(0, 1) = covarianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 0) = covariance(0, 1);
        return covariance;
    }

    Eigen::Matrix<float, 2, 2> PlantModel::encoderPairSqrtNoise(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
            const float sigmaRadps = stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps);
            sqrtNoise(0, 0) = sigmaRadps;
            sqrtNoise(1, 1) = sigmaRadps;
            return sqrtNoise;
        }

        const Eigen::Matrix<float, 2, 2> covariance =
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const Eigen::LLT<Eigen::Matrix<float, 2, 2>> llt(covariance);
        if (llt.info() == Eigen::Success)
        {
            return llt.matrixL();
        }

        Eigen::Matrix<float, 2, 2> fallback = Eigen::Matrix<float, 2, 2>::Zero();
        fallback(0, 0) = 1.0f;
        fallback(1, 1) = 1.0f;
        return fallback;
    }

    float PlantModel::stationaryEncoderOmegaSigmaRadps(float stationaryLinearSpeedSigmaMps) const noexcept
    {
        if (!(_preparedParams.wheelRadiusM > 0.0f) || !std::isfinite(_preparedParams.wheelRadiusM))
        {
            return 1.0f;
        }

        const float resolvedStationarySigmaMps =
            (std::isfinite(stationaryLinearSpeedSigmaMps) && (stationaryLinearSpeedSigmaMps > 0.0f)) ?
            stationaryLinearSpeedSigmaMps :
            1.0f;
        return resolvedStationarySigmaMps * _preparedParams.invWheelRadiusM;
    }

    float PlantModel::measuredLinearSpeedMps(const EncoderObs& observation) const noexcept
    {
        if (!(_preparedParams.wheelRadiusM > 0.0f) || !std::isfinite(_preparedParams.wheelRadiusM))
        {
            return 0.0f;
        }

        return 0.5f * _preparedParams.wheelRadiusM * (observation.omegaLeftRadps + observation.omegaRightRadps);
    }

    float PlantModel::measuredYawRateRadps(const EncoderObs& observation) const noexcept
    {
        const float trackWidthM = ResolvePhysicalTrackWidthM(_preparedParams);
        if (!(_preparedParams.wheelRadiusM > 0.0f) ||
            !std::isfinite(_preparedParams.wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 0.0f;
        }

        return _preparedParams.wheelRadiusM * (observation.omegaLeftRadps - observation.omegaRightRadps) / trackWidthM;
    }

    float PlantModel::measuredYawRateVarianceRadps2(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const float trackWidthM = ResolvePhysicalTrackWidthM(_preparedParams);
        if (!(_preparedParams.wheelRadiusM > 0.0f) ||
            !std::isfinite(_preparedParams.wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 1.0f;
        }

        const bool zeroWheelObservation =
            (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
        const Eigen::Matrix<float, 2, 2> wheelCovarianceRadps2 =
            zeroWheelObservation ?
            (Eigen::Matrix<float, 2, 2>::Identity() *
                (stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps) *
                 stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps))) :
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const float yawScale = _preparedParams.wheelRadiusM / trackWidthM;
        const float variance =
            (yawScale * yawScale) *
            (wheelCovarianceRadps2(0, 0) +
             wheelCovarianceRadps2(1, 1) -
             (2.0f * wheelCovarianceRadps2(0, 1)));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    float PlantModel::measuredWheelVarianceRadps2(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const bool zeroWheelObservation =
            (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
        if (zeroWheelObservation)
        {
            const float stationarySigmaRadps = stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps);
            return stationarySigmaRadps * stationarySigmaRadps;
        }

        const Eigen::Matrix<float, 2, 2> covariance =
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const float variance = (std::max)(covariance(0, 0), covariance(1, 1));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    Eigen::Vector2f PlantModel::wheelLinearVelocityFromBodyState(const StateVector& state) const noexcept
    {
        const float trackWidthM =
            (std::isfinite(_preparedParams.trackWidthM) && (_preparedParams.trackWidthM > 0.0f)) ?
            _preparedParams.trackWidthM :
            0.0f;
        const float forwardSpeedMps =
            std::isfinite(state(VehicleState::kU)) ? state(VehicleState::kU) : 0.0f;
        const float yawRateRadps =
            std::isfinite(state(VehicleState::kR)) ? state(VehicleState::kR) : 0.0f;
        Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
        velocity(0) = forwardSpeedMps + (0.5f * trackWidthM * yawRateRadps);
        velocity(1) = forwardSpeedMps - (0.5f * trackWidthM * yawRateRadps);
        return velocity;
    }

    float PlantModel::sustainedCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        return std::fabs(accelerationMps2) / SafePositive(_preparedParams.raw.combinedAccelSustainedMps2, 1.0f);
    }

    float PlantModel::nominalCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        return std::fabs(accelerationMps2) / SafePositive(_preparedParams.raw.combinedAccelNominalMps2, 1.0f);
    }

    float PlantModel::peakCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        return std::fabs(accelerationMps2) / SafePositive(_preparedParams.raw.combinedAccelPeakMps2, 1.0f);
    }

    float PlantModel::stopExitYawRateUsage(float yawRateRadps) const noexcept
    {
        return std::fabs(yawRateRadps) / SafePositive(_preparedParams.stopExitYawRateRadps, 1.0f);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2,
        float fanDutyCycle) const noexcept
    {
        velocityTargetTechnicalLimits(
            BuildBoundStateVector(),
            _preparedParams,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2,
            fanDutyCycle);
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
        const auto solveLimitProbe =
            [&](float desiredLongitudinalAccelMps2, float desiredYawAccelRadps2) noexcept
        {
            return SolveFeedforwardCanonical(
                *this,
                &_leftDrive,
                &_rightDrive,
                operatingState(VehicleState::kU),
                operatingState(VehicleState::kR),
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                fanDutyCycle,
                params.supplyVoltageV,
                false,
                0.0f,
                0.0f,
                params);
        };

        const DriveCommandSolution positiveLongitudinal =
            solveLimitProbe(kLargeRequestedAccelMagnitude, 0.0f);
        const DriveCommandSolution negativeLongitudinal =
            solveLimitProbe(-kLargeRequestedAccelMagnitude, 0.0f);
        const DriveCommandSolution positiveYaw =
            solveLimitProbe(0.0f, kLargeRequestedAccelMagnitude);
        const DriveCommandSolution negativeYaw =
            solveLimitProbe(0.0f, -kLargeRequestedAccelMagnitude);

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

    static FeedforwardSolveContext BuildFeedforwardSolveContext(
        float fanDutyCycle,
        float batteryVoltageV,
        const PreparedParams& prepared) noexcept
    {
        FeedforwardSolveContext solveContext{};
        solveContext.batteryVoltageV = EffectiveBatteryVoltage(batteryVoltageV, prepared);
        solveContext.fanDutyCycle = Clamp01(fanDutyCycle);
        solveContext.reserveUsage = 1.0f;
        solveContext.slipSpeedFloorMps = (std::max)(prepared.rollingRegularizationMps, 0.0f);
        return solveContext;
    }

    static float ComputeControllerPivotScrubYawMomentNm(
        const PlantModel::StateVector& operatingState,
        bool hasVelocityTargets,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float desiredYawAccelRadps2,
        float effectiveTrackWidthM,
        const PreparedParams& prepared) noexcept
    {
        if (!hasVelocityTargets)
        {
            return 0.0f;
        }
        if (effectiveTrackWidthM <= prepared.forceEpsilonN)
        {
            return 0.0f;
        }
        if ((prepared.pivotScrubBreakawayYawMomentNm <= prepared.forceEpsilonN) &&
            (prepared.pivotScrubRollingYawMomentNm <= prepared.forceEpsilonN))
        {
            return 0.0f;
        }

        const float commandYawRateAbsRadps = std::fabs(targetYawRateRadps);
        const float commandYawGate =
            SmoothStep(
                prepared.pivotScrubMinCommandYawRateRadps,
                prepared.pivotScrubMinCommandYawRateRadps + prepared.pivotScrubMinCommandYawRateRadps,
                commandYawRateAbsRadps);
        if (commandYawGate <= 0.0f)
        {
            return 0.0f;
        }

        const float pivotSpeedMarginMps =
            (0.5f * effectiveTrackWidthM * commandYawRateAbsRadps) -
            std::fabs(targetForwardVelocityMps);
        const float pivotRegimeBlendBandMps =
            (std::max)(prepared.pivotScrubMaxForwardSpeedMps, prepared.rollingRegularizationMps);
        const float pivotRegimeGate =
            SmoothStep(
                0.0f,
                pivotRegimeBlendBandMps,
                pivotSpeedMarginMps);
        if (pivotRegimeGate <= 0.0f)
        {
            return 0.0f;
        }

        const float persistentScrubYawMomentNm = prepared.pivotScrubRollingYawMomentNm;
        const float breakawaySurplusYawMomentNm =
            (std::max)(0.0f, prepared.pivotScrubBreakawayYawMomentNm - persistentScrubYawMomentNm);
        const float breakawaySurplusBlend =
            1.0f -
            SmoothStep(
                prepared.pivotScrubBreakawayYawRateRadps,
                prepared.pivotScrubBreakawayYawRateRadps + prepared.pivotScrubBreakawayYawRateBandRadps,
                std::fabs(operatingState(VehicleState::kR)));
        const float scrubYawMomentNm =
            persistentScrubYawMomentNm +
            (breakawaySurplusBlend * breakawaySurplusYawMomentNm);
        const float commandDirection =
            SignedDirectionFast(targetYawRateRadps, desiredYawAccelRadps2);
        return
            commandDirection *
            commandYawGate *
            pivotRegimeGate *
            scrubYawMomentNm;
    }

    static FeedforwardForceRequest BuildForceRequest(
        const PlantModel& plant,
        const PlantModel::StateVector& operatingState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        bool hasVelocityTargets,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const FeedforwardSolveContext& solveContext,
        const PreparedParams& prepared) noexcept
    {
        FeedforwardForceRequest forceRequest{};

        const CommandVector neutralControl = CommandVector(0.0f, 0.0f);
        const PlantDerivatives baselineDerivatives =
            plant.forwardStep(
                operatingState,
                neutralControl,
                solveContext.fanDutyCycle,
                solveContext.batteryVoltageV,
                prepared);

        const float resolvedLongitudinalAccelMps2 =
            std::isfinite(desiredLongitudinalAccelMps2) ? desiredLongitudinalAccelMps2 : 0.0f;
        const float resolvedYawAccelRadps2 =
            std::isfinite(desiredYawAccelRadps2) ? desiredYawAccelRadps2 : 0.0f;

        const float frontRightForceN =
            baselineDerivatives.contactForces.contacts[kFrontLeft].rightForceN +
            baselineDerivatives.contactForces.contacts[kFrontRight].rightForceN;
        const float rearRightForceN =
            baselineDerivatives.contactForces.contacts[kRearLeft].rightForceN +
            baselineDerivatives.contactForces.contacts[kRearRight].rightForceN;
        forceRequest.baselineLateralYawMomentNm =
            prepared.longitudinalOffsetM * (frontRightForceN - rearRightForceN);
        forceRequest.commonForceRequestN =
            0.5f * prepared.longitudinalMassKg * resolvedLongitudinalAccelMps2;

        const float motionTrackWidthM =
            ResolveMotionTrackWidthM(
                targetForwardVelocityMps,
                targetYawRateRadps,
                prepared);
        const float controllerPivotScrubYawMomentNm =
            ComputeControllerPivotScrubYawMomentNm(
                operatingState,
                hasVelocityTargets,
                targetForwardVelocityMps,
                targetYawRateRadps,
                resolvedYawAccelRadps2,
                hasVelocityTargets ? motionTrackWidthM : prepared.trackWidthM,
                prepared);
        const float effectiveTrackWidthM =
            (hasVelocityTargets &&
             (std::fabs(controllerPivotScrubYawMomentNm) > prepared.forceEpsilonN) &&
             std::isfinite(motionTrackWidthM) &&
             (motionTrackWidthM > prepared.forceEpsilonN)) ?
            motionTrackWidthM :
            prepared.trackWidthM;
        const float inverseTrackWidthPerM =
            (effectiveTrackWidthM > prepared.forceEpsilonN) ? (1.0f / effectiveTrackWidthM) : 0.0f;
        const float requestedYawMomentNm =
            (prepared.yawInertiaKgM2 * resolvedYawAccelRadps2) +
            (prepared.yawDampingNmPerRadps * operatingState(VehicleState::kR)) -
            forceRequest.baselineLateralYawMomentNm +
            controllerPivotScrubYawMomentNm;
        forceRequest.differentialForceRequestN = -requestedYawMomentNm * inverseTrackWidthPerM;
        return forceRequest;
    }

    static FeedforwardForceAllocation AllocateCommonAndDifferentialForces(
        const FeedforwardForceRequest& request,
        float leftTangentialCapacityN,
        float rightTangentialCapacityN,
        float reserveUsage) noexcept
    {
        FeedforwardForceAllocation allocation{};
        const float resolvedReserveUsage =
            (std::isfinite(reserveUsage) && (reserveUsage > 0.0f) && (reserveUsage <= 1.0f)) ?
            reserveUsage :
            1.0f;
        allocation.leftForceLimitN =
            resolvedReserveUsage *
            ((std::isfinite(leftTangentialCapacityN) && (leftTangentialCapacityN > 0.0f)) ? leftTangentialCapacityN : 0.0f);
        allocation.rightForceLimitN =
            resolvedReserveUsage *
            ((std::isfinite(rightTangentialCapacityN) && (rightTangentialCapacityN > 0.0f)) ? rightTangentialCapacityN : 0.0f);

        const float differentialAbsMaxN = (std::min)(allocation.leftForceLimitN, allocation.rightForceLimitN);
        allocation.differentialForceCommandN =
            (std::clamp)(request.differentialForceRequestN, -differentialAbsMaxN, differentialAbsMaxN);

        const float commonForceMinN =
            (std::max)(
                -allocation.leftForceLimitN + allocation.differentialForceCommandN,
                -allocation.rightForceLimitN - allocation.differentialForceCommandN);
        const float commonForceMaxN =
            (std::min)(
                allocation.leftForceLimitN + allocation.differentialForceCommandN,
                allocation.rightForceLimitN - allocation.differentialForceCommandN);
        allocation.commonForceCommandN =
            (commonForceMinN <= commonForceMaxN) ?
            (std::clamp)(request.commonForceRequestN, commonForceMinN, commonForceMaxN) :
            0.0f;

        allocation.leftForceCommandN =
            allocation.commonForceCommandN - allocation.differentialForceCommandN;
        allocation.rightForceCommandN =
            allocation.commonForceCommandN + allocation.differentialForceCommandN;

        constexpr float kClampEpsilonN = 1.0e-4f;
        allocation.commonForceClamped =
            std::fabs(allocation.commonForceCommandN - request.commonForceRequestN) > kClampEpsilonN;
        allocation.differentialForceClamped =
            std::fabs(allocation.differentialForceCommandN - request.differentialForceRequestN) > kClampEpsilonN;
        return allocation;
    }

    static DriveCommandSolution SolveFeedforwardCanonical(
        const PlantModel& plant,
        const MotorEncoderDrive* leftDrive,
        const MotorEncoderDrive* rightDrive,
        float forwardVelocityMps,
        float yawRateRadps,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV,
        bool hasVelocityTargets,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        const PreparedParams& prepared) noexcept
    {
        const PlantModel::StateVector operatingState =
            BuildReducedDriveCommandOperatingState(forwardVelocityMps, yawRateRadps, prepared);
        const FeedforwardSolveContext solveContext =
            BuildFeedforwardSolveContext(fanDutyCycle, batteryVoltageV, prepared);
        const FeedforwardForceRequest forceRequest =
            BuildForceRequest(
                plant,
                operatingState,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                hasVelocityTargets,
                targetForwardVelocityMps,
                targetYawRateRadps,
                solveContext,
                prepared);

        DriveCommandSolution solution{};
        solution.fanDutyCycle = solveContext.fanDutyCycle;
        solution.batteryVoltageV = solveContext.batteryVoltageV;
        solution.requestedCommonForceN = forceRequest.commonForceRequestN;
        solution.requestedDifferentialForceN = forceRequest.differentialForceRequestN;
        solution.closedLoopReserveMode = false;
        solution.reserveUsage = solveContext.reserveUsage;
        solution.slipSpeedFloorMps = solveContext.slipSpeedFloorMps;

        const float totalNormalLoadN =
            prepared.baseNormalLoadN + (solveContext.fanDutyCycle * prepared.fanDownforceAtFullDutyN);
        const float frontWheelNormalLoadN = 0.5f * prepared.frontLoadFraction * totalNormalLoadN;
        const float rearWheelNormalLoadN = 0.5f * prepared.rearLoadFraction * totalNormalLoadN;
        const float loadDenominatorN = (std::max)(totalNormalLoadN, prepared.forceEpsilonN);
        const float envelopeMu =
            (prepared.combinedAccelPeakTimesMass > 0.0f) ?
            (prepared.combinedAccelPeakTimesMass / loadDenominatorN) :
            0.0f;
        const float peakFrontMu = prepared.useEnvelopeMuFront ? envelopeMu : prepared.muFrontBase;
        const float peakRearMu = prepared.useEnvelopeMuRear ? envelopeMu : prepared.muRearBase;
        const float frontCapacityN = (std::max)(0.0f, peakFrontMu * frontWheelNormalLoadN);
        const float rearCapacityN = (std::max)(0.0f, peakRearMu * rearWheelNormalLoadN);
        const float leftBaseTangentialCapacityN =
            FixedSplitBankForwardCapacityN(frontCapacityN, rearCapacityN, prepared);
        const float rightBaseTangentialCapacityN =
            FixedSplitBankForwardCapacityN(frontCapacityN, rearCapacityN, prepared);

        solution.leftTangentialCapacityN = leftBaseTangentialCapacityN;
        solution.rightTangentialCapacityN = rightBaseTangentialCapacityN;

        const FeedforwardForceAllocation allocation =
            AllocateCommonAndDifferentialForces(
                forceRequest,
                solution.leftTangentialCapacityN,
                solution.rightTangentialCapacityN,
                solveContext.reserveUsage);

        solution.commandedCommonForceN = allocation.commonForceCommandN;
        solution.commandedDifferentialForceN = allocation.differentialForceCommandN;
        solution.leftForceLimitN = allocation.leftForceLimitN;
        solution.rightForceLimitN = allocation.rightForceLimitN;
        solution.commonForceClamped = allocation.commonForceClamped;
        solution.differentialForceClamped = allocation.differentialForceClamped;
        solution.tractionLimited = allocation.commonForceClamped || allocation.differentialForceClamped;

        const float leftRequestedForceN = forceRequest.commonForceRequestN - forceRequest.differentialForceRequestN;
        const float rightRequestedForceN = forceRequest.commonForceRequestN + forceRequest.differentialForceRequestN;
        float tractionScale = 1.0f;
        if (std::fabs(leftRequestedForceN) > prepared.forceEpsilonN)
        {
            tractionScale =
                (std::min)(tractionScale, std::fabs(allocation.leftForceCommandN / leftRequestedForceN));
        }
        if (std::fabs(rightRequestedForceN) > prepared.forceEpsilonN)
        {
            tractionScale =
                (std::min)(tractionScale, std::fabs(allocation.rightForceCommandN / rightRequestedForceN));
        }
        solution.tractionScale = (std::clamp)(tractionScale, 0.0f, 1.0f);

        PopulateDriveCommandSolutionFromBankForces(
            leftDrive,
            rightDrive,
            operatingState(VehicleState::kU),
            operatingState(VehicleState::kR),
            forceRequest.baselineLateralYawMomentNm,
            allocation.leftForceCommandN,
            allocation.rightForceCommandN,
            prepared,
            solution);

        const bool predictionValid =
            UpdateVelocityTargetSolutionPrediction(
                plant,
                operatingState,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2,
                prepared,
                solution);
        solution.valid =
            predictionValid &&
            std::isfinite(solution.control.LeftMotorPwm()) &&
            std::isfinite(solution.control.RightMotorPwm()) &&
            std::isfinite(solution.leftWheelTorqueNm) &&
            std::isfinite(solution.rightWheelTorqueNm) &&
            std::isfinite(solution.leftWheelSpeedRadps) &&
            std::isfinite(solution.rightWheelSpeedRadps);
        solution.converged =
            solution.valid &&
            !solution.tractionLimited &&
            (std::fabs(solution.longitudinalAccelErrorMps2) <= 0.05f) &&
            (std::fabs(solution.yawAccelErrorRadps2) <= 0.2f);
        return solution;
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


