
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

    inline float ResolveVelocityTargetResponseTimeS(float responseTimeS) noexcept
    {
        return
            (std::isfinite(responseTimeS) && (responseTimeS > 0.0f)) ?
            responseTimeS :
            PlantModel::kDefaultVelocityTargetResponseTimeS;
    }

    inline float ResolvePhysicalTrackWidthM(const MazeMap::Vehicle& vehicle, const PreparedParams& params) noexcept
    {
        if (std::isfinite(params.trackWidthM) && (params.trackWidthM > 0.0f))
        {
            return params.trackWidthM;
        }

        const float physicalTrackWidthM = vehicle.GetTrackWidth();
        return
            (std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
            physicalTrackWidthM :
            0.0f;
    }

    inline float ResolveMotionTrackWidthM(
        const MazeMap::Vehicle& vehicle,
        float forwardVelocityMps,
        float yawRateRadps,
        const PreparedParams& params) noexcept
    {
        const float effectiveTrackWidthM = vehicle.GetEffectiveTrackWidthForMotion(forwardVelocityMps, yawRateRadps);
        if (std::isfinite(effectiveTrackWidthM) && (effectiveTrackWidthM > 0.0f))
        {
            return effectiveTrackWidthM;
        }

        return ResolvePhysicalTrackWidthM(vehicle, params);
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

    inline float ClampMagnitudeFast(float value, float limit) noexcept
    {
        const float magnitudeLimit =
            (std::isfinite(limit) && (limit > 0.0f)) ? limit : 0.0f;
        return (magnitudeLimit > 0.0f) ?
            (std::clamp)(value, -magnitudeLimit, magnitudeLimit) :
            value;
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

        const float frontAxleRightForceN =
            ClampMagnitudeFast(
                frontAxleRightForceRawN,
                params.frontLoadFraction * params.lateralForceSustainedLimitN);
        const float rearAxleRightForceN =
            ClampMagnitudeFast(
                rearAxleRightForceRawN,
                params.rearLoadFraction * params.lateralForceSustainedLimitN);
        const float fyFlRaw = 0.5f * frontAxleRightForceN;
        const float fyFrRaw = fyFlRaw;
        const float fyRlRaw = 0.5f * rearAxleRightForceN;
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

} // namespace

namespace MazeMap
{
    PlantModel::PlantModel(const Vehicle& vehicle, const VehicleState& runtimeState) noexcept
        : _preparedParams(Prepare(BuildParamsFromVehicle(vehicle)))
        , _vehicle(vehicle)
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
        float& leftAppliedBankTorqueNm,
        float& rightAppliedBankTorqueNm) const noexcept
    {
        const float leftWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaL)) ? currentState(VehicleState::kOmegaL) : 0.0f;
        const float rightWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaR)) ? currentState(VehicleState::kOmegaR) : 0.0f;
        const float leftMotorCommand = std::isfinite(control.LeftMotorPwm()) ? control.LeftMotorPwm() : 0.0f;
        const float rightMotorCommand = std::isfinite(control.RightMotorPwm()) ? control.RightMotorPwm() : 0.0f;
        const float batteryVoltageV = _vehicle.GetBatteryVoltage();

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

    PlantParams PlantModel::BuildParamsFromVehicle(const Vehicle& vehicle) noexcept
    {
        PlantParams params{};
        constexpr float kReliableLaunchDriveCommand = 0.30f;
        const MotorEncoderDrive& leftDrive = vehicle._leftMotor;
        const MotorEncoderDrive& rightDrive = vehicle._rightMotor;
        const VehiclePhysicalModel& physical = vehicle.GetPhysicalModel();
        params.massKg = vehicle.GetMass();
        params.yawInertiaKgM2 = vehicle.GetYawInertia();
        params.trackWidthM = vehicle.GetTrackWidth();
        params.wheelRadiusM = leftDrive.getWheelRadius();
        params.supplyVoltageV = leftDrive.getVoltage();
        params.driveResistanceOhms = leftDrive.getResistance();
        params.torqueConstantNmPerA = leftDrive.getTorqueConstant();
        params.speedConstantRadpsPerVolt = leftDrive.getSpeedConstant();
        params.noLoadCurrentA = leftDrive.getNoLoadCurrent();
        params.gearRatio = leftDrive.getGearRatio();
        params.encoderCountsPerMotorRev = leftDrive.getPulsesPerRev();
        params.equivalentWheelInertiaKgM2 =
            0.5f * (leftDrive.getEquivalentWheelInertiaKgM2() + rightDrive.getEquivalentWheelInertiaKgM2());
        params.effectiveLongitudinalMassKg = params.massKg;
        params.longitudinalTireStiffnessN =
            0.5f * (leftDrive.getLongitudinalTireStiffnessN() + rightDrive.getLongitudinalTireStiffnessN());
        params.corneringStiffnessFrontNPerRad =
            0.5f * (leftDrive.getCorneringStiffnessNPerRad() + rightDrive.getCorneringStiffnessNPerRad());
        params.corneringStiffnessRearNPerRad = params.corneringStiffnessFrontNPerRad;
        params.contactPatchLongitudinalOffsetM = physical.driveWheelLongitudinalOffsetM;
        params.motorCurrentLimitA =
            (leftDrive.getResistance() > 0.0f) ?
            (leftDrive.getVoltage() / leftDrive.getResistance()) :
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
        params.frontLeftSensor = vehicle.GetFrontLeftSensorMount();
        params.frontRightSensor = vehicle.GetFrontRightSensorMount();
        params.sideLeftSensor = vehicle.GetSideLeftSensorMount();
        params.sideRightSensor = vehicle.GetSideRightSensorMount();
        params.backLeftImuMount = vehicle.GetBackLeftImuMount();

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
        static const Vehicle defaultVehicle;
        return PlantModel::BuildParamsFromVehicle(defaultVehicle);
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
        prepared.lateralForceSustainedLimitN =
            (std::isfinite(params.combinedAccelSustainedMps2) &&
             (params.combinedAccelSustainedMps2 > 0.0f) &&
             std::isfinite(params.massKg) &&
             (params.massKg > 0.0f)) ?
            (params.combinedAccelSustainedMps2 * params.massKg) :
            0.0f;

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
        prepared.pivotScrubRollingYawMomentNm = SafePositive(params.pivotScrubRollingYawMomentNm, 0.0f);
        prepared.pivotScrubMaxForwardSpeedMps = SafePositive(params.pivotScrubMaxForwardSpeedMps, 0.0f);

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
        const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(BuildBoundStateVector(), control, _preparedParams);
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return forwardStep(state, control, prepared);
    }

    PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PreparedParams& params) const noexcept
    {
        float leftDriveTorqueNm = 0.0f;
        float rightDriveTorqueNm = 0.0f;
        resolveAppliedBankTorques(
            state,
            control,
            leftDriveTorqueNm,
            rightDriveTorqueNm);
        const float activityNorm =
            (std::max)(std::fabs(control.LeftMotorPwm()), std::fabs(control.RightMotorPwm()));
        return evaluateAppliedBankTorqueStep(
            state,
            leftDriveTorqueNm,
            rightDriveTorqueNm,
            activityNorm,
            params);
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
        float rightAppliedBankTorqueNm) const noexcept
    {
        return forwardStepFromAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            _preparedParams);
    }

    PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return forwardStepFromAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            prepared);
    }

    PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PreparedParams& params) const noexcept
    {
        return evaluateAppliedBankTorqueStep(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            (std::max)(std::fabs(leftAppliedBankTorqueNm), std::fabs(rightAppliedBankTorqueNm)),
            params);
    }

    PlantDerivatives PlantModel::evaluateAppliedBankTorqueStep(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float activityNorm,
        const PreparedParams& params) const noexcept
    {
        const float fanDutyCycle = _vehicle.GetFanDuty();
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
        const int yawRateScrubSign =
            (yawRateRadps > kSignEpsilon) -
            (yawRateRadps < -kSignEpsilon);
        const int yawMomentScrubSign =
            (yawMomentNm > kSignEpsilon) -
            (yawMomentNm < -kSignEpsilon);
        const int scrubSignRaw =
            (yawRateScrubSign != 0) ? yawRateScrubSign : yawMomentScrubSign;
        const float pivotScrubYawMomentNm =
            ((scrubSignRaw != 0) &&
             (params.pivotScrubRollingYawMomentNm > params.forceEpsilonN) &&
             (params.pivotScrubMaxForwardSpeedMps > params.forceEpsilonN) &&
             (std::fabs(forwardVelocityMps) <= params.pivotScrubMaxForwardSpeedMps)) ?
            (static_cast<float>(scrubSignRaw) * params.pivotScrubRollingYawMomentNm) :
            0.0f;
        const float leftPreFrictionWheelTorqueNm =
            leftAppliedBankTorqueNm - (params.wheelRadiusM * rollingForces.leftBankForwardForceN);
        const float rightPreFrictionWheelTorqueNm =
            rightAppliedBankTorqueNm - (params.wheelRadiusM * rollingForces.rightBankForwardForceN);
        const float leftViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * omegaLeftRadps;
        const float rightViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * omegaRightRadps;
        float leftFrictionTorqueNm = leftViscousFrictionTorqueNm;
        float rightFrictionTorqueNm = rightViscousFrictionTorqueNm;
        if ((std::fabs(omegaLeftRadps) <= params.staticFrictionSpeedThresholdRadps) &&
            (std::fabs(omegaRightRadps) <= params.staticFrictionSpeedThresholdRadps))
        {
            const float commonPreFrictionWheelTorqueNm =
                0.5f * (leftPreFrictionWheelTorqueNm + rightPreFrictionWheelTorqueNm);
            const int commonTorqueSign =
                (commonPreFrictionWheelTorqueNm > kSignEpsilon) -
                (commonPreFrictionWheelTorqueNm < -kSignEpsilon);
            const float commonWheelSpeedRadps = 0.5f * (omegaLeftRadps + omegaRightRadps);
            const int commonSpeedSign =
                (commonWheelSpeedRadps > kSignEpsilon) -
                (commonWheelSpeedRadps < -kSignEpsilon);
            const float commonStictionSign =
                static_cast<float>((commonTorqueSign != 0) ? commonTorqueSign : commonSpeedSign);
            leftFrictionTorqueNm += params.staticFrictionTorqueNm * commonStictionSign;
            rightFrictionTorqueNm += params.staticFrictionTorqueNm * commonStictionSign;
        }
        else
        {
            const int leftSpeedSign =
                (omegaLeftRadps > kSignEpsilon) -
                (omegaLeftRadps < -kSignEpsilon);
            const int leftTorqueSign =
                (leftPreFrictionWheelTorqueNm > kSignEpsilon) -
                (leftPreFrictionWheelTorqueNm < -kSignEpsilon);
            leftFrictionTorqueNm +=
                params.rollingFrictionTorqueNm *
                static_cast<float>((leftSpeedSign != 0) ? leftSpeedSign : leftTorqueSign);

            const int rightSpeedSign =
                (omegaRightRadps > kSignEpsilon) -
                (omegaRightRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightPreFrictionWheelTorqueNm > kSignEpsilon) -
                (rightPreFrictionWheelTorqueNm < -kSignEpsilon);
            rightFrictionTorqueNm +=
                params.rollingFrictionTorqueNm *
                static_cast<float>((rightSpeedSign != 0) ? rightSpeedSign : rightTorqueSign);
        }

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
            (((yawMomentNm - pivotScrubYawMomentNm) * params.invYawInertiaKgM2) -
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
        float dtS) const noexcept
    {
        return integrateAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            _preparedParams,
            dtS);
    }

    PlantModel::StateVector PlantModel::integrateAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const PreparedParams& params,
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
                params);
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
            params);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return tireForces(state, control, prepared);
    }

    ContactForces PlantModel::tireForces(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PreparedParams& params) const noexcept
    {
        const float fanDutyCycle = _vehicle.GetFanDuty();
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
        const App::Internal::CommandVector& control) const noexcept
    {
        return imuPlanarAcceleration(state, control, _preparedParams);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return imuPlanarAcceleration(state, control, prepared);
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        const PreparedParams& params) const noexcept
    {
        return forwardStep(state, control, params).imuAccelBodyMps2;
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float dt,
        const PlantParams& params) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        return integrate(state, control, dt, prepared);
    }

    PlantModel::StateVector PlantModel::integrate(
        const StateVector& state,
        const App::Internal::CommandVector& control,
        float dt,
        const PreparedParams& params) const noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return state;
        }

        const float commandNorm =
            (std::max)(std::fabs(control.LeftMotorPwm()), std::fabs(control.RightMotorPwm()));
        const PlantDerivatives derivatives = forwardStep(state, control, params);
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
        float desiredYawRateRadps) const noexcept
    {
        const PreparedParams& params = _preparedParams;
        const float leftBankVelocityMps =
            desiredForwardVelocityMps + (params.halfTrackWidthM * desiredYawRateRadps);
        const float rightBankVelocityMps =
            desiredForwardVelocityMps - (params.halfTrackWidthM * desiredYawRateRadps);
        const float uRefLeft =
            MazeMap::Math::Sqrtf(
                (leftBankVelocityMps * leftBankVelocityMps) +
                (params.rollingRegularizationMps * params.rollingRegularizationMps));
        const float uRefRight =
            MazeMap::Math::Sqrtf(
                (rightBankVelocityMps * rightBankVelocityMps) +
                (params.rollingRegularizationMps * params.rollingRegularizationMps));
        const float uRefBody =
            MazeMap::Math::Sqrtf(
                (desiredForwardVelocityMps * desiredForwardVelocityMps) +
                (params.rollingRegularizationMps * params.rollingRegularizationMps));

        const float frontLateralVelocityMps = params.longitudinalOffsetM * desiredYawRateRadps;
        const float rearLateralVelocityMps = -params.longitudinalOffsetM * desiredYawRateRadps;
        const float alphaFront = std::atan(frontLateralVelocityMps / uRefBody);
        const float alphaRear = std::atan(rearLateralVelocityMps / uRefBody);
        const float frontAxleRightForceRawN =
            -params.frontCorneringStiffnessAxleNPerRad * alphaFront;
        const float rearAxleRightForceRawN =
            -params.rearCorneringStiffnessAxleNPerRad * alphaRear;
        const float frontAxleRightForceLimitN =
            params.frontLoadFraction * params.lateralForceSustainedLimitN;
        const float rearAxleRightForceLimitN =
            params.rearLoadFraction * params.lateralForceSustainedLimitN;
        const float frontAxleRightForceN =
            (frontAxleRightForceLimitN > params.forceEpsilonN) ?
            (std::clamp)(
                frontAxleRightForceRawN,
                -frontAxleRightForceLimitN,
                frontAxleRightForceLimitN) :
            frontAxleRightForceRawN;
        const float rearAxleRightForceN =
            (rearAxleRightForceLimitN > params.forceEpsilonN) ?
            (std::clamp)(
                rearAxleRightForceRawN,
                -rearAxleRightForceLimitN,
                rearAxleRightForceLimitN) :
            rearAxleRightForceRawN;
        const float lateralYawMomentNm =
            params.longitudinalOffsetM * (frontAxleRightForceN - rearAxleRightForceN);

        const int scrubSignRaw =
            (desiredYawRateRadps > kSignEpsilon) -
            (desiredYawRateRadps < -kSignEpsilon);
        const float pivotScrubYawMomentNm =
            ((scrubSignRaw != 0) &&
             (params.pivotScrubRollingYawMomentNm > params.forceEpsilonN) &&
             (params.pivotScrubMaxForwardSpeedMps > params.forceEpsilonN) &&
             (std::fabs(desiredForwardVelocityMps) <= params.pivotScrubMaxForwardSpeedMps)) ?
            (static_cast<float>(scrubSignRaw) * params.pivotScrubRollingYawMomentNm) :
            0.0f;
        const float requestedYawMomentNm =
            (params.yawDampingNmPerRadps * desiredYawRateRadps) +
            pivotScrubYawMomentNm -
            lateralYawMomentNm;
        const float differentialForceRequestN =
            -requestedYawMomentNm * params.invTrackWidthM;
        const float leftForceCommandN = -differentialForceRequestN;
        const float rightForceCommandN = differentialForceRequestN;

        const float leftKappa =
            leftForceCommandN * params.invLongitudinalTireStiffnessN;
        const float rightKappa =
            rightForceCommandN * params.invLongitudinalTireStiffnessN;
        const float leftWheelSpeedRadps =
            (leftBankVelocityMps + (leftKappa * uRefLeft)) * params.invWheelRadiusM;
        const float rightWheelSpeedRadps =
            (rightBankVelocityMps + (rightKappa * uRefRight)) * params.invWheelRadiusM;

        const float leftWheelTorqueRequestNm =
            params.wheelRadiusM * leftForceCommandN;
        const float rightWheelTorqueRequestNm =
            params.wheelRadiusM * rightForceCommandN;

        const float leftViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * leftWheelSpeedRadps;
        const int leftSpeedSign =
            (leftWheelSpeedRadps > kSignEpsilon) -
            (leftWheelSpeedRadps < -kSignEpsilon);
        const int leftTorqueSign =
            (leftWheelTorqueRequestNm > kSignEpsilon) -
            (leftWheelTorqueRequestNm < -kSignEpsilon);
        const float leftFrictionTorqueNm =
            leftViscousFrictionTorqueNm +
            (params.rollingFrictionTorqueNm *
             static_cast<float>((leftSpeedSign != 0) ? leftSpeedSign : leftTorqueSign));

        const float rightViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * rightWheelSpeedRadps;
        const int rightSpeedSign =
            (rightWheelSpeedRadps > kSignEpsilon) -
            (rightWheelSpeedRadps < -kSignEpsilon);
        const int rightTorqueSign =
            (rightWheelTorqueRequestNm > kSignEpsilon) -
            (rightWheelTorqueRequestNm < -kSignEpsilon);
        const float rightFrictionTorqueNm =
            rightViscousFrictionTorqueNm +
            (params.rollingFrictionTorqueNm *
             static_cast<float>((rightSpeedSign != 0) ? rightSpeedSign : rightTorqueSign));

        return CommandVector(
            _leftDrive.getCommandFromTorque(
                leftWheelTorqueRequestNm + leftFrictionTorqueNm,
                leftWheelSpeedRadps,
                params.supplyVoltageV),
            _rightDrive.getCommandFromTorque(
                rightWheelTorqueRequestNm + rightFrictionTorqueNm,
                rightWheelSpeedRadps,
                params.supplyVoltageV));
    }

    App::Internal::CommandVector PlantModel::solveAccelerationFeedforward(
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2) const noexcept
    {
        const PreparedParams& params = _preparedParams;
        const float forwardVelocityMps = _runtimeState.GetVelocity();
        const float rightVelocityMps = _runtimeState.GetLateralVelocity();
        const float yawRateRadps = _runtimeState.GetRotationalVelocity();
        const float leftBankVelocityMps =
            forwardVelocityMps + (params.halfTrackWidthM * yawRateRadps);
        const float rightBankVelocityMps =
            forwardVelocityMps - (params.halfTrackWidthM * yawRateRadps);
        const float uRefBody =
            MazeMap::Math::Sqrtf(
                (forwardVelocityMps * forwardVelocityMps) +
                (params.rollingRegularizationMps * params.rollingRegularizationMps));

        const float wheelInertiaOverRadiusSqKg =
            params.wheelInertiaKgM2 * params.invWheelRadiusM * params.invWheelRadiusM;
        const float leftBankAccelMps2 =
            desiredLongitudinalAccelMps2 + (params.halfTrackWidthM * desiredYawAccelRadps2);
        const float rightBankAccelMps2 =
            desiredLongitudinalAccelMps2 - (params.halfTrackWidthM * desiredYawAccelRadps2);
        const float commonForceRequestN =
            0.5f *
            params.longitudinalMassKg *
            (desiredLongitudinalAccelMps2 - (yawRateRadps * rightVelocityMps));
        const float frontLateralVelocityMps =
            rightVelocityMps + (params.longitudinalOffsetM * yawRateRadps);
        const float rearLateralVelocityMps =
            rightVelocityMps - (params.longitudinalOffsetM * yawRateRadps);
        const float alphaFront = std::atan(frontLateralVelocityMps / uRefBody);
        const float alphaRear = std::atan(rearLateralVelocityMps / uRefBody);
        const float frontAxleRightForceRawN =
            -params.frontCorneringStiffnessAxleNPerRad * alphaFront;
        const float rearAxleRightForceRawN =
            -params.rearCorneringStiffnessAxleNPerRad * alphaRear;
        const float frontAxleRightForceLimitN =
            params.frontLoadFraction * params.lateralForceSustainedLimitN;
        const float rearAxleRightForceLimitN =
            params.rearLoadFraction * params.lateralForceSustainedLimitN;
        const float frontAxleRightForceN =
            (frontAxleRightForceLimitN > params.forceEpsilonN) ?
            (std::clamp)(
                frontAxleRightForceRawN,
                -frontAxleRightForceLimitN,
                frontAxleRightForceLimitN) :
            frontAxleRightForceRawN;
        const float rearAxleRightForceN =
            (rearAxleRightForceLimitN > params.forceEpsilonN) ?
            (std::clamp)(
                rearAxleRightForceRawN,
                -rearAxleRightForceLimitN,
                rearAxleRightForceLimitN) :
            rearAxleRightForceRawN;
        const float lateralYawMomentNm =
            params.longitudinalOffsetM * (frontAxleRightForceN - rearAxleRightForceN);
        const int yawRateScrubSign =
            (yawRateRadps > kSignEpsilon) -
            (yawRateRadps < -kSignEpsilon);
        const int yawAccelScrubSign =
            (desiredYawAccelRadps2 > kSignEpsilon) -
            (desiredYawAccelRadps2 < -kSignEpsilon);
        const int scrubSignRaw =
            (yawRateScrubSign != 0) ? yawRateScrubSign : yawAccelScrubSign;
        const float pivotScrubYawMomentNm =
            ((scrubSignRaw != 0) &&
             (params.pivotScrubRollingYawMomentNm > params.forceEpsilonN) &&
             (params.pivotScrubMaxForwardSpeedMps > params.forceEpsilonN) &&
             (std::fabs(forwardVelocityMps) <= params.pivotScrubMaxForwardSpeedMps)) ?
            (static_cast<float>(scrubSignRaw) * params.pivotScrubRollingYawMomentNm) :
            0.0f;
        const float requestedYawMomentNm =
            (params.yawInertiaKgM2 * desiredYawAccelRadps2) +
            (params.yawDampingNmPerRadps * yawRateRadps) +
            pivotScrubYawMomentNm -
            lateralYawMomentNm;
        const float differentialForceRequestN = -requestedYawMomentNm * params.invTrackWidthM;
        const float leftForceCommandN =
            commonForceRequestN - differentialForceRequestN;
        const float rightForceCommandN =
            commonForceRequestN + differentialForceRequestN;

        const float leftMotorSpeedRadps =
            leftBankVelocityMps * params.invWheelRadiusM;
        const float rightMotorSpeedRadps =
            rightBankVelocityMps * params.invWheelRadiusM;
        const float leftWheelInertiaForceN =
            wheelInertiaOverRadiusSqKg * leftBankAccelMps2;
        const float rightWheelInertiaForceN =
            wheelInertiaOverRadiusSqKg * rightBankAccelMps2;
        const float leftWheelTorqueRequestNm =
            params.wheelRadiusM * (leftForceCommandN + leftWheelInertiaForceN);
        const float rightWheelTorqueRequestNm =
            params.wheelRadiusM * (rightForceCommandN + rightWheelInertiaForceN);

        const float leftViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * leftMotorSpeedRadps;
        const float rightViscousFrictionTorqueNm =
            params.viscousFrictionNmPerRadps * rightMotorSpeedRadps;
        float leftFrictionTorqueNm = leftViscousFrictionTorqueNm;
        float rightFrictionTorqueNm = rightViscousFrictionTorqueNm;
        if ((std::fabs(leftMotorSpeedRadps) <= params.staticFrictionSpeedThresholdRadps) &&
            (std::fabs(rightMotorSpeedRadps) <= params.staticFrictionSpeedThresholdRadps))
        {
            const float commonWheelTorqueRequestNm =
                0.5f * (leftWheelTorqueRequestNm + rightWheelTorqueRequestNm);
            const int commonTorqueSign =
                (commonWheelTorqueRequestNm > kSignEpsilon) -
                (commonWheelTorqueRequestNm < -kSignEpsilon);
            const float commonBankSpeedRadps =
                0.5f * (leftMotorSpeedRadps + rightMotorSpeedRadps);
            const int commonSpeedSign =
                (commonBankSpeedRadps > kSignEpsilon) -
                (commonBankSpeedRadps < -kSignEpsilon);
            const float commonStictionSign =
                static_cast<float>((commonTorqueSign != 0) ? commonTorqueSign : commonSpeedSign);
            leftFrictionTorqueNm += params.staticFrictionTorqueNm * commonStictionSign;
            rightFrictionTorqueNm += params.staticFrictionTorqueNm * commonStictionSign;
        }
        else
        {
            const int leftSpeedSign =
                (leftMotorSpeedRadps > kSignEpsilon) -
                (leftMotorSpeedRadps < -kSignEpsilon);
            const int leftTorqueSign =
                (leftWheelTorqueRequestNm > kSignEpsilon) -
                (leftWheelTorqueRequestNm < -kSignEpsilon);
            leftFrictionTorqueNm +=
                params.rollingFrictionTorqueNm *
                static_cast<float>((leftSpeedSign != 0) ? leftSpeedSign : leftTorqueSign);

            const int rightSpeedSign =
                (rightMotorSpeedRadps > kSignEpsilon) -
                (rightMotorSpeedRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightWheelTorqueRequestNm > kSignEpsilon) -
                (rightWheelTorqueRequestNm < -kSignEpsilon);
            rightFrictionTorqueNm +=
                params.rollingFrictionTorqueNm *
                static_cast<float>((rightSpeedSign != 0) ? rightSpeedSign : rightTorqueSign);
        }

        return CommandVector(
            _leftDrive.getCommandFromTorque(
                leftWheelTorqueRequestNm + leftFrictionTorqueNm,
                leftMotorSpeedRadps,
                params.supplyVoltageV),
            _rightDrive.getCommandFromTorque(
                rightWheelTorqueRequestNm + rightFrictionTorqueNm,
                rightMotorSpeedRadps,
                params.supplyVoltageV));
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
                _vehicle,
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
        const float halfTrackWidthM = 0.5f * ResolvePhysicalTrackWidthM(_vehicle, _preparedParams);
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
        const float trackWidthM = ResolvePhysicalTrackWidthM(_vehicle, _preparedParams);
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
        const float trackWidthM = ResolvePhysicalTrackWidthM(_vehicle, _preparedParams);
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
        float& maxYawAccelRadps2) const noexcept
    {
        velocityTargetTechnicalLimits(
            BuildBoundStateVector(),
            _preparedParams,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        const StateVector& currentState,
        const PlantParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        velocityTargetTechnicalLimits(
            currentState,
            prepared,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        const StateVector& currentState,
        const PreparedParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        maxLongitudinalAccelMps2 = 0.0f;
        maxYawAccelRadps2 = 0.0f;

        if (!(std::isfinite(params.wheelRadiusM) &&
            std::isfinite(params.trackWidthM) &&
            (params.wheelRadiusM > params.forceEpsilonN)))
        {
            return;
        }

        const StateVector operatingState = BuildDriveCommandOperatingState(currentState, params);
        const float leftBankSpeedRadps =
            (operatingState(VehicleState::kU) + (params.halfTrackWidthM * operatingState(VehicleState::kR))) *
            params.invWheelRadiusM;
        const float rightBankSpeedRadps =
            (operatingState(VehicleState::kU) - (params.halfTrackWidthM * operatingState(VehicleState::kR))) *
            params.invWheelRadiusM;
        const auto BankForceCapacityN =
            [&params](const MotorEncoderDrive& drive, float wheelBankSpeedRadps) noexcept
        {
            const float positiveTorqueNm =
                drive.getTorqueFromCommand(1.0f, wheelBankSpeedRadps, params.supplyVoltageV);
            const float negativeTorqueNm =
                drive.getTorqueFromCommand(-1.0f, wheelBankSpeedRadps, params.supplyVoltageV);
            const float symmetricTorqueNm =
                (std::min)(std::fabs(positiveTorqueNm), std::fabs(negativeTorqueNm));
            return
                (std::isfinite(symmetricTorqueNm) && (params.wheelRadiusM > params.forceEpsilonN)) ?
                (symmetricTorqueNm / params.wheelRadiusM) :
                0.0f;
        };
        const float leftBankForceN = BankForceCapacityN(_leftDrive, leftBankSpeedRadps);
        const float rightBankForceN = BankForceCapacityN(_rightDrive, rightBankSpeedRadps);
        const float maxBankForceN =
            (std::min)(
                (std::max)(0.0f, leftBankForceN),
                (std::max)(0.0f, rightBankForceN));
        maxLongitudinalAccelMps2 =
            (2.0f * maxBankForceN) * params.invLongitudinalMassKg;
        maxYawAccelRadps2 =
            (params.trackWidthM * maxBankForceN) * params.invYawInertiaKgM2;
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float forwardVelocityMps,
        float yawRateRadps,
        const PlantParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        const PreparedParams prepared = Prepare(params);
        velocityTargetTechnicalLimits(
            forwardVelocityMps,
            yawRateRadps,
            prepared,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float forwardVelocityMps,
        float yawRateRadps,
        const PreparedParams& params,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        velocityTargetTechnicalLimits(
            BuildReducedDriveCommandOperatingState(forwardVelocityMps, yawRateRadps, params),
            params,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
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


