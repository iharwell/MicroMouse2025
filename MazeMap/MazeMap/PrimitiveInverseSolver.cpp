#include "pch.h"
#include "PrimitiveInverseSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MazeMap
{
    void PrimitiveInverseSolver::solvePrimitiveSamples(
        const PrimitiveDefinition& primitive,
        const OperatingPoint& operatingPoint,
        const PlantModel& plant,
        const PlantParams& params,
        PrimitiveSample* samples,
        size_t maxSamples,
        uint8_t& outCount) const noexcept
    {
        outCount = 0U;
        if (samples == nullptr || maxSamples == 0U)
        {
            return;
        }

        if (!(primitive.durationS > 0.0f) || !(primitive.sampleCount > 0U))
        {
            return;
        }

        const size_t cappedMaxSamples = (std::min)(maxSamples, static_cast<size_t>(std::numeric_limits<uint8_t>::max()));
        const uint8_t count =
            (primitive.sampleCount > cappedMaxSamples) ?
            static_cast<uint8_t>(cappedMaxSamples) :
            primitive.sampleCount;
        outCount = count;

        const float durationS = primitive.durationS;
        const float linearAccelMps2 =
            (operatingPoint.exitSpeedMps - operatingPoint.entrySpeedMps) / durationS;
        const float trackWidthM = params.trackWidthM;
        const float wheelRadiusM = params.wheelRadiusM;

        for (uint8_t index = 0U; index < count; ++index)
        {
            PrimitiveSample& sample = samples[index];
            const float alpha = (count > 1U) ?
                (static_cast<float>(index) / static_cast<float>(count - 1U)) :
                0.0f;
            sample.timeS = durationS * alpha;
            sample.arcLengthM = primitive.pathLengthM * alpha;
            sample.linearSpeedMps =
                operatingPoint.entrySpeedMps +
                (alpha * (operatingPoint.exitSpeedMps - operatingPoint.entrySpeedMps));
            sample.yawRateRadps = primitive.curvatureInvM * sample.linearSpeedMps;

            const float angularAccelRadps2 = primitive.curvatureInvM * linearAccelMps2;
            const float leftSpeedMps = sample.linearSpeedMps + (0.5f * trackWidthM * sample.yawRateRadps);
            const float rightSpeedMps = sample.linearSpeedMps - (0.5f * trackWidthM * sample.yawRateRadps);
            const float leftWheelSpeedRadps = leftSpeedMps / wheelRadiusM;
            const float rightWheelSpeedRadps = rightSpeedMps / wheelRadiusM;
            const float leftWheelAccelRadps2 =
                (linearAccelMps2 + (0.5f * trackWidthM * angularAccelRadps2)) / wheelRadiusM;
            const float rightWheelAccelRadps2 =
                (linearAccelMps2 - (0.5f * trackWidthM * angularAccelRadps2)) / wheelRadiusM;

            const float longitudinalForcePerSideN = 0.5f * params.massKg * linearAccelMps2;
            const float yawForcePerSideN =
                (trackWidthM > 0.0f) ?
                ((params.yawInertiaKgM2 * angularAccelRadps2) / trackWidthM) :
                0.0f;
            const float leftGroundForceN = longitudinalForcePerSideN + yawForcePerSideN;
            const float rightGroundForceN = longitudinalForcePerSideN - yawForcePerSideN;

            const float leftWheelTorqueNm =
                (params.equivalentWheelInertiaKgM2 * leftWheelAccelRadps2) +
                (params.wheelRadiusM * leftGroundForceN) +
                plant.driveFrictionTorque(leftWheelSpeedRadps, params);
            const float rightWheelTorqueNm =
                (params.equivalentWheelInertiaKgM2 * rightWheelAccelRadps2) +
                (params.wheelRadiusM * rightGroundForceN) +
                plant.driveFrictionTorque(rightWheelSpeedRadps, params);

            sample.leftMotorCommand = inverseMotorCommand(leftWheelTorqueNm, leftWheelSpeedRadps, params);
            sample.rightMotorCommand = inverseMotorCommand(rightWheelTorqueNm, rightWheelSpeedRadps, params);
        }
    }

    float PrimitiveInverseSolver::inverseMotorCommand(
        float wheelTorqueNm,
        float wheelSpeedRadps,
        const PlantParams& params) noexcept
    {
        const float motorTorqueNm =
            (params.drivetrainEfficiency > 0.0f) ?
            (wheelTorqueNm / ((std::max)(params.gearRatio * params.drivetrainEfficiency, 1.0e-4f))) :
            0.0f;
        float motorCurrentA =
            (params.torqueConstantNmPerA > 0.0f) ?
            (motorTorqueNm / params.torqueConstantNmPerA) :
            0.0f;
        if (motorCurrentA > 0.0f)
        {
            motorCurrentA += params.noLoadCurrentA;
        }
        else if (motorCurrentA < 0.0f)
        {
            motorCurrentA -= params.noLoadCurrentA;
        }

        const float motorSpeedRadps = wheelSpeedRadps * params.gearRatio;
        const float backEmfVoltageV =
            (params.speedConstantRadpsPerVolt > 0.0f) ?
            (motorSpeedRadps / params.speedConstantRadpsPerVolt) :
            0.0f;
        const float commandedVoltageV =
            (motorCurrentA * params.driveResistanceOhms) + backEmfVoltageV;
        if (!(params.supplyVoltageV > 0.0f))
        {
            return 0.0f;
        }
        return (std::clamp)(commandedVoltageV / params.supplyVoltageV, -1.0f, 1.0f);
    }
}
