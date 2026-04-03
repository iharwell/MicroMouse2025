#pragma once
// Declares the maneuver primitive inverse solver used to back-drive plant targets into motor commands.

#include "PlantModel.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    // Geometric and timing targets for one maneuver primitive.
    struct PrimitiveDefinition
    {
        float pathLengthM = 0.0f;
        float curvatureInvM = 0.0f;
        float durationS = 0.10f;
        uint8_t sampleCount = 16U;
    };

    // Entry and exit operating conditions for an inverse-solved primitive.
    struct OperatingPoint
    {
        float entrySpeedMps = 0.0f;
        float exitSpeedMps = 0.0f;
        float fanDutyCycle = 0.80f;
    };

    // One inverse-solved primitive sample including commanded motor effort.
    struct PrimitiveSample
    {
        float timeS = 0.0f;
        float arcLengthM = 0.0f;
        float linearSpeedMps = 0.0f;
        float yawRateRadps = 0.0f;
        float leftMotorCommand = 0.0f;
        float rightMotorCommand = 0.0f;
    };

    // Fixed-capacity storage for primitive samples.
    template <size_t MaxSamples = 64U>
    struct PrimitiveTable
    {
        uint8_t count = 0U;
        std::array<PrimitiveSample, MaxSamples> samples{};
    };

    // Generates plant-consistent command tables for maneuver primitives.
    class EXPORT PrimitiveInverseSolver
    {
    public:
        template <size_t MaxSamples = 64U>
        PrimitiveTable<MaxSamples> solvePrimitive(
            const PrimitiveDefinition& primitive,
            const OperatingPoint& operatingPoint,
            const PlantModel& plant,
            const PlantParams& params) const noexcept
        {
            PrimitiveTable<MaxSamples> table{};
            solvePrimitiveSamples(
                primitive,
                operatingPoint,
                plant,
                params,
                table.samples.data(),
                MaxSamples,
                table.count);
            return table;
        }

    private:
        void solvePrimitiveSamples(
            const PrimitiveDefinition& primitive,
            const OperatingPoint& operatingPoint,
            const PlantModel& plant,
            const PlantParams& params,
            PrimitiveSample* samples,
            size_t maxSamples,
            uint8_t& outCount) const noexcept;

        static float inverseMotorCommand(float wheelTorqueNm, float wheelSpeedRadps, const PlantParams& params) noexcept;
    };
}
