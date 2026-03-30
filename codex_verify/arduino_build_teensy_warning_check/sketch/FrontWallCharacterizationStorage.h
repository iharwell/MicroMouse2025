#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\FrontWallCharacterizationStorage.h"
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    constexpr uint16_t kFrontWallCharacterizationMaxStoredSamples = 128U;
    constexpr uint32_t kFrontWallCharacterizationStorageMagic = 0x46574356UL; // "FWCV"
    constexpr uint16_t kFrontWallCharacterizationStorageVersion = 1U;

    struct FrontWallCharacterizationStorage
    {
        uint32_t magic = kFrontWallCharacterizationStorageMagic;
        uint16_t version = kFrontWallCharacterizationStorageVersion;
        uint16_t sampleCount = 0U;
        float distanceStepM = 0.0f;
        float commandedReverseSpeedMps = 0.0f;
        float zeroThresholdDifferentialLight = 0.0f;
        float terminalDistanceM = 0.0f;
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> distanceM{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontLeftAmbientLight{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontLeftLitLight{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontLeftDifferentialLight{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontRightAmbientLight{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontRightLitLight{};
        std::array<float, kFrontWallCharacterizationMaxStoredSamples> frontRightDifferentialLight{};
        uint32_t checksum = 0UL;
    };

    struct FrontWallCharacterizationMatch
    {
        bool valid = false;
        uint16_t sampleCount = 0U;
        float scale = 0.0f;
        float normalizedCorrelation = 0.0f;
        float relativeResidual = 1.0f;
    };

    static_assert(
        sizeof(FrontWallCharacterizationStorage) <= 4096U,
        "Front wall characterization storage must fit comfortably inside Teensy EEPROM.");

    inline uint32_t ComputeFrontWallCharacterizationStorageChecksum(
        const FrontWallCharacterizationStorage& storage)
    {
        constexpr uint32_t kFnvOffsetBasis = 2166136261UL;
        constexpr uint32_t kFnvPrime = 16777619UL;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&storage);
        const size_t length = offsetof(FrontWallCharacterizationStorage, checksum);
        uint32_t hash = kFnvOffsetBasis;
        for (size_t index = 0U; index < length; ++index)
        {
            hash ^= bytes[index];
            hash *= kFnvPrime;
        }
        return hash;
    }

    inline void FinalizeFrontWallCharacterizationStorage(
        FrontWallCharacterizationStorage& storage)
    {
        storage.magic = kFrontWallCharacterizationStorageMagic;
        storage.version = kFrontWallCharacterizationStorageVersion;
        storage.checksum = ComputeFrontWallCharacterizationStorageChecksum(storage);
    }

    inline bool IsValidFrontWallCharacterizationStorage(
        const FrontWallCharacterizationStorage& storage)
    {
        if (storage.magic != kFrontWallCharacterizationStorageMagic ||
            storage.version != kFrontWallCharacterizationStorageVersion ||
            storage.sampleCount == 0U ||
            storage.sampleCount > kFrontWallCharacterizationMaxStoredSamples ||
            !std::isfinite(storage.distanceStepM) ||
            storage.distanceStepM <= 0.0f ||
            !std::isfinite(storage.commandedReverseSpeedMps) ||
            storage.commandedReverseSpeedMps <= 0.0f ||
            !std::isfinite(storage.zeroThresholdDifferentialLight) ||
            storage.zeroThresholdDifferentialLight < 0.0f ||
            !std::isfinite(storage.terminalDistanceM) ||
            storage.terminalDistanceM < 0.0f ||
            storage.checksum != ComputeFrontWallCharacterizationStorageChecksum(storage))
        {
            return false;
        }

        float previousDistanceM = -1.0f;
        for (uint16_t index = 0U; index < storage.sampleCount; ++index)
        {
            const float distanceM = storage.distanceM[index];
            if (!std::isfinite(distanceM) ||
                distanceM < 0.0f ||
                (index > 0U && distanceM < previousDistanceM))
            {
                return false;
            }
            previousDistanceM = distanceM;

            if (!std::isfinite(storage.frontLeftAmbientLight[index]) ||
                !std::isfinite(storage.frontLeftLitLight[index]) ||
                !std::isfinite(storage.frontLeftDifferentialLight[index]) ||
                !std::isfinite(storage.frontRightAmbientLight[index]) ||
                !std::isfinite(storage.frontRightLitLight[index]) ||
                !std::isfinite(storage.frontRightDifferentialLight[index]) ||
                storage.frontLeftAmbientLight[index] < 0.0f ||
                storage.frontLeftLitLight[index] < 0.0f ||
                storage.frontLeftDifferentialLight[index] < 0.0f ||
                storage.frontRightAmbientLight[index] < 0.0f ||
                storage.frontRightLitLight[index] < 0.0f ||
                storage.frontRightDifferentialLight[index] < 0.0f)
            {
                return false;
            }
        }

        return storage.terminalDistanceM >= storage.distanceM[storage.sampleCount - 1U];
    }

    inline float EstimateFrontWallCharacterizationChannelFloor(
        const FrontWallCharacterizationStorage& storage,
        bool useFrontRightChannel)
    {
        if (!IsValidFrontWallCharacterizationStorage(storage))
        {
            return 0.0f;
        }

        const std::array<float, kFrontWallCharacterizationMaxStoredSamples>& differentialLight =
            useFrontRightChannel ?
            storage.frontRightDifferentialLight :
            storage.frontLeftDifferentialLight;

        const uint16_t tailCount =
            (storage.sampleCount < 8U) ?
            storage.sampleCount :
            8U;
        if (tailCount == 0U)
        {
            return 0.0f;
        }

        double sum = 0.0;
        uint16_t validCount = 0U;
        for (uint16_t offset = 0U; offset < tailCount; ++offset)
        {
            const uint16_t index = static_cast<uint16_t>(storage.sampleCount - 1U - offset);
            const float value = differentialLight[index];
            if (!std::isfinite(value) || value < 0.0f)
            {
                continue;
            }

            sum += static_cast<double>(value);
            ++validCount;
        }

        if (validCount == 0U)
        {
            return 0.0f;
        }

        return static_cast<float>(sum / static_cast<double>(validCount));
    }

    inline bool TrySampleFrontWallCharacterizationDifferentialLight(
        const FrontWallCharacterizationStorage& storage,
        bool useFrontRightChannel,
        float distanceM,
        float& differentialLight)
    {
        differentialLight = 0.0f;
        if (!IsValidFrontWallCharacterizationStorage(storage) ||
            !std::isfinite(distanceM) ||
            distanceM < 0.0f)
        {
            return false;
        }

        const std::array<float, kFrontWallCharacterizationMaxStoredSamples>& channel =
            useFrontRightChannel ?
            storage.frontRightDifferentialLight :
            storage.frontLeftDifferentialLight;

        if (storage.sampleCount == 1U)
        {
            differentialLight = channel[0];
            return std::isfinite(differentialLight) && differentialLight >= 0.0f;
        }

        if (distanceM <= storage.distanceM[0U])
        {
            differentialLight = channel[0U];
            return std::isfinite(differentialLight) && differentialLight >= 0.0f;
        }

        const uint16_t lastIndex = static_cast<uint16_t>(storage.sampleCount - 1U);
        if (distanceM >= storage.distanceM[lastIndex])
        {
            differentialLight = channel[lastIndex];
            return std::isfinite(differentialLight) && differentialLight >= 0.0f;
        }

        for (uint16_t index = 1U; index < storage.sampleCount; ++index)
        {
            const float previousDistanceM = storage.distanceM[index - 1U];
            const float nextDistanceM = storage.distanceM[index];
            if (distanceM > nextDistanceM)
            {
                continue;
            }

            if (!(std::isfinite(previousDistanceM) &&
                std::isfinite(nextDistanceM) &&
                nextDistanceM > previousDistanceM))
            {
                return false;
            }

            const float previousLight = channel[index - 1U];
            const float nextLight = channel[index];
            if (!(std::isfinite(previousLight) &&
                std::isfinite(nextLight) &&
                previousLight >= 0.0f &&
                nextLight >= 0.0f))
            {
                return false;
            }

            const float fraction = (distanceM - previousDistanceM) / (nextDistanceM - previousDistanceM);
            differentialLight = previousLight + (fraction * (nextLight - previousLight));
            return std::isfinite(differentialLight) && differentialLight >= 0.0f;
        }

        return false;
    }

    inline bool TryMatchFrontWallCharacterizationChannel(
        const FrontWallCharacterizationStorage& storage,
        bool useFrontRightChannel,
        const float* measuredDifferentialLight,
        const float* expectedDistanceM,
        uint16_t sampleCount,
        float signalBaseline,
        FrontWallCharacterizationMatch& match)
    {
        match = FrontWallCharacterizationMatch{};
        if (!IsValidFrontWallCharacterizationStorage(storage) ||
            measuredDifferentialLight == nullptr ||
            expectedDistanceM == nullptr ||
            sampleCount == 0U ||
            !std::isfinite(signalBaseline) ||
            signalBaseline < 0.0f)
        {
            return false;
        }

        const float floorDifferentialLight =
            EstimateFrontWallCharacterizationChannelFloor(storage, useFrontRightChannel);
        constexpr float kMinTemplateRise = 1.0e-5f;
        constexpr float kMinEnergy = 1.0e-7f;

        double templateEnergy = 0.0;
        double measuredEnergy = 0.0;
        double dotProduct = 0.0;
        uint16_t validCount = 0U;
        for (uint16_t index = 0U; index < sampleCount; ++index)
        {
            const float measuredValue = measuredDifferentialLight[index];
            const float distanceM = expectedDistanceM[index];
            float templateDifferentialLight = 0.0f;
            if (!std::isfinite(measuredValue) ||
                !std::isfinite(distanceM) ||
                distanceM < 0.0f ||
                !TrySampleFrontWallCharacterizationDifferentialLight(
                    storage,
                    useFrontRightChannel,
                    distanceM,
                    templateDifferentialLight))
            {
                continue;
            }

            const float templateRise =
                (templateDifferentialLight > floorDifferentialLight) ?
                (templateDifferentialLight - floorDifferentialLight) :
                0.0f;
            if (!(std::isfinite(templateRise) && templateRise > kMinTemplateRise))
            {
                continue;
            }

            const float measuredRise =
                (measuredValue > signalBaseline) ?
                (measuredValue - signalBaseline) :
                0.0f;
            if (!std::isfinite(measuredRise))
            {
                continue;
            }

            templateEnergy += static_cast<double>(templateRise) * static_cast<double>(templateRise);
            measuredEnergy += static_cast<double>(measuredRise) * static_cast<double>(measuredRise);
            dotProduct += static_cast<double>(measuredRise) * static_cast<double>(templateRise);
            ++validCount;
        }

        if (validCount == 0U || templateEnergy <= kMinEnergy)
        {
            return false;
        }

        const double scale = (dotProduct > 0.0) ? (dotProduct / templateEnergy) : 0.0;
        double residualEnergy = 0.0;
        for (uint16_t index = 0U; index < sampleCount; ++index)
        {
            const float measuredValue = measuredDifferentialLight[index];
            const float distanceM = expectedDistanceM[index];
            float templateDifferentialLight = 0.0f;
            if (!std::isfinite(measuredValue) ||
                !std::isfinite(distanceM) ||
                distanceM < 0.0f ||
                !TrySampleFrontWallCharacterizationDifferentialLight(
                    storage,
                    useFrontRightChannel,
                    distanceM,
                    templateDifferentialLight))
            {
                continue;
            }

            const float templateRise =
                (templateDifferentialLight > floorDifferentialLight) ?
                (templateDifferentialLight - floorDifferentialLight) :
                0.0f;
            if (!(std::isfinite(templateRise) && templateRise > kMinTemplateRise))
            {
                continue;
            }

            const float measuredRise =
                (measuredValue > signalBaseline) ?
                (measuredValue - signalBaseline) :
                0.0f;
            if (!std::isfinite(measuredRise))
            {
                continue;
            }

            const double error =
                static_cast<double>(measuredRise) -
                (scale * static_cast<double>(templateRise));
            residualEnergy += error * error;
        }

        match.valid = true;
        match.sampleCount = validCount;
        match.scale = static_cast<float>(scale);
        if (measuredEnergy > kMinEnergy && dotProduct > 0.0)
        {
            match.normalizedCorrelation = static_cast<float>(
                dotProduct / std::sqrt(templateEnergy * measuredEnergy));
            match.relativeResidual = static_cast<float>(residualEnergy / measuredEnergy);
        }
        else
        {
            match.normalizedCorrelation = 0.0f;
            match.relativeResidual = (residualEnergy <= kMinEnergy) ? 0.0f : 1.0f;
        }

        return
            std::isfinite(match.scale) &&
            std::isfinite(match.normalizedCorrelation) &&
            std::isfinite(match.relativeResidual);
    }
}
