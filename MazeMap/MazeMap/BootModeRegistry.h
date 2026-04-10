#pragma once

#include "BootModeDescriptor.h"

#include <cstddef>

namespace MazeMap::App
{
    enum class BootModeSelectorKind : uint8_t
    {
        Fallback,
        PinPair,
    };

    struct BootModeSelectorCondition
    {
        BootModeSelectorKind kind;
        uint8_t pinA;
        uint8_t pinB;

        static constexpr BootModeSelectorCondition Fallback() noexcept
        {
            return BootModeSelectorCondition{ BootModeSelectorKind::Fallback, 0U, 0U };
        }

        static constexpr BootModeSelectorCondition PinPair(uint8_t firstPin, uint8_t secondPin) noexcept
        {
            return BootModeSelectorCondition{ BootModeSelectorKind::PinPair, firstPin, secondPin };
        }
    };

    struct BootModeRegistryEntry
    {
        BootModeId id;
        BootModeSelectorCondition selector;
        bool rebootRequired;
        const BootModeDescriptor* descriptor;
    };

    using BootModeSelectorReader = bool (*)(const BootModeSelectorCondition& selector);

    EXPORT const char* BootModeIdName(BootModeId id) noexcept;
    EXPORT const BootModeRegistryEntry* GetBootModeRegistryEntries() noexcept;
    EXPORT std::size_t GetBootModeRegistryEntryCount() noexcept;
    EXPORT const BootModeRegistryEntry& GetBootModeRegistryEntry(std::size_t index) noexcept;
    EXPORT const BootModeRegistryEntry* FindBootModeRegistryEntry(BootModeId id) noexcept;
    EXPORT const BootModeRegistryEntry& ResolveSelectedBootMode(BootModeSelectorReader reader) noexcept;
    EXPORT const BootModeRegistryEntry& ResolveSelectedBootMode() noexcept;
    EXPORT bool IsBootModeSelectorConditionActive(const BootModeSelectorCondition& selector) noexcept;
    EXPORT bool IsBootModeSelectorActive(BootModeId id) noexcept;
}
