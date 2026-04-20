#include "pch.h"
#include "BootModeRegistry.h"

#include "AuxMeasurementConfig.h"
#include "MazeMapControllerRegistry.h"
#include "PinPairStrap.h"

namespace
{
    constexpr std::size_t kBootModeRegistryEntryCount = 7U;

    constexpr MazeMap::App::BootModeId GetAuxiliarySelectorBootModeId() noexcept
    {
        if constexpr (
            MazeMap::AuxMeasurementConfig::kBootSelectedMode ==
            MazeMap::AuxMeasurementConfig::BootSelectedMode::CorridorRepeatability)
        {
            return MazeMap::App::BootModeId::CorridorRepeatability;
        }

        if constexpr (
            MazeMap::AuxMeasurementConfig::kBootSelectedMode ==
            MazeMap::AuxMeasurementConfig::BootSelectedMode::PositionAccuracyAudit)
        {
            return MazeMap::App::BootModeId::PositionAccuracyAudit;
        }

        return MazeMap::App::BootModeId::AuxiliaryMeasurement;
    }

    const MazeMap::App::BootModeDescriptor& GetAuxiliarySelectorDescriptor() noexcept
    {
        if constexpr (
            MazeMap::AuxMeasurementConfig::kBootSelectedMode ==
            MazeMap::AuxMeasurementConfig::BootSelectedMode::CorridorRepeatability)
        {
            return MazeMap::App::Internal::GetCorridorRepeatabilityBootModeDescriptor();
        }

        if constexpr (
            MazeMap::AuxMeasurementConfig::kBootSelectedMode ==
            MazeMap::AuxMeasurementConfig::BootSelectedMode::PositionAccuracyAudit)
        {
            return MazeMap::App::Internal::GetPositionAccuracyAuditBootModeDescriptor();
        }

        return MazeMap::App::Internal::GetAuxMeasurementBootModeDescriptor();
    }
}

namespace MazeMap::App
{
    const char* BootModeIdName(BootModeId id) noexcept
    {
        switch (id)
        {
        case BootModeId::FrontWallCharacterization:
            return "front_wall_characterization";
        case BootModeId::WallSensorLedCalibration:
            return "wall_sensor_led_calibration";
        case BootModeId::AuxiliaryMeasurement:
            return "auxiliary_measurement";
        case BootModeId::CorridorRepeatability:
            return "corridor_repeatability";
        case BootModeId::PositionAccuracyAudit:
            return "position_accuracy_audit";
        case BootModeId::ManeuverFileTest:
            return "maneuver_file_test";
        case BootModeId::TopSpeedMeasurement:
            return "top_speed_measurement";
        case BootModeId::OpenFloorMeasurement:
            return "open_floor_measurement";
        case BootModeId::Mission:
        default:
            return "mission";
        }
    }

    const BootModeRegistryEntry* GetBootModeRegistryEntries() noexcept
    {
        static const BootModeRegistryEntry entries[kBootModeRegistryEntryCount] = {
            BootModeRegistryEntry{
                BootModeId::FrontWallCharacterization,
                BootModeSelectorCondition::PinPair(39U, 40U),
                true,
                &Internal::GetFrontWallCharacterizationBootModeDescriptor(),
            },
            BootModeRegistryEntry{
                BootModeId::WallSensorLedCalibration,
                BootModeSelectorCondition::PinPair(38U, 39U),
                true,
                &Internal::GetWallSensorLedCalibrationBootModeDescriptor(),
            },
            BootModeRegistryEntry{
                GetAuxiliarySelectorBootModeId(),
                BootModeSelectorCondition::PinPair(28U, 29U),
                true,
                &GetAuxiliarySelectorDescriptor(),
            },
            BootModeRegistryEntry{
                BootModeId::ManeuverFileTest,
                BootModeSelectorCondition::PinPair(29U, 30U),
                true,
                &Internal::GetManeuverFileTestBootModeDescriptor(),
            },
            BootModeRegistryEntry{
                BootModeId::TopSpeedMeasurement,
                BootModeSelectorCondition::PinPair(26U, 27U),
                true,
                &Internal::GetTopSpeedMeasurementBootModeDescriptor(),
            },
            BootModeRegistryEntry{
                BootModeId::OpenFloorMeasurement,
                BootModeSelectorCondition::PinPair(27U, 28U),
                true,
                &Internal::GetOpenFloorMeasurementBootModeDescriptor(),
            },
            BootModeRegistryEntry{
                BootModeId::Mission,
                BootModeSelectorCondition::Fallback(),
                true,
                &Internal::GetMissionRunBootModeDescriptor(),
            },
        };
        return entries;
    }

    std::size_t GetBootModeRegistryEntryCount() noexcept
    {
        return kBootModeRegistryEntryCount;
    }

    const BootModeRegistryEntry& GetBootModeRegistryEntry(std::size_t index) noexcept
    {
        if (index >= kBootModeRegistryEntryCount)
        {
            index = kBootModeRegistryEntryCount - 1U;
        }
        return GetBootModeRegistryEntries()[index];
    }

    const BootModeRegistryEntry* FindBootModeRegistryEntry(BootModeId id) noexcept
    {
        const BootModeRegistryEntry* entries = GetBootModeRegistryEntries();
        for (std::size_t index = 0U; index < kBootModeRegistryEntryCount; ++index)
        {
            if (entries[index].id == id)
            {
                return &entries[index];
            }
        }
        return nullptr;
    }

    const BootModeRegistryEntry& ResolveSelectedBootMode(BootModeSelectorReader reader) noexcept
    {
        const BootModeRegistryEntry* entries = GetBootModeRegistryEntries();
        const BootModeRegistryEntry* fallback = &entries[kBootModeRegistryEntryCount - 1U];

        for (std::size_t index = 0U; index < kBootModeRegistryEntryCount; ++index)
        {
            const BootModeRegistryEntry& entry = entries[index];
            if (entry.selector.kind == BootModeSelectorKind::Fallback)
            {
                fallback = &entry;
                continue;
            }

            if (reader != nullptr && reader(entry.selector))
            {
                return entry;
            }
        }

        return *fallback;
    }

    const BootModeRegistryEntry& ResolveSelectedBootMode() noexcept
    {
        return ResolveSelectedBootMode(&IsBootModeSelectorConditionActive);
    }

    bool IsBootModeSelectorConditionActive(const BootModeSelectorCondition& selector) noexcept
    {
        switch (selector.kind)
        {
        case BootModeSelectorKind::PinPair:
            return IsPinPairStrapped(selector.pinA, selector.pinB);
        case BootModeSelectorKind::Fallback:
        default:
            return true;
        }
    }

    bool IsBootModeSelectorActive(BootModeId id) noexcept
    {
        const BootModeRegistryEntry* entry = FindBootModeRegistryEntry(id);
        return entry != nullptr && IsBootModeSelectorConditionActive(entry->selector);
    }
}
