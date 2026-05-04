#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\BootModeRegistry.h"
#include "..\MazeMap\MazeMapApplicationRuntime.h"
#include "..\MazeMap\Pins.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\PinPairStrap.h"
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    namespace
    {
        constexpr std::size_t kTestBootSelectorCapacity = 8U;
        bool gActiveBootSelectors[kTestBootSelectorCapacity] = {};

        void ClearActiveBootSelectors()
        {
            for (std::size_t index = 0U; index < kTestBootSelectorCapacity; ++index)
            {
                gActiveBootSelectors[index] = false;
            }
        }

        void ActivateBootSelector(uint8_t pinA, uint8_t pinB)
        {
            for (std::size_t index = 0U; index < GetBootModeRegistryEntryCount(); ++index)
            {
                const BootModeSelectorCondition& selector = GetBootModeRegistryEntry(index).selector;
                if (index < kTestBootSelectorCapacity &&
                    selector.kind == BootModeSelectorKind::PinPair &&
                    selector.pinA == pinA &&
                    selector.pinB == pinB)
                {
                    gActiveBootSelectors[index] = true;
                }
            }
        }

        bool ReadActiveBootSelector(const BootModeSelectorCondition& requested)
        {
            for (std::size_t index = 0U; index < GetBootModeRegistryEntryCount(); ++index)
            {
                const BootModeSelectorCondition& selector = GetBootModeRegistryEntry(index).selector;
                if (selector.kind == requested.kind &&
                    selector.pinA == requested.pinA &&
                    selector.pinB == requested.pinB)
                {
                    return index < kTestBootSelectorCapacity && gActiveBootSelectors[index];
                }
            }
            return false;
        }
    }

    TEST_CLASS(ApplicationTest)
    {
    public:
        TEST_METHOD_INITIALIZE(ResetHostPins)
        {
            HostResetDigitalPins();
            ClearActiveBootSelectors();
        }

        TEST_METHOD(BootModeRegistry_ExposesCurrentInventory)
        {
            Assert::IsTrue(GetBootModeRegistryEntryCount() == 8U);
            Assert::IsTrue(GetBootModeRegistryEntryCount() <= kTestBootSelectorCapacity);
            Assert::IsTrue(GetBootModeRegistryEntry(0U).selector.pinA == 39U);
            Assert::IsTrue(GetBootModeRegistryEntry(0U).selector.pinB == 40U);
            Assert::IsTrue(GetBootModeRegistryEntry(1U).selector.pinA == 38U);
            Assert::IsTrue(GetBootModeRegistryEntry(1U).selector.pinB == 39U);
            Assert::IsTrue(GetBootModeRegistryEntry(2U).selector.pinA == 28U);
            Assert::IsTrue(GetBootModeRegistryEntry(2U).selector.pinB == 29U);
            Assert::IsTrue(GetBootModeRegistryEntry(3U).selector.pinA == 29U);
            Assert::IsTrue(GetBootModeRegistryEntry(3U).selector.pinB == 30U);
            Assert::IsTrue(GetBootModeRegistryEntry(4U).selector.pinA == 26U);
            Assert::IsTrue(GetBootModeRegistryEntry(4U).selector.pinB == 27U);
            Assert::IsTrue(GetBootModeRegistryEntry(5U).selector.pinA == 27U);
            Assert::IsTrue(GetBootModeRegistryEntry(5U).selector.pinB == 28U);
            Assert::IsTrue(GetBootModeRegistryEntry(6U).selector.pinA == 9U);
            Assert::IsTrue(GetBootModeRegistryEntry(6U).selector.pinB == 10U);
            Assert::IsTrue(GetBootModeRegistryEntry(7U).selector.kind == BootModeSelectorKind::Fallback);
        }

        TEST_METHOD(BootModeRegistry_DescriptorsAreAuthoritative)
        {
            for (std::size_t index = 0U; index < GetBootModeRegistryEntryCount(); ++index)
            {
                const BootModeRegistryEntry& entry = GetBootModeRegistryEntry(index);
                Assert::IsNotNull(entry.descriptor);
                Assert::IsTrue(entry.descriptor->id == entry.id);
                Assert::IsNotNull(entry.descriptor->stableId);
                Assert::IsTrue(entry.descriptor->stableId[0] != '\0');
                Assert::IsTrue(entry.descriptor->entryMode != nullptr);
                Assert::IsNotNull(entry.descriptor->entryPoint);
                Assert::IsTrue(entry.descriptor->entryPoint[0] != '\0');
                Assert::IsNotNull(entry.descriptor->implementationFile);
                Assert::IsTrue(entry.descriptor->implementationFile[0] != '\0');
            }
        }

        TEST_METHOD(ResolveActiveApplicationMode_UsesDescriptorEntryMode)
        {
            HostSetPinShort(26U, 27U);

            const BootModeRegistryEntry& selectedMode = ResolveSelectedBootMode();
            MazeMap::App::Internal::IApplicationMode& expected = selectedMode.descriptor->entryMode();
            MazeMap::App::Internal::IApplicationMode& actual = MazeMap::App::Internal::ResolveActiveApplicationMode();

            Assert::IsTrue(&actual == &expected);
        }

        TEST_METHOD(BootModeRegistry_DefaultsToMission)
        {
            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::Mission);
        }

        TEST_METHOD(BootModeRegistry_PrefersFrontWallCharacterization)
        {
            ActivateBootSelector(39U, 40U);
            ActivateBootSelector(38U, 39U);
            ActivateBootSelector(28U, 29U);
            ActivateBootSelector(29U, 30U);
            ActivateBootSelector(26U, 27U);
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::FrontWallCharacterization);
        }

        TEST_METHOD(BootModeRegistry_PrefersLedCalibrationOverLaterModes)
        {
            ActivateBootSelector(38U, 39U);
            ActivateBootSelector(28U, 29U);
            ActivateBootSelector(29U, 30U);
            ActivateBootSelector(26U, 27U);
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::WallSensorLedCalibration);
        }

        TEST_METHOD(BootModeRegistry_PrefersAuxiliarySelectorOverMissionModes)
        {
            ActivateBootSelector(28U, 29U);
            ActivateBootSelector(29U, 30U);
            ActivateBootSelector(26U, 27U);
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == GetBootModeRegistryEntry(2U).descriptor->id);
        }

        TEST_METHOD(BootModeRegistry_PrefersManeuverFileTestOverLaterModes)
        {
            ActivateBootSelector(29U, 30U);
            ActivateBootSelector(26U, 27U);
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::ManeuverFileTest);
        }

        TEST_METHOD(BootModeRegistry_PrefersTopSpeedMeasurementOverLaterModes)
        {
            ActivateBootSelector(26U, 27U);
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::TopSpeedMeasurement);
        }

        TEST_METHOD(BootModeRegistry_PrefersOpenFloorMeasurementOverShowcasingDonut)
        {
            ActivateBootSelector(27U, 28U);
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::OpenFloorMeasurement);
        }

        TEST_METHOD(BootModeRegistry_SelectsShowcasingDonutWhenPins9And10AreStrapped)
        {
            ActivateBootSelector(9U, 10U);

            Assert::IsTrue(ResolveSelectedBootMode(&ReadActiveBootSelector).descriptor->id == BootModeId::ShowcasingDonut);
        }

        TEST_METHOD(HostPinShortsDrivePullupInputsLowForTesting)
        {
            HostSetPinShort(38U, 39U);
            pinMode(38U, OUTPUT);
            digitalWrite(38U, LOW);
            pinMode(39U, INPUT_PULLUP);
            Assert::AreEqual(LOW, digitalRead(39U));

            pinMode(38U, INPUT_PULLUP);
            pinMode(39U, OUTPUT);
            digitalWrite(39U, LOW);
            Assert::AreEqual(LOW, digitalRead(38U));

            HostSetPinShort(38U, 39U, false);
            Assert::AreEqual(HIGH, digitalRead(38U));
        }

        TEST_METHOD(PinPairStrapMonitorDetectsLiveRemoval)
        {
            HostSetPinShort(27U, 28U);
            BeginPinPairStrapMonitor(27U, 28U);
            Assert::IsTrue(IsPinPairStrapMonitorClosed(28U));

            HostSetPinShort(27U, 28U, false);
            Assert::IsFalse(IsPinPairStrapMonitorClosed(28U));

            EndPinPairStrapMonitor(27U, 28U);
        }

    };
}

