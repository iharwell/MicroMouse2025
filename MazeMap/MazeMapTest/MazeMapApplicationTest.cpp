#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapApplication.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\WallSensorLedCalibrationPhase.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMapApp
{
    TEST_CLASS(MazeMapApplicationTest)
    {
    public:
        TEST_METHOD_INITIALIZE(ResetHostPins)
        {
            HostResetDigitalPins();
        }

        TEST_METHOD(ResolveStartupMode_DefaultsToMission)
        {
            const StartupModeRequests requests{};
            Assert::IsTrue(ResolveStartupMode(requests) == StartupMode::Mission);
        }

        TEST_METHOD(ResolveStartupMode_PrefersFrontWallCharacterization)
        {
            StartupModeRequests requests{};
            requests.frontWallCharacterization = true;
            requests.wallSensorLedCalibration = true;
            requests.auxiliaryMeasurement = true;
            requests.maneuverFileTest = true;
            requests.primaryDiagnostic = true;

            Assert::IsTrue(ResolveStartupMode(requests) == StartupMode::FrontWallCharacterization);
        }

        TEST_METHOD(ResolveStartupMode_PrefersLedCalibrationOverLaterModes)
        {
            StartupModeRequests requests{};
            requests.wallSensorLedCalibration = true;
            requests.auxiliaryMeasurement = true;
            requests.maneuverFileTest = true;
            requests.primaryDiagnostic = true;

            Assert::IsTrue(ResolveStartupMode(requests) == StartupMode::WallSensorLedCalibration);
        }

        TEST_METHOD(ResolveStartupMode_PrefersAuxiliaryMeasurementOverMissionModes)
        {
            StartupModeRequests requests{};
            requests.auxiliaryMeasurement = true;
            requests.maneuverFileTest = true;
            requests.primaryDiagnostic = true;

            Assert::IsTrue(ResolveStartupMode(requests) == StartupMode::AuxiliaryMeasurement);
        }

        TEST_METHOD(ResolveStartupMode_PrefersManeuverFileTestOverDiagnostic)
        {
            StartupModeRequests requests{};
            requests.maneuverFileTest = true;
            requests.primaryDiagnostic = true;

            Assert::IsTrue(ResolveStartupMode(requests) == StartupMode::ManeuverFileTest);
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

        TEST_METHOD(AdvanceWallSensorLedCalibrationPhase_CompletesAfterSideCapture)
        {
            Assert::IsTrue(
                AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Front, true) ==
                WallSensorLedCalibrationPhase::Front);
            Assert::IsTrue(
                AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Front, false) ==
                WallSensorLedCalibrationPhase::Side);
            Assert::IsTrue(
                AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Side, false) ==
                WallSensorLedCalibrationPhase::Side);
            Assert::IsTrue(
                AdvanceWallSensorLedCalibrationPhase(WallSensorLedCalibrationPhase::Side, true) ==
                WallSensorLedCalibrationPhase::Complete);
        }
    };
}
