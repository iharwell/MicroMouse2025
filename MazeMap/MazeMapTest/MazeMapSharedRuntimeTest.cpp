#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapSharedRuntime.h"
#include "..\MazeMap\Vehicle.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMapApp
{
    TEST_CLASS(MazeMapSharedRuntimeTest)
    {
    public:
        TEST_METHOD(SharedRuntime_SearchVehicleUsesConservativeSearchProfile)
        {
            Internal::SharedRobotRuntime runtime;

            const MazeMap::Vehicle& speedVehicle = runtime.SpeedVehicle();
            const MazeMap::Vehicle& searchVehicle = runtime.SearchVehicle();

            Assert::IsTrue(searchVehicle.GetMaxSpeed() < speedVehicle.GetMaxSpeed());
            Assert::IsTrue(searchVehicle.GetMaxForwardAcceleration() < speedVehicle.GetMaxForwardAcceleration());
            Assert::IsTrue(searchVehicle.GetMaxLateralAcceleration() < speedVehicle.GetMaxLateralAcceleration());
        }

        TEST_METHOD(SharedRuntime_ProvidesDistinctDiagnosticAndTelemetrySensorPipelines)
        {
            Internal::SharedRobotRuntime runtime;

            Assert::IsTrue(&runtime.DiagnosticSensors() != &runtime.TelemetrySensors());
            Assert::IsTrue(&runtime.Drive() == &runtime.Drive());
        }

        TEST_METHOD(SharedRuntime_SingletonIsStable)
        {
            Internal::SharedRobotRuntime& first = Internal::GetSharedRobotRuntime();
            Internal::SharedRobotRuntime& second = Internal::GetSharedRobotRuntime();

            Assert::IsTrue(&first == &second);
        }
    };
}
