#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelDynamicsTestSupport.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace PlantModelDynamicsTestSupport;
    TEST_CLASS(PlantModelFrictionAndSubthresholdTest)
    {
    public:
        TEST_METHOD(PositiveStaticFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"PositiveStaticFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << measurement.expectedStaticFrictionTorqueNm
                << L"\nactual=" << measurement.positiveStaticFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedStaticFrictionTorqueNm,
                measurement.positiveStaticFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(NegativeStaticFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"NegativeStaticFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << -measurement.expectedStaticFrictionTorqueNm
                << L"\nactual=" << measurement.negativeStaticFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -measurement.expectedStaticFrictionTorqueNm,
                measurement.negativeStaticFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PositiveRollingFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"PositiveRollingFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << measurement.expectedRollingFrictionTorqueNm
                << L"\nactual=" << measurement.positiveRollingFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedRollingFrictionTorqueNm,
                measurement.positiveRollingFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(NegativeRollingFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"NegativeRollingFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << -measurement.expectedRollingFrictionTorqueNm
                << L"\nactual=" << measurement.negativeRollingFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -measurement.expectedRollingFrictionTorqueNm,
                measurement.negativeRollingFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveForwardVelocityStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveForwardVelocityStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetForwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                measurement.state.GetForwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveLateralVelocityStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveLateralVelocityStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetRightwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                measurement.state.GetRightwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveYawRateStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveYawRateStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetYawRate()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetYawRate(),
                1.0e-7f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveLeftWheelSpeedStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveLeftWheelSpeedStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetWheelSpeedLeft()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetWheelSpeedLeft(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveRightWheelSpeedStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveRightWheelSpeedStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetWheelSpeedRight()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetWheelSpeedRight(),
                1.0e-6f,
                message.str().c_str());
        }

    };
}
