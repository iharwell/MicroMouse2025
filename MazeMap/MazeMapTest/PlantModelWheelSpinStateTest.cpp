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
    TEST_CLASS(PlantModelWheelSpinStateTest)
    {
    public:
        TEST_METHOD(DifferentialWheelSpinPositionXStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinPositionXStaysFinite"
                << L"\nactual=" << state.GetPositionX()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetPositionX()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinPositionYStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinPositionYStaysFinite"
                << L"\nactual=" << state.GetPositionY()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetPositionY()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinHeadingStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinHeadingStaysFinite"
                << L"\nactual=" << state.GetHeading()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetHeading()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinForwardVelocityStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinForwardVelocityStaysFinite"
                << L"\nactual=" << state.GetForwardVelocity()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetForwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinLateralVelocityStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinLateralVelocityStaysFinite"
                << L"\nactual=" << state.GetRightwardVelocity()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetRightwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinYawRateStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinYawRateStaysFinite"
                << L"\nactual=" << state.GetYawRate()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetYawRate()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinLeftWheelSpeedStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinLeftWheelSpeedStaysFinite"
                << L"\nactual=" << state.GetWheelSpeedLeft()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetWheelSpeedLeft()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinRightWheelSpeedStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinRightWheelSpeedStaysFinite"
                << L"\nactual=" << state.GetWheelSpeedRight()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetWheelSpeedRight()),
                message.str().c_str());
        }

    };
}
