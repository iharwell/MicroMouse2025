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
    TEST_CLASS(PlantModelRestHoldTest)
    {
    public:
        TEST_METHOD(ExactRestHoldPreservesPositionX)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldPreservesPositionX"
                << L"\nexpected=0.03"
                << L"\nactual=" << state.GetPositionX()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.03f, state.GetPositionX(), 1.0e-7f, message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldPreservesPositionY)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldPreservesPositionY"
                << L"\nexpected=0.09"
                << L"\nactual=" << state.GetPositionY()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.09f, state.GetPositionY(), 1.0e-7f, message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldPreservesHeading)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldPreservesHeading"
                << L"\nexpected=0.21"
                << L"\nactual=" << state.GetHeading()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.21f, state.GetHeading(), 1.0e-7f, message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldForwardVelocityStaysZero)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldForwardVelocityStaysZero"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetForwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                state.GetForwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldLateralVelocityStaysZero)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldLateralVelocityStaysZero"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetRightwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                state.GetRightwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldYawRateStaysZero)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldYawRateStaysZero"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetYawRate()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.0f, state.GetYawRate(), 1.0e-7f, message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldLeftWheelSpeedStaysZero)
        {
            const VehicleState state = IntegrateExactRestHold();
            const float tolerance =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            std::wstringstream message;
            message << L"ExactRestHoldLeftWheelSpeedStaysZero"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetWheelSpeedLeft()
                << L"\ntolerance=" << tolerance;

            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), tolerance, message.str().c_str());
        }

        TEST_METHOD(ExactRestHoldRightWheelSpeedStaysZero)
        {
            const VehicleState state = IntegrateExactRestHold();
            const float tolerance =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            std::wstringstream message;
            message << L"ExactRestHoldRightWheelSpeedStaysZero"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetWheelSpeedRight()
                << L"\ntolerance=" << tolerance;

            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), tolerance, message.str().c_str());
        }

    };
}
