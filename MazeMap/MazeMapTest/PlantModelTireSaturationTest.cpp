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
    TEST_CLASS(PlantModelTireSaturationTest)
    {
    public:
        TEST_METHOD(LeftBankPreProjectionUtilizationExceedsUnity)
        {
            const float actual =
                MaxPreProjectionUtilizationForBank(45.0f, 43.0f, 0U, 2U);
            std::wstringstream message;
            message << L"LeftBankPreProjectionUtilizationExceedsUnity"
                << L"\nactual=" << actual
                << L"\ncriterion=actual>1";

            Assert::IsTrue(
                actual > 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(RightBankPreProjectionUtilizationExceedsUnity)
        {
            const float actual =
                MaxPreProjectionUtilizationForBank(45.0f, 43.0f, 1U, 3U);
            std::wstringstream message;
            message << L"RightBankPreProjectionUtilizationExceedsUnity"
                << L"\nactual=" << actual
                << L"\ncriterion=actual>1";

            Assert::IsTrue(
                actual > 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricSpinLeftBankPreProjectionUtilizationExceedsUnity)
        {
            const float actual =
                MaxPreProjectionUtilizationForBank(55.0f, 55.0f, 0U, 2U);
            std::wstringstream message;
            message << L"SymmetricSpinLeftBankPreProjectionUtilizationExceedsUnity"
                << L"\nactual=" << actual
                << L"\ncriterion=actual>1";

            Assert::IsTrue(
                actual > 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricSpinRightBankPreProjectionUtilizationExceedsUnity)
        {
            const float actual =
                MaxPreProjectionUtilizationForBank(55.0f, 55.0f, 1U, 3U);
            std::wstringstream message;
            message << L"SymmetricSpinRightBankPreProjectionUtilizationExceedsUnity"
                << L"\nactual=" << actual
                << L"\ncriterion=actual>1";

            Assert::IsTrue(
                actual > 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricSpinMinimumSaturationClampsToUnity)
        {
            const float actual = MinimumSaturationForSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricSpinMinimumSaturationClampsToUnity"
                << L"\nexpected=1"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                1.0f,
                actual,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricSpinMaximumSaturationClampsToUnity)
        {
            const float actual = MaximumSaturationForSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricSpinMaximumSaturationClampsToUnity"
                << L"\nexpected=1"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                1.0f,
                actual,
                1.0e-6f,
                message.str().c_str());
        }

    };
}
