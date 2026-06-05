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
    TEST_CLASS(PlantModelLateralAccelerationLimitTest)
    {
    public:
        TEST_METHOD(HighSlipLateralAccelerationPlateausAtSustainedLimit)
        {
            const float actual = PeakLateralAccelerationAcrossTicks(1.25f);
            const float maxAllowedAccelMps2 =
                Vehicle::GetSustainedLateralAccelerationReferenceMps2() + 0.20f;
            std::wstringstream message;
            message << L"HighSlipLateralAccelerationPlateausAtSustainedLimit"
                << L"\nactual=" << actual
                << L"\nmax_allowed_accel_mps2=" << maxAllowedAccelMps2
                << L"\ncriterion=actual<=max_allowed";

            Assert::IsTrue(
                actual <= maxAllowedAccelMps2,
                message.str().c_str());
        }

        TEST_METHOD(ExtremeSlipLateralAccelerationPlateausAtSustainedLimit)
        {
            const float actual = PeakLateralAccelerationAcrossTicks(3.50f);
            const float maxAllowedAccelMps2 =
                Vehicle::GetSustainedLateralAccelerationReferenceMps2() + 0.20f;
            std::wstringstream message;
            message << L"ExtremeSlipLateralAccelerationPlateausAtSustainedLimit"
                << L"\nactual=" << actual
                << L"\nmax_allowed_accel_mps2=" << maxAllowedAccelMps2
                << L"\ncriterion=actual<=max_allowed";

            Assert::IsTrue(
                actual <= maxAllowedAccelMps2,
                message.str().c_str());
        }

        TEST_METHOD(ExtremeSlipLateralAccelerationRemainsNearHighSlip)
        {
            const float highSlipAccelMps2 = PeakLateralAccelerationAcrossTicks(1.25f);
            const float extremeSlipAccelMps2 = PeakLateralAccelerationAcrossTicks(3.50f);
            const float minimumExtremeAccelMps2 = highSlipAccelMps2 - 0.40f;
            std::wstringstream message;
            message << L"ExtremeSlipLateralAccelerationRemainsNearHighSlip"
                << L"\nhigh_slip_accel_mps2=" << highSlipAccelMps2
                << L"\nextreme_slip_accel_mps2=" << extremeSlipAccelMps2
                << L"\nminimum_extreme_accel_mps2=" << minimumExtremeAccelMps2
                << L"\ncriterion=extreme>=minimum";

            Assert::IsTrue(
                extremeSlipAccelMps2 >= minimumExtremeAccelMps2,
                message.str().c_str());
        }

    };
}
