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
    TEST_CLASS(PlantModelStationaryPerturbationTest)
    {
    public:
        TEST_METHOD(SmallStationaryPerturbationForwardVelocityReturnsToRest)
        {
            const VehicleState state = IntegrateSmallStationaryPerturbation();
            std::wstringstream message;
            message << L"SmallStationaryPerturbationForwardVelocityReturnsToRest"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetForwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                state.GetForwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SmallStationaryPerturbationLateralVelocityReturnsToRest)
        {
            const VehicleState state = IntegrateSmallStationaryPerturbation();
            std::wstringstream message;
            message << L"SmallStationaryPerturbationLateralVelocityReturnsToRest"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetRightwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                state.GetRightwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SmallStationaryPerturbationYawRateReturnsToRest)
        {
            const VehicleState state = IntegrateSmallStationaryPerturbation();
            std::wstringstream message;
            message << L"SmallStationaryPerturbationYawRateReturnsToRest"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetYawRate()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.0f, state.GetYawRate(), 1.0e-7f, message.str().c_str());
        }

        TEST_METHOD(SmallStationaryPerturbationLeftWheelSpeedReturnsToRest)
        {
            const VehicleState state = IntegrateSmallStationaryPerturbation();
            const float tolerance =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            std::wstringstream message;
            message << L"SmallStationaryPerturbationLeftWheelSpeedReturnsToRest"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetWheelSpeedLeft()
                << L"\ntolerance=" << tolerance;

            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), tolerance, message.str().c_str());
        }

        TEST_METHOD(SmallStationaryPerturbationRightWheelSpeedReturnsToRest)
        {
            const VehicleState state = IntegrateSmallStationaryPerturbation();
            const float tolerance =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            std::wstringstream message;
            message << L"SmallStationaryPerturbationRightWheelSpeedReturnsToRest"
                << L"\nexpected=0"
                << L"\nactual=" << state.GetWheelSpeedRight()
                << L"\ntolerance=" << tolerance;

            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), tolerance, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationForwardVelocityEntersStopBand)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float actual = std::fabs(measurement.final.GetForwardVelocity());
            std::wstringstream message;
            message << L"NearZeroPerturbationForwardVelocityEntersStopBand"
                << L"\nactual_abs=" << actual
                << L"\nlimit=" << kStopEnterSpeedMps
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < kStopEnterSpeedMps, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationLateralVelocityEntersStopBand)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float actual = std::fabs(measurement.final.GetRightwardVelocity());
            std::wstringstream message;
            message << L"NearZeroPerturbationLateralVelocityEntersStopBand"
                << L"\nactual_abs=" << actual
                << L"\nlimit=" << kStopEnterSpeedMps
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < kStopEnterSpeedMps, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationYawRateEntersStopBand)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float actual = std::fabs(measurement.final.GetYawRate());
            std::wstringstream message;
            message << L"NearZeroPerturbationYawRateEntersStopBand"
                << L"\nactual_abs=" << actual
                << L"\nlimit=" << kStopEnterYawRateRadps
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < kStopEnterYawRateRadps, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationLeftWheelSpeedEntersStopBand)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float actual = std::fabs(measurement.final.GetWheelSpeedLeft());
            std::wstringstream message;
            message << L"NearZeroPerturbationLeftWheelSpeedEntersStopBand"
                << L"\nactual_abs=" << actual
                << L"\nlimit=" << kStopEnterWheelSpeedRadps
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < kStopEnterWheelSpeedRadps, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationRightWheelSpeedEntersStopBand)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float actual = std::fabs(measurement.final.GetWheelSpeedRight());
            std::wstringstream message;
            message << L"NearZeroPerturbationRightWheelSpeedEntersStopBand"
                << L"\nactual_abs=" << actual
                << L"\nlimit=" << kStopEnterWheelSpeedRadps
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < kStopEnterWheelSpeedRadps, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationYawRateMagnitudeDecays)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float initial = std::fabs(measurement.initial.GetYawRate());
            const float actual = std::fabs(measurement.final.GetYawRate());
            std::wstringstream message;
            message << L"NearZeroPerturbationYawRateMagnitudeDecays"
                << L"\ninitial_abs=" << initial
                << L"\nactual_abs=" << actual
                << L"\ncriterion=actual_abs<initial_abs";

            Assert::IsTrue(actual < initial, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationLeftWheelMagnitudeDecays)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float initial = std::fabs(measurement.initial.GetWheelSpeedLeft());
            const float actual = std::fabs(measurement.final.GetWheelSpeedLeft());
            std::wstringstream message;
            message << L"NearZeroPerturbationLeftWheelMagnitudeDecays"
                << L"\ninitial_abs=" << initial
                << L"\nactual_abs=" << actual
                << L"\ncriterion=actual_abs<initial_abs";

            Assert::IsTrue(actual < initial, message.str().c_str());
        }

        TEST_METHOD(NearZeroPerturbationRightWheelMagnitudeDecays)
        {
            const NearZeroLateralPerturbationMeasurement measurement =
                IntegrateNearZeroLateralPerturbation();
            const float initial = std::fabs(measurement.initial.GetWheelSpeedRight());
            const float actual = std::fabs(measurement.final.GetWheelSpeedRight());
            std::wstringstream message;
            message << L"NearZeroPerturbationRightWheelMagnitudeDecays"
                << L"\ninitial_abs=" << initial
                << L"\nactual_abs=" << actual
                << L"\ncriterion=actual_abs<initial_abs";

            Assert::IsTrue(actual < initial, message.str().c_str());
        }

    };
}
