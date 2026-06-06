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
    namespace
    {
        float LeftEncoderSlipMps(const VehicleState& state) noexcept
        {
            return
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
        }

        float RightEncoderSlipMps(const VehicleState& state) noexcept
        {
            return
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
        }

        float LeftPlantSlipMps(const PlantModel& plant) noexcept
        {
            return (std::max)(
                std::fabs(plant.contactForwardRelativeVelocityMps(0U)),
                std::fabs(plant.contactForwardRelativeVelocityMps(2U)));
        }

        float RightPlantSlipMps(const PlantModel& plant) noexcept
        {
            return (std::max)(
                std::fabs(plant.contactForwardRelativeVelocityMps(1U)),
                std::fabs(plant.contactForwardRelativeVelocityMps(3U)));
        }
    }

    TEST_CLASS(PlantModelWheelSpinDecayTest)
    {
    public:
        TEST_METHOD(DifferentialWheelSpinReducesLeftWheelSpeed)
        {
            auto vehicle = Vehicle{};
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            PublishEncoderObservationForWheelSpeedsRadps(state, 45.0f, 43.0f);
            auto plant = PlantModel(vehicle, state);

            const App::Internal::CommandVector control{};
            const float initialSlipMps = LeftEncoderSlipMps(state);
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedLeft();
            plant.integrate(control, 0.001f);

            const float finalSlipAbsMps = LeftPlantSlipMps(plant);
            std::wstringstream message;
            message <<
                L"PlantModelDifferentialWheelSpinReducesLeftWheelSpeed\n"
                L"field=left_longitudinal_slip_abs_mps\n"
                L"initial_slip_mps=" << initialSlipMps << L"\n"
                L"final_slip_abs_mps=" << finalSlipAbsMps << L"\n"
                L"initial_slip_abs_mps=" << initialSlipAbsMps << L"\n"
                L"initial_left_wheel_speed_radps=" << initialWheelSpeedRadps << L"\n"
                L"criterion=final_slip_abs_mps<initial_slip_abs_mps\n"
                L"dt_s=0.001";
            Assert::IsTrue(finalSlipAbsMps < initialSlipAbsMps, message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinReducesRightWheelSpeed)
        {
            auto vehicle = Vehicle{};
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            PublishEncoderObservationForWheelSpeedsRadps(state, 45.0f, 43.0f);
            auto plant = PlantModel(vehicle, state);


            const App::Internal::CommandVector control{};
            const float initialSlipMps = RightEncoderSlipMps(state);
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedRight();
            plant.integrate(control, 0.001f);

            const float finalSlipAbsMps = RightPlantSlipMps(plant);
            std::wstringstream message;
            message <<
                L"PlantModelDifferentialWheelSpinReducesRightWheelSpeed\n"
                L"field=right_longitudinal_slip_abs_mps\n"
                L"initial_slip_mps=" << initialSlipMps << L"\n"
                L"final_slip_abs_mps=" << finalSlipAbsMps << L"\n"
                L"initial_slip_abs_mps=" << initialSlipAbsMps << L"\n"
                L"initial_right_wheel_speed_radps=" << initialWheelSpeedRadps << L"\n"
                L"criterion=final_slip_abs_mps<initial_slip_abs_mps\n"
                L"dt_s=0.001";
            Assert::IsTrue(finalSlipAbsMps < initialSlipAbsMps, message.str().c_str());
        }

        TEST_METHOD(SymmetricWheelSpinReducesLeftWheelSpeed)
        {
            const SymmetricWheelSpinStep step = IntegrateSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricWheelSpinReducesLeftWheelSpeed"
                << L"\ninitial_left_slip_abs_radps=" << step.initialLeftAbsRadps
                << L"\nfinal_left_slip_abs_radps=" << step.finalLeftAbsRadps
                << L"\ncriterion=final<initial";

            Assert::IsTrue(
                step.finalLeftAbsRadps < step.initialLeftAbsRadps,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricWheelSpinReducesRightWheelSpeed)
        {
            const SymmetricWheelSpinStep step = IntegrateSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricWheelSpinReducesRightWheelSpeed"
                << L"\ninitial_right_slip_abs_radps=" << step.initialRightAbsRadps
                << L"\nfinal_right_slip_abs_radps=" << step.finalRightAbsRadps
                << L"\ncriterion=final<initial";

            Assert::IsTrue(
                step.finalRightAbsRadps < step.initialRightAbsRadps,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricWheelSpinKeepsWheelSpeedsMatched)
        {
            const SymmetricWheelSpinStep step = IntegrateSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricWheelSpinKeepsWheelSpeedsMatched"
                << L"\nexpected_left=" << step.finalLeftAbsRadps
                << L"\nactual_right=" << step.finalRightAbsRadps
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                step.finalLeftAbsRadps,
                step.finalRightAbsRadps,
                1.0e-4f,
                message.str().c_str());
        }

    };
}
