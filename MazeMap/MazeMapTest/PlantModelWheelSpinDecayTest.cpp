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
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            auto plant = PlantModel(vehicle, state);

            const App::Internal::CommandVector control{};
            const float initialSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedLeft();
            plant.integrate(control, 0.001f);

            const float finalSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            const float finalSlipAbsMps = std::fabs(finalSlipMps);
            const float finalWheelSpeedRadps = state.GetWheelSpeedLeft();
            std::wstringstream message;
            message <<
                L"PlantModelDifferentialWheelSpinReducesLeftWheelSpeed\n"
                L"field=left_longitudinal_slip_abs_mps\n"
                L"initial_slip_mps=" << initialSlipMps << L"\n"
                L"final_slip_mps=" << finalSlipMps << L"\n"
                L"initial_slip_abs_mps=" << initialSlipAbsMps << L"\n"
                L"final_slip_abs_mps=" << finalSlipAbsMps << L"\n"
                L"initial_left_wheel_speed_radps=" << initialWheelSpeedRadps << L"\n"
                L"final_left_wheel_speed_radps=" << finalWheelSpeedRadps << L"\n"
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
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            auto plant = PlantModel(vehicle, state);


            const App::Internal::CommandVector control{};
            const float initialSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedRight();
            plant.integrate(control, 0.001f);

            const float finalSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            const float finalSlipAbsMps = std::fabs(finalSlipMps);
            const float finalWheelSpeedRadps = state.GetWheelSpeedRight();
            std::wstringstream message;
            message <<
                L"PlantModelDifferentialWheelSpinReducesRightWheelSpeed\n"
                L"field=right_longitudinal_slip_abs_mps\n"
                L"initial_slip_mps=" << initialSlipMps << L"\n"
                L"final_slip_mps=" << finalSlipMps << L"\n"
                L"initial_slip_abs_mps=" << initialSlipAbsMps << L"\n"
                L"final_slip_abs_mps=" << finalSlipAbsMps << L"\n"
                L"initial_right_wheel_speed_radps=" << initialWheelSpeedRadps << L"\n"
                L"final_right_wheel_speed_radps=" << finalWheelSpeedRadps << L"\n"
                L"criterion=final_slip_abs_mps<initial_slip_abs_mps\n"
                L"dt_s=0.001";
            Assert::IsTrue(finalSlipAbsMps < initialSlipAbsMps, message.str().c_str());
        }

        TEST_METHOD(SymmetricWheelSpinReducesLeftWheelSpeed)
        {
            const SymmetricWheelSpinStep step = IntegrateSymmetricWheelSpin();
            std::wstringstream message;
            message << L"SymmetricWheelSpinReducesLeftWheelSpeed"
                << L"\ninitial_left_abs_radps=" << step.initialLeftAbsRadps
                << L"\nfinal_left_abs_radps=" << step.finalLeftAbsRadps
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
                << L"\ninitial_right_abs_radps=" << step.initialRightAbsRadps
                << L"\nfinal_right_abs_radps=" << step.finalRightAbsRadps
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
                << L"\nexpected_left=" << step.state.GetWheelSpeedLeft()
                << L"\nactual_right=" << step.state.GetWheelSpeedRight()
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                step.state.GetWheelSpeedLeft(),
                step.state.GetWheelSpeedRight(),
                1.0e-4f,
                message.str().c_str());
        }

    };
}
