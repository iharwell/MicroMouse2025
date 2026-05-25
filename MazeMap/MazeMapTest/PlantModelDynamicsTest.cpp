#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroLinearVelocityToleranceMps = 0.008f;
        constexpr float kSymmetricFrontLoadFraction = 0.5f;
        constexpr float kStopEnterSpeedMps = 0.02f;
        constexpr float kStopEnterYawRateRadps = 0.20f;
        constexpr float kStopEnterWheelSpeedRadps = 2.0f;
        constexpr float kPlantResidualDecayTauS = 0.075f;
    }

    TEST_CLASS(PlantModelDynamicsTest)
    {
    public:
        TEST_METHOD(PlantModelUsesDriveWheelConstructionValues)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            Assert::AreEqual(1.177e-6f, plant.leftDriveEquivalentWheelInertiaKgM2(), 1.0e-10f);
            Assert::AreEqual(1.177e-6f, plant.rightDriveEquivalentWheelInertiaKgM2(), 1.0e-10f);
            Assert::AreEqual(4.12f, plant.leftDriveLongitudinalTireStiffnessN(), 0.01f);
            Assert::AreEqual(4.12f, plant.rightDriveLongitudinalTireStiffnessN(), 0.01f);
        }

        TEST_METHOD(PlantModelSymmetricDriveDoesNotCreateYawBias)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 2.5f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);
            const float initialYawRateRadps = state.GetYawRate();

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.55f);
            control.SetRightCommand(0.55f);

            plant.integrate(control, 0.001f);
            Assert::IsTrue(std::isfinite(state.GetYawRate()));
            Assert::AreEqual(
                initialYawRateRadps,
                state.GetYawRate(),
                1.0e-6f);
        }

        TEST_METHOD(PlantModelTireForcesRetainPreProjectionUtilizationAboveUnity)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            const float leftBankMaxPreProjectionUtilization =
                (std::max)(
                    plant.contactPreProjectionUtilization(control, 0U),
                    plant.contactPreProjectionUtilization(control, 2U));
            const float rightBankMaxPreProjectionUtilization =
                (std::max)(
                    plant.contactPreProjectionUtilization(control, 1U),
                    plant.contactPreProjectionUtilization(control, 3U));

            Assert::IsTrue(std::isfinite(leftBankMaxPreProjectionUtilization));
            Assert::IsTrue(std::isfinite(rightBankMaxPreProjectionUtilization));
            Assert::IsTrue(leftBankMaxPreProjectionUtilization > 1.0f);
            Assert::IsTrue(rightBankMaxPreProjectionUtilization > 1.0f);
        }

        TEST_METHOD(PlantModelTireForcesPreservePreProjectionUtilizationAboveUnityAndClampSaturationRange)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(55.0f);
            state.SetWheelSpeedRight(55.0f);
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            const float leftBankMaxPreProjectionUtilization =
                (std::max)(
                    plant.contactPreProjectionUtilization(control, 0U),
                    plant.contactPreProjectionUtilization(control, 2U));
            const float rightBankMaxPreProjectionUtilization =
                (std::max)(
                    plant.contactPreProjectionUtilization(control, 1U),
                    plant.contactPreProjectionUtilization(control, 3U));
            const float minSaturation =
                (std::min)(
                    (std::min)(
                        plant.contactSaturation(control, 0U),
                        plant.contactSaturation(control, 1U)),
                    (std::min)(
                        plant.contactSaturation(control, 2U),
                        plant.contactSaturation(control, 3U)));
            const float maxSaturation =
                (std::max)(
                    (std::max)(
                        plant.contactSaturation(control, 0U),
                        plant.contactSaturation(control, 1U)),
                    (std::max)(
                        plant.contactSaturation(control, 2U),
                        plant.contactSaturation(control, 3U)));

            Assert::IsTrue(leftBankMaxPreProjectionUtilization > 1.0f);
            Assert::IsTrue(rightBankMaxPreProjectionUtilization > 1.0f);
            Assert::IsTrue(std::isfinite(minSaturation));
            Assert::IsTrue(std::isfinite(maxSaturation));
            Assert::IsTrue(minSaturation >= 0.0f);
            Assert::IsTrue(maxSaturation <= 1.0f);
            Assert::AreEqual(1.0f, minSaturation, 1.0e-6f);
            Assert::AreEqual(1.0f, maxSaturation, 1.0e-6f);
        }

        TEST_METHOD(PlantModelDifferentialWheelSpinIntegratesFiniteState)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            plant.integrate(control, 0.001f);

            std::wstringstream message;
            message <<
                L"PlantModelDifferentialWheelSpinIntegratesFiniteState\n"
                L"field=bound_state\n"
                L"criterion=all state fields finite\n"
                L"x_m=" << state.GetPositionX() << L"\n"
                L"y_m=" << state.GetPositionY() << L"\n"
                L"yaw_rad=" << state.GetHeading() << L"\n"
                L"forward_velocity_mps=" << state.GetForwardVelocity() << L"\n"
                L"lateral_velocity_mps=" << state.GetRightwardVelocity() << L"\n"
                L"yaw_rate_radps=" << state.GetYawRate() << L"\n"
                L"left_wheel_speed_radps=" << state.GetWheelSpeedLeft() << L"\n"
                L"right_wheel_speed_radps=" << state.GetWheelSpeedRight();
            Assert::IsTrue(
                std::isfinite(state.GetPositionX()) &&
                std::isfinite(state.GetPositionY()) &&
                std::isfinite(state.GetHeading()) &&
                std::isfinite(state.GetForwardVelocity()) &&
                std::isfinite(state.GetRightwardVelocity()) &&
                std::isfinite(state.GetYawRate()) &&
                std::isfinite(state.GetWheelSpeedLeft()) &&
                std::isfinite(state.GetWheelSpeedRight()) &&
                std::isfinite(state.GetGyroBiasZ()),
                message.str().c_str());
        }

        TEST_METHOD(PlantModelDifferentialWheelSpinReducesLeftWheelSpeed)
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
                L"pass=" << (finalSlipAbsMps < initialSlipAbsMps) << L"\n"
                L"dt_s=0.001";
            Assert::IsTrue(finalSlipAbsMps < initialSlipAbsMps, message.str().c_str());
        }

        TEST_METHOD(PlantModelDifferentialWheelSpinReducesRightWheelSpeed)
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
                L"pass=" << (finalSlipAbsMps < initialSlipAbsMps) << L"\n"
                L"dt_s=0.001";
            Assert::IsTrue(finalSlipAbsMps < initialSlipAbsMps, message.str().c_str());
        }

        TEST_METHOD(PlantModelSymmetricWheelSpinIntegratesFiniteAndReducesWheelSpeeds)
        {
            auto vehicle = Vehicle{};
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(55.0f);
            state.SetWheelSpeedRight(55.0f);
			auto plant = PlantModel(vehicle, state);
            const App::Internal::CommandVector control{};

            const float initialLeftAbsRadps = std::fabs(state.GetWheelSpeedLeft());
            const float initialRightAbsRadps = std::fabs(state.GetWheelSpeedRight());
            plant.integrate(control, 0.001f);

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetHeading()));
            Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRightwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetYawRate()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < initialLeftAbsRadps);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < initialRightAbsRadps);
            Assert::AreEqual(
                state.GetWheelSpeedLeft(),
                state.GetWheelSpeedRight(),
                1.0e-4f);
        }

        TEST_METHOD(PlantModelFeedforwardDoesNotUsePrediction)
        {
            constexpr uint32_t numFeedforward = 100000;
			constexpr uint32_t numIntegrate = 75000;

            auto vehicle = Vehicle{};
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(1.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetGyroBiasZ(0.0f);
            auto plant = PlantModel(vehicle, state);
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            float feedforwardAccumulator = 0.0f;

			auto startTime1 = std::chrono::high_resolution_clock::now();
            for (int tick = 0; tick < numIntegrate; ++tick)
            {
                plant.integrate(control, dtSeconds);
            }
            auto durationIntegrate = std::chrono::high_resolution_clock::now() - startTime1;
            startTime1 = std::chrono::high_resolution_clock::now();
            for (int tick = 0; tick < numFeedforward; ++tick)
            {
				const App::Internal::CommandVector command = plant.ComputeFeedforward(0.0f, 0.0f);
                feedforwardAccumulator += command.LeftCommand() + command.RightCommand();
            }
            auto durationFeedforward = std::chrono::high_resolution_clock::now() - startTime1;
            auto ss = std::wstringstream();
            ss << "feedforward: " << durationFeedforward.count() << "  integrate: " << durationIntegrate.count();
            Assert::IsTrue(std::isfinite(feedforwardAccumulator));
            Assert::IsTrue(durationFeedforward < durationIntegrate, ss.str().c_str());
        }

        TEST_METHOD(PlantModelLateralAccelerationPlateausAtSustainedLimitAcrossTicks)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            VehicleState highSlipState;
            highSlipState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            highSlipState.SetHeading(0.0f);
            highSlipState.SetForwardVelocity(1.0f);
            highSlipState.SetRightwardVelocity(1.25f);
            highSlipState.SetYawRate(0.0f);
            highSlipState.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            highSlipState.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            highSlipState.SetGyroBiasZ(0.0f);
            PlantModel highSlipPlant(vehicle, highSlipState);
            float highSlipAccelMps2 = 0.0f;
            for (int tick = 0; tick < 5; ++tick)
            {
                const float beforeLateralVelocityMps = highSlipState.GetRightwardVelocity();
                highSlipPlant.integrate(control, dtSeconds);
                const float afterLateralVelocityMps = highSlipState.GetRightwardVelocity();
                highSlipAccelMps2 =
                    (std::max)(
                        highSlipAccelMps2,
                        std::fabs((afterLateralVelocityMps - beforeLateralVelocityMps) / dtSeconds));
            }

            VehicleState extremeSlipState;
            extremeSlipState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            extremeSlipState.SetHeading(0.0f);
            extremeSlipState.SetForwardVelocity(1.0f);
            extremeSlipState.SetRightwardVelocity(3.50f);
            extremeSlipState.SetYawRate(0.0f);
            extremeSlipState.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            extremeSlipState.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            extremeSlipState.SetGyroBiasZ(0.0f);
            PlantModel extremeSlipPlant(vehicle, extremeSlipState);
            float extremeSlipAccelMps2 = 0.0f;
            for (int tick = 0; tick < 5; ++tick)
            {
                const float beforeLateralVelocityMps = extremeSlipState.GetRightwardVelocity();
                extremeSlipPlant.integrate(control, dtSeconds);
                const float afterLateralVelocityMps = extremeSlipState.GetRightwardVelocity();
                extremeSlipAccelMps2 =
                    (std::max)(
                        extremeSlipAccelMps2,
                        std::fabs((afterLateralVelocityMps - beforeLateralVelocityMps) / dtSeconds));
            }

            Assert::IsTrue(highSlipAccelMps2 <= Vehicle::GetSustainedLateralAccelerationReferenceMps2() + 0.20f);
            Assert::IsTrue(extremeSlipAccelMps2 <= Vehicle::GetSustainedLateralAccelerationReferenceMps2() + 0.20f);
            Assert::IsTrue(extremeSlipAccelMps2 >= highSlipAccelMps2 - 0.40f);
        }

        TEST_METHOD(PlantModelInPlaceSlipYawDecelMatchesSustainedLateralWindow)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY = std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float yawRateRadps = 30.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(yawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * yawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * yawRateRadps));
            PlantModel plant(vehicle, state);

            const float totalSustainedLateralForceN =
                vehicle.GetMass() * Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) * totalSustainedLateralForceN;

            Assert::AreEqual(0.0f, plant.contactForwardRelativeVelocityMps(0U), 1.0e-6f);
            Assert::AreEqual(0.0f, plant.contactForwardRelativeVelocityMps(1U), 1.0e-6f);
            Assert::AreEqual(0.0f, plant.contactForwardRelativeVelocityMps(2U), 1.0e-6f);
            Assert::AreEqual(0.0f, plant.contactForwardRelativeVelocityMps(3U), 1.0e-6f);
            Assert::AreEqual(-contactY * yawRateRadps, plant.contactRightRelativeVelocityMps(0U), 1.0e-6f);
            Assert::AreEqual(-contactY * yawRateRadps, plant.contactRightRelativeVelocityMps(1U), 1.0e-6f);
            Assert::AreEqual(contactY * yawRateRadps, plant.contactRightRelativeVelocityMps(2U), 1.0e-6f);
            Assert::AreEqual(contactY * yawRateRadps, plant.contactRightRelativeVelocityMps(3U), 1.0e-6f);
            Assert::AreEqual(-frontContactLimitN, plant.contactRightForceN(control, 0U), 1.0e-4f);
            Assert::AreEqual(-frontContactLimitN, plant.contactRightForceN(control, 1U), 1.0e-4f);
            Assert::AreEqual(rearContactLimitN, plant.contactRightForceN(control, 2U), 1.0e-4f);
            Assert::AreEqual(rearContactLimitN, plant.contactRightForceN(control, 3U), 1.0e-4f);

            constexpr float dtSeconds = 0.001f;
            const float initialYawRateRadps = state.GetYawRate();
            plant.integrate(control, dtSeconds);
            const float observedYawAccelRadps2 =
                state.GetYawAccel();
            const float actualYawRateRadps =
                state.GetYawRate();
            const float saturatedYawDecelRadps2 =
                (2.0f * contactY * (frontContactLimitN + rearContactLimitN)) /
                vehicle.GetYawInertia();
            Assert::IsTrue(std::isfinite(observedYawAccelRadps2));
            Assert::IsTrue(actualYawRateRadps < initialYawRateRadps);
            Assert::AreEqual(
                -saturatedYawDecelRadps2,
                observedYawAccelRadps2,
                1.0e-3f);
        }

        TEST_METHOD(PlantModelYawAccelerationIsSmoothAcrossLowForwardSpeeds)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float referenceForwardSpeedMps = 0.12f;
            constexpr float nearbyDeltaMps = 0.001f;
            constexpr float yawRateRadps = 3.0f;
            constexpr float dtSeconds = 0.001f;
            VehicleState belowState;
            belowState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            belowState.SetHeading(0.0f);
            belowState.SetForwardVelocity(referenceForwardSpeedMps - nearbyDeltaMps);
            belowState.SetRightwardVelocity(0.0f);
            belowState.SetYawRate(yawRateRadps);
            belowState.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(
                    belowState.GetForwardVelocity() + (halfTrackM * yawRateRadps)));
            belowState.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(
                    belowState.GetForwardVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel belowPlant(vehicle, belowState);
            belowPlant.integrate(control, dtSeconds);
            const float belowYawAccelRadps2 = belowState.GetYawAccel();

            VehicleState centerState;
            centerState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            centerState.SetHeading(0.0f);
            centerState.SetForwardVelocity(referenceForwardSpeedMps);
            centerState.SetRightwardVelocity(0.0f);
            centerState.SetYawRate(yawRateRadps);
            centerState.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(
                    centerState.GetForwardVelocity() + (halfTrackM * yawRateRadps)));
            centerState.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(
                    centerState.GetForwardVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel centerPlant(vehicle, centerState);
            centerPlant.integrate(control, dtSeconds);
            const float centerYawAccelRadps2 = centerState.GetYawAccel();

            VehicleState aboveState;
            aboveState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            aboveState.SetHeading(0.0f);
            aboveState.SetForwardVelocity(referenceForwardSpeedMps + nearbyDeltaMps);
            aboveState.SetRightwardVelocity(0.0f);
            aboveState.SetYawRate(yawRateRadps);
            aboveState.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(
                    aboveState.GetForwardVelocity() + (halfTrackM * yawRateRadps)));
            aboveState.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(
                    aboveState.GetForwardVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel abovePlant(vehicle, aboveState);
            abovePlant.integrate(control, dtSeconds);
            const float aboveYawAccelRadps2 = aboveState.GetYawAccel();
            const float maxNeighborDeltaRadps2 =
                (std::max)(
                    std::fabs(centerYawAccelRadps2 - belowYawAccelRadps2),
                    std::fabs(aboveYawAccelRadps2 - centerYawAccelRadps2));
            const float localScaleRadps2 =
                (std::max)(
                    std::fabs(centerYawAccelRadps2),
                    1.0f);
            const float maxAllowedNeighborDeltaRadps2 =
                (0.05f * localScaleRadps2) + 0.10f;

            std::wstringstream message;
            message <<
                L"PlantModelYawAccelerationIsSmoothAcrossLowForwardSpeeds\n"
                L"field=yaw_acceleration_radps2\n"
                L"criterion=finite, decelerating, and nearby low-speed samples remain smooth\n"
                L"reference_forward_speed_mps=" << referenceForwardSpeedMps << L"\n"
                L"nearby_delta_mps=" << nearbyDeltaMps << L"\n"
                L"below_yaw_accel_radps2=" << belowYawAccelRadps2 << L"\n"
                L"center_yaw_accel_radps2=" << centerYawAccelRadps2 << L"\n"
                L"above_yaw_accel_radps2=" << aboveYawAccelRadps2 << L"\n"
                L"max_neighbor_delta_radps2=" << maxNeighborDeltaRadps2 << L"\n"
                L"max_allowed_neighbor_delta_radps2=" << maxAllowedNeighborDeltaRadps2;
            Assert::IsTrue(
                std::isfinite(belowYawAccelRadps2) &&
                std::isfinite(centerYawAccelRadps2) &&
                std::isfinite(aboveYawAccelRadps2) &&
                belowYawAccelRadps2 < 0.0f &&
                centerYawAccelRadps2 < 0.0f &&
                aboveYawAccelRadps2 < 0.0f &&
                maxNeighborDeltaRadps2 <= maxAllowedNeighborDeltaRadps2,
                message.str().c_str());
        }

        TEST_METHOD(PlantModelContactContinuumYawCorrectionProducesPatchForceCouple)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            constexpr float yawRateRadps = 1.0f;

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(yawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * yawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * yawRateRadps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            const float frontLeftForwardForceN = plant.contactForwardForceN(control, 0U);
            const float frontRightForwardForceN = plant.contactForwardForceN(control, 1U);
            const float rearLeftForwardForceN = plant.contactForwardForceN(control, 2U);
            const float rearRightForwardForceN = plant.contactForwardForceN(control, 3U);
            const float totalForwardForceN =
                frontLeftForwardForceN +
                frontRightForwardForceN +
                rearLeftForwardForceN +
                rearRightForwardForceN;

            std::wstringstream message;
            message <<
                L"PlantModelContactContinuumYawCorrectionProducesPatchForceCouple\n"
                L"field=contact_forward_force_n\n"
                L"criterion=finite tuned left/right-opposed patch force couple with near-zero net forward force\n"
                L"fl=" << frontLeftForwardForceN << L"\n"
                L"fr=" << frontRightForwardForceN << L"\n"
                L"rl=" << rearLeftForwardForceN << L"\n"
                L"rr=" << rearRightForwardForceN << L"\n"
                L"sum=" << totalForwardForceN;
            Assert::IsTrue(
                std::isfinite(frontLeftForwardForceN) &&
                std::isfinite(frontRightForwardForceN) &&
                std::isfinite(rearLeftForwardForceN) &&
                std::isfinite(rearRightForwardForceN) &&
                frontLeftForwardForceN < -1.0e-6f &&
                rearLeftForwardForceN < -1.0e-6f &&
                frontRightForwardForceN > 1.0e-6f &&
                rearRightForwardForceN > 1.0e-6f &&
                std::fabs(totalForwardForceN) < 1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(PlantModelContactContinuumYawCorrectionIsFiniteAcrossZeroForwardSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float forwardSpeedsMps[] = { -0.002f, 0.0f, 0.002f };
            const float rightSpeedsMps[] = { -0.020f, 0.0f, 0.020f };
            const float yawRatesRadps[] = { -3.0f, 0.0f, 3.0f };

            for (const float forwardSpeedMps : forwardSpeedsMps)
            {
                for (const float rightSpeedMps : rightSpeedsMps)
                {
                    for (const float yawRateRadps : yawRatesRadps)
                    {
                        VehicleState state;
                        state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
                        state.SetHeading(0.0f);
                        state.SetForwardVelocity(forwardSpeedMps);
                        state.SetRightwardVelocity(rightSpeedMps);
                        state.SetYawRate(yawRateRadps);
                        state.SetWheelSpeedLeft(
                            Vehicle::WheelSpeedFromLinearVelocity(
                                forwardSpeedMps + (halfTrackM * yawRateRadps)));
                        state.SetWheelSpeedRight(
                            Vehicle::WheelSpeedFromLinearVelocity(
                                forwardSpeedMps - (halfTrackM * yawRateRadps)));
                        PlantModel plant(vehicle, state);

                        App::Internal::CommandVector control{};
                        control.SetLeftCommand(0.12f);
                        control.SetRightCommand(-0.10f);

                        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
                        {
                            Assert::IsTrue(std::isfinite(plant.contactForwardRelativeVelocityMps(contactIndex)));
                            Assert::IsTrue(std::isfinite(plant.contactRightRelativeVelocityMps(contactIndex)));
                            Assert::IsTrue(std::isfinite(plant.contactForwardForceN(control, contactIndex)));
                            Assert::IsTrue(std::isfinite(plant.contactRightForceN(control, contactIndex)));
                            Assert::IsTrue(std::isfinite(plant.contactPreProjectionUtilization(control, contactIndex)));
                            Assert::IsTrue(std::isfinite(plant.contactSaturation(control, contactIndex)));
                        }

                        plant.integrate(control, 0.001f);
                        Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
                        Assert::IsTrue(std::isfinite(state.GetRightwardVelocity()));
                        Assert::IsTrue(std::isfinite(state.GetYawRate()));
                        Assert::IsTrue(std::isfinite(state.GetYawAccel()));
                    }
                }
            }
        }

        TEST_METHOD(PlantModelContactContinuumYawCorrectionIsContinuousAcrossZeroForwardSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            constexpr float yawRateRadps = 1.0f;
            constexpr float forwardDeltaMps = 0.0001f;
            constexpr float dtSeconds = 0.001f;
            const App::Internal::CommandVector control{};
            float yawAccelerationsRadps2[3] = {};
            const float forwardSpeedsMps[3] = { -forwardDeltaMps, 0.0f, forwardDeltaMps };

            for (int sample = 0; sample < 3; ++sample)
            {
                const float forwardSpeedMps = forwardSpeedsMps[sample];
                VehicleState state;
                state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
                state.SetHeading(0.0f);
                state.SetForwardVelocity(forwardSpeedMps);
                state.SetRightwardVelocity(0.0f);
                state.SetYawRate(yawRateRadps);
                state.SetWheelSpeedLeft(
                    Vehicle::WheelSpeedFromLinearVelocity(
                        forwardSpeedMps + (halfTrackM * yawRateRadps)));
                state.SetWheelSpeedRight(
                    Vehicle::WheelSpeedFromLinearVelocity(
                        forwardSpeedMps - (halfTrackM * yawRateRadps)));
                PlantModel plant(vehicle, state);
                plant.integrate(control, dtSeconds);
                yawAccelerationsRadps2[sample] = state.GetYawAccel();
            }

            const float maxNeighborDeltaRadps2 =
                (std::max)(
                    std::fabs(yawAccelerationsRadps2[1] - yawAccelerationsRadps2[0]),
                    std::fabs(yawAccelerationsRadps2[2] - yawAccelerationsRadps2[1]));
            std::wstringstream message;
            message <<
                L"PlantModelContactContinuumYawCorrectionIsContinuousAcrossZeroForwardSpeed\n"
                L"field=yaw_acceleration_radps2\n"
                L"criterion=finite and bounded neighboring deltas across Vf=0\n"
                L"below=" << yawAccelerationsRadps2[0] << L"\n"
                L"center=" << yawAccelerationsRadps2[1] << L"\n"
                L"above=" << yawAccelerationsRadps2[2] << L"\n"
                L"max_neighbor_delta=" << maxNeighborDeltaRadps2;
            Assert::IsTrue(
                std::isfinite(yawAccelerationsRadps2[0]) &&
                std::isfinite(yawAccelerationsRadps2[1]) &&
                std::isfinite(yawAccelerationsRadps2[2]) &&
                maxNeighborDeltaRadps2 < 1.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(PlantModelInPlaceSlipYawRateIsPassiveWithBoundedRebound)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY = std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float dtSeconds = 0.001f;
            constexpr float initialYawRateRadps = 30.0f;
            const float totalSustainedLateralForceN =
                vehicle.GetMass() * Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) * totalSustainedLateralForceN;
            const float saturatedYawDecelRadps2 =
                (2.0f * contactY * (frontContactLimitN + rearContactLimitN)) /
                vehicle.GetYawInertia();
            const float saturatedStopTimeS =
                (initialYawRateRadps - kStopEnterYawRateRadps) /
                saturatedYawDecelRadps2;
            const float maxAllowedStopTimeS = 1.20f * saturatedStopTimeS;
            const int maxSteps =
                static_cast<int>(std::ceil(maxAllowedStopTimeS / dtSeconds));

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(initialYawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * initialYawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * initialYawRateRadps));
            PlantModel plant(vehicle, state);

            float previousYawRateAbsRadps = std::fabs(state.GetYawRate());
            float minYawRateAfterInitialDecayAbsRadps = previousYawRateAbsRadps;
            float maxYawRateAbsRadps = previousYawRateAbsRadps;
            float maxReboundRadps = 0.0f;
            int maxReboundStep = -1;
            int stopStep = -1;
            for (int step = 0; step < maxSteps; ++step)
            {
                plant.integrate(control, dtSeconds);
                const float yawRateAbsRadps =
                    std::fabs(state.GetYawRate());

                Assert::IsTrue(yawRateAbsRadps <= previousYawRateAbsRadps + 1.0e-4f);
                maxYawRateAbsRadps = (std::max)(maxYawRateAbsRadps, yawRateAbsRadps);
                if (yawRateAbsRadps < minYawRateAfterInitialDecayAbsRadps)
                {
                    minYawRateAfterInitialDecayAbsRadps = yawRateAbsRadps;
                }
                else if (yawRateAbsRadps > previousYawRateAbsRadps)
                {
                    const float reboundRadps =
                        yawRateAbsRadps - minYawRateAfterInitialDecayAbsRadps;
                    if (reboundRadps > maxReboundRadps)
                    {
                        maxReboundRadps = reboundRadps;
                        maxReboundStep = step + 1;
                    }
                }

                previousYawRateAbsRadps = yawRateAbsRadps;
                if (yawRateAbsRadps <= kStopEnterYawRateRadps)
                {
                    stopStep = step + 1;
                    break;
                }
            }

            constexpr float initialMagnitudeToleranceRadps = 1.0e-4f;
            const float maxAllowedReboundRadps = kStopEnterYawRateRadps;
            std::wstringstream message;
            message <<
                L"PlantModelInPlaceSlipYawRateIsPassiveWithBoundedRebound\n"
                L"field=yaw_rate_abs_radps\n"
                L"criterion=max_yaw_rate_abs<=initial+tolerance and rebound after decay stays below stop-band scale\n"
                L"initial_yaw_rate_abs_radps=" << initialYawRateRadps << L"\n"
                L"max_yaw_rate_abs_radps=" << maxYawRateAbsRadps << L"\n"
                L"initial_magnitude_tolerance_radps=" << initialMagnitudeToleranceRadps << L"\n"
                L"min_yaw_rate_after_initial_decay_abs_radps=" << minYawRateAfterInitialDecayAbsRadps << L"\n"
                L"max_rebound_radps=" << maxReboundRadps << L"\n"
                L"max_allowed_rebound_radps=" << maxAllowedReboundRadps << L"\n"
                L"max_rebound_step=" << maxReboundStep << L"\n"
                L"stop_step=" << stopStep << L"\n"
                L"max_steps=" << maxSteps;
            Assert::IsTrue(
                maxYawRateAbsRadps <= initialYawRateRadps + initialMagnitudeToleranceRadps &&
                maxReboundRadps <= maxAllowedReboundRadps,
                message.str().c_str());
        }

        TEST_METHOD(PlantModelInPlaceSlipSpinDownEnvelope)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY = std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float dtSeconds = 0.001f;
            constexpr float initialYawRateRadps = 30.0f;
            const float totalSustainedLateralForceN =
                vehicle.GetMass() * Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) * totalSustainedLateralForceN;
            const float saturatedYawDecelRadps2 =
                (2.0f * contactY * (frontContactLimitN + rearContactLimitN)) /
                vehicle.GetYawInertia();
            const float saturatedStopTimeS =
                (initialYawRateRadps - kStopEnterYawRateRadps) /
                saturatedYawDecelRadps2;
            const float maxAllowedStopTimeS = 1.20f * saturatedStopTimeS;
            const int maxSteps =
                static_cast<int>(std::ceil(maxAllowedStopTimeS / dtSeconds));

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(initialYawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * initialYawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * initialYawRateRadps));
            PlantModel plant(vehicle, state);

            int stopStep = -1;
            float previousYawRateAbsRadps = std::fabs(state.GetYawRate());
            float minYawRateAfterInitialDecayAbsRadps = previousYawRateAbsRadps;
            float maxReboundRadps = 0.0f;
            int maxReboundStep = -1;
            float finalYawRateAbsRadps = std::fabs(state.GetYawRate());
            for (int step = 0; step < maxSteps; ++step)
            {
                plant.integrate(control, dtSeconds);
                finalYawRateAbsRadps =
                    std::fabs(state.GetYawRate());

                if (finalYawRateAbsRadps < minYawRateAfterInitialDecayAbsRadps)
                {
                    minYawRateAfterInitialDecayAbsRadps = finalYawRateAbsRadps;
                }
                else if (finalYawRateAbsRadps > previousYawRateAbsRadps)
                {
                    const float reboundRadps =
                        finalYawRateAbsRadps - minYawRateAfterInitialDecayAbsRadps;
                    if (reboundRadps > maxReboundRadps)
                    {
                        maxReboundRadps = reboundRadps;
                        maxReboundStep = step + 1;
                    }
                }

                previousYawRateAbsRadps = finalYawRateAbsRadps;
                if (finalYawRateAbsRadps <= kStopEnterYawRateRadps)
                {
                    stopStep = step + 1;
                    break;
                }
            }

            const float stopTimeS =
                (stopStep > 0) ?
                (static_cast<float>(stopStep) * dtSeconds) :
                std::numeric_limits<float>::infinity();
            constexpr float minExpectedStopTimeS = 0.0f;
            constexpr float maxAllowedReboundRadps = kStopEnterYawRateRadps;
            std::wstringstream message;
            message <<
                L"PlantModelInPlaceSlipSpinDownEnvelope\n"
                L"field=stop_time_s\n"
                L"criterion=spin reaches stop band within the sustained-force-derived physical window with bounded rebound\n"
                L"initial_yaw_rate_radps=" << initialYawRateRadps << L"\n"
                L"stop_band_radps=" << kStopEnterYawRateRadps << L"\n"
                L"stop_step=" << stopStep << L"\n"
                L"stop_time_s=" << stopTimeS << L"\n"
                L"min_expected_stop_time_s=" << minExpectedStopTimeS << L"\n"
                L"max_allowed_stop_time_s=" << maxAllowedStopTimeS << L"\n"
                L"final_yaw_rate_abs_radps=" << finalYawRateAbsRadps << L"\n"
                L"max_rebound_radps=" << maxReboundRadps << L"\n"
                L"max_allowed_rebound_radps=" << maxAllowedReboundRadps << L"\n"
                L"max_rebound_step=" << maxReboundStep << L"\n"
                L"max_steps=" << maxSteps << L"\n"
                L"saturated_stop_time_s=" << saturatedStopTimeS << L"\n"
                L"dt_s=" << dtSeconds;
            Assert::IsTrue(
                stopTimeS >= minExpectedStopTimeS &&
                stopTimeS <= maxAllowedStopTimeS + dtSeconds &&
                maxReboundRadps <= maxAllowedReboundRadps,
                message.str().c_str());
        }

        TEST_METHOD(PlantModelExactRestHoldKeepsMotionStateAtZero)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float zeroWheelSpeedToleranceRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.03f, 0.09f));
            state.SetHeading(0.21f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(0.0f);
            state.SetWheelSpeedRight(0.0f);
            state.SetGyroBiasZ(0.12f);
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 1000; ++step)
            {
				plant.integrate(control, dt);
            }

            Assert::AreEqual(0.03f, state.GetPositionX(), 1.0e-7f);
            Assert::AreEqual(0.09f, state.GetPositionY(), 1.0e-7f);
            Assert::AreEqual(0.21f, state.GetHeading(), 1.0e-7f);
            Assert::AreEqual(0.12f, state.GetGyroBiasZ(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetForwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRightwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetYawRate(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelSmallStationaryPerturbationsSnapBackToRest)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float zeroWheelSpeedToleranceRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.05f);
            state.SetWheelSpeedLeft(0.8f);
            state.SetWheelSpeedRight(-0.7f);
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 250; ++step)
            {
				plant.integrate(control, dt);
            }

            Assert::AreEqual(0.0f, state.GetForwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRightwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetYawRate(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelNearZeroLateralPerturbationsSnapBackToRest)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState initialState;
            initialState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            initialState.SetHeading(0.0f);
            initialState.SetForwardVelocity(0.0f);
            initialState.SetRightwardVelocity(0.001f);
            initialState.SetYawRate(0.05f);
            initialState.SetWheelSpeedLeft(0.8f);
            initialState.SetWheelSpeedRight(-0.7f);
            VehicleState state = initialState;
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 25; ++step)
            {
                plant.integrate(control, dt);
            }

            Assert::IsTrue(std::fabs(state.GetForwardVelocity()) < kStopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state.GetRightwardVelocity()) < kStopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state.GetYawRate()) < kStopEnterYawRateRadps);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < kStopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < kStopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state.GetYawRate()) < std::fabs(initialState.GetYawRate()));
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < std::fabs(initialState.GetWheelSpeedLeft()));
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < std::fabs(initialState.GetWheelSpeedRight()));
        }

        TEST_METHOD(PlantModelResidualAccelerationStatesUseExactDeterministicOuDecay)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(0.0f);
            state.SetWheelSpeedRight(0.0f);
            state.SetForwardAccelerationResidual(0.75f);
            state.SetRightwardAccelerationResidual(-1.25f);
            state.SetYawAccelResidual(2.50f);
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            constexpr float dtSeconds = 0.010f;
            const float decay = std::exp(-dtSeconds / kPlantResidualDecayTauS);
            plant.integrate(control, dtSeconds);

            Assert::AreEqual(0.75f * decay, state.GetForwardAccelerationResidual(), 1.0e-6f);
            Assert::AreEqual(-1.25f * decay, state.GetRightwardAccelerationResidual(), 1.0e-6f);
            Assert::AreEqual(2.50f * decay, state.GetYawAccelResidual(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelMixedSlipCommandIntegratesFiniteState)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 2.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.15f);
            state.SetYawRate(1.8f);
            state.SetWheelSpeedLeft(1.05f * Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(0.95f * Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.65f);
            control.SetRightCommand(0.60f);

            const float leftLongitudinalSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            const float rightLongitudinalSlipMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate());
            Assert::IsTrue(std::isfinite(leftLongitudinalSlipMps));
            Assert::IsTrue(std::isfinite(rightLongitudinalSlipMps));
            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                const float forwardRelativeVelocityMps =
                    plant.contactForwardRelativeVelocityMps(contactIndex);
                const float rightRelativeVelocityMps =
                    plant.contactRightRelativeVelocityMps(contactIndex);
                const float rightForceN = plant.contactRightForceN(control, contactIndex);
                const float forwardForceN = plant.contactForwardForceN(control, contactIndex);
                const float saturation = plant.contactSaturation(control, contactIndex);
                const float preProjectionUtilization =
                    plant.contactPreProjectionUtilization(control, contactIndex);
                Assert::IsTrue(std::isfinite(forwardRelativeVelocityMps));
                Assert::IsTrue(std::isfinite(rightRelativeVelocityMps));
                Assert::IsTrue(std::isfinite(rightForceN));
                Assert::IsTrue(std::isfinite(forwardForceN));
                Assert::IsTrue(saturation >= 0.0f);
                Assert::IsTrue(saturation <= 1.0f);
                Assert::IsTrue(std::isfinite(preProjectionUtilization));
                Assert::IsTrue(preProjectionUtilization >= 0.0f);
                Assert::IsTrue(preProjectionUtilization >= saturation);
            }

            plant.integrate(control, 0.001f);

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetHeading()));
            Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRightwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetYawRate()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(state.GetHeading() <= PI_F);
            Assert::IsTrue(state.GetHeading() >= -PI_F);
            Assert::IsTrue(std::fabs(state.GetForwardVelocity()) < 10.0f);
            Assert::IsTrue(std::fabs(state.GetRightwardVelocity()) < 10.0f);
            Assert::IsTrue(std::fabs(state.GetYawRate()) < 50.0f);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < 1000.0f);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < 1000.0f);
        }

        TEST_METHOD(PlantModelImuAccelerationIncludesLeverArmTerms)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float initialYawRateRadps = 5.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(1.4f);
            state.SetRightwardVelocity(0.2f);
            state.SetYawRate(initialYawRateRadps);
            state.SetWheelSpeedLeft(0.9f * Vehicle::WheelSpeedFromLinearVelocity(1.4f));
            state.SetWheelSpeedRight(1.1f * Vehicle::WheelSpeedFromLinearVelocity(1.4f));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.30f);
            control.SetRightCommand(0.55f);

            const float predictedImuRightAccelerationMps2 =
                plant.backLeftImuRightAccelerationMps2(control);
            const float predictedImuForwardAccelerationMps2 =
                plant.backLeftImuForwardAccelerationMps2(control);
            plant.integrate(control, 0.001f);

            const Eigen::Vector2f imuLeverArmBodyM =
                Vehicle::GetBackLeftImuMount().positionBodyM();
            const float yawRateSquaredRadps2 =
                initialYawRateRadps * initialYawRateRadps;
            const float expectedRightAccelerationMps2 =
                state.GetRightAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (state.GetYawAccel() * imuLeverArmBodyM.y());
            const float expectedForwardAccelerationMps2 =
                state.GetForwardAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (state.GetYawAccel() * imuLeverArmBodyM.x());
            const float rightLeverContributionMps2 =
                predictedImuRightAccelerationMps2 - state.GetRightAcceleration();
            const float forwardLeverContributionMps2 =
                predictedImuForwardAccelerationMps2 - state.GetForwardAcceleration();

            Assert::IsTrue(
                std::fabs(rightLeverContributionMps2) > 1.0e-3f ||
                std::fabs(forwardLeverContributionMps2) > 1.0e-3f);
            Assert::AreEqual(
                expectedRightAccelerationMps2,
                predictedImuRightAccelerationMps2,
                1.0e-5f);
            Assert::AreEqual(
                expectedForwardAccelerationMps2,
                predictedImuForwardAccelerationMps2,
                1.0e-5f);
        }

        TEST_METHOD(PlantModelPredictsImuAccelerationInProjectBodyAxes)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float initialYawRateRadps = 4.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(1.1f);
            state.SetRightwardVelocity(-0.3f);
            state.SetYawRate(initialYawRateRadps);
            state.SetWheelSpeedLeft(0.95f * Vehicle::WheelSpeedFromLinearVelocity(1.1f));
            state.SetWheelSpeedRight(1.05f * Vehicle::WheelSpeedFromLinearVelocity(1.1f));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.45f);

            const float predictedImuRightAccelerationMps2 =
                plant.backLeftImuRightAccelerationMps2(control);
            const float predictedImuForwardAccelerationMps2 =
                plant.backLeftImuForwardAccelerationMps2(control);
            plant.integrate(control, 0.001f);

            const Eigen::Vector2f imuLeverArmBodyM =
                Vehicle::GetBackLeftImuMount().positionBodyM();
            const float yawRateSquaredRadps2 =
                initialYawRateRadps * initialYawRateRadps;
            const float expectedRightAccelerationMps2 =
                state.GetRightAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (state.GetYawAccel() * imuLeverArmBodyM.y());
            const float expectedForwardAccelerationMps2 =
                state.GetForwardAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (state.GetYawAccel() * imuLeverArmBodyM.x());

            Assert::AreEqual(
                expectedRightAccelerationMps2,
                predictedImuRightAccelerationMps2,
                1.0e-5f);
            Assert::AreEqual(
                expectedForwardAccelerationMps2,
                predictedImuForwardAccelerationMps2,
                1.0e-5f);
        }

        TEST_METHOD(PlantModelSymmetricPositiveDriveFromRestMovesForward)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(0.0f);
            state.SetWheelSpeedRight(0.0f);
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.50f);
            control.SetRightCommand(0.50f);

            constexpr float dt = 0.002f;
            constexpr int kSteps = 25;
            for (int step = 0; step < kSteps; ++step)
            {
                plant.integrate(control, dt);
            }

            const float totalTimeS = dt * static_cast<float>(kSteps);
            const float averageAccelMps2 = state.GetForwardVelocity() / totalTimeS;

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetHeading()));
            Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRightwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetYawRate()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(state.GetForwardVelocity() > 0.0f);
            Assert::IsTrue(state.GetPositionY() > 0.09f);
            Assert::IsTrue(averageAccelMps2 > 0.0f);
            Assert::IsTrue(averageAccelMps2 < 60.0f);
            Assert::IsTrue(std::fabs(state.GetPositionX()) < 0.002f);
            Assert::IsTrue(std::fabs(state.GetRightwardVelocity()) < 0.02f);
            Assert::IsTrue(std::fabs(state.GetYawRate()) < 0.10f);
            Assert::IsTrue(std::fabs(state.GetHeading()) < 0.01f);
        }

        TEST_METHOD(PlantModelStaticFrictionHoldsSubthresholdDriveAtRest)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(0.0f);
            state.SetWheelSpeedRight(0.0f);
            PlantModel plant(vehicle, state);
            const float staticWindowRadps = plant.staticFrictionSpeedThresholdRadps();

            Assert::AreEqual(
                plant.staticFrictionTorqueNm(),
                plant.driveFrictionTorque(0.5f * staticWindowRadps, 1.0f),
                1.0e-6f);
            Assert::AreEqual(
                -plant.staticFrictionTorqueNm(),
                plant.driveFrictionTorque(-0.5f * staticWindowRadps, -1.0f),
                1.0e-6f);
            Assert::AreEqual(
                plant.rollingFrictionTorqueNm(),
                plant.driveFrictionTorque(1.1f * staticWindowRadps, 1.0f),
                1.0e-6f);
            Assert::AreEqual(
                -plant.rollingFrictionTorqueNm(),
                plant.driveFrictionTorque(-1.1f * staticWindowRadps, -1.0f),
                1.0e-6f);

            for (int step = 0; step < 100; ++step)
            {
                plant.integrate(control, 0.001f);
            }

            Assert::AreEqual(0.0f, state.GetForwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRightwardVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetYawRate(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), 1.0e-6f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelIntegrateSingleLargeStepRemainsFiniteAndSymmetric)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(0.0f);
            state.SetWheelSpeedRight(0.0f);
            PlantModel plant(vehicle, state);
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.45f);
            control.SetRightCommand(0.45f);

            constexpr float dt = 0.004f;
            plant.integrate(control, dt);

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetHeading()));
            Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRightwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetYawRate()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(std::isfinite(state.GetForwardVelocity()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(
                std::fabs(state.GetWheelSpeedLeft() -
                    state.GetWheelSpeedRight()) < 1.0f);
            Assert::IsTrue(std::fabs(state.GetPositionX()) < 0.005f);
            Assert::IsTrue(std::fabs(state.GetYawRate()) < 0.10f);
        }

        TEST_METHOD(PlantModelIntegratePreservesHeadingNormalization)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(PI_F - 0.01f);
            state.SetForwardVelocity(0.5f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(6.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            plant.integrate(control, 0.01f);

            Assert::IsTrue(state.GetHeading() <= PI_F);
            Assert::IsTrue(state.GetHeading() >= -PI_F);
        }

    };
}






