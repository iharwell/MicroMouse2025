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
        constexpr float kRadiansToDegrees = 180.0f / PI_F;
        constexpr float kInPlaceSlipContextDegrees = 22.6f;
        constexpr float kSymmetricFrontLoadFraction = 0.5f;
        constexpr float kStopEnterSpeedMps = 0.02f;
        constexpr float kStopEnterYawRateRadps = 0.20f;
        constexpr float kStopEnterWheelSpeedRadps = 2.0f;
    }

    TEST_CLASS(PlantModelDynamicsTest)
    {
    public:
        TEST_METHOD(PlantModelUsesDriveWheelConstructionValues)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            Assert::AreEqual(2.4e-7f, plant.leftDriveEquivalentWheelInertiaKgM2(), 1.0e-10f);
            Assert::AreEqual(2.4e-7f, plant.rightDriveEquivalentWheelInertiaKgM2(), 1.0e-10f);
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
            state.SetOrientation(0.0f);
            state.SetVelocity(forwardVelocityMps);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);
            const float initialYawRateRadps = state.GetRotationalVelocity();

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.55f);
            control.SetRightCommand(0.55f);

            plant.integrate(control, 0.001f);
            Assert::IsTrue(std::isfinite(state.GetRotationalVelocity()));
            Assert::AreEqual(
                initialYawRateRadps,
                state.GetRotationalVelocity(),
                1.0e-6f);
        }

        TEST_METHOD(PlantModelTireForcesRetainPreProjectionUtilizationAboveUnity)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
                L"yaw_rad=" << state.GetOrientation() << L"\n"
                L"forward_velocity_mps=" << state.GetVelocity() << L"\n"
                L"lateral_velocity_mps=" << state.GetLateralVelocity() << L"\n"
                L"yaw_rate_radps=" << state.GetRotationalVelocity() << L"\n"
                L"left_wheel_speed_radps=" << state.GetWheelSpeedLeft() << L"\n"
                L"right_wheel_speed_radps=" << state.GetWheelSpeedRight();
            Assert::IsTrue(
                std::isfinite(state.GetPositionX()) &&
                std::isfinite(state.GetPositionY()) &&
                std::isfinite(state.GetOrientation()) &&
                std::isfinite(state.GetVelocity()) &&
                std::isfinite(state.GetLateralVelocity()) &&
                std::isfinite(state.GetRotationalVelocity()) &&
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            auto plant = PlantModel(vehicle, state);

            const App::Internal::CommandVector control{};
            const float initialSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedLeft();
            plant.integrate(control, 0.001f);

            const float finalSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(45.0f);
            state.SetWheelSpeedRight(43.0f);
            auto plant = PlantModel(vehicle, state);


            const App::Internal::CommandVector control{};
            const float initialSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
            const float initialSlipAbsMps = std::fabs(initialSlipMps);
            const float initialWheelSpeedRadps = state.GetWheelSpeedRight();
            plant.integrate(control, 0.001f);

            const float finalSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.05f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(55.0f);
            state.SetWheelSpeedRight(55.0f);
			auto plant = PlantModel(vehicle, state);
            const App::Internal::CommandVector control{};

            const float initialLeftAbsRadps = std::fabs(state.GetWheelSpeedLeft());
            const float initialRightAbsRadps = std::fabs(state.GetWheelSpeedRight());
            plant.integrate(control, 0.001f);

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetOrientation()));
            Assert::IsTrue(std::isfinite(state.GetVelocity()));
            Assert::IsTrue(std::isfinite(state.GetLateralVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRotationalVelocity()));
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
            state.SetOrientation(0.0f);
            state.SetVelocity(1.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
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
            highSlipState.SetOrientation(0.0f);
            highSlipState.SetVelocity(1.0f);
            highSlipState.SetLateralVelocity(1.25f);
            highSlipState.SetRotationalVelocity(0.0f);
            highSlipState.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
            highSlipState.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
            highSlipState.SetGyroBiasZ(0.0f);
            PlantModel highSlipPlant(vehicle, highSlipState);
            float highSlipAccelMps2 = 0.0f;
            for (int tick = 0; tick < 5; ++tick)
            {
                const float beforeLateralVelocityMps = highSlipState.GetLateralVelocity();
                highSlipPlant.integrate(control, dtSeconds);
                const float afterLateralVelocityMps = highSlipState.GetLateralVelocity();
                highSlipAccelMps2 =
                    (std::max)(
                        highSlipAccelMps2,
                        std::fabs((afterLateralVelocityMps - beforeLateralVelocityMps) / dtSeconds));
            }

            VehicleState extremeSlipState;
            extremeSlipState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            extremeSlipState.SetOrientation(0.0f);
            extremeSlipState.SetVelocity(1.0f);
            extremeSlipState.SetLateralVelocity(3.50f);
            extremeSlipState.SetRotationalVelocity(0.0f);
            extremeSlipState.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
            extremeSlipState.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(1.0f));
            extremeSlipState.SetGyroBiasZ(0.0f);
            PlantModel extremeSlipPlant(vehicle, extremeSlipState);
            float extremeSlipAccelMps2 = 0.0f;
            for (int tick = 0; tick < 5; ++tick)
            {
                const float beforeLateralVelocityMps = extremeSlipState.GetLateralVelocity();
                extremeSlipPlant.integrate(control, dtSeconds);
                const float afterLateralVelocityMps = extremeSlipState.GetLateralVelocity();
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
            const float contactY = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float yawRateRadps = 30.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(yawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(halfTrackM * yawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(-halfTrackM * yawRateRadps));
            PlantModel plant(vehicle, state);

            const float totalSustainedLateralForceN =
                vehicle.GetMass() * Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) * totalSustainedLateralForceN;

            Assert::AreEqual(
                kInPlaceSlipContextDegrees,
                std::fabs(plant.contactLateralSlipAngleRad(0U)) * kRadiansToDegrees,
                0.5f);
            Assert::AreEqual(
                kInPlaceSlipContextDegrees,
                std::fabs(plant.contactLateralSlipAngleRad(1U)) * kRadiansToDegrees,
                0.5f);
            Assert::AreEqual(
                kInPlaceSlipContextDegrees,
                std::fabs(plant.contactLateralSlipAngleRad(2U)) * kRadiansToDegrees,
                0.5f);
            Assert::AreEqual(
                kInPlaceSlipContextDegrees,
                std::fabs(plant.contactLateralSlipAngleRad(3U)) * kRadiansToDegrees,
                0.5f);
            Assert::AreEqual(-frontContactLimitN, plant.contactRightForceN(control, 0U), 1.0e-4f);
            Assert::AreEqual(-frontContactLimitN, plant.contactRightForceN(control, 1U), 1.0e-4f);
            Assert::AreEqual(rearContactLimitN, plant.contactRightForceN(control, 2U), 1.0e-4f);
            Assert::AreEqual(rearContactLimitN, plant.contactRightForceN(control, 3U), 1.0e-4f);

            constexpr float dtSeconds = 0.001f;
            const float initialYawRateRadps = state.GetRotationalVelocity();
            plant.integrate(control, dtSeconds);
            const float observedYawAccelRadps2 =
                state.GetYawAcceleration();
            const float actualYawRateRadps =
                state.GetRotationalVelocity();
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
            belowState.SetOrientation(0.0f);
            belowState.SetVelocity(referenceForwardSpeedMps - nearbyDeltaMps);
            belowState.SetLateralVelocity(0.0f);
            belowState.SetRotationalVelocity(yawRateRadps);
            belowState.SetWheelSpeedLeft(
                Vehicle::WheelOmegaFromLinearVelocity(
                    belowState.GetVelocity() + (halfTrackM * yawRateRadps)));
            belowState.SetWheelSpeedRight(
                Vehicle::WheelOmegaFromLinearVelocity(
                    belowState.GetVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel belowPlant(vehicle, belowState);
            belowPlant.integrate(control, dtSeconds);
            const float belowYawAccelRadps2 = belowState.GetYawAcceleration();

            VehicleState centerState;
            centerState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            centerState.SetOrientation(0.0f);
            centerState.SetVelocity(referenceForwardSpeedMps);
            centerState.SetLateralVelocity(0.0f);
            centerState.SetRotationalVelocity(yawRateRadps);
            centerState.SetWheelSpeedLeft(
                Vehicle::WheelOmegaFromLinearVelocity(
                    centerState.GetVelocity() + (halfTrackM * yawRateRadps)));
            centerState.SetWheelSpeedRight(
                Vehicle::WheelOmegaFromLinearVelocity(
                    centerState.GetVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel centerPlant(vehicle, centerState);
            centerPlant.integrate(control, dtSeconds);
            const float centerYawAccelRadps2 = centerState.GetYawAcceleration();

            VehicleState aboveState;
            aboveState.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            aboveState.SetOrientation(0.0f);
            aboveState.SetVelocity(referenceForwardSpeedMps + nearbyDeltaMps);
            aboveState.SetLateralVelocity(0.0f);
            aboveState.SetRotationalVelocity(yawRateRadps);
            aboveState.SetWheelSpeedLeft(
                Vehicle::WheelOmegaFromLinearVelocity(
                    aboveState.GetVelocity() + (halfTrackM * yawRateRadps)));
            aboveState.SetWheelSpeedRight(
                Vehicle::WheelOmegaFromLinearVelocity(
                    aboveState.GetVelocity() - (halfTrackM * yawRateRadps)));
            PlantModel abovePlant(vehicle, aboveState);
            abovePlant.integrate(control, dtSeconds);
            const float aboveYawAccelRadps2 = aboveState.GetYawAcceleration();
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

        TEST_METHOD(PlantModelInPlaceSlipYawRateIsPassiveWithBoundedRebound)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);

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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(initialYawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(halfTrackM * initialYawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(-halfTrackM * initialYawRateRadps));
            PlantModel plant(vehicle, state);

            float previousYawRateAbsRadps = std::fabs(state.GetRotationalVelocity());
            float minYawRateAfterInitialDecayAbsRadps = previousYawRateAbsRadps;
            float maxYawRateAbsRadps = previousYawRateAbsRadps;
            float maxReboundRadps = 0.0f;
            int maxReboundStep = -1;
            int stopStep = -1;
            for (int step = 0; step < maxSteps; ++step)
            {
                plant.integrate(control, dtSeconds);
                const float yawRateAbsRadps =
                    std::fabs(state.GetRotationalVelocity());

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
            const float contactY = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);

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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(initialYawRateRadps);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(halfTrackM * initialYawRateRadps));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(-halfTrackM * initialYawRateRadps));
            PlantModel plant(vehicle, state);

            int stopStep = -1;
            float previousYawRateAbsRadps = std::fabs(state.GetRotationalVelocity());
            float minYawRateAfterInitialDecayAbsRadps = previousYawRateAbsRadps;
            float maxReboundRadps = 0.0f;
            int maxReboundStep = -1;
            float finalYawRateAbsRadps = std::fabs(state.GetRotationalVelocity());
            for (int step = 0; step < maxSteps; ++step)
            {
                plant.integrate(control, dtSeconds);
                finalYawRateAbsRadps =
                    std::fabs(state.GetRotationalVelocity());

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
                Vehicle::WheelOmegaFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.03f, 0.09f));
            state.SetOrientation(0.21f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
            Assert::AreEqual(0.21f, state.GetOrientation(), 1.0e-7f);
            Assert::AreEqual(0.12f, state.GetGyroBiasZ(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetLateralVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRotationalVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelSmallStationaryPerturbationsSnapBackToRest)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float zeroWheelSpeedToleranceRadps =
                Vehicle::WheelOmegaFromLinearVelocity(kZeroLinearVelocityToleranceMps);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.05f);
            state.SetWheelSpeedLeft(0.8f);
            state.SetWheelSpeedRight(-0.7f);
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 250; ++step)
            {
				plant.integrate(control, dt);
            }

            Assert::AreEqual(0.0f, state.GetVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetLateralVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRotationalVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelNearZeroLateralPerturbationsSnapBackToRest)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState initialState;
            initialState.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            initialState.SetOrientation(0.0f);
            initialState.SetVelocity(0.0f);
            initialState.SetLateralVelocity(0.001f);
            initialState.SetRotationalVelocity(0.05f);
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

            Assert::IsTrue(std::fabs(state.GetVelocity()) < kStopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state.GetLateralVelocity()) < kStopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state.GetRotationalVelocity()) < kStopEnterYawRateRadps);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < kStopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < kStopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state.GetRotationalVelocity()) < std::fabs(initialState.GetRotationalVelocity()));
            Assert::IsTrue(std::fabs(state.GetWheelSpeedLeft()) < std::fabs(initialState.GetWheelSpeedLeft()));
            Assert::IsTrue(std::fabs(state.GetWheelSpeedRight()) < std::fabs(initialState.GetWheelSpeedRight()));
        }

        TEST_METHOD(PlantModelMixedSlipCommandIntegratesFiniteState)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 2.0f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetOrientation(0.0f);
            state.SetVelocity(forwardVelocityMps);
            state.SetLateralVelocity(0.15f);
            state.SetRotationalVelocity(1.8f);
            state.SetWheelSpeedLeft(1.05f * Vehicle::WheelOmegaFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(0.95f * Vehicle::WheelOmegaFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.65f);
            control.SetRightCommand(0.60f);

            const float leftLongitudinalSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedLeft()) -
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
            const float rightLongitudinalSlipMps =
                Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedRight()) -
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetVelocity(),
                    state.GetRotationalVelocity());
            Assert::IsTrue(std::isfinite(leftLongitudinalSlipMps));
            Assert::IsTrue(std::isfinite(rightLongitudinalSlipMps));
            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                const float lateralSlipAngleRad = plant.contactLateralSlipAngleRad(contactIndex);
                const float rightForceN = plant.contactRightForceN(control, contactIndex);
                const float forwardForceN = plant.contactForwardForceN(control, contactIndex);
                const float saturation = plant.contactSaturation(control, contactIndex);
                const float preProjectionUtilization =
                    plant.contactPreProjectionUtilization(control, contactIndex);
                Assert::IsTrue(std::isfinite(lateralSlipAngleRad));
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
            Assert::IsTrue(std::isfinite(state.GetOrientation()));
            Assert::IsTrue(std::isfinite(state.GetVelocity()));
            Assert::IsTrue(std::isfinite(state.GetLateralVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRotationalVelocity()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(state.GetOrientation() <= PI_F);
            Assert::IsTrue(state.GetOrientation() >= -PI_F);
            Assert::IsTrue(std::fabs(state.GetVelocity()) < 10.0f);
            Assert::IsTrue(std::fabs(state.GetLateralVelocity()) < 10.0f);
            Assert::IsTrue(std::fabs(state.GetRotationalVelocity()) < 50.0f);
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
            state.SetOrientation(0.0f);
            state.SetVelocity(1.4f);
            state.SetLateralVelocity(0.2f);
            state.SetRotationalVelocity(initialYawRateRadps);
            state.SetWheelSpeedLeft(0.9f * Vehicle::WheelOmegaFromLinearVelocity(1.4f));
            state.SetWheelSpeedRight(1.1f * Vehicle::WheelOmegaFromLinearVelocity(1.4f));
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
                state.GetLateralAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (state.GetYawAcceleration() * imuLeverArmBodyM.y());
            const float expectedForwardAccelerationMps2 =
                state.GetLongitudinalAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (state.GetYawAcceleration() * imuLeverArmBodyM.x());
            const float rightLeverContributionMps2 =
                predictedImuRightAccelerationMps2 - state.GetLateralAcceleration();
            const float forwardLeverContributionMps2 =
                predictedImuForwardAccelerationMps2 - state.GetLongitudinalAcceleration();

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
            state.SetOrientation(0.0f);
            state.SetVelocity(1.1f);
            state.SetLateralVelocity(-0.3f);
            state.SetRotationalVelocity(initialYawRateRadps);
            state.SetWheelSpeedLeft(0.95f * Vehicle::WheelOmegaFromLinearVelocity(1.1f));
            state.SetWheelSpeedRight(1.05f * Vehicle::WheelOmegaFromLinearVelocity(1.1f));
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
                state.GetLateralAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (state.GetYawAcceleration() * imuLeverArmBodyM.y());
            const float expectedForwardAccelerationMps2 =
                state.GetLongitudinalAcceleration() -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (state.GetYawAcceleration() * imuLeverArmBodyM.x());

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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
            const float averageAccelMps2 = state.GetVelocity() / totalTimeS;

            Assert::IsTrue(std::isfinite(state.GetPositionX()));
            Assert::IsTrue(std::isfinite(state.GetPositionY()));
            Assert::IsTrue(std::isfinite(state.GetOrientation()));
            Assert::IsTrue(std::isfinite(state.GetVelocity()));
            Assert::IsTrue(std::isfinite(state.GetLateralVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRotationalVelocity()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(state.GetVelocity() > 0.0f);
            Assert::IsTrue(state.GetPositionY() > 0.09f);
            Assert::IsTrue(averageAccelMps2 > 0.0f);
            Assert::IsTrue(averageAccelMps2 < 60.0f);
            Assert::IsTrue(std::fabs(state.GetPositionX()) < 0.002f);
            Assert::IsTrue(std::fabs(state.GetLateralVelocity()) < 0.02f);
            Assert::IsTrue(std::fabs(state.GetRotationalVelocity()) < 0.10f);
            Assert::IsTrue(std::fabs(state.GetOrientation()) < 0.01f);
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
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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

            Assert::AreEqual(0.0f, state.GetVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetLateralVelocity(), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state.GetRotationalVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), 1.0e-6f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelIntegrateSingleLargeStepRemainsFiniteAndSymmetric)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetOrientation(0.0f);
            state.SetVelocity(0.0f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(0.0f);
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
            Assert::IsTrue(std::isfinite(state.GetOrientation()));
            Assert::IsTrue(std::isfinite(state.GetVelocity()));
            Assert::IsTrue(std::isfinite(state.GetLateralVelocity()));
            Assert::IsTrue(std::isfinite(state.GetRotationalVelocity()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(std::isfinite(state.GetGyroBiasZ()));
            Assert::IsTrue(std::isfinite(state.GetVelocity()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedLeft()));
            Assert::IsTrue(std::isfinite(state.GetWheelSpeedRight()));
            Assert::IsTrue(
                std::fabs(state.GetWheelSpeedLeft() -
                    state.GetWheelSpeedRight()) < 1.0f);
            Assert::IsTrue(std::fabs(state.GetPositionX()) < 0.005f);
            Assert::IsTrue(std::fabs(state.GetRotationalVelocity()) < 0.10f);
        }

        TEST_METHOD(PlantModelIntegratePreservesHeadingNormalization)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetOrientation(PI_F - 0.01f);
            state.SetVelocity(0.5f);
            state.SetLateralVelocity(0.0f);
            state.SetRotationalVelocity(6.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(0.5f));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(0.5f));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control{};
            plant.integrate(control, 0.01f);

            Assert::IsTrue(state.GetOrientation() <= PI_F);
            Assert::IsTrue(state.GetOrientation() >= -PI_F);
        }

    };
}






