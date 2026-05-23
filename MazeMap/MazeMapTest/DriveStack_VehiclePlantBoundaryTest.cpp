#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {

        void GetRepresentativeWheelSpeeds(float& leftWheelSpeedRadps, float& rightWheelSpeedRadps) noexcept
        {
            Vehicle::WheelSpeedsFromBodyVelocity(0.72f, 1.35f, leftWheelSpeedRadps, rightWheelSpeedRadps);
        }
    }

    TEST_CLASS(DriveStack_VehiclePlantBoundaryTest)
    {
    public:
        TEST_METHOD(VehicleMass_DrivesPlantLongitudinalTechnicalLimit)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            float actualForwardAccelMps2 = 0.0f;
            float unusedYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(actualForwardAccelMps2, unusedYawAccelRadps2);

            const MotorEncoderDrive& leftDrive = vehicle.GetLeftMotorEncoderDrive();
            const MotorEncoderDrive& rightDrive = vehicle.GetRightMotorEncoderDrive();
            const float batteryVoltageV = vehicle.GetBatteryVoltage();
            const float leftLimitN = (std::min)(
                std::fabs(leftDrive.getForwardForceFromCommand(1.0f, 0.0f, batteryVoltageV)),
                std::fabs(leftDrive.getForwardForceFromCommand(-1.0f, 0.0f, batteryVoltageV)));
            const float rightLimitN = (std::min)(
                std::fabs(rightDrive.getForwardForceFromCommand(1.0f, 0.0f, batteryVoltageV)),
                std::fabs(rightDrive.getForwardForceFromCommand(-1.0f, 0.0f, batteryVoltageV)));
            const float expectedForwardAccelMps2 =
                (2.0f * (std::min)(leftLimitN, rightLimitN)) / vehicle.GetMass();

            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=max_longitudinal_accel_mps2"
                << L"\nexpected=" << expectedForwardAccelMps2
                << L"\nactual=" << actualForwardAccelMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                expectedForwardAccelMps2,
                actualForwardAccelMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(VehicleTrackWidth_DrivesMeasuredYawRate)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            EncoderObs observation{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            GetRepresentativeWheelSpeeds(leftWheelSpeedRadps, rightWheelSpeedRadps);
            observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
            const float actualYawRateRadps = plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=measured_yaw_rate_radps"
                << L"\nexpected=1.35"
                << L"\nactual=" << actualYawRateRadps
                << L"\ntolerance=1e-6"
                << L"\ntrack_width_m=" << vehicle.GetTrackWidth();

            Assert::AreEqual(
                1.35f,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(VehicleYawInertia_DrivesPlantYawTechnicalLimit)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            float unusedForwardAccelMps2 = 0.0f;
            float actualYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(unusedForwardAccelMps2, actualYawAccelRadps2);

            const MotorEncoderDrive& leftDrive = vehicle.GetLeftMotorEncoderDrive();
            const MotorEncoderDrive& rightDrive = vehicle.GetRightMotorEncoderDrive();
            const float batteryVoltageV = vehicle.GetBatteryVoltage();
            const float leftLimitN = (std::min)(
                std::fabs(leftDrive.getForwardForceFromCommand(1.0f, 0.0f, batteryVoltageV)),
                std::fabs(leftDrive.getForwardForceFromCommand(-1.0f, 0.0f, batteryVoltageV)));
            const float rightLimitN = (std::min)(
                std::fabs(rightDrive.getForwardForceFromCommand(1.0f, 0.0f, batteryVoltageV)),
                std::fabs(rightDrive.getForwardForceFromCommand(-1.0f, 0.0f, batteryVoltageV)));
            const float expectedYawAccelRadps2 =
                (vehicle.GetTrackWidth() * (std::min)(leftLimitN, rightLimitN)) / vehicle.GetYawInertia();

            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=max_yaw_accel_radps2"
                << L"\nexpected=" << expectedYawAccelRadps2
                << L"\nactual=" << actualYawAccelRadps2
                << L"\ntolerance=0.05";

            Assert::AreEqual(
                expectedYawAccelRadps2,
                actualYawAccelRadps2,
                5.0e-2f,
                message.str().c_str());
        }

        TEST_METHOD(VehicleWheelRadius_DrivesMeasuredLinearSpeed)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            EncoderObs observation{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            GetRepresentativeWheelSpeeds(leftWheelSpeedRadps, rightWheelSpeedRadps);
            observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
            const float actualSpeedMps = plant.measuredLinearSpeedMps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=measured_linear_speed_mps"
                << L"\nexpected=0.72"
                << L"\nactual=" << actualSpeedMps
                << L"\ntolerance=1e-6"
                << L"\nwheel_radius_m=" << Vehicle::GetDriveWheelRadiusM();

            Assert::AreEqual(
                0.72f,
                actualSpeedMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(VehicleSustainedAcceleration_DrivesPlantUsage)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            const float actualUsage =
                plant.sustainedCombinedAccelerationUsage(
                    Vehicle::GetSustainedLateralAccelerationReferenceMps2());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=sustained_combined_accel_usage"
                << L"\nexpected=1"
                << L"\nactual=" << actualUsage
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                1.0f,
                actualUsage,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PlantDynamics_LateralAccelerationMatchesVehicleSustainedReference)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            runtimeState.SetRightwardVelocity(100.0f);
            constexpr float dtSeconds = 0.001f;
            plant.integrate(App::Internal::CommandVector{}, dtSeconds);
            const float actualAccelMps2 =
                std::fabs(runtimeState.GetRightAcceleration());
            const float expectedAccelMps2 =
                Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=lateral_accel_mps2"
                << L"\nexpected=" << expectedAccelMps2
                << L"\nactual=" << actualAccelMps2
                << L"\ntolerance=1e-3";

            Assert::AreEqual(
                expectedAccelMps2,
                actualAccelMps2,
                1.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ForwardRequestProducesPositivePlantAcceleration)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            runtimeState.SetForwardVelocity(0.30f);
            runtimeState.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(0.30f));
            runtimeState.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(0.30f));

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(1.20f, 0.0f);
            const float initialForwardVelocityMps = runtimeState.GetForwardVelocity();
            plant.integrate(command, 0.001f);
            std::wstringstream commandMessage;
            commandMessage << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=forward_feedforward_command_finite"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";
            std::wstringstream accelMessage;
            accelMessage << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=forward_velocity_after_step_mps"
                << L"\ninitial=" << initialForwardVelocityMps
                << L"\nactual=" << runtimeState.GetForwardVelocity()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                command.IsFinite(),
                commandMessage.str().c_str());
            Assert::IsTrue(
                runtimeState.GetForwardVelocity() > initialForwardVelocityMps,
                accelMessage.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ForwardRequestKeepsWheelCommandsSymmetric)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            runtimeState.SetForwardVelocity(0.30f);
            runtimeState.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(0.30f));
            runtimeState.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(0.30f));

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(1.20f, 0.0f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=forward_command_symmetry"
                << L"\nexpected_left=" << command.LeftCommand()
                << L"\nactual_right=" << command.RightCommand()
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                command.LeftCommand(),
                command.RightCommand(),
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ClockwiseYawRequestCommandsLeftAboveRight)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 8.50f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=clockwise_yaw_command_order"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                command.LeftCommand() > command.RightCommand(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ClockwiseYawRequestProducesPositiveYawAcceleration)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 8.50f);
            plant.integrate(command, 0.001f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=yaw_rate_after_step_radps"
                << L"\nactual=" << runtimeState.GetYawRate()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand();

            Assert::IsTrue(
                runtimeState.GetYawRate() > 0.0f,
                message.str().c_str());
        }

        // This is based on directly observed yaw motion thresholds.
        TEST_METHOD(YawFeedforwardGivesRealisticCommand)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 0.50f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=feedforward yaw"
                << L"\nactual=" << ((command.LeftCommand() - command.RightCommand())/2.0f)
                << L"\ncriterion=actual>0.55";

            Assert::IsTrue(
                ((command.LeftCommand() - command.RightCommand()) / 2.0f) > 0.55f,
                message.str().c_str());
        }

        // This is based on directly observed yaw motion thresholds.
        TEST_METHOD(YawFeedforwardGivesRealisticCommandWithRotation)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
			runtimeState.SetYawRate(0.05f);
            auto plant = PlantModel(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 0.50f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=feedforward yaw"
                << L"\nactual=" << ((command.LeftCommand() - command.RightCommand()) / 2.0f)
                << L"\ncriterion=actual>0.55";

            Assert::IsTrue(
                ((command.LeftCommand() - command.RightCommand()) / 2.0f) > 0.55f,
                message.str().c_str());
        }
        TEST_METHOD(StraightFeedforwardGivesRealisticCommand)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(2.0f, 0.0f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=feedforward accel"
                << L"\nactual=" << ((command.LeftCommand() + command.RightCommand()) / 2.0f)
                << L"\ncriterion=actual>0.2";

            Assert::IsTrue(
                ((command.LeftCommand() + command.RightCommand()) / 2.0f) > 0.2f,
                message.str().c_str());
        }

        TEST_METHOD(Kinematics_VehicleWheelSpeedsRoundTripThroughPlantMeasuredSpeed)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            EncoderObs observation{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(0.30f, 1.75f, leftWheelSpeedRadps, rightWheelSpeedRadps);
            observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
            const float actualSpeedMps = plant.measuredLinearSpeedMps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=round_trip_speed_mps"
                << L"\nexpected=0.3"
                << L"\nactual=" << actualSpeedMps
                << L"\ntolerance=1e-6"
                << L"\nleft_wheel_speed_radps=" << observation.LeftWheelSpeedRadps()
                << L"\nright_wheel_speed_radps=" << observation.RightWheelSpeedRadps();

            Assert::AreEqual(
                0.30f,
                actualSpeedMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Kinematics_VehicleWheelSpeedsRoundTripThroughPlantMeasuredYawRate)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            auto plant = PlantModel(vehicle, runtimeState);
            EncoderObs observation{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(0.30f, 1.75f, leftWheelSpeedRadps, rightWheelSpeedRadps);
            observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
            const float actualYawRateRadps = plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=round_trip_yaw_rate_radps"
                << L"\nexpected=1.75"
                << L"\nactual=" << actualYawRateRadps
                << L"\ntolerance=1e-6"
                << L"\nleft_wheel_speed_radps=" << observation.LeftWheelSpeedRadps()
                << L"\nright_wheel_speed_radps=" << observation.RightWheelSpeedRadps();

            Assert::AreEqual(
                1.75f,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

    };
}
