#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kInf = (std::numeric_limits<float>::infinity)();
        constexpr float kNaN = (std::numeric_limits<float>::quiet_NaN)();
        constexpr float kAccelerationToleranceMps2 = 0.10f;
        constexpr float kYawAccelerationToleranceRadps2 = 10.0f * kPi / 180.0f;

        bool IsFiniteControlVector(const App::Internal::CommandVector& control) noexcept
        {
            return
                std::isfinite(control.LeftCommand()) &&
                std::isfinite(control.RightCommand());
        }

        float IntegratedRateOfChange(
            const float before,
            const float after,
            const float dtSeconds)
        {
            std::wstringstream message;
            message <<
                L"IntegratedRateOfChange\n"
                L"field=dt_seconds\n"
                L"actual=" << dtSeconds << L"\n"
                L"expected=0.001\n"
                L"tolerance=1e-6\n"
                L"criterion=abs(actual - expected) <= tolerance";
            Assert::AreEqual(0.001f, dtSeconds, 1.0e-6f, message.str().c_str());
            return (after - before) / dtSeconds;
        }
    }

    TEST_CLASS(PlantModelDriveCommandTest)
    {
    public:

        static constexpr float dtSeconds = 0.001f;
        TEST_METHOD(PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(0.0f, 0.0f);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand\n"
                    L"field=left_command\n"
                    L"actual=" << control.LeftCommand() << L"\n"
                    L"expected=0\n"
                    L"tolerance=1e-6\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(0.0f, control.LeftCommand(), 1.0e-6f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand\n"
                    L"field=right_command\n"
                    L"actual=" << control.RightCommand() << L"\n"
                    L"expected=0\n"
                    L"tolerance=1e-6\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(0.0f, control.RightCommand(), 1.0e-6f, message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteSymmetricCommandForForwardRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(1.0f, 0.0f);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardReturnsFiniteSymmetricCommandForForwardRequest\n"
                    L"field=feedforward_command\n"
                    L"actual={left_command=" << control.LeftCommand() <<
                    L", right_command=" << control.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(control), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardReturnsFiniteSymmetricCommandForForwardRequest\n"
                    L"field=right_command_symmetry\n"
                    L"actual=" << control.RightCommand() << L"\n"
                    L"expected_left_command=" << control.LeftCommand() << L"\n"
                    L"tolerance=1e-5\n"
                    L"criterion=abs(actual - expected_left_command) <= tolerance";
                Assert::AreEqual(
                    control.LeftCommand(),
                    control.RightCommand(),
                    1.0e-5f,
                    message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsSplitCommandForYawRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(0.0f, 8.0f);
            const float commandSplit = std::fabs(control.LeftCommand() - control.RightCommand());

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardReturnsSplitCommandForYawRequest\n"
                    L"field=feedforward_command\n"
                    L"actual={left_command=" << control.LeftCommand() <<
                    L", right_command=" << control.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(control), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardReturnsSplitCommandForYawRequest\n"
                    L"field=command_split\n"
                    L"actual=" << commandSplit << L"\n"
                    L"left_command=" << control.LeftCommand() << L"\n"
                    L"right_command=" << control.RightCommand() << L"\n"
                    L"criterion=actual > 0.0001";
                Assert::IsTrue(commandSplit > 1.0e-4f, message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteOutputForCombinedRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(1.25f, 3.75f);

            std::wstringstream message;
            message <<
                L"PlantModelAccelerationFeedforwardReturnsFiniteOutputForCombinedRequest\n"
                L"field=feedforward_command\n"
                L"actual={left_command=" << control.LeftCommand() <<
                L", right_command=" << control.RightCommand() << L"}\n"
                L"criterion=isfinite(left_command) && isfinite(right_command)";
            Assert::IsTrue(IsFiniteControlVector(control), message.str().c_str());
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardIgnoresObservedWheelMismatchForForwardRequest)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);
            state.SetForwardVelocity(0.35f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(-30.0f);
            state.SetWheelSpeedRight(70.0f);

            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(4.0f, 0.0f);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardIgnoresObservedWheelMismatchForForwardRequest\n"
                    L"field=feedforward_command\n"
                    L"actual={left_command=" << control.LeftCommand() <<
                    L", right_command=" << control.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(control), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardIgnoresObservedWheelMismatchForForwardRequest\n"
                    L"field=right_command_symmetry\n"
                    L"actual=" << control.RightCommand() << L"\n"
                    L"expected_left_command=" << control.LeftCommand() << L"\n"
                    L"tolerance=1e-5\n"
                    L"criterion=abs(actual - expected_left_command) <= tolerance";
                Assert::AreEqual(
                    control.LeftCommand(),
                    control.RightCommand(),
                    1.0e-5f,
                    message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf)
        {
            Vehicle vehicle;
            VehicleState restState;
            restState.SetForwardVelocity(0.25f);

            // We deliberately set the wheel speeds higher on the slow state to ensure back-emf is not looking at the wheel speeds.
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;

            vehicle.WheelSpeedsFromBodyVelocity(
                restState.GetForwardVelocity(),
                restState.GetYawRate(),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            restState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            restState.SetWheelSpeedRight(rightWheelSpeedRadps);
            PlantModel slowPlant(vehicle, restState);
            const App::Internal::CommandVector slowControl =
                slowPlant.ComputeFeedforward(4.0f, 0.0f);

            VehicleState movingState;
            movingState.SetForwardVelocity(0.75f);
            movingState.SetWheelSpeedLeft(0.0f);
            movingState.SetWheelSpeedRight(0.0f);
            PlantModel movingPlant(vehicle, movingState);
            const App::Internal::CommandVector movingControl =
                movingPlant.ComputeFeedforward(4.0f, 0.0f);
            const float leftCommandDelta =
                std::fabs(slowControl.LeftCommand() - movingControl.LeftCommand());
            const float rightCommandDelta =
                std::fabs(slowControl.RightCommand() - movingControl.RightCommand());

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf\n"
                    L"field=slow_state_feedforward_command\n"
                    L"actual={left_command=" << slowControl.LeftCommand() <<
                    L", right_command=" << slowControl.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(slowControl), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf\n"
                    L"field=moving_state_feedforward_command\n"
                    L"actual={left_command=" << movingControl.LeftCommand() <<
                    L", right_command=" << movingControl.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(movingControl), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf\n"
                    L"field=left_command_delta\n"
                    L"actual=" << leftCommandDelta << L"\n"
                    L"slow_left_command=" << slowControl.LeftCommand() << L"\n"
                    L"moving_left_command=" << movingControl.LeftCommand() << L"\n"
                    L"criterion=actual > 1e-5";
                Assert::IsTrue(leftCommandDelta > 1.0e-5f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf\n"
                    L"field=right_command_delta\n"
                    L"actual=" << rightCommandDelta << L"\n"
                    L"slow_right_command=" << slowControl.RightCommand() << L"\n"
                    L"moving_right_command=" << movingControl.RightCommand() << L"\n"
                    L"criterion=actual > 1e-5";
                Assert::IsTrue(rightCommandDelta > 1.0e-5f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf\n"
                    L"field=moving_average_command\n"
                    L"actual=" << movingControl.Average() << L"\n"
                    L"slow_average_command=" << slowControl.Average() << L"\n"
                    L"criterion=actual > slow_average_command";
                Assert::IsTrue(movingControl.Average() > slowControl.Average(), message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity)
        {
            constexpr float requestedAccelMps2 = 4.0f;
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);

            runtimeState.SetForwardVelocity(0.20f);
            runtimeState.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(runtimeState.GetForwardVelocity()));
            runtimeState.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(runtimeState.GetForwardVelocity()));
            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(requestedAccelMps2, 0.0f);
            const float initialVelocityMps = runtimeState.GetForwardVelocity();
            plant.integrate(command, dtSeconds);
            const float integratedForwardAccelMps2 =
                IntegratedRateOfChange(initialVelocityMps, runtimeState.GetForwardVelocity(), dtSeconds);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity\n"
                    L"field=feedforward_command\n"
                    L"actual={left_command=" << command.LeftCommand() <<
                    L", right_command=" << command.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(command), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity\n"
                    L"field=longitudinal_acceleration_mps2\n"
                    L"actual=" << runtimeState.GetForwardAcceleration() << L"\n"
                    L"criterion=actual > 0";
                Assert::IsTrue(runtimeState.GetForwardAcceleration() > 0.0f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity\n"
                    L"field=runtime_longitudinal_acceleration_mps2\n"
                    L"actual=" << runtimeState.GetForwardAcceleration() << L"\n"
                    L"expected=" << requestedAccelMps2 << L"\n"
                    L"tolerance=" << kAccelerationToleranceMps2 << L"\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(
                    requestedAccelMps2,
                    runtimeState.GetForwardAcceleration(),
                    kAccelerationToleranceMps2,
                    message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity\n"
                    L"field=integrated_forward_acceleration_mps2\n"
                    L"actual=" << integratedForwardAccelMps2 << L"\n"
                    L"expected=" << requestedAccelMps2 << L"\n"
                    L"tolerance=" << kAccelerationToleranceMps2 << L"\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(
                    requestedAccelMps2,
                    integratedForwardAccelMps2,
                    kAccelerationToleranceMps2,
                    message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate)
        {
            constexpr float requestedYawAccelRadps2 = 25.0f;
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, requestedYawAccelRadps2);
            const float initialYawRateRadps = runtimeState.GetYawRate();
            plant.integrate(command, dtSeconds);
            const float integratedYawAccelRadps2 =
                IntegratedRateOfChange(initialYawRateRadps, runtimeState.GetYawRate(), dtSeconds);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate\n"
                    L"field=feedforward_command\n"
                    L"actual={left_command=" << command.LeftCommand() <<
                    L", right_command=" << command.RightCommand() << L"}\n"
                    L"criterion=isfinite(left_command) && isfinite(right_command)";
                Assert::IsTrue(IsFiniteControlVector(command), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate\n"
                    L"field=yaw_acceleration_radps2\n"
                    L"actual=" << runtimeState.GetYawAccel() << L"\n"
                    L"criterion=actual > 0";
                Assert::IsTrue(runtimeState.GetYawAccel() > 0.0f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate\n"
                    L"field=runtime_yaw_acceleration_radps2\n"
                    L"actual=" << runtimeState.GetYawAccel() << L"\n"
                    L"expected=" << requestedYawAccelRadps2 << L"\n"
                    L"tolerance=" << kYawAccelerationToleranceRadps2 << L"\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(
                    requestedYawAccelRadps2,
                    runtimeState.GetYawAccel(),
                    kYawAccelerationToleranceRadps2,
                    message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate\n"
                    L"field=integrated_yaw_acceleration_radps2\n"
                    L"actual=" << integratedYawAccelRadps2 << L"\n"
                    L"expected=" << requestedYawAccelRadps2 << L"\n"
                    L"tolerance=" << kYawAccelerationToleranceRadps2 << L"\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(
                    requestedYawAccelRadps2,
                    integratedYawAccelRadps2,
                    kYawAccelerationToleranceRadps2,
                    message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelWheelProjectionRoundTripsVehicleBodyVelocity)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            constexpr float forwardMps = 1.0f;
            constexpr float yawRateRadps = 2.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            vehicle.WheelSpeedsFromBodyVelocity(forwardMps, yawRateRadps, leftWheelSpeedRadps, rightWheelSpeedRadps);
            EncoderObs observation{};
            observation.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            observation.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const float measuredLinearSpeedMps = plant.measuredLinearSpeedMps(observation);
            const float measuredYawRateRadps = plant.measuredYawRateRadps(observation);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelWheelProjectionRoundTripsVehicleBodyVelocity\n"
                    L"field=measured_linear_speed_mps\n"
                    L"actual=" << measuredLinearSpeedMps << L"\n"
                    L"expected=" << forwardMps << L"\n"
                    L"tolerance=1e-6\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(forwardMps, measuredLinearSpeedMps, 1.0e-6f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelWheelProjectionRoundTripsVehicleBodyVelocity\n"
                    L"field=measured_yaw_rate_radps\n"
                    L"actual=" << measuredYawRateRadps << L"\n"
                    L"expected=" << yawRateRadps << L"\n"
                    L"tolerance=1e-6\n"
                    L"criterion=abs(actual - expected) <= tolerance";
                Assert::AreEqual(yawRateRadps, measuredYawRateRadps, 1.0e-6f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelWheelProjectionRoundTripsVehicleBodyVelocity\n"
                    L"field=left_wheel_speed_radps\n"
                    L"actual=" << leftWheelSpeedRadps << L"\n"
                    L"right_wheel_speed_radps=" << rightWheelSpeedRadps << L"\n"
                    L"criterion=actual > right_wheel_speed_radps";
                Assert::IsTrue(leftWheelSpeedRadps > rightWheelSpeedRadps, message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;

            plant.velocityTargetTechnicalLimits(
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);

            {
                std::wstringstream message;
                message <<
                    L"PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope\n"
                    L"field=max_longitudinal_accel_mps2\n"
                    L"actual=" << maxLongitudinalAccelMps2 << L"\n"
                    L"criterion=isfinite(actual)";
                Assert::IsTrue(std::isfinite(maxLongitudinalAccelMps2), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope\n"
                    L"field=max_yaw_accel_radps2\n"
                    L"actual=" << maxYawAccelRadps2 << L"\n"
                    L"criterion=isfinite(actual)";
                Assert::IsTrue(std::isfinite(maxYawAccelRadps2), message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope\n"
                    L"field=max_longitudinal_accel_mps2\n"
                    L"actual=" << maxLongitudinalAccelMps2 << L"\n"
                    L"criterion=actual > 0";
                Assert::IsTrue(maxLongitudinalAccelMps2 > 0.0f, message.str().c_str());
            }

            {
                std::wstringstream message;
                message <<
                    L"PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope\n"
                    L"field=max_yaw_accel_radps2\n"
                    L"actual=" << maxYawAccelRadps2 << L"\n"
                    L"criterion=actual > 0";
                Assert::IsTrue(maxYawAccelRadps2 > 0.0f, message.str().c_str());
            }
        }

        TEST_METHOD(PlantModelFeedforwardResolvesInfiniteAccelerationObjectivesThroughTechnicalLimits)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(maxLongitudinalAccelMps2, maxYawAccelRadps2);

            const auto forwardPositive = plant.ComputeFeedforward(kInf, 0.0f);
            const auto forwardExpected = plant.ComputeFeedforward(maxLongitudinalAccelMps2, 0.0f);
            Assert::AreEqual(forwardExpected.LeftCommand(), forwardPositive.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(forwardExpected.RightCommand(), forwardPositive.RightCommand(), 1.0e-6f);

            const auto forwardNegative = plant.ComputeFeedforward(-kInf, 0.0f);
            const auto forwardNegativeExpected = plant.ComputeFeedforward(-maxLongitudinalAccelMps2, 0.0f);
            Assert::AreEqual(forwardNegativeExpected.LeftCommand(), forwardNegative.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(forwardNegativeExpected.RightCommand(), forwardNegative.RightCommand(), 1.0e-6f);

            const auto yawNegative = plant.ComputeFeedforward(0.0f, -kInf);
            const auto yawNegativeExpected = plant.ComputeFeedforward(0.0f, -maxYawAccelRadps2);
            Assert::AreEqual(yawNegativeExpected.LeftCommand(), yawNegative.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(yawNegativeExpected.RightCommand(), yawNegative.RightCommand(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelFeedforwardUsesNeutralComponentForInactiveAccelerationObjective)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);

            const auto yawOnly = plant.ComputeFeedforward(kNaN, 8.0f);
            const auto yawOnlyExpected = plant.ComputeFeedforward(0.0f, 8.0f);
            Assert::AreEqual(yawOnlyExpected.LeftCommand(), yawOnly.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(yawOnlyExpected.RightCommand(), yawOnly.RightCommand(), 1.0e-6f);

            const auto forwardOnly = plant.ComputeFeedforward(1.0f, kNaN);
            const auto forwardOnlyExpected = plant.ComputeFeedforward(1.0f, 0.0f);
            Assert::AreEqual(forwardOnlyExpected.LeftCommand(), forwardOnly.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(forwardOnlyExpected.RightCommand(), forwardOnly.RightCommand(), 1.0e-6f);

            const auto noObjective = plant.ComputeFeedforward(kNaN, kNaN);
            const auto noObjectiveExpected = plant.ComputeFeedforward(0.0f, 0.0f);
            Assert::AreEqual(noObjectiveExpected.LeftCommand(), noObjective.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(noObjectiveExpected.RightCommand(), noObjective.RightCommand(), 1.0e-6f);
        }

    };
}
