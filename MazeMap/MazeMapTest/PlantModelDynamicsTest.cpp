#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

        bool Near(const float expected, const float actual, const float tolerance) noexcept
        {
            return std::fabs(expected - actual) <= tolerance;
        }

        void AppendMotionState(std::wstringstream& message, const VehicleState& state)
        {
            message
                << L"\nx_m=" << state.GetPositionX()
                << L"\ny_m=" << state.GetPositionY()
                << L"\nyaw_rad=" << state.GetHeading()
                << L"\nforward_velocity_mps=" << state.GetForwardVelocity()
                << L"\nlateral_velocity_mps=" << state.GetRightwardVelocity()
                << L"\nyaw_rate_radps=" << state.GetYawRate()
                << L"\nleft_wheel_speed_radps=" << state.GetWheelSpeedLeft()
                << L"\nright_wheel_speed_radps=" << state.GetWheelSpeedRight()
                << L"\ngyro_bias_z_radps=" << state.GetGyroBiasZ();
        }

        float MaxPreProjectionUtilizationForBank(
            const float leftWheelSpeedRadps,
            const float rightWheelSpeedRadps,
            const uint8_t firstContactIndex,
            const uint8_t secondContactIndex)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.05f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(leftWheelSpeedRadps);
            state.SetWheelSpeedRight(rightWheelSpeedRadps);
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            return (std::max)(
                plant.contactPreProjectionUtilization(control, firstContactIndex),
                plant.contactPreProjectionUtilization(control, secondContactIndex));
        }

        float MinimumSaturationForSymmetricWheelSpin()
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
            return (std::min)(
                (std::min)(
                    plant.contactSaturation(control, 0U),
                    plant.contactSaturation(control, 1U)),
                (std::min)(
                    plant.contactSaturation(control, 2U),
                    plant.contactSaturation(control, 3U)));
        }

        float MaximumSaturationForSymmetricWheelSpin()
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
            return (std::max)(
                (std::max)(
                    plant.contactSaturation(control, 0U),
                    plant.contactSaturation(control, 1U)),
                (std::max)(
                    plant.contactSaturation(control, 2U),
                    plant.contactSaturation(control, 3U)));
        }

        VehicleState IntegrateDifferentialWheelSpin()
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
            return state;
        }

        struct SymmetricWheelSpinStep final
        {
            VehicleState state;
            float initialLeftAbsRadps;
            float initialRightAbsRadps;
            float finalLeftAbsRadps;
            float finalRightAbsRadps;
        };

        SymmetricWheelSpinStep IntegrateSymmetricWheelSpin()
        {
            Vehicle vehicle;
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
            const float initialLeftAbsRadps = std::fabs(state.GetWheelSpeedLeft());
            const float initialRightAbsRadps = std::fabs(state.GetWheelSpeedRight());
            plant.integrate(control, 0.001f);

            return SymmetricWheelSpinStep{
                state,
                initialLeftAbsRadps,
                initialRightAbsRadps,
                std::fabs(state.GetWheelSpeedLeft()),
                std::fabs(state.GetWheelSpeedRight()) };
        }

        struct FeedforwardTimingMeasurement final
        {
            std::chrono::high_resolution_clock::duration feedforwardDuration;
            std::chrono::high_resolution_clock::duration integrateDuration;
            float accumulator;
        };

        FeedforwardTimingMeasurement MeasureFeedforwardTiming()
        {
            constexpr uint32_t numFeedforward = 100000;
            constexpr uint32_t numIntegrate = 75000;

            Vehicle vehicle;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(1.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetGyroBiasZ(0.0f);
            PlantModel plant(vehicle, state);
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            float feedforwardAccumulator = 0.0f;

            auto startTime = std::chrono::high_resolution_clock::now();
            for (uint32_t tick = 0; tick < numIntegrate; ++tick)
            {
                plant.integrate(control, dtSeconds);
            }
            const auto integrateDuration =
                std::chrono::high_resolution_clock::now() - startTime;

            startTime = std::chrono::high_resolution_clock::now();
            for (uint32_t tick = 0; tick < numFeedforward; ++tick)
            {
                const App::Internal::CommandVector command =
                    plant.ComputeFeedforward(0.0f, 0.0f);
                feedforwardAccumulator += command.LeftCommand() + command.RightCommand();
            }
            const auto feedforwardDuration =
                std::chrono::high_resolution_clock::now() - startTime;

            return FeedforwardTimingMeasurement{
                feedforwardDuration,
                integrateDuration,
                feedforwardAccumulator };
        }

        float PeakLateralAccelerationAcrossTicks(const float initialRightwardVelocityMps)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(1.0f);
            state.SetRightwardVelocity(initialRightwardVelocityMps);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(1.0f));
            state.SetGyroBiasZ(0.0f);
            PlantModel plant(vehicle, state);

            float peakAccelerationMps2 = 0.0f;
            for (int tick = 0; tick < 5; ++tick)
            {
                const float beforeLateralVelocityMps = state.GetRightwardVelocity();
                plant.integrate(control, dtSeconds);
                const float afterLateralVelocityMps = state.GetRightwardVelocity();
                peakAccelerationMps2 =
                    (std::max)(
                        peakAccelerationMps2,
                        std::fabs(
                            (afterLateralVelocityMps - beforeLateralVelocityMps) /
                            dtSeconds));
            }

            return peakAccelerationMps2;
        }

        struct InPlaceSlipYawDecelMeasurement final
        {
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> rightForceN;
            float contactY;
            float yawRateRadps;
            float frontContactLimitN;
            float rearContactLimitN;
            float initialYawRateRadps;
            float actualYawRateRadps;
            float expectedYawAccelRadps2;
            float observedYawAccelRadps2;
        };

        InPlaceSlipYawDecelMeasurement MeasureInPlaceSlipYawDecel()
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY =
                std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());

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
            state.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * yawRateRadps));
            state.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * yawRateRadps));
            PlantModel plant(vehicle, state);

            const float totalSustainedLateralForceN =
                vehicle.GetMass() *
                Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) *
                totalSustainedLateralForceN;

            InPlaceSlipYawDecelMeasurement measurement{
                {},
                {},
                {},
                contactY,
                yawRateRadps,
                frontContactLimitN,
                rearContactLimitN,
                state.GetYawRate(),
                0.0f,
                0.0f,
                0.0f };

            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                measurement.forwardRelativeVelocityMps[contactIndex] =
                    plant.contactForwardRelativeVelocityMps(contactIndex);
                measurement.rightRelativeVelocityMps[contactIndex] =
                    plant.contactRightRelativeVelocityMps(contactIndex);
                measurement.rightForceN[contactIndex] =
                    plant.contactRightForceN(control, contactIndex);
            }

            plant.integrate(control, 0.001f);
            const float saturatedYawDecelRadps2 =
                (2.0f * contactY * (frontContactLimitN + rearContactLimitN)) /
                vehicle.GetYawInertia();
            measurement.actualYawRateRadps = state.GetYawRate();
            measurement.expectedYawAccelRadps2 = -saturatedYawDecelRadps2;
            measurement.observedYawAccelRadps2 = state.GetYawAccel();

            return measurement;
        }

        float InPlaceSlipExpectedRightRelativeVelocity(
            const InPlaceSlipYawDecelMeasurement& measurement,
            const uint8_t contactIndex)
        {
            return (contactIndex < 2U ? -1.0f : 1.0f) *
                measurement.contactY *
                measurement.yawRateRadps;
        }

        float InPlaceSlipExpectedRightForce(
            const InPlaceSlipYawDecelMeasurement& measurement,
            const uint8_t contactIndex)
        {
            return contactIndex < 2U ?
                -measurement.frontContactLimitN :
                measurement.rearContactLimitN;
        }

        float YawAccelerationAtForwardSpeed(const float forwardSpeedMps)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            constexpr float yawRateRadps = 3.0f;

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

            const App::Internal::CommandVector control{};
            plant.integrate(control, 0.001f);
            return state.GetYawAccel();
        }

        struct LowSpeedYawAccelerationMeasurement final
        {
            float belowYawAccelRadps2;
            float centerYawAccelRadps2;
            float aboveYawAccelRadps2;
            float maxNeighborDeltaRadps2;
            float maxAllowedNeighborDeltaRadps2;
        };

        LowSpeedYawAccelerationMeasurement MeasureLowSpeedYawAcceleration()
        {
            constexpr float referenceForwardSpeedMps = 0.12f;
            constexpr float nearbyDeltaMps = 0.001f;
            const float belowYawAccelRadps2 =
                YawAccelerationAtForwardSpeed(
                    referenceForwardSpeedMps - nearbyDeltaMps);
            const float centerYawAccelRadps2 =
                YawAccelerationAtForwardSpeed(referenceForwardSpeedMps);
            const float aboveYawAccelRadps2 =
                YawAccelerationAtForwardSpeed(
                    referenceForwardSpeedMps + nearbyDeltaMps);
            const float maxNeighborDeltaRadps2 =
                (std::max)(
                    std::fabs(centerYawAccelRadps2 - belowYawAccelRadps2),
                    std::fabs(aboveYawAccelRadps2 - centerYawAccelRadps2));
            const float localScaleRadps2 =
                (std::max)(std::fabs(centerYawAccelRadps2), 1.0f);

            return LowSpeedYawAccelerationMeasurement{
                belowYawAccelRadps2,
                centerYawAccelRadps2,
                aboveYawAccelRadps2,
                maxNeighborDeltaRadps2,
                (0.05f * localScaleRadps2) + 0.10f };
        }

        struct ContactForwardForceCoupleMeasurement final
        {
            float frontLeftForwardForceN;
            float frontRightForwardForceN;
            float rearLeftForwardForceN;
            float rearRightForwardForceN;
            float totalForwardForceN;
        };

        ContactForwardForceCoupleMeasurement MeasureContactForwardForceCouple()
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
            state.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(halfTrackM * yawRateRadps));
            state.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(-halfTrackM * yawRateRadps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            const float frontLeftForwardForceN =
                plant.contactForwardForceN(control, 0U);
            const float frontRightForwardForceN =
                plant.contactForwardForceN(control, 1U);
            const float rearLeftForwardForceN =
                plant.contactForwardForceN(control, 2U);
            const float rearRightForwardForceN =
                plant.contactForwardForceN(control, 3U);

            return ContactForwardForceCoupleMeasurement{
                frontLeftForwardForceN,
                frontRightForwardForceN,
                rearLeftForwardForceN,
                rearRightForwardForceN,
                frontLeftForwardForceN +
                    frontRightForwardForceN +
                    rearLeftForwardForceN +
                    rearRightForwardForceN };
        }

        struct ContactContinuumFiniteSample final
        {
            float forwardSpeedMps;
            float rightSpeedMps;
            float yawRateRadps;
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> forwardForceN;
            std::array<float, 4> rightForceN;
            std::array<float, 4> preProjectionUtilization;
            std::array<float, 4> saturation;
            float stateForwardVelocityMps;
            float stateRightwardVelocityMps;
            float stateYawRateRadps;
            float stateYawAccelRadps2;
        };

        ContactContinuumFiniteSample MeasureContactContinuumFiniteSample(
            const float forwardSpeedMps,
            const float rightSpeedMps,
            const float yawRateRadps)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();

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

            ContactContinuumFiniteSample sample{
                forwardSpeedMps,
                rightSpeedMps,
                yawRateRadps,
                {},
                {},
                {},
                {},
                {},
                {},
                0.0f,
                0.0f,
                0.0f,
                0.0f };

            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                sample.forwardRelativeVelocityMps[contactIndex] =
                    plant.contactForwardRelativeVelocityMps(contactIndex);
                sample.rightRelativeVelocityMps[contactIndex] =
                    plant.contactRightRelativeVelocityMps(contactIndex);
                sample.forwardForceN[contactIndex] =
                    plant.contactForwardForceN(control, contactIndex);
                sample.rightForceN[contactIndex] =
                    plant.contactRightForceN(control, contactIndex);
                sample.preProjectionUtilization[contactIndex] =
                    plant.contactPreProjectionUtilization(control, contactIndex);
                sample.saturation[contactIndex] =
                    plant.contactSaturation(control, contactIndex);
            }

            plant.integrate(control, 0.001f);
            sample.stateForwardVelocityMps = state.GetForwardVelocity();
            sample.stateRightwardVelocityMps = state.GetRightwardVelocity();
            sample.stateYawRateRadps = state.GetYawRate();
            sample.stateYawAccelRadps2 = state.GetYawAccel();

            return sample;
        }
        struct ContactContinuumYawAccelerationMeasurement final
        {
            std::array<float, 3> yawAccelerationsRadps2;
            float maxNeighborDeltaRadps2;
        };

        ContactContinuumYawAccelerationMeasurement
            MeasureContactContinuumYawAcceleration()
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            constexpr float yawRateRadps = 1.0f;
            constexpr float forwardDeltaMps = 0.0001f;
            const std::array<float, 3> forwardSpeedsMps{
                -forwardDeltaMps,
                0.0f,
                forwardDeltaMps };
            std::array<float, 3> yawAccelerationsRadps2{};

            for (std::size_t sample = 0U; sample < forwardSpeedsMps.size(); ++sample)
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
                plant.integrate(App::Internal::CommandVector{}, 0.001f);
                yawAccelerationsRadps2[sample] = state.GetYawAccel();
            }

            return ContactContinuumYawAccelerationMeasurement{
                yawAccelerationsRadps2,
                (std::max)(
                    std::fabs(yawAccelerationsRadps2[1] - yawAccelerationsRadps2[0]),
                    std::fabs(yawAccelerationsRadps2[2] - yawAccelerationsRadps2[1])) };
        }

        struct InPlaceSlipSpinDownMeasurement final
        {
            float initialYawRateRadps;
            float maxYawRateAbsRadps;
            float initialMagnitudeToleranceRadps;
            float maxReboundRadps;
            float maxAllowedReboundRadps;
            int firstMonotonicFailureStep;
            float firstMonotonicFailurePreviousRadps;
            float firstMonotonicFailureActualRadps;
            int stopStep;
            float stopTimeS;
            float maxAllowedStopTimeS;
            float dtSeconds;
        };

        InPlaceSlipSpinDownMeasurement MeasureInPlaceSlipSpinDown()
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const float halfTrackM = 0.5f * vehicle.GetTrackWidth();
            const float contactY =
                std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float dtSeconds = 0.001f;
            constexpr float initialYawRateRadps = 30.0f;
            const float totalSustainedLateralForceN =
                vehicle.GetMass() *
                Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float frontContactLimitN =
                0.5f * kSymmetricFrontLoadFraction * totalSustainedLateralForceN;
            const float rearContactLimitN =
                0.5f * (1.0f - kSymmetricFrontLoadFraction) *
                totalSustainedLateralForceN;
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
            state.SetWheelSpeedLeft(
                Vehicle::WheelSpeedFromLinearVelocity(
                    halfTrackM * initialYawRateRadps));
            state.SetWheelSpeedRight(
                Vehicle::WheelSpeedFromLinearVelocity(
                    -halfTrackM * initialYawRateRadps));
            PlantModel plant(vehicle, state);

            float previousYawRateAbsRadps = std::fabs(state.GetYawRate());
            float minYawRateAfterInitialDecayAbsRadps = previousYawRateAbsRadps;
            float maxYawRateAbsRadps = previousYawRateAbsRadps;
            float maxReboundRadps = 0.0f;
            int stopStep = -1;
            int firstMonotonicFailureStep = -1;
            float firstMonotonicFailurePreviousRadps = 0.0f;
            float firstMonotonicFailureActualRadps = 0.0f;
            for (int step = 0; step < maxSteps; ++step)
            {
                plant.integrate(control, dtSeconds);
                const float yawRateAbsRadps = std::fabs(state.GetYawRate());

                if (firstMonotonicFailureStep < 0 &&
                    yawRateAbsRadps > previousYawRateAbsRadps + 1.0e-4f)
                {
                    firstMonotonicFailureStep = step + 1;
                    firstMonotonicFailurePreviousRadps = previousYawRateAbsRadps;
                    firstMonotonicFailureActualRadps = yawRateAbsRadps;
                }
                maxYawRateAbsRadps = (std::max)(maxYawRateAbsRadps, yawRateAbsRadps);
                if (yawRateAbsRadps < minYawRateAfterInitialDecayAbsRadps)
                {
                    minYawRateAfterInitialDecayAbsRadps = yawRateAbsRadps;
                }
                else if (yawRateAbsRadps > previousYawRateAbsRadps)
                {
                    maxReboundRadps =
                        (std::max)(
                            maxReboundRadps,
                            yawRateAbsRadps - minYawRateAfterInitialDecayAbsRadps);
                }

                previousYawRateAbsRadps = yawRateAbsRadps;
                if (yawRateAbsRadps <= kStopEnterYawRateRadps)
                {
                    stopStep = step + 1;
                    break;
                }
            }

            return InPlaceSlipSpinDownMeasurement{
                initialYawRateRadps,
                maxYawRateAbsRadps,
                1.0e-4f,
                maxReboundRadps,
                kStopEnterYawRateRadps,
                firstMonotonicFailureStep,
                firstMonotonicFailurePreviousRadps,
                firstMonotonicFailureActualRadps,
                stopStep,
                stopStep > 0 ?
                    static_cast<float>(stopStep) * dtSeconds :
                    std::numeric_limits<float>::infinity(),
                maxAllowedStopTimeS,
                dtSeconds };
        }

        VehicleState IntegrateExactRestHold()
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
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

            const App::Internal::CommandVector control{};
            for (int step = 0; step < 1000; ++step)
            {
                plant.integrate(control, 0.001f);
            }

            return state;
        }

        VehicleState IntegrateSmallStationaryPerturbation()
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(0.0f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.05f);
            state.SetWheelSpeedLeft(0.8f);
            state.SetWheelSpeedRight(-0.7f);
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector control{};
            for (int step = 0; step < 250; ++step)
            {
                plant.integrate(control, 0.001f);
            }

            return state;
        }

        struct NearZeroLateralPerturbationMeasurement final
        {
            VehicleState initial;
            VehicleState final;
        };

        NearZeroLateralPerturbationMeasurement
            IntegrateNearZeroLateralPerturbation()
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

            const App::Internal::CommandVector control{};
            for (int step = 0; step < 25; ++step)
            {
                plant.integrate(control, 0.001f);
            }

            return NearZeroLateralPerturbationMeasurement{ initialState, state };
        }

        struct ResidualDecayMeasurement final
        {
            float expectedForwardResidual;
            float actualForwardResidual;
            float expectedRightResidual;
            float actualRightResidual;
            float expectedYawResidual;
            float actualYawResidual;
        };

        ResidualDecayMeasurement MeasureResidualDecay()
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

            constexpr float dtSeconds = 0.010f;
            const float decay = std::exp(-dtSeconds / kPlantResidualDecayTauS);
            plant.integrate(App::Internal::CommandVector{}, dtSeconds);

            return ResidualDecayMeasurement{
                0.75f * decay,
                state.GetForwardAccelerationResidual(),
                -1.25f * decay,
                state.GetRightwardAccelerationResidual(),
                2.50f * decay,
                state.GetYawAccelResidual() };
        }

        struct ImuAccelerationMeasurement final
        {
            float rightLeverContributionMps2;
            float forwardLeverContributionMps2;
            float expectedRightAccelerationMps2;
            float predictedRightAccelerationMps2;
            float expectedForwardAccelerationMps2;
            float predictedForwardAccelerationMps2;
        };

        ImuAccelerationMeasurement MeasureImuAcceleration(
            const float initialYawRateRadps,
            const float forwardVelocityMps,
            const float rightwardVelocityMps,
            const float leftWheelScale,
            const float rightWheelScale,
            const float leftCommand,
            const float rightCommand)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(rightwardVelocityMps);
            state.SetYawRate(initialYawRateRadps);
            state.SetWheelSpeedLeft(
                leftWheelScale *
                Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(
                rightWheelScale *
                Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(leftCommand);
            control.SetRightCommand(rightCommand);

            const float predictedRightAccelerationMps2 =
                plant.backLeftImuRightAccelerationMps2(control);
            const float predictedForwardAccelerationMps2 =
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

            return ImuAccelerationMeasurement{
                predictedRightAccelerationMps2 - state.GetRightAcceleration(),
                predictedForwardAccelerationMps2 - state.GetForwardAcceleration(),
                expectedRightAccelerationMps2,
                predictedRightAccelerationMps2,
                expectedForwardAccelerationMps2,
                predictedForwardAccelerationMps2 };
        }

        struct PositiveDriveFromRestMeasurement final
        {
            VehicleState state;
            float averageAccelMps2;
        };

        PositiveDriveFromRestMeasurement IntegratePositiveDriveFromRest()
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

            return PositiveDriveFromRestMeasurement{
                state,
                state.GetForwardVelocity() / (dt * static_cast<float>(kSteps)) };
        }

        struct StaticFrictionRestMeasurement final
        {
            VehicleState state;
            float expectedStaticFrictionTorqueNm;
            float positiveStaticFrictionTorqueNm;
            float negativeStaticFrictionTorqueNm;
            float expectedRollingFrictionTorqueNm;
            float positiveRollingFrictionTorqueNm;
            float negativeRollingFrictionTorqueNm;
        };

        StaticFrictionRestMeasurement MeasureStaticFrictionRest()
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
            const float staticWindowRadps =
                plant.staticFrictionSpeedThresholdRadps();

            const float positiveStaticFrictionTorqueNm =
                plant.driveFrictionTorque(0.5f * staticWindowRadps, 1.0f);
            const float negativeStaticFrictionTorqueNm =
                plant.driveFrictionTorque(-0.5f * staticWindowRadps, -1.0f);
            const float positiveRollingFrictionTorqueNm =
                plant.driveFrictionTorque(1.1f * staticWindowRadps, 1.0f);
            const float negativeRollingFrictionTorqueNm =
                plant.driveFrictionTorque(-1.1f * staticWindowRadps, -1.0f);

            for (int step = 0; step < 100; ++step)
            {
                plant.integrate(control, 0.001f);
            }

            return StaticFrictionRestMeasurement{
                state,
                plant.staticFrictionTorqueNm(),
                positiveStaticFrictionTorqueNm,
                negativeStaticFrictionTorqueNm,
                plant.rollingFrictionTorqueNm(),
                positiveRollingFrictionTorqueNm,
                negativeRollingFrictionTorqueNm };
        }

        struct LargeStepMeasurement final
        {
            VehicleState state;
            float wheelSpeedDeltaRadps;
        };

        LargeStepMeasurement IntegrateSingleLargeStep()
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

            plant.integrate(control, 0.004f);
            return LargeStepMeasurement{
                state,
                state.GetWheelSpeedLeft() - state.GetWheelSpeedRight() };
        }
        struct MixedSlipMeasurement final
        {
            float leftLongitudinalSlipMps;
            float rightLongitudinalSlipMps;
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> rightForceN;
            std::array<float, 4> forwardForceN;
            std::array<float, 4> saturation;
            std::array<float, 4> preProjectionUtilization;
            VehicleState state;
        };

        MixedSlipMeasurement MeasureMixedSlipCommand()
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
            state.SetWheelSpeedLeft(
                1.05f * Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(
                0.95f * Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.65f);
            control.SetRightCommand(0.60f);

            MixedSlipMeasurement measurement{
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft()) -
                    Vehicle::LeftWheelLinearVelocityFromBody(
                        state.GetForwardVelocity(),
                        state.GetYawRate()),
                Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight()) -
                    Vehicle::RightWheelLinearVelocityFromBody(
                        state.GetForwardVelocity(),
                        state.GetYawRate()),
                {},
                {},
                {},
                {},
                {},
                {},
                state };

            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                measurement.forwardRelativeVelocityMps[contactIndex] =
                    plant.contactForwardRelativeVelocityMps(contactIndex);
                measurement.rightRelativeVelocityMps[contactIndex] =
                    plant.contactRightRelativeVelocityMps(contactIndex);
                measurement.rightForceN[contactIndex] =
                    plant.contactRightForceN(control, contactIndex);
                measurement.forwardForceN[contactIndex] =
                    plant.contactForwardForceN(control, contactIndex);
                measurement.saturation[contactIndex] =
                    plant.contactSaturation(control, contactIndex);
                measurement.preProjectionUtilization[contactIndex] =
                    plant.contactPreProjectionUtilization(control, contactIndex);
            }

            plant.integrate(control, 0.001f);
            measurement.state = state;
            return measurement;
        }
    }

    TEST_CLASS(PlantModelDriveBasicsTest)
    {
    public:
        TEST_METHOD(LeftDriveWheelInertiaUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.leftDriveEquivalentWheelInertiaKgM2();
            std::wstringstream message;
            message << L"LeftDriveWheelInertiaUsesConstructionValue"
                << L"\nexpected=1.177e-6"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-10";

            Assert::AreEqual(
                1.177e-6f,
                actual,
                1.0e-10f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveWheelInertiaUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.rightDriveEquivalentWheelInertiaKgM2();
            std::wstringstream message;
            message << L"RightDriveWheelInertiaUsesConstructionValue"
                << L"\nexpected=1.177e-6"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-10";

            Assert::AreEqual(
                1.177e-6f,
                actual,
                1.0e-10f,
                message.str().c_str());
        }

        TEST_METHOD(LeftDriveLongitudinalTireStiffnessUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.leftDriveLongitudinalTireStiffnessN();
            std::wstringstream message;
            message << L"LeftDriveLongitudinalTireStiffnessUsesConstructionValue"
                << L"\nexpected=4.12"
                << L"\nactual=" << actual
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                4.12f,
                actual,
                0.01f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveLongitudinalTireStiffnessUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.rightDriveLongitudinalTireStiffnessN();
            std::wstringstream message;
            message << L"RightDriveLongitudinalTireStiffnessUsesConstructionValue"
                << L"\nexpected=4.12"
                << L"\nactual=" << actual
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                4.12f,
                actual,
                0.01f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricDrivePreservesYawRate)
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
            const float actualYawRateRadps = state.GetYawRate();
            std::wstringstream message;
            message << L"SymmetricDrivePreservesYawRate"
                << L"\ninitial_yaw_rate_radps=" << initialYawRateRadps
                << L"\nactual_yaw_rate_radps=" << actualYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initialYawRateRadps,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(BrakeCommandAddsElectricalRetardingForceAtLowForwardSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 0.09f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector coastCommand{};
            const App::Internal::CommandVector brakeCommand = App::Internal::CommandVector::Brake();
            const float coastForceN = plant.totalForwardContactForceN(coastCommand);
            const float brakeForceN = plant.totalForwardContactForceN(brakeCommand);
            const float addedRetardingForceN = coastForceN - brakeForceN;

            std::wstringstream message;
            message << L"BrakeCommandAddsElectricalRetardingForceAtLowForwardSpeed"
                << L"\ncoast_force_n=" << coastForceN
                << L"\nbrake_force_n=" << brakeForceN
                << L"\nadded_retarding_force_n=" << addedRetardingForceN
                << L"\ncriterion=added_retarding_force_n>0.02";

            Assert::IsTrue(
                std::isfinite(brakeForceN) && (addedRetardingForceN > 0.02f),
                message.str().c_str());
        }

        TEST_METHOD(BrakeCommandAddsElectricalRetardingForceAtLowReverseSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = -0.09f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector coastCommand{};
            const App::Internal::CommandVector brakeCommand = App::Internal::CommandVector::Brake();
            const float coastForceN = plant.totalForwardContactForceN(coastCommand);
            const float brakeForceN = plant.totalForwardContactForceN(brakeCommand);
            const float addedRetardingForceN = brakeForceN - coastForceN;

            std::wstringstream message;
            message << L"BrakeCommandAddsElectricalRetardingForceAtLowReverseSpeed"
                << L"\ncoast_force_n=" << coastForceN
                << L"\nbrake_force_n=" << brakeForceN
                << L"\nadded_retarding_force_n=" << addedRetardingForceN
                << L"\ncriterion=added_retarding_force_n>0.02";

            Assert::IsTrue(
                std::isfinite(brakeForceN) && (addedRetardingForceN > 0.02f),
                message.str().c_str());
        }

    };

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

    TEST_CLASS(PlantModelWheelSpinStateTest)
    {
    public:
        TEST_METHOD(DifferentialWheelSpinPositionXStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinPositionXStaysFinite"
                << L"\nactual=" << state.GetPositionX()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetPositionX()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinPositionYStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinPositionYStaysFinite"
                << L"\nactual=" << state.GetPositionY()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetPositionY()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinHeadingStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinHeadingStaysFinite"
                << L"\nactual=" << state.GetHeading()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetHeading()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinForwardVelocityStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinForwardVelocityStaysFinite"
                << L"\nactual=" << state.GetForwardVelocity()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetForwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinLateralVelocityStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinLateralVelocityStaysFinite"
                << L"\nactual=" << state.GetRightwardVelocity()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetRightwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinYawRateStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinYawRateStaysFinite"
                << L"\nactual=" << state.GetYawRate()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetYawRate()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinLeftWheelSpeedStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinLeftWheelSpeedStaysFinite"
                << L"\nactual=" << state.GetWheelSpeedLeft()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetWheelSpeedLeft()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinRightWheelSpeedStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinRightWheelSpeedStaysFinite"
                << L"\nactual=" << state.GetWheelSpeedRight()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetWheelSpeedRight()),
                message.str().c_str());
        }

        TEST_METHOD(DifferentialWheelSpinGyroBiasStaysFinite)
        {
            const VehicleState state = IntegrateDifferentialWheelSpin();
            std::wstringstream message;
            message << L"DifferentialWheelSpinGyroBiasStaysFinite"
                << L"\nactual=" << state.GetGyroBiasZ()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(state.GetGyroBiasZ()),
                message.str().c_str());
        }

    };

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

    TEST_CLASS(PlantModelFeedforwardTimingTest)
    {
    public:
        TEST_METHOD(FeedforwardAccumulatorStaysFinite)
        {
            const FeedforwardTimingMeasurement measurement = MeasureFeedforwardTiming();
            std::wstringstream message;
            message << L"FeedforwardAccumulatorStaysFinite"
                << L"\nfeedforward_accumulator=" << measurement.accumulator
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.accumulator),
                message.str().c_str());
        }

        TEST_METHOD(FeedforwardRunsFasterThanIntegrate)
        {
            const FeedforwardTimingMeasurement measurement = MeasureFeedforwardTiming();
            std::wstringstream message;
            message << L"FeedforwardRunsFasterThanIntegrate"
                << L"\nfeedforward_ticks=" << measurement.feedforwardDuration.count()
                << L"\nintegrate_ticks=" << measurement.integrateDuration.count()
                << L"\ncriterion=feedforward_ticks<integrate_ticks";

            Assert::IsTrue(
                measurement.feedforwardDuration < measurement.integrateDuration,
                message.str().c_str());
        }

    };

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

    TEST_CLASS(PlantModelInPlaceSlipContactTest)
    {
    public:
        TEST_METHOD(InPlaceSlipContact0ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact0ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[0]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[0],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact1ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[1]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[1],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact2ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[2]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[2],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact3ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[3]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[3],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact0RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 0U);
            std::wstringstream message;
            message << L"InPlaceSlipContact0RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[0]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[0],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 1U);
            std::wstringstream message;
            message << L"InPlaceSlipContact1RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[1]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[1],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 2U);
            std::wstringstream message;
            message << L"InPlaceSlipContact2RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[2]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[2],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 3U);
            std::wstringstream message;
            message << L"InPlaceSlipContact3RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[3]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[3],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact0RightForceMatchesFrontLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 0U);
            std::wstringstream message;
            message << L"InPlaceSlipContact0RightForceMatchesFrontLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[0]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[0],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1RightForceMatchesFrontLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 1U);
            std::wstringstream message;
            message << L"InPlaceSlipContact1RightForceMatchesFrontLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[1]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[1],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2RightForceMatchesRearLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 2U);
            std::wstringstream message;
            message << L"InPlaceSlipContact2RightForceMatchesRearLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[2]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[2],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3RightForceMatchesRearLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 3U);
            std::wstringstream message;
            message << L"InPlaceSlipContact3RightForceMatchesRearLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[3]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[3],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawAccelerationIsFinite)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawAccelerationIsFinite"
                << L"\nactual=" << measurement.observedYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.observedYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateDecreases)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateDecreases"
                << L"\ninitial=" << measurement.initialYawRateRadps
                << L"\nactual=" << measurement.actualYawRateRadps
                << L"\ncriterion=actual<initial";

            Assert::IsTrue(
                measurement.actualYawRateRadps < measurement.initialYawRateRadps,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawAccelerationMatchesSustainedWindow)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawAccelerationMatchesSustainedWindow"
                << L"\nexpected=" << measurement.expectedYawAccelRadps2
                << L"\nactual=" << measurement.observedYawAccelRadps2
                << L"\ntolerance=1e-3";

            Assert::AreEqual(
                measurement.expectedYawAccelRadps2,
                measurement.observedYawAccelRadps2,
                1.0e-3f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelLowSpeedYawTest)
    {
    public:
        TEST_METHOD(BelowLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"BelowLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.belowYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.belowYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(CenterLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"CenterLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.centerYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.centerYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(AboveLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"AboveLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.aboveYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.aboveYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(BelowLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"BelowLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.belowYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.belowYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(CenterLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"CenterLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.centerYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.centerYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(AboveLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"AboveLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.aboveYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.aboveYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(LowSpeedYawAccelerationNeighborDeltaIsBounded)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"LowSpeedYawAccelerationNeighborDeltaIsBounded"
                << L"\nactual=" << measurement.maxNeighborDeltaRadps2
                << L"\nlimit=" << measurement.maxAllowedNeighborDeltaRadps2
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxNeighborDeltaRadps2 <=
                    measurement.maxAllowedNeighborDeltaRadps2,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelContactContinuumForceTest)
    {
    public:
        TEST_METHOD(ContactContinuumFrontLeftForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontLeftForwardForceIsFinite"
                << L"\nactual=" << measurement.frontLeftForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.frontLeftForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontRightForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontRightForwardForceIsFinite"
                << L"\nactual=" << measurement.frontRightForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.frontRightForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearLeftForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearLeftForwardForceIsFinite"
                << L"\nactual=" << measurement.rearLeftForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.rearLeftForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearRightForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearRightForwardForceIsFinite"
                << L"\nactual=" << measurement.rearRightForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.rearRightForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontLeftForwardForceIsNegative)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontLeftForwardForceIsNegative"
                << L"\nactual=" << measurement.frontLeftForwardForceN
                << L"\nlimit=-1e-6"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.frontLeftForwardForceN < -1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearLeftForwardForceIsNegative)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearLeftForwardForceIsNegative"
                << L"\nactual=" << measurement.rearLeftForwardForceN
                << L"\nlimit=-1e-6"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.rearLeftForwardForceN < -1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontRightForwardForceIsPositive)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontRightForwardForceIsPositive"
                << L"\nactual=" << measurement.frontRightForwardForceN
                << L"\nlimit=1e-6"
                << L"\ncriterion=actual>limit";

            Assert::IsTrue(
                measurement.frontRightForwardForceN > 1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearRightForwardForceIsPositive)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearRightForwardForceIsPositive"
                << L"\nactual=" << measurement.rearRightForwardForceN
                << L"\nlimit=1e-6"
                << L"\ncriterion=actual>limit";

            Assert::IsTrue(
                measurement.rearRightForwardForceN > 1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumNetForwardForceIsNearZero)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumNetForwardForceIsNearZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.totalForwardForceN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                0.0f,
                measurement.totalForwardForceN,
                1.0e-5f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelContactContinuumBelowForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumBelowForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateYawAccelerationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_accel_radps2"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelContactContinuumZeroForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumZeroForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateYawAccelerationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_accel_radps2"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelContactContinuumAboveForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumAboveForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateYawAccelerationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_accel_radps2"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }
    };

    TEST_CLASS(PlantModelContactContinuumYawTest)
    {
    public:
        TEST_METHOD(ContactContinuumBelowZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumBelowZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[0]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[0]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[1]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[1]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumAboveZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[2]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[2]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumYawAccelerationNeighborDeltaIsBounded)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumYawAccelerationNeighborDeltaIsBounded"
                << L"\nactual=" << measurement.maxNeighborDeltaRadps2
                << L"\nlimit=1e-3"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.maxNeighborDeltaRadps2 < 1.0e-3f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelInPlaceSlipDecayTest)
    {
    public:
        TEST_METHOD(InPlaceSlipYawRateDoesNotIncreaseDuringDecay)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateDoesNotIncreaseDuringDecay"
                << L"\nexpected_first_failure_step=-1"
                << L"\nactual_first_failure_step="
                << measurement.firstMonotonicFailureStep
                << L"\nprevious_abs_radps="
                << measurement.firstMonotonicFailurePreviousRadps
                << L"\nactual_abs_radps="
                << measurement.firstMonotonicFailureActualRadps;

            Assert::AreEqual(
                -1,
                measurement.firstMonotonicFailureStep,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateNeverExceedsInitialMagnitude)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            const float limit =
                measurement.initialYawRateRadps +
                measurement.initialMagnitudeToleranceRadps;
            std::wstringstream message;
            message << L"InPlaceSlipYawRateNeverExceedsInitialMagnitude"
                << L"\nactual=" << measurement.maxYawRateAbsRadps
                << L"\nlimit=" << limit
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxYawRateAbsRadps <= limit,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateReboundStaysWithinStopBand)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateReboundStaysWithinStopBand"
                << L"\nactual=" << measurement.maxReboundRadps
                << L"\nlimit=" << measurement.maxAllowedReboundRadps
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxReboundRadps <= measurement.maxAllowedReboundRadps,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipSpinDownReachesStopBand)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipSpinDownReachesStopBand"
                << L"\nactual_stop_step=" << measurement.stopStep
                << L"\ncriterion=actual_stop_step>0";

            Assert::IsTrue(
                measurement.stopStep > 0,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipStopTimeStaysWithinPhysicalWindow)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            const float limit =
                measurement.maxAllowedStopTimeS + measurement.dtSeconds;
            std::wstringstream message;
            message << L"InPlaceSlipStopTimeStaysWithinPhysicalWindow"
                << L"\nactual=" << measurement.stopTimeS
                << L"\nlimit=" << limit
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.stopTimeS <= limit,
                message.str().c_str());
        }

    };

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

        TEST_METHOD(ExactRestHoldPreservesGyroBias)
        {
            const VehicleState state = IntegrateExactRestHold();
            std::wstringstream message;
            message << L"ExactRestHoldPreservesGyroBias"
                << L"\nexpected=0.12"
                << L"\nactual=" << state.GetGyroBiasZ()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(0.12f, state.GetGyroBiasZ(), 1.0e-7f, message.str().c_str());
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

    TEST_CLASS(PlantModelResidualDecayTest)
    {
    public:
        TEST_METHOD(ForwardAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"ForwardAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedForwardResidual
                << L"\nactual=" << measurement.actualForwardResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedForwardResidual,
                measurement.actualForwardResidual,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightwardAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"RightwardAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedRightResidual
                << L"\nactual=" << measurement.actualRightResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedRightResidual,
                measurement.actualRightResidual,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(YawAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"YawAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedYawResidual
                << L"\nactual=" << measurement.actualYawResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedYawResidual,
                measurement.actualYawResidual,
                1.0e-6f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelMixedSlipFiniteTest)
    {
    public:
        TEST_METHOD(MixedSlipLeftLongitudinalSlipIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.leftLongitudinalSlipMps;
            std::wstringstream message;
            message << L"MixedSlipLeftLongitudinalSlipIsFinite"
                << L"\nfield=left_longitudinal_slip_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipRightLongitudinalSlipIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightLongitudinalSlipMps;
            std::wstringstream message;
            message << L"MixedSlipRightLongitudinalSlipIsFinite"
                << L"\nfield=right_longitudinal_slip_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"MixedSlipContact0ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"MixedSlipContact1ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"MixedSlipContact2ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"MixedSlipContact3ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"MixedSlipContact0RightRelativeVelocityIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"MixedSlipContact1RightRelativeVelocityIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"MixedSlipContact2RightRelativeVelocityIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"MixedSlipContact3RightRelativeVelocityIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[0];
            std::wstringstream message;
            message << L"MixedSlipContact0RightForceIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[1];
            std::wstringstream message;
            message << L"MixedSlipContact1RightForceIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[2];
            std::wstringstream message;
            message << L"MixedSlipContact2RightForceIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[3];
            std::wstringstream message;
            message << L"MixedSlipContact3RightForceIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[0];
            std::wstringstream message;
            message << L"MixedSlipContact0ForwardForceIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[1];
            std::wstringstream message;
            message << L"MixedSlipContact1ForwardForceIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[2];
            std::wstringstream message;
            message << L"MixedSlipContact2ForwardForceIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[3];
            std::wstringstream message;
            message << L"MixedSlipContact3ForwardForceIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelMixedSlipBoundsTest)
    {
    public:
        TEST_METHOD(MixedSlipContact0SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationIsNonNegative"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[0] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationDoesNotExceedUnity"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[0] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[0] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[0] >= measurement.saturation[0], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationIsNonNegative"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[1] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationDoesNotExceedUnity"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[1] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[1] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[1] >= measurement.saturation[1], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationIsNonNegative"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[2] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationDoesNotExceedUnity"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[2] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[2] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[2] >= measurement.saturation[2], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationIsNonNegative"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[3] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationDoesNotExceedUnity"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[3] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[3] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[3] >= measurement.saturation[3], message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelMixedSlipStateTest)
    {
    public:
        TEST_METHOD(MixedSlipStatePositionXIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetPositionX();
            std::wstringstream message;
            message << L"MixedSlipStatePositionXIsFinite"
                << L"\nfield=x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStatePositionYIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetPositionY();
            std::wstringstream message;
            message << L"MixedSlipStatePositionYIsFinite"
                << L"\nfield=y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingIsFinite"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateForwardVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetForwardVelocity();
            std::wstringstream message;
            message << L"MixedSlipStateForwardVelocityIsFinite"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLateralVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetRightwardVelocity();
            std::wstringstream message;
            message << L"MixedSlipStateLateralVelocityIsFinite"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateYawRateIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetYawRate();
            std::wstringstream message;
            message << L"MixedSlipStateYawRateIsFinite"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLeftWheelSpeedIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"MixedSlipStateLeftWheelSpeedIsFinite"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateRightWheelSpeedIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"MixedSlipStateRightWheelSpeedIsFinite"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateGyroBiasIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetGyroBiasZ();
            std::wstringstream message;
            message << L"MixedSlipStateGyroBiasIsFinite"
                << L"\nfield=gyro_bias_z_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingDoesNotExceedPi)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingDoesNotExceedPi"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\nlimit=PI_F"
                << L"\ncriterion=actual<=PI_F";

            Assert::IsTrue(measurement.state.GetHeading() <= PI_F, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingDoesNotGoBelowNegativePi)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingDoesNotGoBelowNegativePi"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\nlimit=-PI_F"
                << L"\ncriterion=actual>=-PI_F";

            Assert::IsTrue(measurement.state.GetHeading() >= -PI_F, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateForwardVelocityMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetForwardVelocity());
            std::wstringstream message;
            message << L"MixedSlipStateForwardVelocityMagnitudeIsBounded"
                << L"\nfield=forward_velocity_abs_mps"
                << L"\nactual=" << actual
                << L"\nlimit=10"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetForwardVelocity()) < 10.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLateralVelocityMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetRightwardVelocity());
            std::wstringstream message;
            message << L"MixedSlipStateLateralVelocityMagnitudeIsBounded"
                << L"\nfield=lateral_velocity_abs_mps"
                << L"\nactual=" << actual
                << L"\nlimit=10"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetRightwardVelocity()) < 10.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateYawRateMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"MixedSlipStateYawRateMagnitudeIsBounded"
                << L"\nfield=yaw_rate_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=50"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetYawRate()) < 50.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLeftWheelSpeedMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetWheelSpeedLeft());
            std::wstringstream message;
            message << L"MixedSlipStateLeftWheelSpeedMagnitudeIsBounded"
                << L"\nfield=left_wheel_speed_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=1000"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetWheelSpeedLeft()) < 1000.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateRightWheelSpeedMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetWheelSpeedRight());
            std::wstringstream message;
            message << L"MixedSlipStateRightWheelSpeedMagnitudeIsBounded"
                << L"\nfield=right_wheel_speed_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=1000"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetWheelSpeedRight()) < 1000.0f, message.str().c_str());
        }
    };

    TEST_CLASS(PlantModelImuAccelerationTest)
    {
    public:
        TEST_METHOD(ImuRightLeverContributionIsPresent)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuRightLeverContributionIsPresent"
                << L"\nactual_abs=" << std::fabs(measurement.rightLeverContributionMps2)
                << L"\nlimit=1e-3"
                << L"\ncriterion=actual_abs>limit";

            Assert::IsTrue(
                std::fabs(measurement.rightLeverContributionMps2) > 1.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(ImuRightAccelerationMatchesLeverArmEquation)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuRightAccelerationMatchesLeverArmEquation"
                << L"\nexpected=" << measurement.expectedRightAccelerationMps2
                << L"\nactual=" << measurement.predictedRightAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedRightAccelerationMps2,
                measurement.predictedRightAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuForwardAccelerationMatchesLeverArmEquation)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuForwardAccelerationMatchesLeverArmEquation"
                << L"\nexpected=" << measurement.expectedForwardAccelerationMps2
                << L"\nactual=" << measurement.predictedForwardAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedForwardAccelerationMps2,
                measurement.predictedForwardAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuRightAccelerationUsesProjectBodyAxes)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(4.0f, 1.1f, -0.3f, 0.95f, 1.05f, 0.25f, 0.45f);
            std::wstringstream message;
            message << L"ImuRightAccelerationUsesProjectBodyAxes"
                << L"\nexpected=" << measurement.expectedRightAccelerationMps2
                << L"\nactual=" << measurement.predictedRightAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedRightAccelerationMps2,
                measurement.predictedRightAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuForwardAccelerationUsesProjectBodyAxes)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(4.0f, 1.1f, -0.3f, 0.95f, 1.05f, 0.25f, 0.45f);
            std::wstringstream message;
            message << L"ImuForwardAccelerationUsesProjectBodyAxes"
                << L"\nexpected=" << measurement.expectedForwardAccelerationMps2
                << L"\nactual=" << measurement.predictedForwardAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedForwardAccelerationMps2,
                measurement.predictedForwardAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelPositiveDriveTest)
    {
    public:
        TEST_METHOD(SymmetricPositiveDriveForwardVelocityIncreases)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveForwardVelocityIncreases"
                << L"\nactual=" << measurement.state.GetForwardVelocity()
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                measurement.state.GetForwardVelocity() > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDrivePositionYIncreases)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDrivePositionYIncreases"
                << L"\ninitial_y_m=0.09"
                << L"\nactual=" << measurement.state.GetPositionY()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                measurement.state.GetPositionY() > 0.09f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveAverageAccelerationIsPositive)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveAverageAccelerationIsPositive"
                << L"\nactual=" << measurement.averageAccelMps2
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                measurement.averageAccelMps2 > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveAverageAccelerationIsPlausible)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveAverageAccelerationIsPlausible"
                << L"\nactual=" << measurement.averageAccelMps2
                << L"\nlimit=60"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.averageAccelMps2 < 60.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDrivePositionXDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetPositionX());
            std::wstringstream message;
            message << L"SymmetricPositiveDrivePositionXDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.002"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.002f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveLateralVelocityDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetRightwardVelocity());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveLateralVelocityDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.02"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.02f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveYawRateDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveYawRateDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.10"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.10f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveHeadingDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetHeading());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveHeadingDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.01"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.01f, message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelFrictionAndSubthresholdTest)
    {
    public:
        TEST_METHOD(PositiveStaticFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"PositiveStaticFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << measurement.expectedStaticFrictionTorqueNm
                << L"\nactual=" << measurement.positiveStaticFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedStaticFrictionTorqueNm,
                measurement.positiveStaticFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(NegativeStaticFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"NegativeStaticFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << -measurement.expectedStaticFrictionTorqueNm
                << L"\nactual=" << measurement.negativeStaticFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -measurement.expectedStaticFrictionTorqueNm,
                measurement.negativeStaticFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PositiveRollingFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"PositiveRollingFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << measurement.expectedRollingFrictionTorqueNm
                << L"\nactual=" << measurement.positiveRollingFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedRollingFrictionTorqueNm,
                measurement.positiveRollingFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(NegativeRollingFrictionTorqueMatchesConfiguredValue)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"NegativeRollingFrictionTorqueMatchesConfiguredValue"
                << L"\nexpected=" << -measurement.expectedRollingFrictionTorqueNm
                << L"\nactual=" << measurement.negativeRollingFrictionTorqueNm
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -measurement.expectedRollingFrictionTorqueNm,
                measurement.negativeRollingFrictionTorqueNm,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveForwardVelocityStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveForwardVelocityStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetForwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                measurement.state.GetForwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveLateralVelocityStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveLateralVelocityStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetRightwardVelocity()
                << L"\ntolerance=" << kZeroLinearVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                measurement.state.GetRightwardVelocity(),
                kZeroLinearVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveYawRateStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveYawRateStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetYawRate()
                << L"\ntolerance=1e-7";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetYawRate(),
                1.0e-7f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveLeftWheelSpeedStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveLeftWheelSpeedStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetWheelSpeedLeft()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetWheelSpeedLeft(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveRightWheelSpeedStaysAtRest)
        {
            const StaticFrictionRestMeasurement measurement =
                MeasureStaticFrictionRest();
            std::wstringstream message;
            message << L"SubthresholdDriveRightWheelSpeedStaysAtRest"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.state.GetWheelSpeedRight()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.state.GetWheelSpeedRight(),
                1.0e-6f,
                message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelLargeStepIntegrationTest)
    {
    public:
        TEST_METHOD(SingleLargeStepWheelSpeedsRemainSymmetric)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.wheelSpeedDeltaRadps);
            std::wstringstream message;
            message << L"SingleLargeStepWheelSpeedsRemainSymmetric"
                << L"\nactual_abs=" << actual
                << L"\nlimit=1"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 1.0f, message.str().c_str());
        }

        TEST_METHOD(SingleLargeStepPositionXDriftIsBounded)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.state.GetPositionX());
            std::wstringstream message;
            message << L"SingleLargeStepPositionXDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.005"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.005f, message.str().c_str());
        }

        TEST_METHOD(SingleLargeStepYawRateDriftIsBounded)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"SingleLargeStepYawRateDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.10"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.10f, message.str().c_str());
        }

    };

    TEST_CLASS(PlantModelHeadingNormalizationTest)
    {
    public:
        TEST_METHOD(IntegrateHeadingDoesNotExceedPi)
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

            plant.integrate(App::Internal::CommandVector{}, 0.01f);
            std::wstringstream message;
            message << L"IntegrateHeadingDoesNotExceedPi"
                << L"\nactual=" << state.GetHeading()
                << L"\nmaximum=" << PI_F
                << L"\ncriterion=actual<=maximum";

            Assert::IsTrue(state.GetHeading() <= PI_F, message.str().c_str());
        }

        TEST_METHOD(IntegrateHeadingDoesNotGoBelowNegativePi)
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

            plant.integrate(App::Internal::CommandVector{}, 0.01f);
            std::wstringstream message;
            message << L"IntegrateHeadingDoesNotGoBelowNegativePi"
                << L"\nactual=" << state.GetHeading()
                << L"\nminimum=" << -PI_F
                << L"\ncriterion=actual>=minimum";

            Assert::IsTrue(state.GetHeading() >= -PI_F, message.str().c_str());
        }

    };
}






