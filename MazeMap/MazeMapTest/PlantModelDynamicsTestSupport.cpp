#include "pch.h"
#include "PlantModelDynamicsTestSupport.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace MazeMap
{
    namespace PlantModelDynamicsTestSupport
    {
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
                << L"\nright_wheel_speed_radps=" << state.GetWheelSpeedRight();
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
}
