#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        struct ContactBounds final
        {
            float minX;
            float maxX;
            float minY;
            float maxY;
        };

        ContactBounds GetContactBounds(const PlantParams& params) noexcept
        {
            ContactBounds bounds{
                (std::numeric_limits<float>::infinity)(),
                -(std::numeric_limits<float>::infinity)(),
                (std::numeric_limits<float>::infinity)(),
                -(std::numeric_limits<float>::infinity)()};

            for (const Eigen::Vector2f& contact : params.contactPositionsBodyM)
            {
                bounds.minX = (std::min)(bounds.minX, contact.x());
                bounds.maxX = (std::max)(bounds.maxX, contact.x());
                bounds.minY = (std::min)(bounds.minY, contact.y());
                bounds.maxY = (std::max)(bounds.maxY, contact.y());
            }

            return bounds;
        }

        PlantModel MakePlant(Vehicle& vehicle, VehicleState& runtimeState)
        {
            return PlantModel(vehicle, runtimeState);
        }

        void GetRepresentativeWheelOmegas(float& leftOmegaRadps, float& rightOmegaRadps) noexcept
        {
            Vehicle::WheelOmegasFromBodyVelocity(0.72f, 1.35f, leftOmegaRadps, rightOmegaRadps);
        }
    }

    TEST_CLASS(DriveStack_VehiclePlantBoundaryTest)
    {
    public:
        TEST_METHOD(DefaultParams_MassMatchesVehicle)
        {
            const Vehicle vehicle;
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=mass_kg"
                << L"\nexpected=" << vehicle.GetMass()
                << L"\nactual=" << params.massKg
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                vehicle.GetMass(),
                params.massKg,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DefaultParams_TrackWidthMatchesVehicle)
        {
            const Vehicle vehicle;
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=track_width_m"
                << L"\nexpected=" << vehicle.GetTrackWidth()
                << L"\nactual=" << params.trackWidthM
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                vehicle.GetTrackWidth(),
                params.trackWidthM,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DefaultParams_YawInertiaMatchesVehicle)
        {
            const Vehicle vehicle;
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=yaw_inertia_kg_m2"
                << L"\nexpected=" << vehicle.GetYawInertia()
                << L"\nactual=" << params.yawInertiaKgM2
                << L"\ntolerance=1e-8";

            Assert::AreEqual(
                vehicle.GetYawInertia(),
                params.yawInertiaKgM2,
                1.0e-8f,
                message.str().c_str());
        }

        TEST_METHOD(DefaultParams_WheelRadiusMatchesVehicle)
        {
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=wheel_radius_m"
                << L"\nexpected=" << Vehicle::GetDriveWheelRadiusM()
                << L"\nactual=" << params.wheelRadiusM
                << L"\ntolerance=1e-7";

            Assert::AreEqual(
                Vehicle::GetDriveWheelRadiusM(),
                params.wheelRadiusM,
                1.0e-7f,
                message.str().c_str());
        }

        TEST_METHOD(DefaultParams_ContactLongitudinalOffsetMatchesVehicle)
        {
            const VehiclePhysicalModel& physical = Vehicle::GetPhysicalModel();
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=contact_patch_longitudinal_offset_m"
                << L"\nexpected=" << physical.driveWheelLongitudinalOffsetM
                << L"\nactual=" << params.contactPatchLongitudinalOffsetM
                << L"\ntolerance=1e-7";

            Assert::AreEqual(
                physical.driveWheelLongitudinalOffsetM,
                params.contactPatchLongitudinalOffsetM,
                1.0e-7f,
                message.str().c_str());
        }

        TEST_METHOD(DefaultParams_SustainedAccelerationMatchesVehicleReference)
        {
            const PlantParams params = PlantParams::Default();
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=combined_accel_sustained_mps2"
                << L"\nexpected=" << Vehicle::GetSustainedLateralAccelerationReferenceMps2()
                << L"\nactual=" << params.combinedAccelSustainedMps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                Vehicle::GetSustainedLateralAccelerationReferenceMps2(),
                params.combinedAccelSustainedMps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_RetainsVehicleTrackWidth)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=prepared_track_width_m"
                << L"\nexpected=" << params.trackWidthM
                << L"\nactual=" << prepared.trackWidthM
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                params.trackWidthM,
                prepared.trackWidthM,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_RetainsVehicleWheelRadius)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=prepared_wheel_radius_m"
                << L"\nexpected=" << params.wheelRadiusM
                << L"\nactual=" << prepared.wheelRadiusM
                << L"\ntolerance=1e-7";

            Assert::AreEqual(
                params.wheelRadiusM,
                prepared.wheelRadiusM,
                1.0e-7f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_LongitudinalMassDerivesFromVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=longitudinal_mass_kg"
                << L"\nexpected=" << params.massKg
                << L"\nactual=" << prepared.longitudinalMassKg
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                params.massKg,
                prepared.longitudinalMassKg,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_LateralMassDerivesFromVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=lateral_mass_kg"
                << L"\nexpected=" << params.massKg
                << L"\nactual=" << prepared.lateralMassKg
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                params.massKg,
                prepared.lateralMassKg,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_YawInertiaDerivesFromVehicle)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=prepared_yaw_inertia_kg_m2"
                << L"\nexpected=" << params.yawInertiaKgM2
                << L"\nactual=" << prepared.yawInertiaKgM2
                << L"\ntolerance=1e-8";

            Assert::AreEqual(
                params.yawInertiaKgM2,
                prepared.yawInertiaKgM2,
                1.0e-8f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_BaseNormalLoadUsesVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float expectedBaseNormalLoadN = params.massKg * GRAVITY_MPS2;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=base_normal_load_n"
                << L"\nexpected=" << expectedBaseNormalLoadN
                << L"\nactual=" << prepared.baseNormalLoadN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                expectedBaseNormalLoadN,
                prepared.baseNormalLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(Prepare_SustainedLateralForceUsesVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float expectedForceN = params.combinedAccelSustainedMps2 * params.massKg;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=lateral_force_sustained_limit_n"
                << L"\nexpected=" << expectedForceN
                << L"\nactual=" << prepared.lateralForceSustainedLimitN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                expectedForceN,
                prepared.lateralForceSustainedLimitN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactGeometry_MinXUsesVehicleHalfTrack)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalModel().trackWidthM;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=contact_min_x_m"
                << L"\nexpected=" << -halfTrackWidthM
                << L"\nactual=" << bounds.minX
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -halfTrackWidthM,
                bounds.minX,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactGeometry_MaxXUsesVehicleHalfTrack)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalModel().trackWidthM;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=contact_max_x_m"
                << L"\nexpected=" << halfTrackWidthM
                << L"\nactual=" << bounds.maxX
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                halfTrackWidthM,
                bounds.maxX,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactGeometry_MinYUsesVehicleWheelOffset)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float contactOffsetM = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=contact_min_y_m"
                << L"\nexpected=" << -contactOffsetM
                << L"\nactual=" << bounds.minY
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                -contactOffsetM,
                bounds.minY,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactGeometry_MaxYUsesVehicleWheelOffset)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float contactOffsetM = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=contact_max_y_m"
                << L"\nexpected=" << contactOffsetM
                << L"\nactual=" << bounds.maxY
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                contactOffsetM,
                bounds.maxY,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PublicKinematics_MeasuredLinearSpeedUsesVehicleWheelRadius)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            GetRepresentativeWheelOmegas(observation.omegaLeftRadps, observation.omegaRightRadps);
            const float actualSpeedMps = plant.measuredLinearSpeedMps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=measured_linear_speed_mps"
                << L"\nexpected=0.72"
                << L"\nactual=" << actualSpeedMps
                << L"\ntolerance=1e-6"
                << L"\nleft_omega_radps=" << observation.omegaLeftRadps
                << L"\nright_omega_radps=" << observation.omegaRightRadps;

            Assert::AreEqual(
                0.72f,
                actualSpeedMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PublicKinematics_MeasuredYawRateUsesVehicleWheelRadiusAndTrack)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            GetRepresentativeWheelOmegas(observation.omegaLeftRadps, observation.omegaRightRadps);
            const float actualYawRateRadps = plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=measured_yaw_rate_radps"
                << L"\nexpected=1.35"
                << L"\nactual=" << actualYawRateRadps
                << L"\ntolerance=1e-6"
                << L"\nleft_omega_radps=" << observation.omegaLeftRadps
                << L"\nright_omega_radps=" << observation.omegaRightRadps;

            Assert::AreEqual(
                1.35f,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PublicKinematics_LeftWheelProjectionMatchesVehicle)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            PlantModel::StateVector state = PlantModel::StateVector::Zero();
            state(VehicleState::kU) = 0.72f;
            state(VehicleState::kR) = 1.35f;
            const Eigen::Vector2f wheelLinearMps = plant.wheelLinearVelocityFromBodyState(state);
            const float expectedLeftMps = Vehicle::LeftWheelLinearVelocityFromBody(0.72f, 1.35f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=left_wheel_linear_mps"
                << L"\nexpected=" << expectedLeftMps
                << L"\nactual=" << wheelLinearMps(0)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expectedLeftMps,
                wheelLinearMps(0),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PublicKinematics_RightWheelProjectionMatchesVehicle)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            PlantModel::StateVector state = PlantModel::StateVector::Zero();
            state(VehicleState::kU) = 0.72f;
            state(VehicleState::kR) = 1.35f;
            const Eigen::Vector2f wheelLinearMps = plant.wheelLinearVelocityFromBodyState(state);
            const float expectedRightMps = Vehicle::RightWheelLinearVelocityFromBody(0.72f, 1.35f);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=right_wheel_linear_mps"
                << L"\nexpected=" << expectedRightMps
                << L"\nactual=" << wheelLinearMps(1)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expectedRightMps,
                wheelLinearMps(1),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ForwardRequestProducesPositivePlantAcceleration)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            const PlantParams params = PlantParams::Default();
            PlantModel::StateVector state = PlantModel::StateVector::Zero();
            state(VehicleState::kU) = 0.30f;
            state(VehicleState::kOmegaL) = Vehicle::WheelOmegaFromLinearVelocity(0.30f);
            state(VehicleState::kOmegaR) = Vehicle::WheelOmegaFromLinearVelocity(0.30f);
            runtimeState.SetVelocity(state(VehicleState::kU));
            runtimeState.SetWheelSpeedLeft(state(VehicleState::kOmegaL));
            runtimeState.SetWheelSpeedRight(state(VehicleState::kOmegaR));

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(1.20f, 0.0f);
            const PlantDerivatives derivatives =
                plant.forwardStep(state, command, params);
            std::wstringstream commandMessage;
            commandMessage << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=forward_feedforward_command_finite"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";
            std::wstringstream accelMessage;
            accelMessage << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=longitudinal_accel_mps2"
                << L"\nactual=" << derivatives.longitudinalAccelMps2
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                command.IsFinite(),
                commandMessage.str().c_str());
            Assert::IsTrue(
                derivatives.longitudinalAccelMps2 > 0.0f,
                accelMessage.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ForwardRequestKeepsWheelCommandsSymmetric)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            runtimeState.SetVelocity(0.30f);
            runtimeState.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(0.30f));
            runtimeState.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(0.30f));

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
            PlantModel plant = MakePlant(vehicle, runtimeState);

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
            PlantModel plant = MakePlant(vehicle, runtimeState);
            const PlantParams params = PlantParams::Default();

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 8.50f);
            const PlantDerivatives derivatives =
                plant.forwardStep(PlantModel::StateVector::Zero(), command, params);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=yaw_accel_radps2"
                << L"\nactual=" << derivatives.yawAccelRadps2
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand();

            Assert::IsTrue(
                derivatives.yawAccelRadps2 > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(Kinematics_VehicleWheelOmegasRoundTripThroughPlantMeasuredSpeed)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            Vehicle::WheelOmegasFromBodyVelocity(0.30f, 1.75f, observation.omegaLeftRadps, observation.omegaRightRadps);
            const float actualSpeedMps = plant.measuredLinearSpeedMps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=round_trip_speed_mps"
                << L"\nexpected=0.3"
                << L"\nactual=" << actualSpeedMps
                << L"\ntolerance=1e-6"
                << L"\nleft_omega_radps=" << observation.omegaLeftRadps
                << L"\nright_omega_radps=" << observation.omegaRightRadps;

            Assert::AreEqual(
                0.30f,
                actualSpeedMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Kinematics_VehicleWheelOmegasRoundTripThroughPlantMeasuredYawRate)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            Vehicle::WheelOmegasFromBodyVelocity(0.30f, 1.75f, observation.omegaLeftRadps, observation.omegaRightRadps);
            const float actualYawRateRadps = plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=round_trip_yaw_rate_radps"
                << L"\nexpected=1.75"
                << L"\nactual=" << actualYawRateRadps
                << L"\ntolerance=1e-6"
                << L"\nleft_omega_radps=" << observation.omegaLeftRadps
                << L"\nright_omega_radps=" << observation.omegaRightRadps;

            Assert::AreEqual(
                1.75f,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_ZeroFanTotalNormalLoadUsesVehicleMass)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.0f);
            const PlantParams params = PlantParams::Default();
            const float expectedLoadN = vehicle.GetMass() * GRAVITY_MPS2;
            const float actualLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=zero_fan_total_normal_load_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_PreparedBaseNormalLoadUsesVehicleMass)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.0f);
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float expectedLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=prepared_base_normal_load_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << prepared.baseNormalLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::AreEqual(
                expectedLoadN,
                prepared.baseNormalLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_FanDutyAddsPreparedDownforce)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float expectedLoadN =
                prepared.baseNormalLoadN + (vehicle.GetFanDuty() * prepared.fanDownforceAtFullDutyN);
            const float actualLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=fan_duty_total_normal_load_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_FanDutyDeltaUsesPlantDownforcePath)
        {
            Vehicle vehicle;
            const PlantParams params = PlantParams::Default();
            vehicle.SetFanDuty(0.0f);
            const float noFanNormalLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            vehicle.SetFanDuty(0.80f);
            const float fanNormalLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            const float expectedDeltaN = 0.80f * params.fanDownforceAtFullDutyN;
            const float actualDeltaN = fanNormalLoadN - noFanNormalLoadN;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=fan_duty_normal_load_delta_n"
                << L"\nexpected=" << expectedDeltaN
                << L"\nactual=" << actualDeltaN
                << L"\ntolerance=1e-5"
                << L"\nno_fan_normal_load_n=" << noFanNormalLoadN
                << L"\nfan_normal_load_n=" << fanNormalLoadN;

            Assert::AreEqual(
                expectedDeltaN,
                actualDeltaN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_AxleLoadSplitSumsToTotalNormalLoad)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());
            const float expectedTotalLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            const float actualTotalLoadN = frontAxleLoadN + rearAxleLoadN;
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=axle_load_sum_n"
                << L"\nexpected=" << expectedTotalLoadN
                << L"\nactual=" << actualTotalLoadN
                << L"\ntolerance=1e-5"
                << L"\nfront_axle_load_n=" << frontAxleLoadN
                << L"\nrear_axle_load_n=" << rearAxleLoadN;

            Assert::AreEqual(
                expectedTotalLoadN,
                actualTotalLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_FrontAxleLoadIsFinite)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=front_axle_load_n"
                << L"\nactual=" << frontAxleLoadN
                << L"\ncriterion=isfinite(actual)"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::IsTrue(
                std::isfinite(frontAxleLoadN),
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_FrontAxleLoadIsPositive)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=front_axle_load_n"
                << L"\nactual=" << frontAxleLoadN
                << L"\ncriterion=actual>0"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::IsTrue(
                frontAxleLoadN > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_RearAxleLoadIsFinite)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=rear_axle_load_n"
                << L"\nactual=" << rearAxleLoadN
                << L"\ncriterion=isfinite(actual)"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::IsTrue(
                std::isfinite(rearAxleLoadN),
                message.str().c_str());
        }

        TEST_METHOD(ContactLoad_RearAxleLoadIsPositive)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());
            std::wstringstream message;
            message << L"PM05_VEHICLE_PLANT_BOUNDARY"
                << L"\nfield=rear_axle_load_n"
                << L"\nactual=" << rearAxleLoadN
                << L"\ncriterion=actual>0"
                << L"\nfan_duty=" << vehicle.GetFanDuty();

            Assert::IsTrue(
                rearAxleLoadN > 0.0f,
                message.str().c_str());
        }
    };
}
