#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

            Assert::AreEqual(vehicle.GetMass(), params.massKg, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY mass must originate from Vehicle.");
        }

        TEST_METHOD(DefaultParams_TrackWidthMatchesVehicle)
        {
            const Vehicle vehicle;
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(vehicle.GetTrackWidth(), params.trackWidthM, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY track width must originate from Vehicle.");
        }

        TEST_METHOD(DefaultParams_YawInertiaMatchesVehicle)
        {
            const Vehicle vehicle;
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(vehicle.GetYawInertia(), params.yawInertiaKgM2, 1.0e-8f,
                L"PM05_VEHICLE_PLANT_BOUNDARY yaw inertia must originate from Vehicle.");
        }

        TEST_METHOD(DefaultParams_WheelRadiusMatchesVehicle)
        {
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(Vehicle::GetDriveWheelRadiusM(), params.wheelRadiusM, 1.0e-7f,
                L"PM05_VEHICLE_PLANT_BOUNDARY wheel radius must match Vehicle drive construction.");
        }

        TEST_METHOD(DefaultParams_ContactLongitudinalOffsetMatchesVehicle)
        {
            const VehiclePhysicalModel& physical = Vehicle::GetPhysicalModel();
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(
                physical.driveWheelLongitudinalOffsetM,
                params.contactPatchLongitudinalOffsetM,
                1.0e-7f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact longitudinal offset must match Vehicle geometry.");
        }

        TEST_METHOD(DefaultParams_SustainedAccelerationMatchesVehicleReference)
        {
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(
                Vehicle::GetSustainedLateralAccelerationReferenceMps2(),
                params.combinedAccelSustainedMps2,
                1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY sustained acceleration reference must match Vehicle.");
        }

        TEST_METHOD(Prepare_RetainsVehicleTrackWidth)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.trackWidthM, prepared.trackWidthM, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY Prepare must retain Vehicle track width.");
        }

        TEST_METHOD(Prepare_RetainsVehicleWheelRadius)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.wheelRadiusM, prepared.wheelRadiusM, 1.0e-7f,
                L"PM05_VEHICLE_PLANT_BOUNDARY Prepare must retain Vehicle wheel radius.");
        }

        TEST_METHOD(Prepare_LongitudinalMassDerivesFromVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.massKg, prepared.longitudinalMassKg, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY longitudinal mass must derive from Vehicle mass.");
        }

        TEST_METHOD(Prepare_LateralMassDerivesFromVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.massKg, prepared.lateralMassKg, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY lateral mass must derive from Vehicle mass.");
        }

        TEST_METHOD(Prepare_YawInertiaDerivesFromVehicle)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.yawInertiaKgM2, prepared.yawInertiaKgM2, 1.0e-8f,
                L"PM05_VEHICLE_PLANT_BOUNDARY prepared yaw inertia must derive from Vehicle.");
        }

        TEST_METHOD(Prepare_BaseNormalLoadUsesVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.massKg * GRAVITY_MPS2, prepared.baseNormalLoadN, 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY base normal load must be Vehicle mass times gravity.");
        }

        TEST_METHOD(Prepare_SustainedLateralForceUsesVehicleMass)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(
                params.combinedAccelSustainedMps2 * params.massKg,
                prepared.lateralForceSustainedLimitN,
                1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY sustained lateral force must derive from Vehicle mass.");
        }

        TEST_METHOD(ContactGeometry_MinXUsesVehicleHalfTrack)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalModel().trackWidthM;

            Assert::AreEqual(-halfTrackWidthM, bounds.minX, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact geometry must use Vehicle half track width.");
        }

        TEST_METHOD(ContactGeometry_MaxXUsesVehicleHalfTrack)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalModel().trackWidthM;

            Assert::AreEqual(halfTrackWidthM, bounds.maxX, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact geometry must use Vehicle half track width.");
        }

        TEST_METHOD(ContactGeometry_MinYUsesVehicleWheelOffset)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float contactOffsetM = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);

            Assert::AreEqual(-contactOffsetM, bounds.minY, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact geometry must use Vehicle drive-wheel offset.");
        }

        TEST_METHOD(ContactGeometry_MaxYUsesVehicleWheelOffset)
        {
            const PlantParams params = PlantParams::Default();
            const ContactBounds bounds = GetContactBounds(params);
            const float contactOffsetM = std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);

            Assert::AreEqual(contactOffsetM, bounds.maxY, 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact geometry must use Vehicle drive-wheel offset.");
        }

        TEST_METHOD(PublicKinematics_MeasuredLinearSpeedUsesVehicleWheelRadius)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            GetRepresentativeWheelOmegas(observation.omegaLeftRadps, observation.omegaRightRadps);

            Assert::AreEqual(0.72f, plant.measuredLinearSpeedMps(observation), 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY measured speed must use Vehicle wheel radius.");
        }

        TEST_METHOD(PublicKinematics_MeasuredYawRateUsesVehicleWheelRadiusAndTrack)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            GetRepresentativeWheelOmegas(observation.omegaLeftRadps, observation.omegaRightRadps);

            Assert::AreEqual(1.35f, plant.measuredYawRateRadps(observation), 1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY measured yaw rate must use Vehicle wheel radius and track.");
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

            Assert::AreEqual(
                Vehicle::LeftWheelLinearVelocityFromBody(0.72f, 1.35f),
                wheelLinearMps(0),
                1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY left wheel body-state projection must match Vehicle.");
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

            Assert::AreEqual(
                Vehicle::RightWheelLinearVelocityFromBody(0.72f, 1.35f),
                wheelLinearMps(1),
                1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY right wheel body-state projection must match Vehicle.");
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

            Assert::IsTrue(command.IsFinite(),
                L"PM05_VEHICLE_PLANT_BOUNDARY forward feedforward must produce a finite command.");
            Assert::IsTrue(derivatives.longitudinalAccelMps2 > 0.0f,
                L"PM05_VEHICLE_PLANT_BOUNDARY forward feedforward must accelerate the Vehicle-owned plant forward.");
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

            Assert::AreEqual(command.LeftCommand(), command.RightCommand(), 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY pure forward acceleration must keep left/right command symmetry.");
        }

        TEST_METHOD(AccelerationFeedforward_ClockwiseYawRequestCommandsLeftAboveRight)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, 8.50f);

            Assert::IsTrue(command.LeftCommand() > command.RightCommand(),
                L"PM05_VEHICLE_PLANT_BOUNDARY clockwise yaw acceleration must request more left drive than right drive.");
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

            Assert::IsTrue(derivatives.yawAccelRadps2 > 0.0f,
                L"PM05_VEHICLE_PLANT_BOUNDARY clockwise yaw feedforward must accelerate clockwise in the plant.");
        }

        TEST_METHOD(Kinematics_VehicleWheelOmegasRoundTripThroughPlantMeasuredSpeed)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            Vehicle::WheelOmegasFromBodyVelocity(0.30f, 1.75f, observation.omegaLeftRadps, observation.omegaRightRadps);

            Assert::AreEqual(
                0.30f,
                plant.measuredLinearSpeedMps(observation),
                1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY Vehicle wheel omegas must round-trip through PlantModel speed measurement.");
        }

        TEST_METHOD(Kinematics_VehicleWheelOmegasRoundTripThroughPlantMeasuredYawRate)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant = MakePlant(vehicle, runtimeState);
            EncoderObs observation{};
            Vehicle::WheelOmegasFromBodyVelocity(0.30f, 1.75f, observation.omegaLeftRadps, observation.omegaRightRadps);

            Assert::AreEqual(
                1.75f,
                plant.measuredYawRateRadps(observation),
                1.0e-6f,
                L"PM05_VEHICLE_PLANT_BOUNDARY Vehicle wheel omegas must round-trip through PlantModel yaw measurement.");
        }

        TEST_METHOD(ContactLoad_ZeroFanTotalNormalLoadUsesVehicleMass)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.0f);
            const PlantParams params = PlantParams::Default();

            Assert::AreEqual(vehicle.GetMass() * GRAVITY_MPS2, params.TotalNormalLoadN(vehicle.GetFanDuty()), 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact loads must derive from Vehicle mass at zero fan.");
        }

        TEST_METHOD(ContactLoad_PreparedBaseNormalLoadUsesVehicleMass)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.0f);
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(params.TotalNormalLoadN(vehicle.GetFanDuty()), prepared.baseNormalLoadN, 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY prepared base normal load must derive from Vehicle mass.");
        }

        TEST_METHOD(ContactLoad_FanDutyAddsPreparedDownforce)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(
                prepared.baseNormalLoadN + (vehicle.GetFanDuty() * prepared.fanDownforceAtFullDutyN),
                params.TotalNormalLoadN(vehicle.GetFanDuty()),
                1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY contact loads must include Vehicle fan duty.");
        }

        TEST_METHOD(ContactLoad_FanDutyDeltaUsesPlantDownforcePath)
        {
            Vehicle vehicle;
            const PlantParams params = PlantParams::Default();
            vehicle.SetFanDuty(0.0f);
            const float noFanNormalLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());
            vehicle.SetFanDuty(0.80f);
            const float fanNormalLoadN = params.TotalNormalLoadN(vehicle.GetFanDuty());

            Assert::AreEqual(0.80f * params.fanDownforceAtFullDutyN, fanNormalLoadN - noFanNormalLoadN, 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY fan duty must add the PlantModel downforce path.");
        }

        TEST_METHOD(ContactLoad_AxleLoadSplitSumsToTotalNormalLoad)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());

            Assert::AreEqual(params.TotalNormalLoadN(vehicle.GetFanDuty()), frontAxleLoadN + rearAxleLoadN, 1.0e-5f,
                L"PM05_VEHICLE_PLANT_BOUNDARY public axle load split must sum to total normal load.");
        }

        TEST_METHOD(ContactLoad_FrontAxleLoadIsFinite)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());

            Assert::IsTrue(std::isfinite(frontAxleLoadN),
                L"PM05_VEHICLE_PLANT_BOUNDARY front axle load must remain finite.");
        }

        TEST_METHOD(ContactLoad_FrontAxleLoadIsPositive)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float frontAxleLoadN = 2.0f * params.FrontWheelLoadN(vehicle.GetFanDuty());

            Assert::IsTrue(frontAxleLoadN > 0.0f,
                L"PM05_VEHICLE_PLANT_BOUNDARY front axle load must remain positive.");
        }

        TEST_METHOD(ContactLoad_RearAxleLoadIsFinite)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());

            Assert::IsTrue(std::isfinite(rearAxleLoadN),
                L"PM05_VEHICLE_PLANT_BOUNDARY rear axle load must remain finite.");
        }

        TEST_METHOD(ContactLoad_RearAxleLoadIsPositive)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            const PlantParams params = PlantParams::Default();
            const float rearAxleLoadN = 2.0f * params.RearWheelLoadN(vehicle.GetFanDuty());

            Assert::IsTrue(rearAxleLoadN > 0.0f,
                L"PM05_VEHICLE_PLANT_BOUNDARY rear axle load must remain positive.");
        }
    };
}
