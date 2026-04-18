#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\PlantModel.h"

#include <cmath>
#include <cstdint>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using ControlVector = MazeMap::App::Internal::LoopController::ControlVector;

        constexpr std::uint8_t kDriveBaseRightEncoderChannel = 1U;
        constexpr std::uint8_t kDriveBaseLeftEncoderChannel = 2U;
        constexpr float kDriveBasePredictDtSeconds = 0.01f;
        constexpr int kDriveBaseHoldFeedforwardSteps = 400;
        constexpr int kDriveBaseVelocityTargetSteps = 600;

        SensorSnapshot BuildDriveBaseSensorSnapshot(float yawRateRadps = 0.0f) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = yawRateRadps;
            snapshot.gyroRadps = yawRateRadps;
            return snapshot;
        }

        int32_t ConsumeWholeEncoderCounts(
            float deltaCounts,
            float& remainderCounts) noexcept
        {
            remainderCounts += deltaCounts;
            const int32_t wholeCounts =
                (remainderCounts >= 0.0f) ?
                static_cast<int32_t>(std::floor(remainderCounts)) :
                static_cast<int32_t>(std::ceil(remainderCounts));
            remainderCounts -= static_cast<float>(wholeCounts);
            return wholeCounts;
        }

        void SimulateDriveBaseCycle(
            DriveBase& drive,
            PlantModel& plant,
            PlantModel::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            float dtSeconds)
        {
            const PlantParams& params = PlantParams::Default();
            const DriveTelemetry telemetry = drive.GetTelemetry();
            ControlInput control{};
            control.leftMotorCommand = telemetry.leftDriveCommand;
            control.rightMotorCommand = telemetry.rightDriveCommand;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantModel::StateVector previousTruthState = truthState;
            truthState = plant.integrate(truthState, control, dtSeconds, params);

            const float leftDistanceDeltaM =
                0.5f *
                (previousTruthState(VehicleState::kOmegaL) + truthState(VehicleState::kOmegaL)) *
                params.wheelRadiusM *
                dtSeconds;
            const float rightDistanceDeltaM =
                0.5f *
                (previousTruthState(VehicleState::kOmegaR) + truthState(VehicleState::kOmegaR)) *
                params.wheelRadiusM *
                dtSeconds;
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            drive.UpdateOdometry(
                dtSeconds,
                BuildDriveBaseSensorSnapshot(truthState(VehicleState::kR)),
                nullptr,
                nullptr);
        }

        void AssertDriveBaseStateNearTarget(
            const VehicleState::StateVector& state,
            float expectedForwardVelocityMps,
            float expectedYawRateRadps,
            float forwardToleranceMps,
            float yawToleranceRadps,
            float maxLateralVelocityMps)
        {
            Assert::AreEqual(expectedForwardVelocityMps, state(VehicleState::kU), forwardToleranceMps);
            Assert::AreEqual(expectedYawRateRadps, state(VehicleState::kR), yawToleranceRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) <= maxLateralVelocityMps);
        }

        bool IsFiniteControlVector(const ControlVector& command) noexcept
        {
            return std::isfinite(command.leftMotorPwm) && std::isfinite(command.rightMotorPwm);
        }

        void AssertDriveCommandMatchesSolution(
            const ControlVector& command,
            const DriveCommandSolution& solution,
            float tolerance = 1.0e-6f)
        {
            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::AreEqual(solution.control.leftMotorCommand, command.leftMotorPwm, tolerance);
            Assert::AreEqual(solution.control.rightMotorCommand, command.rightMotorPwm, tolerance);
        }

        void PrimeDriveBaseWithEncoderDelta(
            DriveBase& drive,
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds = 0.001f)
        {
            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            const SensorSnapshot snapshot = BuildDriveBaseSensorSnapshot();
            drive.UpdateOdometry(dtSeconds, snapshot, nullptr, nullptr);
        }
    }

    TEST_CLASS(DriveBaseTest)
    {
    public:
        TEST_METHOD(DriveBaseDeltaCommandStaysSymmetricAcrossWheelSpeedMismatch)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(command.leftMotorPwm > 0.0f);
            Assert::AreEqual(command.leftMotorPwm, command.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandHeadingHoldStaysSymmetricWhenAlreadyAligned)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f,
                    MazeMap::CommandPD::StateHeadingPD);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::AreEqual(command.leftMotorPwm, command.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandRawMatchesPlantFeedforwardAtSteadyForwardTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.20f,
                    0.20f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBaseDeltaCommandCombinedRawMatchesPlantFeedforwardAtSteadyTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.20f,
                    0.20f,
                    0.40f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBaseDeltaYawRateCommandRawMatchesPlantFeedforwardAtSteadyYawRateTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaYawRateCommand(
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.0f,
                    0.0f,
                    0.40f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointCommandRawMatchesPlantVelocityTargetFeedforwardAtForwardTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    drive.GetEstimatorStateVector(),
                    0.20f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointCommandCombinedRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointCommand(
                    0.20f,
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    drive.GetEstimatorStateVector(),
                    0.20f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointYawRateCommandRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointYawRateCommand(
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    drive.GetEstimatorStateVector(),
                    0.0f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointMatchesScalarTargets)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            drive.UpdateOdometry(0.001f, BuildDriveBaseSensorSnapshot(0.15f), nullptr, nullptr);

            const ManeuverPoint point(0.0f, 0.0f, 0.25f, 0.30f, 0.20f);
            const ControlVector scalarCommand =
                drive.PointCommand(
                    point.Velocity,
                    point.Omega,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);
            const ControlVector pointCommand =
                drive.PointCommand(
                    point,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::IsTrue(IsFiniteControlVector(scalarCommand));
            Assert::IsTrue(IsFiniteControlVector(pointCommand));
            Assert::AreEqual(scalarCommand.leftMotorPwm, pointCommand.leftMotorPwm, 1.0e-6f);
            Assert::AreEqual(scalarCommand.rightMotorPwm, pointCommand.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointRejectsNonFiniteTargets)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            drive.UpdateOdometry(0.001f, BuildDriveBaseSensorSnapshot(0.0f), nullptr, nullptr);

            const ManeuverPoint invalidPoint(
                0.0f,
                0.0f,
                0.0f,
                std::numeric_limits<float>::quiet_NaN(),
                0.20f);
            const ControlVector command =
                drive.PointCommand(
                    invalidPoint,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::AreEqual(0.0f, command.leftMotorPwm, 1.0e-6f);
            Assert::AreEqual(0.0f, command.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandImuYawTrackingMatchesWheelOnlyAtTargetYawRate)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            drive.UpdateOdometry(0.001f, BuildDriveBaseSensorSnapshot(0.0f), nullptr, nullptr);

            const ControlVector wheelOnlyCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD);
            const ControlVector imuTrackedCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::IsTrue(IsFiniteControlVector(wheelOnlyCommand));
            Assert::IsTrue(IsFiniteControlVector(imuTrackedCommand));
            Assert::AreEqual(wheelOnlyCommand.leftMotorPwm, imuTrackedCommand.leftMotorPwm, 1.0e-6f);
            Assert::AreEqual(wheelOnlyCommand.rightMotorPwm, imuTrackedCommand.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists)
        {
            PlantModel plant;
            DriveBase drive(plant);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            drive.UpdateOdometry(0.001f, BuildDriveBaseSensorSnapshot(0.40f), nullptr, nullptr);

            const ControlVector wheelOnlyCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD);
            const ControlVector imuTrackedCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::IsTrue(IsFiniteControlVector(wheelOnlyCommand));
            Assert::IsTrue(IsFiniteControlVector(imuTrackedCommand));
            Assert::IsTrue(
                (std::fabs(imuTrackedCommand.leftMotorPwm - wheelOnlyCommand.leftMotorPwm) +
                 std::fabs(imuTrackedCommand.rightMotorPwm - wheelOnlyCommand.rightMotorPwm)) > 1.0e-4f);
        }
    };
}
