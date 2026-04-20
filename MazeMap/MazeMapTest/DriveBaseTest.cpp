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

        SensorSnapshot BuildDriveBaseSensorSnapshot(
            float gyroRawRadps,
            float gyroRadps,
            float accelBodyXMps2,
            float accelBodyYMps2,
            bool accelBiasValid) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = gyroRawRadps;
            snapshot.gyroRadps = gyroRadps;
            snapshot.accelBodyXMps2 = accelBodyXMps2;
            snapshot.accelBodyYMps2 = accelBodyYMps2;
            snapshot.accelBiasValid = accelBiasValid;
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

        float ControlVectorDifferenceMagnitude(
            const ControlVector& lhs,
            const ControlVector& rhs) noexcept
        {
            return
                std::fabs(lhs.leftMotorPwm - rhs.leftMotorPwm) +
                std::fabs(lhs.rightMotorPwm - rhs.rightMotorPwm);
        }

        void AssertDriveCommandsEqual(
            const ControlVector& expected,
            const ControlVector& actual,
            float tolerance = 1.0e-4f)
        {
            Assert::IsTrue(IsFiniteControlVector(expected));
            Assert::IsTrue(IsFiniteControlVector(actual));
            Assert::AreEqual(expected.leftMotorPwm, actual.leftMotorPwm, tolerance);
            Assert::AreEqual(expected.rightMotorPwm, actual.rightMotorPwm, tolerance);
        }

        void AssertDriveCommandsDiffer(
            const ControlVector& lhs,
            const ControlVector& rhs,
            float minimumDifference = 1.0e-4f)
        {
            Assert::IsTrue(IsFiniteControlVector(lhs));
            Assert::IsTrue(IsFiniteControlVector(rhs));
            Assert::IsTrue(ControlVectorDifferenceMagnitude(lhs, rhs) > minimumDifference);
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

        void UpdateDriveBaseSignals(
            DriveBase& drive,
            const SensorSnapshot& snapshot,
            const int32_t leftCounts = 0,
            const int32_t rightCounts = 0,
            const float dtSeconds = 0.001f)
        {
            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            drive.UpdateOdometry(dtSeconds, snapshot, nullptr, nullptr);
        }
    }

    TEST_CLASS(DriveBaseTest)
    {
    public:
        TEST_METHOD(DriveBaseDeltaCommandStaysSymmetricAcrossWheelSpeedMismatch)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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
            DriveBase drive(plant, Config::kDriveBasePDCluster);
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

        TEST_METHOD(DriveBasePointCommandCoupledStateYawPdUsesEstimatorYawRateSignal)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);
            const ControlVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsDiffer(rawCommand, stateYawCommand);
            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawUsesCorrectedImuSignal)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.40f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);
            const ControlVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, stateYawCommand);
            AssertDriveCommandsDiffer(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandStateAccelerationPdUsesStateAccelerationSignal)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            ControlVector commandedMotorPwm{};
            commandedMotorPwm.leftMotorPwm = 0.10f;
            commandedMotorPwm.rightMotorPwm = 0.10f;
            drive.CommandGenerated(commandedMotorPwm, 0.0f, 0.0f, false);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    true));
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            const ControlVector imuAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUForwardAccel);

            AssertDriveCommandsDiffer(rawCommand, stateAccelerationCommand);
            AssertDriveCommandsDiffer(stateAccelerationCommand, imuAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandImuForwardAccelUsesCorrectedImuSignal)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            const ControlVector imuAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUForwardAccel);

            AssertDriveCommandsEqual(rawCommand, stateAccelerationCommand);
            AssertDriveCommandsDiffer(rawCommand, imuAccelerationCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyOffersStateAndEncoderSpeedLoops)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, 48, 48);
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateVelocityPD);
            const ControlVector stateWheelOmegaCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD);
            const ControlVector encoderVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::EncoderVelocity);

            AssertDriveCommandsEqual(rawCommand, stateVelocityCommand);
            AssertDriveCommandsEqual(rawCommand, stateWheelOmegaCommand);
            AssertDriveCommandsDiffer(rawCommand, encoderVelocityCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyIgnoresYawSourcePdFlags)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.40f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateYawCommand =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::StateYawPD);
            const ControlVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, stateYawCommand);
            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBaseDeltaYawRateCommandIgnoresYawRatePdFlags)
        {
            PlantModel plant;
            DriveBase drive(plant, Config::kDriveBasePDCluster);
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.DeltaYawRateCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateYawCommand =
                drive.DeltaYawRateCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);
            const ControlVector imuYawCommand =
                drive.DeltaYawRateCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, stateYawCommand);
            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }
    };
}
