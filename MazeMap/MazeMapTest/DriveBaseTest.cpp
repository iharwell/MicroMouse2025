#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\OpenLoopDriveCommand.h"

#include <cmath>
#include <cstdint>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
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
            DriveBase drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const OpenLoopDriveCommand command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f);

            Assert::IsTrue(IsFiniteOpenLoopDriveCommand(command));
            Assert::IsTrue(command.leftDriveCommand > 0.0f);
            Assert::AreEqual(command.leftDriveCommand, command.rightDriveCommand, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandHeadingHoldStaysSymmetricWhenAlreadyAligned)
        {
            DriveBase drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const OpenLoopDriveCommand command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f,
                    MazeMap::CommandPD::StateHeadingPD);

            Assert::IsTrue(IsFiniteOpenLoopDriveCommand(command));
            Assert::AreEqual(command.leftDriveCommand, command.rightDriveCommand, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandRawAgreesWithPredictAtSteadyForwardTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseHoldFeedforwardSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.DeltaCommand(
                        0.20f,
                        0.0f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.20f, 0.0f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.20f, 0.0f, 0.03f, 0.03f, 0.02f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.20f, 0.0f, 0.03f, 0.03f, 0.02f);
        }

        TEST_METHOD(DriveBaseDeltaCommandCombinedRawAgreesWithPredictAtSteadyTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseHoldFeedforwardSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.DeltaCommand(
                        0.20f,
                        0.0f,
                        0.40f,
                        0.0f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.20f, 0.40f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.20f, 0.40f, 0.04f, 0.06f, 0.04f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.20f, 0.40f, 0.04f, 0.06f, 0.04f);
        }

        TEST_METHOD(DriveBaseDeltaYawRateCommandRawAgreesWithPredictAtSteadyYawRateTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseHoldFeedforwardSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.DeltaYawRateCommand(
                        0.40f,
                        0.0f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.0f, 0.40f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.0f, 0.40f, 0.03f, 0.06f, 0.04f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.0f, 0.40f, 0.03f, 0.06f, 0.04f);
        }

        TEST_METHOD(DriveBasePointCommandRawAgreesWithPredictAtForwardTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseVelocityTargetSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.PointCommand(
                        0.20f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.20f, 0.0f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.20f, 0.0f, 0.03f, 0.03f, 0.02f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.20f, 0.0f, 0.03f, 0.03f, 0.02f);
        }

        TEST_METHOD(DriveBasePointCommandCombinedRawAgreesWithPredictAtTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseVelocityTargetSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.PointCommand(
                        0.20f,
                        0.40f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.20f, 0.40f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.20f, 0.40f, 0.04f, 0.06f, 0.04f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.20f, 0.40f, 0.04f, 0.06f, 0.04f);
        }

        TEST_METHOD(DriveBasePointYawRateCommandRawAgreesWithPredictAtTarget)
        {
            DriveBase drive;
            PlantModel plant;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            for (int step = 0; step < kDriveBaseVelocityTargetSteps; ++step)
            {
                const OpenLoopDriveCommand command =
                    drive.PointYawRateCommand(
                        0.40f,
                        MazeMap::CommandPD::RawCommand);
                drive.CommandGenerated(command, 0.0f, 0.40f);
                SimulateDriveBaseCycle(
                    drive,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }

            AssertDriveBaseStateNearTarget(truthState, 0.0f, 0.40f, 0.03f, 0.06f, 0.04f);
            AssertDriveBaseStateNearTarget(drive.GetEstimatorStateVector(), 0.0f, 0.40f, 0.03f, 0.06f, 0.04f);
        }
    };
}
