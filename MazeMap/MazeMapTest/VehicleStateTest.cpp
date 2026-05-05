#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"

#include "..\MazeMap\SrUkfCore.h"

#include <array>
#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        struct RuntimeTuningRestoreScope
        {
            SrUkfCore::RuntimeTuning original = SrUkfCore::GetRuntimeTuning();

            ~RuntimeTuningRestoreScope()
            {
                SrUkfCore::SetRuntimeTuning(original);
            }
        };
    }

    TEST_CLASS(VehicleStateTest)
    {
    public:
        TEST_METHOD(VehicleStateIsStationaryUsesCurrentUkfThresholds)
        {
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedThresholdRadps =
                SrUkfCore::kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;

            VehicleState stationaryState;
            SetVehicleStateFromUkfStateVector(stationaryState, BuildUkfState(
                0.40f,
                -0.18f,
                0.35f,
                0.0f,
                0.0f,
                0.5f * (3.0f * SrUkfCore::kImuYawRateSigmaRadps),
                0.5f * wheelSpeedThresholdRadps,
                -0.5f * wheelSpeedThresholdRadps,
                0.12f));
            Assert::IsTrue(stationaryState.IsStationary());

            VehicleState movingState = stationaryState;
            movingState.SetVelocity(2.0f * SrUkfCore::kStationaryEncoderVelocitySigmaMps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetRotationalVelocity(3.1f * SrUkfCore::kImuYawRateSigmaRadps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetWheelSpeedLeft(1.1f * wheelSpeedThresholdRadps);
            Assert::IsFalse(movingState.IsStationary());
        }

        TEST_METHOD(VehicleStateIsStationaryTracksRuntimeTuningOverrides)
        {
            RuntimeTuningRestoreScope restore{};
            SrUkfCore::RuntimeTuning tuned = SrUkfCore::BuildDefaultRuntimeTuning();
            tuned.stationaryEncoderVelocitySigmaMps = 0.020f;
            tuned.imuYawRateSigmaRadps = 0.10f;
            SrUkfCore::SetRuntimeTuning(tuned);

            const PlantParams params = PlantParams::Default();
            const float wheelSpeedThresholdRadps =
                tuned.stationaryEncoderVelocitySigmaMps / params.wheelRadiusM;

            VehicleState state;
            SetVehicleStateFromUkfStateVector(state, BuildUkfState(
                0.0f,
                0.0f,
                0.0f,
                0.015f,
                -0.015f,
                0.25f,
                0.5f * wheelSpeedThresholdRadps,
                -0.5f * wheelSpeedThresholdRadps,
                0.0f));
            Assert::IsTrue(state.IsStationary());

            state.SetRotationalVelocity(0.31f);
            Assert::IsFalse(state.IsStationary());
        }

        TEST_METHOD(VehicleStateStationaryConstraintKeepsPoseReferenceAndCollapsesStationaryStates)
        {
            VehicleState state;
            SetVehicleStateFromUkfStateVector(state, BuildUkfState(
                0.40f,
                -0.18f,
                0.35f,
                0.8f,
                -0.12f,
                0.4f,
                9.0f,
                7.5f,
                -0.02f));
            state.SetCovariance(BuildUkfCovariance(0.05f, 0.08f, 0.30f, 0.20f, 0.25f, 0.45f, 0.06f));
            Assert::IsFalse(state.IsStationary());

            const VehicleState::StateVector poseReferenceState = BuildUkfState(
                1.20f,
                0.70f,
                -0.20f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            VehicleState::StateMatrix poseReferenceCovariance =
                BuildUkfCovariance(0.012f, 0.02f, 0.15f, 0.11f, 0.09f, 0.40f, 0.03f);
            poseReferenceCovariance(VehicleState::kPx, VehicleState::kPy) = 2.5e-5f;
            poseReferenceCovariance(VehicleState::kPy, VehicleState::kPx) = 2.5e-5f;
            poseReferenceCovariance(VehicleState::kPx, VehicleState::kPsi) = -1.5e-5f;
            poseReferenceCovariance(VehicleState::kPsi, VehicleState::kPx) = -1.5e-5f;
            poseReferenceCovariance(VehicleState::kPy, VehicleState::kPsi) = 1.2e-5f;
            poseReferenceCovariance(VehicleState::kPsi, VehicleState::kPy) = 1.2e-5f;

            state.ApplyStationaryZeroMotionConstraint(
                true,
                true,
                poseReferenceState,
                poseReferenceCovariance);

            const VehicleState::StateMatrix constrainedCovariance = state.GetCovariance();

            Assert::AreEqual(poseReferenceState(VehicleState::kPx), state.GetPositionX(), 1.0e-6f);
            Assert::AreEqual(poseReferenceState(VehicleState::kPy), state.GetPositionY(), 1.0e-6f);
            Assert::AreEqual(poseReferenceState(VehicleState::kPsi), state.GetOrientation(), 1.0e-6f);
            Assert::AreEqual(0.0f, state.GetVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetLateralVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetRotationalVelocity(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedLeft(), 1.0e-7f);
            Assert::AreEqual(0.0f, state.GetWheelSpeedRight(), 1.0e-7f);
            Assert::AreEqual(-0.02f, state.GetGyroBiasZ(), 1.0e-7f);
            Assert::IsTrue(state.IsStationary());

            constexpr std::array<int, 3> kPoseIndices = {
                VehicleState::kPx,
                VehicleState::kPy,
                VehicleState::kPsi
            };
            for (const int row : kPoseIndices)
            {
                for (const int col : kPoseIndices)
                {
                    Assert::AreEqual(
                        poseReferenceCovariance(row, col),
                        constrainedCovariance(row, col),
                        1.0e-7f);
                }
            }

            Assert::IsTrue(constrainedCovariance(VehicleState::kU, VehicleState::kU) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kV, VehicleState::kV) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kR, VehicleState::kR) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <= 1.0e-12f);
            Assert::AreEqual(0.06f * 0.06f, constrainedCovariance(VehicleState::kBgz, VehicleState::kBgz), 1.0e-7f);
        }
    };
}
