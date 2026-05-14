#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"

#include "..\MazeMap\Vehicle.h"

#include <array>
#include <cmath>
#include <cstring>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kStationaryEncoderVelocitySigmaMps = 0.002936f;
        constexpr float kImuYawRateSigmaRadps = 0.0010954451f;

#define VEHICLE_STATE_LOG_TEST_ROW_FIELDS(X) \
    X(VehicleStateLogEntry, ukf_state)

        MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(
            VehicleStateLogTestRow,
            VEHICLE_STATE_LOG_TEST_ROW_FIELDS,
            void Set(const VehicleState& state) noexcept
            {
                ukf_state.Set(state);
            });

#define VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS(X) \
    X(float, ukf_state_px_m) \
    X(float, ukf_state_py_m) \
    X(float, ukf_state_psi_rad) \
    X(float, ukf_state_u_mps) \
    X(float, ukf_state_v_mps) \
    X(float, ukf_state_r_radps) \
    X(float, ukf_state_omega_l_radps) \
    X(float, ukf_state_omega_r_radps) \
    X(float, ukf_state_bgz_radps)

        MMLOG_DEFINE_ROW(
            VehicleStateLogTestFlattenedRow,
            VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS);

#undef VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS
#undef VEHICLE_STATE_LOG_TEST_ROW_FIELDS
    }

    TEST_CLASS(VehicleStateTest)
    {
    public:
        TEST_METHOD(VehicleWheelOmegasFromBodyVelocityMatchesBodyKinematics)
        {
            constexpr float forwardMps = 0.72f;
            constexpr float yawRateRadps = 1.35f;

            float leftOmegaRadps = 0.0f;
            float rightOmegaRadps = 0.0f;
            Vehicle::WheelOmegasFromBodyVelocity(
                forwardMps,
                yawRateRadps,
                leftOmegaRadps,
                rightOmegaRadps);

            const float leftWheelMps = Vehicle::WheelLinearVelocityFromOmega(leftOmegaRadps);
            const float rightWheelMps = Vehicle::WheelLinearVelocityFromOmega(rightOmegaRadps);

            Assert::AreEqual(
                Vehicle::LeftWheelLinearVelocityFromBody(forwardMps, yawRateRadps),
                leftWheelMps,
                1.0e-6f);
            Assert::AreEqual(
                Vehicle::RightWheelLinearVelocityFromBody(forwardMps, yawRateRadps),
                rightWheelMps,
                1.0e-6f);
            Assert::AreEqual(
                forwardMps,
                Vehicle::BodyForwardVelocityFromWheelLinear(leftWheelMps, rightWheelMps),
                1.0e-6f);
            Assert::AreEqual(
                yawRateRadps,
                Vehicle::BodyYawRateFromWheelLinear(leftWheelMps, rightWheelMps),
                1.0e-6f);
        }

        TEST_METHOD(VehicleStateIsStationaryUsesCurrentUkfThresholds)
        {
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedThresholdRadps =
                kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;

            VehicleState stationaryState;
            SetVehicleStateFromUkfStateVector(stationaryState, BuildUkfState(
                0.40f,
                -0.18f,
                0.35f,
                0.0f,
                0.0f,
                0.5f * (3.0f * kImuYawRateSigmaRadps),
                0.5f * wheelSpeedThresholdRadps,
                -0.5f * wheelSpeedThresholdRadps,
                0.12f));
            Assert::IsTrue(stationaryState.IsStationary());

            VehicleState movingState = stationaryState;
            movingState.SetVelocity(2.0f * kStationaryEncoderVelocitySigmaMps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetRotationalVelocity(3.1f * kImuYawRateSigmaRadps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetWheelSpeedLeft(1.1f * wheelSpeedThresholdRadps);
            Assert::IsFalse(movingState.IsStationary());
        }

        TEST_METHOD(VehicleStateLogEntryProjectsThroughDomainGetters)
        {
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(1.25f, -2.5f));
            state.SetOrientation(3.75f);
            state.SetVelocity(-4.125f);
            state.SetLateralVelocity(5.5f);
            state.SetRotationalVelocity(-6.625f);
            state.SetWheelSpeedLeft(7.875f);
            state.SetWheelSpeedRight(-8.25f);
            state.SetGyroBiasZ(9.5f);

            VehicleStateLogTestRow projected{};
            projected.Set(state);

            VehicleStateLogTestFlattenedRow flattened{};
            flattened.ukf_state_px_m = state.GetPositionX();
            flattened.ukf_state_py_m = state.GetPositionY();
            flattened.ukf_state_psi_rad = state.GetOrientation();
            flattened.ukf_state_u_mps = state.GetVelocity();
            flattened.ukf_state_v_mps = state.GetLateralVelocity();
            flattened.ukf_state_r_radps = state.GetRotationalVelocity();
            flattened.ukf_state_omega_l_radps = state.GetWheelSpeedLeft();
            flattened.ukf_state_omega_r_radps = state.GetWheelSpeedRight();
            flattened.ukf_state_bgz_radps = state.GetGyroBiasZ();

            const std::string expectedHeader =
                "f32_ukf_state_px_m,f32_ukf_state_py_m,f32_ukf_state_psi_rad,"
                "f32_ukf_state_u_mps,f32_ukf_state_v_mps,f32_ukf_state_r_radps,"
                "f32_ukf_state_omega_l_radps,f32_ukf_state_omega_r_radps,"
                "f32_ukf_state_bgz_radps";

            Assert::AreEqual(expectedHeader, std::string(VehicleStateLogTestRow::header_cstr()));
            Assert::AreEqual(sizeof(flattened), sizeof(projected));
            Assert::AreEqual(
                0,
                std::memcmp(
                    &flattened,
                    &projected,
                    sizeof(VehicleStateLogTestRow)));
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

