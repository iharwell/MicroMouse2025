#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"

#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kReplayDtSeconds = 0.004f;
        constexpr float kForwardToleranceM = 0.006f;
        constexpr float kYawToleranceRad = 0.012f;
        constexpr float kVelocityToleranceMps = 0.020f;
        constexpr float kYawRateToleranceRadps = 0.030f;

        using CommandVector = App::Internal::CommandVector;

        int32_t ConsumeReplayEncoderCounts(
            const float deltaCounts,
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

        std::wstring ReplayMessage(
            const wchar_t* label,
            const wchar_t* field,
            const float expected,
            const float actual)
        {
            return
                std::wstring(label) +
                L" field=" + field +
                L" expected=" + std::to_wstring(expected) +
                L" actual=" + std::to_wstring(actual);
        }

        void AssertNearReplay(
            const wchar_t* label,
            const wchar_t* field,
            const float expected,
            const float actual,
            const float tolerance)
        {
            const std::wstring message = ReplayMessage(label, field, expected, actual);
            Assert::AreEqual(expected, actual, tolerance, message.c_str());
        }

        struct ReplayEncoderState final
        {
            float leftRemainderCounts = 0.0f;
            float rightRemainderCounts = 0.0f;
            std::int64_t leftTotalCounts = 0;
            std::int64_t rightTotalCounts = 0;
        };

        struct ReplayScenarioResult final
        {
            bool estimatorFault = false;
            std::int64_t leftEncoderTotalCounts = 0;
            std::int64_t rightEncoderTotalCounts = 0;
            float leftEncoderDistanceM = 0.0f;
            float expectedForwardM = 0.0f;
            float expectedYawRad = 0.0f;
            float expectedArcXM = 0.0f;
            float expectedArcYM = 0.0f;
            float expectedVelocityMps = 0.0f;
            float expectedYawRateRadps = 0.0f;
            float actualEncoderYawRateRadps = 0.0f;
            float positionXM = 0.0f;
            float positionYM = 0.0f;
            float yawRad = 0.0f;
            float velocityMps = 0.0f;
            float yawRateRadps = 0.0f;
        };

        SensorSnapshot BuildReplaySnapshot(
            ReplayEncoderState& encoderState,
            const float leftWheelVelocityMps,
            const float rightWheelVelocityMps,
            const float gyroRawRadps,
            const float dtSeconds,
            const bool encoderObservationValid = true)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);

            const int32_t leftDeltaCounts =
                ConsumeReplayEncoderCounts(
                    (leftWheelVelocityMps * dtSeconds) / distancePerCountM,
                    encoderState.leftRemainderCounts);
            const int32_t rightDeltaCounts =
                ConsumeReplayEncoderCounts(
                    (rightWheelVelocityMps * dtSeconds) / distancePerCountM,
                    encoderState.rightRemainderCounts);
            encoderState.leftTotalCounts += static_cast<std::int64_t>(leftDeltaCounts);
            encoderState.rightTotalCounts += static_cast<std::int64_t>(rightDeltaCounts);

            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = gyroRawRadps;
            snapshot.gyroRadps = std::isfinite(gyroRawRadps) ? gyroRawRadps : 0.0f;
            snapshot.encoderObservationValid = encoderObservationValid;
            snapshot.encoderObservation.totalLeftCounts = leftDeltaCounts;
            snapshot.encoderObservation.totalRightCounts = rightDeltaCounts;
            snapshot.encoderObservation.leftDistanceDeltaM =
                static_cast<float>(leftDeltaCounts) * distancePerCountM;
            snapshot.encoderObservation.rightDistanceDeltaM =
                static_cast<float>(rightDeltaCounts) * distancePerCountM;
            snapshot.encoderObservation.leftVelocityMps =
                snapshot.encoderObservation.leftDistanceDeltaM / dtSeconds;
            snapshot.encoderObservation.rightVelocityMps =
                snapshot.encoderObservation.rightDistanceDeltaM / dtSeconds;
            snapshot.encoderObservation.omegaLeftRadps =
                snapshot.encoderObservation.leftVelocityMps / params.wheelRadiusM;
            snapshot.encoderObservation.omegaRightRadps =
                snapshot.encoderObservation.rightVelocityMps / params.wheelRadiusM;
            snapshot.leftEncoderTotalCounts = encoderState.leftTotalCounts;
            snapshot.rightEncoderTotalCounts = encoderState.rightTotalCounts;
            snapshot.leftEncoderDistanceM =
                Vehicle::DriveEncoderDistanceFromCounts(snapshot.leftEncoderTotalCounts);
            snapshot.rightEncoderDistanceM =
                Vehicle::DriveEncoderDistanceFromCounts(snapshot.rightEncoderTotalCounts);
            return snapshot;
        }

        void ApplyReplaySnapshot(
            App::Internal::SharedRobotRuntime& runtime,
            const SensorSnapshot& snapshot,
            const float dtSeconds)
        {
            UpdateDriveEstimator(
                runtime.Estimator(),
                runtime.RuntimeState(),
                dtSeconds,
                snapshot,
                CommandVector{});
        }

        void ReplayWheelMotion(
            App::Internal::SharedRobotRuntime& runtime,
            ReplayEncoderState& encoderState,
            const float leftWheelVelocityMps,
            const float rightWheelVelocityMps,
            const float gyroRawRadps,
            const int steps,
            const bool encoderObservationValid = true)
        {
            for (int step = 0; step < steps; ++step)
            {
                const SensorSnapshot snapshot =
                    BuildReplaySnapshot(
                        encoderState,
                        leftWheelVelocityMps,
                        rightWheelVelocityMps,
                        gyroRawRadps,
                        kReplayDtSeconds,
                        encoderObservationValid);
                ApplyReplaySnapshot(runtime, snapshot, kReplayDtSeconds);
            }
        }

        ReplayScenarioResult CaptureReplayResult(
            App::Internal::SharedRobotRuntime& runtime,
            const ReplayEncoderState& encoderState,
            const float expectedVelocityMps,
            const float expectedYawRateRadps,
            const float expectedYawRad = 0.0f,
            const float expectedArcXM = 0.0f,
            const float expectedArcYM = 0.0f)
        {
            const VehicleState& state = runtime.RuntimeState();
            const SensorSnapshot& snapshot = state.GetSensorSnapshot();

            ReplayScenarioResult result{};
            result.estimatorFault = runtime.Estimator().HasFault();
            result.leftEncoderTotalCounts = snapshot.leftEncoderTotalCounts;
            result.rightEncoderTotalCounts = snapshot.rightEncoderTotalCounts;
            result.leftEncoderDistanceM = snapshot.leftEncoderDistanceM;
            result.expectedForwardM = Vehicle::DriveEncoderDistanceFromCounts(encoderState.leftTotalCounts);
            result.expectedYawRad = expectedYawRad;
            result.expectedArcXM = expectedArcXM;
            result.expectedArcYM = expectedArcYM;
            result.expectedVelocityMps = expectedVelocityMps;
            result.expectedYawRateRadps = expectedYawRateRadps;
            result.actualEncoderYawRateRadps =
                Vehicle::BodyYawRateFromWheelLinear(
                    Vehicle::WheelLinearVelocityFromOmega(snapshot.encoderObservation.omegaLeftRadps),
                    Vehicle::WheelLinearVelocityFromOmega(snapshot.encoderObservation.omegaRightRadps));
            result.positionXM = state.GetPositionX();
            result.positionYM = state.GetPositionY();
            result.yawRad = state.GetOrientation();
            result.velocityMps = state.GetVelocity();
            result.yawRateRadps = state.GetRotationalVelocity();
            return result;
        }

        ReplayScenarioResult RunForwardEncoderReplay()
        {
            App::Internal::SharedRobotRuntime runtime(kReplayDtSeconds);
            (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);

            ReplayEncoderState encoderState{};
            constexpr float forwardVelocityMps = 0.25f;
            constexpr int steps = 80;
            ReplayWheelMotion(
                runtime,
                encoderState,
                forwardVelocityMps,
                forwardVelocityMps,
                std::numeric_limits<float>::quiet_NaN(),
                steps);

            return CaptureReplayResult(runtime, encoderState, forwardVelocityMps, 0.0f);
        }

        ReplayScenarioResult RunDifferentialEncoderReplay()
        {
            App::Internal::SharedRobotRuntime runtime(kReplayDtSeconds);
            (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);

            ReplayEncoderState encoderState{};
            constexpr float forwardVelocityMps = 0.12f;
            constexpr float yawRateRadps = 0.60f;
            constexpr int steps = 70;
            const float leftWheelVelocityMps =
                Vehicle::LeftWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps);
            const float rightWheelVelocityMps =
                Vehicle::RightWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps);

            ReplayWheelMotion(
                runtime,
                encoderState,
                leftWheelVelocityMps,
                rightWheelVelocityMps,
                std::numeric_limits<float>::quiet_NaN(),
                steps);

            const ReplayScenarioResult base =
                CaptureReplayResult(runtime, encoderState, forwardVelocityMps, yawRateRadps);

            ReplayScenarioResult result = base;
            result.expectedYawRad =
                VehicleState::NormalizeAngle(
                    base.actualEncoderYawRateRadps * kReplayDtSeconds * static_cast<float>(steps));
            return result;
        }

        ReplayScenarioResult RunGyroOnlyReplay()
        {
            App::Internal::SharedRobotRuntime runtime(kReplayDtSeconds);
            (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);

            ReplayEncoderState encoderState{};
            constexpr float yawRateRadps = 0.55f;
            constexpr int steps = 80;
            ReplayWheelMotion(
                runtime,
                encoderState,
                0.0f,
                0.0f,
                yawRateRadps,
                steps,
                false);

            const float expectedYawRad =
                VehicleState::NormalizeAngle(yawRateRadps * kReplayDtSeconds * static_cast<float>(steps));
            return CaptureReplayResult(runtime, encoderState, 0.0f, yawRateRadps, expectedYawRad);
        }

        ReplayScenarioResult RunCombinedReplay()
        {
            App::Internal::SharedRobotRuntime runtime(kReplayDtSeconds);
            (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);

            ReplayEncoderState encoderState{};
            constexpr float forwardVelocityMps = 0.22f;
            constexpr float yawRateRadps = 0.45f;
            constexpr int steps = 90;
            const float leftWheelVelocityMps =
                Vehicle::LeftWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps);
            const float rightWheelVelocityMps =
                Vehicle::RightWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps);

            ReplayWheelMotion(
                runtime,
                encoderState,
                leftWheelVelocityMps,
                rightWheelVelocityMps,
                yawRateRadps,
                steps);

            const float totalTimeSeconds = kReplayDtSeconds * static_cast<float>(steps);
            const float expectedYawRad = VehicleState::NormalizeAngle(yawRateRadps * totalTimeSeconds);
            const float expectedRadiusM = forwardVelocityMps / yawRateRadps;
            const float expectedXM = expectedRadiusM * (1.0f - std::cos(expectedYawRad));
            const float expectedYM = expectedRadiusM * std::sin(expectedYawRad);

            return
                CaptureReplayResult(
                    runtime,
                    encoderState,
                    forwardVelocityMps,
                    yawRateRadps,
                    expectedYawRad,
                    expectedXM,
                    expectedYM);
        }
    }

    TEST_CLASS(DriveStack_EstimatorReplayTest)
    {
    public:
        TEST_METHOD(EncoderOnlyForwardReplay_LeftCountsPositive)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            Assert::IsTrue(result.leftEncoderTotalCounts > 0, L"EST40_ENCODER_SIGN field=left_total_counts expected positive forward counts");
        }

        TEST_METHOD(EncoderOnlyForwardReplay_LeftRightCountsMatch)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            Assert::AreEqual(result.leftEncoderTotalCounts, result.rightEncoderTotalCounts, L"EST40_ENCODER_SIGN field=equal_forward_counts expected left/right totals to match");
        }

        TEST_METHOD(EncoderOnlyForwardReplay_LeftDistanceMatchesCounts)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            AssertNearReplay(L"EST40_ENCODER_SIGN", L"left_encoder_distance_m", result.expectedForwardM, result.leftEncoderDistanceM, 1.0e-6f);
        }

        TEST_METHOD(EncoderOnlyForwardReplay_PositionYMatchesEncoderDistance)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            AssertNearReplay(L"EST40_ENCODER_SIGN", L"position_y_m", result.expectedForwardM, result.positionYM, kForwardToleranceM);
        }

        TEST_METHOD(EncoderOnlyForwardReplay_PositionXStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            AssertNearReplay(L"EST40_ENCODER_SIGN", L"position_x_m", 0.0f, result.positionXM, kForwardToleranceM);
        }

        TEST_METHOD(EncoderOnlyForwardReplay_YawStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            AssertNearReplay(L"EST40_ENCODER_SIGN", L"yaw_rad", 0.0f, result.yawRad, kYawToleranceRad);
        }

        TEST_METHOD(EncoderOnlyForwardReplay_VelocityMatchesForward)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            AssertNearReplay(L"EST40_ENCODER_SIGN", L"velocity_mps", result.expectedVelocityMps, result.velocityMps, kVelocityToleranceMps);
        }

        TEST_METHOD(EncoderOnlyForwardReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            Assert::IsFalse(result.estimatorFault, L"EST40_ENCODER_SIGN field=estimator_fault expected false");
        }

        TEST_METHOD(EncoderDifferentialReplay_LeftCountsGreaterThanRight)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            Assert::IsTrue(result.leftEncoderTotalCounts > result.rightEncoderTotalCounts, L"EST40_LEFT_RIGHT_MAPPING field=count_order expected left counts greater than right for clockwise-positive yaw");
        }

        TEST_METHOD(EncoderDifferentialReplay_YawSignPositive)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            Assert::IsTrue(result.yawRad > 0.0f, L"EST40_LEFT_RIGHT_MAPPING field=yaw_sign expected +Yaw clockwise from left>right");
        }

        TEST_METHOD(EncoderDifferentialReplay_YawRateMatchesEncoderDifferential)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            AssertNearReplay(L"EST40_LEFT_RIGHT_MAPPING", L"yaw_rate_radps", result.actualEncoderYawRateRadps, result.yawRateRadps, kYawRateToleranceRadps);
        }

        TEST_METHOD(EncoderDifferentialReplay_YawMatchesEncoderIntegration)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            AssertNearReplay(L"EST40_LEFT_RIGHT_MAPPING", L"yaw_rad", result.expectedYawRad, result.yawRad, kYawToleranceRad);
        }

        TEST_METHOD(EncoderDifferentialReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            Assert::IsFalse(result.estimatorFault, L"EST40_LEFT_RIGHT_MAPPING field=estimator_fault expected false");
        }

        TEST_METHOD(GyroOnlyReplay_YawSignPositive)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            Assert::IsTrue(result.yawRad > 0.0f, L"EST40_GYRO_SIGN field=yaw_sign expected positive gyro to produce +Yaw clockwise");
        }

        TEST_METHOD(GyroOnlyReplay_YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            AssertNearReplay(L"EST40_GYRO_SIGN", L"yaw_rad", result.expectedYawRad, result.yawRad, kYawToleranceRad);
        }

        TEST_METHOD(GyroOnlyReplay_YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            AssertNearReplay(L"EST40_GYRO_SIGN", L"yaw_rate_radps", result.expectedYawRateRadps, result.yawRateRadps, kYawRateToleranceRadps);
        }

        TEST_METHOD(GyroOnlyReplay_ForwardVelocityStaysZero)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            AssertNearReplay(L"EST40_GYRO_SIGN", L"forward_velocity_mps", 0.0f, result.velocityMps, kVelocityToleranceMps);
        }

        TEST_METHOD(GyroOnlyReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            Assert::IsFalse(result.estimatorFault, L"EST40_GYRO_SIGN field=estimator_fault expected false");
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsTrue(std::isfinite(result.positionXM), ReplayMessage(L"EST40_REPLAY_COHERENCE", L"position_x_m", 0.0f, result.positionXM).c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsTrue(std::isfinite(result.positionYM), ReplayMessage(L"EST40_REPLAY_COHERENCE", L"position_y_m", 0.0f, result.positionYM).c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsTrue(std::isfinite(result.yawRad), ReplayMessage(L"EST40_REPLAY_COHERENCE", L"yaw_rad", 0.0f, result.yawRad).c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsTrue(result.positionXM > 0.0f, L"EST40_REPLAY_COHERENCE field=position_x_sign expected clockwise arc to move toward +X");
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsTrue(result.positionYM > 0.0f, L"EST40_REPLAY_COHERENCE field=position_y_sign expected forward arc to move toward +Y");
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            AssertNearReplay(L"EST40_REPLAY_COHERENCE", L"position_x_m", result.expectedArcXM, result.positionXM, 0.010f);
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            AssertNearReplay(L"EST40_REPLAY_COHERENCE", L"position_y_m", result.expectedArcYM, result.positionYM, 0.010f);
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            AssertNearReplay(L"EST40_REPLAY_COHERENCE", L"yaw_rad", result.expectedYawRad, result.yawRad, kYawToleranceRad);
        }

        TEST_METHOD(CombinedEncoderGyroReplay_VelocityMatchesEncoderAverage)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            AssertNearReplay(L"EST40_REPLAY_COHERENCE", L"velocity_mps", result.expectedVelocityMps, result.velocityMps, kVelocityToleranceMps);
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            AssertNearReplay(L"EST40_REPLAY_COHERENCE", L"yaw_rate_radps", result.expectedYawRateRadps, result.yawRateRadps, kYawRateToleranceRadps);
        }

        TEST_METHOD(CombinedEncoderGyroReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            Assert::IsFalse(result.estimatorFault, L"EST40_REPLAY_COHERENCE field=estimator_fault expected false");
        }
    };
}
