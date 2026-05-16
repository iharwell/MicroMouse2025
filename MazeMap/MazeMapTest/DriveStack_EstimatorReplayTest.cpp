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
#include <sstream>
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
            constexpr float yawRateRadps = 1.45f;
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
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=left_total_counts"
                << L"\nactual=" << result.leftEncoderTotalCounts
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.leftEncoderTotalCounts > 0, message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_LeftRightCountsMatch)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=equal_forward_counts"
                << L"\nexpected_left=" << result.leftEncoderTotalCounts
                << L"\nactual_right=" << result.rightEncoderTotalCounts
                << L"\ncriterion=left==right";

            Assert::AreEqual(
                result.leftEncoderTotalCounts,
                result.rightEncoderTotalCounts,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_LeftDistanceMatchesCounts)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=left_encoder_distance_m"
                << L"\nexpected=" << result.expectedForwardM
                << L"\nactual=" << result.leftEncoderDistanceM
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                result.expectedForwardM,
                result.leftEncoderDistanceM,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_PositionYMatchesEncoderDistance)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << result.expectedForwardM
                << L"\nactual=" << result.positionYM
                << L"\ntolerance=" << kForwardToleranceM;

            Assert::AreEqual(
                result.expectedForwardM,
                result.positionYM,
                kForwardToleranceM,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_PositionXStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=position_x_m"
                << L"\nexpected=0"
                << L"\nactual=" << result.positionXM
                << L"\ntolerance=" << kForwardToleranceM;

            Assert::AreEqual(
                0.0f,
                result.positionXM,
                kForwardToleranceM,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_YawStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=yaw_rad"
                << L"\nexpected=0"
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                0.0f,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_VelocityMatchesForward)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=velocity_mps"
                << L"\nexpected=" << result.expectedVelocityMps
                << L"\nactual=" << result.velocityMps
                << L"\ntolerance=" << kVelocityToleranceMps;

            Assert::AreEqual(
                result.expectedVelocityMps,
                result.velocityMps,
                kVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(EncoderOnlyForwardReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }

        TEST_METHOD(EncoderDifferentialReplay_LeftCountsGreaterThanRight)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=count_order"
                << L"\nleft=" << result.leftEncoderTotalCounts
                << L"\nright=" << result.rightEncoderTotalCounts
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                result.leftEncoderTotalCounts > result.rightEncoderTotalCounts,
                message.str().c_str());
        }

        TEST_METHOD(EncoderDifferentialReplay_YawSignPositive)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.yawRad > 0.0f, message.str().c_str());
        }

        TEST_METHOD(EncoderDifferentialReplay_YawRateMatchesEncoderDifferential)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << result.actualEncoderYawRateRadps
                << L"\nactual=" << result.yawRateRadps
                << L"\ntolerance=" << kYawRateToleranceRadps;

            Assert::AreEqual(
                result.actualEncoderYawRateRadps,
                result.yawRateRadps,
                kYawRateToleranceRadps,
                message.str().c_str());
        }

        TEST_METHOD(EncoderDifferentialReplay_YawMatchesEncoderIntegration)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << result.expectedYawRad
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                result.expectedYawRad,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(EncoderDifferentialReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }

        TEST_METHOD(GyroOnlyReplay_YawSignPositive)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.yawRad > 0.0f, message.str().c_str());
        }

        TEST_METHOD(GyroOnlyReplay_YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << result.expectedYawRad
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                result.expectedYawRad,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(GyroOnlyReplay_YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << result.expectedYawRateRadps
                << L"\nactual=" << result.yawRateRadps
                << L"\ntolerance=" << kYawRateToleranceRadps;

            Assert::AreEqual(
                result.expectedYawRateRadps,
                result.yawRateRadps,
                kYawRateToleranceRadps,
                message.str().c_str());
        }

        TEST_METHOD(GyroOnlyReplay_ForwardVelocityStaysZero)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=forward_velocity_mps"
                << L"\nexpected=0"
                << L"\nactual=" << result.velocityMps
                << L"\ntolerance=" << kVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                result.velocityMps,
                kVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(GyroOnlyReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nactual=" << result.positionXM
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.positionXM), message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nactual=" << result.positionYM
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.positionYM), message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.yawRad), message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nactual=" << result.positionXM
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.positionXM > 0.0f, message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nactual=" << result.positionYM
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.positionYM > 0.0f, message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionXMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nexpected=" << result.expectedArcXM
                << L"\nactual=" << result.positionXM
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                result.expectedArcXM,
                result.positionXM,
                0.010f,
                message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_PositionYMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << result.expectedArcYM
                << L"\nactual=" << result.positionYM
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                result.expectedArcYM,
                result.positionYM,
                0.010f,
                message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << result.expectedYawRad
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                result.expectedYawRad,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_VelocityMatchesEncoderAverage)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=velocity_mps"
                << L"\nexpected=" << result.expectedVelocityMps
                << L"\nactual=" << result.velocityMps
                << L"\ntolerance=" << kVelocityToleranceMps;

            Assert::AreEqual(
                result.expectedVelocityMps,
                result.velocityMps,
                kVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << result.expectedYawRateRadps
                << L"\nactual=" << result.yawRateRadps
                << L"\ntolerance=" << kYawRateToleranceRadps;

            Assert::AreEqual(
                result.expectedYawRateRadps,
                result.yawRateRadps,
                kYawRateToleranceRadps,
                message.str().c_str());
        }

        TEST_METHOD(CombinedEncoderGyroReplay_EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }
    };
}
