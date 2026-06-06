#pragma once

#include "EstimatorTestSupport.h"

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


namespace MazeMap
{
    namespace DriveStackEstimatorReplayTestSupport
    {
        constexpr float kReplayDtSeconds = 0.001f;
        constexpr float kForwardToleranceM = 0.006f;
        constexpr float kYawToleranceRad = 0.012f;
        constexpr float kVelocityToleranceMps = 0.020f;
        constexpr float kYawRateToleranceRadps = 0.030f;

        using CommandVector = App::Internal::CommandVector;

        inline int32_t ConsumeReplayEncoderCounts(
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

        inline SensorSnapshot BuildReplaySnapshot(
            ReplayEncoderState& encoderState,
            const float leftWheelVelocityMps,
            const float rightWheelVelocityMps,
            const float gyroRawRadps,
            const float dtSeconds,
            const bool encoderObservationValid = true)
        {
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);

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
            snapshot.SetRawYawRateRadps(gyroRawRadps);
            snapshot.SetYawRateRadps(std::isfinite(gyroRawRadps) ? gyroRawRadps : 0.0f);
            SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
            encoderObservation.SetTotalLeftCounts(leftDeltaCounts);
            encoderObservation.SetTotalRightCounts(rightDeltaCounts);
            encoderObservation.SetLeftDistanceDeltaM(static_cast<float>(leftDeltaCounts) * distancePerCountM);
            encoderObservation.SetRightDistanceDeltaM(static_cast<float>(rightDeltaCounts) * distancePerCountM);
            encoderObservation.SetLeftVelocityMps(encoderObservation.LeftDistanceDeltaM() / dtSeconds);
            encoderObservation.SetRightVelocityMps(encoderObservation.RightDistanceDeltaM() / dtSeconds);
            encoderObservation.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.LeftVelocityMps()));
            encoderObservation.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.RightVelocityMps()));
            snapshot.PublishEncoderObservation(
                encoderObservation,
                encoderObservationValid,
                encoderState.leftTotalCounts,
                encoderState.rightTotalCounts,
                Vehicle::DriveEncoderDistanceFromCounts(encoderState.leftTotalCounts),
                Vehicle::DriveEncoderDistanceFromCounts(encoderState.rightTotalCounts));
            return snapshot;
        }

        inline void ApplyReplaySnapshot(
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

        inline void ReplayWheelMotion(
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

        inline ReplayScenarioResult CaptureReplayResult(
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
            result.leftEncoderTotalCounts = snapshot.LeftEncoderTotalCounts();
            result.rightEncoderTotalCounts = snapshot.RightEncoderTotalCounts();
            result.leftEncoderDistanceM = snapshot.LeftEncoderDistanceM();
            result.expectedForwardM = Vehicle::DriveEncoderDistanceFromCounts(encoderState.leftTotalCounts);
            result.expectedYawRad = expectedYawRad;
            result.expectedArcXM = expectedArcXM;
            result.expectedArcYM = expectedArcYM;
            result.expectedVelocityMps = expectedVelocityMps;
            result.expectedYawRateRadps = expectedYawRateRadps;
            result.actualEncoderYawRateRadps =
                Vehicle::BodyYawRateFromWheelLinear(
                    Vehicle::WheelLinearVelocityFromWheelSpeed(snapshot.EncoderObservation().LeftWheelSpeedRadps()),
                    Vehicle::WheelLinearVelocityFromWheelSpeed(snapshot.EncoderObservation().RightWheelSpeedRadps()));
            result.positionXM = state.GetPositionX();
            result.positionYM = state.GetPositionY();
            result.yawRad = state.GetHeading();
            result.velocityMps = state.GetForwardVelocity();
            result.yawRateRadps = state.GetYawRate();
            return result;
        }

        inline ReplayScenarioResult RunForwardEncoderReplay()
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

        inline ReplayScenarioResult RunDifferentialEncoderReplay()
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
                NormalizeAngle(
                    base.actualEncoderYawRateRadps * kReplayDtSeconds * static_cast<float>(steps));
            return result;
        }

        inline ReplayScenarioResult RunGyroOnlyReplay()
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
                NormalizeAngle(yawRateRadps * kReplayDtSeconds * static_cast<float>(steps));
            return CaptureReplayResult(runtime, encoderState, 0.0f, yawRateRadps, expectedYawRad);
        }

        inline ReplayScenarioResult RunCombinedReplay()
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
            const float expectedYawRad = NormalizeAngle(yawRateRadps * totalTimeSeconds);
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
}
