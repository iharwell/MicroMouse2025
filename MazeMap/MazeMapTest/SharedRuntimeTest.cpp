#include "pch.h"
#include "CppUnitTest.h"
#include "EstimatorTestSupport.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\Vehicle.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace mmlog = MazeMap::mmlog;

#define SHARED_RUNTIME_TEST_LOG_FIELDS(X) \
    X(std::uint32_t, seq) \
    X(float,         value)

MMLOG_DEFINE_ROW(SharedRuntimeTestLogRow, SHARED_RUNTIME_TEST_LOG_FIELDS);

namespace
{
    struct FaultCallbackState
    {
        unsigned int count = 0U;
        char reason[64] = {};
    };

    struct StreamingFaultCallbackState
    {
        MazeMap::App::Internal::SharedRobotRuntime* runtime = nullptr;
        unsigned int linesWritten = 0U;
    };

    struct ScopedMissionFanDuty final
    {
        explicit ScopedMissionFanDuty(const float dutyCycle) noexcept
            : previousDutyCycle(GetMissionFanDutyCycle())
        {
            WriteFanDutyCycle(dutyCycle);
        }

        ~ScopedMissionFanDuty() noexcept
        {
            WriteFanDutyCycle(previousDutyCycle);
        }

        float previousDutyCycle = 0.0f;
    };

    SensorSnapshot BuildSharedRuntimeSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.gyroRawRadps = yawRateRadps;
        snapshot.gyroRadps = yawRateRadps;
        return snapshot;
    }

    void PrimeSharedRuntimeDriveWheelSpeed(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        const float leftWheelSpeedMps,
        const float rightWheelSpeedMps,
        const float dtSeconds = 0.001f) noexcept
    {
        const MazeMap::PlantParams params = MazeMap::PlantParams::Default();
        const float distancePerCountM = MazeMap::DistancePerEncoderCountMeters(params);
        const int32_t leftCounts = static_cast<int32_t>(std::round((leftWheelSpeedMps * dtSeconds) / distancePerCountM));
        const int32_t rightCounts = static_cast<int32_t>(std::round((rightWheelSpeedMps * dtSeconds) / distancePerCountM));
        SensorSnapshot snapshot = BuildSharedRuntimeSensorSnapshot();
        snapshot.encoderObservation.totalLeftCounts = leftCounts;
        snapshot.encoderObservation.totalRightCounts = rightCounts;
        snapshot.encoderObservation.leftDistanceDeltaM = static_cast<float>(leftCounts) * distancePerCountM;
        snapshot.encoderObservation.rightDistanceDeltaM = static_cast<float>(rightCounts) * distancePerCountM;
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds) && (params.wheelRadiusM > 0.0f))
        {
            const float invWheelRadiusM = 1.0f / params.wheelRadiusM;
            const float invDtSeconds = 1.0f / dtSeconds;
            snapshot.encoderObservation.leftVelocityMps =
                snapshot.encoderObservation.leftDistanceDeltaM * invDtSeconds;
            snapshot.encoderObservation.rightVelocityMps =
                snapshot.encoderObservation.rightDistanceDeltaM * invDtSeconds;
            snapshot.encoderObservation.omegaLeftRadps =
                snapshot.encoderObservation.leftVelocityMps * invWheelRadiusM;
            snapshot.encoderObservation.omegaRightRadps =
                snapshot.encoderObservation.rightVelocityMps * invWheelRadiusM;
        }
        snapshot.encoderObservationValid = true;
        snapshot.leftEncoderTotalCounts =
            runtime.RuntimeState().GetSensorSnapshot().leftEncoderTotalCounts +
            static_cast<std::int64_t>(leftCounts);
        snapshot.rightEncoderTotalCounts =
            runtime.RuntimeState().GetSensorSnapshot().rightEncoderTotalCounts +
            static_cast<std::int64_t>(rightCounts);
        snapshot.leftEncoderDistanceM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.leftEncoderTotalCounts);
        snapshot.rightEncoderDistanceM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.rightEncoderTotalCounts);
        UpdateDriveEstimator(runtime.Drive(), runtime.Estimator(), runtime.RuntimeState(), dtSeconds, snapshot);
    }

    void CaptureFaultCallback(void* context, const char* reason) noexcept
    {
        auto* state = static_cast<FaultCallbackState*>(context);
        if (state == nullptr)
        {
            return;
        }

        ++state->count;
        const char* text = (reason != nullptr) ? reason : "";
        std::snprintf(state->reason, sizeof(state->reason), "%s", text);
    }

    void StreamLongFaultDump(void* context, const char* reason) noexcept
    {
        auto* state = static_cast<StreamingFaultCallbackState*>(context);
        if (state == nullptr || state->runtime == nullptr)
        {
            return;
        }

        for (unsigned int index = 0U; index < 32U; ++index)
        {
            char message[256] = {};
            const int length = std::snprintf(
                message,
                sizeof(message),
                "reason=%s;line=%u;payload=%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u",
                (reason != nullptr) ? reason : "unknown",
                index,
                index,
                index + 1U,
                index + 2U,
                index + 3U,
                index + 4U,
                index + 5U,
                index + 6U,
                index + 7U,
                index + 8U,
                index + 9U,
                index + 10U,
                index + 11U);
            if (length <= 0 || length >= static_cast<int>(sizeof(message)))
            {
                return;
            }

            if (!state->runtime->WriteTextLogEntry(1000U + index, "ukf_dump_test", message))
            {
                return;
            }

            state->runtime->FlushTextLog();
            ++state->linesWritten;
        }
    }

}

namespace MazeMap::App
{
    TEST_CLASS(SharedRuntimeTest)
    {
    public:
        static std::string ReadAllBytes(const std::string& path)
        {
            std::ifstream file(path, std::ios::binary);
            return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        }

        static std::string ReplaceExtension(const std::string& path, const char* extension)
        {
            const std::size_t dot = path.find_last_of('.');
            return (dot == std::string::npos) ? (path + extension) : (path.substr(0, dot) + extension);
        }

        static std::string CreateTempPath(const char* stem)
        {
            static unsigned long counter = 0UL;
            char path[96] = {};
            const int length = snprintf(
                path,
                sizeof(path),
                "%s_%lu.mmlog",
                (stem != nullptr) ? stem : "shared_runtime_test",
                ++counter);
            Assert::IsTrue(length > 0 && length < static_cast<int>(sizeof(path)));
            std::remove(path);
            return std::string(path);
        }

        TEST_METHOD(SharedRuntime_ProvidesOneCanonicalVehicle)
        {
            Internal::SharedRobotRuntime runtime;

            auto& first = runtime.Vehicle();
            auto& second = runtime.Vehicle();

            Assert::IsTrue(
                static_cast<const void*>(&first) ==
                static_cast<const void*>(&second));
        }

        TEST_METHOD(SharedRuntime_ProvidesOneCanonicalRuntimeSensorSuite)
        {
            Internal::SharedRobotRuntime runtime;
            auto& first = runtime.Sensors();
            auto& second = runtime.Sensors();

            Assert::IsTrue(
                static_cast<const void*>(&first) ==
                static_cast<const void*>(&second));
            Assert::IsTrue(&runtime.Drive() == &runtime.Drive());
        }

        TEST_METHOD(SharedRuntime_ProvidesOneCanonicalManeuverExecutor)
        {
            Internal::SharedRobotRuntime runtime;
            auto& first = runtime.ManeuverExecutorService();
            auto& second = runtime.ManeuverExecutorService();

            Assert::IsTrue(
                static_cast<const void*>(&first) ==
                static_cast<const void*>(&second));
        }

        TEST_METHOD(SharedRuntime_ProvidesOneCanonicalDriveService)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& first = runtime.DriveService();
            Internal::Drive& second = runtime.DriveService();

            Assert::IsTrue(&first == &second);
        }

        TEST_METHOD(SharedRuntime_DriveServicePreservesConfiguredLimitsVerbatim)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            MotionLimits limits{};
            limits.SetMaxSpeedMps(-0.50f);
            limits.SetAccelMps2(-1.25f);
            limits.SetDecelMps2(-2.50f);
            limits.SetMaxAngularSpeedRadps(-3.50f);
            limits.SetAngularAccelRadps2(-4.50f);
            limits.SetAngleToleranceRad(-0.05f);
            drive.SetLimits(limits);

            const MotionLimits& configured = drive.GetLimits();
            Assert::AreEqual(limits.GetMaxSpeedMps(), configured.GetMaxSpeedMps(), 1.0e-6f);
            Assert::AreEqual(limits.GetAccelMps2(), configured.GetAccelMps2(), 1.0e-6f);
            Assert::AreEqual(limits.GetDecelMps2(), configured.GetDecelMps2(), 1.0e-6f);
            Assert::AreEqual(limits.GetMaxAngularSpeedRadps(), configured.GetMaxAngularSpeedRadps(), 1.0e-6f);
            Assert::AreEqual(limits.GetAngularAccelRadps2(), configured.GetAngularAccelRadps2(), 1.0e-6f);
            Assert::AreEqual(limits.GetAngleToleranceRad(), configured.GetAngleToleranceRad(), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveServiceProducesCommandsWithoutLoopModeState)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);

            drive.StartStraight(0.25f, 0.20f, 0.0f);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
        }

        TEST_METHOD(SharedRuntime_DriveTurnUsesHeadingTargetCommandPath)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            PrimeSharedRuntimeDriveWheelSpeed(runtime, 0.06f, 0.06f);

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);

            const Internal::CommandVector expected =
                runtime.Drive().PointControlVectorWithHeadingTarget(
                    0.0f,
                    limits.GetMaxAngularSpeedRadps(),
                    HALF_PI_F,
                    Config::kDriveVelocityFeedbackSources,
                    Config::kDriveYawRateFeedbackSources,
                    Config::kDriveHeadingFeedbackSources);

            drive.StartTurn(HALF_PI_F);

            bool done = false;
            const Internal::CommandVector actual = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::AreEqual(expected.LeftMotorPwm(), actual.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(expected.RightMotorPwm(), actual.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveStraightUsesDriveBaseHeadingCluster)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(2000.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);

            const Eigen::Vector2f targetHeadingUnit(0.0f, 1.0f);
            const auto runStraightOnce =
                [&runtime, &drive, &targetHeadingUnit]() -> Internal::CommandVector
            {
                runtime.Drive().Brake();
                runtime.Drive().ResetControllers();
                Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.20f));
                drive.StartStraight(0.25f, 0.20f, 0.0f, &targetHeadingUnit, nullptr);

                bool done = false;
                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsFalse(done);
                return control;
            };

            const MazeMap::PDCluster baselineCluster =
                runtime.Drive().GetProportionalDerivativeCluster();
            MazeMap::PDCluster zeroHeadingCluster = baselineCluster;
            zeroHeadingCluster.HeadingStatePD = MazeMap::ProportionalDerivative(0.0f, 0.0f);

            runtime.Drive().SetProportionalDerivativeCluster(baselineCluster);
            const Internal::CommandVector baselineControl = runStraightOnce();

            runtime.Drive().SetProportionalDerivativeCluster(zeroHeadingCluster);
            const Internal::CommandVector zeroHeadingControl = runStraightOnce();

            Assert::IsTrue(
                (std::fabs(baselineControl.LeftMotorPwm() - zeroHeadingControl.LeftMotorPwm()) > 1.0e-6f) ||
                (std::fabs(baselineControl.RightMotorPwm() - zeroHeadingControl.RightMotorPwm()) > 1.0e-6f));
        }

        TEST_METHOD(SharedRuntime_DriveHoldRejectsFanOffWheelSpeedAboveBaseSettleThreshold)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            PrimeSharedRuntimeDriveWheelSpeed(runtime, 0.06f, 0.06f);
            ScopedMissionFanDuty fanDuty(0.0f);

            Internal::Drive& drive = runtime.DriveService();
            drive.StartHold(1U, true);

            bool done = false;
            (void)drive.GetNextControls(done);
            Assert::IsFalse(done);
        }

        TEST_METHOD(SharedRuntime_DriveCompletionDoesNotCancelLatestInstruction)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            drive.StartHold(0U, true);
            Assert::IsFalse(drive.IsEffectivelyComplete());

            bool firstDone = false;
            const Internal::CommandVector firstControl = drive.GetNextControls(firstDone);
            Assert::IsTrue(firstDone);
            Assert::IsTrue(drive.IsEffectivelyComplete());
            Assert::IsFalse(std::isfinite(firstControl.LeftMotorPwm()));
            Assert::IsFalse(std::isfinite(firstControl.RightMotorPwm()));

            bool secondDone = false;
            (void)drive.GetNextControls(secondDone);
            Assert::IsTrue(secondDone);
            Assert::IsTrue(drive.IsEffectivelyComplete());
        }

        TEST_METHOD(SharedRuntime_DriveIsEffectivelyCompleteStartsTrueAndClearsOnNewInstruction)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            Assert::IsTrue(drive.IsEffectivelyComplete());

            drive.StartHold(0U, true);
            Assert::IsFalse(drive.IsEffectivelyComplete());

            bool done = false;
            (void)drive.GetNextControls(done);
            Assert::IsTrue(done);
            Assert::IsTrue(drive.IsEffectivelyComplete());

            drive.StartHold(1U, true);
            Assert::IsFalse(drive.IsEffectivelyComplete());
        }

        TEST_METHOD(SharedRuntime_DriveNoneManeuverStillLatchesLatestInstruction)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            MazeMap::ManeuverInstance maneuver;
            drive.StartManeuver(maneuver);
            Assert::IsFalse(drive.IsEffectivelyComplete());

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsTrue(done);
            Assert::IsTrue(drive.IsEffectivelyComplete());
            Assert::IsFalse(std::isfinite(control.LeftMotorPwm()));
            Assert::IsFalse(std::isfinite(control.RightMotorPwm()));
        }

        TEST_METHOD(SharedRuntime_DriveIllPosedStraightRequestRemainsActiveAndFinite)
        {
            Internal::SharedRobotRuntime runtime;
            Internal::Drive& drive = runtime.DriveService();

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);

            drive.StartStraight(NAN, INFINITY, NAN);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
        }

        TEST_METHOD(SharedRuntime_DriveZeroVectorHeadingOverrideFallsBackToCapturedHeading)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            const Eigen::Vector2f zeroHeading(0.0f, 0.0f);
            const auto sampleStraightControl =
                [&runtime, &drive, &limits](const Eigen::Vector2f* headingOverride) -> Internal::CommandVector
            {
                runtime.Drive().Brake();
                runtime.Drive().ResetControllers();
                Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.35f));
                drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
                drive.SetLimits(limits);
                drive.StartStraight(0.25f, 0.20f, 0.0f, headingOverride, nullptr);

                bool done = false;
                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsFalse(done);
                Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
                Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
                return control;
            };

            const Internal::CommandVector zeroHeadingControl = sampleStraightControl(&zeroHeading);
            const Internal::CommandVector nullHeadingControl = sampleStraightControl(nullptr);
            Assert::AreEqual(nullHeadingControl.LeftMotorPwm(), zeroHeadingControl.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(nullHeadingControl.RightMotorPwm(), zeroHeadingControl.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveInfiniteHeadingCoordinatesBehaveLikeFiniteDirectionHints)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            const auto sampleStraightControl =
                [&runtime, &drive, &limits](const Eigen::Vector2f& headingOverride) -> Internal::CommandVector
            {
                runtime.Drive().Brake();
                runtime.Drive().ResetControllers();
                Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.35f));
                drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
                drive.SetLimits(limits);
                drive.StartStraight(0.25f, 0.20f, 0.0f, &headingOverride, nullptr);

                bool done = false;
                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsFalse(done);
                Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
                Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
                return control;
            };

            const Eigen::Vector2f rightInfiniteHeading(INFINITY, 1.0f);
            const Eigen::Vector2f rightFiniteHeading(1.0f, 0.0f);
            const Internal::CommandVector rightInfiniteControl = sampleStraightControl(rightInfiniteHeading);
            const Internal::CommandVector rightFiniteControl = sampleStraightControl(rightFiniteHeading);
            Assert::AreEqual(rightFiniteControl.LeftMotorPwm(), rightInfiniteControl.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(rightFiniteControl.RightMotorPwm(), rightInfiniteControl.RightMotorPwm(), 1.0e-6f);

            const Eigen::Vector2f diagonalInfiniteHeading(INFINITY, INFINITY);
            const Eigen::Vector2f diagonalFiniteHeading(1.0f, 1.0f);
            const Internal::CommandVector diagonalInfiniteControl = sampleStraightControl(diagonalInfiniteHeading);
            const Internal::CommandVector diagonalFiniteControl = sampleStraightControl(diagonalFiniteHeading);
            Assert::AreEqual(diagonalFiniteControl.LeftMotorPwm(), diagonalInfiniteControl.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(diagonalFiniteControl.RightMotorPwm(), diagonalInfiniteControl.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveInfiniteTargetPositionCoordinatesRemainFunctional)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            const auto sampleStraightControl =
                [&runtime, &drive, &limits](
                    const Eigen::Vector2f& headingOverride,
                    const Eigen::Vector2f& targetPositionOverride,
                    bool& done) -> Internal::CommandVector
            {
                runtime.Drive().Brake();
                runtime.Drive().ResetControllers();
                Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f));
                drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
                drive.SetLimits(limits);
                drive.StartStraight(NAN, 0.20f, 0.0f, &headingOverride, &targetPositionOverride);

                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
                Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
                return control;
            };

            const Eigen::Vector2f rightHeading(1.0f, 0.0f);
            const Eigen::Vector2f oneInfiniteTarget(INFINITY, 0.0f);
            bool oneInfiniteDone = false;
            (void)sampleStraightControl(rightHeading, oneInfiniteTarget, oneInfiniteDone);
            Assert::IsTrue(oneInfiniteDone);

            const Eigen::Vector2f diagonalHeading(1.0f, 1.0f);
            const Eigen::Vector2f bothInfiniteTarget(INFINITY, INFINITY);
            bool bothInfiniteDone = false;
            (void)sampleStraightControl(diagonalHeading, bothInfiniteTarget, bothInfiniteDone);
            Assert::IsTrue(bothInfiniteDone);
        }

        TEST_METHOD(SharedRuntime_DriveInfiniteTurnReportsDoneWhileContinuingMotion)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, DirectionToYawRad(MazeMap::Up)));

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);
            drive.StartTurn(INFINITY);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsTrue(done);
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);
        }

        TEST_METHOD(SharedRuntime_DriveInfiniteArcReportsDoneWhileContinuingMotion)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, DirectionToYawRad(MazeMap::Up)));

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);
            drive.StartArc(INFINITY, 4.0f);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsTrue(done);
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastLinearCommandMps() > 1.0e-4f);
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);
        }

        TEST_METHOD(SharedRuntime_DriveArcUsesMazeContextWhenDirectInputsAreIllPosed)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            const float cellSizeM = MazeMap::Maze::GetCellDimension();
            Assert::IsTrue(runtime.Estimator().ResetPose(1.5f * cellSizeM, 1.5f * cellSizeM, DirectionToYawRad(MazeMap::Up)));

            MazeMap::Maze& maze = runtime.Maze();
            const MazeMap::CellCoordinates cell(1U, 1U);
            maze.SetWall(cell, MazeMap::Up, MazeMap::WallState::Wall);
            maze.SetWall(cell, MazeMap::Left, MazeMap::WallState::Wall);
            maze.SetWall(cell, MazeMap::Right, MazeMap::WallState::NoWall);
            maze.SetWall(cell, MazeMap::Down, MazeMap::WallState::NoWall);

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            Internal::Drive& drive = runtime.DriveService();
            drive.SetOperationMode(Internal::Drive::OperationMode::Maze);
            drive.SetLimits(limits);
            drive.StartArc(NAN, NAN);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastLinearCommandMps() > 1.0e-6f);
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveMalformedRequestIsNonSticky)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());

            Internal::Drive& drive = runtime.DriveService();
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            drive.SetLimits(limits);

            drive.StartTurn(NAN);

            bool malformedDone = false;
            const Internal::CommandVector malformedControl = drive.GetNextControls(malformedDone);
            Assert::IsFalse(malformedDone);
            Assert::IsTrue(std::isfinite(malformedControl.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(malformedControl.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);

            drive.StartTurn(HALF_PI_F);

            bool recoveredDone = false;
            const Internal::CommandVector recoveredControl = drive.GetNextControls(recoveredDone);
            Assert::IsFalse(recoveredDone);
            Assert::IsTrue(std::isfinite(recoveredControl.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(recoveredControl.RightMotorPwm()));
        }

        TEST_METHOD(SharedRuntime_DriveServicePreservesExplicitStraightDirectionAgainstSignedLimits)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f));
            Internal::Drive& drive = runtime.DriveService();

            const Eigen::Vector2f targetHeadingUnit(1.0f, 0.0f);
            const auto sampleStraightControl =
                [&drive, &targetHeadingUnit](const MotionLimits& limits) -> Internal::CommandVector
            {
                drive.SetLimits(limits);
                drive.StartStraight(0.25f, 0.20f, 0.0f, &targetHeadingUnit, nullptr);

                bool done = false;
                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsFalse(done);
                Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
                Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
                return control;
            };

            MotionLimits positiveLimits{};
            positiveLimits.SetMaxSpeedMps(0.40f);
            positiveLimits.SetAccelMps2(2.0f);
            positiveLimits.SetDecelMps2(2.0f);
            positiveLimits.SetMaxAngularSpeedRadps(6.0f);
            positiveLimits.SetAngularAccelRadps2(30.0f);
            positiveLimits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            MotionLimits signedLimits = positiveLimits;
            signedLimits.SetMaxSpeedMps(-positiveLimits.GetMaxSpeedMps());
            signedLimits.SetAccelMps2(-positiveLimits.GetAccelMps2());
            signedLimits.SetDecelMps2(-positiveLimits.GetDecelMps2());
            signedLimits.SetMaxAngularSpeedRadps(-positiveLimits.GetMaxAngularSpeedRadps());
            signedLimits.SetAngularAccelRadps2(-positiveLimits.GetAngularAccelRadps2());
            signedLimits.SetAngleToleranceRad(-positiveLimits.GetAngleToleranceRad());
            const Internal::CommandVector positiveControl = sampleStraightControl(positiveLimits);
            const Internal::CommandVector signedControl = sampleStraightControl(signedLimits);
            Assert::AreEqual(positiveControl.LeftMotorPwm(), signedControl.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(positiveControl.RightMotorPwm(), signedControl.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_DriveTurnRecoversMazeRightTurnFromDegenerateRequest)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(
                1.5f * Config::kCellSizeM,
                1.5f * Config::kCellSizeM,
                DirectionToYawRad(MazeMap::Up)));

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Up, MazeMap::Wall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Left, MazeMap::Wall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Right, MazeMap::NoWall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Down, MazeMap::NoWall);

            Internal::Drive& drive = runtime.DriveService();
            drive.SetLimits(limits);
            drive.StartTurn(NAN);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Left, MazeMap::NoWall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Right, MazeMap::Wall);
            limits.SetMaxAngularSpeedRadps(-limits.GetMaxAngularSpeedRadps());
            drive.SetLimits(limits);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::IsFalse(drive.IsEffectivelyComplete());
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);
        }

        TEST_METHOD(SharedRuntime_DriveArcRecoversMazeRightTurnFromDegenerateRequest)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(
                1.5f * Config::kCellSizeM,
                1.5f * Config::kCellSizeM,
                DirectionToYawRad(MazeMap::Up)));

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Up, MazeMap::Wall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Left, MazeMap::Wall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Right, MazeMap::NoWall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Down, MazeMap::NoWall);

            Internal::Drive& drive = runtime.DriveService();
            drive.SetLimits(limits);
            drive.StartArc(NAN, NAN);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Left, MazeMap::NoWall);
            runtime.Maze().SetWall(MazeMap::CellCoordinates(1U, 1U), MazeMap::Right, MazeMap::Wall);
            limits.SetMaxAngularSpeedRadps(-limits.GetMaxAngularSpeedRadps());
            drive.SetLimits(limits);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::IsFalse(drive.IsEffectivelyComplete());
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);
            Assert::IsTrue(runtime.Drive().GetLastLinearCommandMps() > 1.0e-4f);
        }

        TEST_METHOD(SharedRuntime_DriveArcChoosesDeterministicMotionWhenAllInputsAreAmbiguous)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, DirectionToYawRad(MazeMap::Up)));

            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.40f);
            limits.SetAccelMps2(2.0f);
            limits.SetDecelMps2(2.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            Internal::Drive& drive = runtime.DriveService();
            drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
            drive.SetLimits(limits);
            drive.StartArc(NAN, NAN);

            bool done = false;
            const Internal::CommandVector control = drive.GetNextControls(done);
            Assert::IsFalse(done);
            Assert::IsFalse(drive.IsEffectivelyComplete());
            Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
            Assert::IsTrue(runtime.Drive().GetLastLinearCommandMps() > 1.0e-4f);
            Assert::IsTrue(runtime.Drive().GetLastAngularCommandRadps() > 1.0e-4f);
        }

        TEST_METHOD(SharedRuntime_DriveServiceRecoversManeuverDirectionFromSignedLimitsWhenSpeedIsAmbiguous)
        {
            Internal::SharedRobotRuntime runtime;
            Assert::IsTrue(runtime.Drive().Begin());
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f));

            Internal::Drive& drive = runtime.DriveService();
            const auto sampleRecoveredLinearCommandMps =
                [&runtime, &drive](const MotionLimits& limits) -> float
            {
                drive.SetLimits(limits);
                MazeMap::ManeuverInstance maneuver(MazeMap::S1, MazeMap::DirectionalLocation(), 0.0f, 0.0f);
                drive.StartManeuver(maneuver);
                MotionLimits changedLimits = limits;
                changedLimits.SetMaxSpeedMps(-changedLimits.GetMaxSpeedMps());
                drive.SetLimits(changedLimits);

                bool done = false;
                const Internal::CommandVector control = drive.GetNextControls(done);
                Assert::IsFalse(done);
                Assert::IsTrue(std::isfinite(control.LeftMotorPwm()));
                Assert::IsTrue(std::isfinite(control.RightMotorPwm()));
                return runtime.Drive().GetLastLinearCommandMps();
            };

            MotionLimits forwardLimits{};
            forwardLimits.SetMaxSpeedMps(0.40f);
            forwardLimits.SetAccelMps2(2.0f);
            forwardLimits.SetDecelMps2(2.0f);
            forwardLimits.SetMaxAngularSpeedRadps(6.0f);
            forwardLimits.SetAngularAccelRadps2(30.0f);
            forwardLimits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            MotionLimits reverseHintLimits = forwardLimits;
            reverseHintLimits.SetMaxSpeedMps(-forwardLimits.GetMaxSpeedMps());
            const float forwardCommandMps = sampleRecoveredLinearCommandMps(forwardLimits);
            const float reverseCommandMps = sampleRecoveredLinearCommandMps(reverseHintLimits);
            Assert::IsTrue(forwardCommandMps > 1.0e-6f);
            Assert::IsTrue(reverseCommandMps < -1.0e-6f);
            Assert::AreEqual(std::fabs(forwardCommandMps), std::fabs(reverseCommandMps), 1.0e-6f);
        }

        TEST_METHOD(SharedRuntime_SingletonIsStable)
        {
            Internal::SharedRobotRuntime& first = Internal::GetSharedRobotRuntime();
            Internal::SharedRobotRuntime& second = Internal::GetSharedRobotRuntime();

            Assert::IsTrue(&first == &second);
        }

        TEST_METHOD(SharedRuntime_UtilityLoggingUsesRuntimeOwnedDataAndTextLogs)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()) == dataPath);
                Assert::IsTrue(runtime.WriteUtilityDataLogMetadata("mode", "unit_test"));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "unit_test"));

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));

                row.seq = 7U;
                row.value = 2.5f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));
                Assert::IsTrue(runtime.WriteTextLogPhase(3U, 456U, "phase_a"));
                Assert::IsTrue(runtime.WriteTextLogEntry(789U, "summary", "hello"));
                Assert::IsTrue(runtime.FlushUtilityDataLog());
                runtime.FlushTextLog();
                Assert::IsTrue(runtime.CloseUtilityDataLog());
                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()).empty());
            }

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            Assert::IsTrue(sidecarText.find("mode=unit_test\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("control_log_file=logging.txt\n") != std::string::npos);

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("mode=unit_test") != std::string::npos);
            Assert::IsTrue(controlText.find("phase_id=3;name=phase_a") != std::string::npos);
            Assert::IsTrue(controlText.find("summary: hello") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_TextLogRemainsUsableAcrossDataLogSessions)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_handoff");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("before_data_log"));
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.TextLogIsOpen());

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));
                Assert::IsTrue(runtime.TextLogIsOpen());

                row.seq = 11U;
                row.value = 4.0f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));
                Assert::IsTrue(runtime.AppendTextLogLine("during_data_log"));
                Assert::IsTrue(runtime.FlushUtilityDataLog());
                Assert::IsTrue(runtime.CloseUtilityDataLog());
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.AppendTextLogLine("after_data_log"));
                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("before_data_log") != std::string::npos);
            Assert::IsTrue(controlText.find("during_data_log") != std::string::npos);
            Assert::IsTrue(controlText.find("after_data_log") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_UtilityDataMetadataWritesDuplicatesInCallOrder)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_error");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(std::string(runtime.LastRuntimeLogError()).empty());
                Assert::IsTrue(runtime.WriteUtilityDataLogMetadata("mode", "first"));
                Assert::IsTrue(runtime.WriteUtilityDataLogMetadata("mode", "second"));
                Assert::IsTrue(std::string(runtime.LastRuntimeLogError()).empty());
            }

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            const std::size_t firstMetadata = sidecarText.find("mode=first\n");
            const std::size_t secondMetadata = sidecarText.find("mode=second\n");
            Assert::IsTrue(firstMetadata != std::string::npos);
            Assert::IsTrue(secondMetadata != std::string::npos);
            Assert::IsTrue(firstMetadata < secondMetadata);

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("data_log_metadata_failed:") == std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_ServiceUtilityDataLog_LeavesSubsectorTextBufferedUntilFlush)
        {
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("subsector_text"));
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.ServiceUtilityDataLog());
                Assert::IsTrue(ReadAllBytes(controlPath).empty());

                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("subsector_text") != std::string::npos);

            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FlushTextLogWritesExactBufferedBytesWithoutPadding)
        {
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            const std::string expected = "alpha\nbeta\ngamma\n";
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("alpha"));
                Assert::IsTrue(runtime.AppendTextLogLine("beta"));
                Assert::IsTrue(runtime.AppendTextLogLine("gamma"));
                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText == expected);
            Assert::IsTrue(controlText.find('\0') == std::string::npos);

            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FailActiveModeClosesAndDisablesRuntimeLogs)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_fault");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string retryDataPath = CreateTempPath("codex_shared_runtime_retry");
            const std::string retrySidecarPath = ReplaceExtension(retryDataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;
                FaultCallbackState callbackState{};

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "fault_shutdown"));
                Assert::IsTrue(runtime.RegisterModeFaultHandler(&CaptureFaultCallback, &callbackState, "fault_shutdown"));
                Assert::ExpectException<std::logic_error>(
                    [&runtime, &callbackState]()
                    {
                        (void)runtime.RegisterModeFaultHandler(&CaptureFaultCallback, &callbackState, "fault_shutdown_duplicate");
                    });

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));

                row.seq = 1U;
                row.value = 9.5f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));

                // This eats execution in an infinite loop.
                // (runtime.FailActiveMode("unit_test_fault"));

                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()).empty());
                Assert::IsFalse(runtime.TextLogIsOpen());
                Assert::IsTrue(callbackState.count == 1U);
                Assert::IsTrue(std::string(callbackState.reason) == "unit_test_fault");
                Assert::IsFalse(runtime.WriteTextLogEntry(10U, "summary", "after_fault"));
                Assert::IsFalse(runtime.OpenUtilityDataLogFile(retryDataPath.c_str()));
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("mode=fault_shutdown") != std::string::npos);
            Assert::IsTrue(controlText.find("fault: unit_test_fault") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(retryDataPath.c_str());
            std::remove(retrySidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FaultCallbackCanStreamTextBeyondQueueCapacityByFlushingIncrementally)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_fault_dump");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;
                StreamingFaultCallbackState callbackState{};
                callbackState.runtime = &runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "fault_dump_stream"));
                Assert::IsTrue(runtime.RegisterModeFaultHandler(&StreamLongFaultDump, &callbackState, "fault_dump_stream"));

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));
                row.seq = 2U;
                row.value = 1.0f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));

                // This eats execution in an infinite loop.
                // (runtime.FailActiveMode("stream_fault_dump"));
                Assert::AreEqual(32U, callbackState.linesWritten);
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("fault: stream_fault_dump") != std::string::npos);
            Assert::IsTrue(controlText.find("ukf_dump_test: reason=stream_fault_dump;line=0;") != std::string::npos);
            Assert::IsTrue(controlText.find("ukf_dump_test: reason=stream_fault_dump;line=31;") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }
    };
}


