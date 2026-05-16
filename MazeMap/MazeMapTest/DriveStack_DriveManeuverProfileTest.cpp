#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\DirectionalLocation.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\DriveTelemetry.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\Maneuver.h"
#include "..\MazeMap\ManeuverInstance.h"
#include "..\MazeMap\MazeLocation.h"
#include "..\MazeMap\MotionLimits.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    namespace
    {
        constexpr float kProfileDtSeconds = 0.001f;
        constexpr float kStraightCruiseMps = 0.40f;
        constexpr float kSmoothSpeedMps = 0.25f;
        constexpr float kForwardAccelLimitMps2 = 3.0f;
        constexpr float kAngularAccelLimitRadps2 = 40.0f;
        constexpr float kMaxYawRateRadps = 50.0f;
        constexpr float kTelemetryTolerance = 1.0e-4f;

        MotionLimits MakeProfileLimits() noexcept
        {
            MotionLimits limits{};
            limits.SetMaxSpeedMps(kStraightCruiseMps);
            limits.SetAccelMps2(kForwardAccelLimitMps2);
            limits.SetDecelMps2(kForwardAccelLimitMps2);
            limits.SetMaxAngularSpeedRadps(kMaxYawRateRadps);
            limits.SetAngularAccelRadps2(kAngularAccelLimitRadps2);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            return limits;
        }

        DirectionalLocation MakeProfileStart() noexcept
        {
            return DirectionalLocation(MazeLocation(15, 15), Direction::Up);
        }

        std::wstring CodeName(const ManeuverCode code)
        {
            switch (code)
            {
            case S4: return L"S4";
            case IP90: return L"IP90";
            case IP90_M: return L"IP90_M";
            case S90SS: return L"S90SS";
            case S90SS_M: return L"S90SS_M";
            default:
                return std::wstring(L"code_") + std::to_wstring(static_cast<unsigned int>(code));
            }
        }

        std::wstring ProfileMessage(
            const wchar_t* label,
            const ManeuverCode code,
            const wchar_t* field,
            const float progressM,
            const float expected,
            const float actual)
        {
            return
                std::wstring(label) +
                L" code=" + CodeName(code) +
                L" field=" + field +
                L" progress_m=" + std::to_wstring(progressM) +
                L" expected=" + std::to_wstring(expected) +
                L" actual=" + std::to_wstring(actual);
        }

        void AssertNearProfile(
            const ManeuverCode code,
            const wchar_t* field,
            const float progressM,
            const float expected,
            const float actual,
            const float tolerance = kTelemetryTolerance)
        {
            const std::wstring message =
                ProfileMessage(L"DRV10_PROFILE_REQUEST", code, field, progressM, expected, actual);
            Assert::AreEqual(expected, actual, tolerance, message.c_str());
        }

        void AssertFiniteProfile(
            const ManeuverCode code,
            const wchar_t* field,
            const float progressM,
            const float actual)
        {
            const std::wstring message =
                ProfileMessage(L"DRV10_PROFILE_REQUEST", code, field, progressM, 0.0f, actual);
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        void PublishSyntheticRuntimeProgress(
            Internal::SharedRobotRuntime& runtime,
            const float encoderProgressM,
            const float yawRad,
            const float forwardMps,
            const float yawRateRadps)
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = yawRateRadps;
            snapshot.gyroRadps = yawRateRadps;
            snapshot.leftEncoderDistanceM = encoderProgressM;
            snapshot.rightEncoderDistanceM = encoderProgressM;
            snapshot.encoderObservationValid = true;
            snapshot.encoderObservation.leftDistanceDeltaM = 0.0f;
            snapshot.encoderObservation.rightDistanceDeltaM = 0.0f;
            snapshot.encoderObservation.leftVelocityMps = forwardMps;
            snapshot.encoderObservation.rightVelocityMps = forwardMps;

            VehicleState& state = runtime.RuntimeState();
            state.SetSensorSnapshot(snapshot);
            state.SetOrientation(yawRad);
            state.SetVelocity(forwardMps);
            state.SetRotationalVelocity(yawRateRadps);
        }

        struct ProfileSample final
        {
            bool done = false;
            DriveTelemetry telemetry{};
        };

        ProfileSample SampleProfile(
            Internal::SharedRobotRuntime& runtime,
            Internal::Drive& drive)
        {
            ProfileSample sample{};
            (void)drive.GetNextControls(sample.done);
            sample.telemetry = runtime.DriveBase().LastTelemetry();
            return sample;
        }

        void ArmManeuver(
            Internal::SharedRobotRuntime& runtime,
            const MotionLimits& limits,
            const ManeuverInstance& maneuver,
            const float initialForwardMps = 0.0f)
        {
            runtime.DriveBase().ClearCommandEvidence();
            (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);
            PublishSyntheticRuntimeProgress(runtime, 0.0f, 0.0f, initialForwardMps, 0.0f);
            Internal::Drive& drive = runtime.DriveService();
            drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
            drive.SetLimits(limits);
            drive.StartManeuver(maneuver);
        }

        struct StraightScenario final
        {
            ManeuverInstance maneuver{ S4, MakeProfileStart(), 0.0f, 0.0f };
            Internal::SharedRobotRuntime runtime{ kProfileDtSeconds };

            ProfileSample SampleInitial()
            {
                ArmManeuver(runtime, MakeProfileLimits(), maneuver);
                return SampleProfile(runtime, runtime.DriveService());
            }

            ProfileSample SampleCompletion()
            {
                ArmManeuver(runtime, MakeProfileLimits(), maneuver);
                const float completionProgressM =
                    maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;
                PublishSyntheticRuntimeProgress(runtime, completionProgressM, 0.0f, 0.0f, 0.0f);
                return SampleProfile(runtime, runtime.DriveService());
            }
        };

        struct InPlaceScenario final
        {
            ManeuverCode code = IP90;
            ManeuverInstance maneuver{ code, MakeProfileStart(), 0.0f, 0.0f };
            Internal::SharedRobotRuntime runtime{ kProfileDtSeconds };

            explicit InPlaceScenario(const ManeuverCode maneuverCode)
                : code(maneuverCode)
                , maneuver(maneuverCode, MakeProfileStart(), 0.0f, 0.0f)
            {
            }

            ProfileSample SampleAtYaw(const float yawRad)
            {
                ArmManeuver(runtime, MakeProfileLimits(), maneuver);
                PublishSyntheticRuntimeProgress(runtime, 0.0f, yawRad, 0.0f, 0.0f);
                return SampleProfile(runtime, runtime.DriveService());
            }
        };

        struct SmoothScenario final
        {
            ManeuverCode code = S90SS;
            ManeuverInstance maneuver{ code, MakeProfileStart(), kSmoothSpeedMps, kSmoothSpeedMps };
            Internal::SharedRobotRuntime runtime{ kProfileDtSeconds };

            explicit SmoothScenario(const ManeuverCode maneuverCode)
                : code(maneuverCode)
                , maneuver(maneuverCode, MakeProfileStart(), kSmoothSpeedMps, kSmoothSpeedMps)
            {
            }

            ProfileSample SampleAtProgressFraction(
                const float fraction,
                ManeuverPoint& expectedPoint,
                float& progressM)
            {
                ArmManeuver(runtime, MakeProfileLimits(), maneuver, kSmoothSpeedMps);
                progressM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM) * fraction;
                (void)maneuver.TryGetManeuverPoint(
                    progressM,
                    kSmoothSpeedMps,
                    expectedPoint,
                    Config::kCellSizeM);
                PublishSyntheticRuntimeProgress(runtime, progressM, expectedPoint.Theta, kSmoothSpeedMps, expectedPoint.Omega);
                return SampleProfile(runtime, runtime.DriveService());
            }
        };
    }

    TEST_CLASS(DriveStack_DriveManeuverProfileTest)
    {
    public:
        TEST_METHOD(StraightManeuver_Initial_RemainsActive)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            Assert::IsFalse(sample.done, L"DRV10_PROFILE_REQUEST code=S4 expected initial straight maneuver to remain active");
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedForwardMpsIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertFiniteProfile(S4, L"requestedForwardMps", 0.0f, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedYawRateRadpsIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertFiniteProfile(S4, L"requestedYawRateRadps", 0.0f, sample.telemetry.requestedYawRateRadps);
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedYawRadIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertFiniteProfile(S4, L"requestedYawRad", 0.0f, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsCruiseForwardMps)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertNearProfile(S4, L"requestedForwardMps", 0.0f, kStraightCruiseMps, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsZeroYawRate)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertNearProfile(S4, L"requestedYawRateRadps", 0.0f, 0.0f, sample.telemetry.requestedYawRateRadps);
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsZeroYawTarget)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();

            AssertNearProfile(S4, L"requestedYawRad", 0.0f, 0.0f, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(StraightManeuver_Completion_CompletesFromEncoderProgress)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();

            Assert::IsTrue(sample.done, L"DRV10_PROFILE_REQUEST code=S4 expected encoder-progress completion");
        }

        TEST_METHOD(StraightManeuver_Completion_RequestsZeroYawRate)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();
            const float completionProgressM =
                scenario.maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;

            AssertNearProfile(S4, L"requestedYawRateRadps", completionProgressM, 0.0f, sample.telemetry.requestedYawRateRadps);
        }

        TEST_METHOD(StraightManeuver_Completion_RequestsZeroYawTarget)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();
            const float completionProgressM =
                scenario.maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;

            AssertNearProfile(S4, L"requestedYawRad", completionProgressM, 0.0f, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(InPlaceManeuver_Initial_RemainsActive)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            Assert::IsFalse(sample.done, L"DRV10_PROFILE_REQUEST code=IP90 expected initial in-place turn to remain active");
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            AssertNearProfile(IP90, L"requestedForwardMps", 0.0f, 0.0f, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps > 0.0f,
                L"DRV10_PROFILE_REQUEST code=IP90 field=requestedYawRateRadps expected clockwise-positive yaw request");
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            AssertNearProfile(IP90, L"requestedYawRad", 0.0f, 90.0f * DEG_TO_RAD_F, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RemainsActive)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(45.0f * DEG_TO_RAD_F);

            Assert::IsFalse(sample.done, L"DRV10_PROFILE_REQUEST code=IP90 expected half-yaw sample to remain active");
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(45.0f * DEG_TO_RAD_F);

            AssertNearProfile(IP90, L"requestedForwardMps", 0.0f, 0.0f, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RequestsClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(45.0f * DEG_TO_RAD_F);

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps > 0.0f,
                L"DRV10_PROFILE_REQUEST code=IP90 field=requestedYawRateRadps expected positive yaw through half turn");
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_KeepsNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(45.0f * DEG_TO_RAD_F);

            AssertNearProfile(IP90, L"requestedYawRad", 0.0f, 90.0f * DEG_TO_RAD_F, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(InPlaceManeuver_Completion_CompletesFromRuntimeYaw)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(90.0f * DEG_TO_RAD_F);

            Assert::IsTrue(sample.done, L"DRV10_PROFILE_REQUEST code=IP90 expected runtime-yaw completion");
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RemainsActive)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            Assert::IsFalse(sample.done, L"DRV10_PROFILE_REQUEST code=IP90_M expected initial mirrored turn to remain active");
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            AssertNearProfile(IP90_M, L"requestedForwardMps", 0.0f, 0.0f, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsCounterClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps < 0.0f,
                L"DRV10_PROFILE_REQUEST code=IP90_M field=requestedYawRateRadps expected mirrored yaw request to be negative");
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsNegativeNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);

            AssertNearProfile(IP90_M, L"requestedYawRad", 0.0f, -90.0f * DEG_TO_RAD_F, sample.telemetry.requestedYawRad);
        }

#define DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(SUFFIX, FRACTION, EXPECTED_DONE) \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_DoneStateMatchesProgress) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            const std::wstring message = std::wstring(L"DRV10_PROFILE_REQUEST code=S90SS progress_m=") + std::to_wstring(progressM) + L" expected_done=" + ((EXPECTED_DONE) ? L"true" : L"false"); \
            Assert::IsTrue(sample.done == (EXPECTED_DONE), message.c_str()); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_ForwardMpsMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            AssertNearProfile(S90SS, L"requestedForwardMps", progressM, expectedPoint.Velocity, sample.telemetry.requestedForwardMps); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_YawRateMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            AssertNearProfile(S90SS, L"requestedYawRateRadps", progressM, expectedPoint.Omega, sample.telemetry.requestedYawRateRadps); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_YawTargetMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            AssertNearProfile(S90SS, L"requestedYawRad", progressM, expectedPoint.Theta, sample.telemetry.requestedYawRad); \
        }

        DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(Start, 0.0f, false)
        DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(Quarter, 0.25f, false)
        DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(Midpoint, 0.50f, false)
        DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(ThreeQuarter, 0.75f, false)
        DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(End, 1.0f, true)

#undef DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_RemainsActive)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            Assert::IsFalse(sample.done, L"DRV10_PROFILE_REQUEST code=S90SS_M expected midpoint to remain active");
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_ForwardMpsMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            AssertNearProfile(S90SS_M, L"requestedForwardMps", progressM, expectedPoint.Velocity, sample.telemetry.requestedForwardMps);
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawRateMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            AssertNearProfile(S90SS_M, L"requestedYawRateRadps", progressM, expectedPoint.Omega, sample.telemetry.requestedYawRateRadps);
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawTargetMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            AssertNearProfile(S90SS_M, L"requestedYawRad", progressM, expectedPoint.Theta, sample.telemetry.requestedYawRad);
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawRateIsNegative)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps < 0.0f,
                L"DRV10_PROFILE_REQUEST code=S90SS_M field=requestedYawRateRadps expected mirrored smooth yaw request to be negative");
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawTargetIsNegative)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);

            Assert::IsTrue(
                sample.telemetry.requestedYawRad < 0.0f,
                L"DRV10_PROFILE_REQUEST code=S90SS_M field=requestedYawRad expected mirrored smooth yaw target to be negative");
        }
    };
}
