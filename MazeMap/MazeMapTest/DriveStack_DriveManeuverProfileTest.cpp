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
#include <sstream>
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

        void PublishSyntheticRuntimeProgress(
            Internal::SharedRobotRuntime& runtime,
            const float encoderProgressM,
            const float yawRad,
            const float forwardMps,
            const float yawRateRadps)
        {
            SensorSnapshot snapshot{};
            snapshot.SetRawYawRateRadps(yawRateRadps);
            snapshot.SetYawRateRadps(yawRateRadps);
            snapshot.SetEncoderDistancesM(encoderProgressM, encoderProgressM);
            MazeMap::EncoderObs encoderObservation{};
            encoderObservation.SetLeftDistanceDeltaM(0.0f);
            encoderObservation.SetRightDistanceDeltaM(0.0f);
            encoderObservation.SetLeftVelocityMps(forwardMps);
            encoderObservation.SetRightVelocityMps(forwardMps);
            snapshot.SetEncoderObservation(encoderObservation, true);

            VehicleState& state = runtime.RuntimeState();
            state.SetSensorSnapshot(snapshot);
            state.SetHeading(yawRad);
            state.SetForwardVelocity(forwardMps);
            state.SetYawRate(yawRateRadps);
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
                PublishSyntheticRuntimeProgress(
                    runtime,
                    progressM,
                    expectedPoint.Theta,
                    kSmoothSpeedMps,
                    expectedPoint.Omega);
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
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=done"
                << L"\nprogress_m=0"
                << L"\nexpected=false"
                << L"\nactual=" << sample.done;

            Assert::IsFalse(sample.done, message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedForwardMpsIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedForwardMps"
                << L"\nprogress_m=0"
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(sample.telemetry.requestedForwardMps),
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedYawRateRadpsIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRateRadps"
                << L"\nprogress_m=0"
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(sample.telemetry.requestedYawRateRadps),
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestedYawRadIsFinite)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRad"
                << L"\nprogress_m=0"
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(sample.telemetry.requestedYawRad),
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsCruiseForwardMps)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedForwardMps"
                << L"\nprogress_m=0"
                << L"\nexpected=" << kStraightCruiseMps
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                kStraightCruiseMps,
                sample.telemetry.requestedForwardMps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsZeroYawRate)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRateRadps"
                << L"\nprogress_m=0"
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedYawRateRadps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Initial_RequestsZeroYawTarget)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleInitial();
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRad"
                << L"\nprogress_m=0"
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Completion_CompletesFromEncoderProgress)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();
            const float completionProgressM =
                scenario.maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=done"
                << L"\nprogress_m=" << completionProgressM
                << L"\nexpected=true"
                << L"\nactual=" << sample.done;

            Assert::IsTrue(sample.done, message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Completion_RequestsZeroYawRate)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();
            const float completionProgressM =
                scenario.maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRateRadps"
                << L"\nprogress_m=" << completionProgressM
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedYawRateRadps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(StraightManeuver_Completion_RequestsZeroYawTarget)
        {
            StraightScenario scenario;
            const ProfileSample sample = scenario.SampleCompletion();
            const float completionProgressM =
                scenario.maneuver.GetTravelDistanceMeters(Config::kCellSizeM) + Config::kDistanceToleranceM;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S4"
                << L"\nfield=requestedYawRad"
                << L"\nprogress_m=" << completionProgressM
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_Initial_RemainsActive)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=done"
                << L"\nyaw_rad=0"
                << L"\nexpected=false"
                << L"\nactual=" << sample.done;

            Assert::IsFalse(sample.done, message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedForwardMps"
                << L"\nyaw_rad=0"
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedForwardMps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedYawRateRadps"
                << L"\nyaw_rad=0"
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_Initial_RequestsNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            const float expectedYawRad = 90.0f * DEG_TO_RAD_F;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedYawRad"
                << L"\nyaw_rad=0"
                << L"\nexpected=" << expectedYawRad
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedYawRad,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RemainsActive)
        {
            InPlaceScenario scenario(IP90);
            const float runtimeYawRad = 45.0f * DEG_TO_RAD_F;
            const ProfileSample sample = scenario.SampleAtYaw(runtimeYawRad);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=done"
                << L"\nyaw_rad=" << runtimeYawRad
                << L"\nexpected=false"
                << L"\nactual=" << sample.done;

            Assert::IsFalse(sample.done, message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90);
            const float runtimeYawRad = 45.0f * DEG_TO_RAD_F;
            const ProfileSample sample = scenario.SampleAtYaw(runtimeYawRad);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedForwardMps"
                << L"\nyaw_rad=" << runtimeYawRad
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedForwardMps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_RequestsClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90);
            const float runtimeYawRad = 45.0f * DEG_TO_RAD_F;
            const ProfileSample sample = scenario.SampleAtYaw(runtimeYawRad);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedYawRateRadps"
                << L"\nyaw_rad=" << runtimeYawRad
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_HalfYaw_KeepsNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90);
            const float runtimeYawRad = 45.0f * DEG_TO_RAD_F;
            const ProfileSample sample = scenario.SampleAtYaw(runtimeYawRad);
            const float expectedYawRad = 90.0f * DEG_TO_RAD_F;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=requestedYawRad"
                << L"\nyaw_rad=" << runtimeYawRad
                << L"\nexpected=" << expectedYawRad
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedYawRad,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceManeuver_Completion_CompletesFromRuntimeYaw)
        {
            InPlaceScenario scenario(IP90);
            const float runtimeYawRad = 90.0f * DEG_TO_RAD_F;
            const ProfileSample sample = scenario.SampleAtYaw(runtimeYawRad);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90"
                << L"\nfield=done"
                << L"\nyaw_rad=" << runtimeYawRad
                << L"\nexpected=true"
                << L"\nactual=" << sample.done;

            Assert::IsTrue(sample.done, message.str().c_str());
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RemainsActive)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90_M"
                << L"\nfield=done"
                << L"\nyaw_rad=0"
                << L"\nexpected=false"
                << L"\nactual=" << sample.done;

            Assert::IsFalse(sample.done, message.str().c_str());
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsZeroForwardMps)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90_M"
                << L"\nfield=requestedForwardMps"
                << L"\nyaw_rad=0"
                << L"\nexpected=0"
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                0.0f,
                sample.telemetry.requestedForwardMps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsCounterClockwiseYawRate)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90_M"
                << L"\nfield=requestedYawRateRadps"
                << L"\nyaw_rad=0"
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceMirroredManeuver_Initial_RequestsNegativeNinetyDegreeYawTarget)
        {
            InPlaceScenario scenario(IP90_M);
            const ProfileSample sample = scenario.SampleAtYaw(0.0f);
            const float expectedYawRad = -90.0f * DEG_TO_RAD_F;
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=IP90_M"
                << L"\nfield=requestedYawRad"
                << L"\nyaw_rad=0"
                << L"\nexpected=" << expectedYawRad
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedYawRad,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

#define DEFINE_SMOOTH_PROFILE_SAMPLE_TESTS(SUFFIX, FRACTION, EXPECTED_DONE) \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_DoneStateMatchesProgress) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            std::wstringstream message; \
            message << L"DRV10_PROFILE_REQUEST" \
                << L"\ncode=S90SS" \
                << L"\nfield=done" \
                << L"\nprogress_m=" << progressM \
                << L"\nexpected=" << ((EXPECTED_DONE) ? L"true" : L"false") \
                << L"\nactual=" << sample.done; \
            Assert::IsTrue( \
                sample.done == (EXPECTED_DONE), \
                message.str().c_str()); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_ForwardMpsMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            std::wstringstream message; \
            message << L"DRV10_PROFILE_REQUEST" \
                << L"\ncode=S90SS" \
                << L"\nfield=requestedForwardMps" \
                << L"\nprogress_m=" << progressM \
                << L"\nexpected=" << expectedPoint.Velocity \
                << L"\nactual=" << sample.telemetry.requestedForwardMps \
                << L"\ntolerance=" << kTelemetryTolerance; \
            Assert::AreEqual( \
                expectedPoint.Velocity, \
                sample.telemetry.requestedForwardMps, \
                kTelemetryTolerance, \
                message.str().c_str()); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_YawRateMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            std::wstringstream message; \
            message << L"DRV10_PROFILE_REQUEST" \
                << L"\ncode=S90SS" \
                << L"\nfield=requestedYawRateRadps" \
                << L"\nprogress_m=" << progressM \
                << L"\nexpected=" << expectedPoint.Omega \
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps \
                << L"\ntolerance=" << kTelemetryTolerance; \
            Assert::AreEqual( \
                expectedPoint.Omega, \
                sample.telemetry.requestedYawRateRadps, \
                kTelemetryTolerance, \
                message.str().c_str()); \
        } \
        TEST_METHOD(SmoothManeuver_##SUFFIX##_YawTargetMatchesCatalogPoint) \
        { \
            SmoothScenario scenario(S90SS); \
            ManeuverPoint expectedPoint{}; \
            float progressM = 0.0f; \
            const ProfileSample sample = scenario.SampleAtProgressFraction(FRACTION, expectedPoint, progressM); \
            std::wstringstream message; \
            message << L"DRV10_PROFILE_REQUEST" \
                << L"\ncode=S90SS" \
                << L"\nfield=requestedYawRad" \
                << L"\nprogress_m=" << progressM \
                << L"\nexpected=" << expectedPoint.Theta \
                << L"\nactual=" << sample.telemetry.requestedYawRad \
                << L"\ntolerance=" << kTelemetryTolerance; \
            Assert::AreEqual( \
                expectedPoint.Theta, \
                sample.telemetry.requestedYawRad, \
                kTelemetryTolerance, \
                message.str().c_str()); \
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
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=done"
                << L"\nprogress_m=" << progressM
                << L"\nexpected=false"
                << L"\nactual=" << sample.done;

            Assert::IsFalse(sample.done, message.str().c_str());
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_ForwardMpsMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=requestedForwardMps"
                << L"\nprogress_m=" << progressM
                << L"\nexpected=" << expectedPoint.Velocity
                << L"\nactual=" << sample.telemetry.requestedForwardMps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedPoint.Velocity,
                sample.telemetry.requestedForwardMps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawRateMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=requestedYawRateRadps"
                << L"\nprogress_m=" << progressM
                << L"\nexpected=" << expectedPoint.Omega
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedPoint.Omega,
                sample.telemetry.requestedYawRateRadps,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawTargetMatchesCatalogPoint)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=requestedYawRad"
                << L"\nprogress_m=" << progressM
                << L"\nexpected=" << expectedPoint.Theta
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ntolerance=" << kTelemetryTolerance;

            Assert::AreEqual(
                expectedPoint.Theta,
                sample.telemetry.requestedYawRad,
                kTelemetryTolerance,
                message.str().c_str());
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawRateIsNegative)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=requestedYawRateRadps"
                << L"\nprogress_m=" << progressM
                << L"\nactual=" << sample.telemetry.requestedYawRateRadps
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                sample.telemetry.requestedYawRateRadps < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(SmoothMirroredManeuver_Midpoint_YawTargetIsNegative)
        {
            SmoothScenario scenario(S90SS_M);
            ManeuverPoint expectedPoint{};
            float progressM = 0.0f;
            const ProfileSample sample = scenario.SampleAtProgressFraction(0.50f, expectedPoint, progressM);
            std::wstringstream message;
            message << L"DRV10_PROFILE_REQUEST"
                << L"\ncode=S90SS_M"
                << L"\nfield=requestedYawRad"
                << L"\nprogress_m=" << progressM
                << L"\nactual=" << sample.telemetry.requestedYawRad
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                sample.telemetry.requestedYawRad < 0.0f,
                message.str().c_str());
        }
    };
}
