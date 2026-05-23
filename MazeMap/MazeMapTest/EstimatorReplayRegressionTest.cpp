#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorFilterTestSupport.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        struct LoggedOpenFloorPoseJumpSample
        {
            float dtSeconds = 0.0f;
            float poseXM = 0.0f;
            float poseYM = 0.0f;
            float poseYawRad = 0.0f;
            float measuredLinearSpeedMps = 0.0f;
            float measuredAngularSpeedRadps = 0.0f;
            float leftDriveCommand = 0.0f;
            float rightDriveCommand = 0.0f;
            int32_t leftEncoderCount = 0;
            int32_t rightEncoderCount = 0;
            float leftEncoderWheelSpeedRadps = 0.0f;
            float rightEncoderWheelSpeedRadps = 0.0f;
            float gyroRawRadps = 0.0f;
            float gyroBiasRadps = 0.0f;
            float accelBodyRightMps2 = 0.0f;
            float accelBodyForwardMps2 = 0.0f;
        };

        // D:\open_floor_main.csv rows 10714..10721 from the April 8, 2026 run_id=ofm_13260055 launch fault.
        constexpr LoggedOpenFloorPoseJumpSample kLatestLoggedOpenFloorPoseJumpWindow[] = {
            {
                0.001846f, 0.2263727039f, 0.2257606983f, 0.0033714205f,
                0.0031810037f, -0.0251083225f,
                0.39405635f, -0.4572624862f,
                144, 115,
                0.9068602324f, 0.9068602324f,
                -0.0280998014f, -0.0029914798f,
                0.0326640718f, 0.1455999613f
            },
            {
                0.002804f, 0.2484023273f, 0.2257129699f, 0.0042725629f,
                -0.0062825959f, -0.0507646613f,
                0.0f, 0.0f,
                156, 97,
                1.0090416670f, -0.5045208335f,
                -0.0537561402f, -0.0029914798f,
                0.1570908427f, 0.0690296590f
            },
            {
                0.001036f, 0.2408000827f, 0.2257105708f, 0.0040618582f,
                -0.0028340411f, -0.0580950454f,
                -0.18f, -0.18f,
                155, 98,
                1.8268189430f, -2.8232655525f,
                -0.0610865243f, -0.0029914798f,
                -0.0486919172f, 0.3585612178f
            },
            {
                0.002741f, 0.2401545793f, 0.2257211059f, 0.0032611242f,
                0.0032134983f, -0.1558334827f,
                -0.18f, -0.18f,
                150, 104,
                -0.8989821672f, 0.4494910836f,
                -0.1588249654f, -0.0029914798f,
                -0.5057210326f, -0.3951779306f
            },
            {
                0.001017f, 0.2526942492f, 0.2256730497f, 0.0032170448f,
                0.0115479520f, -0.1265119463f,
                -0.18f, -0.18f,
                151, 107,
                -0.6795662045f, 1.1892408133f,
                -0.1295034289f, -0.0029914798f,
                -0.4315435290f, -0.8928850293f
            },
            {
                0.001739f, 0.2828577459f, 0.2256010771f, 0.0024208522f,
                0.0084418245f, -0.1094077229f,
                -0.18f, -0.18f,
                156, 108,
                0.9157773852f, 0.9157773852f,
                -0.1123992056f, -0.0029914798f,
                0.0661635697f, -1.0173118114f
            },
            {
                0.001016f, 0.3010919690f, 0.2255954593f, -0.0043987799f,
                0.0057796580f, -0.0751992688f,
                -0.18f, -0.18f,
                158, 108,
                1.0711276531f, 0.2677819133f,
                -0.0781907514f, -0.0029914798f,
                0.1259841472f, -1.0819180012f
            },
            {
                0.001796f, 0.5462470055f, 0.2249572873f, 0.0019957498f,
                0.0f, -0.0568733141f,
                -0.18f, -0.18f,
                159, 107,
                0.9166785479f, 0.0f,
                -0.0598647930f, -0.0029914798f,
                0.3317668736f, -1.1082390547f
            }
        };
    }

    TEST_CLASS(EstimatorReplayRegressionTest)
    {
    public:
        TEST_METHOD(EstimatorReplayOfLatestOpenFloorLaunchLogDoesNotProduceXPoseBoundaryJump)
        {
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            constexpr int sampleCount =
                static_cast<int>(sizeof(kLatestLoggedOpenFloorPoseJumpWindow) / sizeof(kLatestLoggedOpenFloorPoseJumpWindow[0]));
            const LoggedOpenFloorPoseJumpSample& first = kLatestLoggedOpenFloorPoseJumpWindow[0];

            EstimatorTestRuntime runtime;
            runtime.runtimeState.SetGyroBiasZ(first.gyroBiasRadps);
            runtime.runtimeState.SetWheelSpeedLeft(first.leftEncoderWheelSpeedRadps);
            runtime.runtimeState.SetWheelSpeedRight(first.rightEncoderWheelSpeedRadps);
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = first.poseXM;
            initialState(1) = first.poseYM;
            initialState(2) = NormalizeAngle(first.poseYawRad);
            initialState(3) = first.measuredLinearSpeedMps;
            initialState(4) = 0.0f;
            initialState(5) = first.gyroRawRadps - first.gyroBiasRadps;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Assert::IsTrue(core.reset(initialState, Estimator::BuildDefaultInitialCovariance()));

            float maxStepDxM = 0.0f;
            float maxAbsDxFromStartM = 0.0f;
            float encoderForwardTravelM = 0.0f;
            float previousXM = first.poseXM;

            for (int index = 1; index < sampleCount; ++index)
            {
                const LoggedOpenFloorPoseJumpSample& sample = kLatestLoggedOpenFloorPoseJumpWindow[index];

                const App::Internal::CommandVector control =
                    App::Internal::CommandVector(
                        sample.leftDriveCommand,
                        sample.rightDriveCommand);
                Assert::IsTrue(core.predict(sample.dtSeconds, control));

                EncoderObs encoderObservation{};
                encoderObservation.SetTotalLeftCounts(sample.leftEncoderCount);
                encoderObservation.SetTotalRightCounts(sample.rightEncoderCount);
                encoderObservation.SetLeftWheelSpeedRadps(sample.leftEncoderWheelSpeedRadps);
                encoderObservation.SetRightWheelSpeedRadps(sample.rightEncoderWheelSpeedRadps);
                (void)core.updateEncoderPair(encoderObservation, sample.dtSeconds, true);
                Assert::IsTrue(core.LastUpdateAttempted());

                (void)core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(core.LastUpdateAttempted());

                const bool accelObservationValid =
                    std::isfinite(sample.accelBodyRightMps2) &&
                    std::isfinite(sample.accelBodyForwardMps2);
                const ImuAccelObs accelObservation(
                    accelObservationValid,
                    sample.accelBodyForwardMps2,
                    sample.accelBodyRightMps2);
                (void)core.updatePlanarAccel(accelObservation);

                const float currentXM = core.workingState()(0);
                maxStepDxM = (std::max)(maxStepDxM, std::fabs(currentXM - previousXM));
                maxAbsDxFromStartM = (std::max)(maxAbsDxFromStartM, std::fabs(currentXM - first.poseXM));
                previousXM = currentXM;

                encoderForwardTravelM += std::fabs(
                    0.5f * static_cast<float>(sample.leftEncoderCount + sample.rightEncoderCount) * distancePerCountM);
            }

            const float finalXM = core.workingState()(0);
            const float finalYM = core.workingState()(1);

            Assert::IsTrue(maxStepDxM < 0.01f);
            Assert::IsTrue(maxAbsDxFromStartM < 0.01f);
            Assert::IsTrue(std::fabs(finalXM - first.poseXM) < 0.01f);
            Assert::IsTrue(std::fabs(finalXM - first.poseXM) < (encoderForwardTravelM + 0.005f));
            Assert::IsTrue(finalYM > (first.poseYM - 0.005f));
        }
    };
}






