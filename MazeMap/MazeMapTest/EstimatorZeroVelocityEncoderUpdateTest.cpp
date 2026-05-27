#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorEncoderMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorZeroVelocityEncoderUpdateTest)
    {
    public:
        TEST_METHOD(ZeroVelocityResetAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the zero-velocity encoder scenario");
        }

        TEST_METHOD(ZeroVelocityForwardContactForceIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.forwardContactForceN, 1.0e-4f, L"forward contact force");
        }

        TEST_METHOD(ZeroVelocityRightContactForceIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.rightContactForceN, 1.0e-4f, L"right contact force");
        }

        TEST_METHOD(ZeroVelocityForwardAccelerationIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.runtimeForwardAccelerationMps2, 1.0e-4f, L"forward acceleration");
        }

        TEST_METHOD(ZeroVelocityRightAccelerationIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.runtimeRightAccelerationMps2, 1.0e-4f, L"right acceleration");
        }

        TEST_METHOD(ZeroVelocityRuntimeForwardVelocityIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.runtimeForwardVelocityMps, 1.0e-6f, L"runtime forward velocity");
        }

        TEST_METHOD(ZeroVelocityRuntimeRightVelocityIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.runtimeRightVelocityMps, 1.0e-6f, L"runtime right velocity");
        }

        TEST_METHOD(ZeroVelocityRuntimeYawRateIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.runtimeYawRateRadps, 1.0e-6f, L"runtime yaw rate");
        }

        TEST_METHOD(ZeroVelocityPredictAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::IsTrue(scenario.predictAccepted, L"predict rejected the zero-velocity encoder scenario");
        }

        TEST_METHOD(ZeroVelocityPredictKeepsAccelResiduals)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxAccelResidualDifference(scenario.initialState, scenario.priorState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity predict residual state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityPredictAddsResidualProcessNoise)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const float expectedForwardVariance =
                scenario.beforePredictCovariance(6, 6) +
                (0.050f * 0.050f * kDefaultEstimatorDtSeconds);

            Assert::AreEqual(expectedForwardVariance, scenario.priorCovariance(6, 6), 1.0e-8f, L"forward residual variance");
        }

        TEST_METHOD(ZeroVelocityUpdateAttempted)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::IsTrue(scenario.updateAttempted, L"zero-velocity encoder update was not attempted");
        }

        TEST_METHOD(ZeroVelocityUpdateAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"zero-velocity encoder update was rejected");
        }

        TEST_METHOD(ZeroVelocityUpdateRecordedAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::IsTrue(scenario.updateRecordedAccepted, L"zero-velocity encoder update was not recorded accepted");
        }

        TEST_METHOD(ZeroVelocityNisMatchesModel)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();

            Assert::AreEqual(scenario.expectation.nis, scenario.actualNis, 1.0e-4f, L"zero-velocity encoder NIS model");
        }

        TEST_METHOD(ZeroVelocityBodyStateMatchesModel)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyStateDifference(scenario.expectation.bodyState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity encoder body state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityKeepsUnmeasuredState)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxUnmeasuredStateDifference(scenario.priorState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity encoder unmeasured state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityBodyCovarianceMatchesModel)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyCovarianceDifference(
                    scenario.expectation.bodyCovariance,
                    scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity encoder body covariance") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-7f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-7f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityClearsBodyResidualCrossCovariance)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyResidualCrossCovariance(scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity body/residual covariance") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-8f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-8f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityKeepsResidualVariance)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const IndexedDifference difference =
                MaxResidualVarianceDifference(
                    scenario.priorCovariance,
                    scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"zero-velocity residual variance") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-8f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-8f, message.c_str());
        }

        TEST_METHOD(ZeroVelocityReducesYawRateVariance)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const std::wstring message =
                std::wstring(L"after yaw variance=") +
                std::to_wstring(scenario.afterCovariance(5, 5)) +
                L", initial=" +
                std::to_wstring(scenario.initialYawRateVarianceRadps2);

            Assert::IsTrue(scenario.afterCovariance(5, 5) < scenario.initialYawRateVarianceRadps2, message.c_str());
        }

        TEST_METHOD(ZeroVelocityKeepsHeadingVarianceBelowYawPrior)
        {
            const EncoderPairUpdateScenario scenario = RunZeroVelocityEncoderPairScenario();
            const std::wstring message =
                std::wstring(L"heading variance=") +
                std::to_wstring(scenario.afterCovariance(2, 2)) +
                L", initial yaw variance=" +
                std::to_wstring(scenario.initialYawRateVarianceRadps2);

            Assert::IsTrue(scenario.afterCovariance(2, 2) < scenario.initialYawRateVarianceRadps2, message.c_str());
        }

    };
}
