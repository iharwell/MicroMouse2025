#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorMovingPredictResidualTest)
    {
    public:
        TEST_METHOD(MovingPredictInitializesScenario)
        {
            const MovingPredictResidualScenario scenario =
                RunMovingPredictResidualScenario();

            Assert::IsTrue(scenario.resetAccepted, L"moving predict scenario reset was rejected");
        }

        TEST_METHOD(MovingPredictAcceptsStep)
        {
            const MovingPredictResidualScenario scenario =
                RunMovingPredictResidualScenario();

            Assert::IsTrue(scenario.predictAccepted, L"moving predict step was rejected");
        }

        TEST_METHOD(MovingPredictKeepsForwardAccelResidual)
        {
            const MovingPredictResidualScenario scenario =
                RunMovingPredictResidualScenario();

            Assert::AreEqual(
                scenario.initialState(6),
                scenario.predictedState(6),
                1.0e-6f,
                L"delta_af_mps2 changed during moving predict");
        }

        TEST_METHOD(MovingPredictKeepsRightAccelResidual)
        {
            const MovingPredictResidualScenario scenario =
                RunMovingPredictResidualScenario();

            Assert::AreEqual(
                scenario.initialState(7),
                scenario.predictedState(7),
                1.0e-6f,
                L"delta_ar_mps2 changed during moving predict");
        }

        TEST_METHOD(MovingPredictKeepsYawAccelResidual)
        {
            const MovingPredictResidualScenario scenario =
                RunMovingPredictResidualScenario();

            Assert::AreEqual(
                scenario.initialState(8),
                scenario.predictedState(8),
                1.0e-6f,
                L"delta_yaw_accel_radps2 changed during moving predict");
        }

    };
}
