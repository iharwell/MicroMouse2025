#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorMovingPredictGyroBiasTest)
    {
    public:
        TEST_METHOD(MovingPredictInitializesScenario)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::IsTrue(scenario.resetAccepted, L"moving predict scenario reset was rejected");
        }

        TEST_METHOD(MovingPredictAcceptsStep)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::IsTrue(scenario.predictAccepted, L"moving predict step was rejected");
        }

        TEST_METHOD(MovingPredictKeepsForwardAccelResidual)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::AreEqual(
                scenario.initialState(6),
                scenario.predictedState(6),
                1.0e-6f,
                L"delta_af_mps2 changed during moving predict");
        }

        TEST_METHOD(MovingPredictKeepsRightAccelResidual)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::AreEqual(
                scenario.initialState(7),
                scenario.predictedState(7),
                1.0e-6f,
                L"delta_ar_mps2 changed during moving predict");
        }

        TEST_METHOD(MovingPredictKeepsYawAccelResidual)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::AreEqual(
                scenario.initialState(8),
                scenario.predictedState(8),
                1.0e-6f,
                L"delta_yaw_accel_radps2 changed during moving predict");
        }

        TEST_METHOD(MovingPredictKeepsGyroBiasVariance)
        {
            const MovingPredictGyroBiasScenario scenario =
                RunMovingPredictGyroBiasScenario();

            Assert::AreEqual(
                scenario.beforeGyroBiasVarianceRadps2,
                scenario.afterGyroBiasVarianceRadps2,
                1.0e-9f,
                L"gyro bias variance changed during moving predict");
        }

    };
}
