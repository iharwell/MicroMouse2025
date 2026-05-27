#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorWallMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorWallUpdateTest)
    {
    public:
        TEST_METHOD(FrontWallLeftRayHits)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();

            Assert::IsTrue(scenario.primaryPredictionHit, L"front-left ray did not hit wall");
        }

        TEST_METHOD(FrontWallRightRayHits)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();

            Assert::IsTrue(scenario.secondaryPredictionHit, L"front-right ray did not hit wall");
        }

        TEST_METHOD(FrontWallUpdateAccepted)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"front wall update was rejected");
        }

        TEST_METHOD(FrontWallMovesForward)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();
            const std::wstring message =
                std::wstring(L"before y=") +
                std::to_wstring(scenario.beforeState(1)) +
                L", after y=" +
                std::to_wstring(scenario.afterState(1));

            Assert::IsTrue(scenario.afterState(1) > (scenario.beforeState(1) + 0.002f), message.c_str());
        }

        TEST_METHOD(FrontWallKeepsXStable)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();
            const float delta = std::fabs(scenario.afterState(0) - scenario.beforeState(0));
            const std::wstring message =
                std::wstring(L"front wall x delta") +
                L" actual=" +
                std::to_wstring(delta) +
                L", limit=" +
                std::to_wstring(0.004f);

            Assert::IsTrue(delta < 0.004f, message.c_str());
        }

        TEST_METHOD(FrontWallReducesForwardCovariance)
        {
            const WallUpdateScenario scenario = RunFrontWallScenario();

            Assert::IsTrue(scenario.afterCovariance(1, 1) < scenario.beforeCovariance(1, 1), L"front wall did not reduce forward covariance");
        }

        TEST_METHOD(LeftWallRayHits)
        {
            const WallUpdateScenario scenario = RunLeftWallScenario();

            Assert::IsTrue(scenario.primaryPredictionHit, L"left wall ray did not hit wall");
        }

        TEST_METHOD(LeftWallUpdateAccepted)
        {
            const WallUpdateScenario scenario = RunLeftWallScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"left wall update was rejected");
        }

        TEST_METHOD(LeftWallMovesLeft)
        {
            const WallUpdateScenario scenario = RunLeftWallScenario();
            const std::wstring message =
                std::wstring(L"before x=") +
                std::to_wstring(scenario.beforeState(0)) +
                L", after x=" +
                std::to_wstring(scenario.afterState(0));

            Assert::IsTrue(scenario.afterState(0) < (scenario.beforeState(0) - 0.002f), message.c_str());
        }

        TEST_METHOD(LeftWallKeepsYStable)
        {
            const WallUpdateScenario scenario = RunLeftWallScenario();
            const float delta = std::fabs(scenario.afterState(1) - scenario.beforeState(1));
            const std::wstring message =
                std::wstring(L"left wall y delta") +
                L" actual=" +
                std::to_wstring(delta) +
                L", limit=" +
                std::to_wstring(0.004f);

            Assert::IsTrue(delta < 0.004f, message.c_str());
        }

        TEST_METHOD(LeftWallReducesLateralCovariance)
        {
            const WallUpdateScenario scenario = RunLeftWallScenario();

            Assert::IsTrue(scenario.afterCovariance(0, 0) < scenario.beforeCovariance(0, 0), L"left wall did not reduce lateral covariance");
        }

    };
}
