#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\OpenFloorMeasurementSpec.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(OpenFloorMeasurementSpecTest)
    {
    public:
        TEST_METHOD(LaunchDriveMagnitudesFollowConfiguredRange)
        {
            Assert::AreEqual(
                MazeMap::kOpenFloorLaunchDriveMagnitudeStart,
                MazeMap::kOpenFloorLaunchDriveMagnitudes.front(),
                1.0e-6f);
            Assert::AreEqual(
                MazeMap::kOpenFloorLaunchDriveMagnitudeEnd,
                MazeMap::kOpenFloorLaunchDriveMagnitudes.back(),
                1.0e-6f);
            Assert::AreEqual(
                static_cast<size_t>(MazeMap::kOpenFloorLaunchDriveMagnitudeCount),
                MazeMap::kOpenFloorLaunchDriveMagnitudes.size());

            for (size_t index = 1U; index < MazeMap::kOpenFloorLaunchDriveMagnitudes.size(); ++index)
            {
                Assert::AreEqual(
                    MazeMap::kOpenFloorLaunchDriveMagnitudeStep,
                    MazeMap::kOpenFloorLaunchDriveMagnitudes[index] -
                        MazeMap::kOpenFloorLaunchDriveMagnitudes[index - 1U],
                    1.0e-6f);
            }
        }

        TEST_METHOD(LaunchSettlingTimeIsConfigurableWithoutDisablingByDefault)
        {
            Assert::IsTrue(MazeMap::kOpenFloorLaunchSettleMs > 0UL);
        }

        TEST_METHOD(RecoveryAcceptanceRadiusIsFifteenMillimeters)
        {
            Assert::AreEqual(0.015f, MazeMap::kOpenFloorRecoveryAcceptanceRadiusM, 1.0e-6f);
        }

        TEST_METHOD(RecoveryLongitudinalDistanceUsesAcceptanceZoneBoundary)
        {
            const Eigen::Vector2f travelHeading = DirectionToUnitVector(MazeMap::Up);
            const float distanceM = MazeMap::OpenFloorRecoverySignedLongitudinalDistanceToAcceptanceZoneM(
                travelHeading,
                0.0f,
                0.040f);

            Assert::AreEqual(0.025f, distanceM, 1.0e-6f);
        }

        TEST_METHOD(RecoveryLongitudinalDistanceAllowsBackwardTravel)
        {
            const Eigen::Vector2f travelHeading = DirectionToUnitVector(MazeMap::Up);
            const float distanceM = MazeMap::OpenFloorRecoverySignedLongitudinalDistanceToAcceptanceZoneM(
                travelHeading,
                0.0f,
                -0.040f);

            Assert::AreEqual(-0.025f, distanceM, 1.0e-6f);
        }

        TEST_METHOD(RecoveryAxisMissUsesOnlyExcessBeyondAcceptanceRadius)
        {
            const Eigen::Vector2f travelHeading = DirectionToUnitVector(MazeMap::Up);
            const float insideMissM = MazeMap::OpenFloorRecoverySignedLateralMissToAcceptanceZoneM(
                travelHeading,
                0.010f,
                0.0f);
            const float outsideMissM = MazeMap::OpenFloorRecoverySignedLateralMissToAcceptanceZoneM(
                travelHeading,
                0.020f,
                0.0f);

            Assert::AreEqual(0.0f, insideMissM, 1.0e-6f);
            Assert::AreEqual(-0.005f, outsideMissM, 1.0e-6f);
        }

        TEST_METHOD(RecoveryArrivalHeadingToleranceIsOneDegree)
        {
            Assert::AreEqual(1.0f * DEG_TO_RAD_F, MazeMap::kOpenFloorRecoveryArrivalHeadingToleranceRad, 1.0e-6f);
        }
    };
}
