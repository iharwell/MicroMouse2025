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

        TEST_METHOD(PostSegmentHoldUsesPositiveCumulativeStationaryBudget)
        {
            Assert::IsTrue(MazeMap::kOpenFloorPostSegmentHoldMs > 0UL);
        }

        TEST_METHOD(LaunchAndStraightSampleScheduleMatchesCurrentOpenFloorPlan)
        {
            Assert::AreEqual(static_cast<std::uint8_t>(3U), MazeMap::kOpenFloorLaunchRepeatsPerMagnitude);
            Assert::AreEqual(static_cast<std::uint8_t>(3U), MazeMap::kOpenFloorStraightRepeatsPerSpeed);
            Assert::AreEqual(250UL, MazeMap::kOpenFloorPostSegmentHoldMs);
            Assert::AreEqual(500UL, MazeMap::kOpenFloorInterPhaseHoldMs);
        }

        TEST_METHOD(YawRepeatNominalAnglesCancelToZeroHeading)
        {
            Assert::AreEqual(static_cast<size_t>(4U), MazeMap::kOpenFloorYawPrimitiveIds.size());
            Assert::AreEqual(static_cast<int>(MazeMap::OpenFloorPrimitiveId::Ip90), static_cast<int>(MazeMap::kOpenFloorYawPrimitiveIds[0U]));
            Assert::AreEqual(static_cast<int>(MazeMap::OpenFloorPrimitiveId::Ip90M), static_cast<int>(MazeMap::kOpenFloorYawPrimitiveIds[1U]));
            Assert::AreEqual(static_cast<int>(MazeMap::OpenFloorPrimitiveId::Ip180), static_cast<int>(MazeMap::kOpenFloorYawPrimitiveIds[2U]));
            Assert::AreEqual(static_cast<int>(MazeMap::OpenFloorPrimitiveId::Ip180M), static_cast<int>(MazeMap::kOpenFloorYawPrimitiveIds[3U]));

            float netAngleRad = 0.0f;
            for (const float angleRad : MazeMap::kOpenFloorYawNominalAnglesRad)
            {
                netAngleRad += angleRad;
            }

            Assert::AreEqual(0.0f, netAngleRad, 1.0e-6f);
        }

        TEST_METHOD(SelectorRemovalFaultRequiresHalfSecondOfContinuousLoss)
        {
            Assert::AreEqual(500UL, MazeMap::kOpenFloorSelectorRemovalFaultDelayMs);
            Assert::IsFalse(MazeMap::HasOpenFloorSelectorRemovalFaultDelayElapsed(1000UL, 1499UL));
            Assert::IsTrue(MazeMap::HasOpenFloorSelectorRemovalFaultDelayElapsed(1000UL, 1500UL));
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
