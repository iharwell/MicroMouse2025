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
    };
}
