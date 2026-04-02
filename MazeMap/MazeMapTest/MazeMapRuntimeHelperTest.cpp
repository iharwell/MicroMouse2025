#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapRuntimeCsvLog.h"
#include "..\MazeMap\MazeMapRuntimeMmLog.h"
#include "..\MazeMap\MazeMapRuntimeSignalHelpers.h"

#include <cstring>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMapApp
{
    TEST_CLASS(MazeMapRuntimeHelperTest)
    {
    public:
        TEST_METHOD(ComputeSignalRiseAboveBaseline_ClampsInvalidAndBelowBaselineValues)
        {
            using MazeMapApp::Internal::Runtime::ComputeSignalRiseAboveBaseline;

            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(std::numeric_limits<float>::quiet_NaN(), 10.0f));
            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(8.0f, 10.0f));
            Assert::AreEqual(2.5f, ComputeSignalRiseAboveBaseline(12.5f, 10.0f));
        }

        TEST_METHOD(UpdateFilteredSignalState_UsesSharedHysteresisThresholds)
        {
            using MazeMapApp::Internal::Runtime::UpdateFilteredSignalState;

            float filteredSignal = 0.0f;
            bool currentState = false;
            bool initialized = false;

            Assert::IsFalse(UpdateFilteredSignalState(8.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsTrue(initialized);
            Assert::AreEqual(8.0f, filteredSignal);

            Assert::IsTrue(UpdateFilteredSignalState(12.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsTrue(currentState);

            Assert::IsTrue(UpdateFilteredSignalState(6.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsFalse(UpdateFilteredSignalState(4.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
        }

        TEST_METHOD(ComputeCorridorError_UsesAvailableWallObservationsConsistently)
        {
            using MazeMapApp::Internal::Runtime::ComputeCorridorError;

            Assert::AreEqual(0.01f, ComputeCorridorError(0.11f, 0.09f, true, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.02f, ComputeCorridorError(0.12f, 0.09f, true, false, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.03f, ComputeCorridorError(0.12f, 0.07f, false, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.0f, ComputeCorridorError(0.12f, 0.07f, false, false, 0.10f), 1.0e-6f);
        }

        TEST_METHOD(SelectSequentialCsvFileName_UsesExplicitNameWhenProvided)
        {
            using MazeMapApp::Internal::Runtime::SelectSequentialCsvFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialCsvFileName(buffer, sizeof(buffer), "custom.csv", "diag%03u.csv", "fallback.csv"));
            Assert::IsTrue(std::strcmp(buffer, "custom.csv") == 0);
        }

        TEST_METHOD(SelectSequentialCsvFileName_UsesHostFallbackWhenExplicitNameMissing)
        {
            using MazeMapApp::Internal::Runtime::SelectSequentialCsvFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialCsvFileName(buffer, sizeof(buffer), nullptr, "diag%03u.csv", "fallback.csv"));
            Assert::IsTrue(std::strcmp(buffer, "fallback.csv") == 0);
        }

        TEST_METHOD(SelectSequentialRuntimeFileName_UsesHostFallbackWhenExplicitNameMissing)
        {
            using MazeMapApp::Internal::Runtime::SelectSequentialRuntimeFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialRuntimeFileName(buffer, sizeof(buffer), nullptr, "diag%03u.mmlog", "fallback.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "fallback.mmlog") == 0);
        }

        TEST_METHOD(BuildSiblingRuntimeFileName_ReplacesExtension)
        {
            using MazeMapApp::Internal::Runtime::BuildSiblingRuntimeFileName;

            char buffer[64] = {};
            Assert::IsTrue(BuildSiblingRuntimeFileName(buffer, sizeof(buffer), "open_floor_main.mmlog", ".events.txt"));
            Assert::IsTrue(std::strcmp(buffer, "open_floor_main.events.txt") == 0);
        }

        TEST_METHOD(PackTextTagOrHash_UsesTagForShortTextAndHashForLongText)
        {
            const uint32_t shortPacked = MazeMapApp::Internal::Runtime::PackTextTagOrHash("imu");
            Assert::AreEqual(mmlog::TAG4('i', 'm', 'u', '\0'), shortPacked);

            const uint32_t longPacked = MazeMapApp::Internal::Runtime::PackTextTagOrHash("front_pair_stream");
            Assert::AreNotEqual(0U, longPacked);
            Assert::AreNotEqual(mmlog::TAG4('f', 'r', 'o', 'n'), longPacked);
        }

        TEST_METHOD(GenericMmLogHeaders_UseExpectedFixedSizes)
        {
            Assert::AreEqual<size_t>(static_cast<size_t>(44u), sizeof(mmlog::GenericFileHeader));
            Assert::AreEqual<size_t>(static_cast<size_t>(8u), sizeof(mmlog::LogRecordHeader));
        }

    };
}
