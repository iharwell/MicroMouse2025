#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapRuntimeCsvLog.h"
#include "..\MazeMap\MazeMapRuntimeMmLog.h"
#include "..\MazeMap\MazeMapRuntimeSignalHelpers.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <cstring>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    TEST_CLASS(MazeMapRuntimeHelperTest)
    {
    public:
        static std::string ReadAllBytes(const std::string& path)
        {
            std::ifstream file(path, std::ios::binary);
            return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        }

        static std::string ReplaceExtension(const std::string& path, const char* extension)
        {
            const std::size_t dot = path.find_last_of('.');
            return (dot == std::string::npos) ? (path + extension) : (path.substr(0, dot) + extension);
        }

        static std::string BaseName(const std::string& path)
        {
            const std::size_t slash = path.find_last_of("\\/");
            return (slash == std::string::npos) ? path : path.substr(slash + 1U);
        }

        static std::string CreateTempPath(const char* extension)
        {
            static unsigned long counter = 0UL;
            char path[96] = {};
            const int length = snprintf(
                path,
                sizeof(path),
                "codex_runtime_binary_log_%lu%s",
                ++counter,
                (extension != nullptr) ? extension : "");
            Assert::IsTrue(length > 0 && length < static_cast<int>(sizeof(path)));
            std::remove(path);
            return std::string(path);
        }

        TEST_METHOD(ComputeSignalRiseAboveBaseline_ClampsInvalidAndBelowBaselineValues)
        {
            using MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline;

            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(std::numeric_limits<float>::quiet_NaN(), 10.0f));
            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(8.0f, 10.0f));
            Assert::AreEqual(2.5f, ComputeSignalRiseAboveBaseline(12.5f, 10.0f));
        }

        TEST_METHOD(UpdateFilteredSignalState_UsesSharedHysteresisThresholds)
        {
            using MazeMap::App::Internal::Runtime::UpdateFilteredSignalState;

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
            using MazeMap::App::Internal::Runtime::ComputeCorridorError;

            Assert::AreEqual(0.01f, ComputeCorridorError(0.11f, 0.09f, true, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.02f, ComputeCorridorError(0.12f, 0.09f, true, false, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.03f, ComputeCorridorError(0.12f, 0.07f, false, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.0f, ComputeCorridorError(0.12f, 0.07f, false, false, 0.10f), 1.0e-6f);
        }

        TEST_METHOD(SelectSequentialRuntimeFileName_UsesExplicitNameWhenProvided)
        {
            using MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialRuntimeFileName(buffer, sizeof(buffer), "custom.mmlog", "diag%03u.mmlog", "fallback.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "custom.mmlog") == 0);
        }

        TEST_METHOD(SelectSequentialRuntimeFileName_UsesHostFallbackWhenExplicitNameMissing)
        {
            using MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialRuntimeFileName(buffer, sizeof(buffer), nullptr, "diag%03u.mmlog", "fallback.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "fallback.mmlog") == 0);
        }

        TEST_METHOD(BuildSiblingRuntimeFileName_ReplacesExtension)
        {
            using MazeMap::App::Internal::Runtime::BuildSiblingRuntimeFileName;

            char buffer[64] = {};
            Assert::IsTrue(BuildSiblingRuntimeFileName(buffer, sizeof(buffer), "open_floor_main.mmlog", ".events.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "open_floor_main.events.mmlog") == 0);
        }

        TEST_METHOD(PackTextTagOrHash_UsesTagForShortTextAndHashForLongText)
        {
            const uint32_t shortPacked = MazeMap::App::Internal::Runtime::PackTextTagOrHash("imu");
            Assert::AreEqual(mmlog::TAG4('i', 'm', 'u', '\0'), shortPacked);

            const uint32_t longPacked = MazeMap::App::Internal::Runtime::PackTextTagOrHash("front_pair_stream");
            Assert::AreNotEqual(0U, longPacked);
            Assert::AreNotEqual(mmlog::TAG4('f', 'r', 'o', 'n'), longPacked);
        }

        TEST_METHOD(RuntimeBinaryLogFile_WritesRevGBindingAndSidecar)
        {
            using MazeMap::App::Internal::Runtime::RuntimeBinaryLogFile;
            using MazeMap::App::Internal::Runtime::RuntimeRecordBuilder;

            const std::string primaryPath = CreateTempPath(".mmlog");
            const std::string sidecarPath = ReplaceExtension(primaryPath, ".sidecar");

            RuntimeBinaryLogFile log;
            Assert::IsTrue(log.BeginSelected(
                primaryPath.c_str(),
                "u32_seq,f32_value,s32_kind",
                3U,
                "mode=test\n",
                "unit_case=runtime_binary_log\n"));

            RuntimeRecordBuilder<3U> record;
            record.U32(1U);
            record.F32(1.25f);
            record.U32(log.InternLabel("TEST_KIND"));
            Assert::IsTrue(log.AppendRecord(record.Data(), record.Count()));
            log.Close();

            const std::string primaryBytes = ReadAllBytes(primaryPath);
            const std::size_t newline = primaryBytes.find('\n');
            Assert::IsTrue(newline != std::string::npos);
            Assert::IsTrue((std::string("sidecar_file=") + BaseName(sidecarPath)) == primaryBytes.substr(0U, newline));
            Assert::IsTrue(static_cast<size_t>(newline + 1U + (3U * sizeof(uint32_t))) == primaryBytes.size());

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            Assert::IsTrue(sidecarText.find("schema_version=2\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("row_bytes=12\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("string_hash=fnv1a32\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("mode=test\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("unit_case=runtime_binary_log\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("u32_seq,f32_value,s32_kind\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("LABELS:\nTEST_KIND\n") != std::string::npos);

            std::remove(primaryPath.c_str());
            std::remove(sidecarPath.c_str());
        }

    };
}

