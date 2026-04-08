#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapSharedRuntime.h"
#include "..\MazeMap\Vehicle.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace mmlog = MazeMap::mmlog;

#define SHARED_RUNTIME_TEST_LOG_FIELDS(X) \
    X(std::uint32_t, seq) \
    X(float,         value)

MMLOG_DEFINE_ROW(SharedRuntimeTestLogRow, SHARED_RUNTIME_TEST_LOG_FIELDS);

namespace
{
    struct FaultCallbackState
    {
        unsigned int count = 0U;
        char reason[64] = {};
    };

    struct StreamingFaultCallbackState
    {
        MazeMap::App::Internal::SharedRobotRuntime* runtime = nullptr;
        unsigned int linesWritten = 0U;
    };

    void CaptureFaultCallback(void* context, const char* reason) noexcept
    {
        auto* state = static_cast<FaultCallbackState*>(context);
        if (state == nullptr)
        {
            return;
        }

        ++state->count;
        const char* text = (reason != nullptr) ? reason : "";
        std::snprintf(state->reason, sizeof(state->reason), "%s", text);
    }

    void StreamLongFaultDump(void* context, const char* reason) noexcept
    {
        auto* state = static_cast<StreamingFaultCallbackState*>(context);
        if (state == nullptr || state->runtime == nullptr)
        {
            return;
        }

        for (unsigned int index = 0U; index < 32U; ++index)
        {
            char message[256] = {};
            const int length = std::snprintf(
                message,
                sizeof(message),
                "reason=%s;line=%u;payload=%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u%.3u",
                (reason != nullptr) ? reason : "unknown",
                index,
                index,
                index + 1U,
                index + 2U,
                index + 3U,
                index + 4U,
                index + 5U,
                index + 6U,
                index + 7U,
                index + 8U,
                index + 9U,
                index + 10U,
                index + 11U);
            if (length <= 0 || length >= static_cast<int>(sizeof(message)))
            {
                return;
            }

            if (!state->runtime->WriteTextLogEntry(1000U + index, "ukf_dump_test", message))
            {
                return;
            }

            state->runtime->FlushTextLog();
            ++state->linesWritten;
        }
    }
}

namespace MazeMap::App
{
    TEST_CLASS(MazeMapSharedRuntimeTest)
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

        static std::string CreateTempPath(const char* stem)
        {
            static unsigned long counter = 0UL;
            char path[96] = {};
            const int length = snprintf(
                path,
                sizeof(path),
                "%s_%lu.mmlog",
                (stem != nullptr) ? stem : "shared_runtime_test",
                ++counter);
            Assert::IsTrue(length > 0 && length < static_cast<int>(sizeof(path)));
            std::remove(path);
            return std::string(path);
        }

        TEST_METHOD(SharedRuntime_SearchVehicleUsesConservativeSearchProfile)
        {
            Internal::SharedRobotRuntime runtime;

            const MazeMap::Vehicle& speedVehicle = runtime.SpeedVehicle();
            const MazeMap::Vehicle& searchVehicle = runtime.SearchVehicle();

            Assert::IsTrue(searchVehicle.GetMaxSpeed() < speedVehicle.GetMaxSpeed());
            Assert::IsTrue(searchVehicle.GetMaxForwardAcceleration() < speedVehicle.GetMaxForwardAcceleration());
            Assert::IsTrue(searchVehicle.GetMaxLateralAcceleration() < speedVehicle.GetMaxLateralAcceleration());
        }

        TEST_METHOD(SharedRuntime_ProvidesDistinctDiagnosticAndTelemetrySensorPipelines)
        {
            Internal::SharedRobotRuntime runtime;

            Assert::IsTrue(
                static_cast<const void*>(&runtime.DiagnosticSensors()) !=
                static_cast<const void*>(&runtime.MissionSensors()));
            Assert::IsTrue(&runtime.Drive() == &runtime.Drive());
        }

        TEST_METHOD(SharedRuntime_SingletonIsStable)
        {
            Internal::SharedRobotRuntime& first = Internal::GetSharedRobotRuntime();
            Internal::SharedRobotRuntime& second = Internal::GetSharedRobotRuntime();

            Assert::IsTrue(&first == &second);
        }

        TEST_METHOD(SharedRuntime_UtilityLoggingUsesRuntimeOwnedDataAndTextLogs)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()) == dataPath);
                Assert::IsTrue(runtime.WriteUtilityDataLogMetadata("mode", "unit_test"));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "unit_test"));

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));

                row.seq = 7U;
                row.value = 2.5f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));
                Assert::IsTrue(runtime.WriteTextLogPhase(3U, 456U, "phase_a"));
                Assert::IsTrue(runtime.WriteTextLogEntry(789U, "summary", "hello"));
                Assert::IsTrue(runtime.FlushUtilityDataLog());
                runtime.FlushTextLog();
                Assert::IsTrue(runtime.CloseUtilityDataLog());
                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()).empty());
            }

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            Assert::IsTrue(sidecarText.find("mode=unit_test\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("control_log_file=logging.txt\n") != std::string::npos);

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("mode=unit_test") != std::string::npos);
            Assert::IsTrue(controlText.find("phase_id=3;name=phase_a") != std::string::npos);
            Assert::IsTrue(controlText.find("summary: hello") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_TextLogRemainsUsableAcrossDataLogSessions)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_handoff");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("before_data_log"));
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.TextLogIsOpen());

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));
                Assert::IsTrue(runtime.TextLogIsOpen());

                row.seq = 11U;
                row.value = 4.0f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));
                Assert::IsTrue(runtime.AppendTextLogLine("during_data_log"));
                Assert::IsTrue(runtime.FlushUtilityDataLog());
                Assert::IsTrue(runtime.CloseUtilityDataLog());
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.AppendTextLogLine("after_data_log"));
                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("before_data_log") != std::string::npos);
            Assert::IsTrue(controlText.find("during_data_log") != std::string::npos);
            Assert::IsTrue(controlText.find("after_data_log") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_UtilityDataMetadataFailurePreservesLastError)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_error");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(std::string(runtime.LastRuntimeLogError()).empty());
                Assert::IsTrue(runtime.WriteUtilityDataLogMetadata("mode", "first"));
                Assert::IsFalse(runtime.WriteUtilityDataLogMetadata("mode", "second"));
                Assert::IsTrue(std::string(runtime.LastRuntimeLogError()) == "Duplicate metadata key.");
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("data_log_metadata_failed: Duplicate metadata key.") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_ServiceUtilityDataLog_LeavesSubsectorTextBufferedUntilFlush)
        {
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("subsector_text"));
                Assert::IsTrue(runtime.TextLogIsOpen());
                Assert::IsTrue(runtime.ServiceUtilityDataLog());
                Assert::IsTrue(ReadAllBytes(controlPath).empty());

                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("subsector_text") != std::string::npos);

            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FlushTextLogWritesExactBufferedBytesWithoutPadding)
        {
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            const std::string expected = "alpha\nbeta\ngamma\n";
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;

                Assert::IsTrue(runtime.AppendTextLogLine("alpha"));
                Assert::IsTrue(runtime.AppendTextLogLine("beta"));
                Assert::IsTrue(runtime.AppendTextLogLine("gamma"));
                runtime.FlushTextLog();
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText == expected);
            Assert::IsTrue(controlText.find('\0') == std::string::npos);

            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FailActiveModeClosesAndDisablesRuntimeLogs)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_fault");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string retryDataPath = CreateTempPath("codex_shared_runtime_retry");
            const std::string retrySidecarPath = ReplaceExtension(retryDataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;
                FaultCallbackState callbackState{};

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "fault_shutdown"));
                Assert::IsTrue(runtime.RegisterModeFaultHandler(&CaptureFaultCallback, &callbackState, "fault_shutdown"));
                Assert::ExpectException<std::logic_error>(
                    [&runtime, &callbackState]()
                    {
                        (void)runtime.RegisterModeFaultHandler(&CaptureFaultCallback, &callbackState, "fault_shutdown_duplicate");
                    });

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));

                row.seq = 1U;
                row.value = 9.5f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));

                Assert::IsFalse(runtime.FailActiveMode("unit_test_fault"));

                Assert::IsTrue(std::string(runtime.ActiveUtilityDataLogFileName()).empty());
                Assert::IsFalse(runtime.TextLogIsOpen());
                Assert::IsTrue(callbackState.count == 1U);
                Assert::IsTrue(std::string(callbackState.reason) == "unit_test_fault");
                Assert::IsFalse(runtime.WriteTextLogEntry(10U, "summary", "after_fault"));
                Assert::IsFalse(runtime.OpenUtilityDataLogFile(retryDataPath.c_str()));
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("mode=fault_shutdown") != std::string::npos);
            Assert::IsTrue(controlText.find("fault: unit_test_fault") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(retryDataPath.c_str());
            std::remove(retrySidecarPath.c_str());
            std::remove(controlPath.c_str());
        }

        TEST_METHOD(SharedRuntime_FaultCallbackCanStreamTextBeyondQueueCapacityByFlushingIncrementally)
        {
            const std::string dataPath = CreateTempPath("codex_shared_runtime_fault_dump");
            const std::string sidecarPath = ReplaceExtension(dataPath, ".sidecar");
            const std::string controlPath = Internal::kSharedRuntimeTextLogFileName;
            std::remove(controlPath.c_str());

            {
                Internal::SharedRobotRuntime runtime;
                StreamingFaultCallbackState callbackState{};
                callbackState.runtime = &runtime;

                Assert::IsTrue(runtime.OpenUtilityDataLogFile(dataPath.c_str()));
                Assert::IsTrue(runtime.WriteTextLogMetadata("mode", "fault_dump_stream"));
                Assert::IsTrue(runtime.RegisterModeFaultHandler(&StreamLongFaultDump, &callbackState, "fault_dump_stream"));

                SharedRuntimeTestLogRow row{};
                Assert::IsTrue(runtime.BeginUtilityDataLogSchema(row));
                row.seq = 2U;
                row.value = 1.0f;
                Assert::IsTrue(runtime.LogUtilityDataRow(row));

                Assert::IsFalse(runtime.FailActiveMode("stream_fault_dump"));
                Assert::AreEqual(32U, callbackState.linesWritten);
            }

            const std::string controlText = ReadAllBytes(controlPath);
            Assert::IsTrue(controlText.find("fault: stream_fault_dump") != std::string::npos);
            Assert::IsTrue(controlText.find("ukf_dump_test: reason=stream_fault_dump;line=0;") != std::string::npos);
            Assert::IsTrue(controlText.find("ukf_dump_test: reason=stream_fault_dump;line=31;") != std::string::npos);

            std::remove(dataPath.c_str());
            std::remove(sidecarPath.c_str());
            std::remove(controlPath.c_str());
        }
    };
}

