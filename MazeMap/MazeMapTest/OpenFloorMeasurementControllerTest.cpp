#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\BootModeRegistry.h"
#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\DiagnosticConfig.h"
#include "..\MazeMap\OpenFloorMeasurementController.h"
#include "..\MazeMap\OpenFloorMeasurementSpec.h"
#include "..\MazeMap\SharedRobotRuntime.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    namespace
    {
        using MazeMap::App::Internal::OpenFloorMeasurementController;
        using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
        using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;
        using MazeMap::App::Internal::SharedRobotRuntime;

        std::pair<std::uint8_t, std::uint8_t> GetOpenFloorSelectorPins()
        {
            const BootModeRegistryEntry* const entry =
                FindBootModeRegistryEntry(BootModeId::OpenFloorMeasurement);
            Assert::IsTrue(entry != nullptr);
            Assert::IsTrue(entry->selector.kind == BootModeSelectorKind::PinPair);
            return std::make_pair(entry->selector.pinA, entry->selector.pinB);
        }

        void SetOpenFloorSelectorInstalled(const bool installed)
        {
            const auto selectorPins = GetOpenFloorSelectorPins();
            HostSetPinShort(selectorPins.first, selectorPins.second, installed);
        }

        void AssertOpenFloorSelectorActive(const bool active)
        {
            const auto selectorPins = GetOpenFloorSelectorPins();
            const BootModeSelectorCondition selector =
                BootModeSelectorCondition::PinPair(selectorPins.first, selectorPins.second);
            Assert::IsTrue(IsBootModeSelectorConditionActive(selector) == active);
        }

        const std::filesystem::path& OpenFloorArtifactDirectory()
        {
            static const std::filesystem::path path = []()
            {
                const std::filesystem::path sourcePath(__FILE__);
                return sourcePath.parent_path().parent_path() / "x64" / "Release";
            }();

            Assert::IsTrue(std::filesystem::exists(path));
            return path;
        }

        std::mutex& OpenFloorControllerTestMutex()
        {
            static std::mutex mutex;
            return mutex;
        }

        std::filesystem::path ArtifactPath(const char* path)
        {
            return OpenFloorArtifactDirectory() / path;
        }

        bool FileExists(const char* path)
        {
            std::ifstream file(ArtifactPath(path), std::ios::binary);
            return file.good();
        }

        std::uintmax_t FileSizeBytes(const char* path)
        {
            std::error_code error;
            const std::uintmax_t size = std::filesystem::file_size(ArtifactPath(path), error);
            return error ? 0U : size;
        }

        bool LogHasAtLeastOneRow(const char* path, const std::size_t rowBytes)
        {
            return FileSizeBytes(path) >= (std::strlen(path) + std::strlen("sidecar_file=\n") + rowBytes);
        }

        void DeleteIfPresent(const char* path)
        {
            std::error_code ignored;
            std::filesystem::remove(ArtifactPath(path), ignored);
        }

        std::string ReadAllBytes(const char* path)
        {
            std::ifstream file(ArtifactPath(path), std::ios::binary);
            return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        }

        std::string ReplaceExtension(const char* path, const char* extension)
        {
            std::string result = path;
            const std::size_t dot = result.find_last_of('.');
            if (dot == std::string::npos)
            {
                result += extension;
                return result;
            }

            result.resize(dot);
            result += extension;
            return result;
        }

        void DeleteOpenFloorArtifacts()
        {
            DeleteIfPresent(MazeMap::kOpenFloorTimingFileName);
            DeleteIfPresent(ReplaceExtension(MazeMap::kOpenFloorTimingFileName, ".sidecar").c_str());
            DeleteIfPresent(MazeMap::kOpenFloorMainFileName);
            DeleteIfPresent(ReplaceExtension(MazeMap::kOpenFloorMainFileName, ".sidecar").c_str());
        }

        template <typename Predicate>
        bool WaitUntil(Predicate&& predicate, const std::chrono::milliseconds timeout)
        {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (predicate())
                {
                    return true;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            return predicate();
        }

        template <typename TRow>
        std::vector<TRow> ReadMmlogRows(const char* path)
        {
            std::vector<TRow> rows;
            const bool parsed = WaitUntil(
                [&]()
                {
                    if (!TryReadMmlogRows<TRow>(path, rows))
                    {
                        return false;
                    }

                    const bool shouldContainRow = LogHasAtLeastOneRow(path, sizeof(TRow));
                    return !shouldContainRow || !rows.empty();
                },
                std::chrono::milliseconds(500));
            Assert::IsTrue(parsed);
            return rows;
        }

        template <typename TRow>
        bool TryReadMmlogRows(const char* path, std::vector<TRow>& rows)
        {
            std::ifstream file(ArtifactPath(path), std::ios::binary);
            if (!file.good())
            {
                return false;
            }

            const std::string primaryBytes(
                (std::istreambuf_iterator<char>(file)),
                std::istreambuf_iterator<char>());
            const std::size_t newline = primaryBytes.find('\n');
            if (newline == std::string::npos)
            {
                return false;
            }

            const std::size_t payloadOffset = newline + 1U;
            if (primaryBytes.size() < payloadOffset)
            {
                return false;
            }

            const std::size_t payloadBytes = primaryBytes.size() - payloadOffset;
            if ((payloadBytes % sizeof(TRow)) != 0U)
            {
                return false;
            }

            rows.assign(payloadBytes / sizeof(TRow), TRow{});
            if (!rows.empty())
            {
                std::memcpy(rows.data(), primaryBytes.data() + payloadOffset, payloadBytes);
            }

            return true;
        }

        std::future<void> StartModeRun(OpenFloorMeasurementController& mode)
        {
            return std::async(std::launch::async, [&mode]() { mode.Run(); });
        }

        std::wstring BuildBeginFailureMessage(const SharedRobotRuntime& runtime)
        {
            std::wstring message = L"OpenFloorMeasurementController::Begin() returned false";
            const char* const error = runtime.LastRuntimeLogError();
            if ((error != nullptr) && (error[0] != '\0'))
            {
                message += L": ";
                message.append(error, error + std::strlen(error));
            }

            return message;
        }

        void FinishModeRun(SharedRobotRuntime& runtime, std::future<void>& runFuture);

        class ModeRunGuard final
        {
        public:
            ModeRunGuard(SharedRobotRuntime& runtime, OpenFloorMeasurementController& mode)
                : _runtime(runtime)
                , _runFuture(StartModeRun(mode))
            {
            }

            ~ModeRunGuard()
            {
                if (_finished)
                {
                    return;
                }

                if (_runFuture.valid())
                {
                    if (_runFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
                    {
                        (void)_runtime.FailActiveMode("test cleanup");
                        (void)_runFuture.wait_for(std::chrono::milliseconds(10000));
                    }

                    if (_runFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
                    {
                        try
                        {
                            _runFuture.get();
                        }
                        catch (...)
                        {
                        }
                    }
                }

                _runtime.FinalizeSuccessfulModeExit();
            }

            std::future<void>& Future() noexcept
            {
                return _runFuture;
            }

            void Finish()
            {
                FinishModeRun(_runtime, _runFuture);
                _finished = true;
            }

        private:
            SharedRobotRuntime& _runtime;
            std::future<void> _runFuture;
            bool _finished{};
        };

        void FinishModeRun(SharedRobotRuntime& runtime, std::future<void>& runFuture)
        {
            if (runFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            {
                (void)runtime.FailActiveMode("test cleanup");
                Assert::IsTrue(runFuture.wait_for(std::chrono::milliseconds(10000)) == std::future_status::ready);
            }

            runFuture.get();
            runtime.FinalizeSuccessfulModeExit();
        }
    }

    TEST_CLASS(OpenFloorMeasurementControllerTest)
    {
    public:
        TEST_METHOD_INITIALIZE(ResetHostState)
        {
            _testLock = std::unique_lock<std::mutex>(OpenFloorControllerTestMutex());
            std::filesystem::current_path(OpenFloorArtifactDirectory());
            HostResetDigitalPins();
            DeleteOpenFloorArtifacts();
        }

        TEST_METHOD_CLEANUP(CleanupArtifacts)
        {
            std::filesystem::current_path(OpenFloorArtifactDirectory());
            HostResetDigitalPins();
            DeleteOpenFloorArtifacts();
            if (_testLock.owns_lock())
            {
                _testLock.unlock();
            }
        }

        TEST_METHOD(OpenFloorMeasurementController_TimingFaultStopsBeforeMainStage)
        {
            SharedRobotRuntime runtime;
            OpenFloorMeasurementController mode(runtime);
            SetOpenFloorSelectorInstalled(true);
            AssertOpenFloorSelectorActive(true);
            const bool began = mode.Begin();
            const std::wstring beginFailure = BuildBeginFailureMessage(runtime);
            Assert::IsTrue(began, beginFailure.c_str());

            ModeRunGuard run(runtime, mode);
            Assert::IsTrue(WaitUntil(
                [&runtime]()
                {
                    return runtime.ControlLoop().SessionActive() &&
                        runtime.ControlLoop().LastDiagnostics().sequence >= 2U;
                },
                std::chrono::milliseconds(1000)));
            Assert::IsTrue(runtime.ControlLoop().LastDiagnostics().sequence < DiagnosticConfig::kTimingCaptureCycles);
            Assert::IsTrue(FileExists(MazeMap::kOpenFloorTimingFileName));
            Assert::IsFalse(FileExists(MazeMap::kOpenFloorMainFileName));

            SetOpenFloorSelectorInstalled(false);
            run.Finish();

            Assert::IsTrue(FileExists(MazeMap::kOpenFloorTimingFileName));
            Assert::IsFalse(FileExists(MazeMap::kOpenFloorMainFileName));

            const std::vector<OpenFloorTimingRow> timingRows =
                ReadMmlogRows<OpenFloorTimingRow>(MazeMap::kOpenFloorTimingFileName);
            Assert::IsTrue(!timingRows.empty());
            Assert::IsTrue(timingRows.size() < DiagnosticConfig::kTimingCaptureCycles);
            for (const OpenFloorTimingRow& row : timingRows)
            {
                Assert::IsTrue(row.phase_id == static_cast<std::uint32_t>(OpenFloorSectionId::Sec00Timing));
            }

            const std::string timingSidecar =
                ReadAllBytes(ReplaceExtension(MazeMap::kOpenFloorTimingFileName, ".sidecar").c_str());
            Assert::IsTrue(
                timingSidecar.find(std::string("stream_type=") + MazeMap::kOpenFloorTimingStreamType + "\n") !=
                std::string::npos);
        }

        TEST_METHOD(OpenFloorMeasurementController_TimingToMainHandoffCreatesSeparateStreamsAndCommitsFinalTimingRow)
        {
            SharedRobotRuntime runtime;
            OpenFloorMeasurementController mode(runtime);
            SetOpenFloorSelectorInstalled(true);
            AssertOpenFloorSelectorActive(true);
            const bool began = mode.Begin();
            const std::wstring beginFailure = BuildBeginFailureMessage(runtime);
            Assert::IsTrue(began, beginFailure.c_str());

            ModeRunGuard run(runtime, mode);
            Assert::IsTrue(WaitUntil(
                []()
                {
                    return FileExists(MazeMap::kOpenFloorTimingFileName) &&
                        LogHasAtLeastOneRow(MazeMap::kOpenFloorMainFileName, sizeof(OpenFloorMainRow));
                },
                std::chrono::milliseconds(5000)));
            run.Finish();

            Assert::IsTrue(FileExists(MazeMap::kOpenFloorTimingFileName));
            Assert::IsTrue(FileExists(MazeMap::kOpenFloorMainFileName));

            const std::vector<OpenFloorTimingRow> timingRows =
                ReadMmlogRows<OpenFloorTimingRow>(MazeMap::kOpenFloorTimingFileName);
            const std::vector<OpenFloorMainRow> mainRows =
                ReadMmlogRows<OpenFloorMainRow>(MazeMap::kOpenFloorMainFileName);
            Assert::IsTrue(timingRows.size() == DiagnosticConfig::kTimingCaptureCycles);
            Assert::IsTrue(!mainRows.empty());

            const OpenFloorTimingRow& finalTimingRow = timingRows.back();
            Assert::IsTrue(finalTimingRow.phase_id == static_cast<std::uint32_t>(OpenFloorSectionId::Sec00Timing));

            const OpenFloorMainRow& firstMainRow = mainRows.front();
            Assert::IsTrue(firstMainRow.phase_id == static_cast<std::uint8_t>(OpenFloorSectionId::Sec10Static));
            Assert::IsTrue(firstMainRow.primitive_id == static_cast<std::uint8_t>(MazeMap::MC_NONE));
            Assert::IsTrue(firstMainRow.speed_bin == MazeMap::kOpenFloorSpeedBinLogIdNone);
            Assert::IsTrue(firstMainRow.repeat_index == 1U);

            const std::string timingSidecar =
                ReadAllBytes(ReplaceExtension(MazeMap::kOpenFloorTimingFileName, ".sidecar").c_str());
            const std::string mainSidecar =
                ReadAllBytes(ReplaceExtension(MazeMap::kOpenFloorMainFileName, ".sidecar").c_str());
            Assert::IsTrue(
                timingSidecar.find(std::string("stream_type=") + MazeMap::kOpenFloorTimingStreamType + "\n") !=
                std::string::npos);
            Assert::IsTrue(
                mainSidecar.find(std::string("stream_type=") + MazeMap::kOpenFloorMainStreamType + "\n") !=
                std::string::npos);
        }

        TEST_METHOD(OpenFloorMeasurementController_FirstMainPhaseIdentityIsStaticHoldAndFaultDoesNotAdvancePastIt)
        {
            SharedRobotRuntime runtime;
            OpenFloorMeasurementController mode(runtime);
            SetOpenFloorSelectorInstalled(true);
            AssertOpenFloorSelectorActive(true);
            const bool began = mode.Begin();
            const std::wstring beginFailure = BuildBeginFailureMessage(runtime);
            Assert::IsTrue(began, beginFailure.c_str());

            ModeRunGuard run(runtime, mode);
            Assert::IsTrue(WaitUntil(
                []()
                {
                    return LogHasAtLeastOneRow(MazeMap::kOpenFloorMainFileName, sizeof(OpenFloorMainRow));
                },
                std::chrono::milliseconds(5000)));

            SetOpenFloorSelectorInstalled(false);
            run.Finish();

            Assert::IsTrue(FileExists(MazeMap::kOpenFloorMainFileName));
            const std::vector<OpenFloorMainRow> mainRows =
                ReadMmlogRows<OpenFloorMainRow>(MazeMap::kOpenFloorMainFileName);
            Assert::IsTrue(!mainRows.empty());

            for (const OpenFloorMainRow& row : mainRows)
            {
                Assert::IsTrue(row.phase_id == static_cast<std::uint8_t>(OpenFloorSectionId::Sec10Static));
                Assert::IsTrue(row.primitive_id == static_cast<std::uint8_t>(MazeMap::MC_NONE));
                Assert::IsTrue(row.speed_bin == MazeMap::kOpenFloorSpeedBinLogIdNone);
                Assert::IsTrue(row.repeat_index == 1U);
            }
        }

    private:
        std::unique_lock<std::mutex> _testLock{};
    };
}
