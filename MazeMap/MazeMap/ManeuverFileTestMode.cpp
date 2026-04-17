#include "pch.h"
#include "ManeuverFileTestMode.h"

#include "LoopController.h"
#include "ManeuverExecutor.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapSharedRuntime.h"

#include <cstdio>
#include <cstring>

namespace MazeMap::App::Internal
{
    namespace
    {
        constexpr const char* kManeuverFileTestFaultSource = "maneuver_file_test";
        constexpr const char* kManeuverFileTestTextLogSource = "maneuver_file_test";
        constexpr const char* kManeuverFileTestDataLogFileName = "maneuver_test.mmlog";
        constexpr std::uint16_t kManeuverQueueCompletionHoldMs = 50U;

        MazeMap::DirectionalLocation ManeuverFileTestStartLocation() noexcept
        {
            return MazeMap::DirectionalLocation(
                MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)),
                MazeMap::Up);
        }
    }

    class ManeuverFileTestMode::Implementation final
    {
    public:
        explicit Implementation(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _speedVehicle(runtime.SpeedVehicle())
            , _sensors(runtime.Sensors())
            , _drive(runtime.Drive())
        {
            ResetRunState();
        }

        bool Begin()
        {
            ResetRunState();
            (void)_runtime.CloseUtilityDataLog();
            if (!_runtime.RegisterModeFaultHandler(&Implementation::HandleRuntimeFault, this, kManeuverFileTestFaultSource))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return Fail("Hardware setup failed");
            }

            ResetStartupTrace("mode:maneuver_file_test");
            (void)_runtime.AppendTextLogLine("Maneuver file test mode");
            (void)_runtime.AppendTextLogLine("Load and execute the maneuver queue stored in test.txt.");
            if (!_drive.Begin())
            {
                return Fail("Drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();
            if (!_sensors.Begin())
            {
                return Fail("Sensor init failed");
            }

            _currentLocation = ManeuverFileTestStartLocation();
            _drive.SetStartPoint(_currentLocation);
            _drive.Brake();
            if (_runtime.ManeuverExecutorService().Active())
            {
                _runtime.ManeuverExecutorService().CancelActivePhase();
            }

            if (!_runtime.AppendTextLogFormatted(
                    "Maneuver file test start pose: cell=(%d,%d),dir=%s",
                    static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetX()),
                    static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetY()),
                    DirectionName(_currentLocation.GetDirection())))
            {
                return false;
            }

            return true;
        }

        void Run()
        {
            if (_faulted)
            {
                return;
            }

            if (!LoadManeuverQueueFromSd("test.txt", _queue))
            {
                _drive.Brake();
                CloseTelemetryLog();
                return;
            }

            if (!OpenTelemetryLog())
            {
                (void)_runtime.AppendTextLogLine(
                    "Maneuver test telemetry log unavailable; continuing without telemetry file");
            }
            else if (!WriteTelemetryEvent("source", "test.txt"))
            {
                (void)_runtime.AppendTextLogLine(
                    "Maneuver test source metadata write failed; continuing without telemetry file");
                CloseTelemetryLog();
            }
            else
            {
                (void)LogLoadedManeuverQueue(_queue);
            }

            (void)_runtime.AppendTextLogFormatted(
                "Loaded maneuver test queue with %u maneuvers",
                static_cast<unsigned>(_queue.size()));

            _phase = Phase::StartupSettleStart;

            bool began = false;
            bool ok = false;
            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &Implementation::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                ok = Fail("Maneuver file test loop session start failed");
            }
            else
            {
                began = true;
                const LoopController::SessionResult result = _loopController.Run();
                ok = (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
            }

            if (began)
            {
                _loopController.EndSession();
            }

            if (_runtime.ManeuverExecutorService().Active())
            {
                _runtime.ManeuverExecutorService().CancelActivePhase();
            }

            _drive.Brake();
            CloseTelemetryLog();
            if (ok)
            {
                (void)_runtime.AppendTextLogLine("Maneuver file test complete");
            }
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            StartupSettleStart,
            QueueLaunch,
            FinalHoldStart,
            Complete
        };

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            const std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<Implementation*>(context);
            if (self == nullptr)
            {
                services.Fault("Maneuver file test callback context was not installed");
                return LoopController::ControlVector::Brake;
            }

            return self->RunTick(loopEndTimeUs, state, services);
        }

        static void HandleRuntimeFault(void* context, const char* reason) noexcept
        {
            if (context != nullptr)
            {
                static_cast<Implementation*>(context)->OnRuntimeFault(reason);
            }
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = Config::kControlPeriodUs;
            return options;
        }

        LoopController::ControlVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            (void)loopEndTimeUs;
            (void)state;
            switch (_phase)
            {
            case Phase::StartupSettleStart:
                if (!LaunchHoldRoutine(
                        Config::kObservationSettleMs,
                        "startup_settle",
                        Phase::QueueLaunch,
                        services))
                {
                    services.Fault("Failed to begin maneuver test startup settle");
                }
                return LoopController::ControlVector::Brake;

            case Phase::QueueLaunch:
            {
                LoopController::ModeCallbacks continuation{};
                continuation.onModeWork = &Implementation::ModeWorkThunk;
                continuation.context = this;
                _phase = Phase::FinalHoldStart;
                if (!_runtime.ManeuverExecutorService().ProceedToManeuverExecutionRoutine(
                        _queue,
                        FinalLimits(),
                        false,
                        _currentLocation,
                        continuation,
                        services,
                        BuildManeuverExecutorHooks(true)))
                {
                    services.Fault("Failed to launch maneuver test queue routine");
                }
                return LoopController::ControlVector::Brake;
            }

            case Phase::FinalHoldStart:
                if (!LaunchHoldRoutine(
                        kManeuverQueueCompletionHoldMs,
                        "final_hold",
                        Phase::Complete,
                        services))
                {
                    services.Fault("Failed to begin maneuver test completion hold");
                }
                return LoopController::ControlVector::Brake;

            case Phase::Complete:
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                services.Fault("Maneuver file test phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        bool LaunchHoldRoutine(
            const std::uint16_t durationMs,
            const char* phaseName,
            const Phase nextPhase,
            LoopController::TickServices& services)
        {
            if (!BeginTelemetryPhase(phaseName))
            {
                return false;
            }

            LoopController::ModeCallbacks continuation{};
            continuation.onModeWork = &Implementation::ModeWorkThunk;
            continuation.context = this;
            _phase = nextPhase;
            return _runtime.ManeuverExecutorService().BeginHoldRoutine(
                durationMs,
                true,
                continuation,
                services,
                BuildManeuverExecutorHooks(false));
        }

        ManeuverExecutor::Hooks BuildManeuverExecutorHooks(const bool includeQueueHooks) noexcept
        {
            ManeuverExecutor::Hooks hooks{};
            hooks.context = this;
            hooks.onSample = &Implementation::ManeuverExecutorSampleHook;
            if (includeQueueHooks)
            {
                hooks.onQueueEntryBegin = &Implementation::ManeuverExecutorQueueEntryBeginHook;
                hooks.onQueueEntryComplete = &Implementation::ManeuverExecutorQueueEntryCompleteHook;
            }
            return hooks;
        }

        static bool ManeuverExecutorSampleHook(
            void* context,
            const bool stationary,
            const LoopController::ModeState& state)
        {
            auto* const self = static_cast<Implementation*>(context);
            if (self == nullptr || !self->_telemetryLoggingEnabled)
            {
                return self != nullptr;
            }

            Runtime::PopulateDiagnosticLogRow(
                self->_telemetryLogRow,
                static_cast<std::uint32_t>(self->_telemetrySampleCount),
                static_cast<std::uint32_t>(self->_telemetryPhaseId),
                stationary,
                state,
                self->_drive);
            if (self->_runtime.LogUtilityDataRow(self->_telemetryLogRow))
            {
                ++self->_telemetrySampleCount;
                return true;
            }

            return self->Fail("Failed to write maneuver test sample");
        }

        static bool ManeuverExecutorQueueEntryBeginHook(
            void* context,
            const std::uint16_t index,
            const MazeMap::ManeuverInstance& entry,
            const MazeMap::DirectionalLocation& location)
        {
            auto* const self = static_cast<Implementation*>(context);
            if (self == nullptr)
            {
                return false;
            }

            self->_currentLocation = location;
            const MazeMap::CellCoordinates cell = static_cast<MazeMap::CellCoordinates>(location.GetLocation());
            char codeName[24] = {};
            char traceLine[160] = {};
            FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
            snprintf(
                traceLine,
                sizeof(traceLine),
                "begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
                static_cast<unsigned>(index),
                codeName,
                cell.GetX(),
                cell.GetY(),
                DirectionName(location.GetDirection()),
                entry.getEntrySpeed(),
                entry.getExitSpeed());
            (void)self->WriteTextLogEvent("maneuver_begin", traceLine);

            char phaseName[48] = {};
            snprintf(phaseName, sizeof(phaseName), "maneuver_%u_%s", static_cast<unsigned>(index), codeName);
            return self->BeginTelemetryPhase(phaseName);
        }

        static bool ManeuverExecutorQueueEntryCompleteHook(
            void* context,
            const std::uint16_t index,
            const MazeMap::ManeuverInstance& entry,
            const MazeMap::DirectionalLocation& location)
        {
            auto* const self = static_cast<Implementation*>(context);
            if (self == nullptr)
            {
                return false;
            }

            self->_currentLocation = location;
            const MazeMap::CellCoordinates cell = static_cast<MazeMap::CellCoordinates>(location.GetLocation());
            char codeName[24] = {};
            char traceLine[160] = {};
            FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
            snprintf(
                traceLine,
                sizeof(traceLine),
                "end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
                static_cast<unsigned>(index),
                codeName,
                cell.GetX(),
                cell.GetY(),
                DirectionName(location.GetDirection()),
                self->_drive.GetPose().xMeters,
                self->_drive.GetPose().yMeters,
                RAD_TO_DEG_F * self->_drive.GetPose().yawRad);
            (void)self->WriteTextLogEvent("maneuver_end", traceLine);
            return true;
        }

        MotionLimits FinalLimits() const noexcept
        {
            MotionLimits limits{};
            limits.maxSpeedMps = _speedVehicle.GetMaxSpeed() * Config::kSpeedRunScale;
            limits.accelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
            limits.decelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
            limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity() * Config::kSpeedRunScale;
            limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration() * Config::kSpeedRunScale;
            return limits;
        }

        bool LoadManeuverQueueFromSd(const char* fileName, MazeMap::ManeuverQueue& queue)
        {
#if defined(ARDUINO_TEENSY41)
            File file = SD.open(fileName, FILE_READ);
            if (!file)
            {
                (void)WriteTextLogEvent("trace", "maneuver_file_unavailable");
                (void)_runtime.AppendTextLogLine("Maneuver file unavailable; skipping maneuver-file test");
                return false;
            }

            MazeMap::ManeuverPath path;
            char line[128] = {};
            std::uint16_t lineNumber = 0U;
            while (file.available())
            {
                const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
                line[lineLength] = '\0';
                ++lineNumber;

                char* hashComment = std::strchr(line, '#');
                if (hashComment != nullptr)
                {
                    *hashComment = '\0';
                }

                char* slashComment = std::strstr(line, "//");
                if (slashComment != nullptr)
                {
                    *slashComment = '\0';
                }

                for (char* token = std::strtok(line, ", \t\r;"); token != nullptr; token = std::strtok(nullptr, ", \t\r;"))
                {
                    MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                    if (!TryParseManeuverCodeToken(token, code))
                    {
                        char message[96] = {};
                        snprintf(message, sizeof(message), "Maneuver file token issue on line %u: %s", lineNumber, token);
                        file.close();
                        (void)WriteTextLogEvent("trace", "maneuver_file_parse_issue");
                        (void)_runtime.AppendTextLogLine(message);
                        return false;
                    }
                    if (!path.push_back(code))
                    {
                        file.close();
                        (void)WriteTextLogEvent("trace", "maneuver_file_path_capacity_reached");
                        (void)_runtime.AppendTextLogLine("Maneuver file exceeded path capacity; skipping maneuver-file test");
                        return false;
                    }
                }
            }

            file.close();

            if (path.GetSize() == 0)
            {
                (void)WriteTextLogEvent("trace", "maneuver_file_empty");
                (void)_runtime.AppendTextLogLine("Maneuver file did not contain any maneuvers");
                return false;
            }

            queue.clear();
            _currentLocation = ManeuverFileTestStartLocation();
            if (!queue.push_back(path, _currentLocation))
            {
                (void)WriteTextLogEvent("trace", "maneuver_queue_build_issue");
                (void)_runtime.AppendTextLogLine("Maneuver file could not be converted into a queue");
                return false;
            }

            queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
            _runtime.ManeuverExecutorService().ApplyAsymmetricQueueLimits(
                queue,
                FinalLimits(),
                0.0f,
                0.0f);
            return true;
#else
            (void)fileName;
            (void)queue;
            (void)WriteTextLogEvent("trace", "maneuver_file_teensy_target_required");
            (void)_runtime.AppendTextLogLine("Maneuver-file test mode requires the Teensy target");
            return false;
#endif
        }

        bool LogLoadedManeuverQueue(const MazeMap::ManeuverQueue& queue)
        {
            if (!_telemetryLoggingEnabled)
            {
                return true;
            }

            char message[128] = {};
            snprintf(message, sizeof(message), "count,%u", static_cast<unsigned>(queue.size()));
            if (!WriteTelemetryEvent("queue", message))
            {
                (void)_runtime.AppendTextLogLine("Maneuver queue logging unavailable; continuing without queue metadata");
                CloseTelemetryLog();
                return true;
            }

            for (std::uint16_t i = 0; i < queue.size(); ++i)
            {
                char codeName[24] = {};
                char queueLine[160] = {};
                FormatManeuverCodeName(queue[i].getCode(), codeName, sizeof(codeName));
                snprintf(
                    queueLine,
                    sizeof(queueLine),
                    "%u,%s,%.6f,%.6f",
                    static_cast<unsigned>(i),
                    codeName,
                    queue[i].getEntrySpeed(),
                    queue[i].getExitSpeed());
                if (!WriteTelemetryEvent("queue_entry", queueLine))
                {
                    (void)_runtime.AppendTextLogLine("Maneuver queue entry logging unavailable; continuing without queue metadata");
                    CloseTelemetryLog();
                    return true;
                }
            }

            return true;
        }

        bool BeginTelemetryPhase(const char* name)
        {
            if (!_telemetryLoggingEnabled)
            {
                return true;
            }

            ++_telemetryPhaseId;
            return _runtime.WriteTextLogPhase(_telemetryPhaseId, micros(), name);
        }

        bool OpenTelemetryLog()
        {
            _telemetryLoggingEnabled =
                MazeMap::App::Internal::Runtime::BeginDiagnosticUtilityTelemetryLog(
                    _runtime,
                    _sensors,
                    _telemetryLogRow,
                    kManeuverFileTestDataLogFileName,
                    "maneuver_test",
                    _telemetryPhaseId,
                    _telemetrySampleCount);
            return _telemetryLoggingEnabled;
        }

        bool WriteTelemetryEvent(const char* type, const char* message)
        {
            return !_telemetryLoggingEnabled || _runtime.WriteTextLogEntry(micros(), type, message);
        }

        bool WriteTextLogEvent(const char* type, const char* message)
        {
            return _runtime.WriteTextLogEntry(
                kManeuverFileTestTextLogSource,
                micros(),
                type,
                message);
        }

        bool Fail(const char* reason)
        {
            return _runtime.FailActiveMode(reason);
        }

        void OnRuntimeFault(const char* reason) noexcept
        {
            _faulted = true;
            const char* const resolvedReason =
                (reason != nullptr && reason[0] != '\0') ?
                    reason :
                    "maneuver_file_test_fault";
            (void)WriteTextLogEvent("fault", resolvedReason);
            if (_telemetryLoggingEnabled)
            {
                (void)WriteTelemetryEvent("fault", resolvedReason);
            }
            _drive.Brake();
            CloseTelemetryLog();
        }

        void CloseTelemetryLog()
        {
            _runtime.FlushTextLog();
            (void)_runtime.CloseUtilityDataLog();
            _telemetryLoggingEnabled = false;
        }

        void ResetRunState() noexcept
        {
            _faulted = false;
            _telemetryLoggingEnabled = false;
            _telemetryPhaseId = 0UL;
            _telemetrySampleCount = 0UL;
            _telemetryLogRow = {};
            _queue.clear();
            _currentLocation = ManeuverFileTestStartLocation();
            _phase = Phase::Idle;
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _speedVehicle;
        RuntimeSensorSuite& _sensors;
        DriveBase& _drive;
        MazeMap::ManeuverQueue _queue{};
        MazeMap::DirectionalLocation _currentLocation{ ManeuverFileTestStartLocation() };
        bool _faulted{};
        bool _telemetryLoggingEnabled{};
        unsigned long _telemetryPhaseId{};
        unsigned long _telemetrySampleCount{};
        DiagnosticLogRow _telemetryLogRow{};
        Phase _phase{ Phase::Idle };
    };

    ManeuverFileTestMode::ManeuverFileTestMode(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    ManeuverFileTestMode::~ManeuverFileTestMode() = default;

    bool ManeuverFileTestMode::Begin()
    {
        return _impl->Begin();
    }

    void ManeuverFileTestMode::Run()
    {
        _impl->Run();
    }

    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ManeuverFileTest,
            BootModeCategory::Utility,
            "maneuver_file_test",
            "Load and execute the maneuver queue stored in test.txt.",
            "logging.txt; maneuver test telemetry mmlog",
            &GetManeuverFileTestMode,
            "GetManeuverFileTestMode",
            "ManeuverFileTestMode.cpp",
            "mode-local setup; optional telemetry log setup; test.txt load; shared maneuver execution",
            "CoreConfig maneuver limits; Maneuver classes; shared runtime drive execution",
            "test.txt is the selected input artifact for this utility workflow",
            "maneuver_test.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetManeuverFileTestMode()
    {
        static ManeuverFileTestMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
