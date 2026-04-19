#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "StartupCalibration.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::Drive;
using MazeMap::App::Internal::SharedRobotRuntime;
using MazeMap::App::Internal::StartupCalibration;

#define FRONT_WALL_CHARACTERIZATION_LOG_FIELDS(X) \
    X(std::uint32_t, index) \
    X(float,         distance_m) \
    X(float,         front_left_ambient) \
    X(float,         front_left_lit) \
    X(float,         front_left_delta) \
    X(float,         front_right_ambient) \
    X(float,         front_right_lit) \
    X(float,         front_right_delta)

MMLOG_DEFINE_ROW(FrontWallCharacterizationLogRow, FRONT_WALL_CHARACTERIZATION_LOG_FIELDS);

namespace
{
    MotionLimits BuildReverseCaptureLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = FrontWallCharacterizationConfig::kReverseSpeedMps;
        limits.accelMps2 = FrontWallCharacterizationConfig::kReverseAccelMps2;
        limits.decelMps2 = FrontWallCharacterizationConfig::kReverseAccelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }
}

class FrontWallCharacterizationController : public IApplicationMode
{
public:
    explicit FrontWallCharacterizationController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    bool Begin() override
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(&FrontWallCharacterizationController::TeardownOnRuntimeFault, this, "front_wall_characterization"))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        (void)ResetStartupTrace("mode:front_wall_characterization");
        (void)_runtime.AppendTextLogLine("Front wall characterization mode");
        (void)_runtime.AppendTextLogLine("Enter by shorting pins 39-40 at boot.");
        (void)_runtime.AppendTextLogLine("Place the nose on a wall in a dark area, then power on.");
        AppendStartupTrace("front_wall_characterization:begin");
        AppendStartupTrace("front_wall_characterization:sd_ready_wait_begin");
        (void)_runtime.AppendTextLogFormatted(
            "SD card ready; waiting %lu ms before starting.",
            static_cast<unsigned long>(FrontWallCharacterizationConfig::kPostSdReadyDelayMs));
        delay(FrontWallCharacterizationConfig::kPostSdReadyDelayMs);
        AppendStartupTrace("front_wall_characterization:sd_ready_wait_complete");

        SetMissionLevelFanEnabled(false);

        const bool driveOk = _drive.Begin();
        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        const bool sensorsOk = _startupCalibration.BringUp();
        _drive.UseNominalWheelControlProfile();

        MazeMap::FrontWallCharacterizationStorage storedCurve{};
        if (TryReadPersistedFrontWallCharacterization(storedCurve))
        {
            char line[160] = {};
            snprintf(
                line,
                sizeof(line),
                "front_wall_characterization:existing_curve_loaded,samples=%u,terminal_distance_m=%.4f,reverse_speed_mps=%.3f",
                static_cast<unsigned>(storedCurve.sampleCount),
                storedCurve.terminalDistanceM,
                storedCurve.commandedReverseSpeedMps);
            AppendStartupTrace(line);
            (void)_runtime.AppendTextLogLine("Existing front-wall curve will be replaced on success.");
        }

        if (!driveOk)
        {
            return Fail("Drive initialization failed");
        }
        if (!sensorsOk)
        {
            return Fail("Sensor initialization failed");
        }

        return true;
    }

    void Run() override
    {
        _phase = Phase::LaunchStartupSettle;

        bool ok = false;
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &FrontWallCharacterizationController::ModeWorkThunk;
        callbacks.context = this;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            ok = Fail("Front wall characterization loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            ok = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }

        _driveService.Cancel();
        _startupCalibration.Cancel();
        _drive.Brake();
        _phase = Phase::Idle;
        _pauseAction = PauseAction::None;
        SetMissionLevelFanEnabled(false);
        if (ok)
        {
            (void)_runtime.AppendTextLogLine("Front wall characterization complete and persisted.");
        }
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    enum class Phase : std::uint8_t
    {
        Idle,
        LaunchStartupSettle,
        RunStartupSettle,
        Capture,
        LaunchPostCaptureSettle,
        RunPostCaptureSettle,
        Complete
    };

    enum class PauseAction : std::uint8_t
    {
        None,
        PersistAndExport
    };

    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<FrontWallCharacterizationController*>(context);
        if (self == nullptr)
        {
            services.Fault("Front wall characterization callback context was null");
            return LoopController::ControlVector::Brake;
        }

        return self->RunTick(loopEndTimeUs, state, services);
    }

    static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        (void)reason;
        auto* const self = static_cast<FrontWallCharacterizationController*>(context);
        if (self == nullptr)
        {
            return;
        }

        self->_phase = Phase::Idle;
        self->_pauseAction = PauseAction::None;
        self->_driveService.Cancel();
        self->_startupCalibration.Cancel();
        self->_drive.Brake();
        SetMissionLevelFanEnabled(false);
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _vehicle;
    DriveBase& _drive;
    Drive& _driveService;
    StartupCalibration& _startupCalibration;
    FrontWallCharacterizationLogRow _logRow{};
    Phase _phase{ Phase::Idle };
    PauseAction _pauseAction{ PauseAction::None };
    MazeMap::FrontWallCharacterizationStorage _captureStorage{};
    Eigen::Vector2f _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
    float _captureStartDistanceM = 0.0f;
    float _captureNextStoredDistanceM = 0.0f;
    std::uint8_t _captureCollapsedConsecutiveSamples = 0U;
    unsigned long _captureStartMs = 0UL;
    unsigned long _captureTimeoutMs = 0UL;
    const char* _captureCompletionReason = "unknown";
    bool _captureElapsedBudgetLogged = false;
    bool _captureStarted = false;

    static LoopController::PauseDisposition PauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<FrontWallCharacterizationController*>(context);
        if (self == nullptr)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Front wall characterization pause callback context was null");
        }

        return self->OnPauseGranted(pause);
    }

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = FrontWallCharacterizationConfig::kControlPeriodUs;
        return options;
    }

    void ResetState() noexcept
    {
        _phase = Phase::Idle;
        _pauseAction = PauseAction::None;
        _logRow = {};
        _captureStorage = {};
        _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
        _captureStartDistanceM = 0.0f;
        _captureNextStoredDistanceM = 0.0f;
        _captureCollapsedConsecutiveSamples = 0U;
        _captureStartMs = 0UL;
        _captureTimeoutMs = 0UL;
        _captureCompletionReason = "unknown";
        _captureElapsedBudgetLogged = false;
        _captureStarted = false;
    }

    bool StartHoldPhase(const char* phaseName, std::uint16_t durationMs)
    {
        if (phaseName != nullptr && phaseName[0] != '\0')
        {
            char line[96] = {};
            snprintf(line, sizeof(line), "front_wall_characterization:phase=%s", phaseName);
            AppendStartupTrace(line);
        }

        _driveService.Cancel();
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.StartHold(durationMs, true);
        return _driveService.Active();
    }

    bool StartCaptureCurvePhase()
    {
        AppendStartupTrace("front_wall_characterization:phase=reverse_capture");
        _captureStorage = {};
        _captureStorage.distanceStepM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        _captureStorage.commandedReverseSpeedMps = FrontWallCharacterizationConfig::kReverseSpeedMps;
        _captureStorage.zeroThresholdDifferentialLight = FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold;
        _captureNextStoredDistanceM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        _captureTimeoutMs =
            static_cast<unsigned long>(2000.0f +
                ((1000.0f * FrontWallCharacterizationConfig::kMaxReverseTravelM) /
                    (std::max)(FrontWallCharacterizationConfig::kReverseSpeedMps, 0.01f)));
        _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
        _captureStartDistanceM = 0.0f;
        _captureCollapsedConsecutiveSamples = 0U;
        _captureStartMs = 0UL;
        _captureCompletionReason = "unknown";
        _captureElapsedBudgetLogged = false;
        _captureStarted = false;
        _phase = Phase::Capture;
        return true;
    }

    LoopController::PauseDisposition OnPauseGranted(const LoopController::PauseContext& pause)
    {
        (void)pause;

        if (_pauseAction != PauseAction::PersistAndExport)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Front wall characterization pause granted without a pending action");
        }

        _pauseAction = PauseAction::None;
        _drive.Brake();

        MazeMap::FrontWallCharacterizationStorage storage = _captureStorage;
        if (storage.sampleCount < 4U)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Front wall characterization captured too few samples");
        }

        MazeMap::FinalizeFrontWallCharacterizationStorage(storage);

        char summary[224] = {};
        snprintf(
            summary,
            sizeof(summary),
            "front_wall_characterization:captured,reason=%s,samples=%u,terminal_distance_m=%.4f,fl_start=%.6f,fl_end=%.6f,fr_start=%.6f,fr_end=%.6f",
            _captureCompletionReason,
            static_cast<unsigned>(storage.sampleCount),
            storage.terminalDistanceM,
            storage.frontLeftDifferentialLight[0],
            storage.frontLeftDifferentialLight[storage.sampleCount - 1U],
            storage.frontRightDifferentialLight[0],
            storage.frontRightDifferentialLight[storage.sampleCount - 1U]);
        AppendStartupTrace(summary);
        (void)_runtime.AppendTextLogLine(summary);

        if (!PersistCurve(storage) || !ExportCurveToSd(storage))
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Front wall characterization persist/export failed");
        }

        _captureStorage = storage;
        _phase = Phase::LaunchPostCaptureSettle;
        return LoopController::PauseDisposition::Resume();
    }

    LoopController::ControlVector PollDrivePhase(
        const Phase nextPhase,
        const char* inactiveReason,
        LoopController::TickServices& services)
    {
        if (!_driveService.Active())
        {
            services.Fault(inactiveReason);
            return LoopController::ControlVector::Brake;
        }

        bool done = false;
        const LoopController::ControlVector control = _driveService.GetNextControls(done);
        if (!done)
        {
            return control;
        }

        _phase = nextPhase;
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector RunTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        switch (_phase)
        {
        case Phase::LaunchStartupSettle:
            if (!StartHoldPhase("startup_settle", FrontWallCharacterizationConfig::kStartupSettleMs))
            {
                services.Fault("Front wall characterization startup settle could not start");
            }
            else
            {
                _phase = Phase::RunStartupSettle;
            }
            return LoopController::ControlVector::Brake;

        case Phase::RunStartupSettle:
            return PollDrivePhase(
                Phase::Capture,
                "Front wall characterization startup settle was not active",
                services);

        case Phase::Capture:
            return CaptureCurveTick(loopEndTimeUs, state, services);

        case Phase::LaunchPostCaptureSettle:
            if (!StartHoldPhase("post_capture_settle", FrontWallCharacterizationConfig::kPostCaptureSettleMs))
            {
                services.Fault("Front wall characterization post-capture settle could not start");
            }
            else
            {
                _phase = Phase::RunPostCaptureSettle;
            }
            return LoopController::ControlVector::Brake;

        case Phase::RunPostCaptureSettle:
        {
            if (!_driveService.Active())
            {
                services.Fault("Front wall characterization post-capture settle was not active");
                return LoopController::ControlVector::Brake;
            }

            bool done = false;
            const LoopController::ControlVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = Phase::Complete;
            services.RequestEndLoop();
            return LoopController::ControlVector::Brake;
        }

        case Phase::Complete:
            return LoopController::ControlVector::Brake;

        case Phase::Idle:
        default:
            services.Fault("Front wall characterization phase was not initialized");
            return LoopController::ControlVector::Brake;
        }
    }

    LoopController::ControlVector CaptureCurveTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto requestPersistPause = [this, &services](const float traveledDistanceM, const char* reason)
        {
            _captureCompletionReason = reason;
            _captureStorage.terminalDistanceM = traveledDistanceM;
            _pauseAction = PauseAction::PersistAndExport;
            LoopController::PauseRequest request{};
            request.onPauseGranted = &FrontWallCharacterizationController::PauseThunk;
            request.reason = reason;
            request.flushLogsBeforeGrant = true;
            request.resetClockOnResume = true;
            services.RequestPause(request);
            return LoopController::ControlVector::Brake;
        };

        const SensorSnapshot& snapshot = state.sensors;
        if (!_captureStarted)
        {
            if (!StartCaptureCurvePhase())
            {
                services.Fault("Front wall characterization capture phase start failed");
                return LoopController::ControlVector::Brake;
            }

            _captureStarted = true;
            _captureTargetHeading = state.estimate.headingUnit;
            _captureStartDistanceM = _drive.GetAverageDistanceMeters();
            _captureStartMs = millis();
            StoreCurveSample(_captureStorage, 0.0f, snapshot);

            _driveService.Cancel();
            _driveService.SetLimits(BuildReverseCaptureLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartStraight(
                FrontWallCharacterizationConfig::kMaxReverseTravelM,
                -FrontWallCharacterizationConfig::kReverseSpeedMps,
                0.0f,
                &_captureTargetHeading);
            if (!_driveService.Active())
            {
                services.Fault("Front wall characterization reverse capture could not start");
                return LoopController::ControlVector::Brake;
            }
        }

        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - _captureStartDistanceM);
        if ((_captureStorage.sampleCount < MazeMap::kFrontWallCharacterizationMaxStoredSamples) &&
            ((traveledDistanceM + Config::kDistanceToleranceM) >= _captureNextStoredDistanceM))
        {
            StoreCurveSample(_captureStorage, traveledDistanceM, snapshot);
            _captureNextStoredDistanceM += FrontWallCharacterizationConfig::kStoredDistanceStepM;
        }

        const bool collapsedToZero =
            (snapshot.frontLeft.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold) &&
            (snapshot.frontRight.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold);
        if ((traveledDistanceM >= FrontWallCharacterizationConfig::kMinimumTravelBeforeCollapseCheckM) && collapsedToZero)
        {
            ++_captureCollapsedConsecutiveSamples;
        }
        else
        {
            _captureCollapsedConsecutiveSamples = 0U;
        }

        if (_captureStorage.sampleCount >= MazeMap::kFrontWallCharacterizationMaxStoredSamples)
        {
            return requestPersistPause(traveledDistanceM, "storage_full");
        }

        if (_captureCollapsedConsecutiveSamples >= FrontWallCharacterizationConfig::kCollapsedConsecutiveSamples)
        {
            _driveService.Cancel();
            return requestPersistPause(traveledDistanceM, "collapsed_to_zero");
        }

        if (traveledDistanceM >= FrontWallCharacterizationConfig::kMaxReverseTravelM)
        {
            _driveService.Cancel();
            return requestPersistPause(traveledDistanceM, "max_reverse_travel");
        }

        if (!_captureElapsedBudgetLogged &&
            (static_cast<unsigned long>(millis() - _captureStartMs) >= _captureTimeoutMs))
        {
            char timeoutLine[192] = {};
            snprintf(
                timeoutLine,
                sizeof(timeoutLine),
                "front_wall_characterization:elapsed_budget_reached,travel_m=%.4f,samples=%u,timeout_ms=%lu",
                traveledDistanceM,
                static_cast<unsigned>(_captureStorage.sampleCount),
                _captureTimeoutMs);
            AppendStartupTrace(timeoutLine);
            _captureElapsedBudgetLogged = true;
        }

        bool driveDone = false;
        const LoopController::ControlVector control = _driveService.GetNextControls(driveDone);
        if (driveDone)
        {
            _driveService.Cancel();
            return requestPersistPause(traveledDistanceM, "drive_complete");
        }

        return control;
    }

    static void StoreCurveSample(
        MazeMap::FrontWallCharacterizationStorage& storage,
        float traveledDistanceM,
        const SensorSnapshot& snapshot)
    {
        if (storage.sampleCount >= MazeMap::kFrontWallCharacterizationMaxStoredSamples ||
            !std::isfinite(traveledDistanceM) ||
            traveledDistanceM < 0.0f)
        {
            return;
        }

        const uint16_t index = storage.sampleCount;
        storage.distanceM[index] = traveledDistanceM;
        storage.frontLeftAmbientLight[index] = snapshot.frontLeft.ambientLight;
        storage.frontLeftLitLight[index] = snapshot.frontLeft.litLight;
        storage.frontLeftDifferentialLight[index] = snapshot.frontLeft.differentialLight;
        storage.frontRightAmbientLight[index] = snapshot.frontRight.ambientLight;
        storage.frontRightLitLight[index] = snapshot.frontRight.litLight;
        storage.frontRightDifferentialLight[index] = snapshot.frontRight.differentialLight;
        ++storage.sampleCount;
    }

    bool PersistCurve(const MazeMap::FrontWallCharacterizationStorage& storage)
    {
        if (!WritePersistedFrontWallCharacterization(storage))
        {
            return Fail("Failed to persist front wall characterization");
        }

        MazeMap::FrontWallCharacterizationStorage verify{};
        if (!TryReadPersistedFrontWallCharacterization(verify))
        {
            return Fail("Failed to verify persisted front wall characterization");
        }

        char line[160] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:persisted,samples=%u,terminal_distance_m=%.4f",
            static_cast<unsigned>(verify.sampleCount),
            verify.terminalDistanceM);
        AppendStartupTrace(line);
        (void)_runtime.AppendTextLogLine(line);
        return true;
    }

    bool ExportCurveToSd(const MazeMap::FrontWallCharacterizationStorage& storage)
    {
        if (!MazeMap::IsValidFrontWallCharacterizationStorage(storage))
        {
            return Fail("Invalid front wall characterization cannot be exported");
        }

        char fileName[32] = {};
        if (!_runtime.OpenUtilityDataLog(
                fileName,
                sizeof(fileName),
                nullptr,
                "fwc%03u.mmlog",
                "front_wall_characterization.mmlog"))
        {
            return Fail("Front wall characterization log name unavailable");
        }

        if (!_runtime.WriteUtilityDataLogMetadata("mode", "front_wall_characterization")) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("samples", static_cast<unsigned long>(storage.sampleCount))) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadataFloat("distance_step_m", storage.distanceStepM, 6)) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadataFloat("reverse_speed_mps", storage.commandedReverseSpeedMps, 6)) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadataFloat("zero_threshold_differential_light", storage.zeroThresholdDifferentialLight, 6)) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadataFloat("terminal_distance_m", storage.terminalDistanceM, 6)) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return Fail("Front wall characterization log metadata failed");
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return Fail("Front wall characterization log metadata failed");

        _logRow = {};
        if (!_runtime.BeginUtilityDataLogSchema(_logRow))
        {
            return Fail("Front wall characterization log open failed");
        }

        for (uint16_t index = 0U; index < storage.sampleCount; ++index)
        {
            _logRow = {};
            _logRow.index = index;
            _logRow.distance_m = storage.distanceM[index];
            _logRow.front_left_ambient = storage.frontLeftAmbientLight[index];
            _logRow.front_left_lit = storage.frontLeftLitLight[index];
            _logRow.front_left_delta = storage.frontLeftDifferentialLight[index];
            _logRow.front_right_ambient = storage.frontRightAmbientLight[index];
            _logRow.front_right_lit = storage.frontRightLitLight[index];
            _logRow.front_right_delta = storage.frontRightDifferentialLight[index];
            if (!_runtime.LogUtilityDataRow(_logRow))
            {
                return Fail("Front wall characterization log write failed");
            }
        }

        if (!_runtime.CloseUtilityDataLog())
        {
            return Fail("Front wall characterization log write failed");
        }

        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:log_exported,file=%s,samples=%u",
            fileName,
            static_cast<unsigned>(storage.sampleCount));
        AppendStartupTrace(line);
        (void)_runtime.AppendTextLogLine(line);
        return true;
    }

    bool Fail(const char* reason)
    {
        return _runtime.FailActiveMode(reason);
    }

};

namespace MazeMap::App::Internal
{
    IApplicationMode& GetFrontWallCharacterizationMode();

    const BootModeDescriptor& GetFrontWallCharacterizationBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::FrontWallCharacterization,
            BootModeCategory::Utility,
            "front_wall_characterization",
            "Capture and save the front-wall sensor response curve.",
            "logging.txt; front-wall mmlog; saved front-wall curve",
            &GetFrontWallCharacterizationMode,
            "GetFrontWallCharacterizationMode",
            "FrontWallCharacterizationController.cpp",
            "startup settle; reverse capture; persist; export; settle",
            "FrontWallCharacterizationConfig; mission drive/sensor tuning",
            "Reverse speed, max travel, sample spacing, and collapse threshold are local.",
            "fwc%03u.mmlog or front_wall_characterization.mmlog; EEPROM front-wall curve",
        };
        return descriptor;
    }

    IApplicationMode& GetFrontWallCharacterizationMode()
    {
        static FrontWallCharacterizationController mode(GetSharedRobotRuntime());
        return mode;
    }
}

