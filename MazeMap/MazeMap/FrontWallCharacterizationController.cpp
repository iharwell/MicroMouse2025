#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

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

class FrontWallCharacterizationController : public IApplicationMode
{
public:
    explicit FrontWallCharacterizationController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _faulted(false)
    {
    }

    bool Begin() override
    {
        _faulted = false;
        _phaseFn = nullptr;
        _pauseAction = PauseAction::None;
        _holdCompletionAction = CompletionAction::StartCapture;
        _holdPhaseName = nullptr;
        _holdDurationMs = 0U;
        _holdStartMs = 0UL;
        _holdStarted = false;
        _captureStorage = {};
        _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
        _captureStartDistanceM = 0.0f;
        _captureCommandedSpeedMps = 0.0f;
        _captureNextStoredDistanceM = 0.0f;
        _captureCollapsedConsecutiveSamples = 0U;
        _captureStartMs = 0UL;
        _captureTimeoutMs = 0UL;
        _captureCompletionReason = "unknown";
        _captureElapsedBudgetLogged = false;
        _captureStarted = false;
        if (!_runtime.RegisterModeFaultHandler(&FrontWallCharacterizationController::HandleRuntimeFault, this, "front_wall_characterization"))
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
        const bool sensorsOk = _sensors.Begin(FrontWallCharacterizationConfig::kControlPeriodUs);
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
        if (_faulted)
        {
            return;
        }

        bool ok = StartHoldPhase(
            "startup_settle",
            FrontWallCharacterizationConfig::kStartupSettleMs,
            CompletionAction::StartCapture);
        if (ok)
        {
            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &FrontWallCharacterizationController::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                _phaseFn = nullptr;
                ok = Fail("Front wall characterization loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                _phaseFn = nullptr;
                _pauseAction = PauseAction::None;
                ok = (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
            }
        }

        _drive.Brake();
        SetMissionLevelFanEnabled(false);
        if (ok)
        {
            (void)_runtime.AppendTextLogLine("Front wall characterization complete and persisted.");
        }
        _runtime.CloseTextLog();
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    using PhaseFn = LoopController::ControlVector (FrontWallCharacterizationController::*)(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    enum class CompletionAction : std::uint8_t
    {
        StartCapture,
        CompleteSession
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
        if ((self == nullptr) || (self->_phaseFn == nullptr))
        {
            services.Fault("Front wall characterization phase callback was not installed");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_phaseFn)(loopEndTimeUs, state, services);
    }

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<FrontWallCharacterizationController*>(context)->OnRuntimeFault(reason);
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    bool _faulted;
    FrontWallCharacterizationLogRow _logRow{};
    PhaseFn _phaseFn{};
    PauseAction _pauseAction{ PauseAction::None };
    CompletionAction _holdCompletionAction{ CompletionAction::StartCapture };
    const char* _holdPhaseName{};
    std::uint16_t _holdDurationMs{};
    unsigned long _holdStartMs{};
    bool _holdStarted{};
    MazeMap::FrontWallCharacterizationStorage _captureStorage{};
    Eigen::Vector2f _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
    float _captureStartDistanceM = 0.0f;
    float _captureCommandedSpeedMps = 0.0f;
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

    bool StartHoldPhase(
        const char* phaseName,
        uint16_t durationMs,
        CompletionAction completionAction)
    {
        if (phaseName != nullptr && phaseName[0] != '\0')
        {
            char line[96] = {};
            snprintf(line, sizeof(line), "front_wall_characterization:phase=%s", phaseName);
            AppendStartupTrace(line);
        }

        _holdCompletionAction = completionAction;
        _holdPhaseName = phaseName;
        _holdDurationMs = durationMs;
        _holdStartMs = 0UL;
        _holdStarted = false;
        _phaseFn = &FrontWallCharacterizationController::HoldStationaryTick;
        return true;
    }

    bool StartCaptureCurvePhase()
    {
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
        _captureCommandedSpeedMps = 0.0f;
        _captureCollapsedConsecutiveSamples = 0U;
        _captureStartMs = 0UL;
        _captureCompletionReason = "unknown";
        _captureElapsedBudgetLogged = false;
        _captureStarted = false;
        _phaseFn = &FrontWallCharacterizationController::CaptureCurveTick;
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
            return _faulted ?
                LoopController::PauseDisposition::Complete() :
                LoopController::PauseDisposition::StopByRuntime(
                    "Front wall characterization persist/export failed");
        }

        _captureStorage = storage;
        if (!StartHoldPhase(
                "post_capture_settle",
                FrontWallCharacterizationConfig::kPostCaptureSettleMs,
                CompletionAction::CompleteSession))
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Front wall characterization post-capture settle start failed");
        }

        return LoopController::PauseDisposition::Resume();
    }

    LoopController::ControlVector HoldStationaryTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        (void)state;
        if (!_holdStarted)
        {
            _holdStarted = true;
            _holdStartMs = millis();
        }

        if (static_cast<unsigned long>(millis() - _holdStartMs) >= _holdDurationMs)
        {
            if (_holdCompletionAction == CompletionAction::CompleteSession)
            {
                services.RequestEndLoop();
            }
            else if (!StartCaptureCurvePhase())
            {
                services.Fault("Front wall characterization capture phase start failed");
            }
        }

        return LoopController::ControlVector::Brake;
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
            _captureStarted = true;
            _captureTargetHeading = state.estimate.headingUnit;
            _captureStartDistanceM = _drive.GetAverageDistanceMeters();
            _captureStartMs = millis();
            StoreCurveSample(_captureStorage, 0.0f, snapshot);
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
            return requestPersistPause(traveledDistanceM, "collapsed_to_zero");
        }

        if (traveledDistanceM >= FrontWallCharacterizationConfig::kMaxReverseTravelM)
        {
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

        _captureCommandedSpeedMps = (std::min)(
            FrontWallCharacterizationConfig::kReverseSpeedMps,
            _captureCommandedSpeedMps + (FrontWallCharacterizationConfig::kReverseAccelMps2 * state.dtSeconds));

        const float headingErrorRad = HeadingErrorRad(_captureTargetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * state.estimate.angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -FrontWallCharacterizationConfig::kMaxAngularCommandRadps,
            FrontWallCharacterizationConfig::kMaxAngularCommandRadps);
        return _drive.PointControlVector(
            -_captureCommandedSpeedMps,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
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

    void OnRuntimeFault(const char* reason) noexcept
    {
        _faulted = true;
        if (reason != nullptr && reason[0] != '\0')
        {
            AppendStartupTrace(reason);
        }
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

