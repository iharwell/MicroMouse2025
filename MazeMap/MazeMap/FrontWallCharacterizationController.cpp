#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootFramework.h"
#include "BootModeDescriptor.h"
#include "Drive.h"
#include "DriveBase.h"
#include "IApplicationMode.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "SharedRobotRuntime.h"
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

namespace MazeMap
{
    namespace
    {
        float AverageEncoderDistanceM(const SensorSnapshot& snapshot) noexcept
        {
            return snapshot.AverageEncoderDistanceM();
        }
    }

    MotionLimits BuildReverseCaptureLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps(FrontWallCharacterizationConfig::kReverseSpeedMps);
        limits.SetAccelMps2(FrontWallCharacterizationConfig::kReverseAccelMps2);
        limits.SetDecelMps2(FrontWallCharacterizationConfig::kReverseAccelMps2);
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxYawRate());
        limits.SetAngularAccelRadps2(vehicle.GetMaxYawAccel());
        return limits;
    }

class FrontWallCharacterizationController : public MazeMap::App::Internal::IApplicationMode
{
public:
    explicit FrontWallCharacterizationController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.Vehicle())
        , _drive(runtime.DriveBase())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    void SetupMode(MazeMap::App::Internal::BootFramework& framework) override
    {
        _bootFramework = &framework;
        ResetState();
        (void)_runtime.AppendTextLogLine("Front wall characterization mode");
        (void)_runtime.AppendTextLogLine("Enter by shorting pins 39-40 at boot.");
        (void)_runtime.AppendTextLogLine("Place the nose on a wall in a dark area, then power on.");
        AppendStartupTraceLine("front_wall_characterization:begin");
        AppendStartupTraceLine("front_wall_characterization:sd_ready_wait_begin");
        (void)_runtime.AppendTextLogFormatted(
            "SD card ready; waiting %lu ms before starting.",
            static_cast<unsigned long>(FrontWallCharacterizationConfig::kPostSdReadyDelayMs));
        delay(FrontWallCharacterizationConfig::kPostSdReadyDelayMs);
        AppendStartupTraceLine("front_wall_characterization:sd_ready_wait_complete");

        _vehicle.SetFanDuty(0.0f);

        _drive.ClearCommandEvidence();
        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);

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
            AppendStartupTraceLine(line);
            (void)_runtime.AppendTextLogLine("Existing front-wall curve will be replaced on success.");
        }

        _phase = Phase::LaunchStartupSettle;
        const auto& runtimeState = _runtime.RuntimeState();
        _loopController.StageNextSessionState(
            FrontWallCharacterizationConfig::kControlPeriodUs,
            runtimeState.GetPositionX(),
            runtimeState.GetPositionY());
    }

private:
    using CommandVector = MazeMap::App::Internal::CommandVector;
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

    void OnModeFault(const char* reason) noexcept override
    {
        (void)reason;
        _phase = Phase::Idle;
        _pauseAction = PauseAction::None;
        _startupCalibration.Cancel();
        _drive.ClearCommandEvidence();
        _vehicle.SetFanDuty(0.0f);
    }

    bool AppendStartupTraceLine(const char* line)
    {
        return _bootFramework != nullptr && _bootFramework->AppendStartupTrace(line);
    }

    void FinalizeSuccessfulRun() noexcept
    {
        _startupCalibration.Cancel();
        _drive.ClearCommandEvidence();
        _phase = Phase::Idle;
        _pauseAction = PauseAction::None;
        _vehicle.SetFanDuty(0.0f);
        (void)_runtime.AppendTextLogLine("Front wall characterization complete and persisted.");
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _vehicle;
    DriveBase& _drive;
    Drive& _driveService;
    StartupCalibration& _startupCalibration;
    MazeMap::App::Internal::BootFramework* _bootFramework{};
    FrontWallCharacterizationLogRow _logRow{};
    Phase _phase{ Phase::Idle };
    PauseAction _pauseAction{ PauseAction::None };
    FrontWallCharacterizationStorage _captureStorage{};
    Eigen::Vector2f _captureTargetHeading = Eigen::Vector2f(0.0f, 1.0f);
    float _captureStartDistanceM = 0.0f;
    float _captureNextStoredDistanceM = 0.0f;
    std::uint8_t _captureCollapsedConsecutiveSamples = 0U;
    unsigned long _captureStartMs = 0UL;
    unsigned long _captureTimeoutMs = 0UL;
    const char* _captureCompletionReason = "unknown";
    bool _captureElapsedBudgetLogged = false;
    bool _captureStarted = false;

    static void PauseThunk(
        void* context,
        LoopController& loopController)
    {
        auto* const self = static_cast<FrontWallCharacterizationController*>(context);
        if (self == nullptr)
        {
            GetSharedRobotRuntime().FailActiveMode(
                "Front wall characterization pause callback context was null");
        }

        self->OnPauseGranted(loopController);
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
            AppendStartupTraceLine(line);
        }

        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.StartHold(durationMs, true);
        return true;
    }

    bool StartCaptureCurvePhase()
    {
        AppendStartupTraceLine("front_wall_characterization:phase=reverse_capture");
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

    void OnPauseGranted(LoopController& loopController)
    {
        (void)loopController;

        if (_pauseAction != PauseAction::PersistAndExport)
        {
            _runtime.FailActiveMode(
                "Front wall characterization pause granted without a pending action");
        }

        _pauseAction = PauseAction::None;
        _drive.ClearCommandEvidence();

        MazeMap::FrontWallCharacterizationStorage storage = _captureStorage;
        if (storage.sampleCount < 4U)
        {
            _runtime.FailActiveMode("Front wall characterization captured too few samples");
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
        AppendStartupTraceLine(summary);
        (void)_runtime.AppendTextLogLine(summary);

        if (!PersistCurve(storage) || !ExportCurveToSd(storage))
        {
            _runtime.FailActiveMode("Front wall characterization persist/export failed");
        }

        _captureStorage = storage;
        _phase = Phase::LaunchPostCaptureSettle;
    }

    CommandVector PollDrivePhase(
        const Phase nextPhase)
    {
        bool done = false;
        const CommandVector control = _driveService.GetNextControls(done);
        if (!done)
        {
            return control;
        }

        _phase = nextPhase;
        return CommandVector::Brake();
    }

    CommandVector RunTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController) override
    {
        (void)loopEndTimeUs;
        switch (_phase)
        {
        case Phase::LaunchStartupSettle:
            if (!StartHoldPhase("startup_settle", FrontWallCharacterizationConfig::kStartupSettleMs))
            {
                _runtime.FailActiveMode("Front wall characterization startup settle could not start");
            }
            else
            {
                _phase = Phase::RunStartupSettle;
            }
            return CommandVector::Brake();

        case Phase::RunStartupSettle:
            return PollDrivePhase(Phase::Capture);

        case Phase::Capture:
            return CaptureCurveTick(loopEndTimeUs, state, loopController);

        case Phase::LaunchPostCaptureSettle:
            if (!StartHoldPhase("post_capture_settle", FrontWallCharacterizationConfig::kPostCaptureSettleMs))
            {
                _runtime.FailActiveMode("Front wall characterization post-capture settle could not start");
            }
            else
            {
                _phase = Phase::RunPostCaptureSettle;
            }
            return CommandVector::Brake();

        case Phase::RunPostCaptureSettle:
        {
            bool done = false;
            const CommandVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = Phase::Complete;
            FinalizeSuccessfulRun();
            loopController.HaltExecutionEndProgram();
            return CommandVector::Brake();
        }

        case Phase::Complete:
            return CommandVector::Brake();

        case Phase::Idle:
        default:
            _runtime.FailActiveMode("Front wall characterization phase was not initialized");
            return CommandVector::Brake();
        }
    }

    CommandVector CaptureCurveTick(
        std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        auto requestPersistPause = [this, &loopController](const float traveledDistanceM, const char* reason)
        {
            _captureCompletionReason = reason;
            _captureStorage.terminalDistanceM = traveledDistanceM;
            _pauseAction = PauseAction::PersistAndExport;
            loopController.RequestPause(&FrontWallCharacterizationController::PauseThunk, this);
            return CommandVector::Brake();
        };

        const SensorSnapshot& snapshot = state.GetSensorSnapshot();
        if (!_captureStarted)
        {
            if (!StartCaptureCurvePhase())
            {
                _runtime.FailActiveMode("Front wall characterization capture phase start failed");
                return CommandVector::Brake();
            }

            _captureStarted = true;
            _captureTargetHeading = state.GetHeadingUnit();
            _captureStartDistanceM = AverageEncoderDistanceM(snapshot);
            _captureStartMs = millis();
            StoreCurveSample(_captureStorage, 0.0f, snapshot);

            _driveService.SetLimits(BuildReverseCaptureLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartStraight(
                FrontWallCharacterizationConfig::kMaxReverseTravelM,
                -FrontWallCharacterizationConfig::kReverseSpeedMps,
                0.0f,
                &_captureTargetHeading);
        }

        const float traveledDistanceM = std::fabs(AverageEncoderDistanceM(snapshot) - _captureStartDistanceM);
        if ((_captureStorage.sampleCount < MazeMap::kFrontWallCharacterizationMaxStoredSamples) &&
            ((traveledDistanceM + Config::kDistanceToleranceM) >= _captureNextStoredDistanceM))
        {
            StoreCurveSample(_captureStorage, traveledDistanceM, snapshot);
            _captureNextStoredDistanceM += FrontWallCharacterizationConfig::kStoredDistanceStepM;
        }

        const bool collapsedToZero =
            (snapshot.FrontLeftTelemetry().differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold) &&
            (snapshot.FrontRightTelemetry().differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold);
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
            AppendStartupTraceLine(timeoutLine);
            _captureElapsedBudgetLogged = true;
        }

        bool driveDone = false;
        const CommandVector control = _driveService.GetNextControls(driveDone);
        if (driveDone)
        {
            return requestPersistPause(traveledDistanceM, "drive_complete");
        }

        return control;
    }

    static void StoreCurveSample(
        FrontWallCharacterizationStorage& storage,
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
        storage.frontLeftAmbientLight[index] = snapshot.FrontLeftTelemetry().ambientLight;
        storage.frontLeftLitLight[index] = snapshot.FrontLeftTelemetry().litLight;
        storage.frontLeftDifferentialLight[index] = snapshot.FrontLeftTelemetry().differentialLight;
        storage.frontRightAmbientLight[index] = snapshot.FrontRightTelemetry().ambientLight;
        storage.frontRightLitLight[index] = snapshot.FrontRightTelemetry().litLight;
        storage.frontRightDifferentialLight[index] = snapshot.FrontRightTelemetry().differentialLight;
        ++storage.sampleCount;
    }

    bool PersistCurve(const FrontWallCharacterizationStorage& storage)
    {
        if (!WritePersistedFrontWallCharacterization(storage))
        {
            _runtime.FailActiveMode("Failed to persist front wall characterization");
			//unreachable
			//return false;
        }

        FrontWallCharacterizationStorage verify{};
        if (!TryReadPersistedFrontWallCharacterization(verify))
        {
            _runtime.FailActiveMode("Failed to verify persisted front wall characterization");
			//unreachable
			//return false;
        }

        char line[160] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:persisted,samples=%u,terminal_distance_m=%.4f",
            static_cast<unsigned>(verify.sampleCount),
            verify.terminalDistanceM);
        AppendStartupTraceLine(line);
        (void)_runtime.AppendTextLogLine(line);
        return true;
    }

    bool ExportCurveToSd(const FrontWallCharacterizationStorage& storage)
    {
        if (!IsValidFrontWallCharacterizationStorage(storage))
        {
            _runtime.FailActiveMode("Invalid front wall characterization cannot be exported");
            return false;
        }

        char fileName[32] = {};
        if (!_runtime.OpenUtilityDataLog(
                fileName,
                sizeof(fileName),
                nullptr,
                "fwc%03u.mmlog",
                "front_wall_characterization.mmlog"))
        {
            _runtime.FailActiveMode("Front wall characterization log name unavailable");
            return false;
        }

        if (!_runtime.WriteUtilityDataLogMetadata("mode", "front_wall_characterization"))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("samples", static_cast<unsigned long>(storage.sampleCount)))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("distance_step_m", storage.distanceStepM, 6))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("reverse_speed_mps", storage.commandedReverseSpeedMps, 6))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("zero_threshold_differential_light", storage.zeroThresholdDifferentialLight, 6))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("terminal_distance_m", storage.terminalDistanceM, 6))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g"))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little"))
        {
            _runtime.FailActiveMode("Front wall characterization log metadata failed");
            return false;
        }

        _logRow = {};
        if (!_runtime.BeginUtilityDataLogSchema(_logRow))
        {
            _runtime.FailActiveMode("Front wall characterization log open failed");
            return false;
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
                _runtime.FailActiveMode("Front wall characterization log write failed");
                return false;
            }
        }

        if (!_runtime.CloseUtilityDataLog())
        {
            _runtime.FailActiveMode("Front wall characterization log write failed");
            return false;
        }

        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "front_wall_characterization:log_exported,file=%s,samples=%u",
            fileName,
            static_cast<unsigned>(storage.sampleCount));
        AppendStartupTraceLine(line);
        (void)_runtime.AppendTextLogLine(line);
        return true;
    }

};

namespace App::Internal
{
    IApplicationMode& GetFrontWallCharacterizationMode();

    const BootModeDescriptor& GetFrontWallCharacterizationBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::FrontWallCharacterization,
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
            true,
        };
        return descriptor;
    }

    IApplicationMode& GetFrontWallCharacterizationMode()
    {
        static FrontWallCharacterizationController mode(GetSharedRobotRuntime());
        return mode;
    }
}


}
