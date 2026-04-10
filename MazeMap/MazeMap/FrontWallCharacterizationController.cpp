#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
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
        , _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
        , _lastControlMicros(0UL)
    {
    }

    bool Begin() override
    {
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
        (void)_runtime.AppendTextLogLine("Place the robot with its nose touching a wall, keep the area dark, then power on.");
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
        _lastControlMicros = micros();

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
            (void)_runtime.AppendTextLogLine("Existing persisted front curve found; it will be replaced on success.");
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
        MazeMap::FrontWallCharacterizationStorage storage{};
        const bool ok =
            HoldStationary("startup_settle", FrontWallCharacterizationConfig::kStartupSettleMs) &&
            CaptureCurve(storage) &&
            PersistCurve(storage) &&
            ExportCurveToSd(storage) &&
            HoldStationary("post_capture_settle", FrontWallCharacterizationConfig::kPostCaptureSettleMs);

        _drive.Brake();
        SetMissionLevelFanEnabled(false);
        if (ok)
        {
            (void)_runtime.AppendTextLogLine("Front wall characterization complete and persisted.");
        }
        _runtime.CloseTextLog();
    }

private:
    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<FrontWallCharacterizationController*>(context)->OnRuntimeFault(reason);
    }

    SharedRobotRuntime& _runtime;
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    unsigned long _lastControlMicros;

    bool HoldStationary(const char* phaseName, uint16_t durationMs)
    {
        if (phaseName != nullptr && phaseName[0] != '\0')
        {
            char line[96] = {};
            snprintf(line, sizeof(line), "front_wall_characterization:phase=%s", phaseName);
            AppendStartupTrace(line);
        }

        const unsigned long startMs = millis();
        while (static_cast<unsigned long>(millis() - startMs) < durationMs)
        {
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);
            (void)timestampUs;
            const DiagnosticSensorSnapshot snapshot = _sensors.Capture(true, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, snapshot);
            _drive.Brake();
        }

        return true;
    }

    bool CaptureCurve(MazeMap::FrontWallCharacterizationStorage& storage)
    {
        storage = {};
        storage.distanceStepM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        storage.commandedReverseSpeedMps = FrontWallCharacterizationConfig::kReverseSpeedMps;
        storage.zeroThresholdDifferentialLight = FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold;

        const Eigen::Vector2f targetHeading = _drive.GetPose().headingUnit;
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const DiagnosticSensorSnapshot initialSnapshot = _sensors.Capture(true, _drive.GetPose());
        StoreCurveSample(storage, 0.0f, initialSnapshot);

        float commandedSpeedMps = 0.0f;
        float nextStoredDistanceM = FrontWallCharacterizationConfig::kStoredDistanceStepM;
        uint8_t collapsedConsecutiveSamples = 0U;
        const unsigned long startMs = millis();
        const unsigned long timeoutMs =
            static_cast<unsigned long>(2000.0f +
                ((1000.0f * FrontWallCharacterizationConfig::kMaxReverseTravelM) /
                    (std::max)(FrontWallCharacterizationConfig::kReverseSpeedMps, 0.01f)));
        bool elapsedBudgetLogged = false;
        const char* completionReason = "unknown";

        while (true)
        {
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);
            (void)timestampUs;

            const DiagnosticSensorSnapshot snapshot = _sensors.Capture(false, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, snapshot);

            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if ((storage.sampleCount < MazeMap::kFrontWallCharacterizationMaxStoredSamples) &&
                ((traveledDistanceM + Config::kDistanceToleranceM) >= nextStoredDistanceM))
            {
                StoreCurveSample(storage, traveledDistanceM, snapshot);
                nextStoredDistanceM += FrontWallCharacterizationConfig::kStoredDistanceStepM;
            }

            const bool collapsedToZero =
                (snapshot.frontLeft.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold) &&
                (snapshot.frontRight.differentialLight <= FrontWallCharacterizationConfig::kCollapsedDifferentialLightThreshold);
            if (traveledDistanceM >= FrontWallCharacterizationConfig::kMinimumTravelBeforeCollapseCheckM && collapsedToZero)
            {
                ++collapsedConsecutiveSamples;
            }
            else
            {
                collapsedConsecutiveSamples = 0U;
            }

            if (storage.sampleCount >= MazeMap::kFrontWallCharacterizationMaxStoredSamples)
            {
                completionReason = "storage_full";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (collapsedConsecutiveSamples >= FrontWallCharacterizationConfig::kCollapsedConsecutiveSamples)
            {
                completionReason = "collapsed_to_zero";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (traveledDistanceM >= FrontWallCharacterizationConfig::kMaxReverseTravelM)
            {
                completionReason = "max_reverse_travel";
                storage.terminalDistanceM = traveledDistanceM;
                break;
            }

            if (!elapsedBudgetLogged &&
                static_cast<unsigned long>(millis() - startMs) >= timeoutMs)
            {
                char timeoutLine[192] = {};
                snprintf(
                    timeoutLine,
                    sizeof(timeoutLine),
                    "front_wall_characterization:elapsed_budget_reached,travel_m=%.4f,samples=%u,timeout_ms=%lu",
                    traveledDistanceM,
                    static_cast<unsigned>(storage.sampleCount),
                    timeoutMs);
                AppendStartupTrace(timeoutLine);
                elapsedBudgetLogged = true;
            }

            commandedSpeedMps = (std::min)(
                FrontWallCharacterizationConfig::kReverseSpeedMps,
                commandedSpeedMps + (FrontWallCharacterizationConfig::kReverseAccelMps2 * dtSeconds));

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps =
                (Config::kStraightHeadingKp * headingErrorRad) -
                (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -FrontWallCharacterizationConfig::kMaxAngularCommandRadps,
                FrontWallCharacterizationConfig::kMaxAngularCommandRadps);
            _drive.CommandVelocity(-commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        _drive.Brake();
        if (storage.sampleCount < 4U)
        {
            return Fail("Front wall characterization captured too few samples");
        }

        MazeMap::FinalizeFrontWallCharacterizationStorage(storage);

        char summary[224] = {};
        snprintf(
            summary,
            sizeof(summary),
            "front_wall_characterization:captured,reason=%s,samples=%u,terminal_distance_m=%.4f,fl_start=%.6f,fl_end=%.6f,fr_start=%.6f,fr_end=%.6f",
            completionReason,
            static_cast<unsigned>(storage.sampleCount),
            storage.terminalDistanceM,
            storage.frontLeftDifferentialLight[0],
            storage.frontLeftDifferentialLight[storage.sampleCount - 1U],
            storage.frontRightDifferentialLight[0],
            storage.frontRightDifferentialLight[storage.sampleCount - 1U]);
        AppendStartupTrace(summary);
        (void)_runtime.AppendTextLogLine(summary);
        return true;
    }

    static void StoreCurveSample(
        MazeMap::FrontWallCharacterizationStorage& storage,
        float traveledDistanceM,
        const DiagnosticSensorSnapshot& snapshot)
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

        FrontWallCharacterizationLogRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return Fail("Front wall characterization log open failed");
        }

        for (uint16_t index = 0U; index < storage.sampleCount; ++index)
        {
            row.index = index;
            row.distance_m = storage.distanceM[index];
            row.front_left_ambient = storage.frontLeftAmbientLight[index];
            row.front_left_lit = storage.frontLeftLitLight[index];
            row.front_left_delta = storage.frontLeftDifferentialLight[index];
            row.front_right_ambient = storage.frontRightAmbientLight[index];
            row.front_right_lit = storage.frontRightLitLight[index];
            row.front_right_delta = storage.frontRightDifferentialLight[index];
            if (!_runtime.LogUtilityDataRow(row))
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

    void WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)
    {
        while ((micros() - _lastControlMicros) < FrontWallCharacterizationConfig::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        timestampUs = micros();
        dtUs = static_cast<uint32_t>(timestampUs - _lastControlMicros);
        _lastControlMicros = timestampUs;
    }

    bool Fail(const char* reason)
    {
        return _runtime.FailActiveMode(reason);
    }

    void OnRuntimeFault(const char* reason) noexcept
    {
        if (reason != nullptr && reason[0] != '\0')
        {
            AppendStartupTrace(reason);
        }
    }
};

namespace MazeMap::App::Internal
{
    const BootModeDescriptor& GetFrontWallCharacterizationBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::FrontWallCharacterization,
            BootModeCategory::Utility,
            "front_wall_characterization",
            "Capture and persist the front-wall sensor response curve.",
            "logging.txt; front-wall characterization mmlog; persisted front-wall curve",
            "GetFrontWallCharacterizationMode",
            "FrontWallCharacterizationController.cpp",
            "startup settle; reverse capture; persist; SD export; post-capture settle",
            "FrontWallCharacterizationConfig; shared mission drive and sensor tuning",
            "reverse speed, max travel, sample spacing, and collapse threshold are local to this characterization",
            "fwc%03u.mmlog or front_wall_characterization.mmlog; EEPROM front-wall storage",
        };
        return descriptor;
    }

    IApplicationMode& GetFrontWallCharacterizationMode()
    {
        static FrontWallCharacterizationController mode(GetSharedRobotRuntime());
        return mode;
    }
}

