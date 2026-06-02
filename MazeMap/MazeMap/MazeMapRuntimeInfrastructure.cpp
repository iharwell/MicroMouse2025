#include "pch.h"
#include "MazeMapRuntimeInfrastructure.h"

#include "Imu.h"
#include "SharedRobotRuntime.h"
#include "RuntimeSensorSuite.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace MazeMap::App::Internal::Runtime
{
    bool BeginDiagnosticUtilityTelemetryLog(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        RuntimeSensorSuite& sensors,
        DiagnosticLogRow& row,
        const char* fileName,
        const char* modeName,
        unsigned long& phaseId,
        unsigned long& sampleCount)
    {
        const char* const resolvedFileName =
            (fileName != nullptr && fileName[0] != '\0') ? fileName : "telemetry.mmlog";
        const char* const resolvedModeName =
            (modeName != nullptr && modeName[0] != '\0') ? modeName : "telemetry";

        phaseId = 0UL;
        sampleCount = 0UL;
        row = {};
        (void)runtime.CloseUtilityDataLog();

        auto fail = [&runtime, &row]() -> bool
        {
            row = {};
            (void)runtime.CloseUtilityDataLog();
            return false;
        };

        if (!runtime.OpenUtilityDataLogFile(resolvedFileName))
        {
            return false;
        }
        if (!runtime.WriteUtilityDataLogMetadata("mode", resolvedModeName))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", Config::kControlPeriodUs))
        {
            return fail();
        }
        {
            const unsigned long imuSampleRateHz =
                MazeMap::Imu::GetUiImuSampleRateHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz))
            {
                return fail();
            }
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::Imu::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                Config::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f &&
                !runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3))
            {
                return fail();
            }
        }
        {
            const float imuGyroLpf1ReferenceHz =
                MazeMap::Imu::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f &&
                !runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3))
            {
                return fail();
            }
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogAccelBiasMetadata(sensors))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g"))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadata("endianness", "little"))
        {
            return fail();
        }

        if (!runtime.BeginUtilityDataLogSchema(row))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("file", runtime.TextLogFileName()))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("data_file", resolvedFileName))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("mode", resolvedModeName))
        {
            return fail();
        }
        if (!WriteDiagnosticTuningEvents(
                runtime.Plant(),
                [&runtime](const char* type, const char* message) -> bool
                {
                    return runtime.WriteTextLogEntry(micros(), type, message);
                }))
        {
            return fail();
        }
        if (!WriteDiagnosticSummaryInstructions(
                [&runtime](const char* type, const char* message) -> bool
                {
                    return runtime.WriteTextLogEntry(micros(), type, message);
                }))
        {
            return fail();
        }

        return true;
    }

}
