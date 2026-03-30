#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ImuSamplingProfile.h"
#pragma once

#include <stdint.h>

namespace MazeMap
{
    enum class UiImuSamplingProfile : uint8_t
    {
        Unsupported = 0U,
        Exact1000Hz,
        Exact2000Hz,
    };

    constexpr UiImuSamplingProfile SelectUiImuSamplingProfile(unsigned long controlPeriodUs)
    {
        return (controlPeriodUs == 500UL) ? UiImuSamplingProfile::Exact2000Hz :
               (controlPeriodUs == 1000UL) ? UiImuSamplingProfile::Exact1000Hz :
               UiImuSamplingProfile::Unsupported;
    }

    constexpr unsigned long GetUiImuSampleRateHz(UiImuSamplingProfile profile)
    {
        switch (profile)
        {
        case UiImuSamplingProfile::Exact1000Hz:
            return 1000UL;
        case UiImuSamplingProfile::Exact2000Hz:
            return 2000UL;
        default:
            return 0UL;
        }
    }

    constexpr unsigned long GetUiImuSampleRateHzForControlPeriodUs(unsigned long controlPeriodUs)
    {
        return GetUiImuSampleRateHz(SelectUiImuSamplingProfile(controlPeriodUs));
    }

    // AN5763 Table 17: FRAC_1_400 selects LPF2 bandwidth = ODR / 400.
    constexpr float GetUiAccelLpf2CutoffHzForControlPeriodUs(unsigned long controlPeriodUs)
    {
        const unsigned long sampleRateHz = GetUiImuSampleRateHzForControlPeriodUs(controlPeriodUs);
        return (sampleRateHz > 0UL) ? (static_cast<float>(sampleRateHz) / 400.0f) : 0.0f;
    }

    // AN5763 Table 21: CUT_213 resolves to 195 Hz at 960 Hz ODR and 210 Hz at 1920 Hz ODR.
    // ST does not publish separate LPF1 cutoffs for the HAODR exact 1000/2000 Hz entries, so this is a datasheet
    // reference value for the nearest native UI ODR used by the same register code.
    constexpr float GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(unsigned long controlPeriodUs)
    {
        switch (SelectUiImuSamplingProfile(controlPeriodUs))
        {
        case UiImuSamplingProfile::Exact1000Hz:
            return 195.0f;
        case UiImuSamplingProfile::Exact2000Hz:
            return 210.0f;
        default:
            return 0.0f;
        }
    }
}
