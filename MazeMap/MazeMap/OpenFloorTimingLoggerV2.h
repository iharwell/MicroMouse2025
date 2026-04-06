#ifndef OPENFLOORTIMINGLOGGERV2_H
#define OPENFLOORTIMINGLOGGERV2_H

// Declares the standardized open-floor timing row used for per-cycle scheduler and sensor timing capture.

#include "MazeMapRuntimeMmLog.h"
#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementSpec.h"

#include <cstdint>

namespace MazeMap::App::Internal::Runtime
{
#define OPEN_FLOOR_TIMING_FIELDS(X)                \
    X(std::uint32_t, mono_time_us)                \
    X(std::uint32_t, control_tick_sequence)       \
    X(std::uint32_t, dt_us)                       \
    X(std::uint32_t, section_id)                  \
    X(std::uint16_t, logger_flags)                \
    X(std::uint32_t, control_start_us)            \
    X(std::uint32_t, control_end_us)              \
    X(std::uint32_t, pwm_latch_us)                \
    X(std::uint32_t, encoder_latch_us)            \
    X(std::uint32_t, encoder_read_done_us)        \
    X(std::uint32_t, ukf_predict_start_us)        \
    X(std::uint32_t, ukf_predict_end_us)          \
    X(std::uint32_t, ukf_predict_duration_us)     \
    X(std::uint32_t, ukf_update_start_us)         \
    X(std::uint32_t, ukf_update_end_us)           \
    X(std::uint32_t, ukf_update_duration_us)      \
    X(std::uint32_t, imu_drdy_us)                 \
    X(std::uint32_t, imu_read_start_us)           \
    X(std::uint32_t, imu_read_done_us)            \
    X(std::uint32_t, front_led_on_us)             \
    X(std::uint32_t, front_adc_on_us)             \
    X(std::uint32_t, front_led_off_us)            \
    X(std::uint32_t, front_adc_off_us)            \
    X(std::uint32_t, front_ready_us)              \
    X(std::uint32_t, left_led_on_us)              \
    X(std::uint32_t, left_adc_on_us)              \
    X(std::uint32_t, left_led_off_us)             \
    X(std::uint32_t, left_adc_off_us)             \
    X(std::uint32_t, left_ready_us)               \
    X(std::uint32_t, right_led_on_us)             \
    X(std::uint32_t, right_adc_on_us)             \
    X(std::uint32_t, right_led_off_us)            \
    X(std::uint32_t, right_adc_off_us)            \
    X(std::uint32_t, right_ready_us)              \
    X(std::uint32_t, cycle_counter_start)         \
    X(std::uint32_t, cycle_counter_end)

    MMLOG_DEFINE_ROW(OpenFloorTimingRow, OPEN_FLOOR_TIMING_FIELDS);
}

#endif
