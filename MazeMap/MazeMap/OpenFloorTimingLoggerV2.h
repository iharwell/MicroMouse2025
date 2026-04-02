#ifndef OPENFLOORTIMINGLOGGERV2_H
#define OPENFLOORTIMINGLOGGERV2_H

// Declares the second-generation open-floor timing logger used to capture per-cycle scheduler and sensor timing.

#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementSpec.h"
#include "OptionalRuntimeEventLog.h"
#include "RuntimeBinaryLogFile.h"
#include "RuntimeTextBlockBuilder.h"

#include <cstdint>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Records the timing-focused open-floor measurement stream and its control-log summaries.
				class OpenFloorTimingLoggerV2
				{
				private:
					static constexpr uint32_t kTimingFieldCount = 36U;
					static constexpr const char* kTimingSchema =
						"u32_mono_time_us,u32_control_tick_sequence,u32_dt_us,u32_section_id,u32_logger_flags,"
						"u32_control_start_us,u32_control_end_us,u32_pwm_latch_us,u32_encoder_latch_us,u32_encoder_read_done_us,"
						"u32_ukf_predict_start_us,u32_ukf_predict_end_us,u32_ukf_predict_duration_us,u32_ukf_update_start_us,u32_ukf_update_end_us,u32_ukf_update_duration_us,"
						"u32_imu_drdy_us,u32_imu_read_start_us,u32_imu_read_done_us,"
						"u32_front_led_on_us,u32_front_adc_on_us,u32_front_led_off_us,u32_front_adc_off_us,u32_front_ready_us,"
						"u32_left_led_on_us,u32_left_adc_on_us,u32_left_led_off_us,u32_left_adc_off_us,u32_left_ready_us,"
						"u32_right_led_on_us,u32_right_adc_on_us,u32_right_led_off_us,u32_right_adc_off_us,u32_right_ready_us,"
						"u32_cycle_counter_start,u32_cycle_counter_end";

					RuntimeBinaryLogFile _sampleLog;
					OptionalRuntimeEventLog _eventLog;
					RuntimeTextBlockBuilder<1024U> _metadata;
					RuntimeTextBlockBuilder<512U> _notes;

				public:
					bool Begin(const char* runId);
					bool LogSample(const OpenFloorMeasurementCycle& cycle);
					bool LogFault(
						const OpenFloorMeasurementCycle& cycle,
						MazeMap::OpenFloorFaultCode faultCode,
						bool controlHalted,
						uint32_t extra0 = 0UL,
						uint32_t extra1 = 0UL);
					bool LogSummary(const char* streamId, unsigned long sampleCount, float meanDelayUs, float jitterUs);
					bool LogFailure(const char* reason);
					void Service();
					void Flush();
					void Close();
				};
			}
		}
	}
}

#endif
