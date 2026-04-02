#ifndef OPENFLOORMAINLOGGERV2_H
#define OPENFLOORMAINLOGGERV2_H

// Declares the second-generation open-floor main logger used to capture measurement rows for each control loop.

#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "OptionalRuntimeEventLog.h"
#include "RuntimeBinaryLogFile.h"
#include "RuntimeTextBlockBuilder.h"

#include <cstdint>

class DiagnosticSensorSuite;
struct PoseEstimate;
class DriveBase;

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Records the primary open-floor measurement stream and emits section/fault markers to the control log.
				class OpenFloorMainLoggerV2
				{
				private:
					static constexpr uint32_t kMainFieldCount = 80U;
					static constexpr const char* kMainSchema =
						"u32_master_time_us,u32_control_tick_sequence,u32_dt_us,u32_section_id,u32_primitive_id,u32_primitive_family,u32_direction_id,u32_phase_id,u32_speed_bin,u32_start_marker_id,u32_mirrored,u32_repeat_index,f32_progress_norm,u32_mode_flags,u32_clipping_flags,u32_saturation_flags,u32_logger_flags,u32_watchdog_flags,u32_measurement_flags,f32_pose_x_m,f32_pose_y_m,f32_pose_yaw_rad,f32_measured_linear_speed_mps,f32_measured_angular_speed_radps,f32_cmd_linear_mps,f32_cmd_angular_radps,f32_left_drive_command,f32_right_drive_command,f32_left_feedforward_command,f32_right_feedforward_command,f32_left_feedback_command,f32_right_feedback_command,f32_left_target_velocity_mps,f32_right_target_velocity_mps,f32_left_launch_assist_floor,f32_right_launch_assist_floor,u32_encoder_timestamp_us,i32_left_encoder_count,i32_right_encoder_count,f32_left_encoder_omega_radps,f32_right_encoder_omega_radps,f32_left_encoder_distance_m,f32_right_encoder_distance_m,f32_left_encoder_velocity_mps,f32_right_encoder_velocity_mps,u32_imu_timestamp_us,u32_imu_status,u32_imu_interrupt_high,u32_accel_bias_valid,i32_imu_gyro_x,i32_imu_gyro_y,i32_imu_gyro_z,i32_imu_accel_x,i32_imu_accel_y,i32_imu_accel_z,i32_imu_temp,f32_gyro_raw_radps,f32_gyro_bias_radps,f32_gyro_radps,f32_accel_body_x_mps2,f32_accel_body_y_mps2,f32_planar_accel_mps2,u32_front_timestamp_us,u32_left_timestamp_us,u32_right_timestamp_us,u32_front_left_obs_class,u32_front_right_obs_class,u32_left_obs_class,u32_right_obs_class,f32_front_left_obs_rho_m,f32_front_right_obs_rho_m,f32_left_obs_rho_m,f32_right_obs_rho_m,f32_front_left_obs_confidence,f32_front_right_obs_confidence,f32_left_obs_confidence,f32_right_obs_confidence,f32_battery_voltage_v,f32_board_temperature_c,f32_fan_duty_cycle";

					RuntimeBinaryLogFile _sampleLog;
					OptionalRuntimeEventLog _eventLog;
					RuntimeTextBlockBuilder<2048U> _metadata;
					RuntimeTextBlockBuilder<512U> _notes;

					bool WriteSectionMarker(const char* type, const OpenFloorMeasurementLabels& labels, const char* reason);

				public:
					bool Begin(const DiagnosticSensorSuite& sensors, const char* runId);
					bool LogSample(
						const OpenFloorMeasurementLabels& labels,
						const PoseEstimate& pose,
						const DriveBase& drive,
						const OpenFloorMeasurementCycle& cycle);
					bool LogFault(
						const OpenFloorMeasurementLabels& labels,
						const OpenFloorMeasurementCycle& cycle,
						MazeMap::OpenFloorFaultCode faultCode,
						bool controlHalted,
						uint32_t extra0 = 0UL,
						uint32_t extra1 = 0UL);
					bool BeginSection(const OpenFloorMeasurementLabels& labels);
					bool EndSection(const OpenFloorMeasurementLabels& labels);
					bool AbortSection(const OpenFloorMeasurementLabels& labels, const char* reason);
					bool WriteEvent(const char* type, const char* message);
					void Service();
					void Flush();
					void Close();
				};
			}
		}
	}
}

#endif
