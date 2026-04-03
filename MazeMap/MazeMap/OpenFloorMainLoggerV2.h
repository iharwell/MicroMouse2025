#ifndef OPENFLOORMAINLOGGERV2_H
#define OPENFLOORMAINLOGGERV2_H

// Declares the second-generation open-floor main logger used to capture measurement rows for each control loop.

#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "OptionalRuntimeEventLog.h"
#include "RuntimeBinaryLogFile.h"
#include "RuntimeTextBlockBuilder.h"
#include "mmlog.h"
#include <cstdint>

class DiagnosticSensorSuite;
struct PoseEstimate;
class DriveBase;

namespace MazeMap
{
	namespace mmlog
	{
#define OPEN_FLOOR_MAIN_FIELDS(X)                   \
    X(uint32_t, master_time_us)                   \
    X(uint32_t, control_tick_sequence)            \
    X(uint32_t, dt_us)                            \
    X(uint8_t,  section_id)                       \
    X(uint8_t,  primitive_id)                     \
    X(uint8_t,  primitive_family)                 \
    X(uint8_t,  direction_id)                     \
    X(uint8_t,  phase_id)                         \
    X(uint8_t,  speed_bin)                        \
    X(uint16_t, start_marker_id)                  \
    X(uint8_t,  mirrored)                         \
    X(uint16_t, repeat_index)                     \
    X(float,         progress_norm)                    \
    X(uint16_t, mode_flags)                       \
    X(uint32_t, clipping_flags)                   \
    X(uint16_t, saturation_flags)                 \
    X(uint16_t, logger_flags)                     \
    X(uint16_t, watchdog_flags)                   \
    X(uint16_t, measurement_flags)                \
    X(float,         pose_x_m)                         \
    X(float,         pose_y_m)                         \
    X(float,         pose_yaw_rad)                     \
    X(float,         measured_linear_speed_mps)        \
    X(float,         measured_angular_speed_radps)     \
    X(float,         cmd_linear_mps)                   \
    X(float,         cmd_angular_radps)                \
    X(float,         left_drive_command)               \
    X(float,         right_drive_command)              \
    X(float,         left_feedforward_command)         \
    X(float,         right_feedforward_command)        \
    X(float,         left_feedback_command)            \
    X(float,         right_feedback_command)           \
    X(float,         left_target_velocity_mps)         \
    X(float,         right_target_velocity_mps)        \
    X(float,         left_launch_assist_floor)         \
    X(float,         right_launch_assist_floor)        \
    X(uint32_t, encoder_timestamp_us)             \
    X(int32_t,  left_encoder_count)               \
    X(int32_t,  right_encoder_count)              \
    X(float,         left_encoder_omega_radps)         \
    X(float,         right_encoder_omega_radps)        \
    X(float,         left_encoder_distance_m)          \
    X(float,         right_encoder_distance_m)         \
    X(float,         left_encoder_velocity_mps)        \
    X(float,         right_encoder_velocity_mps)       \
    X(uint32_t, imu_timestamp_us)                 \
    X(uint8_t,  imu_status)                       \
    X(uint8_t,  imu_interrupt_high)               \
    X(uint8_t,  accel_bias_valid)                 \
    X(int16_t,  imu_gyro_x)                       \
    X(int16_t,  imu_gyro_y)                       \
    X(int16_t,  imu_gyro_z)                       \
    X(int16_t,  imu_accel_x)                      \
    X(int16_t,  imu_accel_y)                      \
    X(int16_t,  imu_accel_z)                      \
    X(int16_t,  imu_temp)                         \
    X(float,         gyro_raw_radps)                   \
    X(float,         gyro_bias_radps)                  \
    X(float,         gyro_radps)                       \
    X(float,         accel_body_x_mps2)                \
    X(float,         accel_body_y_mps2)                \
    X(float,         planar_accel_mps2)                \
    X(uint32_t, front_timestamp_us)               \
    X(uint32_t, left_timestamp_us)                \
    X(uint32_t, right_timestamp_us)               \
    X(uint8_t,  front_left_obs_class)             \
    X(uint8_t,  front_right_obs_class)            \
    X(uint8_t,  left_obs_class)                   \
    X(uint8_t,  right_obs_class)                  \
    X(float,         front_left_obs_rho_m)             \
    X(float,         front_right_obs_rho_m)            \
    X(float,         left_obs_rho_m)                   \
    X(float,         right_obs_rho_m)                  \
    X(float,         front_left_obs_confidence)        \
    X(float,         front_right_obs_confidence)       \
    X(float,         left_obs_confidence)              \
    X(float,         right_obs_confidence)             \
    X(float,         fan_duty_cycle)

				MMLOG_DEFINE_ROW(OpenFloorRow, OPEN_FLOOR_MAIN_FIELDS);
				// Records the primary open-floor measurement stream and emits section/fault markers to the control log.
				/*class OpenFloorMainLoggerV2
				{
				private:
					static constexpr uint32_t kMainSize = 242U;
					static constexpr const char* kMainSchema =
						"u32_master_time_us,"
						"u32_control_tick_sequence,"
						"u32_dt_us,"
						"u8_section_id,"
						"u8_primitive_id,"
						"u8_primitive_family,"
						"u8_direction_id,"
						"u8_phase_id,"
						"u8_speed_bin,"
						"u16_start_marker_id,"
						"u8_mirrored,"
						"u16_repeat_index,"
						"f32_progress_norm,"
						"u16_mode_flags,"
						"u32_clipping_flags,"
						"u16_saturation_flags,"
						"u16_logger_flags,"
						"u16_watchdog_flags,"
						"u16_measurement_flags,"
						"f32_pose_x_m,"
						"f32_pose_y_m,"
						"f32_pose_yaw_rad,"
						"f32_measured_linear_speed_mps,"
						"f32_measured_angular_speed_radps,"
						"f32_cmd_linear_mps,"
						"f32_cmd_angular_radps,"
						"f32_left_drive_command,"
						"f32_right_drive_command,"
						"f32_left_feedforward_command,"
						"f32_right_feedforward_command,"
						"f32_left_feedback_command,"
						"f32_right_feedback_command,"
						"f32_left_target_velocity_mps,"
						"f32_right_target_velocity_mps,"
						"f32_left_launch_assist_floor,"
						"f32_right_launch_assist_floor,"
						"u32_encoder_timestamp_us,"
						"i32_left_encoder_count,"
						"i32_right_encoder_count,"
						"f32_left_encoder_omega_radps,"
						"f32_right_encoder_omega_radps,"
						"f32_left_encoder_distance_m,"
						"f32_right_encoder_distance_m,"
						"f32_left_encoder_velocity_mps,"
						"f32_right_encoder_velocity_mps,"
						"u32_imu_timestamp_us,"
						"u8_imu_status,"
						"u8_imu_interrupt_high,"
						"u8_accel_bias_valid,"
						"i16_imu_gyro_x,"
						"i16_imu_gyro_y,"
						"i16_imu_gyro_z,"
						"i16_imu_accel_x,"
						"i16_imu_accel_y,"
						"i16_imu_accel_z,"
						"i16_imu_temp,"
						"f32_gyro_raw_radps,"
						"f32_gyro_bias_radps,"
						"f32_gyro_radps,"
						"f32_accel_body_x_mps2,"
						"f32_accel_body_y_mps2,"
						"f32_planar_accel_mps2,"
						"u32_front_timestamp_us,"
						"u32_left_timestamp_us,"
						"u32_right_timestamp_us,"
						"u8_front_left_obs_class,"
						"u8_front_right_obs_class,"
						"u8_left_obs_class,"
						"u8_right_obs_class,"
						"f32_front_left_obs_rho_m,"
						"f32_front_right_obs_rho_m,"
						"f32_left_obs_rho_m,"
						"f32_right_obs_rho_m,"
						"f32_front_left_obs_confidence,"
						"f32_front_right_obs_confidence,"
						"f32_left_obs_confidence,"
						"f32_right_obs_confidence,"
						"f32_fan_duty_cycle";

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
				};*/
	}
}

#endif
