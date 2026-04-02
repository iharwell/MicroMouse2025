#include "pch.h"
#include "DriveBase.h"
#include "MazeMapRuntimeSensors.h"
#include "OpenFloorMainLoggerV2.h"
#include "OpenFloorLoggingV2Support.h"
#include "RuntimeBinaryRecordSupport.h"

#include <cmath>
#include <stdio.h>

namespace OpenFloorLoggingV2 = MazeMap::App::Internal::Runtime::OpenFloorLoggingV2;

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				bool OpenFloorMainLoggerV2::Begin(const DiagnosticSensorSuite& sensors, const char* runId)
				{
					_metadata.Clear();
					_notes.Clear();
					if (!_eventLog.BeginSibling(MazeMap::kOpenFloorMainFileName)) return false;
					if (!_metadata.AppendKeyValue("file", MazeMap::kOpenFloorMainFileName)) return false;
					if (_eventLog.IsEnabled() && !_metadata.AppendKeyValue("control_log_file", _eventLog.GetFileName())) return false;
					if (!_metadata.AppendKeyValue("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
					if (!_metadata.AppendKeyValue("stream_type", "open_floor_main")) return false;
					if (!_metadata.AppendKeyValue("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return false;
					if (!_metadata.AppendKeyValue("active_imu_id", MazeMap::kOpenFloorActiveImuId)) return false;
					if (!_metadata.AppendKeyValue("imu_extrinsics_revision", MazeMap::kOpenFloorImuExtrinsicsRevision)) return false;
					if (runId != nullptr && runId[0] != '\0' && !_metadata.AppendKeyValue("run_id", runId)) return false;
					if (!_metadata.AppendUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
					if (!_metadata.AppendFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
					if (!_metadata.AppendFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
					if (!MazeMap::App::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName())) return false;
					if (!_notes.AppendLine("primary_record=one_measurement_row_per_control_loop")) return false;
					if (!_notes.AppendLine("phase_transitions_faults_and_summaries_are_emitted_to_logging_txt")) return false;

					if (_eventLog.IsEnabled())
					{
						(void)_eventLog.WriteMetadata("file", _eventLog.GetFileName());
						(void)_eventLog.WriteMetadata("data_file", MazeMap::kOpenFloorMainFileName);
						(void)_eventLog.WriteMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName);
						(void)_eventLog.WriteMetadata("stream_type", "open_floor_main_control_log");
						(void)_eventLog.WriteMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision);
						if (runId != nullptr && runId[0] != '\0')
						{
							(void)_eventLog.WriteMetadata("run_id", runId);
						}
					}

					if (!_sampleLog.BeginSelected(
							MazeMap::kOpenFloorMainFileName,
							kMainSchema,
							kMainFieldCount,
							_metadata.Data(),
							_notes.Data()))
					{
						_eventLog.Close();
						return false;
					}
					return true;
				}

				bool OpenFloorMainLoggerV2::LogSample(
					const OpenFloorMeasurementLabels& labels,
					const PoseEstimate& pose,
					const DriveBase& drive,
					const OpenFloorMeasurementCycle& cycle)
				{
					const bool encoderValid = cycle.driveTelemetry.encoderObservationValid;
					const bool imuValid = std::isfinite(cycle.sensorSnapshot.gyroRawRadps);
					const float maxRangeM = MazeMap::PlantParams::Default().noHitRangeM;
					MazeMap::WallObs frontLeftObs{};
					MazeMap::WallObs frontRightObs{};
					DriveBase::BuildLoggedFrontPairObservations(cycle.sensorSnapshot, maxRangeM, frontLeftObs, frontRightObs);
					const MazeMap::WallObs leftObs = DriveBase::BuildLoggedLeftSideObservation(cycle.sensorSnapshot, maxRangeM);
					const MazeMap::WallObs rightObs = DriveBase::BuildLoggedRightSideObservation(cycle.sensorSnapshot, maxRangeM);

					MazeMap::App::Internal::Runtime::RuntimeRecordBuilder<kMainFieldCount> record;
					record.U32(cycle.masterTimeUs);
					record.U32(cycle.controlTickSequence);
					record.U32(cycle.dtUs);
					record.U32(static_cast<uint32_t>(labels.sectionId));
					record.U32(static_cast<uint32_t>(labels.primitiveId));
					record.U32(static_cast<uint32_t>(MazeMap::OpenFloorPrimitiveFamilyForId(labels.primitiveId)));
					record.U32(static_cast<uint32_t>(labels.directionId));
					record.U32(static_cast<uint32_t>(labels.phaseId));
					record.U32(static_cast<uint32_t>(labels.speedBin));
					record.U32(static_cast<uint32_t>(labels.startMarkerId));
					record.U32(MazeMap::OpenFloorPrimitiveIsMirrored(labels.primitiveId) ? 1U : 0U);
					record.U32(labels.repeatIndex);
					record.F32(labels.progressNorm);
					record.U32(cycle.driveTelemetry.modeFlags);
					record.U32(cycle.clippingFlags);
					record.U32(cycle.driveTelemetry.saturationFlags);
					record.U32(OpenFloorLoggingV2::LoggerFlags(_sampleLog));
					record.U32(cycle.watchdogFlags);
					record.U32(OpenFloorLoggingV2::MeasurementFlags(
						labels,
						cycle,
						encoderValid,
						imuValid,
						frontLeftObs,
						frontRightObs,
						leftObs,
						rightObs));
					record.F32(pose.xMeters);
					record.F32(pose.yMeters);
					record.F32(pose.yawRad);
					record.F32(cycle.measuredLinearSpeedMps);
					record.F32(cycle.measuredAngularSpeedRadps);
					record.F32(drive.GetLastLinearCommandMps());
					record.F32(drive.GetLastAngularCommandRadps());
					record.F32(cycle.driveTelemetry.leftDriveCommand);
					record.F32(cycle.driveTelemetry.rightDriveCommand);
					record.F32(cycle.driveTelemetry.leftFeedforwardCommand);
					record.F32(cycle.driveTelemetry.rightFeedforwardCommand);
					record.F32(cycle.driveTelemetry.leftFeedbackCommand);
					record.F32(cycle.driveTelemetry.rightFeedbackCommand);
					record.F32(cycle.driveTelemetry.leftTargetVelocityMps);
					record.F32(cycle.driveTelemetry.rightTargetVelocityMps);
					record.F32(cycle.driveTelemetry.leftLaunchAssistFloor);
					record.F32(cycle.driveTelemetry.rightLaunchAssistFloor);
					record.U32(cycle.controlTiming.encoderReadDoneUs);
					record.I32(cycle.driveTelemetry.leftEncoderCount);
					record.I32(cycle.driveTelemetry.rightEncoderCount);
					record.F32(cycle.driveTelemetry.leftEncoderOmegaRadps);
					record.F32(cycle.driveTelemetry.rightEncoderOmegaRadps);
					record.F32(cycle.driveTelemetry.leftDistanceM);
					record.F32(cycle.driveTelemetry.rightDistanceM);
					record.F32(cycle.driveTelemetry.leftVelocityMps);
					record.F32(cycle.driveTelemetry.rightVelocityMps);
					record.U32(cycle.sensorSnapshot.imuTiming.readDoneUs);
					record.U32(cycle.sensorSnapshot.imuBackLeft.status);
					record.U32(cycle.sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U);
					record.U32(cycle.sensorSnapshot.accelBiasValid ? 1U : 0U);
					record.I32(cycle.sensorSnapshot.imuBackLeft.gyroX);
					record.I32(cycle.sensorSnapshot.imuBackLeft.gyroY);
					record.I32(cycle.sensorSnapshot.imuBackLeft.gyroZ);
					record.I32(cycle.sensorSnapshot.imuBackLeft.accelX);
					record.I32(cycle.sensorSnapshot.imuBackLeft.accelY);
					record.I32(cycle.sensorSnapshot.imuBackLeft.accelZ);
					record.I32(cycle.sensorSnapshot.imuBackLeft.temp);
					record.F32(cycle.sensorSnapshot.gyroRawRadps);
					record.F32(cycle.sensorSnapshot.gyroBiasRadps);
					record.F32(cycle.sensorSnapshot.gyroRadps);
					record.F32(cycle.sensorSnapshot.accelBodyXMps2);
					record.F32(cycle.sensorSnapshot.accelBodyYMps2);
					record.F32(cycle.planarAccelMps2);
					record.U32(cycle.sensorSnapshot.frontTiming.observationReadyUs);
					record.U32(cycle.sensorSnapshot.leftTiming.observationReadyUs);
					record.U32(cycle.sensorSnapshot.rightTiming.observationReadyUs);
					record.U32(static_cast<uint32_t>(frontLeftObs.cls));
					record.U32(static_cast<uint32_t>(frontRightObs.cls));
					record.U32(static_cast<uint32_t>(leftObs.cls));
					record.U32(static_cast<uint32_t>(rightObs.cls));
					record.F32(frontLeftObs.rho);
					record.F32(frontRightObs.rho);
					record.F32(leftObs.rho);
					record.F32(rightObs.rho);
					record.F32(frontLeftObs.confidence);
					record.F32(frontRightObs.confidence);
					record.F32(leftObs.confidence);
					record.F32(rightObs.confidence);
					record.F32(cycle.batteryVoltage);
					record.F32(cycle.boardTemperatureC);
					record.F32(cycle.fanDutyCycle);
					return MazeMap::App::Internal::Runtime::AppendBinaryRecord(_sampleLog, record);
				}

				bool OpenFloorMainLoggerV2::LogFault(
					const OpenFloorMeasurementLabels& labels,
					const OpenFloorMeasurementCycle& cycle,
					MazeMap::OpenFloorFaultCode faultCode,
					bool controlHalted,
					uint32_t extra0,
					uint32_t extra1)
				{
					char message[384] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"fault=%s;section_id=%s;primitive_id=%s;direction=%s;phase_id=%s;speed_bin=%s;start_marker=%s;repeat_index=%u;mirrored=%u;control_halted=%u;extra0=%lu;extra1=%lu",
						MazeMap::OpenFloorFaultName(faultCode),
						MazeMap::OpenFloorSectionName(labels.sectionId),
						MazeMap::OpenFloorPrimitiveName(labels.primitiveId),
						MazeMap::OpenFloorDirectionName(labels.directionId),
						MazeMap::OpenFloorPhaseName(labels.phaseId),
						MazeMap::OpenFloorSpeedBinName(labels.speedBin),
						MazeMap::OpenFloorMarkerName(labels.startMarkerId),
						static_cast<unsigned>(labels.repeatIndex),
						MazeMap::OpenFloorPrimitiveIsMirrored(labels.primitiveId) ? 1U : 0U,
						controlHalted ? 1U : 0U,
						static_cast<unsigned long>(extra0),
						static_cast<unsigned long>(extra1));
					if (length <= 0)
					{
						return false;
					}
					message[sizeof(message) - 1U] = '\0';
					return _eventLog.WriteEvent(micros(), "fault", message);
				}

				bool OpenFloorMainLoggerV2::BeginSection(const OpenFloorMeasurementLabels& labels)
				{
					return WriteSectionMarker("section_start", labels, nullptr);
				}

				bool OpenFloorMainLoggerV2::EndSection(const OpenFloorMeasurementLabels& labels)
				{
					return WriteSectionMarker("section_end", labels, nullptr);
				}

				bool OpenFloorMainLoggerV2::AbortSection(const OpenFloorMeasurementLabels& labels, const char* reason)
				{
					return WriteSectionMarker("abort", labels, reason);
				}

				bool OpenFloorMainLoggerV2::WriteEvent(const char* type, const char* message)
				{
					return _eventLog.WriteEvent(micros(), type, message);
				}

				void OpenFloorMainLoggerV2::Service()
				{
					(void)_sampleLog.Service(1U);
				}

				void OpenFloorMainLoggerV2::Flush()
				{
					_sampleLog.Flush();
					_eventLog.Flush();
				}

				void OpenFloorMainLoggerV2::Close()
				{
					_sampleLog.Close();
					_eventLog.Close();
				}

				bool OpenFloorMainLoggerV2::WriteSectionMarker(
					const char* type,
					const OpenFloorMeasurementLabels& labels,
					const char* reason)
				{
					char message[256] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"section_id=%s;primitive_id=%s;direction=%s;start_marker=%s;repeat_index=%u;speed_bin=%s%s%s",
						MazeMap::OpenFloorSectionName(labels.sectionId),
						MazeMap::OpenFloorPrimitiveName(labels.primitiveId),
						MazeMap::OpenFloorDirectionName(labels.directionId),
						MazeMap::OpenFloorMarkerName(labels.startMarkerId),
						static_cast<unsigned>(labels.repeatIndex),
						MazeMap::OpenFloorSpeedBinName(labels.speedBin),
						(reason != nullptr && reason[0] != '\0') ? ";reason=" : "",
						(reason != nullptr && reason[0] != '\0') ? reason : "");
					if (length <= 0 || length >= static_cast<int>(sizeof(message)))
					{
						return false;
					}
					return _eventLog.WriteEvent(micros(), type, message);
				}
			}
		}
	}
}
