#ifndef RUNTIMEINFRASTRUCTURESUPPORT_H
#define RUNTIMEINFRASTRUCTURESUPPORT_H

// Defines shared runtime logging helpers used by MazeMap diagnostic and measurement infrastructure.

#include "OpenFloorMeasurementSpec.h"
#include "DriveBase.h"
#include "MazeMapRuntimeSensors.h"
#include "MazeMapRuntimeCsvLog.h"
#include "MazeMapRuntimeMmLog.h"

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

#include <array>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <stdio.h>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				static constexpr uint32_t kRuntimeBinaryLogFlags = mmlog::FLAG_HAS_METADATA | mmlog::FLAG_HAS_NOTES;
				inline constexpr const char* kRuntimeControlLogFileName = "logging.txt";

				inline bool AppendRuntimeBinaryNotes(RuntimeTextBlockBuilder<512U>& notes, const char* eventFileName)
				{
					if (!notes.AppendKeyValue("format_spec", "micromouse_logging_file_format_rev_g"))
					{
						return false;
					}
					if (!notes.AppendKeyValue("endianness", "little"))
					{
						return false;
					}
					(void)eventFileName;
					return true;
				}

				inline bool AppendRuntimeControlLogLine(const char* line)
				{
					if (line == nullptr || line[0] == '\0')
					{
						return false;
					}

#if defined(ARDUINO_TEENSY41)
					File file = SD.open(kRuntimeControlLogFileName, FILE_WRITE);
					if (!file)
					{
						return false;
					}

					const bool ok = (file.print(line) > 0U) && (file.write('\n') == 1U);
					file.flush();
					file.close();
					return ok;
#else
					std::ofstream file(kRuntimeControlLogFileName, std::ios::out | std::ios::app);
					if (!file.is_open())
					{
						return false;
					}

					file << line << '\n';
					file.flush();
					return file.good();
#endif
				}

				inline bool AppendRuntimeControlLogEntry(
					const char* source,
					unsigned long timestampUs,
					const char* type,
					const char* message)
				{
					char line[384] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s [%lu] %s%s%s",
						(source != nullptr && source[0] != '\0') ? source : "runtime",
						timestampUs,
						(type != nullptr && type[0] != '\0') ? type : "event",
						(message != nullptr && message[0] != '\0') ? ": " : "",
						(message != nullptr) ? message : "");
					if (length <= 0)
					{
						return false;
					}

					line[sizeof(line) - 1U] = '\0';
					return AppendRuntimeControlLogLine(line);
				}

				template <std::size_t N>
				inline bool AppendBinaryRecord(RuntimeBinaryLogFile& log, const RuntimeRecordBuilder<N>& record)
				{
					return record.IsFull() && log.AppendRecord(record.Data(), record.Count());
				}

				template <std::size_t N>
				inline void AppendDriveTelemetryFields(RuntimeRecordBuilder<N>& record, const DriveTelemetry& telemetry)
				{
					record.I32(telemetry.leftEncoderCount);
					record.I32(telemetry.rightEncoderCount);
					record.F32(telemetry.leftDistanceM);
					record.F32(telemetry.rightDistanceM);
					record.F32(telemetry.leftVelocityMps);
					record.F32(telemetry.rightVelocityMps);
				}

				template <std::size_t N>
				inline void AppendImuTelemetryFields(RuntimeRecordBuilder<N>& record, const ImuTelemetry& imu)
				{
					record.U32(imu.status);
					record.I32(imu.gyroX);
					record.I32(imu.gyroY);
					record.I32(imu.gyroZ);
					record.I32(imu.accelX);
					record.I32(imu.accelY);
					record.I32(imu.accelZ);
					record.I32(imu.temp);
					record.U32(imu.interruptHigh ? 1U : 0U);
				}

				template <std::size_t N>
				inline void AppendWallSensorFields(RuntimeRecordBuilder<N>& record, const WallSensorTelemetry& sensor)
				{
					record.F32(sensor.ambientLight);
					record.F32(sensor.litLight);
					record.F32(sensor.differentialLight);
					record.F32(sensor.rawDistanceM);
					record.F32(sensor.distanceM);
				}

				template <std::size_t N>
				inline void AppendControlTimingFields(RuntimeRecordBuilder<N>& record, const ControlCycleTiming& timing)
				{
					record.U32(timing.controlStartUs);
					record.U32(timing.controlEndUs);
					record.U32(timing.pwmLatchUs);
					record.U32(timing.encoderLatchUs);
					record.U32(timing.encoderReadDoneUs);
					record.U32(timing.ukfPredictStartUs);
					record.U32(timing.ukfPredictEndUs);
					record.U32(timing.ukfPredictDurationUs);
					record.U32(timing.ukfUpdateStartUs);
					record.U32(timing.ukfUpdateEndUs);
					record.U32(timing.ukfUpdateDurationUs);
					record.U32(timing.cycleCounterStart);
					record.U32(timing.cycleCounterEnd);
				}

				template <std::size_t N>
				inline void AppendImuTimingFields(RuntimeRecordBuilder<N>& record, const ImuObservationTiming& timing)
				{
					record.U32(timing.drdyUs);
					record.U32(timing.readStartUs);
					record.U32(timing.readDoneUs);
				}

				template <std::size_t N>
				inline void AppendOpticalTimingFields(RuntimeRecordBuilder<N>& record, const OpticalObservationTiming& timing)
				{
					record.U32(timing.ledOnCommandUs);
					record.U32(timing.adcOnSampleUs);
					record.U32(timing.ledOffCommandUs);
					record.U32(timing.adcOffSampleUs);
					record.U32(timing.observationReadyUs);
				}
			}
		}
	}
}

#endif
