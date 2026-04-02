#include "pch.h"
#include "OpenFloorTimingLoggerV2.h"
#include "OpenFloorLoggingV2Support.h"
#include "RuntimeBinaryRecordSupport.h"

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
				bool OpenFloorTimingLoggerV2::Begin(const char* runId)
				{
					_metadata.Clear();
					_notes.Clear();
					if (!_eventLog.BeginSibling(MazeMap::kOpenFloorTimingFileName)) return false;
					if (!_metadata.AppendKeyValue("file", MazeMap::kOpenFloorTimingFileName)) return false;
					if (_eventLog.IsEnabled() && !_metadata.AppendKeyValue("control_log_file", _eventLog.GetFileName())) return false;
					if (!_metadata.AppendKeyValue("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
					if (!_metadata.AppendKeyValue("stream_type", "open_floor_timing")) return false;
					if (!_metadata.AppendKeyValue("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision)) return false;
					if (runId != nullptr && runId[0] != '\0' && !_metadata.AppendKeyValue("run_id", runId)) return false;
					if (!_metadata.AppendUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
					if (!MazeMap::App::Internal::Runtime::AppendRuntimeBinaryNotes(_notes, _eventLog.GetFileName())) return false;

					if (_eventLog.IsEnabled())
					{
						(void)_eventLog.WriteMetadata("file", _eventLog.GetFileName());
						(void)_eventLog.WriteMetadata("data_file", MazeMap::kOpenFloorTimingFileName);
						(void)_eventLog.WriteMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName);
						(void)_eventLog.WriteMetadata("stream_type", "open_floor_timing_control_log");
						(void)_eventLog.WriteMetadata("logging_format_revision", MazeMap::kOpenFloorLoggingFormatRevision);
						if (runId != nullptr && runId[0] != '\0')
						{
							(void)_eventLog.WriteMetadata("run_id", runId);
						}
					}

					if (!_sampleLog.BeginSelected(
							MazeMap::kOpenFloorTimingFileName,
							kTimingSchema,
							kTimingFieldCount,
							_metadata.Data(),
							_notes.Data()))
					{
						_eventLog.Close();
						return false;
					}
					return true;
				}

				bool OpenFloorTimingLoggerV2::LogSample(const OpenFloorMeasurementCycle& cycle)
				{
					MazeMap::App::Internal::Runtime::RuntimeRecordBuilder<kTimingFieldCount> record;
					record.U32(cycle.masterTimeUs);
					record.U32(cycle.controlTickSequence);
					record.U32(cycle.dtUs);
					record.U32(static_cast<uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing));
					record.U32(OpenFloorLoggingV2::LoggerFlags(_sampleLog));
					record.U32(cycle.controlTiming.controlStartUs);
					record.U32(cycle.controlTiming.controlEndUs);
					record.U32(cycle.controlTiming.pwmLatchUs);
					record.U32(cycle.controlTiming.encoderLatchUs);
					record.U32(cycle.controlTiming.encoderReadDoneUs);
					record.U32(cycle.controlTiming.ukfPredictStartUs);
					record.U32(cycle.controlTiming.ukfPredictEndUs);
					record.U32(cycle.controlTiming.ukfPredictDurationUs);
					record.U32(cycle.controlTiming.ukfUpdateStartUs);
					record.U32(cycle.controlTiming.ukfUpdateEndUs);
					record.U32(cycle.controlTiming.ukfUpdateDurationUs);
					record.U32(cycle.sensorSnapshot.imuTiming.drdyUs);
					record.U32(cycle.sensorSnapshot.imuTiming.readStartUs);
					record.U32(cycle.sensorSnapshot.imuTiming.readDoneUs);
					record.U32(cycle.sensorSnapshot.frontTiming.ledOnCommandUs);
					record.U32(cycle.sensorSnapshot.frontTiming.adcOnSampleUs);
					record.U32(cycle.sensorSnapshot.frontTiming.ledOffCommandUs);
					record.U32(cycle.sensorSnapshot.frontTiming.adcOffSampleUs);
					record.U32(cycle.sensorSnapshot.frontTiming.observationReadyUs);
					record.U32(cycle.sensorSnapshot.leftTiming.ledOnCommandUs);
					record.U32(cycle.sensorSnapshot.leftTiming.adcOnSampleUs);
					record.U32(cycle.sensorSnapshot.leftTiming.ledOffCommandUs);
					record.U32(cycle.sensorSnapshot.leftTiming.adcOffSampleUs);
					record.U32(cycle.sensorSnapshot.leftTiming.observationReadyUs);
					record.U32(cycle.sensorSnapshot.rightTiming.ledOnCommandUs);
					record.U32(cycle.sensorSnapshot.rightTiming.adcOnSampleUs);
					record.U32(cycle.sensorSnapshot.rightTiming.ledOffCommandUs);
					record.U32(cycle.sensorSnapshot.rightTiming.adcOffSampleUs);
					record.U32(cycle.sensorSnapshot.rightTiming.observationReadyUs);
					record.U32(cycle.controlTiming.cycleCounterStart);
					record.U32(cycle.controlTiming.cycleCounterEnd);
					return MazeMap::App::Internal::Runtime::AppendBinaryRecord(_sampleLog, record);
				}

				bool OpenFloorTimingLoggerV2::LogFault(
					const OpenFloorMeasurementCycle& cycle,
					MazeMap::OpenFloorFaultCode faultCode,
					bool controlHalted,
					uint32_t extra0,
					uint32_t extra1)
				{
					char message[256] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"fault=%s;section_id=%s;control_halted=%u;tick=%lu;dt_us=%lu;extra0=%lu;extra1=%lu",
						MazeMap::OpenFloorFaultName(faultCode),
						MazeMap::OpenFloorSectionName(MazeMap::OpenFloorSectionId::Sec00Timing),
						controlHalted ? 1U : 0U,
						static_cast<unsigned long>(cycle.controlTickSequence),
						static_cast<unsigned long>(cycle.dtUs),
						static_cast<unsigned long>(extra0),
						static_cast<unsigned long>(extra1));
					if (length <= 0)
					{
						return false;
					}
					message[sizeof(message) - 1U] = '\0';
					return _eventLog.WriteEvent(micros(), "fault", message);
				}

				bool OpenFloorTimingLoggerV2::LogSummary(
					const char* streamId,
					unsigned long sampleCount,
					float meanDelayUs,
					float jitterUs)
				{
					char message[192] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"%s;n=%lu;mean_delay_us=%.3f;jitter_us=%.3f",
						(streamId != nullptr) ? streamId : "timing_stream",
						sampleCount,
						meanDelayUs,
						jitterUs);
					if (length <= 0 || length >= static_cast<int>(sizeof(message)))
					{
						return false;
					}
					return _eventLog.WriteEvent(micros(), "summary", message);
				}

				bool OpenFloorTimingLoggerV2::LogFailure(const char* reason)
				{
					return _eventLog.WriteEvent(micros(), "fault", (reason != nullptr) ? reason : "timing_failure");
				}

				void OpenFloorTimingLoggerV2::Service()
				{
					(void)_sampleLog.Service(1U);
				}

				void OpenFloorTimingLoggerV2::Flush()
				{
					_sampleLog.Flush();
					_eventLog.Flush();
				}

				void OpenFloorTimingLoggerV2::Close()
				{
					_sampleLog.Close();
					_eventLog.Close();
				}
			}
		}
	}
}
