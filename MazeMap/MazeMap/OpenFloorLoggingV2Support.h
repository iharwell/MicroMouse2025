#ifndef OPENFLOORLOGGINGV2SUPPORT_H
#define OPENFLOORLOGGINGV2SUPPORT_H

// Defines shared flag calculations used by the second-generation open-floor runtime loggers.

#include "OpenFloorMeasurementCycle.h"
#include "OpenFloorMeasurementLabels.h"
#include "OpenFloorMeasurementSpec.h"
#include "RuntimeBinaryLogFile.h"
#include "VehicleState.h"

#include <cstdint>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				namespace OpenFloorLoggingV2
				{
					static constexpr uint16_t kLoggerFlagOverflow = 1u << 0;
					static constexpr uint16_t kLoggerFlagWriteFailure = 1u << 1;

					static constexpr uint16_t kMeasurementFlagAbortMarker = 1u << 0;
					static constexpr uint16_t kMeasurementFlagWorkspaceViolation = 1u << 1;
					static constexpr uint16_t kMeasurementFlagEstimatorFault = 1u << 2;
					static constexpr uint16_t kMeasurementFlagFanEnabled = 1u << 3;
					static constexpr uint16_t kMeasurementFlagEncoderValid = 1u << 4;
					static constexpr uint16_t kMeasurementFlagImuValid = 1u << 5;
					static constexpr uint16_t kMeasurementFlagAccelBiasValid = 1u << 6;
					static constexpr uint16_t kMeasurementFlagFrontLeftObsValid = 1u << 7;
					static constexpr uint16_t kMeasurementFlagFrontRightObsValid = 1u << 8;
					static constexpr uint16_t kMeasurementFlagLeftObsValid = 1u << 9;
					static constexpr uint16_t kMeasurementFlagRightObsValid = 1u << 10;

					inline uint16_t LoggerFlags(const RuntimeBinaryLogFile& log)
					{
						uint16_t flags = 0U;
						if (log.HadOverflow()) flags |= kLoggerFlagOverflow;
						if (log.HadWriteFailure()) flags |= kLoggerFlagWriteFailure;
						return flags;
					}

					inline uint16_t MeasurementFlags(
						const OpenFloorMeasurementLabels& labels,
						const OpenFloorMeasurementCycle& cycle,
						bool encoderValid,
						bool imuValid,
						const MazeMap::WallObs& frontLeftObs,
						const MazeMap::WallObs& frontRightObs,
						const MazeMap::WallObs& leftObs,
						const MazeMap::WallObs& rightObs)
					{
						uint16_t flags = 0U;
						if (labels.abortMarker) flags |= kMeasurementFlagAbortMarker;
						if (cycle.workspaceViolation) flags |= kMeasurementFlagWorkspaceViolation;
						if (cycle.estimatorFault) flags |= kMeasurementFlagEstimatorFault;
						if (cycle.fanDutyCycle > 0.0f) flags |= kMeasurementFlagFanEnabled;
						if (encoderValid) flags |= kMeasurementFlagEncoderValid;
						if (imuValid) flags |= kMeasurementFlagImuValid;
						if (cycle.sensorSnapshot.accelBiasValid) flags |= kMeasurementFlagAccelBiasValid;
						if (frontLeftObs.valid) flags |= kMeasurementFlagFrontLeftObsValid;
						if (frontRightObs.valid) flags |= kMeasurementFlagFrontRightObsValid;
						if (leftObs.valid) flags |= kMeasurementFlagLeftObsValid;
						if (rightObs.valid) flags |= kMeasurementFlagRightObsValid;
						return flags;
					}
				}
			}
		}
	}
}

#endif
