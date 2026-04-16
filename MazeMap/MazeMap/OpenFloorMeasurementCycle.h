#ifndef OPENFLOORMEASUREMENTCYCLE_H
#define OPENFLOORMEASUREMENTCYCLE_H

// Declares one captured open-floor control-cycle sample, including timing, drivetrain, sensor, and diagnostic state.

#include "MazeMapRuntimeCore.h"

#include <limits>

struct OpenFloorMeasurementCycle
{
    uint32_t masterTimeUs = 0UL;
    uint32_t controlTickSequence = 0UL;
    uint32_t dtUs = 0UL;
    ControlCycleTiming controlTiming{};
    DriveTelemetry driveTelemetry{};
    SensorSnapshot sensorSnapshot{};
    float measuredLinearSpeedMps = 0.0f;
    float measuredAngularSpeedRadps = 0.0f;
    float planarAccelMps2 = 0.0f;
    float batteryVoltage = std::numeric_limits<float>::quiet_NaN();
    float boardTemperatureC = std::numeric_limits<float>::quiet_NaN();
    float fanDutyCycle = 0.0f;
    uint16_t clippingFlags = 0U;
    uint16_t watchdogFlags = 0U;
    bool selectorJumperRemoved = false;
    bool estimatorFault = false;
};

#endif
