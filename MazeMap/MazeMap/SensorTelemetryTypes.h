#pragma once

#include <cstdint>
#include <stdint.h>

struct OpticalObservationTiming
{
    uint32_t ledOnCommandUs = 0UL;
    uint32_t adcOnSampleUs = 0UL;
    uint32_t ledOffCommandUs = 0UL;
    uint32_t adcOffSampleUs = 0UL;
    uint32_t observationReadyUs = 0UL;
};

struct ImuObservationTiming
{
    uint32_t drdyUs = 0UL;
    uint32_t readStartUs = 0UL;
    uint32_t readDoneUs = 0UL;
};

struct WallSensorTelemetry
{
    float ambientLight = 0.0f;
    float litLight = 0.0f;
    float differentialLight = 0.0f;
    float rawDistanceM = 0.20f;
    float distanceM = 0.20f;
    bool wall = false;
};

struct ImuTelemetry
{
    uint8_t status = 0U;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    int16_t accelX = 0;
    int16_t accelY = 0;
    int16_t accelZ = 0;
    int16_t temp = 0;
    bool interruptHigh = false;
};
