#pragma once

#include "Defines.h"
#include "LSM6DSV16X_IMU.h"
#include "SensorSnapshot.h"

#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    class EXPORT Imu final
    {
    public:
        enum class AccelFilterFreq : uint8_t
        {
            Frac1Over002 = 0x00,
            Frac1Over004 = 0x10,
            Frac1Over010 = 0x30,
            Frac1Over020 = 0x50,
            Frac1Over045 = 0x70,
            Frac1Over100 = 0x90,
            Frac1Over200 = 0xB0,
            Frac1Over400 = 0xD0,
            Frac1Over800 = 0xF0,
        };

        Imu(
            uint8_t csPin,
            uint8_t intPin,
            uint8_t mosiPin,
            uint8_t misoPin,
            uint8_t clockPin,
            const MazeMap::SensorMount& mount = MazeMap::SensorMount()) noexcept;

        bool Begin();
        bool Reset(uint32_t timeoutMs = 50U);
        uint8_t ReadWhoAmI();
        uint8_t ReadWhoAmIWithSettings(uint32_t clockHz, uint8_t dataMode);
        bool IsConnected();
        const char* GetLastBeginFailureReasonName() const noexcept;
        uint8_t GetLastWhoAmI() const noexcept;

        bool ConfigureRuntimeForControlPeriod(
            unsigned long controlPeriodUs,
            bool enableAccel,
            AccelFilterFreq accelFilterFreq);
        void ConfigureRuntimeRanges(bool enableAccel, AccelFilterFreq accelFilterFreq);
        void DisableSelfTest();
        void EnablePositiveSelfTest();

        void ResetRuntimeCalibration() noexcept;
        void SetRuntimeCalibration(
            float gyroBiasRadps,
            bool accelBiasInitialized,
            float accelBiasRightG,
            float accelBiasForwardG) noexcept;
        float RuntimeGyroBiasRadps() const noexcept;
        bool HasRuntimeAccelBias() const noexcept;
        float RuntimeAccelBiasRightG() const noexcept;
        float RuntimeAccelBiasForwardG() const noexcept;

        float AccelSensitivityMgPerLsb() const noexcept;
        float GyroSensitivityMdpsPerLsb() const noexcept;
        ImuTelemetry CaptureTelemetry(ImuObservationTiming* timing = nullptr);
        void CaptureRuntimeInertialSnapshot(SensorSnapshot& snapshot);

        void ResetCalibrationSampling() noexcept;
        void BeginSelfTestSampling() noexcept;
        void AccumulateSelfTestSample() noexcept;
        void StoreCurrentSelfTestAverageAsBaseline() noexcept;
        bool ValidateStimulatedSelfTestAverage() noexcept;
        void BeginStationaryBiasSampling() noexcept;
        void AccumulateStationaryBiasSample() noexcept;
        bool CompleteStationaryBiasSampling() noexcept;
        unsigned long CalibrationCollectedSamples() const noexcept;
        float LastSelfTestAccelDeltaMg(uint8_t axis) const noexcept;
        float LastSelfTestGyroDeltaDps(uint8_t axis) const noexcept;

        bool SelfTestDeltasValid(
            float accelDeltaMgX,
            float accelDeltaMgY,
            float accelDeltaMgZ,
            float gyroDeltaDpsX,
            float gyroDeltaDpsY,
            float gyroDeltaDpsZ) const noexcept;
        static bool IsAccelSelfTestDeltaValidMg(float deltaMg) noexcept;
        static bool IsGyroSelfTestDeltaValidDps(float deltaDps, float fullScaleDps) noexcept;

        static unsigned long GetUiImuSampleRateHzForControlPeriodUs(unsigned long controlPeriodUs) noexcept;
        static float GetUiAccelLpf2CutoffHzForControlPeriodUs(unsigned long controlPeriodUs) noexcept;
        static float GetUiAccelLpf2CutoffHzForControlPeriodUs(
            unsigned long controlPeriodUs,
            AccelFilterFreq filterFreq) noexcept;
        static float GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(unsigned long controlPeriodUs) noexcept;
        static unsigned long ComputeRequiredStationaryBiasSamples(
            unsigned long configuredSamples,
            unsigned long controlPeriodUs,
            uint32_t sampleIntervalTicks,
            unsigned long minimumWindowMs) noexcept;

    private:
        static LSM6DSV16X_IMU::ACCEL_FILTER_FREQ ToDriverAccelFilterFreq(AccelFilterFreq filterFreq) noexcept;

        LSM6DSV16X_IMU _driver;
        float _gyroBiasRadps = 0.0f;
        float _accelBiasRightG = 0.0f;
        float _accelBiasForwardG = 0.0f;
        bool _accelBiasInitialized = false;
        unsigned long _calibrationCollectedSamples = 0UL;
        double _calibrationAccelMgSumX = 0.0;
        double _calibrationAccelMgSumY = 0.0;
        double _calibrationAccelMgSumZ = 0.0;
        double _calibrationGyroDpsSumX = 0.0;
        double _calibrationGyroDpsSumY = 0.0;
        double _calibrationGyroDpsSumZ = 0.0;
        double _calibrationGyroBiasRadpsSum = 0.0;
        double _calibrationAccelBiasRightGSum = 0.0;
        double _calibrationAccelBiasForwardGSum = 0.0;
        float _baselineAccelMgX = 0.0f;
        float _baselineAccelMgY = 0.0f;
        float _baselineAccelMgZ = 0.0f;
        float _baselineGyroDpsX = 0.0f;
        float _baselineGyroDpsY = 0.0f;
        float _baselineGyroDpsZ = 0.0f;
        float _lastSelfTestAccelDeltaMg[3] = {};
        float _lastSelfTestGyroDeltaDps[3] = {};
    };
}
