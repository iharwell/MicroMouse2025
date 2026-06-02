#include "pch.h"
#include "Imu.h"

#include <cmath>

namespace MazeMap
{
    Imu::Imu(
        const uint8_t csPin,
        const uint8_t intPin,
        const uint8_t mosiPin,
        const uint8_t misoPin,
        const uint8_t clockPin,
        const MazeMap::SensorMount& mount) noexcept
        : _driver(csPin, intPin, mosiPin, misoPin, clockPin, mount)
    {
    }

    bool Imu::Begin()
    {
        return _driver.Begin();
    }

    bool Imu::Reset(const uint32_t timeoutMs)
    {
        return _driver.Reset(timeoutMs);
    }

    uint8_t Imu::ReadWhoAmI()
    {
        return _driver.ReadWhoAmI();
    }

    uint8_t Imu::ReadWhoAmIWithSettings(const uint32_t clockHz, const uint8_t dataMode)
    {
        return _driver.ReadWhoAmIWithSettings(clockHz, dataMode);
    }

    bool Imu::IsConnected()
    {
        return _driver.IsConnected();
    }

    const char* Imu::GetLastBeginFailureReasonName() const noexcept
    {
        return _driver.GetLastBeginFailureReasonName();
    }

    uint8_t Imu::GetLastWhoAmI() const noexcept
    {
        return _driver.GetLastWhoAmI();
    }

    bool Imu::ConfigureRuntimeForControlPeriod(
        const unsigned long controlPeriodUs,
        const bool enableAccel,
        const AccelFilterFreq accelFilterFreq)
    {
        switch (controlPeriodUs)
        {
        case 1000UL:
            _driver.ConfigureUiHighAccuracyOdr(
                LSM6DSV16X_IMU::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                enableAccel ? LSM6DSV16X_IMU::ODR_SETTING::ODR_0960HZ_HP_N_LP : LSM6DSV16X_IMU::ODR_SETTING::DISABLE,
                LSM6DSV16X_IMU::ODR_SETTING::ODR_0960HZ_HP_N_LP);
            ConfigureRuntimeRanges(enableAccel, accelFilterFreq);
            return true;
        case 500UL:
            _driver.ConfigureUiHighAccuracyOdr(
                LSM6DSV16X_IMU::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                enableAccel ? LSM6DSV16X_IMU::ODR_SETTING::ODR_1920HZ_HP_N_LP : LSM6DSV16X_IMU::ODR_SETTING::DISABLE,
                LSM6DSV16X_IMU::ODR_SETTING::ODR_1920HZ_HP_N_LP);
            ConfigureRuntimeRanges(enableAccel, accelFilterFreq);
            return true;
        default:
            return false;
        }
    }

    void Imu::ConfigureRuntimeRanges(const bool enableAccel, const AccelFilterFreq accelFilterFreq)
    {
        _driver.SetSelfTest(
            LSM6DSV16X_IMU::SELF_TEST_MODE::DISABLED,
            LSM6DSV16X_IMU::SELF_TEST_MODE::DISABLED);
        if (enableAccel)
        {
            _driver.SetAccelRange(ToDriverAccelFilterFreq(accelFilterFreq), LSM6DSV16X_IMU::ACCEL_FULLSCALE::G8);
        }
        _driver.SetGyroRange(LSM6DSV16X_IMU::GYRO_LPF1_MODE::CUT_213, LSM6DSV16X_IMU::GYRO_FULLSCALE_RANGE::DPS2000);
    }

    void Imu::DisableSelfTest()
    {
        _driver.SetSelfTest(
            LSM6DSV16X_IMU::SELF_TEST_MODE::DISABLED,
            LSM6DSV16X_IMU::SELF_TEST_MODE::DISABLED);
    }

    void Imu::EnablePositiveSelfTest()
    {
        _driver.SetSelfTest(
            LSM6DSV16X_IMU::SELF_TEST_MODE::POSITIVE,
            LSM6DSV16X_IMU::SELF_TEST_MODE::POSITIVE);
    }

    void Imu::ResetRuntimeCalibration() noexcept
    {
        _gyroBiasRadps = 0.0f;
        _accelBiasRightG = 0.0f;
        _accelBiasForwardG = 0.0f;
        _accelBiasInitialized = false;
    }

    void Imu::SetRuntimeCalibration(
        const float gyroBiasRadps,
        const bool accelBiasInitialized,
        const float accelBiasRightG,
        const float accelBiasForwardG) noexcept
    {
        _gyroBiasRadps = std::isfinite(gyroBiasRadps) ? gyroBiasRadps : 0.0f;
        _accelBiasInitialized =
            accelBiasInitialized &&
            std::isfinite(accelBiasRightG) &&
            std::isfinite(accelBiasForwardG);
        if (_accelBiasInitialized)
        {
            _accelBiasRightG = accelBiasRightG;
            _accelBiasForwardG = accelBiasForwardG;
        }
        else
        {
            _accelBiasRightG = 0.0f;
            _accelBiasForwardG = 0.0f;
        }
    }

    float Imu::RuntimeGyroBiasRadps() const noexcept
    {
        return _gyroBiasRadps;
    }

    bool Imu::HasRuntimeAccelBias() const noexcept
    {
        return _accelBiasInitialized;
    }

    float Imu::RuntimeAccelBiasRightG() const noexcept
    {
        return _accelBiasRightG;
    }

    float Imu::RuntimeAccelBiasForwardG() const noexcept
    {
        return _accelBiasForwardG;
    }

    float Imu::AccelSensitivityMgPerLsb() const noexcept
    {
        return _driver.AccelSensitivityMgPerLsb();
    }

    float Imu::GyroSensitivityMdpsPerLsb() const noexcept
    {
        return _driver.GyroSensitivityMdpsPerLsb();
    }

    ImuTelemetry Imu::CaptureTelemetry(ImuObservationTiming* const timing)
    {
        return _driver.CaptureTelemetry(timing);
    }

    void Imu::CaptureRuntimeInertialSnapshot(SensorSnapshot& snapshot)
    {
        ImuObservationTiming timing{};
        const ImuTelemetry telemetry = CaptureTelemetry(&timing);
        snapshot.SetFrontRightImuTelemetry(ImuTelemetry{});
        snapshot.SetBackLeftImuTelemetry(telemetry);
        snapshot.SetImuTiming(timing);

        const float rawYawRateRadps = _driver.GyroRawToBodyYawRadps(telemetry.gyroZ);
        const Eigen::Vector2f accelBodyG = _driver.AccelRawToBodyPlanarG(telemetry.accelX, telemetry.accelY);
        snapshot.SetAccelerationBiasValid(_accelBiasInitialized);
        if (_accelBiasInitialized)
        {
            const float accelDeltaRightG = accelBodyG.x() - _accelBiasRightG;
            const float accelDeltaForwardG = accelBodyG.y() - _accelBiasForwardG;
            snapshot.SetBodyRightAccelerationMps2(GRAVITY_MPS2 * accelDeltaRightG);
            snapshot.SetBodyForwardAccelerationMps2(GRAVITY_MPS2 * accelDeltaForwardG);
            snapshot.SetPlanarAccelerationMps2(
                GRAVITY_MPS2 * MazeMap::Math::Sqrtf(
                    (accelDeltaRightG * accelDeltaRightG) + (accelDeltaForwardG * accelDeltaForwardG)));
        }
        else
        {
            snapshot.SetBodyRightAccelerationMps2(0.0f);
            snapshot.SetBodyForwardAccelerationMps2(0.0f);
            snapshot.SetPlanarAccelerationMps2(0.0f);
        }

        snapshot.SetRawYawRateRadps(rawYawRateRadps);
        snapshot.SetYawRateBiasRadps(_gyroBiasRadps);
        snapshot.SetYawRateRadps(rawYawRateRadps - _gyroBiasRadps);
    }

    void Imu::ResetCalibrationSampling() noexcept
    {
        _calibrationCollectedSamples = 0UL;
        _calibrationAccelMgSumX = 0.0;
        _calibrationAccelMgSumY = 0.0;
        _calibrationAccelMgSumZ = 0.0;
        _calibrationGyroDpsSumX = 0.0;
        _calibrationGyroDpsSumY = 0.0;
        _calibrationGyroDpsSumZ = 0.0;
        _calibrationGyroBiasRadpsSum = 0.0;
        _calibrationAccelBiasRightGSum = 0.0;
        _calibrationAccelBiasForwardGSum = 0.0;
        _baselineAccelMgX = 0.0f;
        _baselineAccelMgY = 0.0f;
        _baselineAccelMgZ = 0.0f;
        _baselineGyroDpsX = 0.0f;
        _baselineGyroDpsY = 0.0f;
        _baselineGyroDpsZ = 0.0f;
        _lastSelfTestAccelDeltaMg[0] = 0.0f;
        _lastSelfTestAccelDeltaMg[1] = 0.0f;
        _lastSelfTestAccelDeltaMg[2] = 0.0f;
        _lastSelfTestGyroDeltaDps[0] = 0.0f;
        _lastSelfTestGyroDeltaDps[1] = 0.0f;
        _lastSelfTestGyroDeltaDps[2] = 0.0f;
    }

    void Imu::BeginSelfTestSampling() noexcept
    {
        _calibrationCollectedSamples = 0UL;
        _calibrationAccelMgSumX = 0.0;
        _calibrationAccelMgSumY = 0.0;
        _calibrationAccelMgSumZ = 0.0;
        _calibrationGyroDpsSumX = 0.0;
        _calibrationGyroDpsSumY = 0.0;
        _calibrationGyroDpsSumZ = 0.0;
    }

    void Imu::AccumulateSelfTestSample() noexcept
    {
        const float accelMgPerLsb = _driver.AccelSensitivityMgPerLsb();
        const float gyroDpsPerLsb = _driver.GyroSensitivityMdpsPerLsb() / 1000.0f;
        _calibrationAccelMgSumX += static_cast<double>(_driver.ReadAccelX()) * accelMgPerLsb;
        _calibrationAccelMgSumY += static_cast<double>(_driver.ReadAccelY()) * accelMgPerLsb;
        _calibrationAccelMgSumZ += static_cast<double>(_driver.ReadAccelZ()) * accelMgPerLsb;
        _calibrationGyroDpsSumX += static_cast<double>(_driver.ReadGyroX()) * gyroDpsPerLsb;
        _calibrationGyroDpsSumY += static_cast<double>(_driver.ReadGyroY()) * gyroDpsPerLsb;
        _calibrationGyroDpsSumZ += static_cast<double>(_driver.ReadGyroZ()) * gyroDpsPerLsb;
        ++_calibrationCollectedSamples;
    }

    void Imu::StoreCurrentSelfTestAverageAsBaseline() noexcept
    {
        if (_calibrationCollectedSamples == 0UL)
        {
            _baselineAccelMgX = 0.0f;
            _baselineAccelMgY = 0.0f;
            _baselineAccelMgZ = 0.0f;
            _baselineGyroDpsX = 0.0f;
            _baselineGyroDpsY = 0.0f;
            _baselineGyroDpsZ = 0.0f;
            return;
        }

        const double normalization = 1.0 / static_cast<double>(_calibrationCollectedSamples);
        _baselineAccelMgX = static_cast<float>(_calibrationAccelMgSumX * normalization);
        _baselineAccelMgY = static_cast<float>(_calibrationAccelMgSumY * normalization);
        _baselineAccelMgZ = static_cast<float>(_calibrationAccelMgSumZ * normalization);
        _baselineGyroDpsX = static_cast<float>(_calibrationGyroDpsSumX * normalization);
        _baselineGyroDpsY = static_cast<float>(_calibrationGyroDpsSumY * normalization);
        _baselineGyroDpsZ = static_cast<float>(_calibrationGyroDpsSumZ * normalization);
    }

    bool Imu::ValidateStimulatedSelfTestAverage() noexcept
    {
        if (_calibrationCollectedSamples == 0UL)
        {
            return false;
        }

        const double normalization = 1.0 / static_cast<double>(_calibrationCollectedSamples);
        const float stimulatedAccelMgX = static_cast<float>(_calibrationAccelMgSumX * normalization);
        const float stimulatedAccelMgY = static_cast<float>(_calibrationAccelMgSumY * normalization);
        const float stimulatedAccelMgZ = static_cast<float>(_calibrationAccelMgSumZ * normalization);
        const float stimulatedGyroDpsX = static_cast<float>(_calibrationGyroDpsSumX * normalization);
        const float stimulatedGyroDpsY = static_cast<float>(_calibrationGyroDpsSumY * normalization);
        const float stimulatedGyroDpsZ = static_cast<float>(_calibrationGyroDpsSumZ * normalization);
        _lastSelfTestAccelDeltaMg[0] = std::fabs(stimulatedAccelMgX - _baselineAccelMgX);
        _lastSelfTestAccelDeltaMg[1] = std::fabs(stimulatedAccelMgY - _baselineAccelMgY);
        _lastSelfTestAccelDeltaMg[2] = std::fabs(stimulatedAccelMgZ - _baselineAccelMgZ);
        _lastSelfTestGyroDeltaDps[0] = std::fabs(stimulatedGyroDpsX - _baselineGyroDpsX);
        _lastSelfTestGyroDeltaDps[1] = std::fabs(stimulatedGyroDpsY - _baselineGyroDpsY);
        _lastSelfTestGyroDeltaDps[2] = std::fabs(stimulatedGyroDpsZ - _baselineGyroDpsZ);
        return SelfTestDeltasValid(
            _lastSelfTestAccelDeltaMg[0],
            _lastSelfTestAccelDeltaMg[1],
            _lastSelfTestAccelDeltaMg[2],
            _lastSelfTestGyroDeltaDps[0],
            _lastSelfTestGyroDeltaDps[1],
            _lastSelfTestGyroDeltaDps[2]);
    }

    void Imu::BeginStationaryBiasSampling() noexcept
    {
        _calibrationCollectedSamples = 0UL;
        _calibrationGyroBiasRadpsSum = 0.0;
        _calibrationAccelBiasRightGSum = 0.0;
        _calibrationAccelBiasForwardGSum = 0.0;
    }

    void Imu::AccumulateStationaryBiasSample() noexcept
    {
        const Eigen::Vector2f accelBodyG = _driver.AccelRawToBodyPlanarG(
            _driver.ReadAccelX(),
            _driver.ReadAccelY());
        _calibrationAccelBiasRightGSum += static_cast<double>(accelBodyG.x());
        _calibrationAccelBiasForwardGSum += static_cast<double>(accelBodyG.y());
        _calibrationGyroBiasRadpsSum += static_cast<double>(_driver.ReadBodyYawRateRadps());
        ++_calibrationCollectedSamples;
    }

    bool Imu::CompleteStationaryBiasSampling() noexcept
    {
        if (_calibrationCollectedSamples == 0UL)
        {
            return false;
        }

        const double normalization = 1.0 / static_cast<double>(_calibrationCollectedSamples);
        SetRuntimeCalibration(
            static_cast<float>(_calibrationGyroBiasRadpsSum * normalization),
            true,
            static_cast<float>(_calibrationAccelBiasRightGSum * normalization),
            static_cast<float>(_calibrationAccelBiasForwardGSum * normalization));
        return true;
    }

    unsigned long Imu::CalibrationCollectedSamples() const noexcept
    {
        return _calibrationCollectedSamples;
    }

    float Imu::LastSelfTestAccelDeltaMg(const uint8_t axis) const noexcept
    {
        return (axis < 3U) ? _lastSelfTestAccelDeltaMg[axis] : 0.0f;
    }

    float Imu::LastSelfTestGyroDeltaDps(const uint8_t axis) const noexcept
    {
        return (axis < 3U) ? _lastSelfTestGyroDeltaDps[axis] : 0.0f;
    }

    bool Imu::SelfTestDeltasValid(
        const float accelDeltaMgX,
        const float accelDeltaMgY,
        const float accelDeltaMgZ,
        const float gyroDeltaDpsX,
        const float gyroDeltaDpsY,
        const float gyroDeltaDpsZ) const noexcept
    {
        const float gyroFullScaleDps = _driver.GyroFullScaleDps();
        return IsAccelSelfTestDeltaValidMg(accelDeltaMgX) &&
            IsAccelSelfTestDeltaValidMg(accelDeltaMgY) &&
            IsAccelSelfTestDeltaValidMg(accelDeltaMgZ) &&
            IsGyroSelfTestDeltaValidDps(gyroDeltaDpsX, gyroFullScaleDps) &&
            IsGyroSelfTestDeltaValidDps(gyroDeltaDpsY, gyroFullScaleDps) &&
            IsGyroSelfTestDeltaValidDps(gyroDeltaDpsZ, gyroFullScaleDps);
    }

    bool Imu::IsAccelSelfTestDeltaValidMg(const float deltaMg) noexcept
    {
        const float absoluteDeltaMg = std::fabs(deltaMg);
        return std::isfinite(absoluteDeltaMg) &&
            (absoluteDeltaMg >= 50.0f) &&
            (absoluteDeltaMg <= 1700.0f);
    }

    bool Imu::IsGyroSelfTestDeltaValidDps(const float deltaDps, const float fullScaleDps) noexcept
    {
        const float absoluteDeltaDps = std::fabs(deltaDps);
        if (!std::isfinite(absoluteDeltaDps) || !std::isfinite(fullScaleDps))
        {
            return false;
        }

        if (std::fabs(fullScaleDps - 250.0f) < 0.5f)
        {
            return (absoluteDeltaDps >= 20.0f) && (absoluteDeltaDps <= 80.0f);
        }

        if (std::fabs(fullScaleDps - 2000.0f) < 0.5f)
        {
            return (absoluteDeltaDps >= 150.0f) && (absoluteDeltaDps <= 700.0f);
        }

        return false;
    }

    unsigned long Imu::GetUiImuSampleRateHzForControlPeriodUs(const unsigned long controlPeriodUs) noexcept
    {
        switch (controlPeriodUs)
        {
        case 1000UL:
            return 1000UL;
        case 500UL:
            return 2000UL;
        default:
            return 0UL;
        }
    }

    float Imu::GetUiAccelLpf2CutoffHzForControlPeriodUs(const unsigned long controlPeriodUs) noexcept
    {
        return GetUiAccelLpf2CutoffHzForControlPeriodUs(controlPeriodUs, AccelFilterFreq::Frac1Over400);
    }

    float Imu::GetUiAccelLpf2CutoffHzForControlPeriodUs(
        const unsigned long controlPeriodUs,
        const AccelFilterFreq filterFreq) noexcept
    {
        const unsigned long sampleRateHz = GetUiImuSampleRateHzForControlPeriodUs(controlPeriodUs);
        if (sampleRateHz == 0UL)
        {
            return 0.0f;
        }

        switch (static_cast<uint8_t>(filterFreq) & 0xF0U)
        {
        case 0x10U:
            return static_cast<float>(sampleRateHz) / 4.0f;
        case 0x30U:
            return static_cast<float>(sampleRateHz) / 10.0f;
        case 0x50U:
            return static_cast<float>(sampleRateHz) / 20.0f;
        case 0x70U:
            return static_cast<float>(sampleRateHz) / 45.0f;
        case 0x90U:
            return static_cast<float>(sampleRateHz) / 100.0f;
        case 0xB0U:
            return static_cast<float>(sampleRateHz) / 200.0f;
        case 0xD0U:
            return static_cast<float>(sampleRateHz) / 400.0f;
        case 0xF0U:
            return static_cast<float>(sampleRateHz) / 800.0f;
        case 0x00U:
        default:
            return 0.0f;
        }
    }

    float Imu::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(const unsigned long controlPeriodUs) noexcept
    {
        switch (controlPeriodUs)
        {
        case 1000UL:
            return 195.0f;
        case 500UL:
            return 210.0f;
        default:
            return 0.0f;
        }
    }

    unsigned long Imu::ComputeRequiredStationaryBiasSamples(
        const unsigned long configuredSamples,
        const unsigned long controlPeriodUs,
        const uint32_t sampleIntervalTicks,
        const unsigned long minimumWindowMs) noexcept
    {
        const unsigned long sampleIntervalUs = sampleIntervalTicks * controlPeriodUs;
        if (sampleIntervalUs == 0UL)
        {
            return configuredSamples;
        }

        const unsigned long minimumWindowUs = minimumWindowMs * 1000UL;
        const unsigned long minimumSamples = (minimumWindowUs + sampleIntervalUs - 1UL) / sampleIntervalUs;
        return (configuredSamples >= minimumSamples) ? configuredSamples : minimumSamples;
    }

    LSM6DSV16X_IMU::ACCEL_FILTER_FREQ Imu::ToDriverAccelFilterFreq(const AccelFilterFreq filterFreq) noexcept
    {
        return static_cast<LSM6DSV16X_IMU::ACCEL_FILTER_FREQ>(static_cast<uint8_t>(filterFreq));
    }
}
