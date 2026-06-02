#include "pch.h"
#include "LSM6DSV16X_IMU.h"

#if defined(ARDUINO_TEENSY41)
#include <SPI.h>

#ifdef DISABLE
#undef DISABLE
#endif
#endif

namespace MazeMap
{
#if defined(ARDUINO_TEENSY41)
    namespace
    {
        SPISettings BuildLsm6Dsv16xSpiSettings()
        {
            return SPISettings(LSM6DSV16X_IMU::kMaxSpiClockHz, MSBFIRST, SPI_MODE3);
        }
    }
#endif

    LSM6DSV16X_IMU::LSM6DSV16X_IMU(
        const uint8_t csPin,
        const uint8_t intPin,
        const uint8_t mosiPin,
        const uint8_t misoPin,
        const uint8_t clockPin,
        const MazeMap::SensorMount& mount) noexcept
        : _csPin(csPin)
        , _intPin(intPin)
        , _mosiPin(mosiPin)
        , _misoPin(misoPin)
        , _clockPin(clockPin)
        , _mount(mount)
    {
    }

    bool LSM6DSV16X_IMU::Begin()
    {
        _lastBeginFailureReason = BeginFailureReason::None;
        _lastWhoAmI = 0U;

        pinMode(_csPin, OUTPUT);
        digitalWrite(_csPin, HIGH);
        pinMode(_intPin, INPUT);

#if defined(ARDUINO_TEENSY41)
        SPI.setMOSI(_mosiPin);
        SPI.setMISO(_misoPin);
        SPI.setSCK(_clockPin);
        SPI.begin();

        if (!Reset())
        {
            _lastBeginFailureReason = BeginFailureReason::ResetTimeout;
            _lastWhoAmI = ReadWhoAmI();
            return false;
        }

        WriteRegister(CTRL3_RW_ADR, static_cast<uint8_t>(CTRL3_BDU | CTRL3_IF_INC));
        _lastWhoAmI = ReadWhoAmI();
        if (_lastWhoAmI != WHO_AM_I_VALUE)
        {
            _lastBeginFailureReason = BeginFailureReason::WhoAmIMismatch;
            return false;
        }
#else
        (void)_mosiPin;
        (void)_misoPin;
        (void)_clockPin;
#endif

        return true;
    }

    bool LSM6DSV16X_IMU::Reset(const uint32_t timeoutMs)
    {
#if defined(ARDUINO_TEENSY41)
        WriteRegister(CTRL3_RW_ADR, CTRL3_SW_RESET);

        const uint32_t startMs = millis();
        while ((ReadRegister(CTRL3_RW_ADR) & CTRL3_SW_RESET) != 0U)
        {
            if ((millis() - startMs) >= timeoutMs)
            {
                return false;
            }

            delay(1);
        }
#else
        (void)timeoutMs;
#endif

        return true;
    }

    uint8_t LSM6DSV16X_IMU::ReadWhoAmI()
    {
        return ReadRegister(WHO_AM_I_R_ADR);
    }

    uint8_t LSM6DSV16X_IMU::ReadWhoAmIWithSettings(const uint32_t clockHz, const uint8_t dataMode)
    {
        return ReadRegisterWithSettings(WHO_AM_I_R_ADR, clockHz, dataMode);
    }

    bool LSM6DSV16X_IMU::IsConnected()
    {
        return ReadWhoAmI() == WHO_AM_I_VALUE;
    }

    LSM6DSV16X_IMU::BeginFailureReason LSM6DSV16X_IMU::GetLastBeginFailureReason() const noexcept
    {
        return _lastBeginFailureReason;
    }

    const char* LSM6DSV16X_IMU::GetLastBeginFailureReasonName() const noexcept
    {
        switch (_lastBeginFailureReason)
        {
        case BeginFailureReason::None:
            return "none";
        case BeginFailureReason::ResetTimeout:
            return "reset-timeout";
        case BeginFailureReason::WhoAmIMismatch:
            return "whoami-mismatch";
        default:
            return "unknown";
        }
    }

    uint8_t LSM6DSV16X_IMU::GetLastWhoAmI() const noexcept
    {
        return _lastWhoAmI;
    }

    void LSM6DSV16X_IMU::WriteRegister(const uint8_t address, const uint8_t data)
    {
#if defined(ARDUINO_TEENSY41)
        SPI.beginTransaction(BuildLsm6Dsv16xSpiSettings());
        Select();
        SPI.transfer(address & kAddressMask);
        SPI.transfer(data);
        Deselect();
        SPI.endTransaction();
#else
        (void)address;
        (void)data;
#endif
    }

    uint8_t LSM6DSV16X_IMU::ReadRegister(const uint8_t address)
    {
#if defined(ARDUINO_TEENSY41)
        SPI.beginTransaction(BuildLsm6Dsv16xSpiSettings());
        Select();
        SPI.transfer(address | kReadMask);
        const uint8_t value = SPI.transfer(0x00);
        Deselect();
        SPI.endTransaction();
        return value;
#else
        (void)address;
        return 0U;
#endif
    }

    uint8_t LSM6DSV16X_IMU::ReadRegisterWithSettings(
        const uint8_t address,
        const uint32_t clockHz,
        const uint8_t dataMode)
    {
#if defined(ARDUINO_TEENSY41)
        SPI.beginTransaction(SPISettings(clockHz, MSBFIRST, dataMode));
        Select();
        SPI.transfer(address | kReadMask);
        const uint8_t value = SPI.transfer(0x00);
        Deselect();
        SPI.endTransaction();
        return value;
#else
        (void)clockHz;
        (void)dataMode;
        return ReadRegister(address);
#endif
    }

    void LSM6DSV16X_IMU::WriteRegisters(const uint8_t startAddress, const uint8_t* const data, const size_t length)
    {
#if defined(ARDUINO_TEENSY41)
        SPI.beginTransaction(BuildLsm6Dsv16xSpiSettings());
        Select();
        SPI.transfer(startAddress & kAddressMask);

        for (size_t i = 0; i < length; ++i)
        {
            SPI.transfer(data[i]);
        }

        Deselect();
        SPI.endTransaction();
#else
        (void)startAddress;
        (void)data;
        (void)length;
#endif
    }

    void LSM6DSV16X_IMU::ReadRegisters(const uint8_t startAddress, uint8_t* const data, const size_t length)
    {
#if defined(ARDUINO_TEENSY41)
        SPI.beginTransaction(BuildLsm6Dsv16xSpiSettings());
        Select();
        SPI.transfer(startAddress | kReadMask);

        for (size_t i = 0; i < length; ++i)
        {
            data[i] = SPI.transfer(0x00);
        }

        Deselect();
        SPI.endTransaction();
#else
        (void)startAddress;
        if (data == nullptr)
        {
            return;
        }
        for (size_t i = 0; i < length; ++i)
        {
            data[i] = 0U;
        }
#endif
    }

    void LSM6DSV16X_IMU::WriteLargeRegister(const uint8_t address, const uint16_t data)
    {
        const uint8_t bytes[2] =
        {
            static_cast<uint8_t>(data & 0xFFU),
            static_cast<uint8_t>((data >> 8) & 0xFFU),
        };

        WriteRegisters(address, bytes, sizeof(bytes));
    }

    uint16_t LSM6DSV16X_IMU::ReadLargeRegister(const uint8_t address)
    {
        uint8_t bytes[2] = {};
        ReadRegisters(address, bytes, sizeof(bytes));
        return static_cast<uint16_t>(bytes[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8U);
    }

    void LSM6DSV16X_IMU::SetAccelMode(const ACCEL_MODE mode, const ODR_SETTING odr)
    {
        WriteRegister(CTRL1_RW_ADR, ToU8(mode) | ToU8(odr));
    }

    void LSM6DSV16X_IMU::ConfigureUiHighAccuracyOdr(
        const HAODR_SELECTION selection,
        const ODR_SETTING accelOdr,
        const ODR_SETTING gyroOdr)
    {
#if defined(ARDUINO_TEENSY41)
        WriteRegister(CTRL1_RW_ADR, ToU8(ACCEL_MODE::HI_ACC) | ToU8(ODR_SETTING::DISABLE));
        WriteRegister(CTRL2_RW_ADR, ToU8(GYRO_MODE::HI_ACC) | ToU8(ODR_SETTING::DISABLE));
        delayMicroseconds(500U);

        uint8_t haodrCfg = ReadRegister(HAODR_CFG_RW_ADR);
        haodrCfg &= static_cast<uint8_t>(~HAODR_CFG_HAODR_SEL_MASK);
        haodrCfg |= (ToU8(selection) & HAODR_CFG_HAODR_SEL_MASK);
        WriteRegister(HAODR_CFG_RW_ADR, haodrCfg);
        delayMicroseconds(500U);

        WriteRegister(CTRL2_RW_ADR, ToU8(GYRO_MODE::HI_ACC) | ToU8(gyroOdr));
        WriteRegister(CTRL1_RW_ADR, ToU8(ACCEL_MODE::HI_ACC) | ToU8(accelOdr));
#else
        (void)selection;
        (void)accelOdr;
        (void)gyroOdr;
#endif
    }

    void LSM6DSV16X_IMU::SetAccelRange(const ACCEL_FILTER_FREQ freq, const ACCEL_FULLSCALE scale)
    {
        _accelScale = scale;

#if defined(ARDUINO_TEENSY41)
        const uint8_t freqBits = ToU8(freq) & CTRL8_HP_LPF2_XL_BW_MASK;
        WriteRegister(CTRL8_RW_ADR, freqBits | ToU8(scale));

        uint8_t ctrl9 = ReadRegister(CTRL9_RW_ADR);
        ctrl9 &= static_cast<uint8_t>(~(CTRL9_HP_SLOPE_XL_EN | CTRL9_LPF2_XL_EN));

        if (freq != ACCEL_FILTER_FREQ::FRAC_1_002)
        {
            ctrl9 |= CTRL9_LPF2_XL_EN;
        }

        WriteRegister(CTRL9_RW_ADR, ctrl9);
#else
        (void)freq;
#endif
    }

    void LSM6DSV16X_IMU::SetGyroMode(const GYRO_MODE mode, const ODR_SETTING odr)
    {
        WriteRegister(CTRL2_RW_ADR, ToU8(mode) | ToU8(odr));
    }

    void LSM6DSV16X_IMU::SetGyroRange(const GYRO_LPF1_MODE lpf1, const GYRO_FULLSCALE_RANGE range)
    {
        _gyroScale = range;
#if defined(ARDUINO_TEENSY41)
        WriteRegister(CTRL6_RW_ADR, ToU8(lpf1) | ToU8(range));
#else
        (void)lpf1;
#endif
    }

    void LSM6DSV16X_IMU::SetSelfTest(const SELF_TEST_MODE gyroMode, const SELF_TEST_MODE accelMode)
    {
#if defined(ARDUINO_TEENSY41)
        const uint8_t gyroBits = static_cast<uint8_t>((ToU8(gyroMode) & 0x03U) << 2U);
        const uint8_t accelBits = static_cast<uint8_t>(ToU8(accelMode) & 0x03U);
        WriteRegister(CTRL10_RW_ADR, gyroBits | accelBits);
#else
        (void)gyroMode;
        (void)accelMode;
#endif
    }

    LSM6DSV16X_IMU::StatusReg LSM6DSV16X_IMU::ReadStatus()
    {
        return StatusReg(ReadRegister(STATUS_REG_R_ADR));
    }

    ImuTelemetry LSM6DSV16X_IMU::CaptureTelemetry(ImuObservationTiming* const timing)
    {
        ImuTelemetry telemetry{};
        const uint32_t readStartUs = micros();
        if (timing != nullptr)
        {
            timing->readStartUs = readStartUs;
            timing->drdyUs = (digitalRead(_intPin) == HIGH) ? readStartUs : 0UL;
        }

        const StatusReg status = ReadStatus();
        telemetry.status = status.Raw();
        telemetry.gyroX = ReadGyroX();
        telemetry.gyroY = ReadGyroY();
        telemetry.gyroZ = ReadGyroZ();
        telemetry.accelX = ReadAccelX();
        telemetry.accelY = ReadAccelY();
        telemetry.accelZ = ReadAccelZ();
        telemetry.temp = ReadTemp();
        telemetry.interruptHigh = (digitalRead(_intPin) == HIGH);
        if (timing != nullptr)
        {
            timing->readDoneUs = micros();
        }
        return telemetry;
    }

    int16_t LSM6DSV16X_IMU::ReadGyroX()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTX_L_G_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadGyroY()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTY_L_G_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadGyroZ()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTZ_L_G_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadAccelX()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTX_L_A_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadAccelY()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTY_L_A_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadAccelZ()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUTZ_L_A_R_ADR));
    }

    int16_t LSM6DSV16X_IMU::ReadTemp()
    {
        return static_cast<int16_t>(ReadLargeRegister(OUT_TEMP_L_R_ADR));
    }

    float LSM6DSV16X_IMU::ReadTempC()
    {
        return 25.0f + (static_cast<float>(ReadTemp()) / 256.0f);
    }

    float LSM6DSV16X_IMU::AccelSensitivityMgPerLsb() const noexcept
    {
        switch (_accelScale)
        {
        case ACCEL_FULLSCALE::G2:
            return 0.061f;
        case ACCEL_FULLSCALE::G4:
            return 0.122f;
        case ACCEL_FULLSCALE::G8:
            return 0.244f;
        case ACCEL_FULLSCALE::G16:
        default:
            return 0.488f;
        }
    }

    float LSM6DSV16X_IMU::GyroSensitivityMdpsPerLsb() const noexcept
    {
        switch (_gyroScale)
        {
        case GYRO_FULLSCALE_RANGE::DPS0125:
            return 4.375f;
        case GYRO_FULLSCALE_RANGE::DPS0250:
            return 8.75f;
        case GYRO_FULLSCALE_RANGE::DPS0500:
            return 17.5f;
        case GYRO_FULLSCALE_RANGE::DPS1000:
            return 35.0f;
        case GYRO_FULLSCALE_RANGE::DPS2000:
            return 70.0f;
        case GYRO_FULLSCALE_RANGE::DPS4000:
        default:
            return 140.0f;
        }
    }

    float LSM6DSV16X_IMU::GyroFullScaleDps() const noexcept
    {
        switch (_gyroScale)
        {
        case GYRO_FULLSCALE_RANGE::DPS0125:
            return 125.0f;
        case GYRO_FULLSCALE_RANGE::DPS0250:
            return 250.0f;
        case GYRO_FULLSCALE_RANGE::DPS0500:
            return 500.0f;
        case GYRO_FULLSCALE_RANGE::DPS1000:
            return 1000.0f;
        case GYRO_FULLSCALE_RANGE::DPS2000:
            return 2000.0f;
        case GYRO_FULLSCALE_RANGE::DPS4000:
        default:
            return 4000.0f;
        }
    }

    float LSM6DSV16X_IMU::AccelRawToG(const int16_t raw) const noexcept
    {
        return (static_cast<float>(raw) * AccelSensitivityMgPerLsb()) / 1000.0f;
    }

    float LSM6DSV16X_IMU::GyroRawToDps(const int16_t raw) const noexcept
    {
        return (static_cast<float>(raw) * GyroSensitivityMdpsPerLsb()) / 1000.0f;
    }

    float LSM6DSV16X_IMU::GyroRawToClockwiseYawDps(const int16_t raw) const noexcept
    {
        return ClockwiseYawFromSensorZSign() * GyroRawToDps(raw);
    }

    float LSM6DSV16X_IMU::GyroRawToBodyYawRadps(const int16_t raw) const noexcept
    {
        return _mount.TransformClockwiseYawRateToBody(GyroRawToClockwiseYawDps(raw) * DEG_TO_RAD_F);
    }

    float LSM6DSV16X_IMU::ReadClockwiseYawDps()
    {
        return GyroRawToClockwiseYawDps(ReadGyroZ());
    }

    float LSM6DSV16X_IMU::ReadBodyYawRateRadps()
    {
        return GyroRawToBodyYawRadps(ReadGyroZ());
    }

    Eigen::Vector2f LSM6DSV16X_IMU::AccelRawToBodyPlanarG(const int16_t rawX, const int16_t rawY) const noexcept
    {
        return _mount.TransformPlanarVectorToBody(
            Eigen::Vector2f(
                AccelRawToG(rawX),
                AccelRawToG(rawY)));
    }

    void LSM6DSV16X_IMU::Select() const
    {
        digitalWrite(_csPin, LOW);
    }

    void LSM6DSV16X_IMU::Deselect() const
    {
        digitalWrite(_csPin, HIGH);
    }
}
