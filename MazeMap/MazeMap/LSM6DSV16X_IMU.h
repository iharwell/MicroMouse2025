#pragma once

#include "Defines.h"
#include "SensorMount.h"
#include "SensorTelemetryTypes.h"

#include <cstddef>
#include <cstdint>

#ifdef DISABLE
#undef DISABLE
#endif

namespace MazeMap
{
    class EXPORT LSM6DSV16X_IMU final
    {
    public:
        static constexpr uint8_t WHO_AM_I_VALUE = 0x70;
        static constexpr uint32_t kMaxSpiClockHz = 10000000UL;

        enum class BeginFailureReason : uint8_t
        {
            None = 0U,
            ResetTimeout,
            WhoAmIMismatch,
        };

        static constexpr uint8_t FUNC_CFG_ACCESS_RW_ADR = 0x01;
        static constexpr uint8_t PIN_CTRL_RW_ADR = 0x02;
        static constexpr uint8_t IF_CFG_RW_ADR = 0x03;
        static constexpr uint8_t ODR_TRIG_CFG_RW_ADR = 0x06;
        static constexpr uint8_t FIFO_CTRL1_RW_ADR = 0x07;
        static constexpr uint8_t FIFO_CTRL2_RW_ADR = 0x08;
        static constexpr uint8_t FIFO_CTRL3_RW_ADR = 0x09;
        static constexpr uint8_t FIFO_CTRL4_RW_ADR = 0x0A;
        static constexpr uint8_t COUNTER_BDR_REG1_RW_ADR = 0x0B;
        static constexpr uint8_t COUNTER_BDR_REG2_RW_ADR = 0x0C;
        static constexpr uint8_t INT1_CTRL_RW_ADR = 0x0D;
        static constexpr uint8_t INT2_CTRL_RW_ADR = 0x0E;
        static constexpr uint8_t WHO_AM_I_R_ADR = 0x0F;
        static constexpr uint8_t CTRL1_RW_ADR = 0x10;
        static constexpr uint8_t CTRL2_RW_ADR = 0x11;
        static constexpr uint8_t CTRL3_RW_ADR = 0x12;
        static constexpr uint8_t CTRL4_RW_ADR = 0x13;
        static constexpr uint8_t CTRL5_RW_ADR = 0x14;
        static constexpr uint8_t CTRL6_RW_ADR = 0x15;
        static constexpr uint8_t CTRL7_RW_ADR = 0x16;
        static constexpr uint8_t CTRL8_RW_ADR = 0x17;
        static constexpr uint8_t CTRL9_RW_ADR = 0x18;
        static constexpr uint8_t CTRL10_RW_ADR = 0x19;
        static constexpr uint8_t CTRL_STATUS_R_ADR = 0x1A;
        static constexpr uint8_t FIFO_STATUS1_R_ADR = 0x1B;
        static constexpr uint8_t FIFO_STATUS2_R_ADR = 0x1C;
        static constexpr uint8_t ALL_INT_SRC_R_ADR = 0x1D;
        static constexpr uint8_t STATUS_REG_R_ADR = 0x1E;
        static constexpr uint8_t OUT_TEMP_L_R_ADR = 0x20;
        static constexpr uint8_t OUT_TEMP_H_R_ADR = 0x21;
        static constexpr uint8_t OUTX_L_G_R_ADR = 0x22;
        static constexpr uint8_t OUTX_H_G_R_ADR = 0x23;
        static constexpr uint8_t OUTY_L_G_R_ADR = 0x24;
        static constexpr uint8_t OUTY_H_G_R_ADR = 0x25;
        static constexpr uint8_t OUTZ_L_G_R_ADR = 0x26;
        static constexpr uint8_t OUTZ_H_G_R_ADR = 0x27;
        static constexpr uint8_t OUTX_L_A_R_ADR = 0x28;
        static constexpr uint8_t OUTX_H_A_R_ADR = 0x29;
        static constexpr uint8_t OUTY_L_A_R_ADR = 0x2A;
        static constexpr uint8_t OUTY_H_A_R_ADR = 0x2B;
        static constexpr uint8_t OUTZ_L_A_R_ADR = 0x2C;
        static constexpr uint8_t OUTZ_H_A_R_ADR = 0x2D;
        static constexpr uint8_t UI_OUTX_L_G_OIS_EIS_R_ADR = 0x2E;
        static constexpr uint8_t UI_OUTX_H_G_OIS_EIS_R_ADR = 0x2F;
        static constexpr uint8_t UI_OUTY_L_G_OIS_EIS_R_ADR = 0x30;
        static constexpr uint8_t UI_OUTY_H_G_OIS_EIS_R_ADR = 0x31;
        static constexpr uint8_t UI_OUTZ_L_G_OIS_EIS_R_ADR = 0x32;
        static constexpr uint8_t UI_OUTZ_H_G_OIS_EIS_R_ADR = 0x33;
        static constexpr uint8_t UI_OUTX_L_A_OIS_DualC_R_ADR = 0x34;
        static constexpr uint8_t UI_OUTX_H_A_OIS_DualC_R_ADR = 0x35;
        static constexpr uint8_t UI_OUTY_L_A_OIS_DualC_R_ADR = 0x36;
        static constexpr uint8_t UI_OUTY_H_A_OIS_DualC_R_ADR = 0x37;
        static constexpr uint8_t UI_OUTZ_L_A_OIS_DualC_R_ADR = 0x38;
        static constexpr uint8_t UI_OUTZ_H_A_OIS_DualC_R_ADR = 0x39;
        static constexpr uint8_t AH_QVAR_OUT_L_R_ADR = 0x3A;
        static constexpr uint8_t AH_QVAR_OUT_H_R_ADR = 0x3B;
        static constexpr uint8_t TIMESTAMP0_R_ADR = 0x40;
        static constexpr uint8_t TIMESTAMP1_R_ADR = 0x41;
        static constexpr uint8_t TIMESTAMP2_R_ADR = 0x42;
        static constexpr uint8_t TIMESTAMP3_R_ADR = 0x43;
        static constexpr uint8_t UI_STATUS_REG_OIS_R_ADR = 0x44;
        static constexpr uint8_t WAKE_UP_SRC_R_ADR = 0x45;
        static constexpr uint8_t TAP_SRC_R_ADR = 0x46;
        static constexpr uint8_t D6D_SRC_R_ADR = 0x47;
        static constexpr uint8_t STATUS_MASTER_MAINPAGE_R_ADR = 0x48;
        static constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_R_ADR = 0x49;
        static constexpr uint8_t FSM_STATUS_MAINPAGE_R_ADR = 0x4A;
        static constexpr uint8_t MLC_STATUS_MAINPAGE_R_ADR = 0x4B;
        static constexpr uint8_t INTERNAL_FREQ_FINE_R_ADR = 0x4F;
        static constexpr uint8_t FUNCTIONS_ENABLE_RW_ADR = 0x50;
        static constexpr uint8_t DEN_RW_ADR = 0x51;
        static constexpr uint8_t INACTIVITY_DUR_RW_ADR = 0x54;
        static constexpr uint8_t INACTIVITY_THS_RW_ADR = 0x55;
        static constexpr uint8_t TAP_CFG0_RW_ADR = 0x56;
        static constexpr uint8_t TAP_CFG1_RW_ADR = 0x57;
        static constexpr uint8_t TAP_CFG2_RW_ADR = 0x58;
        static constexpr uint8_t TAP_THS_6D_RW_ADR = 0x59;
        static constexpr uint8_t TAP_DUR_RW_ADR = 0x5A;
        static constexpr uint8_t WAKE_UP_THS_RW_ADR = 0x5B;
        static constexpr uint8_t WAKE_UP_DUR_RW_ADR = 0x5C;
        static constexpr uint8_t FREE_FALL_RW_ADR = 0x5D;
        static constexpr uint8_t MD1_CFG_RW_ADR = 0x5E;
        static constexpr uint8_t MD2_CFG_RW_ADR = 0x5F;
        static constexpr uint8_t HAODR_CFG_RW_ADR = 0x62;
        static constexpr uint8_t EMB_FUNC_CFG_RW_ADR = 0x63;
        static constexpr uint8_t UI_HANDSHAKE_CTRL_RW_ADR = 0x64;
        static constexpr uint8_t UI_SPI2_SHARED_0_RW_ADR = 0x65;
        static constexpr uint8_t UI_SPI2_SHARED_1_RW_ADR = 0x66;
        static constexpr uint8_t UI_SPI2_SHARED_2_RW_ADR = 0x67;
        static constexpr uint8_t UI_SPI2_SHARED_3_RW_ADR = 0x68;
        static constexpr uint8_t UI_SPI2_SHARED_4_RW_ADR = 0x69;
        static constexpr uint8_t UI_SPI2_SHARED_5_RW_ADR = 0x6A;
        static constexpr uint8_t CTRL_EIS_RW_ADR = 0x6B;
        static constexpr uint8_t UI_INT_OIS_RW_ADR = 0x6F;
        static constexpr uint8_t UI_CTRL1_OIS_RW_ADR = 0x70;
        static constexpr uint8_t UI_CTRL2_OIS_RW_ADR = 0x71;
        static constexpr uint8_t UI_CTRL3_OIS_RW_ADR = 0x72;
        static constexpr uint8_t X_OFS_USR_RW_ADR = 0x73;
        static constexpr uint8_t Y_OFS_USR_RW_ADR = 0x74;
        static constexpr uint8_t Z_OFS_USR_RW_ADR = 0x75;
        static constexpr uint8_t FIFO_DATA_OUT_TAG_R_ADR = 0x78;
        static constexpr uint8_t FIFO_DATA_OUT_X_L_R_ADR = 0x79;
        static constexpr uint8_t FIFO_DATA_OUT_X_H_R_ADR = 0x7A;
        static constexpr uint8_t FIFO_DATA_OUT_Y_L_R_ADR = 0x7B;
        static constexpr uint8_t FIFO_DATA_OUT_Y_H_R_ADR = 0x7C;
        static constexpr uint8_t FIFO_DATA_OUT_Z_L_R_ADR = 0x7D;
        static constexpr uint8_t FIFO_DATA_OUT_Z_H_R_ADR = 0x7E;

        static constexpr uint8_t UI_INT_OIS__ADR = UI_INT_OIS_RW_ADR;
        static constexpr uint8_t UI_CTRL1_OIS__ADR = UI_CTRL1_OIS_RW_ADR;
        static constexpr uint8_t UI_CTRL2_OIS__ADR = UI_CTRL2_OIS_RW_ADR;
        static constexpr uint8_t UI_CTRL3_OIS__ADR = UI_CTRL3_OIS_RW_ADR;

        enum class ACCEL_MODE : uint8_t
        {
            DISABLE = 0x00,
            HI_PERF = 0x00,
            HI_ACC = 0x10,
            ODR_TRIG = 0x30,
            LP_1 = 0x40,
            LP_2 = 0x50,
            LP_3 = 0x60,
            NORMAL = 0x70,
        };

        enum class ODR_SETTING : uint8_t
        {
            DISABLE = 0x00,
            ODR_0002HZ_LP = 0x01,
            ODR_0007HZ_HP_N = 0x02,
            ODR_0015HZ_HP_N_LP = 0x03,
            ODR_0030HZ_HP_N_LP = 0x04,
            ODR_0060HZ_HP_N_LP = 0x05,
            ODR_0120HZ_HP_N_LP = 0x06,
            ODR_0240HZ_HP_N_LP = 0x07,
            ODR_0480HZ_HP_N_LP = 0x08,
            ODR_0960HZ_HP_N_LP = 0x09,
            ODR_1920HZ_HP_N_LP = 0x0A,
            ODR_3840HZ_HP_N_LP = 0x0B,
            ODR_7680HZ_HP_N_LP = 0x0C,
        };

        enum class HAODR_SELECTION : uint8_t
        {
            NATIVE = 0x00,
            EXACT_1000_2000_4000_8000 = 0x01,
            EXACT_800_1600_3200_6400 = 0x02,
        };

        enum class ACCEL_FILTER_FREQ : uint8_t
        {
            FRAC_1_002 = 0x00,
            FRAC_1_004 = 0x10,
            FRAC_1_010 = 0x30,
            FRAC_1_020 = 0x50,
            FRAC_1_045 = 0x70,
            FRAC_1_100 = 0x90,
            FRAC_1_200 = 0xB0,
            FRAC_1_400 = 0xD0,
            FRAC_1_800 = 0xF0,
        };

        enum class ACCEL_FULLSCALE : uint8_t
        {
            G2 = 0x00,
            G4 = 0x01,
            G8 = 0x02,
            G16 = 0x03,
        };

        enum class GYRO_LPF1_MODE : uint8_t
        {
            CUT_280 = 0x00,
            CUT_213 = 0x10,
            CUT_156 = 0x20,
            CUT_400 = 0x30,
            CUT_102 = 0x40,
            CUT_058 = 0x50,
            CUT_029 = 0x60,
            CUT_015 = 0x70,
        };

        enum class GYRO_MODE : uint8_t
        {
            DISABLE = 0x00,
            HI_PERF = 0x00,
            HI_ACC = 0x10,
            ODR_TRIG = 0x30,
            SLEEP_MODE = 0x40,
            LP_MODE = 0x50,
        };

        enum class GYRO_FULLSCALE_RANGE : uint8_t
        {
            DPS0125 = 0x00,
            DPS0250 = 0x01,
            DPS0500 = 0x02,
            DPS1000 = 0x03,
            DPS2000 = 0x04,
            DPS4000 = 0x0C,
        };

        enum class SELF_TEST_MODE : uint8_t
        {
            DISABLED = 0x00,
            POSITIVE = 0x01,
            NEGATIVE = 0x02,
        };

        class StatusReg
        {
        public:
            constexpr explicit StatusReg(uint8_t data = 0U) : data_(data) {}

            constexpr uint8_t Raw() const { return data_; }
            constexpr bool HasTimestampOverflow() const { return (data_ & STATUS_TIMESTAMP_ENDCOUNT) != 0U; }
            constexpr bool HasOisData() const { return (data_ & STATUS_OIS_DRDY) != 0U; }
            constexpr bool HasEisGyroData() const { return (data_ & STATUS_GDA_EIS) != 0U; }
            constexpr bool HasAhQvarData() const { return (data_ & STATUS_AH_QVARDA) != 0U; }
            constexpr bool HasTemperatureData() const { return (data_ & STATUS_TDA) != 0U; }
            constexpr bool HasGyroData() const { return (data_ & STATUS_GDA) != 0U; }
            constexpr bool HasAccelData() const { return (data_ & STATUS_XLDA) != 0U; }

        private:
            static constexpr uint8_t STATUS_TIMESTAMP_ENDCOUNT = 0x80;
            static constexpr uint8_t STATUS_OIS_DRDY = 0x20;
            static constexpr uint8_t STATUS_GDA_EIS = 0x10;
            static constexpr uint8_t STATUS_AH_QVARDA = 0x08;
            static constexpr uint8_t STATUS_TDA = 0x04;
            static constexpr uint8_t STATUS_GDA = 0x02;
            static constexpr uint8_t STATUS_XLDA = 0x01;

            uint8_t data_;
        };

        LSM6DSV16X_IMU(
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
        BeginFailureReason GetLastBeginFailureReason() const noexcept;
        const char* GetLastBeginFailureReasonName() const noexcept;
        uint8_t GetLastWhoAmI() const noexcept;

        void WriteRegister(uint8_t address, uint8_t data);
        uint8_t ReadRegister(uint8_t address);
        uint8_t ReadRegisterWithSettings(uint8_t address, uint32_t clockHz, uint8_t dataMode);
        void WriteRegisters(uint8_t startAddress, const uint8_t* data, size_t length);
        void ReadRegisters(uint8_t startAddress, uint8_t* data, size_t length);
        void WriteLargeRegister(uint8_t address, uint16_t data);
        uint16_t ReadLargeRegister(uint8_t address);

        void SetAccelMode(ACCEL_MODE mode, ODR_SETTING odr);
        void ConfigureUiHighAccuracyOdr(HAODR_SELECTION selection, ODR_SETTING accelOdr, ODR_SETTING gyroOdr);
        void SetAccelRange(ACCEL_FILTER_FREQ freq, ACCEL_FULLSCALE scale);
        void SetGyroMode(GYRO_MODE mode, ODR_SETTING odr);
        void SetGyroRange(GYRO_LPF1_MODE lpf1, GYRO_FULLSCALE_RANGE range);
        void SetSelfTest(SELF_TEST_MODE gyroMode, SELF_TEST_MODE accelMode);

        StatusReg ReadStatus();
        ImuTelemetry CaptureTelemetry(ImuObservationTiming* timing = nullptr);

        int16_t ReadGyroX();
        int16_t ReadGyroY();
        int16_t ReadGyroZ();
        int16_t ReadAccelX();
        int16_t ReadAccelY();
        int16_t ReadAccelZ();
        int16_t ReadTemp();
        float ReadTempC();

        float AccelSensitivityMgPerLsb() const noexcept;
        float GyroSensitivityMdpsPerLsb() const noexcept;
        float GyroFullScaleDps() const noexcept;
        float AccelRawToG(int16_t raw) const noexcept;
        float GyroRawToDps(int16_t raw) const noexcept;
        static constexpr float ClockwiseYawFromSensorZSign() noexcept { return -1.0f; }
        float GyroRawToClockwiseYawDps(int16_t raw) const noexcept;
        float GyroRawToBodyYawRadps(int16_t raw) const noexcept;
        float ReadClockwiseYawDps();
        float ReadBodyYawRateRadps();
        Eigen::Vector2f AccelRawToBodyPlanarG(int16_t rawX, int16_t rawY) const noexcept;

    private:
        static constexpr uint8_t kReadMask = 0x80;
        static constexpr uint8_t kAddressMask = 0x7F;

        static constexpr uint8_t CTRL3_BDU = 0x40;
        static constexpr uint8_t CTRL3_IF_INC = 0x04;
        static constexpr uint8_t CTRL3_SW_RESET = 0x01;

        static constexpr uint8_t CTRL8_HP_LPF2_XL_BW_MASK = 0xE0;
        static constexpr uint8_t HAODR_CFG_HAODR_SEL_MASK = 0x03;

        static constexpr uint8_t CTRL9_HP_SLOPE_XL_EN = 0x10;
        static constexpr uint8_t CTRL9_LPF2_XL_EN = 0x08;

        static constexpr uint8_t ToU8(ACCEL_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ODR_SETTING value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(HAODR_SELECTION value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ACCEL_FILTER_FREQ value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ACCEL_FULLSCALE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_LPF1_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_FULLSCALE_RANGE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(SELF_TEST_MODE value) { return static_cast<uint8_t>(value); }

        void Select() const;
        void Deselect() const;

        uint8_t _csPin;
        uint8_t _intPin;
        uint8_t _mosiPin;
        uint8_t _misoPin;
        uint8_t _clockPin;
        ACCEL_FULLSCALE _accelScale = ACCEL_FULLSCALE::G2;
        GYRO_FULLSCALE_RANGE _gyroScale = GYRO_FULLSCALE_RANGE::DPS0125;
        BeginFailureReason _lastBeginFailureReason = BeginFailureReason::None;
        uint8_t _lastWhoAmI = 0U;
        MazeMap::SensorMount _mount;
    };
}
