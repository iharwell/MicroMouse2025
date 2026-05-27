#pragma once

#include "Defines.h"
#include "ImuSamplingProfile.h"
#include "SensorSnapshot.h"
#include "SensorMount.h"
#include "SensorTelemetryTypes.h"

#include <cmath>

#ifdef ARDUINO_TEENSY41

#include <SPI.h>

#ifdef DISABLE
#undef DISABLE
#endif

namespace MazeMap
{
    template <int CS_PIN, int INT_PIN, int MOSI_PIN, int MISO_PIN, int CLOCK_PIN>
    class LSM6DSV16X_IMU
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
            uint8_t data_;
        };

        explicit LSM6DSV16X_IMU(const MazeMap::SensorMount& mount = MazeMap::SensorMount()) noexcept
            : mount_(mount)
        {
        }

        bool Begin()
        {
            last_begin_failure_reason_ = BeginFailureReason::None;
            last_who_am_i_ = 0U;

            pinMode(CS_PIN, OUTPUT);
            digitalWrite(CS_PIN, HIGH);
            pinMode(INT_PIN, INPUT);

            SPI.setMOSI(MOSI_PIN);
            SPI.setMISO(MISO_PIN);
            SPI.setSCK(CLOCK_PIN);
            SPI.begin();

            if (!Reset())
            {
                last_begin_failure_reason_ = BeginFailureReason::ResetTimeout;
                last_who_am_i_ = ReadWhoAmI();
                return false;
            }

            WriteRegister(CTRL3_RW_ADR, static_cast<uint8_t>(CTRL3_BDU | CTRL3_IF_INC));
            last_who_am_i_ = ReadWhoAmI();
            if (last_who_am_i_ != WHO_AM_I_VALUE)
            {
                last_begin_failure_reason_ = BeginFailureReason::WhoAmIMismatch;
                return false;
            }

            return true;
        }

        bool ConfigureRuntimeForControlPeriod(
            unsigned long controlPeriodUs,
            bool enableAccel,
            ACCEL_FILTER_FREQ accelFilterFreq)
        {
            switch (MazeMap::SelectUiImuSamplingProfile(controlPeriodUs))
            {
            case MazeMap::UiImuSamplingProfile::Exact1000Hz:
                ConfigureUiHighAccuracyOdr(
                    HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                    enableAccel ? ODR_SETTING::ODR_0960HZ_HP_N_LP : ODR_SETTING::DISABLE,
                    ODR_SETTING::ODR_0960HZ_HP_N_LP);
                ConfigureRuntimeRanges(enableAccel, accelFilterFreq);
                return true;
            case MazeMap::UiImuSamplingProfile::Exact2000Hz:
                ConfigureUiHighAccuracyOdr(
                    HAODR_SELECTION::EXACT_1000_2000_4000_8000,
                    enableAccel ? ODR_SETTING::ODR_1920HZ_HP_N_LP : ODR_SETTING::DISABLE,
                    ODR_SETTING::ODR_1920HZ_HP_N_LP);
                ConfigureRuntimeRanges(enableAccel, accelFilterFreq);
                return true;
            default:
                return false;
            }
        }

        void ConfigureRuntimeRanges(bool enableAccel, ACCEL_FILTER_FREQ accelFilterFreq)
        {
            SetSelfTest(SELF_TEST_MODE::DISABLED, SELF_TEST_MODE::DISABLED);
            if (enableAccel)
            {
                SetAccelRange(accelFilterFreq, ACCEL_FULLSCALE::G8);
            }
            SetGyroRange(GYRO_LPF1_MODE::CUT_213, GYRO_FULLSCALE_RANGE::DPS2000);
        }

        void DisableSelfTest()
        {
            SetSelfTest(SELF_TEST_MODE::DISABLED, SELF_TEST_MODE::DISABLED);
        }

        void EnablePositiveSelfTest()
        {
            SetSelfTest(SELF_TEST_MODE::POSITIVE, SELF_TEST_MODE::POSITIVE);
        }

        bool SelfTestDeltasValid(
            float accelDeltaMgX,
            float accelDeltaMgY,
            float accelDeltaMgZ,
            float gyroDeltaDpsX,
            float gyroDeltaDpsY,
            float gyroDeltaDpsZ) const noexcept
        {
            return IsAccelSelfTestDeltaValidMg(accelDeltaMgX) &&
                IsAccelSelfTestDeltaValidMg(accelDeltaMgY) &&
                IsAccelSelfTestDeltaValidMg(accelDeltaMgZ) &&
                IsGyroSelfTestDeltaValidDps(gyroDeltaDpsX) &&
                IsGyroSelfTestDeltaValidDps(gyroDeltaDpsY) &&
                IsGyroSelfTestDeltaValidDps(gyroDeltaDpsZ);
        }

        void ResetRuntimeCalibration() noexcept
        {
            gyro_bias_radps_ = 0.0f;
            accel_bias_right_g_ = 0.0f;
            accel_bias_forward_g_ = 0.0f;
            accel_bias_initialized_ = false;
        }

        void SetRuntimeCalibration(
            float gyroBiasRadps,
            bool accelBiasInitialized,
            float accelBiasRightG,
            float accelBiasForwardG) noexcept
        {
            gyro_bias_radps_ = std::isfinite(gyroBiasRadps) ? gyroBiasRadps : 0.0f;
            accel_bias_initialized_ =
                accelBiasInitialized &&
                std::isfinite(accelBiasRightG) &&
                std::isfinite(accelBiasForwardG);
            if (accel_bias_initialized_)
            {
                accel_bias_right_g_ = accelBiasRightG;
                accel_bias_forward_g_ = accelBiasForwardG;
            }
            else
            {
                accel_bias_right_g_ = 0.0f;
                accel_bias_forward_g_ = 0.0f;
            }
        }

        float RuntimeGyroBiasRadps() const noexcept
        {
            return gyro_bias_radps_;
        }

        bool HasRuntimeAccelBias() const noexcept
        {
            return accel_bias_initialized_;
        }

        float RuntimeAccelBiasRightG() const noexcept
        {
            return accel_bias_right_g_;
        }

        float RuntimeAccelBiasForwardG() const noexcept
        {
            return accel_bias_forward_g_;
        }

        bool Reset(uint32_t timeout_ms = 50U)
        {
            WriteRegister(CTRL3_RW_ADR, CTRL3_SW_RESET);

            const uint32_t start_ms = millis();
            while ((ReadRegister(CTRL3_RW_ADR) & CTRL3_SW_RESET) != 0U)
            {
                if ((millis() - start_ms) >= timeout_ms)
                {
                    return false;
                }

                delay(1);
            }

            return true;
        }

        uint8_t ReadWhoAmI()
        {
            return ReadRegister(WHO_AM_I_R_ADR);
        }

        uint8_t ReadWhoAmIWithSettings(uint32_t clock_hz, uint8_t data_mode)
        {
            return ReadRegisterWithSettings(WHO_AM_I_R_ADR, clock_hz, data_mode);
        }

        bool IsConnected()
        {
            return ReadWhoAmI() == WHO_AM_I_VALUE;
        }

        BeginFailureReason GetLastBeginFailureReason() const
        {
            return last_begin_failure_reason_;
        }

        const char* GetLastBeginFailureReasonName() const
        {
            switch (last_begin_failure_reason_)
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

        uint8_t GetLastWhoAmI() const
        {
            return last_who_am_i_;
        }

        void WriteRegister(uint8_t address, uint8_t data)
        {
            SPI.beginTransaction(GetSpiSettings());
            Select();
            SPI.transfer(address & kAddressMask);
            SPI.transfer(data);
            Deselect();
            SPI.endTransaction();
        }

        uint8_t ReadRegister(uint8_t address)
        {
            SPI.beginTransaction(GetSpiSettings());
            Select();
            SPI.transfer(address | kReadMask);
            const uint8_t value = SPI.transfer(0x00);
            Deselect();
            SPI.endTransaction();
            return value;
        }

        uint8_t ReadRegisterWithSettings(uint8_t address, uint32_t clock_hz, uint8_t data_mode)
        {
            SPI.beginTransaction(SPISettings(clock_hz, MSBFIRST, data_mode));
            Select();
            SPI.transfer(address | kReadMask);
            const uint8_t value = SPI.transfer(0x00);
            Deselect();
            SPI.endTransaction();
            return value;
        }

        void WriteRegisters(uint8_t start_address, const uint8_t *data, size_t length)
        {
            SPI.beginTransaction(GetSpiSettings());
            Select();
            SPI.transfer(start_address & kAddressMask);

            for (size_t i = 0; i < length; ++i)
            {
                SPI.transfer(data[i]);
            }

            Deselect();
            SPI.endTransaction();
        }

        void ReadRegisters(uint8_t start_address, uint8_t *data, size_t length)
        {
            SPI.beginTransaction(GetSpiSettings());
            Select();
            SPI.transfer(start_address | kReadMask);

            for (size_t i = 0; i < length; ++i)
            {
                data[i] = SPI.transfer(0x00);
            }

            Deselect();
            SPI.endTransaction();
        }

        void WriteLargeRegister(uint8_t address, uint16_t data)
        {
            const uint8_t bytes[2] =
            {
                static_cast<uint8_t>(data & 0xFFU),
                static_cast<uint8_t>((data >> 8) & 0xFFU),
            };

            WriteRegisters(address, bytes, sizeof(bytes));
        }

        uint16_t ReadLargeRegister(uint8_t address)
        {
            uint8_t bytes[2] = {};
            ReadRegisters(address, bytes, sizeof(bytes));
            return static_cast<uint16_t>(bytes[0]) |
                   static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8U);
        }

        void SetAccelMode(ACCEL_MODE mode, ODR_SETTING odr)
        {
            WriteRegister(CTRL1_RW_ADR, ToU8(mode) | ToU8(odr));
        }

        void ConfigureUiHighAccuracyOdr(HAODR_SELECTION selection, ODR_SETTING accelOdr, ODR_SETTING gyroOdr)
        {
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
        }

        void SetAccelRange(ACCEL_FILTER_FREQ freq, ACCEL_FULLSCALE scale)
        {
            accel_scale_ = scale;

            const uint8_t freq_bits = ToU8(freq) & CTRL8_HP_LPF2_XL_BW_MASK;
            WriteRegister(CTRL8_RW_ADR, freq_bits | ToU8(scale));

            uint8_t ctrl9 = ReadRegister(CTRL9_RW_ADR);
            ctrl9 &= static_cast<uint8_t>(~(CTRL9_HP_SLOPE_XL_EN | CTRL9_LPF2_XL_EN));

            if (freq != ACCEL_FILTER_FREQ::FRAC_1_002)
            {
                ctrl9 |= CTRL9_LPF2_XL_EN;
            }

            WriteRegister(CTRL9_RW_ADR, ctrl9);
        }

        void SetGyroMode(GYRO_MODE mode, ODR_SETTING odr)
        {
            WriteRegister(CTRL2_RW_ADR, ToU8(mode) | ToU8(odr));
        }

        void SetGyroRange(GYRO_LPF1_MODE lpf1, GYRO_FULLSCALE_RANGE range)
        {
            gyro_scale_ = range;
            WriteRegister(CTRL6_RW_ADR, ToU8(lpf1) | ToU8(range));
        }

        void SetSelfTest(SELF_TEST_MODE gyroMode, SELF_TEST_MODE accelMode)
        {
            const uint8_t gyroBits = static_cast<uint8_t>((ToU8(gyroMode) & 0x03U) << 2U);
            const uint8_t accelBits = static_cast<uint8_t>(ToU8(accelMode) & 0x03U);
            WriteRegister(CTRL10_RW_ADR, gyroBits | accelBits);
        }

        StatusReg ReadStatus()
        {
            return StatusReg(ReadRegister(STATUS_REG_R_ADR));
        }

        ImuTelemetry CaptureTelemetry(ImuObservationTiming* const timing = nullptr)
        {
            ImuTelemetry telemetry{};
            const std::uint32_t readStartUs = micros();
            if (timing != nullptr)
            {
                timing->readStartUs = readStartUs;
                timing->drdyUs = (digitalRead(INT_PIN) == HIGH) ? readStartUs : 0UL;
            }

            const auto status = ReadStatus();
            telemetry.status = status.Raw();
            telemetry.gyroX = ReadGyroX();
            telemetry.gyroY = ReadGyroY();
            telemetry.gyroZ = ReadGyroZ();
            telemetry.accelX = ReadAccelX();
            telemetry.accelY = ReadAccelY();
            telemetry.accelZ = ReadAccelZ();
            telemetry.temp = ReadTemp();
            telemetry.interruptHigh = (digitalRead(INT_PIN) == HIGH);
            if (timing != nullptr)
            {
                timing->readDoneUs = micros();
            }
            return telemetry;
        }

        void CaptureInertialSnapshot(SensorSnapshot& snapshot)
        {
            ImuObservationTiming timing{};
            const ImuTelemetry telemetry = CaptureTelemetry(&timing);
            snapshot.SetFrontRightImuTelemetry(ImuTelemetry{});
            snapshot.SetBackLeftImuTelemetry(telemetry);
            snapshot.SetImuTiming(timing);

            const float rawYawRateRadps = GyroRawToBodyYawRadps(telemetry.gyroZ);
            const Eigen::Vector2f accelBodyG = AccelRawToBodyPlanarG(telemetry.accelX, telemetry.accelY);
            snapshot.SetAccelerationBiasValid(accel_bias_initialized_);
            if (accel_bias_initialized_)
            {
                const float accelDeltaRightG = accelBodyG.x() - accel_bias_right_g_;
                const float accelDeltaForwardG = accelBodyG.y() - accel_bias_forward_g_;
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
            snapshot.SetYawRateBiasRadps(gyro_bias_radps_);
            snapshot.SetYawRateRadps(rawYawRateRadps - gyro_bias_radps_);
        }

        int16_t ReadGyroX()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTX_L_G_R_ADR));
        }

        int16_t ReadGyroY()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTY_L_G_R_ADR));
        }

        int16_t ReadGyroZ()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTZ_L_G_R_ADR));
        }

        int16_t ReadAccelX()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTX_L_A_R_ADR));
        }

        int16_t ReadAccelY()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTY_L_A_R_ADR));
        }

        int16_t ReadAccelZ()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUTZ_L_A_R_ADR));
        }

        int16_t ReadTemp()
        {
            return static_cast<int16_t>(ReadLargeRegister(OUT_TEMP_L_R_ADR));
        }

        float ReadTempC()
        {
            return 25.0f + (static_cast<float>(ReadTemp()) / 256.0f);
        }

        float AccelSensitivityMgPerLsb() const
        {
            switch (accel_scale_)
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

        float GyroSensitivityMdpsPerLsb() const
        {
            switch (gyro_scale_)
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

        float AccelRawToG(int16_t raw) const
        {
            return (static_cast<float>(raw) * AccelSensitivityMgPerLsb()) / 1000.0f;
        }

        float GyroRawToDps(int16_t raw) const
        {
            return (static_cast<float>(raw) * GyroSensitivityMdpsPerLsb()) / 1000.0f;
        }

        static constexpr float ClockwiseYawFromSensorZSign() noexcept
        {
            return -1.0f;
        }

        float GyroRawToClockwiseYawDps(int16_t raw) const
        {
            return ClockwiseYawFromSensorZSign() * GyroRawToDps(raw);
        }

        float GyroRawToBodyYawRadps(int16_t raw) const
        {
            return mount_.TransformClockwiseYawRateToBody(GyroRawToClockwiseYawDps(raw) * DEG_TO_RAD_F);
        }

        float ReadClockwiseYawDps()
        {
            return GyroRawToClockwiseYawDps(ReadGyroZ());
        }

        float ReadBodyYawRateRadps()
        {
            return GyroRawToBodyYawRadps(ReadGyroZ());
        }

        Eigen::Vector2f AccelRawToBodyPlanarG(int16_t rawX, int16_t rawY) const
        {
            return mount_.TransformPlanarVectorToBody(
                Eigen::Vector2f(
                    AccelRawToG(rawX),
                    AccelRawToG(rawY)));
        }

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

        static constexpr uint8_t STATUS_TIMESTAMP_ENDCOUNT = 0x80;
        static constexpr uint8_t STATUS_OIS_DRDY = 0x20;
        static constexpr uint8_t STATUS_GDA_EIS = 0x10;
        static constexpr uint8_t STATUS_AH_QVARDA = 0x08;
        static constexpr uint8_t STATUS_TDA = 0x04;
        static constexpr uint8_t STATUS_GDA = 0x02;
        static constexpr uint8_t STATUS_XLDA = 0x01;

        static SPISettings GetSpiSettings()
        {
            return SPISettings(kMaxSpiClockHz, MSBFIRST, SPI_MODE3);
        }

        static constexpr uint8_t ToU8(ACCEL_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ODR_SETTING value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(HAODR_SELECTION value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ACCEL_FILTER_FREQ value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(ACCEL_FULLSCALE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_LPF1_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_MODE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(GYRO_FULLSCALE_RANGE value) { return static_cast<uint8_t>(value); }
        static constexpr uint8_t ToU8(SELF_TEST_MODE value) { return static_cast<uint8_t>(value); }

        static bool IsAccelSelfTestDeltaValidMg(float deltaMg) noexcept
        {
            const float absoluteDeltaMg = std::fabs(deltaMg);
            return std::isfinite(absoluteDeltaMg) &&
                (absoluteDeltaMg >= 50.0f) &&
                (absoluteDeltaMg <= 1700.0f);
        }

        bool IsGyroSelfTestDeltaValidDps(float deltaDps) const noexcept
        {
            const float absoluteDeltaDps = std::fabs(deltaDps);
            if (!std::isfinite(absoluteDeltaDps))
            {
                return false;
            }

            if (gyro_scale_ == GYRO_FULLSCALE_RANGE::DPS0250)
            {
                return (absoluteDeltaDps >= 20.0f) && (absoluteDeltaDps <= 80.0f);
            }

            if (gyro_scale_ == GYRO_FULLSCALE_RANGE::DPS2000)
            {
                return (absoluteDeltaDps >= 150.0f) && (absoluteDeltaDps <= 700.0f);
            }

            return false;
        }

        static void Select()
        {
            digitalWrite(CS_PIN, LOW);
        }

        static void Deselect()
        {
            digitalWrite(CS_PIN, HIGH);
        }

        ACCEL_FULLSCALE accel_scale_ = ACCEL_FULLSCALE::G2;
        GYRO_FULLSCALE_RANGE gyro_scale_ = GYRO_FULLSCALE_RANGE::DPS0125;
        BeginFailureReason last_begin_failure_reason_ = BeginFailureReason::None;
        uint8_t last_who_am_i_ = 0U;
        MazeMap::SensorMount mount_;
        float gyro_bias_radps_ = 0.0f;
        float accel_bias_right_g_ = 0.0f;
        float accel_bias_forward_g_ = 0.0f;
        bool accel_bias_initialized_ = false;
    };

}

#else

namespace MazeMap
{
    template <int CS_PIN, int INT_PIN, int MOSI_PIN, int MISO_PIN, int CLOCK_PIN>
    class LSM6DSV16X_IMU
    {
    public:
        enum class BeginFailureReason : uint8_t
        {
            None = 0U,
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

        enum class ACCEL_MODE : uint8_t
        {
            DISABLE = 0x00,
            HI_PERF = 0x00,
            HI_ACC = 0x10,
            ODR_TRIG = 0x30,
            LP_MODE = 0x50,
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
            constexpr bool HasTimestampOverflow() const { return false; }
            constexpr bool HasOisData() const { return false; }
            constexpr bool HasEisGyroData() const { return false; }
            constexpr bool HasAhQvarData() const { return false; }
            constexpr bool HasTemperatureData() const { return false; }
            constexpr bool HasGyroData() const { return false; }
            constexpr bool HasAccelData() const { return false; }

        private:
            uint8_t data_ = 0U;
        };

        explicit LSM6DSV16X_IMU(const MazeMap::SensorMount& mount = MazeMap::SensorMount()) noexcept
            : mount_(mount)
        {
        }

        bool Begin() { return true; }
        bool Reset(uint32_t timeout_ms = 50U)
        {
            (void)timeout_ms;
            return true;
        }

        bool ConfigureRuntimeForControlPeriod(
            unsigned long controlPeriodUs,
            bool enableAccel,
            ACCEL_FILTER_FREQ accelFilterFreq)
        {
            (void)controlPeriodUs;
            ConfigureRuntimeRanges(enableAccel, accelFilterFreq);
            return true;
        }

        void ConfigureRuntimeRanges(bool enableAccel, ACCEL_FILTER_FREQ accelFilterFreq)
        {
            (void)accelFilterFreq;
            if (enableAccel)
            {
                accel_scale_ = ACCEL_FULLSCALE::G8;
            }
            gyro_scale_ = GYRO_FULLSCALE_RANGE::DPS2000;
        }

        void DisableSelfTest() const {}
        void EnablePositiveSelfTest() const {}
        bool SelfTestDeltasValid(
            float accelDeltaMgX,
            float accelDeltaMgY,
            float accelDeltaMgZ,
            float gyroDeltaDpsX,
            float gyroDeltaDpsY,
            float gyroDeltaDpsZ) const noexcept
        {
            (void)accelDeltaMgX;
            (void)accelDeltaMgY;
            (void)accelDeltaMgZ;
            (void)gyroDeltaDpsX;
            (void)gyroDeltaDpsY;
            (void)gyroDeltaDpsZ;
            return true;
        }

        void ResetRuntimeCalibration() noexcept
        {
            gyro_bias_radps_ = 0.0f;
            accel_bias_right_g_ = 0.0f;
            accel_bias_forward_g_ = 0.0f;
            accel_bias_initialized_ = false;
        }

        void SetRuntimeCalibration(
            float gyroBiasRadps,
            bool accelBiasInitialized,
            float accelBiasRightG,
            float accelBiasForwardG) noexcept
        {
            gyro_bias_radps_ = std::isfinite(gyroBiasRadps) ? gyroBiasRadps : 0.0f;
            accel_bias_initialized_ =
                accelBiasInitialized &&
                std::isfinite(accelBiasRightG) &&
                std::isfinite(accelBiasForwardG);
            accel_bias_right_g_ = accel_bias_initialized_ ? accelBiasRightG : 0.0f;
            accel_bias_forward_g_ = accel_bias_initialized_ ? accelBiasForwardG : 0.0f;
        }

        float RuntimeGyroBiasRadps() const noexcept
        {
            return gyro_bias_radps_;
        }

        bool HasRuntimeAccelBias() const noexcept
        {
            return accel_bias_initialized_;
        }

        float RuntimeAccelBiasRightG() const noexcept
        {
            return accel_bias_right_g_;
        }

        float RuntimeAccelBiasForwardG() const noexcept
        {
            return accel_bias_forward_g_;
        }

        uint8_t ReadWhoAmI() const { return 0U; }
        uint8_t ReadWhoAmIWithSettings(uint32_t clock_hz, uint8_t data_mode) const
        {
            (void)clock_hz;
            (void)data_mode;
            return 0U;
        }

        bool IsConnected() const { return false; }
        BeginFailureReason GetLastBeginFailureReason() const { return BeginFailureReason::None; }
        const char* GetLastBeginFailureReasonName() const { return "stub"; }
        uint8_t GetLastWhoAmI() const { return 0U; }
        void SetAccelMode(ACCEL_MODE mode, ODR_SETTING odr) const
        {
            (void)mode;
            (void)odr;
        }
        void ConfigureUiHighAccuracyOdr(HAODR_SELECTION selection, ODR_SETTING accelOdr, ODR_SETTING gyroOdr) const
        {
            (void)selection;
            (void)accelOdr;
            (void)gyroOdr;
        }
        void SetAccelRange(ACCEL_FILTER_FREQ freq, ACCEL_FULLSCALE scale)
        {
            (void)freq;
            accel_scale_ = scale;
        }
        void SetGyroMode(GYRO_MODE mode, ODR_SETTING odr) const
        {
            (void)mode;
            (void)odr;
        }
        void SetGyroRange(GYRO_LPF1_MODE lpf1, GYRO_FULLSCALE_RANGE range)
        {
            (void)lpf1;
            gyro_scale_ = range;
        }
        void SetSelfTest(SELF_TEST_MODE gyroMode, SELF_TEST_MODE accelMode) const
        {
            (void)gyroMode;
            (void)accelMode;
        }

        StatusReg ReadStatus() const { return {}; }

        ImuTelemetry CaptureTelemetry(ImuObservationTiming* const timing = nullptr) const
        {
            (void)timing;
            return {};
        }

        void CaptureInertialSnapshot(SensorSnapshot& snapshot)
        {
            snapshot.SetFrontRightImuTelemetry(ImuTelemetry{});
            snapshot.SetBackLeftImuTelemetry(ImuTelemetry{});
            snapshot.SetImuTiming(ImuObservationTiming{});
            snapshot.SetBodyRightAccelerationMps2(0.0f);
            snapshot.SetBodyForwardAccelerationMps2(0.0f);
            snapshot.SetPlanarAccelerationMps2(0.0f);
            snapshot.SetAccelerationBiasValid(accel_bias_initialized_);
            snapshot.SetRawYawRateRadps(0.0f);
            snapshot.SetYawRateBiasRadps(gyro_bias_radps_);
            snapshot.SetYawRateRadps(-gyro_bias_radps_);
        }

        int16_t ReadGyroX() const { return 0; }
        int16_t ReadGyroY() const { return 0; }
        int16_t ReadGyroZ() const { return 0; }

        int16_t ReadAccelX() const { return 0; }
        int16_t ReadAccelY() const { return 0; }
        int16_t ReadAccelZ() const { return 0; }

        int16_t ReadTemp() const { return 0; }
        float ReadTempC() const { return 25.0f; }
        float AccelSensitivityMgPerLsb() const
        {
            switch (accel_scale_)
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
        float GyroSensitivityMdpsPerLsb() const
        {
            switch (gyro_scale_)
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
        float AccelRawToG(int16_t raw) const
        {
            return (static_cast<float>(raw) * AccelSensitivityMgPerLsb()) / 1000.0f;
        }
        float GyroRawToDps(int16_t raw) const
        {
            return (static_cast<float>(raw) * GyroSensitivityMdpsPerLsb()) / 1000.0f;
        }

        static constexpr float ClockwiseYawFromSensorZSign() noexcept
        {
            return -1.0f;
        }

        float GyroRawToClockwiseYawDps(int16_t raw) const
        {
            return ClockwiseYawFromSensorZSign() * GyroRawToDps(raw);
        }

        float GyroRawToBodyYawRadps(int16_t raw) const
        {
            return mount_.TransformClockwiseYawRateToBody(GyroRawToClockwiseYawDps(raw) * DEG_TO_RAD_F);
        }

        float ReadClockwiseYawDps() const
        {
            return GyroRawToClockwiseYawDps(ReadGyroZ());
        }

        float ReadBodyYawRateRadps() const
        {
            return GyroRawToBodyYawRadps(ReadGyroZ());
        }

        Eigen::Vector2f AccelRawToBodyPlanarG(int16_t rawX, int16_t rawY) const
        {
            return mount_.TransformPlanarVectorToBody(
                Eigen::Vector2f(
                    AccelRawToG(rawX),
                    AccelRawToG(rawY)));
        }

    private:
        ACCEL_FULLSCALE accel_scale_ = ACCEL_FULLSCALE::G2;
        GYRO_FULLSCALE_RANGE gyro_scale_ = GYRO_FULLSCALE_RANGE::DPS0125;
        MazeMap::SensorMount mount_;
        float gyro_bias_radps_ = 0.0f;
        float accel_bias_right_g_ = 0.0f;
        float accel_bias_forward_g_ = 0.0f;
        bool accel_bias_initialized_ = false;
    };

}

#endif
