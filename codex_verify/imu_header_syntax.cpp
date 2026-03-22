#define ARDUINO_TEENSY41 1
#include "SPI.h"
#include "../MazeMap/MazeMap/LSV6DSV16X_IMU.h"

SPIClass SPI;

using TestImu = MazeMap::LSV6DSV16X_IMU<10, 11, 12, 13, 14>;

int main()
{
    TestImu imu;
    imu.Begin();
    imu.SetAccelMode(TestImu::ACCEL_MODE::HI_PERF, TestImu::ODR_SETTING::ODR_0240HZ_HP_N_LP);
    imu.SetAccelRange(TestImu::ACCEL_FILTER_FREQ::FRAC_1_100, TestImu::ACCEL_FULLSCALE::G4);
    imu.SetGyroMode(TestImu::GYRO_MODE::HI_PERF, TestImu::ODR_SETTING::ODR_0240HZ_HP_N_LP);
    imu.SetGyroRange(TestImu::GYRO_LPF1_MODE::CUT_102, TestImu::GYRO_FULLSCALE_RANGE::DPS1000);
    auto status = imu.ReadStatus();
    return status.HasAccelData() ? 0 : 0;
}
