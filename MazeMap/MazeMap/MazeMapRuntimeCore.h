#pragma once
// Declares shared runtime state, calibration, and sensor-processing utilities used across the MazeMap application runtime.

#include "Defines.h"
#include "DiagonalWallCentering.h"
#include "EncoderStallPolicy.h"
#include "FanRampProfile.h"
#include "HardwareConfig.h"
#include "ImuCalibrationPolicy.h"
#include "Maze.h"
#include "MissionStartPolicy.h"
#include "MotorEncoderDrive.h"
#include "ProportionalDerivativeCluster.h"
#include "RollingAverageWindow.h"
#include "Vehicle.h"
#include "WallDetectionThresholds.h"
#include "WallObservationPipeline.h"
#include "WallSensorCalibration.h"
#include "Pins.h"

#if !defined(ARDUINO_TEENSY41) && !defined(MAZEMAP_PINS_NAMESPACE_AVAILABLE)
namespace MazeMap::Pins
{
    constexpr uint8_t R_MotorA = 5;
    constexpr uint8_t R_MotorB = 6;
    constexpr uint8_t R_EncA = 7;
    constexpr uint8_t R_EncB = 8;
    constexpr uint8_t L_MotorA = 24;
    constexpr uint8_t L_MotorB = 25;
    constexpr uint8_t L_EncA = 2;
    constexpr uint8_t L_EncB = 3;
    constexpr uint8_t Fan_CTRL = 4;
    constexpr uint8_t IMU_INT_1A = 32;
    constexpr uint8_t IMU_INT_1B = 33;
    constexpr uint8_t LED_Ctrl_Forward_Right = 19;
    constexpr uint8_t LED_Ctrl_Forward_Left = 18;
    constexpr uint8_t LED_Ctrl_Side_Right = 17;
    constexpr uint8_t LED_Ctrl_Side_Left = 16;
}
#endif

#if !defined(MAZEMAP_PINS_NAMESPACE_AVAILABLE)
namespace Pins = MazeMap::Pins;
#endif

#if !defined(ARDUINO_TEENSY41)
inline bool SetupHardware()
{
    return true;
}
#endif

#include "CoreConfig.h"
#include "SensorTelemetryTypes.h"
#include "WallSensorRuntimeTypes.h"


#include "DiagnosticConfig.h"
#include "AuxMeasurementConfig.h"
#include "FrontWallCharacterizationConfig.h"
#include "LedCalibrationConfig.h"
#include "EigenCompat.h"

struct MotionLimits
{
    float maxSpeedMps;
    float accelMps2;
    float decelMps2;
    float maxAngularSpeedRadps;
    float angularAccelRadps2;
    float angleToleranceRad = MazeMap::Config::kAngleToleranceRad;
    float angularSpeedToleranceRadps = MazeMap::Config::kAngularSpeedToleranceRadps;
};

struct ControlCycleTiming
{
    uint32_t controlStartUs = 0UL;
    uint32_t controlEndUs = 0UL;
    uint32_t pwmLatchUs = 0UL;
    uint32_t encoderLatchUs = 0UL;
    uint32_t encoderReadDoneUs = 0UL;
    uint32_t ukfPredictStartUs = 0UL;
    uint32_t ukfPredictEndUs = 0UL;
    uint32_t ukfPredictDurationUs = 0UL;
    uint32_t ukfUpdateStartUs = 0UL;
    uint32_t ukfUpdateEndUs = 0UL;
    uint32_t ukfUpdateDurationUs = 0UL;
    uint32_t ukfTotalDurationUs = 0UL;
    uint32_t cycleCounterStart = 0UL;
    uint32_t cycleCounterEnd = 0UL;
};

struct RawWallSensorSample
{
    uint16_t ambientAdcCode = 0U;
    uint16_t litAdcCode = 0U;
    float ambientLight = 0.0f;
    float litLight = 0.0f;
    float differentialLight = 0.0f;
    float rawDistanceM = 0.20f;
    OpticalObservationTiming timing{};
};

struct WallSensorCalibrationInput
{
    float measuredValue = 0.0f;
    float fallbackDistanceM = 0.20f;
    float differentialLight = 0.0f;
    float ambientLight = 0.0f;
    float litLight = 0.0f;
    OpticalObservationTiming timing{};
};

struct RobustSignalBand
{
    float median = 0.0f;
    float low = 0.0f;
    float high = 0.0f;
};

struct WallSensorCalibrationCapture
{
    WallSensorCalibrationInput input{};
    RobustSignalBand differentialLightBand{};
    bool haveDifferentialLightBand = false;
};

template <uint8_t WindowCycles>
struct AveragedWallSensorInputWindow
{
    void Clear() noexcept
    {
        measuredValue.Clear();
        fallbackDistanceM.Clear();
        differentialLight.Clear();
        ambientLight.Clear();
        litLight.Clear();
        latestTiming = {};
    }

    WallSensorCalibrationInput Average() const noexcept
    {
        WallSensorCalibrationInput averaged{};
        averaged.measuredValue = measuredValue.Average();
        averaged.fallbackDistanceM = fallbackDistanceM.Average();
        averaged.differentialLight = differentialLight.Average();
        averaged.ambientLight = ambientLight.Average();
        averaged.litLight = litLight.Average();
        averaged.timing = latestTiming;
        return averaged;
    }

    WallSensorCalibrationInput PushAndAverage(const WallSensorCalibrationInput& input) noexcept
    {
        measuredValue.Push(input.measuredValue);
        fallbackDistanceM.Push(input.fallbackDistanceM);
        differentialLight.Push(input.differentialLight);
        ambientLight.Push(input.ambientLight);
        litLight.Push(input.litLight);
        latestTiming = input.timing;
        return Average();
    }

    MazeMap::RollingAverageWindow<WindowCycles> measuredValue;
    MazeMap::RollingAverageWindow<WindowCycles> fallbackDistanceM;
    MazeMap::RollingAverageWindow<WindowCycles> differentialLight;
    MazeMap::RollingAverageWindow<WindowCycles> ambientLight;
    MazeMap::RollingAverageWindow<WindowCycles> litLight;
    OpticalObservationTiming latestTiming{};
};

template <uint8_t WindowCycles>
inline WallSensorCalibrationInput UseCompletedWallSensorInputOrAverage(
    const WallSensorCalibrationInput& input,
    AveragedWallSensorInputWindow<WindowCycles>& averageWindow) noexcept
{
    return input.timing.observationReadyUs != 0UL ?
        averageWindow.PushAndAverage(input) :
        averageWindow.Average();
}

inline uint8_t ReadDrivenLowPinWithPullup(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(20);
    const uint8_t level = static_cast<uint8_t>(digitalRead(pin));
    pinMode(pin, INPUT);
    return level;
}

struct AveragedBackLeftImuSample
{
    float accelMgX = 0.0f;
    float accelMgY = 0.0f;
    float accelMgZ = 0.0f;
    float gyroDpsX = 0.0f;
    float gyroDpsY = 0.0f;
    float gyroDpsZ = 0.0f;
};

enum class StationaryImuCalibrationResult : uint8_t
{
    Success = 0U,
    RestartEncoderMotion,
    Failure,
};

inline constexpr unsigned long kImuCalibrationSampleIntervalMs = 2UL;
inline constexpr uint16_t kImuSelfTestAverageSamples = 64U;
inline constexpr uint16_t kImuSelfTestSettleMs = 50U;
inline constexpr float kImuSelfTestGyroFullScaleDps = 2000.0f;

inline MazeMap::EncoderCountPair CaptureDriveEncoderCounts()
{
    MazeMap::EncoderCountPair counts{};
    const auto& leftDriveHardware = MazeMap::MotorEncoderDrive::GetLeftHardwareConfig();
    const auto& rightDriveHardware = MazeMap::MotorEncoderDrive::GetRightHardwareConfig();
    counts.left = MazeMap::Platform::ReadEncoderCount(leftDriveHardware.encoderChannel);
    counts.right = MazeMap::Platform::ReadEncoderCount(rightDriveHardware.encoderChannel);
    return counts;
}

inline bool HaveDriveEncodersMovedSince(const MazeMap::EncoderCountPair& startCounts)
{
    return MazeMap::HaveEncoderCountsChanged(startCounts, CaptureDriveEncoderCounts());
}

inline StationaryImuCalibrationResult WaitForImuCalibrationSettle(
    const MazeMap::EncoderCountPair& startCounts,
    unsigned long settleMs)
{
    const unsigned long settleStartMs = millis();
    while ((millis() - settleStartMs) < settleMs)
    {
        if (HaveDriveEncodersMovedSince(startCounts))
        {
            return StationaryImuCalibrationResult::RestartEncoderMotion;
        }

        delay(1);
    }

    return StationaryImuCalibrationResult::Success;
}

inline StationaryImuCalibrationResult AverageBackLeftImuSelfTestSample(
    MazeMap::Vehicle::ImuBackLeft& imu,
    uint16_t sampleCount,
    const MazeMap::EncoderCountPair& startCounts,
    AveragedBackLeftImuSample& averagedSample)
{
    if (sampleCount == 0U)
    {
        return StationaryImuCalibrationResult::Failure;
    }

    const float accelMgPerLsb = imu.AccelSensitivityMgPerLsb();
    const float gyroDpsPerLsb = imu.GyroSensitivityMdpsPerLsb() / 1000.0f;
    double accelMgSumX = 0.0;
    double accelMgSumY = 0.0;
    double accelMgSumZ = 0.0;
    double gyroDpsSumX = 0.0;
    double gyroDpsSumY = 0.0;
    double gyroDpsSumZ = 0.0;

    for (uint16_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
    {
        if (HaveDriveEncodersMovedSince(startCounts))
        {
            return StationaryImuCalibrationResult::RestartEncoderMotion;
        }

        const MazeMap::Vehicle::ImuBackLeft::Axes accel = imu.ReadAccel();
        const MazeMap::Vehicle::ImuBackLeft::Axes gyro = imu.ReadGyro();
        accelMgSumX += static_cast<double>(accel.x) * accelMgPerLsb;
        accelMgSumY += static_cast<double>(accel.y) * accelMgPerLsb;
        accelMgSumZ += static_cast<double>(accel.z) * accelMgPerLsb;
        gyroDpsSumX += static_cast<double>(gyro.x) * gyroDpsPerLsb;
        gyroDpsSumY += static_cast<double>(gyro.y) * gyroDpsPerLsb;
        gyroDpsSumZ += static_cast<double>(gyro.z) * gyroDpsPerLsb;
        delay(kImuCalibrationSampleIntervalMs);
    }

    if (HaveDriveEncodersMovedSince(startCounts))
    {
        return StationaryImuCalibrationResult::RestartEncoderMotion;
    }

    const double normalization = 1.0 / static_cast<double>(sampleCount);
    averagedSample.accelMgX = static_cast<float>(accelMgSumX * normalization);
    averagedSample.accelMgY = static_cast<float>(accelMgSumY * normalization);
    averagedSample.accelMgZ = static_cast<float>(accelMgSumZ * normalization);
    averagedSample.gyroDpsX = static_cast<float>(gyroDpsSumX * normalization);
    averagedSample.gyroDpsY = static_cast<float>(gyroDpsSumY * normalization);
    averagedSample.gyroDpsZ = static_cast<float>(gyroDpsSumZ * normalization);
    return StationaryImuCalibrationResult::Success;
}

inline void FormatHexByte(uint8_t value, char (&buffer)[3])
{
    snprintf(buffer, sizeof(buffer), "%02X", static_cast<unsigned>(value));
}

inline const char* WallSensorIdName(WallSensorId sensorId)
{
    switch (sensorId)
    {
    case WallSensorId::FrontLeft:
        return "front_left";
    case WallSensorId::FrontRight:
        return "front_right";
    case WallSensorId::SideLeft:
        return "side_left";
    case WallSensorId::SideRight:
        return "side_right";
    default:
        return "unknown";
    }
}

inline const char* CalibrationWallName(CalibrationWall wall)
{
    switch (wall)
    {
    case CalibrationWall::West:
        return "west";
    case CalibrationWall::East:
        return "east";
    case CalibrationWall::South:
        return "south";
    case CalibrationWall::North:
        return "north";
    default:
        return "unknown";
    }
}

inline const char* WallTouchOutcomeName(WallTouchOutcome outcome)
{
    switch (outcome)
    {
    case WallTouchOutcome::SeatedContact:
        return "seated_contact";
    case WallTouchOutcome::PassedThroughNoWall:
        return "passed_through";
    default:
        return "unknown";
    }
}

inline const char* DirectionName(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return "up";
    case MazeMap::UpRight:
        return "up_right";
    case MazeMap::Right:
        return "right";
    case MazeMap::DownRight:
        return "down_right";
    case MazeMap::Down:
        return "down";
    case MazeMap::DownLeft:
        return "down_left";
    case MazeMap::Left:
        return "left";
    case MazeMap::UpLeft:
        return "up_left";
    default:
        return "unknown";
    }
}

inline const char* WallStateName(MazeMap::WallState state)
{
    switch (state)
    {
    case MazeMap::NoWall:
        return "no_wall";
    case MazeMap::Wall:
        return "wall";
    case MazeMap::Unknown:
    default:
        return "unknown";
    }
}

inline float SignF(float value)
{
    return static_cast<float>((value > 0.0f) - (value < 0.0f));
}

inline float WrapAngleRad(float angle)
{
    return std::remainder(angle, TWO_PI_F);
}

inline float AngleErrorRad(float target, float measured)
{
    return WrapAngleRad(target - measured);
}

inline Eigen::Vector2f DirectionToUnitVector(MazeMap::Direction direction)
{
    float dx = 0.0f;
    float dy = 0.0f;
    MazeMap::GetHeading(direction, dx, dy);
    return Eigen::Vector2f(dx, dy);
}

inline float DirectionToYawRad(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return 0.0f;
    case MazeMap::UpRight:
        return 0.25f * PI_F;
    case MazeMap::Right:
        return HALF_PI_F;
    case MazeMap::DownRight:
        return 0.75f * PI_F;
    case MazeMap::Down:
        return PI_F;
    case MazeMap::DownLeft:
        return -0.75f * PI_F;
    case MazeMap::Left:
        return -HALF_PI_F;
    case MazeMap::UpLeft:
        return -0.25f * PI_F;
    default:
        return 0.0f;
    }
}

inline Eigen::Vector2f HeadingUnitFromYawRad(float yawRad)
{
    return Eigen::Vector2f(sinf(yawRad), cosf(yawRad));
}

inline float HeadingErrorRad(const Eigen::Vector2f& targetHeading, const Eigen::Vector2f& measuredHeading)
{
    const float dot = (std::clamp)(targetHeading.dot(measuredHeading), -1.0f, 1.0f);
    const float cross = (targetHeading.x() * measuredHeading.y()) - (targetHeading.y() * measuredHeading.x());
    return atan2f(cross, dot);
}

inline constexpr float kStandardGravityMps2 = 9.80665f;
inline constexpr unsigned long kFanRampStepMs = 20UL;
inline float gMissionFanDutyCycle = 0.0f;

inline uint16_t FanPwmCode(float dutyCycle)
{
#if defined(ARDUINO_TEENSY41)
    const float clampedDutyCycle = (std::clamp)(dutyCycle, 0.0f, 1.0f);
    const uint32_t maxPwmCode = (1UL << HardwareConfig::kPwmBits) - 1UL;
    return static_cast<uint16_t>(clampedDutyCycle * static_cast<float>(maxPwmCode) + 0.5f);
#else
    (void)dutyCycle;
    return 0U;
#endif
}

inline void WriteFanDutyCycle(float dutyCycle)
{
    gMissionFanDutyCycle = (std::clamp)(dutyCycle, 0.0f, 1.0f);
#if defined(ARDUINO_TEENSY41)
    analogWrite(Pins::Fan_CTRL, FanPwmCode(dutyCycle));
#else
    (void)dutyCycle;
#endif
}

inline float GetMissionFanDutyCycle()
{
    return gMissionFanDutyCycle;
}

inline void RampFanDutyCycle(float targetDutyCycle)
{
#if defined(ARDUINO_TEENSY41)
    const unsigned long rampDurationMs = static_cast<unsigned long>(MazeMap::Config::kRacingFanRampMs);
    const float clampedTargetDutyCycle = MazeMap::ComputeFanRampDutyCycle(targetDutyCycle, rampDurationMs, rampDurationMs);
    if (clampedTargetDutyCycle <= 0.0f)
    {
        WriteFanDutyCycle(0.0f);
        return;
    }

    if (rampDurationMs == 0UL)
    {
        WriteFanDutyCycle(clampedTargetDutyCycle);
        return;
    }

    WriteFanDutyCycle(0.0f);
    const unsigned long startMs = millis();
    while (true)
    {
        const unsigned long elapsedMs = millis() - startMs;
        if (elapsedMs >= rampDurationMs)
        {
            break;
        }

        WriteFanDutyCycle(MazeMap::ComputeFanRampDutyCycle(clampedTargetDutyCycle, elapsedMs, rampDurationMs));
        delay((std::min)(kFanRampStepMs, rampDurationMs - elapsedMs));
    }

    WriteFanDutyCycle(clampedTargetDutyCycle);
#else
    (void)targetDutyCycle;
#endif
}

inline void SetMissionLevelFanEnabled(bool enabled)
{
    if (enabled)
    {
        RampFanDutyCycle(MazeMap::Config::kRacingFanDutyCycle);
        return;
    }

    WriteFanDutyCycle(0.0f);
}

inline float ReachableSpeedWithBoundary(float boundarySpeed, float distance, float accel)
{
    if (accel <= 0.0f)
    {
        return (std::max)(boundarySpeed, 0.0f);
    }

    return MazeMap::LinearKinematics::V1IgnoringT((std::max)(distance, 0.0f), boundarySpeed, accel);
}

inline Eigen::Vector2f RightUnitFromHeading(const Eigen::Vector2f& headingUnit)
{
    return Eigen::Vector2f(headingUnit.y(), -headingUnit.x());
}

inline Eigen::Vector2f RotateBodyVectorToWorld(
    const MazeMap::VehicleState& state,
    const Eigen::Vector2f& bodyVector)
{
    const Eigen::Vector2f headingUnit = state.GetHeadingUnit();
    const Eigen::Vector2f rightUnit = RightUnitFromHeading(headingUnit);
    return Eigen::Vector2f(
        (rightUnit.x() * bodyVector.x()) + (headingUnit.x() * bodyVector.y()),
        (rightUnit.y() * bodyVector.x()) + (headingUnit.y() * bodyVector.y()));
}

inline Eigen::Vector2f SensorWorldPosition(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor)
{
    const Eigen::Vector2f worldOffset = RotateBodyVectorToWorld(state, sensor.GetPosition());
    return Eigen::Vector2f(state.GetPositionX() + worldOffset.x(), state.GetPositionY() + worldOffset.y());
}

inline Eigen::Vector2f SensorWorldFacing(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor)
{
    return RotateBodyVectorToWorld(state, sensor.GetFacingDirection());
}

inline bool TryDistanceToWestWall(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    if (sensorFacing.x() >= -0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (westWallXM - sensorPosition.x()) / sensorFacing.x();
    const float intersectionY = sensorPosition.y() + (candidateDistanceM * sensorFacing.y());
    if (candidateDistanceM <= 0.0f || intersectionY < (southWallYM - 0.005f) || intersectionY > (northWallYM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToEastWall(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    if (sensorFacing.x() <= 0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (eastWallXM - sensorPosition.x()) / sensorFacing.x();
    const float intersectionY = sensorPosition.y() + (candidateDistanceM * sensorFacing.y());
    if (candidateDistanceM <= 0.0f || intersectionY < (southWallYM - 0.005f) || intersectionY > (northWallYM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToSouthWall(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    if (sensorFacing.y() >= -0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (southWallYM - sensorPosition.y()) / sensorFacing.y();
    const float intersectionX = sensorPosition.x() + (candidateDistanceM * sensorFacing.x());
    if (candidateDistanceM <= 0.0f || intersectionX < (westWallXM - 0.005f) || intersectionX > (eastWallXM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToNorthWall(const MazeMap::VehicleState& state, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    if (sensorFacing.y() <= 0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (northWallYM - sensorPosition.y()) / sensorFacing.y();
    const float intersectionX = sensorPosition.x() + (candidateDistanceM * sensorFacing.x());
    if (candidateDistanceM <= 0.0f || intersectionX < (westWallXM - 0.005f) || intersectionX > (eastWallXM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryComputeNearestStartCellWallDistanceM(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    float& distanceM)
{
    distanceM = 0.0f;
    float bestDistanceM = INFINITY;
    float candidateDistanceM = 0.0f;
    if (TryDistanceToWestWall(state, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }
    if (TryDistanceToEastWall(state, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }
    if (TryDistanceToSouthWall(state, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }

    if (!std::isfinite(bestDistanceM) || !(bestDistanceM > 0.0f))
    {
        return false;
    }

    distanceM = bestDistanceM;
    return true;
}

inline bool TryComputeEffectiveTurnRadiusM(
    float leftDistanceM,
    float rightDistanceM,
    float yawChangeRad,
    float& turnRadiusM)
{
    if (!std::isfinite(leftDistanceM) ||
        !std::isfinite(rightDistanceM) ||
        !std::isfinite(yawChangeRad) ||
        std::fabs(yawChangeRad) < 1.0e-4f)
    {
        turnRadiusM = 0.0f;
        return false;
    }

    turnRadiusM = std::fabs(0.5f * (leftDistanceM + rightDistanceM) / yawChangeRad);
    return std::isfinite(turnRadiusM) && (turnRadiusM > 0.0f);
}

inline bool TryGetCellCenterMeters(const MazeMap::CellCoordinates& cell, float& xMeters, float& yMeters)
{
    MazeMap::MazeLocation::CellCenter(cell).GetPhysicalLocation(MazeMap::Config::kCellSizeM, xMeters, yMeters);
    return std::isfinite(xMeters) && std::isfinite(yMeters);
}

inline bool TryGetCellWallFaceCoordinateM(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    const float cellBaseXM = static_cast<float>(cell.GetX()) * MazeMap::Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * MazeMap::Config::kCellSizeM;
    switch (wallDirection)
    {
    case MazeMap::Left:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Right:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Down:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Up:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
        return true;
    default:
        return false;
    }
}

inline bool TryComputeDistanceToCellWallM(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& distanceM)
{
    distanceM = 0.0f;

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float cellBaseXM = static_cast<float>(cell.GetX()) * MazeMap::Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * MazeMap::Config::kCellSizeM;
    const float cellInnerMinXM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMaxXM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMinYM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMaxYM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);

    switch (wallDirection)
    {
    case MazeMap::Left:
    case MazeMap::Right:
    {
        if (((wallDirection == MazeMap::Left) && sensorFacing.x() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.x() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.x()) / sensorFacing.x();
        const float intersectionYM = sensorPosition.y() + (candidateDistanceM * sensorFacing.y());
        if (candidateDistanceM <= 0.0f ||
            intersectionYM < (cellInnerMinYM - 0.005f) ||
            intersectionYM > (cellInnerMaxYM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    case MazeMap::Down:
    case MazeMap::Up:
    {
        if (((wallDirection == MazeMap::Down) && sensorFacing.y() >= -0.1f) ||
            ((wallDirection == MazeMap::Up) && sensorFacing.y() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.y()) / sensorFacing.y();
        const float intersectionXM = sensorPosition.x() + (candidateDistanceM * sensorFacing.x());
        if (candidateDistanceM <= 0.0f ||
            intersectionXM < (cellInnerMinXM - 0.005f) ||
            intersectionXM > (cellInnerMaxXM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    default:
        return false;
    }
}

inline bool TryComputeFrontWallCandidateDistancesForPose(
    const MazeMap::VehicleState& state,
    const MazeMap::Vehicle& vehicle,
    const MazeMap::CellCoordinates& observedCell,
    MazeMap::Direction observedDirection,
    float& frontLeftDistanceM,
    float& frontRightDistanceM)
{
    frontLeftDistanceM = NAN;
    frontRightDistanceM = NAN;
    const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
    const bool haveFrontLeftDistance =
        TryComputeDistanceToCellWallM(
            state,
            vehicle.FrontLeft,
            observedCell,
            forwardDirection,
            frontLeftDistanceM);
    const bool haveFrontRightDistance =
        TryComputeDistanceToCellWallM(
            state,
            vehicle.FrontRight,
            observedCell,
            forwardDirection,
            frontRightDistanceM);
    return haveFrontLeftDistance || haveFrontRightDistance;
}

inline bool TryComputeFrontWallObservationSampleDistanceM(
    const MazeMap::Vehicle& vehicle,
    const MazeMap::WallSensor& sensor,
    uint8_t sampleIndex,
    float& distanceM)
{
    distanceM = 0.0f;

    float poseXM = 0.0f;
    float poseYM = 0.0f;
    const float sideSensorForwardOffsetM =
        (std::max)(vehicle.SideLeft.GetPosition().y(), vehicle.SideRight.GetPosition().y());
    const MazeMap::CellCoordinates observedCell(0, 0);
    if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
            observedCell,
            MazeMap::Up,
            MazeMap::Config::kCellSizeM,
            MazeMap::Config::kMazeWallThicknessM,
            sideSensorForwardOffsetM,
            MazeMap::Config::kSideWallSegmentCenterFraction,
            sampleIndex,
            MazeMap::Config::kSearchRollingObservationSampleCount,
            poseXM,
            poseYM))
    {
        return false;
    }

    MazeMap::VehicleState state{};
    state.SetPosition(Eigen::Vector2f(poseXM, poseYM));
    state.SetOrientation(DirectionToYawRad(MazeMap::Up));
    state.SetVelocity(0.0f);
    state.SetRotationalVelocity(0.0f);
    return TryComputeDistanceToCellWallM(state, sensor, observedCell, MazeMap::Up, distanceM);
}

inline bool TryComputeFrontWallObservationThresholdDistancesM(
    const MazeMap::Vehicle& vehicle,
    WallSensorId sensorId,
    float releaseHysteresisDistanceM,
    float& onThresholdM,
    float& offThresholdM)
{
    onThresholdM = 0.0f;
    offThresholdM = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const MazeMap::WallSensor& sensor =
        (sensorId == WallSensorId::FrontLeft) ?
        vehicle.FrontLeft :
        vehicle.FrontRight;
    constexpr uint8_t kLatchSampleIndex =
        MazeMap::Config::kSearchRollingObservationSampleCount - MazeMap::Config::kSearchRollingObservationMajorityCount;
    float preferredOnThresholdM = 0.0f;
    float farthestObservationThresholdM = 0.0f;
    if (!TryComputeFrontWallObservationSampleDistanceM(
            vehicle,
            sensor,
            kLatchSampleIndex,
            preferredOnThresholdM) ||
        !TryComputeFrontWallObservationSampleDistanceM(
            vehicle,
            sensor,
            0U,
            farthestObservationThresholdM))
    {
        return false;
    }

    float preferredOffThresholdM = 0.0f;
    if (!MazeMap::TryExpandWallThresholdDistanceM(
            preferredOnThresholdM,
            releaseHysteresisDistanceM,
            preferredOffThresholdM))
    {
        return false;
    }

    if (std::isfinite(farthestObservationThresholdM) &&
        farthestObservationThresholdM > preferredOnThresholdM &&
        preferredOffThresholdM > farthestObservationThresholdM)
    {
        preferredOffThresholdM = farthestObservationThresholdM;
    }

    return MazeMap::TryClampWallThresholdDistanceRangeM(
        preferredOnThresholdM,
        preferredOffThresholdM,
        MazeMap::Config::kFrontWallOnThresholdM,
        MazeMap::Config::kFrontWallOffThresholdM,
        onThresholdM,
        offThresholdM);
}

inline bool TryComputeWallTouchTargetCoordinateForCellWall(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& targetCoordinateM,
    CalibrationWall& calibrationWall)
{
    targetCoordinateM = 0.0f;
    float wallFaceCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallFaceCoordinateM))
    {
        return false;
    }

    switch (wallDirection)
    {
    case MazeMap::Left:
        calibrationWall = CalibrationWall::West;
        targetCoordinateM = wallFaceCoordinateM + MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Right:
        calibrationWall = CalibrationWall::East;
        targetCoordinateM = wallFaceCoordinateM - MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Down:
        calibrationWall = CalibrationWall::South;
        targetCoordinateM = wallFaceCoordinateM + MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Up:
        calibrationWall = CalibrationWall::North;
        targetCoordinateM = wallFaceCoordinateM - MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    default:
        return false;
    }
}

inline bool TryComputePoseAxisFromObservedWall(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    float measuredDistanceM,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    if (!std::isfinite(measuredDistanceM) || measuredDistanceM <= 0.0f)
    {
        return false;
    }

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const Eigen::Vector2f worldOffset = RotateBodyVectorToWorld(state, sensor.GetPosition());
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    if (wallDirection == MazeMap::Left || wallDirection == MazeMap::Right)
    {
        if (!std::isfinite(worldOffset.x()) ||
            !std::isfinite(sensorFacing.x()) ||
            ((wallDirection == MazeMap::Left) && sensorFacing.x() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.x() <= 0.1f))
        {
            return false;
        }

        coordinateM = wallCoordinateM - worldOffset.x() - (measuredDistanceM * sensorFacing.x());
        return std::isfinite(coordinateM);
    }

    if (!std::isfinite(worldOffset.y()) ||
        !std::isfinite(sensorFacing.y()) ||
        ((wallDirection == MazeMap::Down) && sensorFacing.y() >= -0.1f) ||
        ((wallDirection == MazeMap::Up) && sensorFacing.y() <= 0.1f))
    {
        return false;
    }

    coordinateM = wallCoordinateM - worldOffset.y() - (measuredDistanceM * sensorFacing.y());
    return std::isfinite(coordinateM);
}

inline uint32_t WallSensorAmbientSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

inline uint32_t WallSensorLitSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

inline uint32_t WallSensorLedCalibrationHalfPeriodUs(WallSensorId sensorId)
{
    return (std::max)(WallSensorAmbientSettleTimeUs(sensorId), WallSensorLitSettleTimeUs(sensorId));
}

inline float WallSensorMeasuredValueForCalibration(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    return IsFrontWallSensor(sensorId) ? sample.differentialLight : sample.rawDistanceM;
}

inline WallSensorCalibrationInput BuildWallSensorCalibrationInput(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    WallSensorCalibrationInput input{};
    input.measuredValue = WallSensorMeasuredValueForCalibration(sensorId, sample);
    input.fallbackDistanceM = sample.rawDistanceM;
    input.differentialLight = sample.differentialLight;
    input.ambientLight = sample.ambientLight;
    input.litLight = sample.litLight;
    input.timing = sample.timing;
    return input;
}

struct AsyncWallSensorPairRead
{
    WallSensorId firstSensorId = WallSensorId::FrontLeft;
    WallSensorId secondSensorId = WallSensorId::FrontRight;
    const MazeMap::WallSensor* firstSensor = nullptr;
    const MazeMap::WallSensor* secondSensor = nullptr;
    RawWallSensorSample firstSample{};
    RawWallSensorSample secondSample{};
    uint32_t litReadyUs = 0UL;
    bool active = false;
};

enum class AsyncWallSensorSweepStage : uint8_t
{
    Front = 0U,
    Left = 1U,
    Right = 2U,
    Complete = 3U
};

struct AsyncWallSensorSweepRead
{
    const MazeMap::WallSensor* frontLeftSensor = nullptr;
    const MazeMap::WallSensor* frontRightSensor = nullptr;
    const MazeMap::WallSensor* sideLeftSensor = nullptr;
    const MazeMap::WallSensor* sideRightSensor = nullptr;
    RawWallSensorSample frontLeftSample{};
    RawWallSensorSample frontRightSample{};
    RawWallSensorSample sideLeftSample{};
    RawWallSensorSample sideRightSample{};
    uint32_t nextFrontLeftLedOffCommandUs = 0UL;
    uint32_t nextFrontRightLedOffCommandUs = 0UL;
    uint32_t nextSideLeftLedOffCommandUs = 0UL;
    uint32_t nextSideRightLedOffCommandUs = 0UL;
    uint32_t latestLedOffUs = 0UL;
    uint32_t stageReadyUs = 0UL;
    AsyncWallSensorSweepStage stage = AsyncWallSensorSweepStage::Complete;
    bool active = false;
};

inline bool HasAsyncWallSensorPairSettled(const AsyncWallSensorPairRead& read, uint32_t nowUs) noexcept
{
    return !read.active || (static_cast<int32_t>(nowUs - read.litReadyUs) >= 0);
}

inline void StartAsyncWallSensorPairRead(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    AsyncWallSensorPairRead& read) noexcept
{
    read = AsyncWallSensorPairRead{};
    read.firstSensorId = firstSensorId;
    read.secondSensorId = secondSensorId;
    read.firstSensor = &firstSensor;
    read.secondSensor = &secondSensor;

    const uint32_t ledOffCommandUs = micros();
    firstSensor.SetLedEnabled(false);
    secondSensor.SetLedEnabled(false);
    read.firstSample.timing.ledOffCommandUs = ledOffCommandUs;
    read.secondSample.timing.ledOffCommandUs = ledOffCommandUs;

    read.firstSample.ambientAdcCode = firstSensor.ReadAdcCode();
    read.secondSample.ambientAdcCode = secondSensor.ReadAdcCode();
    const uint32_t ambientSampleUs = micros();
    read.firstSample.timing.adcOffSampleUs = ambientSampleUs;
    read.secondSample.timing.adcOffSampleUs = ambientSampleUs;
    read.firstSample.ambientLight = firstSensor.AdcCodeToLightLevel(read.firstSample.ambientAdcCode);
    read.secondSample.ambientLight = secondSensor.AdcCodeToLightLevel(read.secondSample.ambientAdcCode);

    const uint32_t ledOnCommandUs = micros();
    read.firstSample.timing.ledOnCommandUs = ledOnCommandUs;
    read.secondSample.timing.ledOnCommandUs = ledOnCommandUs;
    firstSensor.SetLedEnabled(true);
    secondSensor.SetLedEnabled(true);
    read.litReadyUs =
        ledOnCommandUs +
        (std::max)(WallSensorLitSettleTimeUs(firstSensorId), WallSensorLitSettleTimeUs(secondSensorId));
    read.active = true;
}

inline bool TryCompleteAsyncWallSensorPairRead(AsyncWallSensorPairRead& read) noexcept
{
    if (!read.active)
    {
        return true;
    }

    if (!HasAsyncWallSensorPairSettled(read, micros()))
    {
        return false;
    }

    read.firstSample.litAdcCode = read.firstSensor->ReadAdcCode();
    read.secondSample.litAdcCode = read.secondSensor->ReadAdcCode();
    const uint32_t litSampleUs = micros();
    read.firstSample.timing.adcOnSampleUs = litSampleUs;
    read.secondSample.timing.adcOnSampleUs = litSampleUs;
    read.firstSample.litLight = read.firstSensor->AdcCodeToLightLevel(read.firstSample.litAdcCode);
    read.secondSample.litLight = read.secondSensor->AdcCodeToLightLevel(read.secondSample.litAdcCode);

    read.firstSample.differentialLight =
        MazeMap::WallSensor::DifferentialLightLevel(read.firstSample.ambientLight, read.firstSample.litLight);
    read.secondSample.differentialLight =
        MazeMap::WallSensor::DifferentialLightLevel(read.secondSample.ambientLight, read.secondSample.litLight);
    read.firstSample.rawDistanceM = read.firstSensor->DistanceFromDifferentialLight(read.firstSample.differentialLight);
    read.secondSample.rawDistanceM = read.secondSensor->DistanceFromDifferentialLight(read.secondSample.differentialLight);

    read.firstSensor->SetLedEnabled(false);
    read.secondSensor->SetLedEnabled(false);
    const uint32_t observationReadyUs = micros();
    read.firstSample.timing.observationReadyUs = observationReadyUs;
    read.secondSample.timing.observationReadyUs = observationReadyUs;
    read.active = false;
    return true;
}

inline void CompleteAsyncWallSensorPairRead(AsyncWallSensorPairRead& read) noexcept
{
    while (!TryCompleteAsyncWallSensorPairRead(read))
    {
        delayMicroseconds(5);
    }
}

inline void PrimeAsyncWallSensorDarkSample(
    uint32_t ledOffCommandUs,
    const MazeMap::WallSensor& sensor,
    RawWallSensorSample& sample) noexcept
{
    sample = RawWallSensorSample{};
    sample.timing.ledOffCommandUs = ledOffCommandUs;
    sample.ambientAdcCode = sensor.ReadAdcCode();
    sample.timing.adcOffSampleUs = micros();
    sample.ambientLight = sensor.AdcCodeToLightLevel(sample.ambientAdcCode);
}

inline void FinalizeAsyncWallSensorLitSample(
    const MazeMap::WallSensor& sensor,
    RawWallSensorSample& sample) noexcept
{
    sample.litAdcCode = sensor.ReadAdcCode();
    sample.timing.adcOnSampleUs = micros();
    sample.litLight = sensor.AdcCodeToLightLevel(sample.litAdcCode);
    sample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(sample.ambientLight, sample.litLight);
    sample.rawDistanceM = sensor.DistanceFromDifferentialLight(sample.differentialLight);
}

inline void StartAsyncWallSensorSweepRead(
    const MazeMap::WallSensor& frontLeft,
    uint32_t frontLeftLedOffCommandUs,
    const MazeMap::WallSensor& frontRight,
    uint32_t frontRightLedOffCommandUs,
    const MazeMap::WallSensor& sideLeft,
    uint32_t sideLeftLedOffCommandUs,
    const MazeMap::WallSensor& sideRight,
    uint32_t sideRightLedOffCommandUs,
    AsyncWallSensorSweepRead& read) noexcept
{
    read = AsyncWallSensorSweepRead{};
    read.frontLeftSensor = &frontLeft;
    read.frontRightSensor = &frontRight;
    read.sideLeftSensor = &sideLeft;
    read.sideRightSensor = &sideRight;
    read.nextFrontLeftLedOffCommandUs = frontLeftLedOffCommandUs;
    read.nextFrontRightLedOffCommandUs = frontRightLedOffCommandUs;
    read.nextSideLeftLedOffCommandUs = sideLeftLedOffCommandUs;
    read.nextSideRightLedOffCommandUs = sideRightLedOffCommandUs;
    read.latestLedOffUs =
        (std::max)(
            (std::max)(frontLeftLedOffCommandUs, frontRightLedOffCommandUs),
            (std::max)(sideLeftLedOffCommandUs, sideRightLedOffCommandUs));

    PrimeAsyncWallSensorDarkSample(frontLeftLedOffCommandUs, frontLeft, read.frontLeftSample);
    PrimeAsyncWallSensorDarkSample(frontRightLedOffCommandUs, frontRight, read.frontRightSample);
    PrimeAsyncWallSensorDarkSample(sideLeftLedOffCommandUs, sideLeft, read.sideLeftSample);
    PrimeAsyncWallSensorDarkSample(sideRightLedOffCommandUs, sideRight, read.sideRightSample);

    const uint32_t frontLedOnCommandUs = micros();
    read.frontLeftSample.timing.ledOnCommandUs = frontLedOnCommandUs;
    read.frontRightSample.timing.ledOnCommandUs = frontLedOnCommandUs;
    frontLeft.SetLedEnabled(true);
    frontRight.SetLedEnabled(true);
    read.stageReadyUs =
        frontLedOnCommandUs +
        (std::max)(
            WallSensorLitSettleTimeUs(WallSensorId::FrontLeft),
            WallSensorLitSettleTimeUs(WallSensorId::FrontRight));
    read.stage = AsyncWallSensorSweepStage::Front;
    read.active = true;
}

inline bool ServiceAsyncWallSensorSweepRead(AsyncWallSensorSweepRead& read) noexcept
{
    if (!read.active)
    {
        return true;
    }

    const uint32_t nowUs = micros();
    if (static_cast<int32_t>(nowUs - read.stageReadyUs) < 0)
    {
        return false;
    }

    switch (read.stage)
    {
    case AsyncWallSensorSweepStage::Front:
    {
        FinalizeAsyncWallSensorLitSample(*read.frontLeftSensor, read.frontLeftSample);
        FinalizeAsyncWallSensorLitSample(*read.frontRightSensor, read.frontRightSample);
        const uint32_t ledOffCommandUs = micros();
        read.frontLeftSensor->SetLedEnabled(false);
        read.frontRightSensor->SetLedEnabled(false);
        read.frontLeftSample.timing.observationReadyUs = ledOffCommandUs;
        read.frontRightSample.timing.observationReadyUs = ledOffCommandUs;
        read.nextFrontLeftLedOffCommandUs = ledOffCommandUs;
        read.nextFrontRightLedOffCommandUs = ledOffCommandUs;
        read.latestLedOffUs = ledOffCommandUs;

        const uint32_t leftLedOnCommandUs = micros();
        read.sideLeftSample.timing.ledOnCommandUs = leftLedOnCommandUs;
        read.sideLeftSensor->SetLedEnabled(true);
        read.stageReadyUs = leftLedOnCommandUs + WallSensorLitSettleTimeUs(WallSensorId::SideLeft);
        read.stage = AsyncWallSensorSweepStage::Left;
        return false;
    }

    case AsyncWallSensorSweepStage::Left:
    {
        FinalizeAsyncWallSensorLitSample(*read.sideLeftSensor, read.sideLeftSample);
        const uint32_t ledOffCommandUs = micros();
        read.sideLeftSensor->SetLedEnabled(false);
        read.sideLeftSample.timing.observationReadyUs = ledOffCommandUs;
        read.nextSideLeftLedOffCommandUs = ledOffCommandUs;
        read.latestLedOffUs = ledOffCommandUs;

        const uint32_t rightLedOnCommandUs = micros();
        read.sideRightSample.timing.ledOnCommandUs = rightLedOnCommandUs;
        read.sideRightSensor->SetLedEnabled(true);
        read.stageReadyUs = rightLedOnCommandUs + WallSensorLitSettleTimeUs(WallSensorId::SideRight);
        read.stage = AsyncWallSensorSweepStage::Right;
        return false;
    }

    case AsyncWallSensorSweepStage::Right:
    {
        FinalizeAsyncWallSensorLitSample(*read.sideRightSensor, read.sideRightSample);
        const uint32_t ledOffCommandUs = micros();
        read.sideRightSensor->SetLedEnabled(false);
        read.sideRightSample.timing.observationReadyUs = ledOffCommandUs;
        read.nextSideRightLedOffCommandUs = ledOffCommandUs;
        read.latestLedOffUs = ledOffCommandUs;
        read.stage = AsyncWallSensorSweepStage::Complete;
        read.active = false;
        return true;
    }

    case AsyncWallSensorSweepStage::Complete:
    default:
        read.active = false;
        return true;
    }
}

inline void AwaitAsyncWallSensorSweepRead(AsyncWallSensorSweepRead& read) noexcept
{
    while (read.active)
    {
        if (ServiceAsyncWallSensorSweepRead(read))
        {
            return;
        }

        const int32_t remainingUs = static_cast<int32_t>(read.stageReadyUs - micros());
        if (remainingUs > 0)
        {
            delayMicroseconds(static_cast<unsigned int>(remainingUs));
        }
    }
}

inline void AbortAsyncWallSensorSweepRead(AsyncWallSensorSweepRead& read) noexcept
{
    if (!read.active)
    {
        return;
    }

    const uint32_t ledOffCommandUs = micros();
    switch (read.stage)
    {
    case AsyncWallSensorSweepStage::Front:
        read.frontLeftSensor->SetLedEnabled(false);
        read.frontRightSensor->SetLedEnabled(false);
        read.nextFrontLeftLedOffCommandUs = ledOffCommandUs;
        read.nextFrontRightLedOffCommandUs = ledOffCommandUs;
        break;

    case AsyncWallSensorSweepStage::Left:
        read.sideLeftSensor->SetLedEnabled(false);
        read.nextSideLeftLedOffCommandUs = ledOffCommandUs;
        break;

    case AsyncWallSensorSweepStage::Right:
        read.sideRightSensor->SetLedEnabled(false);
        read.nextSideRightLedOffCommandUs = ledOffCommandUs;
        break;

    case AsyncWallSensorSweepStage::Complete:
    default:
        break;
    }

    read.latestLedOffUs = ledOffCommandUs;
    read.stage = AsyncWallSensorSweepStage::Complete;
    read.active = false;
}

inline RawWallSensorSample SampleWallSensorRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    RawWallSensorSample sample{};
    sample.timing.ledOffCommandUs = micros();
    sensor.SetLedEnabled(false);
    delayMicroseconds(WallSensorAmbientSettleTimeUs(sensorId));
    sample.ambientAdcCode = sensor.ReadAdcCode();
    sample.timing.adcOffSampleUs = micros();
    sample.ambientLight = sensor.AdcCodeToLightLevel(sample.ambientAdcCode);

    sample.timing.ledOnCommandUs = micros();
    sensor.SetLedEnabled(true);
    delayMicroseconds(WallSensorLitSettleTimeUs(sensorId));
    sample.litAdcCode = sensor.ReadAdcCode();
    sample.timing.adcOnSampleUs = micros();
    sample.litLight = sensor.AdcCodeToLightLevel(sample.litAdcCode);
    sample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(sample.ambientLight, sample.litLight);
    sample.rawDistanceM = sensor.DistanceFromDifferentialLight(sample.differentialLight);
    sample.timing.observationReadyUs = micros();
    sensor.SetLedEnabled(false);
    return sample;
}

inline void SampleWallSensorPairRaw(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    RawWallSensorSample& firstSample,
    RawWallSensorSample& secondSample)
{
    firstSensor.SetLedEnabled(false);
    secondSensor.SetLedEnabled(false);
    firstSample.timing.ledOffCommandUs = micros();
    secondSample.timing.ledOffCommandUs = firstSample.timing.ledOffCommandUs;
    delayMicroseconds((std::max)(WallSensorAmbientSettleTimeUs(firstSensorId), WallSensorAmbientSettleTimeUs(secondSensorId)));
    firstSample.ambientAdcCode = firstSensor.ReadAdcCode();
    secondSample.ambientAdcCode = secondSensor.ReadAdcCode();
    firstSample.timing.adcOffSampleUs = micros();
    secondSample.timing.adcOffSampleUs = firstSample.timing.adcOffSampleUs;
    firstSample.ambientLight = firstSensor.AdcCodeToLightLevel(firstSample.ambientAdcCode);
    secondSample.ambientLight = secondSensor.AdcCodeToLightLevel(secondSample.ambientAdcCode);

    firstSample.timing.ledOnCommandUs = micros();
    secondSample.timing.ledOnCommandUs = firstSample.timing.ledOnCommandUs;
    firstSensor.SetLedEnabled(true);
    secondSensor.SetLedEnabled(true);
    delayMicroseconds((std::max)(WallSensorLitSettleTimeUs(firstSensorId), WallSensorLitSettleTimeUs(secondSensorId)));
    firstSample.litAdcCode = firstSensor.ReadAdcCode();
    secondSample.litAdcCode = secondSensor.ReadAdcCode();
    firstSample.litLight = firstSensor.AdcCodeToLightLevel(firstSample.litAdcCode);
    firstSample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(firstSample.ambientLight, firstSample.litLight);
    firstSample.rawDistanceM = firstSensor.DistanceFromDifferentialLight(firstSample.differentialLight);
    secondSample.litLight = secondSensor.AdcCodeToLightLevel(secondSample.litAdcCode);
    secondSample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(secondSample.ambientLight, secondSample.litLight);
    secondSample.rawDistanceM = secondSensor.DistanceFromDifferentialLight(secondSample.differentialLight);
    firstSample.timing.adcOnSampleUs = micros();
    secondSample.timing.adcOnSampleUs = firstSample.timing.adcOnSampleUs;
    firstSample.timing.observationReadyUs = micros();
    secondSample.timing.observationReadyUs = firstSample.timing.observationReadyUs;
    firstSensor.SetLedEnabled(false);
    secondSensor.SetLedEnabled(false);
}

inline WallSensorCalibrationInput SampleWallCalibrationInputRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    return BuildWallSensorCalibrationInput(sensorId, SampleWallSensorRaw(sensorId, sensor));
}

inline void SampleWallCalibrationInputRawPair(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    WallSensorCalibrationInput& firstInput,
    WallSensorCalibrationInput& secondInput)
{
    RawWallSensorSample firstSample{};
    RawWallSensorSample secondSample{};
    SampleWallSensorPairRaw(firstSensorId, firstSensor, secondSensorId, secondSensor, firstSample, secondSample);
    firstInput = BuildWallSensorCalibrationInput(firstSensorId, firstSample);
    secondInput = BuildWallSensorCalibrationInput(secondSensorId, secondSample);
}

inline WallSensorCalibrationCapture SampleWallCalibrationCaptureAverageRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    AveragedWallSensorInputWindow<static_cast<uint8_t>(MazeMap::Config::kWallCalibrationAverageSampleCount)> averageWindow{};
    std::array<float, MazeMap::Config::kWallCalibrationAverageSampleCount> differentialLightSamples{};
    uint16_t differentialLightCount = 0U;
    for (uint16_t i = 0U; i < MazeMap::Config::kWallCalibrationAverageSampleCount; ++i)
    {
        const WallSensorCalibrationInput input = SampleWallCalibrationInputRaw(sensorId, sensor);
        averageWindow.PushAndAverage(input);
        differentialLightSamples[differentialLightCount] = input.differentialLight;
        ++differentialLightCount;
    }

    WallSensorCalibrationCapture capture{};
    capture.input = averageWindow.Average();
    capture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        differentialLightSamples,
        differentialLightCount,
        MazeMap::Config::kWallCalibrationScaledMadMultiplier,
        capture.differentialLightBand.median,
        capture.differentialLightBand.low,
        capture.differentialLightBand.high);
    return capture;
}

inline void SampleWallCalibrationCaptureAverageRawPair(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    WallSensorCalibrationCapture& firstCapture,
    WallSensorCalibrationCapture& secondCapture)
{
    AveragedWallSensorInputWindow<static_cast<uint8_t>(MazeMap::Config::kWallCalibrationAverageSampleCount)> firstAverageWindow{};
    AveragedWallSensorInputWindow<static_cast<uint8_t>(MazeMap::Config::kWallCalibrationAverageSampleCount)> secondAverageWindow{};
    std::array<float, MazeMap::Config::kWallCalibrationAverageSampleCount> firstDifferentialLightSamples{};
    std::array<float, MazeMap::Config::kWallCalibrationAverageSampleCount> secondDifferentialLightSamples{};
    uint16_t differentialLightCount = 0U;
    for (uint16_t i = 0U; i < MazeMap::Config::kWallCalibrationAverageSampleCount; ++i)
    {
        WallSensorCalibrationInput firstInput{};
        WallSensorCalibrationInput secondInput{};
        SampleWallCalibrationInputRawPair(
            firstSensorId,
            firstSensor,
            secondSensorId,
            secondSensor,
            firstInput,
            secondInput);
        firstAverageWindow.PushAndAverage(firstInput);
        secondAverageWindow.PushAndAverage(secondInput);
        firstDifferentialLightSamples[differentialLightCount] = firstInput.differentialLight;
        secondDifferentialLightSamples[differentialLightCount] = secondInput.differentialLight;
        ++differentialLightCount;
    }

    firstCapture = {};
    firstCapture.input = firstAverageWindow.Average();
    firstCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        firstDifferentialLightSamples,
        differentialLightCount,
        MazeMap::Config::kWallCalibrationScaledMadMultiplier,
        firstCapture.differentialLightBand.median,
        firstCapture.differentialLightBand.low,
        firstCapture.differentialLightBand.high);

    secondCapture = {};
    secondCapture.input = secondAverageWindow.Average();
    secondCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        secondDifferentialLightSamples,
        differentialLightCount,
        MazeMap::Config::kWallCalibrationScaledMadMultiplier,
        secondCapture.differentialLightBand.median,
        secondCapture.differentialLightBand.low,
        secondCapture.differentialLightBand.high);
}

inline bool HysteresisWall(bool currentState, float distanceM, float onThresholdM, float offThresholdM)
{
    if (currentState)
    {
        return distanceM < offThresholdM;
    }
    return distanceM < onThresholdM;
}

inline bool IsApproximatelyDiagonalHeadingUnit(const Eigen::Vector2f& headingUnit)
{
    const float absX = std::fabs(headingUnit.x());
    const float absY = std::fabs(headingUnit.y());
    return absX > 0.5f && absY > 0.5f && std::fabs(absX - absY) <= 0.15f;
}

inline MazeMap::ManeuverCode RelativeToInPlaceCode(MazeMap::RelativeDirection rel)
{
    switch (rel)
    {
    case MazeMap::Left45:
        return MazeMap::IP45_M;
    case MazeMap::Right45:
        return MazeMap::IP45;
    case MazeMap::Left90:
        return MazeMap::IP90_M;
    case MazeMap::Right90:
        return MazeMap::IP90;
    case MazeMap::Left135:
        return MazeMap::IP135_M;
    case MazeMap::Right135:
        return MazeMap::IP135;
    case MazeMap::Reverse:
        return MazeMap::IP180;
    default:
        return MazeMap::MC_NONE;
    }
}

inline bool IsStraightCode(MazeMap::ManeuverCode code)
{
    return code != MazeMap::MC_NONE && code <= MazeMap::S31;
}

inline void TrimAsciiWhitespace(char* text)
{
    if (text == nullptr)
    {
        return;
    }

    char* start = text;
    while (*start != '\0' && isspace(static_cast<unsigned char>(*start)) != 0)
    {
        ++start;
    }

    if (start != text)
    {
        memmove(text, start, strlen(start) + 1U);
    }

    size_t length = strlen(text);
    while (length > 0U && isspace(static_cast<unsigned char>(text[length - 1U])) != 0)
    {
        text[length - 1U] = '\0';
        --length;
    }
}

inline void NormalizeToken(char* text)
{
    TrimAsciiWhitespace(text);
    if (text == nullptr)
    {
        return;
    }

    for (char* cursor = text; *cursor != '\0'; ++cursor)
    {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }
}

inline bool TryParseBaseManeuverCodeName(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr || token[0] == '\0')
    {
        return false;
    }

    if (token[0] == 'S' && isdigit(static_cast<unsigned char>(token[1])) != 0)
    {
        char* end = nullptr;
        const long value = strtol(token + 1, &end, 10);
        if (end != nullptr && *end == '\0' && value >= 1L && value <= 31L)
        {
            code = static_cast<MazeMap::ManeuverCode>(value);
            return true;
        }
    }

    struct NamedCode
    {
        const char* name;
        MazeMap::ManeuverCode code;
    };

    static const NamedCode kNamedCodes[] = {
        { "IP45", MazeMap::IP45 },
        { "IP90", MazeMap::IP90 },
        { "IP135", MazeMap::IP135 },
        { "IP180", MazeMap::IP180 },
        { "S45SS", MazeMap::S45SS },
        { "S45SD", MazeMap::S45SD },
        { "S45LS", MazeMap::S45LS },
        { "S45LD", MazeMap::S45LD },
        { "S90SS", MazeMap::S90SS },
        { "S90SD", MazeMap::S90SD },
        { "S90LS", MazeMap::S90LS },
        { "S90LD", MazeMap::S90LD },
        { "S135SS", MazeMap::S135SS },
        { "S135SD", MazeMap::S135SD },
        { "S135LS", MazeMap::S135LS },
        { "S135LD", MazeMap::S135LD },
        { "S180SS", MazeMap::S180SS },
        { "S180LS", MazeMap::S180LS },
        { "S90ELD", MazeMap::S90ELD },
        { "S180ELS", MazeMap::S180ELS },
    };

    for (const NamedCode& entry : kNamedCodes)
    {
        if (strcmp(token, entry.name) == 0)
        {
            code = entry.code;
            return true;
        }
    }

    return false;
}

inline bool TryParseManeuverCodeToken(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr)
    {
        return false;
    }

    char normalized[24] = {};
    snprintf(normalized, sizeof(normalized), "%s", token);
    NormalizeToken(normalized);
    if (normalized[0] == '\0')
    {
        return false;
    }

    char* numericEnd = nullptr;
    const long numericValue = strtol(normalized, &numericEnd, 0);
    if (numericEnd != nullptr && *numericEnd == '\0' && numericValue >= 0L && numericValue <= 255L)
    {
        code = static_cast<MazeMap::ManeuverCode>(numericValue);
        return true;
    }

    const size_t length = strlen(normalized);
    if (length > 2U && strcmp(normalized + length - 2U, "_M") == 0)
    {
        normalized[length - 2U] = '\0';
        MazeMap::ManeuverCode baseCode = MazeMap::MC_NONE;
        if (!TryParseBaseManeuverCodeName(normalized, baseCode))
        {
            return false;
        }

        code = static_cast<MazeMap::ManeuverCode>(
            static_cast<uint8_t>(baseCode) |
            static_cast<uint8_t>(MazeMap::MIRRORED_MANEUVER_FLAG));
        return true;
    }

    return TryParseBaseManeuverCodeName(normalized, code);
}

inline void FormatManeuverCodeName(MazeMap::ManeuverCode code, char* buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0U)
    {
        return;
    }

    const bool mirrored = (code & MazeMap::MIRRORED_MANEUVER_FLAG) == MazeMap::MIRRORED_MANEUVER_FLAG;
    const MazeMap::ManeuverCode baseCode = code & MazeMap::INVERTED_MIRRORED_MANEUVER_FLAG;

    if (baseCode >= MazeMap::S1 && baseCode <= MazeMap::S31)
    {
        snprintf(
            buffer,
            bufferSize,
            "S%u%s",
            static_cast<unsigned>(static_cast<uint8_t>(baseCode)),
            mirrored ? "_M" : "");
        return;
    }

    const char* name = "UNKNOWN";
    switch (baseCode)
    {
    case MazeMap::IP45: name = "IP45"; break;
    case MazeMap::IP90: name = "IP90"; break;
    case MazeMap::IP135: name = "IP135"; break;
    case MazeMap::IP180: name = "IP180"; break;
    case MazeMap::S45SS: name = "S45SS"; break;
    case MazeMap::S45SD: name = "S45SD"; break;
    case MazeMap::S45LS: name = "S45LS"; break;
    case MazeMap::S45LD: name = "S45LD"; break;
    case MazeMap::S90SS: name = "S90SS"; break;
    case MazeMap::S90SD: name = "S90SD"; break;
    case MazeMap::S90LS: name = "S90LS"; break;
    case MazeMap::S90LD: name = "S90LD"; break;
    case MazeMap::S135SS: name = "S135SS"; break;
    case MazeMap::S135SD: name = "S135SD"; break;
    case MazeMap::S135LS: name = "S135LS"; break;
    case MazeMap::S135LD: name = "S135LD"; break;
    case MazeMap::S180SS: name = "S180SS"; break;
    case MazeMap::S180LS: name = "S180LS"; break;
    case MazeMap::S90ELD: name = "S90ELD"; break;
    case MazeMap::S180ELS: name = "S180ELS"; break;
    default: break;
    }

    snprintf(buffer, bufferSize, "%s%s", name, mirrored ? "_M" : "");
}

inline float ReadBackLeftGyroZRadpsRaw(MazeMap::Vehicle& vehicle)
{
#if defined(ARDUINO_TEENSY41)
    const float blDps = vehicle.IMU_BL.ReadClockwiseYawDps();
    return blDps * DEG_TO_RAD_F;
#else
    (void)vehicle;
    return 0.0f;
#endif
}

inline float EstimateMissionGyroBiasRadps(MazeMap::Vehicle& vehicle)
{
    float accumulatedRadps = 0.0f;
    constexpr unsigned long kBiasSampleIntervalMs = 2UL;
    const unsigned long requiredSamples = MazeMap::ComputeGyroBiasSampleCount(
        static_cast<unsigned long>(MazeMap::Config::kGyroBiasSamples),
        kBiasSampleIntervalMs,
        static_cast<unsigned long>(MazeMap::Config::kGyroBiasMinimumAveragingWindowMs));
    for (unsigned long i = 0UL; i < requiredSamples; ++i)
    {
        accumulatedRadps += ReadBackLeftGyroZRadpsRaw(vehicle);
        delay(kBiasSampleIntervalMs);
    }
    return accumulatedRadps / static_cast<float>(requiredSamples);
}

inline bool TryComputeSideWallAimCoordinateM(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    float& alongWallCoordinateM)
{
    alongWallCoordinateM = 0.0f;
    if (!std::isfinite(state.GetPositionX()) ||
        !std::isfinite(state.GetPositionY()) ||
        !std::isfinite(state.GetOrientation()))
    {
        return false;
    }

    const Eigen::Vector2f sensorPosition = SensorWorldPosition(state, sensor);
    const Eigen::Vector2f sensorFacing = SensorWorldFacing(state, sensor);
    const float sensorXM = sensorPosition.x();
    const float sensorYM = sensorPosition.y();
    const float facingXM = sensorFacing.x();
    const float facingYM = sensorFacing.y();
    const float innerMinCoordinateM =
        MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float innerMaxCoordinateM =
        MazeMap::ComputeCellInnerMaxCoordinateM(
            MazeMap::Config::kCellSizeM,
            MazeMap::Config::kMazeWallThicknessM);

    if (std::fabs(facingXM) >= std::fabs(facingYM))
    {
        if (!(std::fabs(facingXM) > 1.0e-4f))
        {
            return false;
        }

        const float cellBaseXM =
            std::floor(sensorXM / MazeMap::Config::kCellSizeM) * MazeMap::Config::kCellSizeM;
        const float wallFaceXM =
            (facingXM >= 0.0f) ?
            (cellBaseXM + innerMaxCoordinateM) :
            (cellBaseXM + innerMinCoordinateM);
        const float rayScale = (wallFaceXM - sensorXM) / facingXM;
        if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
        {
            return false;
        }

        alongWallCoordinateM = sensorYM + (rayScale * facingYM);
        return std::isfinite(alongWallCoordinateM);
    }

    if (!(std::fabs(facingYM) > 1.0e-4f))
    {
        return false;
    }

    const float cellBaseYM =
        std::floor(sensorYM / MazeMap::Config::kCellSizeM) * MazeMap::Config::kCellSizeM;
    const float wallFaceYM =
        (facingYM >= 0.0f) ?
        (cellBaseYM + innerMaxCoordinateM) :
        (cellBaseYM + innerMinCoordinateM);
    const float rayScale = (wallFaceYM - sensorYM) / facingYM;
    if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
    {
        return false;
    }

    alongWallCoordinateM = sensorXM + (rayScale * facingXM);
    return std::isfinite(alongWallCoordinateM);
}

inline bool IsSideWallDetectionWindowValid(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor)
{
    float alongWallCoordinateM = 0.0f;
    return
        TryComputeSideWallAimCoordinateM(
            state,
            sensor,
            alongWallCoordinateM) &&
        MazeMap::IsWithinWallSegmentCenterWindowM(
            alongWallCoordinateM,
            MazeMap::Config::kCellSizeM,
        MazeMap::Config::kMazeWallThicknessM,
        MazeMap::Config::kSideWallSegmentCenterFraction);
}
