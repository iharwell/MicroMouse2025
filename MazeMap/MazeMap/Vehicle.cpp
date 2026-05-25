#include "pch.h"
#include "Vehicle.h"
#include "CommandVector.h"
#include "EncoderObs.h"
#include "HardwareConfig.h"
#include "Pins.h"
#include "VehicleState.h"
#include "PlantModel.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include "math.h"

// Forward acceleration and max speed remain provisional until the direct forward-traction and top-speed
// characterization runs are completed. The maneuver planner only uses the shared lateral limit here;
// runtime yaw-rate ceilings are applied separately by the motion controller.
static constexpr float kVehiclePeakForwardAccelerationMps2 = 15.0f;
static constexpr float kVehiclePeakYawRateRadps = 27.0f;
static constexpr float kVehiclePeakYawAccelRadps2 = 645.0f;
static constexpr float kVehicleMaxSpeedMps = 4.0f;

static constexpr std::uint8_t kFrontRightWallSensorPin = 23U;
static constexpr std::uint8_t kFrontRightWallSensorLedPin = 19U;
static constexpr std::uint8_t kFrontLeftWallSensorPin = 22U;
static constexpr std::uint8_t kFrontLeftWallSensorLedPin = 18U;
static constexpr std::uint8_t kSideRightWallSensorPin = 21U;
static constexpr std::uint8_t kSideRightWallSensorLedPin = 17U;
static constexpr std::uint8_t kSideLeftWallSensorPin = 20U;
static constexpr std::uint8_t kSideLeftWallSensorLedPin = 16U;
// Placeholder table until measured calibration data is loaded for the chassis.
static constexpr std::array<float, 8> kDefaultWallSensorAdcToLightTable =
{ { 0.0f, 0.1f, 0.2f, 0.35f, 0.55f, 0.8f, 1.1f, 1.5f } };

static constexpr MazeMap::WallSensor::DistanceModel kDefaultWallSensorDistanceModel = {
    0.06f,
    0.9f,
    0.01f,
    0.20f
};

namespace MazeMap
{
    Vehicle::Vehicle()
        : _leftMotor(
            kDriveResistanceOhms,
            kDriveSupplyVoltageV,
            kDriveTorqueConstantNmPerA,
            kDriveSpeedConstantRadpsPerVolt,
            kDriveNoLoadCurrentA,
            kDriveGearRatio,
            kDriveWheelDiameterM,
            kDriveEncoderPulsesPerRev,
            kLeftDriveMotorOutPinA,
            kLeftDriveMotorOutPinB,
            kLeftDriveEncoderInPinA,
            kLeftDriveEncoderInPinB,
            kLeftDriveEncoderChannel,
            kLeftDriveInvertMotorDirection,
            kLeftDriveInvertEncoderDirection,
            1.0f)
        , _rightMotor(
            kDriveResistanceOhms,
            kDriveSupplyVoltageV,
            kDriveTorqueConstantNmPerA,
            kDriveSpeedConstantRadpsPerVolt,
            kDriveNoLoadCurrentA,
            kDriveGearRatio,
            kDriveWheelDiameterM,
            kDriveEncoderPulsesPerRev,
            kRightDriveMotorOutPinA,
            kRightDriveMotorOutPinB,
            kRightDriveEncoderInPinA,
            kRightDriveEncoderInPinB,
            kRightDriveEncoderChannel,
            kRightDriveInvertMotorDirection,
            kRightDriveInvertEncoderDirection,
            1.0f)
        , _frontLeftWallSensor(
            kFrontLeftWallSensorPin,
            kFrontLeftWallSensorLedPin,
            GetFrontLeftSensorMount().positionBodyM(),
            GetFrontLeftSensorMount().SensorForwardBody(),
            kDefaultWallSensorAdcToLightTable,
            kDefaultWallSensorDistanceModel,
            0.30f,
            HardwareConfig::kFrontWallSensorSwitchSettleTime_us,
            HardwareConfig::kFrontWallSensorSwitchSettleTime_us)
        , _frontRightWallSensor(
            kFrontRightWallSensorPin,
            kFrontRightWallSensorLedPin,
            GetFrontRightSensorMount().positionBodyM(),
            GetFrontRightSensorMount().SensorForwardBody(),
            kDefaultWallSensorAdcToLightTable,
            kDefaultWallSensorDistanceModel,
            0.30f,
            HardwareConfig::kFrontWallSensorSwitchSettleTime_us,
            HardwareConfig::kFrontWallSensorSwitchSettleTime_us)
        , _sideLeftWallSensor(
            kSideLeftWallSensorPin,
            kSideLeftWallSensorLedPin,
            GetSideLeftSensorMount().positionBodyM(),
            GetSideLeftSensorMount().SensorForwardBody(),
            kDefaultWallSensorAdcToLightTable,
            kDefaultWallSensorDistanceModel,
            0.30f,
            HardwareConfig::kSideWallSensorSwitchSettleTime_us,
            HardwareConfig::kSideWallSensorSwitchSettleTime_us)
        , _sideRightWallSensor(
            kSideRightWallSensorPin,
            kSideRightWallSensorLedPin,
            GetSideRightSensorMount().positionBodyM(),
            GetSideRightSensorMount().SensorForwardBody(),
            kDefaultWallSensorAdcToLightTable,
            kDefaultWallSensorDistanceModel,
            0.30f,
            HardwareConfig::kSideWallSensorSwitchSettleTime_us,
            HardwareConfig::kSideWallSensorSwitchSettleTime_us)
        , _frontRightImu()
        , _backLeftImu(GetBackLeftImuMount())
        , _fanDuty(0.0f)
        , _peakForwardAcceleration(kVehiclePeakForwardAccelerationMps2)
        , _peakLateralAcceleration(GetSustainedLateralAccelerationReferenceMps2())
        , _peakYawRate(kVehiclePeakYawRateRadps)
        , _peakYawAccel(kVehiclePeakYawAccelRadps2)
        , _maxSpeed(kVehicleMaxSpeedMps)
    {
    }

    void Vehicle::ApplyMotorCommand(const App::Internal::CommandVector& command) noexcept
    {
        if (!command.IsFinite())
        {
            _leftMotor.brake();
            _rightMotor.brake();
            return;
        }

        _leftMotor.setDriveCommand(command.LeftCommand());
        _rightMotor.setDriveCommand(command.RightCommand());
    }

    void Vehicle::ResetDriveEncoders() noexcept
    {
        _leftMotor.resetEncoderCount();
        _rightMotor.resetEncoderCount();
    }

    EncoderObs Vehicle::CaptureEncoderObservation(const float dtSeconds) noexcept
    {
        EncoderObs observation{};
        observation.SetTotalLeftCounts(_leftMotor.consumeEncoderCount());
        observation.SetTotalRightCounts(_rightMotor.consumeEncoderCount());
        observation.SetLeftDistanceDeltaM(_leftMotor.pulsesToDistance(observation.TotalLeftCounts()));
        observation.SetRightDistanceDeltaM(_rightMotor.pulsesToDistance(observation.TotalRightCounts()));

        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float invWheelRadiusM = 2.0f / kDriveWheelDiameterM;
            const float invDtSeconds = 1.0f / dtSeconds;
            observation.SetLeftVelocityMps(observation.LeftDistanceDeltaM() * invDtSeconds);
            observation.SetRightVelocityMps(observation.RightDistanceDeltaM() * invDtSeconds);
            observation.SetLeftWheelSpeedRadps(observation.LeftVelocityMps() * invWheelRadiusM);
            observation.SetRightWheelSpeedRadps(observation.RightVelocityMps() * invWheelRadiusM);
        }

        return observation;
    }

    SensorMount Vehicle::GetBackLeftImuMount() noexcept
    {
        // Body frame is +X right, +Y forward.
        return SensorMount(Eigen::Vector2f(-0.023f, -0.011f), Eigen::Matrix2f::Identity());
    }

    SensorMount Vehicle::GetFrontLeftSensorMount() noexcept
    {
        return SensorMount::FromForwardDirectionBody(
            Eigen::Vector2f(-0.03465f, 0.04223f),
            Eigen::Vector2f(-0.10453f, 0.99452f));
    }
    SensorMount Vehicle::GetFrontRightSensorMount() noexcept
    {
        return SensorMount::FromForwardDirectionBody(
            Eigen::Vector2f(0.03459f, 0.04223f),
            Eigen::Vector2f(0.10453f, 0.99452f));
    }
    SensorMount Vehicle::GetSideLeftSensorMount() noexcept
    {
        return SensorMount::FromForwardDirectionBody(
            Eigen::Vector2f(-0.02918f, 0.05026f),
            Eigen::Vector2f(-1.0f, 0.0f));
    }
    SensorMount Vehicle::GetSideRightSensorMount() noexcept
    {
        return SensorMount::FromForwardDirectionBody(
            Eigen::Vector2f(0.02772f, 0.05026f),
            Eigen::Vector2f(1.0f, 0.0f));
    }
    float Vehicle::GetStraightLineCost(float distance, float initialVelocity, float finalVelocity)
    {
        return const_cast<const Vehicle*>(this)->GetStraightLineCost(distance, initialVelocity, finalVelocity);
    }
    float Vehicle::GetStraightLineCost(float distance, float initialVelocity, float finalVelocity) const
    {
        float distToV1 = (finalVelocity * finalVelocity - initialVelocity * initialVelocity) / (2 * GetMaxForwardAcceleration());
        // Accelerating without enough space to reach target
        if (distToV1 >= distance)
        {
            return (sqrtf(initialVelocity * initialVelocity + 2 * GetMaxForwardAcceleration() * distance) - initialVelocity) / GetMaxForwardAcceleration();
        }
        // Decelerating without enough space to reach target
        else if (-distToV1 >= distance)
        {
            return (sqrtf(finalVelocity * finalVelocity + 2 * GetMaxForwardAcceleration() * distance) - finalVelocity) / GetMaxForwardAcceleration();
        }

        // We have space, so see if we can hit Max V
        float distToMaxV = (GetMaxSpeed() * GetMaxSpeed() - initialVelocity * initialVelocity) / (2.0f * GetMaxForwardAcceleration());
        float distFromMaxV = (GetMaxSpeed() * GetMaxSpeed() - finalVelocity * finalVelocity) / (2.0f * GetMaxForwardAcceleration());

        if (distToMaxV + distFromMaxV < distance)
        {
            float maxVelTime = (distance - distToMaxV - distFromMaxV) / GetMaxSpeed();
            float accelTime = (GetMaxSpeed() - initialVelocity + GetMaxSpeed() - finalVelocity) / GetMaxForwardAcceleration();
            return maxVelTime + accelTime;
        }

        // We have space to hit V1, but not Vmax

        // We need to know the time to switch from accelerating to deccelerating
        // Start with time to hit V1, and then use symmetry on the remaining time
        float tToV1 = fabsf(finalVelocity - initialVelocity) / GetMaxForwardAcceleration();
        float vGreater = fmaxf(initialVelocity, finalVelocity);

        float dSymmetric = (distance - fabsf(distToV1)) / 2.0f;
        float tSymmetric = (sqrtf(vGreater * vGreater + 2.0f * GetMaxForwardAcceleration() * dSymmetric) - vGreater) / GetMaxForwardAcceleration();
        return tToV1 + 2.0f * tSymmetric;
    }
    float Vehicle::GetTurnCost(RelativeDirection relDir, float cellDimensions)
    {
        return const_cast<const Vehicle*>(this)->GetTurnCost(relDir, cellDimensions);
    }
    float Vehicle::GetTurnCost(RelativeDirection relDir, float cellDimensions) const
    {
        const float speed = GetTurnSpeed(relDir, cellDimensions);
        if (!(speed > 0.0f))
        {
            return (std::numeric_limits<float>::infinity)();
        }

        float quarterAngles = 0.0f;
        if (relDir == RelativeDirection::L45 || relDir == RelativeDirection::R45)
        {
            quarterAngles = 1.0f;
        }
        else if (relDir == RelativeDirection::L90 || relDir == RelativeDirection::R90)
        {
            quarterAngles = 2.0f;
        }
        else if (relDir == RelativeDirection::L135 || relDir == RelativeDirection::R135)
        {
            quarterAngles = 3.0f;
        }
        else if (relDir == RelativeDirection::Reverse)
        {
            quarterAngles = 4.0f;
        }
        return quarterAngles * PI_F * cellDimensions / 2.0f / speed;
    }
    float Vehicle::GetTurnSpeed(RelativeDirection relDir, float cellDimensions) { return const_cast<const Vehicle*>(this)->GetTurnSpeed(relDir, cellDimensions); }
    float Vehicle::GetTurnSpeed(RelativeDirection relDir, float cellDimensions) const
    {
        float r = cellDimensions / 2.0f;
        if (relDir == RelativeDirection::R45 || relDir == RelativeDirection::L45)
        {
            r *= (2.0f + sqrtf(2.0f));
        }

        return GetTurnSpeed(r);
    }
    float Vehicle::GetFastestTurnSpeed(float cellDimensions) { return const_cast<const Vehicle*>(this)->GetFastestTurnSpeed(cellDimensions); }
    float Vehicle::GetFastestTurnSpeed(float cellDimensions) const
    {
        return GetTurnSpeed(cellDimensions * 2.0f);
    }
    float Vehicle::GetCompensatedMaxForwardAcceleration(float velocity) { return const_cast<const Vehicle*>(this)->GetCompensatedMaxForwardAcceleration(velocity); }
    float Vehicle::GetCompensatedMaxForwardAcceleration(float velocity) const
    {
        (void)velocity;
        return GetMaxForwardAcceleration();
    }
    float Vehicle::GetTurnSpeed(float turningRadius) const
    {
        if (!(turningRadius > 0.0f))
        {
            return 0.0f;
        }

        const float lateralLimitedSpeed = sqrtf(GetMaxLateralAcceleration() * turningRadius);
        const float maxSpeed = GetMaxSpeed();
        return fminf(maxSpeed, lateralLimitedSpeed);
    }
    float Vehicle::GetSpeedFromCurvature(float curvature) const
    {
        if (!(curvature > 0.0f))
        {
            return GetMaxSpeed();
        }

        return GetTurnSpeed(1.0f / curvature);
    }
    float Vehicle::GetInPlaceTurnTime(float radians) const
    {
        const float angle = fabsf(radians);
        const float maxYawAccel = GetMaxYawAccel();
        if (!(angle > 0.0f) || !(maxYawAccel > 0.0f))
        {
            return 0.0f;
        }

        return 2.0f * sqrtf(angle / maxYawAccel);
    }
    float Vehicle::GetMaxForwardAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxForwardAcceleration(); }
    float Vehicle::GetMaxForwardAcceleration() const { return _peakForwardAcceleration; }
    float Vehicle::GetMaxLateralAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxLateralAcceleration(); }
    float Vehicle::GetMaxLateralAcceleration() const { return _peakLateralAcceleration; }
    float Vehicle::GetMaxYawRate() { return const_cast<const Vehicle*>(this)->GetMaxYawRate(); }
    float Vehicle::GetMaxYawRate() const { return _peakYawRate; }
    float Vehicle::GetMaxYawAccel() { return const_cast<const Vehicle*>(this)->GetMaxYawAccel(); }
    float Vehicle::GetMaxYawAccel() const { return _peakYawAccel; }
    float Vehicle::GetMass() const { return GetPhysicalMassKg(); }
    float Vehicle::GetTrackWidth() const { return GetPhysicalTrackWidthM(); }
    float Vehicle::GetYawInertia() const { return GetPhysicalYawInertiaKgM2(); }
    float Vehicle::GetBatteryVoltage() const noexcept { return kDriveSupplyVoltageV; }
    void Vehicle::SetFanDuty(float dutyCycle) noexcept
    {
        _fanDuty = (std::clamp)(dutyCycle, 0.0f, 1.0f);
#if defined(ARDUINO_TEENSY41)
        const std::uint32_t maxPwmCode = (1UL << HardwareConfig::kPwmBits) - 1UL;
        const std::uint16_t fanPwmCode =
            static_cast<std::uint16_t>(_fanDuty * static_cast<float>(maxPwmCode) + 0.5f);
        analogWrite(Pins::Fan_CTRL, fanPwmCode);
#endif
    }
    float Vehicle::GetFanDuty() const noexcept { return _fanDuty; }
    float Vehicle::GetFanDownforceAtFullDuty() const noexcept { return kFanDownforceAtFullDutyN; }
    float Vehicle::GetNominalCombinedAcceleration() const noexcept { return kNominalCombinedAccelerationMps2; }
    float Vehicle::GetPeakCombinedAcceleration() const noexcept { return kPeakCombinedAccelerationMps2; }
    float Vehicle::GetArcEffectiveTrackWidth(float turningRadiusM) noexcept
    {
        if (!std::isfinite(turningRadiusM) || !(turningRadiusM > 0.0f))
        {
            return GetPhysicalTrackWidthM();
        }

        if (!(GetArcTrackWidthTightRadiusM() > 0.0f) ||
            !(GetArcTrackWidthWideRadiusM() > GetArcTrackWidthTightRadiusM()) ||
            !std::isfinite(GetArcTrackWidthTightTrackWidthM()) ||
            !std::isfinite(GetArcTrackWidthWideTrackWidthM()))
        {
            return GetPhysicalTrackWidthM();
        }

        if (turningRadiusM <= GetArcTrackWidthTightRadiusM())
        {
            return GetArcTrackWidthTightTrackWidthM();
        }

        if (turningRadiusM >= GetArcTrackWidthWideRadiusM())
        {
            return GetArcTrackWidthWideTrackWidthM();
        }

        const float blend =
            (turningRadiusM - GetArcTrackWidthTightRadiusM()) /
            (GetArcTrackWidthWideRadiusM() - GetArcTrackWidthTightRadiusM());
        return GetArcTrackWidthTightTrackWidthM() +
            (blend * (GetArcTrackWidthWideTrackWidthM() - GetArcTrackWidthTightTrackWidthM()));
    }
    float Vehicle::GetEffectiveTrackWidthForMotion(float linearSpeedMps, float yawRateRadps) noexcept
    {
        (void)linearSpeedMps;
        (void)yawRateRadps;
        return GetPhysicalTrackWidthM();
    }
    float Vehicle::GetLength() const { return GetPhysicalLengthM(); }
    float Vehicle::GetFrontWallContactOffset() const { return GetPhysicalFrontWallContactOffsetM(); }
    float Vehicle::GetRearWallContactOffset() const { return GetLength() - GetFrontWallContactOffset(); }
    void Vehicle::SetMaxForwardAcceleration(float maxForwardAcceleration)
    {
        _peakForwardAcceleration = maxForwardAcceleration;
    }
    void Vehicle::SetMaxLateralAcceleration(float maxLateralAcceleration)
    {
        _peakLateralAcceleration = maxLateralAcceleration;
    }
    void Vehicle::SetMaxYawRate(float maxYawRate)
    {
        _peakYawRate = maxYawRate;
    }
    void Vehicle::SetMaxYawAccel(float maxYawAccel)
    {
        _peakYawAccel = maxYawAccel;
    }
    void Vehicle::SetMaxSpeed(float maxSpeed)
    {
        _maxSpeed = maxSpeed;
    }
    float Vehicle::GetMaxSpeed() { return const_cast<const Vehicle*>(this)->GetMaxSpeed(); }
    float Vehicle::GetMaxSpeed() const { return _maxSpeed; }
    float Vehicle::GetWidth() { return const_cast<const Vehicle*>(this)->GetWidth(); }
    float Vehicle::GetWidth() const { return GetPhysicalWidthM(); }

}

