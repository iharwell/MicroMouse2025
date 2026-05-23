#pragma once

#include "Defines.h"
#include "Direction.h"
#include "CircularBuffer.h"
#include "EncoderObs.h"
#include "SensorMount.h"
#include "WallSensor.h"
#include "LSM6DSV16X_IMU.h"
#include "MotorEncoderDrive.h"
#include "MotorModelUnits.h"
#include <cmath>
#include <cstdint>


namespace MazeMap
{
    class PlantModel;
    class RuntimeSensorSuite;

    namespace App::Internal
    {
        class CommandVector;
        class LoopController;
    }

    // The root authority of all details related to the physical layout, construction, or hardware aspects of the robot.
    // The hub class, and the only object one should need to run any particular subsystem.
    // As additional things are called for, build this class out through composition to make the static initialization readable.
    // Because the system uses TCM, there is no reason to copy things from this class elsewhere. Just pass this object or it's members by reference.
    // This class makes any global fields across the project obsolete. If you find any that aren't here, it's time to update this class.
    class EXPORT Vehicle
    {
    private:
        friend class PlantModel;
        friend class App::Internal::LoopController;
        friend class RuntimeSensorSuite;

        inline static constexpr float kPhysicalMassKg = 0.14f;
        inline static constexpr float kPhysicalWidthM = 0.0842f;
        inline static constexpr float kPhysicalLengthM = 0.1085f;
        // Physical yaw inertia estimate from the 140 g mass and measured body envelope. Tire scrub and drivetrain
        // turn dynamics are plant/model effects, not construction facts owned by Vehicle.
        inline static constexpr float kPhysicalYawInertiaKgM2 = 0.000220f;
        inline static constexpr float kFrontWallContactOffsetM = 0.056f;
        // March 22, 2026 aux008 latest completed fast IP180 audit pass fit 84.635 mm effective track width.
        inline static constexpr float kPhysicalTrackWidthM = 0.084635f;
        // Physical tire contact patch span measured on the chassis. Effective track width may exceed this due to
        // scrub dynamics, so these remain informational rather than hard bounds.
        inline static constexpr float kTrackWidthPhysicalMinM = 0.07004f;
        inline static constexpr float kTrackWidthPhysicalMaxM = 0.07868f;
        // Investigation note: plant contact placement still uses the effective track width. Keep the kinematic
        // fit here authoritative until physical contact geometry is split explicitly.
        inline static constexpr float kDriveWheelLongitudinalOffsetM = 0.01475f;
        // April 20, 2026 post-fan-swap open-floor smooth card `12:10:58` is the current hardware baseline and
        // implies about 132.35 mm effective track width in SEC_50_SMOOTH. Only one usable post-swap smooth card
        // exists so both anchor radii currently share that fit.
        inline static constexpr float kArcTrackWidthTightRadiusM = 0.063f;
        inline static constexpr float kArcTrackWidthTightTrackWidthM = 0.13235f;
        inline static constexpr float kArcTrackWidthWideRadiusM = 0.153f;
        inline static constexpr float kArcTrackWidthWideTrackWidthM = 0.13235f;
        inline static constexpr float kDriveNominalVoltageV = 6.0f;
        inline static constexpr float kDriveNominalNoLoadSpeedRpm = 14100.0f;
        inline static constexpr float kDriveSupplyVoltageV = 8.4f;
        inline static constexpr float kDriveResistanceOhms = 4.31f;
        inline static constexpr float kDriveTorqueConstantNmPerA = MilliNewtonMetersToNewtonMeters(3.96f);
        inline static constexpr float kDriveNoLoadCurrentA = MilliAmpsToAmps(45.9f);
        inline static constexpr float kDriveSpeedConstantRadpsPerVolt =
            ComputeMotorSpeedConstantRadpsPerVolt(
                kDriveNominalNoLoadSpeedRpm,
                kDriveNominalVoltageV,
                kDriveNoLoadCurrentA,
                kDriveResistanceOhms);
        inline static constexpr float kDriveGearRatio = 56.0f / 17.0f;
        inline static constexpr float kFanDownforceAtFullDutyN = 0.7f;
        inline static constexpr float kNominalCombinedAccelerationMps2 = 17.5f;
        inline static constexpr float kPeakCombinedAccelerationMps2 = 20.1f;
        // March 22, 2026 low-speed straight-audit fit: aux001-aux003 speed_idx 0 still over-reported outbound
        // encoder distance by about 3.05 mm on the 0.72 m north-corridor run, so trim the shared wheel diameter
        // down by 0.42% to keep the fixed-distance phases from finishing short.
        // Investigation note: the supplied tire OD is 25.000 mm. Revisit this rolling-diameter correction after
        // the post-estimator encoder-distance audit instead of treating this as physical wheel geometry.
        inline static constexpr float kDriveWheelDiameterM = 0.025220f;
        inline static constexpr std::uint16_t kDriveEncoderPulsesPerRev = 4096U;

        inline static constexpr std::uint8_t kLeftDriveMotorOutPinA = 24U;
        inline static constexpr std::uint8_t kLeftDriveMotorOutPinB = 25U;
        inline static constexpr std::uint8_t kLeftDriveEncoderInPinA = 2U;
        inline static constexpr std::uint8_t kLeftDriveEncoderInPinB = 3U;
        inline static constexpr std::uint8_t kLeftDriveEncoderChannel = 2U;
        inline static constexpr bool kLeftDriveInvertMotorDirection = true;
        inline static constexpr bool kLeftDriveInvertEncoderDirection = false;

        inline static constexpr std::uint8_t kRightDriveMotorOutPinA = 5U;
        inline static constexpr std::uint8_t kRightDriveMotorOutPinB = 6U;
        inline static constexpr std::uint8_t kRightDriveEncoderInPinA = 7U;
        inline static constexpr std::uint8_t kRightDriveEncoderInPinB = 8U;
        inline static constexpr std::uint8_t kRightDriveEncoderChannel = 1U;
        inline static constexpr bool kRightDriveInvertMotorDirection = true;
        inline static constexpr bool kRightDriveInvertEncoderDirection = false;

        MotorEncoderDrive _leftMotor;
        MotorEncoderDrive _rightMotor;
        WallSensor _frontLeftWallSensor;
        WallSensor _frontRightWallSensor;
        WallSensor _sideLeftWallSensor;
        WallSensor _sideRightWallSensor;
        LSM6DSV16X_IMU<36, 32, 11, 12, 13> _frontRightImu;
        LSM6DSV16X_IMU<37, 33, 11, 12, 13> _backLeftImu;

        float _fanDuty;
        float _peakForwardAcceleration;
        float _peakLateralAcceleration;
        float _peakYawRate;
        float _peakYawAccel;

        float _maxSpeed;

        void ApplyMotorCommand(const App::Internal::CommandVector& command) noexcept;
        void ResetDriveEncoders() noexcept;
        EncoderObs CaptureEncoderObservation(float dtSeconds) noexcept;
    public:
        Vehicle();

        static constexpr float GetPhysicalMassKg() noexcept { return kPhysicalMassKg; }
        static constexpr float GetPhysicalWidthM() noexcept { return kPhysicalWidthM; }
        static constexpr float GetPhysicalLengthM() noexcept { return kPhysicalLengthM; }
        static constexpr float GetPhysicalYawInertiaKgM2() noexcept { return kPhysicalYawInertiaKgM2; }
        static constexpr float GetPhysicalTrackWidthM() noexcept { return kPhysicalTrackWidthM; }
        static constexpr float GetTrackWidthPhysicalMinM() noexcept { return kTrackWidthPhysicalMinM; }
        static constexpr float GetTrackWidthPhysicalMaxM() noexcept { return kTrackWidthPhysicalMaxM; }
        static constexpr float GetPhysicalFrontWallContactOffsetM() noexcept { return kFrontWallContactOffsetM; }
        static constexpr float GetDriveWheelLongitudinalOffsetM() noexcept { return kDriveWheelLongitudinalOffsetM; }
        static constexpr float GetArcTrackWidthTightRadiusM() noexcept { return kArcTrackWidthTightRadiusM; }
        static constexpr float GetArcTrackWidthTightTrackWidthM() noexcept { return kArcTrackWidthTightTrackWidthM; }
        static constexpr float GetArcTrackWidthWideRadiusM() noexcept { return kArcTrackWidthWideRadiusM; }
        static constexpr float GetArcTrackWidthWideTrackWidthM() noexcept { return kArcTrackWidthWideTrackWidthM; }

        // Shared sustained lateral-acceleration reference used by both planning and estimator retunes.
        // Inclined-ramp slip begins at about 63.2 degrees, matching roughly 1.91 g sustained lateral hold.
        static constexpr float GetSustainedLateralAccelerationReferenceMps2() noexcept
        {
            return 1.91f * GRAVITY_MPS2;
        }

        static constexpr float GetDriveWheelRadiusM() noexcept
        {
            return 0.5f * kDriveWheelDiameterM;
        }

        static constexpr float GetFanDownforceAtFullDutyN() noexcept
        {
            return kFanDownforceAtFullDutyN;
        }

        static constexpr float GetBackLeftImuRuntimeGyroLpfCutoffHz() noexcept
        {
            return 213.0f;
        }

        static constexpr float WheelLinearVelocityFromWheelSpeed(float wheelSpeedRadps) noexcept
        {
            static_assert(kDriveWheelDiameterM > 0.0f, "Drive wheel diameter must be positive.");
            return wheelSpeedRadps * GetDriveWheelRadiusM();
        }

        static constexpr float WheelSpeedFromLinearVelocity(float wheelLinearMps) noexcept
        {
            static_assert(kDriveWheelDiameterM > 0.0f, "Drive wheel diameter must be positive.");
            return wheelLinearMps / GetDriveWheelRadiusM();
        }

        static constexpr float DriveEncoderDistanceFromCounts(std::int64_t counts) noexcept
        {
            static_assert(kDriveWheelDiameterM > 0.0f, "Drive wheel diameter must be positive.");
            static_assert(kDriveGearRatio > 0.0f, "Drive gear ratio must be positive.");
            static_assert(kDriveEncoderPulsesPerRev > 0U, "Drive encoder resolution must be positive.");
            return
                static_cast<float>(counts) *
                ((PI_F * kDriveWheelDiameterM) /
                    (kDriveGearRatio * static_cast<float>(kDriveEncoderPulsesPerRev)));
        }

        static constexpr float LeftWheelLinearVelocityFromBody(float forwardMps, float yawRateRadps) noexcept
        {
            static_assert(kPhysicalTrackWidthM > 0.0f, "Vehicle track width must be positive.");
            return forwardMps + (0.5f * GetPhysicalTrackWidthM() * yawRateRadps);
        }

        static constexpr float RightWheelLinearVelocityFromBody(float forwardMps, float yawRateRadps) noexcept
        {
            static_assert(kPhysicalTrackWidthM > 0.0f, "Vehicle track width must be positive.");
            return forwardMps - (0.5f * GetPhysicalTrackWidthM() * yawRateRadps);
        }

        static constexpr void WheelSpeedsFromBodyVelocity(
            float forwardMps,
            float yawRateRadps,
            float& leftWheelSpeedRadps,
            float& rightWheelSpeedRadps) noexcept
        {
            leftWheelSpeedRadps = WheelSpeedFromLinearVelocity(LeftWheelLinearVelocityFromBody(forwardMps, yawRateRadps));
            rightWheelSpeedRadps = WheelSpeedFromLinearVelocity(RightWheelLinearVelocityFromBody(forwardMps, yawRateRadps));
        }

        static constexpr float BodyForwardVelocityFromWheelLinear(float leftMps, float rightMps) noexcept
        {
            return 0.5f * (leftMps + rightMps);
        }

        static constexpr float BodyYawRateFromWheelLinear(float leftMps, float rightMps) noexcept
        {
            static_assert(kPhysicalTrackWidthM > 0.0f, "Vehicle track width must be positive.");
            return (leftMps - rightMps) / GetPhysicalTrackWidthM();
        }

        const MotorEncoderDrive& GetLeftMotorEncoderDrive() const noexcept { return _leftMotor; }
        const MotorEncoderDrive& GetRightMotorEncoderDrive() const noexcept { return _rightMotor; }
        WallSensor& FrontLeftWallSensor() noexcept { return _frontLeftWallSensor; }
        const WallSensor& FrontLeftWallSensor() const noexcept { return _frontLeftWallSensor; }
        WallSensor& FrontRightWallSensor() noexcept { return _frontRightWallSensor; }
        const WallSensor& FrontRightWallSensor() const noexcept { return _frontRightWallSensor; }
        WallSensor& SideLeftWallSensor() noexcept { return _sideLeftWallSensor; }
        const WallSensor& SideLeftWallSensor() const noexcept { return _sideLeftWallSensor; }
        WallSensor& SideRightWallSensor() noexcept { return _sideRightWallSensor; }
        const WallSensor& SideRightWallSensor() const noexcept { return _sideRightWallSensor; }
        LSM6DSV16X_IMU<36, 32, 11, 12, 13>& FrontRightImu() noexcept { return _frontRightImu; }
        const LSM6DSV16X_IMU<36, 32, 11, 12, 13>& FrontRightImu() const noexcept { return _frontRightImu; }
        LSM6DSV16X_IMU<37, 33, 11, 12, 13>& BackLeftImu() noexcept { return _backLeftImu; }
        const LSM6DSV16X_IMU<37, 33, 11, 12, 13>& BackLeftImu() const noexcept { return _backLeftImu; }

        static SensorMount GetBackLeftImuMount() noexcept;
        static SensorMount GetFrontLeftSensorMount() noexcept;
        static SensorMount GetFrontRightSensorMount() noexcept;
        static SensorMount GetSideLeftSensorMount() noexcept;
        static SensorMount GetSideRightSensorMount() noexcept;

        float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity);
        float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity) const;

        float GetTurnCost(RelativeDirection relDir, float cellDimensions);
        float GetTurnCost(RelativeDirection relDir, float cellDimensions) const;

        float GetTurnSpeed(RelativeDirection relDir, float cellDimensions);
        float GetTurnSpeed(RelativeDirection relDir, float cellDimensions) const;

        float GetTurnSpeed(float turningRadius) const;
        float GetSpeedFromCurvature(float curvature) const;
        float GetInPlaceTurnTime(float radians) const;

        float GetFastestTurnSpeed(float cellDimensions);
        float GetFastestTurnSpeed(float cellDimensions) const;

        float GetPeakYawRate() const { return _peakYawRate; }

        float GetMaxForwardAcceleration();
        float GetMaxForwardAcceleration() const;

        float GetCompensatedMaxForwardAcceleration(float velocity);
        float GetCompensatedMaxForwardAcceleration(float velocity) const;

        float GetMaxLateralAcceleration();
        float GetMaxLateralAcceleration() const;

        float GetMaxYawRate();
        float GetMaxYawRate() const;

        float GetMaxYawAccel();
        float GetMaxYawAccel() const;

        float GetMass() const;
        float GetTrackWidth() const;
        float GetYawInertia() const;
        float GetBatteryVoltage() const noexcept;
        void SetFanDuty(float dutyCycle) noexcept;
        float GetFanDuty() const noexcept;
        float GetFanDownforceAtFullDuty() const noexcept;
        float GetNominalCombinedAcceleration() const noexcept;
        float GetPeakCombinedAcceleration() const noexcept;
        static float GetArcEffectiveTrackWidth(float turningRadiusM) noexcept;
        static float GetEffectiveTrackWidthForMotion(float linearSpeedMps, float yawRateRadps) noexcept;
        float GetLength() const;
        float GetFrontWallContactOffset() const;
        float GetRearWallContactOffset() const;

        void SetMaxForwardAcceleration(float maxForwardAcceleration);
        void SetMaxLateralAcceleration(float maxLateralAcceleration);
        void SetMaxYawRate(float maxYawRate);
        void SetMaxYawAccel(float maxYawAccel);
        void SetMaxSpeed(float maxSpeed);
        float GetMaxSpeed();
        float GetMaxSpeed() const;

        float GetWidth();
        float GetWidth() const;
    };

}
