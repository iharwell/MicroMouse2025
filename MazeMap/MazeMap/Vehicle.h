#pragma once

#include "Defines.h"
#include "Direction.h"
#include "CircularBuffer.h"
#include "SensorMount.h"
#include "WallSensor.h"
#include "LSM6DSV16X_IMU.h"
#include "MotorEncoderDrive.h"
#include "MotorModelUnits.h"
#include <cmath>
#include <cstdint>

class RuntimeSensorSuite;

namespace MazeMap
{
    struct EncoderObs;
    class PlantModel;

    namespace App::Internal
    {
        class CommandVector;
        class LoopController;
    }

    struct ArcTrackWidthInterpolation
    {
        float tightRadiusM;
        float tightTrackWidthM;
        float wideRadiusM;
        float wideTrackWidthM;
    };

    struct VehiclePhysicalModel
    {
        float massKg;
        float widthM;
        float lengthM;
        float yawInertiaKgM2;
        float frontWallContactOffsetM;
        float trackWidthM;
        float trackWidthPhysicalMinM;
        float trackWidthPhysicalMaxM;
        float driveWheelLongitudinalOffsetM;
        ArcTrackWidthInterpolation arcTrackWidthInterpolation;
    };

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
        friend class ::RuntimeSensorSuite;

        inline static constexpr VehiclePhysicalModel kPhysicalModel = {
            0.14f, //Mass
            0.0842f, //Width
            0.1085f, //Length
            // Physical yaw inertia estimate from the 140 g mass and measured body envelope. Tire scrub and drivetrain
            // turn dynamics are plant/model effects, not construction facts owned by Vehicle.
            0.000220f, //Yaw inertia
            0.056f,
            // March 22, 2026 aux008 latest completed fast IP180 audit pass fit 84.635 mm effective track width.
            0.084635f,
            // Physical tire contact patch span measured on the chassis. Effective track width may exceed this due to
            // scrub dynamics, so these remain informational rather than hard bounds.
            0.07004f,
            0.07868f,
            0.01475f,
            // Investigation note: PlantParams still places tire contacts from the effective track width. Keep the
            // kinematic fit here authoritative until physical contact geometry is split explicitly.
            // April 20, 2026 post-fan-swap open-floor smooth card `12:10:58` is the current hardware baseline and
            // implies about 132.35 mm effective track width in SEC_50_SMOOTH. Only one usable post-swap smooth card
            // exists so both anchor radii currently share that fit.
            { 0.063f, 0.13235f, 0.153f, 0.13235f }
        };
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
        // March 22, 2026 low-speed straight-audit fit: aux001-aux003 speed_idx 0 still over-reported outbound
        // encoder distance by about 3.05 mm on the 0.72 m north-corridor run, so trim the shared wheel diameter
        // down by 0.42% to keep the fixed-distance phases from finishing short.
        // Investigation note: the supplied tire OD is 25.000 mm. Revisit this rolling-diameter correction after
        // the post-UKF encoder-distance audit instead of treating this as physical wheel geometry.
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

        float _peakForwardAcceleration;
        float _peakLateralAcceleration;
        float _peakRotationalVelocity;
        float _peakAngularAcceleration;

        float _maxSpeed;

        void ApplyMotorCommand(const App::Internal::CommandVector& command) noexcept;
        void ResetDriveEncoders() noexcept;
        EncoderObs CaptureEncoderObservation(float dtSeconds) noexcept;
    public:
        using ImuFrontRight = LSM6DSV16X_IMU<36, 32, 11, 12, 13>;
        using ImuBackLeft = LSM6DSV16X_IMU<37, 33, 11, 12, 13>;

        Vehicle();

        static constexpr const VehiclePhysicalModel& GetPhysicalModel() noexcept
        {
            return kPhysicalModel;
        }

        // Shared sustained lateral-acceleration reference used by both planning and estimator retunes.
        static constexpr float GetSustainedLateralAccelerationReferenceMps2() noexcept
        {
            return 16.5f;
        }

        static constexpr float GetDriveWheelRadiusM() noexcept
        {
            return 0.5f * kDriveWheelDiameterM;
        }

        static constexpr float WheelLinearVelocityFromOmega(float omegaRadps) noexcept
        {
            static_assert(kDriveWheelDiameterM > 0.0f, "Drive wheel diameter must be positive.");
            return omegaRadps * GetDriveWheelRadiusM();
        }

        static constexpr float WheelOmegaFromLinearVelocity(float wheelLinearMps) noexcept
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
            static_assert(kPhysicalModel.trackWidthM > 0.0f, "Vehicle track width must be positive.");
            return forwardMps + (0.5f * GetPhysicalModel().trackWidthM * yawRateRadps);
        }

        static constexpr float RightWheelLinearVelocityFromBody(float forwardMps, float yawRateRadps) noexcept
        {
            static_assert(kPhysicalModel.trackWidthM > 0.0f, "Vehicle track width must be positive.");
            return forwardMps - (0.5f * GetPhysicalModel().trackWidthM * yawRateRadps);
        }

        static constexpr void WheelOmegasFromBodyVelocity(
            float forward,
            float yawRate,
            float& l_omega,
            float& r_omega) noexcept
        {
            l_omega = WheelOmegaFromLinearVelocity(LeftWheelLinearVelocityFromBody(forward, yawRate));
            r_omega = WheelOmegaFromLinearVelocity(RightWheelLinearVelocityFromBody(forward, yawRate));
        }

        static constexpr float BodyForwardVelocityFromWheelLinear(float leftMps, float rightMps) noexcept
        {
            return 0.5f * (leftMps + rightMps);
        }

        static constexpr float BodyYawRateFromWheelLinear(float leftMps, float rightMps) noexcept
        {
            static_assert(kPhysicalModel.trackWidthM > 0.0f, "Vehicle track width must be positive.");
            return (leftMps - rightMps) / GetPhysicalModel().trackWidthM;
        }

        const MotorEncoderDrive& GetLeftMotorEncoderDrive() const noexcept { return _leftMotor; }
        const MotorEncoderDrive& GetRightMotorEncoderDrive() const noexcept { return _rightMotor; }

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

        float GetPeakRotationalVelocity() const { return _peakRotationalVelocity; }

        float GetMaxForwardAcceleration();
        float GetMaxForwardAcceleration() const;

        float GetCompensatedMaxForwardAcceleration(float velocity);
        float GetCompensatedMaxForwardAcceleration(float velocity) const;

        float GetMaxLateralAcceleration();
        float GetMaxLateralAcceleration() const;

        float GetMaxRotationalVelocity();
        float GetMaxRotationalVelocity() const;

        float GetMaxAngularAcceleration();
        float GetMaxAngularAcceleration() const;

        float GetMass() const;
        float GetTrackWidth() const;
        float GetYawInertia() const;
        static float GetArcEffectiveTrackWidth(float turningRadiusM) noexcept;
        static float GetEffectiveTrackWidthForMotion(float linearSpeedMps, float angularSpeedRadps) noexcept;
        float GetLength() const;
        float GetFrontWallContactOffset() const;
        float GetRearWallContactOffset() const;

        void SetMaxForwardAcceleration(float maxForwardAcceleration);
        void SetMaxLateralAcceleration(float maxLateralAcceleration);
        void SetMaxRotationalVelocity(float maxRotationalVelocity);
        void SetMaxAngularAcceleration(float maxAngularAcceleration);
        void SetMaxSpeed(float maxSpeed);
        float GetMaxSpeed();
        float GetMaxSpeed() const;

        float GetWidth();
        float GetWidth() const;

        WallSensor FrontLeft;
        WallSensor FrontRight;
        WallSensor SideLeft;
        WallSensor SideRight;
        ImuFrontRight IMU_FR;
        ImuBackLeft IMU_BL;
    };

}
