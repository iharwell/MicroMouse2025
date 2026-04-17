#pragma once

#include "Defines.h"
#include "Direction.h"
#include "CircularBuffer.h"
#include "VehicleState.h"
#include "WallSensor.h"
#include "LSM6DSV16X_IMU.h"
#include <cmath>

namespace MazeMap
{
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
            // Investigation note: PlantParams currently places tire contacts from the effective track width. Split
            // physical tire-contact geometry from kinematic track width when the next track-width audit is done.
            // March 22, 2026 aux006 short smooth-turn encoder/gyro fit before left-wall contact implied 96.49 mm
            // effective at the 63 mm nominal radius. The 153 mm point is scaled by the same 1.174285x factor as a
            // better starting point for the next wide-radius audit pass.
            { 0.063f, 0.096491f, 0.153f, 0.097348f }
        };
        VehicleState _state;
        float _peakForwardAcceleration;
        float _peakLateralAcceleration;
        float _peakRotationalVelocity;
        float _peakAngularAcceleration;

        float _maxSpeed;
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

        static ImuExtrinsics GetBackLeftImuExtrinsics() noexcept;
        static SensorExtrinsics GetFrontLeftSensorExtrinsics() noexcept;
        static SensorExtrinsics GetFrontRightSensorExtrinsics() noexcept;
        static SensorExtrinsics GetSideLeftSensorExtrinsics() noexcept;
        static SensorExtrinsics GetSideRightSensorExtrinsics() noexcept;

        const VehicleState& GetVehicleState();
        const VehicleState& GetVehicleState() const;

        void ProgressVehicleState(const VehicleState& previousState, VehicleState& projectedState, float timeDelta);

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
