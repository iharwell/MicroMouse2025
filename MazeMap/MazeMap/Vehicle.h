#pragma once

#include "Defines.h"
#include "Vector2f.h"
#include "Direction.h"
#include "CircularBuffer.h"
#include "VehicleState.h"
#include "WallSensor.h"
#include "LSV6DSV16X_IMU.h"
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

    class EXPORT Vehicle
    {
    private:
        inline static constexpr VehiclePhysicalModel kPhysicalModel = {
            0.14f,
            0.0842f,
            0.1085f,
            // March 22, 2026 aux008 first fast IP180 fit from the clean early acceleration ramp gave an effective
            // yaw inertia around 6.65e-4 kg*m^2. This is intentionally larger than the bare rectangular body estimate
            // because it captures the drivetrain and tire-scrub dynamics the turn controller actually has to drive.
            0.000665f,
            0.056f,
            // March 22, 2026 aux008 latest completed fast IP180 audit pass fit 84.635 mm effective track width.
            0.084635f,
            // Physical tire contact patch span measured on the chassis. Effective track width may exceed this due to
            // scrub dynamics, so these remain informational rather than hard bounds.
            0.07004f,
            0.07868f,
            // March 22, 2026 aux006 short smooth-turn encoder/gyro fit before left-wall contact implied 96.49 mm
            // effective at the 63 mm nominal radius. The 153 mm point is scaled by the same 1.174285x factor as a
            // better starting point for the next wide-radius audit pass.
            { 0.063f, 0.096491f, 0.153f, 0.097348f }
        };
        CircularBuffer<VehicleState, 15> _stateHistory;
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
