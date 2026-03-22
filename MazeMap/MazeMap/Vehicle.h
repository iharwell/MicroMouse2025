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
            0.056f,
            // March 22, 2026 aux000 in-place-turn fit: the latest completed position-audit turn points centered
            // at 86.191 mm effective, with the two completed passes spanning 86.154-86.227 mm.
            0.086191f,
            // Physical tire contact patch span measured on the chassis. Effective track width may exceed this due to
            // scrub dynamics, so these remain informational rather than hard bounds.
            0.07004f,
            0.07868f,
            // March 22, 2026 aux000 smooth-turn fit: the completed S90SS and S90LS passes implied effective arc
            // widths of 85.809 mm at the 63 mm nominal radius and 84.519 mm at the 153 mm nominal radius.
            { 0.063f, 0.085809f, 0.153f, 0.084519f }
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
