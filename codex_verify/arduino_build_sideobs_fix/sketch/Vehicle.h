#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Vehicle.h"
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
            // March 22, 2026 in-place-turn fit: completed position-audit turns from aux000-aux004 averaged
            // 85.22 mm effective, with a completed-run range of 84.40-85.89 mm.
            0.08522f,
            0.08440f,
            0.08589f,
            // March 22, 2026 arc-radius fit: the completed aux004 short-90 pass under-rotated by 11.681 deg at the
            // 63 mm nominal radius, so use a tighter-arc width there and blend back to the long-90 radius fit.
            { 0.063f, 0.09795f, 0.153f, 0.08522f }
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
