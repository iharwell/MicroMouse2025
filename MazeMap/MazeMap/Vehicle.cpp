#include "pch.h"
#include "Vehicle.h"
#include "MouseUkf.h"
#include <array>
#include "math.h"

namespace
{
    // Forward acceleration and max speed remain intentionally conservative until the direct forward-traction and
    // top-speed characterization runs are completed. The maneuver planner only uses the shared lateral limit here;
    // runtime yaw-rate ceilings are applied separately by the motion controller.
    constexpr float kVehiclePeakForwardAccelerationMps2 = 15.0f;
    constexpr float kVehiclePeakLateralAccelerationMps2 = 16.5f;
    constexpr float kVehiclePeakRotationalVelocityRadps = 9.0f;
    constexpr float kVehiclePeakAngularAccelerationRadps2 = 45.0f;
    constexpr float kVehicleMaxSpeedMps = 4.0f;

    constexpr uint8_t kFrontRightWallSensorPin = 23U;
    constexpr uint8_t kFrontRightWallSensorLedPin = 19U;
    constexpr uint8_t kFrontLeftWallSensorPin = 22U;
    constexpr uint8_t kFrontLeftWallSensorLedPin = 18U;
    constexpr uint8_t kSideRightWallSensorPin = 21U;
    constexpr uint8_t kSideRightWallSensorLedPin = 17U;
    constexpr uint8_t kSideLeftWallSensorPin = 20U;
    constexpr uint8_t kSideLeftWallSensorLedPin = 16U;
    constexpr float kArcTrackWidthLinearSpeedEpsilonMps = 1.0e-4f;
    constexpr float kArcTrackWidthAngularSpeedEpsilonRadps = 1.0e-4f;

    const std::array<float, 8>& GetDefaultWallSensorAdcToLightTable()
    {
        // Placeholder table until measured calibration data is loaded for the chassis.
        static const std::array<float, 8> kTable = { 0.0f, 0.1f, 0.2f, 0.35f, 0.55f, 0.8f, 1.1f, 1.5f };
        return kTable;
    }

    const MazeMap::WallSensor::DistanceModel& GetDefaultWallSensorDistanceModel()
    {
        static const MazeMap::WallSensor::DistanceModel kModel = {
            0.06f,
            0.9f,
            0.01f,
            0.20f
        };
        return kModel;
    }

    MazeMap::WallSensor MakeWallSensor(
        uint8_t wallSensorInPin,
        uint8_t ledOutPin,
        const MazeMap::SensorExtrinsics& extrinsics)
    {
        return MazeMap::WallSensor(
            wallSensorInPin,
            ledOutPin,
            extrinsics.positionBodyM,
            extrinsics.directionBody,
            GetDefaultWallSensorAdcToLightTable(),
            GetDefaultWallSensorDistanceModel());
    }

    MazeMap::WallSensor MakeFrontLeftWallSensor()
    {
        return MakeWallSensor(
            kFrontLeftWallSensorPin,
            kFrontLeftWallSensorLedPin,
            MazeMap::Vehicle::GetFrontLeftSensorExtrinsics());
    }

    MazeMap::WallSensor MakeFrontRightWallSensor()
    {
        return MakeWallSensor(
            kFrontRightWallSensorPin,
            kFrontRightWallSensorLedPin,
            MazeMap::Vehicle::GetFrontRightSensorExtrinsics());
    }

    MazeMap::WallSensor MakeSideLeftWallSensor()
    {
        return MakeWallSensor(
            kSideLeftWallSensorPin,
            kSideLeftWallSensorLedPin,
            MazeMap::Vehicle::GetSideLeftSensorExtrinsics());
    }

    MazeMap::WallSensor MakeSideRightWallSensor()
    {
        return MakeWallSensor(
            kSideRightWallSensorPin,
            kSideRightWallSensorLedPin,
            MazeMap::Vehicle::GetSideRightSensorExtrinsics());
    }
}

namespace MazeMap
{
    Vehicle::Vehicle()
        : _peakForwardAcceleration(kVehiclePeakForwardAccelerationMps2)
        , _peakLateralAcceleration(kVehiclePeakLateralAccelerationMps2)
        , _peakRotationalVelocity(kVehiclePeakRotationalVelocityRadps)
        , _peakAngularAcceleration(kVehiclePeakAngularAccelerationRadps2)
        , _maxSpeed(kVehicleMaxSpeedMps)
        , FrontLeft(MakeFrontLeftWallSensor())
        , FrontRight(MakeFrontRightWallSensor())
        , SideLeft(MakeSideLeftWallSensor())
        , SideRight(MakeSideRightWallSensor())
        , IMU_FR()
        , IMU_BL()
    {
    }

    const VehicleState& Vehicle::GetVehicleState() const { return _stateHistory.GetLatest(); }
    SensorExtrinsics Vehicle::GetFrontLeftSensorExtrinsics() noexcept
    {
        return {
            Vectorf<2>(0.04223f, 0.03465f),
            Vectorf<2>(0.99452f, 0.10453f),
            std::atan2(0.10453f, 0.99452f)
        };
    }
    SensorExtrinsics Vehicle::GetFrontRightSensorExtrinsics() noexcept
    {
        return {
            Vectorf<2>(0.04223f, -0.03459f),
            Vectorf<2>(0.99452f, -0.10453f),
            std::atan2(-0.10453f, 0.99452f)
        };
    }
    SensorExtrinsics Vehicle::GetSideLeftSensorExtrinsics() noexcept
    {
        return {
            Vectorf<2>(0.05026f, 0.02918f),
            Vectorf<2>(0.0f, 1.0f),
            0.5f * PI_F
        };
    }
    SensorExtrinsics Vehicle::GetSideRightSensorExtrinsics() noexcept
    {
        return {
            Vectorf<2>(0.05026f, -0.02772f),
            Vectorf<2>(0.0f, -1.0f),
            -0.5f * PI_F
        };
    }
    void Vehicle::ProgressVehicleState(const VehicleState& previousState, VehicleState& projectedState, float timeDelta)
    {
        PlantModel plantModel;
        const PlantParams params = PlantParams::Default();
        projectedState.SetTime(previousState.GetTime() + timeDelta);
        projectedState.SetControlInput(previousState.GetControlInput());
        projectedState.SetStateVector(
            plantModel.integrateMidpoint(
                previousState.GetStateVector(),
                previousState.GetControlInput(),
                timeDelta,
                params));
        projectedState.SetSqrtCovariance(previousState.GetSqrtCovariance());
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
            return INFINITY;
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
        const float maxAngularAcceleration = GetMaxAngularAcceleration();
        if (!(angle > 0.0f) || !(maxAngularAcceleration > 0.0f))
        {
            return 0.0f;
        }

        return 2.0f * sqrtf(angle / maxAngularAcceleration);
    }
    float Vehicle::GetMaxForwardAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxForwardAcceleration(); }
    float Vehicle::GetMaxForwardAcceleration() const { return _peakForwardAcceleration; }
    float Vehicle::GetMaxLateralAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxLateralAcceleration(); }
    float Vehicle::GetMaxLateralAcceleration() const { return _peakLateralAcceleration; }
    float Vehicle::GetMaxRotationalVelocity() { return const_cast<const Vehicle*>(this)->GetMaxRotationalVelocity(); }
    float Vehicle::GetMaxRotationalVelocity() const { return _peakRotationalVelocity; }
    float Vehicle::GetMaxAngularAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxAngularAcceleration(); }
    float Vehicle::GetMaxAngularAcceleration() const { return _peakAngularAcceleration; }
    float Vehicle::GetMass() const { return GetPhysicalModel().massKg; }
    float Vehicle::GetTrackWidth() const { return GetPhysicalModel().trackWidthM; }
    float Vehicle::GetYawInertia() const { return GetPhysicalModel().yawInertiaKgM2; }
    float Vehicle::GetArcEffectiveTrackWidth(float turningRadiusM) noexcept
    {
        const ArcTrackWidthInterpolation& interpolation = GetPhysicalModel().arcTrackWidthInterpolation;
        if (!std::isfinite(turningRadiusM) || !(turningRadiusM > 0.0f))
        {
            return GetPhysicalModel().trackWidthM;
        }

        if (!(interpolation.tightRadiusM > 0.0f) ||
            !(interpolation.wideRadiusM > interpolation.tightRadiusM) ||
            !std::isfinite(interpolation.tightTrackWidthM) ||
            !std::isfinite(interpolation.wideTrackWidthM))
        {
            return GetPhysicalModel().trackWidthM;
        }

        if (turningRadiusM <= interpolation.tightRadiusM)
        {
            return interpolation.tightTrackWidthM;
        }

        if (turningRadiusM >= interpolation.wideRadiusM)
        {
            return interpolation.wideTrackWidthM;
        }

        const float blend =
            (turningRadiusM - interpolation.tightRadiusM) /
            (interpolation.wideRadiusM - interpolation.tightRadiusM);
        return interpolation.tightTrackWidthM +
            (blend * (interpolation.wideTrackWidthM - interpolation.tightTrackWidthM));
    }
    float Vehicle::GetEffectiveTrackWidthForMotion(float linearSpeedMps, float angularSpeedRadps) noexcept
    {
        if (!std::isfinite(linearSpeedMps) ||
            !std::isfinite(angularSpeedRadps) ||
            std::fabs(linearSpeedMps) <= kArcTrackWidthLinearSpeedEpsilonMps ||
            std::fabs(angularSpeedRadps) <= kArcTrackWidthAngularSpeedEpsilonRadps)
        {
            return GetPhysicalModel().trackWidthM;
        }

        return GetArcEffectiveTrackWidth(std::fabs(linearSpeedMps / angularSpeedRadps));
    }
    float Vehicle::GetLength() const { return GetPhysicalModel().lengthM; }
    float Vehicle::GetFrontWallContactOffset() const { return GetPhysicalModel().frontWallContactOffsetM; }
    float Vehicle::GetRearWallContactOffset() const { return GetLength() - GetFrontWallContactOffset(); }
    void Vehicle::SetMaxForwardAcceleration(float maxForwardAcceleration)
    {
        _peakForwardAcceleration = maxForwardAcceleration;
    }
    void Vehicle::SetMaxLateralAcceleration(float maxLateralAcceleration)
    {
        _peakLateralAcceleration = maxLateralAcceleration;
    }
    void Vehicle::SetMaxRotationalVelocity(float maxRotationalVelocity)
    {
        _peakRotationalVelocity = maxRotationalVelocity;
    }
    void Vehicle::SetMaxAngularAcceleration(float maxAngularAcceleration)
    {
        _peakAngularAcceleration = maxAngularAcceleration;
    }
    void Vehicle::SetMaxSpeed(float maxSpeed)
    {
        _maxSpeed = maxSpeed;
    }
    float Vehicle::GetMaxSpeed() { return const_cast<const Vehicle*>(this)->GetMaxSpeed(); }
    float Vehicle::GetMaxSpeed() const { return _maxSpeed; }
    float Vehicle::GetWidth() { return const_cast<const Vehicle*>(this)->GetWidth(); }
    float Vehicle::GetWidth() const { return GetPhysicalModel().widthM; }

    void DirectStateUpdate(const VehicleState& previousState, VehicleState& result, float timeDelta)
    {
        result.SetTime(previousState.GetTime() + timeDelta);

        //result.SetAcceleration(previousState.GetAcceleration() + )
    }
    const VehicleState& Vehicle::GetVehicleState() { return const_cast<const Vehicle*>(this)->GetVehicleState(); }
}
