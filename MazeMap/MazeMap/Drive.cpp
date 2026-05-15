#include "pch.h"
#include "Drive.h"

#include "DriveBase.h"
#include "Maze.h"
#include "SharedRobotRuntime.h"
#include "MissionStartPolicy.h"
#include "MotionTargetProjection.h"
#include "TurnWallEdgeTracker.h"
#include "WallDistanceCalibration.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <new>

namespace MazeMap::App::Internal
{
    constexpr std::size_t kPrimitiveStorageBytes = 16U * sizeof(std::uint32_t);
    constexpr std::size_t kPrimitiveStorageAlignment = 16U;

    class HoldPrimitive final
    {
    public:
        HoldPrimitive(const std::uint16_t requestedTicks, const bool resetOnNonStationary) noexcept :
            _requestedTicks(requestedTicks),
            _remainingTicks(requestedTicks),
            _resetOnNonStationary(resetOnNonStationary)
        {
        }

        void ObserveStationaryState(const bool stationary) noexcept
        {
            if (stationary)
            {
                if (_remainingTicks > 0U)
                {
                    --_remainingTicks;
                }
            }
            else if (_resetOnNonStationary)
            {
                _remainingTicks = _requestedTicks;
            }
        }

        bool IsComplete() const noexcept
        {
            return _remainingTicks == 0U;
        }

    private:
        std::uint16_t _requestedTicks{};
        std::uint16_t _remainingTicks{};
        bool _resetOnNonStationary{};
    };

    class LinearMotionPrimitive final
    {
    public:
        LinearMotionPrimitive(
            const float targetDistanceM,
            const float cruiseMagnitudeMps,
            const float exitMagnitudeMps,
            const float targetYawRad,
            const float direction,
            const float startDistanceM,
            const float commandedSpeedMps) noexcept :
            _targetDistanceM(targetDistanceM),
            _cruiseMagnitudeMps(cruiseMagnitudeMps),
            _exitMagnitudeMps(exitMagnitudeMps),
            _targetYawRad(targetYawRad),
            _direction(direction),
            _startDistanceM(startDistanceM),
            _commandedSpeedMps(commandedSpeedMps)
        {
        }

        float TargetDistanceM() const noexcept { return _targetDistanceM; }
        float CruiseMagnitudeMps() const noexcept { return _cruiseMagnitudeMps; }
        float ExitMagnitudeMps() const noexcept { return _exitMagnitudeMps; }
        float TargetYawRad() const noexcept { return _targetYawRad; }
        float Direction() const noexcept { return _direction; }
        float StartDistanceM() const noexcept { return _startDistanceM; }
        float CommandedSpeedMps() const noexcept { return _commandedSpeedMps; }
        void SetCommandedSpeedMps(const float commandedSpeedMps) noexcept { _commandedSpeedMps = commandedSpeedMps; }

    private:
        float _targetDistanceM{};
        float _cruiseMagnitudeMps{};
        float _exitMagnitudeMps{};
        float _targetYawRad{};
        float _direction{};
        float _startDistanceM{};
        float _commandedSpeedMps{};
    };

    class TurnPrimitive final
    {
    public:
        TurnPrimitive(
            const float targetYawRad,
            const float turnDirection,
            const float retainedYawRateRadps,
            const bool hasFiniteStopCondition,
            MazeMap::TurnWallEdgeTracker* const wallEdgeTracker) noexcept :
            _targetYawRad(targetYawRad),
            _turnDirection(turnDirection),
            _retainedYawRateRadps(retainedYawRateRadps),
            _hasFiniteStopCondition(hasFiniteStopCondition),
            _wallEdgeTracker(wallEdgeTracker)
        {
        }

        void ObserveWallStates(const SensorSnapshot& sensors) noexcept
        {
            if (_wallEdgeTracker != nullptr)
            {
                MazeMap::ObserveTurnWallStates(*_wallEdgeTracker, sensors.leftWall, sensors.rightWall);
            }
        }

        float TargetYawRad() const noexcept { return _targetYawRad; }
        float TurnDirection() const noexcept { return _turnDirection; }
        float RetainedYawRateRadps() const noexcept { return _retainedYawRateRadps; }
        bool HasFiniteStopCondition() const noexcept { return _hasFiniteStopCondition; }

    private:
        float _targetYawRad{};
        float _turnDirection{};
        float _retainedYawRateRadps{};
        bool _hasFiniteStopCondition{};
        MazeMap::TurnWallEdgeTracker* _wallEdgeTracker{};
    };

    class TurnTransitionPrimitive final
    {
    public:
        TurnTransitionPrimitive(
            const float distanceM,
            const float dCurvatureDs,
            const float speedMps,
            const float initialYawRateRadps,
            const float startDistanceM) noexcept :
            _distanceM(distanceM),
            _dCurvatureDs(dCurvatureDs),
            _speedMps(speedMps),
            _initialYawRateRadps(initialYawRateRadps),
            _startDistanceM(startDistanceM)
        {
        }

        float DistanceM() const noexcept { return _distanceM; }
        float CurvatureRatePerMeter() const noexcept { return _dCurvatureDs; }
        float SpeedMps() const noexcept { return _speedMps; }
        float InitialYawRateRadps() const noexcept { return _initialYawRateRadps; }
        float StartDistanceM() const noexcept { return _startDistanceM; }

    private:
        float _distanceM{};
        float _dCurvatureDs{};
        float _speedMps{};
        float _initialYawRateRadps{};
        float _startDistanceM{};
    };

    class ArcPrimitive final
    {
    public:
        ArcPrimitive(
            const float distanceM,
            const float curvature,
            const float speedMps,
            const float startDistanceM) noexcept :
            _distanceM(distanceM),
            _curvature(curvature),
            _speedMps(speedMps),
            _startDistanceM(startDistanceM)
        {
        }

        float DistanceM() const noexcept { return _distanceM; }
        float Curvature() const noexcept { return _curvature; }
        float SpeedMps() const noexcept { return _speedMps; }
        float StartDistanceM() const noexcept { return _startDistanceM; }

    private:
        float _distanceM{};
        float _curvature{};
        float _speedMps{};
        float _startDistanceM{};
    };

    class ManeuverPrimitive final
    {
    public:
        ManeuverPrimitive(
            const MazeMap::ManeuverInstance& maneuver,
            const float maneuverSpeedMps,
            const float startDistanceM,
            const MazeMap::ManeuverPoint& lastPoint,
            const bool lastPointValid) :
            _maneuver(maneuver),
            _startDistanceM(startDistanceM),
            _maneuverSpeedMps(maneuverSpeedMps),
            _lastPoint(lastPoint),
            _lastPointValid(lastPointValid)
        {
        }

        const MazeMap::ManeuverInstance& Maneuver() const noexcept { return _maneuver; }
        float StartDistanceM() const noexcept { return _startDistanceM; }
        float ManeuverSpeedMps() const noexcept { return _maneuverSpeedMps; }

        bool TryGetTrackedPoint(
            const float traveledM,
            const float totalDistanceM,
            const float desiredSpeedMps,
            MazeMap::ManeuverPoint& point)
        {
            if (!_maneuver.SupportsPointTracking())
            {
                return false;
            }

            MazeMap::ManeuverPoint candidate = _lastPoint;
            if (_maneuver.TryGetManeuverPoint(
                    (std::min)(traveledM, totalDistanceM),
                    desiredSpeedMps,
                    candidate,
                    Config::kCellSizeM))
            {
                _lastPoint = candidate;
                _lastPointValid = true;
            }

            if (!_lastPointValid)
            {
                return false;
            }

            point = _lastPoint;
            return true;
        }

    private:
        MazeMap::ManeuverInstance _maneuver{};
        float _startDistanceM{};
        float _maneuverSpeedMps{};
        MazeMap::ManeuverPoint _lastPoint{};
        bool _lastPointValid{};
    };

    static_assert(sizeof(HoldPrimitive) <= kPrimitiveStorageBytes);
    static_assert(sizeof(LinearMotionPrimitive) <= kPrimitiveStorageBytes);
    static_assert(sizeof(TurnPrimitive) <= kPrimitiveStorageBytes);
    static_assert(sizeof(TurnTransitionPrimitive) <= kPrimitiveStorageBytes);
    static_assert(sizeof(ArcPrimitive) <= kPrimitiveStorageBytes);
    static_assert(sizeof(ManeuverPrimitive) <= kPrimitiveStorageBytes);
    static_assert(alignof(HoldPrimitive) <= kPrimitiveStorageAlignment);
    static_assert(alignof(LinearMotionPrimitive) <= kPrimitiveStorageAlignment);
    static_assert(alignof(TurnPrimitive) <= kPrimitiveStorageAlignment);
    static_assert(alignof(TurnTransitionPrimitive) <= kPrimitiveStorageAlignment);
    static_assert(alignof(ArcPrimitive) <= kPrimitiveStorageAlignment);
    static_assert(alignof(ManeuverPrimitive) <= kPrimitiveStorageAlignment);

    namespace
    {
        float ResolveSignedPreference(std::initializer_list<float> signedValues) noexcept
        {
            for (const float value : signedValues)
            {
                const float direction =
                    !std::isnan(value) ?
                    SignF(value) :
                    0.0f;
                if (direction != 0.0f)
                {
                    return direction;
                }
            }

            return 0.0f;
        }

        float ResolveRequestedDirection(std::initializer_list<float> signedValues) noexcept
        {
            const float direction = ResolveSignedPreference(signedValues);
            return (direction != 0.0f) ? direction : 1.0f;
        }

        float ResolveRequestedMagnitude(
            const float primaryValue,
            const float secondaryValue = 0.0f,
            const float fallbackValue = 0.0f) noexcept
        {
            if (std::isinf(primaryValue))
            {
                return (std::numeric_limits<float>::infinity)();
            }

            const float primaryMagnitude =
                std::isfinite(primaryValue) ?
                std::fabs(primaryValue) :
                0.0f;
            if (primaryMagnitude > 0.0f)
            {
                return primaryMagnitude;
            }

            if (std::isinf(secondaryValue))
            {
                return (std::numeric_limits<float>::infinity)();
            }

            const float secondaryMagnitude =
                std::isfinite(secondaryValue) ?
                std::fabs(secondaryValue) :
                0.0f;
            if (secondaryMagnitude > 0.0f)
            {
                return secondaryMagnitude;
            }

            if (std::isinf(fallbackValue))
            {
                return (std::numeric_limits<float>::infinity)();
            }

            return
                std::isfinite(fallbackValue) ?
                std::fabs(fallbackValue) :
                0.0f;
        }

        MazeMap::Direction ResolveNearestCardinalDirectionFromYawRad(const float yawRad) noexcept
        {
            constexpr MazeMap::Direction kCardinalDirections[] =
            {
                MazeMap::Up,
                MazeMap::Right,
                MazeMap::Down,
                MazeMap::Left
            };

            const float resolvedYawRad =
                WrapAngleRad(
                    std::isfinite(yawRad) ?
                    yawRad :
                    0.0f);
            MazeMap::Direction bestDirection = MazeMap::Up;
            float bestAbsErrorRad = (std::numeric_limits<float>::infinity)();
            for (const MazeMap::Direction direction : kCardinalDirections)
            {
                const float absErrorRad = std::fabs(AngleErrorRad(DirectionToYawRad(direction), resolvedYawRad));
                if (absErrorRad < bestAbsErrorRad)
                {
                    bestAbsErrorRad = absErrorRad;
                    bestDirection = direction;
                }
            }

            return bestDirection;
        }

        bool TryGetCurrentMazeCell(
            const MazeMap::VehicleState& state,
            const MazeMap::Maze* maze,
            MazeMap::CellCoordinates& cell) noexcept
        {
            if (maze == nullptr)
            {
                return false;
            }

            const float positionXM = state.GetPositionX();
            const float positionYM = state.GetPositionY();
            if (!std::isfinite(positionXM) || !std::isfinite(positionYM))
            {
                return false;
            }

            const float cellDimensionM = MazeMap::Maze::GetCellDimension();
            if (!(cellDimensionM > 0.0f))
            {
                return false;
            }

            const int cellX = static_cast<int>(std::floor(positionXM / cellDimensionM));
            const int cellY = static_cast<int>(std::floor(positionYM / cellDimensionM));
            if ((cellX < 0) ||
                (cellY < 0) ||
                (cellX >= static_cast<int>(maze->GetXSize())) ||
                (cellY >= static_cast<int>(maze->GetYSize())))
            {
                return false;
            }

            cell = MazeMap::CellCoordinates(
                static_cast<std::uint8_t>(cellX),
                static_cast<std::uint8_t>(cellY));
            return true;
        }

        bool TryResolveMazeSideOpeningQuarterTurnAngleRad(
            const MazeMap::VehicleState& state,
            const MazeMap::Maze* maze,
            float& signedAngleRad) noexcept
        {
            MazeMap::CellCoordinates cell{};
            if (!TryGetCurrentMazeCell(state, maze, cell))
            {
                return false;
            }

            const MazeMap::Direction forward = ResolveNearestCardinalDirectionFromYawRad(state.GetOrientation());
            const MazeMap::Direction left = MazeMap::TurnLeft90(forward);
            const MazeMap::Direction right = MazeMap::TurnRight90(forward);
            const MazeMap::Cell& currentCell = (*maze)[cell];

            const MazeMap::WallState forwardWall = currentCell.GetWall(forward);
            const MazeMap::WallState leftWall = currentCell.GetWall(left);
            const MazeMap::WallState rightWall = currentCell.GetWall(right);
            if (forwardWall != MazeMap::Wall)
            {
                return false;
            }

            if ((leftWall == MazeMap::Wall) && (rightWall == MazeMap::NoWall))
            {
                signedAngleRad = HALF_PI_F;
                return true;
            }

            if ((rightWall == MazeMap::Wall) && (leftWall == MazeMap::NoWall))
            {
                signedAngleRad = -HALF_PI_F;
                return true;
            }

            if (rightWall == MazeMap::NoWall)
            {
                signedAngleRad = HALF_PI_F;
                return true;
            }

            if (leftWall == MazeMap::NoWall)
            {
                signedAngleRad = -HALF_PI_F;
                return true;
            }

            return false;
        }

        bool TryResolveSensorSideOpeningQuarterTurnAngleRad(
            const SensorSnapshot& sensors,
            float& signedAngleRad) noexcept
        {
            if (!sensors.frontWall)
            {
                return false;
            }

            if (sensors.leftWall && !sensors.rightWall)
            {
                signedAngleRad = HALF_PI_F;
                return true;
            }

            if (!sensors.leftWall && sensors.rightWall)
            {
                signedAngleRad = -HALF_PI_F;
                return true;
            }

            if (!sensors.leftWall && !sensors.rightWall)
            {
                signedAngleRad = HALF_PI_F;
                return true;
            }

            return false;
        }

        bool TryResolveContextualQuarterTurnAngleRad(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const MazeMap::Maze* maze,
            const Drive::OperationMode operationMode,
            float& signedAngleRad) noexcept
        {
            if (operationMode != Drive::OperationMode::Maze)
            {
                return false;
            }

            return
                TryResolveMazeSideOpeningQuarterTurnAngleRad(state, maze, signedAngleRad) ||
                TryResolveSensorSideOpeningQuarterTurnAngleRad(sensors, signedAngleRad);
        }

        float ResolveRecoveredQuarterTurnAngleRad(
            const float signedDescriptorHint,
            const MotionLimits& limits,
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const MazeMap::Maze* maze,
            const Drive::OperationMode operationMode) noexcept
        {
            float contextualAngleRad = 0.0f;
            const float preferredSign = ResolveSignedPreference({
                signedDescriptorHint,
                limits.GetMaxAngularSpeedRadps(),
                limits.GetAngularAccelRadps2() });
            if (TryResolveContextualQuarterTurnAngleRad(
                    state,
                    sensors,
                    maze,
                    operationMode,
                    contextualAngleRad))
            {
                return
                    (preferredSign != 0.0f) ?
                    (preferredSign * std::fabs(contextualAngleRad)) :
                    contextualAngleRad;
            }

            return ResolveRequestedDirection({
                signedDescriptorHint,
                limits.GetMaxAngularSpeedRadps(),
                limits.GetAngularAccelRadps2() }) * HALF_PI_F;
        }

        bool RecoverArcGeometry(
            const float requestedDistanceM,
            const float requestedCurvature,
            const MotionLimits& limits,
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const MazeMap::Maze* maze,
            const Drive::OperationMode operationMode,
            float& distanceM,
            float& curvature) noexcept
        {
            const bool hasDistanceDescriptor = !std::isnan(requestedDistanceM);
            const bool hasCurvatureDescriptor = !std::isnan(requestedCurvature);
            distanceM = ResolveRequestedMagnitude(requestedDistanceM);
            curvature = std::isfinite(requestedCurvature) ? requestedCurvature : 0.0f;

            if (hasDistanceDescriptor && std::isfinite(requestedCurvature))
            {
                return true;
            }

            const float recoveredAngleRad = ResolveRecoveredQuarterTurnAngleRad(
                requestedCurvature,
                limits,
                state,
                sensors,
                maze,
                operationMode);
            const float nominalRadiusM = 0.5f * MazeMap::Maze::GetCellDimension();
            const bool hasNominalRadius = nominalRadiusM > 0.0f;

            if (hasDistanceDescriptor)
            {
                if (!std::isfinite(distanceM))
                {
                    if (!hasNominalRadius)
                    {
                        return false;
                    }

                    curvature = SignF(recoveredAngleRad) / nominalRadiusM;
                    // Recovered curvature is only usable when it is finite and nonzero.
                    return std::isfinite(curvature) && (std::fabs(curvature) > 0.0f);
                }

                if (distanceM <= Config::kDistanceToleranceM)
                {
                    curvature = 0.0f;
                    return true;
                }

                curvature = recoveredAngleRad / distanceM;
                return std::isfinite(curvature) && (std::fabs(curvature) > 0.0f);
            }

            if (hasCurvatureDescriptor)
            {
                if (std::isfinite(requestedCurvature))
                {
                    if (!std::isfinite(curvature) || (std::fabs(curvature) <= 0.0f))
                    {
                        distanceM = (std::numeric_limits<float>::infinity)();
                        return true;
                    }

                    distanceM = std::fabs(recoveredAngleRad / curvature);
                    return std::isfinite(distanceM) && (distanceM > Config::kDistanceToleranceM);
                }

                if (!hasNominalRadius)
                {
                    return false;
                }

                curvature = SignF(recoveredAngleRad) / nominalRadiusM;
                if (!std::isfinite(curvature) || (std::fabs(curvature) <= 0.0f))
                {
                    return false;
                }

                distanceM = std::fabs(recoveredAngleRad / curvature);
                return std::isfinite(distanceM) && (distanceM > Config::kDistanceToleranceM);
            }

            if (!hasNominalRadius)
            {
                return false;
            }

            curvature = SignF(recoveredAngleRad) / nominalRadiusM;
            if (!std::isfinite(curvature) || (std::fabs(curvature) <= 0.0f))
            {
                return false;
            }

            distanceM = std::fabs(recoveredAngleRad / curvature);
            return std::isfinite(distanceM) && (distanceM > Config::kDistanceToleranceM);
        }

        float LimitByConfiguredMagnitude(const float command, const float configuredLimit) noexcept
        {
            const float finiteCommand = std::isfinite(command) ? command : 0.0f;
            if (!std::isfinite(configuredLimit))
            {
                return finiteCommand;
            }

            const float maxMagnitude = std::fabs(configuredLimit);
            return SignF(finiteCommand) * (std::min)(std::fabs(finiteCommand), maxMagnitude);
        }

        float LimitMagnitudeByConfiguredMagnitude(const float value, const float configuredLimit) noexcept
        {
            const float magnitude = std::isfinite(value) ? std::fabs(value) : 0.0f;
            if (!std::isfinite(configuredLimit))
            {
                return magnitude;
            }

            return (std::min)(magnitude, std::fabs(configuredLimit));
        }

        bool IsTurnComplete(
            const float remainingRad,
            const MotionLimits& limits) noexcept
        {
            return
                std::fabs(std::isfinite(remainingRad) ? remainingRad : std::numeric_limits<float>::infinity()) <=
                limits.GetEffectiveAngleToleranceRad();
        }

        bool IsMotionSettled(
            const SensorSnapshot& sensors,
            const DriveTelemetry& driveTelemetry,
            const float fanDuty) noexcept
        {
            const float baseThresholdMps = Config::kMotionSettleSpeedThresholdMps;
            return MazeMap::IsMissionStartupStationaryFromSensors(
                driveTelemetry.leftVelocityMps,
                driveTelemetry.rightVelocityMps,
                sensors.gyroRadps,
                (fanDuty > 0.0f) ? (baseThresholdMps * 5.0f) : baseThresholdMps,
                Config::kMotionSettleAngularSpeedThresholdRadps);
        }

        template <typename T>
        T* StorageAs(void* storage) noexcept
        {
            return std::launder(reinterpret_cast<T*>(storage));
        }

        template <typename T>
        const T* StorageAs(const void* storage) noexcept
        {
            return std::launder(reinterpret_cast<const T*>(storage));
        }

        bool TryResolveHeadingYawRadFromComponents(
            const float headingX,
            const float headingY,
            float& targetYawRad) noexcept
        {
            if (std::isinf(headingX) || std::isinf(headingY))
            {
                const float resolvedHeadingX = std::isinf(headingX) ? SignF(headingX) : 0.0f;
                const float resolvedHeadingY = std::isinf(headingY) ? SignF(headingY) : 0.0f;
                if ((resolvedHeadingX != 0.0f) || (resolvedHeadingY != 0.0f))
                {
                    targetYawRad = std::atan2(resolvedHeadingX, resolvedHeadingY);
                    return true;
                }
            }

            if (std::isfinite(headingX) && std::isfinite(headingY))
            {
                const float headingMagnitudeSq = (headingX * headingX) + (headingY * headingY);
                if (headingMagnitudeSq > 0.0f)
                {
                    targetYawRad = std::atan2(headingX, headingY);
                    return true;
                }
            }

            return false;
        }

        bool TryResolveProjectedAxisContribution(
            const float currentCoord,
            const float targetCoord,
            const float directionCoord,
            float& finiteContribution,
            bool& hasPositiveInfinityContribution,
            bool& hasNegativeInfinityContribution) noexcept
        {
            constexpr float kDirectionZeroTolerance = 1.0e-6f;

            if (!std::isfinite(currentCoord) || !std::isfinite(directionCoord))
            {
                return false;
            }

            if (std::fabs(directionCoord) <= kDirectionZeroTolerance)
            {
                finiteContribution = 0.0f;
                return std::isfinite(targetCoord) || std::isinf(targetCoord);
            }

            if (std::isfinite(targetCoord))
            {
                finiteContribution = (targetCoord - currentCoord) * directionCoord;
                return std::isfinite(finiteContribution);
            }

            if (std::isinf(targetCoord))
            {
                finiteContribution = 0.0f;
                const float contributionSign = SignF(targetCoord) * SignF(directionCoord);
                hasPositiveInfinityContribution = hasPositiveInfinityContribution || (contributionSign > 0.0f);
                hasNegativeInfinityContribution = hasNegativeInfinityContribution || (contributionSign < 0.0f);
                return true;
            }

            return false;
        }

        bool TryResolveStraightTargetDistanceM(
            const MazeMap::VehicleState* state,
            const float targetYawRad,
            const Eigen::Vector2f* targetPositionOverride,
            float& projectedTargetDistanceM) noexcept
        {
            if ((state == nullptr) || (targetPositionOverride == nullptr))
            {
                return false;
            }

            const Eigen::Vector2f headingUnit = HeadingUnitFromYawRad(targetYawRad);
            float xContribution = 0.0f;
            float yContribution = 0.0f;
            bool hasPositiveInfinityContribution = false;
            bool hasNegativeInfinityContribution = false;
            if (!TryResolveProjectedAxisContribution(
                    state->GetPositionX(),
                    targetPositionOverride->x(),
                    headingUnit.x(),
                    xContribution,
                    hasPositiveInfinityContribution,
                    hasNegativeInfinityContribution) ||
                !TryResolveProjectedAxisContribution(
                    state->GetPositionY(),
                    targetPositionOverride->y(),
                    headingUnit.y(),
                    yContribution,
                    hasPositiveInfinityContribution,
                    hasNegativeInfinityContribution))
            {
                return false;
            }

            if (hasPositiveInfinityContribution || hasNegativeInfinityContribution)
            {
                projectedTargetDistanceM =
                    (hasPositiveInfinityContribution && !hasNegativeInfinityContribution) ?
                    (std::numeric_limits<float>::infinity)() :
                    (hasNegativeInfinityContribution && !hasPositiveInfinityContribution) ?
                    -(std::numeric_limits<float>::infinity)() :
                    (std::numeric_limits<float>::infinity)();
            }
            else
            {
                projectedTargetDistanceM = xContribution + yContribution;
            }

            projectedTargetDistanceM = std::fabs(projectedTargetDistanceM);
            return !std::isnan(projectedTargetDistanceM);
        }

        float ResolveTurnCommandMagnitudeRadps(
            const float retainedYawRateRadps,
            const MotionLimits& limits,
            const MazeMap::Vehicle* vehicle) noexcept
        {
            if (std::isinf(retainedYawRateRadps))
            {
                return (std::numeric_limits<float>::infinity)();
            }

            const float retainedMagnitudeRadps =
                std::isfinite(retainedYawRateRadps) ?
                std::fabs(retainedYawRateRadps) :
                0.0f;
            if (retainedMagnitudeRadps > 0.0f)
            {
                return retainedMagnitudeRadps;
            }

            if (std::isfinite(limits.GetMaxAngularSpeedRadps()))
            {
                return std::fabs(limits.GetMaxAngularSpeedRadps());
            }

            if ((vehicle != nullptr) && std::isfinite(vehicle->GetMaxRotationalVelocity()))
            {
                return std::fabs(vehicle->GetMaxRotationalVelocity());
            }

            return 0.0f;
        }

        float ResolveManeuverSpeedMps(
            const MazeMap::ManeuverInstance& maneuver,
            const MotionLimits& limits,
            const MazeMap::Vehicle* vehicle) noexcept
        {
            const MazeMap::ManeuverCode baseCode =
                static_cast<MazeMap::ManeuverCode>(maneuver.getCode() & MazeMap::INVERTED_MIRRORED_MANEUVER_FLAG);
            float speedLimitMps = 0.0f;
            bool hasSpeedLimit = false;
            if (maneuver.getCode() != MazeMap::MC_NONE)
            {
                if ((baseCode >= MazeMap::S1) && (baseCode <= MazeMap::S31))
                {
                    if (std::isfinite(limits.GetMaxSpeedMps()))
                    {
                        speedLimitMps = std::fabs(limits.GetMaxSpeedMps());
                        hasSpeedLimit = true;
                    }
                }
                else if (vehicle != nullptr)
                {
                    const float maneuverLimitMps = maneuver.GetSpeedLimit(*vehicle);
                    const bool hasConfiguredLimit = std::isfinite(limits.GetMaxSpeedMps());
                    const bool hasManeuverLimit = std::isfinite(maneuverLimitMps);
                    if (hasConfiguredLimit && hasManeuverLimit)
                    {
                        speedLimitMps = (std::min)(std::fabs(limits.GetMaxSpeedMps()), std::fabs(maneuverLimitMps));
                        hasSpeedLimit = true;
                    }
                    else if (hasConfiguredLimit)
                    {
                        speedLimitMps = std::fabs(limits.GetMaxSpeedMps());
                        hasSpeedLimit = true;
                    }
                    else if (hasManeuverLimit)
                    {
                        speedLimitMps = std::fabs(maneuverLimitMps);
                        hasSpeedLimit = true;
                    }
                }
            }

            const float requestedDirection = ResolveRequestedDirection({
                maneuver.getEntrySpeed(),
                maneuver.getExitSpeed(),
                limits.GetMaxSpeedMps() });
            const float requestedMagnitudeMps = ResolveRequestedMagnitude(
                maneuver.getEntrySpeed(),
                maneuver.getExitSpeed(),
                hasSpeedLimit ? speedLimitMps : 0.0f);
            const float usableRequestedSpeedMps = requestedDirection * requestedMagnitudeMps;
            if (!hasSpeedLimit)
            {
                return usableRequestedSpeedMps;
            }

            return
                SignF(usableRequestedSpeedMps) *
                (std::min)(std::fabs(usableRequestedSpeedMps), speedLimitMps);
        }

        float ResolveSignedCommandForDriveBase(
            const float requestedCommand,
            const float configuredLimit,
            const float fallbackCommand = 0.0f) noexcept
        {
            float resolvedCommand = std::isfinite(fallbackCommand) ? fallbackCommand : 0.0f;
            if (std::isfinite(requestedCommand))
            {
                resolvedCommand = requestedCommand;
            }
            else if (std::isinf(requestedCommand))
            {
                resolvedCommand =
                    std::isfinite(configuredLimit) ?
                    (SignF(requestedCommand) * std::fabs(configuredLimit)) :
                    (SignF(requestedCommand) *
                        (std::isfinite(fallbackCommand) ? std::fabs(fallbackCommand) : 0.0f));
            }

            return LimitByConfiguredMagnitude(resolvedCommand, configuredLimit);
        }

        float ResolveFiniteOr(const float value, const float fallback) noexcept
        {
            return std::isfinite(value) ? value : fallback;
        }

        float ResolvePositiveLimitMagnitude(const float configuredLimit) noexcept
        {
            return
                std::isfinite(configuredLimit) ?
                std::fabs(configuredLimit) :
                (std::numeric_limits<float>::infinity)();
        }

        float ResolveNominalCommandDtSeconds(const float configuredCommandPeriodSeconds) noexcept
        {
            return
                (std::isfinite(configuredCommandPeriodSeconds) && (configuredCommandPeriodSeconds > 0.0f)) ?
                configuredCommandPeriodSeconds :
                (static_cast<float>(Config::kControlPeriodUs) * 1.0e-6f);
        }

        float ResolveUsableDtSeconds(const float dtSeconds) noexcept
        {
            return
                (std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ?
                    dtSeconds :
                    ResolveNominalCommandDtSeconds(0.0f);
        }

        bool IsSpeedMagnitudeIncreasing(const float presentSpeed, const float targetSpeed) noexcept
        {
            const float presentDirection = SignF(presentSpeed);
            const float targetDirection = SignF(targetSpeed);
            if (targetDirection == 0.0f)
            {
                return false;
            }

            if (presentDirection == 0.0f)
            {
                return true;
            }

            return
                (presentDirection == targetDirection) &&
                (std::fabs(targetSpeed) > std::fabs(presentSpeed));
        }

        float StepLinearSpeedByLimits(
            const float presentSpeedMps,
            const float targetSpeedMps,
            const MotionLimits& limits,
            const float dtSeconds) noexcept
        {
            const float resolvedPresentSpeedMps = ResolveFiniteOr(presentSpeedMps, 0.0f);
            const float resolvedTargetSpeedMps =
                ResolveSignedCommandForDriveBase(targetSpeedMps, limits.GetMaxSpeedMps(), resolvedPresentSpeedMps);
            const float deltaMps = resolvedTargetSpeedMps - resolvedPresentSpeedMps;
            if (deltaMps == 0.0f)
            {
                return resolvedPresentSpeedMps;
            }

            const float limitMps2 =
                IsSpeedMagnitudeIncreasing(resolvedPresentSpeedMps, resolvedTargetSpeedMps) ?
                ResolvePositiveLimitMagnitude(limits.GetAccelMps2()) :
                ResolvePositiveLimitMagnitude(limits.GetDecelMps2());
            const float maxDeltaMps = limitMps2 * ResolveUsableDtSeconds(dtSeconds);
            if (!std::isfinite(maxDeltaMps) || (std::fabs(deltaMps) <= maxDeltaMps))
            {
                return resolvedTargetSpeedMps;
            }

            return resolvedPresentSpeedMps + (SignF(deltaMps) * maxDeltaMps);
        }

        float StepAngularRateByLimits(
            const float presentYawRateRadps,
            const float targetYawRateRadps,
            const MotionLimits& limits,
            const float dtSeconds) noexcept
        {
            const float resolvedPresentYawRateRadps = ResolveFiniteOr(presentYawRateRadps, 0.0f);
            const float resolvedTargetYawRateRadps =
                ResolveSignedCommandForDriveBase(
                    targetYawRateRadps,
                    limits.GetMaxAngularSpeedRadps(),
                    resolvedPresentYawRateRadps);
            const float deltaRadps = resolvedTargetYawRateRadps - resolvedPresentYawRateRadps;
            if (deltaRadps == 0.0f)
            {
                return resolvedPresentYawRateRadps;
            }

            const float maxDeltaRadps =
                ResolvePositiveLimitMagnitude(limits.GetAngularAccelRadps2()) *
                ResolveUsableDtSeconds(dtSeconds);
            if (!std::isfinite(maxDeltaRadps) || (std::fabs(deltaRadps) <= maxDeltaRadps))
            {
                return resolvedTargetYawRateRadps;
            }

            return resolvedPresentYawRateRadps + (SignF(deltaRadps) * maxDeltaRadps);
        }

        float ResolveAccelerationFromStep(
            const float presentValue,
            const float nextValue,
            const float dtSeconds) noexcept
        {
            const float usableDtSeconds = ResolveUsableDtSeconds(dtSeconds);
            return (ResolveFiniteOr(nextValue, 0.0f) - ResolveFiniteOr(presentValue, 0.0f)) / usableDtSeconds;
        }

        CommandVector MakeDeltaControlVector(
            DriveBase* const drive,
            const float presentSpeedMps,
            const float nextSpeedMps,
            const float presentYawRateRadps,
            const float nextYawRateRadps,
            const float dtSeconds) noexcept
        {
            if (drive == nullptr)
            {
                return CommandVector::Brake();
            }

            const float desiredLongitudinalAccelMps2 =
                ResolveAccelerationFromStep(presentSpeedMps, nextSpeedMps, dtSeconds);
            const float desiredYawAccelRadps2 =
                ResolveAccelerationFromStep(presentYawRateRadps, nextYawRateRadps, dtSeconds);
            const CommandVector command =
                drive->DeltaCommand(
                    ResolveFiniteOr(presentSpeedMps, 0.0f),
                    desiredLongitudinalAccelMps2,
                    ResolveFiniteOr(presentYawRateRadps, 0.0f),
                    desiredYawAccelRadps2,
                    MazeMap::FeedbackSource::None,
                    MazeMap::FeedbackSource::None);
            return command;
        }

        float ResolveRecoveredTranslationSpeedMps(
            const float capturedSpeedMps,
            const float configuredLimit,
            const bool allowLimitFallback) noexcept
        {
            const float direction = ResolveRequestedDirection({ capturedSpeedMps, configuredLimit });
            const float magnitude = ResolveRequestedMagnitude(
                capturedSpeedMps,
                0.0f,
                allowLimitFallback ? configuredLimit : 0.0f);
            return ResolveSignedCommandForDriveBase(
                direction * magnitude,
                configuredLimit,
                capturedSpeedMps);
        }

        bool IsLinearMotionCompleteAtExit(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const DriveTelemetry& driveTelemetry,
            const float desiredSpeedMps,
            const float exitMagnitudeMps,
            const float fanDuty) noexcept
        {
            return (exitMagnitudeMps <= Config::kSpeedToleranceMps) ?
                IsMotionSettled(sensors, driveTelemetry, fanDuty) :
                (std::fabs((std::isfinite(state.GetVelocity()) ? state.GetVelocity() : desiredSpeedMps) - desiredSpeedMps) <=
                    Config::kSpeedToleranceMps);
        }

        CommandVector MakeFiniteTurnToHeadingControls(
            DriveBase* drive,
            const MotionLimits& limits,
            const MazeMap::VehicleState& state,
            const float targetYawRad,
            const float dtSeconds,
            bool& done) noexcept
        {
            const float measuredYawRad = std::isfinite(state.GetOrientation()) ? state.GetOrientation() : 0.0f;
            const float presentYawRateRadps = ResolveFiniteOr(state.GetRotationalVelocity(), 0.0f);
            const float remainingRad = AngleErrorRad(targetYawRad, measuredYawRad);
            if (IsTurnComplete(remainingRad, limits))
            {
                done = true;
                return CommandVector::Brake();
            }

            const float finiteRemainingRad = std::isfinite(remainingRad) ? remainingRad : 0.0f;
            const float targetYawRateRadps =
                IsTurnComplete(finiteRemainingRad, limits) ?
                0.0f :
                std::isfinite(limits.GetAngularAccelRadps2()) ?
                (SignF(finiteRemainingRad) *
                    ReachableSpeedWithBoundary(0.0f, std::fabs(finiteRemainingRad), std::fabs(limits.GetAngularAccelRadps2()))) :
                0.0f;
            const float desiredYawRateRadps =
                StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, limits, dtSeconds);
            return MakeDeltaControlVector(
                drive,
                0.0f,
                0.0f,
                presentYawRateRadps,
                desiredYawRateRadps,
                dtSeconds);
        }

    }

    Drive::Drive()
        : Drive(ResolveNominalCommandDtSeconds(0.0f))
    {
    }

    Drive::Drive(const float nominalCommandPeriodSeconds)
        : _nominalCommandPeriodSeconds(ResolveNominalCommandDtSeconds(nominalCommandPeriodSeconds))
        , _headingFeedbackSources(Config::kDriveHeadingFeedbackSources)
        , _yawRateFeedbackSources(Config::kDriveYawRateFeedbackSources)
        , _distanceFeedbackSources(Config::kDriveDistanceFeedbackSources)
        , _velocityFeedbackSources(Config::kDriveVelocityFeedbackSources)
    {
    }

    void Drive::SetOperationMode(const OperationMode mode) noexcept
    {
        _operationMode = mode;
    }

    Drive::OperationMode Drive::GetOperationMode() const noexcept
    {
        return _operationMode;
    }

    void Drive::SetLimits(const MotionLimits& limits) noexcept
    {
        _limits = limits;
    }

    const MotionLimits& Drive::GetLimits() const noexcept
    {
        return _limits;
    }

    float Drive::GetNominalCommandPeriodSeconds() const noexcept
    {
        return _nominalCommandPeriodSeconds;
    }

    bool Drive::IsEffectivelyComplete() const noexcept
    {
        return _effectivelyComplete;
    }

    void Drive::StartHold(const std::uint16_t durationMs, const bool requireContinuous)
    {
        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) HoldPrimitive(durationMs, requireContinuous);
        _activePrimitive = ActivePrimitive::Hold;
        _effectivelyComplete = false;
    }

    void Drive::StartStraight(
        float distanceM,
        float cruiseSpeed,
        float exitSpeed,
        const Eigen::Vector2f* targetHeadingOverride,
        const Eigen::Vector2f* targetPositionOverride)
    {
        const MazeMap::VehicleState* const runtimeState =
            (_runtime != nullptr) ? &_runtime->RuntimeState() : nullptr;
        const float capturedYawRad =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetOrientation()) ? runtimeState->GetOrientation() : 0.0f) :
            0.0f;

        // Resolve the latest straight request into one retained straight meaning now; later ticks
        // execute that retained meaning under live limits and live runtime state.
        float targetYawRad = capturedYawRad;
        if (targetHeadingOverride != nullptr)
        {
            (void)TryResolveHeadingYawRadFromComponents(
                targetHeadingOverride->x(),
                targetHeadingOverride->y(),
                targetYawRad);
        }
        const float resolvedTargetYawRad = WrapAngleRad(
            std::isfinite(targetYawRad) ? targetYawRad : capturedYawRad);
        float targetDistanceM = ResolveRequestedMagnitude(distanceM);
        float projectedTargetDistanceM = 0.0f;
        if (TryResolveStraightTargetDistanceM(
                runtimeState,
                resolvedTargetYawRad,
                targetPositionOverride,
                projectedTargetDistanceM))
        {
            targetDistanceM = projectedTargetDistanceM;
        }

        const float driveCommandMps = (_drive != nullptr) ? _drive->GetLastLinearCommandMps() : 0.0f;
        // Prefer the live translational estimate at arm time; otherwise continue from the last command.
        float commandedSpeedMps =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetVelocity()) ? runtimeState->GetVelocity() : driveCommandMps) :
            driveCommandMps;
        const float direction = ResolveRequestedDirection({
            cruiseSpeed,
            exitSpeed,
            commandedSpeedMps,
            _limits.GetMaxSpeedMps(),
            _limits.GetAccelMps2(),
            _limits.GetDecelMps2() });
        if (SignF(commandedSpeedMps) != direction)
        {
            commandedSpeedMps = 0.0f;
        }
        const float cruiseMagnitudeMps = ResolveRequestedMagnitude(cruiseSpeed, exitSpeed, _limits.GetMaxSpeedMps());
        const float exitMagnitudeMps = ResolveRequestedMagnitude(exitSpeed, cruiseSpeed, 0.0f);
        const float startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) LinearMotionPrimitive(
            targetDistanceM,
            cruiseMagnitudeMps,
            exitMagnitudeMps,
            resolvedTargetYawRad,
            direction,
            startDistanceM,
            commandedSpeedMps);
        _activePrimitive = ActivePrimitive::LinearMotion;
        _effectivelyComplete = false;
    }

    void Drive::StartTurn(float angleRad, MazeMap::TurnWallEdgeTracker* wallEdgeTracker)
    {
        const MazeMap::VehicleState* const runtimeState =
            (_runtime != nullptr) ? &_runtime->RuntimeState() : nullptr;
        const MazeMap::VehicleState defaultState{};
        const MazeMap::VehicleState& liveState =
            (runtimeState != nullptr) ? *runtimeState : defaultState;
        const float startYawRad =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetOrientation()) ? runtimeState->GetOrientation() : 0.0f) :
            0.0f;
        const float driveCommandRadps = (_drive != nullptr) ? _drive->GetLastAngularCommandRadps() : 0.0f;
        // Prefer the live rotational estimate at arm time; otherwise continue from the last command.
        const float initialYawRateRadps =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetRotationalVelocity()) ? runtimeState->GetRotationalVelocity() : driveCommandRadps) :
            driveCommandRadps;
        const bool hasFiniteStopCondition = !std::isinf(angleRad);
        const float turnDirection =
            hasFiniteStopCondition ?
                0.0f :
                ResolveRequestedDirection({
                    angleRad,
                    initialYawRateRadps,
                    _limits.GetMaxAngularSpeedRadps(),
                    _limits.GetAngularAccelRadps2() });
        const float resolvedAngleRad =
            hasFiniteStopCondition ?
                (std::isfinite(angleRad) ?
                    angleRad :
                    ResolveRecoveredQuarterTurnAngleRad(
                        angleRad,
                        _limits,
                        liveState,
                        liveState.GetSensorSnapshot(),
                        _maze,
                        _operationMode)) :
                0.0f;
        const float targetYawRad = WrapAngleRad(
            std::isfinite(startYawRad + resolvedAngleRad) ?
                (startYawRad + resolvedAngleRad) :
                startYawRad);
        float retainedYawRateRadps = initialYawRateRadps;
        if (hasFiniteStopCondition || (SignF(retainedYawRateRadps) != turnDirection))
        {
            retainedYawRateRadps = 0.0f;
        }

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) TurnPrimitive(
            targetYawRad,
            turnDirection,
            retainedYawRateRadps,
            hasFiniteStopCondition,
            wallEdgeTracker);
        _activePrimitive = ActivePrimitive::Turn;
        _effectivelyComplete = false;
    }

    void Drive::StartTurnTransition(float distanceM, float dCurvatureDs)
    {
        const MazeMap::VehicleState* const runtimeState =
            (_runtime != nullptr) ? &_runtime->RuntimeState() : nullptr;
        const float driveCommandMps = (_drive != nullptr) ? _drive->GetLastLinearCommandMps() : 0.0f;
        // Prefer the live translational estimate at arm time; otherwise continue from the last command.
        const float initialSpeedMps =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetVelocity()) ? runtimeState->GetVelocity() : driveCommandMps) :
            driveCommandMps;
        const float resolvedDistanceM = ResolveRequestedMagnitude(distanceM);
        const bool hasProgressObjective =
            !std::isfinite(resolvedDistanceM) ||
            (resolvedDistanceM > Config::kDistanceToleranceM);

        const float driveCommandRadps = (_drive != nullptr) ? _drive->GetLastAngularCommandRadps() : 0.0f;
        // Prefer the live rotational estimate at arm time; otherwise continue from the last command.
        const float initialYawRateRadps =
            (runtimeState != nullptr) ?
            (std::isfinite(runtimeState->GetRotationalVelocity()) ? runtimeState->GetRotationalVelocity() : driveCommandRadps) :
            driveCommandRadps;
        const float resolvedCurvatureRate = std::isfinite(dCurvatureDs) ? dCurvatureDs : 0.0f;
        const float speedMps = ResolveRecoveredTranslationSpeedMps(
            initialSpeedMps,
            _limits.GetMaxSpeedMps(),
            hasProgressObjective);
        const float startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) TurnTransitionPrimitive(
            resolvedDistanceM,
            resolvedCurvatureRate,
            speedMps,
            initialYawRateRadps,
            startDistanceM);
        _activePrimitive = ActivePrimitive::TurnTransition;
        _effectivelyComplete = false;
    }

    void Drive::StartArc(float distanceM, float curvature)
    {
        const MazeMap::VehicleState* const runtimeState =
            (_runtime != nullptr) ? &_runtime->RuntimeState() : nullptr;
        const MazeMap::VehicleState defaultState{};
        const MazeMap::VehicleState& liveState =
            (runtimeState != nullptr) ? *runtimeState : defaultState;
        const float driveCommandMps = (_drive != nullptr) ? _drive->GetLastLinearCommandMps() : 0.0f;
        // Prefer the live translational estimate at arm time; otherwise continue from the last command.
        const float initialSpeedMps =
            (runtimeState != nullptr) ?
                (std::isfinite(runtimeState->GetVelocity()) ? runtimeState->GetVelocity() : driveCommandMps) :
            driveCommandMps;

        const float startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;
        // Resolve the latest arc request into one retained arc meaning now; later ticks execute
        // that retained meaning under live limits and live runtime state.
        float resolvedDistanceM = 0.0f;
        float resolvedCurvature = 0.0f;
        (void)RecoverArcGeometry(
            distanceM,
            curvature,
            _limits,
            liveState,
            liveState.GetSensorSnapshot(),
            _maze,
            _operationMode,
            resolvedDistanceM,
            resolvedCurvature);
        const bool hasProgressObjective =
            !std::isfinite(resolvedDistanceM) ||
            (resolvedDistanceM > Config::kDistanceToleranceM);
        const float speedMps = ResolveRecoveredTranslationSpeedMps(
            initialSpeedMps,
            _limits.GetMaxSpeedMps(),
            hasProgressObjective);

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) ArcPrimitive(
            resolvedDistanceM,
            resolvedCurvature,
            speedMps,
            startDistanceM);
        _activePrimitive = ActivePrimitive::Arc;
        _effectivelyComplete = false;
    }

    void Drive::StartManeuver(const MazeMap::ManeuverInstance& maneuver)
    {
        const MazeMap::ManeuverCode code = maneuver.getCode();
        if (code == MazeMap::MC_NONE)
        {
            StartHold(0U, false);
            return;
        }

        const MazeMap::ManeuverCode baseCode =
            static_cast<MazeMap::ManeuverCode>(code & MazeMap::INVERTED_MIRRORED_MANEUVER_FLAG);
        if ((baseCode >= MazeMap::S1) && (baseCode <= MazeMap::S31))
        {
            StartStraight(
                maneuver.GetTravelDistanceMeters(Config::kCellSizeM),
                ResolveManeuverSpeedMps(maneuver, _limits, _vehicle),
                maneuver.getExitSpeed());
            return;
        }

        if ((baseCode == MazeMap::IP45) ||
            (baseCode == MazeMap::IP90) ||
            (baseCode == MazeMap::IP135) ||
            (baseCode == MazeMap::IP180))
        {
            // In-place maneuver codes are semantically pure turns, so arm the canonical turn
            // execution family now instead of keeping a second turn dialect in ManeuverControls(...).
            StartTurn(static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F);
            return;
        }

        const float maneuverSpeedMps = ResolveManeuverSpeedMps(maneuver, _limits, _vehicle);
        const float totalDistanceM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM);

        // Resolve the latest maneuver request into retained maneuver execution state now; later
        // ticks execute that retained meaning under live limits and live runtime state.
        MazeMap::ManeuverPoint lastPoint{};
        bool lastPointValid = false;
        if (maneuver.SupportsPointTracking())
        {
            if (maneuver.TryGetManeuverPoint(
                    0.0f,
                    maneuverSpeedMps,
                    lastPoint,
                    Config::kCellSizeM))
            {
                lastPointValid = totalDistanceM > Config::kDistanceToleranceM;
            }
        }

        const float startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) ManeuverPrimitive(
            maneuver,
            maneuverSpeedMps,
            startDistanceM,
            lastPoint,
            lastPointValid);
        _activePrimitive = ActivePrimitive::Maneuver;
        _effectivelyComplete = false;
    }

    CommandVector Drive::GetNextControls(bool& done)
    {
        done = false;
        if (_activePrimitive == ActivePrimitive::None)
        {
            done = true;
            _effectivelyComplete = true;
            return CommandVector::Brake();
        }

        const MazeMap::VehicleState commandState =
            (_runtime != nullptr) ? _runtime->RuntimeState() : MazeMap::VehicleState{};
        const SensorSnapshot& sensors = commandState.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = (_drive != nullptr) ? _drive->GetTelemetry() : DriveTelemetry{};

        CommandVector control = CommandVector::Brake();
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            control = HoldControls(sensors, driveTelemetry, done);
            break;
        case ActivePrimitive::LinearMotion:
            control = LinearMotionControls(commandState, sensors, driveTelemetry, done);
            break;
        case ActivePrimitive::Turn:
            control = TurnControls(commandState, sensors, done);
            break;
        case ActivePrimitive::TurnTransition:
            control = TurnTransitionControls(commandState, driveTelemetry, done);
            break;
        case ActivePrimitive::Arc:
            control = ArcControls(commandState, sensors, driveTelemetry, done);
            break;
        case ActivePrimitive::Maneuver:
            control = ManeuverControls(commandState, driveTelemetry, done);
            break;
        default:
            done = true;
            break;
        }

        _effectivelyComplete = done;
        return control;
    }

    void Drive::AttachRuntime(SharedRobotRuntime& runtime) noexcept
    {
        _runtime = &runtime;
        _drive = &runtime.DriveBase();
        _vehicle = &runtime.Vehicle();
        _maze = &runtime.Maze();
        MotionLimits motionCommandEnvelope{};
        motionCommandEnvelope.SetMaxSpeedMps(_vehicle->GetMaxSpeed());
        motionCommandEnvelope.SetAccelMps2(_vehicle->GetMaxForwardAcceleration());
        motionCommandEnvelope.SetDecelMps2(_vehicle->GetMaxForwardAcceleration());
        motionCommandEnvelope.SetMaxAngularSpeedRadps(_vehicle->GetMaxRotationalVelocity());
        motionCommandEnvelope.SetAngularAccelRadps2(_vehicle->GetMaxAngularAcceleration());
        SetLimits(motionCommandEnvelope);
    }

    void Drive::ResetActivePrimitive() noexcept
    {
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            StorageAs<HoldPrimitive>(_primitiveStorageWords)->~HoldPrimitive();
            break;
        case ActivePrimitive::LinearMotion:
            StorageAs<LinearMotionPrimitive>(_primitiveStorageWords)->~LinearMotionPrimitive();
            break;
        case ActivePrimitive::Turn:
            StorageAs<TurnPrimitive>(_primitiveStorageWords)->~TurnPrimitive();
            break;
        case ActivePrimitive::TurnTransition:
            StorageAs<TurnTransitionPrimitive>(_primitiveStorageWords)->~TurnTransitionPrimitive();
            break;
        case ActivePrimitive::Arc:
            StorageAs<ArcPrimitive>(_primitiveStorageWords)->~ArcPrimitive();
            break;
        case ActivePrimitive::Maneuver:
            StorageAs<ManeuverPrimitive>(_primitiveStorageWords)->~ManeuverPrimitive();
            break;
        default:
            break;
        }

        std::memset(_primitiveStorageWords, 0, sizeof(_primitiveStorageWords));
        _activePrimitive = ActivePrimitive::None;
    }

    CommandVector Drive::HoldControls(
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& hold = *StorageAs<HoldPrimitive>(_primitiveStorageWords);
        hold.ObserveStationaryState(IsMotionSettled(
            sensors,
            driveTelemetry,
            (_vehicle != nullptr) ? _vehicle->GetFanDuty() : 0.0f));
        done = hold.IsComplete();
        return CommandVector::Brake();
    }

    CommandVector Drive::LinearMotionControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& linear = *StorageAs<LinearMotionPrimitive>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - linear.StartDistanceM());
        const float resolvedTargetYawRad = linear.TargetYawRad();
        const bool hasFiniteStopCondition = std::isfinite(linear.TargetDistanceM());
        const float remainingM =
            hasFiniteStopCondition ?
            (std::max)(0.0f, linear.TargetDistanceM() - traveledM) :
            (std::numeric_limits<float>::infinity)();
        const float direction = linear.Direction();
        const float cruiseMagnitudeMps =
            LimitMagnitudeByConfiguredMagnitude(
                linear.CruiseMagnitudeMps(),
                _limits.GetMaxSpeedMps());
        const float exitMagnitudeMps =
            LimitMagnitudeByConfiguredMagnitude(
                linear.ExitMagnitudeMps(),
                _limits.GetMaxSpeedMps());
        const float usableDtSeconds = GetNominalCommandPeriodSeconds();
        const float presentSpeedMps =
            ResolveFiniteOr(state.GetVelocity(), driveTelemetry.commandedLinearSpeedMps);
        const float presentYawRateRadps =
            ResolveFiniteOr(state.GetRotationalVelocity(), driveTelemetry.commandedAngularSpeedRadps);

        float targetSpeedMps = ResolveSignedCommandForDriveBase(linear.CommandedSpeedMps(), _limits.GetMaxSpeedMps());
        if (remainingM > Config::kDistanceToleranceM)
        {
            // Only apply reachable-speed braking when the configured decel limit is usable.
            const float decelLimitedSpeedMps =
                std::isfinite(_limits.GetDecelMps2()) ?
                ReachableSpeedWithBoundary(exitMagnitudeMps, remainingM, std::fabs(_limits.GetDecelMps2())) :
                cruiseMagnitudeMps;
            targetSpeedMps = ResolveSignedCommandForDriveBase(
                direction * (std::min)(cruiseMagnitudeMps, decelLimitedSpeedMps),
                _limits.GetMaxSpeedMps(),
                linear.CommandedSpeedMps());
        }
        else
        {
            targetSpeedMps = ResolveSignedCommandForDriveBase(
                direction * exitMagnitudeMps,
                _limits.GetMaxSpeedMps(),
                direction * linear.CommandedSpeedMps());
        }

        const float desiredSpeedMps =
            StepLinearSpeedByLimits(presentSpeedMps, targetSpeedMps, _limits, usableDtSeconds);
        linear.SetCommandedSpeedMps(desiredSpeedMps);
        if (remainingM <= Config::kDistanceToleranceM)
        {
            done = IsLinearMotionCompleteAtExit(
                state,
                sensors,
                driveTelemetry,
                desiredSpeedMps,
                exitMagnitudeMps,
                (_vehicle != nullptr) ? _vehicle->GetFanDuty() : 0.0f);
        }

        float targetYawRateRadps =
            (_drive != nullptr) ?
            _drive->GetFeedbackCommand(
                0U,
                0.0f,
                MazeMap::FeedbackSource::None,
                0U,
                resolvedTargetYawRad,
                _headingFeedbackSources).Differential() :
            0.0f;
        if (!hasFiniteStopCondition)
        {
            done = true;
        }
        if ((_operationMode == OperationMode::Maze) &&
            IsApproximatelyDiagonalHeadingUnit(HeadingUnitFromYawRad(resolvedTargetYawRad)))
        {
            targetYawRateRadps += ComputeDiagonalWallCenterOmegaRadps(
                gWallDistanceCalibration,
                sensors.sideLeftDifferentialLight,
                sensors.sideRightDifferentialLight);
        }
        const float desiredYawRateRadps =
            StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, usableDtSeconds);

        return MakeDeltaControlVector(
            _drive,
            presentSpeedMps,
            desiredSpeedMps,
            presentYawRateRadps,
            desiredYawRateRadps,
            usableDtSeconds);
    }

    CommandVector Drive::TurnControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        bool& done)
    {
        auto& turn = *StorageAs<TurnPrimitive>(_primitiveStorageWords);
        const float usableDtSeconds = GetNominalCommandPeriodSeconds();
        turn.ObserveWallStates(sensors);

        if (!turn.HasFiniteStopCondition())
        {
            done = true;
            const float presentYawRateRadps =
                ResolveFiniteOr(state.GetRotationalVelocity(), 0.0f);
            const float targetYawRateRadps =
                ResolveSignedCommandForDriveBase(
                    turn.TurnDirection() * ResolveTurnCommandMagnitudeRadps(turn.RetainedYawRateRadps(), _limits, _vehicle),
                    _limits.GetMaxAngularSpeedRadps(),
                    turn.TurnDirection() * turn.RetainedYawRateRadps());
            const float desiredYawRateRadps =
                StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, usableDtSeconds);
            return MakeDeltaControlVector(
                _drive,
                0.0f,
                0.0f,
                presentYawRateRadps,
                desiredYawRateRadps,
                usableDtSeconds);
        }

        return MakeFiniteTurnToHeadingControls(
            _drive,
            _limits,
            state,
            turn.TargetYawRad(),
            usableDtSeconds,
            done);
    }

    CommandVector Drive::TurnTransitionControls(
        const MazeMap::VehicleState& state,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& transition = *StorageAs<TurnTransitionPrimitive>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - transition.StartDistanceM());
        const float distanceM = transition.DistanceM();
        const bool hasFiniteStopCondition = std::isfinite(distanceM);
        const float progressM =
            hasFiniteStopCondition ?
            (std::min)(traveledM, distanceM) :
            traveledM;
        const float initialSpeedMps =
            ResolveSignedCommandForDriveBase(
                transition.SpeedMps(),
                _limits.GetMaxSpeedMps(),
                transition.SpeedMps());
        const float initialYawRateRadps =
            ResolveSignedCommandForDriveBase(transition.InitialYawRateRadps(), _limits.GetMaxAngularSpeedRadps());
        const float resolvedCurvatureRate = transition.CurvatureRatePerMeter();
        const float finalYawRateRadps =
            initialYawRateRadps +
            (initialSpeedMps * resolvedCurvatureRate * distanceM);
        const float desiredYawRateRadps =
            initialYawRateRadps +
            (initialSpeedMps * resolvedCurvatureRate * progressM);
        done =
            hasFiniteStopCondition &&
            (traveledM >= (distanceM - Config::kDistanceToleranceM));
        if (!hasFiniteStopCondition)
        {
            done = true;
        }
        const float presentSpeedMps =
            ResolveFiniteOr(state.GetVelocity(), driveTelemetry.commandedLinearSpeedMps);
        const float presentYawRateRadps =
            ResolveFiniteOr(state.GetRotationalVelocity(), driveTelemetry.commandedAngularSpeedRadps);
        const float targetSpeedMps =
            ResolveSignedCommandForDriveBase(
                initialSpeedMps,
                _limits.GetMaxSpeedMps(),
                presentSpeedMps);
        const float targetYawRateRadps =
            ResolveSignedCommandForDriveBase(
                done ? finalYawRateRadps : desiredYawRateRadps,
                _limits.GetMaxAngularSpeedRadps(),
                initialYawRateRadps);
        return MakeDeltaControlVector(
            _drive,
            presentSpeedMps,
            StepLinearSpeedByLimits(presentSpeedMps, targetSpeedMps, _limits, GetNominalCommandPeriodSeconds()),
            presentYawRateRadps,
            StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, GetNominalCommandPeriodSeconds()),
            GetNominalCommandPeriodSeconds());
    }

    CommandVector Drive::ArcControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        (void)sensors;
        auto& arc = *StorageAs<ArcPrimitive>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - arc.StartDistanceM());
        const float distanceM = arc.DistanceM();
        const float curvature = arc.Curvature();
        const bool hasFiniteStopCondition = std::isfinite(distanceM);
        const float initialSpeedMps =
            ResolveSignedCommandForDriveBase(
                arc.SpeedMps(),
                _limits.GetMaxSpeedMps(),
                arc.SpeedMps());
        done = hasFiniteStopCondition ? (traveledM >= (distanceM - Config::kDistanceToleranceM)) : true;
        const float presentSpeedMps =
            ResolveFiniteOr(state.GetVelocity(), driveTelemetry.commandedLinearSpeedMps);
        const float presentYawRateRadps =
            ResolveFiniteOr(state.GetRotationalVelocity(), driveTelemetry.commandedAngularSpeedRadps);
        const float targetYawRateRadps =
            ResolveSignedCommandForDriveBase(
                initialSpeedMps * curvature,
                _limits.GetMaxAngularSpeedRadps());
        return MakeDeltaControlVector(
            _drive,
            presentSpeedMps,
            StepLinearSpeedByLimits(presentSpeedMps, initialSpeedMps, _limits, GetNominalCommandPeriodSeconds()),
            presentYawRateRadps,
            StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, GetNominalCommandPeriodSeconds()),
            GetNominalCommandPeriodSeconds());
    }

    CommandVector Drive::ManeuverControls(
        const MazeMap::VehicleState& state,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& maneuverState = *StorageAs<ManeuverPrimitive>(_primitiveStorageWords);
        const MazeMap::ManeuverInstance& maneuver = maneuverState.Maneuver();
        const MazeMap::ManeuverCode code = maneuver.getCode();
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - maneuverState.StartDistanceM());
        const float totalDistanceM = ResolveRequestedMagnitude(maneuver.GetTravelDistanceMeters(Config::kCellSizeM));
        const bool hasFiniteStopCondition = std::isfinite(totalDistanceM);
        const float desiredSpeedMps =
            ResolveSignedCommandForDriveBase(
                maneuverState.ManeuverSpeedMps(),
                _limits.GetMaxSpeedMps(),
                maneuverState.ManeuverSpeedMps());

        MazeMap::ManeuverPoint point{};
        if (maneuverState.TryGetTrackedPoint(traveledM, totalDistanceM, desiredSpeedMps, point))
        {
            done = !hasFiniteStopCondition || (traveledM >= (totalDistanceM - Config::kDistanceToleranceM));
            const float presentSpeedMps =
                ResolveFiniteOr(state.GetVelocity(), driveTelemetry.commandedLinearSpeedMps);
            const float presentYawRateRadps =
                ResolveFiniteOr(state.GetRotationalVelocity(), driveTelemetry.commandedAngularSpeedRadps);
            const float targetSpeedMps =
                ResolveSignedCommandForDriveBase(
                    point.Velocity,
                    _limits.GetMaxSpeedMps(),
                    desiredSpeedMps);
            const float targetYawRateRadps =
                ResolveSignedCommandForDriveBase(
                    point.Omega,
                    _limits.GetMaxAngularSpeedRadps());
            return MakeDeltaControlVector(
                _drive,
                presentSpeedMps,
                StepLinearSpeedByLimits(presentSpeedMps, targetSpeedMps, _limits, GetNominalCommandPeriodSeconds()),
                presentYawRateRadps,
                StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, GetNominalCommandPeriodSeconds()),
                GetNominalCommandPeriodSeconds());
        }

        const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        done = !hasFiniteStopCondition || (traveledM >= (totalDistanceM - Config::kDistanceToleranceM));
        const float presentSpeedMps =
            ResolveFiniteOr(state.GetVelocity(), driveTelemetry.commandedLinearSpeedMps);
        const float presentYawRateRadps =
            ResolveFiniteOr(state.GetRotationalVelocity(), driveTelemetry.commandedAngularSpeedRadps);
        const float targetYawRateRadps =
            ResolveSignedCommandForDriveBase(
                (std::fabs(totalDistanceM) > Config::kDistanceToleranceM) ?
                    (desiredSpeedMps * (angleRad / totalDistanceM)) :
                    0.0f,
                _limits.GetMaxAngularSpeedRadps());
        return MakeDeltaControlVector(
            _drive,
            presentSpeedMps,
            StepLinearSpeedByLimits(presentSpeedMps, desiredSpeedMps, _limits, GetNominalCommandPeriodSeconds()),
            presentYawRateRadps,
            StepAngularRateByLimits(presentYawRateRadps, targetYawRateRadps, _limits, GetNominalCommandPeriodSeconds()),
            GetNominalCommandPeriodSeconds());
    }
}
