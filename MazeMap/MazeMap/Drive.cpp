#include "pch.h"
#include "Drive.h"

#include "DriveBase.h"
#include "Maze.h"
#include "SharedRobotRuntime.h"
#include "MissionStartPolicy.h"
#include "MotionTargetProjection.h"
#include "TurnWallEdgeTracker.h"
#include "WallDistanceCalibration.h"
#include "WheelControlProfile.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <new>

namespace MazeMap::App::Internal
{
    namespace
    {
        using ControlVector = LoopController::ControlVector;

        constexpr std::size_t kPrimitiveStorageBytes = 16U * sizeof(std::uint32_t);
        constexpr std::size_t kPrimitiveStorageAlignment = 16U;
        struct HoldState final
        {
            std::uint16_t requestedTicks{};
            std::uint16_t remainingTicks{};
            bool resetOnNonStationary{};
        };

        struct LinearMotionState final
        {
            float targetDistanceM{};
            float cruiseMagnitudeMps{};
            float exitMagnitudeMps{};
            float targetYawRad{};
            float direction{};
            float startDistanceM{};
            float commandedSpeedMps{};
        };

        struct TurnState final
        {
            float targetYawRad{};
            float turnDirection{};
            float retainedYawRateRadps{};
            bool hasFiniteStopCondition{};
            MazeMap::TurnWallEdgeTracker* wallEdgeTracker{};
        };

        struct TurnTransitionState final
        {
            float distanceM{};
            float dCurvatureDs{};
            float speedMps{};
            float initialYawRateRadps{};
            float startDistanceM{};
        };

        struct ArcState final
        {
            float distanceM{};
            float curvature{};
            float speedMps{};
            float startDistanceM{};
        };

        struct ManeuverState final
        {
            MazeMap::ManeuverInstance maneuver{};
            float startDistanceM{};
            float maneuverSpeedMps{};
            MazeMap::ManeuverPoint lastPoint{};
            bool lastPointValid{};
        };

        static_assert(sizeof(HoldState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(LinearMotionState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(TurnState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(TurnTransitionState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(ArcState) <= kPrimitiveStorageBytes);
        static_assert(sizeof(ManeuverState) <= kPrimitiveStorageBytes);
        static_assert(alignof(HoldState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(LinearMotionState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(TurnState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(TurnTransitionState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(ArcState) <= kPrimitiveStorageAlignment);
        static_assert(alignof(ManeuverState) <= kPrimitiveStorageAlignment);

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
                limits.maxAngularSpeedRadps,
                limits.angularAccelRadps2 });
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
                limits.maxAngularSpeedRadps,
                limits.angularAccelRadps2 }) * HALF_PI_F;
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

        bool WithinConfiguredTolerance(
            const float error,
            const float configuredTolerance,
            const float fallbackTolerance) noexcept
        {
            const float tolerance =
                std::isfinite(configuredTolerance) ?
                std::fabs(configuredTolerance) :
                std::fabs(std::isfinite(fallbackTolerance) ? fallbackTolerance : 0.0f);
            return std::fabs(std::isfinite(error) ? error : std::numeric_limits<float>::infinity()) <= tolerance;
        }

        bool IsTurnComplete(
            const float remainingRad,
            const float measuredYawRateRadps,
            const MotionLimits& limits) noexcept
        {
            return
                WithinConfiguredTolerance(remainingRad, limits.angleToleranceRad, Config::kAngleToleranceRad) &&
                WithinConfiguredTolerance(
                    measuredYawRateRadps,
                    limits.angularSpeedToleranceRadps,
                    Config::kAngularSpeedToleranceRadps);
        }

        bool IsMotionSettled(const SensorSnapshot& sensors, const DriveTelemetry& driveTelemetry) noexcept
        {
            const float baseThresholdMps = Config::kMotionSettleSpeedThresholdMps;
            return MazeMap::IsMissionStartupStationaryFromSensors(
                driveTelemetry.leftVelocityMps,
                driveTelemetry.rightVelocityMps,
                sensors.gyroRadps,
                (GetMissionFanDutyCycle() > 0.0f) ? (baseThresholdMps * 5.0f) : baseThresholdMps,
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

            if (std::isfinite(limits.maxAngularSpeedRadps))
            {
                return std::fabs(limits.maxAngularSpeedRadps);
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
                    if (std::isfinite(limits.maxSpeedMps))
                    {
                        speedLimitMps = std::fabs(limits.maxSpeedMps);
                        hasSpeedLimit = true;
                    }
                }
                else if (vehicle != nullptr)
                {
                    const float maneuverLimitMps = maneuver.GetSpeedLimit(*vehicle);
                    const bool hasConfiguredLimit = std::isfinite(limits.maxSpeedMps);
                    const bool hasManeuverLimit = std::isfinite(maneuverLimitMps);
                    if (hasConfiguredLimit && hasManeuverLimit)
                    {
                        speedLimitMps = (std::min)(std::fabs(limits.maxSpeedMps), std::fabs(maneuverLimitMps));
                        hasSpeedLimit = true;
                    }
                    else if (hasConfiguredLimit)
                    {
                        speedLimitMps = std::fabs(limits.maxSpeedMps);
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
                limits.maxSpeedMps });
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

        LoopController::ControlVector MakePointControlVector(
            DriveBase* drive,
            const float desiredSpeedMps,
            const float desiredYawRateRadps,
            const MazeMap::CommandPD commandPd) noexcept
        {
            return (drive != nullptr) ?
                drive->PointControlVector(
                    std::isfinite(desiredSpeedMps) ? desiredSpeedMps : 0.0f,
                    std::isfinite(desiredYawRateRadps) ? desiredYawRateRadps : 0.0f,
                    commandPd) :
                LoopController::ControlVector::Brake;
        }

        LoopController::ControlVector MakePointControlVectorWithHeadingTarget(
            DriveBase* drive,
            const float desiredSpeedMps,
            const float desiredYawRateRadps,
            const float targetYawRad,
            const MazeMap::CommandPD velocityAndYawPd,
            const MazeMap::CommandPD headingPd) noexcept
        {
            return (drive != nullptr) ?
                drive->PointControlVectorWithHeadingTarget(
                    std::isfinite(desiredSpeedMps) ? desiredSpeedMps : 0.0f,
                    std::isfinite(desiredYawRateRadps) ? desiredYawRateRadps : 0.0f,
                    std::isfinite(targetYawRad) ? targetYawRad : 0.0f,
                    velocityAndYawPd,
                    headingPd) :
                LoopController::ControlVector::Brake;
        }

        bool IsLinearMotionCompleteAtExit(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const DriveTelemetry& driveTelemetry,
            const float desiredSpeedMps,
            const float exitMagnitudeMps) noexcept
        {
            return (exitMagnitudeMps <= Config::kSpeedToleranceMps) ?
                IsMotionSettled(sensors, driveTelemetry) :
                (std::fabs((std::isfinite(state.GetVelocity()) ? state.GetVelocity() : desiredSpeedMps) - desiredSpeedMps) <=
                    Config::kSpeedToleranceMps);
        }

        ControlVector MakeFiniteTurnToHeadingControls(
            DriveBase* drive,
            const MotionLimits& limits,
            const Drive::CommandPDSettings& commandPdSettings,
            const MazeMap::VehicleState& state,
            const float targetYawRad,
            bool& done) noexcept
        {
            const float measuredYawRad = std::isfinite(state.GetOrientation()) ? state.GetOrientation() : 0.0f;
            const float remainingRad = AngleErrorRad(targetYawRad, measuredYawRad);
            if (IsTurnComplete(remainingRad, state.GetRotationalVelocity(), limits))
            {
                done = true;
                return ControlVector::Brake;
            }

            const float finiteRemainingRad = std::isfinite(remainingRad) ? remainingRad : 0.0f;
            const float feedforwardYawRateRadps =
                std::isfinite(limits.angularAccelRadps2) ?
                (SignF(finiteRemainingRad) *
                    ReachableSpeedWithBoundary(0.0f, std::fabs(finiteRemainingRad), std::fabs(limits.angularAccelRadps2))) :
                0.0f;
            return MakePointControlVectorWithHeadingTarget(
                drive,
                0.0f,
                ResolveSignedCommandForDriveBase(feedforwardYawRateRadps, limits.maxAngularSpeedRadps),
                targetYawRad,
                commandPdSettings.yawRate,
                commandPdSettings.heading);
        }

    }

    Drive::Drive()
    {
        _commandPdSettings.heading = Config::kDriveHeadingCommandPd;
        _commandPdSettings.yawRate = Config::kDriveYawRateCommandPd;
        _commandPdSettings.velocity = Config::kDriveVelocityCommandPd;
        _commandPdSettings.distance = Config::kDriveDistanceCommandPd;
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

    void Drive::SetCommandPDSettings(const CommandPDSettings& settings) noexcept
    {
        _commandPdSettings = settings;
    }

    const Drive::CommandPDSettings& Drive::GetCommandPDSettings() const noexcept
    {
        return _commandPdSettings;
    }

    bool Drive::IsEffectivelyComplete() const noexcept
    {
        return _effectivelyComplete;
    }

    void Drive::StartHold(const std::uint16_t durationMs, const bool requireContinuous)
    {
        HoldState compiledHold{};
        compiledHold.requestedTicks = durationMs;
        compiledHold.remainingTicks = durationMs;
        compiledHold.resetOnNonStationary = requireContinuous;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) HoldState{ compiledHold };
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
        LinearMotionState compiledLinear{};

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
        compiledLinear.targetYawRad = WrapAngleRad(
            std::isfinite(targetYawRad) ? targetYawRad : capturedYawRad);
        compiledLinear.targetDistanceM = ResolveRequestedMagnitude(distanceM);
        float projectedTargetDistanceM = 0.0f;
        if (TryResolveStraightTargetDistanceM(
                runtimeState,
                compiledLinear.targetYawRad,
                targetPositionOverride,
                projectedTargetDistanceM))
        {
            compiledLinear.targetDistanceM = projectedTargetDistanceM;
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
            _limits.maxSpeedMps,
            _limits.accelMps2,
            _limits.decelMps2 });
        if (SignF(commandedSpeedMps) != direction)
        {
            commandedSpeedMps = 0.0f;
        }
        compiledLinear.direction = direction;
        compiledLinear.cruiseMagnitudeMps =
            ResolveRequestedMagnitude(
                cruiseSpeed,
                exitSpeed,
                _limits.maxSpeedMps);
        compiledLinear.exitMagnitudeMps =
            ResolveRequestedMagnitude(
                exitSpeed,
                cruiseSpeed,
                0.0f);
        compiledLinear.startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;
        compiledLinear.commandedSpeedMps = commandedSpeedMps;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) LinearMotionState{ compiledLinear };
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
        TurnState compiledTurn{};
        compiledTurn.hasFiniteStopCondition = !std::isinf(angleRad);
        compiledTurn.turnDirection =
            compiledTurn.hasFiniteStopCondition ?
                0.0f :
                ResolveRequestedDirection({
                    angleRad,
                    initialYawRateRadps,
                    _limits.maxAngularSpeedRadps,
                    _limits.angularAccelRadps2 });
        const float resolvedAngleRad =
            compiledTurn.hasFiniteStopCondition ?
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
        compiledTurn.targetYawRad = WrapAngleRad(
            std::isfinite(startYawRad + resolvedAngleRad) ?
                (startYawRad + resolvedAngleRad) :
                startYawRad);
        float retainedYawRateRadps = initialYawRateRadps;
        if (compiledTurn.hasFiniteStopCondition || (SignF(retainedYawRateRadps) != compiledTurn.turnDirection))
        {
            retainedYawRateRadps = 0.0f;
        }
        compiledTurn.retainedYawRateRadps = retainedYawRateRadps;
        compiledTurn.wallEdgeTracker = wallEdgeTracker;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) TurnState{ compiledTurn };
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

        TurnTransitionState compiledTransition{};
        compiledTransition.distanceM = resolvedDistanceM;
        compiledTransition.dCurvatureDs = std::isfinite(dCurvatureDs) ? dCurvatureDs : 0.0f;
        compiledTransition.speedMps = ResolveRecoveredTranslationSpeedMps(
            initialSpeedMps,
            _limits.maxSpeedMps,
            hasProgressObjective);
        compiledTransition.initialYawRateRadps = initialYawRateRadps;
        compiledTransition.startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) TurnTransitionState{ compiledTransition };
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

        ArcState compiledArc{};
        compiledArc.startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;
        // Resolve the latest arc request into one retained arc meaning now; later ticks execute
        // that retained meaning under live limits and live runtime state.
        (void)RecoverArcGeometry(
            distanceM,
            curvature,
            _limits,
            liveState,
            liveState.GetSensorSnapshot(),
            _maze,
            _operationMode,
            compiledArc.distanceM,
            compiledArc.curvature);
        const bool hasProgressObjective =
            !std::isfinite(compiledArc.distanceM) ||
            (compiledArc.distanceM > Config::kDistanceToleranceM);
        compiledArc.speedMps = ResolveRecoveredTranslationSpeedMps(
            initialSpeedMps,
            _limits.maxSpeedMps,
            hasProgressObjective);

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) ArcState{ compiledArc };
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
                ResolveManeuverSpeedMps(maneuver, _limits, _speedVehicle),
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

        ManeuverState compiledManeuver{};
        compiledManeuver.maneuver = maneuver;
        compiledManeuver.maneuverSpeedMps = ResolveManeuverSpeedMps(maneuver, _limits, _speedVehicle);
        const float totalDistanceM = maneuver.GetTravelDistanceMeters(Config::kCellSizeM);

        // Resolve the latest maneuver request into retained maneuver execution state now; later
        // ticks execute that retained meaning under live limits and live runtime state.
        if (maneuver.SupportsPointTracking())
        {
            if (maneuver.TryGetManeuverPoint(
                    0.0f,
                    compiledManeuver.maneuverSpeedMps,
                    compiledManeuver.lastPoint,
                    Config::kCellSizeM))
            {
                compiledManeuver.lastPointValid = totalDistanceM > Config::kDistanceToleranceM;
            }
        }

        compiledManeuver.startDistanceM =
            // Latch wheel-average distance so primitive progress stays relative to this start point.
            (_drive != nullptr) ? _drive->GetAverageDistanceMeters() : 0.0f;

        ResetActivePrimitive();
        (void)::new (_primitiveStorageWords) ManeuverState{ compiledManeuver };
        _activePrimitive = ActivePrimitive::Maneuver;
        _effectivelyComplete = false;
    }

    LoopController::ControlVector Drive::GetNextControls(bool& done)
    {
        done = false;
        if (_activePrimitive == ActivePrimitive::None)
        {
            done = true;
            _effectivelyComplete = true;
            return LoopController::ControlVector::Brake;
        }

        const MazeMap::VehicleState commandState =
            (_runtime != nullptr) ? _runtime->RuntimeState() : MazeMap::VehicleState{};
        const SensorSnapshot& sensors = commandState.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = (_drive != nullptr) ? _drive->GetTelemetry() : DriveTelemetry{};
        const float dtSeconds = (_loopController != nullptr) ? _loopController->CurrentTickDtSeconds() : 0.0f;

        ControlVector control = LoopController::ControlVector::Brake;
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            control = HoldControls(sensors, driveTelemetry, done);
            break;
        case ActivePrimitive::LinearMotion:
            control = LinearMotionControls(commandState, sensors, driveTelemetry, dtSeconds, done);
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
            control = ManeuverControls(driveTelemetry, done);
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
        _loopController = &runtime.ControlLoop();
        _drive = &runtime.Drive();
        _speedVehicle = &runtime.SpeedVehicle();
        _maze = &runtime.Maze();
        MotionLimits runtimeLimits{};
        runtimeLimits.maxSpeedMps = _speedVehicle->GetMaxSpeed();
        runtimeLimits.accelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        runtimeLimits.decelMps2 = _speedVehicle->GetMaxForwardAcceleration();
        runtimeLimits.maxAngularSpeedRadps = _speedVehicle->GetMaxRotationalVelocity();
        runtimeLimits.angularAccelRadps2 = _speedVehicle->GetMaxAngularAcceleration();
        SetLimits(runtimeLimits);
    }

    void Drive::ResetActivePrimitive() noexcept
    {
        switch (_activePrimitive)
        {
        case ActivePrimitive::Hold:
            StorageAs<HoldState>(_primitiveStorageWords)->~HoldState();
            break;
        case ActivePrimitive::LinearMotion:
            StorageAs<LinearMotionState>(_primitiveStorageWords)->~LinearMotionState();
            break;
        case ActivePrimitive::Turn:
            StorageAs<TurnState>(_primitiveStorageWords)->~TurnState();
            break;
        case ActivePrimitive::TurnTransition:
            StorageAs<TurnTransitionState>(_primitiveStorageWords)->~TurnTransitionState();
            break;
        case ActivePrimitive::Arc:
            StorageAs<ArcState>(_primitiveStorageWords)->~ArcState();
            break;
        case ActivePrimitive::Maneuver:
            StorageAs<ManeuverState>(_primitiveStorageWords)->~ManeuverState();
            break;
        default:
            break;
        }

        std::memset(_primitiveStorageWords, 0, sizeof(_primitiveStorageWords));
        _activePrimitive = ActivePrimitive::None;
    }

    LoopController::ControlVector Drive::HoldControls(
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& hold = *StorageAs<HoldState>(_primitiveStorageWords);
        const bool stationary = IsMotionSettled(sensors, driveTelemetry);
        if (stationary)
        {
            if (hold.remainingTicks > 0U)
            {
                --hold.remainingTicks;
            }
        }
        else if (hold.resetOnNonStationary)
        {
            hold.remainingTicks = hold.requestedTicks;
        }

        done = hold.remainingTicks == 0U;
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector Drive::LinearMotionControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        const float dtSeconds,
        bool& done)
    {
        auto& linear = *StorageAs<LinearMotionState>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - linear.startDistanceM);
        const float resolvedTargetYawRad = linear.targetYawRad;
        const bool hasFiniteStopCondition = std::isfinite(linear.targetDistanceM);
        const float remainingM =
            hasFiniteStopCondition ?
            (std::max)(0.0f, linear.targetDistanceM - traveledM) :
            (std::numeric_limits<float>::infinity)();
        const float direction = linear.direction;
        const float cruiseMagnitudeMps =
            LimitMagnitudeByConfiguredMagnitude(
                linear.cruiseMagnitudeMps,
                _limits.maxSpeedMps);
        const float exitMagnitudeMps =
            LimitMagnitudeByConfiguredMagnitude(
                linear.exitMagnitudeMps,
                _limits.maxSpeedMps);
        const float usableDtSeconds =
            (std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ?
                dtSeconds :
                (static_cast<float>(Config::kControlPeriodUs) * 1.0e-6f);

        float desiredSpeedMps = ResolveSignedCommandForDriveBase(linear.commandedSpeedMps, _limits.maxSpeedMps);
        if (remainingM > Config::kDistanceToleranceM)
        {
            const float accelLimitedSpeedMps = (std::min)(
                cruiseMagnitudeMps,
                std::isfinite(_limits.accelMps2) ?
                    (std::fabs(std::isfinite(linear.commandedSpeedMps) ? linear.commandedSpeedMps : 0.0f) +
                        (std::fabs(_limits.accelMps2) * usableDtSeconds)) :
                    cruiseMagnitudeMps);
            // Only apply reachable-speed braking when the configured decel limit is usable.
            const float decelLimitedSpeedMps =
                std::isfinite(_limits.decelMps2) ?
                ReachableSpeedWithBoundary(exitMagnitudeMps, remainingM, std::fabs(_limits.decelMps2)) :
                cruiseMagnitudeMps;
            linear.commandedSpeedMps = ResolveSignedCommandForDriveBase(
                direction * (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps),
                _limits.maxSpeedMps,
                linear.commandedSpeedMps);
            desiredSpeedMps = linear.commandedSpeedMps;
        }
        else
        {
            desiredSpeedMps = ResolveSignedCommandForDriveBase(
                direction * exitMagnitudeMps,
                _limits.maxSpeedMps,
                direction * linear.commandedSpeedMps);
            done = IsLinearMotionCompleteAtExit(
                state,
                sensors,
                driveTelemetry,
                desiredSpeedMps,
                exitMagnitudeMps);
        }

        float desiredYawRateRadps = 0.0f;
        if (!hasFiniteStopCondition)
        {
            done = true;
        }
        if ((_operationMode == OperationMode::Maze) &&
            IsApproximatelyDiagonalHeadingUnit(HeadingUnitFromYawRad(resolvedTargetYawRad)))
        {
            desiredYawRateRadps += ComputeDiagonalWallCenterOmegaRadps(
                gWallDistanceCalibration,
                sensors.sideLeftDifferentialLight,
                sensors.sideRightDifferentialLight);
        }
        desiredYawRateRadps = LimitByConfiguredMagnitude(desiredYawRateRadps, _limits.maxAngularSpeedRadps);

        return MakePointControlVectorWithHeadingTarget(
            _drive,
            desiredSpeedMps,
            desiredYawRateRadps,
            resolvedTargetYawRad,
            _commandPdSettings.velocity | _commandPdSettings.yawRate,
            _commandPdSettings.heading);
    }

    LoopController::ControlVector Drive::TurnControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        bool& done)
    {
        auto& turn = *StorageAs<TurnState>(_primitiveStorageWords);
        if (turn.wallEdgeTracker != nullptr)
        {
            MazeMap::ObserveTurnWallStates(*turn.wallEdgeTracker, sensors.leftWall, sensors.rightWall);
        }

        if (!turn.hasFiniteStopCondition)
        {
            done = true;
            const float desiredYawRateRadps =
                ResolveSignedCommandForDriveBase(
                    turn.turnDirection * ResolveTurnCommandMagnitudeRadps(turn.retainedYawRateRadps, _limits, _speedVehicle),
                    _limits.maxAngularSpeedRadps,
                    turn.turnDirection * turn.retainedYawRateRadps);
            return MakePointControlVector(
                _drive,
                0.0f,
                desiredYawRateRadps,
                _commandPdSettings.yawRate);
        }

        return MakeFiniteTurnToHeadingControls(
            _drive,
            _limits,
            _commandPdSettings,
            state,
            turn.targetYawRad,
            done);
    }

    LoopController::ControlVector Drive::TurnTransitionControls(
        const MazeMap::VehicleState& state,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& transition = *StorageAs<TurnTransitionState>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - transition.startDistanceM);
        const float distanceM = transition.distanceM;
        const bool hasFiniteStopCondition = std::isfinite(distanceM);
        const float progressM =
            hasFiniteStopCondition ?
            (std::min)(traveledM, distanceM) :
            traveledM;
        const float initialSpeedMps =
            ResolveSignedCommandForDriveBase(
                transition.speedMps,
                _limits.maxSpeedMps,
                transition.speedMps);
        const float initialYawRateRadps =
            ResolveSignedCommandForDriveBase(transition.initialYawRateRadps, _limits.maxAngularSpeedRadps);
        const float resolvedCurvatureRate = transition.dCurvatureDs;
        const float finalYawRateRadps =
            initialYawRateRadps +
            (initialSpeedMps * resolvedCurvatureRate * distanceM);
        const float desiredYawRateRadps =
            initialYawRateRadps +
            (initialSpeedMps * resolvedCurvatureRate * progressM);
        done =
            hasFiniteStopCondition &&
            (traveledM >= (distanceM - Config::kDistanceToleranceM)) &&
            WithinConfiguredTolerance(
                state.GetRotationalVelocity() - finalYawRateRadps,
                _limits.angularSpeedToleranceRadps,
                Config::kAngularSpeedToleranceRadps);
        if (!hasFiniteStopCondition)
        {
            done = true;
        }
        return MakePointControlVector(
            _drive,
            initialSpeedMps,
            ResolveSignedCommandForDriveBase(
                done ? finalYawRateRadps : desiredYawRateRadps,
                _limits.maxAngularSpeedRadps,
                initialYawRateRadps),
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
    }

    LoopController::ControlVector Drive::ArcControls(
        const MazeMap::VehicleState& state,
        const SensorSnapshot& sensors,
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        (void)state;
        (void)sensors;
        auto& arc = *StorageAs<ArcState>(_primitiveStorageWords);
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - arc.startDistanceM);
        const float distanceM = arc.distanceM;
        const float curvature = arc.curvature;
        const bool hasFiniteStopCondition = std::isfinite(distanceM);
        const float initialSpeedMps =
            ResolveSignedCommandForDriveBase(
                arc.speedMps,
                _limits.maxSpeedMps,
                arc.speedMps);
        done = hasFiniteStopCondition ? (traveledM >= (distanceM - Config::kDistanceToleranceM)) : true;
        return MakePointControlVector(
            _drive,
            initialSpeedMps,
            ResolveSignedCommandForDriveBase(
                initialSpeedMps * curvature,
                _limits.maxAngularSpeedRadps),
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
    }

    LoopController::ControlVector Drive::ManeuverControls(
        const DriveTelemetry& driveTelemetry,
        bool& done)
    {
        auto& maneuverState = *StorageAs<ManeuverState>(_primitiveStorageWords);
        const MazeMap::ManeuverInstance& maneuver = maneuverState.maneuver;
        const MazeMap::ManeuverCode code = maneuver.getCode();
        // Measure primitive progress from the wheel-average distance latched when the primitive was armed.
        const float traveledM =
            std::fabs((0.5f * (driveTelemetry.leftDistanceM + driveTelemetry.rightDistanceM)) - maneuverState.startDistanceM);
        const float totalDistanceM = ResolveRequestedMagnitude(maneuver.GetTravelDistanceMeters(Config::kCellSizeM));
        const bool hasFiniteStopCondition = std::isfinite(totalDistanceM);
        const float desiredSpeedMps =
            ResolveSignedCommandForDriveBase(
                maneuverState.maneuverSpeedMps,
                _limits.maxSpeedMps,
                maneuverState.maneuverSpeedMps);

        if (maneuver.SupportsPointTracking())
        {
            MazeMap::ManeuverPoint point = maneuverState.lastPoint;
            if (maneuver.TryGetManeuverPoint(
                    (std::min)(traveledM, totalDistanceM),
                    desiredSpeedMps,
                    point,
                    Config::kCellSizeM))
            {
                maneuverState.lastPoint = point;
                maneuverState.lastPointValid = true;
            }

            if (maneuverState.lastPointValid)
            {
                done = !hasFiniteStopCondition || (traveledM >= (totalDistanceM - Config::kDistanceToleranceM));
                return MakePointControlVector(
                    _drive,
                    ResolveSignedCommandForDriveBase(
                        maneuverState.lastPoint.Velocity,
                        _limits.maxSpeedMps,
                        desiredSpeedMps),
                    ResolveSignedCommandForDriveBase(
                        maneuverState.lastPoint.Omega,
                        _limits.maxAngularSpeedRadps),
                    _commandPdSettings.velocity | _commandPdSettings.yawRate);
            }
        }

        const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        done = !hasFiniteStopCondition || (traveledM >= (totalDistanceM - Config::kDistanceToleranceM));
        return MakePointControlVector(
            _drive,
            desiredSpeedMps,
            ResolveSignedCommandForDriveBase(
                desiredSpeedMps * (angleRad / totalDistanceM),
                _limits.maxAngularSpeedRadps),
            _commandPdSettings.velocity | _commandPdSettings.yawRate);
    }
}
