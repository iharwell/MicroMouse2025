#pragma once

#include "ManeuverPath.h"
#include "MazeMapRuntimeCore.h"

#include <cstdint>

namespace MazeMap::App::Internal::AuxMeasurementModeSupport
{
    struct PositionAuditFixtureGeometry final
    {
        MazeMap::Maze maze;
        std::uint8_t northCorridorCellCount = 0U;
        std::uint8_t eastExtensionCellCount = 0U;
        std::uint8_t eastTotalCellCount = 0U;
        float northCorridorSpanYM = 0.0f;
        float eastBranchSpanXM = 0.0f;
        float outDistanceM = 0.0f;
        float farCellCenterYM = 0.0f;
        float farWallTouchYM = 0.0f;
        float eastWallTouchXM = 0.0f;
    };

    using WriteEventCallback = bool (*)(void* context, const char* type, const char* message);
    using FailCallback = bool (*)(void* context, const char* message);

    EXPORT PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry();

    EXPORT bool TryBuildPositionAuditSmoothTurnPaths(
        MazeMap::ManeuverCode code,
        MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath,
        std::uint8_t& preTurnHalfSteps,
        std::uint8_t& postTurnHalfSteps);

    EXPORT bool TryValidatePositionAuditPath(
        const MazeMap::Maze& maze,
        const MazeMap::ManeuverPath& path,
        MazeMap::DirectionalLocation start,
        MazeMap::DirectionalLocation& end);

    EXPORT MotionLimits CorridorRepeatabilityLimits(float cruiseSpeedMps) noexcept;
    EXPORT MotionLimits PositionAccuracyAuditStraightLimits(float cruiseSpeedMps) noexcept;
    EXPORT MotionLimits PositionAccuracyAuditTurnLimits() noexcept;
    EXPORT MotionLimits PositionAccuracyAuditCornerLimits(
        float cruiseSpeedMps,
        float nominalRadiusM) noexcept;
    EXPORT float UtilityModeManeuverDistanceMeters(MazeMap::ManeuverCode code);

    EXPORT bool LogCorridorRepeatabilityMetadata(
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context);

    EXPORT bool LogPositionAccuracyAuditMetadata(
        const PositionAuditFixtureGeometry& geometry,
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context);

    EXPORT bool WriteCorridorRepeatabilityResult(
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context,
        std::uint8_t speedIndex,
        float cruiseSpeedMps,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry,
        const PoseEstimate& finalPose,
        const DriveTelemetry& finalTelemetry);

    EXPORT bool WritePositionStraightAuditResult(
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context,
        std::uint8_t speedIndex,
        float cruiseSpeedMps,
        float northStopErrorM,
        float northTouchCorrectionM,
        float encoderOutErrorM,
        const PoseEstimate& startPose,
        const PoseEstimate& finalPose);

    EXPORT bool WritePositionInPlaceTurnAuditResult(
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context,
        MazeMap::Direction targetDirection,
        float touchCorrectionM,
        float leftDeltaM,
        float rightDeltaM,
        float yawChangeRad,
        float currentYawRad);

    EXPORT bool WritePositionSmoothTurnAuditResult(
        WriteEventCallback writeEvent,
        FailCallback fail,
        void* context,
        MazeMap::ManeuverCode code,
        std::uint8_t speedIndex,
        float cruiseSpeedMps,
        float nominalRadiusM,
        float corridorErrorM,
        float eastTouchCorrectionM,
        float leftArcDeltaM,
        float rightArcDeltaM,
        float yawChangeRad,
        float yawErrorDeg);
}
