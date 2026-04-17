#pragma once

#include "ManeuverPath.h"
#include "MazeMapRuntimeCore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

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

    inline void SetKnownMazeCellWalls(
        MazeMap::Maze& maze,
        const MazeMap::CellCoordinates& cellCoordinates,
        const MazeMap::WallState up,
        const MazeMap::WallState down,
        const MazeMap::WallState left,
        const MazeMap::WallState right)
    {
        MazeMap::Cell& cell = maze[cellCoordinates];
        maze.SetWall(cell, MazeMap::Up, up);
        maze.SetWall(cell, MazeMap::Down, down);
        maze.SetWall(cell, MazeMap::Left, left);
        maze.SetWall(cell, MazeMap::Right, right);
    }

    inline MazeMap::Maze BuildPositionAuditMazeFixture(
        const std::uint8_t northCorridorCellCount,
        const std::uint8_t eastExtensionCellCount)
    {
        MazeMap::Maze maze;

        for (std::uint8_t y = 0U; y < northCorridorCellCount; ++y)
        {
            const MazeMap::CellCoordinates cell(0U, y);
            SetKnownMazeCellWalls(
                maze,
                cell,
                (y + 1U < northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall,
                (y > 0U) ? MazeMap::NoWall : MazeMap::Wall,
                MazeMap::Wall,
                (y + 1U == northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        for (std::uint8_t x = 1U; x <= eastExtensionCellCount; ++x)
        {
            const MazeMap::CellCoordinates cell(
                x,
                static_cast<std::uint8_t>(northCorridorCellCount - 1U));
            SetKnownMazeCellWalls(
                maze,
                cell,
                MazeMap::Wall,
                MazeMap::Wall,
                MazeMap::NoWall,
                (x < eastExtensionCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        return maze;
    }

    inline PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry()
    {
        PositionAuditFixtureGeometry geometry{};
        geometry.northCorridorCellCount = AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount;
        geometry.eastExtensionCellCount = AuxMeasurementConfig::kPositionAuditEastBranchCellCount;
        geometry.eastTotalCellCount = static_cast<std::uint8_t>(geometry.eastExtensionCellCount + 1U);
        geometry.northCorridorSpanYM = Config::kCellSizeM * static_cast<float>(geometry.northCorridorCellCount);
        geometry.eastBranchSpanXM = Config::kCellSizeM * static_cast<float>(geometry.eastTotalCellCount);
        geometry.outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(geometry.northCorridorCellCount - 1U);
        geometry.farCellCenterYM =
            (static_cast<float>(geometry.northCorridorCellCount) - 0.5f) *
            Config::kCellSizeM;
        geometry.farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            geometry.northCorridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.eastWallTouchXM = MazeMap::ComputeWallTouchPoseFromEastWallM(
            geometry.eastBranchSpanXM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.maze = BuildPositionAuditMazeFixture(
            geometry.northCorridorCellCount,
            geometry.eastExtensionCellCount);
        return geometry;
    }

    inline bool TryBuildReverseManeuverPath(
        const MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath)
    {
        reversePath.clear();
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (int index = static_cast<int>(forwardPath.GetSize()) - 1; index >= 0; --index)
        {
            if (!reversePath.push_back(maneuverSet.GetReverseCode(forwardPath[static_cast<std::uint16_t>(index)])))
            {
                reversePath.clear();
                return false;
            }
        }

        return true;
    }

    inline bool TryResolvePositionAuditSmoothTurnHalfSteps(
        const MazeMap::ManeuverCode code,
        std::uint8_t& preTurnHalfSteps,
        std::uint8_t& postTurnHalfSteps)
    {
        switch (code)
        {
        case MazeMap::S90SS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps;
            return true;
        case MazeMap::S90LS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps;
            return true;
        default:
            preTurnHalfSteps = 0U;
            postTurnHalfSteps = 0U;
            return false;
        }
    }

    inline bool TryBuildPositionAuditSmoothTurnPaths(
        const MazeMap::ManeuverCode code,
        MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath,
        std::uint8_t& preTurnHalfSteps,
        std::uint8_t& postTurnHalfSteps)
    {
        forwardPath.clear();
        reversePath.clear();
        if (!TryResolvePositionAuditSmoothTurnHalfSteps(code, preTurnHalfSteps, postTurnHalfSteps))
        {
            return false;
        }

        if (!forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(preTurnHalfSteps)) ||
            !forwardPath.push_back(code) ||
            !forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(postTurnHalfSteps)))
        {
            forwardPath.clear();
            return false;
        }

        return TryBuildReverseManeuverPath(forwardPath, reversePath);
    }

    inline bool TryValidatePositionAuditPath(
        const MazeMap::Maze& maze,
        const MazeMap::ManeuverPath& path,
        MazeMap::DirectionalLocation start,
        MazeMap::DirectionalLocation& end)
    {
        MazeMap::DirectionalLocation current = start;
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (std::uint16_t index = 0U; index < path.GetSize(); ++index)
        {
            const MazeMap::ManeuverCode code = path[index];
            if (!maneuverSet.IsValidMove(code, current, maze))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }

            current = maneuverSet.Move(code, current);
            if (!maze.IsAccessibleLocation(current.GetLocation()))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }
        }

        end = current;
        return true;
    }

    inline MotionLimits CorridorRepeatabilityLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    inline MotionLimits PositionAccuracyAuditStraightLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kPositionAuditAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kPositionAuditDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    inline MotionLimits PositionAccuracyAuditTurnLimits() noexcept
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(0.0f);
        limits.maxSpeedMps = 0.0f;
        return limits;
    }

    inline MotionLimits PositionAccuracyAuditCornerLimits(
        const float cruiseSpeedMps,
        const float nominalRadiusM) noexcept
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        (void)nominalRadiusM;
        limits.maxAngularSpeedRadps = AuxMeasurementConfig::kPositionAuditCornerMaxOmegaRadps;
        return limits;
    }

    inline float UtilityModeManeuverDistanceMeters(const MazeMap::ManeuverCode code)
    {
        return MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, Config::kCellSizeM);
    }

    template <typename WriteEventFn, typename FailFn>
    bool LogCorridorRepeatabilityMetadataImpl(WriteEventFn&& writeEvent, FailFn&& fail)
    {
        char line[160] = {};
        if (!writeEvent(
                "summary",
                "Place the robot in a 5-cell enclosed row like a mission start. This routine runs startup wall calibration, drives to the far end and back at several speeds, and logs closure error at the start cell."))
        {
            return fail("Unable to write corridor repeatability summary");
        }

        std::snprintf(
            line,
            sizeof(line),
            "row_cell_count,%u",
            static_cast<unsigned>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount));
        if (!writeEvent("corridor_repeatability", line))
        {
            return fail("Unable to write corridor repeatability metadata");
        }

        const float outDistanceM =
            (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount > 0U) ?
            (Config::kCellSizeM * static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)) :
            0.0f;
        std::snprintf(line, sizeof(line), "out_distance_m,%.6f", outDistanceM);
        if (!writeEvent("corridor_repeatability", line))
        {
            return fail("Unable to write corridor repeatability metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;turn_max_omega_radps,%.6f;turn_accel_radps2,%.6f",
            AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2,
            Config::kSearchTurnMaxOmegaRadps,
            Config::kSearchTurnAccelRadps2);
        if (!writeEvent("corridor_repeatability", line))
        {
            return fail("Unable to write corridor repeatability metadata");
        }

        for (std::uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            std::snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex]);
            if (!writeEvent("corridor_repeatability_speed", line))
            {
                return fail("Unable to write corridor repeatability speed metadata");
            }
        }

        return true;
    }

    template <typename WriteEventFn, typename FailFn>
    bool LogPositionAccuracyAuditMetadataImpl(
        const PositionAuditFixtureGeometry& geometry,
        WriteEventFn&& writeEvent,
        FailFn&& fail)
    {
        char line[320] = {};
        std::snprintf(
            line,
            sizeof(line),
            "Build a one-cell-wide fixture: normal mission start, a %u-cell north corridor including the start and corner cells, and a %u-cell east extension beyond that corner with solid side walls. All following phases reuse this same fixed geometry.",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount));
        if (!writeEvent("summary", line))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "position_straight_result isolates wheel-diameter, straight feedforward, and stop-distance error through north_touch_correction_m, enc_out_err_m, closure_m, and yaw_err_deg."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "position_in_place_turn_result isolates the shared in-place turn profile through yaw_err_deg, effective_track_width_m, and wall_touch_correction_m."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "position_smooth_turn_result compares S90SS and S90LS against nominal_radius_m, measured_radius_m, effective_track_width_m, corridor_err_m, and east_touch_correction_m to expose radius-dependent feedforward error."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "Phase 1 runs S8, centers in the north corner, turns in place to face down, and runs S8 back to start."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "Phase 2 reseats at start, runs S7 + S90SS + S7, centers at the east end, turns to face left, and returns on the reversed maneuver path."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (!writeEvent(
                "summary",
                "Phase 3 reseats at start, runs S6 + S90LS + S6, recenters at the east end, and returns on the reversed maneuver path."))
        {
            return fail("Unable to write position accuracy audit summary");
        }
        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled &&
            !writeEvent(
                "summary",
                "Smooth-turn phases run with the mission fan enabled; the existing 2 s ramp to 80% completes before motion begins so high-speed S90 data reflects the intended downforce state."))
        {
            return fail("Unable to write position accuracy audit summary");
        }

        std::snprintf(
            line,
            sizeof(line),
            "north_corridor_cells,%u;east_extension_cells,%u;east_total_cells,%u",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount),
            static_cast<unsigned>(geometry.eastTotalCellCount));
        if (!writeEvent("position_audit", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;start_settle_ms,%u",
            AuxMeasurementConfig::kPositionAuditAccelMps2,
            AuxMeasurementConfig::kPositionAuditDecelMps2,
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditStartSettleMs));
        if (!writeEvent("position_audit", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "smooth_turn_fan_enabled,%u;kRacingFanDutyCycle,%.6f;kRacingFanRampMs,%u",
            AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled ? 1U : 0U,
            Config::kRacingFanDutyCycle,
            static_cast<unsigned>(Config::kRacingFanRampMs));
        if (!writeEvent("position_audit", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=1;forward_half_steps=%u;turn=IP180;return_half_steps=%u",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=2;forward=%u,S90SS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=3;forward=%u,S90LS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        for (std::uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            std::snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]);
            if (!writeEvent("position_audit_straight_speed", line))
            {
                return fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (std::uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditCornerSpeedCount; ++speedIndex)
        {
            std::snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]);
            if (!writeEvent("position_audit_corner_speed", line))
            {
                return fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (std::uint8_t codeIndex = 0U; codeIndex < AuxMeasurementConfig::kPositionAuditSmoothTurnCodeCount; ++codeIndex)
        {
            const MazeMap::ManeuverCode code = AuxMeasurementConfig::kPositionAuditSmoothTurnCodes[codeIndex];
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));
            std::snprintf(
                line,
                sizeof(line),
                "code=%s;nominal_radius_m=%.6f;distance_m=%.6f",
                codeName,
                MazeMap::ManeuverSet::GetSet()[code].GetNominalTurnRadiusInCells() * Config::kCellSizeM,
                UtilityModeManeuverDistanceMeters(code));
            if (!writeEvent("position_audit_turn_code", line))
            {
                return fail("Unable to write position accuracy audit turn metadata");
            }
        }

        return true;
    }

    template <typename WriteEventFn, typename FailFn>
    bool WriteCorridorRepeatabilityResultImpl(
        WriteEventFn&& writeEvent,
        FailFn&& fail,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry,
        const PoseEstimate& finalPose,
        const DriveTelemetry& finalTelemetry)
    {
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "speed_index=%u;cruise_mps=%.3f;dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f;"
            "left_delta_m=%.6f;right_delta_m=%.6f;left_delta_cnt=%ld;right_delta_cnt=%ld",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg,
            finalTelemetry.leftDistanceM - startTelemetry.leftDistanceM,
            finalTelemetry.rightDistanceM - startTelemetry.rightDistanceM,
            static_cast<long>(finalTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount),
            static_cast<long>(finalTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount));

        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return fail("Corridor repeatability result event overflowed");
        }
        if (writeEvent("corridor_repeatability_result", message))
        {
            return true;
        }
        return fail("Unable to write corridor repeatability result");
    }

    template <typename WriteEventFn, typename FailFn>
    bool WritePositionStraightAuditResultImpl(
        WriteEventFn&& writeEvent,
        FailFn&& fail,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float northStopErrorM,
        const float northTouchCorrectionM,
        const float encoderOutErrorM,
        const PoseEstimate& startPose,
        const PoseEstimate& finalPose)
    {
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "speed_idx=%u;v=%.3f;stop_err_m=%.6f;touch_correction_m=%.6f;enc_out_err_m=%.6f;"
            "dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            northStopErrorM,
            northTouchCorrectionM,
            encoderOutErrorM,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return fail("Position straight result event overflowed");
        }
        if (writeEvent("position_straight_result", message))
        {
            return true;
        }
        return fail("Unable to write position straight result");
    }

    template <typename WriteEventFn, typename FailFn>
    bool WritePositionInPlaceTurnAuditResultImpl(
        WriteEventFn&& writeEvent,
        FailFn&& fail,
        const MazeMap::Direction targetDirection,
        const float touchCorrectionM,
        const float leftDeltaM,
        const float rightDeltaM,
        const float yawChangeRad,
        const float currentYawRad)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftDeltaM,
            rightDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(DirectionToYawRad(targetDirection), currentYawRad);

        char message[224] = {};
        const int length =
            haveTrackWidth ?
            std::snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=%.6f",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG_F * yawChangeRad,
                effectiveTrackWidthM) :
            std::snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=nan",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG_F * yawChangeRad);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return fail("Position in-place turn result event overflowed");
        }
        if (writeEvent("position_in_place_turn_result", message))
        {
            return true;
        }
        return fail("Unable to write position in-place turn result");
    }

    template <typename WriteEventFn, typename FailFn>
    bool WritePositionSmoothTurnAuditResultImpl(
        WriteEventFn&& writeEvent,
        FailFn&& fail,
        const MazeMap::ManeuverCode code,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float nominalRadiusM,
        const float corridorErrorM,
        const float eastTouchCorrectionM,
        const float leftArcDeltaM,
        const float rightArcDeltaM,
        const float yawChangeRad,
        const float yawErrorDeg)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        float measuredRadiusM = 0.0f;
        const bool haveMeasuredRadius = TryComputeEffectiveTurnRadiusM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            measuredRadiusM);

        char codeName[24] = {};
        FormatManeuverCodeName(code, codeName, sizeof(codeName));
        char measuredRadiusText[24] = {};
        char effectiveTrackWidthText[24] = {};
        if (haveMeasuredRadius)
        {
            std::snprintf(measuredRadiusText, sizeof(measuredRadiusText), "%.6f", measuredRadiusM);
        }
        else
        {
            std::snprintf(measuredRadiusText, sizeof(measuredRadiusText), "nan");
        }
        if (haveTrackWidth)
        {
            std::snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "%.6f", effectiveTrackWidthM);
        }
        else
        {
            std::snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "nan");
        }

        char message[256] = {};
        const int length = std::snprintf(
            message,
            sizeof(message),
            "code=%s;speed_idx=%u;v=%.3f;nominal_radius_m=%.6f;measured_radius_m=%s;"
            "effective_track_width_m=%s;yaw_err_deg=%.3f;corridor_err_m=%.6f;east_touch_correction_m=%.6f",
            codeName,
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            nominalRadiusM,
            measuredRadiusText,
            effectiveTrackWidthText,
            yawErrorDeg,
            corridorErrorM,
            eastTouchCorrectionM);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return fail("Position smooth turn result event overflowed");
        }
        if (writeEvent("position_smooth_turn_result", message))
        {
            return true;
        }
        return fail("Unable to write position smooth turn result");
    }
}
