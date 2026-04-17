#include "pch.h"
#include "MissionModeController.h"

#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "ManeuverInstance.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapSharedRuntime.h"

using MazeMap::App::Internal::SharedRobotRuntime;

namespace
{
    constexpr MazeMap::CommandPD kMissionDriveBaseTrackingCommandPd =
        MazeMap::CommandPD::StateWheelOmegaPD |
        MazeMap::CommandPD::IMUYaw;

    void SetKnownMazeCellWalls(
        MazeMap::Maze& maze,
        const MazeMap::CellCoordinates& cellCoordinates,
        MazeMap::WallState up,
        MazeMap::WallState down,
        MazeMap::WallState left,
        MazeMap::WallState right)
    {
        MazeMap::Cell& cell = maze[cellCoordinates];
        maze.SetWall(cell, MazeMap::Up, up);
        maze.SetWall(cell, MazeMap::Down, down);
        maze.SetWall(cell, MazeMap::Left, left);
        maze.SetWall(cell, MazeMap::Right, right);
    }

    struct PositionAuditFixtureGeometry
    {
        MazeMap::Maze maze;
        uint8_t northCorridorCellCount = 0U;
        uint8_t eastExtensionCellCount = 0U;
        uint8_t eastTotalCellCount = 0U;
        float northCorridorSpanYM = 0.0f;
        float eastBranchSpanXM = 0.0f;
        float outDistanceM = 0.0f;
        float farCellCenterYM = 0.0f;
        float farWallTouchYM = 0.0f;
        float eastWallTouchXM = 0.0f;
    };

    MazeMap::Maze BuildPositionAuditMazeFixture(
        const uint8_t northCorridorCellCount,
        const uint8_t eastExtensionCellCount)
    {
        MazeMap::Maze maze;

        for (uint8_t y = 0U; y < northCorridorCellCount; ++y)
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

        for (uint8_t x = 1U; x <= eastExtensionCellCount; ++x)
        {
            const MazeMap::CellCoordinates cell(
                x,
                static_cast<uint8_t>(northCorridorCellCount - 1U));
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

    PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry()
    {
        PositionAuditFixtureGeometry geometry{};
        geometry.northCorridorCellCount = AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount;
        geometry.eastExtensionCellCount = AuxMeasurementConfig::kPositionAuditEastBranchCellCount;
        geometry.eastTotalCellCount = static_cast<uint8_t>(geometry.eastExtensionCellCount + 1U);
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

    bool TryBuildReverseManeuverPath(
        const MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath)
    {
        reversePath.clear();
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (int index = static_cast<int>(forwardPath.GetSize()) - 1; index >= 0; --index)
        {
            if (!reversePath.push_back(maneuverSet.GetReverseCode(forwardPath[static_cast<uint16_t>(index)])))
            {
                reversePath.clear();
                return false;
            }
        }

        return true;
    }

    bool TryResolvePositionAuditSmoothTurnHalfSteps(
        const MazeMap::ManeuverCode code,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
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

    bool TryBuildPositionAuditSmoothTurnPaths(
        const MazeMap::ManeuverCode code,
        MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
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

    bool TryValidatePositionAuditPath(
        const MazeMap::Maze& maze,
        const MazeMap::ManeuverPath& path,
        MazeMap::DirectionalLocation start,
        MazeMap::DirectionalLocation& end)
    {
        MazeMap::DirectionalLocation current = start;
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (uint16_t index = 0U; index < path.GetSize(); ++index)
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

    MotionLimits CorridorRepeatabilityLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditStraightLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kPositionAuditAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kPositionAuditDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditTurnLimits() noexcept
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(0.0f);
        limits.maxSpeedMps = 0.0f;
        return limits;
    }

    MotionLimits PositionAccuracyAuditCornerLimits(
        const float cruiseSpeedMps,
        const float nominalRadiusM) noexcept
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        (void)nominalRadiusM;
        limits.maxAngularSpeedRadps = AuxMeasurementConfig::kPositionAuditCornerMaxOmegaRadps;
        return limits;
    }

    float UtilityModeManeuverDistanceMeters(const MazeMap::ManeuverCode code)
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

        snprintf(
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
        snprintf(line, sizeof(line), "out_distance_m,%.6f", outDistanceM);
        if (!writeEvent("corridor_repeatability", line))
        {
            return fail("Unable to write corridor repeatability metadata");
        }

        snprintf(
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

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            snprintf(
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
        snprintf(
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

        snprintf(
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

        snprintf(
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

        snprintf(
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

        snprintf(
            line,
            sizeof(line),
            "phase=1;forward_half_steps=%u;turn=IP180;return_half_steps=%u",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=2;forward=%u,S90SS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=3;forward=%u,S90LS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps));
        if (!writeEvent("position_audit_phase", line))
        {
            return fail("Unable to write position accuracy audit metadata");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            snprintf(
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

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditCornerSpeedCount; ++speedIndex)
        {
            snprintf(
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

        for (uint8_t codeIndex = 0U; codeIndex < AuxMeasurementConfig::kPositionAuditSmoothTurnCodeCount; ++codeIndex)
        {
            const MazeMap::ManeuverCode code = AuxMeasurementConfig::kPositionAuditSmoothTurnCodes[codeIndex];
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));
            snprintf(
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
        const uint8_t speedIndex,
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
        const int length = snprintf(
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
        const uint8_t speedIndex,
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
        const int length = snprintf(
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
            snprintf(
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
            snprintf(
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
        const uint8_t speedIndex,
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
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "%.6f", measuredRadiusM);
        }
        else
        {
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "nan");
        }
        if (haveTrackWidth)
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "%.6f", effectiveTrackWidthM);
        }
        else
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "nan");
        }

        char message[256] = {};
        const int length = snprintf(
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

class MazeMap::App::Internal::MissionModeController::Implementation final
{
public:
    explicit Implementation(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _speedVehicle(runtime.SpeedVehicle())
        , _mappingVehicle(runtime.SearchVehicle())
        , _maze(runtime.Maze())
        , _searchPathFinder(runtime.SearchPathFinder())
        , _speedPathFinder(runtime.SpeedPathFinder())
        , _wallBeliefMap(runtime.WallBeliefMap())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _currentCell(0, 0)
        , _currentDirection(MazeMap::Up)
        , _currentDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _goalPauseComplete(false)
        , _missionComplete(false)
        , _faulted(false)
        , _maneuverTestMode(false)
        , _telemetryLoggingEnabled(false)
        , _missionTextLoggingEnabled(false)
        , _missionMazeSnapshotWritten(false)
        , _frontWallCharacterization()
        , _frontWallCharacterizationAvailable(false)
        , _lastWallTouchStandoffEstimateM(0.0f)
        , _hasWallTouchStandoffEstimate(false)
        , _activeModeFaultSource("mission")
        , _telemetryPhaseId(0UL)
        , _telemetrySampleCount(0UL)
    {
        _telemetryLogFileName[0] = '\0';
    }

    Implementation(const Implementation&) = delete;
    Implementation& operator=(const Implementation&) = delete;
    Implementation(Implementation&&) = delete;
    Implementation& operator=(Implementation&&) = delete;

    bool BeginMissionRunMode()
    {
        ResetForMode(false, true, "mission");
        if (!Initialize("Micromouse mission setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        return true;
    }

    void RunMissionRunMode()
    {
        if (_missionComplete || _faulted)
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Exploration start"))
        {
            return;
        }
        _drive.SetWheelControlProfile(BuildMappingWheelControlProfile());
        if (!ExploreFullMaze())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return;
        }
        if (!ReturnToStart())
        {
            return;
        }
        _drive.UseNominalWheelControlProfile();

        _maze.PreCalculate();

        if (!EmitMissionControllerLineOrFail("Speed run 1 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        if (!HandleInterRunServiceCycle())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Speed run 2 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        (void)WriteMissionMazeSnapshot("mission_complete");

        SetRacingFanEnabled(false);
        _drive.Brake();
        _missionComplete = true;
        (void)EmitMissionControllerLine("Mission complete");
        CloseMissionTextLog();
    }

    bool BeginManeuverFileTestMode()
    {
        ResetForMode(true, false, "maneuver_file_test");
        if (!Initialize("Micromouse maneuver test setup", false))
        {
            return false;
        }

        SeedWallBeliefsFromKnownMaze();
        if (!_sensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }
        if (!BeginTelemetryLog("maneuver_test.mmlog", "maneuver_test"))
        {
            AppendStartupTrace("maneuver_test:telemetry_logger_open_failed");
            (void)EmitMissionControllerLine("Maneuver test telemetry log unavailable; continuing without telemetry file");
            _telemetryLoggingEnabled = false;
            return true;
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("maneuver_test:telemetry_logger_opened");
        if (!WriteTelemetryEvent("source", "test.txt"))
        {
            AppendStartupTrace("maneuver_test:source_metadata_write_failed");
            (void)EmitMissionControllerLine("Maneuver test source metadata write failed; disabling telemetry file logging");
            ShutdownTelemetryMode(false);
            return true;
        }
        AppendStartupTrace("maneuver_test:source_metadata_written");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        AppendStartupTrace("maneuver_test:wall_calibration_logged");
        return true;
    }

    void RunManeuverFileTestMode()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("maneuver_test:run_entered");
        MazeMap::ManeuverQueue queue;
        if (!LoadManeuverQueueFromSd("test.txt", queue))
        {
            return;
        }
        AppendStartupTrace("maneuver_test:queue_loaded");

        (void)EmitMissionControllerFormatted(
            "Loaded maneuver test queue with %u maneuvers",
            static_cast<unsigned>(queue.size()));

        if (!HoldPosition(Config::kObservationSettleMs, "startup_settle"))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        if (!ExecuteQueuedManeuvers(queue, false))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        if (!HoldPosition(50, "final_hold"))
        {
            ShutdownTelemetryMode(false);
            return;
        }

        ShutdownTelemetryMode(false);
        AppendStartupTrace("maneuver_test:complete");
        (void)EmitMissionControllerLine("Maneuver file test complete");
        CloseMissionTextLog();
    }

    bool BeginCorridorRepeatabilityMode()
    {
        ResetForMode(false, false, "corridor_repeatability");
        if (!Initialize("Corridor repeatability setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_sensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.mmlog",
                "corridor_repeatability.mmlog"))
        {
            return Fail("Unable to choose corridor repeatability log file");
        }
        if (!BeginTelemetryLog(fileName, "corridor_repeatability"))
        {
            return Fail("Unable to open corridor repeatability log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("corridor_repeatability:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        if (!LogCorridorRepeatabilityMetadataImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                }))
        {
            return false;
        }
        AppendStartupTrace("corridor_repeatability:metadata_written");
        return true;
    }

    void RunCorridorRepeatabilityMode()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("corridor_repeatability:run_entered");
        const bool ok = RunCorridorRepeatabilityPasses();
        ShutdownTelemetryMode(false);
        if (ok)
        {
            AppendStartupTrace("corridor_repeatability:complete");
            (void)EmitMissionControllerLine("Corridor repeatability sweep complete");
        }
        CloseMissionTextLog();
    }

    bool BeginPositionAccuracyAuditMode()
    {
        ResetForMode(false, false, "position_accuracy_audit");
        if (!Initialize("Position accuracy audit setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_sensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName(
                fileName,
                sizeof(fileName),
                nullptr,
                "aux%03u.mmlog",
                "position_accuracy_audit.mmlog"))
        {
            return Fail("Unable to choose position accuracy audit log file");
        }
        if (!BeginTelemetryLog(fileName, "position_accuracy_audit"))
        {
            return Fail("Unable to open position accuracy audit log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("position_accuracy_audit:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        const PositionAuditFixtureGeometry positionAuditGeometry = BuildPositionAuditFixtureGeometry();
        if (!LogPositionAccuracyAuditMetadataImpl(
                positionAuditGeometry,
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                }))
        {
            return false;
        }
        AppendStartupTrace("position_accuracy_audit:metadata_written");
        return true;
    }

    void RunPositionAccuracyAuditMode()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("position_accuracy_audit:run_entered");
        const bool ok = RunPositionAccuracyAuditPasses();
        ShutdownTelemetryMode(true);
        if (ok)
        {
            AppendStartupTrace("position_accuracy_audit:complete");
            (void)EmitMissionControllerLine("Position accuracy audit complete");
        }
        CloseMissionTextLog();
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    static constexpr const char* kMissionControllerTextLogSource = "mission_controller";
    static constexpr const char* kMissionTraceTextLogSource = "mission_trace";

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    MazeMap::Vehicle& _speedVehicle;
    MazeMap::Vehicle& _mappingVehicle;
    MazeMap::Maze& _maze;
    MazeMap::FloodFillPathFinder& _searchPathFinder;
    MazeMap::ManeuverPathFinder& _speedPathFinder;
    MazeMap::WallBeliefMap& _wallBeliefMap;
    RuntimeSensorSuite& _sensors;
    DriveBase& _drive;
    MazeMap::CellCoordinates _currentCell;
    MazeMap::Direction _currentDirection;
    MazeMap::DirectionalLocation _currentDirectionalLocation;
    bool _goalPauseComplete;
    bool _missionComplete;
    bool _faulted;
    bool _maneuverTestMode;
    bool _telemetryLoggingEnabled;
    bool _missionTextLoggingEnabled;
    bool _missionMazeSnapshotWritten;
    MazeMap::FrontWallCharacterizationStorage _frontWallCharacterization;
    bool _frontWallCharacterizationAvailable;
    float _lastWallTouchStandoffEstimateM;
    bool _hasWallTouchStandoffEstimate;
    const char* _activeModeFaultSource;
    char _telemetryLogFileName[64];
    unsigned long _telemetryPhaseId;
    unsigned long _telemetrySampleCount;
    using ActiveLoopTickFn = LoopController::ControlVector (Implementation::*)(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    struct WallTouchPoseResetTarget;

    struct HoldLoopState final
    {
        std::uint16_t durationMs{};
        unsigned long startMs{};
        bool stationary{ true };
        bool started{};
        void* nextState{};
        ActiveLoopTickFn nextTickFn{};
    };

    struct InterRunServicePauseLoopState final
    {
    };

    struct SettleLoopState final
    {
        const char* timeoutMessage{};
        std::uint16_t stationaryHoldMs{ Config::kMotionSettleHoldMs };
        std::uint16_t timeoutMs{ Config::kMotionSettleTimeoutMs };
        unsigned long startMs{};
        unsigned long stationaryStartMs{};
        bool stationaryWindowActive{};
        bool started{};
        bool brakeCommand{ true };
        DriveTelemetry stationaryStartTelemetry{};
        void* nextState{};
        ActiveLoopTickFn nextTickFn{};
    };

    struct StartupStationaryHoldLoopState final
    {
        unsigned long stationaryStartMs{};
        unsigned long lastResetTraceMs{};
        bool stationaryWindowActive{};
        DriveTelemetry stationaryStartTelemetry{};
    };

    struct ObservationCaptureLoopState final
    {
        MazeMap::CellCoordinates observedCell{};
        MazeMap::Direction observedDirection{ MazeMap::None };
        SensorSnapshot* outputSnapshot{};
        SensorSnapshot samples[Config::kSearchRollingObservationSampleCount]{};
        float frontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float frontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        std::uint8_t nextSampleIndex{};
    };

    struct FrontCalibrationSweepLoopState final
    {
        FrontCalibrationSpinSampleSet<Config::kStartupWallCalibrationFrontSpinMaxSamples> sweepSamples{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        MotionLimits limits{};
        float targetSweepAngleRad{};
        float captureStepRad{};
        float accumulatedSweepAngleRad{};
        float lastStoredSweepAngleRad{};
        float previousYawRad{};
        unsigned long expectedCompletionDeadlineMs{};
        bool durationLogged{};
        bool* storedBands{};
        HoldLoopState settleHold{};
    };

    struct ReverseStraightLoopState final
    {
        MotionLimits limits{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        const Eigen::Vector2f* targetPositionOverride{};
        float distanceM{};
        float startDistanceM{};
        float commandedSpeedMps{};
        bool projectionFallbackLogged{};
        unsigned long timeoutMs{};
        EncoderProgressWatchdog translationWatchdog{};
        SettleLoopState completionSettle{};
        HoldLoopState fallbackHold{};
    };

    struct StraightLoopState final
    {
        MotionLimits limits{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        const Eigen::Vector2f* targetPositionOverride{};
        float distanceM{};
        float entrySpeed{};
        float exitSpeed{};
        float cruiseSpeed{};
        float startDistanceM{};
        float commandedSpeedMps{};
        bool useWallCentering{};
        bool diagonalHeading{};
        bool stallLogged{};
        bool durationLogged{};
        float previousCorridorErrorM{};
        float filteredCorridorErrorRateMps{};
        bool previousCorridorErrorValid{};
        unsigned long expectedCompletionDeadlineMs{};
        EncoderProgressWatchdog translationWatchdog{};
        SettleLoopState completionSettle{};
    };

    struct SearchStraightLoopState final
    {
        MazeMap::Direction direction{ MazeMap::None };
        MazeMap::CellCoordinates startCell{};
        MazeMap::CellCoordinates destination{};
        std::uint16_t cellCount{};
        float entrySpeedMps{};
        float cruiseSpeedMps{};
        float exitSpeedMps{};
        bool observeWhileRolling{};
        bool stoppedForReplan{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        float targetXMeters{};
        float targetYMeters{};
        float distanceToTargetM{};
        float commandedSpeedMps{};
        std::uint16_t rollingObservationCount{};
        MazeMap::CellCoordinates nextRollingObservationCell{};
        float rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount]{};
        SensorSnapshot rollingObservationSamples[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationFrontLeftCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationFrontRightCandidateDistanceM[Config::kSearchRollingObservationSampleCount]{};
        float rollingObservationSideResetTriggerTravelM{};
        std::uint8_t rollingObservationNextSampleIndex{};
        bool rollingObservationSideResetPending{};
        bool rollingObservationPlanInitialized{};
        bool stallLogged{};
        bool durationLogged{};
        float previousCorridorErrorM{};
        float filteredCorridorErrorRateMps{};
        bool previousCorridorErrorValid{};
        MazeMap::CellCoordinates replanObservedCell{};
        float replanProjectedTravelM{};
        std::uint16_t replanFrontVoteCount{};
        unsigned long expectedCompletionDeadlineMs{};
        EncoderProgressWatchdog translationWatchdog{};
        SettleLoopState completionSettle{};
    };

    struct TurnLoopState final
    {
        float targetYawRad{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        unsigned long expectedCompletionDeadlineMs{};
        bool durationLogged{};
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker{};
        SettleLoopState completionSettle{};
    };

    struct ArcLoopState final
    {
        MotionLimits limits{};
        float distanceM{};
        float angleRad{};
        float entrySpeed{};
        float exitSpeed{};
        float cruiseSpeed{};
        float startDistanceM{};
        float startYawRad{};
        float curvature{};
        float commandedSpeedMps{};
        unsigned long expectedCompletionDeadlineMs{};
        bool stallLogged{};
        bool durationLogged{};
        EncoderProgressWatchdog translationWatchdog{};
        SettleLoopState completionSettle{};
    };

    struct SmoothTurnLoopState final
    {
        MazeMap::ManeuverInstance maneuver{};
        MotionLimits limits{};
        float entrySpeed{};
        float exitSpeed{};
        float cruiseSpeed{};
        float maneuverSpeedMps{};
        float totalDistanceM{};
        float startDistanceM{};
        unsigned long expectedCompletionDeadlineMs{};
        bool stallLogged{};
        bool durationLogged{};
        EncoderProgressWatchdog translationWatchdog{};
    };

    struct WallTouchLoopState final
    {
        float targetYawRad{};
        float minLatchTravelM{};
        float maxApproachTravelM{};
        bool allowPassThroughNoWall{};
        const WallTouchPoseResetTarget* poseResetTarget{};
        float* seatedYawErrorRad{};
        MazeMap::App::Internal::Runtime::WallTouchExecutionResult result{};
        DriveTelemetry lastMotionTelemetry{};
        unsigned long touchStartMs{};
        unsigned long stateStartMs{};
        unsigned long contactCandidateStartMs{};
        unsigned long contactConfirmedStartMs{};
        unsigned long frontSignalMissingStartMs{};
        unsigned long lastMotionMs{};
        float startDistanceM{};
        float approachDriveCommand{};
        float ditherTurnFraction{};
        float previousCycleFrontSkewMagnitudeM{};
        float currentCycleStartYawRad{};
        float currentCycleMaxFrontSkewMagnitudeM{};
        float currentCycleMaxResidualYawRateRadps{};
        bool currentCycleFrontSignalValid{};
        bool haveSquareSample{};
        unsigned long lastHalfCycleIndex{};
        float lastSquareYawRad{};
        float lastSquareFrontSkewM{};
        float lastSquareYawRateRadps{};
        bool lastSquareFrontSignalValid{};
        std::uint8_t completedHalfCycles{};
        std::uint8_t consecutiveGoodFullCycles{};
        bool contactCandidateActive{};
        bool seatedResetApplied{};
        bool passThroughCompletionPending{};
        MazeMap::App::Internal::Runtime::WallTouchState runtimeState{
            MazeMap::App::Internal::Runtime::WallTouchState::EntryConditioning
        };
        SettleLoopState passThroughSettle{};
    };

    void* _activeLoopState{};
    ActiveLoopTickFn _activeLoopTickFn{};

    bool BeginTelemetryLog(const char* fileName, const char* modeName)
    {
        const char* resolvedFileName = (fileName != nullptr && fileName[0] != '\0') ? fileName : "telemetry.mmlog";
        const char* resolvedModeName = (modeName != nullptr && modeName[0] != '\0') ? modeName : "telemetry";
        _telemetryPhaseId = 0UL;
        _telemetrySampleCount = 0UL;
        _telemetryLogFileName[0] = '\0';
        (void)_runtime.CloseUtilityDataLog();
        snprintf(_telemetryLogFileName, sizeof(_telemetryLogFileName), "%s", resolvedFileName);

        if (!_runtime.OpenUtilityDataLogFile(_telemetryLogFileName))
        {
            return false;
        }

        if (!_runtime.WriteUtilityDataLogMetadata("mode", resolvedModeName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", Config::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !_runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                Config::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;

        DiagnosticLogRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row)) return false;

        if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return false;
        if (!_runtime.WriteTextLogMetadata("data_file", _telemetryLogFileName)) return false;
        if (!_runtime.WriteTextLogMetadata("mode", resolvedModeName)) return false;
        if (!MazeMap::App::Internal::Runtime::WriteDiagnosticTuningEvents(
                [this](const char* type, const char* message) -> bool
                {
                    return _runtime.WriteTextLogEntry(micros(), type, message);
                })) return false;
        return MazeMap::App::Internal::Runtime::WriteDiagnosticSummaryInstructions(
            [this](const char* type, const char* message) -> bool
            {
                return _runtime.WriteTextLogEntry(micros(), type, message);
            });
    }

    bool WriteTelemetryEvent(const char* type, const char* message)
    {
        return !_telemetryLoggingEnabled || _runtime.WriteTextLogEntry(micros(), type, message);
    }

    void ServiceTelemetryLog()
    {
        (void)_runtime.ServiceUtilityDataLog();
    }

    void FlushTelemetryLog()
    {
        _runtime.FlushTextLog();
    }

    void CloseTelemetryLog()
    {
        (void)_runtime.CloseUtilityDataLog();
    }

    static void SetRacingFanEnabled(bool enabled)
    {
        if (enabled)
        {
            RampFanDutyCycle(Config::kRacingFanDutyCycle);
            return;
        }

        WriteFanDutyCycle(0.0f);
    }

    MotionLimits SearchLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kSearchMaxSpeedMps;
        limits.accelMps2 = Config::kSearchAccelMps2;
        limits.decelMps2 = Config::kSearchDecelMps2;
        limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration();
        limits.angleToleranceRad = Config::kMappingAngleToleranceRad;
        limits.angularSpeedToleranceRadps = Config::kMappingAngularSpeedToleranceRadps;
        return limits;
    }

    MotionLimits FinalLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = _speedVehicle.GetMaxSpeed() * Config::kSpeedRunScale;
        limits.accelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.decelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity() * Config::kSpeedRunScale;
        limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration() * Config::kSpeedRunScale;
        return limits;
    }

    static MotionLimits StartupWallCalibrationLimits()
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kStartupWallCalibrationSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationDecelMps2;
        limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kStartupWallCalibrationTurnAccelRadps2;
        return limits;
    }

    static MotionLimits StartupWallCalibrationCenteringLimits()
    {
        MotionLimits limits = StartupWallCalibrationLimits();
        limits.maxSpeedMps = Config::kStartupWallCalibrationCenteringSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationCenteringAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationCenteringDecelMps2;
        return limits;
    }

    static MotionLimits StartupWallCalibrationFrontSweepLimits()
    {
        MotionLimits limits = StartupWallCalibrationLimits();
        limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationFrontSweepMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kStartupWallCalibrationFrontSweepAccelRadps2;
        return limits;
    }

    float SearchUnmappedCruiseSpeedMps() const
    {
        const float frontSensorForwardOffsetM = (std::min)(
            _speedVehicle.FrontLeft.GetPosition().x(),
            _speedVehicle.FrontRight.GetPosition().x());
        float frontWallOnThresholdM = Config::kFrontWallOnThresholdM;
        float frontWallOffThresholdM = Config::kFrontWallOffThresholdM;
        gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            frontWallOnThresholdM,
            frontWallOffThresholdM);

        const float safeCruiseSpeedMps = (std::min)(
            SearchLimits().maxSpeedMps,
            MazeMap::ComputeSafeUnmappedCruiseSpeed(
                SearchLimits().decelMps2,
                frontWallOnThresholdM,
                frontSensorForwardOffsetM,
                Config::kWallTouchContactStandoffM,
                Config::kDistanceToleranceM));
        return MazeMap::ApplyMinimumCruiseSpeedFloor(
            safeCruiseSpeedMps,
            Config::kMinimumAllowedCruiseSpeedMps,
            SearchLimits().maxSpeedMps);
    }

    static MazeMap::WallBeliefConfig BuildWallBeliefConfig()
    {
        MazeMap::WallBeliefConfig config{};
        config.hitLogOdds = Config::kWallBeliefHitLogOdds;
        config.missLogOdds = Config::kWallBeliefMissLogOdds;
        config.contradictoryMissLogOdds = Config::kWallBeliefContradictoryMissLogOdds;
        config.setThreshold = Config::kWallBeliefSetThreshold;
        config.clearThreshold = Config::kWallBeliefClearThreshold;
        config.saturationMagnitude = Config::kWallBeliefSaturationMagnitude;
        return config;
    }

    void SeedWallBeliefsFromKnownMaze()
    {
        _wallBeliefMap.Reset();
        const MazeMap::WallBeliefConfig beliefConfig = BuildWallBeliefConfig();
        constexpr MazeMap::Direction kDirections[] = {
            MazeMap::Up,
            MazeMap::Down,
            MazeMap::Left,
            MazeMap::Right
        };

        for (uint8_t x = 0U; x < 16U; ++x)
        {
            for (uint8_t y = 0U; y < 16U; ++y)
            {
                const MazeMap::CellCoordinates cell(x, y);
                const MazeMap::Cell& knownCell = _maze[cell];
                for (MazeMap::Direction direction : kDirections)
                {
                    const MazeMap::WallState hardState = knownCell.GetWall(direction);
                    if (hardState != MazeMap::Unknown)
                    {
                        _wallBeliefMap.SeedKnownState(cell, direction, hardState, beliefConfig);
                    }
                }
            }
        }
    }

    void SnapToStartPose()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SetStartPoint(_currentDirectionalLocation);
    }

    void PrimeKnownMissionStartCell()
    {
        _maze.SetWall(0, 0, MazeMap::Up, MazeMap::NoWall);
        _maze.SetWall(0, 0, MazeMap::Down, MazeMap::Wall);
        _maze.SetWall(0, 0, MazeMap::Left, MazeMap::Wall);
        _maze.SetWall(0, 0, MazeMap::Right, MazeMap::Wall);
        SeedWallBeliefsFromKnownMaze();
    }

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<Implementation*>(context)->OnRuntimeFault(reason);
    }

    void ResetForMode(bool maneuverTestMode, bool enableMissionTextLogging, const char* activeModeFaultSource)
    {
        _maneuverTestMode = maneuverTestMode;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = enableMissionTextLogging;
        _missionMazeSnapshotWritten = false;
        _goalPauseComplete = false;
        _missionComplete = false;
        _faulted = false;
        _activeModeFaultSource =
            (activeModeFaultSource != nullptr && activeModeFaultSource[0] != '\0') ? activeModeFaultSource : "mission";
        _runtime.FlushTextLog();
        _hasWallTouchStandoffEstimate = false;
    }

    void ShutdownTelemetryMode(bool disableFan)
    {
        if (disableFan)
        {
            SetRacingFanEnabled(false);
        }

        _drive.Brake();
        FlushTelemetryLog();
        _telemetryLoggingEnabled = false;
        CloseTelemetryLog();
    }

    bool OpenMissionTextLog()
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        return _runtime.EnsureTextLogOpen();
    }

    void FlushMissionTextLog()
    {
        if (_missionTextLoggingEnabled)
        {
            _runtime.FlushTextLog();
        }
    }

    void CloseMissionTextLog()
    {
        _runtime.FlushTextLog();
    }

    bool WriteMissionTextLineIfEnabled(const char* message)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (message == nullptr)
        {
            return false;
        }

        return _runtime.WriteTextLogEntry(
            kMissionTraceTextLogSource,
            micros(),
            "trace",
            message);
    }

    void DisableMissionTextLogging(const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return;
        }

        if (traceLabel != nullptr && traceLabel[0] != '\0')
        {
            AppendStartupTrace(traceLabel);
            char message[192] = {};
            const int written = snprintf(
                message,
                sizeof(message),
                "Mission text logging disabled: %s",
                traceLabel);
            if (written > 0 && written < static_cast<int>(sizeof(message)))
            {
                (void)_runtime.WriteTextLogEntry(
                    kMissionControllerTextLogSource,
                    micros(),
                    "status",
                    message);
            }
        }

        CloseMissionTextLog();
        _missionTextLoggingEnabled = false;
    }

    bool WriteMissionTraceLineBestEffort(const char* message, const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (WriteMissionTextLineIfEnabled(message))
        {
            return true;
        }

        DisableMissionTextLogging(traceLabel);
        return true;
    }

    static const char* FrontObservationSourceName(const SensorSnapshot& snapshot)
    {
        if (snapshot.frontWallUsesFallbackDetection)
        {
            return "front_pair_fallback";
        }
        if (snapshot.frontLeftWall && snapshot.frontRightWall)
        {
            return "front_left+front_right";
        }
        if (snapshot.frontLeftWall)
        {
            return "front_left";
        }
        if (snapshot.frontRightWall)
        {
            return "front_right";
        }
        return "front_left+front_right";
    }

    static const char* FrontObservationModeName(const SensorSnapshot& snapshot)
    {
        if (snapshot.frontWallUsesFallbackDetection)
        {
            return "fallback";
        }
        if (snapshot.frontWallUsesCharacterizationDetection)
        {
            return "characterized";
        }
        return "differential";
    }

    void LoadPersistedFrontWallCharacterization()
    {
        _frontWallCharacterization = MazeMap::FrontWallCharacterizationStorage{};
        _frontWallCharacterizationAvailable =
            TryReadPersistedFrontWallCharacterization(_frontWallCharacterization);

        if (_frontWallCharacterizationAvailable)
        {
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "initialize:front_characterization_loaded,samples=%u,terminal_distance_m=%.4f,reverse_speed_mps=%.3f",
                static_cast<unsigned>(_frontWallCharacterization.sampleCount),
                _frontWallCharacterization.terminalDistanceM,
                _frontWallCharacterization.commandedReverseSpeedMps);
            AppendStartupTrace(traceLine);
            (void)EmitMissionControllerLine("Loaded persisted front wall characterization.");
        }
        else
        {
            AppendStartupTrace("initialize:front_characterization_unavailable");
        }
    }

    static bool DoesFrontWallCharacterizationMatchIndicateWall(
        const MazeMap::FrontWallCharacterizationMatch& match)
    {
        return
            match.valid &&
            (match.sampleCount >= Config::kFrontWallCharacterizationMinMatchSamples) &&
            std::isfinite(match.scale) &&
            std::isfinite(match.normalizedCorrelation) &&
            std::isfinite(match.relativeResidual) &&
            (match.scale >= Config::kFrontWallCharacterizationMinScale) &&
            (match.normalizedCorrelation >= Config::kFrontWallCharacterizationMinCorrelation) &&
            (match.relativeResidual <= Config::kFrontWallCharacterizationMaxRelativeResidual);
    }

    bool TryApplyFrontWallCharacterizationToObservation(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        const char* observationOrigin,
        const SensorSnapshot* samples,
        const float* frontLeftCandidateDistanceM,
        const float* frontRightCandidateDistanceM,
        uint8_t sampleCount,
        SensorSnapshot& combinedSnapshot)
    {
        if (!_frontWallCharacterizationAvailable ||
            samples == nullptr ||
            frontLeftCandidateDistanceM == nullptr ||
            frontRightCandidateDistanceM == nullptr ||
            sampleCount == 0U)
        {
            return false;
        }

        float frontLeftBaseline = 0.0f;
        float frontRightBaseline = 0.0f;
        const bool haveFrontLeftBaseline =
            gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
                WallSensorId::FrontLeft,
                frontLeftBaseline);
        const bool haveFrontRightBaseline =
            gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
                WallSensorId::FrontRight,
                frontRightBaseline);

        float frontLeftMeasured[Config::kSearchRollingObservationSampleCount] = {};
        float frontRightMeasured[Config::kSearchRollingObservationSampleCount] = {};
        for (uint8_t index = 0U; index < sampleCount; ++index)
        {
            frontLeftMeasured[index] = samples[index].frontLeftDifferentialLight;
            frontRightMeasured[index] = samples[index].frontRightDifferentialLight;
        }

        MazeMap::FrontWallCharacterizationMatch frontLeftMatch{};
        MazeMap::FrontWallCharacterizationMatch frontRightMatch{};
        const bool haveFrontLeftMatch =
            haveFrontLeftBaseline &&
            MazeMap::TryMatchFrontWallCharacterizationChannel(
                _frontWallCharacterization,
                false,
                frontLeftMeasured,
                frontLeftCandidateDistanceM,
                sampleCount,
                frontLeftBaseline,
                frontLeftMatch);
        const bool haveFrontRightMatch =
            haveFrontRightBaseline &&
            MazeMap::TryMatchFrontWallCharacterizationChannel(
                _frontWallCharacterization,
                true,
                frontRightMeasured,
                frontRightCandidateDistanceM,
                sampleCount,
                frontRightBaseline,
                frontRightMatch);

        if (!(haveFrontLeftMatch || haveFrontRightMatch))
        {
            return false;
        }

        const bool frontLeftDetected =
            haveFrontLeftMatch &&
            DoesFrontWallCharacterizationMatchIndicateWall(frontLeftMatch);
        const bool frontRightDetected =
            haveFrontRightMatch &&
            DoesFrontWallCharacterizationMatchIndicateWall(frontRightMatch);
        combinedSnapshot.frontLeftWall = frontLeftDetected;
        combinedSnapshot.frontRightWall = frontRightDetected;
        combinedSnapshot.frontWall = frontLeftDetected || frontRightDetected;
        combinedSnapshot.frontWallObservationValid = true;
        combinedSnapshot.frontWallUsesFallbackDetection = false;
        combinedSnapshot.frontWallUsesCharacterizationDetection = true;

        AppendMissionTraceFormatted(
            "mission_front_curve_fit,cell=(%d,%d),abs=%s,origin=%s,left_fit=%u,left_n=%u,left_scale=%.3f,left_corr=%.3f,left_rr=%.3f,left_hit=%u,right_fit=%u,right_n=%u,right_scale=%.3f,right_corr=%.3f,right_rr=%.3f,right_hit=%u",
            observedCell.GetX(),
            observedCell.GetY(),
            DirectionName(observedDirection),
            (observationOrigin != nullptr) ? observationOrigin : "unknown",
            haveFrontLeftMatch ? 1U : 0U,
            static_cast<unsigned>(frontLeftMatch.sampleCount),
            frontLeftMatch.scale,
            frontLeftMatch.normalizedCorrelation,
            frontLeftMatch.relativeResidual,
            frontLeftDetected ? 1U : 0U,
            haveFrontRightMatch ? 1U : 0U,
            static_cast<unsigned>(frontRightMatch.sampleCount),
            frontRightMatch.scale,
            frontRightMatch.normalizedCorrelation,
            frontRightMatch.relativeResidual,
            frontRightDetected ? 1U : 0U);
        return true;
    }

    LoopController::ControlVector ObservationCaptureLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& capture = *static_cast<ObservationCaptureLoopState*>(rawState);
        if (!LogTelemetrySample(true, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        if (capture.nextSampleIndex < Config::kSearchRollingObservationSampleCount)
        {
            capture.samples[capture.nextSampleIndex] = state.sensors;
            float frontLeftDistanceM = NAN;
            float frontRightDistanceM = NAN;
            (void)TryComputeFrontWallCandidateDistancesForPose(
                _drive.GetPose(),
                _speedVehicle,
                capture.observedCell,
                capture.observedDirection,
                frontLeftDistanceM,
                frontRightDistanceM);
            capture.frontLeftCandidateDistanceM[capture.nextSampleIndex] = frontLeftDistanceM;
            capture.frontRightCandidateDistanceM[capture.nextSampleIndex] = frontRightDistanceM;
            ++capture.nextSampleIndex;
        }

        if (capture.nextSampleIndex < Config::kSearchRollingObservationSampleCount)
        {
            return LoopController::ControlVector::Brake;
        }

        RollingObservationVoteSummary voteSummary{};
        if (!BuildEvidenceObservationSnapshot(
                capture.samples,
                Config::kSearchRollingObservationSampleCount,
                *capture.outputSnapshot,
                voteSummary))
        {
            return FaultLoopPhase(services, "Stationary observation majority snapshot is invalid");
        }

        if (!TryApplyFrontWallCharacterizationToObservation(
                capture.observedCell,
                capture.observedDirection,
                "stationary",
                capture.samples,
                capture.frontLeftCandidateDistanceM,
                capture.frontRightCandidateDistanceM,
                Config::kSearchRollingObservationSampleCount,
                *capture.outputSnapshot))
        {
            AppendMissionTraceFormatted(
                "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=stationary,fallback_valid=%u",
                capture.observedCell.GetX(),
                capture.observedCell.GetY(),
                DirectionName(capture.observedDirection),
                capture.outputSnapshot->frontWallObservationValid ? 1U : 0U);
        }

        AppendMissionTraceFormatted(
            "mission_observation_stationary,cell=(%d,%d),abs=%s,samples=%u,front_valid=%u,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
            capture.observedCell.GetX(),
            capture.observedCell.GetY(),
            DirectionName(capture.observedDirection),
            static_cast<unsigned>(voteSummary.sampleCount),
            capture.outputSnapshot->frontWallObservationValid ? 1U : 0U,
            static_cast<unsigned>(voteSummary.frontWallVotes),
            static_cast<unsigned>(voteSummary.leftWindowValidVotes),
            static_cast<unsigned>(voteSummary.leftWallVotes),
            static_cast<unsigned>(voteSummary.rightWindowValidVotes),
            static_cast<unsigned>(voteSummary.rightWallVotes));
        return EndLoopPhase(services);
    }

    bool CaptureStationaryObservationSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        SensorSnapshot& observationSnapshot)
    {
        ObservationCaptureLoopState capture{};
        capture.observedCell = observedCell;
        capture.observedDirection = observedDirection;
        capture.outputSnapshot = &observationSnapshot;
        return RunLoopSession(&capture, &Implementation::ObservationCaptureLoopTick);
    }

    bool LogWallObservationDecision(
        const MazeMap::CellCoordinates& observedCell,
        const char* relativeDirectionName,
        MazeMap::Direction absoluteDirection,
        MazeMap::WallState observedState,
        const char* sensorSource,
        const char* sensorMode,
        float primaryDistanceM,
        float secondaryDistanceM,
        bool primaryDetected,
        bool secondaryDetected,
        const SensorSnapshot& snapshot,
        MazeMap::WallState beliefState,
        float beliefLogOdds)
    {
        char line[256] = {};
        const bool haveSecondaryDistance = std::isfinite(secondaryDistanceM);
        const int written =
            haveSecondaryDistance ?
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,obs_state=%s,belief=%s,log_odds=%.3f,sensor=%s,mode=%s,primary_hit=%u,secondary_hit=%u,primary_m=%.4f,secondary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                WallStateName(beliefState),
                beliefLogOdds,
                (sensorSource != nullptr) ? sensorSource : "unknown",
                (sensorMode != nullptr) ? sensorMode : "unknown",
                primaryDetected ? 1U : 0U,
                secondaryDetected ? 1U : 0U,
                primaryDistanceM,
                secondaryDistanceM) :
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,obs_state=%s,belief=%s,log_odds=%.3f,sensor=%s,primary_hit=%u,primary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                WallStateName(beliefState),
                beliefLogOdds,
                (sensorSource != nullptr) ? sensorSource : "unknown",
                primaryDetected ? 1U : 0U,
                primaryDistanceM);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return Fail("Unable to format wall observation log");
        }

        AppendStartupTrace(line);
        (void)WriteMissionTraceLineBestEffort(line, "mission_text_logging:wall_observation_write_failed");
        if (_telemetryLoggingEnabled && !WriteTelemetryEvent("wall_observation", line))
        {
            return Fail("Unable to write wall observation log");
        }
        (void)snapshot;
        return true;
    }

    bool EmitMissionControllerLine(const char* message)
    {
        if (message == nullptr)
        {
            return false;
        }

        if (_missionTextLoggingEnabled)
        {
            if (_runtime.WriteTextLogEntry(
                    kMissionControllerTextLogSource,
                    micros(),
                    "status",
                    message))
            {
                return true;
            }

            DisableMissionTextLogging("mission_text_logging:controller_write_failed");
        }

        return _runtime.WriteTextLogEntry(
            kMissionControllerTextLogSource,
            micros(),
            "status",
            message);
    }

    bool EmitMissionControllerFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLine(line);
    }

    bool EmitMissionControllerLineOrFail(const char* message)
    {
        return EmitMissionControllerLine(message);
    }

    bool EmitMissionControllerFormattedOrFail(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLineOrFail(line);
    }

    void AppendMissionTraceLine(const char* message)
    {
        if (message == nullptr)
        {
            return;
        }

        AppendStartupTrace(message);
        (void)WriteMissionTraceLineBestEffort(message, "mission_text_logging:trace_write_failed");
    }

    void AppendMissionTraceFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return;
        }

        char line[320] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return;
        }

        AppendMissionTraceLine(line);
    }

    bool WriteMissionMazeSnapshot(const char* trigger)
    {
        if (!_missionTextLoggingEnabled || _missionMazeSnapshotWritten)
        {
            return true;
        }

        const bool ok = MazeMap::ExportMazeSnapshot(_maze, "maze.txt");
        AppendStartupTrace(ok ? "mission_maze_snapshot:maze.txt" : "mission_maze_snapshot:write_failed");
        if (ok)
        {
            _missionMazeSnapshotWritten = true;
            (void)EmitMissionControllerFormatted("Maze snapshot written to maze.txt after %s", (trigger != nullptr) ? trigger : "unknown");
        }
        else
        {
            (void)EmitMissionControllerFormatted("Maze snapshot write failed after %s", (trigger != nullptr) ? trigger : "unknown");
        }

        return ok;
    }

    void AppendStartupCalibrationStateTrace(const char* label)
    {
        const PoseEstimate& pose = _drive.GetPose();
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[320] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_state:%s,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f,left_v=%.4f,right_v=%.4f",
            (label != nullptr) ? label : "unknown",
            pose.xMeters,
            pose.yMeters,
            pose.yawRad * RAD_TO_DEG_F,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationMoveTrace(
        const char* axis,
        float startMeters,
        float targetMeters,
        float signedTravelMeters)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_move:%s,start=%.4f,target=%.4f,signed=%.4f",
            (axis != nullptr) ? axis : "unknown",
            startMeters,
            targetMeters,
            signedTravelMeters);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTurnTrace(const char* label, float currentYawRad, float targetYawRad, float angleRad)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_turn:%s,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f",
            (label != nullptr) ? label : "unknown",
            currentYawRad * RAD_TO_DEG_F,
            targetYawRad * RAD_TO_DEG_F,
            angleRad * RAD_TO_DEG_F);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchPlanTrace(
        CalibrationWall wall,
        float expectedTravelM,
        float minLatchTravelM,
        float maxApproachTravelM,
        float targetYawRad)
    {
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch_plan:wall=%s,expected=%.4f,min_latch=%.4f,max_travel=%.4f,target_yaw_deg=%.2f",
            CalibrationWallName(wall),
            expectedTravelM,
            minLatchTravelM,
            maxApproachTravelM,
            targetYawRad * RAD_TO_DEG_F);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchTrace(
        CalibrationWall wall,
        float traveledDistanceM,
        float expectedTravelM,
        float minLatchTravelM,
        float finalYawErrorRad)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch:wall=%s,travel=%.4f,expected=%.4f,min_latch=%.4f,final_yaw_err_deg=%.2f,left_v=%.4f,right_v=%.4f",
            CalibrationWallName(wall),
            traveledDistanceM,
            expectedTravelM,
            minLatchTravelM,
            finalYawErrorRad * RAD_TO_DEG_F,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationSampleTrace(
        WallSensorId sensorId,
        CalibrationWall wall,
        float measuredValue,
        float fallbackDistanceM,
        float actualDistanceM)
    {
        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_sample:sensor=%s,wall=%s,measured=%.6f,fallback=%.4f,actual=%.4f",
            WallSensorIdName(sensorId),
            CalibrationWallName(wall),
            measuredValue,
            fallbackDistanceM,
            actualDistanceM);
        AppendStartupTrace(line);
    }

    bool ReseatMissionStartPoseWithPhasePrefix(const char* phasePrefix, uint16_t settleMs)
    {
        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[64] = {};

        if (!RetreatCalibrationPoseFromSideWallForSafeRotation(Config::kCellSizeM, centeringLimits, phasePrefix))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_south", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Down, CalibrationWall::South))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_west", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Left, CalibrationWall::West))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center_x", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownX(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_rotate_up", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, limits))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        snprintf(phaseName, sizeof(phaseName), "%s_settle", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        return HoldPosition(settleMs, phaseName);
    }

    bool ReseatCorridorRepeatabilityStartPose(uint8_t speedIndex, float centerOffsetFromTouchM)
    {
        (void)centerOffsetFromTouchM;
        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "corridor_%u_reseat", static_cast<unsigned>(speedIndex));
        return ReseatMissionStartPoseWithPhasePrefix(
            phasePrefix,
            AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs);
    }

    bool RunSingleCorridorRepeatabilityPass(uint8_t speedIndex, float cruiseSpeedMps, float outDistanceM, float centerOffsetFromTouchM)
    {
        const MotionLimits limits = CorridorRepeatabilityLimits(cruiseSpeedMps);
        const MotionLimits touchLimits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[48] = {};
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f southHeading = DirectionToUnitVector(MazeMap::Down);
        const float farCellCenterYM = (0.5f * Config::kCellSizeM) + outDistanceM;
        const float corridorSpanYM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount);
        const float farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            corridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        const Eigen::Vector2f farCellCenter(0.5f * Config::kCellSizeM, farCellCenterYM);
        const Eigen::Vector2f startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, farWallTouchYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(farCellCenterYM, centeringLimits, corridorSpanYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_home", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        if (!WriteCorridorRepeatabilityResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                speedIndex,
                cruiseSpeedMps,
                startPose,
                startTelemetry,
                _drive.GetPose(),
                _drive.GetTelemetry()))
        {
            return false;
        }

        return ReseatCorridorRepeatabilityStartPose(speedIndex, centerOffsetFromTouchM);
    }

    bool RunCorridorRepeatabilityPasses()
    {
        if constexpr (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount < 2U)
        {
            return Fail("Corridor repeatability row must be at least two cells long");
        }

        const float outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U);
        const float centerOffsetFromTouchM = MazeMap::ComputeMissionStartCenterAdvanceM(
            Config::kCellSizeM,
            Config::kMissionStartRearWallInsetM);
        if (centerOffsetFromTouchM <= 0.0f)
        {
            return Fail("Invalid corridor repeatability start-cell center offset");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            if (!RunSingleCorridorRepeatabilityPass(
                    speedIndex,
                    AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex],
                    outDistanceM,
                    centerOffsetFromTouchM))
            {
                return false;
            }
        }

        return true;
    }

    bool RunSinglePositionStraightAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t speedIndex,
        float cruiseSpeedMps)
    {
        _maze = geometry.maze;
        const MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        const MotionLimits turnLimits = PositionAccuracyAuditTurnLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f southHeading = DirectionToUnitVector(MazeMap::Down);
        const Eigen::Vector2f farCellCenter(0.5f * Config::kCellSizeM, geometry.farCellCenterYM);
        const Eigen::Vector2f startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        const DriveTelemetry outTelemetry = _drive.GetTelemetry();
        const float northStopErrorM = geometry.farCellCenterYM - poseBeforeTouch.yMeters;
        const float encoderOutDistanceM =
            0.5f *
            ((outTelemetry.leftDistanceM - startTelemetry.leftDistanceM) +
                (outTelemetry.rightDistanceM - startTelemetry.rightDistanceM));
        const float encoderOutErrorM = encoderOutDistanceM - geometry.outDistanceM;

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float northTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, geometry.farWallTouchYM, &northTouchCorrectionM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(geometry.farCellCenterYM, centeringLimits, geometry.northCorridorSpanYM))
        {
            return false;
        }

        const DriveTelemetry turnStartTelemetry = _drive.GetTelemetry();
        const float turnStartYawRad = _drive.GetPose().yawRad;
        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, turnLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            return false;
        }
        const DriveTelemetry turnEndTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - turnStartYawRad);
        const float leftTurnDeltaM = turnEndTelemetry.leftDistanceM - turnStartTelemetry.leftDistanceM;
        const float rightTurnDeltaM = turnEndTelemetry.rightDistanceM - turnStartTelemetry.rightDistanceM;
        if (!WritePositionInPlaceTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                MazeMap::Down,
                northTouchCorrectionM,
                leftTurnDeltaM,
                rightTurnDeltaM,
                yawChangeRad,
                _drive.GetPose().yawRad))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        if (!WritePositionStraightAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                speedIndex,
                cruiseSpeedMps,
                northStopErrorM,
                northTouchCorrectionM,
                encoderOutErrorM,
                startPose,
                _drive.GetPose()))
        {
            return false;
        }

        return true;
    }

    bool RunSinglePositionInPlaceTurnAuditPass(uint8_t turnIndex, MazeMap::Direction targetDirection)
    {
        if (!(targetDirection == MazeMap::Right || targetDirection == MazeMap::Left))
        {
            return Fail("Position audit in-place turn direction is invalid");
        }

        const MotionLimits turnLimits = PositionAccuracyAuditTurnLimits();
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_start", static_cast<unsigned>(turnIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        const float startYawRad = _drive.GetPose().yawRad;
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(startYawRad, DirectionToYawRad(targetDirection), angleRad))
        {
            return Fail("Position audit in-place turn angle is invalid");
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_turn", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteTurnProfile(angleRad, turnLimits))
        {
            return false;
        }

        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - startYawRad);
        const float leftDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;

        float touchCoordinateM = 0.0f;
        CalibrationWall touchWall = CalibrationWall::West;
        if (targetDirection == MazeMap::Right)
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::East;
        }
        else
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::West;
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_touch", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float touchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(targetDirection, touchWall, touchCoordinateM, &touchCorrectionM))
        {
            return false;
        }

        if (!WritePositionInPlaceTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                targetDirection,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                yawChangeRad,
                _drive.GetPose().yawRad))
        {
            return false;
        }

        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "position_ip_turn_%u_reseat", static_cast<unsigned>(turnIndex));
        return ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs);
    }

    bool RunSinglePositionSmoothTurnAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t codeIndex,
        MazeMap::ManeuverCode code,
        uint8_t speedIndex,
        float requestedCruiseSpeedMps)
    {
        _maze = geometry.maze;
        MazeMap::ManeuverPath forwardPath;
        MazeMap::ManeuverPath reversePath;
        uint8_t launchHalfSteps = 0U;
        uint8_t postStraightHalfSteps = 0U;
        if (!TryBuildPositionAuditSmoothTurnPaths(code, forwardPath, reversePath, launchHalfSteps, postStraightHalfSteps))
        {
            return Fail("Position audit smooth turn path is invalid");
        }

        const MazeMap::DirectionalLocation auditStart(
            MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0U, 0U)),
            MazeMap::Up);
        MazeMap::DirectionalLocation finalLocation;
        if (!TryValidatePositionAuditPath(geometry.maze, forwardPath, auditStart, finalLocation))
        {
            return Fail("Position audit smooth turn path does not fit fixture");
        }
        const MazeMap::DirectionalLocation returnStart(finalLocation.GetLocation(), -finalLocation.GetDirection());
        MazeMap::DirectionalLocation returnEnd;
        if (!TryValidatePositionAuditPath(geometry.maze, reversePath, returnStart, returnEnd))
        {
            return Fail("Position audit smooth turn reverse path does not fit fixture");
        }
        if (!(returnEnd.GetLocation() == auditStart.GetLocation()) || returnEnd.GetDirection() != MazeMap::Down)
        {
            return Fail("Position audit smooth turn reverse path does not return to start");
        }

        const MazeMap::ManeuverInstance turnManeuver(code, auditStart);
        const float nominalRadiusM = turnManeuver.GetNominalTurnRadiusMeters(Config::kCellSizeM);
        const MotionLimits straightLimits = PositionAccuracyAuditStraightLimits(requestedCruiseSpeedMps);
        const MotionLimits cornerLimits = PositionAccuracyAuditCornerLimits(requestedCruiseSpeedMps, nominalRadiusM);
        const MotionLimits calibrationLimits = PositionAccuracyAuditTurnLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const float turnCruiseSpeedMps = ManeuverSpeedLimit(code, cornerLimits);
        if (!(turnCruiseSpeedMps > 0.0f))
        {
            return Fail("Position audit smooth turn speed is invalid");
        }

        const MazeMap::DirectionalLocation launchLocation = auditStart.MoveForward(launchHalfSteps);
        const float postStraightDistanceM = 0.5f * Config::kCellSizeM * static_cast<float>(postStraightHalfSteps);

        float finalTargetXM = 0.0f;
        float finalTargetYM = 0.0f;
        finalLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, finalTargetXM, finalTargetYM);
        const Eigen::Vector2f northHeading = DirectionToUnitVector(MazeMap::Up);
        const Eigen::Vector2f finalHeading = DirectionToUnitVector(finalLocation.GetDirection());
        float launchXM = 0.0f;
        float launchYM = 0.0f;
        launchLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, launchXM, launchYM);
        const Eigen::Vector2f launchPosition(launchXM, launchYM);
        const Eigen::Vector2f finalPosition(finalTargetXM, finalTargetYM);
        const float launchDistanceM = launchYM - (0.5f * Config::kCellSizeM);
        const float maneuverExitSpeedMps = turnCruiseSpeedMps;
        const MazeMap::ManeuverInstance trackedManeuver(code, launchLocation, turnCruiseSpeedMps, maneuverExitSpeedMps);
        const float maneuverDistanceM = trackedManeuver.GetTravelDistanceMeters(Config::kCellSizeM);
        const float maneuverAngleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
        const bool hasTrackedManeuver = trackedManeuver.SupportsPointTracking();
        char phaseName[64] = {};

        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled)
        {
            SetRacingFanEnabled(true);
        }

        bool ok = false;
        do
        {
            snprintf(
                phaseName,
                sizeof(phaseName),
                "position_turn_%u_%u_start",
                static_cast<unsigned>(codeIndex),
                static_cast<unsigned>(speedIndex));
            if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
            {
                break;
            }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_launch",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!ExecuteStraightProfile(
                launchDistanceM,
                0.0f,
                requestedCruiseSpeedMps,
                turnCruiseSpeedMps,
                straightLimits,
                true,
                &northHeading,
                &launchPosition))
        {
            break;
        }

        const DriveTelemetry arcStartTelemetry = _drive.GetTelemetry();
        const float arcStartYawRad = _drive.GetPose().yawRad;
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_arc",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (hasTrackedManeuver)
        {
            if (!ExecuteSmoothTurnProfile(
                    trackedManeuver,
                    turnCruiseSpeedMps,
                    cornerLimits))
            {
                break;
            }
        }
        else if (!ExecuteArcProfile(
                maneuverDistanceM,
                maneuverAngleRad,
                turnCruiseSpeedMps,
                maneuverExitSpeedMps,
                turnCruiseSpeedMps,
                cornerLimits))
        {
            break;
        }
        const DriveTelemetry arcEndTelemetry = _drive.GetTelemetry();
        const float arcEndYawRad = _drive.GetPose().yawRad;
        const float leftArcDeltaM = arcEndTelemetry.leftDistanceM - arcStartTelemetry.leftDistanceM;
        const float rightArcDeltaM = arcEndTelemetry.rightDistanceM - arcStartTelemetry.rightDistanceM;
        const float yawChangeRad = WrapAngleRad(arcEndYawRad - arcStartYawRad);

        if (postStraightDistanceM > 0.0f)
        {
            snprintf(
                phaseName,
                sizeof(phaseName),
                "position_turn_%u_%u_post",
                static_cast<unsigned>(codeIndex),
                static_cast<unsigned>(speedIndex));
            if (!BeginTelemetryPhase(phaseName))
            {
                break;
            }
            if (!ExecuteStraightProfile(
                    postStraightDistanceM,
                    turnCruiseSpeedMps,
                    requestedCruiseSpeedMps,
                    0.0f,
                    straightLimits,
                    true,
                    &finalHeading,
                    &finalPosition))
            {
                break;
            }
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        SensorSnapshot snapshotBeforeTouch{};
        _sensors.Capture(true, _drive.GetPose(), snapshotBeforeTouch);
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(DirectionToYawRad(finalLocation.GetDirection()), poseBeforeTouch.yawRad);

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_touch_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        float eastTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Right, CalibrationWall::East, geometry.eastWallTouchXM, &eastTouchCorrectionM))
        {
            break;
        }

        if (!WritePositionSmoothTurnAuditResultImpl(
                [this](const char* type, const char* message) -> bool
                {
                    return WriteTelemetryEvent(type, message);
                },
                [this](const char* message) -> bool
                {
                    return Fail(message);
                },
                code,
                speedIndex,
                turnCruiseSpeedMps,
                nominalRadiusM,
                snapshotBeforeTouch.corridorErrorM,
                eastTouchCorrectionM,
                leftArcDeltaM,
                rightArcDeltaM,
                yawChangeRad,
                yawErrorDeg))
        {
            break;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_center_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!DriveCalibrationPoseToKnownX(finalTargetXM, centeringLimits))
        {
            break;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_face_left",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }
        if (!RotateCalibrationTo(MazeMap::Left, calibrationLimits))
        {
            break;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            break;
        }
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_return",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            break;
        }

            _currentDirectionalLocation = returnStart;
            _currentDirection = _currentDirectionalLocation.GetDirection();
            _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());
            MazeMap::ManeuverQueue queue(reversePath, _currentDirectionalLocation);
            queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
            ApplyAsymmetricQueueLimits(queue, cornerLimits, _speedVehicle, 0.0f, 0.0f);
            ok = ExecuteQueuedManeuvers(queue, cornerLimits, false);
        }
        while (false);

        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled)
        {
            SetRacingFanEnabled(false);
        }
        return ok;
    }

    bool RunPositionAccuracyAuditPasses()
    {
        if constexpr (AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount < 3U)
        {
            return Fail("Position accuracy audit north corridor must be at least three cells");
        }
        if constexpr (AuxMeasurementConfig::kPositionAuditEastBranchCellCount < 1U)
        {
            return Fail("Position accuracy audit east extension must be at least one cell");
        }

        const PositionAuditFixtureGeometry geometry = BuildPositionAuditFixtureGeometry();

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            char phasePrefix[56] = {};
            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase1", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionStraightAuditPass(
                    geometry,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase2", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    0U,
                    MazeMap::S90SS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase3", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    1U,
                    MazeMap::S90LS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }
        }

        return true;
    }

    void SeedStartupWallCalibrationPoseFromSouthWall()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SetPose(0.5f * Config::kCellSizeM, Config::kMissionStartRearWallInsetM, DirectionToYawRad(MazeMap::Up));
        AppendStartupCalibrationStateTrace("seed_south_wall_start");
    }

    bool RotateCalibrationTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Startup calibration turn angle is invalid");
        }
        AppendStartupCalibrationTurnTrace("rotate_begin", _drive.GetPose().yawRad, targetYawRad, angleRad);
        if (!ExecuteTurnProfile(angleRad, limits))
        {
            return false;
        }

        AppendStartupCalibrationStateTrace("rotate_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownX(float targetXMeters, const MotionLimits& limits)
    {
        // Audit reseat can target a global fixture x beyond one cell after an east-wall touch.
        if (!MazeMap::IsValidCalibrationCenterCoordinateM(targetXMeters))
        {
            return Fail("Startup calibration target x is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingX = startPose.headingUnit.x();
        if (std::fabs(headingX) < 0.5f)
        {
            return Fail("Startup calibration x reposition requires east-west heading");
        }

        const float deltaXMeters = targetXMeters - startPose.xMeters;
        const float signedTravelMeters = deltaXMeters / headingX;
        const Eigen::Vector2f targetHeading = startPose.headingUnit;
        const Eigen::Vector2f targetPosition(targetXMeters, startPose.yMeters);
        AppendStartupCalibrationMoveTrace("x", startPose.xMeters, targetXMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        AppendStartupCalibrationStateTrace("x_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits, float maxAllowedYMeters)
    {
        if (!(std::isfinite(targetYMeters) &&
            std::isfinite(maxAllowedYMeters) &&
            targetYMeters >= 0.0f &&
            maxAllowedYMeters >= 0.0f &&
            targetYMeters <= maxAllowedYMeters))
        {
            return Fail("Startup calibration target y is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingY = startPose.headingUnit.y();
        if (std::fabs(headingY) < 0.5f)
        {
            return Fail("Startup calibration y reposition requires north-south heading");
        }

        const float deltaYMeters = targetYMeters - startPose.yMeters;
        const float signedTravelMeters = deltaYMeters / headingY;
        const Eigen::Vector2f targetHeading = startPose.headingUnit;
        const Eigen::Vector2f targetPosition(startPose.xMeters, targetYMeters);
        AppendStartupCalibrationMoveTrace("y", startPose.yMeters, targetYMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        AppendStartupCalibrationStateTrace("y_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits)
    {
        return DriveCalibrationPoseToKnownY(targetYMeters, limits, Config::kCellSizeM);
    }

    float ComputeCalibrationSideRotationClearanceM() const
    {
        return Config::kWallCalibrationWallClearanceM + Config::kDistanceToleranceM;
    }

    float ComputeCalibrationSafeMinCenterXForWestWallRotationM() const
    {
        return MazeMap::ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    float ComputeCalibrationSafeMaxCenterXForEastWallRotationM(float spanXMeters) const
    {
        return MazeMap::ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(
            spanXMeters,
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    bool RetreatCalibrationPoseFromSideWallForSafeRotation(
        float spanXMeters,
        const MotionLimits& limits,
        const char* phasePrefix = nullptr)
    {
        if (!(std::isfinite(spanXMeters) && spanXMeters > 0.0f))
        {
            return Fail("Startup calibration side-clear span is invalid");
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float headingX = pose.headingUnit.x();
        if (std::fabs(headingX) < 0.5f)
        {
            return true;
        }

        const float safeMinCenterXM = ComputeCalibrationSafeMinCenterXForWestWallRotationM();
        const float safeMaxCenterXM = ComputeCalibrationSafeMaxCenterXForEastWallRotationM(spanXMeters);
        if (!(safeMinCenterXM > 0.0f &&
            safeMaxCenterXM > safeMinCenterXM &&
            safeMaxCenterXM < spanXMeters))
        {
            return Fail("Startup calibration side-clear target is invalid");
        }

        float targetXMeters = pose.xMeters;
        if (headingX > 0.5f)
        {
            targetXMeters = (std::min)(pose.xMeters, safeMaxCenterXM);
        }
        else
        {
            targetXMeters = (std::max)(pose.xMeters, safeMinCenterXM);
        }

        if (std::fabs(targetXMeters - pose.xMeters) <= Config::kDistanceToleranceM)
        {
            return true;
        }

        if (phasePrefix != nullptr)
        {
            char phaseName[64] = {};
            snprintf(phaseName, sizeof(phaseName), "%s_clear_side", phasePrefix);
            if (!BeginTelemetryPhase(phaseName))
            {
                return false;
            }
        }

        return DriveCalibrationPoseToKnownX(targetXMeters, limits);
    }

    struct WallTouchPoseResetTarget
    {
        float xMeters = 0.0f;
        float yMeters = 0.0f;
        float yawRad = 0.0f;
        bool enabled = false;
    };

    bool ExecuteWallTouchOff(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        const WallTouchPoseResetTarget* poseResetTarget,
        WallTouchOutcome& outcome,
        float& traveledDistanceM,
        float* seatedYawErrorRad = nullptr)
    {
        return ExecuteWallTouchOffLoopDriven(
            targetYawRad,
            minLatchTravelM,
            maxApproachTravelM,
            allowPassThroughNoWall,
            poseResetTarget,
            outcome,
            traveledDistanceM,
            seatedYawErrorRad);
    }

    LoopController::ControlVector WallTouchLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& wallTouch = *static_cast<WallTouchLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const float clampedMinLatchTravelM = (std::max)(0.0f, wallTouch.minLatchTravelM);
        const float clampedMaxApproachTravelM =
            (std::max)(clampedMinLatchTravelM, wallTouch.maxApproachTravelM);
        if (!(std::isfinite(clampedMaxApproachTravelM) && (clampedMaxApproachTravelM > 0.0f)))
        {
            return FaultLoopPhase(services, "Wall touch-off max travel is invalid");
        }

        auto appendTraceLine = [this](const char* line)
        {
            if (line != nullptr)
            {
                AppendStartupTrace(line);
            }
        };
        auto traceStateTransition =
            [&wallTouch, &appendTraceLine](MazeMap::App::Internal::Runtime::WallTouchState fromState,
                                           MazeMap::App::Internal::Runtime::WallTouchState toState,
                                           float traveledDistanceM)
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(fromState),
                MazeMap::App::Internal::Runtime::WallTouchStateName(toState),
                static_cast<unsigned long>(millis() - wallTouch.touchStartMs),
                traveledDistanceM);
            appendTraceLine(line);
        };

        const MazeMap::App::Internal::Runtime::WallTouchObservation observation =
            MazeMap::App::Internal::Runtime::MakeWallTouchObservation(state.sensors);
        const unsigned long nowMs = millis();
        const unsigned long elapsedMs = nowMs - wallTouch.touchStartMs;
        const unsigned long stateElapsedMs = nowMs - wallTouch.stateStartMs;
        const PoseEstimate& pose = _drive.GetPose();
        const DriveTelemetry& telemetry = state.driveTelemetry;
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - wallTouch.startDistanceM);
        const bool frontSignalActive =
            observation.frontWall ||
            observation.frontLeftWall ||
            observation.frontRightWall;
        wallTouch.result.finalTravelM = traveledDistanceM;

        if ((wallTouch.runtimeState != MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease) &&
            (wallTouch.runtimeState != MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance) &&
            (traveledDistanceM >= clampedMaxApproachTravelM))
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:max_travel,state=%s,travel=%.4f,expected=%.4f,max=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(wallTouch.runtimeState),
                traveledDistanceM,
                clampedMinLatchTravelM,
                clampedMaxApproachTravelM);
            appendTraceLine(line);
            if (wallTouch.allowPassThroughNoWall && (wallTouch.contactConfirmedStartMs == 0UL))
            {
                wallTouch.result.outcome = WallTouchOutcome::PassedThroughNoWall;
                wallTouch.passThroughSettle.timeoutMessage = "Wall touch-off failed to settle after pass-through";
                wallTouch.passThroughSettle.stationaryHoldMs = Config::kStartupWallCalibrationSettleMs;
                wallTouch.passThroughSettle.timeoutMs = 0U;
                wallTouch.passThroughSettle.brakeCommand = true;
                TransitionLoopPhase(&wallTouch.passThroughSettle, &Implementation::SettleLoopTick, services);
                return LoopController::ControlVector::Brake;
            }
            return FaultLoopPhase(services, "Wall touch-off exceeded max travel");
        }

        if (MazeMap::App::Internal::Runtime::HasWallTouchEncoderMotion(
                wallTouch.lastMotionTelemetry,
                telemetry,
                Config::kWallTouchProgressStallDistanceM))
        {
            wallTouch.lastMotionMs = nowMs;
            wallTouch.lastMotionTelemetry = telemetry;
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek)
        {
            wallTouch.approachDriveCommand =
                MazeMap::App::Internal::Runtime::ComputeWallTouchApproachDriveCommand(
                    traveledDistanceM,
                    clampedMinLatchTravelM);
            LoopController::ControlVector command = LoopController::ControlVector::Brake;
            if (MazeMap::App::Internal::Runtime::ShouldBrakeWallTouchApproachForEncoderSpeed(telemetry))
            {
                command = LoopController::ControlVector::Brake;
            }
            else
            {
                wallTouch.approachDriveCommand =
                    MazeMap::App::Internal::Runtime::LimitWallTouchApproachDriveCommandByEncoderSpeed(
                        wallTouch.approachDriveCommand,
                        telemetry);
                command = LoopController::ControlVector::RawMotorPwm(
                    wallTouch.approachDriveCommand,
                    wallTouch.approachDriveCommand);
            }

            const bool motionCollapseIndicator = MazeMap::IsWallTouchContactSample(
                traveledDistanceM,
                pose.linearSpeedMps,
                Config::kWallTouchMinApproachDistanceM,
                clampedMinLatchTravelM,
                Config::kMotionSettleSpeedThresholdMps,
                elapsedMs,
                Config::kWallTouchMinCommandTimeMs);
            const bool progressStallIndicator =
                (elapsedMs >= Config::kWallTouchMinCommandTimeMs) &&
                ((nowMs - wallTouch.lastMotionMs) >= Config::kWallTouchProgressStallWindowMs);
            const std::uint8_t indicatorCount = MazeMap::CountWallTouchContactIndicators(
                frontSignalActive,
                motionCollapseIndicator,
                progressStallIndicator);
            if (indicatorCount >= 2U)
            {
                if (!wallTouch.contactCandidateActive)
                {
                    wallTouch.contactCandidateStartMs = nowMs;
                    wallTouch.contactCandidateActive = true;
                }
                else if (MazeMap::HasWallTouchConfirmedContact(
                    nowMs - wallTouch.contactCandidateStartMs,
                    Config::kWallTouchContactConfirmationMs,
                    indicatorCount))
                {
                    wallTouch.contactConfirmedStartMs = wallTouch.contactCandidateStartMs;
                    wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp;
                    wallTouch.stateStartMs = nowMs;
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:contact_confirmed,travel=%.4f,elapsed_ms=%lu,front=%u,collapse=%u,stall=%u",
                        traveledDistanceM,
                        elapsedMs,
                        frontSignalActive ? 1U : 0U,
                        motionCollapseIndicator ? 1U : 0U,
                        progressStallIndicator ? 1U : 0U);
                    appendTraceLine(line);
                    traceStateTransition(
                        MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek,
                        wallTouch.runtimeState,
                        traveledDistanceM);
                }
            }
            else
            {
                wallTouch.contactCandidateActive = false;
            }

            return command;
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp)
        {
            const float rampAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchSeatRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchSeatRampMs, static_cast<std::uint16_t>(1U)));
            const float seatDriveCommand =
                wallTouch.approachDriveCommand +
                ((Config::kWallTouchSeatRampMaxDriveCommand - wallTouch.approachDriveCommand) * rampAlpha);
            if (stateElapsedMs >= Config::kWallTouchSeatRampMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::SeatingPreloadRamp,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(seatDriveCommand, seatDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell)
        {
            if (stateElapsedMs >= Config::kWallTouchInitialSeatDwellMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither;
                wallTouch.stateStartMs = nowMs;
                wallTouch.currentCycleStartYawRad = pose.yawRad;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                wallTouch.currentCycleFrontSignalValid = true;
                wallTouch.haveSquareSample = false;
                wallTouch.completedHalfCycles = 0U;
                wallTouch.result.completedFullCycles = 0U;
                wallTouch.consecutiveGoodFullCycles = 0U;
                wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
                wallTouch.frontSignalMissingStartMs = 0UL;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::InitialSeatingDwell,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither)
        {
            const unsigned long contactDurationMs =
                (wallTouch.contactConfirmedStartMs > 0UL) ?
                (nowMs - wallTouch.contactConfirmedStartMs) :
                0UL;
            wallTouch.result.confirmedContactMs = contactDurationMs;
            if (!frontSignalActive)
            {
                if (wallTouch.frontSignalMissingStartMs == 0UL)
                {
                    wallTouch.frontSignalMissingStartMs = nowMs;
                }
                else if ((nowMs - wallTouch.frontSignalMissingStartMs) >= Config::kWallTouchContactConfirmationMs)
                {
                    char line[192] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:front_signal_invalid,elapsed_ms=%lu,travel=%.4f",
                        contactDurationMs,
                        traveledDistanceM);
                    appendTraceLine(line);
                    return FaultLoopPhase(services, "Wall touch-off front sensors invalid during square-up");
                }
            }
            else
            {
                wallTouch.frontSignalMissingStartMs = 0UL;
            }

            const MazeMap::OpenLoopDriveCommand ditherCommand = MazeMap::ComputeOpenLoopYawDitherCommand(
                Config::kWallTouchSeatRampMaxDriveCommand,
                stateElapsedMs,
                Config::kWallTouchSeatWiggleHalfPeriodMs,
                Config::kWallTouchSeatWiggleBlendMs,
                wallTouch.ditherTurnFraction,
                Config::kWallTouchSeatWiggleRetainedForwardFraction);

            const unsigned long halfCycleIndex =
                stateElapsedMs /
                (std::max)(Config::kWallTouchSeatWiggleHalfPeriodMs, static_cast<std::uint16_t>(1U));
            if (wallTouch.haveSquareSample && (halfCycleIndex != wallTouch.lastHalfCycleIndex))
            {
                ++wallTouch.completedHalfCycles;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = (std::max)(
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                    std::fabs(wallTouch.lastSquareFrontSkewM));
                wallTouch.currentCycleMaxResidualYawRateRadps = (std::max)(
                    wallTouch.currentCycleMaxResidualYawRateRadps,
                    std::fabs(wallTouch.lastSquareYawRateRadps));
                wallTouch.currentCycleFrontSignalValid =
                    wallTouch.currentCycleFrontSignalValid && wallTouch.lastSquareFrontSignalValid;

                char halfCycleLine[256] = {};
                snprintf(
                    halfCycleLine,
                    sizeof(halfCycleLine),
                    "startup_cal_touch:half_cycle,index=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,turn_fraction=%.3f",
                    static_cast<unsigned>(wallTouch.completedHalfCycles),
                    std::fabs(wallTouch.lastSquareFrontSkewM),
                    std::fabs(wallTouch.lastSquareYawRateRadps),
                    wallTouch.ditherTurnFraction);
                appendTraceLine(halfCycleLine);

                if ((wallTouch.completedHalfCycles & 1U) == 0U)
                {
                    ++wallTouch.result.completedFullCycles;
                    const float netYawChangeMagnitudeRad =
                        std::fabs(AngleErrorRad(wallTouch.currentCycleStartYawRad, wallTouch.lastSquareYawRad));
                    const bool cycleGood = MazeMap::IsWallTouchSquareCycleGood(
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        Config::kWallTouchSquareFrontSkewThresholdM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        Config::kWallTouchSquareResidualYawRateThresholdRadps,
                        netYawChangeMagnitudeRad,
                        Config::kWallTouchSquareNetYawChangeThresholdRad,
                        wallTouch.currentCycleFrontSignalValid);
                    wallTouch.consecutiveGoodFullCycles =
                        cycleGood ? static_cast<std::uint8_t>(wallTouch.consecutiveGoodFullCycles + 1U) : 0U;

                    char cycleLine[320] = {};
                    snprintf(
                        cycleLine,
                        sizeof(cycleLine),
                        "startup_cal_touch:full_cycle,index=%u,good=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,net_yaw_deg=%.2f,contact_ms=%lu,turn_fraction=%.3f",
                        static_cast<unsigned>(wallTouch.result.completedFullCycles),
                        cycleGood ? 1U : 0U,
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        RAD_TO_DEG_F * netYawChangeMagnitudeRad,
                        contactDurationMs,
                        wallTouch.ditherTurnFraction);
                    appendTraceLine(cycleLine);

                    if (!cycleGood &&
                        (wallTouch.ditherTurnFraction < Config::kWallTouchSeatWiggleMaxTurnFraction) &&
                        (MazeMap::HasWallTouchSquareUpSaturated(
                            wallTouch.previousCycleFrontSkewMagnitudeM,
                            wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                            Config::kWallTouchSquareImprovementSaturationThresholdM) ||
                            (wallTouch.result.completedFullCycles >= Config::kWallTouchSeatMinimumFullCycles)))
                    {
                        wallTouch.ditherTurnFraction = MazeMap::ComputeWallTouchSeatWiggleTurnFraction(
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatWiggleTurnFraction,
                            Config::kWallTouchSeatWiggleTurnFractionStep,
                            Config::kWallTouchSeatWiggleMaxTurnFraction);
                    }

                    wallTouch.previousCycleFrontSkewMagnitudeM = wallTouch.currentCycleMaxFrontSkewMagnitudeM;
                    wallTouch.currentCycleStartYawRad = wallTouch.lastSquareYawRad;
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                    wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                    wallTouch.currentCycleFrontSignalValid = true;

                    if (MazeMap::IsWallTouchSquareSuccessEligible(
                            contactDurationMs,
                            Config::kWallTouchMinimumConfirmedContactMs,
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatMinimumFullCycles,
                            wallTouch.consecutiveGoodFullCycles,
                            Config::kWallTouchSeatRequiredGoodFullCycles))
                    {
                        wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold;
                        wallTouch.stateStartMs = nowMs;
                        wallTouch.result.seatedTravelM = traveledDistanceM;
                        wallTouch.result.seatedYawErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
                        traceStateTransition(
                            MazeMap::App::Internal::Runtime::WallTouchState::SquareUpDither,
                            wallTouch.runtimeState,
                            traveledDistanceM);
                    }
                }
            }

            if (contactDurationMs >= Config::kWallTouchSquareUpTimeoutMs)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:square_timeout,contact_ms=%lu,turn_fraction=%.3f,cycles=%u",
                    contactDurationMs,
                    wallTouch.ditherTurnFraction,
                    static_cast<unsigned>(wallTouch.result.completedFullCycles));
                appendTraceLine(line);
                return FaultLoopPhase(services, "Wall touch-off square-up timed out");
            }

            wallTouch.haveSquareSample = true;
            wallTouch.lastHalfCycleIndex = halfCycleIndex;
            wallTouch.lastSquareYawRad = pose.yawRad;
            wallTouch.lastSquareFrontSkewM = observation.frontSkewM;
            wallTouch.lastSquareYawRateRadps = pose.angularSpeedRadps;
            wallTouch.lastSquareFrontSignalValid = frontSignalActive;
            return LoopController::ControlVector::RawMotorPwm(
                ditherCommand.leftDriveCommand,
                ditherCommand.rightDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold)
        {
            if (!wallTouch.seatedResetApplied &&
                (stateElapsedMs >= (Config::kWallTouchPostSquareHoldMs / 2U)))
            {
                wallTouch.seatedResetApplied = true;
                if (wallTouch.poseResetTarget != nullptr && wallTouch.poseResetTarget->enabled)
                {
                    _drive.SetPose(
                        wallTouch.poseResetTarget->xMeters,
                        wallTouch.poseResetTarget->yMeters,
                        wallTouch.poseResetTarget->yawRad);
                    AppendStartupCalibrationStateTrace("touch_pose_set");
                }
            }
            if (stateElapsedMs >= Config::kWallTouchPostSquareHoldMs)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:reset_pose,x=%.4f,y=%.4f,yaw_deg=%.2f,travel=%.4f",
                    _drive.GetPose().xMeters,
                    _drive.GetPose().yMeters,
                    RAD_TO_DEG_F * _drive.GetPose().yawRad,
                    wallTouch.result.seatedTravelM);
                appendTraceLine(line);
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::PostSquareSeatedHold,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease)
        {
            const float releaseAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchReleaseRampMs, static_cast<std::uint16_t>(1U)));
            const float forwardPreloadCommand =
                Config::kWallTouchSeatRampMaxDriveCommand * (1.0f - releaseAlpha);
            float reverseCommand = 0.0f;
            if (Config::kWallTouchReleaseReverseOverlapMs >= Config::kWallTouchReleaseRampMs)
            {
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * releaseAlpha;
            }
            else if (stateElapsedMs >= (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs))
            {
                const unsigned long reverseElapsedMs =
                    stateElapsedMs - (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs);
                const float reverseAlpha =
                    static_cast<float>((std::min)(reverseElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseReverseOverlapMs))) /
                    static_cast<float>((std::max)(Config::kWallTouchReleaseReverseOverlapMs, static_cast<std::uint16_t>(1U)));
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * reverseAlpha;
            }

            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            if ((wallTouch.result.reverseDistanceM >= Config::kDistanceToleranceM) && !frontSignalActive)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:release_clear,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            else if (stateElapsedMs >= Config::kWallTouchReleaseRampMs)
            {
                wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    MazeMap::App::Internal::Runtime::WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                forwardPreloadCommand - reverseCommand,
                forwardPreloadCommand - reverseCommand);
        }

        if (wallTouch.runtimeState == MazeMap::App::Internal::Runtime::WallTouchState::ReverseToClearance)
        {
            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            const float headingErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
            float angularCommandRadps =
                Config::kStraightHeadingKp * headingErrorRad;
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -Config::kWallTouchReverseMaxAngularCommandRadps,
                Config::kWallTouchReverseMaxAngularCommandRadps);

            if (wallTouch.result.reverseDistanceM >= Config::kWallTouchFrontClearanceDistanceM)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_reached,reverse_m=%.4f,target_m=%.4f",
                    wallTouch.result.reverseDistanceM,
                    Config::kWallTouchFrontClearanceDistanceM);
                appendTraceLine(line);
                return EndLoopPhase(services);
            }

            if ((stateElapsedMs >= Config::kMotionSettleTimeoutMs) && frontSignalActive)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_failed,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                return FaultLoopPhase(services, "Wall touch-off failed to establish front-wall clearance");
            }

            return _drive.PointControlVector(
                -Config::kWallTouchReverseSpeedMps,
                angularCommandRadps,
                kMissionDriveBaseTrackingCommandPd);
        }

        return EndLoopPhase(services);
    }

    bool ExecuteWallTouchOffLoopDriven(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        const WallTouchPoseResetTarget* poseResetTarget,
        WallTouchOutcome& outcome,
        float& traveledDistanceM,
        float* seatedYawErrorRad = nullptr)
    {
        outcome = WallTouchOutcome::SeatedContact;
        traveledDistanceM = 0.0f;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = 0.0f;
        }

        WallTouchLoopState wallTouch{};
        wallTouch.targetYawRad = targetYawRad;
        wallTouch.minLatchTravelM = minLatchTravelM;
        wallTouch.maxApproachTravelM = maxApproachTravelM;
        wallTouch.allowPassThroughNoWall = allowPassThroughNoWall;
        wallTouch.poseResetTarget = poseResetTarget;
        wallTouch.seatedYawErrorRad = seatedYawErrorRad;
        wallTouch.startDistanceM = _drive.GetAverageDistanceMeters();
        wallTouch.touchStartMs = millis();
        wallTouch.stateStartMs = wallTouch.touchStartMs;
        wallTouch.lastMotionMs = wallTouch.touchStartMs;
        wallTouch.lastMotionTelemetry = _drive.GetTelemetry();
        wallTouch.approachDriveCommand = Config::kWallTouchDriveCommand;
        wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
        wallTouch.previousCycleFrontSkewMagnitudeM = std::numeric_limits<float>::infinity();
        wallTouch.currentCycleStartYawRad = _drive.GetPose().yawRad;
        wallTouch.runtimeState = MazeMap::App::Internal::Runtime::WallTouchState::ContactSeek;
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                MazeMap::App::Internal::Runtime::WallTouchStateName(
                    MazeMap::App::Internal::Runtime::WallTouchState::EntryConditioning),
                MazeMap::App::Internal::Runtime::WallTouchStateName(wallTouch.runtimeState),
                0UL,
                0.0f);
            AppendStartupTrace(line);
        }

        if (!RunLoopSession(&wallTouch, &Implementation::WallTouchLoopTick))
        {
            return false;
        }

        outcome = wallTouch.result.outcome;
        traveledDistanceM = wallTouch.result.seatedTravelM;
        if (seatedYawErrorRad != nullptr)
        {
            *seatedYawErrorRad = wallTouch.result.seatedYawErrorRad;
        }
        return true;
    }

    bool TryTouchWallAndMaybeSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        bool allowPassThroughNoWall,
        WallTouchOutcome& outcome,
        float* traveledDistanceM = nullptr)
    {
        outcome = WallTouchOutcome::SeatedContact;
        if (!(std::isfinite(targetCoordinateM) && targetCoordinateM >= 0.0f))
        {
            return Fail("Startup calibration touch coordinate is invalid");
        }

        const MotionLimits limits = StartupWallCalibrationLimits();
        if (!RotateCalibrationTo(facingDirection, limits))
        {
            return false;
        }
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        float xMeters = pose.xMeters;
        float yMeters = pose.yMeters;
        float expectedTravelM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.xMeters - xMeters);
            break;
        case CalibrationWall::East:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(xMeters - pose.xMeters);
            break;
        case CalibrationWall::South:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.yMeters - yMeters);
            break;
        case CalibrationWall::North:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(yMeters - pose.yMeters);
            break;
        default:
            break;
        }

        const float targetYawRad = DirectionToYawRad(facingDirection);
        const float minLatchTravelM = MazeMap::ComputeWallTouchMinimumLatchTravelM(
            expectedTravelM,
            Config::kWallTouchMinApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        const float maxApproachTravelM = MazeMap::ComputeWallTouchMaximumApproachDistanceM(
            expectedTravelM,
            Config::kWallTouchBaseMaxApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        AppendStartupCalibrationTouchPlanTrace(wall, expectedTravelM, minLatchTravelM, maxApproachTravelM, targetYawRad);

        float localTravelM = 0.0f;
        float finalYawErrorRad = 0.0f;
        const WallTouchPoseResetTarget poseResetTarget{
            xMeters,
            yMeters,
            DirectionToYawRad(facingDirection),
            true
        };
        if (!ExecuteWallTouchOff(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                &poseResetTarget,
                outcome,
                localTravelM,
                &finalYawErrorRad))
        {
            return false;
        }

        if (outcome == WallTouchOutcome::SeatedContact)
        {
            AppendStartupCalibrationTouchTrace(wall, localTravelM, expectedTravelM, minLatchTravelM, finalYawErrorRad);
        }

        if (traveledDistanceM != nullptr)
        {
            *traveledDistanceM = localTravelM;
        }
        return true;
    }

    bool TouchWallAndSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        float* traveledDistanceM = nullptr)
    {
        WallTouchOutcome outcome = WallTouchOutcome::SeatedContact;
        if (!TryTouchWallAndMaybeSetKnownWallCoordinate(
                facingDirection,
                wall,
                targetCoordinateM,
                false,
                outcome,
                traveledDistanceM))
        {
            return false;
        }

        return true;
    }

    bool TouchWallAndSetPose(MazeMap::Direction facingDirection, CalibrationWall wall, float* traveledDistanceM = nullptr)
    {
        float targetCoordinateM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::East:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::South:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromSouthWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::North:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        default:
            return Fail("Startup calibration wall touch is invalid");
        }

        return TouchWallAndSetKnownWallCoordinate(
            facingDirection,
            wall,
            targetCoordinateM,
            traveledDistanceM);
    }

    bool TryComputeCalibrationReferenceDistanceM(const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM) const
    {
        const PoseEstimate& pose = _drive.GetPose();
        switch (wall)
        {
        case CalibrationWall::West:
            return TryDistanceToWestWall(pose, sensor, actualDistanceM);
        case CalibrationWall::East:
            return TryDistanceToEastWall(pose, sensor, actualDistanceM);
        case CalibrationWall::South:
            return TryDistanceToSouthWall(pose, sensor, actualDistanceM);
        case CalibrationWall::North:
            return false;
        default:
            return false;
        }
    }

    bool StoreWallCalibrationPoint(
        WallSensorId sensorId,
        CalibrationWall wall,
        float actualDistanceM,
        const WallSensorCalibrationInput& input,
        const RobustSignalBand* differentialLightBand = nullptr)
    {
        if (!(std::isfinite(actualDistanceM) && actualDistanceM > 0.0f))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        if (!gWallDistanceCalibration.AddPoint(sensorId, input.measuredValue, actualDistanceM, input.ambientLight))
        {
            return Fail("Unable to store startup wall calibration point");
        }
        if ((sensorId == WallSensorId::SideLeft) || (sensorId == WallSensorId::SideRight))
        {
            if (std::isfinite(input.differentialLight) && input.differentialLight > 0.0f)
            {
                gWallDistanceCalibration.SetSideWallReferenceDifferentialLight(sensorId, input.differentialLight);
                gWallDistanceCalibration.SetSideWallReferenceDistanceM(sensorId, actualDistanceM);
                if ((differentialLightBand != nullptr) &&
                    std::isfinite(differentialLightBand->low) &&
                    std::isfinite(differentialLightBand->high) &&
                    differentialLightBand->low > 0.0f &&
                    differentialLightBand->high >= differentialLightBand->low)
                {
                    gWallDistanceCalibration.SetSideWallReferenceDifferentialLightBand(
                        sensorId,
                        differentialLightBand->low,
                        differentialLightBand->high);
                }
            }
        }
        else if (IsFrontWallSensor(sensorId) &&
            (differentialLightBand != nullptr) &&
            std::isfinite(input.measuredValue) &&
            input.measuredValue > 0.0f &&
            std::isfinite(differentialLightBand->low) &&
            std::isfinite(differentialLightBand->high) &&
            differentialLightBand->low > 0.0f &&
            differentialLightBand->high >= differentialLightBand->low)
        {
            gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
                sensorId,
                input.measuredValue,
                differentialLightBand->low,
                differentialLightBand->high);
        }
        AppendStartupCalibrationSampleTrace(sensorId, wall, input.measuredValue, input.fallbackDistanceM, actualDistanceM);

        return true;
    }

    bool AddWallCalibrationPoint(WallSensorId sensorId, const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM)
    {
        if (!TryComputeCalibrationReferenceDistanceM(sensor, wall, actualDistanceM))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        const WallSensorCalibrationCapture capture = SampleWallCalibrationCaptureAverageRaw(sensorId, sensor);
        return StoreWallCalibrationPoint(
            sensorId,
            wall,
            actualDistanceM,
            capture.input,
            capture.haveDifferentialLightBand ? &capture.differentialLightBand : nullptr);
    }

    template <size_t MaxSamples>
    bool TryStoreFrontCalibrationSpinSensorBands(
        WallSensorId sensorId,
        const std::array<float, MaxSamples>& openSamples,
        uint16_t openSampleCount,
        const std::array<float, MaxSamples>& wallSamples,
        const std::array<float, MaxSamples>& wallDistanceSamples,
        uint16_t wallSampleCount)
    {
        constexpr float kCollapsedFrontSweepSignalEpsilon = 1.0e-4f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        float onTargetDistanceM = 0.0f;
        float offTargetDistanceM = 0.0f;
        if (!TryComputeFrontWallObservationThresholdDistancesM(
                _speedVehicle,
                sensorId,
                Config::kFrontWallReleaseHysteresisM,
                onTargetDistanceM,
                offTargetDistanceM))
        {
            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_observation_geometry_unavailable,sensor=%s",
                WallSensorIdName(sensorId));
            AppendStartupTrace(traceLine);
            return false;
        }

        const auto logOpenPool = [&](const RobustSignalBand* openBand) noexcept
        {
            float minSignal = INFINITY;
            float maxSignal = 0.0f;
            uint16_t validCount = 0U;
            uint16_t collapsedCount = 0U;
            for (uint16_t index = 0U; index < openSampleCount; ++index)
            {
                const float signal = openSamples[index];
                if (!std::isfinite(signal) || signal < 0.0f)
                {
                    continue;
                }

                minSignal = (signal < minSignal) ? signal : minSignal;
                maxSignal = (signal > maxSignal) ? signal : maxSignal;
                if (signal <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++collapsedCount;
                }
                ++validCount;
            }

            char traceLine[320] = {};
            if (openBand != nullptr)
            {
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_front_sweep_open_pool,sensor=%s,count=%u,valid=%u,collapsed=%u,min=%.6f,max=%.6f,median=%.6f,low=%.6f,high=%.6f",
                    WallSensorIdName(sensorId),
                    static_cast<unsigned>(openSampleCount),
                    static_cast<unsigned>(validCount),
                    static_cast<unsigned>(collapsedCount),
                    std::isfinite(minSignal) ? minSignal : 0.0f,
                    maxSignal,
                    openBand->median,
                    openBand->low,
                    openBand->high);
            }
            else
            {
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_front_sweep_open_pool,sensor=%s,count=%u,valid=%u,collapsed=%u,min=%.6f,max=%.6f",
                    WallSensorIdName(sensorId),
                    static_cast<unsigned>(openSampleCount),
                    static_cast<unsigned>(validCount),
                    static_cast<unsigned>(collapsedCount),
                    std::isfinite(minSignal) ? minSignal : 0.0f,
                    maxSignal);
            }
            AppendStartupTrace(traceLine);
        };

        const auto logWallPoolAndProbe = [&](const char* label, float targetDistanceM) noexcept
        {
            float minDistanceM = INFINITY;
            float maxDistanceM = 0.0f;
            float minSignal = INFINITY;
            float maxSignal = 0.0f;
            uint16_t validCount = 0U;
            uint16_t withinTargetCount = 0U;
            uint16_t collapsedCount = 0U;
            uint16_t withinTargetCollapsedCount = 0U;
            float selectedSignals[MaxSamples] = {};
            float selectedDistances[MaxSamples] = {};
            float selectedErrors[MaxSamples] = {};
            for (uint16_t index = 0U; index < wallSampleCount; ++index)
            {
                const float signal = wallSamples[index];
                const float distanceM = wallDistanceSamples[index];
                if (!std::isfinite(signal) ||
                    signal < 0.0f ||
                    !std::isfinite(distanceM) ||
                    distanceM <= 0.0f)
                {
                    continue;
                }

                minDistanceM = (distanceM < minDistanceM) ? distanceM : minDistanceM;
                maxDistanceM = (distanceM > maxDistanceM) ? distanceM : maxDistanceM;
                minSignal = (signal < minSignal) ? signal : minSignal;
                maxSignal = (signal > maxSignal) ? signal : maxSignal;
                if (signal <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++collapsedCount;
                }
                const float errorM = std::fabs(distanceM - targetDistanceM);
                if (errorM <= Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM)
                {
                    ++withinTargetCount;
                    if (signal <= kCollapsedFrontSweepSignalEpsilon)
                    {
                        ++withinTargetCollapsedCount;
                    }
                }

                selectedSignals[validCount] = signal;
                selectedDistances[validCount] = distanceM;
                selectedErrors[validCount] = errorM;
                ++validCount;
            }

            for (uint16_t index = 1U; index < validCount; ++index)
            {
                const float signal = selectedSignals[index];
                const float distanceM = selectedDistances[index];
                const float errorM = selectedErrors[index];
                uint16_t insertIndex = index;
                while ((insertIndex > 0U) && (selectedErrors[insertIndex - 1U] > errorM))
                {
                    selectedSignals[insertIndex] = selectedSignals[insertIndex - 1U];
                    selectedDistances[insertIndex] = selectedDistances[insertIndex - 1U];
                    selectedErrors[insertIndex] = selectedErrors[insertIndex - 1U];
                    --insertIndex;
                }

                selectedSignals[insertIndex] = signal;
                selectedDistances[insertIndex] = distanceM;
                selectedErrors[insertIndex] = errorM;
            }

            const uint16_t retainedCount =
                (validCount < Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount) ?
                validCount :
                Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount;
            float retainedMinDistanceM = INFINITY;
            float retainedMaxDistanceM = 0.0f;
            float retainedMinSignal = INFINITY;
            float retainedMaxSignal = 0.0f;
            uint16_t retainedCollapsedCount = 0U;
            for (uint16_t index = 0U; index < retainedCount; ++index)
            {
                retainedMinDistanceM = (selectedDistances[index] < retainedMinDistanceM) ? selectedDistances[index] : retainedMinDistanceM;
                retainedMaxDistanceM = (selectedDistances[index] > retainedMaxDistanceM) ? selectedDistances[index] : retainedMaxDistanceM;
                retainedMinSignal = (selectedSignals[index] < retainedMinSignal) ? selectedSignals[index] : retainedMinSignal;
                retainedMaxSignal = (selectedSignals[index] > retainedMaxSignal) ? selectedSignals[index] : retainedMaxSignal;
                if (selectedSignals[index] <= kCollapsedFrontSweepSignalEpsilon)
                {
                    ++retainedCollapsedCount;
                }
            }

            char traceLine[384] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_match_probe,sensor=%s,label=%s,target_m=%.4f,valid=%u,collapsed=%u,within=%u,within_collapsed=%u,retain=%u,retain_collapsed=%u,nearest_err=%.4f,last_err=%.4f,dist_min=%.4f,dist_max=%.4f,sel_dist_min=%.4f,sel_dist_max=%.4f,sel_sig_min=%.6f,sel_sig_max=%.6f",
                WallSensorIdName(sensorId),
                (label != nullptr) ? label : "unknown",
                targetDistanceM,
                static_cast<unsigned>(validCount),
                static_cast<unsigned>(collapsedCount),
                static_cast<unsigned>(withinTargetCount),
                static_cast<unsigned>(withinTargetCollapsedCount),
                static_cast<unsigned>(retainedCount),
                static_cast<unsigned>(retainedCollapsedCount),
                (validCount > 0U) ? selectedErrors[0U] : 0.0f,
                (retainedCount > 0U) ? selectedErrors[retainedCount - 1U] : 0.0f,
                std::isfinite(minDistanceM) ? minDistanceM : 0.0f,
                maxDistanceM,
                std::isfinite(retainedMinDistanceM) ? retainedMinDistanceM : 0.0f,
                retainedMaxDistanceM,
                std::isfinite(retainedMinSignal) ? retainedMinSignal : 0.0f,
                retainedMaxSignal);
            AppendStartupTrace(traceLine);

            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_wall_pool,sensor=%s,label=%s,count=%u,collapsed=%u,dist_min=%.4f,dist_max=%.4f,sig_min=%.6f,sig_max=%.6f",
                WallSensorIdName(sensorId),
                (label != nullptr) ? label : "unknown",
                static_cast<unsigned>(wallSampleCount),
                static_cast<unsigned>(collapsedCount),
                std::isfinite(minDistanceM) ? minDistanceM : 0.0f,
                maxDistanceM,
                std::isfinite(minSignal) ? minSignal : 0.0f,
                maxSignal);
            AppendStartupTrace(traceLine);
        };

        RobustSignalBand openBand{};
        const bool haveOpenBand = MazeMap::TryComputeRobustSignalBandFromSamples(
            openSamples,
            openSampleCount,
            Config::kWallCalibrationScaledMadMultiplier,
            openBand.median,
            openBand.low,
            openBand.high);
        logOpenPool(haveOpenBand ? &openBand : nullptr);
        logWallPoolAndProbe("on", onTargetDistanceM);
        RobustSignalBand wallBand{};
        const bool haveWallBand = MazeMap::TryComputeRobustSignalBandFromSamples(
            wallSamples,
            wallSampleCount,
            Config::kWallCalibrationScaledMadMultiplier,
            wallBand.median,
            wallBand.low,
            wallBand.high);
        RobustSignalBand onDistanceBand{};
        const bool haveOnDistanceBand = MazeMap::TryComputeRobustDistanceMatchedSignalBandFromSamples(
            wallSamples,
            wallDistanceSamples,
            wallSampleCount,
            onTargetDistanceM,
            Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMinSamples,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM,
            Config::kWallCalibrationScaledMadMultiplier,
            onDistanceBand.median,
            onDistanceBand.low,
            onDistanceBand.high);
        logWallPoolAndProbe("off", offTargetDistanceM);
        RobustSignalBand offDistanceBand{};
        const bool haveOffDistanceBand = MazeMap::TryComputeRobustDistanceMatchedSignalBandFromSamples(
            wallSamples,
            wallDistanceSamples,
            wallSampleCount,
            offTargetDistanceM,
            Config::kStartupWallCalibrationFrontSweepMatchedWallSampleCount,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMinSamples,
            Config::kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM,
            Config::kWallCalibrationScaledMadMultiplier,
            offDistanceBand.median,
            offDistanceBand.low,
            offDistanceBand.high);
        if (!haveOpenBand ||
            !haveOnDistanceBand ||
            !haveOffDistanceBand ||
            !std::isfinite(openBand.median) ||
            !std::isfinite(openBand.low) ||
            !std::isfinite(openBand.high) ||
            !std::isfinite(onDistanceBand.median) ||
            !std::isfinite(onDistanceBand.low) ||
            !std::isfinite(onDistanceBand.high) ||
            !std::isfinite(offDistanceBand.median) ||
            !std::isfinite(offDistanceBand.low) ||
            !std::isfinite(offDistanceBand.high) ||
            openBand.median < 0.0f ||
            openBand.low < 0.0f ||
            openBand.high < openBand.low ||
            onDistanceBand.median <= 0.0f ||
            onDistanceBand.low <= 0.0f ||
            onDistanceBand.high < onDistanceBand.low ||
            offDistanceBand.median <= 0.0f ||
            offDistanceBand.low <= 0.0f ||
            offDistanceBand.high < offDistanceBand.low)
        {
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_distance_match_unavailable,sensor=%s,on_m=%.4f,off_m=%.4f,open=%u,on=%u,off=%u",
                WallSensorIdName(sensorId),
                onTargetDistanceM,
                offTargetDistanceM,
                haveOpenBand ? 1U : 0U,
                haveOnDistanceBand ? 1U : 0U,
                haveOffDistanceBand ? 1U : 0U);
            AppendStartupTrace(traceLine);

            if (haveOpenBand &&
                haveWallBand &&
                std::isfinite(openBand.median) &&
                std::isfinite(openBand.low) &&
                std::isfinite(openBand.high) &&
                std::isfinite(wallBand.median) &&
                std::isfinite(wallBand.low) &&
                std::isfinite(wallBand.high) &&
                openBand.median >= 0.0f &&
                openBand.low >= 0.0f &&
                openBand.high >= openBand.low &&
                wallBand.median > 0.0f &&
                wallBand.low > 0.0f &&
                wallBand.high >= wallBand.low)
            {
                float wallBucketBaseline = 0.0f;
                float wallBucketOnRiseThreshold = 0.0f;
                float wallBucketOffRiseThreshold = 0.0f;
                if (MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
                        openBand.low,
                        openBand.high,
                        wallBand.low,
                        wallBand.high,
                        Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
                        Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold,
                        wallBucketBaseline))
                {
                    gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(sensorId, openBand.median);
                    gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(sensorId, openBand.low, openBand.high);
                    gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
                        sensorId,
                        wallBand.median,
                        wallBand.low,
                        wallBand.high);
                    gWallDistanceCalibration.SetFrontDirectRiseThresholds(
                        sensorId,
                        wallBucketBaseline,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold);
                    snprintf(
                        traceLine,
                        sizeof(traceLine),
                        "startup_front_sweep_wall_bucket_match,sensor=%s,baseline=%.6f,wall_low=%.6f,wall_high=%.6f,on_rise=%.6f,off_rise=%.6f",
                        WallSensorIdName(sensorId),
                        wallBucketBaseline,
                        wallBand.low,
                        wallBand.high,
                        wallBucketOnRiseThreshold,
                        wallBucketOffRiseThreshold);
                    AppendStartupTrace(traceLine);
                    return true;
                }
            }

            return false;
        }

        float signalBaseline = openBand.high;
        float onSignal = onDistanceBand.low;
        float offSignal = offDistanceBand.high;
        if (!(std::isfinite(signalBaseline) &&
            std::isfinite(onSignal) &&
            std::isfinite(offSignal) &&
            signalBaseline >= 0.0f &&
            onSignal > signalBaseline &&
            offSignal > signalBaseline &&
            offSignal < onSignal))
        {
            signalBaseline = openBand.median;
            onSignal = onDistanceBand.median;
            offSignal = offDistanceBand.median;
            if (!(std::isfinite(signalBaseline) &&
                std::isfinite(onSignal) &&
                std::isfinite(offSignal) &&
                signalBaseline >= 0.0f &&
                onSignal > signalBaseline &&
                offSignal > signalBaseline &&
                offSignal < onSignal))
            {
                return false;
            }

            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_sweep_distance_match_using_medians,sensor=%s,open_hi=%.6f,on_lo=%.6f,off_hi=%.6f",
                WallSensorIdName(sensorId),
                openBand.high,
                onDistanceBand.low,
                offDistanceBand.high);
            AppendStartupTrace(traceLine);
        }

        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(sensorId, openBand.median);
        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(sensorId, openBand.low, openBand.high);
        gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
            sensorId,
            onDistanceBand.median,
            onDistanceBand.low,
            onDistanceBand.high);
        gWallDistanceCalibration.SetFrontDirectRiseThresholds(
            sensorId,
            signalBaseline,
            onSignal - signalBaseline,
            offSignal - signalBaseline);
        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_distance_match,sensor=%s,on_m=%.4f,off_m=%.4f,baseline=%.6f,on_signal=%.6f,off_signal=%.6f",
            WallSensorIdName(sensorId),
            onTargetDistanceM,
            offTargetDistanceM,
            signalBaseline,
            onSignal,
            offSignal);
        AppendStartupTrace(traceLine);
        return true;
    }

    LoopController::ControlVector FrontCalibrationSweepLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& sweep = *static_cast<FrontCalibrationSweepLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float deltaYawRad = WrapAngleRad(pose.yawRad - sweep.previousYawRad);
        sweep.previousYawRad = pose.yawRad;
        sweep.accumulatedSweepAngleRad += (std::max)(0.0f, deltaYawRad);
        sweep.accumulatedSweepAngleRad = (std::min)(sweep.accumulatedSweepAngleRad, sweep.targetSweepAngleRad);

        if ((sweep.accumulatedSweepAngleRad - sweep.lastStoredSweepAngleRad) >= sweep.captureStepRad)
        {
            RawWallSensorSample frontLeftSample{};
            RawWallSensorSample frontRightSample{};
            float frontLeftWallDistanceM = NAN;
            float frontRightWallDistanceM = NAN;
            SampleWallSensorPairRaw(
                WallSensorId::FrontLeft,
                _speedVehicle.FrontLeft,
                WallSensorId::FrontRight,
                _speedVehicle.FrontRight,
                frontLeftSample,
                frontRightSample);
            (void)TryComputeNearestStartCellWallDistanceM(pose, _speedVehicle.FrontLeft, frontLeftWallDistanceM);
            (void)TryComputeNearestStartCellWallDistanceM(pose, _speedVehicle.FrontRight, frontRightWallDistanceM);
            sweep.sweepSamples.Push(
                MazeMap::ClassifyFrontCalibrationSpinHeadingFromNorth(
                    pose.yawRad,
                    Config::kStartupWallCalibrationFrontNorthOpenHalfWidthRad,
                    Config::kStartupWallCalibrationFrontWallMinEastOfNorthRad,
                    Config::kStartupWallCalibrationFrontWallMaxEastOfNorthRad),
                frontLeftSample.differentialLight,
                frontRightSample.differentialLight,
                frontLeftWallDistanceM,
                frontRightWallDistanceM);
            sweep.lastStoredSweepAngleRad = sweep.accumulatedSweepAngleRad;
        }

        const float remainingRad = sweep.targetSweepAngleRad - sweep.accumulatedSweepAngleRad;
        if (MazeMap::IsInPlaceTurnComplete(remainingRad, pose.angularSpeedRadps, sweep.turnProfile))
        {
            sweep.settleHold.durationMs = Config::kStartupWallCalibrationSettleMs;
            sweep.settleHold.stationary = true;
                TransitionLoopPhase(&sweep.settleHold, &Implementation::HoldLoopTick, services);
            return LoopController::ControlVector::Brake;
        }
        if (!sweep.durationLogged && static_cast<long>(sweep.expectedCompletionDeadlineMs - millis()) <= 0)
        {
            sweep.durationLogged = true;
            AppendStartupTrace("startup_wall_calibration:front_sweep_elapsed_budget_exceeded");
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                remainingRad,
                pose.angularSpeedRadps,
                sweep.turnProfile,
                angularCommandRadps))
        {
            return FaultLoopPhase(services, "Startup front calibration sweep profile became invalid");
        }

        return _drive.PointControlVector(
            0.0f,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    bool CaptureAndStoreFrontCalibrationSweep(const MotionLimits& limits, bool& storedBands)
    {
        storedBands = false;
        FrontCalibrationSweepLoopState sweep{};
        sweep.limits = limits;
        sweep.turnProfile = BuildSharedInPlaceTurnProfile(limits);
        sweep.targetSweepAngleRad =
            static_cast<float>(Config::kStartupWallCalibrationFrontSpinTurnCount) * TWO_PI_F;
        sweep.captureStepRad = Config::kStartupWallCalibrationFrontSpinCaptureStepRad;
        sweep.lastStoredSweepAngleRad = -sweep.captureStepRad;
        sweep.previousYawRad = _drive.GetPose().yawRad;
        sweep.expectedCompletionDeadlineMs =
            millis() +
            static_cast<unsigned long>(
                2500.0f +
                (1000.0f * sweep.targetSweepAngleRad / (std::max)(0.25f, limits.maxAngularSpeedRadps)));
        sweep.storedBands = &storedBands;

        if (!RunLoopSession(&sweep, &Implementation::FrontCalibrationSweepLoopTick))
        {
            return false;
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_samples,fl_open=%u,fl_wall=%u,fr_open=%u,fr_wall=%u",
            static_cast<unsigned>(sweep.sweepSamples.frontLeftOpenCount),
            static_cast<unsigned>(sweep.sweepSamples.frontLeftWallCount),
            static_cast<unsigned>(sweep.sweepSamples.frontRightOpenCount),
            static_cast<unsigned>(sweep.sweepSamples.frontRightWallCount));
        AppendStartupTrace(traceLine);

        const bool storedFrontLeftBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontLeft,
            sweep.sweepSamples.frontLeftOpenSamples,
            sweep.sweepSamples.frontLeftOpenCount,
            sweep.sweepSamples.frontLeftWallSamples,
            sweep.sweepSamples.frontLeftWallDistanceSamples,
            sweep.sweepSamples.frontLeftWallCount);
        const bool storedFrontRightBands = TryStoreFrontCalibrationSpinSensorBands(
            WallSensorId::FrontRight,
            sweep.sweepSamples.frontRightOpenSamples,
            sweep.sweepSamples.frontRightOpenCount,
            sweep.sweepSamples.frontRightWallSamples,
            sweep.sweepSamples.frontRightWallDistanceSamples,
            sweep.sweepSamples.frontRightWallCount);

        snprintf(
            traceLine,
            sizeof(traceLine),
            "startup_front_sweep_bands,fl=%u,fr=%u",
            storedFrontLeftBands ? 1U : 0U,
            storedFrontRightBands ? 1U : 0U);
        AppendStartupTrace(traceLine);
        storedBands = storedFrontLeftBands && storedFrontRightBands;
        return true;
    }

    bool CaptureAndStoreFrontOpenBaselineAtCurrentPose()
    {
        WallSensorCalibrationCapture frontLeftCapture{};
        WallSensorCalibrationCapture frontRightCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::FrontLeft,
            _speedVehicle.FrontLeft,
            WallSensorId::FrontRight,
            _speedVehicle.FrontRight,
            frontLeftCapture,
            frontRightCapture);

        const auto storeBaseline = [&](WallSensorId sensorId, const WallSensorCalibrationCapture& capture) -> bool
        {
            if (!(std::isfinite(capture.input.differentialLight) &&
                capture.input.differentialLight >= 0.0f &&
                capture.haveDifferentialLightBand &&
                std::isfinite(capture.differentialLightBand.low) &&
                std::isfinite(capture.differentialLightBand.high) &&
                capture.differentialLightBand.low >= 0.0f &&
                capture.differentialLightBand.high >= capture.differentialLightBand.low))
            {
                return false;
            }

            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(
                sensorId,
                capture.input.differentialLight);
            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(
                sensorId,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_front_open_baseline,sensor=%s,median=%.6f,low=%.6f,high=%.6f",
                WallSensorIdName(sensorId),
                capture.input.differentialLight,
                capture.differentialLightBand.low,
                capture.differentialLightBand.high);
            AppendStartupTrace(traceLine);
            return true;
        };

        return
            storeBaseline(WallSensorId::FrontLeft, frontLeftCapture) &&
            storeBaseline(WallSensorId::FrontRight, frontRightCapture);
    }

    // WARNING: Keep this procedure aligned with the validated hardware sequence unless it is re-proven on the robot.
    // The startup wall calibration must remain:
    // 1. start north with the rear touching the south wall
    // 2. complete the stationary settle
    // 3. drive forward to the start-cell center
    // 4. face west and calibrate the left side sensor against the south-wall-referenced distance
    // 5. face east and calibrate the right side sensor against the south-wall-referenced distance
    // 6. reseat to the start-cell center facing north and capture the per-run front open-scene baseline used to
    //    adapt the persisted dark-room characterization to the current ambient
    // 7. stay in the start-cell center facing north and exit
    bool RunStartupWallCalibration()
    {
        if (!EmitMissionControllerLineOrFail("Startup wall calibration"))
        {
            return false;
        }
        gWallDistanceCalibration.Clear();
        _hasWallTouchStandoffEstimate = false;
        SeedStartupWallCalibrationPoseFromSouthWall();
        if (!WaitForMissionStartupStationaryHold())
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:begin");

        const float targetCenterXM = 0.5f * Config::kCellSizeM;
        const float targetCenterYM = 0.5f * Config::kCellSizeM;
        char geometryTraceLine[256] = {};
        snprintf(
            geometryTraceLine,
            sizeof(geometryTraceLine),
            "startup_cal_geometry:cell_pitch=%.4f,cell_clear=%.4f,center_x=%.4f,center_y=%.4f",
            Config::kCellSizeM,
            Config::kCellClearSpanM,
            targetCenterXM,
            targetCenterYM);
        AppendStartupTrace(geometryTraceLine);

        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        AppendStartupTrace("startup_wall_calibration:settle_at_south_wall_start");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:move_forward_to_center");
        if (!DriveCalibrationPoseToKnownY(targetCenterYM, centeringLimits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_after_forward_move");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        float actualDistanceM = 0.0f;
        float sideDistanceSumM = 0.0f;
        uint8_t sideDistanceCount = 0U;

        AppendStartupTrace("startup_wall_calibration:rotate_west_for_left_side_sample");
        if (!RotateCalibrationTo(MazeMap::Left, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_left_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_left_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideLeft, _speedVehicle.SideLeft, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture rightSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideRight, _speedVehicle.SideRight);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            rightSideBaselineCapture.input.differentialLight);
        if (rightSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideRight,
                rightSideBaselineCapture.differentialLightBand.low,
                rightSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:rotate_east_for_right_side_sample");
        if (!RotateCalibrationTo(MazeMap::Right, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_right_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_right_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideRight, _speedVehicle.SideRight, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture leftSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideLeft, _speedVehicle.SideLeft);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            leftSideBaselineCapture.input.differentialLight);
        if (leftSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideLeft,
                leftSideBaselineCapture.differentialLightBand.low,
                leftSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:reseat_before_front_baseline");
        if (!ReseatMissionStartPoseWithPhasePrefix("startup_front_baseline", Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_front_open_baseline");
        if (!CaptureAndStoreFrontOpenBaselineAtCurrentPose())
        {
            return false;
        }

        if (sideDistanceCount > 0U)
        {
            gWallDistanceCalibration.SetExpectedSideWallDistanceM(sideDistanceSumM / static_cast<float>(sideDistanceCount));
        }

        AppendStartupTrace("startup_wall_calibration:complete");
        AppendStartupCalibrationStateTrace("startup_complete");
        SnapToStartPose();
        return HoldPosition(Config::kStartupWallCalibrationSettleMs);
    }

    bool Initialize(const char* banner, bool observeCurrentCellAfterInit)
    {
        if (!_runtime.RegisterModeFaultHandler(&Implementation::HandleRuntimeFault, this, _activeModeFaultSource))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace(_maneuverTestMode ? "mode:maneuver_file_test" : "mode:mission");
        if (!OpenMissionTextLog())
        {
            AppendStartupTrace("initialize:logging_txt_open_failed");
            DisableMissionTextLogging("initialize:mission_text_log_unavailable");
        }
        if (!EmitMissionControllerLine(banner))
        {
            AppendStartupTrace("initialize:banner_log_failed");
            return false;
        }
        AppendStartupTrace("initialize:setup_hardware_ok");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        AppendStartupTrace("initialize:drive_ok");
        if (!_sensors.Begin())
        {
            return Fail("Sensor init failed");
        }
        AppendStartupTrace("initialize:sensors_ok");

        if (!RunStartupWallCalibration())
        {
            return false;
        }
        AppendStartupTrace("initialize:startup_wall_calibration_ok");
        LoadPersistedFrontWallCharacterization();

        if (observeCurrentCellAfterInit && !ObserveCurrentCell())
        {
            return false;
        }
        if (observeCurrentCellAfterInit)
        {
            AppendStartupTrace("initialize:observed_current_cell");
        }

        return true;
    }

    bool BeginTelemetryPhase(const char* name)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }
        ++_telemetryPhaseId;
        if (_runtime.WriteTextLogPhase(_telemetryPhaseId, micros(), name))
        {
            return true;
        }
        return Fail("Failed to write maneuver test phase marker");
    }

    bool LogWallCalibrationMetadata()
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char line[128] = {};
        snprintf(
            line,
            sizeof(line),
            "calibration_average_samples,%u",
            static_cast<unsigned>(Config::kWallCalibrationAverageSampleCount));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "detection_window_cycles,%u",
            static_cast<unsigned>(Config::kWallDetectionAverageWindowCycles));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_turn_count,%u",
            static_cast<unsigned>(Config::kStartupWallCalibrationFrontSpinTurnCount));
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_capture_step_deg,%.1f",
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontSpinCaptureStepRad);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_sweep_heading_bands_deg,%.1f,%.1f,%.1f",
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontNorthOpenHalfWidthRad,
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontWallMinEastOfNorthRad,
            RAD_TO_DEG_F * Config::kStartupWallCalibrationFrontWallMaxEastOfNorthRad);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "expected_side_distance_m,%.6f",
            gWallDistanceCalibration.GetExpectedSideWallDistanceM());
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "configured_touch_standoff_m,%.6f",
            Config::kWallTouchContactStandoffM);
        if (!WriteTelemetryEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        if (_hasWallTouchStandoffEstimate)
        {
            snprintf(
                line,
                sizeof(line),
                "estimated_touch_standoff_m,%.6f",
                _lastWallTouchStandoffEstimateM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        if (gWallDistanceCalibration.TryComputeSideWallDistanceThresholds(
                Config::kSideWallDistanceLatchFractionOfCalibration,
                Config::kSideWallDistanceReleaseFractionOfCalibration,
                sideWallOnThresholdM,
                sideWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_thresholds_m,%.6f,%.6f",
                sideWallOnThresholdM,
                sideWallOffThresholdM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLight = 0.0f;
        float sideRightReferenceDifferentialLight = 0.0f;
        const bool haveSideLeftReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLight);
        const bool haveSideRightReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLight);
        if (haveSideLeftReferenceDifferentialLight || haveSideRightReferenceDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff,%.6f,%.6f",
                sideLeftReferenceDifferentialLight,
                sideRightReferenceDifferentialLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLightLow = 0.0f;
        float sideLeftReferenceDifferentialLightHigh = 0.0f;
        float sideRightReferenceDifferentialLightLow = 0.0f;
        float sideRightReferenceDifferentialLightHigh = 0.0f;
        const bool haveSideLeftReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLightLow,
            sideLeftReferenceDifferentialLightHigh);
        const bool haveSideRightReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLightLow,
            sideRightReferenceDifferentialLightHigh);
        if (haveSideLeftReferenceDifferentialLightBand || haveSideRightReferenceDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftReferenceDifferentialLightLow,
                sideLeftReferenceDifferentialLightHigh,
                sideRightReferenceDifferentialLightLow,
                sideRightReferenceDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDistanceM = 0.0f;
        float sideRightReferenceDistanceM = 0.0f;
        const bool haveSideLeftReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideLeft,
            sideLeftReferenceDistanceM);
        const bool haveSideRightReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideRight,
            sideRightReferenceDistanceM);
        if (haveSideLeftReferenceDistanceM || haveSideRightReferenceDistanceM)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_distance_m,%.6f,%.6f",
                sideLeftReferenceDistanceM,
                sideRightReferenceDistanceM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLight = 0.0f;
        float sideRightBaselineDifferentialLight = 0.0f;
        const bool haveSideLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLight);
        const bool haveSideRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLight);
        if (haveSideLeftBaselineDifferentialLight || haveSideRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff,%.6f,%.6f",
                sideLeftBaselineDifferentialLight,
                sideRightBaselineDifferentialLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLightLow = 0.0f;
        float sideLeftBaselineDifferentialLightHigh = 0.0f;
        float sideRightBaselineDifferentialLightLow = 0.0f;
        float sideRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveSideLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLightLow,
            sideLeftBaselineDifferentialLightHigh);
        const bool haveSideRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLightLow,
            sideRightBaselineDifferentialLightHigh);
        if (haveSideLeftBaselineDifferentialLightBand || haveSideRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftBaselineDifferentialLightLow,
                sideLeftBaselineDifferentialLightHigh,
                sideRightBaselineDifferentialLightLow,
                sideRightBaselineDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftOnMeasuredThreshold = 0.0f;
        float sideLeftOffMeasuredThreshold = 0.0f;
        float sideRightOnMeasuredThreshold = 0.0f;
        float sideRightOffMeasuredThreshold = 0.0f;
        float sideLeftSignalBaseline = 0.0f;
        float sideRightSignalBaseline = 0.0f;
        const bool haveSideLeftMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideLeft,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideLeftOnMeasuredThreshold,
            sideLeftOffMeasuredThreshold,
            sideLeftSignalBaseline);
        const bool haveSideRightMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideRight,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideRightOnMeasuredThreshold,
            sideRightOffMeasuredThreshold,
            sideRightSignalBaseline);
        if (haveSideLeftMeasuredThreshold || haveSideRightMeasuredThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                sideLeftOnMeasuredThreshold,
                sideLeftOffMeasuredThreshold,
                sideRightOnMeasuredThreshold,
                sideRightOffMeasuredThreshold);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontWallOnThresholdM = 0.0f;
        float frontWallOffThresholdM = 0.0f;
        if (gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
                _speedVehicle,
                Config::kFrontWallReleaseHysteresisM,
                frontWallOnThresholdM,
                frontWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_thresholds_m,%.6f,%.6f",
                frontWallOnThresholdM,
                frontWallOffThresholdM);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftOnMeasuredThreshold = 0.0f;
        float frontLeftOffMeasuredThreshold = 0.0f;
        float frontLeftSignalBaseline = 0.0f;
        float frontRightOnMeasuredThreshold = 0.0f;
        float frontRightOffMeasuredThreshold = 0.0f;
        float frontRightSignalBaseline = 0.0f;
        float frontLeftReferenceAmbientLight = 0.0f;
        float frontRightReferenceAmbientLight = 0.0f;
        const bool haveFrontLeftAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontLeft,
            frontLeftReferenceAmbientLight);
        const bool haveFrontRightAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontRight,
            frontRightReferenceAmbientLight);
        const bool haveFrontLeftThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontLeft,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontLeftAmbient ? frontLeftReferenceAmbientLight : NAN,
            frontLeftOnMeasuredThreshold,
            frontLeftOffMeasuredThreshold,
            frontLeftSignalBaseline);
        const bool haveFrontRightThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontRight,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontRightAmbient ? frontRightReferenceAmbientLight : NAN,
            frontRightOnMeasuredThreshold,
            frontRightOffMeasuredThreshold,
            frontRightSignalBaseline);
        if (haveFrontLeftAmbient || haveFrontRightAmbient)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_reference_ambient,%.6f,%.6f",
                frontLeftReferenceAmbientLight,
                frontRightReferenceAmbientLight);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        if (haveFrontLeftThreshold || haveFrontRightThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                frontLeftOnMeasuredThreshold,
                frontLeftOffMeasuredThreshold,
                frontRightOnMeasuredThreshold,
                frontRightOffMeasuredThreshold);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        const bool haveFrontLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontLeft,
            frontLeftSignalBaseline);
        const bool haveFrontRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontRight,
            frontRightSignalBaseline);
        if (haveFrontLeftBaselineDifferentialLight || haveFrontRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff,%.6f,%.6f",
                frontLeftSignalBaseline,
                frontRightSignalBaseline);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftBaselineDifferentialLightLow = 0.0f;
        float frontLeftBaselineDifferentialLightHigh = 0.0f;
        float frontRightBaselineDifferentialLightLow = 0.0f;
        float frontRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftBaselineDifferentialLightLow,
            frontLeftBaselineDifferentialLightHigh);
        const bool haveFrontRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightBaselineDifferentialLightLow,
            frontRightBaselineDifferentialLightHigh);
        if (haveFrontLeftBaselineDifferentialLightBand || haveFrontRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftBaselineDifferentialLightLow,
                frontLeftBaselineDifferentialLightHigh,
                frontRightBaselineDifferentialLightLow,
                frontRightBaselineDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftWeakestDifferentialLightLow = 0.0f;
        float frontLeftWeakestDifferentialLightHigh = 0.0f;
        float frontRightWeakestDifferentialLightLow = 0.0f;
        float frontRightWeakestDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftWeakestDifferentialLightLow,
            frontLeftWeakestDifferentialLightHigh);
        const bool haveFrontRightWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightWeakestDifferentialLightLow,
            frontRightWeakestDifferentialLightHigh);
        if (haveFrontLeftWeakestDifferentialLightBand || haveFrontRightWeakestDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_weakest_calibration_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftWeakestDifferentialLightLow,
                frontLeftWeakestDifferentialLightHigh,
                frontRightWeakestDifferentialLightLow,
                frontRightWeakestDifferentialLightHigh);
            if (!WriteTelemetryEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        for (uint8_t sensorIndex = 0U; sensorIndex < static_cast<uint8_t>(WallSensorId::Count); ++sensorIndex)
        {
            const WallSensorId sensorId = static_cast<WallSensorId>(sensorIndex);
            const MazeMap::WallSensorCalibrationCurve& curve = gWallDistanceCalibration.GetCurve(sensorId);
            snprintf(
                line,
                sizeof(line),
                "%s,measurement,%s,count,%u",
                WallSensorIdName(sensorId),
                WallSensorCalibrationMeasurementName(sensorId),
                static_cast<unsigned>(curve.GetCount()));
            if (!WriteTelemetryEvent("wall_calibration_curve", line))
            {
                return Fail("Unable to write wall calibration curve metadata");
            }

            for (uint8_t pointIndex = 0U; pointIndex < curve.GetCount(); ++pointIndex)
            {
                const MazeMap::WallSensorCalibrationCurve::Point& point = curve.GetPoint(pointIndex);
                snprintf(
                    line,
                    sizeof(line),
                    "%s,%s,%u,%.6f,%.6f",
                    WallSensorIdName(sensorId),
                    WallSensorCalibrationMeasurementName(sensorId),
                    static_cast<unsigned>(pointIndex),
                    point.measuredValue,
                    point.actualDistanceM);
                if (!WriteTelemetryEvent("wall_calibration_point", line))
                {
                    return Fail("Unable to write wall calibration point metadata");
                }
            }
        }

        return true;
    }

    bool LogTelemetrySample(bool stationary, const LoopController::ModeState& state)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }
        DiagnosticLogRow row{};
        MazeMap::App::Internal::Runtime::PopulateDiagnosticLogRow(
            row,
            _telemetrySampleCount,
            _telemetryPhaseId,
            stationary,
            state.tickStartUs,
            state.dtUs,
            state.estimate,
            _drive,
            state.driveTelemetry,
            state.sensors);
        if (_runtime.LogUtilityDataRow(row))
        {
            ++_telemetrySampleCount;
            return true;
        }
        return Fail("Failed to write maneuver test sample");
    }

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = Config::kControlPeriodUs;
        return options;
    }

    static LoopController::ControlVector ActiveLoopThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<Implementation*>(context);
        if ((self == nullptr) || (self->_activeLoopState == nullptr) || (self->_activeLoopTickFn == nullptr))
        {
            services.Fault("Mission loop callback dispatch was not initialized");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_activeLoopTickFn)(self->_activeLoopState, loopEndTimeUs, state, services);
    }

    static LoopController::PauseDisposition InterRunServicePauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<Implementation*>(context);
        if (self == nullptr)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Mission inter-run service pause callback context was null");
        }

        return self->OnInterRunServicePauseGranted(pause);
    }

    void TransitionLoopPhase(
        void* nextState,
        const ActiveLoopTickFn nextTickFn,
        LoopController::TickServices& services) noexcept
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &Implementation::ActiveLoopThunk;
        callbacks.context = this;
        _activeLoopState = nextState;
        _activeLoopTickFn = nextTickFn;
        services.SetNextModeWorkCallbacks(callbacks);
    }

    LoopController::ControlVector EndLoopPhase(LoopController::TickServices& services) noexcept
    {
        services.RequestEndLoop();
        return LoopController::ControlVector::Brake;
    }

    LoopController::ControlVector FaultLoopPhase(
        LoopController::TickServices& services,
        const char* reason) noexcept
    {
        services.Fault(reason);
        return LoopController::ControlVector::Brake;
    }

    bool RunLoopSession(void* initialState, const ActiveLoopTickFn initialTickFn)
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &Implementation::ActiveLoopThunk;
        callbacks.context = this;
        _activeLoopState = initialState;
        _activeLoopTickFn = initialTickFn;

        const bool began = _loopController.BeginSession(BuildLoopOptions(), callbacks);
        if (!began)
        {
            _activeLoopState = nullptr;
            _activeLoopTickFn = nullptr;
            return Fail("Mission loop controller session could not start");
        }

        const LoopController::SessionResult result = _loopController.Run();
        _loopController.EndSession();
        _activeLoopState = nullptr;
        _activeLoopTickFn = nullptr;

        if (_faulted)
        {
            return false;
        }
        if (result.status != LoopController::SessionResult::Status::Completed)
        {
            return false;
        }

        return true;
    }

    LoopController::PauseDisposition OnInterRunServicePauseGranted(
        const LoopController::PauseContext& pause)
    {
        (void)pause;

        if (!EmitMissionControllerLineOrFail("Install 34-35 jumper before lifting for tire service"))
        {
            return _faulted ?
                LoopController::PauseDisposition::Complete() :
                LoopController::PauseDisposition::StopByRuntime(
                    "Mission inter-run service install prompt failed");
        }

        _drive.Brake();
        while (!IsInterRunServiceJumperInstalled())
        {
            delay(Config::kInterRunServicePollMs);
        }

        if (!EmitMissionControllerLineOrFail("Service jumper detected; place robot back at start facing up and remove jumper"))
        {
            return _faulted ?
                LoopController::PauseDisposition::Complete() :
                LoopController::PauseDisposition::StopByRuntime(
                    "Mission inter-run service remove prompt failed");
        }

        while (IsInterRunServiceJumperInstalled())
        {
            _drive.Brake();
            delay(Config::kInterRunServicePollMs);
        }

        return LoopController::PauseDisposition::Complete();
    }

    LoopController::ControlVector InterRunServicePauseTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)rawState;
        (void)loopEndTimeUs;
        (void)state;

        LoopController::PauseRequest request{};
        request.onPauseGranted = &Implementation::InterRunServicePauseThunk;
        request.reason = "mission_inter_run_service";
        request.flushLogsBeforeGrant = true;
        services.RequestPause(request);
        return LoopController::ControlVector::Brake;
    }

    bool Fail(const char* message)
    {
        return _runtime.FailActiveMode(message);
    }

    void OnRuntimeFault(const char* message) noexcept
    {
        _faulted = true;
        AppendStartupCalibrationStateTrace("fault_state");
        char traceMessage[128] = {};
        snprintf(traceMessage, sizeof(traceMessage), "fault:%s", (message != nullptr) ? message : "unknown");
        AppendStartupTrace(traceMessage);
        (void)EmitMissionControllerFormatted("FAULT: %s", (message != nullptr) ? message : "unknown");
        if (_missionTextLoggingEnabled && !_missionMazeSnapshotWritten)
        {
            (void)WriteMissionMazeSnapshot("mission_fault");
        }
        if (_telemetryLoggingEnabled)
        {
            (void)WriteTelemetryEvent("fault", message);
            FlushTelemetryLog();
            CloseTelemetryLog();
            _telemetryLoggingEnabled = false;
        }
        FlushMissionTextLog();
    }

    LoopController::ControlVector HoldLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& hold = *static_cast<HoldLoopState*>(rawState);
        if (!hold.started)
        {
            hold.started = true;
            hold.startMs = millis();
        }

        if (!LogTelemetrySample(hold.stationary, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        if (static_cast<unsigned long>(millis() - hold.startMs) < hold.durationMs)
        {
            return LoopController::ControlVector::Brake;
        }

        if (hold.nextTickFn != nullptr)
        {
            TransitionLoopPhase(hold.nextState, hold.nextTickFn, services);
            return LoopController::ControlVector::Brake;
        }

        return EndLoopPhase(services);
    }

    bool IsDriveMotionSettled(
        const DriveTelemetry& stationaryReferenceTelemetry,
        unsigned long stationaryReferenceMs,
        const DriveTelemetry& telemetry,
        const SensorSnapshot& snapshot,
        unsigned long nowMs) const
    {
        const unsigned long elapsedMs = nowMs - stationaryReferenceMs;
        return MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            telemetry.leftDistanceM - stationaryReferenceTelemetry.leftDistanceM,
            telemetry.rightDistanceM - stationaryReferenceTelemetry.rightDistanceM,
            static_cast<float>(elapsedMs) * 1.0e-3f,
            snapshot.gyroRadps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    LoopController::ControlVector SettleLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& settle = *static_cast<SettleLoopState*>(rawState);
        if (!settle.started)
        {
            settle.started = true;
            settle.startMs = millis();
        }

        if (!LogTelemetrySample(settle.brakeCommand, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const unsigned long nowMs = millis();
        if (!settle.stationaryWindowActive)
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
            settle.stationaryWindowActive = true;
        }
        else if (!IsDriveMotionSettled(
            settle.stationaryStartTelemetry,
            settle.stationaryStartMs,
            state.driveTelemetry,
            state.sensors,
            nowMs))
        {
            settle.stationaryStartMs = nowMs;
            settle.stationaryStartTelemetry = state.driveTelemetry;
        }
        else if ((nowMs - settle.stationaryStartMs) >= settle.stationaryHoldMs)
        {
            if (settle.nextTickFn != nullptr)
            {
                TransitionLoopPhase(settle.nextState, settle.nextTickFn, services);
                return LoopController::ControlVector::Brake;
            }

            return EndLoopPhase(services);
        }

        if ((settle.timeoutMs > 0U) && ((nowMs - settle.startMs) >= settle.timeoutMs))
        {
            return FaultLoopPhase(
                services,
                (settle.timeoutMessage != nullptr) ? settle.timeoutMessage : "Drive settle timed out");
        }

        return settle.brakeCommand ?
            LoopController::ControlVector::Brake :
            _drive.PointControlVector(0.0f, 0.0f, kMissionDriveBaseTrackingCommandPd);
    }

    bool HoldPosition(uint16_t durationMs, const char* phaseName = nullptr)
    {
        if (phaseName != nullptr && !BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        HoldLoopState hold{};
        hold.durationMs = durationMs;
        hold.stationary = true;
        return RunLoopSession(&hold, &Implementation::HoldLoopTick);
    }

    bool HoldBrakedUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        SettleLoopState settle{};
        settle.timeoutMessage = timeoutMessage;
        settle.stationaryHoldMs = stationaryHoldMs;
        settle.timeoutMs = timeoutMs;
        settle.brakeCommand = true;
        return RunLoopSession(&settle, &Implementation::SettleLoopTick);
    }

    bool HoldZeroVelocityUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        SettleLoopState settle{};
        settle.timeoutMessage = timeoutMessage;
        settle.stationaryHoldMs = stationaryHoldMs;
        settle.timeoutMs = timeoutMs;
        settle.brakeCommand = false;
        return RunLoopSession(&settle, &Implementation::SettleLoopTick);
    }

    LoopController::ControlVector StartupStationaryHoldLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& hold = *static_cast<StartupStationaryHoldLoopState*>(rawState);
        if (!LogTelemetrySample(true, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const unsigned long nowMs = millis();
        if (!hold.stationaryWindowActive)
        {
            hold.stationaryStartMs = nowMs;
            hold.stationaryStartTelemetry = state.driveTelemetry;
            hold.stationaryWindowActive = true;
            return LoopController::ControlVector::Brake;
        }

        const bool stationary = MazeMap::IsMissionStartupStationaryFromEncoderWindow(
            state.driveTelemetry.leftDistanceM - hold.stationaryStartTelemetry.leftDistanceM,
            state.driveTelemetry.rightDistanceM - hold.stationaryStartTelemetry.rightDistanceM,
            static_cast<float>(nowMs - hold.stationaryStartMs) * 1.0e-3f,
            state.sensors.gyroRadps,
            Config::kMissionStartupStationarySpeedThresholdMps,
            Config::kMissionStartupStationaryMaxAbsYawRateRadps);
        if (!stationary)
        {
            if ((nowMs - hold.lastResetTraceMs) >= 1000UL)
            {
                char traceLine[160];
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_stationary_hold:reset,left_dm=%.5f,right_dm=%.5f,gyro=%.5f",
                    state.driveTelemetry.leftDistanceM - hold.stationaryStartTelemetry.leftDistanceM,
                    state.driveTelemetry.rightDistanceM - hold.stationaryStartTelemetry.rightDistanceM,
                    state.sensors.gyroRadps);
                AppendStartupTrace(traceLine);
                hold.lastResetTraceMs = nowMs;
            }
            hold.stationaryStartMs = nowMs;
            hold.stationaryStartTelemetry = state.driveTelemetry;
            return LoopController::ControlVector::Brake;
        }

        if ((nowMs - hold.stationaryStartMs) >= Config::kMissionStartupStationaryHoldMs)
        {
            AppendStartupTrace("startup_stationary_hold:complete");
            return EndLoopPhase(services);
        }

        return LoopController::ControlVector::Brake;
    }

    bool WaitForMissionStartupStationaryHold()
    {
        if (!EmitMissionControllerLineOrFail("Waiting for 2 s stationary start"))
        {
            return false;
        }
        AppendStartupTrace("startup_stationary_hold:waiting");

        StartupStationaryHoldLoopState hold{};
        return RunLoopSession(&hold, &Implementation::StartupStationaryHoldLoopTick);
    }

    LoopController::ControlVector ReverseStraightLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& reverse = *static_cast<ReverseStraightLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - reverse.startDistanceM);
        float remainingM = (std::max)(0.0f, reverse.distanceM - traveledM);
        if (reverse.targetPositionOverride != nullptr)
        {
            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    state.estimate.xMeters,
                    state.estimate.yMeters,
                    reverse.targetPositionOverride->x(),
                    reverse.targetPositionOverride->y(),
                    -reverse.targetHeading.x(),
                    -reverse.targetHeading.y(),
                    projectedRemainingM))
            {
                if (!reverse.projectionFallbackLogged)
                {
                    reverse.projectionFallbackLogged = true;
                    AppendStartupTrace("reverse_profile:projection_fallback_to_encoder_distance");
                }
            }
            else
            {
                remainingM = (std::max)(0.0f, projectedRemainingM);
            }
        }
        if (remainingM <= Config::kDistanceToleranceM)
        {
            reverse.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            reverse.completionSettle.timeoutMs = 0U;
            reverse.completionSettle.brakeCommand = true;
                TransitionLoopPhase(&reverse.completionSettle, &Implementation::SettleLoopTick, services);
            return LoopController::ControlVector::Brake;
        }

        const unsigned long nowMs = millis();
        if (reverse.translationWatchdog.Stalled(traveledM, reverse.commandedSpeedMps, remainingM, nowMs))
        {
            AppendStartupTrace("reverse_profile:encoder_progress_stalled_holding_position");
            reverse.fallbackHold.durationMs = Config::kMotionSettleHoldMs;
            reverse.fallbackHold.stationary = true;
                TransitionLoopPhase(&reverse.fallbackHold, &Implementation::HoldLoopTick, services);
            return LoopController::ControlVector::Brake;
        }
        if (static_cast<long>(reverse.timeoutMs - nowMs) <= 0)
        {
            AppendStartupTrace("reverse_profile:elapsed_budget_reached_holding_position");
            reverse.fallbackHold.durationMs = Config::kMotionSettleHoldMs;
            reverse.fallbackHold.stationary = true;
                TransitionLoopPhase(&reverse.fallbackHold, &Implementation::HoldLoopTick, services);
            return LoopController::ControlVector::Brake;
        }

        const float accelLimitedSpeedMps = (std::min)(
            reverse.limits.maxSpeedMps,
            reverse.commandedSpeedMps + (reverse.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(0.0f, remainingM, reverse.limits.decelMps2);
        reverse.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float headingErrorRad = HeadingErrorRad(reverse.targetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            Config::kStraightHeadingKp * headingErrorRad;
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -reverse.limits.maxAngularSpeedRadps,
            reverse.limits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            -reverse.commandedSpeedMps,
            angularCommandRadps,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteReverseStraightProfile(
        float distanceM,
        const MotionLimits& limits,
        const Eigen::Vector2f* targetHeadingOverride = nullptr,
        const Eigen::Vector2f* targetPositionOverride = nullptr)
    {
        if (!(std::isfinite(distanceM) && distanceM > 0.0f))
        {
            return true;
        }

        ReverseStraightLoopState reverse{};
        reverse.distanceM = distanceM;
        reverse.limits = limits;
        reverse.targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        reverse.targetPositionOverride = targetPositionOverride;
        reverse.startDistanceM = _drive.GetAverageDistanceMeters();
        reverse.timeoutMs =
            millis() +
            static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        reverse.translationWatchdog.Reset(0.0f, millis());
        return RunLoopSession(&reverse, &Implementation::ReverseStraightLoopTick);
    }

    bool LoadManeuverQueueFromSd(const char* fileName, MazeMap::ManeuverQueue& queue)
    {
#if defined(ARDUINO_TEENSY41)
        File file = SD.open(fileName, FILE_READ);
        if (!file)
        {
            AppendStartupTrace("maneuver_test:test_file_unavailable");
            (void)EmitMissionControllerLine("Maneuver file unavailable; skipping maneuver-file test");
            return false;
        }
        AppendStartupTrace("maneuver_test:test_txt_opened");

        MazeMap::ManeuverPath path;
        char line[128] = {};
        uint16_t lineNumber = 0U;
        while (file.available())
        {
            const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
            line[lineLength] = '\0';
            ++lineNumber;

            char* hashComment = strchr(line, '#');
            if (hashComment != nullptr)
            {
                *hashComment = '\0';
            }

            char* slashComment = strstr(line, "//");
            if (slashComment != nullptr)
            {
                *slashComment = '\0';
            }

            for (char* token = strtok(line, ", \t\r;"); token != nullptr; token = strtok(nullptr, ", \t\r;"))
            {
                MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                if (!TryParseManeuverCodeToken(token, code))
                {
                    char message[96] = {};
                    snprintf(message, sizeof(message), "Maneuver file token issue on line %u: %s", lineNumber, token);
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_parse_issue");
                    (void)EmitMissionControllerLine(message);
                    return false;
                }
                if (!path.push_back(code))
                {
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_path_capacity_reached");
                    (void)EmitMissionControllerLine("Maneuver file exceeded path capacity; skipping maneuver-file test");
                    return false;
                }
            }
        }

        file.close();

        if (path.GetSize() == 0)
        {
            AppendStartupTrace("maneuver_test:test_file_empty");
            (void)EmitMissionControllerLine("Maneuver file did not contain any maneuvers");
            return false;
        }
        AppendStartupTrace("maneuver_test:path_parsed");

        queue.clear();
        if (!queue.push_back(path, _currentDirectionalLocation))
        {
            AppendStartupTrace("maneuver_test:queue_build_issue");
            (void)EmitMissionControllerLine("Maneuver file could not be converted into a queue");
            return false;
        }
        AppendStartupTrace("maneuver_test:queue_built");

        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
        AppendStartupTrace("maneuver_test:speeds_computed");
        return LogLoadedManeuverQueue(queue);
#else
        (void)fileName;
        (void)queue;
        AppendStartupTrace("maneuver_test:teensy_target_required");
        (void)EmitMissionControllerLine("Maneuver-file test mode requires the Teensy target");
        return false;
#endif
    }

    bool LogLoadedManeuverQueue(const MazeMap::ManeuverQueue& queue)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char message[128] = {};
        snprintf(message, sizeof(message), "count,%u", static_cast<unsigned>(queue.size()));
        if (!WriteTelemetryEvent("queue", message))
        {
            AppendStartupTrace("maneuver_test:queue_logging_disabled");
            (void)EmitMissionControllerLine("Maneuver queue logging unavailable; continuing without queue metadata");
            FlushTelemetryLog();
            CloseTelemetryLog();
            _telemetryLoggingEnabled = false;
            return true;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            char codeName[24] = {};
            char queueLine[160] = {};
            FormatManeuverCodeName(queue[i].getCode(), codeName, sizeof(codeName));
            snprintf(
                queueLine,
                sizeof(queueLine),
                "%u,%s,%.6f,%.6f",
                static_cast<unsigned>(i),
                codeName,
                queue[i].getEntrySpeed(),
                queue[i].getExitSpeed());

            if (!WriteTelemetryEvent("queue_entry", queueLine))
            {
                AppendStartupTrace("maneuver_test:queue_logging_disabled");
                (void)EmitMissionControllerLine("Maneuver queue entry logging unavailable; continuing without queue metadata");
                FlushTelemetryLog();
                CloseTelemetryLog();
                _telemetryLoggingEnabled = false;
                return true;
            }
        }

        return true;
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, bool snapToExpectedLocation)
    {
        if (!EmitMissionControllerFormattedOrFail("Queued maneuvers: %u", static_cast<unsigned>(queue.size())))
        {
            return false;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            const MazeMap::ManeuverInstance& entry = queue[i];
            const MazeMap::ManeuverCode code = entry.getCode();
            const float entrySpeed = entry.getEntrySpeed();
            const float exitSpeed = entry.getExitSpeed();
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));

            AppendMissionTraceFormatted(
                "mission_maneuver:begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                entrySpeed,
                exitSpeed);

            if (_maneuverTestMode)
            {
                char phaseName[48] = {};
                snprintf(phaseName, sizeof(phaseName), "maneuver_%u_%s", static_cast<unsigned>(i), codeName);
                if (!BeginTelemetryPhase(phaseName))
                {
                    return false;
                }
            }

            bool ok = false;
            if (IsStraightCode(code))
            {
                ok = ExecuteStraightProfile(
                    0.5f * Config::kCellSizeM * static_cast<float>(static_cast<uint8_t>(code)),
                    entrySpeed,
                    limits.maxSpeedMps,
                    exitSpeed,
                    limits,
                    true);
            }
            else
            {
                const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD_F;
                if (entry.SupportsPointTracking())
                {
                    const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                    ok = ExecuteSmoothTurnProfile(entry, maneuverSpeedLimit, limits);
                }
                else
                {
                    const float distanceM = entry.GetTravelDistanceMeters(Config::kCellSizeM);
                    if (distanceM <= 0.0f)
                    {
                        ok = ExecuteTurnProfile(angleRad, limits);
                    }
                    else
                    {
                        const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                        ok = ExecuteArcProfile(distanceM, angleRad, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                    }
                }
            }

            if (!ok)
            {
                return false;
            }

            _currentDirectionalLocation = entry.getEnd();
            _currentDirection = _currentDirectionalLocation.GetDirection();
            _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());

            AppendMissionTraceFormatted(
                "mission_maneuver:end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                _drive.GetPose().xMeters,
                _drive.GetPose().yMeters,
            RAD_TO_DEG_F * _drive.GetPose().yawRad);

            if (snapToExpectedLocation)
            {
                _drive.SetStartPoint(_currentDirectionalLocation);
            }
        }

        return HoldPosition(50);
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, bool snapToExpectedLocation)
    {
        return ExecuteQueuedManeuvers(queue, FinalLimits(), snapToExpectedLocation);
    }

    bool ObserveCellFromSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        const SensorSnapshot& snapshot,
        bool* outForwardWallCommittedFromUnknown = nullptr)
    {
        if (outForwardWallCommittedFromUnknown != nullptr)
        {
            *outForwardWallCommittedFromUnknown = false;
        }

        MazeMap::WallState knownWallState = MazeMap::WallState::Unknown;
        if (MazeMap::TryGetKnownMissionStartWallState(observedCell, MazeMap::Up, knownWallState))
        {
            PrimeKnownMissionStartCell();
            AppendStartupTrace("observe_current_cell:used_known_start_cell_topology");
            return true;
        }

        MazeMap::Cell& cell = _maze[observedCell];
        const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
        const MazeMap::Direction leftDirection = observedDirection + MazeMap::Left90;
        const MazeMap::Direction rightDirection = observedDirection + MazeMap::Right90;
        const bool forwardUnknown = cell.GetWall(forwardDirection) == MazeMap::WallState::Unknown;
        const bool leftUnknown = cell.GetWall(leftDirection) == MazeMap::WallState::Unknown;
        const bool rightUnknown = cell.GetWall(rightDirection) == MazeMap::WallState::Unknown;
        if (!(forwardUnknown || leftUnknown || rightUnknown))
        {
            AppendStartupTrace("observe_current_cell:skipped_known_walls");
            return true;
        }

        const MazeMap::WallBeliefConfig beliefConfig = BuildWallBeliefConfig();
        const uint32_t beliefTick = millis();
        const auto applyBeliefQualifiedObservation =
            [&](const char* relativeDirectionName,
                MazeMap::Direction absoluteDirection,
                MazeMap::WallState observedState,
                const char* sensorSource,
                const char* sensorMode,
                float primaryDistanceM,
                float secondaryDistanceM,
                bool primaryDetected,
                bool secondaryDetected,
                MazeMap::WallBeliefUpdate* outBeliefUpdate = nullptr) -> bool
        {
            const MazeMap::WallBeliefUpdate beliefUpdate =
                _wallBeliefMap.ApplyObservation(
                    observedCell,
                    absoluteDirection,
                    (observedState == MazeMap::Wall) ?
                        MazeMap::WallSampleClassification::WallHit :
                        MazeMap::WallSampleClassification::WallMiss,
                    beliefConfig,
                    beliefTick);
            if (beliefUpdate.valid && beliefUpdate.hardState != MazeMap::Unknown)
            {
                _maze.SetWall(cell, absoluteDirection, beliefUpdate.hardState);
            }
            if (outBeliefUpdate != nullptr)
            {
                *outBeliefUpdate = beliefUpdate;
            }

            return LogWallObservationDecision(
                observedCell,
                relativeDirectionName,
                absoluteDirection,
                observedState,
                sensorSource,
                sensorMode,
                primaryDistanceM,
                secondaryDistanceM,
                primaryDetected,
                secondaryDetected,
                snapshot,
                beliefUpdate.valid ? beliefUpdate.hardState : MazeMap::WallState::Unknown,
                beliefUpdate.valid ? beliefUpdate.logOdds : 0.0f);
        };

        if (forwardUnknown)
        {
            if (!snapshot.frontWallObservationValid)
            {
                AppendMissionTraceFormatted(
                    "mission_front_wall_update_skipped,cell=(%d,%d),abs=%s,reason=insufficient_evidence",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(forwardDirection));
            }
            else
            {
                MazeMap::WallState observedState = snapshot.frontWall ? MazeMap::Wall : MazeMap::NoWall;
                const char* sensorSource = FrontObservationSourceName(snapshot);
                const char* sensorMode = FrontObservationModeName(snapshot);
                MazeMap::WallBeliefUpdate forwardBeliefUpdate{};
                if (!applyBeliefQualifiedObservation(
                        "forward",
                        forwardDirection,
                        observedState,
                        sensorSource,
                        sensorMode,
                        snapshot.frontLeftDistanceM,
                        snapshot.frontRightDistanceM,
                        snapshot.frontLeftWall,
                        snapshot.frontRightWall,
                        &forwardBeliefUpdate))
                {
                    return false;
                }
                if (outForwardWallCommittedFromUnknown != nullptr &&
                    observedState == MazeMap::Wall &&
                    forwardBeliefUpdate.valid &&
                    forwardBeliefUpdate.hardState == MazeMap::Wall)
                {
                    *outForwardWallCommittedFromUnknown = true;
                }
            }
        }
        if (leftUnknown)
        {
            if (snapshot.leftWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.leftWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                if (!applyBeliefQualifiedObservation(
                        "left",
                        leftDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideLeft),
                        nullptr,
                        snapshot.sideLeftDistanceM,
                        NAN,
                        snapshot.leftWallObservation,
                        false))
                {
                    return false;
                }
            }
            else
            {
                AppendMissionTraceFormatted(
                    "mission_side_wall_update_skipped,cell=(%d,%d),abs=%s,reason=%s",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(leftDirection),
                    snapshot.leftTransitionDetected ? "transition_ambiguous" : "outside_window_or_ambiguous");
            }
        }
        if (rightUnknown)
        {
            if (snapshot.rightWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.rightWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                if (!applyBeliefQualifiedObservation(
                        "right",
                        rightDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideRight),
                        nullptr,
                        snapshot.sideRightDistanceM,
                        NAN,
                        snapshot.rightWallObservation,
                        false))
                {
                    return false;
                }
            }
            else
            {
                AppendMissionTraceFormatted(
                    "mission_side_wall_update_skipped,cell=(%d,%d),abs=%s,reason=%s",
                    observedCell.GetX(),
                    observedCell.GetY(),
                    DirectionName(rightDirection),
                    snapshot.rightTransitionDetected ? "transition_ambiguous" : "outside_window_or_ambiguous");
            }
        }
        return true;
    }

    bool ObserveCurrentCellFromSnapshot(const SensorSnapshot& snapshot)
    {
        return ObserveCellFromSnapshot(_currentCell, _currentDirection, snapshot);
    }

    bool ObserveCurrentCell()
    {
        if (!HoldPosition(Config::kObservationSettleMs))
        {
            return false;
        }

        SensorSnapshot snapshot{};
        if (!CaptureStationaryObservationSnapshot(_currentCell, _currentDirection, snapshot))
        {
            return false;
        }
        _drive.Brake();
        return ObserveCurrentCellFromSnapshot(snapshot);
    }

    bool HandleSearchWallMapUpdateStop(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        float projectedTravelM,
        uint16_t frontVoteCount,
        bool* outStoppedForReplan = nullptr)
    {
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = false;
        }

        _currentCell = observedCell;
        _currentDirection = observedDirection;

        _drive.Brake();

        MazeMap::Path<PATH_SIZE> replannedPath;
        _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, replannedPath);
        const MazeMap::SearchReplanResponse replan = MazeMap::PlanSearchReplanResponse(replannedPath, _currentDirection);
        AppendMissionTraceFormatted(
            "mission_observation_replan,cell=(%d,%d),abs=%s,travel_m=%.4f,front_votes=%u,path_size=%u,next_abs=%s,requires_turn=%u",
            _currentCell.GetX(),
            _currentCell.GetY(),
            DirectionName(_currentDirection),
            projectedTravelM,
            static_cast<unsigned>(frontVoteCount),
            static_cast<unsigned>(replannedPath.GetSize()),
            DirectionName(replan.nextDirection),
            replan.requiresTurn ? 1U : 0U);

        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
        {
            return false;
        }

        if (replan.requiresTurn && !OrientTo(replan.nextDirection, SearchLimits()))
        {
            return false;
        }

        _currentDirectionalLocation = MazeMap::DirectionalLocation(
            MazeMap::MazeLocation::CellCenter(_currentCell),
            _currentDirection);
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = true;
        }
        return true;
    }

    LoopController::ControlVector SearchStraightLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& search = *static_cast<SearchStraightLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const PoseEstimate& livePose = _drive.GetPose();
        float projectedRemainingM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                livePose.xMeters,
                livePose.yMeters,
                search.targetXMeters,
                search.targetYMeters,
                search.targetHeading.x(),
                search.targetHeading.y(),
                projectedRemainingM))
        {
            return FaultLoopPhase(services, "Search straight target projection is invalid");
        }
        const float remainingM = (std::max)(0.0f, projectedRemainingM);
        const float projectedTravelM =
            (std::clamp)(search.distanceToTargetM - projectedRemainingM, 0.0f, search.distanceToTargetM);
        const float sideSensorForwardOffsetM =
            (std::max)(_speedVehicle.SideLeft.GetPosition().x(), _speedVehicle.SideRight.GetPosition().x());

        auto resetRollingObservationPlan = [&search]()
        {
            search.rollingObservationNextSampleIndex = 0U;
            search.rollingObservationPlanInitialized = false;
            search.rollingObservationSideResetTriggerTravelM = 0.0f;
            search.rollingObservationSideResetPending = false;
            memset(search.rollingObservationTriggerTravelM, 0, sizeof(search.rollingObservationTriggerTravelM));
            memset(search.rollingObservationSamples, 0, sizeof(search.rollingObservationSamples));
            for (std::uint8_t sampleIndex = 0U;
                sampleIndex < Config::kSearchRollingObservationSampleCount;
                ++sampleIndex)
            {
                search.rollingObservationFrontLeftCandidateDistanceM[sampleIndex] = NAN;
                search.rollingObservationFrontRightCandidateDistanceM[sampleIndex] = NAN;
            }
        };

        auto initializeRollingObservationPlan = [this, &search, sideSensorForwardOffsetM]() -> const char*
        {
            if (!search.observeWhileRolling || search.rollingObservationCount >= search.cellCount)
            {
                return nullptr;
            }
            if (search.rollingObservationPlanInitialized)
            {
                return nullptr;
            }

            const PoseEstimate startPose = _drive.GetPose();
            for (std::uint8_t sampleIndex = 0U;
                sampleIndex < Config::kSearchRollingObservationSampleCount;
                ++sampleIndex)
            {
                float targetObservationXMeters = 0.0f;
                float targetObservationYMeters = 0.0f;
                if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
                        search.nextRollingObservationCell,
                        search.direction,
                        Config::kCellSizeM,
                        Config::kMazeWallThicknessM,
                        sideSensorForwardOffsetM,
                        Config::kSideWallSegmentCenterFraction,
                        sampleIndex,
                        Config::kSearchRollingObservationSampleCount,
                        targetObservationXMeters,
                        targetObservationYMeters))
                {
                    return "Search straight rolling observation sample pose is invalid";
                }

                float triggerTravelM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                        startPose.xMeters,
                        startPose.yMeters,
                        targetObservationXMeters,
                        targetObservationYMeters,
                        search.targetHeading.x(),
                        search.targetHeading.y(),
                        triggerTravelM))
                {
                    return "Search straight rolling observation sample trigger is invalid";
                }
                if (sampleIndex > 0U &&
                    triggerTravelM < (search.rollingObservationTriggerTravelM[sampleIndex - 1U] - Config::kDistanceToleranceM))
                {
                    AppendMissionTraceFormatted(
                        "mission_observation_trigger_recovered,cell=(%d,%d),abs=%s,sample=%u,prev_m=%.4f,raw_m=%.4f",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        static_cast<unsigned>(sampleIndex),
                        search.rollingObservationTriggerTravelM[sampleIndex - 1U],
                        triggerTravelM);
                    triggerTravelM = search.rollingObservationTriggerTravelM[sampleIndex - 1U];
                }

                search.rollingObservationTriggerTravelM[sampleIndex] = triggerTravelM;
            }

            float targetResetXMeters = 0.0f;
            float targetResetYMeters = 0.0f;
            if (!MazeMap::TryComputeSideWallTravelFractionPoseM(
                    search.nextRollingObservationCell,
                    search.direction,
                    Config::kCellSizeM,
                    sideSensorForwardOffsetM,
                    Config::kSideWallStateResetCellEntryFraction,
                    targetResetXMeters,
                    targetResetYMeters))
            {
                return "Search straight side reset trigger pose is invalid";
            }

            float resetTriggerTravelM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    startPose.xMeters,
                    startPose.yMeters,
                    targetResetXMeters,
                    targetResetYMeters,
                    search.targetHeading.x(),
                    search.targetHeading.y(),
                    resetTriggerTravelM))
            {
                return "Search straight side reset trigger is invalid";
            }
            if (resetTriggerTravelM > (search.rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM))
            {
                AppendMissionTraceFormatted(
                    "mission_side_reset_trigger_recovered,cell=(%d,%d),abs=%s,raw_m=%.4f,first_sample_m=%.4f",
                    search.nextRollingObservationCell.GetX(),
                    search.nextRollingObservationCell.GetY(),
                    DirectionName(search.direction),
                    resetTriggerTravelM,
                    search.rollingObservationTriggerTravelM[0]);
                resetTriggerTravelM =
                    (std::max)(0.0f, search.rollingObservationTriggerTravelM[0] - Config::kDistanceToleranceM);
            }

            search.rollingObservationSideResetTriggerTravelM = resetTriggerTravelM;
            search.rollingObservationSideResetPending = true;
            search.rollingObservationPlanInitialized = true;
            return nullptr;
        };

        if (search.observeWhileRolling)
        {
            while (search.rollingObservationCount < search.cellCount)
            {
                if (const char* error = initializeRollingObservationPlan())
                {
                    return FaultLoopPhase(services, error);
                }

                if (search.rollingObservationSideResetPending &&
                    (projectedTravelM + Config::kDistanceToleranceM) >= search.rollingObservationSideResetTriggerTravelM)
                {
                    _sensors.ResetSideWallMemory();
                    search.rollingObservationSideResetPending = false;
                    AppendMissionTraceFormatted(
                        "mission_side_reset,cell=(%d,%d),abs=%s,travel_m=%.4f,trigger_m=%.4f",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        projectedTravelM,
                        search.rollingObservationSideResetTriggerTravelM);
                    break;
                }

                while (search.rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount &&
                    (projectedTravelM + Config::kDistanceToleranceM) >=
                        search.rollingObservationTriggerTravelM[search.rollingObservationNextSampleIndex])
                {
                    search.rollingObservationSamples[search.rollingObservationNextSampleIndex] = state.sensors;
                    float frontLeftCandidateDistanceM = NAN;
                    float frontRightCandidateDistanceM = NAN;
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontLeft,
                        search.nextRollingObservationCell,
                        search.direction,
                        frontLeftCandidateDistanceM);
                    (void)TryComputeDistanceToCellWallM(
                        livePose,
                        _speedVehicle.FrontRight,
                        search.nextRollingObservationCell,
                        search.direction,
                        frontRightCandidateDistanceM);
                    search.rollingObservationFrontLeftCandidateDistanceM[search.rollingObservationNextSampleIndex] =
                        frontLeftCandidateDistanceM;
                    search.rollingObservationFrontRightCandidateDistanceM[search.rollingObservationNextSampleIndex] =
                        frontRightCandidateDistanceM;
                    ++search.rollingObservationNextSampleIndex;
                }

                if (search.rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount)
                {
                    break;
                }

                SensorSnapshot majoritySnapshot{};
                RollingObservationVoteSummary voteSummary{};
                if (!BuildEvidenceObservationSnapshot(
                        search.rollingObservationSamples,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot,
                        voteSummary))
                {
                    return FaultLoopPhase(services, "Search straight rolling observation majority snapshot is invalid");
                }
                if (!TryApplyFrontWallCharacterizationToObservation(
                        search.nextRollingObservationCell,
                        search.direction,
                        "rolling",
                        search.rollingObservationSamples,
                        search.rollingObservationFrontLeftCandidateDistanceM,
                        search.rollingObservationFrontRightCandidateDistanceM,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot))
                {
                    AppendMissionTraceFormatted(
                        "mission_front_curve_fit_unavailable,cell=(%d,%d),abs=%s,origin=rolling,fallback_valid=%u",
                        search.nextRollingObservationCell.GetX(),
                        search.nextRollingObservationCell.GetY(),
                        DirectionName(search.direction),
                        majoritySnapshot.frontWallObservationValid ? 1U : 0U);
                }

                AppendMissionTraceFormatted(
                    "mission_observation_timed,cell=(%d,%d),abs=%s,samples=%u,start_m=%.4f,end_m=%.4f,travel_m=%.4f,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
                    search.nextRollingObservationCell.GetX(),
                    search.nextRollingObservationCell.GetY(),
                    DirectionName(search.direction),
                    static_cast<unsigned>(voteSummary.sampleCount),
                    search.rollingObservationTriggerTravelM[0],
                    search.rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount - 1U],
                    projectedTravelM,
                    static_cast<unsigned>(voteSummary.frontWallVotes),
                    static_cast<unsigned>(voteSummary.leftWindowValidVotes),
                    static_cast<unsigned>(voteSummary.leftWallVotes),
                    static_cast<unsigned>(voteSummary.rightWindowValidVotes),
                    static_cast<unsigned>(voteSummary.rightWallVotes));
                bool forwardWallCommittedFromUnknown = false;
                if (!ObserveCellFromSnapshot(
                        search.nextRollingObservationCell,
                        search.direction,
                        majoritySnapshot,
                        &forwardWallCommittedFromUnknown))
                {
                    return FaultLoopPhase(services, "Search straight observation commit failed");
                }

                if (forwardWallCommittedFromUnknown)
                {
                    search.stoppedForReplan = true;
                    search.replanObservedCell = search.nextRollingObservationCell;
                    search.replanProjectedTravelM = projectedTravelM;
                    search.replanFrontVoteCount = voteSummary.frontWallVotes;
                    search.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
                    search.completionSettle.timeoutMs = 0U;
                    search.completionSettle.brakeCommand = true;
                TransitionLoopPhase(&search.completionSettle, &Implementation::SettleLoopTick, services);
                    return LoopController::ControlVector::Brake;
                }

                ++search.rollingObservationCount;
                if (search.rollingObservationCount < search.cellCount)
                {
                    search.nextRollingObservationCell = search.nextRollingObservationCell >> search.direction;
                }
                resetRollingObservationPlan();
            }
        }

        const bool stoppingAtEndpoint = search.exitSpeedMps <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            search.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            search.completionSettle.timeoutMs = 0U;
            search.completionSettle.brakeCommand = true;
                TransitionLoopPhase(&search.completionSettle, &Implementation::SettleLoopTick, services);
            return LoopController::ControlVector::Brake;
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - search.exitSpeedMps) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return EndLoopPhase(services);
        }

        const unsigned long nowMs = millis();
        if (!search.stallLogged &&
            search.translationWatchdog.Stalled(projectedTravelM, search.commandedSpeedMps, remainingM, nowMs))
        {
            search.stallLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=search_straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                projectedTravelM,
                remainingM,
                search.commandedSpeedMps);
        }
        if (!search.durationLogged && static_cast<long>(search.expectedCompletionDeadlineMs - nowMs) <= 0)
        {
            search.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=search_straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                projectedTravelM,
                remainingM,
                search.commandedSpeedMps);
        }

        const MotionLimits searchLimits = SearchLimits();
        const float accelLimitedSpeedMps = (std::min)(
            search.cruiseSpeedMps,
            search.commandedSpeedMps + (searchLimits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(search.exitSpeedMps, remainingM, searchLimits.decelMps2);
        search.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        float wallOmegaRadps = 0.0f;
        float signalCorridorErrorM = 0.0f;
        if (TryComputeWallGroundedCorridorErrorM(state.sensors, signalCorridorErrorM))
        {
            wallOmegaRadps += ComputeWallCenterPdOmegaRadps(
                signalCorridorErrorM,
                search.commandedSpeedMps,
                state.dtSeconds,
                search.previousCorridorErrorM,
                search.filteredCorridorErrorRateMps,
                search.previousCorridorErrorValid);
        }
        else
        {
            search.filteredCorridorErrorRateMps = 0.0f;
            search.previousCorridorErrorValid = false;
        }
        if (stoppingAtEndpoint &&
            std::isfinite(state.sensors.frontLeftDistanceM) &&
            std::isfinite(state.sensors.frontRightDistanceM) &&
            state.sensors.frontLeftDistanceM < Config::kFrontWallOnThresholdM &&
            state.sensors.frontRightDistanceM < Config::kFrontWallOnThresholdM &&
            remainingM < 0.07f)
        {
            wallOmegaRadps += Config::kFrontSkewGain * state.sensors.frontSkewM;
        }

        const float headingErrorRad = HeadingErrorRad(search.targetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) +
            wallOmegaRadps;
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -searchLimits.maxAngularSpeedRadps,
            searchLimits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            search.commandedSpeedMps,
            angularCommandRadps,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteSearchStraightCellsLoopDriven(
        MazeMap::Direction direction,
        uint16_t cellCount,
        float entrySpeedMps,
        float cruiseSpeedMps,
        float exitSpeedMps,
        bool snapAtEnd,
        bool observeWhileRolling = false,
        bool* outStoppedForReplan = nullptr)
    {
        (void)snapAtEnd;
        if (outStoppedForReplan != nullptr)
        {
            *outStoppedForReplan = false;
        }
        if (cellCount == 0U)
        {
            return true;
        }

        SearchStraightLoopState search{};
        search.direction = direction;
        search.cellCount = cellCount;
        search.entrySpeedMps = entrySpeedMps;
        search.cruiseSpeedMps = cruiseSpeedMps;
        search.exitSpeedMps = exitSpeedMps;
        search.observeWhileRolling = observeWhileRolling;
        search.startCell = _currentCell;
        search.destination = _currentCell;
        for (std::uint16_t index = 0U; index < cellCount; ++index)
        {
            search.destination = search.destination >> direction;
        }
        search.targetHeading = DirectionToUnitVector(direction);
        MazeMap::MazeLocation::CellCenter(search.destination).GetPhysicalLocation(
            Config::kCellSizeM,
            search.targetXMeters,
            search.targetYMeters);

        const PoseEstimate startPose = _drive.GetPose();
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                startPose.xMeters,
                startPose.yMeters,
                search.targetXMeters,
                search.targetYMeters,
                search.targetHeading.x(),
                search.targetHeading.y(),
                search.distanceToTargetM))
        {
            return Fail("Search straight target distance is invalid");
        }
        if (search.distanceToTargetM < -Config::kDistanceToleranceM)
        {
            return Fail("Search straight target fell behind the current pose");
        }

        search.nextRollingObservationCell = search.startCell;
        if (observeWhileRolling)
        {
            search.nextRollingObservationCell = search.nextRollingObservationCell >> direction;
        }
        search.commandedSpeedMps = (std::max)(entrySpeedMps, 0.0f);
        search.translationWatchdog.Reset(0.0f, millis());
        search.expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2500.0f + (5000.0f * search.distanceToTargetM));

        if (!RunLoopSession(&search, &Implementation::SearchStraightLoopTick))
        {
            return false;
        }

        if (search.stoppedForReplan)
        {
            return HandleSearchWallMapUpdateStop(
                search.replanObservedCell,
                direction,
                search.replanProjectedTravelM,
                search.replanFrontVoteCount,
                outStoppedForReplan);
        }

        _currentCell = search.destination;
        _currentDirectionalLocation =
            MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    bool ExploreFullMaze()
    {
        while (!_maze.IsComplete())
        {
            if (_maze.HasFoundGoal() && !_goalPauseComplete)
            {
                if (!EmitMissionControllerLineOrFail("Goal discovered"))
                {
                    return false;
                }
                if (!DriveToGoalAndPause())
                {
                    return false;
                }
                continue;
            }

            MazeMap::Path<PATH_SIZE> path;
            _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);

            if (path.GetSize() < 2)
            {
                if (!ObserveCurrentCell())
                {
                    return false;
                }
                _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);
                if (path.GetSize() < 2 && !_maze.IsComplete())
                {
                    return Fail("Search path stalled before maze completion");
                }
                continue;
            }

            if (!ExecuteSearchPath(path, true))
            {
                return false;
            }
        }

        return true;
    }

    bool DriveToGoalAndPause()
    {
        while (!IsInGoalCell(_currentCell))
        {
            MazeMap::Path<PATH_SIZE> goalPath;
            _searchPathFinder.PathToGoal(_currentCell, _currentDirection, goalPath);
            if (goalPath.GetSize() < 2)
            {
                return Fail("Unable to drive to goal after detection");
            }
            if (!ExecuteSearchPath(goalPath, false))
            {
                return false;
            }
        }

        if (!EmitMissionControllerLineOrFail("Holding in goal for 2 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kGoalPauseMs))
        {
            return false;
        }
        _goalPauseComplete = true;
        return ObserveCurrentCell();
    }

    bool ReturnToStart()
    {
        const MazeMap::CellCoordinates start(0, 0);
        while (_currentCell != start)
        {
            MazeMap::HalfStepPath<PATH_SIZE * 2> returnHalfStepPath;
            _searchPathFinder.HalfStepPathFromTo(_currentCell, _currentDirection, start, returnHalfStepPath);

            MazeMap::ManeuverPath maneuverPath;
            if (returnHalfStepPath.GetSize() > 1U
                && MazeMap::ManeuverPath::FromHalfStep(returnHalfStepPath, _currentDirectionalLocation, maneuverPath)
                && maneuverPath.GetSize() > 0U)
            {
                char traceLine[112] = {};
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "return_to_start:using_search_halfstep_maneuvers,halfsteps=%u,maneuvers=%u",
                    static_cast<unsigned>(returnHalfStepPath.GetSize()),
                    static_cast<unsigned>(maneuverPath.GetSize()));
                AppendStartupTrace(traceLine);

                MazeMap::ManeuverQueue queue(maneuverPath, _currentDirectionalLocation);
                queue.ComputeSpeeds(_mappingVehicle, 0.0f, 0.0f);
                const MotionLimits returnLimits = SearchLimits();
                ApplyAsymmetricQueueLimits(queue, returnLimits, _mappingVehicle, 0.0f, 0.0f);
                if (!ExecuteQueuedManeuvers(queue, returnLimits, false))
                {
                    return false;
                }
            }
            else
            {
                MazeMap::Path<PATH_SIZE> path;
                _searchPathFinder.PathFromTo(_currentCell, _currentDirection, start, path);
                if (path.GetSize() < 2)
                {
                    return Fail("Unable to return to start");
                }
                AppendStartupTrace("return_to_start:fallback_search_path");
                if (!ExecuteSearchPath(path, false))
                {
                    return false;
                }
            }
        }

        if (!OrientTo(MazeMap::Up, SearchLimits()))
        {
            return false;
        }

        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return ObserveCurrentCell();
    }

    bool HandleInterRunServiceCycle()
    {
        InterRunServicePauseLoopState pauseState{};
        return RunLoopSession(&pauseState, &Implementation::InterRunServicePauseTick) &&
            PrepareForSecondSpeedRun();
    }

    bool PrepareForSecondSpeedRun()
    {
        if (!_sensors.Begin())
        {
            return Fail("Sensor reset failed after inter-run service");
        }

        return RunStartupWallCalibration();
    }

    bool FinishSpeedRunAndReturnToStart()
    {
        if (!EmitMissionControllerLineOrFail("Holding at finish for 3 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kSpeedRunFinishPauseMs))
        {
            return false;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return false;
        }
        return ReturnToStart();
    }

    bool ExecuteRacingRunCycle()
    {
        SetRacingFanEnabled(true);
        const bool ok = RunSpeedRun() && FinishSpeedRunAndReturnToStart();
        SetRacingFanEnabled(false);
        return ok;
    }

    bool RunSpeedRun()
    {
        MazeMap::ManeuverPath path;
        _speedPathFinder.ManeuverPathToGoal(_currentCell, _currentDirection, path);
        if (path.GetSize() == 0)
        {
            return Fail("ManeuverPathFinder returned an empty path");
        }

        MazeMap::ManeuverQueue queue(path, _currentDirectionalLocation);
        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
        return ExecuteQueuedManeuvers(queue, true);
    }

    bool ExecuteSearchStraightCells(
        MazeMap::Direction direction,
        uint16_t cellCount,
        float entrySpeedMps,
        float cruiseSpeedMps,
        float exitSpeedMps,
        bool snapAtEnd,
        bool observeWhileRolling = false,
        bool* outStoppedForReplan = nullptr)
    {
        return ExecuteSearchStraightCellsLoopDriven(
            direction,
            cellCount,
            entrySpeedMps,
            cruiseSpeedMps,
            exitSpeedMps,
            snapAtEnd,
            observeWhileRolling,
            outStoppedForReplan);
    }

    bool ExecuteSearchPath(const MazeMap::Path<PATH_SIZE>& path, bool observeFinalCell)
    {
        if (path.GetSize() < 2)
        {
            return observeFinalCell ? ObserveCurrentCell() : true;
        }

        const MotionLimits searchLimits = SearchLimits();
        const float cautiousCruiseSpeedMps = SearchUnmappedCruiseSpeedMps();
        uint16_t pathIndex = 1U;

        while (pathIndex < path.GetSize())
        {
            const MazeMap::SearchStraightPlan plan = MazeMap::PlanSearchStraightSegment(_maze, path, pathIndex);
            if (plan.direction == MazeMap::None || plan.TotalCellCount() == 0U)
            {
                return Fail("Search path contained an invalid segment");
            }

            if (!OrientTo(plan.direction, searchLimits))
            {
                return false;
            }

            float rollingEntrySpeedMps = 0.0f;
            if (plan.fullSpeedCellCount > 0U)
            {
                const float exitSpeedMps = (plan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                if (!ExecuteSearchStraightCells(
                    plan.direction,
                    plan.fullSpeedCellCount,
                    0.0f,
                    searchLimits.maxSpeedMps,
                    exitSpeedMps,
                    plan.cautiousCellCount == 0U,
                    false))
                {
                    return false;
                }
                rollingEntrySpeedMps = exitSpeedMps;
            }

            if (plan.cautiousCellCount > 0U)
            {
                if (!observeFinalCell)
                {
                    const float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            plan.cautiousCellCount,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            0.0f,
                            true))
                    {
                        return false;
                    }
                    pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
                    continue;
                }

                float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? rollingEntrySpeedMps : 0.0f;
                while (true)
                {
                    bool stoppedForReplan = false;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            1U,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            cautiousCruiseSpeedMps,
                            false,
                            true,
                            &stoppedForReplan))
                    {
                        return false;
                    }
                    if (stoppedForReplan)
                    {
                        return true;
                    }

                    cautiousEntrySpeedMps = cautiousCruiseSpeedMps;
                    MazeMap::Path<PATH_SIZE> continuingPath;
                    _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, continuingPath);
                    if (continuingPath.GetSize() < 2U)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    const MazeMap::SearchStraightPlan nextPlan = MazeMap::PlanSearchStraightSegment(_maze, continuingPath, 1U);
                    if (nextPlan.direction == MazeMap::None || nextPlan.TotalCellCount() == 0U)
                    {
                        return Fail("Search path contained an invalid continuation");
                    }

                    if (nextPlan.direction != plan.direction)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    if (nextPlan.fullSpeedCellCount > 0U)
                    {
                        const float exitSpeedMps = (nextPlan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                        if (!ExecuteSearchStraightCells(
                                plan.direction,
                                nextPlan.fullSpeedCellCount,
                                cautiousEntrySpeedMps,
                                searchLimits.maxSpeedMps,
                                exitSpeedMps,
                                nextPlan.cautiousCellCount == 0U,
                                false))
                        {
                            return false;
                        }
                        if (nextPlan.cautiousCellCount == 0U)
                        {
                            return observeFinalCell ? ObserveCurrentCell() : true;
                        }

                        cautiousEntrySpeedMps = exitSpeedMps;
                    }
                }
            }

            pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
        }

        return observeFinalCell ? ObserveCurrentCell() : true;
    }

    struct MapQualifiedSideWallReference
    {
        bool useWall = false;
        MazeMap::CellCoordinates cell = MazeMap::CellCoordinates(0U, 0U);
        MazeMap::Direction wallDirection = MazeMap::None;
    };

    bool TryResolveMapQualifiedSideWallReference(
        const PoseEstimate& pose,
        const MazeMap::WallSensor& sensor,
        bool distanceValidForControl,
        MapQualifiedSideWallReference& reference) const
    {
        // Exclusively for the purpose of centering.
        reference = MapQualifiedSideWallReference{};
        if (!distanceValidForControl)
        {
            return false;
        }

        float alongWallCoordinateM = 0.0f;
        if (!TryComputeSideWallAimCoordinateM(pose, sensor, alongWallCoordinateM))
        {
            return false;
        }

        const Eigen::Vector2f worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
        const float sensorXM = pose.xMeters + worldOffset.x();
        const float sensorYM = pose.yMeters + worldOffset.y();
        const Eigen::Vector2f sensorFacing = SensorWorldFacing(pose, sensor);

        int cellX = -1;
        int cellY = -1;
        MazeMap::Direction wallDirection = MazeMap::None;
        if (std::fabs(sensorFacing.x()) >= std::fabs(sensorFacing.y()))
        {
            if (!std::isfinite(sensorXM) || !std::isfinite(alongWallCoordinateM))
            {
                return false;
            }

            cellX = static_cast<int>(std::floor(sensorXM / Config::kCellSizeM));
            cellY = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
            wallDirection = (sensorFacing.x() >= 0.0f) ? MazeMap::Right : MazeMap::Left;
        }
        else
        {
            if (!std::isfinite(sensorYM) || !std::isfinite(alongWallCoordinateM))
            {
                return false;
            }

            cellX = static_cast<int>(std::floor(alongWallCoordinateM / Config::kCellSizeM));
            cellY = static_cast<int>(std::floor(sensorYM / Config::kCellSizeM));
            wallDirection = (sensorFacing.y() >= 0.0f) ? MazeMap::Up : MazeMap::Down;
        }

        if (cellX < 0 || cellY < 0 ||
            cellX >= static_cast<int>(_maze.GetXSize()) ||
            cellY >= static_cast<int>(_maze.GetYSize()))
        {
            return false;
        }

        const MazeMap::Cell& observedCell = _maze.Index(cellX, cellY);
        if (observedCell.GetWall(wallDirection) != MazeMap::Wall)
        {
            return false;
        }

        reference.useWall = true;
        reference.cell = MazeMap::CellCoordinates(static_cast<uint8_t>(cellX), static_cast<uint8_t>(cellY));
        reference.wallDirection = wallDirection;
        return true;
    }

    void ResolveMapQualifiedSideWalls(const SensorSnapshot& snapshot, bool& useLeftWall, bool& useRightWall) const
    {
        // Exclusively for the purpose of centering.
        const PoseEstimate& pose = _drive.GetPose();
        MapQualifiedSideWallReference leftReference{};
        MapQualifiedSideWallReference rightReference{};
        useLeftWall = TryResolveMapQualifiedSideWallReference(
            pose,
            _speedVehicle.SideLeft,
            snapshot.leftDistanceValidForControl,
            leftReference);
        useRightWall = TryResolveMapQualifiedSideWallReference(
            pose,
            _speedVehicle.SideRight,
            snapshot.rightDistanceValidForControl,
            rightReference);
    }

    bool TryComputeWallGroundedCorridorCoordinateM(const SensorSnapshot& snapshot, float& coordinateM, bool& correctsXAxis) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        switch (_currentDirection)
        {
        case MazeMap::Up:
        case MazeMap::Down:
            correctsXAxis = true;
            break;
        case MazeMap::Left:
        case MazeMap::Right:
            correctsXAxis = false;
            break;
        default:
            return false;
        }

        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate = false;
        bool haveRightCoordinate = false;
        const PoseEstimate& pose = _drive.GetPose();
        MapQualifiedSideWallReference leftReference{};
        MapQualifiedSideWallReference rightReference{};

        if (TryResolveMapQualifiedSideWallReference(
                pose,
                _speedVehicle.SideLeft,
                snapshot.leftDistanceValidForControl,
                leftReference))
        {
            haveLeftCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                leftReference.cell,
                leftReference.wallDirection,
                leftCoordinateM);
        }

        if (TryResolveMapQualifiedSideWallReference(
                pose,
                _speedVehicle.SideRight,
                snapshot.rightDistanceValidForControl,
                rightReference))
        {
            haveRightCoordinate = TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                rightReference.cell,
                rightReference.wallDirection,
                rightCoordinateM);
        }

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        coordinateM = haveLeftCoordinate && haveRightCoordinate ?
            (0.5f * (leftCoordinateM + rightCoordinateM)) :
            (haveLeftCoordinate ? leftCoordinateM : rightCoordinateM);
        return std::isfinite(coordinateM);
    }

    bool TryComputeWallGroundedCorridorErrorM(const SensorSnapshot& snapshot, float& corridorErrorM) const
    {
        corridorErrorM = 0.0f;

        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
        {
            return false;
        }

        float centerXM = 0.0f;
        float centerYM = 0.0f;
        if (!TryGetCellCenterMeters(_currentCell, centerXM, centerYM))
        {
            return false;
        }

        const float errorXM = correctsXAxis ? (corridorCoordinateM - centerXM) : 0.0f;
        const float errorYM = correctsXAxis ? 0.0f : (corridorCoordinateM - centerYM);
        const Eigen::Vector2f heading = DirectionToUnitVector(_currentDirection);
        corridorErrorM = (heading.y() * errorXM) - (heading.x() * errorYM);
        return std::isfinite(corridorErrorM);
    }

    static float ComputeWallCenterPdOmegaRadps(
        float corridorErrorM,
        float forwardSpeedMps,
        float dtSeconds,
        float& previousCorridorErrorM,
        float& filteredCorridorErrorRateMps,
        bool& previousCorridorErrorValid)
    {
        // Exclusively for the purpose of centering.
        float corridorErrorRateMps = filteredCorridorErrorRateMps;
        if (previousCorridorErrorValid && (dtSeconds > 0.0f))
        {
            const float rawCorridorErrorRateMps = (corridorErrorM - previousCorridorErrorM) / dtSeconds;
            const float derivativeAlpha =
                dtSeconds / (Config::kWallCenterDerivativeFilterTauSeconds + dtSeconds);
            corridorErrorRateMps += derivativeAlpha * (rawCorridorErrorRateMps - corridorErrorRateMps);
        }
        else
        {
            corridorErrorRateMps = 0.0f;
        }

        previousCorridorErrorM = corridorErrorM;
        filteredCorridorErrorRateMps = corridorErrorRateMps;
        previousCorridorErrorValid = true;
        const float rawWallCenterOmegaRadps =
            (Config::kWallCenterGain * corridorErrorM) +
            (Config::kWallCenterD * corridorErrorRateMps);
        const float maxCenteringCurvatureMInv =
            (2.0f * Config::kWallCenterMaxClosurePerCellM) /
            (Config::kCellSizeM * Config::kCellSizeM);
        const float maxWallCenterOmegaRadps = std::fabs(forwardSpeedMps) * maxCenteringCurvatureMInv;
        return (std::clamp)(
            rawWallCenterOmegaRadps,
            -maxWallCenterOmegaRadps,
            maxWallCenterOmegaRadps);
    }

    bool ApplyWallGroundedCorridorPoseCorrection(const SensorSnapshot& snapshot)
    {
        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(corridorCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(corridorCoordinateM);
        }

        if (std::fabs(corridorCoordinateM - priorCoordinateM) >= 0.001f)
        {
            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "mission_pose_snap,axis=%s,from=%.4f,to=%.4f,cell=(%d,%d)",
                correctsXAxis ? "x" : "y",
                priorCoordinateM,
                corridorCoordinateM,
                _currentCell.GetX(),
                _currentCell.GetY());
            AppendStartupTrace(traceLine);
        }
        return true;
    }

    bool TryComputeTurnWallEdgeCoordinateM(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker,
        float& coordinateM,
        bool& correctsXAxis,
        const char*& sourceName) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        sourceName = "none";

        // The center post in the 2x2 goal can create side-sensor rising edges that do not correspond to a usable
        // corridor wall boundary, so suppress turn-edge grounding inside the goal area.
        if (IsInGoalCell(_currentCell))
        {
            sourceName = "goal_suppressed";
            return false;
        }

        switch (targetDirection)
        {
        case MazeMap::Up:
        case MazeMap::Down:
            correctsXAxis = true;
            break;
        case MazeMap::Left:
        case MazeMap::Right:
            correctsXAxis = false;
            break;
        default:
            return false;
        }

        const MazeMap::Cell& cell = _maze[_currentCell];
        const MazeMap::Direction leftWallDirection = targetDirection + MazeMap::Left90;
        const MazeMap::Direction rightWallDirection = targetDirection + MazeMap::Right90;
        const PoseEstimate& pose = _drive.GetPose();
        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate =
            edgeTracker.leftWallRose &&
            snapshot.leftDistanceValidForControl &&
            (cell.GetWall(leftWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                _currentCell,
                leftWallDirection,
                leftCoordinateM);
        bool haveRightCoordinate =
            edgeTracker.rightWallRose &&
            snapshot.rightDistanceValidForControl &&
            (cell.GetWall(rightWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                _currentCell,
                rightWallDirection,
                rightCoordinateM);

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        if (haveLeftCoordinate && haveRightCoordinate)
        {
            coordinateM = 0.5f * (leftCoordinateM + rightCoordinateM);
            sourceName = "left+right";
        }
        else if (haveLeftCoordinate)
        {
            coordinateM = leftCoordinateM;
            sourceName = "left";
        }
        else
        {
            coordinateM = rightCoordinateM;
            sourceName = "right";
        }

        return std::isfinite(coordinateM);
    }

    bool ApplyTurnWallEdgePoseCorrection(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker)
    {
        float correctedCoordinateM = 0.0f;
        bool correctsXAxis = false;
        const char* sourceName = "none";
        if (!TryComputeTurnWallEdgeCoordinateM(
                targetDirection,
                snapshot,
                edgeTracker,
                correctedCoordinateM,
                correctsXAxis,
                sourceName))
        {
            if (IsInGoalCell(_currentCell) && (edgeTracker.leftWallRose || edgeTracker.rightWallRose))
            {
                AppendMissionTraceFormatted(
                    "mission_turn_edge_snap:suppressed,cell=(%d,%d),dir=%s,left_rose=%u,right_rose=%u",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    DirectionName(targetDirection),
                    edgeTracker.leftWallRose ? 1U : 0U,
                    edgeTracker.rightWallRose ? 1U : 0U);
            }
            return true;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(correctedCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(correctedCoordinateM);
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn_edge_snap,axis=%s,from=%.4f,to=%.4f,dir=%s,source=%s,cell=(%d,%d)",
            correctsXAxis ? "x" : "y",
            priorCoordinateM,
            correctedCoordinateM,
            DirectionName(targetDirection),
            sourceName,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        (void)WriteMissionTraceLineBestEffort(traceLine, "mission_text_logging:turn_edge_snap_write_failed");

        return true;
    }

    bool OrientTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const MazeMap::RelativeDirection relative = targetDirection - _currentDirection;
        if (relative == MazeMap::Forward)
        {
            return true;
        }

        if (RelativeToInPlaceCode(relative) == MazeMap::MC_NONE)
        {
            return Fail("Unsupported in-place turn requested");
        }

        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Mission turn angle is invalid");
        }
        char traceLine[160] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:begin,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f,cell=(%d,%d)",
            RAD_TO_DEG_F * _drive.GetPose().yawRad,
            RAD_TO_DEG_F * targetYawRad,
            RAD_TO_DEG_F * angleRad,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        MazeMap::TurnWallEdgeTracker wallEdgeTracker{};
        if (!ExecuteTurnProfile(angleRad, limits, &wallEdgeTracker))
        {
            return false;
        }

        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:end,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f",
            _drive.GetPose().xMeters,
            _drive.GetPose().yMeters,
            RAD_TO_DEG_F * _drive.GetPose().yawRad,
            _drive.GetPose().linearSpeedMps,
            _drive.GetPose().angularSpeedRadps);
        AppendStartupTrace(traceLine);
        _currentDirection = targetDirection;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    LoopController::ControlVector StraightLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& straight = *static_cast<StraightLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - straight.startDistanceM);
        float remainingM = (std::max)(0.0f, straight.distanceM - traveledM);
        if (straight.targetPositionOverride != nullptr)
        {
            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    state.estimate.xMeters,
                    state.estimate.yMeters,
                    straight.targetPositionOverride->x(),
                    straight.targetPositionOverride->y(),
                    straight.targetHeading.x(),
                    straight.targetHeading.y(),
                    projectedRemainingM))
            {
                return FaultLoopPhase(services, "Straight target projection is invalid");
            }
            remainingM = (std::max)(0.0f, projectedRemainingM);
        }

        const bool stoppingAtEndpoint = straight.exitSpeed <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            straight.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            straight.completionSettle.timeoutMs = 0U;
            straight.completionSettle.brakeCommand = true;
                TransitionLoopPhase(&straight.completionSettle, &Implementation::SettleLoopTick, services);
            return LoopController::ControlVector::Brake;
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - straight.exitSpeed) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return EndLoopPhase(services);
        }

        const unsigned long nowMs = millis();
        if (!straight.stallLogged &&
            straight.translationWatchdog.Stalled(traveledM, straight.commandedSpeedMps, remainingM, nowMs))
        {
            straight.stallLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                straight.commandedSpeedMps);
        }
        if (!straight.durationLogged && static_cast<long>(straight.expectedCompletionDeadlineMs - nowMs) <= 0)
        {
            straight.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                straight.commandedSpeedMps);
        }

        const float accelLimitedSpeedMps = (std::min)(
            straight.cruiseSpeed,
            straight.commandedSpeedMps + (straight.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(straight.exitSpeed, remainingM, straight.limits.decelMps2);
        straight.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        float wallOmegaRadps = 0.0f;
        if (straight.useWallCentering)
        {
            if (straight.diagonalHeading)
            {
                wallOmegaRadps += ComputeDiagonalWallCenterOmegaRadps(
                    gWallDistanceCalibration,
                    state.sensors.sideLeftDifferentialLight,
                    state.sensors.sideRightDifferentialLight);
            }
            else
            {
                float signalCorridorErrorM = 0.0f;
                if (TryComputeWallGroundedCorridorErrorM(state.sensors, signalCorridorErrorM))
                {
                    wallOmegaRadps += ComputeWallCenterPdOmegaRadps(
                        signalCorridorErrorM,
                        straight.commandedSpeedMps,
                        state.dtSeconds,
                        straight.previousCorridorErrorM,
                        straight.filteredCorridorErrorRateMps,
                        straight.previousCorridorErrorValid);
                }
                else
                {
                    straight.filteredCorridorErrorRateMps = 0.0f;
                    straight.previousCorridorErrorValid = false;
                }
                if (stoppingAtEndpoint &&
                    std::isfinite(state.sensors.frontLeftDistanceM) &&
                    std::isfinite(state.sensors.frontRightDistanceM) &&
                    state.sensors.frontLeftDistanceM < Config::kFrontWallOnThresholdM &&
                    state.sensors.frontRightDistanceM < Config::kFrontWallOnThresholdM &&
                    remainingM < 0.07f)
                {
                    wallOmegaRadps += Config::kFrontSkewGain * state.sensors.frontSkewM;
                }
            }
        }
        else
        {
            straight.filteredCorridorErrorRateMps = 0.0f;
            straight.previousCorridorErrorValid = false;
        }

        const float headingErrorRad = HeadingErrorRad(straight.targetHeading, state.estimate.headingUnit);
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) +
            wallOmegaRadps;
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -straight.limits.maxAngularSpeedRadps,
            straight.limits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            straight.commandedSpeedMps,
            angularCommandRadps,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteStraightProfile(
        float distanceM,
        float entrySpeed,
        float cruiseSpeed,
        float exitSpeed,
        const MotionLimits& limits,
        bool useWallCentering,
        const Eigen::Vector2f* targetHeadingOverride = nullptr,
        const Eigen::Vector2f* targetPositionOverride = nullptr)
    {
        StraightLoopState straight{};
        straight.distanceM = distanceM;
        straight.entrySpeed = entrySpeed;
        straight.cruiseSpeed = cruiseSpeed;
        straight.exitSpeed = exitSpeed;
        straight.limits = limits;
        straight.useWallCentering = useWallCentering;
        straight.targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        straight.targetPositionOverride = targetPositionOverride;
        straight.diagonalHeading = IsApproximatelyDiagonalHeadingUnit(straight.targetHeading);
        straight.commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        straight.startDistanceM = _drive.GetAverageDistanceMeters();
        straight.expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        straight.translationWatchdog.Reset(0.0f, millis());
        return RunLoopSession(&straight, &Implementation::StraightLoopTick);
    }

    LoopController::ControlVector TurnLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& turn = *static_cast<TurnLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }
        if (turn.wallEdgeTracker != nullptr)
        {
            MazeMap::ObserveTurnWallStates(*turn.wallEdgeTracker, state.sensors.leftWall, state.sensors.rightWall);
        }

        const float errorRad = AngleErrorRad(turn.targetYawRad, state.estimate.yawRad);
        if (MazeMap::IsInPlaceTurnComplete(errorRad, state.estimate.angularSpeedRadps, turn.turnProfile))
        {
            turn.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            turn.completionSettle.timeoutMs = 0U;
            turn.completionSettle.brakeCommand = false;
            TransitionLoopPhase(&turn.completionSettle, &Implementation::SettleLoopTick, services);
            return LoopController::ControlVector::Brake;
        }
        if (!turn.durationLogged && static_cast<long>(turn.expectedCompletionDeadlineMs - millis()) <= 0)
        {
            turn.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=turn,reason=elapsed_budget_exceeded,cell=(%d,%d),yaw_err_deg=%.2f,w_radps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                RAD_TO_DEG_F * errorRad,
                state.estimate.angularSpeedRadps);
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                errorRad,
                state.estimate.angularSpeedRadps,
                turn.turnProfile,
                angularCommandRadps))
        {
            return FaultLoopPhase(services, "Turn profile became invalid");
        }

        return _drive.PointControlVector(
            0.0f,
            angularCommandRadps,
            MazeMap::CommandPD::StateWheelOmegaPD);
    }

    bool ExecuteTurnProfile(
        float angleRad,
        const MotionLimits& limits,
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr)
    {
        TurnLoopState turn{};
        turn.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        turn.turnProfile = BuildSharedInPlaceTurnProfile(limits);
        turn.expectedCompletionDeadlineMs = millis() + 2500UL;
        turn.wallEdgeTracker = wallEdgeTracker;
        return RunLoopSession(&turn, &Implementation::TurnLoopTick);
    }

    LoopController::ControlVector ArcLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& arc = *static_cast<ArcLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - arc.startDistanceM);
        const float remainingM = (std::max)(0.0f, arc.distanceM - traveledM);
        const bool stoppingAtEndpoint = arc.exitSpeed <= 0.05f;
        if (stoppingAtEndpoint && (remainingM <= Config::kDistanceToleranceM))
        {
            arc.completionSettle.stationaryHoldMs = Config::kMotionSettleHoldMs;
            arc.completionSettle.timeoutMs = 0U;
            arc.completionSettle.brakeCommand = true;
                TransitionLoopPhase(&arc.completionSettle, &Implementation::SettleLoopTick, services);
            return LoopController::ControlVector::Brake;
        }

        const bool terminalReached =
            (remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps - arc.exitSpeed) <= Config::kSpeedToleranceMps);
        if (terminalReached)
        {
            return EndLoopPhase(services);
        }

        const unsigned long nowMs = millis();
        if (!arc.stallLogged &&
            arc.translationWatchdog.Stalled(traveledM, arc.commandedSpeedMps, remainingM, nowMs))
        {
            arc.stallLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=arc,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                arc.commandedSpeedMps);
        }
        if (!arc.durationLogged && static_cast<long>(arc.expectedCompletionDeadlineMs - nowMs) <= 0)
        {
            arc.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=arc,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                arc.commandedSpeedMps);
        }

        const float accelLimitedSpeedMps = (std::min)(
            arc.cruiseSpeed,
            arc.commandedSpeedMps + (arc.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(arc.exitSpeed, remainingM, arc.limits.decelMps2);
        arc.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float progress = (std::clamp)(traveledM / arc.distanceM, 0.0f, 1.0f);
        const float targetYawRad = WrapAngleRad(arc.startYawRad + (arc.angleRad * progress));
        const float headingErrorRad = AngleErrorRad(targetYawRad, state.estimate.yawRad);
        float angularCommandRadps =
            (arc.curvature * arc.commandedSpeedMps) +
            (Config::kArcHeadingKp * headingErrorRad);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -arc.limits.maxAngularSpeedRadps,
            arc.limits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            arc.commandedSpeedMps,
            angularCommandRadps,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteArcProfile(float distanceM, float angleRad, float entrySpeed, float exitSpeed, float cruiseSpeed, const MotionLimits& limits)
    {
        if (distanceM <= 0.0f)
        {
            return ExecuteTurnProfile(angleRad, limits);
        }

        ArcLoopState arc{};
        arc.distanceM = distanceM;
        arc.angleRad = angleRad;
        arc.entrySpeed = entrySpeed;
        arc.exitSpeed = exitSpeed;
        arc.cruiseSpeed = cruiseSpeed;
        arc.limits = limits;
        arc.startDistanceM = _drive.GetAverageDistanceMeters();
        arc.startYawRad = _drive.GetPose().yawRad;
        arc.curvature = angleRad / distanceM;
        arc.commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        arc.translationWatchdog.Reset(0.0f, millis());
        arc.expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        return RunLoopSession(&arc, &Implementation::ArcLoopTick);
    }

    LoopController::ControlVector SmoothTurnLoopTick(
        void* rawState,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        auto& smoothTurn = *static_cast<SmoothTurnLoopState*>(rawState);
        if (!LogTelemetrySample(false, state))
        {
            return FaultLoopPhase(services, "Failed to write maneuver test sample");
        }

        const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - smoothTurn.startDistanceM);
        const float remainingM = (std::max)(0.0f, smoothTurn.totalDistanceM - traveledM);
        if (remainingM <= Config::kDistanceToleranceM)
        {
            return EndLoopPhase(services);
        }

        const unsigned long nowMs = millis();
        if (!smoothTurn.stallLogged &&
            smoothTurn.translationWatchdog.Stalled(traveledM, smoothTurn.maneuverSpeedMps, remainingM, nowMs))
        {
            smoothTurn.stallLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=smooth_turn,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                smoothTurn.maneuverSpeedMps);
        }
        if (!smoothTurn.durationLogged && static_cast<long>(smoothTurn.expectedCompletionDeadlineMs - nowMs) <= 0)
        {
            smoothTurn.durationLogged = true;
            AppendMissionTraceFormatted(
                "mission_motion_watchdog,mode=smooth_turn,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                _currentCell.GetX(),
                _currentCell.GetY(),
                traveledM,
                remainingM,
                smoothTurn.maneuverSpeedMps);
        }

        MazeMap::ManeuverPoint point{};
        if (!smoothTurn.maneuver.TryGetManeuverPoint(
                traveledM,
                smoothTurn.maneuverSpeedMps,
                point,
                Config::kCellSizeM))
        {
            return FaultLoopPhase(services, "Maneuver point became invalid");
        }

        point.Omega = (std::clamp)(
            point.Omega,
            -smoothTurn.limits.maxAngularSpeedRadps,
            smoothTurn.limits.maxAngularSpeedRadps);
        return _drive.PointControlVector(
            point,
            kMissionDriveBaseTrackingCommandPd);
    }

    bool ExecuteSmoothTurnProfile(
        const MazeMap::ManeuverInstance& maneuver,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        SmoothTurnLoopState smoothTurn{};
        smoothTurn.maneuver = maneuver;
        if (!smoothTurn.maneuver.SupportsPointTracking())
        {
            return Fail("Tracked maneuver geometry is unavailable");
        }

        smoothTurn.maneuverSpeedMps = cruiseSpeed;
        if (!(smoothTurn.maneuverSpeedMps > 0.0f))
        {
            smoothTurn.maneuverSpeedMps = (std::max)(maneuver.getEntrySpeed(), maneuver.getExitSpeed());
        }
        if (!(smoothTurn.maneuverSpeedMps > 0.0f))
        {
            return Fail("Smooth turn speed is invalid");
        }

        smoothTurn.limits = limits;
        smoothTurn.entrySpeed = maneuver.getEntrySpeed();
        smoothTurn.exitSpeed = maneuver.getExitSpeed();
        smoothTurn.cruiseSpeed = cruiseSpeed;
        smoothTurn.totalDistanceM = smoothTurn.maneuver.GetTravelDistanceMeters(Config::kCellSizeM);
        if (!(smoothTurn.totalDistanceM > 0.0f))
        {
            return Fail("Tracked maneuver distance is invalid");
        }
        smoothTurn.startDistanceM = _drive.GetAverageDistanceMeters();
        smoothTurn.translationWatchdog.Reset(0.0f, millis());
        smoothTurn.expectedCompletionDeadlineMs =
            millis() + static_cast<unsigned long>(2500.0f + (5000.0f * smoothTurn.totalDistanceM));
        return RunLoopSession(&smoothTurn, &Implementation::SmoothTurnLoopTick);
    }

    bool IsInGoalCell(MazeMap::CellCoordinates coords) const
    {
        if (!_maze.HasFoundGoal())
        {
            return false;
        }

        const MazeMap::CellCoordinates goal = _maze.GetGoalLowerLeft();
        const bool xMatch = (coords.GetX() == goal.GetX()) || (coords.GetX() == static_cast<uint8_t>(goal.GetX() + 1));
        const bool yMatch = (coords.GetY() == goal.GetY()) || (coords.GetY() == static_cast<uint8_t>(goal.GetY() + 1));
        return xMatch && yMatch;
    }

    static float ManeuverDistanceMeters(MazeMap::ManeuverCode code)
    {
        return MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, Config::kCellSizeM);
    }

    static float ManeuverSpeedLimit(MazeMap::ManeuverCode code, const MotionLimits& limits, const MazeMap::Vehicle& vehicle)
    {
        const MazeMap::ManeuverInstance maneuver(code, MazeMap::DirectionalLocation());
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (IsStraightCode(code))
        {
            return limits.maxSpeedMps;
        }
        return (std::min)(limits.maxSpeedMps, maneuver.GetSpeedLimit(vehicle));
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code, const MotionLimits& limits) const
    {
        return ManeuverSpeedLimit(code, limits, _speedVehicle);
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code) const
    {
        return ManeuverSpeedLimit(code, FinalLimits());
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, const MazeMap::Vehicle& vehicle, float initialEntrySpeed, float finalExitSpeed)
    {
        if (queue.empty())
        {
            return;
        }

        float boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.getCode(), limits, vehicle);
            if (IsStraightCode(entry.getCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.getCode());
                const float entrySpeed = (std::min)(boundarySpeed, speedLimit);
                const float exitSpeed = (std::min)(entry.getExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.setEntrySpeed(entrySpeed);
                entry.setExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)((std::min)(entry.getEntrySpeed(), boundarySpeed), speedLimit);
                entry.setEntrySpeed(maneuverSpeed);
                entry.setExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }

        float requiredExitSpeed = (std::max)(finalExitSpeed, 0.0f);
        for (int i = static_cast<int>(queue.size()) - 1; i >= 0; --i)
        {
            MazeMap::ManeuverInstance& entry = queue[static_cast<uint16_t>(i)];
            const float speedLimit = ManeuverSpeedLimit(entry.getCode(), limits, vehicle);
            if (IsStraightCode(entry.getCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.getCode());
                const float exitSpeed = (std::min)(entry.getExitSpeed(), (std::min)(requiredExitSpeed, speedLimit));
                const float entrySpeed = (std::min)(entry.getEntrySpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(exitSpeed, distanceM, limits.decelMps2)));
                entry.setEntrySpeed(entrySpeed);
                entry.setExitSpeed(exitSpeed);
                requiredExitSpeed = entrySpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.getEntrySpeed(), (std::min)(requiredExitSpeed, speedLimit));
                entry.setEntrySpeed(maneuverSpeed);
                entry.setExitSpeed(maneuverSpeed);
                requiredExitSpeed = maneuverSpeed;
            }
        }

        boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.getCode(), limits, vehicle);
            if (IsStraightCode(entry.getCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.getCode());
                const float entrySpeed = (std::min)(entry.getEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                const float exitSpeed = (std::min)(entry.getExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.setEntrySpeed(entrySpeed);
                entry.setExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.getEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                entry.setEntrySpeed(maneuverSpeed);
                entry.setExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, float initialEntrySpeed, float finalExitSpeed)
    {
        ApplyAsymmetricQueueLimits(queue, FinalLimits(), _speedVehicle, initialEntrySpeed, finalExitSpeed);
    }
};

namespace MazeMap::App::Internal
{
    MissionModeController::MissionModeController(SharedRobotRuntime& runtime)
        : _impl(std::make_unique<Implementation>(runtime))
    {
    }

    MissionModeController::~MissionModeController() = default;

    bool MissionModeController::BeginMissionRunMode()
    {
        return _impl->BeginMissionRunMode();
    }

    void MissionModeController::RunMissionRunMode()
    {
        _impl->RunMissionRunMode();
    }

    bool MissionModeController::BeginManeuverFileTestMode()
    {
        return _impl->BeginManeuverFileTestMode();
    }

    void MissionModeController::RunManeuverFileTestMode()
    {
        _impl->RunManeuverFileTestMode();
    }

    bool MissionModeController::BeginCorridorRepeatabilityMode()
    {
        return _impl->BeginCorridorRepeatabilityMode();
    }

    void MissionModeController::RunCorridorRepeatabilityMode()
    {
        _impl->RunCorridorRepeatabilityMode();
    }

    bool MissionModeController::BeginPositionAccuracyAuditMode()
    {
        return _impl->BeginPositionAccuracyAuditMode();
    }

    void MissionModeController::RunPositionAccuracyAuditMode()
    {
        _impl->RunPositionAccuracyAuditMode();
    }
}
