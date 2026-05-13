#include "pch.h"
#include "AuxMeasurementModeSupport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace MazeMap::App::Internal::AuxMeasurementModeSupport
{
    namespace
    {
        bool WriteEvent(
            const WriteEventCallback callback,
            void* const context,
            const char* const type,
            const char* const message)
        {
            return (callback != nullptr) && callback(context, type, message);
        }

        bool ReportFailure(
            const FailCallback callback,
            void* const context,
            const char* const message)
        {
            return (callback != nullptr) && callback(context, message);
        }

        void SetKnownMazeCellWalls(
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

        MazeMap::Maze BuildPositionAuditMazeFixture(
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

        bool TryBuildReverseManeuverPath(
            const MazeMap::ManeuverPath& forwardPath,
            MazeMap::ManeuverPath& reversePath)
        {
            reversePath.clear();
            const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
            for (int index = static_cast<int>(forwardPath.GetSize()) - 1; index >= 0; --index)
            {
                if (!reversePath.push_back(
                        maneuverSet.GetReverseCode(forwardPath[static_cast<std::uint16_t>(index)])))
                {
                    reversePath.clear();
                    return false;
                }
            }

            return true;
        }

        bool TryResolvePositionAuditSmoothTurnHalfSteps(
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
    }

    PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry()
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

    bool TryBuildPositionAuditSmoothTurnPaths(
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

    bool TryValidatePositionAuditPath(
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

    MotionLimits CorridorRepeatabilityLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps((std::max)(0.0f, cruiseSpeedMps));
        limits.SetAccelMps2(AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2);
        limits.SetDecelMps2(AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2);
        limits.SetMaxAngularSpeedRadps(Config::kSearchTurnMaxOmegaRadps);
        limits.SetAngularAccelRadps2(Config::kSearchTurnAccelRadps2);
        return limits;
    }

    MotionLimits PositionAccuracyAuditStraightLimits(const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps((std::max)(0.0f, cruiseSpeedMps));
        limits.SetAccelMps2(AuxMeasurementConfig::kPositionAuditAccelMps2);
        limits.SetDecelMps2(AuxMeasurementConfig::kPositionAuditDecelMps2);
        limits.SetMaxAngularSpeedRadps(Config::kSearchTurnMaxOmegaRadps);
        limits.SetAngularAccelRadps2(Config::kSearchTurnAccelRadps2);
        return limits;
    }

    MotionLimits PositionAccuracyAuditTurnLimits() noexcept
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(0.0f);
        limits.SetMaxSpeedMps(0.0f);
        return limits;
    }

    MotionLimits PositionAccuracyAuditCornerLimits(
        const float cruiseSpeedMps,
        const float nominalRadiusM) noexcept
    {
        (void)nominalRadiusM;
        MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        limits.SetMaxAngularSpeedRadps(AuxMeasurementConfig::kPositionAuditCornerMaxOmegaRadps);
        return limits;
    }

    float UtilityModeManeuverDistanceMeters(const MazeMap::ManeuverCode code)
    {
        return MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, Config::kCellSizeM);
    }

    bool LogCorridorRepeatabilityMetadata(
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context)
    {
        char line[160] = {};
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "Place the robot in a 5-cell enclosed row like a mission start. This routine runs startup wall calibration, drives to the far end and back at several speeds, and logs closure error at the start cell."))
        {
            return ReportFailure(fail, context, "Unable to write corridor repeatability summary");
        }

        std::snprintf(
            line,
            sizeof(line),
            "row_cell_count,%u",
            static_cast<unsigned>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount));
        if (!WriteEvent(writeEvent, context, "corridor_repeatability", line))
        {
            return ReportFailure(fail, context, "Unable to write corridor repeatability metadata");
        }

        const float outDistanceM =
            (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount > 0U) ?
            (Config::kCellSizeM * static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)) :
            0.0f;
        std::snprintf(line, sizeof(line), "out_distance_m,%.6f", outDistanceM);
        if (!WriteEvent(writeEvent, context, "corridor_repeatability", line))
        {
            return ReportFailure(fail, context, "Unable to write corridor repeatability metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;turn_max_omega_radps,%.6f;turn_accel_radps2,%.6f",
            AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2,
            Config::kSearchTurnMaxOmegaRadps,
            Config::kSearchTurnAccelRadps2);
        if (!WriteEvent(writeEvent, context, "corridor_repeatability", line))
        {
            return ReportFailure(fail, context, "Unable to write corridor repeatability metadata");
        }

        for (std::uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            std::snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex]);
            if (!WriteEvent(writeEvent, context, "corridor_repeatability_speed", line))
            {
                return ReportFailure(fail, context, "Unable to write corridor repeatability speed metadata");
            }
        }

        return true;
    }

    bool LogPositionAccuracyAuditMetadata(
        const PositionAuditFixtureGeometry& geometry,
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context)
    {
        char line[320] = {};
        std::snprintf(
            line,
            sizeof(line),
            "Build a one-cell-wide fixture: normal mission start, a %u-cell north corridor including the start and corner cells, and a %u-cell east extension beyond that corner with solid side walls. All following phases reuse this same fixed geometry.",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount));
        if (!WriteEvent(writeEvent, context, "summary", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "position_straight_result isolates wheel-diameter, straight feedforward, and stop-distance error through north_touch_correction_m, enc_out_err_m, closure_m, and yaw_err_deg."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "position_in_place_turn_result isolates the shared in-place turn profile through yaw_err_deg, effective_track_width_m, and wall_touch_correction_m."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "position_smooth_turn_result compares S90SS and S90LS against nominal_radius_m, measured_radius_m, effective_track_width_m, corridor_err_m, and east_touch_correction_m to expose radius-dependent feedforward error."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "Phase 1 runs S8, centers in the north corner, turns in place to face down, and runs S8 back to start."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "Phase 2 reseats at start, runs S7 + S90SS + S7, centers at the east end, turns to face left, and returns on the reversed maneuver path."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (!WriteEvent(
                writeEvent,
                context,
                "summary",
                "Phase 3 reseats at start, runs S6 + S90LS + S6, recenters at the east end, and returns on the reversed maneuver path."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }
        if (AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled &&
            !WriteEvent(
                writeEvent,
                context,
                "summary",
                "Smooth-turn phases run with the mission fan enabled; the existing 2 s ramp to 80% completes before motion begins so high-speed S90 data reflects the intended downforce state."))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit summary");
        }

        std::snprintf(
            line,
            sizeof(line),
            "north_corridor_cells,%u;east_extension_cells,%u;east_total_cells,%u",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount),
            static_cast<unsigned>(geometry.eastTotalCellCount));
        if (!WriteEvent(writeEvent, context, "position_audit", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;start_settle_ms,%u",
            AuxMeasurementConfig::kPositionAuditAccelMps2,
            AuxMeasurementConfig::kPositionAuditDecelMps2,
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditStartSettleMs));
        if (!WriteEvent(writeEvent, context, "position_audit", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "smooth_turn_fan_enabled,%u;kRacingFanDutyCycle,%.6f;kRacingFanRampMs,%u",
            AuxMeasurementConfig::kPositionAuditSmoothTurnFanEnabled ? 1U : 0U,
            Config::kRacingFanDutyCycle,
            static_cast<unsigned>(Config::kRacingFanRampMs));
        if (!WriteEvent(writeEvent, context, "position_audit", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=1;forward_half_steps=%u;turn=IP180;return_half_steps=%u",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps));
        if (!WriteEvent(writeEvent, context, "position_audit_phase", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=2;forward=%u,S90SS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps));
        if (!WriteEvent(writeEvent, context, "position_audit_phase", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        std::snprintf(
            line,
            sizeof(line),
            "phase=3;forward=%u,S90LS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps));
        if (!WriteEvent(writeEvent, context, "position_audit_phase", line))
        {
            return ReportFailure(fail, context, "Unable to write position accuracy audit metadata");
        }

        for (std::uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            std::snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]);
            if (!WriteEvent(writeEvent, context, "position_audit_straight_speed", line))
            {
                return ReportFailure(fail, context, "Unable to write position accuracy audit speed metadata");
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
            if (!WriteEvent(writeEvent, context, "position_audit_corner_speed", line))
            {
                return ReportFailure(fail, context, "Unable to write position accuracy audit speed metadata");
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
            if (!WriteEvent(writeEvent, context, "position_audit_turn_code", line))
            {
                return ReportFailure(fail, context, "Unable to write position accuracy audit turn metadata");
            }
        }

        return true;
    }

    bool WriteCorridorRepeatabilityResult(
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const MazeMap::VehicleState& startPose,
        const DriveTelemetry& startTelemetry,
        const MazeMap::VehicleState& finalPose,
        const DriveTelemetry& finalTelemetry)
    {
        const float deltaXM = finalPose.GetPositionX() - startPose.GetPositionX();
        const float deltaYM = finalPose.GetPositionY() - startPose.GetPositionY();
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.GetOrientation(), finalPose.GetOrientation());

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
            return ReportFailure(fail, context, "Corridor repeatability result event overflowed");
        }
        if (WriteEvent(writeEvent, context, "corridor_repeatability_result", message))
        {
            return true;
        }
        return ReportFailure(fail, context, "Unable to write corridor repeatability result");
    }

    bool WritePositionStraightAuditResult(
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float northStopErrorM,
        const float northTouchCorrectionM,
        const float encoderOutErrorM,
        const MazeMap::VehicleState& startPose,
        const MazeMap::VehicleState& finalPose)
    {
        const float deltaXM = finalPose.GetPositionX() - startPose.GetPositionX();
        const float deltaYM = finalPose.GetPositionY() - startPose.GetPositionY();
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.GetOrientation(), finalPose.GetOrientation());

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
            return ReportFailure(fail, context, "Position straight result event overflowed");
        }
        if (WriteEvent(writeEvent, context, "position_straight_result", message))
        {
            return true;
        }
        return ReportFailure(fail, context, "Unable to write position straight result");
    }

    bool WritePositionInPlaceTurnAuditResult(
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context,
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
            return ReportFailure(fail, context, "Position in-place turn result event overflowed");
        }
        if (WriteEvent(writeEvent, context, "position_in_place_turn_result", message))
        {
            return true;
        }
        return ReportFailure(fail, context, "Unable to write position in-place turn result");
    }

    bool WritePositionSmoothTurnAuditResult(
        const WriteEventCallback writeEvent,
        const FailCallback fail,
        void* const context,
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
            return ReportFailure(fail, context, "Position smooth turn result event overflowed");
        }
        if (WriteEvent(writeEvent, context, "position_smooth_turn_result", message))
        {
            return true;
        }
        return ReportFailure(fail, context, "Unable to write position smooth turn result");
    }
}
