#pragma once

#include "Cell.h"
#include "CellCoordinates.h"
#include "Defines.h"
#include "Direction.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    inline float ComputeCellWallFaceInsetM(float wallThicknessM)
    {
        if (!std::isfinite(wallThicknessM) || wallThicknessM < 0.0f)
        {
            return 0.0f;
        }

        return 0.5f * wallThicknessM;
    }

    inline float ComputeCellInnerMinCoordinateM(float wallThicknessM)
    {
        return ComputeCellWallFaceInsetM(wallThicknessM);
    }

    inline float ComputeCellInnerMaxCoordinateM(float cellSizeM, float wallThicknessM)
    {
        if (!std::isfinite(cellSizeM) || cellSizeM <= 0.0f)
        {
            return 0.0f;
        }

        const float insetM = ComputeCellWallFaceInsetM(wallThicknessM);
        const float maxCoordinateM = cellSizeM - insetM;
        return (maxCoordinateM > insetM) ? maxCoordinateM : insetM;
    }

    inline float ComputeCellInnerSpanM(float cellSizeM, float wallThicknessM)
    {
        const float minCoordinateM = ComputeCellInnerMinCoordinateM(wallThicknessM);
        const float maxCoordinateM = ComputeCellInnerMaxCoordinateM(cellSizeM, wallThicknessM);
        return (maxCoordinateM > minCoordinateM) ? (maxCoordinateM - minCoordinateM) : 0.0f;
    }

    inline float ComputeWallTouchPoseFromWestWallM(float wallThicknessM, float contactStandoffM)
    {
        if (!std::isfinite(contactStandoffM) || contactStandoffM < 0.0f)
        {
            return 0.0f;
        }

        return ComputeCellInnerMinCoordinateM(wallThicknessM) + contactStandoffM;
    }

    inline float ComputeWallTouchPoseFromEastWallM(float cellSizeM, float wallThicknessM, float contactStandoffM)
    {
        if (!std::isfinite(contactStandoffM) || contactStandoffM < 0.0f)
        {
            return 0.0f;
        }

        const float innerMaxCoordinateM = ComputeCellInnerMaxCoordinateM(cellSizeM, wallThicknessM);
        return (innerMaxCoordinateM > contactStandoffM) ? (innerMaxCoordinateM - contactStandoffM) : 0.0f;
    }

    inline float ComputeWallTouchPoseFromSouthWallM(float wallThicknessM, float contactStandoffM)
    {
        return ComputeWallTouchPoseFromWestWallM(wallThicknessM, contactStandoffM);
    }

    inline float ComputeWallTouchPoseFromNorthWallM(float cellSizeM, float wallThicknessM, float contactStandoffM)
    {
        return ComputeWallTouchPoseFromEastWallM(cellSizeM, wallThicknessM, contactStandoffM);
    }

    inline float ComputeCalibrationSafeMaxCenterXFromEastWallM(
        float cellSizeM,
        float wallThicknessM,
        float trailingOffsetM,
        float clearanceM)
    {
        if (!std::isfinite(trailingOffsetM) ||
            !std::isfinite(clearanceM) ||
            trailingOffsetM < 0.0f ||
            clearanceM < 0.0f)
        {
            return 0.0f;
        }

        const float innerMaxCoordinateM = ComputeCellInnerMaxCoordinateM(cellSizeM, wallThicknessM);
        const float safeMaxCenterXM = innerMaxCoordinateM - trailingOffsetM - clearanceM;
        return (safeMaxCenterXM > 0.0f) ? safeMaxCenterXM : 0.0f;
    }

    inline float ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(
        float cellSizeM,
        float wallThicknessM,
        float trailingOffsetM,
        float halfWidthM,
        float clearanceM)
    {
        if (!std::isfinite(trailingOffsetM) ||
            !std::isfinite(halfWidthM) ||
            !std::isfinite(clearanceM) ||
            trailingOffsetM < 0.0f ||
            halfWidthM < 0.0f ||
            clearanceM < 0.0f)
        {
            return 0.0f;
        }

        const float rearCornerSweepRadiusM = std::sqrt((trailingOffsetM * trailingOffsetM) + (halfWidthM * halfWidthM));
        return ComputeCalibrationSafeMaxCenterXFromEastWallM(
            cellSizeM,
            wallThicknessM,
            rearCornerSweepRadiusM,
            clearanceM);
    }

    inline float ComputeMissionStartCenterAdvanceM(float cellSizeM, float wallTouchContactStandoffM)
    {
        if (!std::isfinite(cellSizeM) || cellSizeM <= 0.0f)
        {
            return 0.0f;
        }
        if (!std::isfinite(wallTouchContactStandoffM) || wallTouchContactStandoffM < 0.0f)
        {
            return 0.0f;
        }

        const float advanceM = (0.5f * cellSizeM) - wallTouchContactStandoffM;
        return (advanceM > 0.0f) ? advanceM : 0.0f;
    }

    inline float ComputeMissionStartTurnClearanceM(float cellSizeM, float wallTouchContactStandoffM)
    {
        return ComputeMissionStartCenterAdvanceM(cellSizeM, wallTouchContactStandoffM);
    }

    inline float ComputeStartupWallCalibrationFrontSampleCenterXM(
        float wallTouchContactStandoffM,
        float frontFarOffsetM,
        float frontStepM,
        unsigned int pointIndex)
    {
        if (!std::isfinite(wallTouchContactStandoffM) || wallTouchContactStandoffM < 0.0f)
        {
            return 0.0f;
        }

        const float farOffsetM = (std::isfinite(frontFarOffsetM) && frontFarOffsetM > 0.0f) ? frontFarOffsetM : 0.0f;
        const float stepM = (std::isfinite(frontStepM) && frontStepM > 0.0f) ? frontStepM : 0.0f;
        return wallTouchContactStandoffM + farOffsetM + (static_cast<float>(pointIndex) * stepM);
    }

    inline float ComputeStartupWallCalibrationFarthestFrontSampleCenterXM(
        float wallTouchContactStandoffM,
        float frontFarOffsetM,
        float frontStepM,
        unsigned int pointCount)
    {
        if (pointCount == 0U)
        {
            return ComputeStartupWallCalibrationFrontSampleCenterXM(
                wallTouchContactStandoffM,
                frontFarOffsetM,
                frontStepM,
                0U);
        }

        return ComputeStartupWallCalibrationFrontSampleCenterXM(
            wallTouchContactStandoffM,
            frontFarOffsetM,
            frontStepM,
            pointCount - 1U);
    }

    inline float ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(
        float startCenterXM,
        float farthestCenterXM,
        unsigned int pointCount,
        unsigned int pointIndex)
    {
        if (!std::isfinite(startCenterXM) || !std::isfinite(farthestCenterXM) || pointCount == 0U)
        {
            return 0.0f;
        }

        const float spanEndXM = (farthestCenterXM >= startCenterXM) ? farthestCenterXM : startCenterXM;
        if (pointCount == 1U)
        {
            return startCenterXM;
        }

        const float normalizedIndex = static_cast<float>((std::min)(pointIndex, pointCount - 1U)) / static_cast<float>(pointCount - 1U);
        return startCenterXM + ((spanEndXM - startCenterXM) * normalizedIndex);
    }

    inline bool TryGetKnownMissionStartWallState(
        const CellCoordinates& coords,
        Direction direction,
        WallState& state)
    {
        if (!(coords == CellCoordinates(0, 0)))
        {
            return false;
        }

        switch (direction)
        {
        case Direction::Up:
            state = WallState::NoWall;
            return true;
        case Direction::Down:
        case Direction::Left:
        case Direction::Right:
            state = WallState::Wall;
            return true;
        default:
            return false;
        }
    }

    inline bool IsWallTouchContactSample(
        float traveledDistanceM,
        float linearSpeedMps,
        float minApproachDistanceM,
        float minTimedLatchTravelM,
        float maxAbsContactSpeedMps,
        unsigned long elapsedMs,
        unsigned long minCommandTimeMs)
    {
        if (!std::isfinite(traveledDistanceM) ||
            !std::isfinite(linearSpeedMps) ||
            !std::isfinite(minApproachDistanceM) ||
            !std::isfinite(minTimedLatchTravelM) ||
            !std::isfinite(maxAbsContactSpeedMps) ||
            minApproachDistanceM < 0.0f ||
            minTimedLatchTravelM < 0.0f ||
            maxAbsContactSpeedMps <= 0.0f)
        {
            return false;
        }

        const bool lowSpeedContact = std::fabs(linearSpeedMps) <= maxAbsContactSpeedMps;
        const bool fullApproachComplete = traveledDistanceM >= minApproachDistanceM;
        const bool timedShortApproachComplete =
            (elapsedMs >= minCommandTimeMs) &&
            (traveledDistanceM >= minTimedLatchTravelM);

        return lowSpeedContact && (fullApproachComplete || timedShortApproachComplete);
    }

    inline float ComputeWallTouchMinimumLatchTravelM(
        float expectedTravelM,
        float minApproachDistanceM,
        float latchSlackM)
    {
        if (!std::isfinite(expectedTravelM) ||
            !std::isfinite(minApproachDistanceM) ||
            !std::isfinite(latchSlackM) ||
            minApproachDistanceM < 0.0f ||
            latchSlackM < 0.0f)
        {
            return 0.0f;
        }

        const float geometryDrivenMinimumM =
            (expectedTravelM > latchSlackM) ?
            (expectedTravelM - latchSlackM) :
            0.0f;
        const float requestedMinimumM = (std::max)(minApproachDistanceM, geometryDrivenMinimumM);
        if (!(expectedTravelM > 0.0f))
        {
            return requestedMinimumM;
        }

        return (std::min)(requestedMinimumM, expectedTravelM);
    }

    inline bool IsWallTouchSeatedSample(
        float traveledDistanceM,
        float minLatchTravelM,
        float linearSpeedMps,
        float angularSpeedRadps,
        float leftWheelSpeedMps,
        float rightWheelSpeedMps,
        float maxAbsLinearSpeedMps,
        float maxAbsAngularSpeedRadps,
        float maxAbsWheelSpeedMps)
    {
        if (!std::isfinite(traveledDistanceM) ||
            !std::isfinite(minLatchTravelM) ||
            !std::isfinite(linearSpeedMps) ||
            !std::isfinite(angularSpeedRadps) ||
            !std::isfinite(leftWheelSpeedMps) ||
            !std::isfinite(rightWheelSpeedMps) ||
            !std::isfinite(maxAbsLinearSpeedMps) ||
            !std::isfinite(maxAbsAngularSpeedRadps) ||
            !std::isfinite(maxAbsWheelSpeedMps) ||
            minLatchTravelM < 0.0f ||
            maxAbsLinearSpeedMps <= 0.0f ||
            maxAbsAngularSpeedRadps <= 0.0f ||
            maxAbsWheelSpeedMps <= 0.0f)
        {
            return false;
        }

        return (traveledDistanceM >= minLatchTravelM) &&
            (std::fabs(linearSpeedMps) <= maxAbsLinearSpeedMps) &&
            (std::fabs(angularSpeedRadps) <= maxAbsAngularSpeedRadps) &&
            (std::fabs(leftWheelSpeedMps) <= maxAbsWheelSpeedMps) &&
            (std::fabs(rightWheelSpeedMps) <= maxAbsWheelSpeedMps);
    }

    inline bool IsMissionStartupStationarySample(
        float linearSpeedMps,
        float angularSpeedRadps,
        float leftWheelSpeedMps,
        float rightWheelSpeedMps,
        float maxAbsLinearSpeedMps,
        float maxAbsAngularSpeedRadps)
    {
        if (!std::isfinite(linearSpeedMps) ||
            !std::isfinite(angularSpeedRadps) ||
            !std::isfinite(leftWheelSpeedMps) ||
            !std::isfinite(rightWheelSpeedMps) ||
            !std::isfinite(maxAbsLinearSpeedMps) ||
            !std::isfinite(maxAbsAngularSpeedRadps) ||
            maxAbsLinearSpeedMps <= 0.0f ||
            maxAbsAngularSpeedRadps <= 0.0f)
        {
            return false;
        }

        return (std::fabs(linearSpeedMps) <= maxAbsLinearSpeedMps) &&
            (std::fabs(angularSpeedRadps) <= maxAbsAngularSpeedRadps) &&
            (std::fabs(leftWheelSpeedMps) <= maxAbsLinearSpeedMps) &&
            (std::fabs(rightWheelSpeedMps) <= maxAbsLinearSpeedMps);
    }

    inline bool ShouldReleaseWallTouchSeat(
        float driveCommand,
        float minimumReleaseDriveCommand,
        unsigned long elapsedMs,
        unsigned long minimumSkidMs,
        bool seatReleaseMotionDetected)
    {
        return
            std::isfinite(driveCommand) &&
            std::isfinite(minimumReleaseDriveCommand) &&
            (driveCommand >= minimumReleaseDriveCommand) &&
            (elapsedMs >= minimumSkidMs) &&
            seatReleaseMotionDetected;
    }
}
