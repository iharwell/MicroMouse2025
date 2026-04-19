#pragma once

#include "Cell.h"
#include "CellCoordinates.h"
#include "Defines.h"
#include "Direction.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

namespace MazeMap
{
    enum class FrontCalibrationSpinHeadingClass : uint8_t
    {
        Ignore = 0U,
        OpenNorth = 1U,
        Wall = 2U,
    };

    inline FrontCalibrationSpinHeadingClass ClassifyFrontCalibrationSpinHeadingFromNorth(
        float yawRad,
        float northOpenHalfWidthRad,
        float wallMinEastOfNorthRad,
        float wallMaxEastOfNorthRad)
    {
        if (!std::isfinite(yawRad) ||
            !std::isfinite(northOpenHalfWidthRad) ||
            !std::isfinite(wallMinEastOfNorthRad) ||
            !std::isfinite(wallMaxEastOfNorthRad) ||
            northOpenHalfWidthRad < 0.0f ||
            wallMinEastOfNorthRad < northOpenHalfWidthRad ||
            wallMaxEastOfNorthRad < wallMinEastOfNorthRad)
        {
            return FrontCalibrationSpinHeadingClass::Ignore;
        }

        const float eastOfNorthRad = std::remainder(yawRad, TWO_PI_F);
        if (std::fabs(eastOfNorthRad) <= northOpenHalfWidthRad)
        {
            return FrontCalibrationSpinHeadingClass::OpenNorth;
        }

        if (eastOfNorthRad < wallMinEastOfNorthRad || eastOfNorthRad > wallMaxEastOfNorthRad)
        {
            return FrontCalibrationSpinHeadingClass::Ignore;
        }
        return FrontCalibrationSpinHeadingClass::Wall;
    }

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

    inline bool TryComputeWallSegmentCenterWindowM(
        float alongWallCoordinateM,
        float cellSizeM,
        float wallThicknessM,
        float keptFraction,
        float& minCoordinateM,
        float& maxCoordinateM)
    {
        minCoordinateM = 0.0f;
        maxCoordinateM = 0.0f;
        if (!std::isfinite(alongWallCoordinateM) ||
            !std::isfinite(cellSizeM) ||
            !std::isfinite(wallThicknessM) ||
            !std::isfinite(keptFraction) ||
            cellSizeM <= 0.0f ||
            wallThicknessM < 0.0f ||
            keptFraction <= 0.0f ||
            keptFraction > 1.0f)
        {
            return false;
        }

        const float cellBaseCoordinateM =
            std::floor(alongWallCoordinateM / cellSizeM) * cellSizeM;
        const float segmentMinCoordinateM =
            cellBaseCoordinateM + ComputeCellInnerMinCoordinateM(wallThicknessM);
        const float segmentMaxCoordinateM =
            cellBaseCoordinateM + ComputeCellInnerMaxCoordinateM(cellSizeM, wallThicknessM);
        const float segmentSpanM = segmentMaxCoordinateM - segmentMinCoordinateM;
        if (!(segmentSpanM > 0.0f))
        {
            return false;
        }

        const float trimPerSideM = 0.5f * (1.0f - keptFraction) * segmentSpanM;
        minCoordinateM = segmentMinCoordinateM + trimPerSideM;
        maxCoordinateM = segmentMaxCoordinateM - trimPerSideM;
        return
            std::isfinite(minCoordinateM) &&
            std::isfinite(maxCoordinateM) &&
            minCoordinateM <= maxCoordinateM;
    }

    inline bool IsWithinWallSegmentCenterWindowM(
        float alongWallCoordinateM,
        float cellSizeM,
        float wallThicknessM,
        float keptFraction)
    {
        float minCoordinateM = 0.0f;
        float maxCoordinateM = 0.0f;
        if (!TryComputeWallSegmentCenterWindowM(
                alongWallCoordinateM,
                cellSizeM,
                wallThicknessM,
                keptFraction,
                minCoordinateM,
                maxCoordinateM))
        {
            return false;
        }

        return
            std::isfinite(alongWallCoordinateM) &&
            alongWallCoordinateM >= minCoordinateM &&
            alongWallCoordinateM <= maxCoordinateM;
    }

    inline bool TryComputeSideWallObservationSamplePoseM(
        CellCoordinates cell,
        Direction heading,
        float cellSizeM,
        float wallThicknessM,
        float sensorForwardOffsetM,
        float keptFraction,
        uint8_t sampleIndex,
        uint8_t sampleCount,
        float& targetXM,
        float& targetYM)
    {
        targetXM = 0.0f;
        targetYM = 0.0f;
        if (!(std::isfinite(cellSizeM) &&
            std::isfinite(wallThicknessM) &&
            std::isfinite(sensorForwardOffsetM) &&
            std::isfinite(keptFraction) &&
            cellSizeM > 0.0f &&
            wallThicknessM >= 0.0f &&
            sensorForwardOffsetM >= 0.0f &&
            keptFraction > 0.0f &&
            keptFraction <= 1.0f &&
            sampleCount > 0U &&
            sampleIndex < sampleCount))
        {
            return false;
        }

        const bool headingIsVertical = (heading == Up) || (heading == Down);
        const bool headingIsHorizontal = (heading == Left) || (heading == Right);
        if (!(headingIsVertical || headingIsHorizontal))
        {
            return false;
        }

        const float cellBaseAlongWallCoordinateM =
            headingIsVertical ?
            (static_cast<float>(cell.GetY()) * cellSizeM) :
            (static_cast<float>(cell.GetX()) * cellSizeM);
        const float cellCenterAlongWallCoordinateM =
            cellBaseAlongWallCoordinateM + (0.5f * cellSizeM);
        float minCoordinateM = 0.0f;
        float maxCoordinateM = 0.0f;
        if (!TryComputeWallSegmentCenterWindowM(
                cellCenterAlongWallCoordinateM,
                cellSizeM,
                wallThicknessM,
                keptFraction,
                minCoordinateM,
                maxCoordinateM))
        {
            return false;
        }

        const float alpha =
            (sampleCount <= 1U) ?
            0.5f :
            (static_cast<float>(sampleIndex) / static_cast<float>(sampleCount - 1U));
        const float alongWallCoordinateM =
            minCoordinateM + (alpha * (maxCoordinateM - minCoordinateM));

        targetXM = (static_cast<float>(cell.GetX()) + 0.5f) * cellSizeM;
        targetYM = (static_cast<float>(cell.GetY()) + 0.5f) * cellSizeM;
        switch (heading)
        {
        case Up:
            targetYM = alongWallCoordinateM - sensorForwardOffsetM;
            break;
        case Down:
            targetYM = alongWallCoordinateM + sensorForwardOffsetM;
            break;
        case Left:
            targetXM = alongWallCoordinateM + sensorForwardOffsetM;
            break;
        case Right:
            targetXM = alongWallCoordinateM - sensorForwardOffsetM;
            break;
        default:
            return false;
        }

        return
            std::isfinite(targetXM) &&
            std::isfinite(targetYM) &&
            targetXM >= 0.0f &&
            targetYM >= 0.0f;
    }

    inline bool TryComputeSideWallTravelFractionPoseM(
        CellCoordinates cell,
        Direction heading,
        float cellSizeM,
        float sensorForwardOffsetM,
        float cellEntryFraction,
        float& targetXM,
        float& targetYM)
    {
        targetXM = 0.0f;
        targetYM = 0.0f;
        if (!(std::isfinite(cellSizeM) &&
            std::isfinite(sensorForwardOffsetM) &&
            std::isfinite(cellEntryFraction) &&
            cellSizeM > 0.0f &&
            sensorForwardOffsetM >= 0.0f &&
            cellEntryFraction >= 0.0f &&
            cellEntryFraction <= 1.0f))
        {
            return false;
        }

        const float cellBaseXM = static_cast<float>(cell.GetX()) * cellSizeM;
        const float cellBaseYM = static_cast<float>(cell.GetY()) * cellSizeM;
        targetXM = (static_cast<float>(cell.GetX()) + 0.5f) * cellSizeM;
        targetYM = (static_cast<float>(cell.GetY()) + 0.5f) * cellSizeM;
        switch (heading)
        {
        case Up:
            targetYM = cellBaseYM + (cellEntryFraction * cellSizeM) - sensorForwardOffsetM;
            break;
        case Down:
            targetYM = cellBaseYM + ((1.0f - cellEntryFraction) * cellSizeM) + sensorForwardOffsetM;
            break;
        case Left:
            targetXM = cellBaseXM + ((1.0f - cellEntryFraction) * cellSizeM) + sensorForwardOffsetM;
            break;
        case Right:
            targetXM = cellBaseXM + (cellEntryFraction * cellSizeM) - sensorForwardOffsetM;
            break;
        default:
            return false;
        }

        return
            std::isfinite(targetXM) &&
            std::isfinite(targetYM) &&
            targetXM >= 0.0f &&
            targetYM >= 0.0f;
    }

    inline bool TryComputeSideWallObservationPoseM(
        CellCoordinates cell,
        Direction heading,
        float cellSizeM,
        float sensorForwardOffsetM,
        float& targetXM,
        float& targetYM)
    {
        targetXM = 0.0f;
        targetYM = 0.0f;
        if (!(std::isfinite(cellSizeM) &&
            std::isfinite(sensorForwardOffsetM) &&
            cellSizeM > 0.0f &&
            sensorForwardOffsetM >= 0.0f))
        {
            return false;
        }

        targetXM = (static_cast<float>(cell.GetX()) + 0.5f) * cellSizeM;
        targetYM = (static_cast<float>(cell.GetY()) + 0.5f) * cellSizeM;
        switch (heading)
        {
        case Up:
            targetYM -= sensorForwardOffsetM;
            break;
        case Down:
            targetYM += sensorForwardOffsetM;
            break;
        case Left:
            targetXM += sensorForwardOffsetM;
            break;
        case Right:
            targetXM -= sensorForwardOffsetM;
            break;
        default:
            return false;
        }

        return
            std::isfinite(targetXM) &&
            std::isfinite(targetYM) &&
            targetXM >= 0.0f &&
            targetYM >= 0.0f;
    }

    inline bool TryComputeSignedTravelToCellCenterAlongHeadingM(
        const CellCoordinates& cell,
        float cellSizeM,
        float poseXMeters,
        float poseYMeters,
        float headingXMeters,
        float headingYMeters,
        float& signedTravelM)
    {
        signedTravelM = 0.0f;
        if (!(std::isfinite(cellSizeM) &&
            std::isfinite(poseXMeters) &&
            std::isfinite(poseYMeters) &&
            std::isfinite(headingXMeters) &&
            std::isfinite(headingYMeters) &&
            cellSizeM > 0.0f))
        {
            return false;
        }

        const float headingMagnitude =
            std::sqrt((headingXMeters * headingXMeters) + (headingYMeters * headingYMeters));
        if (!(std::isfinite(headingMagnitude) && headingMagnitude > 1.0e-4f))
        {
            return false;
        }

        const float normalizedHeadingX = headingXMeters / headingMagnitude;
        const float normalizedHeadingY = headingYMeters / headingMagnitude;
        const float centerXMeters = (static_cast<float>(cell.GetX()) + 0.5f) * cellSizeM;
        const float centerYMeters = (static_cast<float>(cell.GetY()) + 0.5f) * cellSizeM;
        const float deltaXMeters = centerXMeters - poseXMeters;
        const float deltaYMeters = centerYMeters - poseYMeters;
        signedTravelM =
            (deltaXMeters * normalizedHeadingX) +
            (deltaYMeters * normalizedHeadingY);
        return std::isfinite(signedTravelM);
    }

    inline bool IsValidCalibrationCenterCoordinateM(float coordinateM)
    {
        return std::isfinite(coordinateM) && coordinateM >= 0.0f;
    }

    inline bool TrySelectCalibrationDirectionTowardTarget(
        float startCoordinateM,
        float targetCoordinateM,
        float toleranceM,
        Direction negativeDirection,
        Direction positiveDirection,
        Direction& selectedDirection)
    {
        if (!(std::isfinite(startCoordinateM) &&
            std::isfinite(targetCoordinateM) &&
            std::isfinite(toleranceM) &&
            toleranceM >= 0.0f))
        {
            return false;
        }

        const float deltaCoordinateM = targetCoordinateM - startCoordinateM;
        if (std::fabs(deltaCoordinateM) <= toleranceM)
        {
            return false;
        }

        selectedDirection = (deltaCoordinateM > 0.0f) ? positiveDirection : negativeDirection;
        return true;
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

    inline float ComputeCalibrationSafeMinCenterXFromWestWallM(
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

        return ComputeCellInnerMinCoordinateM(wallThicknessM) + trailingOffsetM + clearanceM;
    }

    inline float ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(
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
        return ComputeCalibrationSafeMinCenterXFromWestWallM(
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

    inline float ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(
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
        return wallTouchContactStandoffM - farOffsetM - (static_cast<float>(pointIndex) * stepM);
    }

    inline float ComputeStartupWallCalibrationFarthestFrontSampleCenterFromEastWallXM(
        float wallTouchContactStandoffM,
        float frontFarOffsetM,
        float frontStepM,
        unsigned int pointCount)
    {
        if (pointCount == 0U)
        {
            return ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(
                wallTouchContactStandoffM,
                frontFarOffsetM,
                frontStepM,
                0U);
        }

        return ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(
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

        if (pointCount == 1U)
        {
            return startCenterXM;
        }

        const float normalizedIndex = static_cast<float>((std::min)(pointIndex, pointCount - 1U)) / static_cast<float>(pointCount - 1U);
        return startCenterXM + ((farthestCenterXM - startCenterXM) * normalizedIndex);
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

    inline bool IsMissionStartupStationaryFromSensors(
        float leftWheelSpeedMps,
        float rightWheelSpeedMps,
        float measuredYawRateRadps,
        float maxAbsLinearSpeedMps,
        float maxAbsAngularSpeedRadps)
    {
        if (!std::isfinite(leftWheelSpeedMps) ||
            !std::isfinite(rightWheelSpeedMps))
        {
            return false;
        }

        const float measuredLinearSpeedMps = 0.5f * (leftWheelSpeedMps + rightWheelSpeedMps);
        return IsMissionStartupStationarySample(
            measuredLinearSpeedMps,
            measuredYawRateRadps,
            leftWheelSpeedMps,
            rightWheelSpeedMps,
            maxAbsLinearSpeedMps,
            maxAbsAngularSpeedRadps);
    }

    inline bool IsMissionStartupStationaryFromEncoderWindow(
        float leftEncoderTravelM,
        float rightEncoderTravelM,
        float windowDurationSeconds,
        float measuredYawRateRadps,
        float maxAbsLinearSpeedMps,
        float maxAbsAngularSpeedRadps)
    {
        if (!std::isfinite(leftEncoderTravelM) ||
            !std::isfinite(rightEncoderTravelM) ||
            !std::isfinite(windowDurationSeconds) ||
            windowDurationSeconds <= 0.0f)
        {
            return false;
        }

        const float leftWheelSpeedMps = leftEncoderTravelM / windowDurationSeconds;
        const float rightWheelSpeedMps = rightEncoderTravelM / windowDurationSeconds;
        const float measuredLinearSpeedMps = 0.5f * (leftWheelSpeedMps + rightWheelSpeedMps);
        return IsMissionStartupStationarySample(
            measuredLinearSpeedMps,
            measuredYawRateRadps,
            leftWheelSpeedMps,
            rightWheelSpeedMps,
            maxAbsLinearSpeedMps,
            maxAbsAngularSpeedRadps);
    }

}
