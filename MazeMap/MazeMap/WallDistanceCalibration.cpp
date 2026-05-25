#include "pch.h"
#include "WallDistanceCalibration.h"

#include "CoreConfig.h"
#include "DiagonalWallCentering.h"
#include "Vehicle.h"
#include "WallDetectionThresholds.h"

#include <algorithm>
#include <cmath>

WallDistanceCalibration::WallDistanceCalibration()
    : _frontLeftCurve{}
    , _frontRightCurve{}
    , _sideLeftCurve{}
    , _sideRightCurve{}
    , _expectedSideWallDistanceM(MazeMap::Config::kExpectedSideWallDistanceM)
{
}

void WallDistanceCalibration::Clear()
{
    _frontLeftCurve.Clear();
    _frontRightCurve.Clear();
    _sideLeftCurve.Clear();
    _sideRightCurve.Clear();
    InvalidateFrontSignalModelCache();
    _frontWallBaselineDifferentialLight[0] = 0.0f;
    _frontWallBaselineDifferentialLight[1] = 0.0f;
    _frontWallBaselineValid[0] = false;
    _frontWallBaselineValid[1] = false;
    _frontWallBaselineDifferentialLightLow[0] = 0.0f;
    _frontWallBaselineDifferentialLightLow[1] = 0.0f;
    _frontWallBaselineDifferentialLightHigh[0] = 0.0f;
    _frontWallBaselineDifferentialLightHigh[1] = 0.0f;
    _frontWallBaselineBandValid[0] = false;
    _frontWallBaselineBandValid[1] = false;
    _frontWallWeakestCalibrationMeasuredValue[0] = 0.0f;
    _frontWallWeakestCalibrationMeasuredValue[1] = 0.0f;
    _frontWallWeakestCalibrationDifferentialLightLow[0] = 0.0f;
    _frontWallWeakestCalibrationDifferentialLightLow[1] = 0.0f;
    _frontWallWeakestCalibrationDifferentialLightHigh[0] = 0.0f;
    _frontWallWeakestCalibrationDifferentialLightHigh[1] = 0.0f;
    _frontWallWeakestCalibrationBandValid[0] = false;
    _frontWallWeakestCalibrationBandValid[1] = false;
    _frontWallDirectOnRiseThreshold[0] = 0.0f;
    _frontWallDirectOnRiseThreshold[1] = 0.0f;
    _frontWallDirectOffRiseThreshold[0] = 0.0f;
    _frontWallDirectOffRiseThreshold[1] = 0.0f;
    _frontWallDirectSignalBaseline[0] = 0.0f;
    _frontWallDirectSignalBaseline[1] = 0.0f;
    _frontWallDirectThresholdValid[0] = false;
    _frontWallDirectThresholdValid[1] = false;
    _sideWallBaselineDifferentialLight[0] = 0.0f;
    _sideWallBaselineDifferentialLight[1] = 0.0f;
    _sideWallBaselineValid[0] = false;
    _sideWallBaselineValid[1] = false;
    _sideWallBaselineDifferentialLightLow[0] = 0.0f;
    _sideWallBaselineDifferentialLightLow[1] = 0.0f;
    _sideWallBaselineDifferentialLightHigh[0] = 0.0f;
    _sideWallBaselineDifferentialLightHigh[1] = 0.0f;
    _sideWallBaselineBandValid[0] = false;
    _sideWallBaselineBandValid[1] = false;
    _sideWallReferenceDifferentialLight[0] = 0.0f;
    _sideWallReferenceDifferentialLight[1] = 0.0f;
    _sideWallReferenceValid[0] = false;
    _sideWallReferenceValid[1] = false;
    _sideWallReferenceDifferentialLightLow[0] = 0.0f;
    _sideWallReferenceDifferentialLightLow[1] = 0.0f;
    _sideWallReferenceDifferentialLightHigh[0] = 0.0f;
    _sideWallReferenceDifferentialLightHigh[1] = 0.0f;
    _sideWallReferenceBandValid[0] = false;
    _sideWallReferenceBandValid[1] = false;
    _sideWallReferenceDistanceM[0] = 0.0f;
    _sideWallReferenceDistanceM[1] = 0.0f;
    _sideWallReferenceDistanceValid[0] = false;
    _sideWallReferenceDistanceValid[1] = false;
    _expectedSideWallDistanceM = MazeMap::Config::kExpectedSideWallDistanceM;
}

bool WallDistanceCalibration::AddFrontLeftPoint(float measuredValue, float actualDistanceM, float ambientLight)
{
    return AddFrontPoint(kFrontLeftIndex, _frontLeftCurve, measuredValue, actualDistanceM, ambientLight);
}

bool WallDistanceCalibration::AddFrontRightPoint(float measuredValue, float actualDistanceM, float ambientLight)
{
    return AddFrontPoint(kFrontRightIndex, _frontRightCurve, measuredValue, actualDistanceM, ambientLight);
}

bool WallDistanceCalibration::AddSidePoint(
    MazeMap::RelativeDirection side,
    float measuredValue,
    float actualDistanceM,
    float ambientLight)
{
    return IsSideDirection(side) && AddSidePointAt(SideWallIndex(side), measuredValue, actualDistanceM, ambientLight);
}

float WallDistanceCalibration::ApplyFrontLeft(float measuredValue, float fallbackDistanceM) const
{
    return ApplyCurve(
        _frontLeftCurve,
        MazeMap::WallSensorCalibrationMode::DirectInterpolation,
        measuredValue,
        fallbackDistanceM);
}

float WallDistanceCalibration::ApplyFrontRight(float measuredValue, float fallbackDistanceM) const
{
    return ApplyCurve(
        _frontRightCurve,
        MazeMap::WallSensorCalibrationMode::DirectInterpolation,
        measuredValue,
        fallbackDistanceM);
}

float WallDistanceCalibration::ApplySide(
    MazeMap::RelativeDirection side,
    float measuredValue,
    float fallbackDistanceM) const
{
    if (!IsSideDirection(side))
    {
        return fallbackDistanceM;
    }

    return ApplyCurve(
        SideCurve(side),
        MazeMap::WallSensorCalibrationMode::DistanceOffset,
        measuredValue,
        fallbackDistanceM);
}

void WallDistanceCalibration::SetExpectedSideWallDistanceM(float expectedDistanceM)
{
    if (std::isfinite(expectedDistanceM) && expectedDistanceM > 0.0f)
    {
        _expectedSideWallDistanceM = expectedDistanceM;
    }
}

float WallDistanceCalibration::GetExpectedSideWallDistanceM() const
{
    return _expectedSideWallDistanceM;
}

void WallDistanceCalibration::SetFrontLeftWallBaselineDifferentialLight(float differentialLight)
{
    SetFrontWallBaselineDifferentialLight(kFrontLeftIndex, differentialLight);
}

void WallDistanceCalibration::SetFrontRightWallBaselineDifferentialLight(float differentialLight)
{
    SetFrontWallBaselineDifferentialLight(kFrontRightIndex, differentialLight);
}

void WallDistanceCalibration::SetFrontLeftWallBaselineDifferentialLightBand(
    float lowDifferentialLight,
    float highDifferentialLight)
{
    SetFrontWallBaselineDifferentialLightBand(kFrontLeftIndex, lowDifferentialLight, highDifferentialLight);
}

void WallDistanceCalibration::SetFrontRightWallBaselineDifferentialLightBand(
    float lowDifferentialLight,
    float highDifferentialLight)
{
    SetFrontWallBaselineDifferentialLightBand(kFrontRightIndex, lowDifferentialLight, highDifferentialLight);
}

bool WallDistanceCalibration::TryGetFrontLeftWallBaselineDifferentialLight(float& differentialLight) const
{
    return TryGetFrontWallBaselineDifferentialLight(kFrontLeftIndex, differentialLight);
}

bool WallDistanceCalibration::TryGetFrontRightWallBaselineDifferentialLight(float& differentialLight) const
{
    return TryGetFrontWallBaselineDifferentialLight(kFrontRightIndex, differentialLight);
}

bool WallDistanceCalibration::TryGetFrontLeftWallBaselineDifferentialLightBand(
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    return TryGetFrontWallBaselineDifferentialLightBand(kFrontLeftIndex, lowDifferentialLight, highDifferentialLight);
}

bool WallDistanceCalibration::TryGetFrontRightWallBaselineDifferentialLightBand(
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    return TryGetFrontWallBaselineDifferentialLightBand(kFrontRightIndex, lowDifferentialLight, highDifferentialLight);
}

void WallDistanceCalibration::SetFrontLeftWeakestCalibrationDifferentialLightBand(
    float measuredValue,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    SetFrontWeakestCalibrationDifferentialLightBand(
        kFrontLeftIndex,
        measuredValue,
        lowDifferentialLight,
        highDifferentialLight);
}

void WallDistanceCalibration::SetFrontRightWeakestCalibrationDifferentialLightBand(
    float measuredValue,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    SetFrontWeakestCalibrationDifferentialLightBand(
        kFrontRightIndex,
        measuredValue,
        lowDifferentialLight,
        highDifferentialLight);
}

bool WallDistanceCalibration::TryGetFrontLeftWeakestCalibrationDifferentialLightBand(
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    return TryGetFrontWeakestCalibrationDifferentialLightBand(
        kFrontLeftIndex,
        lowDifferentialLight,
        highDifferentialLight);
}

bool WallDistanceCalibration::TryGetFrontRightWeakestCalibrationDifferentialLightBand(
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    return TryGetFrontWeakestCalibrationDifferentialLightBand(
        kFrontRightIndex,
        lowDifferentialLight,
        highDifferentialLight);
}

void WallDistanceCalibration::SetFrontLeftDirectRiseThresholds(
    float signalBaseline,
    float onRiseThreshold,
    float offRiseThreshold)
{
    SetFrontDirectRiseThresholds(kFrontLeftIndex, signalBaseline, onRiseThreshold, offRiseThreshold);
}

void WallDistanceCalibration::SetFrontRightDirectRiseThresholds(
    float signalBaseline,
    float onRiseThreshold,
    float offRiseThreshold)
{
    SetFrontDirectRiseThresholds(kFrontRightIndex, signalBaseline, onRiseThreshold, offRiseThreshold);
}

bool WallDistanceCalibration::TryGetFrontLeftDirectRiseThresholds(
    float& signalBaseline,
    float& onRiseThreshold,
    float& offRiseThreshold) const
{
    return TryGetFrontDirectRiseThresholds(kFrontLeftIndex, signalBaseline, onRiseThreshold, offRiseThreshold);
}

bool WallDistanceCalibration::TryGetFrontRightDirectRiseThresholds(
    float& signalBaseline,
    float& onRiseThreshold,
    float& offRiseThreshold) const
{
    return TryGetFrontDirectRiseThresholds(kFrontRightIndex, signalBaseline, onRiseThreshold, offRiseThreshold);
}

void WallDistanceCalibration::SetFrontWallBaselineDifferentialLight(uint8_t frontIndex, float differentialLight)
{
    if ((frontIndex > kFrontRightIndex) ||
        !std::isfinite(differentialLight) ||
        differentialLight < 0.0f)
    {
        return;
    }

    _frontWallBaselineDifferentialLight[frontIndex] = differentialLight;
    _frontWallBaselineValid[frontIndex] = true;
}

void WallDistanceCalibration::SetFrontWallBaselineDifferentialLightBand(
    uint8_t frontIndex,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    if ((frontIndex > kFrontRightIndex) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight < 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    _frontWallBaselineDifferentialLightLow[frontIndex] = lowDifferentialLight;
    _frontWallBaselineDifferentialLightHigh[frontIndex] = highDifferentialLight;
    _frontWallBaselineBandValid[frontIndex] = true;
}

bool WallDistanceCalibration::TryGetFrontWallBaselineDifferentialLight(uint8_t frontIndex, float& differentialLight) const
{
    differentialLight = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (!_frontWallBaselineValid[frontIndex])
    {
        return false;
    }

    differentialLight = _frontWallBaselineDifferentialLight[frontIndex];
    return std::isfinite(differentialLight) && differentialLight >= 0.0f;
}

bool WallDistanceCalibration::TryGetFrontWallBaselineDifferentialLightBand(
    uint8_t frontIndex,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (!_frontWallBaselineBandValid[frontIndex])
    {
        return false;
    }

    lowDifferentialLight = _frontWallBaselineDifferentialLightLow[frontIndex];
    highDifferentialLight = _frontWallBaselineDifferentialLightHigh[frontIndex];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight >= 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

void WallDistanceCalibration::SetFrontWeakestCalibrationDifferentialLightBand(
    uint8_t frontIndex,
    float measuredValue,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    if ((frontIndex > kFrontRightIndex) ||
        !std::isfinite(measuredValue) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        measuredValue <= 0.0f ||
        lowDifferentialLight <= 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    if (_frontWallWeakestCalibrationBandValid[frontIndex] &&
        (measuredValue > (_frontWallWeakestCalibrationMeasuredValue[frontIndex] + 0.001f)))
    {
        return;
    }

    _frontWallWeakestCalibrationMeasuredValue[frontIndex] = measuredValue;
    _frontWallWeakestCalibrationDifferentialLightLow[frontIndex] = lowDifferentialLight;
    _frontWallWeakestCalibrationDifferentialLightHigh[frontIndex] = highDifferentialLight;
    _frontWallWeakestCalibrationBandValid[frontIndex] = true;
}

bool WallDistanceCalibration::TryGetFrontWeakestCalibrationDifferentialLightBand(
    uint8_t frontIndex,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (!_frontWallWeakestCalibrationBandValid[frontIndex])
    {
        return false;
    }

    lowDifferentialLight = _frontWallWeakestCalibrationDifferentialLightLow[frontIndex];
    highDifferentialLight = _frontWallWeakestCalibrationDifferentialLightHigh[frontIndex];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight > 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

void WallDistanceCalibration::SetFrontDirectRiseThresholds(
    uint8_t frontIndex,
    float signalBaseline,
    float onRiseThreshold,
    float offRiseThreshold)
{
    if ((frontIndex > kFrontRightIndex) ||
        !std::isfinite(signalBaseline) ||
        signalBaseline < 0.0f ||
        !std::isfinite(onRiseThreshold) ||
        !std::isfinite(offRiseThreshold) ||
        onRiseThreshold <= 0.0f ||
        offRiseThreshold <= 0.0f ||
        offRiseThreshold >= onRiseThreshold)
    {
        return;
    }

    _frontWallDirectSignalBaseline[frontIndex] = signalBaseline;
    _frontWallDirectOnRiseThreshold[frontIndex] = onRiseThreshold;
    _frontWallDirectOffRiseThreshold[frontIndex] = offRiseThreshold;
    _frontWallDirectThresholdValid[frontIndex] = true;
}

bool WallDistanceCalibration::TryGetFrontDirectRiseThresholds(
    uint8_t frontIndex,
    float& signalBaseline,
    float& onRiseThreshold,
    float& offRiseThreshold) const
{
    signalBaseline = 0.0f;
    onRiseThreshold = 0.0f;
    offRiseThreshold = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (!_frontWallDirectThresholdValid[frontIndex])
    {
        return false;
    }

    signalBaseline = _frontWallDirectSignalBaseline[frontIndex];
    onRiseThreshold = _frontWallDirectOnRiseThreshold[frontIndex];
    offRiseThreshold = _frontWallDirectOffRiseThreshold[frontIndex];
    return
        std::isfinite(signalBaseline) &&
        signalBaseline >= 0.0f &&
        std::isfinite(onRiseThreshold) &&
        std::isfinite(offRiseThreshold) &&
        onRiseThreshold > 0.0f &&
        offRiseThreshold > 0.0f &&
        offRiseThreshold < onRiseThreshold;
}

bool WallDistanceCalibration::TryComputeSideWallDistanceThresholds(float latchSignalFraction, float releaseSignalFraction, float& onThresholdM, float& offThresholdM) const
{
    onThresholdM = MazeMap::Config::kSideWallOnThresholdM;
    offThresholdM = MazeMap::Config::kSideWallOffThresholdM;

    float derivedOnThresholdM = 0.0f;
    float derivedOffThresholdM = 0.0f;
    if (!MazeMap::TryComputeLinearWallSignalDistanceThresholdM(
            _expectedSideWallDistanceM,
            latchSignalFraction,
            derivedOnThresholdM) ||
        !MazeMap::TryComputeLinearWallSignalDistanceThresholdM(
            _expectedSideWallDistanceM,
            releaseSignalFraction,
            derivedOffThresholdM))
    {
        return false;
    }

    if (!(std::isfinite(derivedOnThresholdM) &&
        std::isfinite(derivedOffThresholdM) &&
        derivedOnThresholdM > 0.0f &&
        derivedOffThresholdM >= derivedOnThresholdM))
    {
        return false;
    }

    onThresholdM = derivedOnThresholdM;
    offThresholdM = derivedOffThresholdM;
    return true;
}

void WallDistanceCalibration::SetSideWallReferenceDifferentialLight(
    MazeMap::RelativeDirection side,
    float differentialLight)
{
    if (!IsSideDirection(side) ||
        !std::isfinite(differentialLight) ||
        differentialLight <= 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(side);
    _sideWallReferenceDifferentialLight[index] = differentialLight;
    _sideWallReferenceValid[index] = true;
}

void WallDistanceCalibration::SetSideWallReferenceDifferentialLightBand(
    MazeMap::RelativeDirection side,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    if (!IsSideDirection(side) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight <= 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = SideWallIndex(side);
    _sideWallReferenceDifferentialLightLow[index] = lowDifferentialLight;
    _sideWallReferenceDifferentialLightHigh[index] = highDifferentialLight;
    _sideWallReferenceBandValid[index] = true;
}

void WallDistanceCalibration::SetSideWallReferenceDistanceM(MazeMap::RelativeDirection side, float distanceM)
{
    if (!IsSideDirection(side) ||
        !std::isfinite(distanceM) ||
        distanceM <= 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(side);
    _sideWallReferenceDistanceM[index] = distanceM;
    _sideWallReferenceDistanceValid[index] = true;
}

void WallDistanceCalibration::SetSideWallBaselineDifferentialLight(
    MazeMap::RelativeDirection side,
    float differentialLight)
{
    if (!IsSideDirection(side) ||
        !std::isfinite(differentialLight) ||
        differentialLight < 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(side);
    _sideWallBaselineDifferentialLight[index] = differentialLight;
    _sideWallBaselineValid[index] = true;
}

void WallDistanceCalibration::SetSideWallBaselineDifferentialLightBand(
    MazeMap::RelativeDirection side,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    if (!IsSideDirection(side) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight < 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = SideWallIndex(side);
    _sideWallBaselineDifferentialLightLow[index] = lowDifferentialLight;
    _sideWallBaselineDifferentialLightHigh[index] = highDifferentialLight;
    _sideWallBaselineBandValid[index] = true;
}

bool WallDistanceCalibration::TryGetSideWallBaselineDifferentialLight(
    MazeMap::RelativeDirection side,
    float& differentialLight) const
{
    differentialLight = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(side);
    if (!_sideWallBaselineValid[index])
    {
        return false;
    }

    differentialLight = _sideWallBaselineDifferentialLight[index];
    return std::isfinite(differentialLight) && differentialLight >= 0.0f;
}

bool WallDistanceCalibration::TryGetSideWallReferenceDifferentialLightBand(
    MazeMap::RelativeDirection side,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(side);
    if (!_sideWallReferenceBandValid[index])
    {
        return false;
    }

    lowDifferentialLight = _sideWallReferenceDifferentialLightLow[index];
    highDifferentialLight = _sideWallReferenceDifferentialLightHigh[index];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight > 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

bool WallDistanceCalibration::TryGetSideWallBaselineDifferentialLightBand(
    MazeMap::RelativeDirection side,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(side);
    if (!_sideWallBaselineBandValid[index])
    {
        return false;
    }

    lowDifferentialLight = _sideWallBaselineDifferentialLightLow[index];
    highDifferentialLight = _sideWallBaselineDifferentialLightHigh[index];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight >= 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

bool WallDistanceCalibration::TryComputeSideWallNormalizedReferenceDifferentialLight(
    MazeMap::RelativeDirection side,
    float& differentialLight) const
{
    differentialLight = 0.0f;
    return TryGetSideWallReferenceDifferentialLight(side, differentialLight);
}

bool WallDistanceCalibration::TryComputeSideWallMeasuredThresholds(
    MazeMap::RelativeDirection side,
    float latchSignalFraction,
    float releaseSignalFraction,
    float& onMeasuredThreshold,
    float& offMeasuredThreshold,
    float& signalBaseline) const
{
    onMeasuredThreshold = 0.0f;
    offMeasuredThreshold = 0.0f;
    signalBaseline = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    float referenceDifferentialLight = 0.0f;
    if (!TryGetSideWallReferenceDifferentialLight(
            side,
            referenceDifferentialLight))
    {
        return false;
    }

    float baselineDifferentialLight = 0.0f;
    float baselineDifferentialLightLow = 0.0f;
    float baselineDifferentialLightHigh = 0.0f;
    float referenceDifferentialLightLow = 0.0f;
    float referenceDifferentialLightHigh = 0.0f;
    if (TryGetSideWallReferenceDifferentialLightBand(
            side,
            referenceDifferentialLightLow,
            referenceDifferentialLightHigh))
    {
        referenceDifferentialLight = referenceDifferentialLightLow;
    }
    else
    {
        referenceDifferentialLightLow = referenceDifferentialLight;
        referenceDifferentialLightHigh = referenceDifferentialLight;
    }

    if (TryGetSideWallBaselineDifferentialLightBand(
            side,
            baselineDifferentialLightLow,
            baselineDifferentialLightHigh))
    {
    }
    else if (TryGetSideWallBaselineDifferentialLight(side, baselineDifferentialLight))
    {
        baselineDifferentialLightLow = baselineDifferentialLight;
        baselineDifferentialLightHigh = baselineDifferentialLight;
    }

    if (MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
            baselineDifferentialLightLow,
            baselineDifferentialLightHigh,
            referenceDifferentialLightLow,
            referenceDifferentialLightHigh,
            latchSignalFraction,
            releaseSignalFraction,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return true;
    }

    if (TryGetSideWallBaselineDifferentialLight(side, baselineDifferentialLight) &&
        MazeMap::TryComputeSignalRiseThresholds(
            baselineDifferentialLight,
            referenceDifferentialLight,
            latchSignalFraction,
            releaseSignalFraction,
            onMeasuredThreshold,
            offMeasuredThreshold))
    {
        signalBaseline = baselineDifferentialLight;
        return true;
    }

    return MazeMap::TryComputeSignalHighThresholds(
        referenceDifferentialLight,
        latchSignalFraction,
        releaseSignalFraction,
        onMeasuredThreshold,
        offMeasuredThreshold);
}

bool WallDistanceCalibration::TryGetSideWallReferenceDifferentialLight(
    MazeMap::RelativeDirection side,
    float& differentialLight) const
{
    differentialLight = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(side);
    if (!_sideWallReferenceValid[index])
    {
        return false;
    }

    differentialLight = _sideWallReferenceDifferentialLight[index];
    return std::isfinite(differentialLight) && differentialLight > 0.0f;
}

bool WallDistanceCalibration::TryGetSideWallReferenceDistanceM(
    MazeMap::RelativeDirection side,
    float& distanceM) const
{
    distanceM = 0.0f;
    if (!IsSideDirection(side))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(side);
    if (_sideWallReferenceDistanceValid[index])
    {
        distanceM = _sideWallReferenceDistanceM[index];
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }

    const MazeMap::WallSensorCalibrationCurve& curve = SideCurve(side);
    if (curve.GetCount() == 0U)
    {
        return false;
    }

    float distanceSumM = 0.0f;
    uint8_t validCount = 0U;
    for (uint8_t i = 0U; i < curve.GetCount(); ++i)
    {
        const float pointDistanceM = curve.GetPoint(i).actualDistanceM;
        if (!std::isfinite(pointDistanceM) || pointDistanceM <= 0.0f)
        {
            continue;
        }

        distanceSumM += pointDistanceM;
        ++validCount;
    }

    if (validCount == 0U)
    {
        return false;
    }

    distanceM = distanceSumM / static_cast<float>(validCount);
    return std::isfinite(distanceM) && distanceM > 0.0f;
}

bool WallDistanceCalibration::TryGetFrontLeftWeakestCalibrationMeasuredValue(float& measuredValue) const
{
    return TryGetWeakestFrontCalibrationMeasuredValue(_frontLeftCurve, measuredValue);
}

bool WallDistanceCalibration::TryGetFrontRightWeakestCalibrationMeasuredValue(float& measuredValue) const
{
    return TryGetWeakestFrontCalibrationMeasuredValue(_frontRightCurve, measuredValue);
}

bool WallDistanceCalibration::TryComputeFrontWallDistanceThresholds(
    const MazeMap::Vehicle& vehicle,
    float releaseHysteresisDistanceM,
    float& onDistanceThresholdM,
    float& offDistanceThresholdM) const
{
    const float forwardSensorOffsetM = (std::min)(
        vehicle.FrontLeftWallSensor().GetPosition().y(),
        vehicle.FrontRightWallSensor().GetPosition().y());
    if (!MazeMap::TryComputeFrontWallHalfwayIntoAdjacentDistanceM(
            MazeMap::Config::kCellSizeM,
            MazeMap::Config::kMazeWallThicknessM,
            forwardSensorOffsetM,
            onDistanceThresholdM) ||
        !MazeMap::TryExpandWallThresholdDistanceM(
            onDistanceThresholdM,
            releaseHysteresisDistanceM,
            offDistanceThresholdM))
    {
        return false;
    }

    return MazeMap::TryClampWallThresholdDistanceRangeM(
        onDistanceThresholdM,
        offDistanceThresholdM,
        MazeMap::Config::kFrontWallOnThresholdM,
        MazeMap::Config::kFrontWallOffThresholdM,
        onDistanceThresholdM,
        offDistanceThresholdM);
}

bool WallDistanceCalibration::TryComputeFrontLeftSensorMeasuredThresholds(
    const MazeMap::Vehicle& vehicle,
    float releaseHysteresisDistanceM,
    float ambientLight,
    float& onMeasuredThreshold,
    float& offMeasuredThreshold,
    float& signalBaseline) const
{
    return TryComputeFrontSensorMeasuredThresholds(
        kFrontLeftIndex,
        _frontLeftCurve,
        vehicle,
        releaseHysteresisDistanceM,
        ambientLight,
        onMeasuredThreshold,
        offMeasuredThreshold,
        signalBaseline);
}

bool WallDistanceCalibration::TryComputeFrontRightSensorMeasuredThresholds(
    const MazeMap::Vehicle& vehicle,
    float releaseHysteresisDistanceM,
    float ambientLight,
    float& onMeasuredThreshold,
    float& offMeasuredThreshold,
    float& signalBaseline) const
{
    return TryComputeFrontSensorMeasuredThresholds(
        kFrontRightIndex,
        _frontRightCurve,
        vehicle,
        releaseHysteresisDistanceM,
        ambientLight,
        onMeasuredThreshold,
        offMeasuredThreshold,
        signalBaseline);
}

bool WallDistanceCalibration::AddFrontPoint(
    uint8_t frontIndex,
    MazeMap::WallSensorCalibrationCurve& curve,
    float measuredValue,
    float actualDistanceM,
    float ambientLight)
{
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    const bool stored = curve.AddPoint(measuredValue, actualDistanceM, ambientLight);
    if (stored)
    {
        InvalidateFrontSignalModelCache(frontIndex);
    }

    return stored;
}

bool WallDistanceCalibration::AddSidePointAt(
    uint8_t sideIndex,
    float measuredValue,
    float actualDistanceM,
    float ambientLight)
{
    if (sideIndex > kSideRightIndex)
    {
        return false;
    }

    MazeMap::WallSensorCalibrationCurve& curve = (sideIndex == kSideLeftIndex) ? _sideLeftCurve : _sideRightCurve;
    return curve.AddPoint(measuredValue, actualDistanceM, ambientLight);
}

float WallDistanceCalibration::ApplyCurve(
    const MazeMap::WallSensorCalibrationCurve& curve,
    MazeMap::WallSensorCalibrationMode mode,
    float measuredValue,
    float fallbackDistanceM) const
{
    if (!std::isfinite(fallbackDistanceM) || fallbackDistanceM <= 0.0f)
    {
        fallbackDistanceM = measuredValue;
    }
    if (!std::isfinite(measuredValue) || measuredValue <= 0.0f)
    {
        return fallbackDistanceM;
    }
    if (curve.GetCount() == 0U)
    {
        return fallbackDistanceM;
    }
    if ((mode == MazeMap::WallSensorCalibrationMode::DirectInterpolation) && (curve.GetCount() < 2U))
    {
        return fallbackDistanceM;
    }

    return curve.Apply(measuredValue, mode);
}

bool WallDistanceCalibration::IsSideDirection(MazeMap::RelativeDirection side)
{
    return side == MazeMap::RelativeDirection::Left90 || side == MazeMap::RelativeDirection::Right90;
}

uint8_t WallDistanceCalibration::SideWallIndex(MazeMap::RelativeDirection side)
{
    return (side == MazeMap::RelativeDirection::Right90) ? kSideRightIndex : kSideLeftIndex;
}

MazeMap::WallSensorCalibrationCurve& WallDistanceCalibration::SideCurve(MazeMap::RelativeDirection side)
{
    return (side == MazeMap::RelativeDirection::Right90) ? _sideRightCurve : _sideLeftCurve;
}

const MazeMap::WallSensorCalibrationCurve& WallDistanceCalibration::SideCurve(MazeMap::RelativeDirection side) const
{
    return (side == MazeMap::RelativeDirection::Right90) ? _sideRightCurve : _sideLeftCurve;
}

bool WallDistanceCalibration::TryGetWeakestFrontCalibrationMeasuredValue(
    const MazeMap::WallSensorCalibrationCurve& curve,
    float& measuredValue) const
{
    measuredValue = 0.0f;
    if (curve.GetCount() == 0U)
    {
        return false;
    }

    measuredValue = curve.GetPoint(0U).measuredValue;
    return std::isfinite(measuredValue) && measuredValue > 0.0f;
}

bool WallDistanceCalibration::TryComputeFrontSensorMeasuredThresholds(
    uint8_t frontIndex,
    const MazeMap::WallSensorCalibrationCurve& curve,
    const MazeMap::Vehicle& vehicle,
    float releaseHysteresisDistanceM,
    float ambientLight,
    float& onMeasuredThreshold,
    float& offMeasuredThreshold,
    float& signalBaseline) const
{
    onMeasuredThreshold = 0.0f;
    offMeasuredThreshold = 0.0f;
    signalBaseline = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (TryGetFrontDirectRiseThresholds(
            frontIndex,
            signalBaseline,
            onMeasuredThreshold,
            offMeasuredThreshold))
    {
        return true;
    }

    float weakestCalibrationDifferentialLightLow = 0.0f;
    float weakestCalibrationDifferentialLightHigh = 0.0f;
    float baselineDifferentialLightLow = 0.0f;
    float baselineDifferentialLightHigh = 0.0f;
    if (TryGetFrontWeakestCalibrationDifferentialLightBand(
            frontIndex,
            weakestCalibrationDifferentialLightLow,
            weakestCalibrationDifferentialLightHigh) &&
        TryGetFrontWallBaselineDifferentialLightBand(
            frontIndex,
            baselineDifferentialLightLow,
            baselineDifferentialLightHigh) &&
        MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
            baselineDifferentialLightLow,
            baselineDifferentialLightHigh,
            weakestCalibrationDifferentialLightLow,
            weakestCalibrationDifferentialLightHigh,
            MazeMap::Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
            MazeMap::Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return true;
    }

    float weakestCalibrationSignal = 0.0f;
    float baselineDifferentialLight = 0.0f;
    if (TryGetWeakestFrontCalibrationMeasuredValue(curve, weakestCalibrationSignal) &&
        TryGetFrontWallBaselineDifferentialLight(frontIndex, baselineDifferentialLight) &&
        MazeMap::TryComputeSignalRiseThresholds(
            baselineDifferentialLight,
            weakestCalibrationSignal,
            MazeMap::Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
            MazeMap::Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
            onMeasuredThreshold,
            offMeasuredThreshold))
    {
        signalBaseline = baselineDifferentialLight;
        return true;
    }

    float onDistanceThresholdM = 0.0f;
    float offDistanceThresholdM = 0.0f;
    if (!TryComputeFrontWallDistanceThresholds(
            vehicle,
            releaseHysteresisDistanceM,
            onDistanceThresholdM,
            offDistanceThresholdM))
    {
        return false;
    }

    float effectiveAmbientLight = ambientLight;
    if (!(std::isfinite(effectiveAmbientLight) && effectiveAmbientLight >= 0.0f) &&
        !TryComputeFrontSensorRepresentativeAmbientLight(curve, effectiveAmbientLight))
    {
        return false;
    }

    float signalGain = 0.0f;
    float signalLightScale = 0.0f;
    if (!TryGetFrontSignalModel(frontIndex, curve, signalGain, signalLightScale) ||
        !MazeMap::TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(
            signalGain,
            signalLightScale,
            effectiveAmbientLight,
            onDistanceThresholdM,
            onMeasuredThreshold) ||
        !MazeMap::TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(
            signalGain,
            signalLightScale,
            effectiveAmbientLight,
            offDistanceThresholdM,
            offMeasuredThreshold))
    {
        return false;
    }

    if (!MazeMap::TryScaleSignalHighThresholds(
            MazeMap::Config::kFrontWallMeasuredSignalThresholdScale,
            onMeasuredThreshold,
            offMeasuredThreshold))
    {
        onMeasuredThreshold = 0.0f;
        offMeasuredThreshold = 0.0f;
        return false;
    }

    return
        std::isfinite(onMeasuredThreshold) &&
        std::isfinite(offMeasuredThreshold) &&
        onMeasuredThreshold > 0.0f &&
        offMeasuredThreshold > 0.0f &&
        offMeasuredThreshold < onMeasuredThreshold;
}

bool WallDistanceCalibration::TryComputeFrontSensorRepresentativeAmbientLight(
    const MazeMap::WallSensorCalibrationCurve& curve,
    float& ambientLight) const
{
    ambientLight = 0.0f;
    if (curve.GetCount() < 2U)
    {
        return false;
    }

    double ambientLightSum = 0.0;
    uint8_t validPointCount = 0U;
    for (uint8_t index = 0U; index < curve.GetCount(); ++index)
    {
        const MazeMap::WallSensorCalibrationCurve::Point& point = curve.GetPoint(index);
        if (!std::isfinite(point.ambientLight) || point.ambientLight < 0.0f)
        {
            return false;
        }

        ambientLightSum += static_cast<double>(point.ambientLight);
        ++validPointCount;
    }

    if (validPointCount == 0U)
    {
        return false;
    }

    ambientLight = static_cast<float>(ambientLightSum / static_cast<double>(validPointCount));
    return std::isfinite(ambientLight) && ambientLight >= 0.0f;
}

void WallDistanceCalibration::InvalidateFrontSignalModelCache()
{
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        _frontSignalModelCacheValid[index] = false;
        _frontSignalModelCacheGain[index] = 0.0f;
        _frontSignalModelCacheLightScale[index] = 0.0f;
    }
}

void WallDistanceCalibration::InvalidateFrontSignalModelCache(uint8_t frontIndex)
{
    if (frontIndex > kFrontRightIndex)
    {
        return;
    }

    _frontSignalModelCacheValid[frontIndex] = false;
    _frontSignalModelCacheGain[frontIndex] = 0.0f;
    _frontSignalModelCacheLightScale[frontIndex] = 0.0f;
}

bool WallDistanceCalibration::TryGetFrontSignalModel(
    uint8_t frontIndex,
    const MazeMap::WallSensorCalibrationCurve& curve,
    float& gain,
    float& lightScale) const
{
    gain = 0.0f;
    lightScale = 0.0f;
    if (frontIndex > kFrontRightIndex)
    {
        return false;
    }

    if (!_frontSignalModelCacheValid[frontIndex])
    {
        if (!MazeMap::TryFitAmbientAwareLogDifferentialSignalModel(
                curve,
                _frontSignalModelCacheGain[frontIndex],
                _frontSignalModelCacheLightScale[frontIndex]))
        {
            return false;
        }

        _frontSignalModelCacheValid[frontIndex] = true;
    }

    gain = _frontSignalModelCacheGain[frontIndex];
    lightScale = _frontSignalModelCacheLightScale[frontIndex];
    return std::isfinite(gain) && std::isfinite(lightScale) && gain > 0.0f && lightScale > 0.0f;
}

WallDistanceCalibration gWallDistanceCalibration;

float ComputeDiagonalWallCenterOmegaRadps(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    float rightMeasuredSignal)
{
    float leftReferenceSignal = 0.0f;
    float rightReferenceSignal = 0.0f;
    if (!wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(MazeMap::RelativeDirection::Left90, leftReferenceSignal) ||
        !wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(MazeMap::RelativeDirection::Right90, rightReferenceSignal))
    {
        return 0.0f;
    }

    float balanceError = 0.0f;
    if (!MazeMap::TryComputeNormalizedWallSignalBalanceError(
            leftMeasuredSignal,
            leftReferenceSignal,
            rightMeasuredSignal,
            rightReferenceSignal,
            MazeMap::Config::kDiagonalWallMinNormalizedSignal,
            balanceError))
    {
        return 0.0f;
    }

    return MazeMap::Config::kDiagonalWallCenterGain * balanceError;
}

bool WallDistanceCalibration::TryComputeSideWallSignalDistanceM(
    MazeMap::RelativeDirection side,
    float measuredSignal,
    float& distanceM) const
{
    distanceM = 0.0f;

    float referenceSignal = 0.0f;
    float referenceDistanceM = 0.0f;
    if (!TryGetSideWallReferenceDifferentialLight(side, referenceSignal) ||
        !TryGetSideWallReferenceDistanceM(side, referenceDistanceM))
    {
        return false;
    }

    return MazeMap::TryComputeInverseSquareDistanceFromReferenceSignal(
        measuredSignal,
        referenceSignal,
        referenceDistanceM,
        distanceM);
}

bool WallDistanceCalibration::TryComputeSideWallSignalRiseMetrics(
    const MazeMap::RelativeDirection side,
    const float measuredDifferentialLight,
    const float latchSignalFraction,
    const float releaseSignalFraction,
    const float missSignalFractionOfLatch,
    float& signalRise,
    float& latchRiseThreshold,
    float& missRiseThreshold) const
{
    signalRise = 0.0f;
    latchRiseThreshold = 0.0f;
    missRiseThreshold = 0.0f;

    float offMeasuredThreshold = 0.0f;
    float signalBaseline = 0.0f;
    if (!TryComputeSideWallMeasuredThresholds(
            side,
            latchSignalFraction,
            releaseSignalFraction,
            latchRiseThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return false;
    }

    signalRise = ComputeCalibratedSideSignalRise(measuredDifferentialLight, signalBaseline);
    missRiseThreshold = missSignalFractionOfLatch * latchRiseThreshold;
    return
        std::isfinite(signalRise) &&
        std::isfinite(latchRiseThreshold) &&
        std::isfinite(missRiseThreshold) &&
        (latchRiseThreshold > 0.0f) &&
        (missRiseThreshold >= 0.0f);
}

bool WallDistanceCalibration::IsSideWallSignalClassifiable(
    const bool signalMetricsValid,
    const float signalRise,
    const float latchRiseThreshold,
    const float missRiseThreshold) const noexcept
{
    return
        signalMetricsValid &&
        ((signalRise >= latchRiseThreshold) ||
            (signalRise <= missRiseThreshold));
}

bool WallDistanceCalibration::IsSideWallFallbackDistanceValid(const float fallbackDistanceM) const noexcept
{
    return std::isfinite(fallbackDistanceM) && (fallbackDistanceM > 0.0f);
}

bool WallDistanceCalibration::IsSideWallObservationEligible(
    const bool detectionWindowValid,
    const bool signalClassifiable,
    const bool fallbackDistanceValid) const noexcept
{
    return detectionWindowValid && (signalClassifiable || fallbackDistanceValid);
}

bool WallDistanceCalibration::IsSideWallControlRangeValid(
    const bool observationEligible,
    const bool transitionDetected,
    const bool signalMetricsValid,
    const float signalRise,
    const float latchRiseThreshold,
    const bool fallbackDistanceValid,
    const float fallbackDistanceM,
    const float offThresholdM) const noexcept
{
    if (!observationEligible || transitionDetected)
    {
        return false;
    }

    if (signalMetricsValid)
    {
        return signalRise >= latchRiseThreshold;
    }

    return fallbackDistanceValid && (fallbackDistanceM < offThresholdM);
}

bool WallDistanceCalibration::DetectSideWallTransitionFromSignalRise(
    const bool detectionWindowValid,
    const bool signalMetricsValid,
    const float signalRise,
    const float latchRiseThreshold,
    const float transitionSignalFractionOfLatch,
    float& previousSignalRise,
    bool& previousValid) const noexcept
{
    bool transitionDetected = false;
    const float transitionThreshold = latchRiseThreshold * transitionSignalFractionOfLatch;
    const bool currentValid =
        detectionWindowValid &&
        signalMetricsValid &&
        std::isfinite(signalRise) &&
        std::isfinite(transitionThreshold) &&
        (transitionThreshold > 0.0f);
    if (currentValid && previousValid)
    {
        transitionDetected = std::fabs(signalRise - previousSignalRise) >= transitionThreshold;
    }

    previousSignalRise = currentValid ? signalRise : 0.0f;
    previousValid = currentValid;
    return transitionDetected;
}

bool WallDistanceCalibration::ComputeSideWallObservationHit(
    const MazeMap::RelativeDirection side,
    const float measuredDifferentialLight,
    const float fallbackDistanceM,
    const float onThresholdM,
    const bool detectionWindowValid) const
{
    if (!detectionWindowValid)
    {
        return false;
    }

    float onMeasuredThreshold = 0.0f;
    float offMeasuredThreshold = 0.0f;
    float signalBaseline = 0.0f;
    if (TryComputeSideWallMeasuredThresholds(
            side,
            MazeMap::Config::kSideWallMeasuredSignalLatchThreshold,
            MazeMap::Config::kSideWallMeasuredSignalReleaseThreshold,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return ComputeCalibratedSideSignalRise(measuredDifferentialLight, signalBaseline) >= onMeasuredThreshold;
    }

    return std::isfinite(fallbackDistanceM) && (fallbackDistanceM < onThresholdM);
}

bool WallDistanceCalibration::UpdateSideWallState(
    const MazeMap::RelativeDirection side,
    const float measuredDifferentialLight,
    const float fallbackDistanceM,
    const float onThresholdM,
    const float offThresholdM,
    const bool detectionWindowValid,
    float& filteredSignal,
    bool& signalInitialized,
    bool& currentState) const
{
    if (!detectionWindowValid)
    {
        filteredSignal = 0.0f;
        signalInitialized = false;
        currentState = false;
        return false;
    }

    float onMeasuredThreshold = 0.0f;
    float offMeasuredThreshold = 0.0f;
    float signalBaseline = 0.0f;
    if (TryComputeSideWallMeasuredThresholds(
            side,
            MazeMap::Config::kSideWallMeasuredSignalLatchThreshold,
            MazeMap::Config::kSideWallMeasuredSignalReleaseThreshold,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        const float signalRise = ComputeCalibratedSideSignalRise(measuredDifferentialLight, signalBaseline);
        filteredSignal = signalRise;
        signalInitialized = true;
        currentState = currentState ?
            (signalRise >= offMeasuredThreshold) :
            (signalRise >= onMeasuredThreshold);
        return currentState;
    }

    signalInitialized = false;
    currentState = currentState ? (fallbackDistanceM < offThresholdM) : (fallbackDistanceM < onThresholdM);
    return currentState;
}

float WallDistanceCalibration::ComputeCalibratedSideSignalRise(
    const float measuredDifferentialLight,
    const float signalBaseline) noexcept
{
    if (!std::isfinite(measuredDifferentialLight) ||
        !std::isfinite(signalBaseline))
    {
        return 0.0f;
    }

    return (measuredDifferentialLight > signalBaseline) ?
        (measuredDifferentialLight - signalBaseline) :
        0.0f;
}

bool TryComputeStraightWallCenterErrorM(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    bool leftWall,
    float rightMeasuredSignal,
    bool rightWall,
    float& corridorErrorM)
{
    corridorErrorM = 0.0f;

    const float expectedDistanceM = wallCalibration.GetExpectedSideWallDistanceM();
    if (!(std::isfinite(expectedDistanceM) && expectedDistanceM > 0.0f))
    {
        return false;
    }

    if (leftWall && rightWall)
    {
        float leftReferenceSignal = 0.0f;
        float rightReferenceSignal = 0.0f;
        float balanceError = 0.0f;
        if (wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(MazeMap::RelativeDirection::Left90, leftReferenceSignal) &&
            wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(MazeMap::RelativeDirection::Right90, rightReferenceSignal) &&
            MazeMap::TryComputeNormalizedWallSignalBalanceError(
                leftMeasuredSignal,
                leftReferenceSignal,
                rightMeasuredSignal,
                rightReferenceSignal,
                MazeMap::Config::kSideWallMeasuredSignalReleaseThreshold,
                balanceError))
        {
            corridorErrorM = -0.5f * expectedDistanceM * balanceError;
            return std::isfinite(corridorErrorM);
        }
    }

    if (leftWall)
    {
        float leftDistanceM = 0.0f;
        if (wallCalibration.TryComputeSideWallSignalDistanceM(
                MazeMap::RelativeDirection::Left90,
                leftMeasuredSignal,
                leftDistanceM))
        {
            corridorErrorM = leftDistanceM - expectedDistanceM;
            return std::isfinite(corridorErrorM);
        }
    }

    if (rightWall)
    {
        float rightDistanceM = 0.0f;
        if (wallCalibration.TryComputeSideWallSignalDistanceM(
                MazeMap::RelativeDirection::Right90,
                rightMeasuredSignal,
                rightDistanceM))
        {
            corridorErrorM = expectedDistanceM - rightDistanceM;
            return std::isfinite(corridorErrorM);
        }
    }

    return false;
}

