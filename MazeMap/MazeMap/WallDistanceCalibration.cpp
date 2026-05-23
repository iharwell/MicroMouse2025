#include "pch.h"
#include "WallDistanceCalibration.h"

#include "CoreConfig.h"
#include "DiagonalWallCentering.h"
#include "Vehicle.h"
#include "WallDetectionThresholds.h"

#include <algorithm>
#include <cmath>

WallDistanceCalibration::WallDistanceCalibration()
    : _curves{}
    , _frontSignalModelCache{}
    , _expectedSideWallDistanceM(MazeMap::Config::kExpectedSideWallDistanceM)
{
}

void WallDistanceCalibration::Clear()
{
    for (uint8_t i = 0U; i < static_cast<uint8_t>(WallSensorId::Count); ++i)
    {
        _curves[i].Clear();
    }
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

bool WallDistanceCalibration::AddPoint(WallSensorId sensorId, float measuredValue, float actualDistanceM, float ambientLight)
{
    const bool stored = _curves[static_cast<uint8_t>(sensorId)].AddPoint(measuredValue, actualDistanceM, ambientLight);
    if (stored && IsFrontWallSensor(sensorId))
    {
        InvalidateFrontSignalModelCache(sensorId);
    }

    return stored;
}

float WallDistanceCalibration::Apply(WallSensorId sensorId, float measuredValue, float fallbackDistanceM) const
{
    if (!std::isfinite(fallbackDistanceM) || fallbackDistanceM <= 0.0f)
    {
        fallbackDistanceM = measuredValue;
    }
    if (!std::isfinite(measuredValue) || measuredValue <= 0.0f)
    {
        return fallbackDistanceM;
    }

    const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
    if (curve.GetCount() == 0U)
    {
        return fallbackDistanceM;
    }

    const MazeMap::WallSensorCalibrationMode mode = WallSensorCalibrationModeFor(sensorId);
    if (mode == MazeMap::WallSensorCalibrationMode::DirectInterpolation && curve.GetCount() < 2U)
    {
        return fallbackDistanceM;
    }

    return curve.Apply(measuredValue, mode);
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

void WallDistanceCalibration::SetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight)
{
    if (!IsFrontWallSensor(sensorId) ||
        !std::isfinite(differentialLight) ||
        differentialLight < 0.0f)
    {
        return;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    _frontWallBaselineDifferentialLight[index] = differentialLight;
    _frontWallBaselineValid[index] = true;
}

void WallDistanceCalibration::SetFrontWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
{
    if (!IsFrontWallSensor(sensorId) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight < 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    _frontWallBaselineDifferentialLightLow[index] = lowDifferentialLight;
    _frontWallBaselineDifferentialLightHigh[index] = highDifferentialLight;
    _frontWallBaselineBandValid[index] = true;
}

bool WallDistanceCalibration::TryGetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const
{
    differentialLight = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    if (!_frontWallBaselineValid[index])
    {
        return false;
    }

    differentialLight = _frontWallBaselineDifferentialLight[index];
    return std::isfinite(differentialLight) && differentialLight >= 0.0f;
}

bool WallDistanceCalibration::TryGetFrontWallBaselineDifferentialLightBand(
    WallSensorId sensorId,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    if (!_frontWallBaselineBandValid[index])
    {
        return false;
    }

    lowDifferentialLight = _frontWallBaselineDifferentialLightLow[index];
    highDifferentialLight = _frontWallBaselineDifferentialLightHigh[index];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight >= 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

void WallDistanceCalibration::SetFrontWeakestCalibrationDifferentialLightBand(
    WallSensorId sensorId,
    float measuredValue,
    float lowDifferentialLight,
    float highDifferentialLight)
{
    if (!IsFrontWallSensor(sensorId) ||
        !std::isfinite(measuredValue) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        measuredValue <= 0.0f ||
        lowDifferentialLight <= 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    if (_frontWallWeakestCalibrationBandValid[index] &&
        (measuredValue > (_frontWallWeakestCalibrationMeasuredValue[index] + 0.001f)))
    {
        return;
    }

    _frontWallWeakestCalibrationMeasuredValue[index] = measuredValue;
    _frontWallWeakestCalibrationDifferentialLightLow[index] = lowDifferentialLight;
    _frontWallWeakestCalibrationDifferentialLightHigh[index] = highDifferentialLight;
    _frontWallWeakestCalibrationBandValid[index] = true;
}

bool WallDistanceCalibration::TryGetFrontWeakestCalibrationDifferentialLightBand(
    WallSensorId sensorId,
    float& lowDifferentialLight,
    float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    if (!_frontWallWeakestCalibrationBandValid[index])
    {
        return false;
    }

    lowDifferentialLight = _frontWallWeakestCalibrationDifferentialLightLow[index];
    highDifferentialLight = _frontWallWeakestCalibrationDifferentialLightHigh[index];
    return
        std::isfinite(lowDifferentialLight) &&
        std::isfinite(highDifferentialLight) &&
        lowDifferentialLight > 0.0f &&
        highDifferentialLight >= lowDifferentialLight;
}

void WallDistanceCalibration::SetFrontDirectRiseThresholds(
    WallSensorId sensorId,
    float signalBaseline,
    float onRiseThreshold,
    float offRiseThreshold)
{
    if (!IsFrontWallSensor(sensorId) ||
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

    const uint8_t index = FrontWallIndex(sensorId);
    _frontWallDirectSignalBaseline[index] = signalBaseline;
    _frontWallDirectOnRiseThreshold[index] = onRiseThreshold;
    _frontWallDirectOffRiseThreshold[index] = offRiseThreshold;
    _frontWallDirectThresholdValid[index] = true;
}

bool WallDistanceCalibration::TryGetFrontDirectRiseThresholds(
    WallSensorId sensorId,
    float& signalBaseline,
    float& onRiseThreshold,
    float& offRiseThreshold) const
{
    signalBaseline = 0.0f;
    onRiseThreshold = 0.0f;
    offRiseThreshold = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = FrontWallIndex(sensorId);
    if (!_frontWallDirectThresholdValid[index])
    {
        return false;
    }

    signalBaseline = _frontWallDirectSignalBaseline[index];
    onRiseThreshold = _frontWallDirectOnRiseThreshold[index];
    offRiseThreshold = _frontWallDirectOffRiseThreshold[index];
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

void WallDistanceCalibration::SetSideWallReferenceDifferentialLight(WallSensorId sensorId, float differentialLight)
{
    if (!IsSideWallSensor(sensorId) ||
        !std::isfinite(differentialLight) ||
        differentialLight <= 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(sensorId);
    _sideWallReferenceDifferentialLight[index] = differentialLight;
    _sideWallReferenceValid[index] = true;
}

void WallDistanceCalibration::SetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
{
    if (!IsSideWallSensor(sensorId) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight <= 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = SideWallIndex(sensorId);
    _sideWallReferenceDifferentialLightLow[index] = lowDifferentialLight;
    _sideWallReferenceDifferentialLightHigh[index] = highDifferentialLight;
    _sideWallReferenceBandValid[index] = true;
}

void WallDistanceCalibration::SetSideWallReferenceDistanceM(WallSensorId sensorId, float distanceM)
{
    if (!IsSideWallSensor(sensorId) ||
        !std::isfinite(distanceM) ||
        distanceM <= 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(sensorId);
    _sideWallReferenceDistanceM[index] = distanceM;
    _sideWallReferenceDistanceValid[index] = true;
}

void WallDistanceCalibration::SetSideWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight)
{
    if (!IsSideWallSensor(sensorId) ||
        !std::isfinite(differentialLight) ||
        differentialLight < 0.0f)
    {
        return;
    }

    const uint8_t index = SideWallIndex(sensorId);
    _sideWallBaselineDifferentialLight[index] = differentialLight;
    _sideWallBaselineValid[index] = true;
}

void WallDistanceCalibration::SetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
{
    if (!IsSideWallSensor(sensorId) ||
        !std::isfinite(lowDifferentialLight) ||
        !std::isfinite(highDifferentialLight) ||
        lowDifferentialLight < 0.0f ||
        highDifferentialLight < lowDifferentialLight)
    {
        return;
    }

    const uint8_t index = SideWallIndex(sensorId);
    _sideWallBaselineDifferentialLightLow[index] = lowDifferentialLight;
    _sideWallBaselineDifferentialLightHigh[index] = highDifferentialLight;
    _sideWallBaselineBandValid[index] = true;
}

bool WallDistanceCalibration::TryGetSideWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const
{
    differentialLight = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(sensorId);
    if (!_sideWallBaselineValid[index])
    {
        return false;
    }

    differentialLight = _sideWallBaselineDifferentialLight[index];
    return std::isfinite(differentialLight) && differentialLight >= 0.0f;
}

bool WallDistanceCalibration::TryGetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(sensorId);
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

bool WallDistanceCalibration::TryGetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const
{
    lowDifferentialLight = 0.0f;
    highDifferentialLight = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(sensorId);
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
    WallSensorId sensorId,
    float& differentialLight) const
{
    differentialLight = 0.0f;
    return TryGetSideWallReferenceDifferentialLight(sensorId, differentialLight);
}

bool WallDistanceCalibration::TryComputeSideWallMeasuredThresholds(
    WallSensorId sensorId,
    float latchSignalFraction,
    float releaseSignalFraction,
    float& onMeasuredThreshold,
    float& offMeasuredThreshold,
    float& signalBaseline) const
{
    onMeasuredThreshold = 0.0f;
    offMeasuredThreshold = 0.0f;
    signalBaseline = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    float referenceDifferentialLight = 0.0f;
    if (!TryGetSideWallReferenceDifferentialLight(
            sensorId,
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
            sensorId,
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
            sensorId,
            baselineDifferentialLightLow,
            baselineDifferentialLightHigh))
    {
    }
    else if (TryGetSideWallBaselineDifferentialLight(sensorId, baselineDifferentialLight))
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

    if (TryGetSideWallBaselineDifferentialLight(sensorId, baselineDifferentialLight) &&
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

bool WallDistanceCalibration::TryGetSideWallReferenceDifferentialLight(WallSensorId sensorId, float& differentialLight) const
{
    differentialLight = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(sensorId);
    if (!_sideWallReferenceValid[index])
    {
        return false;
    }

    differentialLight = _sideWallReferenceDifferentialLight[index];
    return std::isfinite(differentialLight) && differentialLight > 0.0f;
}

bool WallDistanceCalibration::TryGetSideWallReferenceDistanceM(WallSensorId sensorId, float& distanceM) const
{
    distanceM = 0.0f;
    if (!IsSideWallSensor(sensorId))
    {
        return false;
    }

    const uint8_t index = SideWallIndex(sensorId);
    if (_sideWallReferenceDistanceValid[index])
    {
        distanceM = _sideWallReferenceDistanceM[index];
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }

    const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
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

bool WallDistanceCalibration::TryGetWeakestFrontCalibrationMeasuredValue(
    WallSensorId sensorId,
    float& measuredValue) const
{
    measuredValue = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
    if (curve.GetCount() == 0U)
    {
        return false;
    }

    measuredValue = curve.GetPoint(0U).measuredValue;
    return std::isfinite(measuredValue) && measuredValue > 0.0f;
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

bool WallDistanceCalibration::TryComputeFrontSensorMeasuredThresholds(
    WallSensorId sensorId,
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

    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    if (TryGetFrontDirectRiseThresholds(
            sensorId,
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
            sensorId,
            weakestCalibrationDifferentialLightLow,
            weakestCalibrationDifferentialLightHigh) &&
        TryGetFrontWallBaselineDifferentialLightBand(
            sensorId,
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
    if (TryGetWeakestFrontCalibrationMeasuredValue(sensorId, weakestCalibrationSignal) &&
        TryGetFrontWallBaselineDifferentialLight(sensorId, baselineDifferentialLight) &&
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
        !TryComputeFrontSensorRepresentativeAmbientLight(sensorId, effectiveAmbientLight))
    {
        return false;
    }

    float signalGain = 0.0f;
    float signalLightScale = 0.0f;
    if (!TryGetFrontSignalModel(sensorId, signalGain, signalLightScale) ||
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

bool WallDistanceCalibration::TryComputeFrontSensorRepresentativeAmbientLight(WallSensorId sensorId, float& ambientLight) const
{
    ambientLight = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
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

const MazeMap::WallSensorCalibrationCurve& WallDistanceCalibration::GetCurve(WallSensorId sensorId) const
{
    return _curves[static_cast<uint8_t>(sensorId)];
}

bool WallDistanceCalibration::IsSideWallSensor(WallSensorId sensorId)
{
    return sensorId == WallSensorId::SideLeft || sensorId == WallSensorId::SideRight;
}

uint8_t WallDistanceCalibration::SideWallIndex(WallSensorId sensorId)
{
    return (sensorId == WallSensorId::SideRight) ? 1U : 0U;
}

uint8_t WallDistanceCalibration::FrontWallIndex(WallSensorId sensorId)
{
    return (sensorId == WallSensorId::FrontRight) ? 1U : 0U;
}

void WallDistanceCalibration::InvalidateFrontSignalModelCache()
{
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        _frontSignalModelCache[index] = FrontSignalModelCache{};
    }
}

void WallDistanceCalibration::InvalidateFrontSignalModelCache(WallSensorId sensorId)
{
    if (!IsFrontWallSensor(sensorId))
    {
        return;
    }

    _frontSignalModelCache[FrontWallIndex(sensorId)] = FrontSignalModelCache{};
}

bool WallDistanceCalibration::TryGetFrontSignalModel(WallSensorId sensorId, float& gain, float& lightScale) const
{
    gain = 0.0f;
    lightScale = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    FrontSignalModelCache& cache = _frontSignalModelCache[FrontWallIndex(sensorId)];
    if (!cache.valid)
    {
        const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
        if (!MazeMap::TryFitAmbientAwareLogDifferentialSignalModel(curve, cache.gain, cache.lightScale))
        {
            return false;
        }

        cache.valid = true;
    }

    gain = cache.gain;
    lightScale = cache.lightScale;
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
    if (!wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideLeft, leftReferenceSignal) ||
        !wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideRight, rightReferenceSignal))
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

bool TryComputeSideWallSignalDistanceM(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredSignal,
    float& distanceM)
{
    distanceM = 0.0f;

    float referenceSignal = 0.0f;
    float referenceDistanceM = 0.0f;
    if (!wallCalibration.TryGetSideWallReferenceDifferentialLight(sensorId, referenceSignal) ||
        !wallCalibration.TryGetSideWallReferenceDistanceM(sensorId, referenceDistanceM))
    {
        return false;
    }

    return MazeMap::TryComputeInverseSquareDistanceFromReferenceSignal(
        measuredSignal,
        referenceSignal,
        referenceDistanceM,
        distanceM);
}

float ComputeSignalRiseAboveBaselineValue(
    float measuredDifferentialLight,
    float signalBaseline)
{
    // Exclusively for the purpose of centering.
    if (!std::isfinite(measuredDifferentialLight) ||
        !std::isfinite(signalBaseline))
    {
        return 0.0f;
    }

    return (measuredDifferentialLight > signalBaseline) ?
        (measuredDifferentialLight - signalBaseline) :
        0.0f;
}

bool IsCalibratedSideDistanceValidForControl(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredDifferentialLight)
{
    // Exclusively for the purpose of deciding whether a side distance estimate is trustworthy for control.
    float onMeasuredThreshold = 0.0f;
    float offMeasuredThreshold = 0.0f;
    float signalBaseline = 0.0f;
    if (!wallCalibration.TryComputeSideWallMeasuredThresholds(
            sensorId,
            MazeMap::Config::kSideWallMeasuredSignalLatchThreshold,
            MazeMap::Config::kSideWallMeasuredSignalReleaseThreshold,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return false;
    }

    return ComputeSignalRiseAboveBaselineValue(measuredDifferentialLight, signalBaseline) >= onMeasuredThreshold;
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
        if (wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideLeft, leftReferenceSignal) &&
            wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideRight, rightReferenceSignal) &&
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
        if (TryComputeSideWallSignalDistanceM(
                wallCalibration,
                WallSensorId::SideLeft,
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
        if (TryComputeSideWallSignalDistanceM(
                wallCalibration,
                WallSensorId::SideRight,
                rightMeasuredSignal,
                rightDistanceM))
        {
            corridorErrorM = expectedDistanceM - rightDistanceM;
            return std::isfinite(corridorErrorM);
        }
    }

    return false;
}

