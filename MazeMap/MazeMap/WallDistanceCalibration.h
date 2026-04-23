#pragma once

#include "WallSensorCalibration.h"
#include "WallSensorRuntimeTypes.h"

#include <stdint.h>

namespace MazeMap
{
    class Vehicle;
}

class WallDistanceCalibration
{
public:
    WallDistanceCalibration();

    void Clear();
    bool AddPoint(WallSensorId sensorId, float measuredValue, float actualDistanceM, float ambientLight = 0.0f);
    float Apply(WallSensorId sensorId, float measuredValue, float fallbackDistanceM) const;
    void SetExpectedSideWallDistanceM(float expectedDistanceM);
    float GetExpectedSideWallDistanceM() const;
    void SetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight);
    void SetFrontWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight);
    bool TryGetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const;
    bool TryGetFrontWallBaselineDifferentialLightBand(
        WallSensorId sensorId,
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    void SetFrontWeakestCalibrationDifferentialLightBand(
        WallSensorId sensorId,
        float measuredValue,
        float lowDifferentialLight,
        float highDifferentialLight);
    bool TryGetFrontWeakestCalibrationDifferentialLightBand(
        WallSensorId sensorId,
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    void SetFrontDirectRiseThresholds(
        WallSensorId sensorId,
        float signalBaseline,
        float onRiseThreshold,
        float offRiseThreshold);
    bool TryGetFrontDirectRiseThresholds(
        WallSensorId sensorId,
        float& signalBaseline,
        float& onRiseThreshold,
        float& offRiseThreshold) const;
    bool TryComputeSideWallDistanceThresholds(float latchSignalFraction, float releaseSignalFraction, float& onThresholdM, float& offThresholdM) const;
    void SetSideWallReferenceDifferentialLight(WallSensorId sensorId, float differentialLight);
    void SetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight);
    void SetSideWallReferenceDistanceM(WallSensorId sensorId, float distanceM);
    void SetSideWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight);
    void SetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight);
    bool TryGetSideWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const;
    bool TryGetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const;
    bool TryGetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const;
    bool TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId sensorId, float& differentialLight) const;
    bool TryComputeSideWallMeasuredThresholds(
        WallSensorId sensorId,
        float latchSignalFraction,
        float releaseSignalFraction,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;
    bool TryGetSideWallReferenceDifferentialLight(WallSensorId sensorId, float& differentialLight) const;
    bool TryGetSideWallReferenceDistanceM(WallSensorId sensorId, float& distanceM) const;
    bool TryGetWeakestFrontCalibrationMeasuredValue(WallSensorId sensorId, float& measuredValue) const;
    bool TryComputeFrontWallDistanceThresholds(
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float& onDistanceThresholdM,
        float& offDistanceThresholdM) const;
    bool TryComputeFrontSensorMeasuredThresholds(
        WallSensorId sensorId,
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float ambientLight,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;
    bool TryComputeFrontSensorRepresentativeAmbientLight(WallSensorId sensorId, float& ambientLight) const;
    const MazeMap::WallSensorCalibrationCurve& GetCurve(WallSensorId sensorId) const;

private:
    struct FrontSignalModelCache
    {
        bool valid = false;
        float gain = 0.0f;
        float lightScale = 0.0f;
    };

    MazeMap::WallSensorCalibrationCurve _curves[static_cast<uint8_t>(WallSensorId::Count)];
    mutable FrontSignalModelCache _frontSignalModelCache[2];
    float _frontWallBaselineDifferentialLight[2] = {};
    bool _frontWallBaselineValid[2] = {};
    float _frontWallBaselineDifferentialLightLow[2] = {};
    float _frontWallBaselineDifferentialLightHigh[2] = {};
    bool _frontWallBaselineBandValid[2] = {};
    float _frontWallWeakestCalibrationMeasuredValue[2] = {};
    float _frontWallWeakestCalibrationDifferentialLightLow[2] = {};
    float _frontWallWeakestCalibrationDifferentialLightHigh[2] = {};
    bool _frontWallWeakestCalibrationBandValid[2] = {};
    float _frontWallDirectOnRiseThreshold[2] = {};
    float _frontWallDirectOffRiseThreshold[2] = {};
    float _frontWallDirectSignalBaseline[2] = {};
    bool _frontWallDirectThresholdValid[2] = {};
    float _sideWallBaselineDifferentialLight[2] = {};
    bool _sideWallBaselineValid[2] = {};
    float _sideWallBaselineDifferentialLightLow[2] = {};
    float _sideWallBaselineDifferentialLightHigh[2] = {};
    bool _sideWallBaselineBandValid[2] = {};
    float _sideWallReferenceDifferentialLight[2] = {};
    bool _sideWallReferenceValid[2] = {};
    float _sideWallReferenceDifferentialLightLow[2] = {};
    float _sideWallReferenceDifferentialLightHigh[2] = {};
    bool _sideWallReferenceBandValid[2] = {};
    float _sideWallReferenceDistanceM[2] = {};
    bool _sideWallReferenceDistanceValid[2] = {};
    float _expectedSideWallDistanceM;

    static bool IsSideWallSensor(WallSensorId sensorId);
    static uint8_t SideWallIndex(WallSensorId sensorId);
    static uint8_t FrontWallIndex(WallSensorId sensorId);
    void InvalidateFrontSignalModelCache();
    void InvalidateFrontSignalModelCache(WallSensorId sensorId);
    bool TryGetFrontSignalModel(WallSensorId sensorId, float& gain, float& lightScale) const;
};

extern WallDistanceCalibration gWallDistanceCalibration;

float ComputeDiagonalWallCenterOmegaRadps(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    float rightMeasuredSignal);

bool TryComputeSideWallSignalDistanceM(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredSignal,
    float& distanceM);

float ComputeSignalRiseAboveBaselineValue(
    float measuredDifferentialLight,
    float signalBaseline);

bool IsCalibratedSideDistanceValidForControl(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredDifferentialLight);

bool TryComputeStraightWallCenterErrorM(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    bool leftWall,
    float rightMeasuredSignal,
    bool rightWall,
    float& corridorErrorM);
