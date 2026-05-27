#pragma once

#include "Direction.h"
#include "WallSensorCalibration.h"

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
    bool AddFrontLeftPoint(float measuredValue, float actualDistanceM, float ambientLight = 0.0f);
    bool AddFrontRightPoint(float measuredValue, float actualDistanceM, float ambientLight = 0.0f);
    bool AddSidePoint(MazeMap::RelativeDirection side, float measuredValue, float actualDistanceM, float ambientLight = 0.0f);
    float ApplyFrontLeft(float measuredValue, float fallbackDistanceM) const;
    float ApplyFrontRight(float measuredValue, float fallbackDistanceM) const;
    float ApplySide(MazeMap::RelativeDirection side, float measuredValue, float fallbackDistanceM) const;
    void SetExpectedSideWallDistanceM(float expectedDistanceM);
    float GetExpectedSideWallDistanceM() const;
    void SetFrontLeftWallBaselineDifferentialLight(float differentialLight);
    void SetFrontRightWallBaselineDifferentialLight(float differentialLight);
    void SetFrontLeftWallBaselineDifferentialLightBand(float lowDifferentialLight, float highDifferentialLight);
    void SetFrontRightWallBaselineDifferentialLightBand(float lowDifferentialLight, float highDifferentialLight);
    bool TryGetFrontLeftWallBaselineDifferentialLight(float& differentialLight) const;
    bool TryGetFrontRightWallBaselineDifferentialLight(float& differentialLight) const;
    bool TryGetFrontLeftWallBaselineDifferentialLightBand(
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    bool TryGetFrontRightWallBaselineDifferentialLightBand(
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    void SetFrontLeftWeakestCalibrationDifferentialLightBand(
        float measuredValue,
        float lowDifferentialLight,
        float highDifferentialLight);
    void SetFrontRightWeakestCalibrationDifferentialLightBand(
        float measuredValue,
        float lowDifferentialLight,
        float highDifferentialLight);
    bool TryGetFrontLeftWeakestCalibrationDifferentialLightBand(
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    bool TryGetFrontRightWeakestCalibrationDifferentialLightBand(
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    void SetFrontLeftDirectRiseThresholds(
        float signalBaseline,
        float onRiseThreshold,
        float offRiseThreshold);
    void SetFrontRightDirectRiseThresholds(
        float signalBaseline,
        float onRiseThreshold,
        float offRiseThreshold);
    bool TryGetFrontLeftDirectRiseThresholds(
        float& signalBaseline,
        float& onRiseThreshold,
        float& offRiseThreshold) const;
    bool TryGetFrontRightDirectRiseThresholds(
        float& signalBaseline,
        float& onRiseThreshold,
        float& offRiseThreshold) const;
    void SetSideWallReferenceDifferentialLight(MazeMap::RelativeDirection side, float differentialLight);
    void SetSideWallReferenceDifferentialLightBand(MazeMap::RelativeDirection side, float lowDifferentialLight, float highDifferentialLight);
    void SetSideWallReferenceDistanceM(MazeMap::RelativeDirection side, float distanceM);
    void SetSideWallBaselineDifferentialLight(MazeMap::RelativeDirection side, float differentialLight);
    void SetSideWallBaselineDifferentialLightBand(MazeMap::RelativeDirection side, float lowDifferentialLight, float highDifferentialLight);
    bool TryGetSideWallBaselineDifferentialLight(MazeMap::RelativeDirection side, float& differentialLight) const;
    bool TryGetSideWallReferenceDifferentialLightBand(MazeMap::RelativeDirection side, float& lowDifferentialLight, float& highDifferentialLight) const;
    bool TryGetSideWallBaselineDifferentialLightBand(MazeMap::RelativeDirection side, float& lowDifferentialLight, float& highDifferentialLight) const;
    bool TryComputeSideWallNormalizedReferenceDifferentialLight(MazeMap::RelativeDirection side, float& differentialLight) const;
    bool TryComputeSideWallSignalDistanceM(
        MazeMap::RelativeDirection side,
        float measuredSignal,
        float& distanceM) const;
    bool TryComputeSideWallSignalRiseMetrics(
        MazeMap::RelativeDirection side,
        float measuredDifferentialLight,
        float latchSignalFraction,
        float releaseSignalFraction,
        float missSignalFractionOfLatch,
        float& signalRise,
        float& latchRiseThreshold,
        float& missRiseThreshold) const;
    bool IsSideWallSignalClassifiable(
        bool signalMetricsValid,
        float signalRise,
        float latchRiseThreshold,
        float missRiseThreshold) const noexcept;
    bool IsSideWallObservationEligible(
        bool detectionWindowValid,
        bool signalClassifiable) const noexcept;
    bool IsSideWallControlRangeValid(
        bool observationEligible,
        bool transitionDetected,
        bool signalMetricsValid,
        float signalRise,
        float latchRiseThreshold) const noexcept;
    bool DetectSideWallTransitionFromSignalRise(
        bool detectionWindowValid,
        bool signalMetricsValid,
        float signalRise,
        float latchRiseThreshold,
        float transitionSignalFractionOfLatch,
        float& previousSignalRise,
        bool& previousValid) const noexcept;
    bool ComputeSideWallObservationHit(
        MazeMap::RelativeDirection side,
        float measuredDifferentialLight,
        bool detectionWindowValid) const;
    bool UpdateSideWallState(
        MazeMap::RelativeDirection side,
        float measuredDifferentialLight,
        bool detectionWindowValid,
        float& filteredSignal,
        bool& signalInitialized,
        bool& currentState) const;
    bool TryComputeSideWallMeasuredThresholds(
        MazeMap::RelativeDirection side,
        float latchSignalFraction,
        float releaseSignalFraction,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;
    bool TryGetSideWallReferenceDifferentialLight(MazeMap::RelativeDirection side, float& differentialLight) const;
    bool TryGetSideWallReferenceDistanceM(MazeMap::RelativeDirection side, float& distanceM) const;
    bool TryGetFrontLeftWeakestCalibrationMeasuredValue(float& measuredValue) const;
    bool TryGetFrontRightWeakestCalibrationMeasuredValue(float& measuredValue) const;
    bool TryComputeFrontWallDistanceThresholds(
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float& onDistanceThresholdM,
        float& offDistanceThresholdM) const;
    bool TryComputeFrontLeftSensorMeasuredThresholds(
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float ambientLight,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;
    bool TryComputeFrontRightSensorMeasuredThresholds(
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float ambientLight,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;

private:
    static constexpr uint8_t kFrontLeftIndex = 0U;
    static constexpr uint8_t kFrontRightIndex = 1U;
    static constexpr uint8_t kSideLeftIndex = 0U;
    static constexpr uint8_t kSideRightIndex = 1U;

    MazeMap::WallSensorCalibrationCurve _frontLeftCurve;
    MazeMap::WallSensorCalibrationCurve _frontRightCurve;
    MazeMap::WallSensorCalibrationCurve _sideLeftCurve;
    MazeMap::WallSensorCalibrationCurve _sideRightCurve;
    mutable bool _frontSignalModelCacheValid[2] = {};
    mutable float _frontSignalModelCacheGain[2] = {};
    mutable float _frontSignalModelCacheLightScale[2] = {};
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

    bool AddFrontPoint(uint8_t frontIndex, MazeMap::WallSensorCalibrationCurve& curve, float measuredValue, float actualDistanceM, float ambientLight);
    bool AddSidePointAt(uint8_t sideIndex, float measuredValue, float actualDistanceM, float ambientLight);
    float ApplyCurve(
        const MazeMap::WallSensorCalibrationCurve& curve,
        MazeMap::WallSensorCalibrationMode mode,
        float measuredValue,
        float fallbackDistanceM) const;
    static bool IsSideDirection(MazeMap::RelativeDirection side);
    static uint8_t SideWallIndex(MazeMap::RelativeDirection side);
    MazeMap::WallSensorCalibrationCurve& SideCurve(MazeMap::RelativeDirection side);
    const MazeMap::WallSensorCalibrationCurve& SideCurve(MazeMap::RelativeDirection side) const;
    void SetFrontWallBaselineDifferentialLight(uint8_t frontIndex, float differentialLight);
    void SetFrontWallBaselineDifferentialLightBand(uint8_t frontIndex, float lowDifferentialLight, float highDifferentialLight);
    bool TryGetFrontWallBaselineDifferentialLight(uint8_t frontIndex, float& differentialLight) const;
    bool TryGetFrontWallBaselineDifferentialLightBand(uint8_t frontIndex, float& lowDifferentialLight, float& highDifferentialLight) const;
    void SetFrontWeakestCalibrationDifferentialLightBand(
        uint8_t frontIndex,
        float measuredValue,
        float lowDifferentialLight,
        float highDifferentialLight);
    bool TryGetFrontWeakestCalibrationDifferentialLightBand(
        uint8_t frontIndex,
        float& lowDifferentialLight,
        float& highDifferentialLight) const;
    void SetFrontDirectRiseThresholds(uint8_t frontIndex, float signalBaseline, float onRiseThreshold, float offRiseThreshold);
    bool TryGetFrontDirectRiseThresholds(
        uint8_t frontIndex,
        float& signalBaseline,
        float& onRiseThreshold,
        float& offRiseThreshold) const;
    bool TryGetWeakestFrontCalibrationMeasuredValue(const MazeMap::WallSensorCalibrationCurve& curve, float& measuredValue) const;
    bool TryComputeFrontSensorMeasuredThresholds(
        uint8_t frontIndex,
        const MazeMap::WallSensorCalibrationCurve& curve,
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float ambientLight,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const;
    bool TryComputeFrontSensorRepresentativeAmbientLight(const MazeMap::WallSensorCalibrationCurve& curve, float& ambientLight) const;
    void InvalidateFrontSignalModelCache();
    void InvalidateFrontSignalModelCache(uint8_t frontIndex);
    bool TryGetFrontSignalModel(uint8_t frontIndex, const MazeMap::WallSensorCalibrationCurve& curve, float& gain, float& lightScale) const;
    static float ComputeCalibratedSideSignalRise(float measuredDifferentialLight, float signalBaseline) noexcept;
};

extern WallDistanceCalibration gWallDistanceCalibration;

float ComputeDiagonalWallCenterOmegaRadps(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    float rightMeasuredSignal);

bool TryComputeStraightWallCenterErrorM(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    bool leftWall,
    float rightMeasuredSignal,
    bool rightWall,
    float& corridorErrorM);
