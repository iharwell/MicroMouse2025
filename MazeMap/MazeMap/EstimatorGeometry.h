#pragma once

#include "Defines.h"
#include "EigenCompat.h"

namespace MazeMap
{
    struct PlantParams;
    struct PlantPreparedParams;
    class PlantModel;

    struct EstimatorGeometry
    {
        float effectiveWheelRadiusM = 0.0f;
        float inverseEffectiveWheelRadiusPerM = 0.0f;
        float effectiveTrackWidthM = 0.0f;
        float halfEffectiveTrackWidthM = 0.0f;
        float inverseEffectiveTrackWidthPerM = 0.0f;
        float effectiveYawLeverArmM = 0.0f;
        Eigen::Vector2f imuPositionBodyM = Eigen::Vector2f::Zero();
    };

    struct AppliedTorqueEstimate
    {
        float leftAppliedBankTorqueNm = 0.0f;
        float rightAppliedBankTorqueNm = 0.0f;
        bool leftCurrentLimited = false;
        bool rightCurrentLimited = false;
        bool batteryVoltageAvailable = false;
    };

    struct GripUtilizationSnapshot
    {
        float longitudinalClosureSeverity = 0.0f;
        float differentialClosureSeverity = 0.0f;
        float lateralAccelerationSeverity = 0.0f;
        float yawConsistencySeverity = 0.0f;

        float leftBankAnomalySeverity = 0.0f;
        float rightBankAnomalySeverity = 0.0f;

        float leftBankPreProjectionUtilization = 0.0f;
        float rightBankPreProjectionUtilization = 0.0f;
    };

    struct TransientContactMemoryState
    {
        float leftBankMemory = 0.0f;
        float rightBankMemory = 0.0f;
    };

    struct RegripRecoveryState
    {
        bool leftBankInRecovery = false;
        bool rightBankInRecovery = false;

        float leftBankRecoveryScore = 0.0f;
        float rightBankRecoveryScore = 0.0f;

        float leftBankRecoveryTimeRemainingS = 0.0f;
        float rightBankRecoveryTimeRemainingS = 0.0f;
    };

    struct FrozenCycleSchedule
    {
        bool exactStationaryLock = false;
        bool planarAccelForwardUpdateEnabled = false;
        bool planarAccelLateralUpdateEnabled = false;
        bool softOdometryEnabled = false;

        float closureCovarianceScaleLeft = 1.0f;
        float closureCovarianceScaleRight = 1.0f;
        float lateralPseudoMeasurementCovarianceScale = 1.0f;

        float forwardSpeedProcessNoiseScale = 1.0f;
        float lateralSpeedProcessNoiseScale = 1.0f;
        float yawRateProcessNoiseScale = 1.0f;
        float leftWheelSpeedProcessNoiseScale = 1.0f;
        float rightWheelSpeedProcessNoiseScale = 1.0f;

        float leftEdgeShapeStrength = 0.0f;
        float rightEdgeShapeStrength = 0.0f;

        bool leftBankHoldoffActive = false;
        bool rightBankHoldoffActive = false;
    };

    struct ModelCycleContext
    {
        EstimatorGeometry geometry{};
        AppliedTorqueEstimate appliedTorque{};
        GripUtilizationSnapshot utilization{};
        FrozenCycleSchedule schedule{};
        TransientContactMemoryState memory{};
        RegripRecoveryState regrip{};
        float dtS = 0.0f;
    };

    EXPORT EstimatorGeometry BuildEstimatorGeometry(const PlantParams& params) noexcept;
    EXPORT EstimatorGeometry BuildEstimatorGeometry(const PlantPreparedParams& params) noexcept;
}
