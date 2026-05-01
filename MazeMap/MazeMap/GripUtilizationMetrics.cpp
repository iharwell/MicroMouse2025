#include "pch.h"
#include "GripUtilizationMetrics.h"

#include "EstimatorPredictModel.h"

#include <algorithm>
#include <cmath>

namespace MazeMap
{
    namespace
    {
        float Clamp01(float value) noexcept
        {
            if (!std::isfinite(value))
            {
                return 0.0f;
            }

            if (value <= 0.0f)
            {
                return 0.0f;
            }

            if (value >= 1.0f)
            {
                return 1.0f;
            }

            return value;
        }

        float PositiveOr(float value, float fallback) noexcept
        {
            return (std::isfinite(value) && (value > 0.0f)) ? value : fallback;
        }

        float PrecursorSeverity(float utilization) noexcept
        {
            if (!std::isfinite(utilization))
            {
                return 0.0f;
            }

            return Clamp01((utilization - 0.75f) / 0.30f);
        }
    }

    GripUtilizationSnapshot GripUtilizationMetrics::Compute(
        const VehicleState::StateVector& currentState,
        const AppliedTorqueEstimate& appliedTorque,
        const PlantPreparedParams& params,
        const GripUtilizationInputs& inputs) noexcept
    {
        EstimatorPredictModel predictModel;
        EstimatorPredictModel::PredictInput predictInput{};
        predictInput.currentState = currentState;
        predictInput.leftAppliedBankTorqueNm = appliedTorque.leftAppliedBankTorqueNm;
        predictInput.rightAppliedBankTorqueNm = appliedTorque.rightAppliedBankTorqueNm;
        predictInput.fanDutyCycle = inputs.fanDutyCycle;
        const PlantDerivatives derivatives =
            predictModel.EvaluateStep(
                predictInput,
                params);
        const float leftBankPreProjectionUtilization =
            (std::max)(
                0.0f,
                std::isfinite(derivatives.contactForces.LeftBankMaxPreProjectionUtilization()) ?
                    derivatives.contactForces.LeftBankMaxPreProjectionUtilization() :
                    0.0f);
        const float rightBankPreProjectionUtilization =
            (std::max)(
                0.0f,
                std::isfinite(derivatives.contactForces.RightBankMaxPreProjectionUtilization()) ?
                    derivatives.contactForces.RightBankMaxPreProjectionUtilization() :
                    0.0f);
        const float longitudinalClosureSeverity =
            std::isfinite(inputs.longitudinalClosureSeverity) ? inputs.longitudinalClosureSeverity : 0.0f;
        const float differentialClosureSeverity =
            std::isfinite(inputs.differentialClosureSeverity) ? inputs.differentialClosureSeverity : 0.0f;
        const float lateralAccelerationSeverity =
            std::isfinite(inputs.lateralAccelerationSeverity) ? inputs.lateralAccelerationSeverity : 0.0f;
        const float yawConsistencySeverity =
            std::isfinite(inputs.yawConsistencySeverity) ? inputs.yawConsistencySeverity : 0.0f;
        const float leftBankAnomalySeverity =
            std::isfinite(inputs.leftBankAnomalySeverity) ? inputs.leftBankAnomalySeverity : 0.0f;
        const float rightBankAnomalySeverity =
            std::isfinite(inputs.rightBankAnomalySeverity) ? inputs.rightBankAnomalySeverity : 0.0f;

        GripUtilizationSnapshot snapshot{};
        snapshot.longitudinalClosureSeverity =
            Clamp01(
                (std::max)(
                    longitudinalClosureSeverity,
                    std::fabs(derivatives.longitudinalAccelMps2) /
                        PositiveOr(params.raw.combinedAccelSustainedMps2, 1.0f)));
        snapshot.differentialClosureSeverity =
            Clamp01(
                (std::max)(
                    differentialClosureSeverity,
                    std::fabs(derivatives.yawAccelRadps2) /
                        PositiveOr(params.raw.combinedAccelNominalMps2, 1.0f)));
        snapshot.lateralAccelerationSeverity =
            Clamp01(
                (std::max)(
                    lateralAccelerationSeverity,
                    std::fabs(derivatives.lateralAccelMps2) /
                        PositiveOr(params.raw.combinedAccelPeakMps2, 1.0f)));
        snapshot.yawConsistencySeverity =
            Clamp01(
                (std::max)(
                    yawConsistencySeverity,
                    std::fabs(currentState(VehicleState::kR)) /
                        PositiveOr(params.stopExitYawRateRadps, 1.0f)));

        snapshot.leftBankPreProjectionUtilization = leftBankPreProjectionUtilization;
        snapshot.rightBankPreProjectionUtilization = rightBankPreProjectionUtilization;

        snapshot.leftBankAnomalySeverity =
            Clamp01((std::max)(
                (std::max)(
                    leftBankAnomalySeverity,
                    PrecursorSeverity(snapshot.leftBankPreProjectionUtilization)),
                appliedTorque.leftCurrentLimited ? 1.0f : 0.0f));
        snapshot.rightBankAnomalySeverity =
            Clamp01((std::max)(
                (std::max)(
                    rightBankAnomalySeverity,
                    PrecursorSeverity(snapshot.rightBankPreProjectionUtilization)),
                appliedTorque.rightCurrentLimited ? 1.0f : 0.0f));

        if (!appliedTorque.batteryVoltageAvailable)
        {
            snapshot.leftBankAnomalySeverity = Clamp01((std::max)(snapshot.leftBankAnomalySeverity, 0.1f));
            snapshot.rightBankAnomalySeverity = Clamp01((std::max)(snapshot.rightBankAnomalySeverity, 0.1f));
        }

        return snapshot;
    }
}
