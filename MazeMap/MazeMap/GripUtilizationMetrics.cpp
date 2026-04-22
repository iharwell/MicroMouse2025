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

        float FiniteOrZero(float value) noexcept
        {
            return std::isfinite(value) ? value : 0.0f;
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
            (std::max)(0.0f, FiniteOrZero(derivatives.contactForces.LeftBankMaxPreProjectionUtilization()));
        const float rightBankPreProjectionUtilization =
            (std::max)(0.0f, FiniteOrZero(derivatives.contactForces.RightBankMaxPreProjectionUtilization()));

        GripUtilizationSnapshot snapshot{};
        snapshot.longitudinalClosureSeverity =
            Clamp01(
                (std::max)(
                    FiniteOrZero(inputs.longitudinalClosureSeverity),
                    std::fabs(derivatives.longitudinalAccelMps2) /
                        PositiveOr(params.raw.combinedAccelSustainedMps2, 1.0f)));
        snapshot.differentialClosureSeverity =
            Clamp01(
                (std::max)(
                    FiniteOrZero(inputs.differentialClosureSeverity),
                    std::fabs(derivatives.yawAccelRadps2) /
                        PositiveOr(params.raw.combinedAccelNominalMps2, 1.0f)));
        snapshot.lateralAccelerationSeverity =
            Clamp01(
                (std::max)(
                    FiniteOrZero(inputs.lateralAccelerationSeverity),
                    std::fabs(derivatives.lateralAccelMps2) /
                        PositiveOr(params.raw.combinedAccelPeakMps2, 1.0f)));
        snapshot.yawConsistencySeverity =
            Clamp01(
                (std::max)(
                    FiniteOrZero(inputs.yawConsistencySeverity),
                    std::fabs(currentState(VehicleState::kR)) /
                        PositiveOr(params.stopExitYawRateRadps, 1.0f)));

        snapshot.leftBankPreProjectionUtilization = leftBankPreProjectionUtilization;
        snapshot.rightBankPreProjectionUtilization = rightBankPreProjectionUtilization;

        snapshot.leftBankAnomalySeverity =
            Clamp01((std::max)(
                (std::max)(
                    FiniteOrZero(inputs.leftBankAnomalySeverity),
                    PrecursorSeverity(snapshot.leftBankPreProjectionUtilization)),
                appliedTorque.leftCurrentLimited ? 1.0f : 0.0f));
        snapshot.rightBankAnomalySeverity =
            Clamp01((std::max)(
                (std::max)(
                    FiniteOrZero(inputs.rightBankAnomalySeverity),
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
