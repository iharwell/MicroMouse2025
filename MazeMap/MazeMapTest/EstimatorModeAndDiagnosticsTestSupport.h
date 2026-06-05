#pragma once

#include "EstimatorFilterTestSupport.h"
#include "..\MazeMap\PlantModel.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>


namespace MazeMap
{
    namespace EstimatorModeAndDiagnosticsTestSupport
    {
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;
        using PairCovarianceMatrix = Eigen::Matrix<float, 2, 2>;

        constexpr float kPlanarAccelUpdateTestDtSeconds = 0.001f;

        inline const wchar_t* BoolText(const bool value) noexcept
        {
            return value ? L"true" : L"false";
        }

        inline float NaN() noexcept
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        inline StateVector NaNState()
        {
            return StateVector::Constant(NaN());
        }

        inline CovarianceMatrix NaNCovariance()
        {
            return CovarianceMatrix::Constant(NaN());
        }

        inline std::wstring StatusMessage(const bool completed, const int step, const wchar_t* operation)
        {
            return std::wstring(L"setup_completed=") + BoolText(completed) +
                L" operation=" + operation +
                L" step=" + std::to_wstring(step);
        }

        inline std::wstring ValueMessage(
            const wchar_t* field,
            const float actual,
            const wchar_t* detail)
        {
            return std::wstring(field) +
                L" actual=" + std::to_wstring(actual) +
                L" detail=" + detail;
        }

        inline std::wstring BoolMessage(
            const wchar_t* field,
            const bool actual,
            const wchar_t* expected,
            const wchar_t* detail)
        {
            return std::wstring(field) +
                L" actual=" + BoolText(actual) +
                L" expected=" + expected +
                L" detail=" + detail;
        }

        inline std::wstring LimitMessage(
            const wchar_t* field,
            const float actual,
            const wchar_t* relation,
            const float limit,
            const wchar_t* detail)
        {
            return std::wstring(field) +
                L" actual=" + std::to_wstring(actual) +
                L" expected=" + relation + L" " + std::to_wstring(limit) +
                L" detail=" + detail;
        }

        struct ScenarioStatus final
        {
            bool completed = true;
            int step = -1;
            const wchar_t* operation = L"complete";
        };

        inline void RecordOperation(
            ScenarioStatus& status,
            const bool accepted,
            const int step,
            const wchar_t* operation) noexcept
        {
            if (!status.completed)
            {
                return;
            }

            if (accepted)
            {
                return;
            }

            status.completed = false;
            status.step = step;
            status.operation = operation;
        }

        inline std::wstring ScenarioMessage(const ScenarioStatus& status)
        {
            return StatusMessage(status.completed, status.step, status.operation);
        }

        inline std::wstring ScenarioPairMessage(
            const ScenarioStatus& first,
            const ScenarioStatus& second,
            const wchar_t* firstName,
            const wchar_t* secondName)
        {
            return std::wstring(firstName) + L"{" + ScenarioMessage(first) + L"} " +
                secondName + L"{" + ScenarioMessage(second) + L"}";
        }

        inline std::wstring ScenarioTripleMessage(
            const ScenarioStatus& first,
            const ScenarioStatus& second,
            const ScenarioStatus& third,
            const wchar_t* firstName,
            const wchar_t* secondName,
            const wchar_t* thirdName)
        {
            return std::wstring(firstName) + L"{" + ScenarioMessage(first) + L"} " +
                secondName + L"{" + ScenarioMessage(second) + L"} " +
                thirdName + L"{" + ScenarioMessage(third) + L"}";
        }

        template <typename Matrix>
        inline float MaxAbsCoeff(const Matrix& matrix)
        {
            return matrix.cwiseAbs().maxCoeff();
        }

        template <typename Matrix>
        inline float MaxAbsDelta(const Matrix& left, const Matrix& right)
        {
            return MaxAbsCoeff(left - right);
        }

        inline StateVector BuildPlanarAccelUpdateTestState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(0) = 0.03f;
            state(1) = 0.11f;
            state(2) = NormalizeAngle(0.08f);
            state(3) = 1.2f;
            state(4) = 0.02f;
            state(5) = 0.15f;
            return state;
        }

        inline CovarianceMatrix BuildPlanarAccelUpdateTestCovariance() noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.02f * 0.02f;
            covariance(1, 1) = 0.02f * 0.02f;
            covariance(2, 2) = 0.05f * 0.05f;
            covariance(3, 3) = 0.20f * 0.20f;
            covariance(4, 4) = 0.15f * 0.15f;
            covariance(5, 5) = 0.25f * 0.25f;
            covariance(6, 6) = 0.50f * 0.50f;
            covariance(7, 7) = 0.50f * 0.50f;
            covariance(8, 8) = 0.50f * 0.50f;
            return covariance;
        }

        inline App::Internal::CommandVector BuildPlanarAccelUpdateTestControl() noexcept
        {
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.26f);
            control.SetRightCommand(0.23f);
            return control;
        }

        inline ImuAccelObs BuildPlanarAccelObservation(
            EstimatorTestRuntime& runtime,
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const float rightAccelDeltaMps2,
            const float forwardAccelDeltaMps2) noexcept
        {
            runtime.runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtime.runtimeState.SetHeading(state(2));
            runtime.runtimeState.SetForwardVelocity(state(3));
            runtime.runtimeState.SetRightwardVelocity(state(4));
            runtime.runtimeState.SetYawRate(state(5));
            runtime.runtimeState.SetForwardAccelerationResidual(state(6));
            runtime.runtimeState.SetRightwardAccelerationResidual(state(7));
            runtime.runtimeState.SetYawAccelResidual(state(8));

            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                state(3),
                state(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            runtime.runtimeState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            runtime.runtimeState.SetWheelSpeedRight(rightWheelSpeedRadps);
            runtime.plantModel.integrate(control, kPlanarAccelUpdateTestDtSeconds);

            const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
            const float yawRateSquaredRadps2 = state(5) * state(5);
            const Eigen::Vector2f predicted(
                runtime.runtimeState.GetRightAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                    (runtime.runtimeState.GetYawAccel() * imuLeverArmBodyM.y()),
                runtime.runtimeState.GetForwardAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                    (runtime.runtimeState.GetYawAccel() * imuLeverArmBodyM.x()));
            return ImuAccelObs(
                true,
                predicted.y() + forwardAccelDeltaMps2,
                predicted.x() + rightAccelDeltaMps2);
        }

        inline StateVector BuildPivotConflictInitialState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.20f;
            return state;
        }

        inline CovarianceMatrix BuildPivotConflictInitialCovariance() noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.001f * 0.001f;
            covariance(1, 1) = 0.001f * 0.001f;
            covariance(2, 2) = 0.01f * 0.01f;
            covariance(3, 3) = 0.04f * 0.04f;
            covariance(4, 4) = 0.04f * 0.04f;
            covariance(5, 5) = 0.20f * 0.20f;
            covariance(6, 6) = 10.0f * 10.0f;
            covariance(7, 7) = 10.0f * 10.0f;
            covariance(8, 8) = 0.02f * 0.02f;
            return covariance;
        }

        inline StateVector BuildDefaultMovingState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.10f;
            return state;
        }

        inline CovarianceMatrix BuildDefaultMovingCovariance() noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.01f * 0.01f;
            covariance(1, 1) = 0.01f * 0.01f;
            covariance(2, 2) = 0.03f * 0.03f;
            covariance(3, 3) = 0.05f * 0.05f;
            covariance(4, 4) = 0.05f * 0.05f;
            covariance(5, 5) = 0.10f * 0.10f;
            covariance(6, 6) = 0.30f * 0.30f;
            covariance(7, 7) = 0.30f * 0.30f;
            covariance(8, 8) = 0.03f * 0.03f;
            return covariance;
        }

        inline StateVector BuildLaunchTransientInitialState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.20f;
            state(4) = 0.40f;
            return state;
        }

        inline CovarianceMatrix BuildLaunchTransientInitialCovariance() noexcept
        {
            CovarianceMatrix covariance = BuildDefaultMovingCovariance();
            covariance(4, 4) = 0.30f * 0.30f;
            covariance(5, 5) = 0.05f * 0.05f;
            return covariance;
        }

        inline StateVector BuildGripInitialState(
            const float forwardVelocityMps,
            const float yawRateRadps) noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = 0.0f;
            state(3) = forwardVelocityMps;
            state(4) = 0.20f;
            state(5) = yawRateRadps;
            return state;
        }

        inline CovarianceMatrix BuildGripInitialCovariance(const float lateralVarianceMps2) noexcept
        {
            CovarianceMatrix covariance = BuildDefaultMovingCovariance();
            covariance(4, 4) = lateralVarianceMps2;
            covariance(5, 5) = 0.10f * 0.10f;
            return covariance;
        }

        struct EncoderPairNoiseResult final
        {
            PairCovarianceMatrix covariance = PairCovarianceMatrix::Constant(NaN());
            float expectedVarianceRadps2 = NaN();
            float expectedCovarianceRadps2 = NaN();
        };

        inline EncoderPairNoiseResult RunEncoderPairNoiseScenario()
        {
            EncoderPairNoiseResult result{};
            EstimatorTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            EncoderObs observation{};
            observation.SetLeftWheelSpeedRadps(1.0f);
            observation.SetRightWheelSpeedRadps(1.0f);

            const PairCovarianceMatrix sqrtNoise =
                plantModel.encoderPairSqrtNoise(
                    observation,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps);
            result.covariance = sqrtNoise * sqrtNoise.transpose();

            const float varianceUMps2 =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps * kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps * kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalTrackWidthM();
            const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
            const float invWheelRadius2 = 1.0f / (wheelRadiusM * wheelRadiusM);
            result.expectedVarianceRadps2 =
                (varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;
            result.expectedCovarianceRadps2 =
                (varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;
            return result;
        }
    }

    class EstimatorModeAndDiagnosticsTest final
    {
    public:
        using StateVector = EstimatorModeAndDiagnosticsTestSupport::StateVector;
        using CovarianceMatrix = EstimatorModeAndDiagnosticsTestSupport::CovarianceMatrix;

        static StateVector WorkingState(const Estimator& core) noexcept
        {
            return core.workingState();
        }

        static CovarianceMatrix WorkingCovariance(const Estimator& core) noexcept
        {
            return core.workingCovariance();
        }

        static bool LastUpdateAttempted(const Estimator& core) noexcept
        {
            return core.LastUpdateAttempted();
        }

        static bool LastUpdateAccepted(const Estimator& core) noexcept
        {
            return core.LastUpdateAccepted();
        }

        static float LastUpdateNis(const Estimator& core) noexcept
        {
            return core.LastUpdateNis();
        }

        static bool Reset(
            Estimator& core,
            const StateVector& state,
            const CovarianceMatrix& covariance)
        {
            return core.reset(state, covariance);
        }

        static CovarianceMatrix BuildDefaultInitialCovariance()
        {
            return Estimator::BuildDefaultInitialCovariance();
        }
    };

    namespace EstimatorModeAndDiagnosticsTestSupport
    {
        inline ScenarioStatus PrimeCoreForPlanarAccelUpdate(
            Estimator& core,
            const StateVector& initialState,
            const CovarianceMatrix& initialCovariance,
            const App::Internal::CommandVector& control)
        {
            ScenarioStatus status{};
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(core, initialState, initialCovariance);
            RecordOperation(status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return status;
            }

            const bool predictAccepted = core.predict(kPlanarAccelUpdateTestDtSeconds, control);
            RecordOperation(status, predictAccepted, -1, L"predict");
            if (!predictAccepted)
            {
                return status;
            }

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(0);
            encoder.SetTotalRightCounts(0);
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                EstimatorModeAndDiagnosticsTest::WorkingState(core)(3),
                EstimatorModeAndDiagnosticsTest::WorkingState(core)(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const bool encoderAccepted =
                core.updateEncoderPair(encoder, kPlanarAccelUpdateTestDtSeconds, true);
            RecordOperation(status, encoderAccepted, -1, L"encoder");
            return status;
        }

        struct PivotConflictResult final
        {
            ScenarioStatus status{};
            StateVector stateBeforePivot = NaNState();
            StateVector stateAfterPredict = NaNState();
            StateVector stateAfterEncoder = NaNState();
            StateVector stateAfterPivot = NaNState();
            CovarianceMatrix covarianceAfterPredict = NaNCovariance();
            CovarianceMatrix covarianceAfterEncoder = NaNCovariance();
            CovarianceMatrix covarianceAfterPivot = NaNCovariance();
            float pivotEncoderLeftWheelSpeedRadps = NaN();
            float pivotEncoderRightWheelSpeedRadps = NaN();
            float runtimeLeftWheelSpeedRadps = NaN();
            float runtimeRightWheelSpeedRadps = NaN();
            float pivotEncoderNis = NaN();
            float pivotYawNis = NaN();
            float expectedYawNis = NaN();
            float expectedYawRateRadps = NaN();
            float expectedYawVarianceRadps2 = NaN();
            float gyroCorrectedYawRateRadps = NaN();
            float encoderDerivedYawRateRadps = NaN();
            bool pivotEncoderAttempted = false;
            bool pivotEncoderAccepted = false;
            bool pivotEncoderLastAccepted = false;
            bool pivotYawAttempted = false;
            bool pivotYawAccepted = false;
            bool pivotYawLastAccepted = false;
        };

        inline PivotConflictResult RunPivotConflictScenario()
        {
            PivotConflictResult result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(
                    core,
                    BuildPivotConflictInitialState(),
                    BuildPivotConflictInitialCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return result;
            }

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.60f);
            control.SetRightCommand(-0.60f);

            constexpr float pivotDtSeconds = 0.001f;
            constexpr int pivotLegacyStepScale = 10;
            int seedLeftCounts = 0;
            int seedRightCounts = 0;
            for (int seedIndex = 0; seedIndex < (8 * pivotLegacyStepScale); ++seedIndex)
            {
                seedLeftCounts += (100 / pivotLegacyStepScale);
                seedRightCounts -= (100 / pivotLegacyStepScale);
                const bool predictAccepted = core.predict(pivotDtSeconds, control);
                RecordOperation(result.status, predictAccepted, seedIndex, L"seed_predict");
                if (!predictAccepted)
                {
                    return result;
                }

                EncoderObs seedEncoder{};
                seedEncoder.SetTotalLeftCounts(seedLeftCounts);
                seedEncoder.SetTotalRightCounts(seedRightCounts);
                seedEncoder.SetLeftWheelSpeedRadps(12.0f);
                seedEncoder.SetRightWheelSpeedRadps(-12.0f);
                runtime.runtimeState.SetWheelSpeedLeft(seedEncoder.LeftWheelSpeedRadps());
                runtime.runtimeState.SetWheelSpeedRight(seedEncoder.RightWheelSpeedRadps());
                const bool seedEncoderAccepted =
                    core.updateEncoderPair(seedEncoder, pivotDtSeconds, false);
                RecordOperation(result.status, seedEncoderAccepted, seedIndex, L"seed_encoder");
                if (!seedEncoderAccepted)
                {
                    return result;
                }

                const bool seedYawAccepted = core.updateYawRate(0.0f);
                RecordOperation(result.status, seedYawAccepted, seedIndex, L"seed_yaw");
                if (!seedYawAccepted)
                {
                    return result;
                }
            }

            result.stateBeforePivot = EstimatorModeAndDiagnosticsTest::WorkingState(core);
            constexpr float pivotGyroCorrectedYawRateRadps = 1.80f;
            EncoderObs pivotEncoderObservation{};
            pivotEncoderObservation.SetLeftWheelSpeedRadps(18.0f);
            pivotEncoderObservation.SetRightWheelSpeedRadps(-18.0f);
            result.encoderDerivedYawRateRadps =
                runtime.plantModel.measuredYawRateRadps(pivotEncoderObservation);

            const bool predictAccepted = core.predict(pivotDtSeconds, control);
            RecordOperation(result.status, predictAccepted, -1, L"pivot_predict");
            if (!predictAccepted)
            {
                return result;
            }

            result.stateAfterPredict = EstimatorModeAndDiagnosticsTest::WorkingState(core);
            result.covarianceAfterPredict = EstimatorModeAndDiagnosticsTest::WorkingCovariance(core);

            pivotEncoderObservation.SetTotalLeftCounts(seedLeftCounts + (120 / pivotLegacyStepScale));
            pivotEncoderObservation.SetTotalRightCounts(seedRightCounts - (120 / pivotLegacyStepScale));
            runtime.runtimeState.SetWheelSpeedLeft(pivotEncoderObservation.LeftWheelSpeedRadps());
            runtime.runtimeState.SetWheelSpeedRight(pivotEncoderObservation.RightWheelSpeedRadps());
            result.pivotEncoderLeftWheelSpeedRadps = pivotEncoderObservation.LeftWheelSpeedRadps();
            result.pivotEncoderRightWheelSpeedRadps = pivotEncoderObservation.RightWheelSpeedRadps();
            result.runtimeLeftWheelSpeedRadps = runtime.runtimeState.GetWheelSpeedLeft();
            result.runtimeRightWheelSpeedRadps = runtime.runtimeState.GetWheelSpeedRight();

            result.pivotEncoderAccepted =
                core.updateEncoderPair(pivotEncoderObservation, pivotDtSeconds, false);
            result.pivotEncoderAttempted = EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(core);
            result.pivotEncoderLastAccepted = EstimatorModeAndDiagnosticsTest::LastUpdateAccepted(core);
            result.pivotEncoderNis = EstimatorModeAndDiagnosticsTest::LastUpdateNis(core);
            RecordOperation(result.status, result.pivotEncoderAccepted, -1, L"pivot_encoder");
            if (!result.pivotEncoderAccepted)
            {
                return result;
            }

            result.stateAfterEncoder = EstimatorModeAndDiagnosticsTest::WorkingState(core);
            result.covarianceAfterEncoder = EstimatorModeAndDiagnosticsTest::WorkingCovariance(core);

            result.gyroCorrectedYawRateRadps = pivotGyroCorrectedYawRateRadps;
            const float gyroScaleRadpsPerLsb =
                runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(result.gyroCorrectedYawRateRadps) *
                kEstimatorTestImuGyroSensitivityToleranceFraction /
                std::sqrt(3.0f);
            const float yawInnovation =
                result.gyroCorrectedYawRateRadps - result.stateAfterEncoder(5);
            const float yawInnovationVariance =
                result.covarianceAfterEncoder(5, 5) +
                kEstimatorTestImuYawRateVarianceRadps2 +
                ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps);
            const float yawGain = result.covarianceAfterEncoder(5, 5) / yawInnovationVariance;
            result.expectedYawRateRadps =
                result.stateAfterEncoder(5) + (yawGain * yawInnovation);
            result.expectedYawVarianceRadps2 =
                result.covarianceAfterEncoder(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            result.expectedYawNis = (yawInnovation * yawInnovation) / yawInnovationVariance;

            result.pivotYawAccepted = core.updateYawRate(pivotGyroCorrectedYawRateRadps);
            result.pivotYawAttempted = EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(core);
            result.pivotYawLastAccepted = EstimatorModeAndDiagnosticsTest::LastUpdateAccepted(core);
            result.pivotYawNis = EstimatorModeAndDiagnosticsTest::LastUpdateNis(core);
            RecordOperation(result.status, result.pivotYawAccepted, -1, L"pivot_yaw");
            if (result.pivotYawAccepted)
            {
                result.stateAfterPivot = EstimatorModeAndDiagnosticsTest::WorkingState(core);
                result.covarianceAfterPivot = EstimatorModeAndDiagnosticsTest::WorkingCovariance(core);
            }
            return result;
        }

        struct PivotCommandResult final
        {
            ScenarioStatus status{};
            StateVector stateBeforeEncoder = NaNState();
            StateVector stateAfterEncoder = NaNState();
            float measuredForwardSpeedMps = NaN();
            float measuredYawRateRadps = NaN();
        };

        inline PivotCommandResult RunPivotCommandScenario()
        {
            PivotCommandResult result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(
                    core,
                    BuildDefaultMovingState(),
                    BuildDefaultMovingCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return result;
            }

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.60f);
            control.SetRightCommand(-0.60f);
            const bool predictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, predictAccepted, -1, L"predict");
            if (!predictAccepted)
            {
                return result;
            }
            result.stateBeforeEncoder = EstimatorModeAndDiagnosticsTest::WorkingState(core);

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(10);
            encoder.SetTotalRightCounts(-10);
            encoder.SetLeftWheelSpeedRadps(0.60f);
            encoder.SetRightWheelSpeedRadps(-0.60f);
            result.measuredForwardSpeedMps = runtime.plantModel.measuredLinearSpeedMps(encoder);
            result.measuredYawRateRadps = runtime.plantModel.measuredYawRateRadps(encoder);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            RecordOperation(result.status, encoderAccepted, -1, L"encoder");
            if (encoderAccepted)
            {
                result.stateAfterEncoder = EstimatorModeAndDiagnosticsTest::WorkingState(core);
            }
            return result;
        }

        struct DiagnosticResult final
        {
            ScenarioStatus status{};
            bool pivotScrubMode = true;
            bool encoderBodyUpdateSkipped = true;
            bool zeroUSoftApplied = true;
            float encoderMaskedDeltaNorm = NaN();
            float zeroUInnovationMps = NaN();
            float gyroMaskedDeltaNorm = NaN();
            std::size_t pivotModeIndex = 0U;
            std::size_t dumpLineCount = 0U;
            bool pivotLineReportsInactive = false;
        };

        inline DiagnosticResult RunDiagnosticScenario()
        {
            DiagnosticResult result{};
            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(
                    core,
                    BuildDefaultMovingState(),
                    BuildDefaultMovingCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return result;
            }

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);
            const bool predictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, predictAccepted, -1, L"predict");
            if (!predictAccepted)
            {
                return result;
            }

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(6);
            encoder.SetTotalRightCounts(6);
            encoder.SetLeftWheelSpeedRadps(0.80f);
            encoder.SetRightWheelSpeedRadps(0.80f);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            RecordOperation(result.status, encoderAccepted, -1, L"encoder");
            if (!encoderAccepted)
            {
                return result;
            }

            const bool yawAccepted = core.updateYawRate(0.02f);
            RecordOperation(result.status, yawAccepted, -1, L"yaw");
            if (!yawAccepted)
            {
                return result;
            }

            result.pivotScrubMode =
                FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "pivot_scrub_mode");
            result.encoderBodyUpdateSkipped =
                FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "encoder_body_update_skipped");
            result.zeroUSoftApplied =
                FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "zero_u_soft_applied");
            result.encoderMaskedDeltaNorm =
                FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_encoder", "masked_delta_norm");
            result.zeroUInnovationMps =
                FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_zero_u", "innovation_mps");
            result.gyroMaskedDeltaNorm =
                FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_gyro", "masked_delta_norm");

            const std::vector<std::pair<std::string, std::string>> dumpLines =
                CollectDebugDumpLines(core);
            result.dumpLineCount = dumpLines.size();
            result.pivotModeIndex =
                FindFirstDumpLineIndexContaining(dumpLines, "estimator_dump_pivot_scrub");
            if (result.pivotModeIndex < dumpLines.size())
            {
                result.pivotLineReportsInactive =
                    dumpLines[result.pivotModeIndex].second.find("pivot_scrub_mode=false") !=
                    std::string::npos;
            }
            return result;
        }

        struct LaunchTransientResult final
        {
            ScenarioStatus status{};
            float lateralVelocityMps = NaN();
        };

        inline LaunchTransientResult RunLaunchTransientScenario()
        {
            LaunchTransientResult result{};
            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(
                    core,
                    BuildLaunchTransientInitialState(),
                    BuildLaunchTransientInitialCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return result;
            }

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.20f);
            control.SetRightCommand(0.20f);
            const bool forwardPredictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, forwardPredictAccepted, -1, L"forward_predict");
            if (!forwardPredictAccepted)
            {
                return result;
            }

            control.SetLeftCommand(-0.20f);
            control.SetRightCommand(-0.20f);
            const bool reversePredictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, reversePredictAccepted, -1, L"reverse_predict");
            if (!reversePredictAccepted)
            {
                return result;
            }

            EncoderObs encoder{};
            encoder.SetLeftWheelSpeedRadps(2.0f);
            encoder.SetRightWheelSpeedRadps(2.0f);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            RecordOperation(result.status, encoderAccepted, -1, L"encoder");
            if (!encoderAccepted)
            {
                return result;
            }

            const bool yawAccepted = core.updateYawRate(0.0f);
            RecordOperation(result.status, yawAccepted, -1, L"yaw");
            if (yawAccepted)
            {
                result.lateralVelocityMps = EstimatorModeAndDiagnosticsTest::WorkingState(core)(4);
            }
            return result;
        }

        struct PlanarAccelGripResult final
        {
            ScenarioStatus status{};
            float initialLateralVelocityMps = NaN();
            float finalLateralVelocityMps = NaN();
            float initialLateralVarianceMps2 = NaN();
            float finalLateralVarianceMps2 = NaN();
            bool planarAccelAttempted = false;
            bool planarAccelAccepted = false;
        };

        inline PlanarAccelGripResult RunPlanarAccelGripScenario(
            const float forwardVelocityMps,
            const float yawRateRadps,
            const float lateralVarianceMps2)
        {
            PlanarAccelGripResult result{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                forwardVelocityMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            Estimator core = MakeDefaultEstimator();
            const StateVector initialState =
                BuildGripInitialState(forwardVelocityMps, yawRateRadps);
            const bool resetAccepted =
                EstimatorModeAndDiagnosticsTest::Reset(
                    core,
                    initialState,
                    BuildGripInitialCovariance(lateralVarianceMps2));
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (!resetAccepted)
            {
                return result;
            }

            EncoderObs encoder{};
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            RecordOperation(result.status, encoderAccepted, -1, L"encoder");
            if (!encoderAccepted)
            {
                return result;
            }

            result.initialLateralVelocityMps = initialState(4);
            result.initialLateralVarianceMps2 =
                EstimatorModeAndDiagnosticsTest::WorkingCovariance(core)(4, 4);

            const bool yawAccepted = core.updateYawRate(yawRateRadps);
            RecordOperation(result.status, yawAccepted, -1, L"yaw");
            if (!yawAccepted)
            {
                return result;
            }

            const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
            result.planarAccelAccepted = core.updatePlanarAccel(noPlanarAccelObservation);
            result.planarAccelAttempted = EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(core);
            if (result.planarAccelAttempted)
            {
                result.finalLateralVelocityMps = EstimatorModeAndDiagnosticsTest::WorkingState(core)(4);
                result.finalLateralVarianceMps2 =
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(core)(4, 4);
            }
            return result;
        }

        struct YawPlanarAccelResult final
        {
            ScenarioStatus mergedPrimeStatus{};
            ScenarioStatus sequentialPrimeStatus{};
            bool sequentialYawAttempted = false;
            bool sequentialYawAccepted = false;
            bool sequentialAccelAttempted = false;
            bool sequentialAccelAccepted = false;
            bool mergedYawAttempted = false;
            bool mergedYawAccepted = false;
            bool mergedAccelAttempted = false;
            bool mergedAccelAccepted = false;
            float stateMaxAbsDelta = NaN();
            float covarianceMaxAbsDelta = NaN();
        };

        inline YawPlanarAccelResult RunYawPlanarAccelScenario()
        {
            YawPlanarAccelResult result{};
            EstimatorTestRuntime runtime;
            const StateVector initialState = BuildPlanarAccelUpdateTestState();
            const CovarianceMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator mergedCore = MakeDefaultEstimator();
            Estimator sequentialCore = MakeDefaultEstimator();
            result.mergedPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(mergedCore, initialState, initialCovariance, control);
            result.sequentialPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(sequentialCore, initialState, initialCovariance, control);
            if (!result.mergedPrimeStatus.completed || !result.sequentialPrimeStatus.completed)
            {
                return result;
            }

            const ImuAccelObs accelObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    EstimatorModeAndDiagnosticsTest::WorkingState(mergedCore),
                    control,
                    0.35f,
                    -0.55f);
            const float gyroObservation =
                EstimatorModeAndDiagnosticsTest::WorkingState(mergedCore)(5) + 0.08f;

            result.sequentialYawAccepted = sequentialCore.updateYawRate(gyroObservation);
            result.sequentialYawAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(sequentialCore);
            result.sequentialAccelAccepted = sequentialCore.updatePlanarAccel(accelObservation);
            result.sequentialAccelAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(sequentialCore);
            result.mergedYawAccepted = mergedCore.updateYawRate(gyroObservation);
            result.mergedYawAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(mergedCore);
            result.mergedAccelAccepted = mergedCore.updatePlanarAccel(accelObservation);
            result.mergedAccelAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(mergedCore);

            result.stateMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingState(mergedCore),
                    EstimatorModeAndDiagnosticsTest::WorkingState(sequentialCore));
            result.covarianceMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(mergedCore),
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(sequentialCore));
            return result;
        }

        struct PlanarAccelChannelResult final
        {
            ScenarioStatus baselinePrimeStatus{};
            ScenarioStatus rightPrimeStatus{};
            ScenarioStatus forwardPrimeStatus{};
            bool baselineAttempted = false;
            bool baselineAccepted = false;
            bool rightAttempted = false;
            bool rightAccepted = false;
            bool forwardAttempted = false;
            bool forwardAccepted = false;
            float rightStateMaxAbsDelta = NaN();
            float rightCovarianceMaxAbsDelta = NaN();
            float forwardStateMaxAbsDelta = NaN();
            float forwardCovarianceMaxAbsDelta = NaN();
        };

        inline PlanarAccelChannelResult RunPlanarAccelChannelScenario()
        {
            PlanarAccelChannelResult result{};
            EstimatorTestRuntime runtime;
            const StateVector initialState = BuildPlanarAccelUpdateTestState();
            const CovarianceMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator baselineCore = MakeDefaultEstimator();
            Estimator rightPerturbedCore = MakeDefaultEstimator();
            Estimator forwardPerturbedCore = MakeDefaultEstimator();
            result.baselinePrimeStatus =
                PrimeCoreForPlanarAccelUpdate(baselineCore, initialState, initialCovariance, control);
            result.rightPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(rightPerturbedCore, initialState, initialCovariance, control);
            result.forwardPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(forwardPerturbedCore, initialState, initialCovariance, control);
            if (!result.baselinePrimeStatus.completed ||
                !result.rightPrimeStatus.completed ||
                !result.forwardPrimeStatus.completed)
            {
                return result;
            }

            const ImuAccelObs baselineObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    EstimatorModeAndDiagnosticsTest::WorkingState(baselineCore),
                    control,
                    0.0f,
                    0.35f);
            const ImuAccelObs rightPerturbedObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    EstimatorModeAndDiagnosticsTest::WorkingState(rightPerturbedCore),
                    control,
                    25.0f,
                    0.35f);
            const ImuAccelObs forwardPerturbedObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    EstimatorModeAndDiagnosticsTest::WorkingState(forwardPerturbedCore),
                    control,
                    0.0f,
                    0.55f);

            result.baselineAccepted = baselineCore.updatePlanarAccel(baselineObservation);
            result.baselineAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(baselineCore);
            result.rightAccepted = rightPerturbedCore.updatePlanarAccel(rightPerturbedObservation);
            result.rightAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(rightPerturbedCore);
            result.forwardAccepted = forwardPerturbedCore.updatePlanarAccel(forwardPerturbedObservation);
            result.forwardAttempted =
                EstimatorModeAndDiagnosticsTest::LastUpdateAttempted(forwardPerturbedCore);

            result.rightStateMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingState(baselineCore),
                    EstimatorModeAndDiagnosticsTest::WorkingState(rightPerturbedCore));
            result.rightCovarianceMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(baselineCore),
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(rightPerturbedCore));
            result.forwardStateMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingState(baselineCore),
                    EstimatorModeAndDiagnosticsTest::WorkingState(forwardPerturbedCore));
            result.forwardCovarianceMaxAbsDelta =
                MaxAbsDelta(
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(baselineCore),
                    EstimatorModeAndDiagnosticsTest::WorkingCovariance(forwardPerturbedCore));
            return result;
        }

    }
}
