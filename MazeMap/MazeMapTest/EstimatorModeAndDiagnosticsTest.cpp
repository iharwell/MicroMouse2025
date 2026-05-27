#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorFilterTestSupport.h"
#include "..\MazeMap\PlantModel.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;
        using PairCovarianceMatrix = Eigen::Matrix<float, 2, 2>;

        constexpr float kPlanarAccelUpdateTestDtSeconds = 0.001f;

        const wchar_t* BoolText(const bool value) noexcept
        {
            return value ? L"true" : L"false";
        }

        float NaN() noexcept
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        StateVector NaNState()
        {
            return StateVector::Constant(NaN());
        }

        CovarianceMatrix NaNCovariance()
        {
            return CovarianceMatrix::Constant(NaN());
        }

        std::wstring StatusMessage(const bool completed, const int step, const wchar_t* operation)
        {
            return std::wstring(L"setup_completed=") + BoolText(completed) +
                L" operation=" + operation +
                L" step=" + std::to_wstring(step);
        }

        std::wstring ValueMessage(
            const wchar_t* field,
            const float actual,
            const wchar_t* detail)
        {
            return std::wstring(field) +
                L" actual=" + std::to_wstring(actual) +
                L" detail=" + detail;
        }

        std::wstring BoolMessage(
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

        std::wstring LimitMessage(
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

        void RecordOperation(
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

        std::wstring ScenarioMessage(const ScenarioStatus& status)
        {
            return StatusMessage(status.completed, status.step, status.operation);
        }

        std::wstring ScenarioPairMessage(
            const ScenarioStatus& first,
            const ScenarioStatus& second,
            const wchar_t* firstName,
            const wchar_t* secondName)
        {
            return std::wstring(firstName) + L"{" + ScenarioMessage(first) + L"} " +
                secondName + L"{" + ScenarioMessage(second) + L"}";
        }

        std::wstring ScenarioTripleMessage(
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
        float MaxAbsCoeff(const Matrix& matrix)
        {
            return matrix.cwiseAbs().maxCoeff();
        }

        template <typename Matrix>
        float MaxAbsDelta(const Matrix& left, const Matrix& right)
        {
            return MaxAbsCoeff(left - right);
        }

        StateVector BuildPlanarAccelUpdateTestState() noexcept
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

        CovarianceMatrix BuildPlanarAccelUpdateTestCovariance() noexcept
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

        App::Internal::CommandVector BuildPlanarAccelUpdateTestControl() noexcept
        {
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.26f);
            control.SetRightCommand(0.23f);
            return control;
        }

        ImuAccelObs BuildPlanarAccelObservation(
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

        StateVector BuildPivotConflictInitialState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.20f;
            return state;
        }

        CovarianceMatrix BuildPivotConflictInitialCovariance() noexcept
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

        StateVector BuildDefaultMovingState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.10f;
            return state;
        }

        CovarianceMatrix BuildDefaultMovingCovariance() noexcept
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

        StateVector BuildLaunchTransientInitialState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(3) = 0.20f;
            state(4) = 0.40f;
            return state;
        }

        CovarianceMatrix BuildLaunchTransientInitialCovariance() noexcept
        {
            CovarianceMatrix covariance = BuildDefaultMovingCovariance();
            covariance(4, 4) = 0.30f * 0.30f;
            covariance(5, 5) = 0.05f * 0.05f;
            return covariance;
        }

        StateVector BuildGripInitialState(
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

        CovarianceMatrix BuildGripInitialCovariance(const float lateralVarianceMps2) noexcept
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

        EncoderPairNoiseResult RunEncoderPairNoiseScenario()
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

    namespace
    {
        ScenarioStatus PrimeCoreForPlanarAccelUpdate(
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

        PivotConflictResult RunPivotConflictScenario()
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
            constexpr float pivotGyroRawYawRateRadps = 1.80f;
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

            result.gyroCorrectedYawRateRadps =
                pivotGyroRawYawRateRadps - runtime.runtimeState.GetGyroBiasZ();
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
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                runtime.runtimeState.GetGyroBiasZVar();
            const float yawGain = result.covarianceAfterEncoder(5, 5) / yawInnovationVariance;
            result.expectedYawRateRadps =
                result.stateAfterEncoder(5) + (yawGain * yawInnovation);
            result.expectedYawVarianceRadps2 =
                result.covarianceAfterEncoder(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            result.expectedYawNis = (yawInnovation * yawInnovation) / yawInnovationVariance;

            result.pivotYawAccepted = core.updateYawRate(pivotGyroRawYawRateRadps);
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

        PivotCommandResult RunPivotCommandScenario()
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

        DiagnosticResult RunDiagnosticScenario()
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

        LaunchTransientResult RunLaunchTransientScenario()
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

        PlanarAccelGripResult RunPlanarAccelGripScenario(
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

        YawPlanarAccelResult RunYawPlanarAccelScenario()
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

        PlanarAccelChannelResult RunPlanarAccelChannelScenario()
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

    TEST_CLASS(EstimatorEncoderNoiseTuningTest)
    {
    public:
        TEST_METHOD(GeneralSigmaLeftVarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedVarianceRadps2, result.covariance(0, 0), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaRightVarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedVarianceRadps2, result.covariance(1, 1), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaLeftRightCovarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedCovarianceRadps2, result.covariance(0, 1), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaRightLeftCovarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedCovarianceRadps2, result.covariance(1, 0), 1.0e-5f);
        }

        TEST_METHOD(StationaryWheelSpeedSigmaUsesRequestedZeroSpeedSigma)
        {
            EstimatorTestRuntime runtime;
            const float expectedWheelSpeedSigmaRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            Assert::AreEqual(
                expectedWheelSpeedSigmaRadps,
                runtime.plantModel.stationaryEncoderWheelSpeedSigmaRadps(kEstimatorTestStationaryEncoderVelocitySigmaMps),
                1.0e-6f);
        }

    };

    TEST_CLASS(EstimatorDefaultInitialCovarianceTest)
    {
    public:
        TEST_METHOD(PoseXMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-5f, covariance(0, 0), 1.0e-9f);
        }

        TEST_METHOD(PoseYMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-5f, covariance(1, 1), 1.0e-9f);
        }

        TEST_METHOD(HeadingMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(2, 2), 1.0e-9f);
        }

        TEST_METHOD(ForwardVelocityMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(3, 3), 1.0e-9f);
        }

        TEST_METHOD(LateralVelocityMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(4, 4), 1.0e-9f);
        }

        TEST_METHOD(YawRateMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(5, 5), 1.0e-9f);
        }

        TEST_METHOD(ForwardAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(6, 6), 1.0e-9f);
        }

        TEST_METHOD(RightwardAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(7, 7), 1.0e-9f);
        }

        TEST_METHOD(YawAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(8, 8), 1.0e-9f);
        }

    };

    TEST_CLASS(EstimatorPivotConflictEncoderTest)
    {
    public:
        TEST_METHOD(CopiesLeftWheelSpeedToRuntime)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.pivotEncoderLeftWheelSpeedRadps, result.runtimeLeftWheelSpeedRadps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(CopiesRightWheelSpeedToRuntime)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.pivotEncoderRightWheelSpeedRadps, result.runtimeRightWheelSpeedRadps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(UpdateIsAttempted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_encoder_attempted", result.pivotEncoderAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotEncoderAttempted, message.c_str());
        }

        TEST_METHOD(UpdateIsAccepted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_encoder_accepted", result.pivotEncoderAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotEncoderAccepted, message.c_str());
        }

        TEST_METHOD(NisIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.pivotEncoderNis, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesPoseXUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(0), result.stateAfterEncoder(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesPoseYUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(1), result.stateAfterEncoder(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesHeadingUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(2), result.stateAfterEncoder(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesForwardVelocityUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(3), result.stateAfterEncoder(3), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesLateralVelocityUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(4), result.stateAfterEncoder(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesYawRateUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(5), result.stateAfterEncoder(5), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesForwardAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(6), result.stateAfterEncoder(6), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesRightwardAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(7), result.stateAfterEncoder(7), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesYawAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(8), result.stateAfterEncoder(8), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesCovarianceUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const float delta = MaxAbsDelta(result.covarianceAfterPredict, result.covarianceAfterEncoder);
            const std::wstring message = LimitMessage(L"covariance_max_abs_delta", delta, L"<=", 1.0e-7f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(delta <= 1.0e-7f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorPivotConflictYawTest)
    {
    public:
        TEST_METHOD(UpdateIsAttempted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_yaw_attempted", result.pivotYawAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotYawAttempted, message.c_str());
        }

        TEST_METHOD(UpdateIsAccepted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_yaw_accepted", result.pivotYawAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotYawAccepted, message.c_str());
        }

        TEST_METHOD(NisMatchesOracle)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawNis, result.pivotYawNis, 1.0e-4f, message.c_str());
        }

        TEST_METHOD(KeepsPoseX)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(0), result.stateAfterPivot(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(1), result.stateAfterPivot(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(2), result.stateAfterPivot(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsForwardVelocity)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(3), result.stateAfterPivot(3), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocity)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(4), result.stateAfterPivot(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(RateUsesGyro)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawRateRadps, result.stateAfterPivot(5), 1.0e-5f, message.c_str());
        }

        TEST_METHOD(VarianceMatchesOracle)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawVarianceRadps2, result.covarianceAfterPivot(5, 5), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CovarianceForwardAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 6), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(CovarianceRightwardAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 7), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(CovarianceYawAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 8), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(RateIsCloserToGyroThanEncoder)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const float gyroError = std::fabs(result.stateAfterPivot(5) - result.gyroCorrectedYawRateRadps);
            const float encoderError = std::fabs(result.stateAfterPivot(5) - result.encoderDerivedYawRateRadps);
            const std::wstring message = LimitMessage(L"gyro_error_radps", gyroError, L"< encoder_error", encoderError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(gyroError < encoderError, message.c_str());
        }

    };

    TEST_CLASS(EstimatorPivotCommandTest)
    {
    public:
        TEST_METHOD(KeepsPoseX)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(0), result.stateAfterEncoder(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(1), result.stateAfterEncoder(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(2), result.stateAfterEncoder(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocity)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(4), result.stateAfterEncoder(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(MovesForwardVelocityTowardEncoder)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const float afterError = std::fabs(result.stateAfterEncoder(3) - result.measuredForwardSpeedMps);
            const float beforeError = std::fabs(result.stateBeforeEncoder(3) - result.measuredForwardSpeedMps);
            const std::wstring message = LimitMessage(L"forward_velocity_error_mps", afterError, L"< before", beforeError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(afterError < beforeError, message.c_str());
        }

        TEST_METHOD(MovesYawRateTowardEncoder)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const float afterError = std::fabs(result.stateAfterEncoder(5) - result.measuredYawRateRadps);
            const float beforeError = std::fabs(result.stateBeforeEncoder(5) - result.measuredYawRateRadps);
            const std::wstring message = LimitMessage(L"yaw_rate_error_radps", afterError, L"< before", beforeError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(afterError < beforeError, message.c_str());
        }

    };

    TEST_CLASS(EstimatorDiagnosticDumpTest)
    {
    public:
        TEST_METHOD(ReportsPivotScrubInactive)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"pivot_scrub_mode", result.pivotScrubMode, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.pivotScrubMode, message.c_str());
        }

        TEST_METHOD(ReportsEncoderBodyUpdateNotSkipped)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"encoder_body_update_skipped", result.encoderBodyUpdateSkipped, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.encoderBodyUpdateSkipped, message.c_str());
        }

        TEST_METHOD(ReportsZeroUSoftNotApplied)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"zero_u_soft_applied", result.zeroUSoftApplied, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.zeroUSoftApplied, message.c_str());
        }

        TEST_METHOD(EncoderMaskedDeltaIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.encoderMaskedDeltaNorm, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ZeroUInnovationIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.zeroUInnovationMps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(GyroMaskedDeltaIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.gyroMaskedDeltaNorm, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ContainsPivotScrubLine)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = LimitMessage(L"pivot_mode_index", static_cast<float>(result.pivotModeIndex), L"< dump_line_count", static_cast<float>(result.dumpLineCount), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotModeIndex < result.dumpLineCount, message.c_str());
        }

        TEST_METHOD(PivotScrubLineReportsInactive)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"pivot_line_reports_inactive", result.pivotLineReportsInactive, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotLineReportsInactive, message.c_str());
        }

    };

    TEST_CLASS(EstimatorLaunchTransientTest)
    {
    public:
        TEST_METHOD(KeepsLateralVelocityNonzero)
        {
            const LaunchTransientResult result = RunLaunchTransientScenario();
            const std::wstring message = LimitMessage(L"lateral_velocity_mps", std::fabs(result.lateralVelocityMps), L">", 0.10f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.lateralVelocityMps) > 0.10f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorPlanarAccelCredibleGripTest)
    {
    public:
        TEST_METHOD(AttemptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = BoolMessage(L"planar_accel_attempted", result.planarAccelAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAttempted, message.c_str());
        }

        TEST_METHOD(AcceptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = BoolMessage(L"planar_accel_accepted", result.planarAccelAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAccepted, message.c_str());
        }

        TEST_METHOD(LateralVelocityIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = ValueMessage(L"lateral_velocity_mps", result.finalLateralVelocityMps, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVelocityMps), message.c_str());
        }

        TEST_METHOD(DampsLateralVelocityBelowThreshold)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L"<", 0.05f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual < 0.05f, message.c_str());
        }

        TEST_METHOD(ReducesLateralVelocityMagnitude)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const float limit = std::fabs(result.initialLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L"< initial", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual < limit, message.c_str());
        }

        TEST_METHOD(LateralVarianceIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = ValueMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVarianceMps2), message.c_str());
        }

        TEST_METHOD(ReducesLateralVariance)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = LimitMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, L"< initial", result.initialLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalLateralVarianceMps2 < result.initialLateralVarianceMps2, message.c_str());
        }

    };

    TEST_CLASS(EstimatorPlanarAccelNoncredibleGripTest)
    {
    public:
        TEST_METHOD(AttemptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const std::wstring message = BoolMessage(L"planar_accel_attempted", result.planarAccelAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAttempted, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocityLarge)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L">", 0.10f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual > 0.10f, message.c_str());
        }

        TEST_METHOD(LateralVarianceIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const std::wstring message = ValueMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVarianceMps2), message.c_str());
        }

        TEST_METHOD(KeepsLateralVarianceLoose)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const float limit = 0.020f * 0.020f;
            const std::wstring message = LimitMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, L">=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalLateralVarianceMps2 >= limit, message.c_str());
        }

    };

    TEST_CLASS(EstimatorYawPlanarAccelSeparabilityTest)
    {
    public:
        TEST_METHOD(SequentialYawUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_yaw_attempted", result.sequentialYawAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialYawAttempted, message.c_str());
        }

        TEST_METHOD(SequentialYawUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_yaw_accepted", result.sequentialYawAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialYawAccepted, message.c_str());
        }

        TEST_METHOD(SequentialPlanarAccelUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_accel_attempted", result.sequentialAccelAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialAccelAttempted, message.c_str());
        }

        TEST_METHOD(SequentialPlanarAccelUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_accel_accepted", result.sequentialAccelAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialAccelAccepted, message.c_str());
        }

        TEST_METHOD(MergedYawUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_yaw_attempted", result.mergedYawAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedYawAttempted, message.c_str());
        }

        TEST_METHOD(MergedYawUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_yaw_accepted", result.mergedYawAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedYawAccepted, message.c_str());
        }

        TEST_METHOD(MergedPlanarAccelUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_accel_attempted", result.mergedAccelAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedAccelAttempted, message.c_str());
        }

        TEST_METHOD(MergedPlanarAccelUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_accel_accepted", result.mergedAccelAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedAccelAccepted, message.c_str());
        }

        TEST_METHOD(SequentialAndMergedUpdatesProduceSameState)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = LimitMessage(L"state_max_abs_delta", result.stateMaxAbsDelta, L"<=", 1.0e-6f, ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.stateMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(SequentialAndMergedUpdatesProduceSameCovariance)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = LimitMessage(L"covariance_max_abs_delta", result.covarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.covarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorPlanarAccelChannelSensitivityTest)
    {
    public:
        TEST_METHOD(BaselineUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"baseline_attempted", result.baselineAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.baselineAttempted, message.c_str());
        }

        TEST_METHOD(BaselineUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"baseline_accepted", result.baselineAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.baselineAccepted, message.c_str());
        }

        TEST_METHOD(RightPerturbedUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"right_attempted", result.rightAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightAttempted, message.c_str());
        }

        TEST_METHOD(RightPerturbedUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"right_accepted", result.rightAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightAccepted, message.c_str());
        }

        TEST_METHOD(ForwardPerturbedUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"forward_attempted", result.forwardAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardAttempted, message.c_str());
        }

        TEST_METHOD(ForwardPerturbedUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"forward_accepted", result.forwardAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardAccepted, message.c_str());
        }

        TEST_METHOD(RightPerturbationChangesState)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"right_state_max_abs_delta", result.rightStateMaxAbsDelta, L">", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightStateMaxAbsDelta > 1.0e-6f, message.c_str());
        }

        TEST_METHOD(RightPerturbationKeepsCovarianceSame)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"right_covariance_max_abs_delta", result.rightCovarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightCovarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ForwardPerturbationChangesState)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"forward_state_max_abs_delta", result.forwardStateMaxAbsDelta, L">", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardStateMaxAbsDelta > 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ForwardPerturbationKeepsCovarianceSame)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"forward_covariance_max_abs_delta", result.forwardCovarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardCovarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }
    };
}
