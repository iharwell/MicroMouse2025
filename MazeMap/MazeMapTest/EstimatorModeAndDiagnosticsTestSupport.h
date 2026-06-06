#pragma once

#include "EstimatorFilterTestSupport.h"
#include "..\MazeMap\PlantModel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>


namespace MazeMap
{
    namespace EstimatorModeAndDiagnosticsTestSupport
    {
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        constexpr float kPlanarAccelUpdateTestDtSeconds = 0.001f;
        constexpr int kDiagnosticStateHeadingIndex = 2;
        constexpr int kDiagnosticStateYawRateIndex = 5;

        inline const wchar_t* BoolText(const bool value) noexcept
        {
            return value ? L"true" : L"false";
        }

        inline float NaN() noexcept
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        inline float MaxAbsDiagnosticValue(const float left, const float right) noexcept
        {
            return (std::max)(std::fabs(left), std::fabs(right));
        }

        inline StateVector NaNState()
        {
            return StateVector::Constant(NaN());
        }

        inline CovarianceMatrix NaNCovariance()
        {
            return CovarianceMatrix::Constant(NaN());
        }

        inline float MaxAbsStateDeltaExceptIndex(
            const StateVector& before,
            const StateVector& after,
            const int allowedIndex) noexcept
        {
            float maxAbsDelta = 0.0f;
            for (int index = 0; index < VehicleState::kDimension; ++index)
            {
                if (index == allowedIndex)
                {
                    continue;
                }

                const float delta =
                    (index == kDiagnosticStateHeadingIndex) ?
                    NormalizeAngle(after(index) - before(index)) :
                    (after(index) - before(index));
                maxAbsDelta = (std::max)(maxAbsDelta, std::fabs(delta));
            }
            return maxAbsDelta;
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

        inline SensorSnapshot::EncoderObs PublishDiagnosticEncoderObservationToRuntime(
            VehicleState& runtimeState,
            const std::int32_t totalLeftCounts,
            const std::int32_t totalRightCounts,
            const float leftWheelSpeedRadps,
            const float rightWheelSpeedRadps) noexcept
        {
            SensorSnapshot snapshot = runtimeState.GetSensorSnapshot();
            SensorSnapshot::EncoderObs observation = snapshot.EncoderObservation();
            observation.SetTotalLeftCounts(totalLeftCounts);
            observation.SetTotalRightCounts(totalRightCounts);
            observation.SetDistanceDeltasM(
                Vehicle::DriveEncoderDistanceFromCounts(totalLeftCounts),
                Vehicle::DriveEncoderDistanceFromCounts(totalRightCounts));
            observation.SetWheelLinearVelocityMps(
                Vehicle::WheelLinearVelocityFromWheelSpeed(leftWheelSpeedRadps),
                Vehicle::WheelLinearVelocityFromWheelSpeed(rightWheelSpeedRadps));
            observation.SetWheelSpeedRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
            snapshot.PublishEncoderObservation(
                observation,
                true,
                snapshot.LeftEncoderTotalCounts() + static_cast<std::int64_t>(totalLeftCounts),
                snapshot.RightEncoderTotalCounts() + static_cast<std::int64_t>(totalRightCounts),
                snapshot.LeftEncoderDistanceM() + observation.LeftDistanceDeltaM(),
                snapshot.RightEncoderDistanceM() + observation.RightDistanceDeltaM());
            runtimeState.SetSensorSnapshot(snapshot);
            return observation;
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
            VehicleState& runtimeState,
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

            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                EstimatorModeAndDiagnosticsTest::WorkingState(core)(3),
                EstimatorModeAndDiagnosticsTest::WorkingState(core)(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            SetEncoderCountDeltasForWheelSpeedsOverTick(
                encoder,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                kPlanarAccelUpdateTestDtSeconds);
            (void)PublishEncoderObservationToRuntime(
                runtimeState,
                encoder,
                kPlanarAccelUpdateTestDtSeconds);
            const bool predictAccepted = core.predict(kPlanarAccelUpdateTestDtSeconds, control);
            RecordOperation(status, predictAccepted, -1, L"predict");
            return status;
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
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);

            constexpr float diagnosticDtSeconds = 0.001f;
            constexpr std::int32_t diagnosticLeftCounts = 6;
            constexpr std::int32_t diagnosticRightCounts = 6;
            constexpr float diagnosticLeftWheelSpeedRadps = 0.80f;
            constexpr float diagnosticRightWheelSpeedRadps = 0.80f;
            const SensorSnapshot::EncoderObs publishedEncoder =
                PublishDiagnosticEncoderObservationToRuntime(
                    runtime.runtimeState,
                    diagnosticLeftCounts,
                    diagnosticRightCounts,
                    diagnosticLeftWheelSpeedRadps,
                    diagnosticRightWheelSpeedRadps);

            const bool predictAccepted = core.predict(diagnosticDtSeconds, control);
            RecordOperation(result.status, predictAccepted, -1, L"predict");
            if (!predictAccepted)
            {
                return result;
            }

            const StateVector stateBeforeYaw = EstimatorModeAndDiagnosticsTest::WorkingState(core);
            const bool yawAccepted = core.updateYawRate(0.02f);
            RecordOperation(result.status, yawAccepted, -1, L"yaw");
            if (!yawAccepted)
            {
                return result;
            }
            const StateVector stateAfterYaw = EstimatorModeAndDiagnosticsTest::WorkingState(core);

            const std::vector<std::pair<std::string, std::string>> dumpLines =
                CollectDebugDumpLines(core);
            const bool predictionEncoderInput =
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "prediction_encoder_input");
            result.dumpLineCount = dumpLines.size();
            result.pivotModeIndex =
                FindFirstDumpLineIndexContaining(dumpLines, "estimator_dump_prediction_encoder_input");
            const bool predictionEncoderInputLinePresent = result.pivotModeIndex < dumpLines.size();
            const float dumpLeftWheelSpeedRadps =
                FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "left_wheel_speed_radps");
            const float dumpRightWheelSpeedRadps =
                FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "right_wheel_speed_radps");
            const float dumpLeftCounts =
                FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "total_left_counts");
            const float dumpRightCounts =
                FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "total_right_counts");
            const float expectedForwardSpeedMps =
                0.5f *
                Vehicle::GetDriveWheelRadiusM() *
                (publishedEncoder.LeftWheelSpeedRadps() + publishedEncoder.RightWheelSpeedRadps());
            const float dumpForwardSpeedMps =
                0.5f *
                Vehicle::GetDriveWheelRadiusM() *
                (dumpLeftWheelSpeedRadps + dumpRightWheelSpeedRadps);

            result.pivotScrubMode = !predictionEncoderInput;
            result.encoderMaskedDeltaNorm =
                (std::max)(
                    MaxAbsDiagnosticValue(
                        dumpLeftWheelSpeedRadps - publishedEncoder.LeftWheelSpeedRadps(),
                        dumpRightWheelSpeedRadps - publishedEncoder.RightWheelSpeedRadps()),
                    MaxAbsDiagnosticValue(
                        dumpLeftCounts - static_cast<float>(publishedEncoder.TotalLeftCounts()),
                        dumpRightCounts - static_cast<float>(publishedEncoder.TotalRightCounts())));
            result.encoderBodyUpdateSkipped =
                !predictionEncoderInputLinePresent ||
                !predictionEncoderInput ||
                !(result.encoderMaskedDeltaNorm <= 1.0e-6f);
            result.zeroUInnovationMps = dumpForwardSpeedMps - expectedForwardSpeedMps;
            result.zeroUSoftApplied =
                !(std::fabs(dumpForwardSpeedMps) > 1.0e-6f) ||
                !(std::fabs(result.zeroUInnovationMps) <= 1.0e-6f);
            result.gyroMaskedDeltaNorm =
                MaxAbsStateDeltaExceptIndex(
                    stateBeforeYaw,
                    stateAfterYaw,
                    kDiagnosticStateYawRateIndex);
            result.pivotLineReportsInactive =
                predictionEncoderInputLinePresent &&
                predictionEncoderInput &&
                (result.encoderMaskedDeltaNorm <= 1.0e-6f);
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
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
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

            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            SetEncoderCountDeltasForWheelSpeedsOverTick(encoder, 2.0f, 2.0f, 0.001f);
            (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, 0.001f);

            control.SetLeftCommand(-0.20f);
            control.SetRightCommand(-0.20f);
            const bool reversePredictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, reversePredictAccepted, -1, L"reverse_predict");
            if (!reversePredictAccepted)
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

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
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

            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            SetEncoderCountDeltasForWheelSpeedsOverTick(
                encoder,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                0.001f);
            (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, 0.001f);

            App::Internal::CommandVector control{};
            const bool predictAccepted = core.predict(0.001f, control);
            RecordOperation(result.status, predictAccepted, -1, L"predict");
            if (!predictAccepted)
            {
                return result;
            }

            result.initialLateralVelocityMps =
                EstimatorModeAndDiagnosticsTest::WorkingState(core)(4);
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
            EstimatorTestRuntime mergedRuntime;
            EstimatorTestRuntime sequentialRuntime;
            const StateVector initialState = BuildPlanarAccelUpdateTestState();
            const CovarianceMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator mergedCore(mergedRuntime.vehicle, mergedRuntime.plantModel, mergedRuntime.runtimeState);
            Estimator sequentialCore(sequentialRuntime.vehicle, sequentialRuntime.plantModel, sequentialRuntime.runtimeState);
            result.mergedPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(
                    mergedCore,
                    mergedRuntime.runtimeState,
                    initialState,
                    initialCovariance,
                    control);
            result.sequentialPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(
                    sequentialCore,
                    sequentialRuntime.runtimeState,
                    initialState,
                    initialCovariance,
                    control);
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
            EstimatorTestRuntime baselineRuntime;
            EstimatorTestRuntime rightPerturbedRuntime;
            EstimatorTestRuntime forwardPerturbedRuntime;
            const StateVector initialState = BuildPlanarAccelUpdateTestState();
            const CovarianceMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator baselineCore(
                baselineRuntime.vehicle,
                baselineRuntime.plantModel,
                baselineRuntime.runtimeState);
            Estimator rightPerturbedCore(
                rightPerturbedRuntime.vehicle,
                rightPerturbedRuntime.plantModel,
                rightPerturbedRuntime.runtimeState);
            Estimator forwardPerturbedCore(
                forwardPerturbedRuntime.vehicle,
                forwardPerturbedRuntime.plantModel,
                forwardPerturbedRuntime.runtimeState);
            result.baselinePrimeStatus =
                PrimeCoreForPlanarAccelUpdate(
                    baselineCore,
                    baselineRuntime.runtimeState,
                    initialState,
                    initialCovariance,
                    control);
            result.rightPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(
                    rightPerturbedCore,
                    rightPerturbedRuntime.runtimeState,
                    initialState,
                    initialCovariance,
                    control);
            result.forwardPrimeStatus =
                PrimeCoreForPlanarAccelUpdate(
                    forwardPerturbedCore,
                    forwardPerturbedRuntime.runtimeState,
                    initialState,
                    initialCovariance,
                    control);
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
