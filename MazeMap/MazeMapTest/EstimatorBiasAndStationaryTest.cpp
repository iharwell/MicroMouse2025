#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\Estimator.h"

#include "EstimatorFilterTestSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        constexpr float kStationaryMotionTolerance = 5.0e-4f;
        constexpr float kStationaryPoseDriftToleranceM = 1.0e-3f;
        constexpr float kStationaryYawVarianceToleranceRadps2 = 1.1e-4f;
        constexpr float kStationaryBiasSeedYawRateTolerance = 5.0e-3f;
        constexpr float kStationaryBiasWalkTolerance = 1.0e-4f;

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

        std::wstring ValueMessage(
            const wchar_t* name,
            const float actual,
            const wchar_t* detail)
        {
            return std::wstring(name) +
                L" actual=" + std::to_wstring(actual) +
                L" detail=" + detail;
        }

        std::wstring LimitMessage(
            const wchar_t* name,
            const float actual,
            const wchar_t* relation,
            const float limit,
            const wchar_t* detail)
        {
            return std::wstring(name) +
                L" actual=" + std::to_wstring(actual) +
                L" expected=" + relation + L" " + std::to_wstring(limit) +
                L" detail=" + detail;
        }

        std::wstring StatusMessage(const bool completed, const int step, const wchar_t* operation)
        {
            return std::wstring(L"setup_completed=") + BoolText(completed) +
                L" operation=" + operation +
                L" step=" + std::to_wstring(step);
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

        StateVector BuildZeroState() noexcept
        {
            return StateVector::Zero();
        }

        CovarianceMatrix BuildZeroMotionCovariance(
            const float lateralVelocityVariance = 0.005f * 0.005f,
            const float yawRateVariance = 0.05f * 0.05f) noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.001f * 0.001f;
            covariance(1, 1) = 0.001f * 0.001f;
            covariance(2, 2) = 0.01f * 0.01f;
            covariance(3, 3) = 0.005f * 0.005f;
            covariance(4, 4) = lateralVelocityVariance;
            covariance(5, 5) = yawRateVariance;
            covariance(6, 6) = 0.05f * 0.05f;
            covariance(7, 7) = 0.05f * 0.05f;
            covariance(8, 8) = 0.02f * 0.02f;
            return covariance;
        }

        CovarianceMatrix BuildStationaryLockCovariance() noexcept
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

        StateVector BuildStationaryYawState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(4) = 0.35f;
            return state;
        }

        StateVector BuildNonzeroCountStationaryState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(0) = 0.42f;
            state(1) = -0.17f;
            state(2) = NormalizeAngle(0.28f);
            return state;
        }

        StateVector BuildReferenceStationaryState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(0) = 1.20f;
            state(1) = 0.70f;
            state(2) = NormalizeAngle(-0.20f);
            state(3) = 0.020f;
            state(4) = -0.015f;
            state(5) = 0.040f;
            state(6) = 0.60f;
            state(7) = -0.50f;
            return state;
        }

        CovarianceMatrix BuildReferenceStationaryCovariance() noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.012f * 0.012f;
            covariance(1, 1) = 0.012f * 0.012f;
            covariance(2, 2) = 0.02f * 0.02f;
            covariance(3, 3) = 0.15f * 0.15f;
            covariance(4, 4) = 0.11f * 0.11f;
            covariance(5, 5) = 0.09f * 0.09f;
            covariance(6, 6) = 0.40f * 0.40f;
            covariance(7, 7) = 0.40f * 0.40f;
            covariance(8, 8) = 0.03f * 0.03f;
            covariance(0, 1) = 2.5e-5f;
            covariance(1, 0) = 2.5e-5f;
            covariance(0, 2) = -1.5e-5f;
            covariance(2, 0) = -1.5e-5f;
            covariance(1, 2) = 1.2e-5f;
            covariance(2, 1) = 1.2e-5f;
            return covariance;
        }

        StateVector BuildRepeatedStationaryState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(0) = 0.18f;
            state(1) = 0.27f;
            state(2) = NormalizeAngle(0.11f);
            state(3) = 0.35f;
            state(4) = -0.22f;
            state(5) = 0.18f;
            state(6) = 8.0f;
            state(7) = -7.5f;
            state(8) = 0.04f;
            return state;
        }

        CovarianceMatrix BuildRepeatedStationaryCovariance() noexcept
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.02f * 0.02f;
            covariance(1, 1) = 0.02f * 0.02f;
            covariance(2, 2) = 0.04f * 0.04f;
            covariance(3, 3) = 0.20f * 0.20f;
            covariance(4, 4) = 0.15f * 0.15f;
            covariance(5, 5) = 0.25f * 0.25f;
            covariance(6, 6) = 0.50f * 0.50f;
            covariance(7, 7) = 0.50f * 0.50f;
            covariance(8, 8) = 0.05f * 0.05f;
            return covariance;
        }

        struct FilterSnapshot final
        {
            ScenarioStatus status{};
            StateVector initialState = NaNState();
            StateVector state = NaNState();
            CovarianceMatrix covariance = NaNCovariance();
            CovarianceMatrix firstCertifiedCovariance = NaNCovariance();
            bool capturedFirstCertifiedCovariance = false;
            float gyroBiasRadps = NaN();
            float gyroBiasVarianceRadps2 = NaN();
        };
    }

    class EstimatorBiasAndStationaryTest final
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

        static bool Reset(
            Estimator& core,
            const StateVector& state,
            const CovarianceMatrix& covariance)
        {
            return core.reset(state, covariance);
        }
    };

    namespace
    {
        void CaptureSnapshot(
            FilterSnapshot& snapshot,
            Estimator& core,
            VehicleState* runtimeState = nullptr)
        {
            if (!snapshot.status.completed)
            {
                return;
            }

            snapshot.state = EstimatorBiasAndStationaryTest::WorkingState(core);
            snapshot.covariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
            if (runtimeState != nullptr)
            {
                snapshot.gyroBiasRadps = runtimeState->GetGyroBiasZ();
                snapshot.gyroBiasVarianceRadps2 = runtimeState->GetGyroBiasZVar();
            }
        }

        void RunStationaryEncoderYawCycles(
            Estimator& core,
            ScenarioStatus& status,
            const App::Internal::CommandVector& control,
            EncoderObs& encoder,
            const float dtSeconds,
            const float yawRateRadps,
            const int cycleCount,
            CovarianceMatrix* firstCertifiedCovariance = nullptr,
            bool* capturedFirstCertifiedCovariance = nullptr)
        {
            for (int step = 0; step < cycleCount; ++step)
            {
                const bool predictAccepted = core.predict(dtSeconds, control);
                RecordOperation(status, predictAccepted, step, L"predict");
                if (!predictAccepted)
                {
                    break;
                }

                const bool encoderAccepted = core.updateEncoderPair(encoder, dtSeconds, true);
                RecordOperation(status, encoderAccepted, step, L"encoder");
                if (!encoderAccepted)
                {
                    break;
                }

                const bool yawAccepted = core.updateYawRate(yawRateRadps);
                RecordOperation(status, yawAccepted, step, L"yaw");
                if (!yawAccepted)
                {
                    break;
                }

                if ((firstCertifiedCovariance != nullptr) &&
                    (capturedFirstCertifiedCovariance != nullptr) &&
                    !(*capturedFirstCertifiedCovariance) &&
                    (step >= static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dtSeconds))))
                {
                    *firstCertifiedCovariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
                    *capturedFirstCertifiedCovariance = true;
                }
            }
        }

        FilterSnapshot RunStationaryYawConstraint()
        {
            FilterSnapshot result{};
            result.initialState = BuildStationaryYawState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildStationaryLockCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            const int steps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    result.status,
                    control,
                    encoder,
                    dt,
                    0.04f,
                    steps);
            }

            CaptureSnapshot(result, core, &runtime.runtimeState);
            return result;
        }

        FilterSnapshot RunExactStationaryLockWithNonzeroCounts()
        {
            FilterSnapshot result{};
            result.initialState = BuildNonzeroCountStationaryState();

            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildStationaryLockCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(12);
            encoder.SetTotalRightCounts(8);
            constexpr float dt = 0.002f;
            const int steps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(core, result.status, control, encoder, dt, 0.0f, steps);
            }

            CaptureSnapshot(result, core);
            return result;
        }

        FilterSnapshot RunInitialStationaryGyroBias()
        {
            FilterSnapshot result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            InitialStationaryGyroBiasExpectation expected{};
            constexpr float dt = 0.0005f;
            const int steps =
                (std::max)(
                    200,
                    static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 20);

            for (int step = 0; step < steps; ++step)
            {
                const float rawStationaryGyroRadps =
                    (step < 49) ? 0.01f :
                    ((step < 150) ? 0.04f : 0.07f);
                const bool predictAccepted = core.predict(dt, control);
                RecordOperation(result.status, predictAccepted, step, L"predict");
                if (!predictAccepted)
                {
                    break;
                }

                const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                RecordOperation(result.status, encoderAccepted, step, L"encoder");
                if (!encoderAccepted)
                {
                    break;
                }

                const bool yawAccepted = core.updateYawRate(rawStationaryGyroRadps);
                RecordOperation(result.status, yawAccepted, step, L"yaw");
                if (!yawAccepted)
                {
                    break;
                }

                AdvanceInitialStationaryGyroBiasExpectation(expected, rawStationaryGyroRadps, dt);
            }

            result.capturedFirstCertifiedCovariance = expected.seedApplied;
            CaptureSnapshot(result, core, &runtime.runtimeState);
            return result;
        }

        struct BiasVarianceResult final
        {
            ScenarioStatus status{};
            float beforeBiasVarianceRadps2 = NaN();
            float afterBiasVarianceRadps2 = NaN();
        };

        BiasVarianceResult RunStationaryGyroBiasVariance()
        {
            BiasVarianceResult result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;

            result.beforeBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            for (int step = 0; step < 200; ++step)
            {
                const bool predictAccepted = core.predict(dt, control);
                RecordOperation(result.status, predictAccepted, step, L"predict");
                if (!predictAccepted)
                {
                    break;
                }

                const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                RecordOperation(result.status, encoderAccepted, step, L"encoder");
                if (!encoderAccepted)
                {
                    break;
                }

                const bool yawAccepted = core.updateYawRate(0.04f);
                RecordOperation(result.status, yawAccepted, step, L"yaw");
                if (!yawAccepted)
                {
                    break;
                }
            }
            result.afterBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            return result;
        }

        FilterSnapshot RunExactStationaryReference()
        {
            FilterSnapshot result{};
            result.initialState = BuildReferenceStationaryState();

            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildReferenceStationaryCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            const int steps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    result.status,
                    control,
                    encoder,
                    dt,
                    0.0f,
                    steps,
                    &result.firstCertifiedCovariance,
                    &result.capturedFirstCertifiedCovariance);
            }

            CaptureSnapshot(result, core);
            return result;
        }

        FilterSnapshot RunZeroMotionCycles(
            const int cycleCount,
            const App::Internal::CommandVector& control = App::Internal::CommandVector(0.0f, 0.0f))
        {
            FilterSnapshot result{};
            result.initialState = BuildZeroState();

            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildZeroMotionCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < cycleCount; ++step)
                {
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
                    {
                        break;
                    }

                    const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                    RecordOperation(result.status, encoderAccepted, step, L"encoder");
                    if (!encoderAccepted)
                    {
                        break;
                    }

                    const bool yawAccepted = core.updateYawRate(0.0f);
                    RecordOperation(result.status, yawAccepted, step, L"yaw");
                    if (!yawAccepted)
                    {
                        break;
                    }

                    const ImuAccelObs accel(true, 0.0f, 0.0f);
                    const bool accelAccepted = core.updatePlanarAccel(accel);
                    RecordOperation(result.status, accelAccepted, step, L"planar_accel");
                    if (!accelAccepted)
                    {
                        break;
                    }
                }
            }

            CaptureSnapshot(result, core);
            return result;
        }

        FilterSnapshot RunZeroEncoderYawVariance()
        {
            FilterSnapshot result{};
            result.initialState = BuildZeroState();

            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(
                    core,
                    result.initialState,
                    BuildZeroMotionCovariance(0.005f * 0.005f, 1.0f));
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < 1000; ++step)
                {
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
                    {
                        break;
                    }

                    const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                    RecordOperation(result.status, encoderAccepted, step, L"encoder");
                    if (!encoderAccepted)
                    {
                        break;
                    }
                }
            }

            CaptureSnapshot(result, core);
            return result;
        }

        struct LateralVarianceResult final
        {
            ScenarioStatus status{};
            float initialVarianceMps2 = NaN();
            float finalVarianceMps2 = NaN();
        };

        LateralVarianceResult RunZeroEncoderLateralVariance()
        {
            LateralVarianceResult result{};
            Estimator core = MakeDefaultEstimator();
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, BuildZeroState(), BuildZeroMotionCovariance(1.0f, 1.0f));
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (resetAccepted)
            {
                result.initialVarianceMps2 = EstimatorBiasAndStationaryTest::WorkingCovariance(core)(4, 4);
            }

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < 1000; ++step)
                {
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
                    {
                        break;
                    }

                    const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                    RecordOperation(result.status, encoderAccepted, step, L"encoder");
                    if (!encoderAccepted)
                    {
                        break;
                    }
                }
            }
            if (result.status.completed)
            {
                result.finalVarianceMps2 = EstimatorBiasAndStationaryTest::WorkingCovariance(core)(4, 4);
            }
            return result;
        }

        FilterSnapshot RunRepeatedStationaryCycles()
        {
            FilterSnapshot result{};
            result.initialState = BuildRepeatedStationaryState();

            EstimatorTestRuntime runtime;
            runtime.runtimeState.SetGyroBiasZ(0.04f);
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildRepeatedStationaryCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    result.status,
                    control,
                    encoder,
                    dt,
                    0.0f,
                    1000,
                    &result.firstCertifiedCovariance,
                    &result.capturedFirstCertifiedCovariance);
            }

            CaptureSnapshot(result, core, &runtime.runtimeState);
            return result;
        }

        struct ReleaseCovarianceResult final
        {
            ScenarioStatus status{};
            CovarianceMatrix stationaryCovariance = NaNCovariance();
            CovarianceMatrix releasedCovariance = NaNCovariance();
        };

        ReleaseCovarianceResult RunStationaryRelease()
        {
            ReleaseCovarianceResult result{};
            Estimator core = MakeDefaultEstimator();
            App::Internal::CommandVector stationaryControl{};
            EncoderObs stationaryEncoder{};
            constexpr float dt = 0.002f;
            const int stationarySteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < stationarySteps; ++step)
            {
                const bool predictAccepted = core.predict(dt, stationaryControl);
                RecordOperation(result.status, predictAccepted, step, L"stationary_predict");
                if (!predictAccepted)
                {
                    break;
                }

                const bool encoderAccepted = core.updateEncoderPair(stationaryEncoder, dt, true);
                RecordOperation(result.status, encoderAccepted, step, L"stationary_encoder");
                if (!encoderAccepted)
                {
                    break;
                }

                const bool yawAccepted = core.updateYawRate(0.0f);
                RecordOperation(result.status, yawAccepted, step, L"stationary_yaw");
                if (!yawAccepted)
                {
                    break;
                }
            }

            if (result.status.completed)
            {
                result.stationaryCovariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
            }

            App::Internal::CommandVector launchControl{};
            launchControl.SetLeftCommand(0.30f);
            launchControl.SetRightCommand(0.30f);
            const bool launchPredictAccepted = core.predict(dt, launchControl);
            RecordOperation(result.status, launchPredictAccepted, -1, L"launch_predict");

            EncoderObs launchEncoder{};
            launchEncoder.SetTotalLeftCounts(2);
            launchEncoder.SetTotalRightCounts(2);
            launchEncoder.SetLeftWheelSpeedRadps(8.0f);
            launchEncoder.SetRightWheelSpeedRadps(8.0f);
            if (launchPredictAccepted)
            {
                const bool launchEncoderAccepted = core.updateEncoderPair(launchEncoder, dt, true);
                RecordOperation(result.status, launchEncoderAccepted, -1, L"launch_encoder");
            }

            if (result.status.completed)
            {
                result.releasedCovariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
            }
            return result;
        }

        float ExpectedZeroEncoderYawVarianceLimit()
        {
            EstimatorTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float stationaryEncoderWheelSpeedSigmaRadps =
                plantModel.stationaryEncoderWheelSpeedSigmaRadps(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            const float stationaryYawRateSigmaRadps =
                std::sqrt(2.0f) *
                Vehicle::WheelLinearVelocityFromWheelSpeed(stationaryEncoderWheelSpeedSigmaRadps) /
                Vehicle::GetPhysicalTrackWidthM();
            const float stationaryYawMeasurementVarianceRadps2 =
                stationaryYawRateSigmaRadps * stationaryYawRateSigmaRadps;
            return (std::max)(1.01f * stationaryYawMeasurementVarianceRadps2, 1.0e-8f);
        }

    }

    TEST_CLASS(EstimatorStationaryYawConstraintTest)
    {
    public:
        TEST_METHOD(ForwardVelocityCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(LateralVelocityCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(YawRateCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ForwardAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(6), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(RightwardAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(7), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(YawAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(8), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(LearnsGyroBias)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"gyro_bias_radps", result.gyroBiasRadps, L"magnitude >", 1.0e-3f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.gyroBiasRadps) > 1.0e-3f, message.c_str());
        }

        TEST_METHOD(BiasTracksRawGyro)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const float errorRadps = std::fabs((result.state(5) + result.gyroBiasRadps) - 0.04f);
            const std::wstring message = LimitMessage(L"gyro_bias_plus_yaw_rate_error_radps", errorRadps, L"<", 0.04f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(errorRadps < 0.04f, message.c_str());
        }

        TEST_METHOD(ForwardVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(LateralVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const float limit = 0.005f * 0.005f;
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < limit, message.c_str());
        }

        TEST_METHOD(YawRateVarianceRetainsFloor)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const float limit = 0.010f * 0.010f;
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L">=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) >= limit, message.c_str());
        }

        TEST_METHOD(ForwardAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[6,6]", result.covariance(6, 6), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(6, 6) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(RightwardAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[7,7]", result.covariance(7, 7), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(7, 7) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(YawAccelVariancePositive)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[8,8]", result.covariance(8, 8), L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(8, 8) > 0.0f, message.c_str());
        }

        TEST_METHOD(YawAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[8,8]", result.covariance(8, 8), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(8, 8) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(GyroBiasVariancePositive)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"gyro_bias_variance_radps2", result.gyroBiasVarianceRadps2, L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.gyroBiasVarianceRadps2 > 0.0f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorNonzeroStationaryEncoderCountsTest)
    {
    public:
        TEST_METHOD(KeepPoseX)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepPoseY)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepHeading)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-4f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorStationaryGyroBiasTest)
    {
    public:
        TEST_METHOD(InitialSeedApplies)
        {
            const FilterSnapshot result = RunInitialStationaryGyroBias();
            const std::wstring message = std::wstring(L"seed_applied=") + BoolText(result.capturedFirstCertifiedCovariance) + L" " + ScenarioMessage(result.status);
            Assert::IsTrue(result.capturedFirstCertifiedCovariance, message.c_str());
        }

        TEST_METHOD(InitialSeedKeepsYawRateNearZero)
        {
            const FilterSnapshot result = RunInitialStationaryGyroBias();
            const std::wstring message = LimitMessage(L"yaw_rate_radps", result.state(5), L"magnitude <", kStationaryBiasSeedYawRateTolerance, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(5)) < kStationaryBiasSeedYawRateTolerance, message.c_str());
        }

        TEST_METHOD(InitialSeedMovesAboveLowValue)
        {
            const FilterSnapshot result = RunInitialStationaryGyroBias();
            const std::wstring message = LimitMessage(L"gyro_bias_radps", result.gyroBiasRadps, L">", 0.04f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.gyroBiasRadps > 0.04f, message.c_str());
        }

        TEST_METHOD(InitialSeedStaysBelowLaterSamples)
        {
            const FilterSnapshot result = RunInitialStationaryGyroBias();
            const std::wstring message = LimitMessage(L"gyro_bias_radps", result.gyroBiasRadps, L"<", 0.08f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.gyroBiasRadps < 0.08f, message.c_str());
        }

        TEST_METHOD(InitialSeedVarianceStaysFinite)
        {
            const FilterSnapshot result = RunInitialStationaryGyroBias();
            const std::wstring message = ValueMessage(L"gyro_bias_variance_radps2", result.gyroBiasVarianceRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.gyroBiasVarianceRadps2), message.c_str());
        }

        TEST_METHOD(VarianceStartsPositive)
        {
            const BiasVarianceResult result = RunStationaryGyroBiasVariance();
            const std::wstring message = LimitMessage(L"before_bias_variance_radps2", result.beforeBiasVarianceRadps2, L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.beforeBiasVarianceRadps2 > 0.0f, message.c_str());
        }

        TEST_METHOD(VarianceStaysFinite)
        {
            const BiasVarianceResult result = RunStationaryGyroBiasVariance();
            const std::wstring message = ValueMessage(L"after_bias_variance_radps2", result.afterBiasVarianceRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.afterBiasVarianceRadps2), message.c_str());
        }

        TEST_METHOD(VarianceStaysPositive)
        {
            const BiasVarianceResult result = RunStationaryGyroBiasVariance();
            const std::wstring message = LimitMessage(L"after_bias_variance_radps2", result.afterBiasVarianceRadps2, L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.afterBiasVarianceRadps2 > 0.0f, message.c_str());
        }

        TEST_METHOD(VarianceShrinks)
        {
            const BiasVarianceResult result = RunStationaryGyroBiasVariance();
            const std::wstring message = LimitMessage(L"after_bias_variance_radps2", result.afterBiasVarianceRadps2, L"< before", result.beforeBiasVarianceRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.afterBiasVarianceRadps2 < result.beforeBiasVarianceRadps2, message.c_str());
        }

    };

    TEST_CLASS(EstimatorExactStationaryLockTest)
    {
    public:
        TEST_METHOD(KeepsReferencePoseX)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsReferencePoseY)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsReferenceHeading)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(CollapsesForwardVelocity)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesLateralVelocity)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesYawRate)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesForwardAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(6), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesRightwardAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(7), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(PreservesYawAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(8), result.state(8), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CapturesFirstCertifiedCovariance)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = std::wstring(L"captured=") + BoolText(result.capturedFirstCertifiedCovariance) + L" " + ScenarioMessage(result.status);
            Assert::IsTrue(result.capturedFirstCertifiedCovariance, message.c_str());
        }

        TEST_METHOD(PoseXVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(0, 0) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) <= limit, message.c_str());
        }

        TEST_METHOD(PoseYVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(1, 1) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) <= limit, message.c_str());
        }

        TEST_METHOD(HeadingVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(2, 2) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[2,2]", result.covariance(2, 2), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(2, 2) <= limit, message.c_str());
        }

        TEST_METHOD(ForwardVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(LateralVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(YawRateVarianceRetainsFloor)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = 0.010f * 0.010f;
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L">=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) >= limit, message.c_str());
        }

    };

    TEST_CLASS(EstimatorGyroBiasTuningTest)
    {
    public:
        TEST_METHOD(YawRateVarianceMatchesTuning)
        {
            Assert::AreEqual(1.2e-6f, kEstimatorTestImuYawRateVarianceRadps2, 1.0e-12f);
        }

        TEST_METHOD(YawRateSigmaSquaresToVariance)
        {
            Assert::AreEqual(kEstimatorTestImuYawRateVarianceRadps2, kEstimatorTestImuYawRateSigmaRadps * kEstimatorTestImuYawRateSigmaRadps, 1.0e-12f);
        }

        TEST_METHOD(AccelSigmaMatchesTuning)
        {
            Assert::AreEqual(0.569900f, kEstimatorTestImuAccelSigmaMps2, 1.0e-6f);
        }

        TEST_METHOD(MovingProcessVarianceIsZero)
        {
            Assert::AreEqual(0.0f, kEstimatorTestGyroBiasProcessVarianceMovingRadps2PerSample, 0.0f);
        }

        TEST_METHOD(StationaryProcessVarianceMatchesTuning)
        {
            Assert::AreEqual(3.0e-16f, kEstimatorTestGyroBiasProcessVarianceStationaryRadps2PerSample, 1.0e-20f);
        }

        TEST_METHOD(InitialVarianceMatchesTuning)
        {
            Assert::AreEqual(3.05e-4f, kEstimatorTestGyroBiasInitialVarianceUnseededRadps2, 1.0e-12f);
        }

    };

    TEST_CLASS(EstimatorRepeatedZeroMotionStabilityTest)
    {
    public:
        TEST_METHOD(KeepsPoseXStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"position_x_m", result.state(0), L"magnitude <", kStationaryPoseDriftToleranceM, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(0)) < kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(BoundsPoseXVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<", 10.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) < 10.0f, message.c_str());
        }

        TEST_METHOD(KeepsPoseYStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"position_y_m", result.state(1), L"magnitude <", kStationaryPoseDriftToleranceM, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(1)) < kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(BoundsPoseYVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<", 100.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) < 100.0f, message.c_str());
        }

        TEST_METHOD(KeepsForwardVelocityStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"forward_velocity_mps", result.state(3), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(3)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsForwardVelocityVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocityStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"lateral_velocity_mps", result.state(4), L"magnitude <", 1.0e-5f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(4)) < 1.0e-5f, message.c_str());
        }

        TEST_METHOD(BoundsLateralVelocityVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsYawRateStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"yaw_rate_radps", result.state(5), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(5)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsYawRateVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", kStationaryYawVarianceToleranceRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= kStationaryYawVarianceToleranceRadps2, message.c_str());
        }

        TEST_METHOD(KeepsForwardAccelResidualStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"forward_accel_residual_mps2", result.state(6), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(6)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsForwardAccelVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[6,6]", result.covariance(6, 6), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(6, 6) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsRightwardAccelResidualStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"rightward_accel_residual_mps2", result.state(7), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(7)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsRightwardAccelVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[7,7]", result.covariance(7, 7), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(7, 7) < 0.0001f, message.c_str());
        }

    };

    TEST_CLASS(EstimatorZeroEncoderStationaryVarianceTest)
    {
    public:
        TEST_METHOD(YawRateVarianceIsFinite)
        {
            const FilterSnapshot result = RunZeroEncoderYawVariance();
            const std::wstring message = ValueMessage(L"covariance[5,5]", result.covariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.covariance(5, 5)), message.c_str());
        }

        TEST_METHOD(YawRateVarianceDropsToMeasurementLimit)
        {
            const FilterSnapshot result = RunZeroEncoderYawVariance();
            const float limit = ExpectedZeroEncoderYawVarianceLimit();
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= limit, message.c_str());
        }

        TEST_METHOD(InitialLateralVarianceMatchesProfile)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(1.0f, result.initialVarianceMps2, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(FinalLateralVarianceIsFinite)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = ValueMessage(L"final_lateral_variance_mps2", result.finalVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalVarianceMps2), message.c_str());
        }

        TEST_METHOD(FinalLateralVarianceShrinks)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = LimitMessage(L"final_lateral_variance_mps2", result.finalVarianceMps2, L"< initial", result.initialVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalVarianceMps2 < result.initialVarianceMps2, message.c_str());
        }

    };

    TEST_CLASS(EstimatorRepeatedStationaryCycleTest)
    {
    public:
        TEST_METHOD(CapturesCertifiedCovariance)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = std::wstring(L"captured=") + BoolText(result.capturedFirstCertifiedCovariance) + L" " + ScenarioMessage(result.status);
            Assert::IsTrue(result.capturedFirstCertifiedCovariance, message.c_str());
        }

        TEST_METHOD(KeepsPoseX)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-4f, message.c_str());
        }

        TEST_METHOD(ZeroesForwardVelocity)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ZeroesLateralVelocity)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ZeroesYawRate)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ReseedsGyroBias)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.gyroBiasRadps, kStationaryBiasWalkTolerance, message.c_str());
        }

        TEST_METHOD(PoseXVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(0, 0) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) <= limit, message.c_str());
        }

        TEST_METHOD(PoseYVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(1, 1) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) <= limit, message.c_str());
        }

        TEST_METHOD(HeadingVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(2, 2) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[2,2]", result.covariance(2, 2), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(2, 2) <= limit, message.c_str());
        }

    };

    TEST_CLASS(EstimatorStationaryReleaseCovarianceTest)
    {
    public:
        TEST_METHOD(InflatesForwardVelocityVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[3,3]", result.releasedCovariance(3, 3), L"> stationary", result.stationaryCovariance(3, 3), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(3, 3) > result.stationaryCovariance(3, 3), message.c_str());
        }

        TEST_METHOD(InflatesLateralVelocityVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[4,4]", result.releasedCovariance(4, 4), L"> stationary", result.stationaryCovariance(4, 4), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(4, 4) > result.stationaryCovariance(4, 4), message.c_str());
        }

        TEST_METHOD(InflatesYawRateVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[5,5]", result.releasedCovariance(5, 5), L"> stationary", result.stationaryCovariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(5, 5) > result.stationaryCovariance(5, 5), message.c_str());
        }

        TEST_METHOD(InflatesForwardAccelVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[6,6]", result.releasedCovariance(6, 6), L"> stationary", result.stationaryCovariance(6, 6), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(6, 6) > result.stationaryCovariance(6, 6), message.c_str());
        }

        TEST_METHOD(InflatesRightwardAccelVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[7,7]", result.releasedCovariance(7, 7), L"> stationary", result.stationaryCovariance(7, 7), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(7, 7) > result.stationaryCovariance(7, 7), message.c_str());
        }
    };
}
