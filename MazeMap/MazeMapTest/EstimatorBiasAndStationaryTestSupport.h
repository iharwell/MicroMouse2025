#pragma once

#include "..\MazeMap\Estimator.h"

#include "EstimatorFilterTestSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>


namespace MazeMap
{
    namespace EstimatorBiasAndStationaryTestSupport
    {
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        constexpr float kStationaryMotionTolerance = 5.0e-4f;
        constexpr float kStationaryPoseDriftToleranceM = 1.0e-3f;
        constexpr float kStationaryYawVarianceToleranceRadps2 = 1.1e-4f;

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

        inline std::wstring ValueMessage(
            const wchar_t* name,
            const float actual,
            const wchar_t* detail)
        {
            return std::wstring(name) +
                L" actual=" + std::to_wstring(actual) +
                L" detail=" + detail;
        }

        inline std::wstring LimitMessage(
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

        inline std::wstring StatusMessage(const bool completed, const int step, const wchar_t* operation)
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

        inline StateVector BuildZeroState() noexcept
        {
            return StateVector::Zero();
        }

        inline CovarianceMatrix BuildZeroMotionCovariance(
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

        inline CovarianceMatrix BuildStationaryLockCovariance() noexcept
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

        inline StateVector BuildStationaryYawState() noexcept
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(2) = NormalizeAngle(0.0f);
            state(4) = 0.35f;
            return state;
        }

        inline StateVector BuildReferenceStationaryState() noexcept
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

        inline CovarianceMatrix BuildReferenceStationaryCovariance() noexcept
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

        inline StateVector BuildRepeatedStationaryState() noexcept
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

        inline CovarianceMatrix BuildRepeatedStationaryCovariance() noexcept
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
        };
    }

    class EstimatorBiasAndStationaryTest final
    {
    public:
        using StateVector = EstimatorBiasAndStationaryTestSupport::StateVector;
        using CovarianceMatrix = EstimatorBiasAndStationaryTestSupport::CovarianceMatrix;

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

    namespace EstimatorBiasAndStationaryTestSupport
    {
        inline void CaptureSnapshot(
            FilterSnapshot& snapshot,
            Estimator& core)
        {
            if (!snapshot.status.completed)
            {
                return;
            }

            snapshot.state = EstimatorBiasAndStationaryTest::WorkingState(core);
            snapshot.covariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
        }

        inline void RunStationaryEncoderYawCycles(
            Estimator& core,
            VehicleState& runtimeState,
            ScenarioStatus& status,
            const App::Internal::CommandVector& control,
            SensorSnapshot::EncoderObs& encoder,
            const float dtSeconds,
            const float yawRateRadps,
            const int cycleCount,
            CovarianceMatrix* firstCertifiedCovariance = nullptr,
            bool* capturedFirstCertifiedCovariance = nullptr)
        {
            for (int step = 0; step < cycleCount; ++step)
            {
                (void)PublishEncoderObservationToRuntime(runtimeState, encoder, dtSeconds);
                const bool predictAccepted = core.predict(dtSeconds, control);
                RecordOperation(status, predictAccepted, step, L"predict");
                if (!predictAccepted)
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

        inline FilterSnapshot RunStationaryYawConstraint()
        {
            FilterSnapshot result{};
            result.initialState = BuildStationaryYawState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildStationaryLockCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.002f;
            const int steps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    runtime.runtimeState,
                    result.status,
                    control,
                    encoder,
                    dt,
                    0.0f,
                    steps);
            }

            CaptureSnapshot(result, core);
            return result;
        }

        inline FilterSnapshot RunExactStationaryReference()
        {
            FilterSnapshot result{};
            result.initialState = BuildReferenceStationaryState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildReferenceStationaryCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.002f;
            const int steps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    runtime.runtimeState,
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

        inline FilterSnapshot RunZeroMotionCycles(
            const int cycleCount,
            const App::Internal::CommandVector& control = App::Internal::CommandVector(0.0f, 0.0f))
        {
            FilterSnapshot result{};
            result.initialState = BuildZeroState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildZeroMotionCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < cycleCount; ++step)
                {
                    (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, dt);
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
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

        inline FilterSnapshot RunZeroEncoderYawVariance()
        {
            FilterSnapshot result{};
            result.initialState = BuildZeroState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(
                    core,
                    result.initialState,
                    BuildZeroMotionCovariance(0.005f * 0.005f, 1.0f));
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < 1000; ++step)
                {
                    (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, dt);
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
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

        inline LateralVarianceResult RunZeroEncoderLateralVariance()
        {
            LateralVarianceResult result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, BuildZeroState(), BuildZeroMotionCovariance(1.0f, 1.0f));
            RecordOperation(result.status, resetAccepted, -1, L"reset");
            if (resetAccepted)
            {
                result.initialVarianceMps2 = EstimatorBiasAndStationaryTest::WorkingCovariance(core)(4, 4);
            }

            App::Internal::CommandVector control{};
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                for (int step = 0; step < 1000; ++step)
                {
                    (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, dt);
                    const bool predictAccepted = core.predict(dt, control);
                    RecordOperation(result.status, predictAccepted, step, L"predict");
                    if (!predictAccepted)
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

        inline FilterSnapshot RunRepeatedStationaryCycles()
        {
            FilterSnapshot result{};
            result.initialState = BuildRepeatedStationaryState();

            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const bool resetAccepted =
                EstimatorBiasAndStationaryTest::Reset(core, result.initialState, BuildRepeatedStationaryCovariance());
            RecordOperation(result.status, resetAccepted, -1, L"reset");

            App::Internal::CommandVector control{};
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.001f;
            if (resetAccepted)
            {
                RunStationaryEncoderYawCycles(
                    core,
                    runtime.runtimeState,
                    result.status,
                    control,
                    encoder,
                    dt,
                    0.0f,
                    1000,
                    &result.firstCertifiedCovariance,
                    &result.capturedFirstCertifiedCovariance);
            }

            CaptureSnapshot(result, core);
            return result;
        }

        struct ReleaseCovarianceResult final
        {
            ScenarioStatus status{};
            CovarianceMatrix stationaryCovariance = NaNCovariance();
            CovarianceMatrix releasedCovariance = NaNCovariance();
        };

        inline ReleaseCovarianceResult RunStationaryRelease()
        {
            ReleaseCovarianceResult result{};
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector stationaryControl{};
            SensorSnapshot::EncoderObs stationaryEncoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.002f;
            const int stationarySteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < stationarySteps; ++step)
            {
                (void)PublishEncoderObservationToRuntime(runtime.runtimeState, stationaryEncoder, dt);
                const bool predictAccepted = core.predict(dt, stationaryControl);
                RecordOperation(result.status, predictAccepted, step, L"stationary_predict");
                if (!predictAccepted)
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
            SensorSnapshot::EncoderObs launchEncoder = SensorSnapshot{}.EncoderObservation();
            SetEncoderCountDeltasForWheelSpeedsOverTick(launchEncoder, 8.0f, 8.0f, dt);
            (void)PublishEncoderObservationToRuntime(runtime.runtimeState, launchEncoder, dt);
            const bool launchPredictAccepted = core.predict(dt, launchControl);
            RecordOperation(result.status, launchPredictAccepted, -1, L"launch_predict");

            if (result.status.completed)
            {
                result.releasedCovariance = EstimatorBiasAndStationaryTest::WorkingCovariance(core);
            }
            return result;
        }

        inline float ExpectedZeroEncoderYawVarianceLimit()
        {
            const float stationaryEncoderWheelSpeedSigmaRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            const float stationaryYawRateSigmaRadps =
                std::sqrt(2.0f) *
                Vehicle::WheelLinearVelocityFromWheelSpeed(stationaryEncoderWheelSpeedSigmaRadps) /
                Vehicle::GetPhysicalTrackWidthM();
            const float stationaryYawMeasurementVarianceRadps2 =
                stationaryYawRateSigmaRadps * stationaryYawRateSigmaRadps;
            return (std::max)(1.01f * stationaryYawMeasurementVarianceRadps2, 1.0e-8f);
        }

    }
}
